#pragma once

#if !defined(ZONAI_TEST_SUBSTEP_TRACE)
#error "Joint substep tracing is available only in test-enabled zonai builds."
#endif

#include <algorithm>
#include <array>
#include <cstdio>
#include <bit>
#include <cstdint>
#include <cmath>
#include <span>
#include <stdexcept>
#include <string>
#include <type_traits>
#include <utility>
#include <vector>

#include "joint_cache_snapshot.h"

namespace zonai::determinism::test
{

struct jointSubstepTraceRecord
{
    jointTraceStage2 stage{};
    int subStepIndex = -1;
    stateSnapshot snapshot{};
};

struct jointSubstepTraceRecorder
{
    std::vector<bodyId> bodies{};
    std::vector<jointId> joints{};
    std::vector<jointSubstepTraceRecord> records{};
    // R2a-8: active Contact and Island graph fields only for contact fixtures.
    bool captureContactAndIsland = false;
    std::size_t maxContacts = 0;
    std::size_t maxIslands = 0;
    int maxPointCount = 0;
    bool sawNormalImpulse = false;
    bool sawTangentImpulse = false;
};

[[nodiscard]] inline const char* jointTraceStageName( jointTraceStage2 stage )
{
    switch( stage )
    {
    case jointTraceStage2::prepare: return "prepare";
    case jointTraceStage2::integratedVelocity: return "integratedVelocity";
    case jointTraceStage2::warmStart: return "warmStart";
    case jointTraceStage2::solveBias: return "solveBias";
    case jointTraceStage2::integratedPosition: return "integratedPosition";
    case jointTraceStage2::solveRelax: return "solveRelax";
    case jointTraceStage2::restitution: return "restitution";
    case jointTraceStage2::store: return "store";
    }
    throw std::logic_error( "unknown Joint trace stage" );
}

// Validate the true engine phase sequence, then compare complete transient
// snapshots from separate World instances. Emit every raw field only once.
inline void compareAndEmitJointSubsteps( const jointSubstepTraceRecorder& left,
    const jointSubstepTraceRecorder& right, const char* scene,
    std::uint32_t tick, int subStepCount, std::FILE* output )
{
    const std::size_t expected = static_cast<std::size_t>( 3 + 5 * subStepCount );
    if( left.records.size() != expected || right.records.size() != expected )
        throw std::logic_error( std::string( scene ) +
            ": expected one Prepare, five phases per substep, Restitution and Store" );

    for( std::size_t i = 0; i < expected; ++i )
    {
        const jointTraceStage2 stage = i == 0 ? jointTraceStage2::prepare :
            i == expected - 2 ? jointTraceStage2::restitution :
            i == expected - 1 ? jointTraceStage2::store :
            std::array<jointTraceStage2, 5>{
                jointTraceStage2::integratedVelocity, jointTraceStage2::warmStart,
                jointTraceStage2::solveBias, jointTraceStage2::integratedPosition,
                jointTraceStage2::solveRelax
            }[( i - 1 ) % 5];
        const int index = i == 0 ? -1 :
            i >= expected - 2 ? subStepCount : static_cast<int>( ( i - 1 ) / 5 );
        const jointSubstepTraceRecord& a = left.records[i];
        const jointSubstepTraceRecord& b = right.records[i];

        if( a.stage != stage || b.stage != stage ||
            a.subStepIndex != index || b.subStepIndex != index )
            throw std::logic_error( std::string( scene ) + ": substep phase ordering changed" );

        const auto first = firstDifference( a.snapshot, b.snapshot );
        if( first || a.snapshot.bytes != b.snapshot.bytes )
        {
            const std::string field = first ? first->field : "snapshot.rawBytes";
            throw std::runtime_error( std::string( scene ) + " Tick " +
                std::to_string( tick ) + " substep " + std::to_string( index ) +
                " stage " + jointTraceStageName( stage ) +
                " first mismatch " + field );
        }

        if( output )
        {
            for( const snapshotField& field : a.snapshot.fields )
            {
                std::fprintf( output, "%s.tick.%03u.substep.%d.%s.%s\t%016llx/%u\n",
                    scene, tick, index, jointTraceStageName( stage ),
                    field.name.c_str(),
                    static_cast<unsigned long long>( field.bits ),
                    static_cast<unsigned>( field.byteCount ) );
            }
        }
    }

    if( output && std::ferror( output ) )
        throw std::runtime_error( "substep trace output failed" );
}


// Narrow read-only callback installed by tests. In production builds the
// observer fields and all invocations are compiled out of world entirely.
struct jointSubstepTraceAccess
{
    static void install( world& simulation, jointSubstepTraceRecorder& recorder )
    {
        if( simulation.jointSubstepObserver_ != nullptr )
            throw std::logic_error( "only one substep observer per World" );

        simulation.jointSubstepObserver_ = &capture;
        simulation.jointSubstepObserverContext_ = &recorder;
    }

    static void uninstall( world& simulation )
    {
        simulation.jointSubstepObserver_ = nullptr;
        simulation.jointSubstepObserverContext_ = nullptr;
    }

    // R2a-9: capture active/free Contact slots, recycling anchors, simplex
    // cache and stable generations without serializing pointers or padding.
    [[nodiscard]] static stateSnapshot captureContactLifecycle( const world& simulation )
    {
        stateSnapshotWriter writer{};
        writer.writeU32( "lifecycle.schema.version", 1 );
        writer.writeU64( "lifecycle.liveCount", simulation.contactCount_ );
        writer.writeU64( "lifecycle.slotCount", simulation.contacts_.size() );
        writer.writeS32( "lifecycle.freeHead", simulation.contactFreeList_ );
        writer.writeU64( "lifecycle.recycledThisUpdate", simulation.recycledContactCount_ );

        std::size_t live = 0;
        std::size_t free = 0;
        for( std::size_t i = 0; i < simulation.contacts_.size(); ++i )
        {
            const contact2& slot = simulation.contacts_[i];
            const contactSim2& sim = simulation.contactSims_[i];
            const std::string p = "lifecycle.slot[" + std::to_string( i ) + "].";
            const bool active = slot.contactId != contact2::NULL_INDEX;
            writer.writeBool( p + "active", active );
            writer.writeU32( p + "generation", slot.generation );
            writer.writeS32( p + "freeNext", slot.nextFreeId );

            if( !active )
            {
                ++free;
                if( sim.contactId != contactSim2::NULL_INDEX ||
                    sim.manifold.pointCount != 0 || sim.recycleCacheValid )
                    throw std::logic_error( "freed Contact retains hot simulation state" );
                for( const contactImpulse2& point : sim.impulses )
                {
                    if( point.normalImpulse != 0.0f || point.tangentImpulse != 0.0f )
                        throw std::logic_error( "freed Contact retains warm-start impulses" );
                }
                continue;
            }

            ++live;
            if( slot.contactId != static_cast<std::int32_t>( i ) ||
                sim.contactId != static_cast<std::int32_t>( i ) ||
                sim.simplexCache.count > sim.simplexCache.indexA.size() )
                throw std::logic_error( "active Contact slot/hot-data mismatch" );
            writer.writeBool( p + "enableRecycling", slot.enableRecycling );
            writer.writeBool( p + "recycleCacheValid", sim.recycleCacheValid );
            writer.writeS32( p + "shapeA", sim.shapeIdA );
            writer.writeS32( p + "shapeB", sim.shapeIdB );
            writer.writeS32( p + "bodyA", sim.bodyIdA );
            writer.writeS32( p + "bodyB", sim.bodyIdB );
            writer.writeU16( p + "simplex.count", sim.simplexCache.count );
            for( std::size_t j = 0; j < sim.simplexCache.count; ++j )
            {
                const std::string n = p + "simplex[" + std::to_string( j ) + "].";
                writer.writeU16( n + "indexA", sim.simplexCache.indexA[j] );
                writer.writeU16( n + "indexB", sim.simplexCache.indexB[j] );
            }

            if( !sim.recycleCacheValid ) continue;

            const auto vec = [&]( std::string name, vec2 v )
            {
                writer.writeFloat( p + name + ".x", v.x );
                writer.writeFloat( p + name + ".y", v.y );
            };
            const auto rot = [&]( std::string name, rot2 rotation )
            {
                writer.writeFloat( p + name + ".c", rotation.c );
                writer.writeFloat( p + name + ".s", rotation.s );
            };
            vec( "cachedTransform.position", sim.cachedRelativeTransform.position );
            rot( "cachedTransform.rotation", sim.cachedRelativeTransform.rotation );
            rot( "cachedRotationA", sim.cachedRotationA );
            rot( "cachedRotationB", sim.cachedRotationB );
            for( int j = 0; j < sim.manifold.pointCount; ++j )
            {
                const std::string n = "recyclePoint[" + std::to_string( j ) + "].";
                vec( n + "anchorA", sim.recyclePointA[j] );
                vec( n + "anchorB", sim.recyclePointB[j] );
                writer.writeFloat( p + n + "separation", sim.recycleSeparation[j] );
            }
        }

        if( live != simulation.contactCount_ || live + free != simulation.contacts_.size() )
            throw std::logic_error( "Contact lifecycle active/free count mismatch" );

        std::vector<bool> visited( simulation.contacts_.size(), false );
        std::size_t walked = 0;
        for( std::int32_t index = simulation.contactFreeList_;
            index != contact2::NULL_INDEX; index = simulation.contacts_[index].nextFreeId )
        {
            if( index < 0 || static_cast<std::size_t>( index ) >= visited.size() ||
                visited[index] || simulation.contacts_[index].contactId != contact2::NULL_INDEX )
                throw std::logic_error( "Contact free-list cycle or invalid link" );
            visited[index] = true;
            writer.writeS32( "lifecycle.freeOrder[" + std::to_string( walked ) + "]", index );
            ++walked;
        }
        if( walked != free )
            throw std::logic_error( "Contact free-list length does not equal retired slots" );

        return std::move( writer ).finish();
    }

    // Stable persistent Contact slots after the final collision refresh.
    // Unlike transient constraints, this can also be called at Tick boundaries.
    [[nodiscard]] static stateSnapshot capturePersistentContacts( const world& simulation )
    {
        stateSnapshotWriter writer{};
        writer.writeU32( "contactCache.schema.version", 1 );
        writer.writeU64( "contactCache.liveCount", simulation.contactCount_ );
        std::size_t counted = 0;

        for( std::size_t i = 0; i < simulation.contactSims_.size(); ++i )
        {
            const contactSim2& sim = simulation.contactSims_[i];
            if( sim.contactId == contactSim2::NULL_INDEX ) continue;

            const contact2& slot = simulation.contacts_[i];
            if( sim.contactId != static_cast<std::int32_t>( i ) ||
                slot.contactId != static_cast<std::int32_t>( i ) )
                throw std::logic_error( "Contact stable slot and hot cache disagree" );

            if( sim.manifold.pointCount < 0 ||
                sim.manifold.pointCount > static_cast<int>( MAX_MANIFOLD_POINTS ) )
                throw std::logic_error( "invalid persistent Contact manifold point count" );

            ++counted;
            const std::string p = "contactCache.slot[" + std::to_string( i ) + "].";
            writer.writeS32( p + "id", sim.contactId );
            writer.writeU32( p + "generation", slot.generation );
            writer.writeS32( p + "shapeA", sim.shapeIdA );
            writer.writeS32( p + "shapeB", sim.shapeIdB );
            writer.writeS32( p + "bodyA", sim.bodyIdA );
            writer.writeS32( p + "bodyB", sim.bodyIdB );
            writer.writeBool( p + "recycleValid", sim.recycleCacheValid );
            writer.writeU32( p + "pointCount",
                static_cast<std::uint32_t>( sim.manifold.pointCount ) );
            writer.writeFloat( p + "normal.x", sim.manifold.normal.x );
            writer.writeFloat( p + "normal.y", sim.manifold.normal.y );

            for( int j = 0; j < sim.manifold.pointCount; ++j )
            {
                const auto& point = sim.manifold.points[j];
                const auto& impulse = sim.impulses[j];
                const std::string base = p + "point[" + std::to_string( j ) + "].";
                writer.writeU16( base + "id", point.id );
                writer.writeFloat( base + "position.x", point.point.x );
                writer.writeFloat( base + "position.y", point.point.y );
                writer.writeFloat( base + "separation", point.separation );
                writer.writeFloat( base + "normalImpulse", impulse.normalImpulse );
                writer.writeFloat( base + "tangentImpulse", impulse.tangentImpulse );
            }
        }

        if( counted != simulation.contactCount_ )
            throw std::logic_error( "active Contact cache count mismatch" );
        return std::move( writer ).finish();
    }

private:
    static void capture( const world& simulation, jointTraceStage2 stage,
        int subStepIndex, std::span<const world::jointConstraint> constraints,
        std::span<const contactConstraint2> contacts, const islandGraph2& graph, void* context )
    {
        auto& recorder = *static_cast<jointSubstepTraceRecorder*>( context );
        stateSnapshotWriter writer{};

        writer.writeU32( "substep.schema.version", 1 );
        writer.writeU32( "substep.stage", static_cast<std::uint32_t>( stage ) );
        writer.writeS32( "substep.index", subStepIndex );
        writer.writeU32( "substep.constraintCount", static_cast<std::uint32_t>( constraints.size() ) );

        if( recorder.captureContactAndIsland )
        {
            if( contacts.size() != graph.contactIds.size() )
                throw std::logic_error( "Contact constraint order does not match island graph" );

            recorder.maxContacts = std::max( recorder.maxContacts, contacts.size() );
            recorder.maxIslands = std::max( recorder.maxIslands, graph.islands.size() );
            writer.writeU32( "island.schema.version", 1 );
            writer.writeU32( "island.count", static_cast<std::uint32_t>( graph.islands.size() ) );
            writer.writeU32( "island.bodyCount", static_cast<std::uint32_t>( graph.bodyIds.size() ) );
            writer.writeU32( "island.contactCount", static_cast<std::uint32_t>( graph.contactIds.size() ) );
            writer.writeU32( "island.jointCount", static_cast<std::uint32_t>( graph.jointIds.size() ) );

            for( std::size_t i = 0; i < graph.islands.size(); ++i )
            {
                const island2& island = graph.islands[i];
                const std::string p = "island[" + std::to_string( i ) + "].";
                writer.writeU64( p + "bodyStart", island.bodyStart );
                writer.writeU64( p + "bodyCount", island.bodyCount );
                writer.writeU64( p + "contactStart", island.contactStart );
                writer.writeU64( p + "contactCount", island.contactCount );
                writer.writeU64( p + "jointStart", island.jointStart );
                writer.writeU64( p + "jointCount", island.jointCount );
                if( island.bodyStart + island.bodyCount > graph.bodyIds.size() ||
                    island.contactStart + island.contactCount > graph.contactIds.size() ||
                    island.jointStart + island.jointCount > graph.jointIds.size() )
                    throw std::logic_error( "invalid Island graph flat ranges" );
            }

            for( std::size_t i = 0; i < graph.bodyIds.size(); ++i )
                writer.writeS32( "island.bodyOrder[" + std::to_string( i ) + "]", graph.bodyIds[i] );
            for( std::size_t i = 0; i < graph.contactIds.size(); ++i )
                writer.writeS32( "island.contactOrder[" + std::to_string( i ) + "]", graph.contactIds[i] );
            for( std::size_t i = 0; i < graph.jointIds.size(); ++i )
                writer.writeS32( "island.jointOrder[" + std::to_string( i ) + "]", graph.jointIds[i] );

            for( std::size_t i = 0; i < contacts.size(); ++i )
            {
                const contactConstraint2& c = contacts[i];
                if( c.contactId != graph.contactIds[i] ||
                    c.contactId < 0 ||
                    static_cast<std::size_t>( c.contactId ) >= simulation.contactSims_.size() )
                    throw std::logic_error( "Contact constraint index/order mismatch" );
                if( c.pointCount < 0 ||
                    c.pointCount > static_cast<int>( MAX_MANIFOLD_POINTS ) )
                    throw std::logic_error( "invalid transient Contact point count" );

                const contactSim2& sim = simulation.contactSims_[c.contactId];
                if( c.pointCount != sim.manifold.pointCount )
                    throw std::logic_error( "Contact constraint/manifold point count mismatch" );
                const std::string p = "contact[" + std::to_string( i ) + "].";
                writer.writeS32( p + "id", c.contactId );
                writer.writeU32( p + "generation", simulation.contacts_[c.contactId].generation );
                writer.writeS32( p + "bodyA", c.bodyIdA );
                writer.writeS32( p + "bodyB", c.bodyIdB );
                writer.writeU32( p + "pointCount", static_cast<std::uint32_t>( c.pointCount ) );
                writer.writeFloat( p + "normal.x", c.normal.x );
                writer.writeFloat( p + "normal.y", c.normal.y );
                writer.writeFloat( p + "friction", c.friction );
                writer.writeFloat( p + "restitution", c.restitution );
                writer.writeFloat( p + "softness.biasRate", c.softness.biasRate );
                writer.writeFloat( p + "softness.massScale", c.softness.massScale );
                writer.writeFloat( p + "softness.impulseScale", c.softness.impulseScale );
                writer.writeFloat( p + "maxPushSpeed", c.maxPushSpeed );
                writer.writeFloat( p + "invTimeStep", c.invTimeStep );
                recorder.maxPointCount = std::max( recorder.maxPointCount, c.pointCount );

                for( int j = 0; j < c.pointCount; ++j )
                {
                    const contactConstraintPoint2& point = c.points[j];
                    const std::string n = p + "point[" + std::to_string( j ) + "].";
                    writer.writeU16( n + "featureId", sim.manifold.points[j].id );
                    writer.writeFloat( n + "anchorA.x", point.anchorA.x );
                    writer.writeFloat( n + "anchorA.y", point.anchorA.y );
                    writer.writeFloat( n + "anchorB.x", point.anchorB.x );
                    writer.writeFloat( n + "anchorB.y", point.anchorB.y );
                    writer.writeFloat( n + "baseSeparation", point.baseSeparation );
                    writer.writeFloat( n + "relativeNormalVelocity", point.relativeNormalVelocity );
                    writer.writeFloat( n + "normalMass", point.normalMass );
                    writer.writeFloat( n + "tangentMass", point.tangentMass );
                    writer.writeFloat( n + "normalImpulse", point.normalImpulse );
                    writer.writeFloat( n + "totalNormalImpulse", point.totalNormalImpulse );
                    writer.writeFloat( n + "restitutionImpulse", point.restitutionImpulse );
                    writer.writeFloat( n + "tangentImpulse", point.tangentImpulse );
                    if( point.normalImpulse > 0.0f )
                        recorder.sawNormalImpulse = true;
                    if( point.tangentImpulse != 0.0f )
                        recorder.sawTangentImpulse = true;
                }
            }
        }

        for( const bodyId id : recorder.bodies )
        {
            if( !simulation.IsValid( id ) )
                throw std::invalid_argument( "invalid observed Body in substep trace" );
            const std::size_t index = static_cast<std::size_t>( id.index1 - 1 );
            const bodyState& state = simulation.bodyStates_[index];
            const std::string prefix = "substep.body[" + std::to_string( id.index1 ) + "].";
            writer.writeFloat( prefix + "linearVelocity.x", state.linearVelocity.x );
            writer.writeFloat( prefix + "linearVelocity.y", state.linearVelocity.y );
            writer.writeFloat( prefix + "angularVelocity", state.angularVelocity );
            writer.writeFloat( prefix + "deltaPosition.x", state.deltaPosition.x );
            writer.writeFloat( prefix + "deltaPosition.y", state.deltaPosition.y );
            writer.writeFloat( prefix + "deltaRotation.c", state.deltaRotation.c );
            writer.writeFloat( prefix + "deltaRotation.s", state.deltaRotation.s );
        }

        for( const jointId id : recorder.joints )
        {
            if( !simulation.IsValid( id ) )
                throw std::invalid_argument( "invalid observed Joint in substep trace" );
            const std::string prefix = "substep.joint[" + std::to_string( id.index1 ) + "].";
            const std::int32_t index = id.index1 - 1;
            bool matched = false;

            for( const world::jointConstraint& value : constraints )
            {
                std::visit( [&]( const auto& constraint )
                {
                    using constraintType = std::remove_cvref_t<decltype( constraint )>;
                    if( constraint.jointId != index ) return;
                    if( matched )
                        throw std::logic_error( "duplicate transient Joint constraint" );
                    matched = true;

                    const auto scalar = [&]( std::string name, float value )
                    {
                        writer.writeFloat( prefix + name, value );
                    };
                    const auto vector = [&]( std::string name, vec2 v )
                    {
                        scalar( name + ".x", v.x );
                        scalar( name + ".y", v.y );
                    };
                    const auto doubleBits = [&]( std::string name, double value )
                    {
                        if( !std::isfinite( value ) )
                            throw std::domain_error( "nonfinite Joint transient double" );
                        writer.writeU64( prefix + name, std::bit_cast<std::uint64_t>( value ) );
                    };

                    writer.writeS32( prefix + "slot", index );
                    // Mouse constraints intentionally have only bodyB. They
                    // are not yet tracked, but std::visit instantiates all arms.
                    if constexpr( std::is_same_v<constraintType, mouseJointConstraint2> )
                        writer.writeS32( prefix + "bodyA", -1 );
                    else
                        writer.writeS32( prefix + "bodyA", constraint.bodyIdA );
                    writer.writeS32( prefix + "bodyB", constraint.bodyIdB );

                    if constexpr( std::is_same_v<constraintType, revoluteJointConstraint2> )
                    {
                        writer.writeU32( prefix + "type", 1 );
                        vector( "impulse", constraint.impulse );
                        scalar( "lowerImpulse", constraint.lowerImpulse );
                        scalar( "upperImpulse", constraint.upperImpulse );
                        scalar( "motorImpulse", constraint.motorImpulse );
                        scalar( "angularMass", constraint.angularMass );
                        scalar( "invSubStepTime", constraint.invSubStepTime );
                        vector( "anchorA", constraint.anchorA );
                        vector( "anchorB", constraint.anchorB );
                    }
                    else if constexpr( std::is_same_v<constraintType, prismaticJointConstraint2> )
                    {
                        writer.writeU32( prefix + "type", 2 );
                        vector( "impulse", constraint.impulse );
                        scalar( "lowerImpulse", constraint.lowerImpulse );
                        scalar( "upperImpulse", constraint.upperImpulse );
                        scalar( "motorImpulse", constraint.motorImpulse );
                        scalar( "springImpulse", constraint.springImpulse );
                        scalar( "invSubStepTime", constraint.invSubStepTime );
                        vector( "axisA", constraint.axisA );
                        vector( "anchorA", constraint.anchorA );
                        vector( "anchorB", constraint.anchorB );
                    }
                    else if constexpr( std::is_same_v<constraintType, weldJointConstraint2> )
                    {
                        writer.writeU32( prefix + "type", 3 );
                        vector( "impulse", constraint.impulse );
                        scalar( "angularImpulse", constraint.angularImpulse );
                        scalar( "angularMass", constraint.angularMass );
                        vector( "anchorA", constraint.anchorA );
                        vector( "anchorB", constraint.anchorB );
                    }
                    else if constexpr( std::is_same_v<constraintType, motorJointConstraint2> )
                    {
                        writer.writeU32( prefix + "type", 4 );
                        vector( "linearVelocityImpulse", constraint.linearVelocityImpulse );
                        vector( "linearSpringImpulse", constraint.linearSpringImpulse );
                        scalar( "angularVelocityImpulse", constraint.angularVelocityImpulse );
                        scalar( "angularSpringImpulse", constraint.angularSpringImpulse );
                        scalar( "angularMass", constraint.angularMass );
                        vector( "anchorA", constraint.anchorA );
                        vector( "anchorB", constraint.anchorB );
                    }
                    else if constexpr( std::is_same_v<constraintType, splineRailJointConstraint2> )
                    {
                        writer.writeU32( prefix + "type", 5 );
                        vector( "impulse", constraint.impulse );
                        scalar( "springImpulse", constraint.springImpulse );
                        scalar( "lowerImpulse", constraint.lowerImpulse );
                        scalar( "upperImpulse", constraint.upperImpulse );
                        scalar( "motorImpulse", constraint.motorImpulse );
                        scalar( "subStepTime", constraint.subStepTime );
                        writer.writeS32( prefix + "parameter.segmentIndex",
                            static_cast<std::int32_t>( constraint.parameter.segmentIndex ) );
                        scalar( "parameter.t", constraint.parameter.t );
                        doubleBits( "distanceOffset", constraint.distanceOffset );
                        writer.writeBool( prefix + "trackingValid", constraint.trackingValid );
                        vector( "force", constraint.force );
                        scalar( "torque", constraint.torque );
                    }
                    else
                    {
                        throw std::invalid_argument(
                            "substep trace selected a Joint type not yet supported" );
                    }
                }, value );
            }

            if( !matched )
                throw std::logic_error( "tracked Joint missing from active transient solver set" );
        }

        stateSnapshot snapshot = std::move( writer ).finish();

        if( stage == jointTraceStage2::store )
        {
            const stateSnapshot persistent = jointCacheSnapshotAccess::capture(
                simulation, recorder.joints );
            snapshot.bytes.insert( snapshot.bytes.end(),
                persistent.bytes.begin(), persistent.bytes.end() );
            snapshot.fields.insert( snapshot.fields.end(),
                persistent.fields.begin(), persistent.fields.end() );
        }
        if( recorder.captureContactAndIsland && stage == jointTraceStage2::store )
        {
            const stateSnapshot persistent = capturePersistentContacts( simulation );
            snapshot.bytes.insert( snapshot.bytes.end(),
                persistent.bytes.begin(), persistent.bytes.end() );
            snapshot.fields.insert( snapshot.fields.end(),
                persistent.fields.begin(), persistent.fields.end() );
        }

        recorder.records.push_back( { stage, subStepIndex, std::move( snapshot ) } );
    }
};

} // namespace zonai::determinism::test
