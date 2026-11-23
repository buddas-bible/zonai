#pragma once

#if !defined(ZONAI_TEST_SUBSTEP_TRACE)
#error "Joint substep tracing is available only in test-enabled zonai builds."
#endif

#include <algorithm>
#include <array>
#include <cstdio>
#include <bit>
#include <cstdint>
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
    const std::size_t expected = static_cast<std::size_t>( 2 + 5 * subStepCount );
    if( left.records.size() != expected || right.records.size() != expected )
        throw std::logic_error( std::string( scene ) +
            ": expected one Prepare, five phases per substep and one Store" );

    for( std::size_t i = 0; i < expected; ++i )
    {
        const jointTraceStage2 stage = i == 0 ? jointTraceStage2::prepare :
            i == expected - 1 ? jointTraceStage2::store :
            std::array<jointTraceStage2, 5>{
                jointTraceStage2::integratedVelocity, jointTraceStage2::warmStart,
                jointTraceStage2::solveBias, jointTraceStage2::integratedPosition,
                jointTraceStage2::solveRelax
            }[( i - 1 ) % 5];
        const int index = i == 0 ? -1 :
            i == expected - 1 ? subStepCount : static_cast<int>( ( i - 1 ) / 5 );
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

private:
    static void capture( const world& simulation, jointTraceStage2 stage,
        int subStepIndex, std::span<const world::jointConstraint> constraints, void* context )
    {
        auto& recorder = *static_cast<jointSubstepTraceRecorder*>( context );
        stateSnapshotWriter writer{};

        writer.writeU32( "substep.schema.version", 1 );
        writer.writeU32( "substep.stage", static_cast<std::uint32_t>( stage ) );
        writer.writeS32( "substep.index", subStepIndex );
        writer.writeU32( "substep.constraintCount", static_cast<std::uint32_t>( constraints.size() ) );

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

        recorder.records.push_back( { stage, subStepIndex, std::move( snapshot ) } );
    }
};

} // namespace zonai::determinism::test
