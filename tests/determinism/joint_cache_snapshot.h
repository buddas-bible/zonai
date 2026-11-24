#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include "world_state_snapshot.h"

namespace zonai::determinism::test
{

// Test-only read-only bridge to the persistent Joint simulation slots.
// Capture named, defined fields rather than struct padding, addresses,
// variant object bytes, or World-specific opaque handle tokens.
struct jointCacheSnapshotAccess
{
    [[nodiscard]] static stateSnapshot capture(
        const world& simulation, std::span<const jointId> trackedJoints )
    {
        std::vector<jointId> ordered( trackedJoints.begin(), trackedJoints.end() );
        std::sort( ordered.begin(), ordered.end(), []( jointId a, jointId b )
        {
            if( a.index1 != b.index1 ) return a.index1 < b.index1;
            return a.generation < b.generation;
        } );

        stateSnapshotWriter writer{};
        writer.writeU32( "cache.schema.version", 1 );
        writer.writeU64( "cache.trackedJointCount", ordered.size() );
        writer.writeU64( "cache.worldJointCount", simulation.jointCount_ );

        for( std::size_t i = 0; i < ordered.size(); ++i )
        {
            const jointId id = ordered[i];
            if( !simulation.IsValid( id ) )
                throw std::invalid_argument( "Joint cache snapshot received an invalid or foreign handle" );
            if( i && ordered[i - 1].index1 == id.index1 )
                throw std::invalid_argument( "Joint cache snapshot received a duplicate slot" );

            const auto index = static_cast<std::size_t>( id.index1 - 1 );
            const joint2& slot = simulation.joints_[index];
            if( slot.jointId != static_cast<std::int32_t>( index ) ||
                slot.generation != id.generation )
                throw std::logic_error( "Joint cache stable slot metadata mismatch" );

            const std::string prefix = "cache.joint[" + std::to_string( id.index1 ) + "].";
            writer.writeS32( prefix + "index1", id.index1 );
            writer.writeU16( prefix + "generation", id.generation );
            writer.writeS32( prefix + "bodyA", slot.edges[0].bodyId );
            writer.writeS32( prefix + "bodyB", slot.edges[1].bodyId );
            writer.writeBool( prefix + "collideConnected", slot.collideConnected );

            const auto writeFloat = [&]( std::string name, float value )
            {
                writer.writeFloat( prefix + name, value );
            };
            const auto writeDouble = [&]( std::string name, double value )
            {
                if( !std::isfinite( value ) )
                    throw std::domain_error( "non-finite Joint persistent double: " + name );
                writer.writeU64( prefix + name, std::bit_cast<std::uint64_t>( value ) );
            };
            const auto writeVec = [&]( std::string name, vec2 value )
            {
                writeFloat( name + ".x", value.x );
                writeFloat( name + ".y", value.y );
            };
            const auto& storage = simulation.jointSims_[index];

            if( const auto* sim = std::get_if<revoluteJointSim2>( &storage ) )
            {
                writer.writeU32( prefix + "type", 1 );
                writeVec( "impulse", sim->impulse );
                writeFloat( "lowerImpulse", sim->lowerImpulse );
                writeFloat( "upperImpulse", sim->upperImpulse );
                writeFloat( "motorImpulse", sim->motorImpulse );
                writeFloat( "subStepTime", sim->subStepTime );
                writeFloat( "referenceAngle", sim->referenceAngle );
                writer.writeBool( prefix + "enableLimit", sim->enableLimit );
                writer.writeBool( prefix + "enableMotor", sim->enableMotor );
                writeFloat( "lowerAngle", sim->lowerAngle );
                writeFloat( "upperAngle", sim->upperAngle );
                writeFloat( "motorSpeed", sim->motorSpeed );
            }
            else if( const auto* sim = std::get_if<prismaticJointSim2>( &storage ) )
            {
                writer.writeU32( prefix + "type", 2 );
                writeVec( "impulse", sim->impulse );
                writeFloat( "springImpulse", sim->springImpulse );
                writeFloat( "lowerImpulse", sim->lowerImpulse );
                writeFloat( "upperImpulse", sim->upperImpulse );
                writeFloat( "motorImpulse", sim->motorImpulse );
                writeFloat( "subStepTime", sim->subStepTime );
                writeFloat( "referenceAngle", sim->referenceAngle );
                writeVec( "localAxisA", sim->localAxisA );
                writer.writeBool( prefix + "enableSpring", sim->enableSpring );
                writer.writeBool( prefix + "enableLimit", sim->enableLimit );
                writer.writeBool( prefix + "enableMotor", sim->enableMotor );
                writeFloat( "targetTranslation", sim->targetTranslation );
                writeFloat( "lowerTranslation", sim->lowerTranslation );
                writeFloat( "upperTranslation", sim->upperTranslation );
                writeFloat( "motorSpeed", sim->motorSpeed );
            }
            else if( const auto* sim = std::get_if<weldJointSim2>( &storage ) )
            {
                writer.writeU32( prefix + "type", 3 );
                writeVec( "impulse", sim->impulse );
                writeFloat( "angularImpulse", sim->angularImpulse );
                writeFloat( "subStepTime", sim->subStepTime );
                writeFloat( "referenceAngle", sim->referenceAngle );
                writeFloat( "linearHertz", sim->linearHertz );
                writeFloat( "angularHertz", sim->angularHertz );
            }
            else if( const auto* sim = std::get_if<motorJointSim2>( &storage ) )
            {
                writer.writeU32( prefix + "type", 4 );
                writeVec( "linearVelocityImpulse", sim->linearVelocityImpulse );
                writeVec( "linearSpringImpulse", sim->linearSpringImpulse );
                writeFloat( "angularVelocityImpulse", sim->angularVelocityImpulse );
                writeFloat( "angularSpringImpulse", sim->angularSpringImpulse );
                writeFloat( "subStepTime", sim->subStepTime );
                writeFloat( "referenceAngle", sim->referenceAngle );
                writeFloat( "angularHertz", sim->angularHertz );
            }
            else if( const auto* sim = std::get_if<splineRailJointSim2>( &storage ) )
            {
                writer.writeU32( prefix + "type", 5 );
                writeVec( "impulse", sim->impulse );
                writeFloat( "springImpulse", sim->springImpulse );
                writeFloat( "lowerImpulse", sim->lowerImpulse );
                writeFloat( "upperImpulse", sim->upperImpulse );
                writeFloat( "motorImpulse", sim->motorImpulse );
                writeFloat( "subStepTime", sim->subStepTime );
                writeDouble( "length", sim->length );
                writeDouble( "referenceDistance", sim->referenceDistance );
                writeDouble( "distanceOffset", sim->distanceOffset );
                writer.writeS32( prefix + "parameter.segmentIndex",
                    static_cast<std::int32_t>( sim->parameter.segmentIndex ) );
                writeFloat( "parameter.t", sim->parameter.t );
                writer.writeBool( prefix + "trackingValid", sim->trackingValid );
                writer.writeBool( prefix + "isLoop", sim->isLoop );
                writer.writeBool( prefix + "enableRotation", sim->enableRotation );
                writer.writeBool( prefix + "enableSpring", sim->enableSpring );
                writer.writeBool( prefix + "enableMotor", sim->enableMotor );
                writer.writeBool( prefix + "enableLimit", sim->enableLimit );
                writeFloat( "tangentOffset", sim->tangentOffset );
                writeFloat( "referenceAngle", sim->referenceAngle );
                writeVec( "force", sim->force );
                writeFloat( "torque", sim->torque );
            }
            else
            {
                throw std::invalid_argument( "Joint cache snapshot does not cover this Joint type" );
            }
        }

        return std::move( writer ).finish();
    }
};

} // namespace zonai::determinism::test
