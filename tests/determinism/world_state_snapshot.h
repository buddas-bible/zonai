#pragma once

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <optional>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "dynamics/world.h"

namespace zonai::determinism::test
{

// R1a test-only format. No struct-memory dumps, pointer values or world lifetime tokens.
struct snapshotField
{
    std::string name;
    std::uint64_t bits = 0;
    std::uint8_t byteCount = 0;
};

struct stateSnapshot
{
    std::vector<std::uint8_t> bytes;
    std::vector<snapshotField> fields;

    [[nodiscard]] std::uint64_t hash() const noexcept
    {
        // Diagnostic checksum only. Equality must compare the complete bytes/fields.
        std::uint64_t result = 14695981039346656037ull;

        for( const std::uint8_t byte : bytes )
        {
            result ^= byte;
            result *= 1099511628211ull;
        }

        return result;
    }
};

struct snapshotDifference
{
    std::string field;
    std::uint64_t leftBits = 0;
    std::uint64_t rightBits = 0;
    std::uint8_t leftWidth = 0;
    std::uint8_t rightWidth = 0;
};

class stateSnapshotWriter
{
public:
    void writeU8( std::string name, std::uint8_t value ) { writeField( std::move( name ), value, 1 ); }
    void writeU16( std::string name, std::uint16_t value ) { writeField( std::move( name ), value, 2 ); }
    void writeU32( std::string name, std::uint32_t value ) { writeField( std::move( name ), value, 4 ); }
    void writeU64( std::string name, std::uint64_t value ) { writeField( std::move( name ), value, 8 ); }

    void writeS32( std::string name, std::int32_t value )
    {
        writeU32( std::move( name ), static_cast<std::uint32_t>( value ) );
    }

    void writeBool( std::string name, bool value )
    {
        writeU8( std::move( name ), value ? 1 : 0 );
    }

    void writeFloat( std::string name, float value )
    {
        static_assert( sizeof( float ) == sizeof( std::uint32_t ) );
        static_assert( std::numeric_limits<float>::is_iec559 );

        if( !std::isfinite( value ) )
            throw std::domain_error( "non-finite authoritative float: " + name );

        // Preserve signed zero and every IEEE-754 bit. Do not stringify or round.
        writeU32( std::move( name ), std::bit_cast<std::uint32_t>( value ) );
    }

    [[nodiscard]] stateSnapshot finish() && { return std::move( snapshot_ ); }

private:
    void appendLittleEndian( std::uint64_t value, std::uint8_t byteCount )
    {
        for( std::uint8_t i = 0; i < byteCount; ++i )
            snapshot_.bytes.push_back( static_cast<std::uint8_t>( value >> ( 8 * i ) ) );
    }

    void writeField( std::string name, std::uint64_t bits, std::uint8_t byteCount )
    {
        if( name.empty() || name.size() > std::numeric_limits<std::uint16_t>::max() )
            throw std::invalid_argument( "invalid snapshot field name" );

        // v1 wire record: u16 name size LE, UTF-8 bytes, u8 value width, LE value bits.
        appendLittleEndian( static_cast<std::uint16_t>( name.size() ), 2 );
        for( const unsigned char letter : name ) snapshot_.bytes.push_back( letter );
        snapshot_.bytes.push_back( byteCount );
        appendLittleEndian( bits, byteCount );
        snapshot_.fields.push_back( { std::move( name ), bits, byteCount } );
    }

    stateSnapshot snapshot_{};
};

[[nodiscard]] inline std::optional<snapshotDifference> firstDifference(
    const stateSnapshot& left, const stateSnapshot& right )
{
    const std::size_t shared = std::min( left.fields.size(), right.fields.size() );

    for( std::size_t i = 0; i < shared; ++i )
    {
        const snapshotField& a = left.fields[i];
        const snapshotField& b = right.fields[i];

        if( a.name != b.name || a.bits != b.bits || a.byteCount != b.byteCount )
        {
            return snapshotDifference{
                a.name == b.name ? a.name : a.name + " != " + b.name,
                a.bits, b.bits, a.byteCount, b.byteCount
            };
        }
    }

    if( left.fields.size() != right.fields.size() )
    {
        const bool leftHasExtra = left.fields.size() > shared;
        return snapshotDifference{
            leftHasExtra ? left.fields[shared].name : right.fields[shared].name,
            leftHasExtra ? left.fields[shared].bits : 0,
            leftHasExtra ? 0 : right.fields[shared].bits,
            leftHasExtra ? left.fields[shared].byteCount : 0,
            leftHasExtra ? 0 : right.fields[shared].byteCount
        };
    }

    // The field records completely describe this format; catch an encoding regression.
    if( left.bytes != right.bytes )
        return snapshotDifference{ "snapshot.encoding", left.hash(), right.hash(), 8, 8 };

    return std::nullopt;
}

// R1a intentionally covers public World settings and tracked Body observations.
// It is NOT yet a complete authoritative state snapshot: Contact/Joint caches,
// shape/proxy internals, pending forces and fluid history need dedicated accessors
// and versioned schemas in follow-up R1 work.
[[nodiscard]] inline stateSnapshot captureWorldState(
    const world& simulation, std::span<const bodyId> trackedBodyIds,
    std::uint64_t tick, float timeStep, std::uint32_t substepCount )
{
    if( substepCount == 0 || !std::isfinite( timeStep ) || timeStep <= 0.0f )
        throw std::invalid_argument( "invalid fixed Tick configuration" );

    std::vector<bodyId> ordered( trackedBodyIds.begin(), trackedBodyIds.end() );

    for( const bodyId id : ordered )
    {
        if( !simulation.IsValid( id ) )
            throw std::invalid_argument( "snapshot contains an invalid or foreign Body handle" );
    }

    std::sort( ordered.begin(), ordered.end(),
        []( bodyId a, bodyId b )
        {
            if( a.index1 != b.index1 ) return a.index1 < b.index1;
            return a.generation < b.generation;
        } );

    for( std::size_t i = 1; i < ordered.size(); ++i )
    {
        if( ordered[i - 1].index1 == ordered[i].index1 &&
            ordered[i - 1].generation == ordered[i].generation )
            throw std::invalid_argument( "snapshot contains duplicate Body handles" );
    }

    stateSnapshotWriter writer{};
    writer.writeU32( "schema.magic", 0x31414e5au ); // "ZNA1" in little endian.
    writer.writeU32( "schema.version", 1 );
    writer.writeU64( "tick", tick );
    writer.writeFloat( "step.dt", timeStep );
    writer.writeU32( "step.substeps", substepCount );

    const vec2 gravity = simulation.GetGravity();
    writer.writeFloat( "world.gravity.x", gravity.x );
    writer.writeFloat( "world.gravity.y", gravity.y );
    writer.writeFloat( "world.maximumLinearSpeed", simulation.GetMaximumLinearSpeed() );
    writer.writeFloat( "world.contactRecycleDistance", simulation.GetContactRecycleDistance() );
    writer.writeFloat( "world.contactHitSpeedThreshold", simulation.getContactHitSpeedThreshold() );
    writer.writeBool( "world.sleepingEnabled", simulation.IsSleepingEnabled() );
    writer.writeBool( "world.continuousEnabled", simulation.IsContinuousEnabled() );
    writer.writeU64( "world.bodyCount", simulation.GetBodyCount() );
    writer.writeU64( "world.shapeCount", simulation.GetShapeCount() );
    writer.writeU64( "world.contactCount", simulation.GetContactCount() );
    writer.writeU64( "world.jointCount", simulation.getJointCount() );
    writer.writeU64( "snapshot.trackedBodyCount", ordered.size() );

    for( const bodyId id : ordered )
    {
        const std::string prefix = "body[" + std::to_string( id.index1 ) + "].";
        const body& record = simulation.GetBody( id );
        const transform2 pose = simulation.GetBodyTransform( id );
        const vec2 velocity = simulation.GetBodyLinearVelocity( id );
        const vec2 localCenter = simulation.GetBodyLocalCenter( id );

        // A World token is only an opaque per-world lifetime guard, not physics state.
        writer.writeS32( prefix + "index1", id.index1 );
        writer.writeU16( prefix + "generation", id.generation );
        writer.writeU32( prefix + "type", static_cast<std::uint32_t>( record.type ) );
        writer.writeFloat( prefix + "position.x", pose.position.x );
        writer.writeFloat( prefix + "position.y", pose.position.y );
        writer.writeFloat( prefix + "rotation.c", pose.rotation.c );
        writer.writeFloat( prefix + "rotation.s", pose.rotation.s );
        writer.writeFloat( prefix + "linearVelocity.x", velocity.x );
        writer.writeFloat( prefix + "linearVelocity.y", velocity.y );
        writer.writeFloat( prefix + "angularVelocity", simulation.GetBodyAngularVelocity( id ) );
        writer.writeFloat( prefix + "mass", simulation.GetBodyMass( id ) );
        writer.writeFloat( prefix + "inertia", simulation.GetBodyRotationalInertia( id ) );
        writer.writeFloat( prefix + "localCenter.x", localCenter.x );
        writer.writeFloat( prefix + "localCenter.y", localCenter.y );
        writer.writeFloat( prefix + "linearDamping", simulation.GetBodyLinearDamping( id ) );
        writer.writeFloat( prefix + "angularDamping", simulation.GetBodyAngularDamping( id ) );
        writer.writeFloat( prefix + "gravityScale", simulation.GetBodyGravityScale( id ) );
        writer.writeBool( prefix + "awake", simulation.IsBodyAwake( id ) );
        writer.writeBool( prefix + "sleepEnabled", simulation.IsBodySleepEnabled( id ) );
        writer.writeFloat( prefix + "sleepTime", record.sleepTime );
        writer.writeFloat( prefix + "sleepThreshold", simulation.GetBodySleepThreshold( id ) );
        writer.writeFloat( prefix + "safetyFactor", simulation.GetBodySafetyFactor( id ) );
        writer.writeBool( prefix + "fast", simulation.IsBodyFast( id ) );
        writer.writeBool( prefix + "bullet", simulation.IsBodyBullet( id ) );
        writer.writeBool( prefix + "timeOfImpact", simulation.HadBodyTimeOfImpact( id ) );
        writer.writeBool( prefix + "fastRotationAllowed", simulation.IsBodyFastRotationAllowed( id ) );
        writer.writeBool( prefix + "contactRecyclingEnabled", simulation.IsBodyContactRecyclingEnabled( id ) );

        writer.writeS32( prefix + "contactCount", record.contactCount );
        writer.writeS32( prefix + "jointCount", record.jointCount );
        writer.writeS32( prefix + "chainCount", record.chainCount );
        writer.writeS32( prefix + "shapeCount", record.shapeCount );
        writer.writeS32( prefix + "headContactKey", record.headContactKey );
        writer.writeS32( prefix + "headJointKey", record.headJointKey );
        writer.writeS32( prefix + "headChainId", record.headChainId );
        writer.writeS32( prefix + "headShapeId", record.headShapeId );
    }

    return std::move( writer ).finish();
}

} // namespace zonai::determinism::test
