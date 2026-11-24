#include "world_state_snapshot.h"

#include <array>
#include <bit>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <stdexcept>

using namespace zonai;
using namespace zonai::determinism::test;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "World snapshot test failed: %s\n", message );
    std::exit( EXIT_FAILURE );
}

void checkSame( const stateSnapshot& left, const stateSnapshot& right, int tick )
{
    const auto mismatch = firstDifference( left, right );

    if( mismatch )
    {
        std::fprintf( stderr, "World snapshot diverged at Tick %d, field %s:"
            " left=0x%llx/%u right=0x%llx/%u\n",
            tick, mismatch->field.c_str(),
            static_cast<unsigned long long>( mismatch->leftBits ), mismatch->leftWidth,
            static_cast<unsigned long long>( mismatch->rightBits ), mismatch->rightWidth );
        std::exit( EXIT_FAILURE );
    }

    // Compare full bytes. A matching FNV hash is not a proof of identical state.
    check( left.bytes == right.bytes, "canonical byte arrays differ" );
    check( left.hash() == right.hash(), "identical byte arrays have different hashes" );
}

struct worldScene
{
    world simulation{};
    std::array<bodyId, 3> bodies{};

    worldScene()
    {
        simulation.SetGravity( { 0.0f, -9.8f } );
        bodies[0] = simulation.CreateBody( bodyType::Static,
            { { 0.0f, -1.0f }, {} } );
        ( void )simulation.CreateShape( bodies[0], MakeBox( { 3.0f, 0.25f } ) );

        bodyDef first{};
        first.type = bodyType::Dynamic;
        first.transform.position = { -0.6f, 1.0f };
        first.linearVelocity = { 0.85f, 0.0f };
        first.angularVelocity = 0.2f;
        first.enableSleep = false;
        bodies[1] = simulation.CreateBody( first );
        ( void )simulation.CreateShape( bodies[1], circle2{ {}, 0.35f } );

        bodyDef second{};
        second.type = bodyType::Dynamic;
        second.transform.position = { 0.6f, 2.2f };
        second.linearVelocity = { -0.2f, 0.0f };
        second.enableSleep = false;
        bodies[2] = simulation.CreateBody( second );
        ( void )simulation.CreateShape( bodies[2], MakeBox( { 0.3f, 0.3f } ) );
    }

    void advance( int tick )
    {
        if( tick % 7 == 0 )
            simulation.ApplyForceToCenter( bodies[1], { 0.2f, 0.0f } );

        simulation.Step( 1.0f / 60.0f, 4 );
    }

    [[nodiscard]] stateSnapshot capture( std::uint64_t tick ) const
    {
        return captureWorldState( simulation, bodies, tick, 1.0f / 60.0f, 4 );
    }
};

void testWireFormat()
{
    stateSnapshotWriter writer{};
    writer.writeU32( "word", 0x01020304u );
    writer.writeFloat( "one", 1.0f );
    const stateSnapshot snapshot = std::move( writer ).finish();

    check( snapshot.fields.size() == 2, "wire field count" );
    check( snapshot.fields[0].bits == 0x01020304u && snapshot.fields[0].byteCount == 4,
        "fixed width unsigned encoding" );
    check( snapshot.fields[1].bits == std::bit_cast<std::uint32_t>( 1.0f ),
        "float bits must not be formatted or rounded" );

    // u16 name length + UTF-8 name + u8 width + u32 little endian.
    check( snapshot.bytes.size() == ( 2 + 4 + 1 + 4 ) + ( 2 + 3 + 1 + 4 ),
        "wire format size" );
    check( snapshot.bytes[7] == 0x04 && snapshot.bytes[8] == 0x03 &&
           snapshot.bytes[9] == 0x02 && snapshot.bytes[10] == 0x01,
        "u32 must use little-endian byte order" );
    check( snapshot.bytes[17] == 0x00 && snapshot.bytes[18] == 0x00 &&
           snapshot.bytes[19] == 0x80 && snapshot.bytes[20] == 0x3f,
        "float bits must use little-endian byte order" );

    stateSnapshotWriter plus{}, minus{};
    plus.writeFloat( "zero", 0.0f );
    minus.writeFloat( "zero", -0.0f );
    const stateSnapshot positive = std::move( plus ).finish();
    const stateSnapshot negative = std::move( minus ).finish();
    const auto signDifference = firstDifference( positive, negative );

    check( signDifference && signDifference->field == "zero" &&
        signDifference->leftBits == 0 && signDifference->rightBits == 0x80000000u,
        "signed zero must be bit-distinguishable" );

    bool rejectedNonFinite = false;
    try
    {
        stateSnapshotWriter invalid{};
        invalid.writeFloat( "invalid", std::numeric_limits<float>::infinity() );
    }
    catch( const std::domain_error& )
    {
        rejectedNonFinite = true;
    }
    check( rejectedNonFinite, "non-finite authoritative state must fail" );
}

void testCanonicalOrderingAndWorldLifetime()
{
    worldScene first{};
    worldScene second{};

    // Different world lifetime tokens must not affect the physical state snapshot.
    check( first.bodies[0].worldToken != second.bodies[0].worldToken,
        "fixture must have different World lifetimes" );
    checkSame( first.capture( 0 ), second.capture( 0 ), 0 );

    std::array<bodyId, 3> permuted{
        first.bodies[2], first.bodies[0], first.bodies[1]
    };
    const stateSnapshot unordered = captureWorldState(
        first.simulation, permuted, 0, 1.0f / 60.0f, 4 );
    checkSame( first.capture( 0 ), unordered, 0 );

    bool rejectedDuplicates = false;
    const std::array<bodyId, 2> duplicate{ first.bodies[0], first.bodies[0] };
    try
    {
        ( void )captureWorldState( first.simulation, duplicate, 0, 1.0f / 60.0f, 4 );
    }
    catch( const std::invalid_argument& )
    {
        rejectedDuplicates = true;
    }
    check( rejectedDuplicates, "duplicate Body handles must be rejected" );

    bool rejectedForeignHandle = false;
    try
    {
        ( void )captureWorldState( first.simulation, second.bodies, 0, 1.0f / 60.0f, 4 );
    }
    catch( const std::invalid_argument& )
    {
        rejectedForeignHandle = true;
    }
    check( rejectedForeignHandle, "foreign World Body handles must be rejected" );
}

void testRepeatedWorldStepsAndFirstDifference()
{
    worldScene first{};
    worldScene second{};

    const std::uint64_t initialHash = first.capture( 0 ).hash();

    for( int tick = 1; tick <= 50; ++tick )
    {
        first.advance( tick );
        second.advance( tick );
        checkSame( first.capture( tick ), second.capture( tick ), tick );
    }

    check( first.capture( 50 ).hash() != initialHash,
        "scene must change physical state across multiple ticks" );

    // A single velocity change must be reported as a named first divergent field.
    const stateSnapshot baseline = second.capture( 50 );
    const vec2 original = first.simulation.GetBodyLinearVelocity( first.bodies[1] );
    first.simulation.SetBodyLinearVelocity( first.bodies[1], { original.x + 1.0f, original.y } );
    const auto diff = firstDifference( baseline, first.capture( 50 ) );

    check( diff.has_value(), "velocity mutation must change the canonical snapshot" );
    check( diff->field == "body[2].linearVelocity.x",
        "first mismatch must identify the changed Body field" );

    const auto differentTick = firstDifference( first.capture( 50 ), first.capture( 51 ) );
    check( differentTick && differentTick->field == "tick",
        "snapshot must include fixed simulation Tick" );
}

} // namespace

int main()
{
    testWireFormat();
    testCanonicalOrderingAndWorldLifetime();
    testRepeatedWorldStepsAndFirstDifference();
    std::puts( "world state snapshot R1a passed" );
    return EXIT_SUCCESS;
}
