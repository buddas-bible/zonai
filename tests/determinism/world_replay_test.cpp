#include "world_replay_archive.h"

#include <algorithm>
#include <array>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <string>
#include <vector>

using namespace zonai;
using namespace zonai::determinism::test;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;
    std::fprintf( stderr, "World replay R1b failed: %s\n", message );
    std::exit( EXIT_FAILURE );
}

struct replayScene
{
    world simulation{};
    std::array<bodyId, 3> bodies{};

    replayScene()
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
};

[[nodiscard]] replayInput exampleInput()
{
    replayInput input{};
    input.tickCount = 60;
    input.timeStep = 1.0f / 60.0f;
    input.substepCount = 4;

    for( std::uint32_t tick = 1; tick <= input.tickCount; ++tick )
    {
        if( tick % 7 == 0 )
            input.commands.push_back( { tick, replayCommandType::forceToCenter, 1, 0.2f, 0.0f } );

        if( tick % 9 == 0 )
            input.commands.push_back( { tick, replayCommandType::impulseToCenter, 2, -0.08f, 0.01f } );

        if( tick == 20 )
        {
            // Same-Tick ordering must be recorded and replayed exactly as entered.
            input.commands.push_back( { tick, replayCommandType::setLinearVelocity, 1, 0.3f, 0.0f } );
            input.commands.push_back( { tick, replayCommandType::impulseToCenter, 1, 0.04f, 0.01f } );
        }

        if( tick == 40 )
            input.commands.push_back( { tick, replayCommandType::setGravity, 0, 0.0f, -6.0f } );
    }

    return input;
}

void applyCommand( replayScene& scene, const replayCommand& command )
{
    const vec2 value{ command.x, command.y };

    switch( command.type )
    {
    case replayCommandType::forceToCenter:
        scene.simulation.ApplyForceToCenter( scene.bodies[command.bodySlot], value );
        return;
    case replayCommandType::impulseToCenter:
        scene.simulation.ApplyLinearImpulseToCenter( scene.bodies[command.bodySlot], value );
        return;
    case replayCommandType::setLinearVelocity:
        scene.simulation.SetBodyLinearVelocity( scene.bodies[command.bodySlot], value );
        return;
    case replayCommandType::setGravity:
        scene.simulation.SetGravity( value );
        return;
    }

    throw std::invalid_argument( "unsupported replay command" );
}

[[nodiscard]] replayArchive executeReplay( const replayInput& input )
{
    validateReplayInput( input );
    replayScene scene{};

    replayArchive archive{};
    archive.inputBytes = encodeReplayInput( input );
    archive.frames.reserve( static_cast<std::size_t>( input.tickCount ) + 1 );
    archive.frames.push_back( captureWorldState( scene.simulation,
        scene.bodies, 0, input.timeStep, input.substepCount ) );

    std::size_t nextCommand = 0;

    for( std::uint32_t tick = 1; tick <= input.tickCount; ++tick )
    {
        while( nextCommand < input.commands.size() &&
            input.commands[nextCommand].tick == tick )
        {
            applyCommand( scene, input.commands[nextCommand] );
            ++nextCommand;
        }

        scene.simulation.Step( input.timeStep, static_cast<int>( input.substepCount ) );
        archive.frames.push_back( captureWorldState( scene.simulation,
            scene.bodies, tick, input.timeStep, input.substepCount ) );
    }

    check( nextCommand == input.commands.size(), "replay must consume every input command" );
    return archive;
}

int reportComparison( const replayArchive& expected, const replayArchive& actual )
{
    const auto mismatch = firstReplayDifference( expected, actual );

    if( !mismatch )
    {
        std::printf( "identical replay archives: %zu snapshots\n", expected.frames.size() );
        return EXIT_SUCCESS;
    }

    const snapshotDifference& d = mismatch->difference;
    std::fprintf( stderr, "first divergence: tick=%u field=%s left=0x%llx/%u right=0x%llx/%u\n",
        mismatch->tick, d.field.c_str(),
        static_cast<unsigned long long>( d.leftBits ), static_cast<unsigned>( d.leftWidth ),
        static_cast<unsigned long long>( d.rightBits ), static_cast<unsigned>( d.rightWidth ) );
    return EXIT_FAILURE;
}

void selfTest()
{
    const replayInput input = exampleInput();
    const auto inputBytes = encodeReplayInput( input );
    const replayInput restoredInput = decodeReplayInput( inputBytes );

    check( encodeReplayInput( restoredInput ) == inputBytes,
        "recorded Tick commands must round-trip bit-for-bit" );
    check( restoredInput.commands.size() == input.commands.size(),
        "recording must retain all commands" );

    const replayArchive first = executeReplay( restoredInput );
    const replayArchive second = executeReplay( restoredInput );

    check( !firstReplayDifference( first, second ),
        "two independent World replays must produce identical per-Tick bytes" );
    check( first.frames.size() == input.tickCount + 1, "trace must include Tick zero" );
    check( first.frames[0].bytes != first.frames.back().bytes,
        "fixed-input scene must actually advance" );

    const auto traceBytes = encodeReplayArchive( first );
    const replayArchive roundTrip = decodeReplayArchive( traceBytes );

    check( !firstReplayDifference( first, roundTrip ), "exported trace round-trip mismatch" );
    check( encodeReplayArchive( roundTrip ) == traceBytes,
        "trace encoder must emit identical portable bytes" );

    // Simulate one changed physical bit in a valid archived frame.
    replayArchive modified = roundTrip;
    constexpr std::uint32_t divergenceTick = 14;
    auto& frame = modified.frames[divergenceTick].bytes;
    const std::string target = "body[2].linearVelocity.x";
    const auto location = std::search( frame.begin(), frame.end(), target.begin(), target.end() );

    check( location != frame.end(), "target field must exist in archived snapshot" );

    const std::size_t valueOffset =
        static_cast<std::size_t>( location - frame.begin() ) + target.size() + 1;
    check( valueOffset + 4 <= frame.size(), "target float bits within frame" );
    frame[valueOffset] ^= 1u;
    modified.frames[divergenceTick] = decodeStateSnapshot( frame );

    const auto divergence = firstReplayDifference( first, modified );
    check( divergence && divergence->tick == divergenceTick &&
        divergence->difference.field == target,
        "first divergence must name the original Tick and changed Body field" );

    const replayArchive modifiedFromDiskFormat = decodeReplayArchive( encodeReplayArchive( modified ) );
    const auto diskMismatch = firstReplayDifference( first, modifiedFromDiskFormat );
    check( diskMismatch && diskMismatch->tick == divergenceTick &&
        diskMismatch->difference.field == target,
        "archive comparison must reproduce field mismatch after binary round-trip" );

    replayInput invalid = input;
    invalid.commands.push_back( { 1, replayCommandType::forceToCenter, 1, 0.0f, 0.0f } );
    bool rejectedUnsorted = false;

    try { ( void )encodeReplayInput( invalid ); }
    catch( const std::invalid_argument& ) { rejectedUnsorted = true; }
    check( rejectedUnsorted, "unsorted Tick commands must be rejected" );

    auto truncated = traceBytes;
    truncated.pop_back();
    bool rejectedTruncated = false;

    try { ( void )decodeReplayArchive( truncated ); }
    catch( const std::runtime_error& ) { rejectedTruncated = true; }
    check( rejectedTruncated, "truncated trace must be rejected" );

    // Exercise actual binary persistence, not only in-memory encoders.
    const auto folder = std::filesystem::current_path();
    const auto inputPath = folder / "zonai-r1b-test-input.bin";
    const auto tracePath = folder / "zonai-r1b-test-trace.bin";

    writeReplayFile( inputPath.string(), inputBytes );
    writeReplayFile( tracePath.string(), traceBytes );
    const auto savedInput = readReplayFile( inputPath.string() );
    const auto savedTrace = readReplayFile( tracePath.string() );

    check( savedInput == inputBytes && savedTrace == traceBytes,
        "binary file persistence must preserve exact bytes" );
    check( !firstReplayDifference( decodeReplayArchive( savedTrace ),
        executeReplay( decodeReplayInput( savedInput ) ) ),
        "recorded input file must reproduce the exported physics trace" );

    std::filesystem::remove( inputPath );
    std::filesystem::remove( tracePath );
    std::puts( "world replay R1b passed" );
}

} // namespace

int main( int argc, char** argv )
{
    try
    {
        if( argc == 1 )
        {
            selfTest();
            return EXIT_SUCCESS;
        }

        const std::string operation = argv[1];

        if( argc == 3 && operation == "--record-input" )
        {
            const auto input = encodeReplayInput( exampleInput() );
            writeReplayFile( argv[2], input );
            std::printf( "wrote replay input: %s\n", argv[2] );
            return EXIT_SUCCESS;
        }

        if( argc == 3 && operation == "--export" )
        {
            const auto trace = encodeReplayArchive( executeReplay( exampleInput() ) );
            writeReplayFile( argv[2], trace );
            std::printf( "exported replay snapshots: %s\n", argv[2] );
            return EXIT_SUCCESS;
        }

        if( argc == 4 && operation == "--replay" )
        {
            const replayInput input = decodeReplayInput( readReplayFile( argv[2] ) );
            const auto trace = encodeReplayArchive( executeReplay( input ) );
            writeReplayFile( argv[3], trace );
            std::printf( "replayed Tick inputs to: %s\n", argv[3] );
            return EXIT_SUCCESS;
        }

        if( argc == 3 && operation == "--verify" )
        {
            const replayArchive saved = decodeReplayArchive( readReplayFile( argv[2] ) );
            const replayInput input = decodeReplayInput( saved.inputBytes );
            return reportComparison( saved, executeReplay( input ) );
        }

        if( argc == 4 && operation == "--compare" )
        {
            const replayArchive left = decodeReplayArchive( readReplayFile( argv[2] ) );
            const replayArchive right = decodeReplayArchive( readReplayFile( argv[3] ) );
            return reportComparison( left, right );
        }

        std::fprintf( stderr,
            "Usage:\n"
            "  worldReplayTests                         (self-tests)\n"
            "  worldReplayTests --record-input FILE\n"
            "  worldReplayTests --export TRACE\n"
            "  worldReplayTests --replay INPUT TRACE\n"
            "  worldReplayTests --verify TRACE\n"
            "  worldReplayTests --compare TRACE_A TRACE_B\n" );
        return 2;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "World replay R1b error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
