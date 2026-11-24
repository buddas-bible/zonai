#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#include "joint_substep_trace.h"

using namespace zonai;
using namespace zonai::determinism::test;

namespace
{

void check( bool value, const char* description )
{
    if( value ) return;
    std::fprintf( stderr, "Contact/Island determinism failed: %s\n", description );
    std::exit( EXIT_FAILURE );
}

struct contactFixture
{
    static constexpr float timeStep = 1.0f / 60.0f;
    static constexpr int substeps = 4;

    world simulation{};
    std::array<bodyId, 4> bodies{};

    contactFixture()
    {
        simulation.SetGravity( { 0.0f, -9.8f } );

        // A shared static floor does NOT union dynamic bodies into one Island.
        // One stack on the left and one separate box on the right create two
        // independently ordered solver components with two-point manifolds.
        bodies[0] = simulation.CreateBody( bodyType::Static, { { 0.0f, -0.6f }, {} } );
        ( void )simulation.CreateShape( bodies[0], MakeBox( { 5.0f, 0.25f } ) );

        const auto makeDynamic = [&]( std::size_t slot, vec2 position, vec2 velocity )
        {
            bodyDef definition{};
            definition.type = bodyType::Dynamic;
            definition.transform.position = position;
            definition.linearVelocity = velocity;
            definition.enableSleep = false;
            bodies[slot] = simulation.CreateBody( definition );
            ( void )simulation.CreateShape( bodies[slot], MakeBox( { 0.35f, 0.35f } ) );
        };

        makeDynamic( 1, { -2.0f, -0.03f }, { 0.65f, -0.1f } );
        makeDynamic( 2, { 2.0f, -0.03f }, { -0.35f, -0.15f } );
        makeDynamic( 3, { -2.0f, 0.65f }, { 0.1f, -0.2f } );
    }

    void advance( std::uint32_t tick )
    {
        if( tick == 20 )
            simulation.ApplyLinearImpulseToCenter( bodies[1], { 0.2f, 0.0f } );
        if( tick == 40 )
            simulation.ApplyAngularImpulse( bodies[3], 0.15f );
        if( tick == 60 )
            simulation.ApplyLinearImpulseToCenter( bodies[2], { -0.1f, -0.05f } );
        simulation.Step( timeStep, substeps );
    }

    [[nodiscard]] stateSnapshot capture( std::uint32_t tick ) const
    {
        stateSnapshot snapshot = captureWorldState(
            simulation, bodies, tick, timeStep, substeps );
        stateSnapshot cache = jointSubstepTraceAccess::capturePersistentContacts( simulation );
        snapshot.bytes.insert( snapshot.bytes.end(), cache.bytes.begin(), cache.bytes.end() );
        snapshot.fields.insert( snapshot.fields.end(), cache.fields.begin(), cache.fields.end() );
        return snapshot;
    }
};

void runFixture( std::FILE* output )
{
    contactFixture first{}, second{}, withoutObserver{};
    jointSubstepTraceRecorder firstTrace{}, secondTrace{};
    firstTrace.bodies.assign( first.bodies.begin(), first.bodies.end() );
    secondTrace.bodies.assign( second.bodies.begin(), second.bodies.end() );
    firstTrace.captureContactAndIsland = true;
    secondTrace.captureContactAndIsland = true;
    jointSubstepTraceAccess::install( first.simulation, firstTrace );
    jointSubstepTraceAccess::install( second.simulation, secondTrace );

    constexpr const char* scene = "contactIslands";
    constexpr std::uint32_t totalTicks = 90;

    for( std::uint32_t tick = 0; tick <= totalTicks; ++tick )
    {
        if( tick != 0 )
        {
            first.advance( tick );
            second.advance( tick );
            withoutObserver.advance( tick );
            compareAndEmitJointSubsteps( firstTrace, secondTrace,
                scene, tick, contactFixture::substeps, output );
            firstTrace.records.clear();
            secondTrace.records.clear();
        }

        const stateSnapshot a = first.capture( tick );
        const stateSnapshot b = second.capture( tick );
        const stateSnapshot control = withoutObserver.capture( tick );

        for( const auto& [name, other] : {
            std::pair{ "independent World", &b },
            std::pair{ "World without observer", &control } } )
        {
            const auto difference = firstDifference( a, *other );
            if( difference || a.bytes != other->bytes )
            {
                const std::string field = difference ? difference->field : "snapshot.rawBytes";
                throw std::runtime_error( std::string( scene ) + " Tick " +
                    std::to_string( tick ) + " " + name + " first difference: " + field );
            }
        }

        if( output )
        {
            for( const snapshotField& field : a.fields )
            {
                std::fprintf( output, "%s.tick.%03u.%s\t%016llx/%u\n",
                    scene, tick, field.name.c_str(),
                    static_cast<unsigned long long>( field.bits ),
                    static_cast<unsigned>( field.byteCount ) );
            }
        }
    }

    jointSubstepTraceAccess::uninstall( first.simulation );
    jointSubstepTraceAccess::uninstall( second.simulation );

    check( firstTrace.maxContacts >= 2, "fixture must form multiple live Contacts" );
    check( firstTrace.maxIslands >= 2, "fixture must form at least two solver Islands" );
    check( firstTrace.maxPointCount == 2, "fixture must exercise a two-point manifold" );
    check( firstTrace.sawNormalImpulse, "fixture must solve nonzero normal impulse" );
    check( firstTrace.sawTangentImpulse, "fixture must solve nonzero friction impulse" );
    check( firstTrace.maxContacts == secondTrace.maxContacts &&
        firstTrace.maxIslands == secondTrace.maxIslands &&
        firstTrace.maxPointCount == secondTrace.maxPointCount &&
        firstTrace.sawNormalImpulse == secondTrace.sawNormalImpulse &&
        firstTrace.sawTangentImpulse == secondTrace.sawTangentImpulse,
        "Contact fixture coverage must be reproducible" );

    if( output && std::ferror( output ) )
        throw std::runtime_error( "contact/island trace write failed" );
}

} // namespace

int main( int argc, char** argv )
{
    try
    {
        std::FILE* output = nullptr;
        if( argc != 1 )
        {
            if( argc != 3 || std::string( argv[1] ) != "--emit" )
            {
                std::fprintf( stderr, "Usage: contactIslandDeterminismTests [--emit TRACE.tsv]\n" );
                return EXIT_FAILURE;
            }

            output = std::fopen( argv[2], "wb" );
            if( output == nullptr )
                throw std::runtime_error( "cannot create Contact/Island trace" );
        }

        runFixture( output );

        if( output && std::fclose( output ) != 0 )
            throw std::runtime_error( "cannot close Contact/Island trace" );

        std::puts( "contact/island substep observations and unobserved control World passed" );
        return EXIT_SUCCESS;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "Contact/Island trace error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
