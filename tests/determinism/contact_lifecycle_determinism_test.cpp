#include <array>
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

void check( bool condition, const char* description )
{
    if( condition ) return;
    std::fprintf( stderr, "Contact lifecycle regression failed: %s\n", description );
    std::exit( EXIT_FAILURE );
}

[[nodiscard]] std::uint64_t fieldBits( const stateSnapshot& snapshot, const std::string& name )
{
    for( const snapshotField& field : snapshot.fields )
        if( field.name == name ) return field.bits;
    throw std::logic_error( "missing lifecycle snapshot field: " + name );
}

void refresh( world& simulation )
{
    simulation.UpdateCollisions( []( const contactData& ) {} );
}

[[nodiscard]] contactId findTouchingContact( const world& simulation, bodyId bodyId )
{
    std::array<contactData, 8> contacts{};
    const std::size_t count = simulation.GetBodyContactData( bodyId, contacts );
    check( count > 0 && count <= contacts.size(), "fixture needs an actual manifold" );
    return contacts[0].id;
}

struct recycleScene
{
    static constexpr float timeStep = 1.0f / 60.0f;
    static constexpr int substeps = 4;
    static constexpr std::uint32_t ticks = 40;

    world simulation{};
    std::array<bodyId, 2> bodies{};
    shapeId movableShape{};
    contactId current{};
    std::uint32_t reused = 0;
    std::uint32_t recycled = 0;
    bool sawDisabled = false;

    recycleScene()
    {
        simulation.SetGravity( {} );
        simulation.SetSleepingEnabled( false );
        simulation.SetContinuousEnabled( false );
        simulation.SetContactRecycleDistance( 0.3f );
        bodies[0] = simulation.CreateBody( bodyType::Static, { { 0.0f, -0.6f }, {} } );
        bodies[1] = simulation.CreateBody( bodyType::Dynamic, { { 0.0f, -0.03f }, {} } );
        simulation.SetBodyContactRecyclingEnabled( bodies[0], true );
        simulation.SetBodyContactRecyclingEnabled( bodies[1], true );
        ( void )simulation.CreateShape( bodies[0], MakeBox( { 2.0f, 0.25f } ) );
        movableShape = simulation.CreateShape( bodies[1], MakeBox( { 0.35f, 0.35f } ) );
        refresh( simulation );
        current = findTouchingContact( simulation, bodies[1] );
        check( simulation.IsValid( current ), "initial Contact handle" );

        // Identical geometry/poses must reuse the fresh narrowphase cache.
        refresh( simulation );
        check( simulation.GetRecycledContactCount() > 0,
            "recycling path must be exercised by an unchanged pair" );
        ++recycled;
    }

    void removeShape()
    {
        check( simulation.IsValid( current ), "Contact must exist before deletion" );
        const contactId stale = current;
        simulation.DestroyShape( movableShape );
        check( !simulation.IsValid( stale ), "destroyed Contact handle must be invalid" );
        check( simulation.GetContactCount() == 0, "destroy must remove the only pair" );
        check( simulation.GetBodyContactCapacity( bodies[1] ) == 0,
            "destroy must detach intrusive contact edges" );
    }

    void recreateShape( bool allowRecycling )
    {
        const contactId stale = current;
        simulation.SetBodyTransform( bodies[1], { { 0.0f, -0.03f }, {} } );
        simulation.SetBodyContactRecyclingEnabled( bodies[1], allowRecycling );
        movableShape = simulation.CreateShape( bodies[1], MakeBox( { 0.35f, 0.35f } ) );
        refresh( simulation );
        current = findTouchingContact( simulation, bodies[1] );
        check( current.index1 == stale.index1 && current.generation != stale.generation,
            "Contact free-list must reuse slot with a new generation" );
        check( !simulation.IsValid( stale ) && simulation.IsValid( current ),
            "old generation must never identify reused contact" );
        ++reused;

        const stateSnapshot snapshot = jointSubstepTraceAccess::captureContactLifecycle( simulation );
        const std::string p = "lifecycle.slot[" + std::to_string( current.index1 - 1 ) + "].";
        check( fieldBits( snapshot, p + "enableRecycling" ) == ( allowRecycling ? 1u : 0u ),
            "new Contact must snapshot body recycling preference at creation" );

        refresh( simulation );
        const bool recycledNow = simulation.GetRecycledContactCount() > 0;
        check( recycledNow == allowRecycling, "only recycling-enabled new Contact may reuse manifold" );
        if( recycledNow ) ++recycled;
        else sawDisabled = true;
    }

    void advance( std::uint32_t tick )
    {
        if( tick == 8 )
            removeShape();
        if( tick == 9 )
            recreateShape( true );
        if( tick == 18 )
        {
            removeShape();
            recreateShape( false );
        }
        if( tick == 24 )
        {
            removeShape();
            recreateShape( true );
        }
        if( tick == 30 )
        {
            const contactId stale = current;
            simulation.SetBodyTransform( bodies[1], { { 0.0f, 3.0f }, {} } );
            refresh( simulation );
            check( !simulation.IsValid( stale ), "fat AABB separation must destroy old Contact" );
            check( simulation.GetContactCount() == 0, "no Contact remains after separation" );
        }
        if( tick == 31 )
        {
            const contactId stale = current;
            simulation.SetBodyTransform( bodies[1], { { 0.0f, -0.03f }, {} } );
            refresh( simulation );
            current = findTouchingContact( simulation, bodies[1] );
            check( current.index1 == stale.index1 && current.generation != stale.generation,
                "AABB-separated pair must reuse slot with a fresh generation" );
            ++reused;
        }

        simulation.Step( timeStep, substeps );
        if( simulation.GetRecycledContactCount() > 0 ) ++recycled;
    }

    [[nodiscard]] stateSnapshot capture( std::uint32_t tick ) const
    {
        stateSnapshot result = captureWorldState( simulation, bodies, tick, timeStep, substeps );
        stateSnapshotWriter writer{};
        writer.writeU32( "lifecycle.reuseCount", reused );
        writer.writeU32( "lifecycle.recyclingObservedCount", recycled );
        writer.writeBool( "lifecycle.sawDisabledCase", sawDisabled );
        stateSnapshot extra = std::move( writer ).finish();
        stateSnapshot storage = jointSubstepTraceAccess::captureContactLifecycle( simulation );
        stateSnapshot contact = jointSubstepTraceAccess::capturePersistentContacts( simulation );

        for( const stateSnapshot* part : { &extra, &storage, &contact } )
        {
            result.bytes.insert( result.bytes.end(), part->bytes.begin(), part->bytes.end() );
            result.fields.insert( result.fields.end(), part->fields.begin(), part->fields.end() );
        }
        return result;
    }

    void verify() const
    {
        check( reused >= 4, "fixture must reuse a Contact slot repeatedly" );
        check( recycled >= 2, "fixture must execute actual Contact manifold recycling" );
        check( sawDisabled, "fixture must reject recycling for the new disabled Contact" );
    }
};

struct sleepScene
{
    static constexpr float timeStep = 1.0f / 60.0f;
    static constexpr int substeps = 4;
    static constexpr std::uint32_t ticks = 38;

    world simulation{};
    std::array<bodyId, 4> bodies{};
    bool sawSleepingIsland = false;
    bool sawWakingIsland = false;
    bool sawWorldWake = false;

    sleepScene()
    {
        simulation.SetGravity( {} );
        simulation.SetContinuousEnabled( false );
        simulation.SetContactRecycleDistance( 0.3f );
        bodies[0] = simulation.CreateBody( bodyType::Static, { { 0.0f, -0.6f }, {} } );
        ( void )simulation.CreateShape( bodies[0], MakeBox( { 5.0f, 0.25f } ) );

        for( std::size_t i = 1; i < bodies.size(); ++i )
        {
            bodyDef definition{};
            definition.type = bodyType::Dynamic;
            definition.transform.position = i == 1 ? vec2{ -2.0f, -0.03f } :
                i == 2 ? vec2{ -2.0f, 0.64f } : vec2{ 2.0f, -0.03f };
            definition.enableSleep = i != 3;
            bodies[i] = simulation.CreateBody( definition );
            ( void )simulation.CreateShape( bodies[i], MakeBox( { 0.35f, 0.35f } ) );
        }
        refresh( simulation );
        check( simulation.GetContactCount() >= 3,
            "sleep fixture requires the connected pair and independent floor contact" );
    }

    void advance( std::uint32_t tick )
    {
        if( tick == 12 || tick == 28 )
        {
            simulation.SetBodyTransform( bodies[1],
                { { -2.0f, -0.03f }, {} } );
            simulation.SetBodyTransform( bodies[2],
                { { -2.0f, 0.64f }, {} } );
            refresh( simulation );
            simulation.SetBodyAwake( bodies[1], false );
            check( !simulation.IsBodyAwake( bodies[1] ) &&
                !simulation.IsBodyAwake( bodies[2] ),
                "sleep must propagate through a solver-active dynamic Contact" );
            check( simulation.IsBodyAwake( bodies[3] ),
                "unconnected island must remain awake" );
        }
        if( tick == 20 )
        {
            simulation.SetBodyAwake( bodies[2], true );
            check( simulation.IsBodyAwake( bodies[1] ) &&
                simulation.IsBodyAwake( bodies[2] ),
                "waking a connected Body must wake its Contact island" );
        }
        if( tick == 29 )
        {
            simulation.SetSleepingEnabled( false );
            check( simulation.IsBodyAwake( bodies[1] ) &&
                simulation.IsBodyAwake( bodies[2] ) &&
                simulation.IsBodyAwake( bodies[3] ),
                "disabling world sleep must wake all non-static bodies" );
            sawWorldWake = true;
        }
        if( tick == 34 )
            simulation.SetSleepingEnabled( true );

        simulation.Step( timeStep, substeps );
        if( tick >= 12 && tick < 20 )
            check( !simulation.IsBodyAwake( bodies[1] ) &&
                !simulation.IsBodyAwake( bodies[2] ), "sleeping island must stay excluded until wake" );
        if( tick >= 20 && tick < 28 )
            check( simulation.IsBodyAwake( bodies[1] ) &&
                simulation.IsBodyAwake( bodies[2] ), "woken island must rejoin simulation" );
    }

    [[nodiscard]] stateSnapshot capture( std::uint32_t tick ) const
    {
        stateSnapshot result = captureWorldState( simulation, bodies, tick, timeStep, substeps );
        stateSnapshotWriter writer{};
        writer.writeBool( "lifecycle.sleepEnabled", simulation.IsSleepingEnabled() );
        for( std::size_t i = 1; i < bodies.size(); ++i )
            writer.writeBool( "lifecycle.awake[" + std::to_string( i ) + "]",
                simulation.IsBodyAwake( bodies[i] ) );
        stateSnapshot flags = std::move( writer ).finish();
        stateSnapshot contacts = jointSubstepTraceAccess::captureContactLifecycle( simulation );
        for( const stateSnapshot* part : { &flags, &contacts } )
        {
            result.bytes.insert( result.bytes.end(), part->bytes.begin(), part->bytes.end() );
            result.fields.insert( result.fields.end(), part->fields.begin(), part->fields.end() );
        }
        return result;
    }

    void observeIslands( const jointSubstepTraceRecorder& recorder, std::uint32_t tick )
    {
        const stateSnapshot& prepare = recorder.records.front().snapshot;
        const std::uint64_t count = fieldBits( prepare, "island.bodyCount" );
        if( tick >= 12 && tick < 20 )
        {
            check( count == 1 && fieldBits( prepare, "island.bodyOrder[0]" ) == 3,
                "sleeping connected bodies must disappear from active island graph" );
            sawSleepingIsland = true;
        }
        if( tick >= 20 && tick < 28 )
        {
            check( count == 3, "waking connected bodies must rejoin active island graph" );
            sawWakingIsland = true;
        }
    }

    void verify() const
    {
        check( sawSleepingIsland && sawWakingIsland && sawWorldWake,
            "fixture must cover sleep, contact-propagated wake and world-wide wake" );
    }
};

template<typename Fixture>
void runScene( const char* sceneName, std::FILE* output )
{
    Fixture first{}, second{}, unobserved{};
    jointSubstepTraceRecorder firstTrace{}, secondTrace{};
    firstTrace.bodies.assign( first.bodies.begin(), first.bodies.end() );
    secondTrace.bodies.assign( second.bodies.begin(), second.bodies.end() );
    firstTrace.captureContactAndIsland = true;
    secondTrace.captureContactAndIsland = true;
    jointSubstepTraceAccess::install( first.simulation, firstTrace );
    jointSubstepTraceAccess::install( second.simulation, secondTrace );

    for( std::uint32_t tick = 0; tick <= Fixture::ticks; ++tick )
    {
        if( tick != 0 )
        {
            first.advance( tick );
            second.advance( tick );
            unobserved.advance( tick );
            if constexpr( requires { first.observeIslands( firstTrace, tick ); } )
            {
                first.observeIslands( firstTrace, tick );
                second.observeIslands( secondTrace, tick );
            }
            compareAndEmitJointSubsteps( firstTrace, secondTrace,
                sceneName, tick, Fixture::substeps, output );
            firstTrace.records.clear();
            secondTrace.records.clear();
        }

        const stateSnapshot a = first.capture( tick );
        const stateSnapshot b = second.capture( tick );
        const stateSnapshot control = unobserved.capture( tick );
        for( const auto& [label, value] : {
            std::pair{ "second World", &b }, std::pair{ "unobserved World", &control } } )
        {
            const auto difference = firstDifference( a, *value );
            if( difference || a.bytes != value->bytes )
                throw std::runtime_error( std::string( sceneName ) + " tick " +
                    std::to_string( tick ) + " " + label + " differs at " +
                    ( difference ? difference->field : "snapshot.bytes" ) );
        }

        if( output )
        {
            for( const snapshotField& field : a.fields )
                std::fprintf( output, "%s.tick.%03u.%s\t%016llx/%u\n",
                    sceneName, tick, field.name.c_str(),
                    static_cast<unsigned long long>( field.bits ),
                    static_cast<unsigned>( field.byteCount ) );
        }
    }

    first.verify();
    second.verify();
    // The control World intentionally has no observer, so it cannot
    // accumulate island-stage coverage flags. Its full authoritative
    // Tick snapshots were still checked against both traced Worlds.
    jointSubstepTraceAccess::uninstall( first.simulation );
    jointSubstepTraceAccess::uninstall( second.simulation );
    if( output && std::ferror( output ) )
        throw std::runtime_error( "Contact lifecycle trace could not be written" );
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
                std::fprintf( stderr,
                    "Usage: contactLifecycleDeterminismTests [--emit TRACE.tsv]\n" );
                return EXIT_FAILURE;
            }
            output = std::fopen( argv[2], "wb" );
            if( !output )
                throw std::runtime_error( "cannot create Contact lifecycle output" );
        }

        runScene<recycleScene>( "recycleReuse", output );
        runScene<sleepScene>( "sleepWake", output );

        if( output && std::fclose( output ) != 0 )
            throw std::runtime_error( "could not close Contact lifecycle output" );
        std::puts( "Contact recycling/reuse and Sleep/Wake island traces passed" );
        return EXIT_SUCCESS;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "Contact lifecycle trace error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
