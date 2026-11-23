#include <array>
#include <bit>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <string_view>

#include "joint_cache_snapshot.h"

using namespace zonai;
using namespace zonai::determinism::test;

namespace
{

void check( bool valid, const char* description )
{
    if( valid ) return;
    std::fprintf( stderr, "Joint cache snapshot failed: %s\n", description );
    std::exit( EXIT_FAILURE );
}

[[nodiscard]] std::uint64_t fieldBits( const stateSnapshot& snapshot, std::string_view name )
{
    for( const snapshotField& field : snapshot.fields )
        if( field.name == name ) return field.bits;

    throw std::invalid_argument( "missing named persistent Joint cache field" );
}

template<typename Action>
void expectInvalidHandle( Action action )
{
    try
    {
        action();
    }
    catch( const std::invalid_argument& )
    {
        return;
    }
    check( false, "invalid, stale, foreign, and duplicate Joint handles must be rejected" );
}

struct simpleJointScene
{
    world simulation{};
    bodyId ground{}, moving{};
    jointId hinge{};

    simpleJointScene()
    {
        simulation.SetGravity( {} );
        ground = simulation.CreateBody();
        moving = simulation.CreateBody( bodyType::Dynamic,
            { {}, rot2::FromRadians( 0.15f ) } );
        ( void )simulation.CreateShape( moving, circle2{ {}, 0.25f } );
        simulation.SetBodySleepEnabled( moving, false );

        revoluteJointDef definition{};
        definition.bodyA = ground;
        definition.bodyB = moving;
        definition.enableMotor = true;
        definition.motorSpeed = 1.5f;
        definition.maxMotorTorque = 20.0f;
        definition.enableLimit = true;
        definition.lowerAngle = -0.75f;
        definition.upperAngle = 0.75f;
        hinge = simulation.createRevoluteJoint( definition );
        check( simulation.IsValid( hinge ), "hinge creation" );
    }

    [[nodiscard]] stateSnapshot capture() const
    {
        const std::array<jointId, 1> ids{ hinge };
        return jointCacheSnapshotAccess::capture( simulation, ids );
    }
};

void verifyRawCacheAndReset()
{
    simpleJointScene first, second;

    check( first.hinge.worldToken != second.hinge.worldToken,
        "distinct Worlds have distinct opaque handle tokens" );
    const stateSnapshot initial = first.capture();
    check( initial.bytes == second.capture().bytes,
        "initial cache must ignore the World-specific identity token" );
    check( fieldBits( initial, "cache.joint[1].motorImpulse" ) == 0,
        "motor impulse is zero before the first step" );
    check( fieldBits( initial, "cache.joint[1].subStepTime" ) == 0,
        "cache time step starts at zero" );

    first.simulation.Step( 1.0f / 60.0f, 4 );
    second.simulation.Step( 1.0f / 60.0f, 4 );
    const stateSnapshot warmed = first.capture();
    check( warmed.bytes == second.capture().bytes,
        "persistent cache bits must match between two World instances" );
    check( fieldBits( warmed, "cache.joint[1].motorImpulse" ) != 0,
        "running motor must accumulate nonzero angular impulse" );
    check( fieldBits( warmed, "cache.joint[1].subStepTime" ) ==
        std::bit_cast<std::uint32_t>( ( 1.0f / 60.0f ) / 4.0f ),
        "cached impulse substep time is stored exactly" );
    check( warmed.bytes != initial.bytes,
        "running the solver must update persistent cache state" );

    first.simulation.setRevoluteJointMotor( first.hinge, true, -1.5f, 20.0f );
    const stateSnapshot reset = first.capture();
    check( fieldBits( reset, "cache.joint[1].motorImpulse" ) == 0,
        "changing motor speed immediately invalidates motor warm-start cache" );
    check( fieldBits( reset, "cache.joint[1].subStepTime" ) ==
        fieldBits( warmed, "cache.joint[1].subStepTime" ),
        "changing motor speed should not erase the cached solver time" );
}

void verifyHandleSafetyAndGeneration()
{
    simpleJointScene local, foreign;
    const std::array<jointId, 2> duplicates{ local.hinge, local.hinge };
    const std::array<jointId, 1> invalid{ jointId{} };
    const std::array<jointId, 1> alien{ foreign.hinge };

    expectInvalidHandle( [&] { ( void )jointCacheSnapshotAccess::capture( local.simulation, duplicates ); } );
    expectInvalidHandle( [&] { ( void )jointCacheSnapshotAccess::capture( local.simulation, invalid ); } );
    expectInvalidHandle( [&] { ( void )jointCacheSnapshotAccess::capture( local.simulation, alien ); } );

    const jointId stale = local.hinge;
    local.simulation.destroyJoint( stale );
    check( !local.simulation.IsValid( stale ), "destroyed Joint handle becomes invalid" );
    const std::array<jointId, 1> staleHandles{ stale };
    expectInvalidHandle( [&] { ( void )jointCacheSnapshotAccess::capture( local.simulation, staleHandles ); } );

    revoluteJointDef definition{};
    definition.bodyA = local.ground;
    definition.bodyB = local.moving;
    local.hinge = local.simulation.createRevoluteJoint( definition );
    check( local.simulation.IsValid( local.hinge ), "reused Joint is valid" );
    check( local.hinge.index1 == stale.index1 &&
        local.hinge.generation != stale.generation,
        "joint free-list reuses slot but advances generation" );
    check( fieldBits( local.capture(), "cache.joint[1].generation" ) ==
        local.hinge.generation,
        "the snapshot stores the new generation without a World token" );
}

} // namespace

int main()
{
    try
    {
        verifyRawCacheAndReset();
        verifyHandleSafetyAndGeneration();
        std::puts( "persistent Joint cache inspection and lifecycle passed" );
        return EXIT_SUCCESS;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "Joint cache snapshot error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
