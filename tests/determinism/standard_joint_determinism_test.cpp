#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <string>
#include <utility>

#include "joint_cache_snapshot.h"
#include "joint_substep_trace.h"
#include "world_state_snapshot.h"

using namespace zonai;
using namespace zonai::determinism::test;

namespace
{

void check( bool condition, const char* description )
{
    if( condition ) return;
    std::fprintf( stderr, "standard Joint determinism failed: %s\n", description );
    std::exit( EXIT_FAILURE );
}

enum class jointSceneType
{
    revolute,
    prismatic,
    weld,
    motor
};

[[nodiscard]] const char* sceneName( jointSceneType type )
{
    switch( type )
    {
    case jointSceneType::revolute: return "revolute";
    case jointSceneType::prismatic: return "prismatic";
    case jointSceneType::weld: return "weld";
    case jointSceneType::motor: return "motor";
    }
    return "invalid";
}

struct jointFixture
{
    static constexpr float timeStep = 1.0f / 60.0f;
    static constexpr int substeps = 4;

    world simulation{};
    std::array<bodyId, 2> bodies{};
    jointId constraint{};
    jointSceneType type{};

    explicit jointFixture( jointSceneType requestedType ) : type( requestedType )
    {
        simulation.SetGravity( {} );
        bodies[0] = simulation.CreateBody();
        bodies[1] = simulation.CreateBody( bodyType::Dynamic,
            { { 0.15f, 0.2f }, rot2::FromRadians( 0.35f ) } );
        ( void )simulation.CreateShape( bodies[1], circle2{ {}, 0.3f } );
        simulation.SetBodySleepEnabled( bodies[1], false );
        simulation.SetBodyLinearVelocity( bodies[1], { 0.3f, -0.15f } );
        simulation.SetBodyAngularVelocity( bodies[1], -0.7f );

        switch( type )
        {
        case jointSceneType::revolute:
        {
            revoluteJointDef definition{};
            definition.bodyA = bodies[0];
            definition.bodyB = bodies[1];
            definition.localAnchorB = { -0.15f, -0.2f };
            definition.enableLimit = true;
            definition.lowerAngle = -0.45f;
            definition.upperAngle = 0.45f;
            definition.enableMotor = true;
            definition.motorSpeed = 0.8f;
            definition.maxMotorTorque = 20.0f;
            constraint = simulation.createRevoluteJoint( definition );
            break;
        }
        case jointSceneType::prismatic:
        {
            prismaticJointDef definition{};
            definition.bodyA = bodies[0];
            definition.bodyB = bodies[1];
            definition.localAnchorB = { -0.15f, -0.2f };
            definition.enableSpring = true;
            definition.hertz = 2.0f;
            definition.dampingRatio = 0.7f;
            definition.targetTranslation = 0.0f;
            definition.enableLimit = true;
            definition.lowerTranslation = -1.0f;
            definition.upperTranslation = 1.0f;
            definition.enableMotor = true;
            definition.motorSpeed = 0.35f;
            definition.maxMotorForce = 20.0f;
            constraint = simulation.createPrismaticJoint( definition );
            break;
        }
        case jointSceneType::weld:
        {
            weldJointDef definition{};
            definition.bodyA = bodies[0];
            definition.bodyB = bodies[1];
            definition.localAnchorB = { -0.15f, -0.2f };
            definition.linearHertz = 2.5f;
            definition.linearDampingRatio = 0.7f;
            definition.angularHertz = 3.0f;
            definition.angularDampingRatio = 0.6f;
            constraint = simulation.createWeldJoint( definition );
            break;
        }
        case jointSceneType::motor:
        {
            motorJointDef definition{};
            definition.bodyA = bodies[0];
            definition.bodyB = bodies[1];
            definition.linearVelocity = { 0.0f, 0.0f };
            definition.maxVelocityForce = 15.0f;
            definition.angularVelocity = 0.3f;
            definition.maxVelocityTorque = 20.0f;
            definition.linearHertz = 2.0f;
            definition.linearDampingRatio = 0.65f;
            definition.maxSpringForce = 40.0f;
            definition.angularHertz = 3.0f;
            definition.angularDampingRatio = 0.7f;
            definition.maxSpringTorque = 30.0f;
            constraint = simulation.createMotorJoint( definition );
            break;
        }
        }

        check( simulation.IsValid( constraint ), "fixture Joint must be valid" );
    }

    void advance( std::uint32_t tick )
    {
        if( tick == 40 )
            simulation.ApplyAngularImpulse( bodies[1], 0.08f );
        if( tick == 80 )
            simulation.ApplyForceToCenter( bodies[1], { 0.2f, -0.3f } );
        if( tick == 120 )
            simulation.ApplyAngularImpulse( bodies[1], -0.12f );
        simulation.Step( timeStep, substeps );
    }

    [[nodiscard]] stateSnapshot capture( std::uint32_t tick ) const
    {
        stateSnapshot snapshot = captureWorldState(
            simulation, bodies, tick, timeStep, substeps );
        stateSnapshotWriter writer{};
        writer.writeU32( "joint.sceneType", static_cast<std::uint32_t>( type ) );
        const auto writeVector = [&]( const char* field, vec2 value )
        {
            writer.writeFloat( std::string( field ) + ".x", value.x );
            writer.writeFloat( std::string( field ) + ".y", value.y );
        };

        switch( type )
        {
        case jointSceneType::revolute:
        {
            const auto data = simulation.getRevoluteJointData( constraint );
            writer.writeFloat( "joint.currentAngle", data.currentAngle );
            writeVector( "joint.force", data.force );
            writer.writeFloat( "joint.torque", data.torque );
            writer.writeFloat( "joint.motorTorque", data.motorTorque );
            writeVector( "joint.anchorA", data.anchorA );
            writeVector( "joint.anchorB", data.anchorB );
            break;
        }
        case jointSceneType::prismatic:
        {
            const auto data = simulation.getPrismaticJointData( constraint );
            writer.writeFloat( "joint.currentAngle", data.currentAngle );
            writer.writeFloat( "joint.translation", data.currentTranslation );
            writer.writeFloat( "joint.lateralError", data.lateralError );
            writeVector( "joint.force", data.force );
            writer.writeFloat( "joint.torque", data.torque );
            writer.writeFloat( "joint.motorForce", data.motorForce );
            writer.writeFloat( "joint.springForce", data.springForce );
            break;
        }
        case jointSceneType::weld:
        {
            const auto data = simulation.getWeldJointData( constraint );
            writer.writeFloat( "joint.currentAngle", data.currentAngle );
            writeVector( "joint.force", data.force );
            writer.writeFloat( "joint.torque", data.torque );
            writeVector( "joint.anchorA", data.anchorA );
            writeVector( "joint.anchorB", data.anchorB );
            break;
        }
        case jointSceneType::motor:
        {
            const auto data = simulation.getMotorJointData( constraint );
            writeVector( "joint.force", data.force );
            writer.writeFloat( "joint.torque", data.torque );
            writeVector( "joint.anchorA", data.anchorA );
            writeVector( "joint.anchorB", data.anchorB );
            writer.writeFloat( "joint.referenceAngle", data.referenceAngle );
            break;
        }
        }

        stateSnapshot added = std::move( writer ).finish();
        snapshot.bytes.insert( snapshot.bytes.end(), added.bytes.begin(), added.bytes.end() );
        snapshot.fields.insert( snapshot.fields.end(), added.fields.begin(), added.fields.end() );

        // Persisted solver state is different from the public reaction query:
        // compare individual cached impulses and warm-start dt, not only their sum.
        const std::array<jointId, 1> jointIds{ constraint };
        stateSnapshot cache = jointCacheSnapshotAccess::capture( simulation, jointIds );
        snapshot.bytes.insert( snapshot.bytes.end(), cache.bytes.begin(), cache.bytes.end() );
        snapshot.fields.insert( snapshot.fields.end(), cache.fields.begin(), cache.fields.end() );
        return snapshot;
    }
};

void runScene( jointSceneType type, std::FILE* output )
{
    jointFixture first( type ), second( type );
    jointFixture withoutObserver( type );
    jointSubstepTraceRecorder firstTrace{}, secondTrace{};
    firstTrace.bodies.assign( first.bodies.begin(), first.bodies.end() );
    secondTrace.bodies.assign( second.bodies.begin(), second.bodies.end() );
    firstTrace.joints.push_back( first.constraint );
    secondTrace.joints.push_back( second.constraint );
    jointSubstepTraceAccess::install( first.simulation, firstTrace );
    jointSubstepTraceAccess::install( second.simulation, secondTrace );

    for( std::uint32_t tick = 0; tick <= 180; ++tick )
    {
        if( tick != 0 )
        {
            first.advance( tick );
            second.advance( tick );
            withoutObserver.advance( tick );
            compareAndEmitJointSubsteps( firstTrace, secondTrace,
                sceneName( type ), tick,
                jointFixture::substeps, output );
            firstTrace.records.clear();
            secondTrace.records.clear();
        }

        const stateSnapshot left = first.capture( tick );
        const stateSnapshot right = second.capture( tick );
        const auto difference = firstDifference( left, right );

        if( difference )
        {
            std::fprintf( stderr, "Joint divergence: scene=%s tick=%u field=%s"
                " left=0x%llx right=0x%llx\n",
                sceneName( type ), tick, difference->field.c_str(),
                static_cast<unsigned long long>( difference->leftBits ),
                static_cast<unsigned long long>( difference->rightBits ) );
            std::exit( EXIT_FAILURE );
        }

        check( left.bytes == right.bytes,
            "independent World instances must produce identical snapshot bytes" );
        // Explicitly verify that registering the read-only observer did not
        // perturb a third otherwise-identical, completely unobserved World.
        const stateSnapshot control = withoutObserver.capture( tick );
        if( const auto difference = firstDifference( left, control ) )
            throw std::runtime_error( std::string( "tracing changed authoritative state: " ) +
                difference->field );
        check( left.bytes == control.bytes, "tracing must not change physics" );

        if( output )
        {
            for( const snapshotField& field : left.fields )
            {
                std::fprintf( output, "%s.tick.%03u.%s\t%016llx/%u\n",
                    sceneName( type ), tick, field.name.c_str(),
                    static_cast<unsigned long long>( field.bits ),
                    static_cast<unsigned>( field.byteCount ) );
            }
        }
    }

    jointSubstepTraceAccess::uninstall( first.simulation );
    jointSubstepTraceAccess::uninstall( second.simulation );

    if( output && std::ferror( output ) )
        throw std::runtime_error( "Joint fingerprint output failed" );
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
                std::fprintf( stderr, "Usage: standardJointDeterminismTests [--emit TRACE.tsv]\n" );
                return EXIT_FAILURE;
            }

            output = std::fopen( argv[2], "wb" );
            if( !output )
                throw std::runtime_error( "cannot create standard Joint trace" );
        }

        for( const jointSceneType type : {
            jointSceneType::revolute,
            jointSceneType::prismatic,
            jointSceneType::weld,
            jointSceneType::motor } )
            runScene( type, output );

        if( output && std::fclose( output ) != 0 )
            throw std::runtime_error( "cannot close standard Joint trace" );

        std::puts( "four standard Joint replay scenes passed" );
        return EXIT_SUCCESS;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "standard Joint determinism error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
