#include <array>
#include <bit>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <span>
#include <utility>
#include <stdexcept>
#include <string>

#include "world_state_snapshot.h"

using namespace zonai;
using namespace zonai::determinism::test;

namespace
{

void check( bool condition, const char* message )
{
    if( condition ) return;

    std::fprintf( stderr, "Spline Rail determinism failed: %s\n", message );
    std::exit( EXIT_FAILURE );
}

struct splineRailFixture
{
    static constexpr float timeStep = 1.0f / 60.0f;
    static constexpr int substeps = 4;

    world simulation{};
    std::array<bodyId, 2> bodies{};
    jointId rail{};
    bool loop = false;

    explicit splineRailFixture( bool closed ) : loop( closed )
    {
        simulation.SetGravity( {} );

        const vec2 openPoints[]{ { -4.0f, 0.0f }, { -2.0f, 0.0f },
            { 0.0f, 2.0f }, { 2.0f, 0.0f }, { 4.0f, 0.0f } };
        const vec2 loopPoints[]{ { 2.0f, 0.0f }, { 0.0f, 2.0f },
            { -2.0f, 0.0f }, { 0.0f, -2.0f } };

        bodies[0] = simulation.CreateBody();
        bodies[1] = simulation.CreateBody( bodyType::Dynamic,
            { loop ? loopPoints[0] : openPoints[0], {} } );
        ( void )simulation.CreateShape( bodies[1], circle2{ {}, 0.2f } );
        simulation.SetBodySleepEnabled( bodies[1], false );

        splineRailJointDef definition{};
        definition.bodyA = bodies[0];
        definition.bodyB = bodies[1];
        definition.points = loop ? std::span<const vec2>{ loopPoints } :
            std::span<const vec2>{ openPoints };
        definition.isLoop = loop;
        definition.rotationMode = loop ? splineRailRotationMode::followTangent :
            splineRailRotationMode::fixed;
        definition.enableAngularSpring = true;
        definition.angularHertz = 2.0f;
        definition.angularDampingRatio = 0.6f;
        definition.enableMotor = true;
        definition.motorSpeed = loop ? 2.5f : 1.5f;
        definition.maxMotorForce = 50.0f;

        if( !loop )
        {
            definition.enableLimit = true;
            definition.lowerTranslation = 0.0f;
            definition.upperTranslation = 7.0f;
        }

        rail = simulation.createSplineRailJoint( definition );
        check( simulation.IsValid( rail ), "Spline Rail fixture creation" );
    }

    void advance( std::uint32_t tick )
    {
        // Exercise motor, spring and cached impulses without random inputs.
        if( !loop && tick == 80 )
        {
            simulation.setSplineRailJointMotor( rail, false, 0.0f, 0.0f );
            simulation.setSplineRailJointSpring( rail, true, 2.0f, 0.8f, 1.0f );
        }

        if( !loop && tick == 160 )
        {
            simulation.setSplineRailJointSpring( rail, false, 0.0f, 0.8f, 0.0f );
            simulation.setSplineRailJointMotor( rail, true, 1.5f, 50.0f );
        }

        if( loop && tick == 150 )
            simulation.setSplineRailJointMotor( rail, true, -2.5f, 50.0f );

        simulation.Step( timeStep, substeps );
    }

    [[nodiscard]] stateSnapshot capture( std::uint32_t tick ) const
    {
        stateSnapshot snapshot = captureWorldState(
            simulation, bodies, tick, timeStep, substeps );
        const splineRailJointData data = simulation.getSplineRailJointData( rail );

        if( !data.trackingValid )
            throw std::runtime_error( "Spline Rail tracking invalid at tick " +
                std::to_string( tick ) );

        stateSnapshotWriter writer{};
        const auto writeDouble = [&]( std::string name, double value )
        {
            if( !std::isfinite( value ) )
                throw std::runtime_error( "non-finite Spline Rail field: " + name );

            writer.writeU64( std::move( name ), std::bit_cast<std::uint64_t>( value ) );
        };

        writer.writeU32( "rail.schema.version", 1 );
        writer.writeBool( "rail.isLoop", data.isLoop );
        writer.writeBool( "rail.trackingValid", data.trackingValid );
        writer.writeU32( "rail.segmentIndex", static_cast<std::uint32_t>( data.parameter.segmentIndex ) );
        writer.writeFloat( "rail.parameter.t", data.parameter.t );
        writeDouble( "rail.length", data.length );
        writeDouble( "rail.referenceDistance", data.referenceDistance );
        writeDouble( "rail.currentTranslation", data.currentTranslation );
        writer.writeFloat( "rail.currentSpeed", data.currentSpeed );
        writer.writeFloat( "rail.currentAngularSpeed", data.currentAngularSpeed );
        writer.writeFloat( "rail.currentAngle", data.currentAngle );
        // Read-only diagnostics of the follow-tangent angle math. Both raw
        // inputs are already present in this snapshot; the libm outputs
        // let us distinguish a transcendental difference from a solver one.
        const transform2 poseA = simulation.GetBodyTransform( bodies[0] );
        const transform2 poseB = simulation.GetBodyTransform( bodies[1] );
        const rot2 relativeRotation = Inverse( poseA.rotation ) * poseB.rotation;
        const float tangentAngle = std::atan2( data.tangent.y, data.tangent.x );
        const float relativeAngle = std::atan2( relativeRotation.s, relativeRotation.c );
        writer.writeFloat( "diagnostic.tangentAtan2", tangentAngle );
        writer.writeFloat( "diagnostic.relativeAtan2", relativeAngle );
        writer.writeFloat( "diagnostic.angleRemainder",
            std::remainder( relativeAngle - tangentAngle, 2.0f * std::numbers::pi_v<float> ) );
        writer.writeFloat( "rail.lateralError", data.lateralError );
        writer.writeFloat( "rail.anchorA.x", data.anchorA.x );
        writer.writeFloat( "rail.anchorA.y", data.anchorA.y );
        writer.writeFloat( "rail.anchorB.x", data.anchorB.x );
        writer.writeFloat( "rail.anchorB.y", data.anchorB.y );
        writer.writeFloat( "rail.tangent.x", data.tangent.x );
        writer.writeFloat( "rail.tangent.y", data.tangent.y );
        writer.writeFloat( "rail.normal.x", data.normal.x );
        writer.writeFloat( "rail.normal.y", data.normal.y );
        writer.writeFloat( "rail.force.x", data.force.x );
        writer.writeFloat( "rail.force.y", data.force.y );
        writer.writeFloat( "rail.torque", data.torque );
        writer.writeFloat( "rail.springForce", data.springForce );
        writer.writeFloat( "rail.motorForce", data.motorForce );

        stateSnapshot added = std::move( writer ).finish();
        snapshot.bytes.insert( snapshot.bytes.end(), added.bytes.begin(), added.bytes.end() );
        snapshot.fields.insert( snapshot.fields.end(), added.fields.begin(), added.fields.end() );
        return snapshot;
    }
};

void runFixture( bool loop, std::FILE* output )
{
    splineRailFixture first( loop ), second( loop );
    const char* label = loop ? "loop" : "open";

    for( std::uint32_t tick = 0; tick <= 240; ++tick )
    {
        if( tick != 0 )
        {
            first.advance( tick );
            second.advance( tick );
        }

        const stateSnapshot left = first.capture( tick );
        const stateSnapshot right = second.capture( tick );
        const auto difference = firstDifference( left, right );

        if( difference )
        {
            std::fprintf( stderr, "Spline Rail divergence: scene=%s tick=%u field=%s"
                " left=0x%llx right=0x%llx\n",
                label, tick, difference->field.c_str(),
                static_cast<unsigned long long>( difference->leftBits ),
                static_cast<unsigned long long>( difference->rightBits ) );
            std::exit( EXIT_FAILURE );
        }

        check( left.bytes == right.bytes, "separate World instances must match exactly" );

        if( output )
        {
            for( const snapshotField& field : left.fields )
            {
                std::fprintf( output, "%s.tick.%03u.%s\t%016llx/%u\n",
                    label, tick, field.name.c_str(),
                    static_cast<unsigned long long>( field.bits ),
                    static_cast<unsigned>( field.byteCount ) );
            }
        }
    }

    if( output && std::ferror( output ) )
        throw std::runtime_error( "Spline Rail fingerprint output failed" );
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
                std::fprintf( stderr, "Usage: splineRailDeterminismTests [--emit TRACE.tsv]\n" );
                return EXIT_FAILURE;
            }

            output = std::fopen( argv[2], "wb" );
            if( output == nullptr )
                throw std::runtime_error( "cannot create Spline Rail trace" );
        }

        runFixture( false, output );
        runFixture( true, output );

        if( output && std::fclose( output ) != 0 )
            throw std::runtime_error( "cannot close Spline Rail trace" );

        std::puts( "Spline Rail deterministic World snapshots passed" );
        return EXIT_SUCCESS;
    }
    catch( const std::exception& error )
    {
        std::fprintf( stderr, "Spline Rail determinism error: %s\n", error.what() );
        return EXIT_FAILURE;
    }
}
