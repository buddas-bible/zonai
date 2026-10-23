#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <utility>
#include <limits>
#include "dynamics/world.h"

using namespace zonai;

namespace
{
void check( bool ok, const char* message )
{
    if( ok ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

void checkStraight()
{
    for( int mode : { 0, 1, 2 } )
    {
        world physics{};
        physics.SetGravity( {} );
        const auto a = physics.CreateBody( bodyType::Kinematic, { {}, rot2::FromRadians( 0.4f ) } );
        const auto b = physics.CreateBody( bodyType::Dynamic );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        const vec2 points[] = { { -4, 0 }, { 4, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
        definition.enableAngularSpring = mode == 2;
        definition.enableMotor = true;
        definition.motorSpeed = 9.0f;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "straight angular speed fixture is created" );
        physics.SetBodyLinearVelocity( a, { 2, 3 } );
        physics.SetBodyLinearVelocity( b, { 5, -4 } );
        physics.SetBodyAngularVelocity( a, 0.7f );
        physics.SetBodyAngularVelocity( b, -1.2f );
        auto data = physics.getSplineRailJointData( id );
        check( data.trackingValid && std::abs( data.currentAngularSpeed + 1.9f ) < 0.000001f, "straight angular error speed is relative angular velocity in every mode and spring setting" );
        physics.SetBodyAngularVelocity( b, 2.3f );
        data = physics.getSplineRailJointData( id );
        check( std::abs( data.currentAngularSpeed - 1.6f ) < 0.000001f, "relative angular error speed preserves its sign" );
        check( physics.setSplineRailJointRotationReference( id, 0.9f ) && physics.getSplineRailJointData( id ).currentAngularSpeed == data.currentAngularSpeed, "constant rotation reference changes do not change the instantaneous error rate" );
    }
}

transform2 advance( const world& physics, bodyId id, transform2 pose, float time )
{
    const vec2 center = physics.GetBodyLocalCenter( id );
    const vec2 position = TransformPoint( pose, center ) + time * physics.GetBodyLinearVelocity( id );
    pose.rotation = rot2::FromRadians( time * physics.GetBodyAngularVelocity( id ) ) * pose.rotation;
    pose.position = position - Rotate( pose.rotation, center );
    return pose;
}

void checkDerivative( world& physics, jointId id, float step = 0.001f )
{
    const auto before = physics.getSplineRailJointData( id );
    check( before.trackingValid, "angular derivative starts with valid tracking" );
    const auto a = before.bodyA, b = before.bodyB;
    const auto poseA = physics.GetBodyTransform( a ), poseB = physics.GetBodyTransform( b );
    physics.SetBodyTransform( a, advance( physics, a, poseA, step ) );
    physics.SetBodyTransform( b, advance( physics, b, poseB, step ) );
    const auto plus = physics.getSplineRailJointData( id );
    physics.SetBodyTransform( a, advance( physics, a, poseA, -step ) );
    physics.SetBodyTransform( b, advance( physics, b, poseB, -step ) );
    const auto minus = physics.getSplineRailJointData( id );
    physics.SetBodyTransform( a, poseA );
    physics.SetBodyTransform( b, poseB );
    const double difference = std::remainder( plus.currentAngle - minus.currentAngle, 2.0f * std::numbers::pi_v<float> ) / ( 2.0 * step );
    if( !plus.trackingValid || !minus.trackingValid || std::abs( before.currentAngularSpeed - difference ) >= 0.004 ) std::fprintf( stderr, "mode=%d segment=%zu t=%g speed=%g diff=%g angles=%g,%g,%g valid=%d,%d\n", static_cast<int>( before.rotationMode ), static_cast<std::size_t>( before.parameter.segmentIndex ), before.parameter.t, before.currentAngularSpeed, difference, minus.currentAngle, before.currentAngle, plus.currentAngle, minus.trackingValid, plus.trackingValid );
    check( plus.trackingValid && minus.trackingValid && std::abs( before.currentAngularSpeed - difference ) < 0.004, "reported angular error speed matches independent wrapped-angle differentiation" );
}

void checkCurveAndMovingOwner()
{
    for( int mode : { 0, 1, 2 } )
        for( bool moving : { false, true } )
        {
            world physics{};
            physics.SetGravity( {} );
            const transform2 poseA{ { 3, -2 }, rot2::FromRadians( 0.4f ) };
            const auto a = physics.CreateBody( moving ? bodyType::Dynamic : bodyType::Static, poseA );
            if( moving ) ( void )physics.CreateShape( a, circle2{ { 0.3f, 0.2f }, 0.2f } );
            const vec2 points[] = { { -3, 0 }, { 0, 2 }, { 3, 0 } };
            const auto tangent = Normalize( Derivative( { points }, 0, 0.6f ) );
            const vec2 point = Evaluate( { points }, 0, 0.6f ) + 0.2f * Cross( 1.0f, tangent );
            const vec2 anchor{ -0.3f, 0.4f };
            const auto rotation = rot2::FromRadians( -0.3f );
            const auto b = physics.CreateBody( bodyType::Dynamic, { TransformPoint( poseA, point ) - Rotate( rotation, anchor ), rotation } );
            ( void )physics.CreateShape( b, circle2{ { 0.2f, -0.15f }, 0.1f } );
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            definition.localAnchorB = anchor;
            definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
            definition.enableAngularSpring = mode == 2;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "curved angular speed fixture is created" );
            physics.SetBodyLinearVelocity( a, { 0.7f, -0.4f } );
            physics.SetBodyAngularVelocity( a, 0.8f );
            physics.SetBodyLinearVelocity( b, { 2.1f, -0.3f } );
            physics.SetBodyAngularVelocity( b, -1.2f );
            checkDerivative( physics, id );
            const auto before = physics.getSplineRailJointData( id );
            if( mode != 0 )
                check( std::abs( before.currentAngularSpeed - ( physics.GetBodyAngularVelocity( b ) - physics.GetBodyAngularVelocity( a ) ) ) > 0.1f, "curved tangent mode includes changing target direction rather than just relative spin" );
            check( physics.rebaseSplineRailJoint( id ) && physics.rebaseSplineRailJointRotation( id ) && physics.getSplineRailJointData( id ).currentAngularSpeed == before.currentAngularSpeed, "linear and angular origin changes preserve the instantaneous angular error rate" );
        }
}

jointId createLoop( world& physics, float direction, int mode )
{
    physics.SetGravity( {} );
    const auto a = physics.CreateBody( bodyType::Kinematic );
    const auto b = physics.CreateBody( bodyType::Dynamic, { { 2, 0 }, {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    physics.SetBodySleepEnabled( b, false );
    const vec2 points[] = { { 2, 0 }, { 0, 2 }, { -2, 0 }, { 0, -2 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.isLoop = true;
    definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
    definition.enableAngularSpring = mode == 2;
    definition.angularHertz = 1.5f;
    definition.enableMotor = true;
    definition.motorSpeed = direction * 2.0f;
    definition.maxMotorForce = 1.0f;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "closed speed fixture is created" );
    physics.SetBodyLinearVelocity( a, { 0.3f, -0.1f } );
    physics.SetBodyAngularVelocity( a, 0.2f );
    return id;
}

void checkSeamsAndQueries()
{
    for( int mode : { 0, 1, 2 } )
        for( int substeps : { 1, 4, 8 } )
        for( float direction : { -1.0f, 1.0f } )
        {
            world physics{}, reference{};
            const auto id = createLoop( physics, direction, mode );
            const auto referenceId = createLoop( reference, direction, mode );
            auto previous = physics.getSplineRailJointData( id );
            int seams = 0;
            for( int frame = 0; frame < 900; ++frame )
            {
                physics.Step( 1.0f / 60.0f, substeps );
                reference.Step( 1.0f / 60.0f, substeps );
                const auto data = physics.getSplineRailJointData( id );
                check( data.trackingValid && std::isfinite( data.currentAngularSpeed ), "closed angular speed stays valid while rail moves and rotates" );
                if( ( direction > 0.0f && previous.parameter.segmentIndex == 3 && data.parameter.segmentIndex == 0 ) ||
                    ( direction < 0.0f && previous.parameter.segmentIndex == 0 && data.parameter.segmentIndex == 3 ) ) ++seams;
                for( int query = 0; query < 5; ++query )
                {
                    const auto repeated = physics.getSplineRailJointData( id );
                    check( repeated.currentAngularSpeed == data.currentAngularSpeed && repeated.currentSpeed == data.currentSpeed && repeated.currentTranslation == data.currentTranslation && repeated.parameter.segmentIndex == data.parameter.segmentIndex && repeated.parameter.t == data.parameter.t && repeated.motorForce == data.motorForce, "repeated angular speed queries preserve distance, branch and cached motor reaction" );
                }
                previous = data;
            }
            const auto expected = reference.getSplineRailJointData( referenceId );
            check( seams >= 2 && direction * previous.currentTranslation > 2.0 * previous.length, "angular speed is checked over multiple forward or reverse circuits" );
            check( previous.currentTranslation == expected.currentTranslation && previous.currentAngularSpeed == expected.currentAngularSpeed && previous.currentSpeed == expected.currentSpeed && previous.force.x == expected.force.x && previous.force.y == expected.force.y && previous.torque == expected.torque, "query frequency does not change future rail motion or reactions" );
            for( auto bodies : { std::pair{ previous.bodyA, expected.bodyA }, std::pair{ previous.bodyB, expected.bodyB } } )
            {
                const auto actualPose = physics.GetBodyTransform( bodies.first ), expectedPose = reference.GetBodyTransform( bodies.second );
                check( LengthSquared( actualPose.position - expectedPose.position ) == 0.0f && actualPose.rotation.c == expectedPose.rotation.c && actualPose.rotation.s == expectedPose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( bodies.first ) - reference.GetBodyLinearVelocity( bodies.second ) ) == 0.0f && physics.GetBodyAngularVelocity( bodies.first ) == reference.GetBodyAngularVelocity( bodies.second ), "angular speed queries preserve body poses and velocities exactly" );
            }
        }
}

void checkSeamDerivative()
{
    for( int mode : { 0, 1, 2 } )
        for( float direction : { -1.0f, 1.0f } )
        {
            world physics{};
            const auto id = createLoop( physics, direction, mode );
            const auto data = physics.getSplineRailJointData( id );
            physics.SetBodyLinearVelocity( data.bodyB, { 0.3f, -0.1f + 0.4f + direction * 2.0f } );
            physics.SetBodyAngularVelocity( data.bodyB, -0.7f );
            checkDerivative( physics, id, 0.0001f );
        }
}

void checkOpenEndpointsAndWrap()
{
    for( int mode : { 0, 1, 2 } )
        for( bool end : { false, true } )
        {
            world physics{};
            const auto a = physics.CreateBody( bodyType::Kinematic, { {}, rot2::FromRadians( 0.4f ) } );
            const vec2 points[] = { { -3, 0 }, { 0, 2 }, { 3, 0 } };
            const std::size_t segment = end ? 1 : 0;
            const float t = end ? 1.0f : 0.0f;
            const auto tangent = Normalize( Derivative( { points }, segment, t ) );
            const vec2 position = Evaluate( { points }, segment, t ) + ( end ? 1.0f : -1.0f ) * tangent + 0.1f * Cross( 1.0f, tangent );
            const auto b = physics.CreateBody( bodyType::Dynamic, { TransformPoint( physics.GetBodyTransform( a ), position ), {} } );
            ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
            definition.enableAngularSpring = mode == 2;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "curved open endpoint fixture is created" );
            physics.SetBodyLinearVelocity( a, { 1, -2 } );
            physics.SetBodyAngularVelocity( a, 0.7f );
            physics.SetBodyLinearVelocity( b, { -2, 1 } );
            physics.SetBodyAngularVelocity( b, -1.2f );
            checkDerivative( physics, id );
            check( std::abs( physics.getSplineRailJointData( id ).currentAngularSpeed + 1.9f ) < 0.000001f, "outside open endpoints the tangent is extended without curvature rotation" );
            const auto data = physics.getSplineRailJointData( id );
            const float target = ( mode == 0 ? data.referenceAngle : std::atan2( InverseRotate( physics.GetBodyTransform( a ).rotation, data.tangent ).y, InverseRotate( physics.GetBodyTransform( a ).rotation, data.tangent ).x ) + data.tangentOffset );
            const auto pose = physics.GetBodyTransform( b );
            physics.SetBodyTransform( b, { pose.position, physics.GetBodyTransform( a ).rotation * rot2::FromRadians( target + std::numbers::pi_v<float> - 0.0001f ) } );
            checkDerivative( physics, id );
        }
}

void checkInvalidAndSleeping()
{
    for( int mode : { 0, 1, 2 } )
    {
        world physics{};
        physics.SetGravity( { 0, -2 } );
        const auto a = physics.CreateBody( bodyType::Kinematic );
        const auto b = physics.CreateBody( bodyType::Dynamic );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        const vec2 points[] = { { -4, 0 }, { 4, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
        definition.enableAngularSpring = mode == 2;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "sleeping angular query fixture is created" );
        for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
        physics.SetBodyAwake( a, false );
        physics.SetBodyAwake( b, false );
        const auto before = physics.getSplineRailJointData( id );
        check( Length( before.force ) > 0.01f, "sleeping query has a cached normal reaction" );
        for( int query = 0; query < 10; ++query )
        {
            const auto data = physics.getSplineRailJointData( id );
            check( data.trackingValid && data.currentAngularSpeed == 0.0f && data.currentTranslation == before.currentTranslation && data.parameter.segmentIndex == before.parameter.segmentIndex && data.parameter.t == before.parameter.t && data.force.x == before.force.x && data.force.y == before.force.y && data.torque == before.torque && data.referenceAngle == before.referenceAngle && data.tangentOffset == before.tangentOffset && !physics.IsBodyAwake( a ) && !physics.IsBodyAwake( b ), "sleeping angular query preserves caches, references, tracking and both sleeping bodies" );
        }
    }
    for( int mode : { 0, 1, 2 } )
    {
        world physics{};
        const auto a = physics.CreateBody();
        const auto b = physics.CreateBody( bodyType::Dynamic, { { 0, 2 }, {} } );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        const vec2 points[] = { { -2, 0 }, { 0, 2 }, { 2, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
        definition.enableAngularSpring = mode == 2;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "invalid angular query starts with valid tracking" );
        const auto before = physics.getSplineRailJointData( id );
        const auto curvature = Curvature( { points }, before.parameter.segmentIndex, before.parameter.t );
        check( curvature.has_value() && *curvature != 0.0, "singular angular query fixture has curvature" );
        physics.SetBodyTransform( b, { before.anchorA + static_cast<float>( 1.0 / *curvature ) * before.normal, {} } );
        physics.SetBodyAngularVelocity( b, 1.5f );
        const auto failed = physics.getSplineRailJointData( id );
        check( !failed.trackingValid && failed.currentAngularSpeed == 0.0f, "invalid frame reports zero instead of nonzero relative angular velocity" );
        physics.SetBodyTransform( b, { { 0, 2 }, {} } );
        physics.setSplineRailJointSpring( id, true, std::numeric_limits<float>::max(), 0.7f, 1.0f );
        physics.Step( 1.0f / 60.0f, 4 );
        check( !physics.getSplineRailJointData( id ).trackingValid, "solver overflow invalidates stored tracking" );
        physics.SetBodyTransform( b, { { 0, 2 }, {} } );
        physics.SetBodyAngularVelocity( b, 1.5f );
        const auto stored = physics.getSplineRailJointData( id );
        check( !stored.trackingValid && stored.currentAngularSpeed == 0.0f, "stored tracking failure reports zero even after restoring a valid geometric frame" );
    }
}

}

int main()
{
    checkStraight();
    checkCurveAndMovingOwner();
    checkOpenEndpointsAndWrap();
    checkSeamDerivative();
    checkSeamsAndQueries();
    checkInvalidAndSleeping();
    return 0;
}
