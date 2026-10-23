#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <numbers>
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

float wrap( float angle )
{
    return std::remainder( angle, 2.0f * std::numbers::pi_v<float> );
}

float relativeAngle( const world& physics, bodyId a, bodyId b )
{
    const auto relative = Inverse( physics.GetBodyTransform( a ).rotation ) * physics.GetBodyTransform( b ).rotation;
    return std::atan2( relative.s, relative.c );
}

void checkResponse()
{
    std::array<float, 3> firstError{};
    for( int mode : { 0, 1, 2 } )
    {
        world physics{};
        physics.SetGravity( {} );
        const auto a = physics.CreateBody( bodyType::Static, { {}, rot2::FromRadians( 0.4f ) } );
        const auto b = physics.CreateBody( bodyType::Dynamic, { {}, rot2::FromRadians( 0.6f ) } );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        physics.SetBodySleepEnabled( b, false );
        const vec2 points[] = { { -4, -2 }, { 4, 2 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.referenceAngle = -0.2f;
        definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
        definition.enableAngularSpring = mode != 1;
        definition.angularHertz = 1.0f;
        definition.angularDampingRatio = 0.7f;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "reference response fixture is created" );
        const auto before = physics.getSplineRailJointData( id );
        const float initialOffset = 0.2f - std::atan2( 2.0f, 4.0f );
        check( std::abs( before.tangentOffset - initialOffset ) < 0.000001f, "query exposes the tangent offset captured during creation in either mode" );
        check( physics.setSplineRailJointRotationReference( id, 0.7f ), "a finite angle can set the selected rotation reference" );
        const auto set = physics.getSplineRailJointData( id );
        check( mode == 0 ? set.referenceAngle == 0.7f && set.tangentOffset == before.tangentOffset : set.tangentOffset == 0.7f && set.referenceAngle == before.referenceAngle, "only the selected mode reference changes" );
        check( set.rotationMode == before.rotationMode && set.enableAngularSpring == before.enableAngularSpring && set.angularHertz == before.angularHertz && set.angularDampingRatio == before.angularDampingRatio, "setting a reference preserves mode and angular tuning" );
        physics.Step( 1.0f / 60.0f, 1 );
        firstError[mode] = std::abs( physics.getSplineRailJointData( id ).currentAngle );
        for( int frame = 0; frame < 360; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
        const auto after = physics.getSplineRailJointData( id );
        const float target = mode == 0 ? 0.7f : 0.7f + std::atan2( 2.0f, 4.0f );
        check( after.trackingValid && std::abs( after.currentAngle ) < 0.001f && std::abs( wrap( relativeAngle( physics, a, b ) - target ) ) < 0.001f, "fixed, hard tangent and spring tangent converge to the explicitly requested angle" );
        check( physics.rebaseSplineRailJointRotation( id ) && std::abs( physics.getSplineRailJointData( id ).currentAngle ) < 0.000001f, "current-pose rebase continues to work after explicit angle settings" );
    }
    check( firstError[2] > firstError[1] + 0.05f, "explicit reference retains the spring response instead of imposing a hard constraint" );
}

void checkTravel()
{
    for( bool loop : { false, true } )
        for( int mode : { 0, 1, 2 } )
        for( int substeps : { 1, 4, 8 } )
        {
            world physics{};
            physics.SetGravity( {} );
            const vec2 line[] = { { -4, 0 }, { -2, 0 }, { 0, 2 }, { 2, 0 }, { 4, 0 } };
            const vec2 ring[] = { { 2, 0 }, { 0, 2 }, { -2, 0 }, { 0, -2 } };
            const auto points = loop ? std::span<const vec2>{ ring } : std::span<const vec2>{ line };
            const transform2 poseA{ { 0.3f, -0.2f }, rot2::FromRadians( 0.4f ) };
            const auto a = physics.CreateBody( bodyType::Kinematic, poseA );
            const vec2 anchor = loop ? vec2{} : vec2{ 0.2f, -0.1f };
            const auto rotation = poseA.rotation * rot2::FromRadians( 0.1f );
            const auto position = TransformPoint( poseA, Evaluate( { points, loop }, 0, 0.2f ) );
            const auto b = physics.CreateBody( bodyType::Dynamic, { position - Rotate( rotation, anchor ), rotation } );
            const auto shape = physics.CreateShape( b, circle2{ { 0.05f, 0.1f }, 0.1f } );
            physics.SetBodySleepEnabled( b, false );
            physics.SetShapeFriction( shape, 0.42f );
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            definition.isLoop = loop;
            definition.localAnchorB = anchor;
            definition.referenceDistance = 0.35;
            definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
            definition.enableAngularSpring = mode == 2;
            definition.angularHertz = 3.7f;
            definition.angularDampingRatio = 0.6f;
            definition.enableSpring = true;
            definition.hertz = 0.0f;
            definition.targetTranslation = 0.6f;
            definition.enableLimit = true;
            definition.lowerTranslation = -100.0f;
            definition.upperTranslation = 100.0f;
            definition.enableMotor = true;
            definition.motorSpeed = loop ? 2.0f : 1.0f;
            definition.maxMotorForce = 1.0f;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "rotation travel fixture is created" );
            if( loop )
            {
                const double initial = physics.getSplineRailJointData( id ).currentTranslation;
                for( int frame = 0; frame < 900; ++frame ) physics.Step( 1.0f / 60.0f, substeps );
                const auto travelled = physics.getSplineRailJointData( id );
                check( travelled.currentTranslation - initial > 2.0 * travelled.length && physics.rebaseSplineRailJoint( id ), "rotation reset follows multiple circuits and a linear-origin reset" );
            }
            auto pose = physics.GetBodyTransform( b );
            const auto oldAnchor = TransformPoint( pose, anchor );
            pose.rotation = physics.GetBodyTransform( a ).rotation * rot2::FromRadians( 0.7f );
            pose.position = oldAnchor - Rotate( pose.rotation, anchor );
            physics.SetBodyTransform( b, pose );
            physics.SetBodyLinearVelocity( a, { 0.1f, -0.2f } );
            physics.SetBodyAngularVelocity( a, 0.1f );
            const auto before = physics.getSplineRailJointData( id );
            const auto velocity = physics.GetBodyLinearVelocity( b );
            const float spin = physics.GetBodyAngularVelocity( b ), mass = physics.GetBodyMass( b );
            check( physics.setSplineRailJointRotationReference( id, 0.35f ), "rotation reference can be set during travel without changing mode" );
            const auto after = physics.getSplineRailJointData( id );
            check( ( mode == 0 ? after.referenceAngle == 0.35f && after.tangentOffset == before.tangentOffset : after.tangentOffset == 0.35f && after.referenceAngle == before.referenceAngle ) && after.currentTranslation == before.currentTranslation && after.currentSpeed == before.currentSpeed && after.referenceDistance == before.referenceDistance && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t, "setting the reference preserves accumulated distance, speed, branch and linear origin" );
            check( after.enableSpring == before.enableSpring && after.hertz == before.hertz && after.dampingRatio == before.dampingRatio && after.targetTranslation == before.targetTranslation && after.enableLimit == before.enableLimit && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && after.enableMotor == before.enableMotor && after.motorSpeed == before.motorSpeed && after.maxMotorForce == before.maxMotorForce && after.enableAngularSpring == before.enableAngularSpring && after.angularHertz == before.angularHertz && after.angularDampingRatio == before.angularDampingRatio, "setting the reference preserves all translational and angular tuning" );
            check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyTransform( b ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( b ) == spin && physics.GetBodyMass( b ) == mass && physics.GetShapeFriction( shape ) == 0.42f && physics.IsValid( shape ) && physics.IsValid( id ) && physics.GetBodyAngularVelocity( a ) == 0.1f, "reset preserves dynamic state, owner motion, handles and material" );
            for( int frame = 0; frame < 360; ++frame )
            {
                physics.Step( 1.0f / 60.0f, substeps );
                const auto data = physics.getSplineRailJointData( id );
                const auto localTangent = InverseRotate( physics.GetBodyTransform( a ).rotation, data.tangent );
                const float target = mode == 0 ? 0.35f : 0.35f + std::atan2( localTangent.y, localTangent.x );
                const float error = std::remainder( relativeAngle( physics, a, b ) - target, 2.0f * std::numbers::pi_v<float> );
                check( data.trackingValid && std::isfinite( data.torque ) && std::abs( std::remainder( data.currentAngle - error, 2.0f * std::numbers::pi_v<float> ) ) < 0.000002f && ( mode == 2 || frame < 90 || std::abs( error ) < 0.03f ), "future fixed or tangent rotation follows the newly chosen reference" );
            }
            const auto travelled = physics.getSplineRailJointData( id );
            check( travelled.currentTranslation > after.currentTranslation + ( loop ? 10.0 : 5.0 ), "normal motor travel continues after changing only the rotation reference" );
        }
}

void checkSleepAndFailure()
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
        check( physics.IsValid( id ), "sleeping reference fixture is created" );
        for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
        physics.SetBodyAwake( a, false );
        physics.SetBodyAwake( b, false );
        const auto before = physics.getSplineRailJointData( id );
        const auto pose = physics.GetBodyTransform( b );
        check( Length( before.force ) > 0.01f, "reference fixture has a cached normal load" );
        check( physics.setSplineRailJointRotationReference( id, 0.0f ), "same reference succeeds" );
        const float nan = std::numeric_limits<float>::quiet_NaN(), infinity = std::numeric_limits<float>::infinity();
        for( float invalid : { nan, infinity, -infinity } )
            check( !physics.setSplineRailJointRotationReference( id, invalid ), "nonfinite reference fails without changes" );
        const auto same = physics.getSplineRailJointData( id );
        check( !physics.IsBodyAwake( a ) && !physics.IsBodyAwake( b ) && same.force.x == before.force.x && same.force.y == before.force.y && same.torque == before.torque && same.referenceAngle == before.referenceAngle && same.tangentOffset == before.tangentOffset && same.parameter.segmentIndex == before.parameter.segmentIndex && same.parameter.t == before.parameter.t && same.currentTranslation == before.currentTranslation && same.trackingValid == before.trackingValid, "no-op and failed settings preserve sleep, reactions, both references, tracking and travel" );
        check( physics.setSplineRailJointRotationReference( id, 0.3f ), "a changed sleeping reference succeeds" );
        const auto changed = physics.getSplineRailJointData( id );
        check( physics.IsBodyAwake( a ) && physics.IsBodyAwake( b ) && LengthSquared( changed.force ) == 0.0f && changed.torque == 0.0f && changed.springForce == 0.0f && changed.motorForce == 0.0f, "reference changes clear coupled reactions and wake both bodies in every mode" );
        check( physics.GetBodyTransform( b ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && std::abs( changed.currentAngle + 0.3f ) < 0.000001f, "reference change changes the error without moving the body" );
        physics.SetBodyAwake( a, false );
        physics.SetBodyAwake( b, false );
        check( physics.setSplineRailJointRotationReference( id, 0.3f ) && !physics.IsBodyAwake( a ) && !physics.IsBodyAwake( b ), "repeated changed reference is an exact no-op" );
        for( float angle : { -3.4f, 3.4f } )
        {
            check( physics.setSplineRailJointRotationReference( id, angle ), "finite angles across the pi boundary are accepted" );
            const auto data = physics.getSplineRailJointData( id );
            check( ( mode == 0 ? data.referenceAngle : data.tangentOffset ) == angle && std::abs( wrap( data.currentAngle - wrap( -angle ) ) ) < 0.000001f, "reference readback preserves input while angular error uses the existing wrap policy" );
        }
    }
}

}

int main()
{
    checkResponse();
    checkTravel();
    checkSleepAndFailure();
    return 0;
}
