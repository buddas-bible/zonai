#include <algorithm>
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

void checkInitialError()
{
    for( int mode : { 0, 1, 2 } )
    {
        world physics{};
        physics.SetGravity( {} );
        const transform2 poseA{ { 0.3f, -0.2f }, rot2::FromRadians( 0.4f ) };
        const auto a = physics.CreateBody( bodyType::Kinematic, poseA );
        const vec2 points[] = { { -2, 0 }, { 0, 2 }, { 2, 0 } };
        const vec2 position = TransformPoint( poseA, Evaluate( { points }, 0, 0.4f ) );
        const auto b = physics.CreateBody( bodyType::Dynamic, { position, poseA.rotation * rot2::FromRadians( 0.1f ) } );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.referenceAngle = -0.2f;
        definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
        definition.enableAngularSpring = mode == 2;
        definition.angularHertz = 3.7f;
        definition.angularDampingRatio = 0.6f;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "rotation rebase fixture is created" );
        physics.SetBodyTransform( b, { position, poseA.rotation * rot2::FromRadians( 0.7f ) } );
        physics.SetBodyLinearVelocity( a, { 0.1f, -0.2f } );
        physics.SetBodyAngularVelocity( a, 0.2f );
        physics.SetBodyLinearVelocity( b, { 2, 1 } );
        physics.SetBodyAngularVelocity( b, 0.4f );
        const auto before = physics.getSplineRailJointData( id );
        const auto pose = physics.GetBodyTransform( b );
        check( before.trackingValid && std::abs( before.currentAngle ) > 0.5f, "fixture starts with a nonzero fixed or tangent angular error" );
        check( physics.rebaseSplineRailJointRotation( id ), "valid tracking permits rebasing the current rotation mode" );
        const auto after = physics.getSplineRailJointData( id );
        check( after.trackingValid && std::abs( after.currentAngle ) < 0.000001f && after.rotationMode == before.rotationMode && after.enableAngularSpring == before.enableAngularSpring && after.angularHertz == before.angularHertz && after.angularDampingRatio == before.angularDampingRatio, "rotation rebase zeroes the error without changing mode or spring tuning" );
        check( after.currentTranslation == before.currentTranslation && after.currentSpeed == before.currentSpeed && after.referenceDistance == before.referenceDistance && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyTransform( b ).rotation.s == pose.rotation.s && physics.GetBodyLinearVelocity( b ).x == 2.0f && physics.GetBodyAngularVelocity( b ) == 0.4f && physics.GetBodyAngularVelocity( a ) == 0.2f, "rotation origin changes preserve travel, branch, physical transforms and velocities" );
    }
}
float relativeAngle( const world& physics, bodyId a, bodyId b )
{
    const auto relative = Inverse( physics.GetBodyTransform( a ).rotation ) * physics.GetBodyTransform( b ).rotation;
    return std::atan2( relative.s, relative.c );
}

void checkMotion()
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
            const auto tangent = InverseRotate( physics.GetBodyTransform( a ).rotation, before.tangent );
            const float startTangent = std::atan2( tangent.y, tangent.x );
            check( physics.rebaseSplineRailJointRotation( id ), "rotation can be reset during travel without changing mode" );
            const auto after = physics.getSplineRailJointData( id );
            check( std::abs( after.currentAngle ) < 0.000001f && after.currentTranslation == before.currentTranslation && after.currentSpeed == before.currentSpeed && after.referenceDistance == before.referenceDistance && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t, "rotation reset preserves accumulated distance, speed, branch and linear origin" );
            check( after.enableSpring == before.enableSpring && after.hertz == before.hertz && after.dampingRatio == before.dampingRatio && after.targetTranslation == before.targetTranslation && after.enableLimit == before.enableLimit && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && after.enableMotor == before.enableMotor && after.motorSpeed == before.motorSpeed && after.maxMotorForce == before.maxMotorForce && after.enableAngularSpring == before.enableAngularSpring && after.angularHertz == before.angularHertz && after.angularDampingRatio == before.angularDampingRatio, "rotation reset preserves all translational and angular tuning" );
            check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyTransform( b ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( b ) == spin && physics.GetBodyMass( b ) == mass && physics.GetShapeFriction( shape ) == 0.42f && physics.IsValid( shape ) && physics.IsValid( id ) && physics.GetBodyAngularVelocity( a ) == 0.1f, "reset preserves dynamic state, owner motion, handles and material" );
            float maxRotation = 0.0f;
            for( int frame = 0; frame < 360; ++frame )
            {
                physics.Step( 1.0f / 60.0f, substeps );
                const auto data = physics.getSplineRailJointData( id );
                const auto localTangent = InverseRotate( physics.GetBodyTransform( a ).rotation, data.tangent );
                maxRotation = std::max( maxRotation, std::abs( std::remainder( relativeAngle( physics, a, b ) - 0.7f, 2.0f * std::numbers::pi_v<float> ) ) );
                const float target = mode == 0 ? 0.7f : 0.7f + std::atan2( localTangent.y, localTangent.x ) - startTangent;
                const float error = std::remainder( relativeAngle( physics, a, b ) - target, 2.0f * std::numbers::pi_v<float> );
                check( data.trackingValid && std::isfinite( data.torque ) && std::abs( std::remainder( data.currentAngle - error, 2.0f * std::numbers::pi_v<float> ) ) < 0.000002f && ( mode == 2 || std::abs( error ) < 0.03f ), "future fixed or tangent rotation follows the newly chosen reference" );
            }
            const auto travelled = physics.getSplineRailJointData( id );
            check( travelled.currentTranslation > after.currentTranslation + ( loop ? 10.0 : 5.0 ), "normal motor travel continues after changing only the rotation reference" );
            if( mode != 0 )
                check( maxRotation > 0.2f, "tangent mode rotates with the curve instead of freezing the rebased orientation" );
        }
}

void checkHardAndSpring()
{
    std::array<float, 2> firstError{};
    for( int spring : { 0, 1 } )
    {
        world physics{};
        physics.SetGravity( {} );
        const auto a = physics.CreateBody();
        const auto b = physics.CreateBody( bodyType::Dynamic );
        physics.SetBodySleepEnabled( b, false );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        const vec2 points[] = { { -4, 0 }, { 4, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.rotationMode = splineRailRotationMode::followTangent;
        definition.enableAngularSpring = spring != 0;
        definition.angularHertz = 1.0f;
        definition.angularDampingRatio = 0.7f;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "angular response fixture is created" );
        physics.SetBodyTransform( b, { {}, rot2::FromRadians( 0.3f ) } );
        check( physics.rebaseSplineRailJointRotation( id ), "angular response reference is rebased" );
        physics.SetBodyTransform( b, { {}, rot2::FromRadians( 0.8f ) } );
        physics.Step( 1.0f / 60.0f, 1 );
        firstError[spring] = std::abs( physics.getSplineRailJointData( id ).currentAngle );
        for( int frame = 0; frame < 360; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
        const auto data = physics.getSplineRailJointData( id );
        check( std::abs( data.currentAngle ) < 0.001f && std::abs( relativeAngle( physics, a, b ) - 0.3f ) < 0.001f && data.enableAngularSpring == ( spring != 0 ) && data.angularHertz == 1.0f && data.angularDampingRatio == 0.7f, "both hard and spring constraints return to the rebased angle with their tuning intact" );
    }
    check( firstError[1] > firstError[0] + 0.05f, "rebasing retains the softer first response of an enabled angular spring" );
}

void checkSleepAndCaches()
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
        definition.angularHertz = 1.0f;
        definition.angularDampingRatio = 0.7f;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "sleeping rotation fixture is created" );
        for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
        physics.SetBodyAwake( a, false );
        physics.SetBodyAwake( b, false );
        const auto before = physics.getSplineRailJointData( id );
        check( Length( before.force ) > 0.01f, "unchanged basis fixture has a cached normal load" );
        check( physics.rebaseSplineRailJointRotation( id ), "unchanged sleeping rotation origin succeeds" );
        const auto after = physics.getSplineRailJointData( id );
        check( !physics.IsBodyAwake( a ) && !physics.IsBodyAwake( b ) && after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && after.currentTranslation == before.currentTranslation, "same rotation reference preserves both sleeping bodies and their constraint cache" );
        if( mode == 2 )
        {
            for( int frame = 0; frame < 600; ++frame )
            {
                physics.ApplyTorque( b, 0.001f );
                physics.Step( 1.0f / 60.0f, 4 );
            }
            physics.SetBodyAwake( a, false );
            physics.SetBodyAwake( b, false );
            const auto loaded = physics.getSplineRailJointData( id );
            const auto pose = physics.GetBodyTransform( b );
            check( std::abs( loaded.currentAngle ) > 0.01f && std::abs( loaded.torque ) > 0.0001f, "changed basis fixture has a loaded angular spring" );
            check( physics.rebaseSplineRailJointRotation( id ), "loaded sleeping angular reference can change" );
            const auto changed = physics.getSplineRailJointData( id );
            check( physics.IsBodyAwake( a ) && physics.IsBodyAwake( b ) && std::abs( changed.currentAngle ) < 0.000001f && LengthSquared( changed.force ) == 0.0f && changed.torque == 0.0f && changed.springForce == 0.0f && changed.motorForce == 0.0f && physics.GetBodyTransform( b ).rotation.s == pose.rotation.s, "changed reference wakes both bodies and clears coupled cache without rotating the body" );
            physics.SetBodyAwake( a, false );
            physics.SetBodyAwake( b, false );
            check( physics.rebaseSplineRailJointRotation( id ) && !physics.IsBodyAwake( a ) && !physics.IsBodyAwake( b ), "repeated rotation reset is an exact no-op" );
        }
    }
}

void checkFailure()
{
    for( int mode : { 0, 1, 2 } )
    {
        world physics{};
        physics.SetGravity( {} );
        const auto a = physics.CreateBody();
        const auto b = physics.CreateBody( bodyType::Dynamic, { { 0, 2 }, rot2::FromRadians( 0.1f ) } );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        const vec2 points[] = { { -2, 0 }, { 0, 2 }, { 2, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.referenceAngle = -0.2f;
        definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
        definition.enableAngularSpring = mode == 2;
        definition.targetTranslation = 1.0f;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "failed rotation fixture is created" );
        const transform2 pose{ { 0, 2 }, rot2::FromRadians( 0.6f ) };
        physics.SetBodyTransform( b, pose );
        const auto valid = physics.getSplineRailJointData( id );
        const auto curvature = Curvature( { points }, valid.parameter.segmentIndex, valid.parameter.t );
        check( curvature.has_value() && *curvature != 0.0, "failed rotation fixture has nonzero curvature" );
        physics.SetBodyTransform( b, { valid.anchorA + static_cast<float>( 1.0 / *curvature ) * valid.normal, rot2::FromRadians( 0.9f ) } );
        physics.SetBodyAwake( b, false );
        const auto before = physics.getSplineRailJointData( id );
        check( !before.trackingValid && !physics.rebaseSplineRailJointRotation( id ), "singular tracked projection cannot change any rotation reference" );
        const auto after = physics.getSplineRailJointData( id );
        check( !after.trackingValid && after.referenceAngle == before.referenceAngle && after.referenceDistance == before.referenceDistance && after.targetTranslation == before.targetTranslation && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && after.torque == before.torque && !physics.IsBodyAwake( b ), "failure preserves stored basis, branch, control settings, cache and sleep" );
        physics.SetBodyTransform( b, pose );
        const auto restored = physics.getSplineRailJointData( id );
        check( restored.trackingValid && restored.currentAngle == valid.currentAngle && restored.currentTranslation == valid.currentTranslation, "recovering the old pose exposes the unchanged angular and linear references" );
    }
    world physics{};
    physics.SetGravity( {} );
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { {}, rot2::FromRadians( 0.3f ) } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { -4, 0 }, { 4, 0 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.enableSpring = true;
    definition.hertz = std::numeric_limits<float>::max();
    definition.targetTranslation = 1.0f;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "stored tracking-failure fixture is created" );
    physics.Step( 1.0f / 60.0f, 4 );
    const auto before = physics.getSplineRailJointData( id );
    check( !before.trackingValid && !physics.rebaseSplineRailJointRotation( id ), "solver-invalidated tracking rejects rotation rebase even if the geometry frame is valid" );
    const auto after = physics.getSplineRailJointData( id );
    check( after.referenceAngle == before.referenceAngle && after.currentAngle == before.currentAngle && after.hertz == before.hertz && after.enableSpring == before.enableSpring, "stored tracking failure remains untouched" );
}

}

int main()
{
    checkInitialError();
    checkMotion();
    checkHardAndSpring();
    checkSleepAndCaches();
    checkFailure();
    return 0;
}
