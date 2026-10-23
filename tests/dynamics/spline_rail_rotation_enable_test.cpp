#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/world.h"
#include "dynamics/joints/splineRailJointConstraint2.h"

using namespace zonai;

namespace
{
void check( bool ok, const char* message )
{
    if( ok ) return;
    std::fprintf( stderr, "%s\n", message );
    std::exit( EXIT_FAILURE );
}

void checkFreeRotation()
{
    for( int mode : { 0, 1, 2 } )
        for( int substeps : { 1, 4, 8 } )
        {
            world physics{};
            physics.SetGravity( {} );
            const auto a = physics.CreateBody();
            const auto b = physics.CreateBody( bodyType::Dynamic );
            ( void )physics.CreateShape( b, circle2{ {}, 0.2f } );
            physics.SetBodySleepEnabled( b, false );
            const vec2 points[] = { { -4, 0 }, { 4, 0 } };
            splineRailJointDef definition{};
            check( definition.enableRotation, "default preserves the enabled angular constraint" );
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            definition.enableRotation = false;
            definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
            definition.enableAngularSpring = mode == 2;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ) && !physics.getSplineRailJointData( id ).enableRotation, "rotation can start disabled in every stored rotation mode" );
            physics.SetBodyAngularVelocity( b, 2.0f );
            for( int frame = 0; frame < 60; ++frame ) physics.Step( 1.0f / 60.0f, substeps );
            const auto data = physics.getSplineRailJointData( id );
            const auto pose = physics.GetBodyTransform( b );
            check( data.trackingValid && std::abs( physics.GetBodyAngularVelocity( b ) - 2.0f ) < 0.00001f && std::abs( std::atan2( pose.rotation.s, pose.rotation.c ) - 2.0f ) < 0.001f && data.torque == 0.0f, "disabled angular constraint permits unforced free spin instead of restoring the stored angle" );
            check( std::abs( data.currentAngle - 2.0f ) < 0.001f && std::abs( data.currentAngularSpeed - 2.0f ) < 0.00001f, "angle queries keep their stored-reference meaning while rotation is unconstrained" );
        }
}

void checkToggleAndResponse()
{
    std::array<float, 3> firstError{};
    for( int mode : { 0, 1, 2 } )
    {
        world physics{};
        physics.SetGravity( {} );
        const auto a = physics.CreateBody( bodyType::Kinematic, { {}, rot2::FromRadians( 0.4f ) } );
        const auto b = physics.CreateBody( bodyType::Dynamic, { {}, rot2::FromRadians( 0.6f ) } );
        ( void )physics.CreateShape( b, circle2{ {}, 0.2f } );
        physics.SetBodySleepEnabled( b, false );
        const vec2 points[] = { { -4, 0 }, { 4, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.referenceAngle = 0.2f;
        definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
        definition.enableAngularSpring = mode == 2;
        definition.angularHertz = 1.0f;
        definition.angularDampingRatio = 0.7f;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "toggle fixture is created" );
        const auto before = physics.getSplineRailJointData( id );
        physics.setSplineRailJointRotationEnabled( id, false );
        check( !physics.getSplineRailJointData( id ).enableRotation, "the enabled constraint can be disabled without changing rotation mode" );
        physics.SetBodyAngularVelocity( b, 1.0f );
        for( int frame = 0; frame < 30; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
        const auto free = physics.getSplineRailJointData( id );
        check( std::abs( free.currentAngle - 0.5f ) < 0.001f && std::abs( physics.GetBodyAngularVelocity( b ) - 1.0f ) < 0.00001f, "disabled existing joint permits rotation without recapturing its reference" );
        physics.setSplineRailJointRotationEnabled( id, true );
        const auto resumed = physics.getSplineRailJointData( id );
        check( resumed.enableRotation && resumed.currentAngle == free.currentAngle && resumed.referenceAngle == before.referenceAngle && resumed.tangentOffset == before.tangentOffset && resumed.rotationMode == before.rotationMode && resumed.enableAngularSpring == before.enableAngularSpring && resumed.angularHertz == before.angularHertz && resumed.angularDampingRatio == before.angularDampingRatio, "reactivation preserves both references, error, mode and spring tuning" );
        physics.Step( 1.0f / 60.0f, 1 );
        firstError[mode] = std::abs( physics.getSplineRailJointData( id ).currentAngle );
        for( int frame = 0; frame < 360; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
        check( std::abs( physics.getSplineRailJointData( id ).currentAngle ) < 0.001f, "reactivation restores the saved fixed or tangent reference" );
    }
    check( firstError[2] > firstError[1] + 0.05f, "reactivation resumes the spring response rather than a hard angular constraint" );
}

void checkSleepAndState()
{
    for( int mode : { 0, 1, 2 } )
    {
        world physics{};
        physics.SetGravity( { 0, -2 } );
        const auto a = physics.CreateBody( bodyType::Kinematic );
        const auto b = physics.CreateBody( bodyType::Dynamic );
        const auto shape = physics.CreateShape( b, circle2{ {}, 0.2f } );
        const vec2 points[] = { { -4, 0 }, { 4, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
        definition.enableAngularSpring = mode == 2;
        definition.angularHertz = 2.7f;
        definition.angularDampingRatio = 0.4f;
        definition.enableSpring = true;
        definition.hertz = 2.0f;
        definition.enableLimit = true;
        definition.lowerTranslation = -10.0f;
        definition.upperTranslation = 10.0f;
        definition.enableMotor = true;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "sleep fixture is created" );
        for( bool enabled : { true, false, true } )
        {
            for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
            physics.SetBodyAwake( a, false );
            physics.SetBodyAwake( b, false );
            const auto before = physics.getSplineRailJointData( id );
            const auto pose = physics.GetBodyTransform( b );
            check( before.enableRotation == enabled && Length( before.force ) > 0.01f, "both enabled and free fixtures have cached normal load" );
            physics.setSplineRailJointRotationEnabled( id, enabled );
            const auto same = physics.getSplineRailJointData( id );
            check( !physics.IsBodyAwake( a ) && !physics.IsBodyAwake( b ) && same.force.x == before.force.x && same.force.y == before.force.y && same.torque == before.torque, "same flag preserves both sleeping bodies and coupled reactions" );
            physics.setSplineRailJointRotationEnabled( id, !enabled );
            const auto after = physics.getSplineRailJointData( id );
            check( after.enableRotation != enabled && physics.IsBodyAwake( a ) && physics.IsBodyAwake( b ) && LengthSquared( after.force ) == 0.0f && after.torque == 0.0f && after.springForce == 0.0f && after.motorForce == 0.0f, "changed flag clears coupled caches and wakes both bodies" );
            check( after.referenceAngle == before.referenceAngle && after.tangentOffset == before.tangentOffset && after.rotationMode == before.rotationMode && after.enableAngularSpring == before.enableAngularSpring && after.angularHertz == before.angularHertz && after.angularDampingRatio == before.angularDampingRatio && after.enableSpring == before.enableSpring && after.hertz == before.hertz && after.dampingRatio == before.dampingRatio && after.targetTranslation == before.targetTranslation && after.enableLimit == before.enableLimit && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && after.enableMotor == before.enableMotor && after.motorSpeed == before.motorSpeed && after.maxMotorForce == before.maxMotorForce, "toggle preserves every stored mode, reference and axial or angular control setting" );
            check( after.currentTranslation == before.currentTranslation && after.referenceDistance == before.referenceDistance && after.currentSpeed == before.currentSpeed && after.currentAngle == before.currentAngle && after.currentAngularSpeed == before.currentAngularSpeed && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && after.trackingValid == before.trackingValid && physics.GetBodyTransform( b ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.IsValid( shape ), "toggle preserves travel, origin, branch, error queries, physical pose and handles" );
        }
    }
}

void checkAxialControls()
{
    for( int control : { 0, 1, 2, 3 } )
    {
        world physics{};
        physics.SetGravity( {} );
        const auto a = physics.CreateBody();
        const auto b = physics.CreateBody( bodyType::Dynamic );
        ( void )physics.CreateShape( b, circle2{ {}, 0.2f } );
        physics.SetBodySleepEnabled( b, false );
        const vec2 points[] = { { -4, 0 }, { 4, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.referenceDistance = 4.0;
        definition.enableRotation = false;
        definition.enableMotor = control != 1;
        definition.motorSpeed = 3.0f;
        definition.maxMotorForce = 10.0f;
        definition.enableSpring = control == 1;
        definition.hertz = 2.0f;
        definition.dampingRatio = 0.7f;
        definition.targetTranslation = 1.0f;
        definition.enableLimit = control == 2;
        definition.lowerTranslation = -1.0f;
        definition.upperTranslation = 1.0f;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "free axial control fixture is created" );
        physics.SetBodyAngularVelocity( b, 2.0f );
        for( int frame = 0; frame < ( control == 0 ? 30 : 240 ); ++frame ) physics.Step( 1.0f / 60.0f, 4 );
        const auto forward = physics.getSplineRailJointData( id );
        check( std::abs( physics.GetBodyAngularVelocity( b ) - 2.0f ) < 0.00001f && std::abs( forward.lateralError ) < 0.001f, "motor, spring, user limits and physical ends retain free spin and normal guidance" );
        if( control == 0 )
            check( std::abs( forward.currentSpeed - 3.0f ) < 0.01f && forward.currentTranslation > 1.0, "free rotation retains motor speed tracking" );
        else
            check( std::abs( forward.currentTranslation - ( control == 3 ? 4.0 : 1.0 ) ) < 0.01, "free rotation retains spring target, user upper limit or open upper end" );
        if( control == 1 ) physics.setSplineRailJointSpring( id, true, 2.0f, 0.7f, -1.0f );
        else physics.setSplineRailJointMotor( id, true, -3.0f, 10.0f );
        for( int frame = 0; frame < ( control == 0 ? 30 : 240 ); ++frame ) physics.Step( 1.0f / 60.0f, 4 );
        const auto reverse = physics.getSplineRailJointData( id );
        check( !reverse.enableRotation && std::abs( physics.GetBodyAngularVelocity( b ) - 2.0f ) < 0.00001f, "editing axial settings does not re-enable rotation" );
        if( control == 0 ) check( std::abs( reverse.currentSpeed + 3.0f ) < 0.01f, "free motor reverses direction" );
        else check( std::abs( reverse.currentTranslation - ( control == 3 ? -4.0 : -1.0 ) ) < 0.01, "free spring and lower limits continue to work after direction changes" );
    }
}

void checkClosedMovingRail()
{
    for( int mode : { 0, 1, 2 } )
        for( int substeps : { 1, 4, 8 } )
            for( float direction : { -1.0f, 1.0f } )
            {
                world physics{};
                physics.SetGravity( {} );
                const transform2 poseA{ { 0.3f, -0.2f }, rot2::FromRadians( 0.4f ) };
                const auto a = physics.CreateBody( bodyType::Kinematic, poseA );
                const vec2 points[] = { { 2, 0 }, { 0, 2 }, { -2, 0 }, { 0, -2 } };
                const auto b = physics.CreateBody( bodyType::Dynamic, { TransformPoint( poseA, Evaluate( { points, true }, 0, 0.2f ) ), {} } );
                const auto shape = physics.CreateShape( b, circle2{ {}, 0.2f } );
                physics.SetBodySleepEnabled( b, false );
                splineRailJointDef definition{};
                definition.bodyA = a;
                definition.bodyB = b;
                definition.points = points;
                definition.isLoop = true;
                definition.enableRotation = false;
                definition.rotationMode = mode == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
                definition.enableAngularSpring = mode == 2;
                definition.enableMotor = true;
                definition.motorSpeed = direction * 2.0f;
                definition.maxMotorForce = 1.0f;
                const auto id = physics.createSplineRailJoint( definition );
                check( physics.IsValid( id ), "free closed rail fixture is created" );
                physics.SetBodyLinearVelocity( a, { 0.3f, -0.1f } );
                physics.SetBodyAngularVelocity( a, 0.2f );
                physics.SetBodyAngularVelocity( b, 2.3f );
                auto previous = physics.getSplineRailJointData( id );
                const double initial = previous.currentTranslation;
                int seams = 0;
                for( int frame = 0; frame < 900; ++frame )
                {
                    physics.Step( 1.0f / 60.0f, substeps );
                    const auto data = physics.getSplineRailJointData( id );
                    check( data.trackingValid && !data.enableRotation && std::abs( physics.GetBodyAngularVelocity( b ) - 2.3f ) < 0.00001f && std::abs( data.lateralError ) < 0.02f, "moving curved rail guides the centered anchor without constraining free angular velocity" );
                    if( ( direction > 0.0f && previous.parameter.segmentIndex == 3 && data.parameter.segmentIndex == 0 ) || ( direction < 0.0f && previous.parameter.segmentIndex == 0 && data.parameter.segmentIndex == 3 ) ) ++seams;
                    previous = data;
                }
                check( seams >= 2 && direction * ( previous.currentTranslation - initial ) > 2.0 * previous.length && std::abs( previous.currentSpeed - direction * 2.0f ) < 0.04f, "free rotation retains forward and reverse multi-circuit motor travel across seams" );
                const auto pose = physics.GetBodyTransform( b );
                const auto velocity = physics.GetBodyLinearVelocity( b );
                const vec2 edited[] = { { 2.2f, 0 }, { 0, 2.2f }, { -2.2f, 0 }, { 0, -2.2f } };
                check( physics.setSplineRailJointPoints( id, edited ), "free rail geometry remains editable" );
                physics.setSplineRailJointRotation( id, definition.rotationMode, 4.7f, 0.4f, definition.enableAngularSpring );
                const auto editedData = physics.getSplineRailJointData( id );
                check( !editedData.enableRotation && std::abs( editedData.currentTranslation - previous.currentTranslation ) < 0.000001 && editedData.referenceDistance == previous.referenceDistance && editedData.referenceAngle == previous.referenceAngle && editedData.tangentOffset == previous.tangentOffset && LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( b ) == 2.3f && physics.IsValid( shape ), "editing preserves free mode, accumulated distance, both references and body state" );
                const vec2 invalid[] = { {}, {}, {}, {} };
                check( !physics.setSplineRailJointPoints( id, invalid ) && !physics.getSplineRailJointData( id ).enableRotation && physics.getSplineRailJointData( id ).currentTranslation == editedData.currentTranslation && physics.IsValid( a ) && physics.IsValid( b ) && physics.GetBodyAngularVelocity( b ) == 2.3f, "failed geometry edit preserves free mode, winding and physical state" );
                physics.setSplineRailJointRotationEnabled( id, true );
                const auto resumed = physics.getSplineRailJointData( id );
                check( resumed.enableRotation && resumed.angularHertz == 4.7f && resumed.angularDampingRatio == 0.4f && resumed.rotationMode == editedData.rotationMode && resumed.tangentOffset == editedData.tangentOffset && resumed.referenceAngle == editedData.referenceAngle && resumed.currentAngle == editedData.currentAngle && resumed.currentTranslation == editedData.currentTranslation && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( b ) == 2.3f, "reactivation during curved travel preserves references, travel and velocity without recapturing the angle" );
                for( int frame = 0; frame < 60; ++frame ) physics.Step( 1.0f / 60.0f, substeps );
                check( physics.getSplineRailJointData( id ).trackingValid && std::abs( physics.GetBodyAngularVelocity( b ) - 2.3f ) > 0.05f, "saved angular control responds again on the moving curved rail" );
            }
}

void checkAnchorTorqueAndWarmStart()
{
    world physics{};
    physics.SetGravity( {} );
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { { -0.5f, -0.5f }, {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.2f } );
    const vec2 points[] = { { -4, 0 }, { 4, 0 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.localAnchorB = { 0.5f, 0.5f };
    definition.enableRotation = false;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "noncentral free anchor fixture is created" );
    physics.SetBodyLinearVelocity( b, { 0, 2 } );
    physics.Step( 1.0f / 60.0f, 4 );
    check( physics.getSplineRailJointData( id ).trackingValid && std::abs( physics.GetBodyAngularVelocity( b ) ) > 0.1f && std::abs( physics.getSplineRailJointData( id ).torque ) > 0.00001f, "normal reaction at a noncentral anchor still rotates the body while angular restoration is disabled" );

    splineRailJointSim2 joint{};
    joint.enableRotation = false;
    joint.rotationMode = splineRailRotationMode::followTangent;
    joint.subStepTime = 1.0f / 60.0f;
    check( setSplineRailGeometry( joint, points ), "warm-start geometry is created" );
    joint.trackingValid = true;
    joint.impulse = { 0.2f, 9.0f };
    bodySim bodyA{}, bodyB{};
    bodyB.invMass = bodyB.invInertia = 1.0f;
    auto constraint = prepareSplineRailJointConstraint( joint, bodyA, bodyB, joint.subStepTime );
    bodyState stateA{}, stateB{};
    stateB.angularVelocity = 2.0f;
    check( constraint.impulse.x == 0.2f && constraint.impulse.y == 0.0f, "disabled constraint discards stale angular warm-start impulse while retaining normal impulse" );
    warmStartSplineRailJointConstraint( constraint, stateA, stateB );
    check( stateB.angularVelocity == 2.0f && stateB.linearVelocity.y == 0.2f, "free warm start applies only normal reaction" );
    bodyB.invMass = bodyB.invInertia = 0.0f;
    constraint = prepareSplineRailJointConstraint( joint, bodyA, bodyB, joint.subStepTime );
    for( bool bias : { true, false } ) solveSplineRailJointConstraint( constraint, stateA, stateB, bias );
    check( IsFinite( stateB.linearVelocity ) && stateB.angularVelocity == 2.0f && IsFinite( constraint.force ) && std::isfinite( constraint.torque ), "zero effective mass remains finite without an angular row" );
}

}

int main()
{
    checkFreeRotation();
    checkToggleAndResponse();
    checkSleepAndState();
    checkAxialControls();
    checkClosedMovingRail();
    checkAnchorTorqueAndWarmStart();
    return 0;
}
