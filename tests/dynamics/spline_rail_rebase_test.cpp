#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <utility>
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

void checkOpen()
{
    world physics{};
    physics.SetGravity( {} );
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { { 1, 0 }, {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { -4, 0 }, { 4, 0 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.referenceDistance = 0.5;
    definition.enableSpring = true;
    definition.hertz = 2.0f;
    definition.targetTranslation = 3.0f;
    definition.enableLimit = true;
    definition.lowerTranslation = 1.0f;
    definition.upperTranslation = 7.0f;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "open rebase fixture is created" );
    physics.SetBodyLinearVelocity( b, { 2, 0 } );
    const auto before = physics.getSplineRailJointData( id );
    check( physics.rebaseSplineRailJoint( id ), "valid open rail accepts a new control origin" );
    const auto after = physics.getSplineRailJointData( id );
    check( after.currentTranslation == 0.0 && after.referenceDistance == 5.0 && after.currentSpeed == before.currentSpeed, "rebase changes the origin without changing actual speed" );
    check( after.targetTranslation == -1.5f && after.lowerTranslation == -3.5f && after.upperTranslation == 2.5f, "spring target and limits keep their physical positions in the new coordinate system" );
}
void checkPreserved( const splineRailJointData& before, const splineRailJointData& after )
{
    check( after.trackingValid && after.currentTranslation == 0.0 && after.referenceDistance == before.referenceDistance + before.currentTranslation, "new reference is the current unwrapped distance" );
    check( after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && after.length == before.length && after.isLoop == before.isLoop && after.currentSpeed == before.currentSpeed && after.currentAngle == before.currentAngle && after.lateralError == before.lateralError, "rebase preserves branch, geometry, speed and rotation reference" );
    check( after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && after.springForce == before.springForce && after.motorForce == before.motorForce, "rebase preserves cached reactions instead of clearing warm-start state" );
    check( after.referenceAngle == before.referenceAngle && after.rotationMode == before.rotationMode && after.enableAngularSpring == before.enableAngularSpring && after.angularHertz == before.angularHertz && after.angularDampingRatio == before.angularDampingRatio && after.enableSpring == before.enableSpring && after.hertz == before.hertz && after.dampingRatio == before.dampingRatio && after.enableLimit == before.enableLimit && after.enableMotor == before.enableMotor && after.motorSpeed == before.motorSpeed && after.maxMotorForce == before.maxMotorForce, "rebase preserves all tuning and mode selections" );
    check( std::abs( after.referenceDistance + after.targetTranslation - before.referenceDistance - before.targetTranslation ) < 0.00001 && std::abs( after.referenceDistance + after.lowerTranslation - before.referenceDistance - before.lowerTranslation ) < 0.00001 && std::abs( after.referenceDistance + after.upperTranslation - before.referenceDistance - before.upperTranslation ) < 0.00001, "spring and limit physical coordinates are unchanged within float rounding" );
}

jointId createRail( world& physics, bool loop, int rotation = 0 )
{
    physics.SetGravity( {} );
    const auto a = physics.CreateBody( bodyType::Kinematic );
    const auto b = physics.CreateBody( bodyType::Dynamic, { loop ? vec2{ 2, 0 } : vec2{ 1, 0 }, {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    physics.SetBodySleepEnabled( b, false );
    const vec2 line[] = { { -8, 0 }, { 8, 0 } };
    const vec2 ring[] = { { 2, 0 }, { 0, 2 }, { -2, 0 }, { 0, -2 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = loop ? std::span<const vec2>{ ring } : std::span<const vec2>{ line };
    definition.isLoop = loop;
    definition.referenceDistance = 0.3;
    definition.rotationMode = rotation == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
    definition.enableAngularSpring = rotation == 2;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "paired rebase rail is created" );
    physics.SetBodyLinearVelocity( a, { 0.1f, -0.2f } );
    physics.SetBodyAngularVelocity( a, 0.1f );
    return id;
}

void checkMotion( const world& physics, jointId id, const world& reference, jointId referenceId, double shift )
{
    const auto actual = physics.getSplineRailJointData( id ), expected = reference.getSplineRailJointData( referenceId );
    check( actual.trackingValid && expected.trackingValid && std::abs( actual.currentTranslation + shift - expected.currentTranslation ) < 0.002 && std::abs( actual.currentSpeed - expected.currentSpeed ) < 0.002f, "rebased accumulated travel and speed match the unre-based world" );
    for( auto bodies : { std::pair{ actual.bodyA, expected.bodyA }, std::pair{ actual.bodyB, expected.bodyB } } )
    {
        const auto pose = physics.GetBodyTransform( bodies.first ), expectedPose = reference.GetBodyTransform( bodies.second );
        check( Length( pose.position - expectedPose.position ) < 0.002f && std::abs( pose.rotation.c - expectedPose.rotation.c ) < 0.002f && std::abs( pose.rotation.s - expectedPose.rotation.s ) < 0.002f && Length( physics.GetBodyLinearVelocity( bodies.first ) - reference.GetBodyLinearVelocity( bodies.second ) ) < 0.002f && std::abs( physics.GetBodyAngularVelocity( bodies.first ) - reference.GetBodyAngularVelocity( bodies.second ) ) < 0.002f, "rebase preserves subsequent body motion within float solver rounding" );
    }
}

void checkTurnsAndSpring()
{
    for( int substeps : { 1, 4, 8 } )
        for( int rotation : { 0, 1, 2 } )
        for( float direction : { -1.0f, 1.0f } )
        {
            world physics{}, reference{};
            const auto id = createRail( physics, true, rotation ), referenceId = createRail( reference, true, rotation );
            physics.setSplineRailJointMotor( id, true, direction * 2.0f, 1.0f );
            reference.setSplineRailJointMotor( referenceId, true, direction * 2.0f, 1.0f );
            for( int frame = 0; frame < 900; ++frame )
            {
                physics.Step( 1.0f / 60.0f, substeps );
                reference.Step( 1.0f / 60.0f, substeps );
            }
            const auto travelled = physics.getSplineRailJointData( id );
            check( direction * travelled.currentTranslation > 2.0 * travelled.length, "origin reset starts after multiple forward or reverse turns" );
            const float target = static_cast<float>( travelled.currentTranslation + direction * 0.8 );
            for( auto rails : { std::pair{ &physics, id }, std::pair{ &reference, referenceId } } )
            {
                rails.first->setSplineRailJointMotor( rails.second, false, 0.0f, 1.0f );
                rails.first->setSplineRailJointSpring( rails.second, true, 0.25f, 1.0f, target );
                rails.first->setSplineRailJointLimit( rails.second, true, static_cast<float>( travelled.currentTranslation - 1.2 ), static_cast<float>( travelled.currentTranslation + 1.2 ) );
                for( int frame = 0; frame < 5; ++frame ) rails.first->Step( 1.0f / 60.0f, substeps );
            }
            const auto before = physics.getSplineRailJointData( id );
            check( before.springForce != 0.0f, "rebase fixture has a live cached spring reaction" );
            const auto pose = physics.GetBodyTransform( before.bodyB );
            const auto velocity = physics.GetBodyLinearVelocity( before.bodyB );
            const auto spin = physics.GetBodyAngularVelocity( before.bodyB );
            check( physics.rebaseSplineRailJoint( id ), "multi-turn rail can reset its control origin while moving" );
            const auto after = physics.getSplineRailJointData( id );
            checkPreserved( before, after );
            check( LengthSquared( physics.GetBodyTransform( before.bodyB ).position - pose.position ) == 0.0f && physics.GetBodyTransform( before.bodyB ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( before.bodyB ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( before.bodyB ) == spin, "origin reset never teleports or changes body velocities" );
            check( physics.rebaseSplineRailJoint( id ) && physics.getSplineRailJointData( id ).referenceDistance == after.referenceDistance, "a repeated origin reset is a no-op" );
            for( int frame = 0; frame < 480; ++frame )
            {
                physics.Step( 1.0f / 60.0f, substeps );
                reference.Step( 1.0f / 60.0f, substeps );
                checkMotion( physics, id, reference, referenceId, before.currentTranslation );
            }
            const auto settled = physics.getSplineRailJointData( id );
            check( std::abs( settled.currentTranslation - settled.targetTranslation ) < 0.02, "rebased spring converges to the original physical target" );
        }
}

void checkLimitsAndOpenStops()
{
    for( bool loop : { false, true } )
        for( int limit : { 0, 1, 2 } )
        {
            world physics{}, reference{};
            const auto id = createRail( physics, loop ), referenceId = createRail( reference, loop );
            const float start = static_cast<float>( physics.getSplineRailJointData( id ).currentTranslation );
            const float target = start + ( limit == 1 ? -0.6f : 0.6f );
            for( auto rails : { std::pair{ &physics, id }, std::pair{ &reference, referenceId } } )
            {
                if( limit == 2 ) rails.first->setSplineRailJointLimit( rails.second, true, target, target );
                else rails.first->setSplineRailJointLimit( rails.second, true, start - 0.6f, start + 0.6f );
                rails.first->setSplineRailJointMotor( rails.second, true, limit == 1 ? -2.0f : 2.0f, 1.0f );
                for( int frame = 0; frame < 180; ++frame ) rails.first->Step( 1.0f / 60.0f, 4 );
            }
            const auto before = physics.getSplineRailJointData( id );
            check( std::abs( before.currentTranslation - target ) < 0.02, "limit fixture reaches its physical stop" );
            check( physics.rebaseSplineRailJoint( id ), "loaded lower, upper or equal limit can be rebased" );
            checkPreserved( before, physics.getSplineRailJointData( id ) );
            for( int frame = 0; frame < 180; ++frame )
            {
                physics.Step( 1.0f / 60.0f, 4 );
                reference.Step( 1.0f / 60.0f, 4 );
                checkMotion( physics, id, reference, referenceId, before.currentTranslation );
            }
            if( !loop )
            {
                physics.setSplineRailJointLimit( id, false, 0.0f, 0.0f );
                reference.setSplineRailJointLimit( referenceId, false, 0.0f, 0.0f );
                for( int frame = 0; frame < 400; ++frame )
                {
                    physics.Step( 1.0f / 60.0f, 4 );
                    reference.Step( 1.0f / 60.0f, 4 );
                    checkMotion( physics, id, reference, referenceId, before.currentTranslation );
                }
                const auto stopped = physics.getSplineRailJointData( id );
                const double endpoint = limit == 1 ? -stopped.referenceDistance : stopped.length - stopped.referenceDistance;
                check( std::abs( stopped.currentTranslation - endpoint ) < 0.02, "open physical endpoint stops remain in place after rebasing" );
            }
        }
}

void checkSleeping()
{
    world physics{};
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { { 1, 0 }, {} } );
    const auto shape = physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { -4, 0 }, { 4, 0 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "sleeping fixture is created" );
    physics.SetBodyAwake( b, false );
    const auto before = physics.getSplineRailJointData( id );
    const float mass = physics.GetBodyMass( b ), friction = physics.GetShapeFriction( shape );
    check( physics.rebaseSplineRailJoint( id ), "sleeping rail can reset its origin" );
    checkPreserved( before, physics.getSplineRailJointData( id ) );
    check( !physics.IsBodyAwake( b ) && physics.IsValid( shape ) && physics.IsValid( id ) && physics.GetBodyMass( b ) == mass && physics.GetShapeFriction( shape ) == friction, "rebase preserves sleeping state, handles and material" );
    check( physics.rebaseSplineRailJoint( id ) && !physics.IsBodyAwake( b ), "repeated zero-origin reset does not wake a sleeping body" );
}

void checkFailures()
{
    for( int failure : { 0, 1, 2, 3, 4, 5 } )
    {
        world physics{};
        const auto a = physics.CreateBody();
        const auto b = physics.CreateBody( bodyType::Dynamic, { { failure == 2 ? 1.0e38f : ( failure == 3 || failure == 4 ) ? 1.0e10f : failure == 5 ? 1.0e8f : -5.0e37f, 0 }, {} } );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        const vec2 points[] = { { -4, 0 }, { 4, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        if( failure == 0 ) definition.targetTranslation = std::numeric_limits<float>::max();
        if( failure == 1 ) definition.lowerTranslation = definition.upperTranslation = std::numeric_limits<float>::max();
        if( failure == 3 ) { definition.lowerTranslation = 1.0f; definition.upperTranslation = 2.0f; }
        if( failure == 5 ) { definition.enableLimit = true; definition.lowerTranslation = -20.0f; definition.upperTranslation = 1.0f; }
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "numeric failure fixture starts valid" );
        physics.SetBodyAwake( b, false );
        const auto before = physics.getSplineRailJointData( id );
        check( before.trackingValid && !physics.rebaseSplineRailJoint( id ), failure == 4 ? "implicit open endpoints must not collapse during rebase" : failure == 5 ? "clipped open limit interval must not collapse during rebase" : "overflow or a collapsed nonzero limit interval rejects rebase" );
        const auto after = physics.getSplineRailJointData( id );
        check( after.referenceDistance == before.referenceDistance && after.currentTranslation == before.currentTranslation && after.targetTranslation == before.targetTranslation && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && !physics.IsBodyAwake( b ), "numeric conversion failure leaves all coordinates and sleep state unchanged" );
    }
    world physics{};
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { { 0, 2 }, {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { -2, 0 }, { 0, 2 }, { 2, 0 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "tracking failure fixture starts valid" );
    const auto valid = physics.getSplineRailJointData( id );
    const auto curvature = Curvature( { points }, valid.parameter.segmentIndex, valid.parameter.t );
    check( curvature.has_value() && *curvature != 0.0, "tracking failure fixture has nonzero curvature" );
    physics.SetBodyTransform( b, { valid.anchorA + static_cast<float>( 1.0 / *curvature ) * valid.normal, {} } );
    physics.SetBodyAwake( b, false );
    const auto before = physics.getSplineRailJointData( id );
    check( !before.trackingValid && !physics.rebaseSplineRailJoint( id ), "invalid projection cannot replace the control origin" );
    const auto after = physics.getSplineRailJointData( id );
    check( after.referenceDistance == before.referenceDistance && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && after.targetTranslation == before.targetTranslation && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && !physics.IsBodyAwake( b ), "tracking failure preserves coordinates, branch and sleeping state" );
}

}

int main()
{
    checkOpen();
    checkTurnsAndSpring();
    checkLimitsAndOpenStops();
    checkSleeping();
    checkFailures();
    return 0;
}
