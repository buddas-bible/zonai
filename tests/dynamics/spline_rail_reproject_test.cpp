#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
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

void checkTeleport()
{
    world physics{};
    physics.SetGravity( {} );
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { { 2, 0 }, {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { 2, 0 }, { 0, 2 }, { -2, 0 }, { 0, -2 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.isLoop = true;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "reprojection fixture is created" );
    const double length = physics.getSplineRailJointData( id ).length;
    physics.SetBodyTransform( b, { { -2, 0 }, {} } );
    physics.SetBodyLinearVelocity( b, { 0, -1 } );
    check( !physics.getSplineRailJointData( id ).trackingValid, "opposite teleport invalidates local branch tracking" );
    check( physics.reprojectSplineRailJoint( id ), "explicit global reprojection recovers a teleported rail" );
    const auto data = physics.getSplineRailJointData( id );
    check( data.trackingValid && std::abs( data.currentTranslation + length * 0.5 ) < 0.000001, "half-circuit tie selects the smaller accumulated coordinate" );
    check( physics.GetBodyTransform( b ).position.x == -2.0f && physics.GetBodyLinearVelocity( b ).y == -1.0f, "reprojection never teleports or changes body velocity" );
    physics.SetBodyTransform( b, { points[0], {} } );
    check( physics.reprojectSplineRailJoint( id ) && std::abs( physics.getSplineRailJointData( id ).currentTranslation + length ) < 0.000001, "opposite half-circuit tie also selects the smaller accumulated coordinate" );
}
void checkSequentialTeleports()
{
    world physics{};
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { { 2, 0 }, {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { 2, 0 }, { 0, 2 }, { -2, 0 }, { 0, -2 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.isLoop = true;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "sequential teleport fixture is created" );
    const double length = physics.getSplineRailJointData( id ).length;
    for( int step = 1; step <= 5; ++step )
    {
        physics.SetBodyTransform( b, { points[step % 4], {} } );
        check( physics.reprojectSplineRailJoint( id ), "each explicit teleport updates the stored tracking reference" );
        const auto data = physics.getSplineRailJointData( id );
        check( data.trackingValid && std::abs( data.currentTranslation - step * length * 0.25 ) < 0.000001, "successive reprojections choose the nearest winding from the last successful call without a physics step" );
    }
}

void checkSettings( const splineRailJointData& before, const splineRailJointData& after )
{
    check( after.referenceDistance == before.referenceDistance && after.referenceAngle == before.referenceAngle && after.targetTranslation == before.targetTranslation && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && after.length == before.length && after.isLoop == before.isLoop, "reprojection preserves origin, rotation reference, spring target, limits and geometry" );
    check( after.rotationMode == before.rotationMode && after.enableAngularSpring == before.enableAngularSpring && after.angularHertz == before.angularHertz && after.angularDampingRatio == before.angularDampingRatio && after.enableSpring == before.enableSpring && after.hertz == before.hertz && after.dampingRatio == before.dampingRatio && after.enableLimit == before.enableLimit && after.enableMotor == before.enableMotor && after.motorSpeed == before.motorSpeed && after.maxMotorForce == before.maxMotorForce && after.collideConnected == before.collideConnected, "reprojection preserves all tuning and mode selections" );
}

void checkAnchorAndOpenEndpoint()
{
    world physics{};
    physics.SetGravity( {} );
    const transform2 poseA{ { 0.3f, -0.2f }, rot2::FromRadians( 0.4f ) };
    const auto a = physics.CreateBody( bodyType::Kinematic, poseA );
    const vec2 anchor{ 0.25f, -0.4f };
    const auto rotation = rot2::FromRadians( 0.7f );
    const auto b = physics.CreateBody( bodyType::Dynamic, { TransformPoint( poseA, { -1, 0 } ) - Rotate( rotation, anchor ), rotation } );
    ( void )physics.CreateShape( b, circle2{ { 0.1f, 0.2f }, 0.1f } );
    const vec2 points[] = { { -4, 0 }, { 4, 0 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.localAnchorB = anchor;
    definition.referenceDistance = 0.35;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "offset anchor reprojection fixture is created" );
    const transform2 pose{ TransformPoint( poseA, { 6, 0.2f } ) - Rotate( rotation, anchor ), rotation };
    physics.SetBodyTransform( b, pose );
    physics.SetBodyLinearVelocity( b, { 3, 1 } );
    physics.SetBodyAngularVelocity( b, 0.4f );
    const auto before = physics.getSplineRailJointData( id );
    check( physics.reprojectSplineRailJoint( id ), "offset anchor can be reprojected past an open endpoint" );
    const auto after = physics.getSplineRailJointData( id );
    checkSettings( before, after );
    check( after.trackingValid && std::abs( after.currentTranslation - 9.65 ) < 0.00001 && Length( after.anchorB - TransformPoint( poseA, { 6, 0.2f } ) ) < 0.00001f, "open distance includes endpoint extension and uses the body anchor instead of its origin or center of mass" );
    check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyTransform( b ).rotation.s == rotation.s && physics.GetBodyLinearVelocity( b ).x == 3.0f && physics.GetBodyAngularVelocity( b ) == 0.4f, "offset-anchor reprojection preserves dynamic pose and velocity" );
}

void checkCrossing()
{
    world physics{};
    physics.SetGravity( {} );
    const vec2 points[] = { { -2, 0 }, { 0, 0 }, { 2, 2 }, { 2, -2 }, { 0, 0 }, { -2, 2 } };
    const auto a = physics.CreateBody( bodyType::Dynamic );
    ( void )physics.CreateShape( a, circle2{ {}, 0.2f } );
    const auto b = physics.CreateBody( bodyType::Dynamic, { Evaluate( { points }, 3, 0.9f ), {} } );
    const auto shape = physics.CreateShape( b, circle2{ {}, 0.1f } );
    physics.SetShapeFriction( shape, 0.42f );
    physics.SetShapeRestitution( shape, 0.23f );
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.referenceDistance = 0.3;
    definition.rotationMode = splineRailRotationMode::followTangent;
    definition.enableAngularSpring = false;
    definition.enableSpring = true;
    definition.hertz = 2.0f;
    definition.targetTranslation = 4.0f;
    definition.enableLimit = true;
    definition.lowerTranslation = 0.0f;
    definition.upperTranslation = 100.0f;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "crossing reprojection fixture is created" );
    physics.SetBodyTransform( b, { { 0, 0 }, rot2::FromRadians( 0.3f ) } );
    physics.SetBodyLinearVelocity( b, { 1, -2 } );
    physics.SetBodyAngularVelocity( b, 0.7f );
    const auto before = physics.getSplineRailJointData( id );
    check( before.trackingValid && before.parameter.segmentIndex >= 3, "ordinary tracking retains the later crossing branch" );
    const auto pose = physics.GetBodyTransform( b );
    const auto velocity = physics.GetBodyLinearVelocity( b );
    const float spin = physics.GetBodyAngularVelocity( b ), mass = physics.GetBodyMass( b );
    check( physics.reprojectSplineRailJoint( id ), "explicit reprojection may select the earlier nearest crossing branch" );
    const auto after = physics.getSplineRailJointData( id );
    checkSettings( before, after );
    check( after.trackingValid && after.parameter.segmentIndex < 2 && after.currentTranslation < before.currentTranslation - 1.0, "open reprojection reports the newly selected geometric distance" );
    check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyTransform( b ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( b ) == spin && physics.GetBodyMass( b ) == mass && physics.IsValid( shape ) && physics.IsValid( id ) && physics.GetShapeFriction( shape ) == 0.42f && physics.GetShapeRestitution( shape ) == 0.23f, "changing projection preserves body pose, velocity, handles and material" );
    physics.SetBodyAwake( a, false );
    physics.SetBodyAwake( b, false );
    check( physics.reprojectSplineRailJoint( id ) && !physics.IsBodyAwake( a ) && !physics.IsBodyAwake( b ), "identical repeated reprojection does not wake either connected body" );
}

void checkTurnsAndRebase()
{
    for( int substeps : { 1, 4, 8 } )
        for( int rotation : { 0, 1, 2 } )
        for( float direction : { -1.0f, 1.0f } )
        {
            world physics{};
            physics.SetGravity( {} );
            std::array points{ vec2{ 2, 0 }, vec2{ 0, 2 }, vec2{ -2, 0 }, vec2{ 0, -2 } };
            const transform2 poseA{ { 0.3f, -0.2f }, rot2::FromRadians( 0.2f ) };
            const auto a = physics.CreateBody( bodyType::Kinematic, poseA );
            const auto b = physics.CreateBody( bodyType::Dynamic, { TransformPoint( poseA, points[0] ), poseA.rotation } );
            ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
            physics.SetBodySleepEnabled( b, false );
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            definition.isLoop = true;
            definition.referenceDistance = 0.35;
            definition.rotationMode = rotation == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
            definition.enableAngularSpring = rotation == 2;
            definition.enableMotor = true;
            definition.motorSpeed = direction * 2.0f;
            definition.maxMotorForce = 1.0f;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "multi-turn reprojection fixture is created" );
            const double initial = physics.getSplineRailJointData( id ).currentTranslation;
            for( int frame = 0; frame < 900; ++frame ) physics.Step( 1.0f / 60.0f, substeps );
            auto before = physics.getSplineRailJointData( id );
            check( direction * ( before.currentTranslation - initial ) > 2.0 * before.length, "reprojection follows multiple forward or reverse turns" );
            if( rotation != 0 )
            {
                check( physics.rebaseSplineRailJoint( id ), "multi-turn origin can be reset before reprojection" );
                check( physics.getSplineRailJointData( id ).currentTranslation == 0.0, "origin reset starts with zero control displacement" );
            }
            if( rotation == 2 )
            {
                points = { vec2{ 2.3f, 0 }, vec2{ 0, 2.1f }, vec2{ -2.4f, 0 }, vec2{ 0, -2 } };
                check( physics.setSplineRailJointPoints( id, points ), "edited closed rail retains a nonintegral phase offset" );
            }
            before = physics.getSplineRailJointData( id );
            const auto geometry = prepareSpline( { points, true } );
            check( geometry.has_value(), "independent distance reference is prepared" );
            const auto oldPhase = geometry->getDistance( before.parameter.segmentIndex, before.parameter.t, 1.0e-8 );
            const std::size_t segment = ( before.parameter.segmentIndex + 1 ) % points.size();
            const auto newPhase = geometry->getDistance( segment, 0.25f, 1.0e-8 );
            check( oldPhase.has_value() && newPhase.has_value(), "independent winding reference distances are available" );
            const double oldDistance = before.currentTranslation + before.referenceDistance;
            const double base = *newPhase + oldDistance - *oldPhase;
            const double lower = base + std::floor( ( oldDistance - base ) / before.length ) * before.length;
            const double expected = oldDistance - lower <= lower + before.length - oldDistance ? lower : lower + before.length;
            auto pose = physics.GetBodyTransform( b );
            pose.position = TransformPoint( poseA, geometry->evaluate( segment, 0.25f ) );
            physics.SetBodyTransform( b, pose );
            const auto velocity = physics.GetBodyLinearVelocity( b );
            const auto spin = physics.GetBodyAngularVelocity( b );
            check( physics.reprojectSplineRailJoint( id ), "closed reprojection succeeds after turns, rebase and edits" );
            const auto after = physics.getSplineRailJointData( id );
            checkSettings( before, after );
            check( after.trackingValid && std::abs( after.currentTranslation + after.referenceDistance - expected ) < 0.00001, "closed reprojection selects the closest accumulated coordinate and preserves the edited phase offset" );
            check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyTransform( b ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( b ) == spin && after.springForce == 0.0f && after.motorForce == 0.0f && LengthSquared( after.force ) == 0.0f && after.torque == 0.0f, "changed reprojection preserves physical state and starts with cleared constraint reactions" );
            for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, substeps );
            const auto resumed = physics.getSplineRailJointData( id );
            check( resumed.trackingValid && direction * ( resumed.currentTranslation - after.currentTranslation ) > 3.0 && std::abs( resumed.currentSpeed - direction * 2.0f ) < 0.04f, "normal signed motor travel resumes after explicit reprojection" );
        }
}

void checkNoOpAndWake()
{
    world physics{};
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { -4, 0 }, { 4, 0 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "loaded no-op fixture is created" );
    for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
    physics.SetBodyAwake( b, false );
    const auto before = physics.getSplineRailJointData( id );
    check( Length( before.force ) > 0.01f, "unchanged reprojection has a cached load reaction to preserve" );
    check( physics.reprojectSplineRailJoint( id ), "identical loaded projection succeeds" );
    const auto after = physics.getSplineRailJointData( id );
    check( !physics.IsBodyAwake( b ) && after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && after.currentTranslation == before.currentTranslation, "identical reprojection preserves sleeping state and cached reactions" );
    physics.SetBodyTransform( b, { { 1, 0 }, {} } );
    physics.SetBodyAwake( b, false );
    check( physics.reprojectSplineRailJoint( id ) && physics.IsBodyAwake( b ), "a changed stored projection wakes the connected dynamic body" );
}

void checkFailure()
{
    world physics{};
    physics.SetGravity( {} );
    std::array<vec2, 32> points{};
    for( std::size_t i = 0; i < points.size(); ++i )
    {
        const double angle = 2.0 * std::numbers::pi * i / points.size();
        points[i] = { static_cast<float>( 2.0 * std::cos( angle ) ), static_cast<float>( 2.0 * std::sin( angle ) ) };
    }
    const auto a = physics.CreateBody( bodyType::Dynamic );
    ( void )physics.CreateShape( a, circle2{ {}, 0.1f } );
    const auto b = physics.CreateBody( bodyType::Dynamic, { points[0], {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.isLoop = true;
    definition.referenceDistance = 0.35;
    definition.enableSpring = true;
    definition.hertz = 0.5f;
    definition.targetTranslation = 1.0f;
    definition.enableLimit = true;
    definition.lowerTranslation = -10.0f;
    definition.upperTranslation = 10.0f;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "projection failure fixture is created" );
    const auto valid = physics.getSplineRailJointData( id );
    physics.SetBodyTransform( b, {} );
    physics.SetBodyAwake( a, false );
    physics.SetBodyAwake( b, false );
    const auto before = physics.getSplineRailJointData( id );
    check( !before.trackingValid, "center projection is singular before explicit reprojection" );
    for( int call = 0; call < 3; ++call )
    {
        check( !physics.reprojectSplineRailJoint( id ), "singular global projection is rejected" );
        const auto after = physics.getSplineRailJointData( id );
        checkSettings( before, after );
        check( !after.trackingValid && after.currentSpeed == before.currentSpeed && after.currentTranslation == before.currentTranslation && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && !physics.IsBodyAwake( a ) && !physics.IsBodyAwake( b ), "failed reprojection preserves tracking, cached state and sleep without partial updates" );
    }
    physics.SetBodyTransform( b, { points[0], {} } );
    const auto restored = physics.getSplineRailJointData( id );
    check( restored.trackingValid && restored.currentTranslation == valid.currentTranslation && restored.parameter.segmentIndex == valid.parameter.segmentIndex && restored.parameter.t == valid.parameter.t, "failed reprojections leave the original winding and branch available for later recovery" );
}

}

int main()
{
    checkTeleport();
    checkSequentialTeleports();
    checkCrossing();
    checkAnchorAndOpenEndpoint();
    checkTurnsAndRebase();
    checkNoOpAndWake();
    checkFailure();
    return 0;
}
