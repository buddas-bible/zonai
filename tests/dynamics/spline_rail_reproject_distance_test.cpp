#include <cmath>
#include <cstdio>
#include <cstdlib>
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

void checkBoundaries()
{
    for( bool selected : { false, true } )
        for( float distance : { 0.5f, 1.0f, 1.5f } )
        {
            world physics{};
            physics.SetGravity( {} );
            const auto a = physics.CreateBody();
            const auto b = physics.CreateBody( bodyType::Dynamic, { { -1, 0 }, {} } );
            ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
            const vec2 points[] = { { -4, 0 }, { 4, 0 } };
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "distance boundary fixture is created" );
            physics.SetBodyTransform( b, { { 0, distance }, {} } );
            physics.SetBodyAwake( b, false );
            const bool result = selected ? physics.reprojectSplineRailJointWithinDistance( id, 0, 1.0f ) : physics.reprojectSplineRailJointWithinDistance( id, 1.0f );
            check( result == ( distance <= 1.0f ), "reprojection accepts distance below or equal to the limit and rejects distance above it" );
            check( physics.IsBodyAwake( b ) == result, "only an accepted changed projection wakes the body" );
        }
}
void checkAnchorAndTransform()
{
    for( bool selected : { false, true } )
        for( bool near : { false, true } )
        {
            world physics{};
            physics.SetGravity( {} );
            const transform2 poseA{ { 5, -3 }, { 0, 1 } };
            const auto a = physics.CreateBody( bodyType::Kinematic, poseA );
            const vec2 anchor{ 3, 0 };
            const auto b = physics.CreateBody( bodyType::Dynamic, { TransformPoint( poseA, { -1, 0 } ) - anchor, {} } );
            const auto shape = physics.CreateShape( b, circle2{ { 0.2f, 0 }, 0.1f } );
            physics.SetShapeFriction( shape, 0.42f );
            const vec2 points[] = { { -4, 0 }, { 4, 0 } };
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            definition.localAnchorB = anchor;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "transformed anchor distance fixture is created" );
            const transform2 pose{ TransformPoint( poseA, { 0, near ? 0.5f : -3.0f } ) - anchor, {} };
            physics.SetBodyTransform( b, pose );
            physics.SetBodyLinearVelocity( a, { 0.1f, -0.2f } );
            physics.SetBodyAngularVelocity( a, 0.3f );
            physics.SetBodyLinearVelocity( b, { 2, 1 } );
            physics.SetBodyAngularVelocity( b, 0.4f );
            const float mass = physics.GetBodyMass( b );
            const bool result = selected ? physics.reprojectSplineRailJointWithinDistance( id, 0, 1.0f ) : physics.reprojectSplineRailJointWithinDistance( id, 1.0f );
            check( result == near, "distance uses the offset anchor in transformed world coordinates, not the body origin or mass center" );
            check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyLinearVelocity( b ).x == 2.0f && physics.GetBodyAngularVelocity( b ) == 0.4f && LengthSquared( physics.GetBodyTransform( a ).position - poseA.position ) == 0.0f && physics.GetBodyLinearVelocity( a ).x == 0.1f && physics.GetBodyAngularVelocity( a ) == 0.3f && physics.GetBodyMass( b ) == mass && physics.GetShapeFriction( shape ) == 0.42f && physics.IsValid( shape ) && physics.IsValid( id ), "distance acceptance and rejection preserve both bodies, handles and material" );
        }
}

void checkEndpoints()
{
    for( bool selected : { false, true } )
        for( float direction : { -1.0f, 1.0f } )
        for( float limit : { 4.99f, 5.0f, 5.01f } )
        {
            world physics{};
            physics.SetGravity( {} );
            const auto a = physics.CreateBody();
            const auto b = physics.CreateBody( bodyType::Dynamic );
            ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
            const vec2 points[] = { { -4, 0 }, { 4, 0 } };
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "endpoint distance fixture is created" );
            physics.SetBodyTransform( b, { { direction * 7.0f, 4 }, {} } );
            const bool result = selected ? physics.reprojectSplineRailJointWithinDistance( id, 0, limit ) : physics.reprojectSplineRailJointWithinDistance( id, limit );
            check( result == ( limit >= 5.0f ), "open endpoint distance includes tangent separation in the 3-4-5 triangle, not only lateral error" );
            if( result ) check( std::abs( physics.getSplineRailJointData( id ).currentTranslation - ( direction * 7.0 + 4.0 ) ) < 0.000001, "bounded reprojection keeps the existing open endpoint translation extension" );
        }
}

void checkLargeAndZeroDistances()
{
    for( bool selected : { false, true } )
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
        check( physics.IsValid( id ), "large and zero distance fixture is created" );
        physics.SetBodyAwake( b, false );
        const bool exact = selected ? physics.reprojectSplineRailJointWithinDistance( id, 0, 0.0f ) : physics.reprojectSplineRailJointWithinDistance( id, 0.0f );
        check( exact && !physics.IsBodyAwake( b ), "zero distance accepts an exact unchanged point without waking it" );
        physics.SetBodyTransform( b, { { 0, 2.0e20f }, {} } );
        const bool outside = selected ? physics.reprojectSplineRailJointWithinDistance( id, 0, 1.0e20f ) : physics.reprojectSplineRailJointWithinDistance( id, 1.0e20f );
        const bool inside = selected ? physics.reprojectSplineRailJointWithinDistance( id, 0, 3.0e20f ) : physics.reprojectSplineRailJointWithinDistance( id, 3.0e20f );
        check( !outside && inside, "double squared-distance arithmetic distinguishes large finite distances without float overflow" );
    }
}

void checkSeamAndRebase()
{
    for( bool selected : { false, true } )
        for( float direction : { -1.0f, 1.0f } )
        {
            world physics{};
            physics.SetGravity( {} );
            const auto a = physics.CreateBody();
            const auto b = physics.CreateBody( bodyType::Dynamic, { { 2, 0 }, {} } );
            ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
            physics.SetBodySleepEnabled( b, false );
            const vec2 points[] = { { 2, 0 }, { 0, 2 }, { -2, 0 }, { 0, -2 } };
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            definition.isLoop = true;
            definition.referenceDistance = 0.35;
            definition.enableMotor = true;
            definition.motorSpeed = direction * 2.0f;
            definition.maxMotorForce = 1.0f;
            definition.targetTranslation = 2.0f;
            definition.lowerTranslation = -100.0f;
            definition.upperTranslation = 100.0f;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "bounded seam fixture is created" );
            const double initial = physics.getSplineRailJointData( id ).currentTranslation;
            for( int frame = 0; frame < 900; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
            auto before = physics.getSplineRailJointData( id );
            check( direction * ( before.currentTranslation - initial ) > 2.0 * before.length && physics.rebaseSplineRailJoint( id ), "bounded reprojection follows multiple circuits and a control-origin reset" );
            before = physics.getSplineRailJointData( id );
            const double oldDistance = before.referenceDistance;
            const double lower = std::floor( oldDistance / before.length ) * before.length;
            const double expected = oldDistance - lower <= lower + before.length - oldDistance ? lower : lower + before.length;
            physics.SetBodyTransform( b, { { 2.25f, 0 }, {} } );
            const auto pose = physics.GetBodyTransform( b );
            const auto velocity = physics.GetBodyLinearVelocity( b );
            const bool outside = selected ? physics.reprojectSplineRailJointWithinDistance( id, 3, 0.2f ) : physics.reprojectSplineRailJointWithinDistance( id, 0.2f );
            check( !outside, "distance outside a closed seam is rejected before changing the winding reference" );
            const bool inside = selected ? physics.reprojectSplineRailJointWithinDistance( id, 3, 0.25f ) : physics.reprojectSplineRailJointWithinDistance( id, 0.25f );
            check( inside, "distance exactly on the closed seam threshold is accepted" );
            const auto after = physics.getSplineRailJointData( id );
            check( after.trackingValid && std::abs( after.currentTranslation + after.referenceDistance - expected ) < 0.000001 && after.referenceDistance == before.referenceDistance && after.targetTranslation == before.targetTranslation && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && after.motorSpeed == before.motorSpeed, "closed distance rejection preserves the next winding choice and rebase settings" );
            check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f, "bounded seam reprojection preserves physical pose and velocity" );
            for( int frame = 0; frame < 240; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
            const auto resumed = physics.getSplineRailJointData( id );
            check( resumed.trackingValid && direction * ( resumed.currentTranslation - after.currentTranslation ) > 7.0 && std::abs( resumed.currentSpeed - direction * 2.0f ) < 0.04f, "ordinary signed seam crossing resumes after bounded reprojection" );
        }
}

void checkSameState( const splineRailJointData& before, const splineRailJointData& after )
{
    check( after.trackingValid == before.trackingValid && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && after.currentTranslation == before.currentTranslation && after.currentSpeed == before.currentSpeed && after.referenceDistance == before.referenceDistance && after.targetTranslation == before.targetTranslation && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && after.referenceAngle == before.referenceAngle && after.rotationMode == before.rotationMode, "distance rejection preserves branch, winding, origin and control settings" );
    check( after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && after.springForce == before.springForce && after.motorForce == before.motorForce, "distance rejection and unchanged acceptance preserve cached reactions" );
}

void checkFailuresAndNoOp()
{
    world physics{};
    physics.SetGravity( { 2, 0 } );
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { { 2, 0 }, {} } );
    const auto shape = physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { 2, 0 }, { 0, 2 }, { -2, 0 }, { 0, -2 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.isLoop = true;
    definition.referenceDistance = 0.35;
    definition.targetTranslation = 1.0f;
    definition.lowerTranslation = -10.0f;
    definition.upperTranslation = 10.0f;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "distance failure fixture is created" );
    for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
    physics.SetBodyAwake( b, false );
    const auto before = physics.getSplineRailJointData( id );
    const auto pose = physics.GetBodyTransform( b );
    const auto velocity = physics.GetBodyLinearVelocity( b );
    check( before.trackingValid && Length( before.force ) > 0.01f && before.anchorB.x > 2.0f, "failure fixture has a nonzero cached load and nonzero curve distance" );
    for( bool selected : { false, true } )
    {
        for( float limit : { 0.0f, -1.0f, std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity() } )
        {
            const bool result = selected ? physics.reprojectSplineRailJointWithinDistance( id, 0, limit ) : physics.reprojectSplineRailJointWithinDistance( id, limit );
            check( !result, "too-small, negative and nonfinite distance limits are rejected even for an unchanged projection" );
            checkSameState( before, physics.getSplineRailJointData( id ) );
            check( !physics.IsBodyAwake( b ) && LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f && physics.IsValid( shape ) && physics.IsValid( id ), "distance failure preserves sleep and body state without partial mutation" );
        }
        const bool unchanged = selected ? physics.reprojectSplineRailJointWithinDistance( id, 0, 1.0f ) : physics.reprojectSplineRailJointWithinDistance( id, 1.0f );
        check( unchanged && !physics.IsBodyAwake( b ), "bounded acceptance of the same projection does not wake the body" );
        checkSameState( before, physics.getSplineRailJointData( id ) );
    }
    for( std::size_t segment : { std::size_t{ 1 }, std::size_t{ 4 }, std::numeric_limits<std::size_t>::max() } )
    {
        check( !physics.reprojectSplineRailJointWithinDistance( id, segment, 100.0f ), "invalid or singular selected segment cannot fall back to a nearby global projection" );
        checkSameState( before, physics.getSplineRailJointData( id ) );
        check( !physics.IsBodyAwake( b ), "selected projection failure does not wake the body" );
    }
    check( physics.reprojectSplineRailJoint( id ) && physics.reprojectSplineRailJoint( id, 0 ), "unbounded overloads retain their existing behavior" );
    checkSameState( before, physics.getSplineRailJointData( id ) );
}

}

int main()
{
    checkBoundaries();
    checkAnchorAndTransform();
    checkEndpoints();
    checkLargeAndZeroDistances();
    checkSeamAndRebase();
    checkFailuresAndNoOp();
    return 0;
}
