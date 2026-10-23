#include <array>
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

void checkCrossing()
{
    world physics{};
    physics.SetGravity( {} );
    const vec2 points[] = { { -2, 0 }, { 0, 0 }, { 2, 2 }, { 2, -2 }, { 0, 0 }, { -2, 2 } };
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { Evaluate( { points }, 3, 0.9f ), {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "segment crossing fixture is created" );
    physics.SetBodyTransform( b, {} );
    check( physics.reprojectSplineRailJoint( id, 0 ), "caller can select the first crossing branch" );
    const auto first = physics.getSplineRailJointData( id );
    check( first.trackingValid && first.parameter.segmentIndex == 0, "first segment is selected at a shared crossing point" );
    check( physics.reprojectSplineRailJoint( id, 3 ), "caller can select the later crossing branch" );
    const auto later = physics.getSplineRailJointData( id );
    check( later.trackingValid && later.parameter.segmentIndex == 3 && later.currentTranslation > first.currentTranslation + 1.0, "segment selection distinguishes geometric crossings at the same position" );
}
void checkEndpointSelection()
{
    splineRailJointSim2 rail{};
    const vec2 line[] = { { -2, 0 }, { 0, 0 }, { 2, 0 } };
    check( setSplineRailGeometry( rail, line ), "restricted endpoint geometry is valid" );
    const transform2 body{ { 1, 0 }, {} };
    const auto selected = getSplineRailFrame( rail, {}, body, {}, false, 0.0, 0 );
    check( selected.valid && selected.parameter.segmentIndex == 0 && selected.parameter.t == 1.0f && std::abs( selected.distance - 2.0 ) < 0.000001, "selection clamps to the requested segment even when the neighboring segment is closer" );
    const auto tracked = getSplineRailFrame( rail, {}, body, selected.parameter, true );
    check( tracked.valid && tracked.parameter.segmentIndex == 1 && std::abs( tracked.distance - 3.0 ) < 0.000001, "normal connected tracking can leave the selected segment after reprojection" );
    for( float x : { -6.0f, 6.0f } )
    {
        world physics{};
        physics.SetGravity( {} );
        const transform2 poseA{ { 0.3f, -0.2f }, rot2::FromRadians( 0.4f ) };
        const auto a = physics.CreateBody( bodyType::Kinematic, poseA );
        const vec2 anchor{ 0.25f, -0.4f };
        const auto rotation = rot2::FromRadians( 0.7f );
        const auto b = physics.CreateBody( bodyType::Dynamic, { TransformPoint( poseA, {} ) - Rotate( rotation, anchor ), rotation } );
        ( void )physics.CreateShape( b, circle2{ { 0.1f, 0.2f }, 0.1f } );
        const vec2 points[] = { { -4, 0 }, { 0, 0 }, { 4, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.localAnchorB = anchor;
        definition.referenceDistance = 0.35;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "open endpoint selection fixture is created" );
        const transform2 pose{ TransformPoint( poseA, { x, 0.2f } ) - Rotate( rotation, anchor ), rotation };
        physics.SetBodyTransform( b, pose );
        physics.SetBodyLinearVelocity( b, { 3, 1 } );
        physics.SetBodyAngularVelocity( b, 0.4f );
        const std::size_t segment = x < 0.0f ? 0 : 1;
        check( physics.reprojectSplineRailJoint( id, segment ), "caller may select either physical open endpoint" );
        const auto data = physics.getSplineRailJointData( id );
        check( data.trackingValid && data.parameter.segmentIndex == segment && std::abs( data.currentTranslation - ( x + 4.0 - 0.35 ) ) < 0.00001 && Length( data.anchorB - TransformPoint( poseA, { x, 0.2f } ) ) < 0.00001f, "open endpoint extension is measured from the offset anchor in the rail owner's coordinates" );
        check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyTransform( b ).rotation.s == rotation.s && physics.GetBodyLinearVelocity( b ).x == 3.0f && physics.GetBodyAngularVelocity( b ) == 0.4f, "endpoint selection preserves body pose and velocity" );
    }
}

void checkSeam()
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
    check( physics.IsValid( id ), "selected seam fixture is created" );
    const double length = physics.getSplineRailJointData( id ).length;
    physics.SetBodyTransform( b, { points[2], {} } );
    check( physics.reprojectSplineRailJoint( id, 1 ), "selected half-circuit endpoint is projected" );
    check( std::abs( physics.getSplineRailJointData( id ).currentTranslation + length * 0.5 ) < 0.000001, "half-circuit tie still selects the smaller unwrapped coordinate" );
    physics.SetBodyTransform( b, { points[0], {} } );
    check( physics.reprojectSplineRailJoint( id, 3 ), "last closed segment may select the shared seam endpoint" );
    auto data = physics.getSplineRailJointData( id );
    check( data.trackingValid && data.parameter.segmentIndex == 3 && data.parameter.t == 1.0f && std::abs( data.currentTranslation + length ) < 0.000001, "selected last-segment seam retains the correct negative winding" );
    check( physics.reprojectSplineRailJoint( id, 0 ), "first segment may select the same seam position" );
    data = physics.getSplineRailJointData( id );
    check( data.parameter.segmentIndex == 0 && data.parameter.t == 0.0f && std::abs( data.currentTranslation + length ) < 0.000001, "switching seam representations does not add or remove a circuit" );
    physics.setSplineRailJointMotor( id, true, 2.0f, 1.0f );
    for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
    check( physics.getSplineRailJointData( id ).trackingValid && physics.getSplineRailJointData( id ).currentTranslation > -length + 3.5, "ordinary forward travel continues from the selected seam" );
}

void checkTurns()
{
    for( int substeps : { 1, 4, 8 } )
        for( float direction : { -1.0f, 1.0f } )
        {
            world physics{};
            physics.SetGravity( {} );
            std::array points{ vec2{ 2, 0 }, vec2{ 0, 2 }, vec2{ -2, 0 }, vec2{ 0, -2 } };
            const transform2 poseA{ { 0.3f, -0.2f }, rot2::FromRadians( 0.2f ) };
            const auto a = physics.CreateBody( bodyType::Kinematic, poseA );
            const auto b = physics.CreateBody( bodyType::Dynamic, { TransformPoint( poseA, points[0] ), poseA.rotation } );
            const auto shape = physics.CreateShape( b, circle2{ {}, 0.1f } );
            physics.SetBodySleepEnabled( b, false );
            physics.SetShapeFriction( shape, 0.42f );
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            definition.isLoop = true;
            definition.referenceDistance = 0.35;
            definition.rotationMode = substeps == 1 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
            definition.enableAngularSpring = substeps == 8;
            definition.targetTranslation = 2.0f;
            definition.enableLimit = true;
            definition.lowerTranslation = -100.0f;
            definition.upperTranslation = 100.0f;
            definition.enableMotor = true;
            definition.motorSpeed = direction * 2.0f;
            definition.maxMotorForce = 1.0f;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "multi-turn segment fixture is created" );
            const double initial = physics.getSplineRailJointData( id ).currentTranslation;
            for( int frame = 0; frame < 900; ++frame ) physics.Step( 1.0f / 60.0f, substeps );
            auto before = physics.getSplineRailJointData( id );
            check( direction * ( before.currentTranslation - initial ) > 2.0 * before.length, "selected reprojection follows multiple forward or reverse circuits" );
            if( substeps != 1 ) check( physics.rebaseSplineRailJoint( id ), "control origin may be reset before segment selection" );
            if( substeps == 8 )
            {
                points = { vec2{ 2.3f, 0 }, vec2{ 0, 2.1f }, vec2{ -2.4f, 0 }, vec2{ 0, -2 } };
                check( physics.setSplineRailJointPoints( id, points ), "closed edit keeps the prior accumulated distance" );
            }
            before = physics.getSplineRailJointData( id );
            const auto geometry = prepareSpline( { points, true } );
            check( geometry.has_value(), "selected winding distance reference is prepared" );
            const std::size_t segment = ( before.parameter.segmentIndex + 1 ) % points.size();
            const auto oldPhase = geometry->getDistance( before.parameter.segmentIndex, before.parameter.t, 1.0e-8 );
            const auto newPhase = geometry->getDistance( segment, 0.37f, 1.0e-8 );
            check( oldPhase.has_value() && newPhase.has_value(), "independent selected winding distances are available" );
            const double oldDistance = before.currentTranslation + before.referenceDistance;
            const double base = *newPhase + oldDistance - *oldPhase;
            const double lower = base + std::floor( ( oldDistance - base ) / before.length ) * before.length;
            const double expected = oldDistance - lower <= lower + before.length - oldDistance ? lower : lower + before.length;
            auto pose = physics.GetBodyTransform( b );
            pose.position = TransformPoint( poseA, geometry->evaluate( segment, 0.37f ) );
            physics.SetBodyTransform( b, pose );
            physics.SetBodyLinearVelocity( a, { 0.1f, -0.2f } );
            physics.SetBodyAngularVelocity( a, 0.1f );
            const auto velocity = physics.GetBodyLinearVelocity( b );
            const float spin = physics.GetBodyAngularVelocity( b ), mass = physics.GetBodyMass( b );
            check( physics.reprojectSplineRailJoint( id, segment ), "segment reprojection succeeds after turns, rebase and geometry edits" );
            const auto after = physics.getSplineRailJointData( id );
            check( after.trackingValid && after.parameter.segmentIndex == segment && std::abs( after.currentTranslation + after.referenceDistance - expected ) < 0.00001, "selected projection preserves edited phase offsets and chooses the closest winding" );
            check( after.referenceDistance == before.referenceDistance && after.referenceAngle == before.referenceAngle && after.targetTranslation == before.targetTranslation && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && after.enableLimit == before.enableLimit && after.enableSpring == before.enableSpring && after.rotationMode == before.rotationMode && after.enableAngularSpring == before.enableAngularSpring && after.angularHertz == before.angularHertz && after.motorSpeed == before.motorSpeed, "segment selection preserves origin, targets, limits and rotation settings" );
            check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyTransform( b ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( b ) == spin && physics.GetBodyMass( b ) == mass && physics.GetShapeFriction( shape ) == 0.42f && physics.IsValid( shape ) && physics.IsValid( id ) && physics.GetBodyAngularVelocity( a ) == 0.1f, "selected projection preserves both connected bodies and physical handles" );
            check( after.springForce == 0.0f && after.motorForce == 0.0f && LengthSquared( after.force ) == 0.0f && after.torque == 0.0f, "changed segment projection clears the old constraint cache" );
            for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, substeps );
            const auto resumed = physics.getSplineRailJointData( id );
            check( resumed.trackingValid && direction * ( resumed.currentTranslation - after.currentTranslation ) > 3.0 && std::abs( resumed.currentSpeed - direction * 2.0f ) < 0.04f, "motor tracking continues with a moving owner after segment selection" );
        }
}

void checkFailuresAndNoOp()
{
    world physics{};
    physics.SetGravity( { 2, 0 } );
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { { 2, 0 }, {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
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
    check( physics.IsValid( id ), "failed selection cache fixture is created" );
    for( int frame = 0; frame < 120; ++frame ) physics.Step( 1.0f / 60.0f, 4 );
    physics.SetBodyAwake( b, false );
    const auto before = physics.getSplineRailJointData( id );
    check( before.trackingValid && Length( before.force ) > 0.01f, "failed selected projection has a nonzero cached load to preserve" );
    const auto pose = physics.GetBodyTransform( b );
    const auto velocity = physics.GetBodyLinearVelocity( b );
    for( std::size_t segment : { std::size_t{ 1 }, std::size_t{ 4 }, std::numeric_limits<std::size_t>::max() } )
    {
        check( !physics.reprojectSplineRailJoint( id, segment ), "singular requested segment and out-of-range indices fail without global fallback" );
        const auto after = physics.getSplineRailJointData( id );
        check( after.trackingValid && after.parameter.segmentIndex == before.parameter.segmentIndex && after.parameter.t == before.parameter.t && after.currentTranslation == before.currentTranslation && after.referenceDistance == before.referenceDistance && after.targetTranslation == before.targetTranslation && after.lowerTranslation == before.lowerTranslation && after.upperTranslation == before.upperTranslation && after.force.x == before.force.x && after.force.y == before.force.y && after.torque == before.torque && !physics.IsBodyAwake( b ), "failed segment reprojection preserves branch, winding, control settings, cache and sleep atomically" );
        check( LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f, "failed segment selection does not change body state" );
    }
    check( physics.reprojectSplineRailJoint( id, 0 ), "unchanged selected segment succeeds" );
    const auto unchanged = physics.getSplineRailJointData( id );
    check( !physics.IsBodyAwake( b ) && unchanged.force.x == before.force.x && unchanged.force.y == before.force.y, "same selected projection retains sleeping state and warm-start reaction" );
    physics.SetBodyTransform( b, { points[2], {} } );
    physics.SetBodyAwake( b, false );
    check( physics.reprojectSplineRailJoint( id, 2 ) && physics.IsBodyAwake( b ), "changed segment projection wakes the connected dynamic body" );
}

}

int main()
{
    checkCrossing();
    checkEndpointSelection();
    checkSeam();
    checkTurns();
    checkFailuresAndNoOp();
    return 0;
}
