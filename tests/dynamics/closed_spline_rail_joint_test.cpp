#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
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

constexpr std::array points{ vec2{ 2, 0 }, vec2{ 0, 2 }, vec2{ -2, 0 }, vec2{ 0, -2 } };

void checkTurns()
{
    for( int substeps : { 1, 4, 8 } )
        for( int rotation : { 0, 1, 2 } )
        for( float direction : { 1.0f, -1.0f } )
        {
            world physics{};
            physics.SetGravity( {} );
            const auto a = physics.CreateBody();
            const auto b = physics.CreateBody( bodyType::Dynamic, { points[0], {} } );
            ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
            physics.SetBodySleepEnabled( b, false );
            splineRailJointDef definition{};
            definition.bodyA = a;
            definition.bodyB = b;
            definition.points = points;
            definition.isLoop = true;
            definition.rotationMode = rotation == 0 ? splineRailRotationMode::fixed : splineRailRotationMode::followTangent;
            definition.enableAngularSpring = rotation == 2;
            definition.enableMotor = true;
            definition.motorSpeed = direction * 2.0f;
            definition.maxMotorForce = 100.0f;
            const auto id = physics.createSplineRailJoint( definition );
            check( physics.IsValid( id ), "closed rail is created" );
            const double length = physics.getSplineRailJointData( id ).length;
            double previous = physics.getSplineRailJointData( id ).currentTranslation;
            for( int frame = 0; frame < 800; ++frame )
            {
                physics.Step( 1.0f / 60.0f, substeps );
                const auto data = physics.getSplineRailJointData( id );
                check( data.trackingValid && std::abs( data.lateralError ) < 0.025f && std::abs( data.currentAngle ) < ( rotation == 2 ? 0.3f : 0.025f ) && IsFinite( data.force ) && std::isfinite( data.torque ), "closed motor rail remains tracked and finite" );
                check( std::abs( data.currentTranslation - previous ) < 0.1, "seam crossings never jump accumulated distance by one circuit" );
                for( int query = 0; query < 3; ++query ) check( physics.getSplineRailJointData( id ).currentTranslation == data.currentTranslation, "repeated rail queries do not accumulate distance" );
                previous = data.currentTranslation;
            }
            check( direction * previous > 2.0 * length, "closed rail moves through more than two circuits in either direction without endpoint stops" );
            check( std::abs( previous - direction * 2.0 * 800.0 / 60.0 ) < 0.1, "multi-turn distance agrees with motor travel without repeated solver accumulation" );
            check( physics.getSplineRailJointData( id ).isLoop, "snapshot retains closed mode" );
        }
}

void checkSeamState()
{
    splineRailJointSim2 rail{};
    rail.isLoop = true;
    rail.referenceDistance = 0.25;
    check( setSplineRailGeometry( rail, points ), "closed solver fixture builds" );
    bodySim a{}, b{};
    b.invMass = b.invInertia = 1.0f;
    b.transform.position = rail.geometry.evaluate( 0, 0.01f );
    b.center = b.transform.position;
    rail.parameter = { 3, 0.99f };
    rail.trackingValid = true;
    rail.distanceOffset = 3.0 * rail.length;
    const auto expectedPhase = rail.geometry.getDistance( 0, 0.01f, 1.0e-7 );
    check( expectedPhase.has_value(), "seam reference distance is available" );
    const auto frame = getSplineRailFrame( rail, a.transform, b.transform, rail.parameter, true, rail.distanceOffset );
    check( frame.valid && frame.parameter.segmentIndex == 0 && std::abs( frame.distance - 4.0 * rail.length - *expectedPhase ) < 1.0e-6, "forward seam crossing preserves the positive winding offset" );
    auto constraint = prepareSplineRailJointConstraint( rail, a, b, 1.0f / 60.0f );
    bodyState sa{}, sb{};
    for( int pass = 0; pass < 12; ++pass )
    {
        solveSplineRailJointConstraint( constraint, sa, sb, pass % 2 == 0 );
        check( constraint.trackingValid && constraint.parameter.segmentIndex == 0 && constraint.distanceOffset == 4.0 * rail.length, "repeated solver passes count a seam only once" );
    }
    rail.parameter = { 0, 0.01f };
    rail.distanceOffset = -2.0 * rail.length;
    b.transform.position = rail.geometry.evaluate( 3, 0.99f );
    const auto reverse = getSplineRailFrame( rail, a.transform, b.transform, rail.parameter, true, rail.distanceOffset );
    check( reverse.valid && reverse.parameter.segmentIndex == 3 && reverse.distance < -2.0 * rail.length && reverse.distanceOffset == -3.0 * rail.length, "reverse seam crossing preserves negative accumulated distance" );
    const transform2 pose{ { 3, -2 }, rot2::FromRadians( 0.6f ) };
    const auto rotated = getSplineRailFrame( rail, pose, { TransformPoint( pose, b.transform.position ), pose.rotation }, rail.parameter, true, rail.distanceOffset );
    check( rotated.valid && std::abs( rotated.distance - reverse.distance ) < 1.0e-5 && Length( rotated.tangent - Rotate( pose.rotation, reverse.tangent ) ) < 1.0e-5f, "moving and rotating the rail frame preserves accumulated distance" );
    const vec2 crossing[] = { { -2, 0 }, { 0, 0 }, { 2, 2 }, { 2, -2 }, { 0, 0 }, { -2, 2 } };
    check( setSplineRailGeometry( rail, crossing ), "closed crossing fixture is regular" );
    const auto branch = getSplineRailFrame( rail, {}, { { 0, 0 }, {} }, { 3, 0.9f }, true, 0.0 );
    check( branch.valid && branch.parameter.segmentIndex >= 3, "closed crossing projection stays on the connected later branch" );
}

void checkLimitsAndSpring()
{
    for( int substeps : { 1, 4, 8 } )
    {
        world physics{};
        physics.SetGravity( {} );
        const auto a = physics.CreateBody();
        const auto b = physics.CreateBody( bodyType::Dynamic, { points[0], {} } );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        physics.SetBodySleepEnabled( b, false );
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        definition.isLoop = true;
        definition.referenceDistance = 0.3;
        definition.enableMotor = true;
        definition.motorSpeed = 2.0f;
        definition.maxMotorForce = 1.0f;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "closed limit fixture builds" );
        const auto data = [&] { return physics.getSplineRailJointData( id ); };
        const auto advance = [&]( int frames ) { for( int i = 0; i < frames; ++i ) physics.Step( 1.0f / 60.0f, substeps ); };
        const double length = data().length;
        const float bound = static_cast<float>( 2.0 * length + 0.25 );
        physics.setSplineRailJointLimit( id, true, -bound, bound );
        advance( 1300 );
        check( std::abs( data().currentTranslation - bound ) < 0.03 && Length( physics.GetBodyLinearVelocity( b ) ) < 0.03f, "upper limit stops at the specified positive accumulated distance after multiple turns" );
        physics.setSplineRailJointMotor( id, true, -2.0f, 1.0f );
        advance( 2000 );
        check( std::abs( data().currentTranslation + bound ) < 0.03 && Length( physics.GetBodyLinearVelocity( b ) ) < 0.03f, "lower limit stops at the specified negative accumulated distance after reversal" );
        const float locked = -bound - 0.1f;
        physics.setSplineRailJointLimit( id, true, locked, locked );
        advance( 240 );
        check( std::abs( data().currentTranslation - locked ) < 0.03, "equal closed limits lock an accumulated coordinate beyond the first circuit" );
        physics.setSplineRailJointLimit( id, false, locked, locked );
        physics.setSplineRailJointMotor( id, false, 0.0f, 1.0f );
        const float target = static_cast<float>( -length + 0.35 );
        physics.setSplineRailJointSpring( id, true, 0.15f, 1.0f, target );
        advance( 1800 );
        check( data().trackingValid && std::abs( data().currentTranslation - target ) < 0.03 && Length( physics.GetBodyLinearVelocity( b ) ) < 0.03f, "spring restores the specified accumulated target rather than its nearest wrapped equivalent" );
        const auto before = data();
        physics.setSplineRailJointLimit( id, true, 3.0f, 2.0f );
        check( !data().enableLimit && data().currentTranslation == before.currentTranslation, "invalid limit input leaves accumulated state unchanged" );
        physics.setSplineRailJointLimit( id, true, static_cast<float>( 3.0 * length ), static_cast<float>( 4.0 * length ) );
        check( data().enableLimit && data().lowerTranslation > length * 2.9, "closed limits need not intersect the first geometric circuit" );
    }
}

void checkEditing()
{
    world physics{};
    physics.SetGravity( {} );
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { points[0], {} } );
    const auto shape = physics.CreateShape( b, circle2{ {}, 0.1f } );
    physics.SetBodySleepEnabled( b, false );
    physics.SetShapeFriction( shape, 0.42f );
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.isLoop = true;
    definition.referenceDistance = 0.4;
    definition.enableMotor = true;
    definition.motorSpeed = 2.0f;
    definition.maxMotorForce = 100.0f;
    definition.rotationMode = splineRailRotationMode::followTangent;
    definition.enableAngularSpring = false;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "closed edit fixture builds" );
    for( int i = 0; i < 800; ++i ) physics.Step( 1.0f / 60.0f, 4 );
    const auto accumulated = physics.getSplineRailJointData( id ).currentTranslation;
    physics.SetShapeDensity( shape, 2.0f );
    check( physics.getSplineRailJointData( id ).currentTranslation == accumulated, "mass changes clear impulses without resetting closed distance" );
    const auto before = physics.getSplineRailJointData( id );
    check( before.currentTranslation > 2.0 * before.length, "edit starts after multiple circuits" );
    const auto pose = physics.GetBodyTransform( b );
    const auto velocity = physics.GetBodyLinearVelocity( b );
    const auto spin = physics.GetBodyAngularVelocity( b );
    const auto mass = physics.GetBodyMass( b );
    const std::array edited{ vec2{ 2.5f, 0 }, vec2{ 1.75f, 1.75f }, vec2{ 0, 2.5f }, vec2{ -1.75f, 1.75f }, vec2{ -2.5f, 0 }, vec2{ -1.75f, -1.75f }, vec2{ 0, -2.5f }, vec2{ 1.75f, -1.75f } };
    check( physics.setSplineRailJointPoints( id, edited ), "closed geometry edit with a changed circuit length is accepted" );
    const auto after = physics.getSplineRailJointData( id );
    check( std::abs( after.currentTranslation - before.currentTranslation ) < 1.0e-9 && after.length > before.length && after.referenceDistance == before.referenceDistance && after.motorSpeed == before.motorSpeed && !after.enableAngularSpring, "edit preserves accumulated distance and tuning while realigning the geometric phase" );
    check( physics.IsValid( id ) && physics.IsValid( shape ) && LengthSquared( physics.GetBodyTransform( b ).position - pose.position ) == 0.0f && physics.GetBodyTransform( b ).rotation.s == pose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( b ) - velocity ) == 0.0f && physics.GetBodyAngularVelocity( b ) == spin && physics.GetBodyMass( b ) == mass && physics.GetShapeFriction( shape ) == 0.42f, "edit preserves handles physical pose velocity mass and material" );
    std::array<vec2, 32> singular{};
    for( std::size_t i = 0; i < singular.size(); ++i )
    {
        const float angle = 2.0f * std::numbers::pi_v<float> * static_cast<float>( i ) / static_cast<float>( singular.size() );
        singular[i] = pose.position + 2.0f * vec2{ std::cos( angle ), std::sin( angle ) };
    }
    splineRailJointSim2 candidate{};
    candidate.isLoop = true;
    check( setSplineRailGeometry( candidate, singular ), "projection failure fixture has valid regular closed geometry" );
    check( !physics.setSplineRailJointPoints( id, singular ) && physics.getSplineRailJointPoints( id ).size() == edited.size() && physics.getSplineRailJointData( id ).currentTranslation == after.currentTranslation, "singular reprojection rejects a valid geometry without changing accumulated state" );
    const std::array invalid{ vec2{ 0, 0 }, vec2{ 1, 0 } };
    check( !physics.setSplineRailJointPoints( id, invalid ) && physics.getSplineRailJointPoints( id ).size() == edited.size() && physics.getSplineRailJointData( id ).currentTranslation == after.currentTranslation, "failed closed edit preserves geometry and accumulated state atomically" );
    physics.SetBodyAwake( b, false );
    check( physics.setSplineRailJointPoints( id, edited ) && !physics.IsBodyAwake( b ), "unchanged closed geometry does not wake or reset accumulated state" );
    const auto asleep = physics.getSplineRailJointData( id ).currentTranslation;
    auto moved = edited;
    moved[1].y += 0.05f;
    check( physics.setSplineRailJointPoints( id, moved ) && physics.IsBodyAwake( b ) && std::abs( physics.getSplineRailJointData( id ).currentTranslation - asleep ) < 1.0e-9, "changed geometry wakes the slider without resetting accumulated distance" );
    physics.setSplineRailJointMotor( id, true, -1.0f, 20.0f );
    physics.setSplineRailJointSpring( id, true, 0.0f, 0.7f, 2.0f );
    physics.setSplineRailJointLimit( id, true, -100.0f, 100.0f );
    physics.setSplineRailJointRotation( id, splineRailRotationMode::fixed, 4.0f, 0.5f, false );
    check( std::abs( physics.getSplineRailJointData( id ).currentTranslation - asleep ) < 1.0e-9, "cache invalidating setters preserve accumulated distance" );
    for( int i = 0; i < 900; ++i ) physics.Step( 1.0f / 60.0f, 4 );
    const auto reversed = physics.getSplineRailJointData( id );
    check( reversed.trackingValid && reversed.currentTranslation < asleep - 14.5 && reversed.currentTranslation > asleep - 15.5, "post-edit reverse travel remains continuous across the new seam" );
    physics.DestroyBody( b );
    check( !physics.IsValid( id ), "destroying a closed slider removes its joint" );
}

void checkDynamicOwner()
{
    world physics{};
    physics.SetGravity( {} );
    const auto a = physics.CreateBody( bodyType::Dynamic );
    const auto b = physics.CreateBody( bodyType::Dynamic, { points[0], {} } );
    ( void )physics.CreateShape( a, circle2{ {}, 0.2f } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    physics.SetBodySleepEnabled( a, false );
    physics.SetBodySleepEnabled( b, false );
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.isLoop = true;
    definition.enableMotor = true;
    definition.motorSpeed = 2.0f;
    definition.maxMotorForce = 1.0f;
    definition.rotationMode = splineRailRotationMode::followTangent;
    definition.enableAngularSpring = false;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "dynamic closed rail fixture builds" );
    const double length = physics.getSplineRailJointData( id ).length;
    for( int frame = 0; frame < 900; ++frame )
    {
        physics.Step( 1.0f / 60.0f, 4 );
        const auto data = physics.getSplineRailJointData( id );
        const auto momentum = physics.GetBodyMass( a ) * physics.GetBodyLinearVelocity( a ) + physics.GetBodyMass( b ) * physics.GetBodyLinearVelocity( b );
        check( data.trackingValid && std::abs( data.lateralError ) < 0.025f && std::abs( data.currentAngle ) < 0.025f && Length( momentum ) < 1.0e-4f, "moving dynamic owner retains closed tracking and equal opposite linear reactions" );
    }
    check( physics.getSplineRailJointData( id ).currentTranslation > 2.0 * length && Length( physics.GetBodyTransform( a ).position ) > 0.01f && std::abs( physics.GetBodyTransform( a ).rotation.s ) > 0.01f, "closed rail completes multiple circuits while its owner moves and rotates" );
}
}

int main()
{
    checkSeamState();
    checkTurns();
    checkLimitsAndSpring();
    checkEditing();
    checkDynamicOwner();
}
