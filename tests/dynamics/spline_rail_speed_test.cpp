#include <cmath>
#include <cstdio>
#include <cstdlib>
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

void checkStraight()
{
    world physics{};
    physics.SetGravity( {} );
    const auto a = physics.CreateBody( bodyType::Kinematic );
    const auto b = physics.CreateBody( bodyType::Dynamic );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { -4, 0 }, { 4, 0 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    definition.enableMotor = true;
    definition.motorSpeed = 9.0f;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "straight speed fixture is created" );
    physics.SetBodyLinearVelocity( a, { 2, 3 } );
    physics.SetBodyLinearVelocity( b, { 5, -4 } );
    auto data = physics.getSplineRailJointData( id );
    check( data.trackingValid && std::abs( data.currentSpeed - 3.0f ) < 1.0e-6f, "straight speed is relative tangent speed, independent of normal motion and motor target" );
    physics.SetBodyLinearVelocity( b, { -2, 3 } );
    data = physics.getSplineRailJointData( id );
    check( std::abs( data.currentSpeed + 4.0f ) < 1.0e-6f, "reverse rail speed has negative sign" );
}
transform2 advance( const world& physics, bodyId id, transform2 pose, float time )
{
    const vec2 center = physics.GetBodyLocalCenter( id );
    const vec2 position = TransformPoint( pose, center ) + time * physics.GetBodyLinearVelocity( id );
    pose.rotation = rot2::FromRadians( time * physics.GetBodyAngularVelocity( id ) ) * pose.rotation;
    pose.position = position - Rotate( pose.rotation, center );
    return pose;
}

void checkDerivative( world& physics, jointId id )
{
    const auto before = physics.getSplineRailJointData( id );
    check( before.trackingValid, "distance derivative starts with valid tracking" );
    const auto a = before.bodyA, b = before.bodyB;
    const auto poseA = physics.GetBodyTransform( a ), poseB = physics.GetBodyTransform( b );
    constexpr float step = 0.001f;
    physics.SetBodyTransform( a, advance( physics, a, poseA, step ) );
    physics.SetBodyTransform( b, advance( physics, b, poseB, step ) );
    const auto plus = physics.getSplineRailJointData( id );
    physics.SetBodyTransform( a, advance( physics, a, poseA, -step ) );
    physics.SetBodyTransform( b, advance( physics, b, poseB, -step ) );
    const auto minus = physics.getSplineRailJointData( id );
    physics.SetBodyTransform( a, poseA );
    physics.SetBodyTransform( b, poseB );
    const double difference = ( plus.currentTranslation - minus.currentTranslation ) / ( 2.0 * step );
    check( plus.trackingValid && minus.trackingValid && std::abs( before.currentSpeed - difference ) < 0.003, "reported speed matches independent accumulated-distance differentiation" );
}

void checkOpenEndpoints()
{
    for( float x : { -5.0f, 5.0f } )
    {
        world physics{};
        const auto a = physics.CreateBody( bodyType::Kinematic );
        const auto b = physics.CreateBody( bodyType::Dynamic, { { x, 0.3f }, {} } );
        ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
        const vec2 points[] = { { -4, 0 }, { 4, 0 } };
        splineRailJointDef definition{};
        definition.bodyA = a;
        definition.bodyB = b;
        definition.points = points;
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "open endpoint fixture is created" );
        physics.SetBodyLinearVelocity( a, { 1, -2 } );
        physics.SetBodyAngularVelocity( a, 0.7f );
        physics.SetBodyLinearVelocity( b, { -2, 1 } );
        checkDerivative( physics, id );
    }
}

void checkCurveAndMovingOwner()
{
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
        const auto id = physics.createSplineRailJoint( definition );
        check( physics.IsValid( id ), "curved speed fixture is created" );
        physics.SetBodyLinearVelocity( a, { 0.7f, -0.4f } );
        physics.SetBodyAngularVelocity( a, 0.8f );
        physics.SetBodyLinearVelocity( b, { 2.1f, -0.3f } );
        physics.SetBodyAngularVelocity( b, -1.2f );
        checkDerivative( physics, id );
    }
}

jointId createLoop( world& physics, float direction )
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
    for( int substeps : { 1, 4, 8 } )
        for( float direction : { -1.0f, 1.0f } )
        {
            world physics{}, reference{};
            const auto id = createLoop( physics, direction );
            const auto referenceId = createLoop( reference, direction );
            auto previous = physics.getSplineRailJointData( id );
            int seams = 0;
            for( int frame = 0; frame < 900; ++frame )
            {
                physics.Step( 1.0f / 60.0f, substeps );
                reference.Step( 1.0f / 60.0f, substeps );
                const auto data = physics.getSplineRailJointData( id );
                check( data.trackingValid && std::isfinite( data.currentSpeed ), "closed speed stays valid while rail moves and rotates" );
                if( frame > 10 )
                {
                    check( std::abs( data.currentSpeed - direction * 2.0f ) < 0.04f, "actual speed follows motor through forward and reverse seams" );
                    check( std::abs( ( data.currentTranslation - previous.currentTranslation ) * 60.0 - data.currentSpeed ) < 0.08, "closed speed agrees with step distance without a winding jump" );
                }
                if( ( direction > 0.0f && previous.parameter.segmentIndex == 3 && data.parameter.segmentIndex == 0 ) ||
                    ( direction < 0.0f && previous.parameter.segmentIndex == 0 && data.parameter.segmentIndex == 3 ) ) ++seams;
                for( int query = 0; query < 5; ++query )
                {
                    const auto repeated = physics.getSplineRailJointData( id );
                    check( repeated.currentSpeed == data.currentSpeed && repeated.currentTranslation == data.currentTranslation && repeated.parameter.segmentIndex == data.parameter.segmentIndex && repeated.parameter.t == data.parameter.t && repeated.motorForce == data.motorForce, "repeated speed queries preserve distance, branch and cached motor reaction" );
                }
                previous = data;
            }
            const auto expected = reference.getSplineRailJointData( referenceId );
            check( seams >= 2 && direction * previous.currentTranslation > 2.0 * previous.length, "actual speed is checked over multiple forward or reverse circuits" );
            check( previous.currentTranslation == expected.currentTranslation && previous.currentSpeed == expected.currentSpeed && previous.force.x == expected.force.x && previous.force.y == expected.force.y && previous.torque == expected.torque, "query frequency does not change future rail motion or reactions" );
            for( auto bodies : { std::pair{ previous.bodyA, expected.bodyA }, std::pair{ previous.bodyB, expected.bodyB } } )
            {
                const auto actualPose = physics.GetBodyTransform( bodies.first ), expectedPose = reference.GetBodyTransform( bodies.second );
                check( LengthSquared( actualPose.position - expectedPose.position ) == 0.0f && actualPose.rotation.c == expectedPose.rotation.c && actualPose.rotation.s == expectedPose.rotation.s && LengthSquared( physics.GetBodyLinearVelocity( bodies.first ) - reference.GetBodyLinearVelocity( bodies.second ) ) == 0.0f && physics.GetBodyAngularVelocity( bodies.first ) == reference.GetBodyAngularVelocity( bodies.second ), "speed queries preserve body poses and velocities exactly" );
            }
        }
}

void checkInvalidAndSleeping()
{
    world physics{};
    physics.SetGravity( {} );
    const auto a = physics.CreateBody();
    const auto b = physics.CreateBody( bodyType::Dynamic, { { 0, 2 }, {} } );
    ( void )physics.CreateShape( b, circle2{ {}, 0.1f } );
    const vec2 points[] = { { -2, 0 }, { 0, 2 }, { 2, 0 } };
    splineRailJointDef definition{};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.points = points;
    const auto id = physics.createSplineRailJoint( definition );
    check( physics.IsValid( id ), "invalid tracking fixture starts valid" );
    physics.SetBodyAwake( b, false );
    const auto before = physics.getSplineRailJointData( id );
    for( int query = 0; query < 10; ++query )
    {
        const auto data = physics.getSplineRailJointData( id );
        check( data.trackingValid && data.currentSpeed == 0.0f && data.currentTranslation == before.currentTranslation && !physics.IsBodyAwake( b ), "sleeping rail query returns zero without waking or changing distance" );
    }
    const auto curvature = Curvature( { points }, before.parameter.segmentIndex, before.parameter.t );
    check( curvature.has_value() && std::abs( *curvature ) > 0.1, "singular projection fixture has nonzero curvature" );
    physics.SetBodyTransform( b, { before.anchorA + static_cast<float>( 1.0 / *curvature ) * before.normal, {} } );
    physics.SetBodyLinearVelocity( b, { 3, 0 } );
    const auto failed = physics.getSplineRailJointData( id );
    check( !failed.trackingValid && failed.currentSpeed == 0.0f, "failed curve tracking reports zero speed instead of an unusable tangent projection" );
}

}

int main()
{
    checkStraight();
    checkCurveAndMovingOwner();
    checkOpenEndpoints();
    checkSeamsAndQueries();
    checkInvalidAndSleeping();
    return 0;
}
