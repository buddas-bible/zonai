#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/world.h"

using namespace zonai;
namespace
{
void check( bool ok, const char* message ) { if( !ok ) { std::fprintf( stderr, "%s\n", message ); std::exit( EXIT_FAILURE ); } }
}
int main()
{
    world simulation; simulation.SetGravity( {} );
    const auto ground = simulation.CreateBody();
    const auto body = simulation.CreateBody( bodyType::Dynamic );
    const auto shape = simulation.CreateShape( body, circle2{ {}, 0.5f } );
    check( simulation.testShapePoint( shape, { 0.4f, 0.0f } ) && !simulation.testShapePoint( shape, { 0.49f, 0.49f } ), "circle picking excludes AABB corner" );
    mouseJointDef definition{}; definition.bodyA = ground; definition.bodyB = body; definition.target = { 0.0f, 0.3f }; definition.maxForce = 100.0f;
    const auto joint = simulation.createMouseJoint( definition );
    const auto before = simulation.GetBodyTransform( body );
    simulation.setMouseJointTarget( joint, { 2.0f, 0.3f } );
    check( simulation.GetBodyTransform( body ).position.x == before.position.x, "target setter never teleports body" );
    simulation.Step( 1.0f / 60.0f, 4 );
    check( simulation.GetBodyTransform( body ).position.x > 0.0f && simulation.GetBodyAngularVelocity( body ) < 0.0f, "off-center World drag translates and rotates" );
    check( Length( simulation.getMouseJointData( joint ).force ) <= 100.001f, "World force budget" );
    for( int i = 0; i < 240; ++i ) { simulation.Step( 1.0f / 60.0f, 4 ); }
    check( Length( simulation.getMouseJointData( joint ).anchorB - simulation.getMouseJointData( joint ).target ) < 0.03f, "grabbed point converges to target" );
    simulation.SetBodyAwake( body, false ); simulation.setMouseJointTarget( joint, { 1.0f, 1.0f } );
    check( simulation.IsBodyAwake( body ), "new target wakes dragged component" );
    simulation.Step( 1.0f / 60.0f, 4 );
    check( LengthSquared( simulation.getMouseJointData( joint ).force ) > 0.0f, "cache exists before mass reset" );
    simulation.SetShapeDensity( shape, 2.0f );
    check( LengthSquared( simulation.getMouseJointData( joint ).force ) == 0.0f, "mass change clears mouse cache" );
    simulation.Step( 1.0f / 60.0f, 4 );
    check( LengthSquared( simulation.getMouseJointData( joint ).force ) > 0.0f, "cache exists before pose reset" );
    simulation.SetBodyTransform( body, { {}, {} } );
    check( LengthSquared( simulation.getMouseJointData( joint ).force ) == 0.0f, "pose change clears mouse cache" );
    simulation.setMouseJointTuning( joint, 5.0f, 0.7f, 0.0f );
    check( LengthSquared( simulation.getMouseJointData( joint ).force ) == 0.0f, "tuning clears cached force" );
    simulation.destroyJoint( joint ); check( !simulation.IsValid( joint ), "mouse joint destroy invalidates handle" );
    const auto reused = simulation.createMouseJoint( definition );
    check( simulation.IsValid( reused ) && !simulation.IsValid( joint ), "slot reuse rejects stale mouse handle" );
    const auto distanceBody = simulation.CreateBody( bodyType::Dynamic, { { 0.0f, -1.0f }, {} } );
    (void)simulation.CreateShape( distanceBody, circle2{ {}, 0.1f } );
    distanceJointDef distance{}; distance.bodyA = body; distance.bodyB = distanceBody;
    const auto distanceJoint = simulation.createDistanceJoint( distance );
    simulation.Step( 1.0f / 60.0f, 4 );
    check( simulation.IsValid( distanceJoint ) && IsFinite( simulation.GetBodyTransform( distanceBody ).position ), "mixed mouse and distance solver graph" );
    simulation.DestroyBody( body );
    check( !simulation.IsValid( reused ) && !simulation.IsValid( distanceJoint ) && simulation.getJointCount() == 0, "body cascade removes mixed joint kinds" );
    world other; check( !other.IsValid( reused ), "foreign World rejects mouse ID" );
    const auto box = simulation.CreateBody( bodyType::Dynamic, { { 4.0f, 1.0f }, rot2::FromRadians( 0.5f ) } );
    const auto boxShape = simulation.CreateShape( box, MakeBox( { 1.0f, 0.2f } ) );
    check( simulation.testShapePoint( boxShape, TransformPoint( simulation.GetBodyTransform( box ), { 0.9f, 0.1f } ) ), "rotated polygon local picking" );
    check( !simulation.testShapePoint( boxShape, TransformPoint( simulation.GetBodyTransform( box ), { 0.0f, 0.3f } ) ), "polygon rejects exterior" );
    const auto capsule = simulation.CreateShape( box, capsule2{ { -0.5f, 0.0f }, { 0.5f, 0.0f }, 0.2f } );
    check( simulation.testShapePoint( capsule, TransformPoint( simulation.GetBodyTransform( box ), { 0.65f, 0.0f } ) ), "capsule cap picking" );
    auto rounded = MakeBox( { 1.0f, 0.2f } ); rounded.radius = 0.1f;
    const auto roundedShape = simulation.CreateShape( box, rounded );
    check( simulation.testShapePoint( roundedShape, TransformPoint( simulation.GetBodyTransform( box ), { 1.08f, 0.2f } ) ), "rounded edge picking" );
    check( !simulation.testShapePoint( roundedShape, TransformPoint( simulation.GetBodyTransform( box ), { 1.09f, 0.29f } ) ), "rounded corner requires circle distance" );
    const auto segment = simulation.CreateShape( box, segment2{ { -1.0f, 0.0f }, { 1.0f, 0.0f } } );
    check( !simulation.testShapePoint( segment, simulation.GetBodyTransform( box ).position ), "zero-area segment does not pick" );
    world graph; graph.SetGravity( {} );
    const auto staticBody = graph.CreateBody();
    const auto first = graph.CreateBody( bodyType::Dynamic );
    const auto second = graph.CreateBody( bodyType::Dynamic, { { 3.0f, 0.0f }, {} } );
    (void)graph.CreateShape( first, circle2{ {}, 0.2f } ); (void)graph.CreateShape( second, circle2{ {}, 0.2f } );
    mouseJointDef mouse{}; mouse.bodyA = staticBody; mouse.bodyB = first;
    graph.SetBodyAwake( second, false ); const auto active = graph.createMouseJoint( mouse );
    check( !graph.IsBodyAwake( second ), "shared Static does not propagate mouse wake" );
    graph.setMouseJointTarget( active, { 1.0f, 0.0f } );
    for( int i = 0; i < 64; ++i )
    {
        if( i % 2 == 0 ) { (void)graph.createMouseJoint( mouse ); }
        else { distanceJointDef d{}; d.bodyA = staticBody; d.bodyB = first; (void)graph.createDistanceJoint( d ); }
    }
    graph.Step( 1.0f / 60.0f, 4 );
    check( graph.IsValid( active ) && graph.getJointCount() == 65, "mixed storage growth keeps handle and type" );
    graph.DestroyBody( staticBody );
    check( graph.getJointCount() == 0 && !graph.IsValid( active ), "Static deletion cascades mixed joints" );
    return EXIT_SUCCESS;
}
