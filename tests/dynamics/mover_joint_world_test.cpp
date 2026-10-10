#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/world.h"

using namespace zonai;

namespace
{

void check( bool ok, const char* message )
{
    if( !ok )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}

bool near( float a, float b )
{
    return std::abs( a - b ) < 0.0001f;
}

}

int main()
{
    world simulation;
    simulation.SetGravity( {} );

    const bodyId ground = simulation.CreateBody();
    const bodyId driven = simulation.CreateBody( bodyType::Dynamic );
    ( void )simulation.CreateShape( driven, circle2{ {}, 0.5f } );
    simulation.SetBodyAngularVelocity( driven, 1.25f );

    moverJointDef definition{};
    definition.bodyA = ground;
    definition.bodyB = driven;
    definition.linearVelocity = { 2.0f, -1.0f };
    definition.maxVelocityForce = { 1000.0f, 1000.0f };

    const jointId mover = simulation.createMoverJoint( definition );
    auto data = simulation.getMoverJointData( mover );
    check( near( data.linearVelocity.x, 2.0f ) && near( data.linearVelocity.y, -1.0f ), "Mover create/query preserves target velocity" );
    check( near( data.maxVelocityForce.x, 1000.0f ) && near( data.maxVelocityForce.y, 1000.0f ), "Mover create/query preserves per-axis force limits" );
    check( simulation.getJointCount() == 1 && simulation.GetBody( driven ).jointCount == 1, "Mover shares the common joint graph" );

    simulation.Step( 1.0f / 60.0f, 1 );
    const vec2 velocity = simulation.GetBodyLinearVelocity( driven );
    check( near( velocity.x, 2.0f ) && near( velocity.y, -1.0f ), "World Mover reaches target relative linear velocity" );
    check( near( simulation.GetBodyAngularVelocity( driven ), 1.25f ), "World Mover does not affect angular velocity" );
    data = simulation.getMoverJointData( mover );
    check( IsFinite( data.force ) && LengthSquared( data.force ) > 0.0f, "Mover stores a finite linear reaction force" );

    // 목표속도 변경은 이전 velocity actuator 해를 버리고 연결된 Body를 깨움.
    simulation.SetBodyAwake( driven, false );
    simulation.setMoverJointLinearVelocity( mover, { -1.0f, 0.5f } );
    check( simulation.IsBodyAwake( driven ), "changing Mover target velocity wakes connected body" );
    data = simulation.getMoverJointData( mover );
    check( near( data.linearVelocity.x, -1.0f ) && near( data.linearVelocity.y, 0.5f ), "Mover velocity setter updates query state" );
    check( LengthSquared( data.force ) == 0.0f, "Mover velocity setter clears cached reaction" );

    simulation.SetBodyAwake( driven, false );
    simulation.setMoverJointLinearVelocity( mover, { -1.0f, 0.5f } );
    check( !simulation.IsBodyAwake( driven ), "unchanged Mover target velocity does not wake body" );

    simulation.SetBodyAwake( driven, true );
    simulation.setMoverJointLinearVelocity( mover, { 20.0f, -20.0f } );
    simulation.setMoverJointMaxVelocityForce( mover, { 6.0f, 12.0f } );
    simulation.Step( 1.0f / 60.0f, 1 );
    data = simulation.getMoverJointData( mover );
    check( std::abs( data.force.x ) <= 6.0001f && std::abs( data.force.y ) <= 12.0001f, "Mover clamps reaction independently on x and y" );

    simulation.SetBodyAwake( driven, false );
    simulation.setMoverJointMaxVelocityForce( mover, { 3.0f, 9.0f } );
    check( simulation.IsBodyAwake( driven ), "changing Mover force limits wakes connected body" );
    data = simulation.getMoverJointData( mover );
    check( near( data.maxVelocityForce.x, 3.0f ) && near( data.maxVelocityForce.y, 9.0f ), "Mover force setter updates query state" );
    check( LengthSquared( data.force ) == 0.0f, "Mover force setter clears cached reaction" );

    simulation.SetBodyAwake( driven, false );
    simulation.setMoverJointMaxVelocityForce( mover, { 3.0f, 9.0f } );
    check( !simulation.IsBodyAwake( driven ), "unchanged Mover force limits do not wake body" );

    simulation.SetBodyAwake( driven, true );
    simulation.Step( 1.0f / 60.0f, 1 );
    check( LengthSquared( simulation.getMoverJointData( mover ).force ) > 0.0f, "Mover reaction rebuilds after setter changes" );

    simulation.SetBodyTransform( driven, { { 1.0f, 0.0f }, {} } );
    check( LengthSquared( simulation.getMoverJointData( mover ).force ) == 0.0f, "pose change clears Mover warm-start cache" );

    simulation.destroyJoint( mover );
    check( !simulation.IsValid( mover ) && simulation.getJointCount() == 0, "destroy invalidates Mover handle" );
    const jointId reused = simulation.createMoverJoint( definition );
    check( simulation.IsValid( reused ) && !simulation.IsValid( mover ), "Mover slot reuse preserves generation validation" );
    simulation.DestroyBody( driven );
    check( !simulation.IsValid( reused ) && simulation.GetBody( ground ).jointCount == 0, "body destruction removes connected Mover" );

    world pair;
    pair.SetGravity( {} );
    const bodyId a = pair.CreateBody( bodyType::Dynamic );
    const bodyId b = pair.CreateBody( bodyType::Dynamic );
    ( void )pair.CreateShape( a, circle2{ {}, 0.5f } );
    ( void )pair.CreateShape( b, circle2{ {}, 0.5f } );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 1, "overlapping bodies initially create a Contact" );

    definition = {};
    definition.bodyA = a;
    definition.bodyB = b;
    definition.maxVelocityForce = { 10.0f, 10.0f };
    const jointId pairMover = pair.createMoverJoint( definition );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 0, "Mover connected collision suppression uses shared joint path" );

    pair.SetBodyAwake( a, false );
    check( !pair.IsBodyAwake( b ), "Mover connected dynamic bodies sleep together" );
    pair.setMoverJointLinearVelocity( pairMover, { 1.0f, 0.0f } );
    check( pair.IsBodyAwake( a ) && pair.IsBodyAwake( b ), "Mover setter wake propagates through joint graph" );

    pair.destroyJoint( pairMover );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 1, "destroying Mover restores candidate contacts" );

    definition.collideConnected = true;
    const jointId colliding = pair.createMoverJoint( definition );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 1 && pair.getMoverJointData( colliding ).collideConnected, "explicit Mover connected collisions remain enabled" );

    world foreign;
    check( !foreign.IsValid( colliding ), "foreign World rejects Mover handle" );

    // Pogo World API/lifecycle RED: 지면 anchor와 mover anchor 사이 길이를 pogo 축으로 재고 normal 방향 반력을 만듦.
    world pogoWorld;
    pogoWorld.SetGravity( {} );
    const bodyId pogoGround = pogoWorld.CreateBody();
    const bodyId pogoBody = pogoWorld.CreateBody( bodyType::Dynamic, { { 0.0f, 0.75f }, {} } );
    ( void )pogoWorld.CreateShape( pogoBody, circle2{ {}, 0.5f } );

    pogoJointDef pogoDef{};
    pogoDef.bodyA = pogoGround;
    pogoDef.bodyB = pogoBody;
    pogoDef.localAnchorA = {};
    pogoDef.localAnchorB = {};
    pogoDef.localPogoAxisB = { 0.0f, 1.0f };
    pogoDef.normal = { 0.0f, 1.0f };
    pogoDef.restLength = 1.0f;
    pogoDef.hertz = 4.0f;
    pogoDef.dampingRatio = 0.7f;
    pogoDef.maxTensionForce = 50.0f;
    pogoDef.maxCompressionForce = 1000.0f;
    pogoDef.impulse = 0.1f;
    pogoDef.velocity = -0.2f;

    const jointId pogo = pogoWorld.createPogoJoint( pogoDef );
    auto pogoData = pogoWorld.getPogoJointData( pogo );
    check( near( pogoData.restLength, 1.0f ) && near( pogoData.hertz, 4.0f ), "Pogo create/query preserves spring settings" );
    check( near( pogoData.impulse, 0.1f ) && near( pogoData.velocity, -0.2f ), "Pogo create accepts recreation state seed" );
    check( pogoWorld.getJointCount() == 1 && pogoWorld.GetBody( pogoBody ).jointCount == 1, "Pogo shares the common joint graph" );

    pogoWorld.Step( 1.0f / 60.0f, 1 );
    pogoData = pogoWorld.getPogoJointData( pogo );
    check( pogoWorld.GetBodyLinearVelocity( pogoBody ).y > 0.0f, "World Pogo pushes compressed mover along contact normal" );
    check( IsFinite( pogoData.force ) && pogoData.force.y > 0.0f, "Pogo reports a finite reaction force" );
    check( std::abs( pogoData.force.y ) <= 1000.0001f, "Pogo respects compression force limit" );

    pogoWorld.SetBodyAwake( pogoBody, false );
    pogoWorld.setPogoJointSpring( pogo, 1.25f, 5.0f, 0.8f );
    check( pogoWorld.IsBodyAwake( pogoBody ), "changing Pogo spring wakes connected body" );
    pogoData = pogoWorld.getPogoJointData( pogo );
    check( near( pogoData.restLength, 1.25f ) && near( pogoData.hertz, 5.0f ) && near( pogoData.dampingRatio, 0.8f ), "Pogo spring setter updates query state" );

    pogoWorld.SetBodyAwake( pogoBody, false );
    pogoWorld.setPogoJointSpring( pogo, 1.25f, 5.0f, 0.8f );
    check( !pogoWorld.IsBodyAwake( pogoBody ), "unchanged Pogo spring does not wake body" );

    pogoWorld.setPogoJointForceLimits( pogo, 0.0f, 25.0f );
    pogoData = pogoWorld.getPogoJointData( pogo );
    check( near( pogoData.maxTensionForce, 0.0f ) && near( pogoData.maxCompressionForce, 25.0f ), "Pogo force-limit setter updates query state" );

    pogoWorld.destroyJoint( pogo );
    check( !pogoWorld.IsValid( pogo ) && pogoWorld.getJointCount() == 0, "destroy invalidates Pogo handle" );
    const jointId pogoReused = pogoWorld.createPogoJoint( pogoDef );
    check( pogoWorld.IsValid( pogoReused ) && !pogoWorld.IsValid( pogo ), "Pogo slot reuse preserves generation validation" );
    pogoWorld.DestroyBody( pogoBody );
    check( !pogoWorld.IsValid( pogoReused ) && pogoWorld.GetBody( pogoGround ).jointCount == 0, "body destruction removes connected Pogo" );

    return EXIT_SUCCESS;
}
