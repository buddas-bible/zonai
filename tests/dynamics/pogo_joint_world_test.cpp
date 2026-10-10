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

bool near( float a, float b, float epsilon = 0.0001f )
{
    return std::abs( a - b ) < epsilon;
}

}

int main()
{
    constexpr float h = 1.0f / 60.0f;

    world simulation;
    simulation.SetGravity( {} );

    const bodyId ground = simulation.CreateBody();
    const bodyId driven = simulation.CreateBody( bodyType::Dynamic, { { 0.0f, 0.75f }, {} } );
    ( void )simulation.CreateShape( driven, circle2{ {}, 0.5f } );

    pogoJointDef definition{};
    definition.bodyA = ground;
    definition.bodyB = driven;
    definition.localAnchorA = {};
    definition.localAnchorB = {};
    definition.localPogoAxisB = { 0.0f, 2.0f }; // 생성 시 단위 벡터로 정규화함.
    definition.normal = { 0.0f, 3.0f };
    definition.restLength = 1.0f;
    definition.hertz = 4.0f;
    definition.dampingRatio = 0.7f;
    definition.maxTensionForce = 50.0f;
    definition.maxCompressionForce = 1000.0f;
    definition.impulse = 0.1f;
    definition.velocity = -0.2f;

    const jointId pogo = simulation.createPogoJoint( definition );
    auto data = simulation.getPogoJointData( pogo );
    check( near( data.restLength, 1.0f ) && near( data.hertz, 4.0f ) && near( data.dampingRatio, 0.7f ), "Pogo create/query preserves spring settings" );
    check( near( Length( data.pogoAxis ), 1.0f ) && near( Length( data.normal ), 1.0f ), "Pogo normalizes its measurement axis and contact normal" );
    check( near( data.impulse, 0.1f ) && near( data.velocity, -0.2f ), "Pogo create accepts recreation state seed" );
    check( simulation.getJointCount() == 1 && simulation.GetBody( driven ).jointCount == 1, "Pogo shares the common joint graph" );

    const float beforeCorrectionY = simulation.GetBodyTransform( driven ).position.y;
    simulation.Step( h, 1 );
    data = simulation.getPogoJointData( pogo );
    // 순수 위치 오차가 만든 Pogo 속도는 integration에만 쓰고 relaxation에서 제거함.
    check( simulation.GetBodyTransform( driven ).position.y > beforeCorrectionY, "World Pogo integrates compressed position upward along the contact normal" );
    check( near( simulation.GetBodyLinearVelocity( driven ).y, 0.0f, 0.001f ), "Pogo relaxation removes pure position-correction velocity" );
    check( IsFinite( data.force ) && std::abs( data.force.y ) <= 1000.0001f, "Pogo reports a finite reaction within the compression limit" );

    // Spring 설정 변경은 내부 상태를 이어가되 연결된 component를 깨워 새 설정을 즉시 반영함.
    simulation.SetBodyAwake( driven, false );
    simulation.setPogoJointSpring( pogo, 1.25f, 5.0f, 0.8f );
    check( simulation.IsBodyAwake( driven ), "changing Pogo spring wakes connected body" );
    data = simulation.getPogoJointData( pogo );
    check( near( data.restLength, 1.25f ) && near( data.hertz, 5.0f ) && near( data.dampingRatio, 0.8f ), "Pogo spring setter updates query state" );

    simulation.SetBodyAwake( driven, false );
    simulation.setPogoJointSpring( pogo, 1.25f, 5.0f, 0.8f );
    check( !simulation.IsBodyAwake( driven ), "unchanged Pogo spring does not wake body" );

    simulation.setPogoJointForceLimits( pogo, 0.0f, 25.0f );
    data = simulation.getPogoJointData( pogo );
    check( near( data.maxTensionForce, 0.0f ) && near( data.maxCompressionForce, 25.0f ), "Pogo force-limit setter updates query state" );
    simulation.SetBodyLinearVelocity( driven, { 0.0f, -20.0f } );
    simulation.Step( h, 1 );
    data = simulation.getPogoJointData( pogo );
    check( data.force.y >= -0.0001f && data.force.y <= 25.0001f, "Compression-only Pogo cannot pull downward and clamps upward force" );

    // Pose 변경은 이전 작용점에 해당하던 warm-start 상태를 폐기함.
    simulation.SetBodyTransform( driven, { { 0.0f, 0.75f }, {} } );
    data = simulation.getPogoJointData( pogo );
    check( near( data.impulse, 0.0f ) && near( data.velocity, 0.0f ), "pose change clears Pogo solver state" );

    simulation.destroyJoint( pogo );
    check( !simulation.IsValid( pogo ) && simulation.getJointCount() == 0, "destroy invalidates Pogo handle" );
    const jointId reused = simulation.createPogoJoint( definition );
    check( simulation.IsValid( reused ) && !simulation.IsValid( pogo ), "Pogo slot reuse preserves generation validation" );
    simulation.DestroyBody( driven );
    check( !simulation.IsValid( reused ) && simulation.GetBody( ground ).jointCount == 0, "body destruction removes connected Pogo" );

    // Pogo도 공용 Joint 연결을 사용하므로 collision suppression, sleep component, wake propagation을 그대로 따름.
    world pair;
    pair.SetGravity( {} );
    const bodyId a = pair.CreateBody( bodyType::Dynamic );
    const bodyId b = pair.CreateBody( bodyType::Dynamic );
    ( void )pair.CreateShape( a, circle2{ {}, 0.5f } );
    ( void )pair.CreateShape( b, circle2{ {}, 0.5f } );
    pair.Step( h, 1 );
    check( pair.GetContactCount() == 1, "overlapping bodies initially create a Contact" );

    pogoJointDef pairDefinition{};
    pairDefinition.bodyA = a;
    pairDefinition.bodyB = b;
    pairDefinition.hertz = 4.0f;
    pairDefinition.maxCompressionForce = 100.0f;
    pairDefinition.maxTensionForce = 100.0f;
    const jointId pairPogo = pair.createPogoJoint( pairDefinition );
    pair.Step( h, 1 );
    check( pair.GetContactCount() == 0, "Pogo connected collision suppression uses shared joint path" );

    pair.SetBodyAwake( a, false );
    check( !pair.IsBodyAwake( b ), "Pogo connected dynamic bodies sleep together" );
    pair.setPogoJointSpring( pairPogo, 0.25f, 5.0f, 0.7f );
    check( pair.IsBodyAwake( a ) && pair.IsBodyAwake( b ), "Pogo setter wake propagates through joint graph" );

    pair.destroyJoint( pairPogo );
    pair.Step( h, 1 );
    check( pair.GetContactCount() == 1, "destroying Pogo restores candidate contacts" );

    pairDefinition.collideConnected = true;
    const jointId colliding = pair.createPogoJoint( pairDefinition );
    pair.Step( h, 1 );
    check( pair.GetContactCount() == 1 && pair.getPogoJointData( colliding ).collideConnected, "explicit Pogo connected collisions remain enabled" );

    world foreign;
    check( !foreign.IsValid( colliding ), "foreign World rejects Pogo handle" );

    return EXIT_SUCCESS;
}
