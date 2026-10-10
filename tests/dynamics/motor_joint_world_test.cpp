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
    const auto ground = simulation.CreateBody();
    const auto body = simulation.CreateBody( bodyType::Dynamic );
    ( void )simulation.CreateShape( body, circle2{ {}, 0.5f } );

    motorJointDef definition{};
    definition.bodyA = ground;
    definition.bodyB = body;
    definition.linearVelocity = { 2.0f, -1.0f };
    definition.maxVelocityForce = 1000.0f;
    definition.angularVelocity = 3.0f;
    definition.maxVelocityTorque = 1000.0f;
    const auto joint = simulation.createMotorJoint( definition );

    check( simulation.getJointCount() == 1 && simulation.GetBody( body ).jointCount == 1, "Motor shares common joint graph" );
    auto data = simulation.getMotorJointData( joint );
    check( near( data.linearVelocity.x, 2.0f ) && near( data.linearVelocity.y, -1.0f ) && near( data.angularVelocity, 3.0f ), "Motor create/query preserves targets" );

    simulation.Step( 1.0f / 60.0f, 1 );
    data = simulation.getMotorJointData( joint );
    check( IsFinite( data.force ) && std::isfinite( data.torque ), "Motor reaction query remains finite" );
    check( LengthSquared( data.force ) > 0.0f && std::abs( data.torque ) > 0.0f, "Motor stores linear and angular reaction caches" );

    const float torqueBeforeLinearChange = data.torque;
    simulation.SetBodyAwake( body, false );
    simulation.setMotorJointLinearVelocity( joint, { -1.0f, 0.5f }, 500.0f );
    check( simulation.IsBodyAwake( body ), "changing Motor linear settings wakes connected body" );
    data = simulation.getMotorJointData( joint );
    check( near( data.linearVelocity.x, -1.0f ) && near( data.maxVelocityForce, 500.0f ), "Motor linear setter updates query state" );
    check( LengthSquared( data.force ) == 0.0f && near( data.torque, torqueBeforeLinearChange ), "Motor linear setter clears only linear cache" );

    simulation.SetBodyAwake( body, false );
    simulation.setMotorJointLinearVelocity( joint, { -1.0f, 0.5f }, 500.0f );
    check( !simulation.IsBodyAwake( body ), "unchanged Motor linear settings do not wake body" );

    simulation.SetBodyAwake( body, true );
    simulation.Step( 1.0f / 60.0f, 1 );
    const auto rebuilt = simulation.getMotorJointData( joint );
    check( LengthSquared( rebuilt.force ) > 0.0f, "Motor rebuilds linear reaction after setter change" );

    const vec2 forceBeforeAngularChange = rebuilt.force;
    simulation.SetBodyAwake( body, false );
    simulation.setMotorJointAngularVelocity( joint, -2.0f, 700.0f );
    check( simulation.IsBodyAwake( body ), "changing Motor angular settings wakes connected body" );
    data = simulation.getMotorJointData( joint );
    check( near( data.angularVelocity, -2.0f ) && near( data.maxVelocityTorque, 700.0f ), "Motor angular setter updates query state" );
    check( data.torque == 0.0f && near( data.force.x, forceBeforeAngularChange.x ) && near( data.force.y, forceBeforeAngularChange.y ), "Motor angular setter clears only angular cache" );

    simulation.SetBodyAwake( body, false );
    simulation.setMotorJointAngularVelocity( joint, -2.0f, 700.0f );
    check( !simulation.IsBodyAwake( body ), "unchanged Motor angular settings do not wake body" );

    simulation.SetBodyTransform( body, { { 1.0f, 0.0f }, {} } );
    data = simulation.getMotorJointData( joint );
    check( LengthSquared( data.force ) == 0.0f && data.torque == 0.0f, "pose change clears Motor warm-start cache" );

    simulation.destroyJoint( joint );
    check( !simulation.IsValid( joint ) && simulation.getJointCount() == 0, "destroy invalidates Motor handle" );
    const auto reused = simulation.createMotorJoint( definition );
    check( simulation.IsValid( reused ) && !simulation.IsValid( joint ), "Motor slot reuse preserves generation validation" );
    simulation.DestroyBody( body );
    check( !simulation.IsValid( reused ) && simulation.GetBody( ground ).jointCount == 0, "body destruction removes connected Motor" );

    world pair;
    pair.SetGravity( {} );
    const auto a = pair.CreateBody( bodyType::Dynamic );
    const auto b = pair.CreateBody( bodyType::Dynamic );
    ( void )pair.CreateShape( a, circle2{ {}, 0.5f } );
    ( void )pair.CreateShape( b, circle2{ {}, 0.5f } );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 1, "overlapping bodies initially create a Contact" );

    definition = {};
    definition.bodyA = a;
    definition.bodyB = b;
    const auto pairJoint = pair.createMotorJoint( definition );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 0, "Motor connected collision suppression uses shared joint path" );

    pair.SetBodyAwake( a, false );
    check( !pair.IsBodyAwake( b ), "Motor connected dynamic bodies sleep together" );
    pair.setMotorJointLinearVelocity( pairJoint, { 1.0f, 0.0f }, 10.0f );
    check( pair.IsBodyAwake( a ) && pair.IsBodyAwake( b ), "Motor setter wake propagates through joint graph" );

    pair.destroyJoint( pairJoint );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 1, "destroying Motor restores candidate contacts" );

    definition.collideConnected = true;
    const auto colliding = pair.createMotorJoint( definition );
    pair.Step( 1.0f / 60.0f, 1 );
    check( pair.GetContactCount() == 1 && pair.getMotorJointData( colliding ).collideConnected, "explicit Motor connected collisions remain enabled" );

    world foreign;
    check( !foreign.IsValid( colliding ), "foreign World rejects Motor handle" );

    return EXIT_SUCCESS;
}
