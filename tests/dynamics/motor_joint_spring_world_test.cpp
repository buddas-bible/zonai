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
    // Stage 1 public aggregate의 기존 positional field 순서를 Stage 2가 바꾸면 안 됨.
    const motorJointDef aggregateCompatibility{ {}, {}, {}, {}, {}, 0.0f, 0.0f, 0.0f, true };
    check( aggregateCompatibility.collideConnected, "Motor aggregate initialization preserves collideConnected position" );
    check( aggregateCompatibility.linearHertz == 0.0f && aggregateCompatibility.angularHertz == 0.0f, "Motor appended spring fields default to disabled" );

    world simulation;
    simulation.SetGravity( {} );
    const bodyId ground = simulation.CreateBody();
    const bodyId driven = simulation.CreateBody( bodyType::Dynamic, { { 2.0f, 0.0f }, rot2::FromRadians( 0.5f ) } );
    ( void )simulation.CreateShape( driven, MakeBox( { 0.5f, 0.5f } ) );

    motorJointDef definition{};
    definition.bodyA = ground;
    definition.bodyB = driven;
    definition.referenceAngle = 0.0f;
    definition.linearHertz = 4.0f;
    definition.linearDampingRatio = 0.7f;
    definition.maxSpringForce = 1000.0f;
    definition.angularHertz = 4.0f;
    definition.angularDampingRatio = 0.7f;
    definition.maxSpringTorque = 1000.0f;

    const jointId motor = simulation.createMotorJoint( definition );
    auto data = simulation.getMotorJointData( motor );
    check( near( data.referenceAngle, 0.0f ) && near( data.linearHertz, 4.0f ) && near( data.angularHertz, 4.0f ), "Motor create/query preserves transform spring settings" );

    simulation.Step( 1.0f / 60.0f, 1 );
    check( simulation.GetBodyLinearVelocity( driven ).x < 0.0f, "World linear Motor spring pulls body toward target anchor" );
    check( simulation.GetBodyAngularVelocity( driven ) < 0.0f, "World angular Motor spring rotates body toward reference angle" );
    data = simulation.getMotorJointData( motor );
    check( LengthSquared( data.force ) > 0.0f && std::abs( data.torque ) > 0.0f, "Motor reaction includes transform spring impulses" );

    const float torqueBeforeLinearChange = data.torque;
    simulation.SetBodyAwake( driven, false );
    simulation.setMotorJointLinearSpring( motor, 2.0f, 0.5f, 250.0f );
    check( simulation.IsBodyAwake( driven ), "changing Motor linear spring wakes connected body" );
    data = simulation.getMotorJointData( motor );
    check( near( data.linearHertz, 2.0f ) && near( data.linearDampingRatio, 0.5f ) && near( data.maxSpringForce, 250.0f ), "Motor linear spring setter updates query state" );
    check( LengthSquared( data.force ) == 0.0f && near( data.torque, torqueBeforeLinearChange ), "Motor linear spring setter clears only linear spring cache" );

    simulation.SetBodyAwake( driven, false );
    simulation.setMotorJointLinearSpring( motor, 2.0f, 0.5f, 250.0f );
    check( !simulation.IsBodyAwake( driven ), "unchanged Motor linear spring settings do not wake body" );

    simulation.SetBodyAwake( driven, true );
    simulation.Step( 1.0f / 60.0f, 1 );
    const vec2 forceBeforeAngularChange = simulation.getMotorJointData( motor ).force;

    simulation.SetBodyAwake( driven, false );
    simulation.setMotorJointAngularSpring( motor, 0.2f, 3.0f, 0.6f, 300.0f );
    check( simulation.IsBodyAwake( driven ), "changing Motor angular spring wakes connected body" );
    data = simulation.getMotorJointData( motor );
    check( near( data.referenceAngle, 0.2f ) && near( data.angularHertz, 3.0f ) && near( data.angularDampingRatio, 0.6f ) && near( data.maxSpringTorque, 300.0f ), "Motor angular spring setter updates query state" );
    check( data.torque == 0.0f && near( data.force.x, forceBeforeAngularChange.x ) && near( data.force.y, forceBeforeAngularChange.y ), "Motor angular spring setter clears only angular spring cache" );

    simulation.SetBodyAwake( driven, false );
    simulation.setMotorJointAngularSpring( motor, 0.2f, 3.0f, 0.6f, 300.0f );
    check( !simulation.IsBodyAwake( driven ), "unchanged Motor angular spring settings do not wake body" );

    simulation.SetBodyAwake( driven, true );
    simulation.Step( 1.0f / 60.0f, 1 );
    data = simulation.getMotorJointData( motor );
    check( LengthSquared( data.force ) > 0.0f || std::abs( data.torque ) > 0.0f, "Motor spring reactions rebuild after tuning changes" );

    simulation.SetBodyTransform( driven, { { 1.0f, 0.0f }, rot2::FromRadians( 0.35f ) } );
    data = simulation.getMotorJointData( motor );
    check( LengthSquared( data.force ) == 0.0f && data.torque == 0.0f, "pose change clears velocity and spring Motor caches" );

    return EXIT_SUCCESS;
}
