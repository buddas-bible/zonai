#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string_view>

#include "demo.h"
#include "rigidBodyDemo.h"

using namespace zonai;
using namespace zonai::sandbox;

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
    check( getDemoEntries().size() == 11, "Motor Joint demo is independently selectable" );
    check( std::string_view{ getDemoEntry( demoKind::motorJointPlayground ).category } == "조인트", "Motor Joint demo stays in joint category" );

    rigidBodyDemo motor{ demoKind::motorJointPlayground };
    check( motor.getWorld().GetBodyCount() == 2 && motor.getWorld().getJointCount() == 1 && motor.getShapes().size() == 2, "Motor scene has reference and driven bodies" );
    check( motor.getWorld().IsValid( motor.getMotorJoint() ), "Motor scene owns a valid Motor Joint" );

    motor.applyMotorJointPreset( motorJointDemoPreset::brake );
    auto data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    check( LengthSquared( data.linearVelocity ) == 0.0f && data.angularVelocity == 0.0f && data.maxVelocityForce > 0.0f && data.maxVelocityTorque > 0.0f, "Motor brake preset uses zero targets with finite limits" );
    check( data.linearHertz == 0.0f && data.angularHertz == 0.0f, "Motor brake preset disables transform springs" );

    motor.applyMotorJointPreset( motorJointDemoPreset::linear );
    data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    check( data.linearVelocity.x > 0.0f && data.maxVelocityForce > 0.0f && data.angularVelocity == 0.0f && data.maxVelocityTorque == 0.0f, "Motor linear preset isolates relative linear velocity" );
    check( data.linearHertz == 0.0f && data.angularHertz == 0.0f, "Motor linear velocity preset leaves springs disabled" );

    motor.applyMotorJointPreset( motorJointDemoPreset::angular );
    data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    check( LengthSquared( data.linearVelocity ) == 0.0f && data.maxVelocityForce == 0.0f && data.angularVelocity > 0.0f && data.maxVelocityTorque > 0.0f, "Motor angular preset isolates relative angular velocity" );

    motor.applyMotorJointPreset( motorJointDemoPreset::combined );
    data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    check( LengthSquared( data.linearVelocity ) > 0.0f && data.maxVelocityForce > 0.0f && data.angularVelocity > 0.0f && data.maxVelocityTorque > 0.0f, "Motor combined preset enables both velocity channels" );

    motor.applyMotorJointPreset( motorJointDemoPreset::linearSpring );
    data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    check( data.maxVelocityForce == 0.0f && data.maxVelocityTorque == 0.0f && data.linearHertz > 0.0f && data.maxSpringForce > 0.0f && data.angularHertz == 0.0f, "Motor linear spring preset isolates anchor restoration" );

    motor.applyMotorJointPreset( motorJointDemoPreset::angularSpring );
    data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    check( data.maxVelocityForce == 0.0f && data.maxVelocityTorque == 0.0f && data.linearHertz == 0.0f && data.angularHertz > 0.0f && data.maxSpringTorque > 0.0f, "Motor angular spring preset isolates relative-angle restoration" );
    check( data.referenceAngle > 0.0f, "Motor angular spring preset exposes a visible target angle" );

    motor.applyMotorJointPreset( motorJointDemoPreset::springBoth );
    data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    check( data.linearHertz > 0.0f && data.angularHertz > 0.0f && data.maxVelocityForce == 0.0f && data.maxVelocityTorque == 0.0f, "Motor spring-both preset enables both transform springs only" );

    motor.applyMotorJointPreset( motorJointDemoPreset::velocityAndSpring );
    data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    check( data.maxVelocityForce > 0.0f && data.maxVelocityTorque > 0.0f && data.linearHertz > 0.0f && data.angularHertz > 0.0f, "Motor velocity-and-spring preset demonstrates actuator coexistence" );

    motor.setMotorJointLinearSettings( { -1.25f, 0.75f }, 12.0f );
    motor.setMotorJointAngularSettings( -2.5f, 8.0f );
    motor.setMotorJointLinearSpringSettings( 3.0f, 0.5f, 18.0f );
    motor.setMotorJointAngularSpringSettings( 0.25f, 4.0f, 0.6f, 9.0f );
    data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    check( near( data.linearVelocity.x, -1.25f ) && near( data.linearVelocity.y, 0.75f ) && near( data.maxVelocityForce, 12.0f ), "Motor inspector applies linear velocity target and force" );
    check( near( data.angularVelocity, -2.5f ) && near( data.maxVelocityTorque, 8.0f ), "Motor inspector applies angular velocity target and torque" );
    check( near( data.linearHertz, 3.0f ) && near( data.linearDampingRatio, 0.5f ) && near( data.maxSpringForce, 18.0f ), "Motor inspector applies linear spring settings" );
    check( near( data.referenceAngle, 0.25f ) && near( data.angularHertz, 4.0f ) && near( data.angularDampingRatio, 0.6f ) && near( data.maxSpringTorque, 9.0f ), "Motor inspector applies angular spring settings" );

    motor.applyMotorJointPreset( motorJointDemoPreset::linear );
    data = motor.getWorld().getMotorJointData( motor.getMotorJoint() );
    const bodyId driven = data.bodyB;
    motor.step( 1.0f / 60.0f, 1 );
    check( motor.getWorld().GetBodyLinearVelocity( driven ).x > 0.0f, "Motor demo visibly drives the target body" );

    return EXIT_SUCCESS;
}
