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

void check( bool condition, const char* message )
{
    if( !condition )
    {
        std::fprintf( stderr, "%s\n", message );
        std::exit( EXIT_FAILURE );
    }
}

}

int main()
{
    check( getDemoEntries().size() == 7, "Prismatic demo is independently selectable" );
    check( std::string_view{ getDemoEntry( demoKind::mouseJointPlayground ).category } == "조인트", "mouse joint demo stays in joint category" );
    check( std::string_view{ getDemoEntry( demoKind::prismaticRail ).category } == "조인트", "Prismatic rail stays in joint category" );

    rigidBodyDemo mouse{ demoKind::mouseJointPlayground };
    check( mouse.getWorld().GetBodyCount() == 4 && mouse.getWorld().getJointCount() == 0 && mouse.getShapes().size() == 4, "mouse scene has ground and three draggable bodies" );
    const auto shapes = mouse.getShapes();
    const float lightMass = mouse.getWorld().GetBodyMass( shapes[1].bodyHandle );
    const float mediumMass = mouse.getWorld().GetBodyMass( shapes[2].bodyHandle );
    const float heavyMass = mouse.getWorld().GetBodyMass( shapes[3].bodyHandle );
    check( lightMass < mediumMass && mediumMass < heavyMass, "mouse scene exposes visibly different masses" );

    demoInput input{};
    input.mousePressed = true;
    input.mouseHeld = true;
    input.mousePosition = mouse.getWorld().GetBodyTransform( shapes[1].bodyHandle ).position;
    mouse.handleInput( input );
    check( mouse.getWorld().IsValid( mouse.getMouseJoint() ), "dedicated mouse scene starts drag" );
    mouse.setMouseSettings( 8.0f, 1.0f, 250.0f );
    const auto mouseData = mouse.getWorld().getMouseJointData( mouse.getMouseJoint() );
    check( mouseData.bodyB == shapes[1].bodyHandle && mouseData.hertz == 8.0f && mouseData.dampingRatio == 1.0f && mouseData.maxForce == 250.0f, "mouse tuning updates active drag" );
    input.mousePressed = false;
    input.mouseHeld = false;
    mouse.handleInput( input );
    check( mouse.getWorld().getJointCount() == 0, "mouse release destroys transient joint" );

    rigidBodyDemo distance{ demoKind::distancePendulum };
    distance.applyDistancePreset( distanceDemoPreset::spring );
    auto distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( distanceData.enableSpring && !distanceData.enableLimit && !distanceData.enableMotor && distanceData.hertz == 2.0f, "distance spring preset" );
    distance.applyDistancePreset( distanceDemoPreset::limit );
    distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( distanceData.enableSpring && distanceData.hertz == 0.0f && distanceData.enableLimit && !distanceData.enableMotor, "distance limit preset" );
    distance.applyDistancePreset( distanceDemoPreset::motor );
    distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( distanceData.enableSpring && distanceData.hertz == 0.0f && !distanceData.enableLimit && distanceData.enableMotor, "distance motor preset" );
    distance.applyDistancePreset( distanceDemoPreset::rigid );
    distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( !distanceData.enableSpring && !distanceData.enableLimit && !distanceData.enableMotor, "distance rigid preset" );

    rigidBodyDemo revolute{ demoKind::revoluteHinge };
    revolute.applyRevolutePreset( revoluteDemoPreset::limit );
    auto revoluteData = revolute.getWorld().getRevoluteJointData( revolute.getRevoluteJoint() );
    check( revoluteData.enableLimit && !revoluteData.enableMotor, "revolute limit preset" );
    revolute.applyRevolutePreset( revoluteDemoPreset::motor );
    revoluteData = revolute.getWorld().getRevoluteJointData( revolute.getRevoluteJoint() );
    check( !revoluteData.enableLimit && revoluteData.enableMotor, "revolute motor preset" );
    revolute.applyRevolutePreset( revoluteDemoPreset::motorLimit );
    revoluteData = revolute.getWorld().getRevoluteJointData( revolute.getRevoluteJoint() );
    check( revoluteData.enableLimit && revoluteData.enableMotor, "revolute combined preset" );
    revolute.applyRevolutePreset( revoluteDemoPreset::free );
    revoluteData = revolute.getWorld().getRevoluteJointData( revolute.getRevoluteJoint() );
    check( !revoluteData.enableLimit && !revoluteData.enableMotor, "revolute free preset" );

    rigidBodyDemo wheel{ demoKind::wheelSuspension };
    wheel.applyWheelPreset( wheelDemoPreset::limit );
    auto wheelData = wheel.getWorld().getWheelJointData( wheel.getWheelJoint() );
    check( !wheelData.enableSpring && wheelData.enableLimit && !wheelData.enableMotor, "wheel limit preset" );
    wheel.applyWheelPreset( wheelDemoPreset::motor );
    wheelData = wheel.getWorld().getWheelJointData( wheel.getWheelJoint() );
    check( wheelData.enableSpring && !wheelData.enableLimit && wheelData.enableMotor, "wheel motor preset" );
    wheel.applyWheelPreset( wheelDemoPreset::combined );
    wheelData = wheel.getWorld().getWheelJointData( wheel.getWheelJoint() );
    check( wheelData.enableSpring && wheelData.enableLimit && wheelData.enableMotor, "wheel combined preset" );
    wheel.applyWheelPreset( wheelDemoPreset::spring );
    wheelData = wheel.getWorld().getWheelJointData( wheel.getWheelJoint() );
    check( wheelData.enableSpring && !wheelData.enableLimit && !wheelData.enableMotor, "wheel spring preset" );

    rigidBodyDemo prismatic{ demoKind::prismaticRail };
    check( prismatic.getWorld().GetBodyCount() == 2 && prismatic.getWorld().getJointCount() == 1 && prismatic.getShapes().size() == 2, "Prismatic scene has rail reference and one slider" );
    const jointId prismaticId = prismatic.getPrismaticJoint();
    check( prismatic.getWorld().IsValid( prismaticId ), "Prismatic scene owns a valid rail joint" );
    auto prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( std::abs( prismaticData.lateralError ) < 0.00001f && std::abs( prismaticData.currentAngle ) < 0.00001f, "Prismatic slider starts on its rail" );

    // Inspector는 preset이 아닌 임의 값을 실시간으로 적용할 수 있어야 함.
    prismatic.setPrismaticSpringSettings( true, 5.0f, 1.2f, 1.25f );
    prismatic.setPrismaticLimitSettings( true, -1.25f, 1.75f );
    prismatic.setPrismaticMotorSettings( true, -3.0f, 40.0f );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( prismaticData.enableSpring && prismaticData.hertz == 5.0f && prismaticData.dampingRatio == 1.2f && prismaticData.targetTranslation == 1.25f, "Prismatic inspector applies spring values" );
    check( prismaticData.enableLimit && prismaticData.lowerTranslation == -1.25f && prismaticData.upperTranslation == 1.75f, "Prismatic inspector applies limit values" );
    check( prismaticData.enableMotor && prismaticData.motorSpeed == -3.0f && prismaticData.maxMotorForce == 40.0f, "Prismatic inspector applies motor values" );

    const bodyId inspectorSlider = prismaticData.bodyB;
    prismatic.getWorld().SetBodyLinearVelocity( inspectorSlider, { 2.5f, 0.0f } );
    check( std::abs( prismatic.getPrismaticCurrentSpeed() - 2.5f ) < 0.0001f, "Prismatic inspector reports axial relative speed" );

    transform2 inspectorTransform = prismatic.getWorld().GetBodyTransform( inspectorSlider );
    inspectorTransform.position = { 0.8f, 0.0f };
    prismatic.getWorld().SetBodyTransform( inspectorSlider, inspectorTransform );
    prismatic.setPrismaticSpringTargetToCurrent();
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( std::abs( prismaticData.targetTranslation - prismaticData.currentTranslation ) < 0.0001f, "Prismatic inspector can copy current translation to spring target" );

    prismatic.applyPrismaticPreset( prismaticDemoPreset::limited );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( !prismaticData.enableSpring && prismaticData.enableLimit && prismaticData.lowerTranslation == -2.0f && prismaticData.upperTranslation == 2.0f && !prismaticData.enableMotor, "Prismatic limited preset isolates the translation range" );
    prismatic.applyPrismaticPreset( prismaticDemoPreset::locked );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( !prismaticData.enableSpring && prismaticData.enableLimit && prismaticData.lowerTranslation == 0.0f && prismaticData.upperTranslation == 0.0f && !prismaticData.enableMotor, "Prismatic locked preset uses equal limits without motor" );
    prismatic.applyPrismaticPreset( prismaticDemoPreset::free );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( !prismaticData.enableSpring && !prismaticData.enableLimit && !prismaticData.enableMotor, "Prismatic free preset disables active axial controls" );

    prismatic.applyPrismaticPreset( prismaticDemoPreset::motorForward );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( !prismaticData.enableSpring && !prismaticData.enableLimit && prismaticData.enableMotor && prismaticData.motorSpeed == 2.0f && prismaticData.maxMotorForce == 20.0f, "Prismatic forward motor preset" );
    prismatic.applyPrismaticPreset( prismaticDemoPreset::motorReverse );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( !prismaticData.enableSpring && !prismaticData.enableLimit && prismaticData.enableMotor && prismaticData.motorSpeed == -2.0f && prismaticData.maxMotorForce == 20.0f, "Prismatic reverse motor preset" );
    prismatic.applyPrismaticPreset( prismaticDemoPreset::brake );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( !prismaticData.enableSpring && !prismaticData.enableLimit && prismaticData.enableMotor && prismaticData.motorSpeed == 0.0f && prismaticData.maxMotorForce == 20.0f, "Prismatic brake preset uses zero target speed" );
    prismatic.applyPrismaticPreset( prismaticDemoPreset::motorLimit );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( !prismaticData.enableSpring && prismaticData.enableLimit && prismaticData.lowerTranslation == -2.0f && prismaticData.upperTranslation == 2.0f && prismaticData.enableMotor && prismaticData.motorSpeed == 2.0f, "Prismatic motor and limit preset combines axial constraints" );

    prismatic.applyPrismaticPreset( prismaticDemoPreset::spring );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( prismaticData.enableSpring && prismaticData.hertz == 2.0f && prismaticData.dampingRatio == 0.7f && prismaticData.targetTranslation == 1.0f && !prismaticData.enableLimit && !prismaticData.enableMotor, "Prismatic spring preset isolates target translation" );
    prismatic.applyPrismaticPreset( prismaticDemoPreset::springLimit );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( prismaticData.enableSpring && prismaticData.targetTranslation == 3.0f && prismaticData.enableLimit && prismaticData.lowerTranslation == -2.0f && prismaticData.upperTranslation == 2.0f && !prismaticData.enableMotor, "Prismatic spring and limit preset lets the limit oppose an outside target" );
    prismatic.applyPrismaticPreset( prismaticDemoPreset::combined );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( prismaticData.enableSpring && prismaticData.targetTranslation == 3.0f && prismaticData.enableLimit && prismaticData.enableMotor && prismaticData.motorSpeed == 2.0f, "Prismatic combined preset enables spring motor and limit" );

    prismatic.applyPrismaticPreset( prismaticDemoPreset::free );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( !prismaticData.enableSpring && !prismaticData.enableLimit && !prismaticData.enableMotor, "Prismatic free preset clears every axial mode after combined preset" );
    const bodyId slider = prismaticData.bodyB;
    const float initialTranslation = prismaticData.currentTranslation;
    prismatic.getWorld().ApplyLinearImpulseToCenter( slider, { 2.0f, 0.0f } );
    for( int i = 0; i < 30; ++i ) prismatic.step( 1.0f / 60.0f, 4 );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( prismaticData.currentTranslation > initialTranslation + 0.2f, "Prismatic demo visibly leaves axial translation free" );

    prismatic.getWorld().ApplyLinearImpulseToCenter( slider, { 0.0f, 4.0f } );
    prismatic.getWorld().ApplyAngularImpulse( slider, 2.0f );
    for( int i = 0; i < 60; ++i ) prismatic.step( 1.0f / 60.0f, 4 );
    prismaticData = prismatic.getWorld().getPrismaticJointData( prismaticId );
    check( std::abs( prismaticData.lateralError ) < 0.01f && std::abs( prismaticData.currentAngle ) < 0.01f, "Prismatic demo blocks lateral translation and relative rotation" );

    return EXIT_SUCCESS;
}
