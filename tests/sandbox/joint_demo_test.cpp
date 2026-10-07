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
    check( getDemoEntries().size() == 6, "mouse joint demo is independently selectable" );
    check( std::string_view{ getDemoEntry( demoKind::mouseJointPlayground ).category } == "조인트", "mouse joint demo stays in joint category" );

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

    return EXIT_SUCCESS;
}
