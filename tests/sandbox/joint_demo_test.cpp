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
    check( std::string_view{ getDemoEntry( demoKind::mouseJointPlayground ).category } == "조인트", "mouse joint demo stays in joint category" );
    check( std::string_view{ getDemoEntry( demoKind::prismaticRail ).category } == "조인트", "Prismatic rail stays in joint category" );
    check( std::string_view{ getDemoEntry( demoKind::weldPair ).category } == "조인트", "Weld pair stays in joint category" );
    check( std::string_view{ getDemoEntry( demoKind::ragdoll ).category } == "연결 장치", "Ragdoll stays in assembly category" );

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
    check( mouse.getWorld().GetBodyCount() == 4, "mouse release removes its temporary reference body" );
    rigidBodyDemo freeWeld{ demoKind::weldPair };
    demoInput emptyPick{};
    emptyPick.mousePressed = emptyPick.mouseHeld = true;
    emptyPick.mousePosition = { 100.0f, 100.0f };
    freeWeld.handleInput( emptyPick );
    check( freeWeld.getWorld().GetBodyCount() == 2, "empty pick must not allocate a mouse reference body" );
    for( int trial = 0; trial < 3; ++trial )
    {
        demoInput drag{};
        drag.mousePressed = drag.mouseHeld = true;
        drag.mousePosition = freeWeld.getWorld().GetBodyTransform( freeWeld.getImpulseBody() ).position;
        freeWeld.handleInput( drag );
        check( freeWeld.getWorld().IsValid( freeWeld.getMouseJoint() ) && freeWeld.getWorld().GetBodyCount() == 3, "drag must work without scene static bodies" );
        freeWeld.cancelInput();
        check( freeWeld.getWorld().GetBodyCount() == 2 && freeWeld.getWorld().getJointCount() == 1, "cancel must remove only transient mouse objects" );
    }
    for( int index : { 1, 2, 3 } )
    {
        rigidBodyDemo lifting{ demoKind::mouseJointPlayground };
        lifting.setMouseSettings( 5.0f, 0.7f, 10.0f );
        const auto body = lifting.getShapes()[index].bodyHandle;
        const auto start = lifting.getWorld().GetBodyTransform( body ).position;
        demoInput drag{};
        drag.mousePressed = drag.mouseHeld = true;
        drag.mousePosition = start;
        lifting.handleInput( drag );
        drag.mousePressed = false;
        drag.mousePosition = start + vec2{ 0.0f, 2.0f };
        lifting.handleInput( drag );
        for( int frame = 0; frame < 60; ++frame )
        {
            lifting.step( 1.0f / 60.0f, 4 );
            check( Length( lifting.getWorld().getMouseJointData( lifting.getMouseJoint() ).force ) <= 10.01f, "lifting force must respect the 10N budget" );
        }
        const float height = lifting.getWorld().GetBodyTransform( body ).position.y;
        check( index == 1 ? height > start.y + 1.0f : height < start.y, "10N must lift the light body but fail against heavier bodies' weight" );
        lifting.getWorld().DestroyBody( body );
        lifting.handleInput( drag );
        check( !lifting.getWorld().IsValid( lifting.getMouseJoint() ) && lifting.getWorld().GetBodyCount() == 3, "deleting dragged body must also clean up the reference body" );
    }

    rigidBodyDemo distance{ demoKind::distancePendulum };
    distance.applyDistancePreset( distanceDemoPreset::spring );
    auto distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( distanceData.enableSpring && !distanceData.enableLimit && !distanceData.enableMotor && distanceData.hertz == 2.0f, "distance spring preset" );
    distance.applyDistancePreset( distanceDemoPreset::limit );
    distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( distanceData.enableSpring && distanceData.hertz == 0.0f && distanceData.enableLimit && !distanceData.enableMotor, "distance limit preset" );
    distance.applyDistancePreset( distanceDemoPreset::motor );
    distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( distanceData.enableSpring && distanceData.hertz == 0.0f && distanceData.enableLimit && distanceData.enableMotor, "distance motor preset stays bounded" );
    for( int frame = 0; frame < 600; ++frame ) distance.step( 1.0f / 60.0f, 4 );
    distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( std::abs( distanceData.currentLength - distanceData.maxLength ) < 0.02f, "extension stops at upper limit" );
    distance.getWorld().setDistanceJointMotor( distance.getPendulumJoint(), true, -1.0f, 10.0f );
    for( int frame = 0; frame < 180; ++frame ) distance.step( 1.0f / 60.0f, 4 );
    distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( std::abs( distanceData.currentLength - distanceData.minLength ) < 0.02f, "winding stops at lower limit" );
    distance.getWorld().setDistanceJointMotor( distance.getPendulumJoint(), true, 1.0f, 10.0f );
    for( int frame = 0; frame < 20; ++frame ) distance.step( 1.0f / 60.0f, 4 );
    const float brakeLength = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() ).currentLength;
    distance.getWorld().setDistanceJointMotor( distance.getPendulumJoint(), true, 0.0f, 10.0f );
    for( int frame = 0; frame < 120; ++frame ) distance.step( 1.0f / 60.0f, 4 );
    check( std::abs( distance.getWorld().getDistanceJointData( distance.getPendulumJoint() ).currentLength - brakeLength ) < 0.02f, "brake holds length between the limits" );
    distance.applyDistancePreset( distanceDemoPreset::rigid );
    distanceData = distance.getWorld().getDistanceJointData( distance.getPendulumJoint() );
    check( !distanceData.enableSpring && !distanceData.enableLimit && !distanceData.enableMotor, "distance rigid preset" );
    distance.getWorld().setDistanceJointLength( distance.getPendulumJoint(), 1.0f );
    for( int frame = 0; frame < 120; ++frame ) distance.step( 1.0f / 60.0f, 4 );
    check( std::abs( distance.getWorld().getDistanceJointData( distance.getPendulumJoint() ).currentLength - 1.0f ) < 0.01f, "changing target length wakes and moves the pendulum" );

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
    for( const int substeps : { 1, 4, 8 } )
    {
        rigidBodyDemo hinge{ demoKind::revoluteHinge };
        hinge.testRevoluteLimit();
        bool hitLimit = false;
        bool returnedInside = false;
        for( int frame = 0; frame < 240; ++frame )
        {
            hinge.step( 1.0f / 60.0f, substeps );
            const auto joint = hinge.getWorld().getRevoluteJointData( hinge.getRevoluteJoint() );
            hitLimit |= joint.currentAngle >= joint.upperAngle - 0.02f;
            returnedInside |= hitLimit && joint.currentAngle < joint.upperAngle - 0.1f;
            check( joint.enableLimit && !joint.enableMotor && joint.currentAngle <= joint.upperAngle + 0.03f && joint.currentAngle >= joint.lowerAngle - 0.03f, "limit trial must stay inside the angular bounds" );
            check( Length( joint.anchorB - joint.anchorA ) < 0.02f, "limit trial must preserve the hinge pivot" );
        }
        check( hitLimit, "limit trial must reach the boundary" );
        check( returnedInside, "limit trial must allow motion back inside" );
    }
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

    rigidBodyDemo weld{ demoKind::weldPair };
    for( const int substeps : { 1, 4, 8 } )
        for( const bool upper : { false, true } )
        {
            rigidBodyDemo suspension{ demoKind::wheelSuspension };
            suspension.testWheelLimit( upper );
            bool reached = false;
            for( int frame = 0; frame < 240; ++frame )
            {
                suspension.step( 1.0f / 60.0f, substeps );
                const auto joint = suspension.getWorld().getWheelJointData( suspension.getWheelJoint() );
                reached |= upper ? joint.currentTranslation >= joint.upperTranslation - 0.02f : joint.currentTranslation <= joint.lowerTranslation + 0.02f;
                check( joint.currentTranslation <= joint.upperTranslation + 0.03f && joint.currentTranslation >= joint.lowerTranslation - 0.03f, "wheel trial must remain inside travel limits" );
                check( std::abs( joint.lateralError ) < 0.01f, "wheel trial must preserve the suspension axis" );
            }
            const auto joint = suspension.getWorld().getWheelJointData( suspension.getWheelJoint() );
            check( reached, "wheel trial must reach the selected boundary" );
            check( std::abs( joint.currentTranslation ) < 0.1f, "wheel trial must return toward spring equilibrium" );
            check( std::abs( suspension.getWorld().GetBodyAngularVelocity( joint.bodyB ) - 3.0f ) < 0.01f, "wheel rotation must continue independently of travel limits" );
        }
    check( weld.getWorld().GetBodyCount() == 2 && weld.getWorld().getJointCount() == 1 && weld.getShapes().size() == 2, "Weld scene has two connected dynamic bodies" );
    const jointId weldId = weld.getWeldJoint();
    check( weld.getWorld().IsValid( weldId ), "Weld scene owns a valid joint" );
    auto weldData = weld.getWorld().getWeldJointData( weldId );
    check( Length( weldData.anchorB - weldData.anchorA ) < 0.00001f && std::abs( weldData.currentAngle ) < 0.00001f, "Weld pair starts at its reference transform" );

    const transform2 initialA = weld.getWorld().GetBodyTransform( weldData.bodyA );
    const transform2 initialB = weld.getWorld().GetBodyTransform( weldData.bodyB );
    const vec2 initialLocalDelta = Rotate( Inverse( initialA.rotation ), initialB.position - initialA.position );
    weld.getWorld().ApplyLinearImpulseToCenter( weldData.bodyB, { 2.0f, 1.0f } );
    weld.getWorld().ApplyAngularImpulse( weldData.bodyB, 0.3f );
    for( int i = 0; i < 180; ++i ) weld.step( 1.0f / 60.0f, 4 );
    weldData = weld.getWorld().getWeldJointData( weldId );
    check( Length( weldData.anchorB - weldData.anchorA ) < 0.01f && std::abs( weldData.currentAngle ) < 0.01f, "Weld demo restores anchor and relative angle after impulses" );
    const transform2 finalA = weld.getWorld().GetBodyTransform( weldData.bodyA );
    const transform2 finalB = weld.getWorld().GetBodyTransform( weldData.bodyB );
    const vec2 finalLocalDelta = Rotate( Inverse( finalA.rotation ), finalB.position - finalA.position );
    check( Length( finalLocalDelta - initialLocalDelta ) < 0.05f, "Weld demo keeps the body origins fixed in A local frame" );

    rigidBodyDemo prismatic{ demoKind::prismaticRail };
    for( const int substeps : { 1, 4, 8 } )
        for( const auto preset : { weldDemoPreset::rigid, weldDemoPreset::softLinear, weldDemoPreset::softAngular, weldDemoPreset::softBoth } )
        {
            rigidBodyDemo beam{ demoKind::weldPair };
            beam.applyWeldPreset( preset );
            beam.setWeldCantilever( true );
            for( int frame = 0; frame < 600; ++frame ) beam.step( 1.0f / 60.0f, substeps );
            auto data = beam.getWorld().getWeldJointData( beam.getWeldJoint() );
            const bool linear = preset == weldDemoPreset::softLinear || preset == weldDemoPreset::softBoth;
            const bool angular = preset == weldDemoPreset::softAngular || preset == weldDemoPreset::softBoth;
            check( beam.getWorld().GetBody( data.bodyA ).type == bodyType::Static && beam.getWorld().GetBodyCount() == 2, "cantilever must have one fixed support without accumulating bodies" );
            check( IsFinite( data.anchorB ) && Length( data.anchorB - data.anchorA ) < 0.2f && std::abs( data.currentAngle ) < 0.5f, "cantilever must remain supported under gravity" );
            check( linear ? Length( data.anchorB - data.anchorA ) > 0.01f : Length( data.anchorB - data.anchorA ) < 0.01f, "linear softness must control sustained anchor deflection" );
            check( angular ? std::abs( data.currentAngle ) > 0.01f : std::abs( data.currentAngle ) < 0.01f, "angular softness must control sustained bending" );
            beam.resetWeldPair();
            data = beam.getWorld().getWeldJointData( beam.getWeldJoint() );
            check( Length( data.anchorB - data.anchorA ) < 0.001f && std::abs( data.currentAngle ) < 0.001f, "reset must restore reference pose" );
            check( data.linearHertz == ( linear ? 3.0f : 0.0f ) && data.angularHertz == ( angular ? 3.0f : 0.0f ), "reset must preserve softness settings" );
            demoInput drag{};
            drag.mousePressed = drag.mouseHeld = true;
            drag.mousePosition = beam.getWorld().GetBodyTransform( data.bodyB ).position;
            beam.handleInput( drag );
            check( beam.getWorld().IsValid( beam.getMouseJoint() ), "anchored Weld must allow dragging the loaded body" );
            beam.resetWeldPair();
            check( !beam.getWorld().IsValid( beam.getMouseJoint() ) && beam.getWorld().getJointCount() == 1, "reset must cancel dragging before rebuilding bodies" );
            beam.setWeldCantilever( false );
            data = beam.getWorld().getWeldJointData( beam.getWeldJoint() );
            check( beam.getWorld().GetBody( data.bodyA ).type == bodyType::Dynamic && beam.getWorld().getJointCount() == 1 && beam.getShapes().size() == 2, "free pair must restore its original scene without leaking joints or shapes" );
            beam.getWorld().ApplyLinearImpulseToCenter( data.bodyB, { 0.0f, 3.0f } );
            for( int frame = 0; frame < 30; ++frame ) beam.step( 1.0f / 60.0f, substeps );
            beam.resetWeldPair();
            data = beam.getWorld().getWeldJointData( beam.getWeldJoint() );
            check( Length( beam.getWorld().GetBodyTransform( data.bodyB ).position - vec2{ 0.7f, 0.0f } ) < 0.001f && Length( beam.getWorld().GetBodyLinearVelocity( data.bodyB ) ) == 0.0f, "free pair reset must restore position and stop drift" );
        }
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

    rigidBodyDemo ragdoll{ demoKind::ragdoll };
    for( const int substeps : { 1, 4, 8 } )
    {
        rigidBodyDemo incline{ demoKind::prismaticRail };
        incline.setPrismaticIncline( true );
        incline.setPrismaticMotorSettings( true, 2.0f, 1.0f );
        for( int frame = 0; frame < 60; ++frame ) incline.step( 1.0f / 60.0f, substeps );
        auto data = incline.getWorld().getPrismaticJointData( incline.getPrismaticJoint() );
        check( data.currentTranslation < -1.8f && data.currentTranslation >= -2.03f, "weak motor must lose to downhill load and stop at the limit" );
        incline.resetPrismaticSlider();
        data = incline.getWorld().getPrismaticJointData( incline.getPrismaticJoint() );
        check( std::abs( data.currentTranslation + 1.5f ) < 0.001f && data.maxMotorForce == 1.0f, "reset must restore position without changing settings" );
        incline.setPrismaticMotorSettings( true, 2.0f, 20.0f );
        for( int frame = 0; frame < 30; ++frame ) incline.step( 1.0f / 60.0f, substeps );
        data = incline.getWorld().getPrismaticJointData( incline.getPrismaticJoint() );
        check( data.currentTranslation > -0.8f && std::abs( data.lateralError ) < 0.01f && std::abs( data.currentAngle ) < 0.01f, "strong motor must climb while staying on the inclined rail" );
        incline.setPrismaticMotorSettings( true, 0.0f, 20.0f );
        for( int frame = 0; frame < 30; ++frame ) incline.step( 1.0f / 60.0f, substeps );
        const float held = incline.getWorld().getPrismaticJointData( incline.getPrismaticJoint() ).currentTranslation;
        for( int frame = 0; frame < 120; ++frame ) incline.step( 1.0f / 60.0f, substeps );
        check( std::abs( incline.getWorld().getPrismaticJointData( incline.getPrismaticJoint() ).currentTranslation - held ) < 0.01f, "brake must hold against gravity away from the limits" );
        incline.applyPrismaticPreset( prismaticDemoPreset::spring );
        for( int frame = 0; frame < 240; ++frame ) incline.step( 1.0f / 60.0f, substeps );
        data = incline.getWorld().getPrismaticJointData( incline.getPrismaticJoint() );
        check( data.currentTranslation > 0.8f && data.currentTranslation < 1.0f, "spring must support gravity with a small displacement below its target" );
        incline.setPrismaticIncline( false );
        data = incline.getWorld().getPrismaticJointData( incline.getPrismaticJoint() );
        check( std::abs( data.axis.y ) < 0.001f && data.enableLimit && data.enableMotor, "horizontal scene must restore bounded default drive" );
    }
    check( ragdoll.getWorld().GetBodyCount() == 12 && ragdoll.getWorld().getJointCount() == 10 && ragdoll.getShapes().size() == 12, "Ragdoll scene has ground plus eleven body parts and ten joints" );
    check( ragdoll.getRagdollBodies().size() == 11 && ragdoll.getRagdollJoints().size() == 10, "Ragdoll exposes its body and joint assembly" );
    for( const bodyId body : ragdoll.getRagdollBodies() )
    {
        check( ragdoll.getWorld().IsValid( body ), "Ragdoll body handle stays valid" );
    }
    for( const jointId joint : ragdoll.getRagdollJoints() )
    {
        check( ragdoll.getWorld().IsValid( joint ), "Ragdoll joint handle stays valid" );
        const revoluteJointData data = ragdoll.getWorld().getRevoluteJointData( joint );
        check( data.enableLimit && !data.enableMotor && data.lowerAngle < data.upperAngle, "Ragdoll uses bounded passive Revolute joints" );
        check( IsFinite( data.anchorA ) && IsFinite( data.anchorB ) && std::isfinite( data.currentAngle ), "Ragdoll joint starts finite" );
    }

    ragdoll.getWorld().ApplyLinearImpulseToCenter( ragdoll.getRagdollBodies()[1], { 12.0f, 3.0f } );
    for( int i = 0; i < 240; ++i ) ragdoll.step( 1.0f / 60.0f, 4 );
    for( const bodyId body : ragdoll.getRagdollBodies() )
    {
        const transform2 transform = ragdoll.getWorld().GetBodyTransform( body );
        check( IsFinite( transform.position ), "Ragdoll remains finite after a sustained multi-joint simulation" );
    }
    for( const jointId joint : ragdoll.getRagdollJoints() )
    {
        const revoluteJointData data = ragdoll.getWorld().getRevoluteJointData( joint );
        check( data.currentAngle >= data.lowerAngle - 0.15f && data.currentAngle <= data.upperAngle + 0.15f, "Ragdoll settles inside each angular limit" );
    }

    for( const int substeps : { 1, 4, 8 } )
        for( const bool selfCollision : { false, true } )
            for( const float friction : { 0.0f, 0.3f } )
            {
                rigidBodyDemo actor{ demoKind::ragdoll };
                actor.setRagdollJointFriction( friction );
                actor.setRagdollSelfCollision( selfCollision );
                auto& simulation = actor.getWorld();
                simulation.ApplyLinearImpulseToCenter( actor.getImpulseBody(), { 2.0f, 1.0f } );
                bool sawSelfContact = false;
                bool sawFloorContact = false;
                const auto floor = actor.getShapes()[0].shapeHandle;
                for( int frame = 0; frame < 600; ++frame )
                {
                    actor.step( 1.0f / 60.0f, substeps );
                    for( const auto& contact : actor.getContacts() )
                    {
                        const bool internal = contact.shapeA != floor && contact.shapeB != floor;
                        sawFloorContact |= !internal;
                        if( !selfCollision ) check( !internal, "disabled self collision must remove internal contacts" );
                        sawSelfContact |= internal;
                    }
                    for( const auto id : actor.getRagdollJoints() )
                    {
                        const auto joint = simulation.getRevoluteJointData( id );
                        check( IsFinite( joint.anchorA ) && IsFinite( joint.anchorB ) && Length( joint.anchorB - joint.anchorA ) < 0.2f, "ragdoll joints must remain connected through impact" );
                        check( joint.enableMotor == ( friction > 0.0f ) && joint.motorSpeed == 0.0f && joint.maxMotorTorque == friction, "friction setting must reach every joint without a pose target" );
                        check( std::abs( joint.motorTorque ) <= friction + 0.0001f, "joint friction must respect its torque budget" );
                    }
                }
                if( selfCollision ) check( sawSelfContact, "enabled self collision must exercise internal contact" );
                check( sawFloorContact, "self collision selection must preserve floor contact" );
                for( const auto id : actor.getRagdollJoints() )
                {
                    const auto joint = simulation.getRevoluteJointData( id );
                    check( joint.currentAngle >= joint.lowerAngle - 0.15f && joint.currentAngle <= joint.upperAngle + 0.15f, "ragdoll must settle inside angular limits" );
                }
                actor.resetRagdoll();
                check( actor.getRagdollSelfCollision() == selfCollision && actor.getRagdollJointFriction() == friction, "pose recovery must preserve comparison settings" );
                check( Length( simulation.GetBodyTransform( actor.getImpulseBody() ).position - vec2{ 0.0f, 5.7f } ) < 0.001f, "pose recovery must restore initial torso position" );
                for( const auto body : actor.getRagdollBodies() )
                    check( LengthSquared( simulation.GetBodyLinearVelocity( body ) ) == 0.0f && simulation.GetBodyAngularVelocity( body ) == 0.0f, "pose recovery must stop all body parts" );
                check( simulation.GetBodyCount() == 12 && simulation.getJointCount() == 10, "pose recovery must not accumulate simulation objects" );
            }
    for( const int substeps : { 1, 4, 8 } )
    {
        float freeMotion = 0.0f;
        for( const float friction : { 0.0f, 0.3f } )
        {
            rigidBodyDemo actor{ demoKind::ragdoll };
            auto& simulation = actor.getWorld();
            simulation.SetGravity( {} );
            actor.setRagdollSelfCollision( false );
            actor.setRagdollJointFriction( friction );
            for( const auto id : actor.getRagdollJoints() )
            {
                const auto joint = simulation.getRevoluteJointData( id );
                simulation.setRevoluteJointLimit( id, false, joint.lowerAngle, joint.upperAngle );
            }
            const auto torso = actor.getImpulseBody();
            simulation.ApplyAngularImpulse( torso, simulation.GetBodyRotationalInertia( torso ) * 3.0f );
            for( int frame = 0; frame < 120; ++frame ) actor.step( 1.0f / 60.0f, substeps );
            float relativeMotion = 0.0f;
            for( const auto id : actor.getRagdollJoints() )
            {
                const auto joint = simulation.getRevoluteJointData( id );
                relativeMotion += std::abs( simulation.GetBodyAngularVelocity( joint.bodyB ) - simulation.GetBodyAngularVelocity( joint.bodyA ) );
            }
            if( friction == 0.0f ) freeMotion = relativeMotion;
            else check( relativeMotion < freeMotion * 0.2f, "joint friction must reduce relative rotation" );
            actor.resetRagdoll();
            for( const auto id : actor.getRagdollJoints() ) check( !simulation.getRevoluteJointData( id ).enableLimit, "recovery must preserve edited angular limits" );
        }
    }
    return EXIT_SUCCESS;
}
