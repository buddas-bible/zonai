#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "demo.h"
#include "rigidBodyDemo.h"

using namespace zonai;
using namespace zonai::sandbox;

#ifdef ZONAI_TEST_SANDBOX_UI
void checkDemoUi();
#endif

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

struct counterDemo final : demo
{
    int steps = 0;
    bool held = false;

    void step( float timeStep, int subSteps ) override
    {
        check( timeStep == 1.0f / 60.0f && subSteps > 0, "fixed step contract" );
        ++steps;
    }

    void handleInput( const demoInput& input ) override { held = input.left; }

    void cancelInput() override { held = false; }
};

std::unique_ptr<demo> createCounter( demoKind )
{
    return std::make_unique<counterDemo>();
}

}

int main()
{
    check( getDemoEntries().size() == 12, "eleven independently selectable demo entries" );
    demoSession session{ createRigidBodyDemo };
    auto& playground = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( playground.getWorld().GetBodyCount() == 6 && playground.getWorld().getJointCount() == 0, "playground separated from pendulum" );
    const bodyId oldBody = playground.getImpulseBody();
    session.setPlaying( true );
    session.advance( 1.0f / 60.0f, 4 );
    check( session.getStepCount() == 1, "playing advances" );
    session.selectDemo( demoKind::distancePendulum );
    check( !session.isPlaying() && session.getStepCount() == 0, "switch resets playback" );
    auto& pendulum = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( pendulum.getWorld().GetBodyCount() == 2 && pendulum.getWorld().getJointCount() == 1, "independent pendulum scene" );
    check( !pendulum.getWorld().IsValid( oldBody ), "old demo handle cannot refer to new World" );
    const auto jointData = pendulum.getWorld().getDistanceJointData( pendulum.getPendulumJoint() );
    check( !jointData.enableSpring && jointData.hertz == 2.0f && jointData.dampingRatio == 0.7f, "pendulum starts rigid with reproducible spring tuning" );
    check( !jointData.enableLimit && jointData.minLength == 1.5f && jointData.maxLength == 2.5f, "pendulum starts with reproducible limit range" );
    check( !jointData.enableMotor && jointData.motorSpeed == 1.0f && jointData.maxMotorForce == 10.0f, "pendulum starts with reproducible motor tuning" );
    const auto initial = pendulum.getWorld().GetBodyTransform( pendulum.getPendulumBody() );
    demoInput input{};
    input.jumpPressed = true;
    session.handleInput( input, true );
    check( pendulum.getWorld().GetBodyLinearVelocity( pendulum.getPendulumBody() ).x > 0.0f, "pendulum keyboard kick" );
    session.stepOnce( 4 );
    check( session.getStepCount() == 1 && !session.isPlaying(), "single step stays paused" );
    pendulum.getWorld().setDistanceJointSpring( pendulum.getPendulumJoint(), true, 3.0f, 1.0f );
    pendulum.getWorld().setDistanceJointLimit( pendulum.getPendulumJoint(), true, 1.0f, 3.0f );
    pendulum.getWorld().setDistanceJointMotor( pendulum.getPendulumJoint(), true, -2.0f, 20.0f );
    session.reset();
    auto& reset = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( reset.getWorld().GetBodyTransform( reset.getPendulumBody() ).position.x == initial.position.x && session.getStepCount() == 0, "restart current demo" );
    const auto resetJoint = reset.getWorld().getDistanceJointData( reset.getPendulumJoint() );
    check( !resetJoint.enableSpring && resetJoint.hertz == 2.0f && resetJoint.dampingRatio == 0.7f, "reset clears spring mode and tuning" );
    check( !resetJoint.enableLimit && resetJoint.minLength == 1.5f && resetJoint.maxLength == 2.5f, "reset clears limit mode and range" );
    check( !resetJoint.enableMotor && resetJoint.motorSpeed == 1.0f && resetJoint.maxMotorForce == 10.0f, "reset clears motor mode and tuning" );
    input = {};
    input.impulsePressed = true;
    input.mousePosition = { 2.0f, -2.0f };
    session.handleInput( input, true );
    check( reset.getWorld().GetBodyLinearVelocity( reset.getPendulumBody() ).x > 0.0f, "pendulum mouse experiment" );
    reset.getWorld().SetBodyLinearVelocity( reset.getPendulumBody(), {} );
    input = {};
    input.impulsePressed = true;
    input.mousePosition = { 2.0f, -2.0f };
    session.handleInput( input, false );
    check( LengthSquared( reset.getWorld().GetBodyLinearVelocity( reset.getPendulumBody() ) ) == 0.0f, "UI click must not trigger experiment" );
    input = {};
    input.right = true;
    session.handleInput( input, true );
    session.stepOnce( 4 );
    check( reset.getWorld().GetBodyLinearVelocity( reset.getPendulumBody() ).x > 0.0f, "held key applies force each step" );
    reset.getWorld().SetBodyLinearVelocity( reset.getPendulumBody(), {} );
    reset.getWorld().SetBodyAngularVelocity( reset.getPendulumBody(), 0.0f );
    reset.getWorld().SetBodyTransform( reset.getPendulumBody(), initial );
    input = {};
    input.right = true;
    session.handleInput( input, true );
    session.handleInput( {}, false );
    session.stepOnce( 4 );
    check( std::abs( reset.getWorld().GetBodyLinearVelocity( reset.getPendulumBody() ).x ) < 0.000001f, "capture loss clears held force" );

    input = {};
    input.mousePressed = true;
    input.mouseHeld = true;
    input.mousePosition = initial.position;
    session.handleInput( input, true );
    const auto drag = reset.getMouseJoint();
    check( reset.getWorld().IsValid( drag ), "click inside dynamic geometry starts drag" );
    input.mousePressed = false;
    input.mousePosition = { 1.0f, -2.0f };
    session.handleInput( input, true );
    check( reset.getWorld().getMouseJointData( drag ).target.x == 1.0f, "held drag updates target" );
    session.handleInput( input, false );
    check( !reset.getWorld().IsValid( drag ) && reset.getWorld().getJointCount() == 1, "capture loss removes only mouse joint" );
    session.handleInput( input, true );
    check( !reset.getWorld().IsValid( reset.getMouseJoint() ), "held button after capture loss cannot restart drag" );
    input.mousePressed = true;
    input.mousePosition = initial.position;
    session.handleInput( input, true );
    check( reset.getWorld().IsValid( reset.getMouseJoint() ), "fresh press starts new drag" );
    input.mousePressed = false;
    input.mouseHeld = false;
    session.handleInput( input, true );
    check( reset.getWorld().getJointCount() == 1, "release removes mouse joint" );
    input = {};
    input.mousePressed = true;
    input.mouseHeld = true;
    input.mousePosition = { 0.0f, 0.0f };
    session.handleInput( input, true );
    check( reset.getWorld().getJointCount() == 1, "static geometry cannot be dragged" );
    input.mousePosition = initial.position;
    session.handleInput( input, true );
    const auto oldDrag = reset.getMouseJoint();
    session.reset();
    auto& fresh = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( !fresh.getWorld().IsValid( oldDrag ) && fresh.getWorld().getJointCount() == 1, "reset rejects old drag ID" );
    session.selectDemo( demoKind::playground );
    auto& pick = static_cast<rigidBodyDemo&>( session.getDemo() );
    input = {};
    input.mousePressed = true;
    input.mouseHeld = true;
    input.mousePosition = pick.getWorld().GetBodyTransform( pick.getTorqueBody() ).position;
    session.handleInput( input, true );
    const auto deletedDrag = pick.getMouseJoint();
    check( pick.getWorld().IsValid( deletedDrag ), "polygon picking starts drag" );
    pick.getWorld().DestroyBody( pick.getTorqueBody() );
    input.mousePressed = false;
    session.handleInput( input, true );
    check( !pick.getWorld().IsValid( pick.getMouseJoint() ) && !pick.getWorld().IsValid( deletedDrag ), "deleted grabbed Body clears stale drag" );

    demoSession clock{ createCounter };
    auto& counter = static_cast<counterDemo&>( clock.getDemo() );
    clock.advance( 0.5f, 1 );
    check( counter.steps == 0, "paused does not accumulate time" );
    clock.setPlaying( true );
    clock.advance( 1.0f / 120.0f, 1 );
    check( counter.steps == 0, "fractional frame waits" );
    clock.advance( 1.0f / 120.0f, 1 );
    check( counter.steps == 1, "fractional frames accumulate" );
    clock.advance( 1.0f / 120.0f, 1 );
    clock.setPlaying( false );
    clock.setPlaying( true );
    clock.advance( 1.0f / 120.0f, 1 );
    check( counter.steps == 1, "pause discards old fractional time" );
    clock.advance( 1.0f, 1 );
    check( counter.steps == 9, "long frame capped at eight steps" );
    clock.advance( 0.0f, 1 );
    check( counter.steps == 9, "catchup remainder discarded" );
    input = {};
    input.left = true;
    clock.handleInput( input, true );
    check( counter.held, "active input delivered" );
    clock.handleInput( input, false );
    check( !counter.held, "inactive input cancelled" );
    clock.reset();
    check( !static_cast<counterDemo&>( clock.getDemo() ).held && clock.getStepCount() == 0, "reset replaces demo state" );
#ifdef ZONAI_TEST_SANDBOX_UI
    checkDemoUi();
#endif
    demoSession project{ createRigidBodyDemo };
    collisionMatrix matrix;
    matrix.setPair( 0, 63, false );
    project.setCollisionMatrix( matrix );
    project.selectDemo( demoKind::distancePendulum );
    project.reset();
    check( !project.getCollisionMatrix().allows( 1, std::uint64_t{ 1 } << 63 ), "project matrix persists across demo switch and Reset" );
    auto& shared = static_cast<rigidBodyDemo&>( project.getDemo() );
    check( shared.getWorld().getCollisionMatrix() == project.getCollisionMatrix(), "new demo receives project matrix" );

    session.selectDemo( demoKind::revoluteHinge );
    auto& hinge = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( hinge.getWorld().GetBodyCount() == 2 && hinge.getWorld().getJointCount() == 1, "independent hinge scene" );
    const auto hingeId = hinge.getRevoluteJoint();
    const auto rod = hinge.getImpulseBody();
    input = {};
    input.spinPressed = true;
    session.handleInput( input, true );
    check( hinge.getWorld().GetBodyAngularVelocity( rod ) > 0.0f, "hinge keyboard spin" );
    for( int i = 0; i < 120; ++i )
    {
        session.stepOnce( 4 );
    }
    const auto hingeData = hinge.getWorld().getRevoluteJointData( hingeId );
    check( Length( hingeData.anchorB - hingeData.anchorA ) < 0.01f, "demo rod remains attached to pivot" );
    const auto hingePose = hinge.getWorld().GetBodyTransform( rod );
    input = {};
    input.mousePressed = true;
    input.mouseHeld = true;
    input.mousePosition = hingePose.position;
    session.handleInput( input, true );
    check( hinge.getWorld().getJointCount() == 2 && hinge.getWorld().IsValid( hinge.getMouseJoint() ), "mouse drag coexists with hinge" );
    session.handleInput( {}, false );
    check( hinge.getWorld().getJointCount() == 1 && hinge.getWorld().IsValid( hingeId ), "UI capture cancels drag without destroying hinge" );
    session.reset();
    auto& resetHinge = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( !resetHinge.getWorld().IsValid( hingeId ) && resetHinge.getWorld().getJointCount() == 1 && session.getStepCount() == 0, "reset reconstructs hinge and rejects old handle" );

    const auto resetHingeJoint = resetHinge.getRevoluteJoint();
    const auto resetData = resetHinge.getWorld().getRevoluteJointData( resetHingeJoint );
    check( !resetData.enableLimit && resetData.lowerAngle < -0.78f && resetData.upperAngle > 0.78f, "reset restores free hinge with editable angle range" );
    resetHinge.getWorld().setRevoluteJointLimit( resetHingeJoint, true, resetData.lowerAngle, resetData.upperAngle );
    const auto resetRod = resetHinge.getImpulseBody();
    resetHinge.getWorld().ApplyAngularImpulse( resetRod, resetHinge.getWorld().GetBodyRotationalInertia( resetRod ) * 10.0f );
    float maximumAngle = 0.0f;
    for( int i = 0; i < 180; ++i )
    {
        session.stepOnce( 4 );
        const auto data = resetHinge.getWorld().getRevoluteJointData( resetHingeJoint );
        maximumAngle = std::max( maximumAngle, data.currentAngle );
        check( data.currentAngle >= data.lowerAngle - 0.025f && data.currentAngle <= data.upperAngle + 0.025f, "rod respects angular range under gravity and impulse" );
        check( Length( data.anchorB - data.anchorA ) < 0.015f, "angular limit preserves off-center rod pivot" );
    }
    check( maximumAngle > 0.7f, "rod impulse exercises upper boundary rather than only interior motion" );

    session.reset();
    auto& poweredHinge = static_cast<rigidBodyDemo&>( session.getDemo() );
    const auto poweredJoint = poweredHinge.getRevoluteJoint();
    const auto motorDefaults = poweredHinge.getWorld().getRevoluteJointData( poweredJoint );
    check( !motorDefaults.enableMotor && motorDefaults.motorSpeed == 2.0f && motorDefaults.maxMotorTorque == 10.0f, "demo starts with motor off and useful editable settings" );
    poweredHinge.getWorld().setRevoluteJointMotor( poweredJoint, true, 2.0f, 20.0f );
    float turnedAngle = 0.0f;
    for( int i = 0; i < 180; ++i )
    {
        session.stepOnce( 4 );
        const auto data = poweredHinge.getWorld().getRevoluteJointData( poweredJoint );
        turnedAngle = std::max( turnedAngle, std::abs( data.currentAngle ) );
        check( Length( data.anchorB - data.anchorA ) < 0.015f && std::abs( data.motorTorque ) <= 20.001f, "powered rod keeps pivot under gravity within motor torque budget" );
    }
    check( turnedAngle > 2.0f, "motor demo exercises substantial rotation rather than only local motion" );
    session.reset();
    auto& freeHinge = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( !freeHinge.getWorld().getRevoluteJointData( freeHinge.getRevoluteJoint() ).enableMotor, "reset disables motor" );

    session.selectDemo( demoKind::wheelSuspension );
    auto& suspension = static_cast<rigidBodyDemo&>( session.getDemo() );
    const auto wheelJoint = suspension.getWheelJoint();
    const auto wheelBody = suspension.getImpulseBody();
    check( suspension.getWorld().GetBodyCount() == 2 && suspension.getWorld().getJointCount() == 1 && suspension.getWorld().IsValid( wheelJoint ), "independent suspension scene" );
    input = {};
    input.spinPressed = true;
    input.jumpPressed = true;
    session.handleInput( input, true );
    check( suspension.getWorld().GetBodyAngularVelocity( wheelBody ) > 2.9f && suspension.getWorld().GetBodyLinearVelocity( wheelBody ).y > 4.9f, "wheel keyboard rotation and suspension kick" );
    for( int i = 0; i < 180; ++i )
    {
        session.stepOnce( 4 );
        const auto data = suspension.getWorld().getWheelJointData( wheelJoint );
        check( std::abs( data.lateralError ) < 0.015f && IsFinite( data.force ), "suspension demo keeps wheel on axis under gravity and impulse" );
    }
    const auto suspensionData = suspension.getWorld().getWheelJointData( wheelJoint );
    check( suspensionData.currentTranslation < -0.01f && suspensionData.currentTranslation > -0.06f && suspension.getWorld().GetBodyAngularVelocity( wheelBody ) > 2.9f, "demo suspension settles while wheel rotation remains free" );
    input = {};
    input.mousePressed = true;
    input.mouseHeld = true;
    input.mousePosition = suspension.getWorld().GetBodyTransform( wheelBody ).position;
    session.handleInput( input, true );
    check( suspension.getWorld().getJointCount() == 2 && suspension.getWorld().IsValid( suspension.getMouseJoint() ), "mouse drag coexists with wheel joint" );
    session.handleInput( {}, false );
    check( suspension.getWorld().getJointCount() == 1 && suspension.getWorld().IsValid( wheelJoint ), "UI capture cancels mouse drag and preserves suspension" );
    suspension.getWorld().setWheelJointSpring( wheelJoint, false, 0.0f, 0.0f );
    session.reset();
    auto& resetSuspension = static_cast<rigidBodyDemo&>( session.getDemo() );
    const auto restoredSpring = resetSuspension.getWorld().getWheelJointData( resetSuspension.getWheelJoint() );
    check( !resetSuspension.getWorld().IsValid( wheelJoint ) && restoredSpring.enableSpring && restoredSpring.hertz == 3.0f && restoredSpring.dampingRatio == 0.7f && session.getStepCount() == 0 && !session.isPlaying(), "reset restores suspension defaults and rejects old handles" );
    check( !restoredSpring.enableLimit && restoredSpring.lowerTranslation == -0.5f && restoredSpring.upperTranslation == 0.5f, "demo starts with limits off and useful editable travel range" );
    const auto limitedJoint = resetSuspension.getWheelJoint();
    const auto limitedWheel = resetSuspension.getImpulseBody();
    resetSuspension.getWorld().setWheelJointSpring( limitedJoint, false, 0.0f, 0.7f );
    resetSuspension.getWorld().setWheelJointLimit( limitedJoint, true, -0.5f, 0.5f );
    resetSuspension.getWorld().SetBodyLinearVelocity( limitedWheel, { 0.0f, 8.0f } );
    resetSuspension.getWorld().SetBodyAngularVelocity( limitedWheel, 3.0f );
    float maximumTranslation = -0.5f;
    for( int i = 0; i < 180; ++i )
    {
        session.stepOnce( 4 );
        const auto current = resetSuspension.getWorld().getWheelJointData( limitedJoint );
        maximumTranslation = std::max( maximumTranslation, current.currentTranslation );
        check( current.currentTranslation >= -0.515f && current.currentTranslation <= 0.515f && std::abs( current.lateralError ) < 0.015f, "demo limits preserve translation range and axis with spring disabled" );
    }
    const auto limitedData = resetSuspension.getWorld().getWheelJointData( limitedJoint );
    check( maximumTranslation > 0.45f && limitedData.currentTranslation < -0.48f && limitedData.limitForce > 0.0f && resetSuspension.getWorld().GetBodyAngularVelocity( limitedWheel ) > 2.9f, "demo exercises both boundaries and gravity reaction without locking wheel rotation" );
    session.reset();
    auto& freeSuspension = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( !freeSuspension.getWorld().getWheelJointData( freeSuspension.getWheelJoint() ).enableLimit, "reset disables translation limit" );

    const auto motorJoint = freeSuspension.getWheelJoint();
    const auto wheelMotorDefaults = freeSuspension.getWorld().getWheelJointData( motorJoint );
    check( !wheelMotorDefaults.enableMotor && wheelMotorDefaults.motorSpeed == 3.0f && wheelMotorDefaults.maxMotorTorque == 1.0f, "demo motor starts off with useful editable speed and torque" );
    freeSuspension.getWorld().setWheelJointLimit( motorJoint, true, -0.5f, 0.5f );
    freeSuspension.getWorld().setWheelJointMotor( motorJoint, true, 3.0f, 0.2f );
    for( int i = 0; i < 180; ++i )
    {
        session.stepOnce( 4 );
        const auto current = freeSuspension.getWorld().getWheelJointData( motorJoint );
        check( std::abs( current.motorTorque ) <= 0.20001f && std::abs( current.lateralError ) < 0.015f && current.currentTranslation >= -0.515f && current.currentTranslation <= 0.515f, "demo motor coexists with spring and travel limit" );
    }
    check( std::abs( freeSuspension.getWorld().GetBodyAngularVelocity( freeSuspension.getImpulseBody() ) - 3.0f ) < 0.001f, "demo motor reaches target speed" );
    freeSuspension.getWorld().setWheelJointMotor( motorJoint, true, -3.0f, 0.2f );
    for( int i = 0; i < 60; ++i ) session.stepOnce( 4 );
    check( std::abs( freeSuspension.getWorld().GetBodyAngularVelocity( freeSuspension.getImpulseBody() ) + 3.0f ) < 0.001f, "demo motor reverses direction" );
    session.reset();
    auto& resetMotor = static_cast<rigidBodyDemo&>( session.getDemo() );
    const auto restoredMotor = resetMotor.getWorld().getWheelJointData( resetMotor.getWheelJoint() );
    check( !restoredMotor.enableMotor && restoredMotor.motorSpeed == 3.0f && restoredMotor.maxMotorTorque == 1.0f && restoredMotor.motorTorque == 0.0f, "reset restores disabled motor and clears torque" );

    session.selectDemo( demoKind::motorCar );
    auto& car = static_cast<rigidBodyDemo&>( session.getDemo() );
    const auto chassis = car.getImpulseBody();
    check( car.getWorld().GetBodyCount() == 5 && car.getWorld().getJointCount() == 2 && car.getShapes().size() == 5, "car composes chassis, two wheels, ground and ramp" );
    check( car.getCarMotorSpeed() == 8.0f && car.getCarMaxMotorTorque() == 5.0f, "car has reproducible motor settings" );
    for( const auto id : car.getCarJoints() )
    {
        const auto data = car.getWorld().getWheelJointData( id );
        check( data.bodyA == chassis && data.enableSpring && data.hertz == 4.0f && data.dampingRatio == 0.7f && data.enableLimit && data.lowerTranslation == -0.25f && data.upperTranslation == 0.25f && !data.enableMotor && data.maxMotorTorque == 5.0f, "both wheels share sprung, limited, initially coasting chassis" );
    }
    for( int i = 0; i < 120; ++i ) session.stepOnce( 4 );
    const float startX = car.getWorld().GetBodyTransform( chassis ).position.x;
    input = {};
    input.right = true;
    session.handleInput( input, true );
    for( int i = 0; i < 120; ++i )
    {
        session.stepOnce( 4 );
        for( const auto id : car.getCarJoints() )
        {
            const auto data = car.getWorld().getWheelJointData( id );
            check( data.enableMotor && data.motorSpeed == -8.0f && std::abs( data.motorTorque ) <= 5.0001f && std::abs( data.lateralError ) < 0.04f && std::abs( data.currentTranslation ) < 0.3f, "clockwise wheels propel right while torque and suspension stay bounded" );
        }
    }
    const float forwardX = car.getWorld().GetBodyTransform( chassis ).position.x;
    check( forwardX > startX + 2.0f && car.getWorld().GetBodyLinearVelocity( chassis ).x > 1.0f, "car actually drives right through tire contact" );
    input = {};
    input.left = true;
    session.handleInput( input, true );
    for( int i = 0; i < 120; ++i ) session.stepOnce( 4 );
    check( car.getWorld().GetBodyTransform( chassis ).position.x < forwardX - 2.0f && car.getWorld().GetBodyLinearVelocity( chassis ).x < -1.0f, "car reverses through motor torque rather than direct chassis force" );
    input.brake = true;
    input.jumpPressed = true;
    const float beforeBrakeY = car.getWorld().GetBodyLinearVelocity( chassis ).y;
    session.handleInput( input, true );
    check( car.getWorld().GetBodyLinearVelocity( chassis ).y == beforeBrakeY, "car space input brakes without applying old jump impulse" );
    for( int i = 0; i < 120; ++i ) session.stepOnce( 4 );
    check( std::abs( car.getWorld().GetBodyLinearVelocity( chassis ).x ) < 0.1f, "held brake stops car on level ground" );
    for( const auto id : car.getCarJoints() ) check( car.getWorld().getWheelJointData( id ).motorSpeed == 0.0f, "brake overrides drive direction" );
    input = {};
    input.left = input.right = true;
    session.handleInput( input, true );
    for( const auto id : car.getCarJoints() ) check( !car.getWorld().getWheelJointData( id ).enableMotor, "opposing drive keys coast" );
    rigidBodyDemo noTorqueCar{ demoKind::motorCar };
    rigidBodyDemo coastCar{ demoKind::motorCar };
    for( int i = 0; i < 120; ++i )
    {
        noTorqueCar.step( 1.0f / 60.0f, 4 );
        coastCar.step( 1.0f / 60.0f, 4 );
    }
    noTorqueCar.setCarMotorSettings( 8.0f, 0.0f );
    input = {};
    input.right = true;
    noTorqueCar.handleInput( input );
    for( int i = 0; i < 60; ++i )
    {
        noTorqueCar.step( 1.0f / 60.0f, 4 );
        coastCar.step( 1.0f / 60.0f, 4 );
    }
    check( std::abs( noTorqueCar.getWorld().GetBodyTransform( noTorqueCar.getImpulseBody() ).position.x - coastCar.getWorld().GetBodyTransform( coastCar.getImpulseBody() ).position.x ) < 0.01f && std::abs( noTorqueCar.getWorld().GetBodyLinearVelocity( noTorqueCar.getImpulseBody() ).x ) < 0.01f, "zero torque drive matches coasting rather than applying chassis force" );
    session.handleInput( input, true );
    const auto beforeCancel = car.getWorld().GetBodyLinearVelocity( chassis );
    car.setCarMotorSettings( 4.0f, 2.0f );
    for( const auto id : car.getCarJoints() )
    {
        const auto data = car.getWorld().getWheelJointData( id );
        check( data.enableMotor && data.motorSpeed == -4.0f && data.maxMotorTorque == 2.0f, "live motor settings update both active wheels" );
    }
    session.handleInput( {}, false );
    check( LengthSquared( car.getWorld().GetBodyLinearVelocity( chassis ) - beforeCancel ) == 0.0f, "input cancellation preserves physical velocity" );
    for( const auto id : car.getCarJoints() ) check( !car.getWorld().getWheelJointData( id ).enableMotor, "UI capture cancels both wheel motors" );
    input = {};
    input.mousePressed = input.mouseHeld = true;
    input.mousePosition = car.getWorld().GetBodyTransform( chassis ).position;
    session.handleInput( input, true );
    check( car.getWorld().getJointCount() == 3 && car.getWorld().IsValid( car.getMouseJoint() ), "mouse can drag chassis with both wheel joints intact" );
    session.handleInput( {}, false );
    check( car.getWorld().getJointCount() == 2, "capture removes mouse joint and preserves car suspension" );
    session.setPlaying( true );
    input = {};
    input.right = true;
    session.handleInput( input, true );
    session.setPlaying( false );
    for( const auto id : car.getCarJoints() ) check( !car.getWorld().getWheelJointData( id ).enableMotor, "pause cancels active car drive" );
    car.setCarMotorSettings( 2.0f, 1.0f );
    const auto oldCarJoint = car.getCarJoints()[0];
    session.reset();
    auto& restartedCar = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( !restartedCar.getWorld().IsValid( chassis ) && !restartedCar.getWorld().IsValid( oldCarJoint ) && restartedCar.getCarMotorSpeed() == 8.0f && restartedCar.getCarMaxMotorTorque() == 5.0f && !session.isPlaying() && session.getStepCount() == 0, "car reset restores fresh handles, settings and playback" );

    for( const int subSteps : { 1, 4 } )
    {
        rigidBodyDemo rampCar{ demoKind::motorCar };
        for( int i = 0; i < 120; ++i ) rampCar.step( 1.0f / 60.0f, subSteps );
        input = {};
        input.right = true;
        rampCar.handleInput( input );
        float peakHeight = 0.0f;
        for( int i = 0; i < 360; ++i )
        {
            rampCar.step( 1.0f / 60.0f, subSteps );
            const auto pose = rampCar.getWorld().GetBodyTransform( rampCar.getImpulseBody() );
            peakHeight = std::max( peakHeight, pose.position.y );
            check( IsFinite( pose.position ) && std::abs( std::atan2( pose.rotation.s, pose.rotation.c ) ) < 0.7f, "car stays upright through low ramp and landing" );
            for( const auto id : rampCar.getCarJoints() )
            {
                const auto data = rampCar.getWorld().getWheelJointData( id );
                check( std::abs( data.lateralError ) < 0.05f && std::abs( data.currentTranslation ) < 0.4f && std::abs( data.motorTorque ) <= 5.0001f, "ramp keeps coupled suspension and motor within expected soft-error bounds" );
            }
        }
        check( peakHeight > 1.2f && rampCar.getWorld().GetBodyTransform( rampCar.getImpulseBody() ).position.x > 14.0f, "car climbs ramp and continues onto level ground" );
    }

    return EXIT_SUCCESS;
}
