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
    check( getDemoEntries().size() == 3, "three demo entries" );
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

    return EXIT_SUCCESS;
}
