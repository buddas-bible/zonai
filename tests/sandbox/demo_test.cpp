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
    if( !condition ) { std::fprintf( stderr, "%s\n", message ); std::exit( EXIT_FAILURE ); }
}

struct counterDemo final : demo
{
    int steps = 0;
    bool held = false;
    void step( float timeStep, int subSteps ) override
    {
        check( timeStep == 1.0f / 60.0f && subSteps > 0, "fixed step contract" ); ++steps;
    }
    void handleInput( const demoInput& input ) override { held = input.left; }
    void cancelInput() override { held = false; }
};
std::unique_ptr<demo> createCounter( demoKind ) { return std::make_unique<counterDemo>(); }
}

int main()
{
    check( getDemoEntries().size() == 2, "two demo entries" );
    demoSession session{ createRigidBodyDemo };
    auto& playground = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( playground.getWorld().GetBodyCount() == 6 && playground.getWorld().getJointCount() == 0, "playground separated from pendulum" );
    const bodyId oldBody = playground.getImpulseBody();
    session.setPlaying( true ); session.advance( 1.0f / 60.0f, 4 );
    check( session.getStepCount() == 1, "playing advances" );
    session.selectDemo( demoKind::distancePendulum );
    check( !session.isPlaying() && session.getStepCount() == 0, "switch resets playback" );
    auto& pendulum = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( pendulum.getWorld().GetBodyCount() == 2 && pendulum.getWorld().getJointCount() == 1, "independent pendulum scene" );
    check( !pendulum.getWorld().IsValid( oldBody ), "old demo handle cannot refer to new World" );
    const auto jointData = pendulum.getWorld().getDistanceJointData( pendulum.getPendulumJoint() );
    check( !jointData.enableSpring && jointData.hertz == 2.0f && jointData.dampingRatio == 0.7f, "pendulum starts rigid with reproducible spring tuning" );
    check( !jointData.enableLimit && jointData.minLength == 1.5f && jointData.maxLength == 2.5f, "pendulum starts with reproducible limit range" );
    const auto initial = pendulum.getWorld().GetBodyTransform( pendulum.getPendulumBody() );
    demoInput input{}; input.jumpPressed = true;
    session.handleInput( input, true );
    check( pendulum.getWorld().GetBodyLinearVelocity( pendulum.getPendulumBody() ).x > 0.0f, "pendulum keyboard kick" );
    session.stepOnce( 4 );
    check( session.getStepCount() == 1 && !session.isPlaying(), "single step stays paused" );
    pendulum.getWorld().setDistanceJointSpring( pendulum.getPendulumJoint(), true, 3.0f, 1.0f );
    pendulum.getWorld().setDistanceJointLimit( pendulum.getPendulumJoint(), true, 1.0f, 3.0f );
    session.reset();
    auto& reset = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( reset.getWorld().GetBodyTransform( reset.getPendulumBody() ).position.x == initial.position.x && session.getStepCount() == 0, "restart current demo" );
    const auto resetJoint = reset.getWorld().getDistanceJointData( reset.getPendulumJoint() );
    check( !resetJoint.enableSpring && resetJoint.hertz == 2.0f && resetJoint.dampingRatio == 0.7f, "reset clears spring mode and tuning" );
    check( !resetJoint.enableLimit && resetJoint.minLength == 1.5f && resetJoint.maxLength == 2.5f, "reset clears limit mode and range" );
    input = {}; input.impulsePressed = true; input.mousePosition = { 2.0f, -2.0f };
    session.handleInput( input, true );
    check( reset.getWorld().GetBodyLinearVelocity( reset.getPendulumBody() ).x > 0.0f, "pendulum mouse experiment" );
    reset.getWorld().SetBodyLinearVelocity( reset.getPendulumBody(), {} );
    input = {}; input.impulsePressed = true; input.mousePosition = { 2.0f, -2.0f };
    session.handleInput( input, false );
    check( LengthSquared( reset.getWorld().GetBodyLinearVelocity( reset.getPendulumBody() ) ) == 0.0f, "UI click must not trigger experiment" );
    input = {}; input.right = true; session.handleInput( input, true ); session.stepOnce( 4 );
    check( reset.getWorld().GetBodyLinearVelocity( reset.getPendulumBody() ).x > 0.0f, "held key applies force each step" );
    reset.getWorld().SetBodyLinearVelocity( reset.getPendulumBody(), {} );
    reset.getWorld().SetBodyAngularVelocity( reset.getPendulumBody(), 0.0f );
    reset.getWorld().SetBodyTransform( reset.getPendulumBody(), initial );
    input = {}; input.right = true; session.handleInput( input, true );
    session.handleInput( {}, false ); session.stepOnce( 4 );
    check( std::abs( reset.getWorld().GetBodyLinearVelocity( reset.getPendulumBody() ).x ) < 0.000001f, "capture loss clears held force" );

    input = {}; input.mousePressed = true; input.mouseHeld = true; input.mousePosition = initial.position;
    session.handleInput( input, true );
    const auto drag = reset.getMouseJoint();
    check( reset.getWorld().IsValid( drag ), "click inside dynamic geometry starts drag" );
    input.mousePressed = false; input.mousePosition = { 1.0f, -2.0f }; session.handleInput( input, true );
    check( reset.getWorld().getMouseJointData( drag ).target.x == 1.0f, "held drag updates target" );
    session.handleInput( input, false );
    check( !reset.getWorld().IsValid( drag ) && reset.getWorld().getJointCount() == 1, "capture loss removes only mouse joint" );
    session.handleInput( input, true );
    check( !reset.getWorld().IsValid( reset.getMouseJoint() ), "held button after capture loss cannot restart drag" );
    input.mousePressed = true; input.mousePosition = initial.position; session.handleInput( input, true );
    check( reset.getWorld().IsValid( reset.getMouseJoint() ), "fresh press starts new drag" );
    input.mousePressed = false; input.mouseHeld = false; session.handleInput( input, true );
    check( reset.getWorld().getJointCount() == 1, "release removes mouse joint" );
    input = {}; input.mousePressed = true; input.mouseHeld = true; input.mousePosition = { 0.0f, 0.0f }; session.handleInput( input, true );
    check( reset.getWorld().getJointCount() == 1, "static geometry cannot be dragged" );
    input.mousePosition = initial.position; session.handleInput( input, true );
    const auto oldDrag = reset.getMouseJoint(); session.reset();
    auto& fresh = static_cast<rigidBodyDemo&>( session.getDemo() );
    check( !fresh.getWorld().IsValid( oldDrag ) && fresh.getWorld().getJointCount() == 1, "reset rejects old drag ID" );
    session.selectDemo( demoKind::playground );
    auto& pick = static_cast<rigidBodyDemo&>( session.getDemo() );
    input = {}; input.mousePressed = true; input.mouseHeld = true;
    input.mousePosition = pick.getWorld().GetBodyTransform( pick.getTorqueBody() ).position;
    session.handleInput( input, true ); const auto deletedDrag = pick.getMouseJoint();
    check( pick.getWorld().IsValid( deletedDrag ), "polygon picking starts drag" );
    pick.getWorld().DestroyBody( pick.getTorqueBody() ); input.mousePressed = false; session.handleInput( input, true );
    check( !pick.getWorld().IsValid( pick.getMouseJoint() ) && !pick.getWorld().IsValid( deletedDrag ), "deleted grabbed Body clears stale drag" );

    demoSession clock{ createCounter };
    auto& counter = static_cast<counterDemo&>( clock.getDemo() );
    clock.advance( 0.5f, 1 ); check( counter.steps == 0, "paused does not accumulate time" );
    clock.setPlaying( true ); clock.advance( 1.0f / 120.0f, 1 ); check( counter.steps == 0, "fractional frame waits" );
    clock.advance( 1.0f / 120.0f, 1 ); check( counter.steps == 1, "fractional frames accumulate" );
    clock.advance( 1.0f / 120.0f, 1 );
    clock.setPlaying( false ); clock.setPlaying( true );
    clock.advance( 1.0f / 120.0f, 1 ); check( counter.steps == 1, "pause discards old fractional time" );
    clock.advance( 1.0f, 1 ); check( counter.steps == 9, "long frame capped at eight steps" );
    clock.advance( 0.0f, 1 ); check( counter.steps == 9, "catchup remainder discarded" );
    input = {}; input.left = true; clock.handleInput( input, true ); check( counter.held, "active input delivered" );
    clock.handleInput( input, false ); check( !counter.held, "inactive input cancelled" );
    clock.reset(); check( !static_cast<counterDemo&>( clock.getDemo() ).held && clock.getStepCount() == 0, "reset replaces demo state" );
#ifdef ZONAI_TEST_SANDBOX_UI
    checkDemoUi();
#endif
    return EXIT_SUCCESS;
}
