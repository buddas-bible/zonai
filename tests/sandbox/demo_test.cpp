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
    check( getDemoEntries().size() == 11, "eleven independently selectable demo entries" );
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
    check( reset.getWorld().GetBodyLinearVelocity( reset.getPendulumBody() ).y > 0.0f, "pendulum jump impulse" );
    input = {};
    input.torquePressed = true;
    session.handleInput( input, true );
    check( reset.getWorld().GetBodyAngularVelocity( reset.getPendulumBody() ) > 0.0f, "pendulum torque impulse" );

#ifdef ZONAI_TEST_SANDBOX_UI
    checkDemoUi();
#endif

    return EXIT_SUCCESS;
}
