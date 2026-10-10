#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/moverJointConstraint2.h"
#include "dynamics/joints/pogoJointConstraint2.h"

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
    bodySim bodyA{};
    bodyA.bodyId = 0;

    bodySim bodyB{};
    bodyB.bodyId = 1;
    bodyB.invMass = 1.0f;
    bodyB.invInertia = 1.0f;

    moverJointSim2 joint{};
    joint.jointId = 0;
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    joint.linearVelocity = { 2.0f, -1.0f };
    joint.maxVelocityForce = { 1000.0f, 1000.0f };

    const float h = 1.0f / 60.0f;
    auto constraint = prepareMoverJointConstraint( joint, bodyA, bodyB, h );
    bodyState stateA{};
    bodyState stateB{};

    solveMoverJointConstraint( constraint, stateA, stateB );

    check( near( stateB.linearVelocity.x - stateA.linearVelocity.x, 2.0f ) && near( stateB.linearVelocity.y - stateA.linearVelocity.y, -1.0f ), "Mover reaches desired relative linear velocity" );
    check( near( stateA.angularVelocity, 0.0f ) && near( stateB.angularVelocity, 0.0f ), "Mover does not affect rotation" );

    // 같은 substep h에서는 이전 누적 impulse를 다음 solve의 초기값으로 재사용함.
    moverJointSim2 cached = joint;
    cached.linearVelocityImpulse = { 0.25f, -0.5f };
    cached.subStepTime = h;

    auto warmConstraint = prepareMoverJointConstraint( cached, bodyA, bodyB, h );
    check( near( warmConstraint.linearVelocityImpulse.x, 0.25f ) && near( warmConstraint.linearVelocityImpulse.y, -0.5f ), "Mover reuses cached impulse for the same substep time" );

    bodyState warmStateA{};
    bodyState warmStateB{};
    warmStartMoverJointConstraint( warmConstraint, warmStateA, warmStateB );
    check( near( warmStateB.linearVelocity.x, 0.25f ) && near( warmStateB.linearVelocity.y, -0.5f ), "Mover warm start reapplies cached linear impulse" );
    check( near( warmStateA.angularVelocity, 0.0f ) && near( warmStateB.angularVelocity, 0.0f ), "Mover warm start does not affect rotation" );

    auto changedTimeConstraint = prepareMoverJointConstraint( cached, bodyA, bodyB, 0.5f * h );
    check( near( changedTimeConstraint.linearVelocityImpulse.x, 0.0f ) && near( changedTimeConstraint.linearVelocityImpulse.y, 0.0f ), "Mover discards cached impulse when substep time changes" );

    // Pogo는 spring 길이 오차를 pogo axis로 측정하되 실제 impulse는 contact normal 방향으로 가함.
    bodySim pogoBodyA{};
    pogoBodyA.bodyId = 0;
    bodySim pogoBodyB{};
    pogoBodyB.bodyId = 1;
    pogoBodyB.invMass = 1.0f;
    pogoBodyB.invInertia = 1.0f;

    pogoJointSim2 pogo{};
    pogo.jointId = 1;
    pogo.bodyIdA = 0;
    pogo.bodyIdB = 1;
    pogo.localAnchorA = {};
    pogo.localAnchorB = { 1.0f, 0.0f };
    pogo.localPogoAxisB = { 0.0f, 1.0f };
    pogo.normal = { 0.0f, 1.0f };
    pogo.restLength = 1.0f;
    pogo.hertz = 4.0f;
    pogo.dampingRatio = 0.7f;
    pogo.maxCompressionForce = 1000.0f;
    pogo.maxTensionForce = 1000.0f;

    auto pogoConstraint = preparePogoJointConstraint( pogo, pogoBodyA, pogoBodyB, h );
    bodyState pogoStateA{};
    bodyState pogoStateB{};
    pogoStateB.deltaPosition = { 0.0f, -0.25f };
    solvePogoJointConstraint( pogoConstraint, pogoStateA, pogoStateB, true );

    check( pogoStateB.linearVelocity.y > 0.0f, "Pogo compression pushes body B along the contact normal" );
    check( pogoStateB.angularVelocity > 0.0f, "Off-center Pogo impulse produces physical angular response" );

    return EXIT_SUCCESS;
}
