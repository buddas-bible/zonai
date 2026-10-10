#include <cmath>
#include <cstdio>
#include <cstdlib>

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

bool near( float a, float b, float epsilon = 0.0001f )
{
    return std::abs( a - b ) < epsilon;
}

pogoJointSim2 makePogo()
{
    pogoJointSim2 joint{};
    joint.jointId = 0;
    joint.bodyIdA = 0;
    joint.bodyIdB = 1;
    joint.localAnchorA = {};
    joint.localAnchorB = { 1.0f, 0.0f };
    joint.localPogoAxisB = { 0.0f, 1.0f };
    joint.normal = { 0.0f, 1.0f };
    joint.restLength = 1.0f;
    joint.hertz = 4.0f;
    joint.dampingRatio = 0.7f;
    joint.maxTensionForce = 1000.0f;
    joint.maxCompressionForce = 1000.0f;
    return joint;
}

}

int main()
{
    constexpr float h = 1.0f / 60.0f;

    bodySim bodyA{};
    bodyA.bodyId = 0;

    bodySim bodyB{};
    bodyB.bodyId = 1;
    bodyB.invMass = 1.0f;
    bodyB.invInertia = 1.0f;

    // 압축된 Pogo는 contact normal 방향으로 B를 밀고, 중심 밖 작용점이면 실제 회전도 함께 만듦.
    {
        pogoJointSim2 joint = makePogo();
        auto constraint = preparePogoJointConstraint( joint, bodyA, bodyB, h );
        bodyState stateA{};
        bodyState stateB{};
        stateB.deltaPosition = { 0.0f, -0.25f };

        solvePogoJointConstraint( constraint, stateA, stateB, true );

        check( stateB.linearVelocity.y > 0.0f, "Pogo compression pushes body B along the contact normal" );
        check( stateB.angularVelocity > 0.0f, "Off-center Pogo impulse produces physical angular response" );
        check( constraint.impulse > 0.0f, "Pogo compression accumulates positive normal impulse" );
    }

    // 길이 측정 축과 힘 방향은 독립적임. x 길이 오차를 측정해도 y normal로 반력을 가할 수 있음.
    {
        pogoJointSim2 joint = makePogo();
        joint.localAnchorB = {};
        joint.localPogoAxisB = { 1.0f, 0.0f };
        joint.normal = { 0.0f, 1.0f };
        joint.restLength = 1.0f;

        auto constraint = preparePogoJointConstraint( joint, bodyA, bodyB, h );
        bodyState stateA{};
        bodyState stateB{};
        stateB.deltaPosition = { 0.5f, 0.0f };

        solvePogoJointConstraint( constraint, stateA, stateB, true );
        check( near( stateB.linearVelocity.x, 0.0f ), "Pogo does not apply impulse along its measurement axis when normal differs" );
        check( stateB.linearVelocity.y > 0.0f, "Pogo applies correction along the contact normal" );
    }

    // 인장과 압축 한도는 서로 다른 force * h 범위로 누적 impulse를 제한함.
    {
        pogoJointSim2 joint = makePogo();
        joint.localAnchorB = {};
        joint.maxCompressionForce = 6.0f;
        joint.maxTensionForce = 3.0f;

        auto compression = preparePogoJointConstraint( joint, bodyA, bodyB, h );
        bodyState stateA{};
        bodyState stateB{};
        stateB.deltaPosition = { 0.0f, -0.75f };
        solvePogoJointConstraint( compression, stateA, stateB, true );
        check( compression.impulse <= 6.0f * h + 0.0001f && compression.impulse >= 0.0f, "Pogo clamps compression impulse by maxCompressionForce * h" );

        auto tension = preparePogoJointConstraint( joint, bodyA, bodyB, h );
        stateA = {};
        stateB = {};
        stateB.deltaPosition = { 0.0f, 1.5f };
        solvePogoJointConstraint( tension, stateA, stateB, true );
        check( tension.impulse >= -3.0f * h - 0.0001f && tension.impulse <= 0.0f, "Pogo clamps tension impulse by maxTensionForce * h" );
    }

    // 새로 재생성된 Pogo는 이전 hit에서 넘긴 state를 seed로 사용하고, 기존 Pogo는 같은 h에서만 impulse를 warm start함.
    {
        pogoJointSim2 seeded = makePogo();
        seeded.impulse = 0.2f;
        seeded.velocity = -0.4f;
        seeded.subStepTime = 0.0f;
        auto recreated = preparePogoJointConstraint( seeded, bodyA, bodyB, h );
        check( near( recreated.impulse, 0.2f ) && near( recreated.velocity, -0.4f ), "Recreated Pogo accepts explicit impulse and spring-velocity seed" );

        seeded.subStepTime = h;
        auto sameTime = preparePogoJointConstraint( seeded, bodyA, bodyB, h );
        check( near( sameTime.impulse, 0.2f ), "Pogo reuses cached impulse for the same substep time" );

        auto changedTime = preparePogoJointConstraint( seeded, bodyA, bodyB, 0.5f * h );
        check( near( changedTime.impulse, 0.0f ), "Existing Pogo discards cached impulse when substep time changes" );
        check( near( changedTime.velocity, -0.4f ), "Pogo preserves spring state velocity across timestep changes" );
    }

    // Warm start는 normal impulse와 그 작용점 모멘트를 그대로 재적용함.
    {
        pogoJointSim2 joint = makePogo();
        joint.impulse = 0.15f;
        joint.subStepTime = h;
        auto constraint = preparePogoJointConstraint( joint, bodyA, bodyB, h );
        bodyState stateA{};
        bodyState stateB{};
        warmStartPogoJointConstraint( constraint, stateA, stateB );
        check( near( stateB.linearVelocity.y, 0.15f ), "Pogo warm start reapplies cached normal impulse" );
        check( near( stateB.angularVelocity, 0.15f ), "Pogo warm start reapplies off-center angular response" );
    }

    // 0 Hz는 Pogo를 비활성화하며 seed와 solve impulse를 사용하지 않음.
    {
        pogoJointSim2 joint = makePogo();
        joint.hertz = 0.0f;
        joint.impulse = 0.5f;
        joint.velocity = 1.0f;
        auto constraint = preparePogoJointConstraint( joint, bodyA, bodyB, h );
        bodyState stateA{};
        bodyState stateB{};
        stateB.deltaPosition = { 0.0f, -1.0f };
        solvePogoJointConstraint( constraint, stateA, stateB, true );
        check( near( constraint.impulse, 0.0f ) && near( stateB.linearVelocity.y, 0.0f ), "Zero-Hz Pogo stays inactive" );
    }

    // Bias pass가 만든 위치 복원 속도는 relaxation에서 실제 persistent 속도로 남지 않아야 함.
    {
        pogoJointSim2 joint = makePogo();
        joint.localAnchorB = {};
        auto constraint = preparePogoJointConstraint( joint, bodyA, bodyB, h );
        bodyState stateA{};
        bodyState stateB{};
        stateB.deltaPosition = { 0.0f, -0.25f };

        solvePogoJointConstraint( constraint, stateA, stateB, true );
        check( stateB.linearVelocity.y > 0.0f, "Pogo bias pass creates temporary recovery velocity" );
        solvePogoJointConstraint( constraint, stateA, stateB, false );
        check( near( stateB.linearVelocity.y, 0.0f, 0.001f ), "Pogo relaxation removes temporary recovery velocity" );
    }

    // Box2D Pogo는 한 substep 동안 Prepare 시점의 pogo 축을 고정함.
    // Body B의 deltaRotation은 anchor lever arm에는 반영하지만 길이 측정축에는 다시 곱하지 않음.
    {
        pogoJointSim2 joint = makePogo();
        joint.localAnchorB = {};
        joint.restLength = 0.0f;
        auto constraint = preparePogoJointConstraint( joint, bodyA, bodyB, h );
        bodyState stateA{};
        bodyState stateB{};
        stateB.deltaPosition = { 0.0f, 1.0f };
        stateB.deltaRotation = rot2::FromRadians( 0.5f * 3.14159265358979323846f );

        solvePogoJointConstraint( constraint, stateA, stateB, true );
        check( stateB.linearVelocity.y < 0.0f, "Pogo keeps the prepared measurement axis fixed within the substep" );
    }

    return EXIT_SUCCESS;
}
