#include <cmath>
#include <cstdio>
#include <cstdlib>
#include "dynamics/joints/motorJointConstraint2.h"

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
    const float h = 1.0f / 60.0f;

    // Spring의 force / torque 한도는 velocity motor와 마찬가지로 한 substep의 impulse 한도로 변환됨.
    bodySim limitBodyA{};
    limitBodyA.bodyId = 0;
    bodySim limitBodyB{};
    limitBodyB.bodyId = 1;
    limitBodyB.center = { 10.0f, 0.0f };
    limitBodyB.transform.rotation = rot2::FromRadians( 1.0f );
    limitBodyB.invMass = 1.0f;
    limitBodyB.invInertia = 1.0f;

    motorJointSim2 limited{};
    limited.jointId = 0;
    limited.bodyIdA = 0;
    limited.bodyIdB = 1;
    limited.linearHertz = 4.0f;
    limited.linearDampingRatio = 0.7f;
    limited.maxSpringForce = 6.0f;
    limited.angularHertz = 4.0f;
    limited.angularDampingRatio = 0.7f;
    limited.maxSpringTorque = 12.0f;

    auto limitConstraint = prepareMotorJointConstraint( limited, limitBodyA, limitBodyB, h );
    bodyState limitStateA{};
    bodyState limitStateB{};
    solveMotorJointConstraint( limitConstraint, limitStateA, limitStateB );
    check( near( Length( limitConstraint.linearSpringImpulse ), limited.maxSpringForce * h ), "linear spring impulse is limited by max spring force times h" );
    check( near( std::abs( limitConstraint.angularSpringImpulse ), limited.maxSpringTorque * h ), "angular spring impulse is limited by max spring torque times h" );

    // 같은 timestep에서만 네 종류의 누적 impulse를 재사용하고 Warm Start에서는 방향별로 합쳐 적용함.
    bodySim cacheBodyA{};
    cacheBodyA.bodyId = 2;
    cacheBodyA.invMass = 1.0f;
    cacheBodyA.invInertia = 1.0f;
    bodySim cacheBodyB{};
    cacheBodyB.bodyId = 3;
    cacheBodyB.invMass = 1.0f;
    cacheBodyB.invInertia = 1.0f;

    motorJointSim2 cached{};
    cached.jointId = 1;
    cached.bodyIdA = 2;
    cached.bodyIdB = 3;
    cached.maxVelocityForce = 100.0f;
    cached.maxSpringForce = 100.0f;
    cached.maxVelocityTorque = 100.0f;
    cached.maxSpringTorque = 100.0f;
    cached.linearVelocityImpulse = { 0.1f, -0.2f };
    cached.linearSpringImpulse = { 0.3f, 0.4f };
    cached.angularVelocityImpulse = 0.5f;
    cached.angularSpringImpulse = 0.6f;
    cached.subStepTime = h;

    auto cachedConstraint = prepareMotorJointConstraint( cached, cacheBodyA, cacheBodyB, h );
    check( near( cachedConstraint.linearVelocityImpulse.x, 0.1f ) && near( cachedConstraint.linearSpringImpulse.y, 0.4f ), "same timestep preserves separate linear Motor caches" );
    check( near( cachedConstraint.angularVelocityImpulse, 0.5f ) && near( cachedConstraint.angularSpringImpulse, 0.6f ), "same timestep preserves separate angular Motor caches" );

    bodyState cacheStateA{};
    bodyState cacheStateB{};
    warmStartMotorJointConstraint( cachedConstraint, cacheStateA, cacheStateB );
    check( near( cacheStateB.linearVelocity.x, 0.4f ) && near( cacheStateB.linearVelocity.y, 0.2f ), "warm start applies velocity and spring linear impulses together" );
    check( near( cacheStateB.angularVelocity, 1.1f ) && near( cacheStateA.angularVelocity, -1.1f ), "warm start applies velocity and spring angular impulses together" );

    const auto changedStepConstraint = prepareMotorJointConstraint( cached, cacheBodyA, cacheBodyB, 0.5f * h );
    check( LengthSquared( changedStepConstraint.linearVelocityImpulse ) == 0.0f && LengthSquared( changedStepConstraint.linearSpringImpulse ) == 0.0f, "changed timestep discards linear Motor warm-start caches" );
    check( changedStepConstraint.angularVelocityImpulse == 0.0f && changedStepConstraint.angularSpringImpulse == 0.0f, "changed timestep discards angular Motor warm-start caches" );

    // Velocity Motor와 transform spring은 동시에 켜져도 서로의 누적 해를 덮어쓰지 않음.
    bodySim coexistBodyA{};
    coexistBodyA.bodyId = 4;
    bodySim coexistBodyB{};
    coexistBodyB.bodyId = 5;
    coexistBodyB.center = { 2.0f, 0.0f };
    coexistBodyB.transform.rotation = rot2::FromRadians( 0.5f );
    coexistBodyB.invMass = 1.0f;
    coexistBodyB.invInertia = 1.0f;

    motorJointSim2 coexist{};
    coexist.jointId = 2;
    coexist.bodyIdA = 4;
    coexist.bodyIdB = 5;
    coexist.linearVelocity = { 1.0f, 0.0f };
    coexist.maxVelocityForce = 1000.0f;
    coexist.linearHertz = 4.0f;
    coexist.linearDampingRatio = 0.7f;
    coexist.maxSpringForce = 1000.0f;
    coexist.angularVelocity = 1.0f;
    coexist.maxVelocityTorque = 1000.0f;
    coexist.angularHertz = 4.0f;
    coexist.angularDampingRatio = 0.7f;
    coexist.maxSpringTorque = 1000.0f;

    auto coexistConstraint = prepareMotorJointConstraint( coexist, coexistBodyA, coexistBodyB, h );
    bodyState coexistStateA{};
    bodyState coexistStateB{};
    solveMotorJointConstraint( coexistConstraint, coexistStateA, coexistStateB );
    check( LengthSquared( coexistConstraint.linearVelocityImpulse ) > 0.0f && LengthSquared( coexistConstraint.linearSpringImpulse ) > 0.0f, "linear velocity and spring channels accumulate independently" );
    check( std::abs( coexistConstraint.angularVelocityImpulse ) > 0.0f && std::abs( coexistConstraint.angularSpringImpulse ) > 0.0f, "angular velocity and spring channels accumulate independently" );

    // COM 밖의 선형 spring은 실제 r x P 토크를 만들기 때문에 angular spring과 물리적으로 경쟁할 수 있음.
    // 따라서 매 substep마다 두 오차가 각각 단조 감소해야 한다고 강제하지 않고, 각 actuator가 자기 오차에 반대되는 복원 impulse를 만드는지 확인함.
    bodySim coupledBodyA{};
    coupledBodyA.bodyId = 6;
    bodySim coupledBodyB{};
    coupledBodyB.bodyId = 7;
    coupledBodyB.center = { 2.0f, 0.0f };
    coupledBodyB.transform.rotation = rot2::FromRadians( 0.5f );
    coupledBodyB.invMass = 1.0f;
    coupledBodyB.invInertia = 1.0f;

    motorJointSim2 coupled{};
    coupled.jointId = 3;
    coupled.bodyIdA = 6;
    coupled.bodyIdB = 7;
    coupled.localAnchorB = { 0.0f, 1.0f };
    coupled.linearHertz = 4.0f;
    coupled.linearDampingRatio = 0.7f;
    coupled.maxSpringForce = 1000.0f;
    coupled.angularHertz = 4.0f;
    coupled.angularDampingRatio = 0.7f;
    coupled.maxSpringTorque = 1000.0f;

    auto coupledConstraint = prepareMotorJointConstraint( coupled, coupledBodyA, coupledBodyB, h );
    bodyState coupledStateA{};
    bodyState coupledStateB{};
    solveMotorJointConstraint( coupledConstraint, coupledStateA, coupledStateB );
    solveMotorJointConstraint( coupledConstraint, coupledStateA, coupledStateB );

    const vec2 initialSeparation = coupledConstraint.deltaCenter + coupledConstraint.anchorB - coupledConstraint.anchorA;
    check( Dot( coupledConstraint.linearSpringImpulse, initialSeparation ) < 0.0f, "off-center linear spring impulse opposes anchor separation" );
    check( coupledConstraint.angularSpringImpulse < 0.0f, "off-center angular spring impulse opposes positive relative angle" );
    check( IsFinite( coupledStateB.linearVelocity ) && std::isfinite( coupledStateB.angularVelocity ), "off-center combined spring response stays finite" );

    return EXIT_SUCCESS;
}
