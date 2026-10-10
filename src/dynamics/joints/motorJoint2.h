#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// Motor Joint 생성 시 World에 전달하는 입력 설정.
// 두 Body의 상대 위치를 고정하지 않고 작용점의 상대 선속도와 상대 각속도를 목표값으로 제어함.
struct motorJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // Body origin 기준의 로컬 작용점. 중심 밖에 있으면 선형 motor impulse가 회전에도 영향을 줌.
    vec2 localAnchorA{};
    vec2 localAnchorB{};

    // World 좌표계에서 B 작용점이 A 작용점에 대해 가져야 하는 목표 상대 선속도와 최대 힘.
    // 목표속도 0 + 유한한 힘은 위치를 잠그는 것이 아니라 선형 brake / friction처럼 동작함.
    vec2 linearVelocity{};
    float maxVelocityForce = 0.0f;

    // B가 A에 대해 가져야 하는 목표 상대 각속도와 최대 토크.
    // 목표속도 0 + 유한한 토크는 회전을 잠그지 않고 회전 brake처럼 동작함.
    float angularVelocity = 0.0f;
    float maxVelocityTorque = 0.0f;

    bool collideConnected = false;
};

// getMotorJointData()가 반환하는 현재 Motor Joint 상태 snapshot.
// World 내부의 persistent storage가 아니므로 조회 이후 simulation이 진행되어도 자동 갱신되지 않음.
struct motorJointData
{
    bodyId bodyA{};
    bodyId bodyB{};

    // 현재 Body transform으로 계산한 두 월드 작용점.
    vec2 anchorA{};
    vec2 anchorB{};

    // linearVelocity는 생성/설정 때와 같은 World 좌표계의 목표 상대속도임.
    vec2 linearVelocity{};
    float maxVelocityForce = 0.0f;
    float angularVelocity = 0.0f;
    float maxVelocityTorque = 0.0f;

    // 마지막 substep 누적 impulse / h. B에 작용한 motor의 실제 force / torque임.
    vec2 force{};
    float torque = 0.0f;

    bool collideConnected = false;
};

} // namespace zonai
