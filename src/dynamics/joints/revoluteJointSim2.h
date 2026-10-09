#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Revolute Joint의 simulation 상태.
// 기준 관계와 limit/motor 설정, 다음 substep warm start에 필요한 누적 impulse를 유지함.
struct revoluteJointSim2
{
    // 공용 Joint slot과 연결된 id, 그리고 두 Body의 simulation index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Body origin 기준의 로컬 회전축 작용점. Prepare에서 COM 기준 lever arm으로 변환함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};

    // 두 작용점을 일치시키는 2D 선형 제약의 이전 누적 impulse.
    vec2 impulse{};
    float subStepTime = 0.0f; // warm-start cache가 계산된 시간 간격.

    // B가 A에 대해 허용되는 상대 각도 범위와 각 한쪽 limit의 warm-start cache.
    float referenceAngle = 0.0f;
    bool enableLimit = false;
    float lowerAngle = 0.0f;
    float upperAngle = 0.0f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    // 상대 각속도를 만드는 모터 설정과 warm-start cache.
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorTorque = 0.0f;
    float motorImpulse = 0.0f;
};

} // namespace zonai
