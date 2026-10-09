#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// 두 작용점과 기준 상대 각도를 함께 유지해 두 Body의 상대 transform을 고정함.
struct weldJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // Body origin 기준의 로컬 작용점. 질량 중심이 바뀌어도 연결 위치를 유지함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    // B의 A에 대한 기준 각도 (rad). 현재 상대각도를 유지하려면 생성 시 그 값을 명시함.
    float referenceAngle = 0.0f;
    bool collideConnected = false;
};

struct weldJointData
{
    bodyId bodyA{};
    bodyId bodyB{};

    // 월드 좌표의 두 작용점. 두 점의 차이가 선형 Weld 오차임.
    vec2 anchorA{};
    vec2 anchorB{};
    // 기준 각도를 뺀 B의 A에 대한 상대 각도 (rad).
    float currentAngle = 0.0f;
    float referenceAngle = 0.0f;
    // 마지막 substep 누적 임펄스 / 시간 간격. B에 작용하는 반력과 반토크.
    vec2 force{};
    float torque = 0.0f;
    bool collideConnected = false;
};

} // namespace zonai
