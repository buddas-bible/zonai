#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// 두 작용점을 일치시키고 상대 회전은 허용하는 기본 회전축.
struct revoluteJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // 물체 원점 기준의 로컬 작용점. 질량 중심이 바뀌어도 연결 위치를 유지함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    bool collideConnected = false;
};

struct revoluteJointData
{
    bodyId bodyA{};
    bodyId bodyB{};

    // 월드 좌표의 작용점. 두 점의 차이가 회전축의 위치 오차임.
    vec2 anchorA{};
    vec2 anchorB{};
    // B의 A에 대한 상대 각도 (rad). -pi부터 pi까지이며 회전 횟수를 누적하지 않음.
    float currentAngle = 0.0f;
    // 마지막 substep의 누적 임펄스 / 시간 간격. B에 작용하는 반력 (N).
    vec2 force{};
    bool collideConnected = false;
};

} // namespace zonai
