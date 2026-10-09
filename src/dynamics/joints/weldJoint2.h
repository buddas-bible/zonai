#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// Weld Joint를 생성할 때 World에 전달하는 설정.
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

    // 기존 positional aggregate 초기화 호환성을 위해 새 tuning 필드는 기존 필드 뒤에 추가함.
    // 0 Hz는 해당 방향을 hard Weld로 유지하고, 양수 Hz는 독립적인 spring-damper Weld로 만듦.
    float linearHertz = 0.0f;
    float linearDampingRatio = 0.0f;
    float angularHertz = 0.0f;
    float angularDampingRatio = 0.0f;
};

// getWeldJointData()가 반환하는 현재 상태 snapshot.
// 현재 상대 transform 오차와 마지막 substep의 반력/반토크, linear/angular softness 설정을 조회할 때 사용함.
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

    float linearHertz = 0.0f;
    float linearDampingRatio = 0.0f;
    float angularHertz = 0.0f;
    float angularDampingRatio = 0.0f;

    // 마지막 substep 누적 임펄스 / 시간 간격. B에 작용하는 반력과 반토크.
    vec2 force{};
    float torque = 0.0f;
    bool collideConnected = false;
};

} // namespace zonai
