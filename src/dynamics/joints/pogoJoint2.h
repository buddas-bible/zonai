#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// Pogo Joint 생성 시 World에 전달하는 설정.
// 길이 오차는 B의 로컬 pogo 축으로 측정하고 실제 반력은 world-space contact normal 방향으로 가함.
struct pogoJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // Body origin 기준 작용점. 보통 A는 지면 hit point, B는 character 하단점임.
    vec2 localAnchorA{};
    vec2 localAnchorB{};

    // B와 함께 회전하는 길이 측정 축. World 생성 시 단위 벡터로 정규화함.
    vec2 localPogoAxisB{ 0.0f, 1.0f };

    // 실제 impulse가 작용하는 world-space contact normal. World 생성 시 단위 벡터로 정규화함.
    vec2 normal{ 0.0f, 1.0f };

    float restLength = 0.0f;
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    float maxTensionForce = 0.0f;
    float maxCompressionForce = 0.0f;

    // Pogo는 지면 Body가 바뀌면 매 frame 재생성될 수 있으므로 이전 내부 상태를 새 Joint에 넘길 수 있게 함.
    float impulse = 0.0f;
    float velocity = 0.0f;

    bool collideConnected = true;
};

// getPogoJointData()가 반환하는 현재 Pogo 상태 snapshot.
struct pogoJointData
{
    bodyId bodyA{};
    bodyId bodyB{};

    vec2 anchorA{};
    vec2 anchorB{};
    vec2 pogoAxis{};
    vec2 normal{};

    float length = 0.0f;
    float restLength = 0.0f;
    float hertz = 0.0f;
    float dampingRatio = 0.0f;
    float maxTensionForce = 0.0f;
    float maxCompressionForce = 0.0f;

    // 마지막 substep에서 누적된 normal impulse / h와 Pogo의 내부 spring velocity.
    vec2 force{};
    float impulse = 0.0f;
    float velocity = 0.0f;

    bool collideConnected = true;
};

} // namespace zonai
