#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// Mover Joint 생성 시 World에 전달하는 입력 설정.
// 두 Body의 COM 상대 선속도만 제어하며 회전에는 어떤 제약이나 torque도 만들지 않음.
struct moverJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // World 좌표계에서 B가 A에 대해 가져야 하는 목표 상대 선속도.
    vec2 linearVelocity{};

    // x/y 방향별 최대 구동 힘. 서로 독립적으로 제한해서 축마다 다른 가속 능력을 줄 수 있음.
    vec2 maxVelocityForce{};

    bool collideConnected = false;
};

// getMoverJointData()가 반환하는 현재 Mover Joint 상태 snapshot.
struct moverJointData
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 linearVelocity{};
    vec2 maxVelocityForce{};

    // 마지막 substep의 누적 impulse / h. Body B에 작용한 Mover의 실제 force임.
    vec2 force{};

    bool collideConnected = false;
};

} // namespace zonai
