#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{
// World의 Joint cold slot과 같은 index에 저장하는 persistent solver 데이터.
#pragma region Simulation

struct distanceJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Body origin 기준 local anchor. COM 이동과 분리해서 보관함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    float length = 1.0f;
    float impulse = 0.0f;
    float subStepTime = 0.0f;

    // Spring 설정. Rigid 제약의 수치 안정화 계수와 구분함.
    bool enableSpring = false;
    float hertz = 5.0f;
    float dampingRatio = 0.7f;

    // 거리의 하한과 상한. 각 limit의 임펄스는 별도로 누적함.
    bool enableLimit = false;
    float minLength = 0.0f;
    float maxLength = 1.0e5f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;
};

#pragma endregion
} // namespace zonai
