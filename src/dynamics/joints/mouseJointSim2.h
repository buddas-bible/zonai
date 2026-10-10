#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Mouse Joint의 simulation 상태.
// 월드 target과 Body B의 클릭 지점, spring 설정, 다음 substep warm start에 필요한 impulse를 유지함.
struct mouseJointSim2
{
    // 공용 Joint slot과 연결된 id. A는 연결 수명용 정적 Body, B는 실제로 끌어당기는 Body임.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    vec2 target{}; // 마우스가 끌어당기는 월드 좌표.
    vec2 localAnchorB{}; // 클릭 시점의 Body B origin 기준 로컬 작용점.

    // target과 anchorB를 연결하는 실제 spring-damper 설정.
    float hertz = 5.0f;
    float dampingRatio = 0.7f;
    float maxForce = 1000.0f;

    // 이전 substep의 누적 impulse. 다음 warm start와 현재 반력 조회에 사용함.
    vec2 impulse{};
    float subStepTime = 0.0f; // cached impulse가 계산된 시간 간격.
};

} // namespace zonai
