#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// Mouse Joint를 생성할 때 World에 전달하는 설정.
// A는 수명과 연결 관계를 유지하는 정적 Body이고 target은 월드 좌표이며, B의 클릭 지점을 spring으로 target에 끌어당김.
struct mouseJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 target{};
    float hertz = 5.0f;        // 스프링 주파수 (Hz)
    float dampingRatio = 0.7f; // 감쇠비. 1이면 임계 감쇠
    float maxForce = 1000.0f;  // 허용하는 힘의 최대 크기
};

// getMouseJointData()가 반환하는 현재 상태 snapshot.
// 월드 target과 현재 Body B 작용점, 마지막 substep의 반력 및 spring 설정을 조회할 때 사용함.
struct mouseJointData
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 target{};
    vec2 anchorB{};
    vec2 force{};
    float hertz = 0.0f;        // 스프링 주파수 (Hz)
    float dampingRatio = 0.0f; // 감쇠비. 1이면 임계 감쇠
    float maxForce = 0.0f;     // 허용하는 힘의 최대 크기
};

} // namespace zonai
