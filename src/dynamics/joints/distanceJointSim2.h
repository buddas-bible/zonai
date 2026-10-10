#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{

// World의 stable Joint slot에 계속 보관되는 Distance Joint의 simulation 상태.
// 생성 설정과 다음 substep의 warm start에 사용할 누적 impulse를 함께 유지함.
struct distanceJointSim2
{
    // 공용 Joint slot과 연결된 id, 그리고 두 Body의 simulation index.
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // Body origin 기준의 로컬 작용점. Prepare에서 현재 COM 기준 lever arm으로 변환함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};

    float length = 1.0f; // 고정 거리 또는 스프링의 목표 거리.

    // 기본 거리 제약의 이전 substep 누적 impulse와 그것을 계산한 시간 간격.
    // 같은 substepTime일 때만 다음 Prepare의 warm-start 초기값으로 재사용함.
    float impulse = 0.0f;
    float subStepTime = 0.0f;

    // 물리 스프링 설정. 고정 거리 제약의 수치 안정화 계수와 구분함.
    bool enableSpring = false;
    float hertz = 5.0f;        // 스프링 주파수 (Hz).
    float dampingRatio = 0.7f; // 감쇠비. 1이면 임계 감쇠.

    // 거리의 하한/상한 설정과 각각의 warm-start cache.
    bool enableLimit = false;
    float minLength = 0.0f;
    float maxLength = 1.0e5f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    // 축 방향 속도 모터 설정과 warm-start cache.
    // 스프링 모드에서만 작동하며 0 Hz에서도 사용할 수 있음.
    bool enableMotor = false;
    float motorSpeed = 0.0f;    // 목표 상대속도 (m/s). 양수는 늘이고 음수는 줄임.
    float maxMotorForce = 0.0f; // 양방향 최대 힘 (N). 유한한 비음수.
    float motorImpulse = 0.0f;
};

} // namespace zonai
