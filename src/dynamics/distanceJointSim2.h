#pragma once

#include <cstdint>
#include "math/vec2.h"

namespace zonai
{
// World의 조인트 슬롯과 같은 인덱스에 보관함. 누적 임펄스를 다음 step까지 유지함.

struct distanceJointSim2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;

    // 물체 원점 기준의 로컬 작용점. 질량 중심이 이동해도 유지함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    float length = 1.0f; // 고정 거리 또는 스프링의 목표 거리
    float impulse = 0.0f;
    float subStepTime = 0.0f; // 누적 임펄스를 구한 시간 간격

    // 물리 스프링 설정. 고정 거리 제약의 수치 안정화 계수와 구분함.
    bool enableSpring = false;
    float hertz = 5.0f; // 스프링 주파수 (Hz)
    float dampingRatio = 0.7f; // 감쇠비. 1이면 임계 감쇠

    // 거리의 하한과 상한. 각 limit의 임펄스는 별도로 누적함.
    bool enableLimit = false;
    float minLength = 0.0f;
    float maxLength = 1.0e5f;
    float lowerImpulse = 0.0f;
    float upperImpulse = 0.0f;

    // 축 방향 속도 모터. 스프링 모드에서만 작동하며 0 Hz에서도 사용할 수 있음.
    bool enableMotor = false;
    float motorSpeed = 0.0f; // 목표 상대속도 (m/s). 양수는 늘이고 음수는 줄임
    float maxMotorForce = 0.0f; // 양방향 최대 힘 (N). 유한한 비음수
    float motorImpulse = 0.0f;
};

} // namespace zonai
