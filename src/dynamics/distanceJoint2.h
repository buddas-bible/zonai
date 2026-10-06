#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

struct distanceJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // 물체 원점 기준의 로컬 작용점. 질량 중심이 이동해도 유지함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    float length = 1.0f; // 고정 거리 또는 스프링의 목표 거리
    bool collideConnected = false;

    // false이면 고정 거리 제약이며 거리 제한을 무시함. true이고 주파수가 0이면 스프링 힘만 끔.
    bool enableSpring = false;
    float hertz = 5.0f; // 스프링 주파수 (Hz)
    float dampingRatio = 0.7f; // 감쇠비. 1이면 임계 감쇠

    // 거리의 하한과 상한. 각 limit의 임펄스는 별도로 누적함.
    bool enableLimit = false;
    // 유한한 비음수, min <= max. World에서 LINEAR_SLOP 이상으로 제한함.
    // 하한과 상한이 같으면 Box2D처럼 length의 고정 거리 제약으로 돌아감.
    float minLength = 0.0f;
    float maxLength = 1.0e5f;

    // 축 방향 속도 모터. 스프링 모드에서만 작동하며 0 Hz에서도 사용할 수 있음.
    bool enableMotor = false;
    float motorSpeed = 0.0f; // 목표 상대속도 (m/s). 양수는 늘이고 음수는 줄임
    float maxMotorForce = 0.0f; // 양방향 최대 힘 (N). 유한한 비음수
};

struct distanceJointData
{
    bodyId bodyA{};
    bodyId bodyB{};

    // 월드 좌표의 작용점.
    vec2 anchorA{};
    vec2 anchorB{};
    float length = 0.0f; // 고정 거리 또는 스프링의 목표 거리
    float currentLength = 0.0f; // 현재 두 작용점 사이의 거리
    bool collideConnected = false;

    // 물리 스프링 설정. 고정 거리 제약의 수치 안정화 계수와 구분함.
    bool enableSpring = false;
    float hertz = 0.0f; // 스프링 주파수 (Hz)
    float dampingRatio = 0.0f; // 감쇠비. 1이면 임계 감쇠
    // 마지막 substep의 합산 임펄스 / 시간 간격. 음수는 인장, 양수는 압축임.
    float axialForce = 0.0f;

    // 거리의 하한과 상한. 각 limit의 임펄스는 별도로 누적함.
    bool enableLimit = false;
    float minLength = 0.0f;
    float maxLength = 0.0f;

    // 축 방향 속도 모터. 스프링 모드에서만 작동하며 0 Hz에서도 사용할 수 있음.
    bool enableMotor = false;
    float motorSpeed = 0.0f; // 목표 상대속도 (m/s). 양수는 늘이고 음수는 줄임
    float maxMotorForce = 0.0f; // 양방향 최대 힘 (N). 유한한 비음수
    float motorForce = 0.0f; // 마지막 substep의 모터 임펄스 / 시간 간격
};

} // namespace zonai
