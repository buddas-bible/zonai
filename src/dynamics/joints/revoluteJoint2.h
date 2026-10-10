#pragma once

#include "dynamics/id.h"
#include "math/vec2.h"

namespace zonai
{

// Revolute Joint를 생성할 때 World에 전달하는 설정.
// 두 작용점을 일치시키고 상대 회전은 허용하며, 필요하면 각도 limit과 회전 motor를 추가함.
struct revoluteJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    // 물체 원점 기준의 로컬 작용점. 질량 중심이 바뀌어도 연결 위치를 유지함.
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    bool collideConnected = false;

    // B의 A에 대한 기준 각도 (rad). 0이면 기존 상대 각도와 같으며 회전 횟수를 누적하지 않음.
    float referenceAngle = 0.0f;
    bool enableLimit = false;
    // 유한한 lower <= upper. World에서 ±0.99*pi로 제한하며 같으면 해당 각도를 유지함.
    float lowerAngle = 0.0f;
    float upperAngle = 0.0f;
    bool enableMotor = false;
    float motorSpeed = 0.0f; // B의 A에 대한 목표 각속도 (rad/s). 0이면 제동함.
    float maxMotorTorque = 0.0f; // 유한한 비음수 토크 한도 (N*m). 0이면 모터 힘이 없음.
};

// getRevoluteJointData()가 반환하는 현재 상태 snapshot.
// 현재 축 위치/상대 각도와 마지막 substep의 반력·limit torque·motor torque를 조회할 때 사용함.
struct revoluteJointData
{
    bodyId bodyA{};
    bodyId bodyB{};

    // 월드 좌표의 작용점. 두 점의 차이가 회전축의 위치 오차임.
    vec2 anchorA{};
    vec2 anchorB{};
    // 기준 각도를 뺀 B의 A에 대한 상대 각도 (rad). -pi부터 pi까지이며 회전 횟수를 누적하지 않음.
    float currentAngle = 0.0f;
    // 마지막 substep의 누적 임펄스 / 시간 간격. B에 작용하는 반력 (N).
    vec2 force{};
    bool collideConnected = false;

    float referenceAngle = 0.0f;
    bool enableLimit = false;
    float lowerAngle = 0.0f;
    float upperAngle = 0.0f;
    // 마지막 substep의 (lower - upper) 임펄스 / 시간 간격. B에 작용하는 제한 토크 (N*m).
    float torque = 0.0f;
    bool enableMotor = false;
    float motorSpeed = 0.0f;
    float maxMotorTorque = 0.0f;
    // 마지막 substep의 모터 임펄스 / 시간 간격. B에 작용하는 모터 토크 (N*m).
    float motorTorque = 0.0f;
};

} // namespace zonai
