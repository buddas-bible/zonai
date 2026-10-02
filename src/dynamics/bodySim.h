#pragma once

#include <cstdint>
#include <limits>

#include "math/transform2.h"

namespace zonai
{

// Solver와 collision 준비에서 자주 사용하는 body의 simulation 상태.
// solver set이 도입되기 전까지는 World가 body와 같은 stable slot index로 보관함.
struct bodySim
{
    static constexpr std::int32_t NULL_INDEX = -1;

    // body origin의 world transform.
    transform2 transform{};

    // body local space의 center of mass.
    vec2 localCenter{};

    // center of mass의 world-space 위치.
    vec2 center{};

    // 한 simulation step 동안 누적되는 외력 / 토크.
    // Step에서 velocity에 반영한 뒤 0으로 초기화함.
    vec2 force{};
    float torque = 0.0f;

    // Solver에서 곱셈으로 사용하기 위한 역질량 / 역관성.
    float invMass = 0.0f;
    float invInertia = 0.0f;

    // velocity integration에서 사용하는 감쇠 계수.
    // 0이면 감쇠하지 않고 값이 클수록 현재 속도를 더 빠르게 줄임.
    float linearDamping = 0.0f;
    float angularDamping = 0.0f;

    // world gravity에 곱하는 body별 배율.
    // Kinematic / Static은 invMass가 0이므로 solver에서 gravity를 적용하지 않음.
    float gravityScale = 1.0f;

    // bullet은 이후 continuous collision pass에서 더 넓은 body type을 검사함.
    bool isBullet = false;

    // 이번 Step에서 CCD가 필요한 속도로 움직였는지 나타내는 transient 상태.
    bool isFast = false;

    // 이번 Step에서 TOI가 실제 이동 fraction을 줄였는지 나타내는 debug 상태.
    bool hadTimeOfImpact = false;

    // true면 MAX_ROTATION 기반 각속도 제한을 적용하지 않음.
    // 원형 바퀴처럼 빠른 회전이 안전한 body에만 사용함.
    bool allowFastRotation = false;

    // CCD 기준이 되는 shape의 최소 두께와
    // 가장 먼 body point의 속도를 계산하기 위한 최대 반경.
    float minExtent = std::numeric_limits<float>::max();
    float maxExtent = 0.0f;

    // 이 simulation 데이터가 대응하는 World 내부 body index.
    // NULL_INDEX면 현재 free slot임.
    std::int32_t bodyId = NULL_INDEX;
};

} // namespace zonai
