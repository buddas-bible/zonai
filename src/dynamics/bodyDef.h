#pragma once

#include "dynamics/bodyType.h"
#include "math/transform2.h"
#include "math/vec2.h"

namespace zonai
{

// Body 생성 시 한 번에 전달하는 초기 simulation 설정.
struct bodyDef
{
    bodyType type = bodyType::Static;
    transform2 transform{};

    vec2 linearVelocity{};
    float angularVelocity = 0.0f;

    float linearDamping = 0.0f;
    float angularDamping = 0.0f;
    float gravityScale = 1.0f;

    bool enableSleep = true;
    bool isAwake = true;
    float sleepThreshold = 0.05f;

    // 한 Step의 이동량이 shape 최소 두께의 이 비율을 넘으면 fast body로 분류함.
    float safetyFactor = 0.5f;

    // bullet은 CCD에서 static뿐 아니라 kinematic / dynamic body까지 검사함.
    bool isBullet = false;
    bool allowFastRotation = false;
};

} // namespace zonai
