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

    bool allowFastRotation = false;
};

} // namespace zonai
