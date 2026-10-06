#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/mouseJointSim2.h"

namespace zonai
{
struct mouseJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdB = -1;
    vec2 anchorB{};
    vec2 deltaCenter{};
    vec2 massX{};
    vec2 massY{};
    float invMass = 0.0f;
    float invInertia = 0.0f;
    float maxImpulse = 0.0f;
    vec2 impulse{};
    constraintSoftness2 softness{};
};

[[nodiscard]] mouseJointConstraint2 prepareMouseJointConstraint( const mouseJointSim2& joint, const bodySim& body, float subStepTime );
void warmStartMouseJointConstraint( const mouseJointConstraint2& constraint, bodyState& state );
void solveMouseJointConstraint( mouseJointConstraint2& constraint, bodyState& state );
} // namespace zonai
