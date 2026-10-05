#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/constraintSoftness2.h"
#include "dynamics/distanceJointSim2.h"

namespace zonai
{
struct distanceJointConstraint2
{
    std::int32_t jointId = -1;
    std::int32_t bodyIdA = -1;
    std::int32_t bodyIdB = -1;
    vec2 anchorA{};
    vec2 anchorB{};
    vec2 deltaCenter{};
    float invMassA = 0.0f;
    float invMassB = 0.0f;
    float invInertiaA = 0.0f;
    float invInertiaB = 0.0f;
    float length = 1.0f;
    float axialMass = 0.0f;
    float impulse = 0.0f;
    constraintSoftness2 softness{};
};

[[nodiscard]] distanceJointConstraint2 prepareDistanceJointConstraint( const distanceJointSim2& joint, const bodySim& bodyA, const bodySim& bodyB, float subStepTime );
void warmStartDistanceJointConstraint( const distanceJointConstraint2& constraint, bodyState& stateA, bodyState& stateB );
void solveDistanceJointConstraint( distanceJointConstraint2& constraint, bodyState& stateA, bodyState& stateB, bool useBias );
} // namespace zonai
