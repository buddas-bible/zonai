#pragma once

#include "dynamics/bodySim.h"
#include "dynamics/bodyState.h"
#include "dynamics/joints/splineRailJointSim2.h"

namespace zonai
{

struct splineRailFrame2
{
    bool valid = false;
    splineParameter2 parameter{};
    vec2 localPoint{}, anchorA{}, anchorB{}, tangent{}, normal{};
    double distance = 0.0, translation = 0.0, curvature = 0.0;
    float lateralError = 0.0f, angleError = 0.0f, arcScale = 1.0f;
};

struct splineRailJointConstraint2
{
    std::int32_t jointId = -1, bodyIdA = -1, bodyIdB = -1;
    const splineRailJointSim2* joint = nullptr;
    bodySim bodyA{}, bodyB{};
    vec2 impulse{};
    float springImpulse = 0.0f, lowerImpulse = 0.0f, upperImpulse = 0.0f, motorImpulse = 0.0f;
    float subStepTime = 0.0f;
    splineParameter2 parameter{};
    vec2 trackedPoint{};
    bool trackingValid = false;
    vec2 force{};
    float torque = 0.0f;
};

bool setSplineRailGeometry( splineRailJointSim2& joint, std::span<const vec2> points );
splineRailFrame2 getSplineRailFrame( const splineRailJointSim2& joint, const transform2& bodyA, const transform2& bodyB, splineParameter2 seed, bool tracked );
[[nodiscard]] splineRailJointConstraint2 prepareSplineRailJointConstraint( const splineRailJointSim2& joint, const bodySim& bodySimA, const bodySim& bodySimB, float subStepTime );
void warmStartSplineRailJointConstraint( splineRailJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB );
void solveSplineRailJointConstraint( splineRailJointConstraint2& constraint, bodyState& bodyStateA, bodyState& bodyStateB, bool useBias );

} // namespace zonai
