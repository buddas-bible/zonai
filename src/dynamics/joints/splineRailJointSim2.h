#pragma once

#include <vector>
#include "dynamics/joints/splineRailJoint2.h"

namespace zonai
{

// 구간 계수는 geometry가 소유하고 레일 고유의 누적 길이만 따로 보관함.
struct splineRailSegment2
{
    double prefix = 0.0, length = 0.0;
};

struct splineRailJointSim2
{
    std::int32_t jointId = -1, bodyIdA = -1, bodyIdB = -1;
    std::vector<vec2> points{};
    preparedCatmullRomSpline2 geometry{};
    std::vector<splineRailSegment2> segments{};
    double length = 0.0, referenceDistance = 0.0;
    vec2 localAnchorB{};
    float referenceAngle = 0.0f, tangentOffset = 0.0f;
    splineRailRotationMode rotationMode = splineRailRotationMode::fixed;
    float angularHertz = 3.0f, angularDampingRatio = 0.7f;
    bool enableSpring = false;
    float hertz = 0.0f, dampingRatio = 0.7f, targetTranslation = 0.0f;
    bool enableLimit = false;
    float lowerTranslation = 0.0f, upperTranslation = 0.0f;
    bool enableMotor = false;
    float motorSpeed = 0.0f, maxMotorForce = 0.0f;
    vec2 impulse{};
    float springImpulse = 0.0f, lowerImpulse = 0.0f, upperImpulse = 0.0f, motorImpulse = 0.0f;
    float subStepTime = 0.0f;
    splineParameter2 parameter{};
    bool trackingValid = false;
    vec2 force{};
    float torque = 0.0f;
};

} // namespace zonai
