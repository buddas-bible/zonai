#pragma once

#include <vector>
#include "dynamics/joints/splineRailJoint2.h"

namespace zonai
{

// 편집 때 계수와 길이를 준비해 solver에서 전체 경로를 다시 검증하지 않음.
struct splineRailSegment2
{
    vec2 start{}, end{}, tangent{}, quadratic{}, cubic{};
    double prefix = 0.0, length = 0.0;
};

struct splineRailJointSim2
{
    std::int32_t jointId = -1, bodyIdA = -1, bodyIdB = -1;
    std::vector<vec2> points{};
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
