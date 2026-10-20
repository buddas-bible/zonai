#pragma once

#include <span>
#include "dynamics/id.h"
#include "geometry/catmullRomSpline2.h"

namespace zonai
{

enum class splineRailRotationMode
{
    fixed,
    followTangent
};

// A의 로컬 열린 곡선을 따라 B anchor를 구속함. 이동량은 곡선 길이 - referenceDistance임.
struct splineRailJointDef
{
    bodyId bodyA{}, bodyB{};
    std::span<const vec2> points{};
    vec2 localAnchorB{};
    double referenceDistance = 0.0;
    float referenceAngle = 0.0f;
    splineRailRotationMode rotationMode = splineRailRotationMode::fixed;
    float angularHertz = 3.0f;
    float angularDampingRatio = 0.7f;
    bool enableSpring = false;
    float hertz = 0.0f;
    float dampingRatio = 0.7f;
    float targetTranslation = 0.0f;
    bool enableLimit = false;
    float lowerTranslation = 0.0f, upperTranslation = 0.0f;
    bool enableMotor = false;
    float motorSpeed = 0.0f, maxMotorForce = 0.0f;
    bool collideConnected = false;
};

struct splineRailJointData
{
    bodyId bodyA{}, bodyB{};
    vec2 anchorA{}, anchorB{}, tangent{}, normal{}, force{};
    splineParameter2 parameter{};
    double length = 0.0, referenceDistance = 0.0, currentTranslation = 0.0;
    float lateralError = 0.0f, currentAngle = 0.0f, torque = 0.0f;
    bool trackingValid = false;
    float referenceAngle = 0.0f;
    splineRailRotationMode rotationMode = splineRailRotationMode::fixed;
    float angularHertz = 3.0f, angularDampingRatio = 0.7f;
    bool enableSpring = false;
    float hertz = 0.0f, dampingRatio = 0.7f, targetTranslation = 0.0f, springForce = 0.0f;
    bool enableLimit = false;
    float lowerTranslation = 0.0f, upperTranslation = 0.0f;
    bool enableMotor = false;
    float motorSpeed = 0.0f, maxMotorForce = 0.0f, motorForce = 0.0f;
    bool collideConnected = false;
};

} // namespace zonai
