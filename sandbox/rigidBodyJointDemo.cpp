#include "rigidBodyDemo.h"

#include <array>
#include <numbers>

namespace zonai::sandbox
{

#pragma region Presets

void rigidBodyDemo::applyDistancePreset( distanceDemoPreset preset )
{
    if( kind_ != demoKind::distancePendulum || !world_.IsValid( pendulumJoint_ ) ) return;

    constexpr float hertz = 2.0f;
    constexpr float dampingRatio = 0.7f;
    constexpr float minLength = 1.5f;
    constexpr float maxLength = 2.5f;
    constexpr float motorSpeed = 1.0f;
    constexpr float maxMotorForce = 10.0f;

    switch( preset )
    {
    case distanceDemoPreset::rigid:
        world_.setDistanceJointSpring( pendulumJoint_, false, hertz, dampingRatio );
        world_.setDistanceJointLimit( pendulumJoint_, false, minLength, maxLength );
        world_.setDistanceJointMotor( pendulumJoint_, false, motorSpeed, maxMotorForce );
        break;

    case distanceDemoPreset::spring:
        world_.setDistanceJointSpring( pendulumJoint_, true, hertz, dampingRatio );
        world_.setDistanceJointLimit( pendulumJoint_, false, minLength, maxLength );
        world_.setDistanceJointMotor( pendulumJoint_, false, motorSpeed, maxMotorForce );
        break;

    case distanceDemoPreset::limit:
        // Distance limit은 spring mode에서 풀리므로 0 Hz로 스프링 힘만 제거함.
        world_.setDistanceJointSpring( pendulumJoint_, true, 0.0f, dampingRatio );
        world_.setDistanceJointLimit( pendulumJoint_, true, minLength, maxLength );
        world_.setDistanceJointMotor( pendulumJoint_, false, motorSpeed, maxMotorForce );
        break;

    case distanceDemoPreset::motor:
        // 0 Hz와 limit off로 축을 자유롭게 두고 motor 효과만 관찰함.
        world_.setDistanceJointSpring( pendulumJoint_, true, 0.0f, dampingRatio );
        world_.setDistanceJointLimit( pendulumJoint_, false, minLength, maxLength );
        world_.setDistanceJointMotor( pendulumJoint_, true, motorSpeed, maxMotorForce );
        break;
    }
}

void rigidBodyDemo::applyRevolutePreset( revoluteDemoPreset preset )
{
    if( kind_ != demoKind::revoluteHinge || !world_.IsValid( revoluteJoint_ ) ) return;

    constexpr float lowerAngle = -0.25f * std::numbers::pi_v<float>;
    constexpr float upperAngle = 0.25f * std::numbers::pi_v<float>;
    constexpr float motorSpeed = 2.0f;
    constexpr float maxMotorTorque = 10.0f;

    const bool enableLimit = preset == revoluteDemoPreset::limit || preset == revoluteDemoPreset::motorLimit;
    const bool enableMotor = preset == revoluteDemoPreset::motor || preset == revoluteDemoPreset::motorLimit;
    world_.setRevoluteJointLimit( revoluteJoint_, enableLimit, lowerAngle, upperAngle );
    world_.setRevoluteJointMotor( revoluteJoint_, enableMotor, motorSpeed, maxMotorTorque );
}

void rigidBodyDemo::applyWheelPreset( wheelDemoPreset preset )
{
    if( kind_ != demoKind::wheelSuspension || !world_.IsValid( wheelJoint_ ) ) return;

    constexpr float hertz = 3.0f;
    constexpr float dampingRatio = 0.7f;
    constexpr float lowerTranslation = -0.5f;
    constexpr float upperTranslation = 0.5f;
    constexpr float motorSpeed = 3.0f;
    constexpr float maxMotorTorque = 1.0f;

    const bool enableSpring = preset != wheelDemoPreset::limit;
    const bool enableLimit = preset == wheelDemoPreset::limit || preset == wheelDemoPreset::combined;
    const bool enableMotor = preset == wheelDemoPreset::motor || preset == wheelDemoPreset::combined;
    world_.setWheelJointSpring( wheelJoint_, enableSpring, hertz, dampingRatio );
    world_.setWheelJointLimit( wheelJoint_, enableLimit, lowerTranslation, upperTranslation );
    world_.setWheelJointMotor( wheelJoint_, enableMotor, motorSpeed, maxMotorTorque );
}

#pragma endregion Presets

#pragma region SceneSetup

void rigidBodyDemo::createPrismaticRail()
{
    world_.SetGravity( {} );

    const bodyId rail = world_.CreateBody();
    const shapeId railShape = world_.CreateShape( rail, MakeBox( { 3.0f, 0.06f } ) );
    shapes_.push_back( { rail, railShape, "레일 [정적]" } );

    impulseBody_ = world_.CreateBody( bodyType::Dynamic, { { -1.5f, 0.0f }, {} } );
    torqueBody_ = impulseBody_;
    const shapeId sliderShape = world_.CreateShape( impulseBody_, MakeBox( { 0.35f, 0.35f } ) );
    shapes_.push_back( { impulseBody_, sliderShape, "슬라이더 [프리즈매틱]" } );

    prismaticJointDef joint{};
    joint.bodyA = rail;
    joint.bodyB = impulseBody_;
    prismaticJoint_ = world_.createPrismaticJoint( joint );
}

void rigidBodyDemo::createMouseJointPlayground()
{
    const bodyId ground = world_.CreateBody( bodyType::Static, { { 0.0f, -2.5f }, {} } );
    const shapeId groundShape = world_.CreateShape( ground, MakeBox( { 4.5f, 0.25f } ) );
    shapes_.push_back( { ground, groundShape, "바닥 [정적]" } );

    constexpr std::array densities{ 0.5f, 1.5f, 4.0f };
    constexpr std::array xPositions{ -2.0f, 0.0f, 2.0f };
    constexpr std::array labels{ "가벼운 상자 [마우스 조인트]", "중간 상자 [마우스 조인트]", "무거운 상자 [마우스 조인트]" };
    for( std::size_t i = 0; i < densities.size(); ++i )
    {
        const bodyId body = world_.CreateBody( bodyType::Dynamic, { { xPositions[i], 0.5f }, {} } );
        const shapeId shape = world_.CreateShape( body, MakeBox( { 0.5f, 0.5f } ), {}, densities[i] );
        shapes_.push_back( { body, shape, labels[i] } );
        if( i == 1 ) impulseBody_ = body;
        if( i == 2 ) torqueBody_ = body;
    }
}

#pragma endregion SceneSetup

} // namespace zonai::sandbox
