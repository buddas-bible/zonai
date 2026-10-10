#include "rigidBodyDemo.h"

#include <array>
#include <numbers>

namespace zonai::sandbox
{

#pragma region Settings

void rigidBodyDemo::setPrismaticSpringSettings( bool enableSpring, float hertz, float dampingRatio, float targetTranslation )
{
    if( kind_ != demoKind::prismaticRail || !world_.IsValid( prismaticJoint_ ) ) return;
    world_.setPrismaticJointSpring( prismaticJoint_, enableSpring, hertz, dampingRatio, targetTranslation );
}

void rigidBodyDemo::setPrismaticLimitSettings( bool enableLimit, float lowerTranslation, float upperTranslation )
{
    if( kind_ != demoKind::prismaticRail || !world_.IsValid( prismaticJoint_ ) ) return;
    world_.setPrismaticJointLimit( prismaticJoint_, enableLimit, lowerTranslation, upperTranslation );
}

void rigidBodyDemo::setPrismaticMotorSettings( bool enableMotor, float motorSpeed, float maxMotorForce )
{
    if( kind_ != demoKind::prismaticRail || !world_.IsValid( prismaticJoint_ ) ) return;
    world_.setPrismaticJointMotor( prismaticJoint_, enableMotor, motorSpeed, maxMotorForce );
}

void rigidBodyDemo::setPrismaticSpringTargetToCurrent()
{
    if( kind_ != demoKind::prismaticRail || !world_.IsValid( prismaticJoint_ ) ) return;
    const prismaticJointData joint = world_.getPrismaticJointData( prismaticJoint_ );
    world_.setPrismaticJointSpring( prismaticJoint_, joint.enableSpring, joint.hertz, joint.dampingRatio, joint.currentTranslation );
}

void rigidBodyDemo::setWeldLinearSettings( float hertz, float dampingRatio )
{
    if( kind_ != demoKind::weldPair || !world_.IsValid( weldJoint_ ) ) return;
    world_.setWeldJointLinearTuning( weldJoint_, hertz, dampingRatio );
}

void rigidBodyDemo::setWeldAngularSettings( float hertz, float dampingRatio )
{
    if( kind_ != demoKind::weldPair || !world_.IsValid( weldJoint_ ) ) return;
    world_.setWeldJointAngularTuning( weldJoint_, hertz, dampingRatio );
}


void rigidBodyDemo::setMotorJointLinearSettings( vec2 linearVelocity, float maxVelocityForce )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;
    world_.setMotorJointLinearVelocity( motorJoint_, linearVelocity, maxVelocityForce );
}

void rigidBodyDemo::setMotorJointAngularSettings( float angularVelocity, float maxVelocityTorque )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;
    world_.setMotorJointAngularVelocity( motorJoint_, angularVelocity, maxVelocityTorque );
}


void rigidBodyDemo::setMotorJointLinearSpringSettings( float hertz, float dampingRatio, float maxSpringForce )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;
    world_.setMotorJointLinearSpring( motorJoint_, hertz, dampingRatio, maxSpringForce );
}

void rigidBodyDemo::setMotorJointAngularSpringSettings( float referenceAngle, float hertz, float dampingRatio, float maxSpringTorque )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;
    world_.setMotorJointAngularSpring( motorJoint_, referenceAngle, hertz, dampingRatio, maxSpringTorque );
}


void rigidBodyDemo::setMoverJointSettings( vec2 linearVelocity, vec2 maxVelocityForce )
{
    if( kind_ != demoKind::moverJointPlayground || !world_.IsValid( moverJoint_ ) ) return;
    world_.setMoverJointLinearVelocity( moverJoint_, linearVelocity );
    world_.setMoverJointMaxVelocityForce( moverJoint_, maxVelocityForce );
}

void rigidBodyDemo::setPogoJointSettings( float restLength, float hertz, float dampingRatio, float maxTensionForce, float maxCompressionForce )
{
    if( kind_ != demoKind::pogoJointPlayground || !world_.IsValid( pogoJoint_ ) ) return;
    world_.setPogoJointSpring( pogoJoint_, restLength, hertz, dampingRatio );
    world_.setPogoJointForceLimits( pogoJoint_, maxTensionForce, maxCompressionForce );
}

#pragma endregion Settings

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

void rigidBodyDemo::applyPrismaticPreset( prismaticDemoPreset preset )
{
    if( kind_ != demoKind::prismaticRail || !world_.IsValid( prismaticJoint_ ) ) return;

    constexpr float hertz = 2.0f;
    constexpr float dampingRatio = 0.7f;
    constexpr float springTarget = 1.0f;
    constexpr float constrainedSpringTarget = 3.0f;
    constexpr float lowerTranslation = -2.0f;
    constexpr float upperTranslation = 2.0f;
    constexpr float motorSpeed = 2.0f;
    constexpr float maxMotorForce = 20.0f;

    bool enableSpring = false;
    float targetTranslation = springTarget;
    bool enableLimit = false;
    float lower = lowerTranslation;
    float upper = upperTranslation;
    bool enableMotor = false;
    float speed = motorSpeed;

    switch( preset )
    {
    case prismaticDemoPreset::free:
        break;

    case prismaticDemoPreset::limited:
        enableLimit = true;
        break;

    case prismaticDemoPreset::locked:
        enableLimit = true;
        lower = 0.0f;
        upper = 0.0f;
        break;

    case prismaticDemoPreset::motorForward:
        enableMotor = true;
        break;

    case prismaticDemoPreset::motorReverse:
        enableMotor = true;
        speed = -motorSpeed;
        break;

    case prismaticDemoPreset::brake:
        enableMotor = true;
        speed = 0.0f;
        break;

    case prismaticDemoPreset::motorLimit:
        enableLimit = true;
        enableMotor = true;
        break;

    case prismaticDemoPreset::spring:
        enableSpring = true;
        break;

    case prismaticDemoPreset::springLimit:
        enableSpring = true;
        targetTranslation = constrainedSpringTarget;
        enableLimit = true;
        break;

    case prismaticDemoPreset::combined:
        enableSpring = true;
        targetTranslation = constrainedSpringTarget;
        enableLimit = true;
        enableMotor = true;
        break;
    }

    // Box2D Prismatic solve 순서와 같은 개념 순서로 설정해 각 preset의 축 제약을 한눈에 비교함.
    world_.setPrismaticJointSpring( prismaticJoint_, enableSpring, hertz, dampingRatio, targetTranslation );
    world_.setPrismaticJointMotor( prismaticJoint_, enableMotor, speed, maxMotorForce );
    world_.setPrismaticJointLimit( prismaticJoint_, enableLimit, lower, upper );
}

void rigidBodyDemo::applyWeldPreset( weldDemoPreset preset )
{
    if( kind_ != demoKind::weldPair || !world_.IsValid( weldJoint_ ) ) return;

    constexpr float hertz = 3.0f;
    constexpr float dampingRatio = 0.7f;

    const bool softLinear = preset == weldDemoPreset::softLinear || preset == weldDemoPreset::softBoth;
    const bool softAngular = preset == weldDemoPreset::softAngular || preset == weldDemoPreset::softBoth;

    // Weld는 0 Hz 자체가 hard constraint 의미라 별도 enable flag 없이 두 채널을 독립적으로 비교함.
    world_.setWeldJointLinearTuning( weldJoint_, softLinear ? hertz : 0.0f, dampingRatio );
    world_.setWeldJointAngularTuning( weldJoint_, softAngular ? hertz : 0.0f, dampingRatio );
}


void rigidBodyDemo::applyMotorJointPreset( motorJointDemoPreset preset )
{
    if( kind_ != demoKind::motorJointPlayground || !world_.IsValid( motorJoint_ ) ) return;

    constexpr vec2 linearVelocity{ 2.0f, 0.5f };
    constexpr float maxVelocityForce = 20.0f;
    constexpr float angularVelocity = 2.0f;
    constexpr float maxVelocityTorque = 10.0f;
    constexpr float referenceAngle = 0.75f;
    constexpr float springHertz = 3.0f;
    constexpr float springDampingRatio = 0.7f;
    constexpr float maxSpringForce = 20.0f;
    constexpr float maxSpringTorque = 10.0f;

    vec2 targetLinearVelocity{};
    float linearForce = 0.0f;
    float targetAngularVelocity = 0.0f;
    float angularTorque = 0.0f;
    float linearSpringHertz = 0.0f;
    float linearSpringForce = 0.0f;
    float angularSpringTarget = 0.0f;
    float angularSpringHertz = 0.0f;
    float angularSpringTorque = 0.0f;

    switch( preset )
    {
    case motorJointDemoPreset::brake:
        // 목표속도 0에 유한한 힘/토크를 주면 transform lock이 아니라 속도를 죽이는 brake가 됨.
        linearForce = maxVelocityForce;
        angularTorque = maxVelocityTorque;
        break;

    case motorJointDemoPreset::linear:
        targetLinearVelocity = linearVelocity;
        linearForce = maxVelocityForce;
        break;

    case motorJointDemoPreset::angular:
        targetAngularVelocity = angularVelocity;
        angularTorque = maxVelocityTorque;
        break;

    case motorJointDemoPreset::combined:
        targetLinearVelocity = linearVelocity;
        linearForce = maxVelocityForce;
        targetAngularVelocity = angularVelocity;
        angularTorque = maxVelocityTorque;
        break;

    case motorJointDemoPreset::linearSpring:
        linearSpringHertz = springHertz;
        linearSpringForce = maxSpringForce;
        break;

    case motorJointDemoPreset::angularSpring:
        angularSpringTarget = referenceAngle;
        angularSpringHertz = springHertz;
        angularSpringTorque = maxSpringTorque;
        break;

    case motorJointDemoPreset::springBoth:
        linearSpringHertz = springHertz;
        linearSpringForce = maxSpringForce;
        angularSpringTarget = referenceAngle;
        angularSpringHertz = springHertz;
        angularSpringTorque = maxSpringTorque;
        break;

    case motorJointDemoPreset::velocityAndSpring:
        targetLinearVelocity = linearVelocity;
        linearForce = maxVelocityForce;
        targetAngularVelocity = angularVelocity;
        angularTorque = maxVelocityTorque;
        linearSpringHertz = springHertz;
        linearSpringForce = maxSpringForce;
        angularSpringTarget = referenceAngle;
        angularSpringHertz = springHertz;
        angularSpringTorque = maxSpringTorque;
        break;
    }

    // 모든 actuator를 항상 명시적으로 설정해 이전 preset의 velocity / spring 상태가 남지 않게 함.
    world_.setMotorJointLinearVelocity( motorJoint_, targetLinearVelocity, linearForce );
    world_.setMotorJointAngularVelocity( motorJoint_, targetAngularVelocity, angularTorque );
    world_.setMotorJointLinearSpring( motorJoint_, linearSpringHertz, springDampingRatio, linearSpringForce );
    world_.setMotorJointAngularSpring( motorJoint_, angularSpringTarget, angularSpringHertz, springDampingRatio, angularSpringTorque );
}


void rigidBodyDemo::applyMoverJointPreset( moverJointDemoPreset preset )
{
    if( kind_ != demoKind::moverJointPlayground || !world_.IsValid( moverJoint_ ) ) return;

    vec2 targetVelocity{};
    vec2 maxForce{ 20.0f, 20.0f };

    switch( preset )
    {
    case moverJointDemoPreset::horizontal:
        targetVelocity = { 2.0f, 0.0f };
        break;

    case moverJointDemoPreset::vertical:
        targetVelocity = { 0.0f, 2.0f };
        break;

    case moverJointDemoPreset::diagonal:
        targetVelocity = { 1.5f, 1.5f };
        break;

    case moverJointDemoPreset::anisotropic:
        targetVelocity = { 2.0f, 2.0f };
        // 같은 속도 목표라도 x/y actuator 예산을 다르게 줘 축별 clamp를 눈으로 비교함.
        maxForce = { 20.0f, 5.0f };
        break;
    }

    setMoverJointSettings( targetVelocity, maxForce );
}

void rigidBodyDemo::applyPogoJointPreset( pogoJointDemoPreset preset )
{
    if( kind_ != demoKind::pogoJointPlayground || !world_.IsValid( pogoJoint_ ) ) return;

    constexpr float restLength = 0.8f;
    constexpr float dampingRatio = 0.7f;
    float hertz = 2.0f;
    float maxTensionForce = 50.0f;
    float maxCompressionForce = 200.0f;

    switch( preset )
    {
    case pogoJointDemoPreset::soft:
        break;

    case pogoJointDemoPreset::stiff:
        hertz = 8.0f;
        maxTensionForce = 100.0f;
        maxCompressionForce = 400.0f;
        break;

    case pogoJointDemoPreset::compressionOnly:
        hertz = 4.0f;
        maxTensionForce = 0.0f;
        maxCompressionForce = 300.0f;
        break;

    case pogoJointDemoPreset::asymmetric:
        hertz = 4.0f;
        maxTensionForce = 40.0f;
        maxCompressionForce = 350.0f;
        break;
    }

    setPogoJointSettings( restLength, hertz, dampingRatio, maxTensionForce, maxCompressionForce );
}

#pragma endregion Presets

#pragma region Queries

float rigidBodyDemo::getPrismaticCurrentSpeed() const
{
    if( kind_ != demoKind::prismaticRail || !world_.IsValid( prismaticJoint_ ) ) return 0.0f;

    const prismaticJointData joint = world_.getPrismaticJointData( prismaticJoint_ );
    const vec2 centerA = TransformPoint( world_.GetBodyTransform( joint.bodyA ), world_.GetBodyLocalCenter( joint.bodyA ) );
    const vec2 centerB = TransformPoint( world_.GetBodyTransform( joint.bodyB ), world_.GetBodyLocalCenter( joint.bodyB ) );
    const vec2 rA = joint.anchorA - centerA;
    const vec2 rB = joint.anchorB - centerB;
    const vec2 velocityA = world_.GetBodyLinearVelocity( joint.bodyA ) + Cross( world_.GetBodyAngularVelocity( joint.bodyA ), rA );
    const vec2 velocityB = world_.GetBodyLinearVelocity( joint.bodyB ) + Cross( world_.GetBodyAngularVelocity( joint.bodyB ), rB );
    return Dot( joint.axis, velocityB - velocityA );
}

#pragma endregion Queries

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

void rigidBodyDemo::createWeldPair()
{
    world_.SetGravity( {} );

    const bodyId bodyA = world_.CreateBody( bodyType::Dynamic, { { -0.7f, 0.0f }, {} } );
    const shapeId shapeA = world_.CreateShape( bodyA, MakeBox( { 0.55f, 0.35f } ) );
    shapes_.push_back( { bodyA, shapeA, "왼쪽 상자 [웰드]" } );

    const bodyId bodyB = world_.CreateBody( bodyType::Dynamic, { { 0.7f, 0.0f }, {} } );
    const shapeId shapeB = world_.CreateShape( bodyB, MakeBox( { 0.55f, 0.35f } ) );
    shapes_.push_back( { bodyB, shapeB, "오른쪽 상자 [웰드]" } );

    weldJointDef joint{};
    joint.bodyA = bodyA;
    joint.bodyB = bodyB;
    joint.localAnchorA = { 0.7f, 0.0f };
    joint.localAnchorB = { -0.7f, 0.0f };
    weldJoint_ = world_.createWeldJoint( joint );

    impulseBody_ = bodyB;
    torqueBody_ = bodyB;
}


void rigidBodyDemo::createMoverJointPlayground()
{
    world_.SetGravity( {} );

    // Mover는 Body 위치나 anchor를 제약하지 않음. A는 상대 선속도의 기준 Body로만 사용함.
    const bodyId reference = world_.CreateBody( bodyType::Static, { { -2.0f, 0.0f }, {} } );
    const shapeId referenceShape = world_.CreateShape( reference, MakeBox( { 0.25f, 0.25f } ) );
    shapes_.push_back( { reference, referenceShape, "기준 Body A [Mover]" } );

    const bodyId driven = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 0.0f }, {} } );
    const shapeId drivenShape = world_.CreateShape( driven, MakeBox( { 0.55f, 0.35f } ) );
    shapes_.push_back( { driven, drivenShape, "구동 Body B [Mover]" } );

    moverJointDef joint{};
    joint.bodyA = reference;
    joint.bodyB = driven;
    joint.linearVelocity = { 2.0f, 0.0f };
    joint.maxVelocityForce = { 20.0f, 20.0f };
    moverJoint_ = world_.createMoverJoint( joint );

    impulseBody_ = driven;
    torqueBody_ = driven;
}

void rigidBodyDemo::createPogoJointPlayground()
{
    world_.SetGravity( { 0.0f, -9.8f } );

    const bodyId ground = world_.CreateBody( bodyType::Static, { { 0.0f, -0.25f }, {} } );
    const shapeId groundShape = world_.CreateShape( ground, MakeBox( { 3.0f, 0.25f } ) );
    shapes_.push_back( { ground, groundShape, "지면 Body A [Pogo]" } );

    const bodyId character = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 0.75f }, {} } );
    const shapeId characterShape = world_.CreateShape( character, MakeBox( { 0.4f, 0.35f } ) );
    shapes_.push_back( { character, characterShape, "캐릭터 Body B [Pogo]" } );

    pogoJointDef joint{};
    joint.bodyA = ground;
    joint.bodyB = character;
    joint.localAnchorA = { 0.0f, 0.25f };
    joint.localAnchorB = { 0.0f, -0.35f };
    joint.localPogoAxisB = { 0.0f, 1.0f };
    joint.normal = { 0.0f, 1.0f };
    joint.restLength = 0.8f;
    joint.hertz = 2.0f;
    joint.dampingRatio = 0.7f;
    joint.maxTensionForce = 50.0f;
    joint.maxCompressionForce = 200.0f;
    // 실제 character mover처럼 Pogo와 별개로 ground contact도 유지할 수 있게 함.
    joint.collideConnected = true;
    pogoJoint_ = world_.createPogoJoint( joint );

    impulseBody_ = character;
    torqueBody_ = character;
}

void rigidBodyDemo::createMotorJointPlayground()
{
    world_.SetGravity( {} );

    // A는 움직이지 않는 기준 frame, B는 Motor의 목표 상대속도를 따라가는 Dynamic body임.
    const bodyId reference = world_.CreateBody( bodyType::Static, { { -2.0f, 0.0f }, {} } );
    const shapeId referenceShape = world_.CreateShape( reference, MakeBox( { 0.25f, 0.25f } ) );
    shapes_.push_back( { reference, referenceShape, "기준 Body A [정적]" } );

    const bodyId driven = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 0.0f }, {} } );
    const shapeId drivenShape = world_.CreateShape( driven, MakeBox( { 0.55f, 0.35f } ) );
    shapes_.push_back( { driven, drivenShape, "구동 Body B [Motor Joint]" } );

    motorJointDef joint{};
    joint.bodyA = reference;
    joint.bodyB = driven;
    joint.maxVelocityForce = 20.0f;
    joint.maxVelocityTorque = 10.0f;
    motorJoint_ = world_.createMotorJoint( joint );

    impulseBody_ = driven;
    torqueBody_ = driven;
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
