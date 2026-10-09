#include "rigidBodyDemo.h"

#include <cassert>
#include <cmath>
#include <numbers>

namespace zonai::sandbox
{

#pragma region LifetimeAndSimulation

rigidBodyDemo::rigidBodyDemo( demoKind kind ) : kind_( kind )
{
    shapes_.reserve( 8 );
    contacts_.reserve( 16 );
    if( kind == demoKind::playground )
    {
        createPlayground();
    }
    else if( kind == demoKind::distancePendulum )
    {
        createPendulum();
    }
    else if( kind == demoKind::revoluteHinge )
    {
        createRevoluteHinge();
    }
    else if( kind == demoKind::wheelSuspension )
    {
        createWheelSuspension();
    }
    else if( kind == demoKind::prismaticRail )
    {
        createPrismaticRail();
    }
    else if( kind == demoKind::weldPair )
    {
        createWeldPair();
    }
    else if( kind == demoKind::mouseJointPlayground )
    {
        createMouseJointPlayground();
    }
    else
    {
        assert( kind == demoKind::motorCar );

        createMotorCar();
    }
    refreshContacts();
}

void rigidBodyDemo::step( float timeStep, int subStepCount )
{
    const bodyId target = kind_ == demoKind::distancePendulum ? pendulumBody_ : impulseBody_;
    const float direction = static_cast<float>( rightHeld_ ) - static_cast<float>( leftHeld_ );

    // 누르고 있는 이동 입력은 매 physics step에 힘으로 적용함. 입력을 취소해도 현재 물리 속도는 유지함.
    if( kind_ != demoKind::motorCar && direction != 0.0f && world_.IsValid( target ) )
    {
        world_.ApplyForceToCenter( target, { direction * world_.GetBodyMass( target ) * 5.0f, 0.0f } );
    }

    world_.Step( timeStep, subStepCount );
    refreshContacts();
}

#pragma endregion LifetimeAndSimulation

#pragma region Input

void rigidBodyDemo::handleInput( const demoInput& input )
{
    assert( IsFinite( input.mousePosition ) );

    leftHeld_ = input.left;
    rightHeld_ = input.right;
    brakeHeld_ = input.brake;
    updateCarMotors();
    if( !world_.IsValid( mouseJoint_ ) )
    {
        mouseJoint_ = {};
    }

    // 마우스를 놓으면 조인트를 제거하고, 잡고 있으면 목표점만 갱신함.
    if( !input.mouseHeld )
    {
        if( world_.IsValid( mouseJoint_ ) )
        {
            world_.destroyJoint( mouseJoint_ );
            mouseJoint_ = {};
        }
    }
    else
    {
        if( input.mousePressed && !world_.IsValid( mouseJoint_ ) )
        {
            startMouseDrag( input.mousePosition );
        }
        if( world_.IsValid( mouseJoint_ ) )
        {
            world_.setMouseJointTarget( mouseJoint_, input.mousePosition );
        }
    }
    const bodyId target = kind_ == demoKind::distancePendulum ? pendulumBody_ : impulseBody_;
    const float mass = world_.IsValid( target ) ? world_.GetBodyMass( target ) : 0.0f;

    // 한 번 누른 입력은 임펄스로 적용함. 누르고 있는 입력의 힘과 구분함.
    if( kind_ != demoKind::motorCar && input.jumpPressed && world_.IsValid( target ) )
    {
        world_.ApplyLinearImpulseToCenter( target, kind_ == demoKind::distancePendulum ? vec2{ mass * 2.0f, 0.0f } : vec2{ 0.0f, mass * 5.0f } );
    }
    if( kind_ != demoKind::motorCar && input.spinPressed && world_.IsValid( torqueBody_ ) )
    {
        world_.ApplyAngularImpulse( torqueBody_, world_.GetBodyRotationalInertia( torqueBody_ ) * 3.0f );
    }
    if( input.impulsePressed && world_.IsValid( target ) )
    {
        const vec2 center = TransformPoint( world_.GetBodyTransform( target ), world_.GetBodyLocalCenter( target ) );
        world_.ApplyLinearImpulseToCenter( target, Normalize( input.mousePosition - center ) * mass * 2.0f );
    }
}

void rigidBodyDemo::cancelInput()
{
    leftHeld_ = false;
    rightHeld_ = false;
    brakeHeld_ = false;
    updateCarMotors();
    // UI 조작, 포커스 상실, 데모 교체에서 마우스 제약을 제거함. 물체 삭제로 이미 사라졌을 수도 있음.
    if( world_.IsValid( mouseJoint_ ) )
    {
        world_.destroyJoint( mouseJoint_ );
    }
    mouseJoint_ = {};
}

#pragma endregion Input

#pragma region Settings

void rigidBodyDemo::setCollisionMatrix( const collisionMatrix& matrix )
{
    world_.setCollisionMatrix( matrix );
    refreshContacts();
}

void rigidBodyDemo::setMouseSettings( float hertz, float dampingRatio, float maxForce )
{
    assert( std::isfinite( hertz ) && hertz >= 0.0f && std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );
    assert( std::isfinite( maxForce ) && maxForce >= 0.0f );

    mouseSettings_.hertz = hertz;
    mouseSettings_.dampingRatio = dampingRatio;
    mouseSettings_.maxForce = maxForce;
    if( world_.IsValid( mouseJoint_ ) )
    {
        world_.setMouseJointTuning( mouseJoint_, hertz, dampingRatio, maxForce );
    }
}

void rigidBodyDemo::setCarMotorSettings( float speed, float maxTorque )
{
    assert( std::isfinite( speed ) && speed >= 0.0f && std::isfinite( maxTorque ) && maxTorque >= 0.0f );

    if( carMotorSpeed_ == speed && carMaxMotorTorque_ == maxTorque ) return;

    carMotorSpeed_ = speed;
    carMaxMotorTorque_ = maxTorque;
    updateCarMotors();
}

#pragma endregion Settings

#pragma region Contacts

void rigidBodyDemo::refreshContacts()
{
    contacts_.clear();
    world_.UpdateCollisions(
        [this]( const contactData& contact )
        {
            contacts_.push_back( contact );
        } );
}

#pragma endregion Contacts

#pragma region SceneSetup

void rigidBodyDemo::createPlayground()
{

    // 넓은 정적 바닥.
    const bodyId ground = world_.CreateBody( bodyType::Static, { { 0.0f, -4.0f }, {} } );

    const shapeId groundShape = world_.CreateShape( ground, MakeBox( { 8.0f, 0.5f } ) );

    shapes_.push_back( { ground, groundShape, "바닥 [정적]" } );

    // 기울어진 정적 경사면.
    const bodyId ramp = world_.CreateBody( bodyType::Static, { { 4.0f, -1.6f }, rot2::FromRadians( 0.22f ) } );

    const shapeId rampShape = world_.CreateShape( ramp, segment2{ { -2.0f, 0.0f }, { 2.0f, 0.0f } } );

    shapes_.push_back( { ramp, rampShape, "경사면 [정적]" } );

    // 임펄스 입력으로 직접 밀어볼 동적 원.
    impulseBody_ = world_.CreateBody( bodyType::Dynamic, { { -3.2f, 3.5f }, {} } );

    const shapeId circleShape = world_.CreateShape( impulseBody_, circle2{ {}, 0.65f } );

    shapes_.push_back( { impulseBody_, circleShape, "원 [동적]" } );

    // 질량 중심과 물체 원점이 일치하는 동적 상자.
    torqueBody_ = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 5.0f }, rot2::FromRadians( 0.15f ) } );

    const shapeId boxShape = world_.CreateShape( torqueBody_, MakeBox( { 0.75f, 0.55f } ) );

    shapes_.push_back( { torqueBody_, boxShape, "상자 [동적]" } );

    // 길쭉한 동적 캡슐.
    const bodyId capsuleBody = world_.CreateBody( bodyType::Dynamic, { { 3.0f, 4.0f }, rot2::FromRadians( -0.25f ) } );

    const shapeId capsuleShape = world_.CreateShape( capsuleBody, capsule2{ { -0.8f, 0.0f }, { 0.8f, 0.0f }, 0.35f } );

    shapes_.push_back( { capsuleBody, capsuleShape, "캡슐 [동적]" } );

    // 중력과 힘의 영향을 받지 않고 지정한 속도로 움직이는 키네마틱 발판.
    const bodyId kinematicBody = world_.CreateBody( bodyType::Kinematic, { { -5.5f, -1.5f }, {} } );

    const shapeId kinematicShape = world_.CreateShape( kinematicBody, MakeBox( { 0.8f, 0.3f } ) );

    world_.SetBodyLinearVelocity( kinematicBody, { 1.25f, 0.0f } );

    shapes_.push_back( { kinematicBody, kinematicShape, "발판 [키네마틱]" } );
}

void rigidBodyDemo::createPendulum()
{
    const bodyId anchor = world_.CreateBody();
    const shapeId anchorShape = world_.CreateShape( anchor, circle2{ {}, 0.08f } );
    shapes_.push_back( { anchor, anchorShape, "고정점 [정적]" } );
    pendulumBody_ = world_.CreateBody( bodyType::Dynamic, { { 0.0f, -2.0f }, {} } );
    const shapeId pendulumShape = world_.CreateShape( pendulumBody_, circle2{ {}, 0.3f } );
    shapes_.push_back( { pendulumBody_, pendulumShape, "진자 [거리 조인트]" } );

    // 고정 거리와 스프링, 거리 제한, 축 방향 모터를 같은 진자에서 비교하기 위한 설정.
    distanceJointDef joint{};
    joint.bodyA = anchor;
    joint.bodyB = pendulumBody_;
    joint.length = 2.0f;
    joint.hertz = 2.0f;
    joint.dampingRatio = 0.7f;
    joint.minLength = 1.5f;
    joint.maxLength = 2.5f;
    joint.motorSpeed = 1.0f;
    joint.maxMotorForce = 10.0f;
    pendulumJoint_ = world_.createDistanceJoint( joint );
}

void rigidBodyDemo::createRevoluteHinge()
{
    const bodyId anchor = world_.CreateBody();
    const shapeId anchorShape = world_.CreateShape( anchor, circle2{ {}, 0.08f } );
    shapes_.push_back( { anchor, anchorShape, "회전축 [정적]" } );

    // 막대의 위쪽 끝을 연결함. 질량 중심 밖의 작용점으로 선형·회전 운동의 결합을 관찰함.
    impulseBody_ = world_.CreateBody( bodyType::Dynamic, { { 0.0f, -1.0f }, {} } );
    torqueBody_ = impulseBody_;
    const shapeId rodShape = world_.CreateShape( impulseBody_, MakeBox( { 0.18f, 1.0f } ) );
    shapes_.push_back( { impulseBody_, rodShape, "막대 [회전 조인트]" } );

    revoluteJointDef joint{};
    joint.bodyA = anchor;
    joint.bodyB = impulseBody_;
    joint.localAnchorB = { 0.0f, 1.0f };
    joint.lowerAngle = -0.25f * std::numbers::pi_v<float>;
    joint.upperAngle = 0.25f * std::numbers::pi_v<float>;
    // 자유 회전으로 시작하고, 모터를 켜면 토크 한도와 중력 하중의 관계를 비교함.
    joint.motorSpeed = 2.0f;
    joint.maxMotorTorque = 10.0f;
    revoluteJoint_ = world_.createRevoluteJoint( joint );
}

void rigidBodyDemo::createWheelSuspension()
{
    const bodyId frame = world_.CreateBody( bodyType::Static, { { 0.0f, 0.8f }, {} } );
    const shapeId frameShape = world_.CreateShape( frame, MakeBox( { 0.6f, 0.1f } ) );
    shapes_.push_back( { frame, frameShape, "서스펜션 지지대 [정적]" } );
    impulseBody_ = world_.CreateBody( bodyType::Dynamic, { { 0.0f, -0.8f }, {} } );
    torqueBody_ = impulseBody_;
    const shapeId wheelShape = world_.CreateShape( impulseBody_, circle2{ {}, 0.35f } );
    shapes_.push_back( { impulseBody_, wheelShape, "바퀴 [서스펜션]" } );

    // 지지대 원점 아래 1.3 m를 스프링 기준으로 둠. 바퀴는 그 위치보다 0.3 m 아래에서 시작함.
    wheelJointDef joint{};
    joint.bodyA = frame;
    joint.bodyB = impulseBody_;
    joint.localAnchorA = { 0.0f, -1.3f };
    joint.lowerTranslation = -0.5f;
    joint.upperTranslation = 0.5f;
    // 먼저 자유 회전을 관찰한 뒤 모터를 켜서 비교함.
    joint.motorSpeed = 3.0f;
    joint.maxMotorTorque = 1.0f;
    wheelJoint_ = world_.createWheelJoint( joint );
}

void rigidBodyDemo::createMotorCar()
{
    const bodyId ground = world_.CreateBody( bodyType::Static, { { 0.0f, -0.5f }, {} } );
    const shapeId groundShape = world_.CreateShape( ground, MakeBox( { 25.0f, 0.5f } ) );
    world_.SetShapeFriction( groundShape, 0.9f );
    shapes_.push_back( { ground, groundShape, "주행 바닥 [정적]" } );
    const bodyId ramp = world_.CreateBody( bodyType::Static, { { 10.0f, 0.24f }, rot2::FromRadians( 0.12f ) } );
    const shapeId rampShape = world_.CreateShape( ramp, MakeBox( { 3.0f, 0.12f } ) );
    world_.SetShapeFriction( rampShape, 0.9f );
    shapes_.push_back( { ramp, rampShape, "낮은 경사면 [정적]" } );

    impulseBody_ = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 1.1f }, {} } );
    const shapeId chassisShape = world_.CreateShape( impulseBody_, MakeBox( { 1.3f, 0.22f } ), {}, 2.0f );
    shapes_.push_back( { impulseBody_, chassisShape, "차체 [동적]" } );
    for( std::size_t i = 0; i < carJoints_.size(); ++i )
    {
        const float x = i == 0 ? -0.9f : 0.9f;
        const bodyId wheel = world_.CreateBody( bodyType::Dynamic, { { x, 0.45f }, {} } );
        const shapeId wheelShape = world_.CreateShape( wheel, circle2{ {}, 0.35f } );
        world_.SetShapeFriction( wheelShape, 0.9f );
        shapes_.push_back( { wheel, wheelShape, i == 0 ? "왼쪽 바퀴 [구동]" : "오른쪽 바퀴 [구동]" } );

        // A의 로컬 위쪽 축을 따라 서스펜션이 움직임. 연결된 차체-바퀴 충돌은 Joint가 제외함.
        wheelJointDef joint{};
        joint.bodyA = impulseBody_;
        joint.bodyB = wheel;
        joint.localAnchorA = { x, -0.45f };
        joint.hertz = 4.0f;
        joint.enableLimit = true;
        joint.lowerTranslation = -0.25f;
        joint.upperTranslation = 0.25f;
        carJoints_[i] = world_.createWheelJoint( joint );
    }
    updateCarMotors();
}

#pragma endregion SceneSetup

#pragma region CarDrive

void rigidBodyDemo::updateCarMotors()
{
    if( kind_ != demoKind::motorCar ) return;

    const float direction = static_cast<float>( rightHeld_ ) - static_cast<float>( leftHeld_ );
    // 바닥에서 오른쪽으로 구르려면 시계 방향(-w)이 필요함. 제동은 목표 상대속도 0임.
    const float speed = brakeHeld_ ? 0.0f : -direction * carMotorSpeed_;
    for( const jointId id : carJoints_ )
    {
        if( world_.IsValid( id ) )
        {
            world_.setWheelJointMotor( id, brakeHeld_ || direction != 0.0f, speed, carMaxMotorTorque_ );
        }
    }
}

#pragma endregion CarDrive

#pragma region MouseDrag

void rigidBodyDemo::startMouseDrag( vec2 point )
{
    bodyId ground{};
    for( const auto& visual : shapes_ )
    {
        if( world_.IsValid( visual.bodyHandle ) && world_.GetBody( visual.bodyHandle ).type == bodyType::Static )
        {
            ground = visual.bodyHandle;
            break;
        }
    }
    if( !world_.IsValid( ground ) ) return;

    // ponytail: 작은 데모의 shape 목록을 O(n)으로 pick함. 많은 물체의 데모가 필요하면 World overlap query로 후보를 줄임.
    // 역순으로 검사해 같은 위치에서는 나중에 그린 동적 도형을 우선함. 센서는 잡지 않음.
    for( auto visual = shapes_.rbegin(); visual != shapes_.rend(); ++visual )
    {
        if( !world_.IsValid( visual->bodyHandle ) || !world_.IsValid( visual->shapeHandle ) ) continue;

        if( world_.GetBody( visual->bodyHandle ).type != bodyType::Dynamic || world_.IsShapeSensor( visual->shapeHandle ) ) continue;

        if( !world_.testShapePoint( visual->shapeHandle, point ) ) continue;

        // 클릭한 월드 위치를 목표점으로 지정하면 World가 로컬 작용점을 저장함.
        auto definition = mouseSettings_;
        definition.bodyA = ground;
        definition.bodyB = visual->bodyHandle;
        definition.target = point;
        mouseJoint_ = world_.createMouseJoint( definition );
        break;
    }
}

#pragma endregion MouseDrag

std::unique_ptr<demo> createRigidBodyDemo( demoKind kind )
{
    return std::make_unique<rigidBodyDemo>( kind );
}

} // namespace zonai::sandbox
