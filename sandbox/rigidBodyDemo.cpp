#include "rigidBodyDemo.h"

#include <cassert>
#include <cmath>

namespace zonai::sandbox
{
#pragma region LifetimeAndSimulation
rigidBodyDemo::rigidBodyDemo( demoKind kind ) : kind_( kind )
{
    shapes_.reserve( 8 ); contacts_.reserve( 16 );
    if( kind == demoKind::playground ) { createPlayground(); }
    else { assert( kind == demoKind::distancePendulum ); createPendulum(); }
    refreshContacts();
}

void rigidBodyDemo::step( float timeStep, int subStepCount )
{
    const bodyId target = kind_ == demoKind::playground ? impulseBody_ : pendulumBody_;
    const float direction = static_cast<float>( rightHeld_ ) - static_cast<float>( leftHeld_ );
    // Held input은 각 physics step에 force로 적용함. 취소해도 기존 물리 속도는 유지함.
    if( direction != 0.0f && world_.IsValid( target ) ) { world_.ApplyForceToCenter( target, { direction * world_.GetBodyMass( target ) * 5.0f, 0.0f } ); }
    world_.Step( timeStep, subStepCount );
    refreshContacts();
}

void rigidBodyDemo::handleInput( const demoInput& input )
{
    assert( IsFinite( input.mousePosition ) );
    leftHeld_ = input.left; rightHeld_ = input.right;
    if( !world_.IsValid( mouseJoint_ ) ) { mouseJoint_ = {}; }
    if( !input.mouseHeld )
    {
        if( world_.IsValid( mouseJoint_ ) ) { world_.destroyJoint( mouseJoint_ ); mouseJoint_ = {}; }
    }
    else
    {
        if( input.mousePressed && !world_.IsValid( mouseJoint_ ) ) { startMouseDrag( input.mousePosition ); }
        if( world_.IsValid( mouseJoint_ ) ) { world_.setMouseJointTarget( mouseJoint_, input.mousePosition ); }
    }
    const bodyId target = kind_ == demoKind::playground ? impulseBody_ : pendulumBody_;
    const float mass = world_.IsValid( target ) ? world_.GetBodyMass( target ) : 0.0f;
    if( input.jumpPressed && world_.IsValid( target ) )
    {
        world_.ApplyLinearImpulseToCenter( target, kind_ == demoKind::playground ? vec2{ 0.0f, mass * 5.0f } : vec2{ mass * 2.0f, 0.0f } );
    }
    if( input.spinPressed && world_.IsValid( torqueBody_ ) ) { world_.ApplyAngularImpulse( torqueBody_, world_.GetBodyRotationalInertia( torqueBody_ ) * 3.0f ); }
    if( input.impulsePressed && world_.IsValid( target ) )
    {
        const vec2 center = TransformPoint( world_.GetBodyTransform( target ), world_.GetBodyLocalCenter( target ) );
        world_.ApplyLinearImpulseToCenter( target, Normalize( input.mousePosition - center ) * mass * 2.0f );
    }
}

void rigidBodyDemo::cancelInput()
{
    leftHeld_ = false; rightHeld_ = false;
    // UI capture / focus / demo 교체에서 제약까지 제거함. Body 삭제로 이미 사라졌을 수도 있음.
    if( world_.IsValid( mouseJoint_ ) ) { world_.destroyJoint( mouseJoint_ ); }
    mouseJoint_ = {};
}

void rigidBodyDemo::setMouseSettings( float hertz, float dampingRatio, float maxForce )
{
    assert( std::isfinite( hertz ) && hertz >= 0.0f && std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );
    assert( std::isfinite( maxForce ) && maxForce >= 0.0f );
    mouseSettings_.hertz = hertz; mouseSettings_.dampingRatio = dampingRatio; mouseSettings_.maxForce = maxForce;
    if( world_.IsValid( mouseJoint_ ) ) { world_.setMouseJointTuning( mouseJoint_, hertz, dampingRatio, maxForce ); }
}

void rigidBodyDemo::refreshContacts()
{
    contacts_.clear();
    world_.UpdateCollisions( [this]( const contactData& contact ) { contacts_.push_back( contact ); } );
}
#pragma endregion

#pragma region SceneSetup
void rigidBodyDemo::createPlayground()
{

    // 넓은 정적 바닥.
    const bodyId ground =
        world_.CreateBody(
            bodyType::Static,
            {
                { 0.0f, -4.0f },
                {}
            }
        );

    const shapeId groundShape =
        world_.CreateShape(
            ground,
            MakeBox( { 8.0f, 0.5f } )
        );

    shapes_.push_back(
        { ground, groundShape, "Ground [Static]" }
    );

    // 기울어진 정적 ramp.
    const bodyId ramp =
        world_.CreateBody(
            bodyType::Static,
            {
                { 4.0f, -1.6f },
                rot2::FromRadians( 0.22f )
            }
        );

    const shapeId rampShape =
        world_.CreateShape(
            ramp,
            segment2
            {
                { -2.0f, 0.0f },
                {  2.0f, 0.0f }
            }
        );

    shapes_.push_back(
        { ramp, rampShape, "Ramp [Static]" }
    );

    // impulse 버튼으로 직접 밀어볼 Dynamic Circle.
    impulseBody_ =
        world_.CreateBody(
            bodyType::Dynamic,
            {
                { -3.2f, 3.5f },
                {}
            }
        );

    const shapeId circleShape =
        world_.CreateShape(
            impulseBody_,
            circle2{ {}, 0.65f }
        );

    shapes_.push_back(
        { impulseBody_, circleShape, "Circle [Dynamic]" }
    );

    // COM이 origin과 일치하는 Dynamic Box.
    torqueBody_ =
        world_.CreateBody(
            bodyType::Dynamic,
            {
                { 0.0f, 5.0f },
                rot2::FromRadians( 0.15f )
            }
        );

    const shapeId boxShape =
        world_.CreateShape(
            torqueBody_,
            MakeBox( { 0.75f, 0.55f } )
        );

    shapes_.push_back(
        { torqueBody_, boxShape, "Box [Dynamic]" }
    );

    // 길쭉한 Dynamic Capsule.
    const bodyId capsuleBody =
        world_.CreateBody(
            bodyType::Dynamic,
            {
                { 3.0f, 4.0f },
                rot2::FromRadians( -0.25f )
            }
        );

    const shapeId capsuleShape =
        world_.CreateShape(
            capsuleBody,
            capsule2
            {
                { -0.8f, 0.0f },
                {  0.8f, 0.0f },
                0.35f
            }
        );

    shapes_.push_back(
        { capsuleBody, capsuleShape, "Capsule [Dynamic]" }
    );

    // gravity와 force의 영향을 받지 않고 지정한 velocity로만 움직이는 Kinematic Body.
    const bodyId kinematicBody =
        world_.CreateBody(
            bodyType::Kinematic,
            {
                { -5.5f, -1.5f },
                {}
            }
        );

    const shapeId kinematicShape =
        world_.CreateShape(
            kinematicBody,
            MakeBox( { 0.8f, 0.3f } )
        );

    world_.SetBodyLinearVelocity(
        kinematicBody,
        { 1.25f, 0.0f }
    );

    shapes_.push_back(
        {
            kinematicBody,
            kinematicShape,
            "Platform [Kinematic]"
        }
    );

}

void rigidBodyDemo::createPendulum()
{
    const bodyId anchor = world_.CreateBody();
    const shapeId anchorShape = world_.CreateShape( anchor, circle2{ {}, 0.08f } );
    shapes_.push_back( { anchor, anchorShape, "Anchor [Static]" } );
    pendulumBody_ = world_.CreateBody( bodyType::Dynamic, { { 0.0f, -2.0f }, {} } );
    const shapeId pendulumShape = world_.CreateShape( pendulumBody_, circle2{ {}, 0.3f } );
    shapes_.push_back( { pendulumBody_, pendulumShape, "Pendulum [Distance Joint]" } );
    distanceJointDef joint{};
    joint.bodyA = anchor; joint.bodyB = pendulumBody_; joint.length = 2.0f;
    joint.hertz = 2.0f; joint.dampingRatio = 0.7f;
    joint.minLength = 1.5f; joint.maxLength = 2.5f;
    pendulumJoint_ = world_.createDistanceJoint( joint );
}

void rigidBodyDemo::startMouseDrag( vec2 point )
{
    bodyId ground{};
    for( const auto& visual : shapes_ )
    {
        if( world_.IsValid( visual.bodyHandle ) && world_.GetBody( visual.bodyHandle ).type == bodyType::Static ) { ground = visual.bodyHandle; break; }
    }
    if( !world_.IsValid( ground ) ) { return; }
    // ponytail: 작은 데모의 shape 목록을 O(n)으로 pick함. 많은 물체의 데모가 필요하면 World overlap query로 후보를 줄임.
    // 역순으로 검사해 같은 위치에서는 나중에 그린 Dynamic shape를 우선함. Sensor는 잡지 않음.
    for( auto visual = shapes_.rbegin(); visual != shapes_.rend(); ++visual )
    {
        if( !world_.IsValid( visual->bodyHandle ) || !world_.IsValid( visual->shapeHandle ) ) { continue; }
        if( world_.GetBody( visual->bodyHandle ).type != bodyType::Dynamic || world_.IsShapeSensor( visual->shapeHandle ) ) { continue; }
        if( !world_.testShapePoint( visual->shapeHandle, point ) ) { continue; }
        auto definition = mouseSettings_;
        definition.bodyA = ground; definition.bodyB = visual->bodyHandle; definition.target = point;
        mouseJoint_ = world_.createMouseJoint( definition );
        break;
    }
}
#pragma endregion

std::unique_ptr<demo> createRigidBodyDemo( demoKind kind ) { return std::make_unique<rigidBodyDemo>( kind ); }
} // namespace zonai::sandbox
