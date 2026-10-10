#include "dynamics/world.h"

#include <cassert>
#include <cmath>

namespace zonai
{

#pragma region PogoJointLifecycle

jointId world::createPogoJoint( const pogoJointDef& definition )
{
    const std::int32_t bodyIndexA = GetBodyIndex( definition.bodyA );
    const std::int32_t bodyIndexB = GetBodyIndex( definition.bodyB );
    assert( bodyIndexA != bodyIndexB );
    assert( bodies_[bodyIndexA].type == bodyType::Dynamic || bodies_[bodyIndexB].type == bodyType::Dynamic );
    assert( IsFinite( definition.localAnchorA ) && IsFinite( definition.localAnchorB ) );
    assert( IsFinite( definition.localPogoAxisB ) && LengthSquared( definition.localPogoAxisB ) > 0.0f );
    assert( IsFinite( definition.normal ) && LengthSquared( definition.normal ) > 0.0f );
    assert( std::isfinite( definition.restLength ) && definition.restLength >= 0.0f );
    assert( std::isfinite( definition.hertz ) && definition.hertz >= 0.0f );
    assert( std::isfinite( definition.dampingRatio ) && definition.dampingRatio >= 0.0f );
    assert( std::isfinite( definition.maxTensionForce ) && definition.maxTensionForce >= 0.0f );
    assert( std::isfinite( definition.maxCompressionForce ) && definition.maxCompressionForce >= 0.0f );
    assert( std::isfinite( definition.impulse ) && std::isfinite( definition.velocity ) );

    const std::int32_t index = allocateJoint( bodyIndexA, bodyIndexB, definition.collideConnected );
    pogoJointSim2 sim{};
    sim.jointId = index;
    sim.bodyIdA = bodyIndexA;
    sim.bodyIdB = bodyIndexB;
    sim.localAnchorA = definition.localAnchorA;
    sim.localAnchorB = definition.localAnchorB;
    sim.localPogoAxisB = Normalize( definition.localPogoAxisB );
    sim.normal = Normalize( definition.normal );
    sim.restLength = definition.restLength;
    sim.hertz = definition.hertz;
    sim.dampingRatio = definition.dampingRatio;
    sim.maxTensionForce = definition.maxTensionForce;
    sim.maxCompressionForce = definition.maxCompressionForce;
    sim.impulse = definition.hertz > 0.0f ? definition.impulse : 0.0f;
    sim.velocity = definition.hertz > 0.0f ? definition.velocity : 0.0f;
    jointSims_[index] = sim;

    return makeJointId( index );
}

void world::setPogoJointSpring( jointId id, float restLength, float hertz, float dampingRatio )
{
    assert( std::isfinite( restLength ) && restLength >= 0.0f );
    assert( std::isfinite( hertz ) && hertz >= 0.0f );
    assert( std::isfinite( dampingRatio ) && dampingRatio >= 0.0f );

    auto& joint = std::get<pogoJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.restLength == restLength && joint.hertz == hertz && joint.dampingRatio == dampingRatio ) return;

    joint.restLength = restLength;
    joint.hertz = hertz;
    joint.dampingRatio = dampingRatio;
    if( hertz == 0.0f )
    {
        joint.impulse = 0.0f;
        joint.velocity = 0.0f;
    }

    if( bodies_[joint.bodyIdA].type != bodyType::Static ) WakeBodyByIndex( joint.bodyIdA );
    if( bodies_[joint.bodyIdB].type != bodyType::Static ) WakeBodyByIndex( joint.bodyIdB );
}

void world::setPogoJointForceLimits( jointId id, float maxTensionForce, float maxCompressionForce )
{
    assert( std::isfinite( maxTensionForce ) && maxTensionForce >= 0.0f );
    assert( std::isfinite( maxCompressionForce ) && maxCompressionForce >= 0.0f );

    auto& joint = std::get<pogoJointSim2>( jointSims_[getJointIndex( id )] );
    if( joint.maxTensionForce == maxTensionForce && joint.maxCompressionForce == maxCompressionForce ) return;

    joint.maxTensionForce = maxTensionForce;
    joint.maxCompressionForce = maxCompressionForce;

    if( bodies_[joint.bodyIdA].type != bodyType::Static ) WakeBodyByIndex( joint.bodyIdA );
    if( bodies_[joint.bodyIdB].type != bodyType::Static ) WakeBodyByIndex( joint.bodyIdB );
}

pogoJointData world::getPogoJointData( jointId id ) const
{
    const std::int32_t index = getJointIndex( id );
    const pogoJointSim2& sim = std::get<pogoJointSim2>( jointSims_[index] );
    const bodySim& bodySimA = bodySims_[sim.bodyIdA];
    const bodySim& bodySimB = bodySims_[sim.bodyIdB];

    pogoJointData data{};
    data.bodyA = MakeBodyId( sim.bodyIdA );
    data.bodyB = MakeBodyId( sim.bodyIdB );
    data.anchorA = TransformPoint( bodySimA.transform, sim.localAnchorA );
    data.anchorB = TransformPoint( bodySimB.transform, sim.localAnchorB );
    data.pogoAxis = Rotate( bodySimB.transform.rotation, sim.localPogoAxisB );
    data.normal = sim.normal;
    data.length = Dot( data.anchorB - data.anchorA, data.pogoAxis );
    data.restLength = sim.restLength;
    data.hertz = sim.hertz;
    data.dampingRatio = sim.dampingRatio;
    data.maxTensionForce = sim.maxTensionForce;
    data.maxCompressionForce = sim.maxCompressionForce;
    data.force = sim.subStepTime > 0.0f ? ( sim.impulse / sim.subStepTime ) * sim.normal : vec2{};
    data.impulse = sim.impulse;
    data.velocity = sim.velocity;
    data.collideConnected = joints_[index].collideConnected;
    return data;
}

#pragma endregion PogoJointLifecycle

} // namespace zonai
