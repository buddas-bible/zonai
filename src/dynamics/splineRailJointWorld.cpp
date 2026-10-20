#include "dynamics/world.h"

#include <cmath>
#include <limits>

namespace zonai
{
namespace
{

bool nonnegative( float value )
{
    return std::isfinite( value ) && value >= 0.0f;
}

bool validMode( splineRailRotationMode mode )
{
    return mode == splineRailRotationMode::fixed || mode == splineRailRotationMode::followTangent;
}

}

#pragma region SplineRailLifecycle

jointId world::createSplineRailJoint( const splineRailJointDef& definition )
{
    if( !IsValid( definition.bodyA ) || !IsValid( definition.bodyB ) || definition.bodyA == definition.bodyB ) return {};
    const std::int32_t a = GetBodyIndex( definition.bodyA ), b = GetBodyIndex( definition.bodyB );
    if( bodies_[a].type != bodyType::Dynamic && bodies_[b].type != bodyType::Dynamic ) return {};
    if( !IsFinite( definition.localAnchorB ) || !std::isfinite( definition.referenceDistance ) || std::abs( definition.referenceDistance ) > std::numeric_limits<float>::max() * 0.25 || !std::isfinite( definition.referenceAngle ) || !validMode( definition.rotationMode ) || !nonnegative( definition.angularHertz ) || !nonnegative( definition.angularDampingRatio ) || !nonnegative( definition.hertz ) || !nonnegative( definition.dampingRatio ) || !std::isfinite( definition.targetTranslation ) || !std::isfinite( definition.lowerTranslation ) || !std::isfinite( definition.upperTranslation ) || definition.lowerTranslation > definition.upperTranslation || !std::isfinite( definition.motorSpeed ) || !nonnegative( definition.maxMotorForce ) ) return {};
    splineRailJointSim2 sim{};
    sim.bodyIdA = a;
    sim.bodyIdB = b;
    sim.localAnchorB = definition.localAnchorB;
    sim.referenceDistance = definition.referenceDistance;
    sim.referenceAngle = definition.referenceAngle;
    sim.rotationMode = definition.rotationMode;
    sim.enableAngularSpring = definition.enableAngularSpring;
    sim.angularHertz = definition.angularHertz;
    sim.angularDampingRatio = definition.angularDampingRatio;
    sim.enableSpring = definition.enableSpring;
    sim.hertz = definition.hertz;
    sim.dampingRatio = definition.dampingRatio;
    sim.targetTranslation = definition.targetTranslation;
    sim.enableLimit = definition.enableLimit;
    sim.lowerTranslation = definition.lowerTranslation;
    sim.upperTranslation = definition.upperTranslation;
    sim.enableMotor = definition.enableMotor;
    sim.motorSpeed = definition.motorSpeed;
    sim.maxMotorForce = definition.maxMotorForce;
    if( !setSplineRailGeometry( sim, definition.points ) ) return {};
    const auto frame = getSplineRailFrame( sim, bodySims_[a].transform, bodySims_[b].transform, {}, false );
    if( !frame.valid ) return {};
    sim.parameter = frame.parameter;
    sim.trackingValid = true;
    const rot2 relative = Inverse( bodySims_[a].transform.rotation ) * bodySims_[b].transform.rotation;
    const vec2 tangent = InverseRotate( bodySims_[a].transform.rotation, frame.tangent );
    sim.tangentOffset = std::atan2( relative.s, relative.c ) - std::atan2( tangent.y, tangent.x );
    const std::int32_t index = allocateJoint( a, b, definition.collideConnected );
    sim.jointId = index;
    jointSims_[index] = std::move( sim );
    return makeJointId( index );
}

void world::resetSplineRailJoint( jointId id )
{
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    sim.impulse = {};
    sim.springImpulse = sim.motorImpulse = sim.lowerImpulse = sim.upperImpulse = 0.0f;
    sim.force = {};
    sim.torque = 0.0f;
    if( bodies_[sim.bodyIdA].type != bodyType::Static ) WakeBodyByIndex( sim.bodyIdA );
    if( bodies_[sim.bodyIdB].type != bodyType::Static ) WakeBodyByIndex( sim.bodyIdB );
}

void world::setSplineRailJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio, float targetTranslation )
{
    if( !nonnegative( hertz ) || !nonnegative( dampingRatio ) || !std::isfinite( targetTranslation ) ) return;
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    if( sim.enableSpring == enableSpring && sim.hertz == hertz && sim.dampingRatio == dampingRatio && sim.targetTranslation == targetTranslation ) return;
    sim.enableSpring = enableSpring;
    sim.hertz = hertz;
    sim.dampingRatio = dampingRatio;
    sim.targetTranslation = targetTranslation;
    resetSplineRailJoint( id );
}

void world::setSplineRailJointLimit( jointId id, bool enableLimit, float lowerTranslation, float upperTranslation )
{
    if( !std::isfinite( lowerTranslation ) || !std::isfinite( upperTranslation ) || lowerTranslation > upperTranslation ) return;
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    if( enableLimit && ( upperTranslation < -sim.referenceDistance || lowerTranslation > sim.length - sim.referenceDistance ) ) return;
    if( sim.enableLimit == enableLimit && sim.lowerTranslation == lowerTranslation && sim.upperTranslation == upperTranslation ) return;
    sim.enableLimit = enableLimit;
    sim.lowerTranslation = lowerTranslation;
    sim.upperTranslation = upperTranslation;
    resetSplineRailJoint( id );
}

void world::setSplineRailJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce )
{
    if( !std::isfinite( motorSpeed ) || !nonnegative( maxMotorForce ) ) return;
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    if( sim.enableMotor == enableMotor && sim.motorSpeed == motorSpeed && sim.maxMotorForce == maxMotorForce ) return;
    sim.enableMotor = enableMotor;
    sim.motorSpeed = motorSpeed;
    sim.maxMotorForce = maxMotorForce;
    resetSplineRailJoint( id );
}

void world::setSplineRailJointRotation( jointId id, splineRailRotationMode mode, float angularHertz, float angularDampingRatio, bool enableAngularSpring )
{
    if( !validMode( mode ) || !nonnegative( angularHertz ) || !nonnegative( angularDampingRatio ) ) return;
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    if( sim.rotationMode == mode && sim.angularHertz == angularHertz && sim.angularDampingRatio == angularDampingRatio && sim.enableAngularSpring == enableAngularSpring ) return;
    if( sim.rotationMode != mode && mode == splineRailRotationMode::followTangent )
    {
        const auto frame = getSplineRailFrame( sim, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, sim.parameter, true );
        if( !frame.valid ) return;
        const rot2 relative = Inverse( bodySims_[sim.bodyIdA].transform.rotation ) * bodySims_[sim.bodyIdB].transform.rotation;
        const vec2 tangent = InverseRotate( bodySims_[sim.bodyIdA].transform.rotation, frame.tangent );
        sim.tangentOffset = std::atan2( relative.s, relative.c ) - std::atan2( tangent.y, tangent.x );
    }
    sim.rotationMode = mode;
    sim.enableAngularSpring = enableAngularSpring;
    sim.angularHertz = angularHertz;
    sim.angularDampingRatio = angularDampingRatio;
    resetSplineRailJoint( id );
}

bool world::setSplineRailJointPoints( jointId id, std::span<const vec2> points )
{
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    if( points.size() == sim.points.size() &&
        std::equal( points.begin(), points.end(), sim.points.begin(),
            []( vec2 a, vec2 b )
            {
                return a.x == b.x && a.y == b.y;
            } ) )
        return true;
    auto candidate = sim;
    if( !setSplineRailGeometry( candidate, points ) ) return false;
    const auto frame = getSplineRailFrame( candidate, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, {}, false );
    if( !frame.valid ) return false;
    candidate.parameter = frame.parameter;
    candidate.trackingValid = true;
    sim = std::move( candidate );
    resetSplineRailJoint( id );
    return true;
}

std::span<const vec2> world::getSplineRailJointPoints( jointId id ) const
{
    return std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] ).points;
}

splineRailJointData world::getSplineRailJointData( jointId id ) const
{
    const auto index = getJointIndex( id );
    const auto& sim = std::get<splineRailJointSim2>( jointSims_[index] );
    const auto frame = getSplineRailFrame( sim, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, sim.parameter, true );
    splineRailJointData data{};
    data.bodyA = MakeBodyId( sim.bodyIdA );
    data.bodyB = MakeBodyId( sim.bodyIdB );
    data.length = sim.length;
    data.referenceDistance = sim.referenceDistance;
    data.trackingValid = frame.valid && sim.trackingValid;
    data.parameter = frame.parameter;
    data.anchorA = frame.anchorA;
    data.anchorB = frame.anchorB;
    data.tangent = frame.tangent;
    data.normal = frame.normal;
    data.currentTranslation = frame.translation;
    data.lateralError = frame.lateralError;
    data.currentAngle = frame.angleError;
    data.force = sim.force;
    data.torque = sim.torque;
    data.referenceAngle = sim.referenceAngle;
    data.rotationMode = sim.rotationMode;
    data.enableAngularSpring = sim.enableAngularSpring;
    data.angularHertz = sim.angularHertz;
    data.angularDampingRatio = sim.angularDampingRatio;
    data.enableSpring = sim.enableSpring;
    data.hertz = sim.hertz;
    data.dampingRatio = sim.dampingRatio;
    data.targetTranslation = sim.targetTranslation;
    data.enableLimit = sim.enableLimit;
    data.lowerTranslation = sim.lowerTranslation;
    data.upperTranslation = sim.upperTranslation;
    data.enableMotor = sim.enableMotor;
    data.motorSpeed = sim.motorSpeed;
    data.maxMotorForce = sim.maxMotorForce;
    data.collideConnected = joints_[index].collideConnected;
    data.springForce = sim.subStepTime > 0.0f ? sim.springImpulse / sim.subStepTime : 0.0f;
    data.motorForce = sim.subStepTime > 0.0f ? sim.motorImpulse / sim.subStepTime : 0.0f;
    return data;
}

#pragma endregion SplineRailLifecycle

} // namespace zonai
