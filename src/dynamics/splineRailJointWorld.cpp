#include "dynamics/world.h"

#include <algorithm>
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
    sim.isLoop = definition.isLoop;
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
    if( enableLimit && !sim.isLoop && ( upperTranslation < -sim.referenceDistance || lowerTranslation > sim.length - sim.referenceDistance ) ) return;
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
        const auto frame = getSplineRailFrame( sim, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, sim.parameter, true, sim.distanceOffset );
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

bool world::reprojectSplineRailJointWithinDistance( jointId id, float maxDistance )
{
    const auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    const auto frame = getSplineRailFrame( sim, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, {}, false );
    return applySplineRailProjection( id, frame, maxDistance );
}

bool world::reprojectSplineRailJointWithinDistance( jointId id, std::size_t segmentIndex, float maxDistance )
{
    const auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    const auto frame = getSplineRailFrame( sim, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, {}, false, 0.0, segmentIndex );
    return applySplineRailProjection( id, frame, maxDistance );
}

bool world::reprojectSplineRailJoint( jointId id, std::size_t segmentIndex )
{
    const auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    const auto frame = getSplineRailFrame( sim, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, {}, false, 0.0, segmentIndex );
    return applySplineRailProjection( id, frame );
}

bool world::reprojectSplineRailJoint( jointId id )
{
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    const auto& a = bodySims_[sim.bodyIdA].transform;
    const auto& b = bodySims_[sim.bodyIdB].transform;
    const auto frame = getSplineRailFrame( sim, a, b, {}, false );
    return applySplineRailProjection( id, frame );
}

bool world::applySplineRailProjection( jointId id, const splineRailFrame2& frame, std::optional<float> maxDistance )
{
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    if( !frame.valid ) return false;
    if( maxDistance )
    {
        if( !nonnegative( *maxDistance ) ) return false;
        const double dx = static_cast<double>( frame.anchorB.x ) - frame.anchorA.x, dy = static_cast<double>( frame.anchorB.y ) - frame.anchorA.y;
        const double limit = *maxDistance;
        if( dx * dx + dy * dy > limit * limit ) return false;
    }
    double offset = sim.isLoop ? sim.distanceOffset : 0.0;
    if( sim.isLoop )
    {
        const double previous = getSplineRailDistance( sim, sim.parameter.segmentIndex, sim.parameter.t );
        if( !std::isfinite( previous ) ) return false;
        // 편집으로 생긴 위상 보정을 유지하며 마지막 구간 좌표에 가장 가까운 주회를 고름. 동률은 작은 값임.
        const double delta = previous - frame.distance;
        if( delta > sim.length * 0.5 ) offset += sim.length;
        else if( delta <= -sim.length * 0.5 ) offset -= sim.length;
    }
    const double translation = frame.distance + offset - sim.referenceDistance;
    if( !std::isfinite( offset ) || !std::isfinite( translation ) || std::abs( translation ) > std::numeric_limits<float>::max() ) return false;
    if( sim.trackingValid && sim.parameter.segmentIndex == frame.parameter.segmentIndex && sim.parameter.t == frame.parameter.t && sim.distanceOffset == offset ) return true;
    sim.parameter = frame.parameter;
    sim.distanceOffset = offset;
    sim.trackingValid = true;
    resetSplineRailJoint( id );
    return true;
}

bool world::setSplineRailJointRotationReference( jointId id, float angle )
{
    if( !std::isfinite( angle ) ) return false;
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    float& reference = sim.rotationMode == splineRailRotationMode::fixed ? sim.referenceAngle : sim.tangentOffset;
    if( reference == angle ) return true;
    reference = angle;
    resetSplineRailJoint( id );
    return true;
}

bool world::rebaseSplineRailJointRotation( jointId id )
{
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    const auto& a = bodySims_[sim.bodyIdA].transform;
    const auto& b = bodySims_[sim.bodyIdB].transform;
    const auto frame = getSplineRailFrame( sim, a, b, sim.parameter, true, sim.distanceOffset );
    if( !sim.trackingValid || !frame.valid ) return false;
    const rot2 relative = Inverse( a.rotation ) * b.rotation;
    float angle = std::atan2( relative.s, relative.c );
    if( sim.rotationMode == splineRailRotationMode::followTangent )
    {
        const vec2 tangent = InverseRotate( a.rotation, frame.tangent );
        angle -= std::atan2( tangent.y, tangent.x );
    }
    if( !std::isfinite( angle ) ) return false;
    float& reference = sim.rotationMode == splineRailRotationMode::fixed ? sim.referenceAngle : sim.tangentOffset;
    if( reference == angle ) return true;
    reference = angle;
    resetSplineRailJoint( id );
    return true;
}

bool world::rebaseSplineRailJoint( jointId id )
{
    auto& sim = std::get<splineRailJointSim2>( jointSims_[getJointIndex( id )] );
    const auto frame = getSplineRailFrame( sim, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, sim.parameter, true, sim.distanceOffset );
    if( !sim.trackingValid || !frame.valid ) return false;
    if( frame.translation == 0.0 ) return true;
    if( std::abs( frame.distance ) > std::numeric_limits<float>::max() * 0.25 ) return false;
    const double target = sim.targetTranslation - frame.translation, lower = sim.lowerTranslation - frame.translation, upper = sim.upperTranslation - frame.translation;
    for( double value : { target, lower, upper } )
        if( !std::isfinite( value ) || std::abs( value ) > std::numeric_limits<float>::max() ) return false;
    const float newTarget = static_cast<float>( target ), newLower = static_cast<float>( lower ), newUpper = static_cast<float>( upper );
    if( sim.lowerTranslation < sim.upperTranslation && newLower == newUpper ) return false;
    if( !sim.isLoop )
    {
        if( static_cast<float>( -frame.distance ) == static_cast<float>( sim.length - frame.distance ) ) return false;
        if( sim.enableLimit )
        {
            const double oldLower = std::max( -sim.referenceDistance, static_cast<double>( sim.lowerTranslation ) ), oldUpper = std::min( sim.length - sim.referenceDistance, static_cast<double>( sim.upperTranslation ) );
            const float lowerStop = static_cast<float>( std::max( -frame.distance, static_cast<double>( newLower ) ) ), upperStop = static_cast<float>( std::min( sim.length - frame.distance, static_cast<double>( newUpper ) ) );
            if( oldLower < oldUpper && lowerStop >= upperStop ) return false;
        }
    }
    // 좌표만 평행 이동하므로 물리 상태와 warm-start impulse는 유지함.
    sim.referenceDistance = frame.distance;
    sim.targetTranslation = newTarget;
    sim.lowerTranslation = newLower;
    sim.upperTranslation = newUpper;
    return true;
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
    // 닫힌 경로 편집은 누적 거리를 보존하고 새 투영과의 위상만 맞춤.
    double distance = 0.0;
    if( sim.isLoop )
    {
        const auto before = getSplineRailFrame( sim, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, sim.parameter, true, sim.distanceOffset );
        if( !before.valid ) return false;
        distance = before.distance;
    }
    auto candidate = sim;
    if( !setSplineRailGeometry( candidate, points ) ) return false;
    const auto frame = getSplineRailFrame( candidate, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, {}, false );
    if( !frame.valid ) return false;
    candidate.distanceOffset = sim.isLoop ? distance - frame.distance : 0.0;
    if( !std::isfinite( candidate.distanceOffset ) ) return false;
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
    const auto frame = getSplineRailFrame( sim, bodySims_[sim.bodyIdA].transform, bodySims_[sim.bodyIdB].transform, sim.parameter, true, sim.distanceOffset );
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
    if( data.trackingValid )
    {
        const auto& a = bodyStates_[sim.bodyIdA];
        const auto& b = bodyStates_[sim.bodyIdB];
        const vec2 rA = frame.anchorB - bodySims_[sim.bodyIdA].center, rB = frame.anchorB - bodySims_[sim.bodyIdB].center;
        // solver의 누적 거리 row와 같은 앵커 속도·곡률 보정을 사용함.
        data.currentSpeed = frame.arcScale * ( Dot( frame.tangent, b.linearVelocity - a.linearVelocity ) + Cross( rB, frame.tangent ) * b.angularVelocity - Cross( rA, frame.tangent ) * a.angularVelocity );
        data.currentAngularSpeed = b.angularVelocity - a.angularVelocity;
        if( sim.rotationMode == splineRailRotationMode::followTangent )
        {
            // solver의 회전 row와 같은 곡률·앵커 보정으로 오차 변화율을 구함.
            const float curvature = static_cast<float>( frame.curvature );
            const vec2 axis = -curvature * ( frame.arcScale * frame.tangent );
            const float angularA = 1.0f - curvature * ( frame.arcScale * Cross( rA, frame.tangent ) );
            const float angularB = 1.0f - curvature * ( frame.arcScale * Cross( rB, frame.tangent ) );
            data.currentAngularSpeed = Dot( axis, b.linearVelocity - a.linearVelocity ) + angularB * b.angularVelocity - angularA * a.angularVelocity;
        }
    }
    data.lateralError = frame.lateralError;
    data.currentAngle = frame.angleError;
    data.force = sim.force;
    data.torque = sim.torque;
    data.referenceAngle = sim.referenceAngle;
    data.tangentOffset = sim.tangentOffset;
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
    data.isLoop = sim.isLoop;
    data.springForce = sim.subStepTime > 0.0f ? sim.springImpulse / sim.subStepTime : 0.0f;
    data.motorForce = sim.subStepTime > 0.0f ? sim.motorImpulse / sim.subStepTime : 0.0f;
    return data;
}

#pragma endregion SplineRailLifecycle

} // namespace zonai
