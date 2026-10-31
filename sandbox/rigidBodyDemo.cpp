#include "rigidBodyDemo.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>
#include <variant>

#include "geometry/catmullRomSpline2.h"

namespace zonai::sandbox
{
namespace
{

constexpr double SPLINE_PATH_TOLERANCE = 1.0e-4;
constexpr double SPLINE_PATH_SPEED = 1.0;
constexpr std::array SPLINE_PATH_POINTS{ vec2{ -4.0f, 0.0f }, vec2{ -2.0f, 0.0f }, vec2{ 0.0f, 2.0f }, vec2{ 2.0f, 0.0f }, vec2{ 4.0f, 0.0f } };

constexpr float TERRAIN_POINT_PICK_RADIUS = 0.25f;
constexpr std::array<vec2, 9> TERRAIN_CONTROL_POINTS{
    vec2{ 8.0f, -2.0f },
    vec2{ 6.0f, -1.2f },
    vec2{ 4.0f, -1.0f },
    vec2{ 2.0f, 0.0f },
    vec2{ 0.0f, 0.8f },
    vec2{ -2.0f, 0.1f },
    vec2{ -4.0f, -0.8f },
    vec2{ -6.0f, -0.4f },
    vec2{ -8.0f, -1.2f }
};

} // namespace

#pragma region LifetimeAndSimulation

rigidBodyDemo::rigidBodyDemo( demoKind kind ) : kind_( kind )
{
    shapes_.reserve( 16 );
    contacts_.reserve( 16 );
    if( kind == demoKind::collisionComparison || kind == demoKind::frictionComparison || kind == demoKind::restitutionComparison || kind == demoKind::boxStack || kind == demoKind::ccdComparison || kind == demoKind::collisionFilterComparison || kind == demoKind::sensorWallComparison || kind == demoKind::sleepWakeComparison || kind == demoKind::dampingComparison || kind == demoKind::gravityScaleComparison || kind == demoKind::massImpulseComparison || kind == demoKind::impulsePointComparison || kind == demoKind::rotationalInertiaComparison || kind == demoKind::forceImpulseComparison || kind == demoKind::torqueImpulseComparison || kind == demoKind::dynamicKinematicComparison || kind == demoKind::compoundCenterComparison || kind == demoKind::fastRotationComparison || kind == demoKind::maximumLinearSpeedComparison || kind == demoKind::frictionMixingComparison || kind == demoKind::restitutionMixingComparison || kind == demoKind::lowSpeedRestitutionComparison || kind == demoKind::collisionGroupComparison || kind == demoKind::movingPlatformFrictionComparison || kind == demoKind::rollingSlidingFrictionComparison || kind == demoKind::frictionForceComparison || kind == demoKind::obliqueImpactFrictionComparison || kind == demoKind::collisionMassRatioComparison || kind == demoKind::centralOffsetImpactComparison )
    {
        createCollisionTrial();
    }
    else if( kind == demoKind::playground )
    {
        createPlayground();
    }
    else if( kind == demoKind::windFlow )
    {
        createWindFlow();
    }
    else if( kind == demoKind::terrainChain || kind == demoKind::splineTerrain )
    {
        createTerrainChain();
    }
    else if( kind == demoKind::splineRail )
    {
        createSplineRail();
    }
    else if( kind == demoKind::splinePath )
    {
        createSplinePath();
    }
    else if( kind == demoKind::weldElasticityComparison )
    {
        createWeldElasticityComparison();
    }
    else if( kind == demoKind::suspensionMotorComparison )
    {
        createSuspensionMotorComparison();
    }
    else if( kind == demoKind::suspensionLimitComparison )
    {
        createSuspensionLimitComparison();
    }
    else if( kind == demoKind::suspensionLoadComparison )
    {
        createSuspensionLoadComparison();
    }
    else if( kind == demoKind::revoluteLimitMotorComparison )
    {
        createRevoluteLimitMotorComparison();
    }
    else if( kind == demoKind::revoluteMotorTorqueComparison )
    {
        createRevoluteMotorTorqueComparison();
    }
    else if( kind == demoKind::distanceLimitMotorComparison )
    {
        createDistanceLimitMotorComparison();
    }
    else if( kind == demoKind::distanceMotorForceComparison )
    {
        createDistanceMotorForceComparison();
    }
    else if( kind == demoKind::springDampingComparison )
    {
        createSpringDampingComparison();
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
    else if( kind == demoKind::motorJointPlayground )
    {
        createMotorJointPlayground();
    }
    else if( kind == demoKind::moverJointPlayground )
    {
        createMoverJointPlayground();
    }
    else if( kind == demoKind::pogoJointPlayground )
    {
        createPogoJointPlayground();
    }
    else if( kind == demoKind::filterJointPlayground )
    {
        createFilterJointPlayground();
    }
    else if( kind == demoKind::motorCar )
    {
        createMotorCar();
    }
    else
    {
        assert( kind == demoKind::ragdoll );

        createRagdoll();
    }
    refreshContacts();
}

void rigidBodyDemo::step( float timeStep, int subStepCount )
{
    if( kind_ == demoKind::fastRotationComparison && timeStep > 0.0f ) rotationTrialTimeStep_ = timeStep;
    if( kind_ == demoKind::playground && timeStep > 0.0f && world_.IsValid( shapes_[5].bodyHandle ) )
    {
        const auto platform = shapes_[5].bodyHandle;
        const float x = world_.GetBodyTransform( platform ).position.x;
        const float speed = world_.GetBodyLinearVelocity( platform ).x;
        if( x >= -2.5f && speed > 0.0f ) world_.SetBodyLinearVelocity( platform, { -1.25f, 0.0f } );
        else if( x <= -5.5f && speed < 0.0f ) world_.SetBodyLinearVelocity( platform, { 1.25f, 0.0f } );
    }
    if( kind_ == demoKind::motorJointPlayground && motorTargetTracking_ && timeStep > 0.0f && world_.IsValid( motorJoint_ ) )
    {
        motorTargetTime_ += timeStep;
        const auto joint = world_.getMotorJointData( motorJoint_ );
        const auto current = world_.GetBodyTransform( joint.bodyA );
        const vec2 next{ -2.0f + 1.5f * std::sin( 0.8f * motorTargetTime_ ), 0.75f * std::sin( 1.2f * motorTargetTime_ ) };
        const float nextAngle = 0.6f * std::sin( 0.7f * motorTargetTime_ );
        // Box2D의 목표 transform 갱신처럼, 다음 위치까지 필요한 속도로 kinematic 목표를 이동함.
        world_.SetBodyLinearVelocity( joint.bodyA, ( next - current.position ) / timeStep );
        world_.SetBodyAngularVelocity( joint.bodyA, ( nextAngle - std::atan2( current.rotation.s, current.rotation.c ) ) / timeStep );
    }
    if( kind_ == demoKind::pogoJointPlayground && timeStep > 0.0f && world_.IsValid( impulseBody_ ) )
    {
        updatePogoSupport();
        const float direction = static_cast<float>( rightHeld_ ) - static_cast<float>( leftHeld_ );
        if( world_.IsValid( moverJoint_ ) ) world_.setMoverJointLinearVelocity( moverJoint_, { direction * 3.0f, 0.0f } );
    }
    const bodyId target = kind_ == demoKind::distancePendulum ? pendulumBody_ : impulseBody_;
    const float direction = static_cast<float>( rightHeld_ ) - static_cast<float>( leftHeld_ );

    // 누르고 있는 이동 입력은 매 physics step에 힘으로 적용함. 입력을 취소해도 현재 물리 속도는 유지함.
    if( kind_ != demoKind::motorCar && kind_ != demoKind::pogoJointPlayground && direction != 0.0f && world_.IsValid( target ) )
    {
        const vec2 axis = kind_ == demoKind::prismaticRail && world_.IsValid( prismaticJoint_ ) ? world_.getPrismaticJointData( prismaticJoint_ ).axis : vec2{ 1.0f, 0.0f };
        world_.ApplyForceToCenter( target, direction * world_.GetBodyMass( target ) * 5.0f * axis );
    }

    if( ( kind_ == demoKind::forceImpulseComparison || kind_ == demoKind::frictionForceComparison ) && forceTrialRunning_ && timeStep > 0.0f )
    {
        const double duration = std::min( static_cast<double>( timeStep ), std::max( 0.0, 1.0 - forceTrialTime_ ) );
        // 끝점을 넘는 step도 총 1초분의 힘만 전달함.
        if( kind_ == demoKind::forceImpulseComparison )
        {
            if( duration > 0.0 && world_.IsValid( shapes_[0].bodyHandle ) ) world_.ApplyForceToCenter( shapes_[0].bodyHandle, { 0, 2.0f * static_cast<float>( duration / timeStep ) } );
        }
        else if( duration > 0.0 )
        {
            for( std::size_t i = 0; i < 2; ++i )
                if( world_.IsValid( shapes_[2 * i + 1].bodyHandle ) ) world_.ApplyForceToCenter( shapes_[2 * i + 1].bodyHandle, { ( i == 0 ? 2.0f : 8.0f ) * static_cast<float>( duration / timeStep ), 0 } );
        }
        forceTrialTime_ += timeStep;
    }
    if( kind_ == demoKind::torqueImpulseComparison && torqueTrialRunning_ && timeStep > 0.0f )
    {
        const double duration = std::min( static_cast<double>( timeStep ), std::max( 0.0, 1.0 - torqueTrialTime_ ) );
        // 끝점을 넘는 step도 총 1초분의 토크만 전달함.
        if( duration > 0.0 && world_.IsValid( shapes_[0].bodyHandle ) ) world_.ApplyTorque( shapes_[0].bodyHandle, static_cast<float>( duration / timeStep ) );
        torqueTrialTime_ += timeStep;
    }
    updateSplinePath( timeStep );
    // Fluid와 강체의 외부 Step을 동일한 dt로 교대 전진시킨다. 이 옵션은
    // Sandbox 전용의 시간 동기화 실험이며 implicit added-mass 결합은 아니다.
    // Cut-cell은 아직 Body 유효질량을 압력 행렬에 결합하지 않으므로
    // explicit 압력 반력을 더 자주 주입하는 동기화를 사용하지 않는다.
    const bool synchronizedWind = windField_ && windObstacleCouplingEnabled_ &&
        windSynchronizedCouplingEnabled_ && !windField_->isCutCellProjectionEnabled() &&
        timeStep > 0.0f && subStepCount > 1;
    const int windSteps = synchronizedWind ? subStepCount : 1;
    const float windTimeStep = timeStep / windSteps;
    windCouplingSteps_ = windField_ && timeStep > 0.0f ? windSteps : 0;
    if( windField_ )
    {
        windDragBodyImpulse_ = {};
        windDragFluidImpulse_ = {};
    }
    std::array<fluidObstacleForce2, 3> summedPressureImpulse{};
    if( contactEventVisualizationEnabled_ ) syncContactEventSubscriptions();
    for( int windSubstep = 0; windSubstep < windSteps; ++windSubstep )
    {
        if( windField_ && windTimeStep > 0.0f )
        {
            windTime_ += windTimeStep;
            if( windSourceEnabled_ )
            {
                const vec2 source{ -3.5f, 0.0f };
                const vec2 velocityDelta{ windSourceStrength_ * windTimeStep, 0.5f * windSourceStrength_ * windTimeStep * std::sin( 1.7f * windTime_ ) };
                windField_->addVelocity( source, velocityDelta, 1.7f );
            }
            // fluidField2는 World나 Body를 소유하지 않음. 매 Step마다 실제 pose/속도를 전달함.
            std::array<fluidShapeObstacle2, 3> obstacles{};
            const std::size_t obstacleCount = std::min( shapes_.size(), obstacles.size() );
            for( std::size_t i = 0; i < obstacleCount; ++i )
            {
                const visualShape& visual = shapes_[i];
                if( !world_.IsValid( visual.bodyHandle ) || !world_.IsValid( visual.shapeHandle ) ) continue;
                const shapeGeometry& geometry = world_.GetShape( visual.shapeHandle ).geometry;
                if( !std::holds_alternative<circle2>( geometry ) && !std::holds_alternative<capsule2>( geometry ) &&
                    !std::holds_alternative<polygon2>( geometry ) ) continue;
                const transform2 transform = world_.GetBodyTransform( visual.bodyHandle );
                const vec2 center = TransformPoint( transform, world_.GetBodyLocalCenter( visual.bodyHandle ) );
                obstacles[i] = { geometry, transform, center,
                    world_.GetBodyLinearVelocity( visual.bodyHandle ), world_.GetBodyAngularVelocity( visual.bodyHandle ) };
                // Time-synchronized binary pressure path: Body effective inverse mass
                // enters the same projection operator as fluid face corrections.
                if( synchronizedWind && !windField_->isCutCellProjectionEnabled() &&
                    world_.GetBody( visual.bodyHandle ).type == bodyType::Dynamic )
                {
                    const float mass = world_.GetBodyMass( visual.bodyHandle );
                    const float inertia = world_.GetBodyRotationalInertia( visual.bodyHandle );
                    obstacles[i].inverseMass = mass > 0.0f ? 1.0f / mass : 0.0f;
                    obstacles[i].inverseInertia = inertia > 0.0f ? 1.0f / inertia : 0.0f;
                }
            }
    
            windPressureForces_.fill( {} );
            if( windObstacleCouplingEnabled_ )
                windField_->step( windTimeStep, std::span<const fluidShapeObstacle2>( obstacles ).first( obstacleCount ),
                    std::span<fluidObstacleForce2>( windPressureForces_ ).first( obstacleCount ) );
            else windField_->step( windTimeStep );
    
            // Drag는 기존 유체 face를 갱신하기 전에 모든 Body의 상대속도로 계산한다.
            // Solid 내부 샘플은 경계 속도에 오염되므로 바깥쪽 네 방향의 흐름을 사용.
            std::array<vec2, 3> dragForces{};
            std::array<vec2, 3> dragCenters{};
            std::array<float, 3> dragRadii{};
            const float spacing = windField_->getCellSize();
    
            for( std::size_t i = 0; i < obstacleCount; ++i )
            {
                const visualShape& visual = shapes_[i];
                if( !world_.IsValid( visual.bodyHandle ) ||
                    world_.GetBody( visual.bodyHandle ).type != bodyType::Dynamic ) continue;
    
                const vec2 center = TransformPoint( world_.GetBodyTransform( visual.bodyHandle ),
                    world_.GetBodyLocalCenter( visual.bodyHandle ) );
                const float radius = std::max( Length( Extents( world_.GetShapeAABB( visual.shapeHandle ) ) ), spacing );
                const float distance = radius + spacing;
                vec2 fluidVelocity = windField_->sampleVelocity( center );
                if( windObstacleCouplingEnabled_ )
                {
                    fluidVelocity = 0.25f * (
                        windField_->sampleVelocity( center + vec2{ distance, 0.0f } ) +
                        windField_->sampleVelocity( center - vec2{ distance, 0.0f } ) +
                        windField_->sampleVelocity( center + vec2{ 0.0f, distance } ) +
                        windField_->sampleVelocity( center - vec2{ 0.0f, distance } ) );
                }
    
                dragCenters[i] = center;
                dragRadii[i] = radius;
                dragForces[i] = windDragCoefficient_ * ( fluidVelocity - world_.GetBodyLinearVelocity( visual.bodyHandle ) );
            }
    
            // 압력 traction은 별도 근사. 여기서는 Drag에 한해 유체/Body 충격량을 쌍으로 적용.
            // World::Step에서 Body force가 소비되기 전에, 유체가 실제로 받은 -J 만큼만 Body에 +J를 줌.
            for( std::size_t i = 0; i < obstacleCount; ++i )
            {
                const visualShape& visual = shapes_[i];
                if( !world_.IsValid( visual.bodyHandle ) ||
                    world_.GetBody( visual.bodyHandle ).type != bodyType::Dynamic ) continue;
    
                if( windObstacleCouplingEnabled_ )
                {
                    world_.ApplyForceToCenter( visual.bodyHandle, windPressureForces_[i].force );
                    world_.ApplyTorque( visual.bodyHandle, windPressureForces_[i].torque );
                }
    
                vec2 dragForce = dragForces[i];
                if( windObstacleCouplingEnabled_ )
                {
                    const vec2 upstream = dragCenters[i] -
                        ( dragRadii[i] + spacing ) * Normalize( dragForce );
                    const vec2 fluidImpulse = windField_->applyMomentumImpulse(
                        upstream, -windTimeStep * dragForce, 2.0f * spacing );
                    // 벽 또는 Solid에 가려 유효 face가 없으면 그 성분의 drag도 전달하지 않음.
                    dragForce = -fluidImpulse / windTimeStep;
                    windDragFluidImpulse_ += fluidImpulse;
                }
    
                world_.ApplyForceToCenter( visual.bodyHandle, dragForce );
                windDragBodyImpulse_ += windTimeStep * dragForce;
            }
        }
        if( synchronizedWind )
        {
            for( std::size_t i = 0; i < windPressureForces_.size(); ++i )
            {
                summedPressureImpulse[i].force += windTimeStep * windPressureForces_[i].force;
                summedPressureImpulse[i].torque += windTimeStep * windPressureForces_[i].torque;
            }
            // Contact/Joint solver 역시 같은 substep 경계에서 전진시켜
            // 다음 Fluid Step이 최신 위치·속도로 경계조건을 다시 계산하게 한다.
            world_.Step( windTimeStep, 1 );
        }
    }
    if( synchronizedWind )
        for( std::size_t i = 0; i < windPressureForces_.size(); ++i )
        {
            windPressureForces_[i].force = summedPressureImpulse[i].force / timeStep;
            windPressureForces_[i].torque = summedPressureImpulse[i].torque / timeStep;
        }
    // 새로 생성된 Shape에도 구독을 적용하되 표시를 끄면 순회 비용을 내지 않음.
    if( !synchronizedWind ) world_.Step( timeStep, subStepCount );
    if( contactEventVisualizationEnabled_ ) updateContactEventTrail( timeStep );
    if( kind_ == demoKind::splinePath && splinePathDistance_ == splinePathLength_ && world_.IsValid( impulseBody_ ) )
    {
        world_.SetBodyLinearVelocity( impulseBody_, {} );
        world_.SetBodyAngularVelocity( impulseBody_, 0.0f );
    }
    refreshContacts();
    updateTerrainTrialObservation( timeStep, subStepCount );
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
        stopMouseDrag();
    }

    if( !input.mouseHeld )
    {
        terrainPointDragIndex_.reset();
        splineDragRecorded_ = false;
        splineTrialDragging_ = false;
        splineInserting_ = false;
    }
    else if( kind_ == demoKind::terrainChain || hasEditableSpline() )
    {
        if( input.mousePressed )
        {
            splineInserting_ = hasEditableSpline() && input.insertControlPoint;
            splineTrialDragging_ = kind_ == demoKind::splineTerrain && input.selectTrialPosition && !splineInserting_;
            if( splineInserting_ || splineTrialDragging_ )
            {
                terrainPointDragIndex_.reset();
                splineDragRecorded_ = false;
                endSplineSamplingEdit();
                stopMouseDrag();
                if( splineInserting_ ) insertSplineControlPointAt( input.mousePosition, input.pixelsPerMeter );
            }
            else startTerrainPointDrag( input.mousePosition );
        }
        if( splineTrialDragging_ ) updateSplineTrialPosition( input.mousePosition );
        else if( !splineInserting_ && terrainPointDragIndex_.has_value() )
        {
            updateTerrainPointDrag( input.mousePosition );
        }
    }

    // Terrain 편집 / 시험 위치 선택은 같은 왼쪽 입력으로 Mouse Joint를 만들지 않음.
    if( !input.mouseHeld )
    {
        stopMouseDrag();
    }
    else if( !terrainPointDragIndex_.has_value() && !splineTrialDragging_ && !splineInserting_ )
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
    if( kind_ != demoKind::motorCar && input.jumpPressed && world_.IsValid( target ) &&
        ( kind_ != demoKind::pogoJointPlayground || ( world_.IsValid( pogoJoint_ ) && !pogoJumping_ ) ) )
    {
        world_.ApplyLinearImpulseToCenter( target, kind_ == demoKind::distancePendulum ? vec2{ mass * 2.0f, 0.0f } : vec2{ 0.0f, mass * 5.0f } );
        if( kind_ == demoKind::pogoJointPlayground ) pogoJumping_ = true;
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
    terrainPointDragIndex_.reset();
    splineDragRecorded_ = false;
    splineTrialDragging_ = false;
    splineInserting_ = false;
    updateCarMotors();
    stopMouseDrag();
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

void rigidBodyDemo::setContactEventVisualizationEnabled( bool enabled )
{
    if( contactEventVisualizationEnabled_ == enabled ) return;

    contactEventVisualizationEnabled_ = enabled;
    clearContactEventTrail();
    syncContactEventSubscriptions();
}

void rigidBodyDemo::setContactEventTrailDuration( float seconds )
{
    if( !std::isfinite( seconds ) ) return;
    contactEventTrailDuration_ = std::clamp( seconds, 0.05f, 2.0f );
}

void rigidBodyDemo::syncContactEventSubscriptions()
{
    for( const visualShape& visual : shapes_ )
    {
        const shapeId id = visual.shapeHandle;
        if( !world_.IsValid( id ) || world_.IsShapeSensor( id ) ) continue;

        if( world_.AreShapeContactEventsEnabled( id ) != contactEventVisualizationEnabled_ )
            world_.SetShapeContactEventsEnabled( id, contactEventVisualizationEnabled_ );
        if( world_.areShapeHitEventsEnabled( id ) != contactEventVisualizationEnabled_ )
            world_.setShapeHitEventsEnabled( id, contactEventVisualizationEnabled_ );
    }
}

void rigidBodyDemo::updateContactEventTrail( float timeStep )
{
    // World event의 span은 다음 Step에서 사라지므로 표시 위치를 여기서 복사함.
    const float elapsed = std::max( timeStep, 0.0f );
    for( contactEventTrace2& trace : contactEventTraces_ ) trace.remainingTime -= elapsed;
    std::erase_if( contactEventTraces_, []( const contactEventTrace2& trace ) { return trace.remainingTime <= 0.0f; } );

    const auto recordPair = [&]( shapeId a, shapeId b, contactEventTraceType type )
    {
        // 파괴된 Shape의 End는 위치를 추정할 수 없으므로 기록하지 않음.
        if( !world_.IsValid( a ) || !world_.IsValid( b ) ) return;
        const vec2 point = 0.5f * ( Center( world_.GetShapeAABB( a ) ) + Center( world_.GetShapeAABB( b ) ) );
        contactEventTraces_.push_back( { type, point, {}, 0.0f, 0.0f, contactEventTrailDuration_, contactEventTrailDuration_ } );
    };

    for( const contactBeginEvent2& event : world_.GetContactBeginEvents() )
        recordPair( event.shapeA, event.shapeB, contactEventTraceType::begin );

    for( const contactEndEvent2& event : world_.GetContactEndEvents() )
        recordPair( event.shapeA, event.shapeB, contactEventTraceType::end );

    for( const contactHitEvent2& hit : world_.getContactHitEvents() )
        contactEventTraces_.push_back( { contactEventTraceType::hit, hit.point, hit.normal,
            hit.approachSpeed, hit.normalImpulse, contactEventTrailDuration_, contactEventTrailDuration_ } );

    // 많은 충돌이 한 Step에 발생해도 Sandbox의 디버그 기록은 상한을 지킴.
    constexpr std::size_t MAX_TRACES = 256;
    if( contactEventTraces_.size() > MAX_TRACES )
        contactEventTraces_.erase( contactEventTraces_.begin(),
            contactEventTraces_.begin() + static_cast<std::ptrdiff_t>( contactEventTraces_.size() - MAX_TRACES ) );
}

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

#pragma region Queries

std::size_t rigidBodyDemo::getTerrainPointCount() const noexcept
{
    return world_.IsValid( terrainChain_ ) ? terrainSegments_.size() + 1 : 0;
}

vec2 rigidBodyDemo::getTerrainPoint( std::size_t pointIndex ) const
{
    const std::size_t pointCount = getTerrainPointCount();
    assert( pointIndex < pointCount );
    if( pointIndex >= pointCount ) return {};

    const shapeId segmentId = pointIndex < terrainSegments_.size() ? terrainSegments_[pointIndex] : terrainSegments_.back();
    assert( world_.IsValid( segmentId ) );
    if( !world_.IsValid( segmentId ) ) return {};

    const chainSegment2& segment = std::get<chainSegment2>( world_.GetShape( segmentId ).geometry );
    return pointIndex < terrainSegments_.size() ? segment.segment.a : segment.segment.b;
}

std::span<const vec2> rigidBodyDemo::getSplinePathPoints() const noexcept
{
    return kind_ == demoKind::splinePath ? std::span<const vec2>{ splineControlPoints_ } : std::span<const vec2>{};
}

bool rigidBodyDemo::hasEditableSpline() const noexcept
{
    if( kind_ == demoKind::splineTerrain ) return world_.IsValid( terrainChain_ );
    if( kind_ == demoKind::splinePath ) return world_.IsValid( impulseBody_ );
    return kind_ == demoKind::splineRail && world_.IsValid( splineRailJoint_ );
}

transform2 rigidBodyDemo::getSplineControlTransform() const
{
    if( kind_ == demoKind::splineTerrain && !shapes_.empty() ) return world_.GetBodyTransform( shapes_[0].bodyHandle );
    if( kind_ == demoKind::splineRail && world_.IsValid( splineRailBody_ ) ) return world_.GetBodyTransform( splineRailBody_ );
    return {};
}

#pragma endregion Queries

#pragma region SceneSetup

void rigidBodyDemo::createSplinePath()
{
    splineControlPoints_.assign( SPLINE_PATH_POINTS.begin(), SPLINE_PATH_POINTS.end() );
    const catmullRomSpline2 path{ splineControlPoints_ };
    auto geometry = prepareSpline( path );
    const auto length = GetLength( path, SPLINE_PATH_TOLERANCE );
    if( !geometry || !length ) return;
    splinePathGeometry_ = std::move( *geometry );
    const auto start = getSplinePathSample( 0.0 );
    if( !start ) return;
    splinePathLength_ = *length;
    impulseBody_ = world_.CreateBody( bodyType::Kinematic, { start->point, rot2::FromRadians( std::atan2( start->tangent.y, start->tangent.x ) ) } );
    const auto shape = world_.CreateShape( impulseBody_, MakeBox( { 0.6f, 0.2f } ) );
    shapes_.push_back( { impulseBody_, shape, "경로 이동 [키네마틱]" } );
}

void rigidBodyDemo::createSplineRail()
{
    splineControlPoints_.assign( SPLINE_PATH_POINTS.begin(), SPLINE_PATH_POINTS.end() );
    splineRailBody_ = world_.CreateBody( bodyType::Static );
    impulseBody_ = world_.CreateBody( bodyType::Dynamic, { splineControlPoints_.front(), {} } );
    const auto shape = world_.CreateShape( impulseBody_, MakeBox( { 0.6f, 0.2f } ) );
    shapes_.push_back( { impulseBody_, shape, "스플라인 레일 [다이나믹]" } );
    splineRailJointDef definition{};
    definition.bodyA = splineRailBody_;
    definition.bodyB = impulseBody_;
    definition.points = splineControlPoints_;
    definition.enableSpring = false;
    definition.hertz = 2.0f;
    definition.dampingRatio = 0.7f;
    const auto length = GetLength( { splineControlPoints_ }, SPLINE_PATH_TOLERANCE );
    if( !length ) return;
    definition.targetTranslation = static_cast<float>( *length * 0.5 );
    splineRailJoint_ = world_.createSplineRailJoint( definition );
    splinePathLength_ = *length;
}

std::optional<splineSample2> rigidBodyDemo::getSplinePathSample( double distance ) const
{
    const std::size_t count = splinePathGeometry_.getSegmentCount();
    if( count == 0 ) return {};

    // 거리 역조회와 오차 예산은 기존 EvaluateByDistance의 전체 범위 경로를 유지함.
    const auto parameter = GetParameterByDistance( { splineControlPoints_ }, { 0, 0.0f }, { count - 1, 1.0f }, distance, SPLINE_PATH_TOLERANCE );
    if( !parameter ) return {};
    return splinePathGeometry_.evaluateSample( parameter->segmentIndex, parameter->t, SPLINE_PATH_TOLERANCE );
}

void rigidBodyDemo::updateSplinePath( float timeStep )
{
    if( kind_ != demoKind::splinePath || timeStep <= 0.0f || !world_.IsValid( impulseBody_ ) ) return;
    world_.SetBodyLinearVelocity( impulseBody_, {} );
    world_.SetBodyAngularVelocity( impulseBody_, 0.0f );
    if( splinePathDistance_ == splinePathLength_ && !splinePathNeedsAlignment_ ) return;
    const double next = std::min( splinePathLength_, splinePathDistance_ + SPLINE_PATH_SPEED * timeStep );
    const auto target = getSplinePathSample( next );
    if( !target ) return;
    const auto current = world_.GetBodyTransform( impulseBody_ );
    const vec2 velocity = ( target->point - current.position ) / timeStep;
    const float angle = std::remainder( std::atan2( target->tangent.y, target->tangent.x ) - std::atan2( current.rotation.s, current.rotation.c ), 2.0f * std::numbers::pi_v<float> );
    const float angularVelocity = angle / timeStep;
    if( !IsFinite( velocity ) || !std::isfinite( angularVelocity ) ) return;
    // 다음 조회 위치까지 필요한 속도를 주어 기존 kinematic step으로 이동함.
    world_.SetBodyLinearVelocity( impulseBody_, velocity );
    world_.SetBodyAngularVelocity( impulseBody_, angularVelocity );
    splinePathDistance_ = next;
    splinePathNeedsAlignment_ = false;
}

void rigidBodyDemo::resetSplinePath()
{
    if( kind_ != demoKind::splinePath || !world_.IsValid( impulseBody_ ) ) return;
    const auto start = getSplinePathSample( 0.0 );
    if( !start ) return;
    cancelInput();
    world_.SetBodyTransform( impulseBody_, { start->point, rot2::FromRadians( std::atan2( start->tangent.y, start->tangent.x ) ) } );
    world_.SetBodyLinearVelocity( impulseBody_, {} );
    world_.SetBodyAngularVelocity( impulseBody_, 0.0f );
    splinePathDistance_ = 0.0;
    refreshContacts();
}

void rigidBodyDemo::createCollisionTrial()
{
    if( kind_ == demoKind::collisionComparison )
    {
        world_.SetGravity( {} );
        const shapeGeometry geometries[] = { circle2{ {}, 0.5f }, MakeBox( { 0.5f, 0.5f } ), capsule2{ { -0.3f, 0 }, { 0.3f, 0 }, 0.3f } };
        const char* labels[][2] = { { "원 A", "원 B" }, { "상자 A", "상자 B" }, { "캡슐 A", "캡슐 B" } };
        for( std::size_t i = 0; i < 3; ++i )
            for( std::size_t j = 0; j < 2; ++j )
            {
                const auto body = world_.CreateBody( bodyType::Dynamic, { { j == 0 ? -3.0f : 3.0f, 2.0f - 2.0f * static_cast<float>( i ) }, {} } );
                const auto shape = world_.CreateShape( body, geometries[i] );
                world_.SetShapeFriction( shape, 0.0f );
                world_.SetShapeRestitution( shape, 0.5f );
                world_.SetBodyLinearVelocity( body, { j == 0 ? 3.0f : -3.0f, 0 } );
                shapes_.push_back( { body, shape, labels[i][j] } );
            }
    }
    else if( kind_ == demoKind::frictionComparison )
    {
        const float friction[] = { 0.0f, 0.1f, 0.8f };
        const char* ramps[] = { "경사면 A", "경사면 B", "경사면 C" };
        const char* boxes[] = { "상자 A (초기 마찰 0)", "상자 B (초기 마찰 0.1)", "상자 C (초기 마찰 0.8)" };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const transform2 rampPose{ { 0, 3.0f - 3.0f * static_cast<float>( i ) }, rot2::FromRadians( -0.35f ) };
            const auto ramp = world_.CreateBody( bodyType::Static, rampPose );
            const auto surface = world_.CreateShape( ramp, MakeBox( { 3.5f, 0.15f } ) );
            world_.SetShapeFriction( surface, 1.0f );
            shapes_.push_back( { ramp, surface, ramps[i] } );
            const auto box = world_.CreateBody( bodyType::Dynamic, { TransformPoint( rampPose, { -2.0f, 0.65f } ), rampPose.rotation } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.4f, 0.4f } ) );
            world_.SetShapeFriction( shape, friction[i] );
            shapes_.push_back( { box, shape, boxes[i] } );
        }
    }
    else if( kind_ == demoKind::restitutionComparison )
    {
        const auto floor = world_.CreateBody( bodyType::Static, { { 0, -0.5f }, {} } );
        const auto surface = world_.CreateShape( floor, MakeBox( { 5.0f, 0.5f } ) );
        world_.SetShapeRestitution( surface, 0.0f );
        shapes_.push_back( { floor, surface, "바닥" } );
        const float restitution[] = { 0.0f, 0.5f, 1.0f };
        const char* labels[] = { "원 A (초기 반발 0)", "원 B (초기 반발 0.5)", "원 C (초기 반발 1)" };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const auto circle = world_.CreateBody( bodyType::Dynamic, { { -3.0f + 3.0f * static_cast<float>( i ), 3.0f }, {} } );
            const auto shape = world_.CreateShape( circle, circle2{ {}, 0.5f } );
            world_.SetShapeRestitution( shape, restitution[i] );
            shapes_.push_back( { circle, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::boxStack )
    {
        const auto floor = world_.CreateBody( bodyType::Static, { { 0, -0.5f }, {} } );
        const auto surface = world_.CreateShape( floor, MakeBox( { 5.0f, 0.5f } ) );
        world_.SetShapeFriction( surface, 0.6f );
        shapes_.push_back( { floor, surface, "바닥" } );
        const char* labels[] = { "상자 1", "상자 2", "상자 3", "상자 4", "상자 5", "상자 6", "상자 7", "상자 8" };
        for( std::size_t i = 0; i < 8; ++i )
        {
            const auto box = world_.CreateBody( bodyType::Dynamic, { { 0, 0.6f + 1.1f * static_cast<float>( i ) }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.5f } ) );
            world_.SetShapeFriction( shape, 0.6f );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::ccdComparison )
    {
        world_.SetGravity( {} );
        const auto wall = world_.CreateBody( bodyType::Static );
        const auto surface = world_.CreateShape( wall, MakeBox( { 0.05f, 2.0f } ) );
        world_.SetShapeFriction( surface, 0.0f );
        shapes_.push_back( { wall, surface, "벽" } );
        const auto projectile = world_.CreateBody( bodyType::Dynamic, { { -0.8f, 0 }, {} } );
        const auto shape = world_.CreateShape( projectile, circle2{ {}, 0.1f } );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetBodyLinearVelocity( projectile, { 100.0f, 0 } );
        shapes_.push_back( { projectile, shape, "원" } );
    }
    else if( kind_ == demoKind::collisionFilterComparison )
    {
        world_.SetGravity( {} );
        const char* labels[][2] = { { "위 A (초기 레이어 01)", "위 B (초기 레이어 02)" }, { "아래 A (초기 레이어 01)", "아래 B (초기 레이어 02)" } };
        for( std::size_t i = 0; i < 2; ++i )
            for( std::size_t j = 0; j < 2; ++j )
            {
                const auto body = world_.CreateBody( bodyType::Dynamic, { { j == 0 ? -2.0f : 2.0f, i == 0 ? 1.5f : -1.5f }, {} } );
                const auto shape = world_.CreateShape( body, circle2{ {}, 0.35f } );
                world_.SetShapeFriction( shape, 0.0f );
                world_.SetShapeRestitution( shape, 1.0f );
                collisionFilter filter;
                filter.categoryBits = std::uint64_t{ 1 } << j;
                if( i == 1 && j == 0 ) filter.maskBits &= ~std::uint64_t{ 2 };
                world_.SetShapeFilter( shape, filter );
                world_.SetBodyLinearVelocity( body, { j == 0 ? 2.0f : -2.0f, 0 } );
                shapes_.push_back( { body, shape, labels[i][j] } );
            }
    }
    else if( kind_ == demoKind::sensorWallComparison )
    {
        world_.SetGravity( {} );
        const char* labels[][2] = { { "일반 벽", "위 원" }, { "센서", "아래 원" } };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const float y = i == 0 ? 1.5f : -1.5f;
            const auto wall = world_.CreateBody( bodyType::Static, { { 0, y }, {} } );
            const auto surface = i == 0 ? world_.CreateShape( wall, MakeBox( { 0.75f, 0.6f } ) ) : world_.CreateSensorShape( wall, MakeBox( { 0.75f, 0.6f } ) );
            world_.SetShapeFriction( surface, 0.0f );
            shapes_.push_back( { wall, surface, labels[i][0] } );
            const auto body = world_.CreateBody( bodyType::Dynamic, { { -3.0f, y }, {} } );
            const auto shape = world_.CreateShape( body, circle2{ {}, 0.35f } );
            if( i == 1 )
            {
                world_.SetShapeSensorEventsEnabled( surface, true );
                world_.SetShapeSensorEventsEnabled( shape, true );
            }
            world_.SetShapeFriction( shape, 0.0f );
            world_.SetBodyLinearVelocity( body, { 2.0f, 0 } );
            shapes_.push_back( { body, shape, labels[i][1] } );
        }
    }
    else if( kind_ == demoKind::sleepWakeComparison )
    {
        const auto floor = world_.CreateBody( bodyType::Static, { { 0, -0.5f }, {} } );
        const auto surface = world_.CreateShape( floor, MakeBox( { 5.0f, 0.5f } ) );
        world_.SetShapeFriction( surface, 0.6f );
        shapes_.push_back( { floor, surface, "바닥" } );
        const char* labels[] = { "왼쪽 (초기 수면 허용)", "오른쪽 (초기 수면 금지)" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto box = world_.CreateBody( bodyType::Dynamic, { { i == 0 ? -2.0f : 2.0f, 1.5f }, {} } );
            world_.SetBodySleepEnabled( box, i == 0 );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.5f } ) );
            world_.SetShapeFriction( shape, 0.6f );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::dampingComparison )
    {
        world_.SetGravity( {} );
        const float damping[] = { 0.0f, 0.5f, 2.0f };
        const char* labels[] = { "상자 A (초기 감쇠 0)", "상자 B (초기 감쇠 0.5)", "상자 C (초기 감쇠 2)" };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const auto box = world_.CreateBody( bodyType::Dynamic, { { -3.0f + 3.0f * static_cast<float>( i ), 0 }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.25f } ) );
            world_.SetBodyLinearDamping( box, damping[i] );
            world_.SetBodyAngularDamping( box, damping[i] );
            world_.SetBodySleepEnabled( box, false );
            world_.SetBodyLinearVelocity( box, { 0, 2.0f } );
            world_.SetBodyAngularVelocity( box, 2.0f );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::gravityScaleComparison )
    {
        world_.SetGravity( { 0, -10.0f } );
        const float gravityScale[] = { 0.0f, 0.5f, 1.0f };
        const char* labels[] = { "상자 A (초기 배율 0)", "상자 B (초기 배율 0.5)", "상자 C (초기 배율 1)" };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const auto box = world_.CreateBody( bodyType::Dynamic, { { -3.0f + 3.0f * static_cast<float>( i ), 3.0f }, {} } );
            world_.SetBodyGravityScale( box, gravityScale[i] );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.5f } ) );
            world_.SetBodySleepEnabled( box, false );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::massImpulseComparison )
    {
        world_.SetGravity( {} );
        const float density[] = { 1.0f, 2.0f, 4.0f };
        const char* labels[] = { "상자 A (초기 밀도 1)", "상자 B (초기 밀도 2)", "상자 C (초기 밀도 4)" };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const auto box = world_.CreateBody( bodyType::Dynamic, { { -3.0f + 3.0f * static_cast<float>( i ), 0 }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.5f } ), {}, density[i] );
            world_.SetBodySleepEnabled( box, false );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::impulsePointComparison )
    {
        world_.SetGravity( {} );
        const char* labels[] = { "상자 A (중심)", "상자 B (로컬 왼쪽)", "상자 C (로컬 오른쪽)" };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const auto box = world_.CreateBody( bodyType::Dynamic, { { -3.0f + 3.0f * static_cast<float>( i ), 0 }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.5f } ) );
            world_.SetBodySleepEnabled( box, false );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::rotationalInertiaComparison )
    {
        world_.SetGravity( {} );
        const char* labels[] = { "정사각형 (1×1m)", "막대 (2×0.5m)" };
        const vec2 extents[] = { { 0.5f, 0.5f }, { 1.0f, 0.25f } };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto box = world_.CreateBody( bodyType::Dynamic, { { i == 0 ? -2.5f : 2.5f, 0 }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( extents[i] ) );
            world_.SetBodySleepEnabled( box, false );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::forceImpulseComparison || kind_ == demoKind::torqueImpulseComparison )
    {
        world_.SetGravity( {} );
        const char* labels[] = { kind_ == demoKind::torqueImpulseComparison ? "상자 A (토크 1N·m·1초)" : "상자 A (힘 2N·1초)", kind_ == demoKind::torqueImpulseComparison ? "상자 B (회전 충격량 1N·m·s)" : "상자 B (충격량 2N·s)" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto box = world_.CreateBody( bodyType::Dynamic, { { i == 0 ? -2.5f : 2.5f, 0 }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.5f } ) );
            world_.SetBodySleepEnabled( box, false );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::dynamicKinematicComparison )
    {
        world_.SetGravity( {} );
        const char* labels[] = { "위 A (동적 이동)", "위 B (동적 표적)", "아래 A (키네마틱 이동)", "아래 B (동적 표적)" };
        for( std::size_t i = 0; i < 4; ++i )
        {
            const auto box = world_.CreateBody( i == 2 ? bodyType::Kinematic : bodyType::Dynamic, { { i % 2 == 0 ? -2.0f : 0.0f, i < 2 ? 1.5f : -1.5f }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.5f } ) );
            world_.SetShapeFriction( shape, 0.0f );
            world_.SetShapeRestitution( shape, 0.0f );
            world_.SetBodySleepEnabled( box, false );
            if( i % 2 == 0 ) world_.SetBodyLinearVelocity( box, { 2, 0 } );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::compoundCenterComparison )
    {
        world_.SetGravity( {} );
        const char* labels[] = { "대칭 왼쪽", "대칭 오른쪽", "비대칭 왼쪽", "비대칭 오른쪽" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto composite = world_.CreateBody( bodyType::Dynamic, { { i == 0 ? -3.0f : 3.0f, 0 }, {} } );
            world_.SetBodySleepEnabled( composite, false );
            for( std::size_t part = 0; part < 2; ++part )
            {
                const float density = i == 0 ? 2.0f : part == 0 ? 1.0f : 3.0f;
                const auto shape = world_.CreateShape( composite, circle2{ { part == 0 ? -0.75f : 0.75f, 0 }, 0.5f }, {}, density );
                shapes_.push_back( { composite, shape, labels[2 * i + part] } );
            }
        }
    }
    else if( kind_ == demoKind::fastRotationComparison )
    {
        world_.SetGravity( {} );
        const char* labels[] = { "막대 A (빠른 회전 제한)", "막대 B (빠른 회전 허용)" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto rod = world_.CreateBody( bodyType::Dynamic, { { i == 0 ? -2.5f : 2.5f, 0 }, {} } );
            const auto shape = world_.CreateShape( rod, MakeBox( { 1.0f, 0.125f } ) );
            world_.SetBodySleepEnabled( rod, false );
            world_.SetBodyAngularVelocity( rod, 120.0f );
            world_.SetBodyFastRotationAllowed( rod, i == 1 );
            shapes_.push_back( { rod, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::maximumLinearSpeedComparison )
    {
        world_.SetGravity( {} );
        world_.SetMaximumLinearSpeed( 4.0f );
        const char* labels[] = { "상자 A (초기 2m/s)", "상자 B (초기 8m/s)" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const auto box = world_.CreateBody( bodyType::Dynamic, { { i == 0 ? -2.5f : 2.5f, 0 }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.5f } ) );
            world_.SetBodySleepEnabled( box, false );
            world_.SetBodyLinearVelocity( box, vec2{ 0.6f, 0.8f } * ( i == 0 ? 2.0f : 8.0f ) );
            shapes_.push_back( { box, shape, labels[i] } );
        }
    }
    else if( kind_ == demoKind::frictionMixingComparison )
    {
        world_.SetGravity( { 0, -10 } );
        const float floorFriction[] = { 0.25f, 1.0f, 1.0f };
        const float boxFriction[] = { 1.0f, 0.25f, 0.0f };
        const char* floors[] = { "바닥 A", "바닥 B", "바닥 C" };
        const char* boxes[] = { "상자 A", "상자 B", "상자 C" };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const float lane = 2.0f - 2.0f * static_cast<float>( i );
            const auto floor = world_.CreateBody( bodyType::Static, { { 0, lane }, {} } );
            const auto surface = world_.CreateShape( floor, MakeBox( { 6.0f, 0.15f } ) );
            world_.SetShapeFriction( surface, floorFriction[i] );
            world_.SetShapeRestitution( surface, 0.0f );
            shapes_.push_back( { floor, surface, floors[i] } );
            const auto box = world_.CreateBody( bodyType::Dynamic, { { -3.0f, lane + 0.55f }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.4f, 0.4f } ) );
            world_.SetShapeFriction( shape, boxFriction[i] );
            world_.SetShapeRestitution( shape, 0.0f );
            world_.SetBodySleepEnabled( box, false );
            world_.SetBodyLinearVelocity( box, { 4, 0 } );
            shapes_.push_back( { box, shape, boxes[i] } );
        }
    }
    else if( kind_ == demoKind::restitutionMixingComparison )
    {
        world_.SetGravity( { 0, -10 } );
        const float floorRestitution[] = { 0.0f, 0.8f, 0.0f };
        const float circleRestitution[] = { 0.8f, 0.0f, 0.0f };
        const char* floors[] = { "바닥 A", "바닥 B", "바닥 C" };
        const char* circles[] = { "원 A", "원 B", "원 C" };
        for( std::size_t i = 0; i < 3; ++i )
        {
            const float x = -3.0f + 3.0f * static_cast<float>( i );
            const auto floor = world_.CreateBody( bodyType::Static, { { x, -0.15f }, {} } );
            const auto surface = world_.CreateShape( floor, MakeBox( { 1.25f, 0.15f } ) );
            world_.SetShapeFriction( surface, 0.0f );
            world_.SetShapeRestitution( surface, floorRestitution[i] );
            shapes_.push_back( { floor, surface, floors[i] } );
            const auto circle = world_.CreateBody( bodyType::Dynamic, { { x, 3.0f }, {} } );
            const auto shape = world_.CreateShape( circle, circle2{ {}, 0.4f } );
            world_.SetShapeFriction( shape, 0.0f );
            world_.SetShapeRestitution( shape, circleRestitution[i] );
            world_.SetBodySleepEnabled( circle, false );
            shapes_.push_back( { circle, shape, circles[i] } );
        }
    }
    else if( kind_ == demoKind::lowSpeedRestitutionComparison )
    {
        world_.SetGravity( {} );
        const char* floors[] = { "바닥 A", "바닥 B" };
        const char* circles[] = { "원 A (0.5m/s)", "원 B (2m/s)" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const float x = i == 0 ? -2.0f : 2.0f;
            const auto floor = world_.CreateBody( bodyType::Static, { { x, -0.15f }, {} } );
            const auto surface = world_.CreateShape( floor, MakeBox( { 1.25f, 0.15f } ) );
            world_.SetShapeFriction( surface, 0.0f );
            world_.SetShapeRestitution( surface, 0.0f );
            shapes_.push_back( { floor, surface, floors[i] } );
            const auto circle = world_.CreateBody( bodyType::Dynamic, { { x, 0.4f }, {} } );
            const auto shape = world_.CreateShape( circle, circle2{ {}, 0.4f } );
            world_.SetShapeFriction( shape, 0.0f );
            world_.SetShapeRestitution( shape, 1.0f );
            world_.SetBodySleepEnabled( circle, false );
            world_.SetBodyLinearVelocity( circle, { 0, i == 0 ? -0.5f : -2.0f } );
            shapes_.push_back( { circle, shape, circles[i] } );
        }
    }
    else if( kind_ == demoKind::collisionGroupComparison )
    {
        world_.SetGravity( {} );
        const char* labels[][2] = { { "A 왼쪽", "A 오른쪽" }, { "B 왼쪽", "B 오른쪽" }, { "C 왼쪽", "C 오른쪽" } };
        for( std::size_t i = 0; i < 3; ++i )
            for( std::size_t j = 0; j < 2; ++j )
            {
                const auto body = world_.CreateBody( bodyType::Dynamic, { { j == 0 ? -2.0f : 2.0f, 2.0f - 2.0f * static_cast<float>( i ) }, {} } );
                const auto shape = world_.CreateShape( body, circle2{ {}, 0.35f } );
                world_.SetShapeFriction( shape, 0.0f );
                world_.SetShapeRestitution( shape, 1.0f );
                world_.SetBodySleepEnabled( body, false );
                collisionFilter filter;
                filter.categoryBits = std::uint64_t{ 1 } << j;
                if( i < 2 ) filter.maskBits = 0;
                filter.groupIndex = i == 0 ? 0 : i == 1 ? 1 : -1;
                world_.SetShapeFilter( shape, filter );
                world_.SetBodyLinearVelocity( body, { j == 0 ? 2.0f : -2.0f, 0 } );
                shapes_.push_back( { body, shape, labels[i][j] } );
            }
    }
    else if( kind_ == demoKind::movingPlatformFrictionComparison )
    {
        world_.SetGravity( { 0, -10 } );
        const char* platforms[] = { "발판 A", "발판 B" };
        const char* boxes[] = { "상자 A", "상자 B" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const float lane = i == 0 ? 1.5f : -1.5f;
            const auto platform = world_.CreateBody( bodyType::Kinematic, { { -2.0f, lane }, {} } );
            const auto surface = world_.CreateShape( platform, MakeBox( { 4.0f, 0.15f } ) );
            world_.SetShapeFriction( surface, 0.8f );
            world_.SetShapeRestitution( surface, 0.0f );
            world_.SetBodyLinearVelocity( platform, { 2, 0 } );
            shapes_.push_back( { platform, surface, platforms[i] } );
            const auto box = world_.CreateBody( bodyType::Dynamic, { { -2.0f, lane + 0.55f }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.6f, 0.4f } ) );
            world_.SetShapeFriction( shape, i == 0 ? 0.8f : 0.0f );
            world_.SetShapeRestitution( shape, 0.0f );
            world_.SetBodySleepEnabled( box, false );
            shapes_.push_back( { box, shape, boxes[i] } );
        }
    }
    else if( kind_ == demoKind::rollingSlidingFrictionComparison )
    {
        world_.SetGravity( { 0, -10 } );
        const char* floors[] = { "바닥 A", "바닥 B" };
        const char* circles[] = { "원 A", "원 B" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const float lane = i == 0 ? 1.5f : -1.5f;
            const auto floor = world_.CreateBody( bodyType::Static, { { 0, lane }, {} } );
            const auto surface = world_.CreateShape( floor, MakeBox( { 6.0f, 0.15f } ) );
            world_.SetShapeFriction( surface, 0.8f );
            world_.SetShapeRestitution( surface, 0.0f );
            shapes_.push_back( { floor, surface, floors[i] } );
            const auto circle = world_.CreateBody( bodyType::Dynamic, { { -3.0f, lane + 0.55f }, {} } );
            const auto shape = world_.CreateShape( circle, circle2{ {}, 0.4f } );
            world_.SetShapeFriction( shape, i == 0 ? 0.8f : 0.0f );
            world_.SetShapeRestitution( shape, 0.0f );
            world_.SetBodySleepEnabled( circle, false );
            world_.SetBodyLinearVelocity( circle, { 4, 0 } );
            shapes_.push_back( { circle, shape, circles[i] } );
        }
    }
    else if( kind_ == demoKind::frictionForceComparison )
    {
        world_.SetGravity( { 0, -10 } );
        const char* floors[] = { "바닥 A", "바닥 B" };
        const char* boxes[] = { "상자 A (2N)", "상자 B (8N)" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const float lane = i == 0 ? 1.5f : -1.5f;
            const auto floor = world_.CreateBody( bodyType::Static, { { 0, lane }, {} } );
            const auto surface = world_.CreateShape( floor, MakeBox( { 6.0f, 0.15f } ) );
            world_.SetShapeFriction( surface, 0.5f );
            world_.SetShapeRestitution( surface, 0.0f );
            shapes_.push_back( { floor, surface, floors[i] } );
            const auto box = world_.CreateBody( bodyType::Dynamic, { { -3.0f, lane + 0.65f }, {} } );
            const auto shape = world_.CreateShape( box, MakeBox( { 0.5f, 0.5f } ) );
            world_.SetShapeFriction( shape, 0.5f );
            world_.SetShapeRestitution( shape, 0.0f );
            world_.SetBodySleepEnabled( box, false );
            shapes_.push_back( { box, shape, boxes[i] } );
        }
    }
    else if( kind_ == demoKind::obliqueImpactFrictionComparison )
    {
        world_.SetGravity( {} );
        const char* floors[] = { "바닥 A", "바닥 B" };
        const char* circles[] = { "원 A", "원 B" };
        for( std::size_t i = 0; i < 2; ++i )
        {
            const float lane = i == 0 ? 1.5f : -1.5f;
            const auto floor = world_.CreateBody( bodyType::Static, { { 0, lane }, {} } );
            const auto surface = world_.CreateShape( floor, MakeBox( { 6.0f, 0.15f } ) );
            world_.SetShapeFriction( surface, 0.8f );
            world_.SetShapeRestitution( surface, 1.0f );
            shapes_.push_back( { floor, surface, floors[i] } );
            const auto circle = world_.CreateBody( bodyType::Dynamic, { { -2.0f, lane + 1.75f }, {} } );
            const auto shape = world_.CreateShape( circle, circle2{ {}, 0.4f } );
            world_.SetShapeFriction( shape, i == 0 ? 0.8f : 0.0f );
            world_.SetShapeRestitution( shape, 1.0f );
            world_.SetBodySleepEnabled( circle, false );
            world_.SetBodyLinearVelocity( circle, { 3, -4 } );
            shapes_.push_back( { circle, shape, circles[i] } );
        }
    }
    else if( kind_ == demoKind::collisionMassRatioComparison )
    {
        world_.SetGravity( {} );
        const char* labels[][2] = { { "이동 원 A", "표적 A" }, { "이동 원 B", "표적 B" }, { "이동 원 C", "표적 C" } };
        for( std::size_t i = 0; i < 3; ++i )
            for( std::size_t j = 0; j < 2; ++j )
            {
                const auto body = world_.CreateBody( bodyType::Dynamic, { { j == 0 ? -2.0f : 0.0f, 2.0f - 2.0f * static_cast<float>( i ) }, {} } );
                const auto shape = world_.CreateShape( body, circle2{ {}, 0.35f } );
                world_.SetShapeFriction( shape, 0.0f );
                world_.SetShapeRestitution( shape, 1.0f );
                world_.SetShapeDensity( shape, ( i == 1 && j == 1 ) || ( i == 2 && j == 0 ) ? 4.0f : 1.0f );
                world_.SetBodySleepEnabled( body, false );
                if( j == 0 ) world_.SetBodyLinearVelocity( body, { 4, 0 } );
                shapes_.push_back( { body, shape, labels[i][j] } );
            }
    }
    else if( kind_ == demoKind::centralOffsetImpactComparison )
    {
        world_.SetGravity( {} );
        const char* labels[][2] = { { "원 A", "상자 A" }, { "원 B", "상자 B" }, { "원 C", "상자 C" } };
        for( std::size_t i = 0; i < 3; ++i )
            for( std::size_t j = 0; j < 2; ++j )
            {
                const float offset = j == 0 ? ( i == 1 ? 0.25f : i == 2 ? -0.25f : 0.0f ) : 0.0f;
                const auto body = world_.CreateBody( bodyType::Dynamic, { { j == 0 ? -2.0f : 0.0f, 2.0f - 2.0f * static_cast<float>( i ) + offset }, {} } );
                const auto shape = j == 0 ? world_.CreateShape( body, circle2{ {}, 0.2f } ) : world_.CreateShape( body, MakeBox( { 0.5f, 0.5f } ) );
                world_.SetShapeFriction( shape, 0.0f );
                world_.SetShapeRestitution( shape, 1.0f );
                world_.SetBodySleepEnabled( body, false );
                if( j == 0 ) world_.SetBodyLinearVelocity( body, { 4, 0 } );
                shapes_.push_back( { body, shape, labels[i][j] } );
            }
    }
    for( const auto& visual : shapes_ )
    {
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( visual.bodyHandle ), world_.GetBodyLinearVelocity( visual.bodyHandle ), world_.GetBodyAngularVelocity( visual.bodyHandle ) } );
        if( world_.GetBody( visual.bodyHandle ).type == bodyType::Dynamic && !world_.IsValid( impulseBody_ ) ) impulseBody_ = torqueBody_ = visual.bodyHandle;
    }
}

void rigidBodyDemo::applyMassTrialImpulse()
{
    if( kind_ != demoKind::massImpulseComparison ) return;
    for( const auto& visual : shapes_ )
        if( world_.IsValid( visual.bodyHandle ) ) world_.ApplyLinearImpulseToCenter( visual.bodyHandle, { 0, 2.0f } );
}

std::optional<vec2> rigidBodyDemo::getImpulseTrialPoint( std::size_t shapeIndex ) const
{
    if( kind_ == demoKind::compoundCenterComparison )
    {
        if( shapeIndex >= shapes_.size() || shapeIndex % 2 != 0 ) return std::nullopt;
        const auto body = shapes_[shapeIndex].bodyHandle;
        if( !world_.IsValid( body ) ) return std::nullopt;
        return world_.GetBodyTransform( body ).position;
    }
    if( kind_ != demoKind::impulsePointComparison || shapeIndex >= shapes_.size() ) return std::nullopt;
    const auto body = shapes_[shapeIndex].bodyHandle;
    if( !world_.IsValid( body ) ) return std::nullopt;
    const float offset = shapeIndex == 1 ? -0.5f : shapeIndex == 2 ? 0.5f : 0.0f;
    return TransformPoint( world_.GetBodyTransform( body ), world_.GetBodyLocalCenter( body ) + vec2{ offset, 0 } );
}

void rigidBodyDemo::startTorqueImpulseTrial()
{
    if( kind_ != demoKind::torqueImpulseComparison ) return;
    resetCollisionTrial();
    torqueTrialRunning_ = true;
    if( world_.IsValid( shapes_[1].bodyHandle ) ) world_.ApplyAngularImpulse( shapes_[1].bodyHandle, 1.0f );
}

void rigidBodyDemo::startFrictionForceTrial()
{
    if( kind_ != demoKind::frictionForceComparison ) return;
    resetCollisionTrial();
    forceTrialRunning_ = true;
}

void rigidBodyDemo::startForceImpulseTrial()
{
    if( kind_ != demoKind::forceImpulseComparison ) return;
    resetCollisionTrial();
    forceTrialRunning_ = true;
    if( world_.IsValid( shapes_[1].bodyHandle ) ) world_.ApplyLinearImpulseToCenter( shapes_[1].bodyHandle, { 0, 2.0f } );
}

void rigidBodyDemo::applyInertiaTrialImpulse()
{
    if( kind_ != demoKind::rotationalInertiaComparison ) return;
    for( const auto& visual : shapes_ )
        if( world_.IsValid( visual.bodyHandle ) ) world_.ApplyAngularImpulse( visual.bodyHandle, 1.0f );
}

void rigidBodyDemo::applyPointTrialImpulse()
{
    if( kind_ != demoKind::impulsePointComparison && kind_ != demoKind::compoundCenterComparison ) return;
    for( std::size_t i = 0; i < shapes_.size(); ++i )
        if( const auto point = getImpulseTrialPoint( i ) ) world_.ApplyLinearImpulse( shapes_[i].bodyHandle, { 0, 2.0f }, *point );
}

void rigidBodyDemo::resetCollisionTrial()
{
    if( collisionTrialInitialStates_.empty() ) return;
    clearContactEventTrail();
    rotationTrialTimeStep_ = 0.0f;
    forceTrialTime_ = 0.0;
    forceTrialRunning_ = false;
    torqueTrialTime_ = 0.0;
    torqueTrialRunning_ = false;
    cancelInput();
    for( std::size_t i = 0; i < shapes_.size(); ++i )
    {
        const auto body = shapes_[i].bodyHandle;
        if( !world_.IsValid( body ) ) continue;
        const auto& initial = collisionTrialInitialStates_[i];
        world_.SetBodyTransform( body, initial.transform );
        world_.SetBodyLinearVelocity( body, initial.velocity );
        world_.SetBodyAngularVelocity( body, initial.angularVelocity );
        world_.ClearForces( body );
    }
    // 시간 진행 없이 초기 위치의 sensor overlap을 다시 계산함.
    if( kind_ == demoKind::sensorWallComparison ) world_.Step( 0.0f );
    refreshContacts();
}

void rigidBodyDemo::setWindSourceStrength( float strength )
{
    if( std::isfinite( strength ) ) windSourceStrength_ = std::clamp( strength, 0.0f, 25.0f );
}

void rigidBodyDemo::setWindDragCoefficient( float rate )
{
    if( std::isfinite( rate ) ) windDragCoefficient_ = std::clamp( rate, 0.0f, 15.0f );
}

void rigidBodyDemo::addWindVortex()
{
    if( kind_ != demoKind::windFlow || !windField_ ) return;
    // 같은 격자에 8방향 접선 속도를 넣어 두 이류 방식의 확산/운동량을 비교.
    constexpr int samples = 8;
    constexpr float radius = 1.5f;
    constexpr float speed = 3.0f;
    constexpr float tau = 2.0f * std::numbers::pi_v<float>;
    for( int i = 0; i < samples; ++i )
    {
        const float angle = tau * i / samples;
        const vec2 radial{ std::cos( angle ), std::sin( angle ) };
        const vec2 tangent{ -radial.y, radial.x };
        windField_->addVelocity( radius * radial, speed * tangent, 0.65f );
    }
}

void rigidBodyDemo::seedWindTracer()
{
    if( kind_ != demoKind::windFlow || !windField_ ) return;
    // 닫힌 도메인 왼쪽에 tracer patch를 심어 흐름과 보존량을 관찰.
    windField_->setTracerConcentration( 6, 6, 4.0f );
    windField_->setTracerConcentration( 7, 6, 4.0f );
}

void rigidBodyDemo::beginWindAdvectionTrial()
{
    if( kind_ != demoKind::windFlow || !windField_ ) return;
    resetWindFlow();
    // 유체 이류를 분리 평가하기 위해 외부 source/Drag/강체 연성을 끈다.
    windSourceEnabled_ = false;
    windObstacleCouplingEnabled_ = false;
    windDragCoefficient_ = 0.0f;
    addWindVortex();
}

void rigidBodyDemo::resetWindFlow()
{
    if( kind_ != demoKind::windFlow || !windField_ ) return;
    windField_->clear();
    windPressureForces_.fill( {} );
    windDragBodyImpulse_ = {};
    windDragFluidImpulse_ = {};
    windTime_ = 0.0f;
    cancelInput();
    constexpr std::array<vec2, 3> POSITIONS{ vec2{ -1.8f, 0.0f }, vec2{ 0.4f, 1.0f }, vec2{ 2.5f, -1.0f } };
    for( std::size_t i = 0; i < shapes_.size() && i < POSITIONS.size(); ++i )
    {
        const bodyId body = shapes_[i].bodyHandle;
        if( !world_.IsValid( body ) ) continue;
        world_.SetBodyTransform( body, { POSITIONS[i], {} } );
        world_.SetBodyLinearVelocity( body, {} );
        world_.SetBodyAngularVelocity( body, 0.0f );
        world_.ClearForces( body );
    }
    clearContactEventTrail();
    refreshContacts();
}

void rigidBodyDemo::createWindFlow()
{
    // 유체와 World는 소유권과 Solver가 독립적. 원, 캡슐, 볼록 다각형을 혼합해 비교.
    windField_.emplace( 24, 14, 0.5f, vec2{ -6.0f, -3.5f } );
    world_.SetGravity( {} );

    constexpr std::array<vec2, 3> POSITIONS{ vec2{ -1.8f, 0.0f }, vec2{ 0.4f, 1.0f }, vec2{ 2.5f, -1.0f } };
    const std::array<shapeGeometry, 3> GEOMETRIES{
        circle2{ {}, 0.32f },
        capsule2{ { -0.45f, 0.0f }, { 0.45f, 0.0f }, 0.25f },
        MakeBox( { 0.45f, 0.35f } )
    };
    constexpr std::array<const char*, 3> LABELS{ "원", "캡슐", "다각형 상자" };

    for( std::size_t i = 0; i < POSITIONS.size(); ++i )
    {
        const bodyId body = world_.CreateBody( bodyType::Dynamic, { POSITIONS[i], {} } );
        world_.SetBodySleepEnabled( body, false );
        const shapeId shape = world_.CreateShape( body, GEOMETRIES[i], {}, i == 1 ? 3.0f : 1.0f );
        shapes_.push_back( { body, shape, LABELS[i] } );
    }
    impulseBody_ = shapes_[0].bodyHandle;
}

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
    for( std::size_t i = 0; i < shapes_.size(); ++i ) playgroundInitialTransforms_[i] = world_.GetBodyTransform( shapes_[i].bodyHandle );
}

void rigidBodyDemo::resetPlayground()
{
    if( kind_ != demoKind::playground ) return;
    cancelInput();
    for( std::size_t i = 0; i < shapes_.size(); ++i )
    {
        const auto body = shapes_[i].bodyHandle;
        if( !world_.IsValid( body ) ) continue;
        world_.SetBodyTransform( body, playgroundInitialTransforms_[i] );
        world_.SetBodyLinearVelocity( body, i == 5 ? vec2{ 1.25f, 0.0f } : vec2{} );
        world_.SetBodyAngularVelocity( body, 0.0f );
        world_.ClearForces( body );
    }
    refreshContacts();
}

void rigidBodyDemo::createTerrainChain()
{
    // Chain의 front side가 위쪽을 향하도록 오른쪽에서 왼쪽 순서로 terrain을 구성함.
    // 양 끝 ghost는 실제 충돌 edge 바깥에서 endpoint normal만 이어줌.
    const auto& controlPoints = TERRAIN_CONTROL_POINTS;

    splineControlPoints_.assign( controlPoints.begin(), controlPoints.end() );
    const std::vector<vec2> points = kind_ == demoKind::splineTerrain ? SampleAdaptive( { controlPoints }, splineSamplingError_ ) : std::vector<vec2>{ controlPoints.begin(), controlPoints.end() };

    if( points.size() < 2 ) return;

    for( std::size_t i = 0; i < points.size(); ++i )
    {
        if( !IsFinite( points[i] ) ) return;

        if( i > 0 && LengthSquared( points[i] - points[i - 1] ) <= LINEAR_SLOP * LINEAR_SLOP ) return;
    }

    chainDef definition{};
    definition.points = points;
    definition.ghost1 = points.front() + ( points.front() - points[1] );
    definition.ghost2 = points.back() + ( points.back() - points[points.size() - 2] );
    definition.friction = 0.8f;

    if( !IsFinite( definition.ghost1 ) || !IsFinite( definition.ghost2 ) ) return;
    if( LengthSquared( definition.ghost1 - points.front() ) <= LINEAR_SLOP * LINEAR_SLOP || LengthSquared( definition.ghost2 - points.back() ) <= LINEAR_SLOP * LINEAR_SLOP ) return;

    const bodyId ground = world_.CreateBody( bodyType::Static );

    terrainChain_ = world_.CreateChain( ground, definition );

    terrainSegments_.resize( points.size() - 1 );
    const std::size_t segmentCount = world_.GetChainSegments( terrainChain_, terrainSegments_ );
    assert( segmentCount == terrainSegments_.size() );

    for( shapeId segment : terrainSegments_ )
    {
        shapes_.push_back( { ground, segment, "지형 edge [Chain]" } );
    }

    // 서로 다른 geometry가 내부 vertex를 지나가며 one-sided / ghost 처리를 함께 검증함.
    impulseBody_ = world_.CreateBody( bodyType::Dynamic, { { 5.0f, 2.2f }, {} } );
    const shapeId circleShape = world_.CreateShape( impulseBody_, circle2{ {}, 0.45f } );
    shapes_.push_back( { impulseBody_, circleShape, "원 [Chain Probe]" } );

    const bodyId capsuleBody = world_.CreateBody( bodyType::Dynamic, { { 1.0f, 3.2f }, rot2::FromRadians( -0.2f ) } );
    const shapeId capsuleShape = world_.CreateShape( capsuleBody, capsule2{ { -0.65f, 0.0f }, { 0.65f, 0.0f }, 0.3f } );
    shapes_.push_back( { capsuleBody, capsuleShape, "캡슐 [Chain Probe]" } );

    torqueBody_ = world_.CreateBody( bodyType::Dynamic, { { -4.0f, 2.4f }, rot2::FromRadians( 0.12f ) } );
    const shapeId boxShape = world_.CreateShape( torqueBody_, MakeBox( { 0.55f, 0.45f } ) );
    shapes_.push_back( { torqueBody_, boxShape, "상자 [Chain Probe]" } );
    for( std::size_t i = 0; i < terrainInitialTransforms_.size(); ++i ) terrainInitialTransforms_[i] = world_.GetBodyTransform( shapes_[terrainSegments_.size() + i].bodyHandle );
}

void rigidBodyDemo::resetTerrainProbes()
{
    if( kind_ != demoKind::terrainChain && kind_ != demoKind::splineTerrain ) return;
    terrainTrialObservation_ = {};
    cancelInput();
    for( std::size_t i = 0; i < terrainInitialTransforms_.size(); ++i )
    {
        const auto body = shapes_[terrainSegments_.size() + i].bodyHandle;
        if( !world_.IsValid( body ) ) continue;
        world_.SetBodyTransform( body, terrainInitialTransforms_[i] );
        world_.SetBodyLinearVelocity( body, {} );
        world_.SetBodyAngularVelocity( body, 0.0f );
        world_.ClearForces( body );
    }
    refreshContacts();
}

terrainTrialComparison rigidBodyDemo::getTerrainTrialComparison() const noexcept
{
    if( !terrainTrialReference_ ) return {};
    const auto& current = terrainTrialObservation_;
    if( !current.active ) return { terrainTrialComparisonStatus::noObservation };
    const auto& reference = *terrainTrialReference_;
    if( current.probeIndex != reference.probeIndex || current.trial != reference.trial || current.launchFraction != reference.launchFraction || current.launchSpeed != reference.launchSpeed || current.launchGravity.x != reference.launchGravity.x || current.launchGravity.y != reference.launchGravity.y ) return { terrainTrialComparisonStatus::differentConditions };
    if( current.launchGravityScale != reference.launchGravityScale ) return { terrainTrialComparisonStatus::differentConditions };
    if( current.launchFriction != reference.launchFriction || current.launchRestitution != reference.launchRestitution || current.launchMass != reference.launchMass || current.launchInertia != reference.launchInertia ) return { terrainTrialComparisonStatus::differentConditions };
    if( !current.firstContactTime || !reference.firstContactTime ) return { terrainTrialComparisonStatus::waitingForContact };
    const bool settingsMatch = current.firstContactTimeStep == reference.firstContactTimeStep && current.firstContactSubStepCount == reference.firstContactSubStepCount;
    const auto stepSettings = current.preContactSettingsChanged || reference.preContactSettingsChanged ? terrainTrialStepComparison::changed : settingsMatch ? terrainTrialStepComparison::constantMatch : terrainTrialStepComparison::constantDifferent;
    return { terrainTrialComparisonStatus::ready, *current.firstContactTime - *reference.firstContactTime, settingsMatch, stepSettings, current.preContactGravityChanged || reference.preContactGravityChanged, current.preContactMaterialChanged || reference.preContactMaterialChanged, current.preContactMassChanged || reference.preContactMassChanged, current.preContactInertiaChanged || reference.preContactInertiaChanged, current.preContactGravityScaleChanged || reference.preContactGravityScaleChanged };
}

bool rigidBodyDemo::relaunchTerrainTrialReference()
{
    if( !terrainTrialReference_ || !getTerrainTrialTarget( terrainTrialReference_->trial ) ) return false;
    const auto& reference = *terrainTrialReference_;
    const std::size_t probeIndex = terrainSegments_.size() + reference.probeIndex;
    if( probeIndex >= shapes_.size() || !world_.IsValid( shapes_[probeIndex].shapeHandle ) || !world_.IsValid( shapes_[probeIndex].bodyHandle ) ) return false;
    if( reference.launchFraction )
    {
        setSplineTrialFraction( *reference.launchFraction );
        setSplineTrialSpeed( reference.launchSpeed );
    }
    launchTerrainProbe( reference.probeIndex, reference.trial );
    return true;
}

bool rigidBodyDemo::pinTerrainTrialReference()
{
    if( !terrainTrialObservation_.active || terrainTrialObservation_.elapsed <= 0.0f || !world_.IsValid( terrainChain_ ) ) return false;
    terrainTrialReference_ = terrainTrialObservation_;
    return true;
}

void rigidBodyDemo::updateTerrainTrialObservation( float timeStep, int subStepCount )
{
    if( !terrainTrialObservation_.active && !terrainTrialReference_ ) return;
    if( !world_.IsValid( terrainChain_ ) )
    {
        terrainTrialObservation_ = {};
        terrainTrialReference_.reset();
        return;
    }
    if( !terrainTrialObservation_.active ) return;
    const std::size_t probeIndex = terrainSegments_.size() + terrainTrialObservation_.probeIndex;
    if( probeIndex >= shapes_.size() || !world_.IsValid( shapes_[probeIndex].shapeHandle ) )
    {
        terrainTrialObservation_ = {};
        return;
    }
    if( timeStep <= 0.0f ) return;
    if( !terrainTrialObservation_.firstContactTime )
    {
        const vec2 gravity = world_.GetGravity();
        if( gravity.x != terrainTrialObservation_.launchGravity.x || gravity.y != terrainTrialObservation_.launchGravity.y ) terrainTrialObservation_.preContactGravityChanged = true;
        if( world_.GetBodyGravityScale( shapes_[probeIndex].bodyHandle ) != terrainTrialObservation_.launchGravityScale ) terrainTrialObservation_.preContactGravityScaleChanged = true;
        if( world_.GetShapeFriction( shapes_[probeIndex].shapeHandle ) != terrainTrialObservation_.launchFriction || world_.GetShapeRestitution( shapes_[probeIndex].shapeHandle ) != terrainTrialObservation_.launchRestitution ) terrainTrialObservation_.preContactMaterialChanged = true;
        if( world_.GetBodyMass( shapes_[probeIndex].bodyHandle ) != terrainTrialObservation_.launchMass ) terrainTrialObservation_.preContactMassChanged = true;
        if( world_.GetBodyRotationalInertia( shapes_[probeIndex].bodyHandle ) != terrainTrialObservation_.launchInertia ) terrainTrialObservation_.preContactInertiaChanged = true;
        if( terrainTrialObservation_.initialSubStepCount == 0 )
        {
            terrainTrialObservation_.initialTimeStep = timeStep;
            terrainTrialObservation_.initialSubStepCount = subStepCount;
        }
        else if( timeStep != terrainTrialObservation_.initialTimeStep || subStepCount != terrainTrialObservation_.initialSubStepCount ) terrainTrialObservation_.preContactSettingsChanged = true;
    }
    terrainTrialObservation_.elapsed += timeStep;
    terrainTrialObservation_.touching = false;
    const shapeId probe = shapes_[probeIndex].shapeHandle;
    for( const auto& contact : contacts_ )
    {
        if( contact.manifold.pointCount == 0 || ( contact.shapeA != probe && contact.shapeB != probe ) ) continue;
        const shapeId other = contact.shapeA == probe ? contact.shapeB : contact.shapeA;
        if( world_.GetShapeParentChain( other ) != terrainChain_ ) continue;
        terrainTrialObservation_.touching = true;
        break;
    }
    if( terrainTrialObservation_.touching && !terrainTrialObservation_.firstContactTime )
    {
        terrainTrialObservation_.firstContactTime = terrainTrialObservation_.elapsed;
        terrainTrialObservation_.firstContactTimeStep = timeStep;
        terrainTrialObservation_.firstContactSubStepCount = subStepCount;
    }
}

void rigidBodyDemo::updateSplineTrialPosition( vec2 point )
{
    if( !world_.IsValid( terrainChain_ ) || terrainSegments_.empty() ) return;
    point = InverseTransformPoint( world_.GetBodyTransform( shapes_[0].bodyHandle ), point );
    if( !IsFinite( point ) ) return;
    double totalLength = 0.0, selectedLength = 0.0;
    double bestDistance = std::numeric_limits<double>::infinity();
    for( std::size_t i = 0; i < terrainSegments_.size(); ++i )
    {
        const segment2 edge{ getTerrainPoint( i ), getTerrainPoint( i + 1 ) };
        const vec2 closest = ClosestPoint( edge, point );
        const double dx = static_cast<double>( closest.x ) - point.x, dy = static_cast<double>( closest.y ) - point.y;
        const double distance = dx * dx + dy * dy;
        if( distance < bestDistance )
        {
            bestDistance = distance;
            selectedLength = totalLength + Length( closest - edge.a );
        }
        totalLength += Length( edge );
    }
    if( std::isfinite( bestDistance ) ) setSplineTrialFraction( std::clamp( static_cast<float>( selectedLength / totalLength ), 0.0f, 1.0f ) );
}

void rigidBodyDemo::setSplineTrialFraction( float fraction )
{
    if( kind_ != demoKind::splineTerrain || !std::isfinite( fraction ) || fraction < 0.0f || fraction > 1.0f ) return;
    splineTrialFraction_ = fraction;
}

void rigidBodyDemo::setSplineTrialSpeed( float speed )
{
    if( kind_ != demoKind::splineTerrain || !std::isfinite( speed ) || speed < 1.0f || speed > 20.0f ) return;
    splineTrialSpeed_ = speed;
}

std::optional<terrainTrialTarget> rigidBodyDemo::getTerrainTrialTarget( terrainTrial trial ) const
{
    if( ( kind_ != demoKind::terrainChain && kind_ != demoKind::splineTerrain ) || !world_.IsValid( terrainChain_ ) || terrainSegments_.empty() || shapes_.empty() || !world_.IsValid( shapes_[0].bodyHandle ) ) return {};
    std::size_t edge = trial == terrainTrial::seams ? 0 : std::min( std::size_t{ 3 }, terrainSegments_.size() - 1 );
    float t = 0.5f;
    if( kind_ == demoKind::splineTerrain && trial != terrainTrial::seams )
    {
        double distance = 0.0;
        for( std::size_t i = 0; i < terrainSegments_.size(); ++i ) distance += Length( getTerrainPoint( i + 1 ) - getTerrainPoint( i ) );
        distance *= splineTrialFraction_;
        for( edge = 0; edge < terrainSegments_.size(); ++edge )
        {
            const float length = Length( getTerrainPoint( edge + 1 ) - getTerrainPoint( edge ) );
            if( distance <= length || edge + 1 == terrainSegments_.size() )
            {
                t = std::clamp( static_cast<float>( distance / length ), 0.0f, 1.0f );
                break;
            }
            distance -= length;
        }
    }
    const auto ground = world_.GetBodyTransform( shapes_[edge].bodyHandle );
    const vec2 a = TransformPoint( ground, getTerrainPoint( edge ) );
    const vec2 b = TransformPoint( ground, getTerrainPoint( edge + 1 ) );
    const vec2 tangent = Normalize( b - a );
    return terrainTrialTarget{ a + t * ( b - a ), tangent };
}

void rigidBodyDemo::launchTerrainProbe( std::size_t probeIndex, terrainTrial trial )
{
    if( probeIndex >= terrainInitialTransforms_.size() ) return;
    const auto target = getTerrainTrialTarget( trial );
    if( !target ) return;
    const auto body = shapes_[terrainSegments_.size() + probeIndex].bodyHandle;
    const auto shape = shapes_[terrainSegments_.size() + probeIndex].shapeHandle;
    if( !world_.IsValid( body ) || !world_.IsValid( shape ) ) return;
    resetTerrainProbes();
    const vec2 tangent = target->tangent;
    const vec2 normal = Cross( tangent, 1.0f );
    const float side = trial == terrainTrial::back ? -1.0f : 1.0f;
    world_.SetBodyTransform( body, { target->point + side * 1.5f * normal, rot2::FromRadians( std::atan2( tangent.y, tangent.x ) ) } );
    const float speed = trial == terrainTrial::seams ? 6.0f : kind_ == demoKind::splineTerrain ? splineTrialSpeed_ : 8.0f;
    world_.SetBodyLinearVelocity( body, trial == terrainTrial::seams ? speed * tangent : -side * speed * normal );
    refreshContacts();
    if( kind_ == demoKind::splineTerrain )
    {
        terrainTrialObservation_.active = true;
        terrainTrialObservation_.probeIndex = probeIndex;
        terrainTrialObservation_.trial = trial;
        terrainTrialObservation_.launchSpeed = speed;
        terrainTrialObservation_.launchGravity = world_.GetGravity();
        terrainTrialObservation_.launchGravityScale = world_.GetBodyGravityScale( body );
        terrainTrialObservation_.launchFriction = world_.GetShapeFriction( shape );
        terrainTrialObservation_.launchRestitution = world_.GetShapeRestitution( shape );
        terrainTrialObservation_.launchMass = world_.GetBodyMass( body );
        terrainTrialObservation_.launchInertia = world_.GetBodyRotationalInertia( body );
        if( trial != terrainTrial::seams ) terrainTrialObservation_.launchFraction = splineTrialFraction_;
    }
}

void rigidBodyDemo::createWeldElasticityComparison()
{
    world_.SetGravity( { 0.0f, -9.8f } );
    const auto anchor = world_.CreateBody( bodyType::Static );
    const char* labels[] = { "물체 A", "물체 B", "물체 C" };
    for( std::size_t i = 0; i < 3; ++i )
    {
        const float x = -3.0f + 3.0f * static_cast<float>( i );
        const auto body = world_.CreateBody( bodyType::Dynamic, { { x, 1.0f }, {} } );
        const auto shape = world_.CreateShape( body, MakeBox( { 0.5f, 0.1f } ) );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetShapeRestitution( shape, 0.0f );
        world_.SetBodySleepEnabled( body, false );
        shapes_.push_back( { body, shape, labels[i] } );
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( body ), {}, 0.0f } );
        weldJointDef joint{};
        joint.bodyA = anchor;
        joint.bodyB = body;
        joint.localAnchorA = { x - 0.5f, 1.0f };
        joint.localAnchorB = { -0.5f, 0.0f };
        joint.linearHertz = i == 1 ? 1.0f : 0.0f;
        joint.angularHertz = i == 2 ? 1.0f : 0.0f;
        joint.linearDampingRatio = 0.7f;
        joint.angularDampingRatio = 0.7f;
        weldComparisonJoints_[i] = world_.createWeldJoint( joint );
    }
    impulseBody_ = torqueBody_ = shapes_[0].bodyHandle;
}

void rigidBodyDemo::createSuspensionMotorComparison()
{
    world_.SetGravity( { 0.0f, -9.8f } );
    const auto anchor = world_.CreateBody( bodyType::Static );
    const char* labels[] = { "물체 A", "물체 B" };
    for( std::size_t i = 0; i < 2; ++i )
    {
        const float x = -2.0f + 4.0f * static_cast<float>( i );
        const auto body = world_.CreateBody( bodyType::Dynamic, { { x, 1.0f }, {} } );
        const auto shape = world_.CreateShape( body, circle2{ {}, 0.2f } );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetShapeRestitution( shape, 0.0f );
        world_.SetBodySleepEnabled( body, false );
        shapes_.push_back( { body, shape, labels[i] } );
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( body ), {}, 0.0f } );
        wheelJointDef joint{};
        joint.bodyA = anchor;
        joint.bodyB = body;
        joint.localAnchorA = { x, 1.0f };
        joint.localAxisA = { 0.0f, 1.0f };
        joint.enableSpring = true;
        joint.hertz = 1.0f;
        joint.enableMotor = i == 1;
        joint.motorSpeed = 4.0f;
        joint.maxMotorTorque = 0.05f;
        joint.dampingRatio = 0.7f;
        suspensionMotorJoints_[i] = world_.createWheelJoint( joint );
    }
    impulseBody_ = torqueBody_ = shapes_[0].bodyHandle;
}

void rigidBodyDemo::createSuspensionLimitComparison()
{
    world_.SetGravity( { 0.0f, -9.8f } );
    const auto anchor = world_.CreateBody( bodyType::Static );
    const char* labels[] = { "물체 A", "물체 B" };
    for( std::size_t i = 0; i < 2; ++i )
    {
        const float x = -2.0f + 4.0f * static_cast<float>( i );
        const auto body = world_.CreateBody( bodyType::Dynamic, { { x, 1.0f }, {} } );
        const auto shape = world_.CreateShape( body, circle2{ {}, 0.2f } );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetShapeRestitution( shape, 0.0f );
        world_.SetBodySleepEnabled( body, false );
        shapes_.push_back( { body, shape, labels[i] } );
        world_.SetBodyAngularVelocity( body, 2.0f );
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( body ), {}, 2.0f } );
        wheelJointDef joint{};
        joint.bodyA = anchor;
        joint.bodyB = body;
        joint.localAnchorA = { x, 1.0f };
        joint.localAxisA = { 0.0f, 1.0f };
        joint.enableSpring = true;
        joint.hertz = 0.5f;
        joint.enableLimit = i == 1;
        joint.lowerTranslation = -0.4f;
        joint.upperTranslation = 0.4f;
        joint.dampingRatio = 0.7f;
        suspensionLimitJoints_[i] = world_.createWheelJoint( joint );
    }
    impulseBody_ = torqueBody_ = shapes_[0].bodyHandle;
}

void rigidBodyDemo::createSuspensionLoadComparison()
{
    world_.SetGravity( { 0.0f, -9.8f } );
    const auto anchor = world_.CreateBody( bodyType::Static );
    const char* labels[] = { "물체 A", "물체 B", "물체 C" };
    const float frequency[] = { 0.5f, 1.0f, 2.0f };
    for( std::size_t i = 0; i < 3; ++i )
    {
        const float x = -3.0f + 3.0f * static_cast<float>( i );
        const auto body = world_.CreateBody( bodyType::Dynamic, { { x, 1.0f }, {} } );
        const auto shape = world_.CreateShape( body, circle2{ {}, 0.2f } );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetShapeRestitution( shape, 0.0f );
        world_.SetBodySleepEnabled( body, false );
        shapes_.push_back( { body, shape, labels[i] } );
        world_.SetBodyAngularVelocity( body, 2.0f );
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( body ), {}, 2.0f } );
        wheelJointDef joint{};
        joint.bodyA = anchor;
        joint.bodyB = body;
        joint.localAnchorA = { x, 1.0f };
        joint.localAxisA = { 0.0f, 1.0f };
        joint.enableSpring = true;
        joint.hertz = frequency[i];
        joint.dampingRatio = 0.7f;
        suspensionComparisonJoints_[i] = world_.createWheelJoint( joint );
    }
    impulseBody_ = torqueBody_ = shapes_[0].bodyHandle;
}

void rigidBodyDemo::createRevoluteLimitMotorComparison()
{
    world_.SetGravity( {} );
    const auto anchor = world_.CreateBody( bodyType::Static );
    const char* labels[] = { "물체 A", "물체 B" };
    for( std::size_t i = 0; i < 2; ++i )
    {
        const float x = -2.0f + 4.0f * static_cast<float>( i );
        const auto body = world_.CreateBody( bodyType::Dynamic, { { x, 0.0f }, {} } );
        const auto shape = world_.CreateShape( body, MakeBox( { 0.5f, 0.1f } ) );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetShapeRestitution( shape, 0.0f );
        world_.SetBodySleepEnabled( body, false );
        shapes_.push_back( { body, shape, labels[i] } );
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( body ), {}, 0.0f } );
        revoluteJointDef joint{};
        joint.bodyA = anchor;
        joint.bodyB = body;
        joint.localAnchorA = { x, 0.0f };
        joint.enableMotor = true;
        joint.motorSpeed = 2.0f;
        joint.maxMotorTorque = 0.2f;
        joint.enableLimit = i == 1;
        joint.lowerAngle = -0.25f * std::numbers::pi_v<float>;
        joint.upperAngle = 0.25f * std::numbers::pi_v<float>;
        angleLimitComparisonJoints_[i] = world_.createRevoluteJoint( joint );
    }
    impulseBody_ = torqueBody_ = shapes_[0].bodyHandle;
}

void rigidBodyDemo::createRevoluteMotorTorqueComparison()
{
    world_.SetGravity( {} );
    const auto anchor = world_.CreateBody( bodyType::Static );
    const char* labels[] = { "물체 A", "물체 B", "물체 C" };
    const float maxTorque[] = { 0.0f, 0.01f, 0.2f };
    for( std::size_t i = 0; i < 3; ++i )
    {
        const float x = -2.0f + 2.0f * static_cast<float>( i );
        const auto body = world_.CreateBody( bodyType::Dynamic, { { x, 0.0f }, {} } );
        const auto shape = world_.CreateShape( body, MakeBox( { 0.5f, 0.1f } ) );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetShapeRestitution( shape, 0.0f );
        world_.SetBodySleepEnabled( body, false );
        shapes_.push_back( { body, shape, labels[i] } );
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( body ), {}, 0.0f } );
        revoluteJointDef joint{};
        joint.bodyA = anchor;
        joint.bodyB = body;
        joint.localAnchorA = { x, 0.0f };
        joint.enableMotor = true;
        joint.motorSpeed = 2.0f;
        joint.maxMotorTorque = maxTorque[i];
        torqueComparisonJoints_[i] = world_.createRevoluteJoint( joint );
    }
    impulseBody_ = torqueBody_ = shapes_[0].bodyHandle;
}

void rigidBodyDemo::createDistanceLimitMotorComparison()
{
    world_.SetGravity( {} );
    const auto anchor = world_.CreateBody( bodyType::Static );
    const char* labels[] = { "물체 A", "물체 B" };
    for( std::size_t i = 0; i < 2; ++i )
    {
        const float x = -2.0f + 4.0f * static_cast<float>( i );
        const auto body = world_.CreateBody( bodyType::Dynamic, { { x, 0.5f }, {} } );
        const auto shape = world_.CreateShape( body, circle2{ {}, 0.2f } );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetShapeRestitution( shape, 0.0f );
        world_.SetBodySleepEnabled( body, false );
        shapes_.push_back( { body, shape, labels[i] } );
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( body ), {}, 0.0f } );
        distanceJointDef joint{};
        joint.bodyA = anchor;
        joint.bodyB = body;
        joint.localAnchorA = { x, 1.5f };
        joint.length = 1.0f;
        joint.enableSpring = true;
        joint.hertz = 0.0f;
        joint.dampingRatio = 0.0f;
        joint.enableMotor = true;
        joint.motorSpeed = 1.0f;
        joint.maxMotorForce = 2.0f;
        joint.enableLimit = i == 1;
        joint.minLength = 0.75f;
        joint.maxLength = 1.5f;
        limitComparisonJoints_[i] = world_.createDistanceJoint( joint );
    }
    impulseBody_ = torqueBody_ = shapes_[0].bodyHandle;
}

void rigidBodyDemo::createDistanceMotorForceComparison()
{
    world_.SetGravity( {} );
    const auto anchor = world_.CreateBody( bodyType::Static );
    const char* labels[] = { "물체 A", "물체 B", "물체 C" };
    const float maxForce[] = { 0.0f, 0.05f, 2.0f };
    for( std::size_t i = 0; i < 3; ++i )
    {
        const float x = -3.0f + 3.0f * static_cast<float>( i );
        const auto body = world_.CreateBody( bodyType::Dynamic, { { x, 0.5f }, {} } );
        const auto shape = world_.CreateShape( body, circle2{ {}, 0.2f } );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetShapeRestitution( shape, 0.0f );
        world_.SetBodySleepEnabled( body, false );
        shapes_.push_back( { body, shape, labels[i] } );
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( body ), {}, 0.0f } );
        distanceJointDef joint{};
        joint.bodyA = anchor;
        joint.bodyB = body;
        joint.localAnchorA = { x, 1.5f };
        joint.length = 1.0f;
        joint.enableSpring = true;
        joint.hertz = 0.0f;
        joint.dampingRatio = 0.0f;
        joint.enableMotor = true;
        joint.motorSpeed = 1.0f;
        joint.maxMotorForce = maxForce[i];
        motorComparisonJoints_[i] = world_.createDistanceJoint( joint );
    }
    impulseBody_ = torqueBody_ = shapes_[0].bodyHandle;
}

void rigidBodyDemo::createSpringDampingComparison()
{
    world_.SetGravity( {} );
    const auto anchor = world_.CreateBody( bodyType::Static );
    const char* labels[] = { "물체 A", "물체 B", "물체 C" };
    const float damping[] = { 0.0f, 0.7f, 2.0f };
    for( std::size_t i = 0; i < 3; ++i )
    {
        const float x = -3.0f + 3.0f * static_cast<float>( i );
        const auto body = world_.CreateBody( bodyType::Dynamic, { { x, 0 }, {} } );
        const auto shape = world_.CreateShape( body, circle2{ {}, 0.2f } );
        world_.SetShapeFriction( shape, 0.0f );
        world_.SetShapeRestitution( shape, 0.0f );
        world_.SetBodySleepEnabled( body, false );
        shapes_.push_back( { body, shape, labels[i] } );
        collisionTrialInitialStates_.push_back( { world_.GetBodyTransform( body ), {}, 0.0f } );
        distanceJointDef joint{};
        joint.bodyA = anchor;
        joint.bodyB = body;
        joint.localAnchorA = { x, 1.5f };
        joint.length = 1.0f;
        joint.enableSpring = true;
        joint.hertz = 2.0f;
        joint.dampingRatio = damping[i];
        springComparisonJoints_[i] = world_.createDistanceJoint( joint );
    }
    impulseBody_ = torqueBody_ = shapes_[0].bodyHandle;
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

void rigidBodyDemo::createRagdoll()
{
    const bodyId ground = world_.CreateBody( bodyType::Static, { { 0.0f, -0.5f }, {} } );
    const shapeId groundShape = world_.CreateShape( ground, MakeBox( { 7.0f, 0.5f } ) );
    world_.SetShapeFriction( groundShape, 0.9f );
    shapes_.push_back( { ground, groundShape, "렉돌 바닥 [정적]" } );

    // 순서: 머리, 몸통, 골반, 왼팔 위/아래, 오른팔 위/아래, 왼다리 위/아래, 오른다리 위/아래.
    ragdollBodies_[0] = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 6.8f }, {} } );
    shapeId shape = world_.CreateShape( ragdollBodies_[0], circle2{ {}, 0.35f } );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[0], shape, "머리 [렉돌]" } );

    ragdollBodies_[1] = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 5.7f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[1], MakeBox( { 0.55f, 0.75f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[1], shape, "몸통 [렉돌]" } );

    ragdollBodies_[2] = world_.CreateBody( bodyType::Dynamic, { { 0.0f, 4.6f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[2], MakeBox( { 0.5f, 0.35f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[2], shape, "골반 [렉돌]" } );

    ragdollBodies_[3] = world_.CreateBody( bodyType::Dynamic, { { -1.1f, 5.95f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[3], MakeBox( { 0.55f, 0.16f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[3], shape, "왼쪽 위팔 [렉돌]" } );

    ragdollBodies_[4] = world_.CreateBody( bodyType::Dynamic, { { -2.2f, 5.95f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[4], MakeBox( { 0.55f, 0.14f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[4], shape, "왼쪽 아래팔 [렉돌]" } );

    ragdollBodies_[5] = world_.CreateBody( bodyType::Dynamic, { { 1.1f, 5.95f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[5], MakeBox( { 0.55f, 0.16f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[5], shape, "오른쪽 위팔 [렉돌]" } );

    ragdollBodies_[6] = world_.CreateBody( bodyType::Dynamic, { { 2.2f, 5.95f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[6], MakeBox( { 0.55f, 0.14f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[6], shape, "오른쪽 아래팔 [렉돌]" } );

    ragdollBodies_[7] = world_.CreateBody( bodyType::Dynamic, { { -0.28f, 3.5f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[7], MakeBox( { 0.18f, 0.75f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[7], shape, "왼쪽 허벅지 [렉돌]" } );

    ragdollBodies_[8] = world_.CreateBody( bodyType::Dynamic, { { -0.28f, 2.0f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[8], MakeBox( { 0.16f, 0.75f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[8], shape, "왼쪽 종아리 [렉돌]" } );

    ragdollBodies_[9] = world_.CreateBody( bodyType::Dynamic, { { 0.28f, 3.5f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[9], MakeBox( { 0.18f, 0.75f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[9], shape, "오른쪽 허벅지 [렉돌]" } );

    ragdollBodies_[10] = world_.CreateBody( bodyType::Dynamic, { { 0.28f, 2.0f }, {} } );
    shape = world_.CreateShape( ragdollBodies_[10], MakeBox( { 0.16f, 0.75f } ) );
    world_.SetShapeFriction( shape, 0.6f );
    shapes_.push_back( { ragdollBodies_[10], shape, "오른쪽 종아리 [렉돌]" } );

    const auto connect = [this]( std::size_t index, bodyId bodyA, bodyId bodyB, vec2 localAnchorA, vec2 localAnchorB, float lowerAngle, float upperAngle )
    {
        revoluteJointDef joint{};
        joint.bodyA = bodyA;
        joint.bodyB = bodyB;
        joint.localAnchorA = localAnchorA;
        joint.localAnchorB = localAnchorB;
        joint.enableLimit = true;
        joint.lowerAngle = lowerAngle;
        joint.upperAngle = upperAngle;
        ragdollJoints_[index] = world_.createRevoluteJoint( joint );
    };

    constexpr float DEGREE = std::numbers::pi_v<float> / 180.0f;
    connect( 0, ragdollBodies_[1], ragdollBodies_[0], { 0.0f, 0.75f }, { 0.0f, -0.35f }, -25.0f * DEGREE, 25.0f * DEGREE );
    connect( 1, ragdollBodies_[2], ragdollBodies_[1], { 0.0f, 0.35f }, { 0.0f, -0.75f }, -20.0f * DEGREE, 20.0f * DEGREE );
    connect( 2, ragdollBodies_[1], ragdollBodies_[3], { -0.55f, 0.25f }, { 0.55f, 0.0f }, -90.0f * DEGREE, 90.0f * DEGREE );
    connect( 3, ragdollBodies_[3], ragdollBodies_[4], { -0.55f, 0.0f }, { 0.55f, 0.0f }, -120.0f * DEGREE, 10.0f * DEGREE );
    connect( 4, ragdollBodies_[1], ragdollBodies_[5], { 0.55f, 0.25f }, { -0.55f, 0.0f }, -90.0f * DEGREE, 90.0f * DEGREE );
    connect( 5, ragdollBodies_[5], ragdollBodies_[6], { 0.55f, 0.0f }, { -0.55f, 0.0f }, -10.0f * DEGREE, 120.0f * DEGREE );
    connect( 6, ragdollBodies_[2], ragdollBodies_[7], { -0.28f, -0.35f }, { 0.0f, 0.75f }, -55.0f * DEGREE, 55.0f * DEGREE );
    connect( 7, ragdollBodies_[7], ragdollBodies_[8], { 0.0f, -0.75f }, { 0.0f, 0.75f }, -10.0f * DEGREE, 115.0f * DEGREE );
    connect( 8, ragdollBodies_[2], ragdollBodies_[9], { 0.28f, -0.35f }, { 0.0f, 0.75f }, -55.0f * DEGREE, 55.0f * DEGREE );
    connect( 9, ragdollBodies_[9], ragdollBodies_[10], { 0.0f, -0.75f }, { 0.0f, 0.75f }, -115.0f * DEGREE, 10.0f * DEGREE );

    // 기존 Sandbox 입력과 Revolute 시각화를 그대로 재사용함. 허리를 대표 limit으로 표시함.
    impulseBody_ = ragdollBodies_[1];
    torqueBody_ = ragdollBodies_[1];
    revoluteJoint_ = ragdollJoints_[1];
    for( std::size_t i = 0; i < ragdollBodies_.size(); ++i ) ragdollInitialTransforms_[i] = world_.GetBodyTransform( ragdollBodies_[i] );
}

void rigidBodyDemo::resetRagdoll()
{
    if( kind_ != demoKind::ragdoll ) return;
    cancelInput();
    for( std::size_t i = 0; i < ragdollBodies_.size(); ++i )
    {
        const auto body = ragdollBodies_[i];
        if( !world_.IsValid( body ) ) continue;
        world_.SetBodyTransform( body, ragdollInitialTransforms_[i] );
        world_.SetBodyLinearVelocity( body, {} );
        world_.SetBodyAngularVelocity( body, 0.0f );
        world_.ClearForces( body );
    }
    refreshContacts();
}

void rigidBodyDemo::setRagdollJointFriction( float torque )
{
    assert( std::isfinite( torque ) && torque >= 0.0f );
    if( kind_ != demoKind::ragdoll ) return;
    for( const auto id : ragdollJoints_ )
        if( world_.IsValid( id ) ) world_.setRevoluteJointMotor( id, torque > 0.0f, 0.0f, torque );
}

float rigidBodyDemo::getRagdollJointFriction() const
{
    if( kind_ == demoKind::ragdoll )
        for( const auto id : ragdollJoints_ )
            if( world_.IsValid( id ) )
            {
                const auto joint = world_.getRevoluteJointData( id );
                return joint.enableMotor ? joint.maxMotorTorque : 0.0f;
            }
    return 0.0f;
}

void rigidBodyDemo::setRagdollSelfCollision( bool enabled )
{
    if( kind_ != demoKind::ragdoll ) return;
    for( const auto& visual : shapes_ )
    {
        if( !world_.IsValid( visual.shapeHandle ) || !world_.IsValid( visual.bodyHandle ) || world_.GetBody( visual.bodyHandle ).type != bodyType::Dynamic ) continue;
        auto filter = world_.GetShape( visual.shapeHandle ).filter;
        filter.groupIndex = enabled ? 0 : -1;
        world_.SetShapeFilter( visual.shapeHandle, filter );
    }
    refreshContacts();
}

bool rigidBodyDemo::getRagdollSelfCollision() const
{
    if( kind_ == demoKind::ragdoll )
        for( const auto& visual : shapes_ )
            if( world_.IsValid( visual.shapeHandle ) && world_.IsValid( visual.bodyHandle ) && world_.GetBody( visual.bodyHandle ).type == bodyType::Dynamic )
                return world_.GetShape( visual.shapeHandle ).filter.groupIndex >= 0;
    return false;
}

#pragma endregion SceneSetup

#pragma region CarDrive

void rigidBodyDemo::resetCar()
{
    if( kind_ != demoKind::motorCar || !world_.IsValid( impulseBody_ ) ) return;
    cancelInput();
    world_.SetBodyTransform( impulseBody_, { { 0.0f, 1.1f }, {} } );
    world_.SetBodyLinearVelocity( impulseBody_, {} );
    world_.SetBodyAngularVelocity( impulseBody_, 0.0f );
    world_.ClearForces( impulseBody_ );
    for( std::size_t i = 0; i < carJoints_.size(); ++i )
    {
        if( !world_.IsValid( carJoints_[i] ) ) continue;
        const auto wheel = world_.getWheelJointData( carJoints_[i] ).bodyB;
        world_.SetBodyTransform( wheel, { { i == 0 ? -0.9f : 0.9f, 0.45f }, {} } );
        world_.SetBodyLinearVelocity( wheel, {} );
        world_.SetBodyAngularVelocity( wheel, 0.0f );
        world_.ClearForces( wheel );
    }
    refreshContacts();
}

void rigidBodyDemo::applyCarSuspensionPreset( carSuspensionPreset preset )
{
    if( kind_ != demoKind::motorCar ) return;
    const float hertz = preset == carSuspensionPreset::soft ? 2.0f : preset == carSuspensionPreset::stiff ? 8.0f : 4.0f;
    for( const auto id : carJoints_ )
    {
        if( !world_.IsValid( id ) ) continue;
        world_.setWheelJointSpring( id, preset != carSuspensionPreset::noSpring, hertz, 0.7f );
        world_.setWheelJointLimit( id, true, -0.25f, 0.25f );
    }
}

void rigidBodyDemo::setCarLowFriction( bool enabled )
{
    if( kind_ != demoKind::motorCar ) return;
    for( const auto& visual : shapes_ )
        if( visual.bodyHandle != impulseBody_ && world_.IsValid( visual.shapeHandle ) ) world_.SetShapeFriction( visual.shapeHandle, enabled ? 0.02f : 0.9f );
}

std::optional<vec2> rigidBodyDemo::getCameraTarget() const
{
    if( kind_ == demoKind::motorCar && carCameraTracking_ && world_.IsValid( impulseBody_ ) ) return world_.GetBodyTransform( impulseBody_ ).position;
    return std::nullopt;
}

carWheelObservation rigidBodyDemo::getCarWheelObservation( std::size_t wheelIndex ) const
{
    carWheelObservation result{};
    if( kind_ != demoKind::motorCar || wheelIndex >= carJoints_.size() || !world_.IsValid( carJoints_[wheelIndex] ) ) return result;
    const auto wheel = world_.getWheelJointData( carJoints_[wheelIndex] ).bodyB;
    const vec2 center = TransformPoint( world_.GetBodyTransform( wheel ), world_.GetBodyLocalCenter( wheel ) );
    std::array<contactData, 4> contacts{};
    const auto count = world_.GetBodyContactData( wheel, contacts );
    for( std::size_t index = 0; index < count; ++index )
        for( const auto& visual : shapes_ )
        {
            if( !world_.IsValid( visual.bodyHandle ) || world_.GetBody( visual.bodyHandle ).type != bodyType::Static ) continue;
            const auto& contact = contacts[index];
            vec2 normal{};
            if( contact.shapeA == visual.shapeHandle ) normal = contact.manifold.normal;
            else if( contact.shapeB == visual.shapeHandle ) normal = -contact.manifold.normal;
            if( normal.y < 0.5f ) continue;
            for( int point = 0; point < contact.manifold.pointCount; ++point )
            {
                const auto& hit = contact.manifold.points[point];
                if( hit.separation > LINEAR_SLOP ) continue;
                result.grounded = true;
                const vec2 velocity = world_.GetBodyLinearVelocity( wheel ) + Cross( world_.GetBodyAngularVelocity( wheel ), hit.point - center );
                result.slipSpeed = std::max( result.slipSpeed, std::abs( Dot( velocity, Cross( 1.0f, normal ) ) ) );
            }
        }
    return result;
}

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

#pragma region TerrainEdit

void rigidBodyDemo::startTerrainPointDrag( vec2 point )
{
    terrainPointDragIndex_.reset();
    splineDragRecorded_ = false;
    endSplineSamplingEdit();
    if( !( kind_ == demoKind::terrainChain && world_.IsValid( terrainChain_ ) ) && !hasEditableSpline() ) return;
    point = InverseTransformPoint( kind_ == demoKind::terrainChain ? world_.GetBodyTransform( shapes_[0].bodyHandle ) : getSplineControlTransform(), point );

    const float pickRadiusSquared = TERRAIN_POINT_PICK_RADIUS * TERRAIN_POINT_PICK_RADIUS;
    float bestDistanceSquared = pickRadiusSquared;

    const std::size_t pointCount = hasEditableSpline() ? splineControlPoints_.size() : getTerrainPointCount();
    for( std::size_t i = 0; i < pointCount; ++i )
    {
        const vec2 control = hasEditableSpline() ? splineControlPoints_[i] : getTerrainPoint( i );
        const float distanceSquared = LengthSquared( control - point );
        if( distanceSquared > bestDistanceSquared ) continue;

        bestDistanceSquared = distanceSquared;
        terrainPointDragIndex_ = i;
    }
}

void rigidBodyDemo::updateTerrainPointDrag( vec2 point )
{
    if( !terrainPointDragIndex_.has_value() || ( !world_.IsValid( terrainChain_ ) && !hasEditableSpline() ) ) return;
    point = InverseTransformPoint( kind_ == demoKind::terrainChain ? world_.GetBodyTransform( shapes_[0].bodyHandle ) : getSplineControlTransform(), point );

    const std::size_t pointIndex = *terrainPointDragIndex_;
    if( hasEditableSpline() )
    {
        updateSplineControlPoint( pointIndex, point );
        return;
    }
    const vec2 currentPoint = getTerrainPoint( pointIndex );
    if( currentPoint.x == point.x && currentPoint.y == point.y ) return;

    if( !canMoveTerrainPoint( pointIndex, point ) ) return;

    world_.SetChainPoint( terrainChain_, pointIndex, point );
    // Pause 상태에서도 contact debug 표시가 이전 geometry를 가리키지 않게 즉시 다시 수집함.
    refreshContacts();
}

void rigidBodyDemo::updateSplineControlPoint( std::size_t pointIndex, vec2 point )
{
    if( pointIndex >= splineControlPoints_.size() || !IsFinite( point ) ) return;
    const vec2 current = splineControlPoints_[pointIndex];
    if( current.x == point.x && current.y == point.y ) return;

    auto candidate = splineControlPoints_;
    candidate[pointIndex] = point;
    if( updateSplineGeometry( candidate, splineSamplingError_ ) )
    {
        if( !splineDragRecorded_ ) rememberSplineEdit();
        splineDragRecorded_ = true;
        splineControlPoints_ = candidate;
    }
}

bool rigidBodyDemo::insertSplineControlPoint( std::size_t segmentIndex )
{
    if( !hasEditableSpline() || segmentIndex >= splineControlPoints_.size() - 1 ) return false;
    return insertSplineControlPoint( segmentIndex, Evaluate( { splineControlPoints_ }, segmentIndex, 0.5f ) );
}

bool rigidBodyDemo::insertSplineControlPointAt( vec2 point, float pixelsPerMeter )
{
    if( !hasEditableSpline() || !std::isfinite( pixelsPerMeter ) || pixelsPerMeter <= 0.0f ) return false;
    const float radius = 10.0f / pixelsPerMeter;
    if( !std::isfinite( radius ) || radius <= 0.0f ) return false;
    point = InverseTransformPoint( kind_ == demoKind::terrainChain ? world_.GetBodyTransform( shapes_[0].bodyHandle ) : getSplineControlTransform(), point );
    if( !IsFinite( point ) ) return false;
    // 10px 판정에 비해 충분히 작은 0.01px 거리 오차로 실제 곡선 구간을 찾음.
    const auto closest = ClosestPoint( { splineControlPoints_ }, point, static_cast<double>( radius ) * 0.001 );
    if( !closest || closest->distanceSquared > static_cast<double>( radius ) * radius ) return false;
    if( !insertSplineControlPoint( closest->segmentIndex, point ) ) return false;
    terrainPointDragIndex_ = closest->segmentIndex + 1;
    return true;
}

bool rigidBodyDemo::insertSplineControlPoint( std::size_t segmentIndex, vec2 point )
{
    if( !hasEditableSpline() || segmentIndex >= splineControlPoints_.size() - 1 || !IsFinite( point ) ) return false;
    auto candidate = splineControlPoints_;
    candidate.insert( candidate.begin() + static_cast<std::ptrdiff_t>( segmentIndex + 1 ), point );
    if( !updateSplineGeometry( candidate, splineSamplingError_ ) ) return false;
    rememberSplineEdit();
    splineControlPoints_.swap( candidate );
    terrainPointDragIndex_.reset();
    splineDragRecorded_ = false;
    endSplineSamplingEdit();
    return true;
}

bool rigidBodyDemo::removeSplineControlPoint( std::size_t pointIndex )
{
    if( !hasEditableSpline() || splineControlPoints_.size() <= 2 || pointIndex >= splineControlPoints_.size() ) return false;
    auto candidate = splineControlPoints_;
    candidate.erase( candidate.begin() + static_cast<std::ptrdiff_t>( pointIndex ) );
    if( !updateSplineGeometry( candidate, splineSamplingError_ ) ) return false;
    rememberSplineEdit();
    splineControlPoints_.swap( candidate );
    terrainPointDragIndex_.reset();
    splineDragRecorded_ = false;
    endSplineSamplingEdit();
    return true;
}

bool rigidBodyDemo::resetSplineTerrain()
{
    if( kind_ != demoKind::splineTerrain || !world_.IsValid( terrainChain_ ) ) return false;
    const bool original = std::equal( splineControlPoints_.begin(), splineControlPoints_.end(), TERRAIN_CONTROL_POINTS.begin(), TERRAIN_CONTROL_POINTS.end(),
        []( vec2 a, vec2 b ) { return a.x == b.x && a.y == b.y; } );
    if( original && splineSamplingError_ == 0.01f ) return false;
    if( !updateSplineGeometry( TERRAIN_CONTROL_POINTS, 0.01f ) ) return false;
    rememberSplineEdit();
    splineControlPoints_.assign( TERRAIN_CONTROL_POINTS.begin(), TERRAIN_CONTROL_POINTS.end() );
    splineSamplingError_ = 0.01f;
    terrainPointDragIndex_.reset();
    splineDragRecorded_ = false;
    endSplineSamplingEdit();
    return true;
}

void rigidBodyDemo::setSplineSamplingError( float error )
{
    if( kind_ != demoKind::splineTerrain || !world_.IsValid( terrainChain_ ) || !std::isfinite( error ) || error < 0.005f || error > 0.05f || error == splineSamplingError_ ) return;
    if( updateSplineGeometry( splineControlPoints_, error ) )
    {
        if( !splineSamplingEditActive_ || !splineSamplingEditRecorded_ ) rememberSplineEdit();
        splineSamplingEditRecorded_ = true;
        splineSamplingError_ = error;
        terrainPointDragIndex_.reset();
        splineDragRecorded_ = false;
    }
}

void rigidBodyDemo::beginSplineSamplingEdit()
{
    splineSamplingEditActive_ = true;
    splineSamplingEditRecorded_ = false;
}

void rigidBodyDemo::endSplineSamplingEdit()
{
    splineSamplingEditActive_ = false;
    splineSamplingEditRecorded_ = false;
}

void rigidBodyDemo::rememberSplineEdit()
{
    splineUndo_ = splineEditState{ splineControlPoints_, splineSamplingError_ };
    splineRedo_.reset();
}

bool rigidBodyDemo::restoreSplineEdit( std::optional<splineEditState>& source, std::optional<splineEditState>& destination )
{
    if( !hasEditableSpline() || !source ) return false;
    if( !updateSplineGeometry( source->points, source->error ) ) return false;
    destination = splineEditState{ splineControlPoints_, splineSamplingError_ };
    splineControlPoints_.swap( source->points );
    splineSamplingError_ = source->error;
    source.reset();
    terrainPointDragIndex_.reset();
    splineDragRecorded_ = false;
    endSplineSamplingEdit();
    return true;
}

bool rigidBodyDemo::undoSplineEdit()
{
    return restoreSplineEdit( splineUndo_, splineRedo_ );
}

bool rigidBodyDemo::redoSplineEdit()
{
    return restoreSplineEdit( splineRedo_, splineUndo_ );
}

bool rigidBodyDemo::updateSplineGeometry( std::span<const vec2> points, float error )
{
    if( kind_ == demoKind::splinePath || kind_ == demoKind::splineRail )
    {
        if( !hasEditableSpline() ) return false;
        if( kind_ == demoKind::splineRail ) return world_.setSplineRailJointPoints( splineRailJoint_, points );
        splineRailJointSim2 candidate{};
        if( !setSplineRailGeometry( candidate, points ) ) return false;
        const auto length = GetLength( { points }, SPLINE_PATH_TOLERANCE );
        if( !length ) return false;
        const double fraction = splinePathLength_ > 0.0 ? splinePathDistance_ / splinePathLength_ : 0.0;
        splinePathGeometry_ = std::move( candidate.geometry );
        splinePathLength_ = *length;
        splinePathDistance_ = fraction * *length;
        splinePathNeedsAlignment_ = true;
        return true;
    }
    const auto samples = SampleAdaptive( { points }, error );
    if( samples.size() < 2 ) return false;
    const vec2 ghost1 = samples.front() + ( samples.front() - samples[1] );
    const vec2 ghost2 = samples.back() + ( samples.back() - samples[samples.size() - 2] );
    const std::size_t oldCount = terrainSegments_.size();
    const bodyId ground = shapes_[0].bodyHandle;
    const std::array<visualShape, 3> probes{ shapes_[oldCount], shapes_[oldCount + 1], shapes_[oldCount + 2] };
    shapes_.reserve( samples.size() + 2 );
    terrainSegments_.reserve( samples.size() - 1 );
    if( !world_.SetChainPoints( terrainChain_, samples, ghost1, ghost2 ) ) return false;

    if( samples.size() - 1 != oldCount )
    {
        terrainSegments_.resize( samples.size() - 1 );
        const std::size_t count = world_.GetChainSegments( terrainChain_, terrainSegments_ );
        assert( count == terrainSegments_.size() );
        shapes_.clear();
        for( const shapeId child : terrainSegments_ ) shapes_.push_back( { ground, child, "지형 edge [Chain]" } );
        for( const auto probe : probes ) shapes_.push_back( probe );
    }
    terrainTrialObservation_ = {};
    terrainTrialReference_.reset();
    refreshContacts();
    return true;
}

bool rigidBodyDemo::canMoveTerrainPoint( std::size_t pointIndex, vec2 point ) const
{
    if( !IsFinite( point ) ) return false;

    const std::size_t pointCount = getTerrainPointCount();
    if( pointIndex >= pointCount ) return false;

    const float minimumLengthSquared = LINEAR_SLOP * LINEAR_SLOP;
    if( pointIndex > 0 && LengthSquared( point - getTerrainPoint( pointIndex - 1 ) ) <= minimumLengthSquared ) return false;

    if( pointIndex + 1 < pointCount && LengthSquared( getTerrainPoint( pointIndex + 1 ) - point ) <= minimumLengthSquared ) return false;

    if( pointIndex == 0 )
    {
        const chainSegment2& first = std::get<chainSegment2>( world_.GetShape( terrainSegments_.front() ).geometry );
        if( LengthSquared( point - first.ghost1 ) <= minimumLengthSquared ) return false;
    }

    if( pointIndex + 1 == pointCount )
    {
        const chainSegment2& last = std::get<chainSegment2>( world_.GetShape( terrainSegments_.back() ).geometry );
        if( LengthSquared( last.ghost2 - point ) <= minimumLengthSquared ) return false;
    }

    return true;
}

#pragma endregion TerrainEdit

#pragma region MouseDrag

void rigidBodyDemo::startMouseDrag( vec2 point )
{
    // World Tree에서 실제로 점을 포함하는 Shape 후보를 먼저 찾음.
    // Inspector의 충돌 필터와 무관하게 선택할 수 있도록 조회에는 모든 category bit를 사용함.
    std::vector<shapeId> candidates;
    const std::uint64_t allBits = std::numeric_limits<std::uint64_t>::max();
    world_.overlapPoint( point, queryFilter{ allBits, allBits },
        [&]( shapeId id )
        {
            candidates.push_back( id );
            return true;
        } );

    // 화면 그리기 순서가 곧 선택 우선순위임. Tree의 방문 순서에는 의존하지 않음.
    for( auto visual = shapes_.rbegin(); visual != shapes_.rend(); ++visual )
    {
        if( !world_.IsValid( visual->bodyHandle ) || !world_.IsValid( visual->shapeHandle ) ) continue;
        if( world_.GetBody( visual->bodyHandle ).type != bodyType::Dynamic || world_.IsShapeSensor( visual->shapeHandle ) ) continue;

        const collisionFilter filter = world_.GetShapeFilter( visual->shapeHandle );
        // category/mask가 0인 Shape는 World query filter로 조회할 수 없지만,
        // 충돌 비활성화와 마우스 선택은 별개이므로 기존 직접 점 판정을 유지함.
        const bool excludedFromQuery = filter.categoryBits == 0 || filter.maskBits == 0;
        if( excludedFromQuery )
        {
            if( !world_.testShapePoint( visual->shapeHandle, point ) ) continue;
        }
        else if( std::find( candidates.begin(), candidates.end(), visual->shapeHandle ) == candidates.end() )
            continue;

        // 클릭한 월드 위치를 목표점으로 지정하면 World가 로컬 작용점을 저장함.
        auto definition = mouseSettings_;
        mouseReference_ = world_.CreateBody();
        definition.bodyA = mouseReference_;
        definition.bodyB = visual->bodyHandle;
        definition.target = point;
        mouseJoint_ = world_.createMouseJoint( definition );
        break;
    }
}

void rigidBodyDemo::stopMouseDrag()
{
    // 참조 물체 삭제가 연결된 Mouse Joint도 함께 제거함. 잡힌 물체가 먼저 삭제돼도 안전함.
    if( world_.IsValid( mouseReference_ ) ) world_.DestroyBody( mouseReference_ );
    mouseReference_ = {};
    mouseJoint_ = {};
}

#pragma endregion MouseDrag

std::unique_ptr<demo> createRigidBodyDemo( demoKind kind )
{
    return std::make_unique<rigidBodyDemo>( kind );
}

} // namespace zonai::sandbox
