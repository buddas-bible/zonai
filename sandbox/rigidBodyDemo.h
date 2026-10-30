#pragma once

#include <array>
#include <optional>
#include <vector>

#include "demo.h"
#include "dynamics/world.h"
#include "fluid/fluidField2.h"

namespace zonai::sandbox
{

struct visualShape
{
    bodyId bodyHandle{};
    shapeId shapeHandle{};
    const char* label = "";
};

// World의 한 Step짜리 이벤트를 Sandbox에서 잠깐 유지하는 위치 snapshot.
enum class contactEventTraceType { begin, end, hit };

struct contactEventTrace2
{
    contactEventTraceType type = contactEventTraceType::begin;
    vec2 point{};
    vec2 normal{};
    float approachSpeed = 0.0f;
    float normalImpulse = 0.0f;
    float remainingTime = 0.0f;
    float duration = 0.0f;
};

enum class distanceDemoPreset
{
    rigid,
    spring,
    limit,
    motor
};

enum class revoluteDemoPreset
{
    free,
    limit,
    motor,
    motorLimit
};

enum class wheelDemoPreset
{
    spring,
    limit,
    motor,
    combined
};

enum class prismaticDemoPreset
{
    free,
    limited,
    locked,
    motorForward,
    motorReverse,
    brake,
    motorLimit,
    spring,
    springLimit,
    combined
};

enum class weldDemoPreset
{
    rigid,
    softLinear,
    softAngular,
    softBoth
};

enum class carSuspensionPreset { soft, standard, stiff, noSpring };
enum class terrainTrial { seams, front, back };

// 현재 충돌 지형의 world-space 기준점과 단위 접선.
struct terrainTrialTarget
{
    vec2 point{};
    vec2 tangent{};
};

struct terrainTrialObservation
{
    bool active = false;
    std::size_t probeIndex = 0;
    terrainTrial trial = terrainTrial::front;
    float launchSpeed = 0.0f;
    vec2 launchGravity{};
    float launchGravityScale = 0.0f;
    float launchFriction = 0.0f;
    float launchRestitution = 0.0f;
    float launchMass = 0.0f;
    float launchInertia = 0.0f;
    std::optional<float> launchFraction{};
    float elapsed = 0.0f;
    bool touching = false;
    std::optional<float> firstContactTime{};
    float firstContactTimeStep = 0.0f;
    int firstContactSubStepCount = 0;
    float initialTimeStep = 0.0f;
    int initialSubStepCount = 0;
    bool preContactSettingsChanged = false;
    bool preContactGravityChanged = false;
    bool preContactGravityScaleChanged = false;
    bool preContactMaterialChanged = false;
    bool preContactMassChanged = false;
    bool preContactInertiaChanged = false;
};

enum class terrainTrialComparisonStatus { noReference, noObservation, differentConditions, waitingForContact, ready };
enum class terrainTrialStepComparison { unavailable, constantMatch, constantDifferent, changed };

struct terrainTrialComparison
{
    terrainTrialComparisonStatus status = terrainTrialComparisonStatus::noReference;
    float firstContactTimeDifference = 0.0f;
    bool contactStepSettingsMatch = false;
    terrainTrialStepComparison stepSettings = terrainTrialStepComparison::unavailable;
    bool gravityChangesObserved = false;
    bool materialChangesObserved = false;
    bool massChangesObserved = false;
    bool inertiaChangesObserved = false;
    bool gravityScaleChangesObserved = false;
};

struct carWheelObservation
{
    bool grounded = false;
    float slipSpeed = 0.0f;
};

// Motor Joint의 velocity actuator와 transform spring을 독립 / 결합해서 비교하는 빠른 설정.
enum class motorJointDemoPreset
{
    brake,
    linear,
    angular,
    combined,
    linearSpring,
    angularSpring,
    springBoth,
    velocityAndSpring
};

// Mover Joint의 x/y 목표속도와 축별 힘 한도를 빠르게 비교하는 설정.
enum class moverJointDemoPreset
{
    horizontal,
    vertical,
    diagonal,
    anisotropic
};

// Pogo의 spring 응답과 인장/압축 force budget 차이를 빠르게 비교하는 설정.
enum class pogoJointDemoPreset
{
    soft,
    stiff,
    compressionOnly,
    asymmetric
};

class rigidBodyDemo : public demo
{
public:
#pragma region LifetimeAndSimulation
    explicit rigidBodyDemo( demoKind kind );
    void step( float timeStep, int subStepCount ) override;
#pragma endregion LifetimeAndSimulation

#pragma region Input
    void handleInput( const demoInput& input ) override;
    void cancelInput() override;
#pragma endregion Input

#pragma region Settings
    void setCollisionMatrix( const collisionMatrix& matrix ) override;
    void setMouseSettings( float hertz, float dampingRatio, float maxForce );
    void setCarMotorSettings( float speed, float maxTorque );
    void resetPlayground();
    void resetCollisionTrial();
    void applyMassTrialImpulse();
    void applyPointTrialImpulse();
    void applyInertiaTrialImpulse();
    void startForceImpulseTrial();
    void startFrictionForceTrial();
    void startTorqueImpulseTrial();

    [[nodiscard]] float getRotationTrialTimeStep() const noexcept { return rotationTrialTimeStep_; }

    [[nodiscard]] double getTorqueTrialTime() const noexcept { return torqueTrialTime_; }

    [[nodiscard]] double getForceTrialTime() const noexcept { return forceTrialTime_; }
    [[nodiscard]] bool isForceTrialActive() const noexcept { return forceTrialRunning_ && forceTrialTime_ < 1.0; }
    void resetSplinePath();
    void resetTerrainProbes();
    void launchTerrainProbe( std::size_t probeIndex, terrainTrial trial );
    void resetCar();
    void applyCarSuspensionPreset( carSuspensionPreset preset );
    void setCarLowFriction( bool enabled );
    void setCarCameraTracking( bool enabled ) { carCameraTracking_ = enabled; }
    [[nodiscard]] bool getCarCameraTracking() const { return carCameraTracking_; }
    [[nodiscard]] std::optional<vec2> getCameraTarget() const override;
    [[nodiscard]] carWheelObservation getCarWheelObservation( std::size_t wheelIndex ) const;
    void setPrismaticSpringSettings( bool enableSpring, float hertz, float dampingRatio, float targetTranslation );
    void setPrismaticLimitSettings( bool enableLimit, float lowerTranslation, float upperTranslation );
    void setPrismaticMotorSettings( bool enableMotor, float motorSpeed, float maxMotorForce );
    void setPrismaticSpringTargetToCurrent();
    void resetPrismaticSlider();
    void setPrismaticIncline( bool enabled );
    void setWeldLinearSettings( float hertz, float dampingRatio );
    void setWeldAngularSettings( float hertz, float dampingRatio );
    void setMotorJointLinearSettings( vec2 linearVelocity, float maxVelocityForce );
    void setMotorJointAngularSettings( float angularVelocity, float maxVelocityTorque );
    void setMotorJointLinearSpringSettings( float hertz, float dampingRatio, float maxSpringForce );
    void setMotorJointAngularSpringSettings( float referenceAngle, float hertz, float dampingRatio, float maxSpringTorque );
    void setMoverJointSettings( vec2 linearVelocity, vec2 maxVelocityForce );
    void setPogoJointSettings( float restLength, float hertz, float dampingRatio, float maxTensionForce, float maxCompressionForce );
    // Filter는 설정값이 아니라 Joint 존재 여부 자체로 collision 차단을 전환함.
    void setFilterJointEnabled( bool enabled );
    void resetFilterJointDemo();
    void testFilterCollision();
    void prepareFilterSleepTest();
    void wakeFilterBody();
    void applyDistancePreset( distanceDemoPreset preset );
    void applyRevolutePreset( revoluteDemoPreset preset );
    void testRevoluteLimit();
    void applyWheelPreset( wheelDemoPreset preset );
    void testWheelLimit( bool upper );
    void applyPrismaticPreset( prismaticDemoPreset preset );
    void applyWeldPreset( weldDemoPreset preset );
    void resetWeldPair();
    void setWeldCantilever( bool enabled );
    void applyMotorJointPreset( motorJointDemoPreset preset );
    void resetMotorJointDemo();
    void setMotorTargetTracking( bool enabled );
    void applyMoverJointPreset( moverJointDemoPreset preset );
    void resetMoverJointDemo();
    void setMoverGravity( bool enabled );
    void resetRagdoll();
    void setRagdollJointFriction( float torque );
    [[nodiscard]] float getRagdollJointFriction() const;
    void setRagdollSelfCollision( bool enabled );
    [[nodiscard]] bool getRagdollSelfCollision() const;
    void applyPogoJointPreset( pogoJointDemoPreset preset );
    void setWindSourceStrength( float strength );
    void setWindDragCoefficient( float rate );
    void setWindSourceEnabled( bool enabled ) noexcept { windSourceEnabled_ = enabled; }
    void setWindObstacleCouplingEnabled( bool enabled ) noexcept { windObstacleCouplingEnabled_ = enabled; }
    void setWindCutCellProjectionEnabled( bool enabled ) noexcept
    {
        if( windField_ ) windField_->setCutCellProjectionEnabled( enabled );
    }
    [[nodiscard]] bool isWindCutCellProjectionEnabled() const noexcept
    {
        return windField_ && windField_->isCutCellProjectionEnabled();
    }
    void resetWindFlow();
    [[nodiscard]] float getWindSourceStrength() const noexcept { return windSourceStrength_; }
    [[nodiscard]] float getWindDragCoefficient() const noexcept { return windDragCoefficient_; }
    [[nodiscard]] bool isWindSourceEnabled() const noexcept { return windSourceEnabled_; }
    [[nodiscard]] bool isWindObstacleCouplingEnabled() const noexcept { return windObstacleCouplingEnabled_; }
    [[nodiscard]] std::span<const fluidObstacleForce2> getWindPressureForces() const noexcept { return windPressureForces_; }
    [[nodiscard]] vec2 getWindDragBodyImpulse() const noexcept { return windDragBodyImpulse_; }
    [[nodiscard]] vec2 getWindDragFluidImpulse() const noexcept { return windDragFluidImpulse_; }
    [[nodiscard]] vec2 getWindDragExchangeResidual() const noexcept { return windDragBodyImpulse_ + windDragFluidImpulse_; }
#pragma endregion Settings

#pragma region Contacts
    void refreshContacts();
    void setContactEventVisualizationEnabled( bool enabled );
    [[nodiscard]] bool isContactEventVisualizationEnabled() const noexcept { return contactEventVisualizationEnabled_; }
    void setContactEventTrailDuration( float seconds );
    [[nodiscard]] float getContactEventTrailDuration() const noexcept { return contactEventTrailDuration_; }
    [[nodiscard]] std::span<const contactEventTrace2> getContactEventTraces() const noexcept { return contactEventTraces_; }
    void clearContactEventTrail() noexcept { contactEventTraces_.clear(); }
#pragma endregion Contacts

#pragma region Queries

    [[nodiscard]] demoKind getKind() const noexcept { return kind_; }

    [[nodiscard]] const fluidField2* getWindField() const noexcept { return windField_ ? &*windField_ : nullptr; }
    [[nodiscard]] world& getWorld() noexcept { return world_; }

    [[nodiscard]] const world& getWorld() const noexcept { return world_; }

    [[nodiscard]] std::span<const visualShape> getShapes() const noexcept { return shapes_; }

    [[nodiscard]] std::span<const contactData> getContacts() const noexcept { return contacts_; }

    [[nodiscard]] std::optional<vec2> getImpulseTrialPoint( std::size_t shapeIndex ) const;

    [[nodiscard]] bodyId getImpulseBody() const noexcept { return impulseBody_; }

    [[nodiscard]] bodyId getTorqueBody() const noexcept { return torqueBody_; }

    [[nodiscard]] bodyId getPendulumBody() const noexcept { return pendulumBody_; }

    [[nodiscard]] jointId getPendulumJoint() const noexcept { return pendulumJoint_; }
    [[nodiscard]] std::span<const jointId> getSpringComparisonJoints() const noexcept { return springComparisonJoints_; }
    [[nodiscard]] std::span<const jointId> getMotorComparisonJoints() const noexcept { return motorComparisonJoints_; }
    [[nodiscard]] std::span<const jointId> getLimitComparisonJoints() const noexcept { return limitComparisonJoints_; }
    [[nodiscard]] std::span<const jointId> getTorqueComparisonJoints() const noexcept { return torqueComparisonJoints_; }
    [[nodiscard]] std::span<const jointId> getAngleLimitComparisonJoints() const noexcept { return angleLimitComparisonJoints_; }
    [[nodiscard]] std::span<const jointId> getSuspensionComparisonJoints() const noexcept { return suspensionComparisonJoints_; }
    [[nodiscard]] std::span<const jointId> getSuspensionLimitJoints() const noexcept { return suspensionLimitJoints_; }
    [[nodiscard]] std::span<const jointId> getSuspensionMotorJoints() const noexcept { return suspensionMotorJoints_; }
    [[nodiscard]] std::span<const jointId> getWeldComparisonJoints() const noexcept { return weldComparisonJoints_; }

    [[nodiscard]] jointId getRevoluteJoint() const noexcept { return revoluteJoint_; }

    [[nodiscard]] jointId getWheelJoint() const noexcept { return wheelJoint_; }

    [[nodiscard]] jointId getPrismaticJoint() const noexcept { return prismaticJoint_; }

    [[nodiscard]] jointId getWeldJoint() const noexcept { return weldJoint_; }

    [[nodiscard]] jointId getMotorJoint() const noexcept { return motorJoint_; }

    [[nodiscard]] jointId getMoverJoint() const noexcept { return moverJoint_; }

    [[nodiscard]] jointId getPogoJoint() const noexcept { return pogoJoint_; }
    [[nodiscard]] const pogoJointDef& getPogoSettings() const noexcept { return pogoSettings_; }
    [[nodiscard]] vec2 getPogoProbeOrigin() const noexcept { return pogoProbeOrigin_; }
    [[nodiscard]] vec2 getPogoProbeTranslation() const noexcept { return pogoProbeTranslation_; }

    [[nodiscard]] jointId getFilterJoint() const noexcept { return filterJoint_; }

    [[nodiscard]] std::span<const bodyId> getRagdollBodies() const noexcept { return ragdollBodies_; }

    [[nodiscard]] std::span<const jointId> getRagdollJoints() const noexcept { return ragdollJoints_; }

    [[nodiscard]] float getPrismaticCurrentSpeed() const;

    [[nodiscard]] std::span<const jointId> getCarJoints() const noexcept { return carJoints_; }

    [[nodiscard]] float getCarMotorSpeed() const noexcept { return carMotorSpeed_; }

    [[nodiscard]] float getCarMaxMotorTorque() const noexcept { return carMaxMotorTorque_; }

    [[nodiscard]] jointId getMouseJoint() const noexcept { return mouseJoint_; }

    [[nodiscard]] const mouseJointDef& getMouseSettings() const noexcept { return mouseSettings_; }

    [[nodiscard]] std::span<const vec2> getSplinePathPoints() const noexcept;
    [[nodiscard]] jointId getSplineRailJoint() const noexcept { return splineRailJoint_; }
    [[nodiscard]] double getSplinePathDistance() const noexcept { return splinePathDistance_; }
    [[nodiscard]] double getSplinePathLength() const noexcept { return splinePathLength_; }

    [[nodiscard]] std::size_t getTerrainPointCount() const noexcept;

    [[nodiscard]] vec2 getTerrainPoint( std::size_t pointIndex ) const;

    // 추가 / 삭제 / 편집 복원 후에는 이전 span을 다시 조회해야 함.
    [[nodiscard]] std::span<const vec2> getSplineControlPoints() const noexcept { return kind_ == demoKind::splineTerrain || kind_ == demoKind::splinePath || kind_ == demoKind::splineRail ? std::span<const vec2>{ splineControlPoints_ } : std::span<const vec2>{}; }

    [[nodiscard]] bool hasEditableSpline() const noexcept;
    [[nodiscard]] transform2 getSplineControlTransform() const;

    [[nodiscard]] float getSplineSamplingError() const noexcept { return splineSamplingError_; }
    void setSplineSamplingError( float error );
    bool insertSplineControlPoint( std::size_t segmentIndex );
    bool removeSplineControlPoint( std::size_t pointIndex );
    bool resetSplineTerrain();
    [[nodiscard]] float getSplineTrialFraction() const noexcept { return splineTrialFraction_; }
    void setSplineTrialFraction( float fraction );
    [[nodiscard]] float getSplineTrialSpeed() const noexcept { return splineTrialSpeed_; }
    void setSplineTrialSpeed( float speed );
    [[nodiscard]] terrainTrialObservation getTerrainTrialObservation() const noexcept { return terrainTrialObservation_; }
    [[nodiscard]] std::optional<terrainTrialObservation> getTerrainTrialReference() const noexcept { return terrainTrialReference_; }
    bool pinTerrainTrialReference();
    bool relaunchTerrainTrialReference();
    [[nodiscard]] terrainTrialComparison getTerrainTrialComparison() const noexcept;
    void clearTerrainTrialReference() noexcept { terrainTrialReference_.reset(); }
    [[nodiscard]] std::optional<terrainTrialTarget> getTerrainTrialTarget( terrainTrial trial ) const;
    [[nodiscard]] bool canUndoSplineEdit() const noexcept { return splineUndo_.has_value(); }
    [[nodiscard]] bool canRedoSplineEdit() const noexcept { return splineRedo_.has_value(); }
    bool undoSplineEdit();
    bool redoSplineEdit();
    void beginSplineSamplingEdit();
    void endSplineSamplingEdit();

    [[nodiscard]] std::optional<std::size_t> getTerrainPointDragIndex() const noexcept { return terrainPointDragIndex_; }

#pragma endregion Queries

private:
#pragma region SceneSetup
    void createPlayground();
    void createWindFlow();
    void createCollisionTrial();
    void createSplinePath();
    void createSplineRail();
    [[nodiscard]] std::optional<splineSample2> getSplinePathSample( double distance ) const;
    void updateSplinePath( float timeStep );
    void createTerrainChain();
    void createPendulum();
    void createSpringDampingComparison();
    void createDistanceMotorForceComparison();
    void createDistanceLimitMotorComparison();
    void createRevoluteMotorTorqueComparison();
    void createRevoluteLimitMotorComparison();
    void createSuspensionLoadComparison();
    void createSuspensionLimitComparison();
    void createSuspensionMotorComparison();
    void createWeldElasticityComparison();
    void createRevoluteHinge();
    void createWheelSuspension();
    void createPrismaticRail();
    void createWeldPair( bool anchored = false );
    void createMouseJointPlayground();
    void createMotorJointPlayground();
    void createMoverJointPlayground();
    void createPogoJointPlayground();
    void updatePogoSupport();
    void createFilterJointPlayground();
    void createMotorCar();
    void createRagdoll();
#pragma endregion SceneSetup

#pragma region CarDrive
    void updateCarMotors();
#pragma endregion CarDrive

#pragma region TerrainEdit
    struct splineEditState
    {
        std::vector<vec2> points;
        float error;
    };

    void rememberSplineEdit();
    bool restoreSplineEdit( std::optional<splineEditState>& source, std::optional<splineEditState>& destination );
    void startTerrainPointDrag( vec2 point );
    void updateTerrainPointDrag( vec2 point );
    void updateSplineTrialPosition( vec2 point );
    void updateTerrainTrialObservation( float timeStep, int subStepCount );
    void updateSplineControlPoint( std::size_t pointIndex, vec2 point );
    bool insertSplineControlPointAt( vec2 point, float pixelsPerMeter );
    bool insertSplineControlPoint( std::size_t segmentIndex, vec2 point );
    bool updateSplineGeometry( std::span<const vec2> points, float error );
    [[nodiscard]] bool canMoveTerrainPoint( std::size_t pointIndex, vec2 point ) const;
#pragma endregion TerrainEdit

#pragma region MouseDrag
    void startMouseDrag( vec2 point );
    void stopMouseDrag();
#pragma endregion MouseDrag

#pragma region ContactEventVisualization
    void syncContactEventSubscriptions();
    void updateContactEventTrail( float timeStep );
#pragma endregion ContactEventVisualization

#pragma region Data

    struct trialBodyState
    {
        transform2 transform{};
        vec2 velocity{};
        float angularVelocity = 0.0f;
    };

    std::vector<trialBodyState> collisionTrialInitialStates_;
    double forceTrialTime_ = 0.0;
    bool forceTrialRunning_ = false;
    double torqueTrialTime_ = 0.0;
    bool torqueTrialRunning_ = false;
    float rotationTrialTimeStep_ = 0.0f;
    demoKind kind_;
    world world_;
    std::vector<visualShape> shapes_;
    std::vector<contactData> contacts_;
    bool contactEventVisualizationEnabled_ = false;
    float contactEventTrailDuration_ = 0.35f;
    std::vector<contactEventTrace2> contactEventTraces_;
    std::optional<fluidField2> windField_{};
    float windSourceStrength_ = 6.0f;
    float windDragCoefficient_ = 3.0f;
    float windTime_ = 0.0f;
    bool windSourceEnabled_ = true;
    bool windObstacleCouplingEnabled_ = true;
    std::array<fluidObstacleForce2, 3> windPressureForces_{};
    vec2 windDragBodyImpulse_{};
    vec2 windDragFluidImpulse_{};

    bodyId impulseBody_{}; // 이동과 임펄스 입력을 적용할 물체
    std::array<transform2, 6> playgroundInitialTransforms_{};
    preparedCatmullRomSpline2 splinePathGeometry_{};
    double splinePathDistance_ = 0.0;
    double splinePathLength_ = 0.0;
    bool splinePathNeedsAlignment_ = false;
    bodyId torqueBody_{};  // 회전 임펄스를 적용할 물체
    bodyId pendulumBody_{};
    jointId pendulumJoint_{};
    std::array<jointId, 3> springComparisonJoints_{};
    std::array<jointId, 3> motorComparisonJoints_{};
    std::array<jointId, 2> limitComparisonJoints_{};
    std::array<jointId, 3> torqueComparisonJoints_{};
    std::array<jointId, 2> angleLimitComparisonJoints_{};
    std::array<jointId, 3> suspensionComparisonJoints_{};
    std::array<jointId, 2> suspensionLimitJoints_{};
    std::array<jointId, 2> suspensionMotorJoints_{};
    std::array<jointId, 3> weldComparisonJoints_{};
    jointId revoluteJoint_{};
    jointId wheelJoint_{};
    jointId prismaticJoint_{};
    jointId splineRailJoint_{};
    bodyId splineRailBody_{};
    jointId weldJoint_{};
    jointId motorJoint_{}; // Motor Joint 전용 학습 데모에서 사용하는 persistent handle.
    bool motorTargetTracking_ = false;
    float motorTargetTime_ = 0.0f;
    jointId moverJoint_{}; // Mover Joint 전용 학습 데모의 persistent handle.
    jointId pogoJoint_{}; // 발밑 지면을 다시 탐지할 때 교체되는 지지 조인트.
    pogoJointDef pogoSettings_{};
    vec2 pogoProbeOrigin_{};
    vec2 pogoProbeTranslation_{};
    bool pogoJumping_ = false;
    jointId filterJoint_{}; // Filter Joint 데모에서 collision 차단 상태를 나타내는 handle.
    bodyId filterTestBody_{};
    std::array<bodyId, 11> ragdollBodies_{};
    std::array<transform2, 11> ragdollInitialTransforms_{};
    std::array<jointId, 10> ragdollJoints_{};
    std::array<jointId, 2> carJoints_{};
    float carMotorSpeed_ = 8.0f; // 주행 목표의 크기 (rad/s). 방향은 A/D 입력이 결정함.
    float carMaxMotorTorque_ = 5.0f; // 주행과 제동의 토크 한도 (N*m).
    bool carCameraTracking_ = true;

    chainId terrainChain_{};
    std::vector<vec2> splineControlPoints_{};
    float splineSamplingError_ = 0.01f;
    float splineTrialFraction_ = 0.5f;
    float splineTrialSpeed_ = 8.0f;
    terrainTrialObservation terrainTrialObservation_{};
    std::optional<terrainTrialObservation> terrainTrialReference_{};
    bool splineTrialDragging_ = false;
    bool splineInserting_ = false;
    std::optional<splineEditState> splineUndo_{};
    std::optional<splineEditState> splineRedo_{};
    bool splineDragRecorded_ = false;
    bool splineSamplingEditActive_ = false;
    bool splineSamplingEditRecorded_ = false;
    std::array<transform2, 3> terrainInitialTransforms_{};
    std::vector<shapeId> terrainSegments_{};
    std::optional<std::size_t> terrainPointDragIndex_{};

    jointId mouseJoint_{}; // 드래그 중인 마우스 조인트
    bodyId mouseReference_{}; // 드래그 동안만 존재하는 정적 참조 물체.
    mouseJointDef mouseSettings_{};

    bool leftHeld_ = false; // 누르고 있는 이동 입력
    bool rightHeld_ = false;
    bool brakeHeld_ = false;
#pragma endregion Data
};

[[nodiscard]] std::unique_ptr<demo> createRigidBodyDemo( demoKind kind );

} // namespace zonai::sandbox
