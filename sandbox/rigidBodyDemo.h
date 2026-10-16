#pragma once

#include <array>
#include <optional>
#include <vector>

#include "demo.h"
#include "dynamics/world.h"

namespace zonai::sandbox
{

struct visualShape
{
    bodyId bodyHandle{};
    shapeId shapeHandle{};
    const char* label = "";
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
#pragma endregion Settings

#pragma region Contacts
    void refreshContacts();
#pragma endregion Contacts

#pragma region Queries

    [[nodiscard]] demoKind getKind() const noexcept { return kind_; }

    [[nodiscard]] world& getWorld() noexcept { return world_; }

    [[nodiscard]] const world& getWorld() const noexcept { return world_; }

    [[nodiscard]] std::span<const visualShape> getShapes() const noexcept { return shapes_; }

    [[nodiscard]] std::span<const contactData> getContacts() const noexcept { return contacts_; }

    [[nodiscard]] bodyId getImpulseBody() const noexcept { return impulseBody_; }

    [[nodiscard]] bodyId getTorqueBody() const noexcept { return torqueBody_; }

    [[nodiscard]] bodyId getPendulumBody() const noexcept { return pendulumBody_; }

    [[nodiscard]] jointId getPendulumJoint() const noexcept { return pendulumJoint_; }

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

    [[nodiscard]] std::size_t getTerrainPointCount() const noexcept;

    [[nodiscard]] vec2 getTerrainPoint( std::size_t pointIndex ) const;

    // 추가 / 삭제 / 편집 복원 후에는 이전 span을 다시 조회해야 함.
    [[nodiscard]] std::span<const vec2> getSplineControlPoints() const noexcept { return kind_ == demoKind::splineTerrain ? std::span<const vec2>{ splineControlPoints_ } : std::span<const vec2>{}; }

    [[nodiscard]] float getSplineSamplingError() const noexcept { return splineSamplingError_; }
    void setSplineSamplingError( float error );
    bool insertSplineControlPoint( std::size_t segmentIndex );
    bool removeSplineControlPoint( std::size_t pointIndex );
    bool resetSplineTerrain();
    [[nodiscard]] float getSplineTrialFraction() const noexcept { return splineTrialFraction_; }
    void setSplineTrialFraction( float fraction );
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
    void createTerrainChain();
    void createPendulum();
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
    void updateSplineControlPoint( std::size_t pointIndex, vec2 point );
    bool updateSplineTerrain( std::span<const vec2> points, float error );
    [[nodiscard]] bool canMoveTerrainPoint( std::size_t pointIndex, vec2 point ) const;
#pragma endregion TerrainEdit

#pragma region MouseDrag
    void startMouseDrag( vec2 point );
    void stopMouseDrag();
#pragma endregion MouseDrag

#pragma region Data
    demoKind kind_;
    world world_;
    std::vector<visualShape> shapes_;
    std::vector<contactData> contacts_;

    bodyId impulseBody_{}; // 이동과 임펄스 입력을 적용할 물체
    std::array<transform2, 6> playgroundInitialTransforms_{};
    bodyId torqueBody_{};  // 회전 임펄스를 적용할 물체
    bodyId pendulumBody_{};
    jointId pendulumJoint_{};
    jointId revoluteJoint_{};
    jointId wheelJoint_{};
    jointId prismaticJoint_{};
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
