#pragma once

#include <array>
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
    void setPrismaticSpringSettings( bool enableSpring, float hertz, float dampingRatio, float targetTranslation );
    void setPrismaticLimitSettings( bool enableLimit, float lowerTranslation, float upperTranslation );
    void setPrismaticMotorSettings( bool enableMotor, float motorSpeed, float maxMotorForce );
    void setPrismaticSpringTargetToCurrent();
    void setWeldLinearSettings( float hertz, float dampingRatio );
    void setWeldAngularSettings( float hertz, float dampingRatio );
    void setMotorJointLinearSettings( vec2 linearVelocity, float maxVelocityForce );
    void setMotorJointAngularSettings( float angularVelocity, float maxVelocityTorque );
    void setMotorJointLinearSpringSettings( float hertz, float dampingRatio, float maxSpringForce );
    void setMotorJointAngularSpringSettings( float referenceAngle, float hertz, float dampingRatio, float maxSpringTorque );
    void setMoverJointSettings( vec2 linearVelocity, vec2 maxVelocityForce );
    void setPogoJointSettings( float restLength, float hertz, float dampingRatio, float maxTensionForce, float maxCompressionForce );
    void applyDistancePreset( distanceDemoPreset preset );
    void applyRevolutePreset( revoluteDemoPreset preset );
    void applyWheelPreset( wheelDemoPreset preset );
    void applyPrismaticPreset( prismaticDemoPreset preset );
    void applyWeldPreset( weldDemoPreset preset );
    void applyMotorJointPreset( motorJointDemoPreset preset );
    void applyMoverJointPreset( moverJointDemoPreset preset );
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

    [[nodiscard]] float getPrismaticCurrentSpeed() const;

    [[nodiscard]] std::span<const jointId> getCarJoints() const noexcept { return carJoints_; }

    [[nodiscard]] float getCarMotorSpeed() const noexcept { return carMotorSpeed_; }

    [[nodiscard]] float getCarMaxMotorTorque() const noexcept { return carMaxMotorTorque_; }

    [[nodiscard]] jointId getMouseJoint() const noexcept { return mouseJoint_; }

    [[nodiscard]] const mouseJointDef& getMouseSettings() const noexcept { return mouseSettings_; }

#pragma endregion Queries

private:
#pragma region SceneSetup
    void createPlayground();
    void createPendulum();
    void createRevoluteHinge();
    void createWheelSuspension();
    void createPrismaticRail();
    void createWeldPair();
    void createMouseJointPlayground();
    void createMotorJointPlayground();
    void createMoverJointPlayground();
    void createPogoJointPlayground();
    void createMotorCar();
#pragma endregion SceneSetup

#pragma region CarDrive
    void updateCarMotors();
#pragma endregion CarDrive

#pragma region MouseDrag
    void startMouseDrag( vec2 point );
#pragma endregion MouseDrag

#pragma region Data
    demoKind kind_;
    world world_;
    std::vector<visualShape> shapes_;
    std::vector<contactData> contacts_;

    bodyId impulseBody_{}; // 이동과 임펄스 입력을 적용할 물체
    bodyId torqueBody_{};  // 회전 임펄스를 적용할 물체
    bodyId pendulumBody_{};
    jointId pendulumJoint_{};
    jointId revoluteJoint_{};
    jointId wheelJoint_{};
    jointId prismaticJoint_{};
    jointId weldJoint_{};
    jointId motorJoint_{}; // Motor Joint 전용 학습 데모에서 사용하는 persistent handle.
    jointId moverJoint_{}; // Mover Joint 전용 학습 데모의 persistent handle.
    jointId pogoJoint_{}; // Pogo Joint 전용 학습 데모의 persistent handle.
    std::array<jointId, 2> carJoints_{};
    float carMotorSpeed_ = 8.0f; // 주행 목표의 크기 (rad/s). 방향은 A/D 입력이 결정함.
    float carMaxMotorTorque_ = 5.0f; // 주행과 제동의 토크 한도 (N*m).

    jointId mouseJoint_{}; // 드래그 중인 마우스 조인트
    mouseJointDef mouseSettings_{};

    bool leftHeld_ = false; // 누르고 있는 이동 입력
    bool rightHeld_ = false;
    bool brakeHeld_ = false;
#pragma endregion Data
};

[[nodiscard]] std::unique_ptr<demo> createRigidBodyDemo( demoKind kind );

} // namespace zonai::sandbox
