#pragma once

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

    [[nodiscard]] jointId getMouseJoint() const noexcept { return mouseJoint_; }

    [[nodiscard]] const mouseJointDef& getMouseSettings() const noexcept { return mouseSettings_; }

#pragma endregion Queries

private:
#pragma region SceneSetup
    void createPlayground();
    void createPendulum();
#pragma endregion SceneSetup

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

    jointId mouseJoint_{}; // 드래그 중인 마우스 조인트
    mouseJointDef mouseSettings_{};

    bool leftHeld_ = false; // 누르고 있는 이동 입력
    bool rightHeld_ = false;
#pragma endregion Data
};

[[nodiscard]] std::unique_ptr<demo> createRigidBodyDemo( demoKind kind );

} // namespace zonai::sandbox
