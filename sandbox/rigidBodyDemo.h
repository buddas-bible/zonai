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
    void handleInput( const demoInput& input ) override;
    void cancelInput() override;
    void setMouseSettings( float hertz, float dampingRatio, float maxForce );
    void refreshContacts();
#pragma endregion

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
#pragma endregion

private:
    void createPlayground();
    void createPendulum();
    void startMouseDrag( vec2 point );
    demoKind kind_;
    world world_;
    std::vector<visualShape> shapes_;
    std::vector<contactData> contacts_;
    bodyId impulseBody_{};
    bodyId torqueBody_{};
    bodyId pendulumBody_{};
    jointId pendulumJoint_{};
    jointId mouseJoint_{};
    mouseJointDef mouseSettings_{};
    bool leftHeld_ = false;
    bool rightHeld_ = false;
};

[[nodiscard]] std::unique_ptr<demo> createRigidBodyDemo( demoKind kind );
} // namespace zonai::sandbox
