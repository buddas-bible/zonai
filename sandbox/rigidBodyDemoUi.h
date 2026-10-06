#pragma once

#include "rigidBodyDemo.h"

namespace zonai::sandbox
{
// Physics/입력 model과 ImGui 표시를 분리해 GUI 없이 같은 데모를 검증함.
class rigidBodyDemoUi final : public rigidBodyDemo
{
public:
    explicit rigidBodyDemoUi( demoKind kind );
    void drawControls() override;
    void draw( debugDraw& draw ) const override;

private:
#pragma region Controls
    void drawWorldSettings();
    void drawInspector();
    void drawExperimentControls();
    void drawMouseControls();
    void drawDebugSettings();
#pragma endregion
    int selectedShapeIndex_ = 0;
    bool showGrid_ = true;
    bool showShapeAABBs_ = true;
    bool showFatAABBs_ = false;
    bool showContacts_ = true;
    bool showContactDetails_ = true;
    bool showCOM_ = true;
    bool showVelocities_ = true;
    bool showLabels_ = true;
    bool showDynamicTree_ = false;
    bool showKinematicTree_ = false;
    bool showStaticTree_ = false;
    bool showTreeLeaves_ = true;
    bool showTreeInternal_ = true;
    bool showTreeLabels_ = false;
};

[[nodiscard]] std::unique_ptr<demo> createDemoView( demoKind kind );
} // namespace zonai::sandbox
