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
#pragma endregion Controls
    int selectedShapeIndex_ = 0;
    bool showGrid_ = true;
    bool showShapeAABBs_ = false;
    bool showFatAABBs_ = true;
    bool showContacts_ = true;
    bool showContactDetails_ = false;
    bool showCOM_ = true;
    bool showVelocities_ = true;
    bool showLabels_ = false;
    bool showDynamicTree_ = true;
    bool showKinematicTree_ = false;
    bool showStaticTree_ = false;
    bool showTreeLeaves_ = true;
    bool showTreeInternal_ = true;
    bool showTreeLabels_ = false;
};

[[nodiscard]] bool initializeDemoUi();
[[nodiscard]] std::unique_ptr<demo> createDemoView( demoKind kind );

// 편집 화면에 표시할 레이어만 보관함. 숨기는 것은 충돌 규칙을 바꾸지 않음.
struct collisionSettingsUi
{
    std::uint64_t visibleLayers = 1;
};

void drawProjectCollisionSettings( demoSession& session, collisionSettingsUi& state );

} // namespace zonai::sandbox
