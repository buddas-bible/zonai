#pragma once

#include "rigidBodyDemo.h"

namespace zonai::sandbox
{

// Renderer-only settings. Tracer / MAC field values are NEVER changed by these.
struct windVisualization2
{
    bool dye = true;
    bool vectors = true;
    bool speedHeatmap = false;
    bool pressureHeatmap = false;
    bool cutCells = false;
    float dyeReference = 4.0f; // kg/m^3 for near-saturation
    float dyeOpacity = 0.90f;
    float speedReference = 8.0f; // m/s, saturated face-speed display
    float pressureReference = 10.0f; // relative pressure, density = 1
    float overlayOpacity = 0.55f;
};

// Physics/입력 model과 ImGui 표시를 분리해 GUI 없이 같은 데모를 검증함.
class rigidBodyDemoUi final : public rigidBodyDemo
{
public:
    explicit rigidBodyDemoUi( demoKind kind );
    void handleInput( const demoInput& input ) override;
    void cancelInput() override;
    void drawControls() override;
    void draw( debugDraw& draw ) const override;
    [[nodiscard]] const windVisualization2& getWindVisualization() const noexcept
    {
        return windVisualization_;
    }
    // Explicit test/debug settings; normal user edits live in drawExperimentControls().
    [[nodiscard]] windVisualization2& editWindVisualization() noexcept
    {
        return windVisualization_;
    }
    [[nodiscard]] int getWindProbeX() const noexcept { return windProbeX_; }
    [[nodiscard]] int getWindProbeY() const noexcept { return windProbeY_; }
    // Reads active fluid-fluid MAC faces and focuses the largest raw speed.
    // Does not change any simulated velocity, tracer or obstacle.
    [[nodiscard]] bool focusPeakWindFace() noexcept;

private:
#pragma region Controls
    void drawWorldSettings();
    void drawInspector();
    void drawExperimentControls();
    void drawSplineEditorControls();
    void drawMouseControls();
    void drawDebugSettings();
    void drawWindProbeControls();
#pragma endregion Controls
    int selectedShapeIndex_ = 0;
    shapeId selectedShape_{};
    int windFieldWidthEdit_ = 24;
    int windFieldHeightEdit_ = 14;
    float windCellSizeEdit_ = 0.5f;
    float windStaticPositionEdit_[2]{ 0.0f, 0.0f };
    float windStaticHalfExtentsEdit_[2]{ 0.7f, 0.3f };
    float windStaticRadiusEdit_ = 0.5f;
    int terrainProbeIndex_ = 0;
    std::size_t splineControlIndex_ = 0;
    bool splineRailSelecting_ = false;
    bool splineRailReprojectSelected_ = false;
    std::size_t splineRailReprojectSegment_ = 0;
    float splineRailReprojectDistance_ = 0.5f;
    std::optional<bool> splineRailReprojectResult_{};
    windVisualization2 windVisualization_{};
    int windProbeX_ = -1;
    int windProbeY_ = -1;
    int windProbeFaceX_ = -1;
    int windProbeFaceY_ = -1;
    bool windProbeFaceIsU_ = true;
    bool showGrid_ = true;
    bool showShapeAABBs_ = false;
    bool showFatAABBs_ = false;
    bool showContacts_ = true;
    bool showContactDetails_ = false;
    bool showCOM_ = true;
    bool showVelocities_ = true;
    bool showLabels_ = false;
    bool showDynamicTree_ = false;
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
