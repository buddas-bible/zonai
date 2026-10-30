#pragma once

#include <span>
#include <vector>

#include "collision/shapeGeometry.h"
#include "math/vec2.h"

namespace zonai
{

// Body 좌표가 아닌 유체 world 좌표. 원형 solid의 cell-centered occupancy 근사.
struct fluidCircleObstacle2
{
    vec2 center{};
    float radius = 0.0f;
    vec2 linearVelocity{};
    float angularVelocity = 0.0f;
};

// World shape geometry를 복사해서 넘기는 비소유(non-owning world ID) 장애물 snapshot.
// transform은 Shape의 body-local geometry를 world-space로 변환하고 center는 실제 COM.
// 속도는 center(COM)의 world-space velocity. 정적 obstacle도 동일한 형식으로 처리.
struct fluidShapeObstacle2
{
    shapeGeometry geometry{};
    transform2 transform{};
    vec2 center{};
    vec2 linearVelocity{};
    float angularVelocity = 0.0f;
};

// 2D 단위 깊이, 유체 밀도 1kg/m³ 가정의 압력 반력. force(N), torque(N·m).
struct fluidObstacleForce2
{
    vec2 force{};
    float torque = 0.0f;
};

// Pressure projection 한 번의 운동량 수지. 모든 값은 N·s.
// fluid: 유체-유체 MAC face 속도 보정량의 운동량 변화.
// solid: 유체 압력으로 강체에 가해진 충격량. wall: 외곽 닫힌 벽에 가해진 충격량.
// 압력 이외의 이류, cell 점유 변화, moving boundary, drag는 포함하지 않는다.
struct fluidPressureMomentum2
{
    vec2 fluid{};
    vec2 solid{};
    vec2 wall{};

    [[nodiscard]] vec2 residual() const noexcept { return fluid + solid + wall; }
};

// 장애물 갱신, 이류, 압력 보정 단계별로 유체-유체 face의 운동량을 측정.
// getFluidMomentum()과 같은 불완전한 dual-face 진단량이며 전체 유체 질량 적분은 아님.
struct fluidStepMomentum2
{
    vec2 before{};
    vec2 afterObstacles{};
    vec2 afterAdvection{};
    vec2 afterPressure{};

    [[nodiscard]] vec2 occupancyDelta() const noexcept { return afterObstacles - before; }
    [[nodiscard]] vec2 advectionDelta() const noexcept { return afterAdvection - afterObstacles; }
    [[nodiscard]] vec2 pressureDelta() const noexcept { return afterPressure - afterAdvection; }
};

// Moving solid의 cell-centered 마스크 전이 및 재노출 MAC face 재구성 통계.
// fluidVolumeDelta는 2D unit-depth(1m)에서의 *이산 셀 면적* 변화이며
// 실제 이동하는 연속 표면의 체적이나 보존된 유체 질량이 아님.
struct fluidSolidTransition2
{
    int exposedCells = 0;
    int coveredCells = 0;
    int openedUFaces = 0;
    int openedVFaces = 0;
    int closedUFaces = 0;
    int closedVFaces = 0;
    int extrapolatedFaces = 0;
    int fallbackFaces = 0;
    float fluidVolumeDelta = 0.0f;
};

// 장애물과 격자의 교차 면적/face 개방 비율을 sub-cell 표본으로 계산한 관측량.
// covered/released는 이전과 현재 snapshot의 국소 점유 변화량 (m³, unit depth).
// 장애물을 통과한 swept flux 또는 실제 유체 질량 보존량이 아님.
struct fluidCutCellTransition2
{
    float previousFluidVolume = 0.0f;
    float fluidVolume = 0.0f;
    float coveredVolume = 0.0f;
    float releasedVolume = 0.0f;
    int partialCells = 0;
    int partialUFaces = 0;
    int partialVFaces = 0;

    [[nodiscard]] float volumeDelta() const noexcept { return fluidVolume - previousFluidVolume; }
};

// 부분 개방 면의 유량을 사용하는 실험용 압력 투영 결과.
// divergenceBefore/After는 div(a*u) + ΔfluidFraction/dt의 RMS (1/s).
// sourceImbalance: 닫힌 도메인에서 압력으로 해소할 수 없는 전체 유량 (m³/s).
// compatibilityCorrection: 서로 분리된 각 유체 영역의 유량 불일치 절댓값 합 (m³/s).
// volumeRate: 직전/현재 sub-cell 기하의 부호 있는 유체 체적 변화율 (m³/s).
// 최초 geometry snapshot은 초기 상태로 취급해 시간적 소스를 적용하지 않음.
struct fluidCutCellPressure2
{
    float divergenceBefore = 0.0f;
    float divergenceAfter = 0.0f;
    float pressureResidual = 0.0f;
    float sourceImbalance = 0.0f;
    float compatibilityCorrection = 0.0f;
    float volumeRate = 0.0f;
    float redistributedVolumeRate = 0.0f;
    float unresolvedVolumeRate = 0.0f;
    int activeCells = 0;
    int connectedRegions = 0;
    bool hasVolumeHistory = false;
};

// 균일 MAC(staggered) grid: x속도는 수직 face, y속도는 수평 face에 저장.
// 내부 Circle/Capsule/Convex Polygon solid 경계 face는 no-through로 처리함.
class fluidField2
{
public:
#pragma region Lifetime
    // 너비와 높이는 cell 개수. 최소 2, 최대 256. origin은 좌하단 경계.
    fluidField2( int width, int height, float cellSize, vec2 origin = {} );
#pragma endregion Lifetime

#pragma region Simulation
    void clear() noexcept;

    // 실험 모드: sub-cell 개방 비율의 유량과 가중 압력 Poisson 사용.
    // 기본은 기존 이진 solid 압력 solver. 위치 snapshot의 체적 변화율만 근사하며
    // 정확한 swept-volume 경로/보존형 이류는 아직 구현하지 않음.
    void setCutCellProjectionEnabled( bool enabled ) noexcept { cutCellProjectionEnabled_ = enabled; }
    [[nodiscard]] bool isCutCellProjectionEnabled() const noexcept { return cutCellProjectionEnabled_; }

    // 지정 위치 주변 face 속도에 world-space 속도 변화(m/s)를 더함.
    // 소스가 격자 바깥이거나 radius가 유효하지 않으면 아무 일도 하지 않음.
    void addVelocity( vec2 position, vec2 velocityDelta, float radius );

    // rho = 1kg/m³, depth = 1m, face 유효 질량 = cellSize² kg.
    // 원형 support의 fluid-fluid 내부 face에 각 성분 충격량을 정규화해서 배분.
    // 실제 배분한 충격량을 돌려주며, 유효 face가 없는 성분은 0 (반력 없음).
    // 압력 투영/이류/경계 외력까지 포함한 총 운동량 보존을 뜻하지 않음.
    [[nodiscard]] vec2 applyMomentumImpulse( vec2 position, vec2 impulse, float radius );

    // Semi-Lagrangian advection 후 Jacobi pressure projection.
    // no-through(경계 수직 속도 0) 조건. dt<=0 또는 비유한 값은 무시.
    void step( float timeStep, int pressureIterations = 80 );

    // 매 Step 제공한 원의 위치/속도로 solid occupancy를 갱신한다.
    // feedback은 obstacle 개수와 정확히 같은 크기여야 한다.
    // stationary 또는 이동하는 solid 경계의 법선 속도를 사용하며 pressure force를 돌려준다.
    void step( float timeStep, std::span<const fluidCircleObstacle2> obstacles,
        std::span<fluidObstacleForce2> feedback, int pressureIterations = 80 );

    // Circle, Capsule, Convex Polygon을 body-local geometry + world transform으로 전달.
    // unsupported geometry는 비점유로 처리. feedback은 obstacle 순서와 1:1 대응.
    void step( float timeStep, std::span<const fluidShapeObstacle2> obstacles,
        std::span<fluidObstacleForce2> feedback, int pressureIterations = 80 );
#pragma endregion Simulation

#pragma region Queries
    [[nodiscard]] vec2 sampleVelocity( vec2 position ) const noexcept;
    [[nodiscard]] float getDivergenceRms() const noexcept;

    // 내부 fluid-fluid MAC face 속도를 dual-cell mass로 적분한 운동량.
    // solid 경계의 지정 속도 및 외곽 wall face는 제외한다.
    [[nodiscard]] vec2 getFluidMomentum() const noexcept;
    [[nodiscard]] const fluidPressureMomentum2& getPressureMomentum() const noexcept { return pressureMomentum_; }
    [[nodiscard]] const fluidStepMomentum2& getStepMomentum() const noexcept { return stepMomentum_; }
    [[nodiscard]] const fluidSolidTransition2& getSolidTransition() const noexcept { return solidTransition_; }
    [[nodiscard]] const fluidCutCellTransition2& getCutCellTransition() const noexcept { return cutCellTransition_; }
    [[nodiscard]] const fluidCutCellPressure2& getCutCellPressure() const noexcept { return cutCellPressure_; }
    // 기하 표본의 유체 면적 비율 [0,1]. 범위 밖은 0.
    // 외곽 wall face는 기하상 열려있어도 닫힌 경계이므로 0.
    [[nodiscard]] float getCellFluidFraction( int x, int y ) const noexcept;
    [[nodiscard]] float getUFaceOpenFraction( int x, int y ) const noexcept;
    [[nodiscard]] float getVFaceOpenFraction( int x, int y ) const noexcept;
    [[nodiscard]] bool isSolidCell( int x, int y ) const noexcept;
    [[nodiscard]] int getSolidCellCount() const noexcept { return solidCellCount_; }
    [[nodiscard]] vec2 getCellCenter( int x, int y ) const noexcept;
    [[nodiscard]] int getWidth() const noexcept { return width_; }
    [[nodiscard]] int getHeight() const noexcept { return height_; }
    [[nodiscard]] float getCellSize() const noexcept { return cellSize_; }
    [[nodiscard]] vec2 getOrigin() const noexcept { return origin_; }
#pragma endregion Queries

private:
#pragma region Helpers
    [[nodiscard]] int cellIndex( int x, int y ) const noexcept { return y * width_ + x; }
    [[nodiscard]] int uIndex( int x, int y ) const noexcept { return y * ( width_ + 1 ) + x; }
    [[nodiscard]] int vIndex( int x, int y ) const noexcept { return y * width_ + x; }
    [[nodiscard]] vec2 sampleFrom( const std::vector<float>& u, const std::vector<float>& v, vec2 position ) const noexcept;
    static float sampleComponent( const std::vector<float>& values, int width, int height, float x, float y ) noexcept;
    void updateObstacles( std::span<const fluidShapeObstacle2> obstacles );
    void updateCutCellGeometry( std::span<const fluidShapeObstacle2> obstacles );
    void remapExposedFaces();
    [[nodiscard]] float cutCellUOpen( int x, int y ) const noexcept;
    [[nodiscard]] float cutCellVOpen( int x, int y ) const noexcept;
    [[nodiscard]] float cutCellFluxDivergenceRms( float inverseTimeStep ) const noexcept;
    void prepareCutCellVolumeChange( float timeStep );
    void applyCutCellBoundaries() noexcept;
    void projectCutCell( float timeStep, int pressureIterations,
        std::span<const fluidShapeObstacle2> obstacles, std::span<fluidObstacleForce2> feedback );
    void applySolidBoundaries( std::span<const fluidShapeObstacle2> obstacles );
    void project( float timeStep, int pressureIterations, std::span<const fluidShapeObstacle2> obstacles,
        std::span<fluidObstacleForce2> feedback );
#pragma endregion Helpers

#pragma region Data
    int width_ = 0;
    int height_ = 0;
    float cellSize_ = 1.0f;
    vec2 origin_{};
    std::vector<float> u_;
    std::vector<float> v_;
    std::vector<float> nextU_;
    std::vector<float> nextV_;
    std::vector<float> pressure_;
    std::vector<float> nextPressure_;
    std::vector<float> divergence_;
    std::vector<int> solidOwner_;
    std::vector<int> previousSolidOwner_;
    std::vector<float> cellFluidFraction_;
    std::vector<float> previousCellFluidFraction_;
    std::vector<float> uOpenFraction_;
    std::vector<float> vOpenFraction_;
    std::vector<float> uSolidVelocity_;
    std::vector<float> vSolidVelocity_;
    std::vector<float> cutCellVolumeRate_;
    std::vector<int> cutCellComponent_;
    std::vector<int> cutCellNearestFluid_;
    std::vector<int> cutCellQueue_;
    int solidCellCount_ = 0;
    fluidSolidTransition2 solidTransition_{};
    fluidCutCellTransition2 cutCellTransition_{};
    fluidCutCellPressure2 cutCellPressure_{};
    bool cutCellProjectionEnabled_ = false;
    bool hasCutCellHistory_ = false;
    bool volumeSourceValid_ = false;
    fluidPressureMomentum2 pressureMomentum_{};
    fluidStepMomentum2 stepMomentum_{};
#pragma endregion Data
};

} // namespace zonai
