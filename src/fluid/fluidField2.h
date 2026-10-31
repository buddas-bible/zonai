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
    // 압력-강체 연립 투영에서 사용할 COM 역질량/역관성.
    // 둘 다 0이면 기존 고정 운동학 경계와 Jacobi 압력 solver로 처리.
    float inverseMass = 0.0f;
    float inverseInertia = 0.0f;
};

// 2D 단위 깊이, 유체 밀도 1kg/m³ 가정의 압력 반력. force(N), torque(N·m).
struct fluidObstacleForce2
{
    vec2 force{};
    float torque = 0.0f;
};

// PCG 압력 해의 종료 원인. noBoundary는 결합할 Fluid-Solid 면이 없어 일반 압력 투영 사용.
enum class fluidCoupledPressureStatus2
{
    notRequested,
    converged,
    noBoundary,
    iterationLimit,
    numericalBreakdown,
    invalidPressure
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
    bool usedCoupledProjection = false;
    bool coupledSolveFailed = false;
    fluidCoupledPressureStatus2 coupledStatus = fluidCoupledPressureStatus2::notRequested;
    int coupledIterations = 0;
    int coupledIterationLimit = 0;
    // 압력 연립식 A*p=b의 보정된 RHS 기준 RMS, 상대 잔차. Fluid 발산 RMS와는 별개.
    float coupledInitialResidualRms = 0.0f;
    float coupledFinalResidualRms = 0.0f;
    float coupledRelativeResidual = 0.0f;
    // 수렴한 압력이 예측한 Body Δv/Δω를 MAC solid 법선 face에 반영한 결과.
    // 실제 World의 후속 Drag/Contact/Joint Δv는 여기에 포함하지 않는다.
    int synchronizedBoundaryFaces = 0;
    float boundaryNormalChangeRms = 0.0f;

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

// 연속한 Shape 변환 사이의 내부 MAC 면을 통과한 Solid 속도를 시간 적분한 근사.
// 경로를 사용할 수 없으면 직전 셀 점유율 차이로 fallback 한다.
// totalAbsoluteFlux는 면마다 |적분량|을 합한 값이며 실제 물체 체적은 아님.
// geometryMismatch는 swept face divergence와 snapshot Δfluid volume의 셀별 L1 차이.
struct fluidSweptFaceFlux2
{
    bool usedSweptFlux = false;
    int timeSamples = 0;
    float totalAbsoluteFlux = 0.0f;
    float geometryMismatch = 0.0f;
    float rawGeometryMismatch = 0.0f;
    float correctionAbsoluteFlux = 0.0f;
    int correctedFaces = 0;
};

// 이류 단계만의 가중 dual MAC face 운동량 계측 (rho=1, depth=1m).
// 부분 개방 면은 open fraction을 유효 질량에 적용한다.
// 이류 경계의 내부 flux는 양쪽에서 상쇄되지만 압력·이동 경계와
// 전체 유체 질량/운동량의 보존까지 의미하지 않는다.
struct fluidAdvectionStats2
{
    vec2 before{};
    vec2 after{};
    float maxCfl = 0.0f;
    int substeps = 0;
    bool conservative = false;
    bool cflLimited = false;

    [[nodiscard]] vec2 delta() const noexcept { return after - before; }
};

// 양수 수동 tracer의 보존형 유량 수송. 질량은 셀에 보관하며 외곽 유량은 0.
// trappedMass는 현재 solid가 차지한 셀에 남아 있는 질량: 물리적인 displaced-fluid remap이 아님.
struct fluidTracerStats2
{
    float beforeMass = 0.0f;
    float afterMass = 0.0f;
    float trappedMass = 0.0f;
    // 이번 Step에서 줄어든 유체 점유 면적에 대해 displaced mass를 재배치한 양.
    // Swept 면의 시간 적분 부피 × 출발 셀 농도로 제안된 이송량(제한 전).
    float sweptRequestedMass = 0.0f;
    // 출발 셀의 감소 체적으로 이송량을 제한한 양.
    float sweptLimitedMass = 0.0f;
    float sweptRemappedMass = 0.0f;
    float fallbackRemappedMass = 0.0f;
    float unresolvedDisplacementMass = 0.0f;
    bool usedSweptRemap = false;
    float maxCfl = 0.0f;
    int substeps = 0;
    bool cflLimited = false;

    [[nodiscard]] float delta() const noexcept { return afterMass - beforeMass; }
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

    // 이동 중인 solid의 공유 MAC 면 swept volume을 주변 cell의 기하 ΔV와
    // 국소적으로 맞추는 실험 경로. 부호가 반대인 인접 잔차만 상쇄한다.
    // 기본 false. 닫힌 외벽의 유량 및 전체 체적 오차를 임의로 제거하지 않는다.
    void setSweptGeometryCorrectionEnabled( bool enabled ) noexcept { sweptGeometryCorrectionEnabled_ = enabled; }
    [[nodiscard]] bool isSweptGeometryCorrectionEnabled() const noexcept { return sweptGeometryCorrectionEnabled_; }

    // MAC dual control volume의 upwind flux-form 이류. 선택형이며 기본 false.
    // 고정된 현재 open fraction에 대한 face 운동량만 보존한다.
    void setConservativeAdvectionEnabled( bool enabled ) noexcept { conservativeAdvectionEnabled_ = enabled; }
    [[nodiscard]] bool isConservativeAdvectionEnabled() const noexcept { return conservativeAdvectionEnabled_; }

    // 셀의 tracer 밀도(kg/m³, unit depth)를 지정. 음수/비유한 값과 범위 밖은 무시.
    // 일반 이류/압력 Solver를 바꾸지 않으며, 보존형 tracer 수송은 활성화된 뒤 매 Step 동작.
    void setTracerConcentration( int x, int y, float concentration ) noexcept;

    // 선택형 moving solid remap. 유효한 swept 경로에서만 수동 tracer 질량을
    // 열린 이웃 또는 가까운 유체 셀로 옮김. 기본 false (Part 12a 호환).
    void setSweptTracerRemapEnabled( bool enabled ) noexcept { sweptTracerRemapEnabled_ = enabled; }
    [[nodiscard]] bool isSweptTracerRemapEnabled() const noexcept { return sweptTracerRemapEnabled_; }

    // 지정 위치 주변 face 속도에 world-space 속도 변화(m/s)를 더함.
    // 소스가 격자 바깥이거나 radius가 유효하지 않으면 아무 일도 하지 않음.
    void addVelocity( vec2 position, vec2 velocityDelta, float radius );

    // rho = 1kg/m³, depth = 1m, face 유효 질량 = cellSize² kg.
    // 원형 support의 fluid-fluid 내부 face에 각 성분 충격량을 정규화해서 배분.
    // 실제 배분한 충격량을 돌려주며, 유효 face가 없는 성분은 0 (반력 없음).
    // 압력 투영/이류/경계 외력까지 포함한 총 운동량 보존을 뜻하지 않음.
    [[nodiscard]] vec2 applyMomentumImpulse( vec2 position, vec2 impulse, float radius );

    // 선택한 Semi-Lagrangian 또는 보존형 upwind advection 후 Jacobi pressure projection.
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
    [[nodiscard]] const fluidSweptFaceFlux2& getSweptFaceFlux() const noexcept { return sweptFaceFlux_; }
    [[nodiscard]] const fluidAdvectionStats2& getAdvectionStats() const noexcept { return advectionStats_; }
    [[nodiscard]] float getTracerConcentration( int x, int y ) const noexcept;
    [[nodiscard]] float getTracerMass() const noexcept;
    [[nodiscard]] const fluidTracerStats2& getTracerStats() const noexcept { return tracerStats_; }
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
    [[nodiscard]] float advectionUOpen( int x, int y ) const noexcept;
    [[nodiscard]] float advectionVOpen( int x, int y ) const noexcept;
    [[nodiscard]] vec2 getAdvectionFaceMomentum() const noexcept;
    void advectConservative( float timeStep );
    [[nodiscard]] float tracerCellFraction( int x, int y ) const noexcept;
    void transportTracer( float timeStep );
    void remapTracerSwept();
    [[nodiscard]] float cutCellUOpen( int x, int y ) const noexcept;
    [[nodiscard]] float cutCellVOpen( int x, int y ) const noexcept;
    [[nodiscard]] float cutCellFluxDivergenceRms( float inverseTimeStep ) const noexcept;
    void prepareCutCellVolumeChange( float timeStep );
    void updateSweptSolidFlux( float timeStep, std::span<const fluidShapeObstacle2> obstacles );
    void reconcileSweptSolidFlux();
    void applyCutCellBoundaries() noexcept;
    void projectCutCell( float timeStep, int pressureIterations,
        std::span<const fluidShapeObstacle2> obstacles, std::span<fluidObstacleForce2> feedback );
    void applySolidBoundaries( std::span<const fluidShapeObstacle2> obstacles,
        std::span<const fluidObstacleForce2> pressureFeedback = {}, float timeStep = 0.0f );
    [[nodiscard]] bool solveCoupledPressure( float timeStep, int iterations,
        std::span<const fluidShapeObstacle2> obstacles );
    void project( float timeStep, int pressureIterations, std::span<const fluidShapeObstacle2> obstacles,
        std::span<fluidObstacleForce2> feedback );
#pragma endregion Helpers

#pragma region Data
    // 압력 연립 반복의 작업 버퍼는 Field 크기에 묶어 재사용한다.
    // Cell/Body 수가 일정한 시뮬레이션에서 매 투영마다 heap 할당하지 않는다.
    struct coupledPressureFace
    {
        int cell = 0;
        int owner = 0;
        double fx = 0.0;
        double fy = 0.0;
        double torque = 0.0;
    };
    struct coupledBodyResponse
    {
        double fx = 0.0;
        double fy = 0.0;
        double torque = 0.0;
    };
    struct coupledPressureScratch
    {
        std::vector<coupledPressureFace> faces;
        std::vector<coupledBodyResponse> response;
        std::vector<double> diagonal;
        std::vector<double> rhs;
        std::vector<double> x;
        std::vector<double> residual;
        std::vector<double> z;
        std::vector<double> direction;
        std::vector<double> product;
    };

    int width_ = 0;
    int height_ = 0;
    float cellSize_ = 1.0f;
    vec2 origin_{};
    std::vector<float> u_;
    std::vector<float> v_;
    std::vector<float> nextU_;
    std::vector<float> nextV_;
    std::vector<double> tracerMass_;
    std::vector<double> nextTracerMass_;
    std::vector<double> tracerOutgoing_;
    std::vector<double> tracerUFlux_;
    std::vector<double> tracerVFlux_;
    std::vector<float> pressure_;
    coupledPressureScratch coupledPressureScratch_{};
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
    std::vector<float> sweptUVolume_;
    std::vector<float> sweptVVolume_;
    std::vector<fluidShapeObstacle2> previousObstacles_;
    std::vector<int> cutCellComponent_;
    std::vector<int> cutCellNearestFluid_;
    std::vector<int> cutCellQueue_;
    int solidCellCount_ = 0;
    fluidSolidTransition2 solidTransition_{};
    fluidCutCellTransition2 cutCellTransition_{};
    fluidCutCellPressure2 cutCellPressure_{};
    fluidSweptFaceFlux2 sweptFaceFlux_{};
    fluidAdvectionStats2 advectionStats_{};
    fluidTracerStats2 tracerStats_{};
    bool tracerEnabled_ = false;
    bool sweptTracerRemapEnabled_ = false;
    bool sweptGeometryCorrectionEnabled_ = false;
    bool cutCellProjectionEnabled_ = false;
    bool conservativeAdvectionEnabled_ = false;
    bool hasCutCellHistory_ = false;
    bool volumeSourceValid_ = false;
    fluidPressureMomentum2 pressureMomentum_{};
    fluidStepMomentum2 stepMomentum_{};
#pragma endregion Data
};

} // namespace zonai
