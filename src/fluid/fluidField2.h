#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <utility>
#include <span>
#include <vector>

#include "collision/shapeGeometry.h"
#include "math/vec2.h"

namespace zonai
{

struct analyticDisjointCache2;

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

// Pressure 전후 자유 fluid-fluid MAC face와 강체 예측속도의 운동 에너지(J).
// Solid 경계/외벽 및 누락된 interface dual volume은 포함하지 않는 진단용 부분 장부.
// 양의 delta가 곧 전체 FSI 에너지 생성이라는 뜻은 아니다.
struct fluidPressureEnergy2
{
    double fluidBefore = 0.0;
    double fluidAfter = 0.0;
    double rigidBefore = 0.0;
    double rigidAfter = 0.0;

    [[nodiscard]] double fluidDelta() const noexcept { return fluidAfter - fluidBefore; }
    [[nodiscard]] double rigidDelta() const noexcept { return rigidAfter - rigidBefore; }
    [[nodiscard]] double totalDelta() const noexcept { return fluidDelta() + rigidDelta(); }
};

// 하나의 coupled Schur A*p=b solve 안에서 소비된 CPU wall-clock(ms) 분해.
// `setCutCellPressureTimingEnabled(true)`인 경우에만 유효하며 기본 OFF.
// assembly에는 matrix-free 연산자에 필요한 diagonal, RHS 및 face clipping을 포함.
// multiply는 PCG 반복 중 A*v 횟수만, validation은 별도 실제 b-A*x 재검증.
struct fluidCoupledPressureTiming2
{
    double total = 0.0;
    double assembly = 0.0;
    double factorization = 0.0;
    double multiply = 0.0;
    double preconditioner = 0.0;
    // Diagnostic subsets of preconditioner (not additional accounted phases).
    // Nonzero only for the split active-cell IC(0) path with timing enabled.
    double ic0Forward = 0.0;
    double ic0Backward = 0.0;
    // Opt-in previous-pressure probe: summed ||b||², ||b-A*p_previous||².
    // These are diagnostics, not extra phases in accounted().
    double warmStartRhsSquared = 0.0;
    double warmStartGuessSquared = 0.0;
    int warmStartProbes = 0;
    // Diagnostic counters for the opt-in guarded actual warm start.
    int warmStartAttempts = 0;
    int warmStartAccepted = 0;
    double vector = 0.0;
    double validation = 0.0;
    double output = 0.0;
    int matrixProducts = 0;
    int preconditionerApplications = 0;
    int cachedLaplacianEdges = 0;

    [[nodiscard]] double accounted() const noexcept
    {
        return assembly + factorization + multiply + preconditioner + vector + validation + output;
    }

    void accumulate( const fluidCoupledPressureTiming2& next ) noexcept
    {
        total += next.total;
        assembly += next.assembly;
        factorization += next.factorization;
        multiply += next.multiply;
        preconditioner += next.preconditioner;
        ic0Forward += next.ic0Forward;
        ic0Backward += next.ic0Backward;
        warmStartRhsSquared += next.warmStartRhsSquared;
        warmStartGuessSquared += next.warmStartGuessSquared;
        warmStartProbes += next.warmStartProbes;
        warmStartAttempts += next.warmStartAttempts;
        warmStartAccepted += next.warmStartAccepted;
        vector += next.vector;
        validation += next.validation;
        output += next.output;
        matrixProducts += next.matrixProducts;
        preconditionerApplications += next.preconditionerApplications;
        cachedLaplacianEdges += next.cachedLaplacianEdges;
    }
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
    fluidPressureEnergy2 energy{};
    bool usedCoupledProjection = false;
    bool coupledSolveFailed = false;
    fluidCoupledPressureStatus2 coupledStatus = fluidCoupledPressureStatus2::notRequested;
    int coupledIterations = 0;
    int coupledIterationLimit = 0;
    // 압력 연립식 A*p=b의 보정된 RHS 기준 RMS, 상대 잔차. Fluid 발산 RMS와는 별개.
    float coupledInitialResidualRms = 0.0f;
    float coupledFinalResidualRms = 0.0f;
    float coupledRelativeResidual = 0.0f;
    bool usedIncompleteCholesky = false;
    fluidCoupledPressureTiming2 coupledTiming{};
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
    // 분리된 다중 Shape 또는 엄격히 포함된 두 Circle의 분석적 경로.
    bool usedDisjointAnalyticGeometry = false;
    int disjointAnalyticShapes = 0;
    int containedAnalyticShapes = 0;
    bool usedPartialCircleAnalyticUnion = false;

    [[nodiscard]] float volumeDelta() const noexcept { return fluidVolume - previousFluidVolume; }
};

// 분석적 Cut-cell 연립 압력 Step에서만 측정한 이산 충격량(N·s).
// fluid: 개방 MAC face 속도 보정 × 열린 dual-face 면적, solid: Body 반력 × dt,
// wall: 닫힌 외벽의 전체 face 압력 반력. 이 셋의 합은 이산 압력 exchange 잔차.
// 이류/drag/기하 이동·고체 표면의 정확한 연속 적분을 포함하지 않음.
struct fluidCutCellPressureImpulse2
{
    vec2 fluid{};
    vec2 solid{};
    vec2 wall{};

    [[nodiscard]] vec2 residual() const noexcept { return fluid + solid + wall; }
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
    // 선택적 Body 질량 결합 PCG는 이진 MAC 장부와 별도 상태로 보고한다.
    bool usedCoupledProjection = false;
    bool coupledSolveFailed = false;
    fluidCoupledPressureStatus2 coupledStatus = fluidCoupledPressureStatus2::notRequested;
    int coupledIterations = 0;
    float coupledRelativeResidual = 0.0f;
    bool usedIncompleteCholesky = false;
    // Experimental Galerkin pressure pooling P^T (L + B M^-1 B^T) P.
    // A disappearing fluid cell shares pressure with a connected larger cell.
    // Volume-source RHS is summed (not discarded); individual sliver
    // continuity can remain nonzero even when the group constraint converges.
    int mergedPressureCells = 0;
    int unmergedSmallPressureCells = 0;
    float mergedFluidFraction = 0.0f;
    // Snapshot decrease in fluid area for merged shrinking cells (m^3,
    // unit depth). Not swept transfer or actual tracer mass movement.
    float mergedSnapshotDecreaseVolume = 0.0f;
    // RMS of grouped flux+moving-volume continuity (including Body response);
    // each merged group is one conservation row, NOT exact child-cell continuity.
    // Valid only when mergedPressureCells > 0 and coupled PCG has converged.
    float groupedPredictedContinuityRms = 0.0f;
    // L1 volume hypothetically exchanged within pooled cells to satisfy
    // group-only continuity (m^3 with unit depth, one Fluid substep).
    // This is NOT actual tracer remap or guaranteed pointwise conservation.
    float virtualInternalExchangeVolume = 0.0f;
    // opt-in 측정: Schur PCG 호출에 소비된 wall clock 시간(ms). 전체 Field Step은 아님.
    double coupledSolveMilliseconds = 0.0;
    fluidCoupledPressureTiming2 coupledTiming{};
    // 수렴한 Cut-cell PCG의 Body 예측속도를 다시 적용한 solid-normal MAC face.
    // 부분 개방 face는 blocked 구간만 갱신하며 fluid face 속도는 보존한다.
    int synchronizedBoundaryFaces = 0;
    int synchronizedPartialFaces = 0;
    float boundaryNormalChangeRms = 0.0f;
    // 압력으로 예측한 추가 고체 face flux까지 포함한 국소 연속방정식 잔차.
    // 분리된 pocket의 호환성 보정 후 값이며 전체 기하 보존을 뜻하지 않는다.
    float predictedContinuityRms = 0.0f;
    float predictedSolidVolumeL1 = 0.0f;
    // 다중 분석적 Cut-cell의 Schur 결합 압력이 실제 수렴했을 때만 유효.
    // 기존 이진 MAC getPressureMomentum()과 별도 측정이며 전체 FSI 보존값은 아님.
    bool hasAnalyticImpulseBudget = false;
    fluidCutCellPressureImpulse2 impulse{};
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
    // 선택형 시간 적응 표본: refining 전 L1 오차와 최종 채택한 시간 표본 수.
    // 0은 적응 검사 미실행. geometryMismatch는 이후 국소 면 보정까지 반영한다.
    int initialTimeSamples = 0;
    int refinements = 0;
    float initialGeometryMismatch = 0.0f;
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

// Diagnostic accounting for displaced tracer that the swept face + fallback
// remapper did not move in a single Fluid substep. All masses are kg for unit
// depth. Each unresolved source is counted in exactly one bin in EACH of the
// three mutually exclusive pairs (partial/solid, connected/disconnected,
// no-outward-sweep/insufficient-sweep). The pairs are different explanations,
// not additional mass to sum together.
struct fluidSweptTracerUnresolvedAudit2
{
    double totalMass = 0.0;
    double partialCellMass = 0.0;
    double fullyCoveredCellMass = 0.0;
    double connectedFluidFaceMass = 0.0;
    double noConnectedFluidFaceMass = 0.0;
    double noUsableOutwardSweepMass = 0.0;
    double insufficientOutwardSweepMass = 0.0;
    int sources = 0;

    // Largest unresolved source, ordered West, East, South, North.
    int worstX = -1;
    int worstY = -1;
    double worstUnmatchedMass = 0.0;
    double worstDisplacedMass = 0.0;
    double worstSweptTransferMass = 0.0;
    float worstPreviousFraction = 0.0f;
    float worstCurrentFraction = 0.0f;
    std::array<float, 4> worstFaceOpen{};
    std::array<float, 4> worstNeighborFraction{};
    std::array<float, 4> worstNeighborGain{};
    // Signed swept volume (m^3), outward-positive from the source;
    // includes blocked/closed destinations BEFORE remapper filtering.
    std::array<double, 4> worstRawOutwardSweptVolume{};
    // The exact MAC tracer transport, AFTER remap and AFTER the usual
    // donor-outflow limiter, accumulated across every tracer CFL substep.
    // W/E/S/N positive means mass left this source cell (kg).
    std::array<double, 4> worstMacOutwardTracerMass{};
    // Nominal open-fluid volume through each MAC face (m^3/unit depth):
    // velocity * open aperture * h * dt. Not mass and not limited.
    std::array<double, 4> worstMacOutwardFluidVolume{};
    // This is the *applied* swept remap from the worst donor, by W/E/S/N.
    // Values exclude directionally blocked or inward signed swept faces.
    std::array<double, 4> worstSweptOutwardTracerMass{};
    // Snapshot in that source before/after ALL simultaneous swept transfers.
    // The difference also includes incoming swept remaps from OTHER sources.
    double worstMassBeforeSweptRemap = 0.0;
    double worstMassAfterSweptRemap = 0.0;
    double worstMassBeforeMacAdvection = 0.0;
    double worstMassAfterMacAdvection = 0.0;
};

// 양수 수동 tracer의 보존형 유량 수송. 질량은 셀에 보관하며 외곽 유량은 0.
// trappedMass는 현재 solid가 차지한 셀에 남아 있는 질량: 물리적인 displaced-fluid remap이 아님.
// Diagnostic callback boundaries for the two conservative tracer operators.
// Every callback observes the actual stored per-cell mass; no snapshot copy
// occurs in the engine when the observer is absent.
enum class fluidTracerStage2
{
    afterSweptRemap,
    afterMacAdvection
};

// Actual, applied tracer transfer (kg, unit depth), not a requested swept
// volume or an unresolved displacement. Cell IDs use y * width + x.
// Swept/MAC face transfers are between neighbors; fallback can jump across
// fully covered cells. Every record subtracts mass from one cell and adds it
// to another, so the sum of records is independently conservative.
enum class fluidTracerTransferKind2
{
    sweptFace,
    fallback,
    macFace,
    connectedResidual // Research only: endpoint-open neighboring face.
};

struct fluidTracerTransfer2
{
    fluidTracerTransferKind2 kind = fluidTracerTransferKind2::sweptFace;
    int sourceCell = -1;
    int destinationCell = -1;
    double mass = 0.0;
};

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
    // Experimental-only aperture-weighted transfer of remaining displaced
    // tracer across currently open adjacent faces. Not a swept volume flux.
    float connectedResidualRemappedMass = 0.0f;
    int connectedResidualSources = 0;
    float unresolvedDisplacementMass = 0.0f;
    fluidSweptTracerUnresolvedAudit2 unresolvedAudit{};
    bool usedSweptRemap = false;
    float maxCfl = 0.0f;
    int substeps = 0;
    bool cflLimited = false;

    [[nodiscard]] float delta() const noexcept { return afterMass - beforeMass; }
};

// Side-effect-free snapshot of the current MAC face state. Not a stored Step result.
// For Cut-cell, continuity includes the latest swept-volume source; the
// predicted post-pressure solid flux is a separate diagnostic.
// Read-only Cut-cell volume ledger for one current Fluid substep (unit depth).
// Signed volume is outward-positive from (x,y), in m^3.
// MAC uses the *current/endpoint* face open fraction and velocity times dt.
// It is NOT the actual time-integral of a changing space-time aperture.
// The pressure source may use swept volume and relocation from solid cells;
// therefore snapshot, swept and pressure-source residuals are distinct.
struct fluidCutCellVolumeBudget2
{
    bool valid = false;
    bool hasVolumeHistory = false;
    bool hasSweptHistory = false;
    bool active = false;
    float previousFraction = 0.0f;
    float currentFraction = 0.0f;
    std::array<double, 4> macOutwardVolume{}; // W/E/S/N, m^3
    std::array<double, 4> solidSweptOutwardVolume{}; // W/E/S/N, m^3
    double snapshotDeltaVolume = 0.0;
    double pressureSourceDeltaVolume = 0.0;
    double solidSweptDivergenceVolume = 0.0;
    double macNetOutwardVolume = 0.0;
    double pressureResidualVolume = 0.0; // MAC + solver source
    double snapshotResidualVolume = 0.0; // MAC + geometric delta
    double sweptResidualVolume = 0.0; // MAC + signed solid swept divergence
};

// O(grid) opt-in summary; does not run or allocate in the normal Step.
// RMS metrics count active fluid cells only; totalSnapshotDeltaVolume and
// modifiedCellCount account for fully covered cells as well.
struct fluidCutCellVolumeSummary2
{
    bool valid = false;
    int activeCells = 0;
    int modifiedCells = 0;
    int modifiedFullyCoveredCells = 0;
    double totalSnapshotDeltaVolume = 0.0;
    double totalPressureSourceDeltaVolume = 0.0;
    double totalMacNetOutwardVolume = 0.0;
    double totalAbsSweptSnapshotMismatchVolume = 0.0;
    double activePressureResidualRmsVolume = 0.0;
    double activeSnapshotResidualRmsVolume = 0.0;
    double maxAbsPressureResidualVolume = 0.0;
    double maxAbsSnapshotResidualVolume = 0.0;
    int worstPressureX = -1;
    int worstPressureY = -1;
    int worstSnapshotX = -1;
    int worstSnapshotY = -1;
};

struct fluidFlowAudit2
{
    float continuityRms = 0.0f;     // 1/s
    float maxFluidFaceSpeed = 0.0f; // m/s, open fluid faces only
    float maxOpenWeightedFaceSpeed = 0.0f; // m/s, |velocity| * open fraction
    float maxFaceVelocity = 0.0f;   // signed face component, m/s
    float maxFaceOpenFraction = 0.0f;
    int maxFaceX = -1;
    int maxFaceY = -1;
    bool maxFaceIsU = true;
    int nonFiniteFluidFaces = 0;
};

// A pressure row local to one fluid cell; the Schur term includes every
// moving Body response affecting that row. Values only exist for an opt-in
// stage trace; no solver values are modified.
struct fluidFaceCellPressureAudit2
{
    float westOpen = 0.0f;
    float eastOpen = 0.0f;
    float southOpen = 0.0f;
    float northOpen = 0.0f;
    double rhs = 0.0;
    double laplacian = 0.0;
    double schur = 0.0;
    double rowResidual = 0.0;
    double finalOpenFlux = 0.0; // Net open face flux in face-speed units.
    double rawContinuity = 0.0; // Includes swept volume rate, not region mean.
};

// Selected staggered face velocity at successive Fluid stages.
// Read-only and opt-in: this describes one face, not a global maximum.
// Before geometry uses the previous snapshot; all later values use the
// current face opening. A blocked face can contain solid boundary velocity.
struct fluidFaceStageAudit2
{
    bool valid = false;
    bool isU = true;
    int x = -1;
    int y = -1;
    float openFraction = 0.0f;
    float beforeGeometry = 0.0f;
    float afterRemap = 0.0f;
    float afterBoundary = 0.0f;
    float afterAdvection = 0.0f;
    float afterPressure = 0.0f;
    // The two cells sharing the monitored face. These are sampled after the
    // pressure solve and never change pressure or the fluid boundary.
    float lowerCellFraction = 0.0f;
    float upperCellFraction = 0.0f;
    float lowerVolumeRate = 0.0f;
    float upperVolumeRate = 0.0f;
    float lowerPressure = 0.0f;
    float upperPressure = 0.0f;
    float predictedPressureDelta = 0.0f;
    fluidFaceCellPressureAudit2 lowerRow{};
    fluidFaceCellPressureAudit2 upperRow{};
};

// 마지막 유효 Fluid Step의 경과시간 분해(ms), 계측 옵션을 켠 경우에만 유효.
// wall-clock 계측이며 World::Step 및 World에서 전달한 drag/힘 처리 시간을 포함하지 않는다.
struct fluidStepTiming2
{
    double total = 0.0;
    double occupancy = 0.0;    // Body 중심점 마스크
    double geometry = 0.0;     // Cut-cell 면적 및 face 개방율
    double swept = 0.0;        // 이동 표면 swept + 선택한 오차 보정
    double remap = 0.0;        // 새로 열린 MAC face 재구성
    double boundaries = 0.0;   // 이류 전/후 경계 적용 및 속도 진단
    double advection = 0.0;    // 유체 속도 이류
    double pressure = 0.0;     // 투영 전체(PCG 또는 Jacobi), 반력 및 경계 예측
    double tracer = 0.0;       // 추적 물질 수송
    double other = 0.0;        // 원래 관측량 및 이전 장애물 snapshot 저장

    [[nodiscard]] double accounted() const noexcept
    {
        return occupancy + geometry + swept + remap + boundaries +
            advection + pressure + tracer + other;
    }

    void accumulate( const fluidStepTiming2& next ) noexcept
    {
        total += next.total;
        occupancy += next.occupancy;
        geometry += next.geometry;
        swept += next.swept;
        remap += next.remap;
        boundaries += next.boundaries;
        advection += next.advection;
        pressure += next.pressure;
        tracer += next.tracer;
        other += next.other;
    }
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

    // 선택형 단계별 wall-clock profiler. 기본 OFF, 성능 측정 중에만 켠다.
    void setStepTimingEnabled( bool enabled ) noexcept
    {
        stepTimingEnabled_ = enabled;
        if( !enabled ) stepTiming_ = {};
    }
    [[nodiscard]] bool isStepTimingEnabled() const noexcept { return stepTimingEnabled_; }

    // Opt-in read-only tracer-stage observer. Its field reference is valid
    // only during this call. Snapshot/copy the queried masses if persistence
    // is needed; callback exceptions propagate like other observers.
    using tracerStageObserver2 = std::function<void(const fluidField2&, fluidTracerStage2)>;
    void setTracerStageObserver( tracerStageObserver2 observer )
    {
        tracerStageObserver_ = std::move( observer );
    }

    // Opt-in per-transfer read-only ledger for Swept, fallback and limited
    // MAC advection. Disabled by default, no storage/allocation when unused.
    // Records correspond to applied transfers; unresolved material stays in
    // its source cell and emits no transfer record. Observer must not reenter
    // or mutate the field while it is stepping.
    using tracerTransferObserver2 = std::function<void(const fluidTracerTransfer2&)>;
    void setTracerTransferObserver( tracerTransferObserver2 observer )
    {
        tracerTransferObserver_ = std::move( observer );
    }

    // S0 diagnostic only. Called after advection/boundaries, immediately
    // before Cut-cell projection. The caller can COPY this field to freeze
    // every MAC face, geometry/swept history, warm-start pressure and cache.
    // A snapshot contains its own immutable copy of the obstacle inputs.
    // No observer means no snapshot allocation or pressure replay overhead.
    using prePressureSnapshotObserver2 = std::function<void(const fluidField2&)>;
    void setPrePressureSnapshotObserver( prePressureSnapshotObserver2 observer )
    {
        prePressureSnapshotObserver_ = std::move( observer );
    }

    // Executes only Cut-cell pressure on a COPY made inside the observer.
    // One replay per copy; returns nullopt for an ordinary/later Field state.
    // -1 uses the captured normal Step iteration budget. [1,200] replays
    // the original pressure budget on a frozen COPY; (200,3200] is
    // diagnostic-only, NONCOUPLED Jacobi reference research. Higher budgets
    // with a movable inverse-mass/inertia obstacle are rejected atomically.
    // No change to production Step(), advection or solver defaults.
    // finishStepForRestart is strictly diagnostic: also performs tracer
    // transport and commits the current obstacles as the NEXT step's
    // moving-geometry history on this copy. Default false preserves the
    // original pressure-only replay semantics and all existing callers.
    // Pressure and feedback remain observable on the copy after replay.
    // Optional [0,1] Jacobi relaxation is a frozen COPY-ONLY diagnostic:
    // (0,1] is accepted, 1 keeps the original undamped algorithm.
    // A nonunit value is rejected for any movable coupled obstacle.
    [[nodiscard]] std::optional<std::vector<fluidObstacleForce2>>
        replayFrozenCutCellPressure( int pressureIterationsOverride = -1,
            bool finishStepForRestart = false, float jacobiRelaxation = 1.0f );

    // Only callable on a pre-projection frozen COPY (before its one replay).
    // Replaces OPEN interior U/V face speeds with the same continuous
    // world-space velocity function on any grid. Closed faces retain their
    // prescribed moving-solid boundary speeds. Invalid samples never
    // partially change velocities. This is an explicit synthetic
    // counterfactual, not a physically unique prolongation/restriction.
    // Use endpoint MAC budgets, not the original step-entry linear proxy,
    // after this override: the recorded entry velocity history is unchanged.
    [[nodiscard]] bool setFrozenMacVelocityForAudit(
        const std::function<vec2(vec2)>& worldVelocity );

    // Opt-in single interior MAC-face stage trace; negative coordinates disable.
    // Does not change fluid velocities, projection, or geometry.
    void setFaceStageAuditTarget( int x, int y, bool isU ) noexcept
    {
        faceStageAuditEnabled_ = isU ? x > 0 && x < width_ && y >= 0 && y < height_
            : x >= 0 && x < width_ && y > 0 && y < height_;
        faceStageAuditX_ = x;
        faceStageAuditY_ = y;
        faceStageAuditIsU_ = isU;
        faceStageAudit_ = {};
    }

    // 장애물 수/구성이나 이산 기하 규칙이 바뀌면 첫 새 snapshot을
    // 이전 체적과 차분하지 않는다. 유체 속도·Tracer는 초기화하지 않음.
    void invalidateObstacleHistory() noexcept;

    // 실험 모드: sub-cell 개방 비율의 유량과 가중 압력 Poisson 사용.
    // 기본은 기존 이진 solid 압력 solver. 위치 snapshot의 체적 변화율만 근사하며
    // 정확한 swept-volume 경로/보존형 이류는 아직 구현하지 않음.
    void setCutCellProjectionEnabled( bool enabled ) noexcept { cutCellProjectionEnabled_ = enabled; }
    [[nodiscard]] bool isCutCellProjectionEnabled() const noexcept { return cutCellProjectionEnabled_; }

    // 셀 내부 NxN 및 공유 MAC face N개를 모두 같은 공간 표본으로 계산한다.
    // 지원 값은 4(기본) 또는 8. 변경 시 이전 형상/체적 이력을 무효화하여
    // 표본 해상도 차이를 실제 물체 이동으로 계산하지 않는다.
    void setCutCellSampleCount( int count ) noexcept;
    [[nodiscard]] int getCutCellSampleCount() const noexcept { return cutCellSampleCount_; }
    // 선택형 단일 Circle 전용 분석적 셀 면적/공유 MAC 면 클리핑.
    // 다중 장애물, Capsule, Polygon은 기존 공간 표본 경로를 유지한다.
    // 모드 변경 시 이산 점유율 차이를 swept 이동으로 오해하지 않도록 이력 초기화.
    void setAnalyticCircleCutCellEnabled( bool enabled ) noexcept;
    [[nodiscard]] bool isAnalyticCircleCutCellEnabled() const noexcept { return analyticCircleCutCellEnabled_; }

    // 단일 sharp convex Polygon(radius=0)에서 면적과 공유 면을 직접 clip.
    // Circle 설정과 독립적이며 기본 OFF. Polygon 외 형상은 공간 표본 경로.
    void setAnalyticPolygonCutCellEnabled( bool enabled ) noexcept;
    [[nodiscard]] bool isAnalyticPolygonCutCellEnabled() const noexcept { return analyticPolygonCutCellEnabled_; }

    // 단일 Capsule (회전 지원)를 중앙 strip + 양 끝 반원으로 분석적 적분.
    // 기본 OFF. 장애물 여러 개 또는 비정상 Capsule은 기존 공간 표본 사용.
    void setAnalyticCapsuleCutCellEnabled( bool enabled ) noexcept;
    [[nodiscard]] bool isAnalyticCapsuleCutCellEnabled() const noexcept { return analyticCapsuleCutCellEnabled_; }

    // 단일 radius>0인 strictly convex Polygon의 core/edge/corner 분석적 적분.
    // 기본 OFF. 다중·미지원 Shape은 기존 공간 표본 경로.
    void setAnalyticRoundedPolygonCutCellEnabled( bool enabled ) noexcept;
    [[nodiscard]] bool isAnalyticRoundedPolygonCutCellEnabled() const noexcept
    {
        return analyticRoundedPolygonCutCellEnabled_;
    }

    // 분리된 convex Shape, 두 Circle의 엄격한 포함/부분 교차에 분석적 union 계산.
    // 다른 실제 교차/접촉 근접/불확실 구성은 기존 표본 합집합 사용.
    void setDisjointAnalyticCutCellEnabled( bool enabled ) noexcept;
    [[nodiscard]] bool isDisjointAnalyticCutCellEnabled() const noexcept
    {
        return disjointAnalyticCutCellEnabled_;
    }

    // 실험적 Cut-cell/Body 유효질량 연립 PCG. 기본 OFF로 기존 Jacobi 경로 보존.
    // 수렴한 경우에만 Body 압력 반력을 돌려주며, 실패 시 해당 압력을 생략한다.
    void setCutCellCoupledProjectionEnabled( bool enabled ) noexcept { cutCellCoupledProjectionEnabled_ = enabled; }
    [[nodiscard]] bool isCutCellCoupledProjectionEnabled() const noexcept { return cutCellCoupledProjectionEnabled_; }

    // Experimental shrinking small-cell pressure pooling. Default 0 disables it.
    // Only active on valid moving-geometry history and a shrinking fluid
    // fraction. Stationary narrow gaps retain the original pressure cells.
    // Only used with coupled Cut-cell PCG: p = P*q, solve P^T*A*P with
    // A = L + B*M^-1*B^T, and preserve sum of moving-volume RHS per group.
    // Reject non-finite/negative/too-large thresholds. The inherited IC(0)
    // preconditioner is lifted/restricted with P to stay SPD.
    void setSmallCellPressureMergeFraction( float fraction ) noexcept
    {
        if( fraction >= 0.0f && fraction <= 0.05f )
            smallCellPressureMergeFraction_ = fraction;
    }
    [[nodiscard]] float getSmallCellPressureMergeFraction() const noexcept
    {
        return smallCellPressureMergeFraction_;
    }

    // 분석적 다중 Cut-cell Schur의 전처리기 비교용 설정. 기본 IC(0),
    // OFF는 기존 Jacobi. 원래 Schur 행렬/수렴 허용오차/기하 이력은 그대로 둔다.
    void setAnalyticCutCellIncompleteCholeskyEnabled( bool enabled ) noexcept
    {
        analyticCutCellIncompleteCholeskyEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellIncompleteCholeskyEnabled() const noexcept
    {
        return analyticCutCellIncompleteCholeskyEnabled_;
    }

    // IC(0) forward/backward sweep에서 열린 면 계수와 역피벗을 Step당 한 번 계산.
    // 기본 ON. OFF는 같은 Schur 연산자의 기존 매-적용 조회/나눗셈 기준 경로.
    // 장애물/격자 변화가 있는 각 coupled solve마다 새로 구축하고 프레임 간 재사용하지 않는다.
    void setAnalyticCutCellCachedIc0Enabled( bool enabled ) noexcept
    {
        analyticCutCellCachedIc0Enabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellCachedIc0Enabled() const noexcept
    {
        return analyticCutCellCachedIc0Enabled_;
    }

    // Cache IC(0) apply의 비활성 셀과 매-반복 x/y 경계 분기를 제거.
    // 기본 ON. OFF는 기존 격자 전체 순회 cached IC(0) 경로로 성능 비교.
    void setAnalyticCutCellActiveIc0SweepEnabled( bool enabled ) noexcept
    {
        analyticCutCellActiveIc0SweepEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellActiveIc0SweepEnabled() const noexcept
    {
        return analyticCutCellActiveIc0SweepEnabled_;
    }

    // Analytic Cut-cell의 PCG 내적과 벡터 갱신에서 활성 Fluid Cell만 순회.
    // Active IC(0)에서 이미 만든 row-major 셀 목록을 재사용한다. 기본 ON.
    // OFF는 격자 전체를 방문하는 기존 벡터 연산 참조 경로.
    void setAnalyticCutCellActivePcgVectorsEnabled( bool enabled ) noexcept
    {
        analyticCutCellActivePcgVectorsEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellActivePcgVectorsEnabled() const noexcept
    {
        return analyticCutCellActivePcgVectorsEnabled_;
    }

    // IC(0) active forward/backward sweep 계수를 두 소형 배열로 분리해
    // 각 순회에서 필요한 계수만 순차적으로 읽는다. 기본 ON.
    // OFF는 동일한 pivot과 수식을 쓰는 기존 AoS 캐시 참조 경로.
    void setAnalyticCutCellSplitIc0CoefficientsEnabled( bool enabled ) noexcept
    {
        analyticCutCellSplitIc0CoefficientsEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellSplitIc0CoefficientsEnabled() const noexcept
    {
        return analyticCutCellSplitIc0CoefficientsEnabled_;
    }

    // 분석적 Cut-cell Schur L의 MAC face 연결/개방율을 coupled solve마다 한 번 캐시.
    // 기본 ON. OFF는 기존 matrix-free 격자 face 조회 경로로 결과/속도 비교.
    // Body Schur B M^-1 B^T 및 이진 MAC 연산자는 변경하지 않는다.
    void setAnalyticCutCellCachedSchurEdgesEnabled( bool enabled ) noexcept
    {
        analyticCutCellCachedSchurEdgesEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellCachedSchurEdgesEnabled() const noexcept
    {
        return analyticCutCellCachedSchurEdgesEnabled_;
    }

    // Analytic Cut-cell geometry에서 막힌 MAC face만 Schur B 조립에 방문한다.
    // 기본 ON. OFF는 전체 내부 face를 다시 검사하는 참조 경로.
    void setAnalyticCutCellIndexedBoundaryFacesEnabled( bool enabled ) noexcept
    {
        analyticCutCellIndexedBoundaryFacesEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellIndexedBoundaryFacesEnabled() const noexcept
    {
        return analyticCutCellIndexedBoundaryFacesEnabled_;
    }

    // 압력 반력에 따른 고체 경계 속도 동기화 시, 이미 계산한 막힌 face만 방문.
    // 기본 ON. OFF는 모든 내부 face를 방문하는 참조 경로.
    void setAnalyticCutCellIndexedBoundarySyncEnabled( bool enabled ) noexcept
    {
        analyticCutCellIndexedBoundarySyncEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellIndexedBoundarySyncEnabled() const noexcept
    {
        return analyticCutCellIndexedBoundarySyncEnabled_;
    }

    // Analytic Cut-cell 셀 면적 계산은 Shape AABB와 교차하는 셀만 방문한다.
    // 기본 ON. OFF는 모든 셀에서 전체 Shape를 순회하는 참조 경로.
    void setAnalyticCutCellBoundedCellAreasEnabled( bool enabled ) noexcept
    {
        analyticCutCellBoundedCellAreasEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellBoundedCellAreasEnabled() const noexcept
    {
        return analyticCutCellBoundedCellAreasEnabled_;
    }

    // 서로 분리된 여러 Analytic Shape의 MAC face 교차는 AABB 범위에서만 계산.
    // 기본 ON. OFF는 face마다 모든 Shape를 확인하는 참조 경로.
    void setAnalyticCutCellBoundedFaceClippingEnabled( bool enabled ) noexcept
    {
        analyticCutCellBoundedFaceClippingEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellBoundedFaceClippingEnabled() const noexcept
    {
        return analyticCutCellBoundedFaceClippingEnabled_;
    }

    // 분리된 다중 Shape의 Schur B 조립은 움직일 수 있는 강체의 경계만 검사.
    // 정적 Shape는 M_body^-1=0이고 PCG 연산에 기여하지 않는다.
    // 기본 ON. OFF는 모든 Shape를 검사하는 참조 경로.
    void setAnalyticCutCellMovableSchurFacesEnabled( bool enabled ) noexcept
    {
        analyticCutCellMovableSchurFacesEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellMovableSchurFacesEnabled() const noexcept
    {
        return analyticCutCellMovableSchurFacesEnabled_;
    }

    // 현재 Fluid Step의 Analytic Shape 기하를 압력 행렬과 경계 예측에서 재사용.
    // Swept flux의 중간 시각 Shape는 별도 구성하며 재사용하지 않는다.
    // 기본 ON. OFF는 각 단계에서 재구축하는 참조 경로.
    void setAnalyticCutCellCurrentGeometryReuseEnabled( bool enabled ) noexcept
    {
        analyticCutCellCurrentGeometryReuseEnabled_ = enabled;
        if( !enabled ) analyticDisjointCache_.reset();
    }
    [[nodiscard]] bool isAnalyticCutCellCurrentGeometryReuseEnabled() const noexcept
    {
        return analyticCutCellCurrentGeometryReuseEnabled_;
    }

    // Analytic swept flux의 중간 시간 샘플은 실제 위치/회전이 움직인
    // Shape의 경계만 합산한다. 서로 분리된 다중 Shape에서만 적용.
    // 기본 ON. OFF는 정지 Shape를 포함한 원래 전체 face 조회 경로.
    void setAnalyticCutCellMovingSweptFacesEnabled( bool enabled ) noexcept
    {
        analyticCutCellMovingSweptFacesEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellMovingSweptFacesEnabled() const noexcept
    {
        return analyticCutCellMovingSweptFacesEnabled_;
    }

    // Moving-only analytic swept 적분에서 Shape AABB와 교차하는 MAC face만 clip.
    // 기본 ON. OFF는 모든 내부 face를 검사하는 참조 경로.
    void setAnalyticCutCellBoundedSweptFacesEnabled( bool enabled ) noexcept
    {
        analyticCutCellBoundedSweptFacesEnabled_ = enabled;
    }
    [[nodiscard]] bool isAnalyticCutCellBoundedSweptFacesEnabled() const noexcept
    {
        return analyticCutCellBoundedSweptFacesEnabled_;
    }

    // 벤치마크 전용 PCG 시간 측정. 기본 OFF로 일반 Step에 시계 측정 비용 없음.
    void setCutCellPressureTimingEnabled( bool enabled ) noexcept { cutCellPressureTimingEnabled_ = enabled; }
    [[nodiscard]] bool isCutCellPressureTimingEnabled() const noexcept { return cutCellPressureTimingEnabled_; }

    // 계측 전용: 이전 압력을 현재 연립 압력식에 대입한 실제 잔차 비교.
    // 기본 OFF. PCG 초기값과 물리 계산에는 사용하지 않는다.
    void setCoupledPressureWarmStartProbeEnabled( bool enabled ) noexcept
    {
        coupledPressureWarmStartProbeEnabled_ = enabled;
    }
    [[nodiscard]] bool isCoupledPressureWarmStartProbeEnabled() const noexcept
    {
        return coupledPressureWarmStartProbeEnabled_;
    }

    // 실험적 연립 Cut-cell PCG 이전 압력 Warm Start. 기본 OFF.
    // 현재 방정식의 잔차가 0 초기값 대비 충분히 작을 때만 채택하며,
    // 실제 b 기준 수렴 허용오차와 마지막 true residual 검증을 유지한다.
    void setCoupledPressureWarmStartEnabled( bool enabled ) noexcept
    {
        coupledPressureWarmStartEnabled_ = enabled;
    }
    [[nodiscard]] bool isCoupledPressureWarmStartEnabled() const noexcept
    {
        return coupledPressureWarmStartEnabled_;
    }

    // 이동 중인 solid의 공유 MAC 면 swept volume을 주변 cell의 기하 ΔV와
    // 국소적으로 맞추는 실험 경로. 부호가 반대인 인접 잔차만 상쇄한다.
    // 기본 false. 닫힌 외벽의 유량 및 전체 체적 오차를 임의로 제거하지 않는다.
    void setSweptGeometryCorrectionEnabled( bool enabled ) noexcept { sweptGeometryCorrectionEnabled_ = enabled; }
    [[nodiscard]] bool isSweptGeometryCorrectionEnabled() const noexcept { return sweptGeometryCorrectionEnabled_; }

    // 선택형 swept face 시간 표본 재계산: 셀 점유 변화 대비 L1 오차가 감소할 때만 채택.
    // 기본 OFF. 국소 면 flux 재배분과 독립이며 선택한 공간 표본 수는 유지한다.
    void setAdaptiveSweptSamplingEnabled( bool enabled ) noexcept { adaptiveSweptSamplingEnabled_ = enabled; }
    [[nodiscard]] bool isAdaptiveSweptSamplingEnabled() const noexcept { return adaptiveSweptSamplingEnabled_; }

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

    // S2c research ONLY, default OFF. If a *partially open* shrinking cell
    // cannot evacuate all displaced tracer through outward swept volumes,
    // distribute its remaining mass to adjacent fluid cells using positive
    // endpoint face apertures. Never cross a closed face or extend the swept
    // transfer limit; only tracer is changed, not pressure or MAC velocities.
    // Endpoint aperture is NOT a space-time fluid-flux reconstruction.
    // Fully covered sources retain the existing nearest-fluid fallback.
    void setConnectedResidualTracerRemapEnabled( bool enabled ) noexcept
    {
        connectedResidualTracerRemapEnabled_ = enabled;
    }
    [[nodiscard]] bool isConnectedResidualTracerRemapEnabled() const noexcept
    {
        return connectedResidualTracerRemapEnabled_;
    }

    // S2c read-only diagnostic. Midpoint quadrature of the moving obstacle's
    // actual *open face length* at the same selected time samples as Swept
    // solid flux. Separate from swept solid normal velocity and endpoint MAC.
    // OFF by default: no grid-sized arrays or extra clipping per normal Step.
    void setSweptApertureAuditEnabled( bool enabled ) noexcept
    {
        sweptApertureAuditEnabled_ = enabled;
        if( !enabled )
        {
            sweptApertureAuditValid_ = false;
            sweptUTimeMeanOpen_.clear();
            sweptVTimeMeanOpen_.clear();
            sweptUAlphaOpenMoment_.clear();
            sweptVAlphaOpenMoment_.clear();
            sweptUStartVelocity_.clear();
            sweptVStartVelocity_.clear();
        }
    }
    [[nodiscard]] bool isSweptApertureAuditEnabled() const noexcept
    {
        return sweptApertureAuditEnabled_;
    }
    [[nodiscard]] bool hasSweptApertureAudit() const noexcept
    {
        return sweptApertureAuditValid_;
    }

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
    // Explicit diagnostic: reads the current field, including changes after Step.
    // Not called on the normal simulation path; timeStep must be finite and positive.
    [[nodiscard]] fluidFlowAudit2 getFlowAudit( float timeStep ) const noexcept;
    // Endpoint (not time-centered) MAC-vs-geometry and pressure-source ledgers.
    // Invalid coordinates / nonpositive dt return valid=false.
    [[nodiscard]] fluidCutCellVolumeBudget2 getCutCellVolumeBudget(
        int x, int y, float timeStep ) const noexcept;
    [[nodiscard]] fluidCutCellVolumeSummary2 getCutCellVolumeSummary(
        float timeStep ) const noexcept;
    [[nodiscard]] const fluidFaceStageAudit2& getFaceStageAudit() const noexcept { return faceStageAudit_; }

    // 내부 fluid-fluid MAC face 속도를 dual-cell mass로 적분한 운동량.
    // solid 경계의 지정 속도 및 외곽 wall face는 제외한다.
    [[nodiscard]] vec2 getFluidMomentum() const noexcept;
    // fluid-fluid U/V MAC face에 각 0.5*h²*v²를 합산한 부분 운동 에너지.
    // Solid/moving boundary dual face volume은 포함하지 않는다.
    [[nodiscard]] double getFluidKineticEnergy() const noexcept;
    [[nodiscard]] const fluidPressureMomentum2& getPressureMomentum() const noexcept { return pressureMomentum_; }
    [[nodiscard]] const fluidStepMomentum2& getStepMomentum() const noexcept { return stepMomentum_; }
    [[nodiscard]] const fluidStepTiming2& getStepTiming() const noexcept { return stepTiming_; }
    [[nodiscard]] const fluidSolidTransition2& getSolidTransition() const noexcept { return solidTransition_; }
    [[nodiscard]] const fluidCutCellTransition2& getCutCellTransition() const noexcept { return cutCellTransition_; }
    [[nodiscard]] const fluidCutCellPressure2& getCutCellPressure() const noexcept { return cutCellPressure_; }
    [[nodiscard]] const fluidSweptFaceFlux2& getSweptFaceFlux() const noexcept { return sweptFaceFlux_; }
    [[nodiscard]] const fluidAdvectionStats2& getAdvectionStats() const noexcept { return advectionStats_; }
    [[nodiscard]] float getTracerConcentration( int x, int y ) const noexcept;
    [[nodiscard]] float getTracerMass() const noexcept;
    // Raw stored cell mass (kg, unit depth), including tracer trapped in
    // fully covered cells where concentration queries return zero.
    // Diagnostic only: does not normalize by current fluid fraction.
    [[nodiscard]] double getTracerCellMass( int x, int y ) const noexcept;
    [[nodiscard]] const fluidTracerStats2& getTracerStats() const noexcept { return tracerStats_; }
    // 기하 표본의 유체 면적 비율 [0,1]. 범위 밖은 0.
    // 외곽 wall face는 기하상 열려있어도 닫힌 경계이므로 0.
    [[nodiscard]] float getCellFluidFraction( int x, int y ) const noexcept;
    [[nodiscard]] float getUFaceOpenFraction( int x, int y ) const noexcept;
    [[nodiscard]] float getVFaceOpenFraction( int x, int y ) const noexcept;
    // Time-average OPEN fraction on one interior face over the most recent
    // valid swept moving-geometry interval. Zero if unavailable/invalid.
    // Approximate quadrature, not an exact space-time interface integral.
    [[nodiscard]] float getSweptUTimeMeanOpenFraction( int x, int y ) const noexcept;
    [[nodiscard]] float getSweptVTimeMeanOpenFraction( int x, int y ) const noexcept;
    // S2c DIAGNOSTIC, not a measured time-dependent velocity field. Computes
    // h*dt*∫_0^1 open(alpha)*lerp(uAtStepEntry, uAfterPressure, alpha)dalpha.
    // The two velocity endpoints are known but their intermediate history is
    // not. Only valid during/after the latest step's pressure+tracer stage.
    // Invalid/missing time samples return zero. Signed +U/+V canonical face.
    [[nodiscard]] double getSweptULinearVelocityProxyVolume(
        int x, int y, float timeStep ) const noexcept;
    [[nodiscard]] double getSweptVLinearVelocityProxyVolume(
        int x, int y, float timeStep ) const noexcept;
    // Sub-face에서 막힌 구간의 평균 solid 법선 속도. 부분 개방 U/V의 fluid 성분과 별개.
    [[nodiscard]] float getUFaceSolidVelocity( int x, int y ) const noexcept;
    [[nodiscard]] float getVFaceSolidVelocity( int x, int y ) const noexcept;
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
    void synchronizeCutCellBoundaries( float timeStep, std::span<const fluidShapeObstacle2> obstacles,
        std::span<const fluidObstacleForce2> feedback );
    void projectCutCell( float timeStep, int pressureIterations,
        std::span<const fluidShapeObstacle2> obstacles, std::span<fluidObstacleForce2> feedback,
        float jacobiRelaxation = 1.0f );
    void applySolidBoundaries( std::span<const fluidShapeObstacle2> obstacles,
        std::span<const fluidObstacleForce2> pressureFeedback = {}, float timeStep = 0.0f );
    [[nodiscard]] bool solveCoupledPressure( float timeStep, int iterations,
        std::span<const fluidShapeObstacle2> obstacles, bool cutCell = false );
    void project( float timeStep, int pressureIterations, std::span<const fluidShapeObstacle2> obstacles,
        std::span<fluidObstacleForce2> feedback );
#pragma endregion Helpers

#pragma region Data
    // 압력 연립 반복의 작업 버퍼는 Field 크기에 묶어 재사용한다.
    // Cell/Body 수가 일정한 시뮬레이션에서 매 투영마다 heap 할당하지 않는다.
    struct analyticFaceAccumulator
    {
        double blockedLength = 0.0;
        double velocitySum = 0.0;
    };
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
    struct incompleteCachedCoefficients
    {
        double west = 0.0;
        double south = 0.0;
        double east = 0.0;
        double north = 0.0;
        double westOverPivot = 0.0;
        double southOverPivot = 0.0;
        double inversePivot = 0.0;
    };
    struct incompleteForwardCoefficients
    {
        double westOverPivot = 0.0;
        double southOverPivot = 0.0;
    };
    struct incompleteBackwardCoefficients
    {
        double east = 0.0;
        double north = 0.0;
        double inversePivot = 0.0;
    };
    struct coupledPressureEdge
    {
        int first = 0;
        int second = 0;
        double weight = 0.0;
    };
    struct coupledPressureScratch
    {
        std::vector<coupledPressureFace> faces;
        std::vector<coupledPressureEdge> laplacianEdges;
        std::vector<coupledBodyResponse> response;
        std::vector<int> dynamicResponseOwners;
        std::vector<double> diagonal;
        std::vector<double> incompletePivot;
        std::vector<double> incompleteForward;
        std::vector<incompleteCachedCoefficients> incompleteCached;
        std::vector<incompleteForwardCoefficients> incompleteSplitForward;
        std::vector<incompleteBackwardCoefficients> incompleteSplitBackward;
        std::vector<int> incompleteActiveCells;
        std::vector<int> smallCellPressureRoots;
        std::vector<double> smallCellLiftedResidual;
        std::vector<double> smallCellReducedPreconditioner;
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
    // Reused double accumulation preserves per-cell analytic shape summation order.
    std::vector<double> analyticCoveredAreaScratch_;
    std::vector<float> uOpenFraction_;
    std::vector<float> vOpenFraction_;
    // Exact positive blocked lengths from this step's analytic geometry pass.
    // U/V entries retain row-major face traversal order for Schur assembly.
    std::vector<int> analyticUBlockedFaces_;
    std::vector<int> analyticVBlockedFaces_;
    // Snapshot immutable between geometry update, Schur solve and boundary sync.
    // Opaque here to keep analytic polygon/capsule internals in the .cpp file.
    std::shared_ptr<const analyticDisjointCache2> analyticDisjointCache_;
    std::vector<analyticFaceAccumulator> analyticUFaceScratch_;
    std::vector<analyticFaceAccumulator> analyticVFaceScratch_;
    std::vector<float> uSolidVelocity_;
    std::vector<float> vSolidVelocity_;
    // 압력 반력으로 예상된 추가 blocked-face swept flux (m/s, blocked fraction 포함).
    std::vector<float> pressureUSolidFlux_;
    std::vector<float> pressureVSolidFlux_;
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
    tracerStageObserver2 tracerStageObserver_{};
    tracerTransferObserver2 tracerTransferObserver_{};
    std::vector<float> sweptUTimeMeanOpen_;
    std::vector<float> sweptVTimeMeanOpen_;
    // Temporal first moments ∫alpha*open(alpha)dalpha, plus actual MAC
    // face velocity at entry to this Fluid step. No normal-step allocations.
    std::vector<float> sweptUAlphaOpenMoment_;
    std::vector<float> sweptVAlphaOpenMoment_;
    std::vector<float> sweptUStartVelocity_;
    std::vector<float> sweptVStartVelocity_;
    bool sweptApertureAuditEnabled_ = false;
    bool sweptApertureAuditValid_ = false;
    bool tracerEnabled_ = false;
    bool sweptTracerRemapEnabled_ = false;
    bool connectedResidualTracerRemapEnabled_ = false;
    bool sweptGeometryCorrectionEnabled_ = false;
    bool adaptiveSweptSamplingEnabled_ = false;
    int cutCellSampleCount_ = 4;
    bool analyticCircleCutCellEnabled_ = false;
    bool analyticPolygonCutCellEnabled_ = false;
    bool analyticCapsuleCutCellEnabled_ = false;
    bool analyticRoundedPolygonCutCellEnabled_ = false;
    bool disjointAnalyticCutCellEnabled_ = false;
    bool previousDisjointAnalyticGeometry_ = false;
    int previousContainedCircleIndex_ = -1;
    bool previousPartialCircleOverlap_ = false;
    bool cutCellProjectionEnabled_ = false;
    bool cutCellCoupledProjectionEnabled_ = false;
    float smallCellPressureMergeFraction_ = 0.0f;
    bool analyticCutCellIncompleteCholeskyEnabled_ = true;
    bool analyticCutCellCachedIc0Enabled_ = true;
    bool analyticCutCellActiveIc0SweepEnabled_ = true;
    bool analyticCutCellActivePcgVectorsEnabled_ = true;
    bool analyticCutCellSplitIc0CoefficientsEnabled_ = true;
    bool analyticCutCellCachedSchurEdgesEnabled_ = true;
    bool analyticCutCellIndexedBoundaryFacesEnabled_ = true;
    bool analyticCutCellIndexedBoundarySyncEnabled_ = true;
    bool analyticCutCellBoundedCellAreasEnabled_ = true;
    bool analyticCutCellBoundedFaceClippingEnabled_ = true;
    bool analyticCutCellMovableSchurFacesEnabled_ = true;
    bool analyticCutCellCurrentGeometryReuseEnabled_ = true;
    bool analyticCutCellMovingSweptFacesEnabled_ = true;
    bool analyticCutCellBoundedSweptFacesEnabled_ = true;
    bool cutCellPressureTimingEnabled_ = false;
    bool coupledPressureWarmStartProbeEnabled_ = false;
    bool coupledPressureWarmStartEnabled_ = false;
    bool conservativeAdvectionEnabled_ = false;
    bool hasCutCellHistory_ = false;
    bool volumeSourceValid_ = false;
    fluidPressureMomentum2 pressureMomentum_{};
    fluidStepMomentum2 stepMomentum_{};
    fluidStepTiming2 stepTiming_{};
    bool stepTimingEnabled_ = false;
    // Diagnostics must remain OFF in regular simulation.
    // Diagnostic callback never runs unless explicitly installed. Only the
    // observer's copied Field retains a one-use frozen projection context.
    prePressureSnapshotObserver2 prePressureSnapshotObserver_{};
    std::vector<fluidShapeObstacle2> frozenPressureObstacles_;
    float frozenPressureTimeStep_ = 0.0f;
    int frozenPressureIterations_ = 0;
    bool frozenPressureReplayReady_ = false;
    bool faceStageAuditEnabled_ = false;
    bool faceStageAuditIsU_ = true;
    int faceStageAuditX_ = -1;
    int faceStageAuditY_ = -1;
    fluidFaceStageAudit2 faceStageAudit_{};
#pragma endregion Data
};

} // namespace zonai
