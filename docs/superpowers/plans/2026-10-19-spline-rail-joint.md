# Editable Spline Rail Joint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox syntax for tracking.

**Goal:** Add a dynamic spline rail joint with prismatic-compatible controls, rotation modes, and editable path/rail demos.

**Architecture:** A dedicated joint participates in the existing World graph and impulse solver. Its owned local curve supplies a continuously tracked frame and arc coordinate; the normal and angular Jacobians are coupled, while spring/motor/limits use the arc Jacobian. Existing kinematic path motion remains a separate feature.

**Tech Stack:** C++20, existing CMake/CTest, existing spline geometry and constraint softness.

**Spec:** `docs/superpowers/specs/2026-10-19-spline-rail-joint-design.md` (approved).

## Global Constraints

- Open regular curves only; physical endpoint stops; no closed-loop extrapolation.
- Body A owns a copied local curve; Body B remains dynamic. No pose overwrites during physical solving/editing.
- Prismatic setting names, units, 0Hz spring-off and motor braking semantics remain.
- Preserve existing path behavior, existing code style/CRLF; no dependencies or unrelated reformatting.
- Full Debug and Release builds/tests plus independent final review are required.

## Review Focus

- Projection continuity at crossings and high speed: test previous-parameter tracking and finite failure.
- Curvature/offset-anchor coupling with dynamic A: test momentum response and angular following.
- Singular/near-singular tangents: reject edit atomically, skip invalid solver state without NaNs.
- Endpoints and explicit limits: test disjoint limit rejection and outward motor stopping.
- Editing gestures and stale history: test 10px gate, one gesture/Undo record, no physics mouse-joint fallback.

## Task 1: Curve frame and straight solver equivalence

**Files:** new `src/dynamics/joints/splineRailJoint2.h`, `splineRailJointSim2.h`, `splineRailJointConstraint2.h/.cpp`; new `tests/dynamics/spline_rail_joint_test.cpp`; `tests/CMakeLists.txt`.

**Interfaces:** spec public types; `prepareSplineRailJointConstraint`, `warmStartSplineRailJointConstraint`, `solveSplineRailJointConstraint` use existing bodySim/bodyState signatures. Joint geometry preparation validates regularity and prepares per-segment/prefix lengths once per edit.

- [x] Add runtime-failing stub and direct solver comparisons against prismatic on a long line: free, spring, 0Hz, motor/brake, limits/locked, off-center anchors, dynamic A and mass scales. Record RED.
- [x] Implement curve frame and dedicated impulse rows. On a line use the identical prismatic block/axis algebra, including softness and cache behavior. Record GREEN.
- [x] Add regularity/invalid-frame/finite endpoint-stop regressions and implement bounded failure handling.

## Task 2: World integration and lifecycle

**Files:** new `src/dynamics/splineRailJointWorld.cpp`; `src/dynamics/world.h/.cpp`; new `tests/dynamics/spline_rail_joint_world_test.cpp`; test registration.

**Interfaces:** exact create/set/query API in spec; `jointId` and Body graph remain unchanged. Sim stores owned points, prepared segment data, parameter, reference distance, tracking status and impulses.

- [x] RED: World construction/step snapshot and line parity, invalid points, setter atomicity.
- [x] Register variants and prepare/warm/solve/store/reset dispatch; implement create/set/query and connected-component waking. GREEN.
- [x] Test owned geometry, Body/joint deletion, stale and cross-world handles, slot reuse, collision filtering, sleep/wake, substep cache reset and unchanged state on invalid setters.

## Task 3: Curved tracking and rotation

**Files:** joint constraint implementation and both new tests.

**Interfaces:** local projected frame contains position, unit tangent/normal, signed curvature, parameter and signed arc translation. Default fixed-angle mode and optional tangent spring use the same stored public settings.

- [x] RED: curved rail gravity/motor movement and normal error, spring settling, crossing branch continuity, off-rail projection denominator failure.
- [x] Track adjacent segments from previous/velocity-predicted parameter; recompute frame from solver deltas. Include arc-coordinate and curvature-dependent angular Jacobians. GREEN.
- [x] RED/GREEN: fixed orientation; tangent-offset tracking/angle wrap; angular damping/hard 0Hz; dynamic A reaction, mode changes, endpoints, disjoint limits, high speeds and finite invalid-frame snapshots.

## Task 4: Editable path and rail demos

**Files:** `sandbox/demo.h/.cpp`, `rigidBodyDemo.h/.cpp`, `rigidBodyJointDemo.cpp`, `rigidBodyDemoUi.cpp`; spline-specific demo file only if it keeps these edits focused; existing sandbox tests and new rail demo tests.

**Interfaces:** editable per-instance path controls; new `demoKind::splineRail`. Share geometry editing/history with existing spline terrain without its trial diagnostics. New World geometry setter keeps handles/materials/velocities.

- [x] RED: existing path can drag controls and preserves progress ratio/reset behavior; add rail demo and dynamic-body expectations.
- [x] Connect shared drag, Ctrl+click within 10px, deletion to minimum2, Undo/Redo and help/drawing. GREEN.
- [x] RED/GREEN: invalid edit rejection; stable handles/mass/materials/selection/velocities; retained joint settings, cache reset/wake; zoom gate and no mouse-joint input conflict. Keep terrain/path regressions green.

## Task 5: Final verification

- [x] Run independent whole-branch review; fix actionable findings with regression-first tests.
- [x] Full `cmake --build build --config Debug --parallel` and `ctest --test-dir build -C Debug --output-on-failure`.
- [x] Full Release build and CTest with the same commands/configuration.
- [x] Scan compiler diagnostics, check diff/line endings and representative step costs; record actual counts and unresolved limitations.
- [x] Report uncommitted implementation unless user requests integration; no PR/push/master merge in this implementation turn.

## 검증 결과 (2026-10-19)

- 직선 prismatic 동등성, World 연결, 곡선 추적, 데모 편집, 실제 UI 조작 순서로 RED/GREEN을 확인했다.
- 독립 리뷰의 수치 overflow·큰 이동 투영·끝점 회전 Jacobian·질량 변경 후 교차 분기 문제를 실패 테스트로 재현하고 수정했다. 재검토에서 필수 수정 사항은 없었다.
- Debug·Release 전체 빌드와 CTest 각각 74/74 통과. 최종 테스트 시간은 Debug 9.95s, Release 4.97s. 두 빌드에서 warning/error 진단은 없었다.
- 중력 아래 곡선 이동/끝점 정지/스프링 복원, 고정·hard/soft 접선 회전, 동적 A의 운동량·토크 반작용과 공통 좌표 변환 불변성을 검사했다.
- 소유 제어점, 연결 충돌 필터, 수면·깨우기, 삭제·슬롯 세대·World 소유권, 내부 분할 변경, 교차점의 밀도 변경을 검사했다.
- 두 데모의 드래그·10px 삽입(20/200px 배율)·최소 2점 삭제·Undo/Redo·잘못된 편집의 기록 보존을 검사했다. 기존 경로 이동과 terrain 회귀 검사도 통과했다.
- Release 실행 비용: 4 substep의 World::Step 4,000회 × 5회에서 회당 평균의 중앙값은 prismatic 1.531μs, 곡선 rail 8.128μs(약 5.31배). 단일 조인트·물체 장면의 대표 측정이며 일반적인 성능 보장은 아니다. 측정 코드는 저장소 밖에 뒀다.
- 새 파일과 변경 구간의 스타일·CRLF를 확인했다. 기존 데이터 표 배치와 변경 밖 코드는 유지했다.
- ImGui headless 화면 조작·그리기 검사는 실행했으며, 실제 창에서의 수동 시각 검사는 하지 않았다. 조인트 계산 실패 시 impulse를 건너뛰고 trackingValid=false를 반환한다.
- 구현 변경은 미커밋 상태로 유지한다.
