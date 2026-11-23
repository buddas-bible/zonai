# R0 — zonai 정식 엔진 전환: 코드 감사와 결정론 기준선

> 작성: 2026-11-23 (Asia/Seoul)  
> 기준: \`master\` \`3f6dc69545d62d7c5a0ebf7ebe83a67454b8ea8d\`  
> 상태: **R0 1차 정적 코드·의존성 감사**. 변경·삭제 승인 문서가 아니다. 이 PR은 문서만 추가하고 엔진 동작, 기본값, 실험 브랜치를 변경하지 않는다.  
> 후속 작업은 실제 코드·사용처·테스트·현재 CI를 다시 확인한 뒤 작은 독립 PR로 진행한다.

## 1. 최상위 계약

zonai는 **실제 게임에 사용할 수 있는 안정적인 2D 물리엔진**을 목표로 한다. 정식 기능은 안정성·정확성·성능·유지보수성뿐 아니라, 지원되는 CPU/OS/컴파일러 조합에서 **플랫폼 간 비트 단위 결정론**까지 검증한다. 이는 유체 전용 목표가 아니라 Math, BroadPhase, Collision, Contact, Island, World, Joint, CCD, Fluid, 향후 Cloth/Terrain 전체에 적용한다.

- 동일한 엔진 버전, 초기 권위 상태, 입력 시퀀스, 시뮬레이션 설정 및 고정 Tick/서브스텝이면 각 Tick의 권위 상태와 이벤트가 비트 단위로 같아야 한다.
- 다른 \`dt\`, substep 설정, 물리 설정, 엔진 버전 간 동일 결과까지 요구하지 않는다. 서브스텝은 주로 정확도·안정성을 위한 장치이며 결정론의 대체재가 아니다.
- 결정론과 수치 안정성은 별도 검사한다. 같은 압력 폭주가 반복된다고 게임용으로 안정적인 것은 아니다.
- 공식 지원 플랫폼 범위는 **우선 Windows x64/MSVC + Linux x64/GCC·Clang**으로 제안하되, 컴파일러 버전·FP 정책·CPU 기능·실제 통과 여부를 기록하기 전에는 달성했다고 쓰지 않는다.
- GPU/렌더/ImGui/벽시계 프로파일러는 비권위적이어야 한다. GPU 결과를 물리 상태에 반영한다면 동일한 결정론 검증 대상이다.
- 결정론 복구를 이유로 구조를 전면 재작성하지 않는다. 먼저 최초 불일치 Tick/변수를 찾고 최소 수정한다.
- 임의의 pressure/speed clamp, 이력/source 누락, 힘/체적 보존 포기 같은 우회책은 안정화 완료로 인정하지 않는다.

참고: 기존 풍/물/지형의 S0–S9 로드맵은 [Draft PR #318](https://github.com/buddas-bible/zonai/pull/318)에만 있고 \`master\`에 아직 병합되지 않았다. R0–R8은 **코드 정리 실행 트랙**이며 기존 S0–S9 기능 로드맵을 대체하지 않는다.

## 2. GitHub 상태 및 코드 기준선

2026-11-23 조회 결과:

| 항목 | 확인된 기준 |
| --- | --- |
| \`master\` | \`3f6dc69545d62d7c5a0ebf7ebe83a67454b8ea8d\` |
| 저장소 파일 | recursive Git tree 기준 407개 blob |
| \`src/fluid/fluidField2.cpp\` | 242,120 bytes — 유체·기하·압력·연성 구현 집중 |
| \`src/fluid/fluidField2.h\` | 48,759 bytes — 물리 옵션, 최적화 비교, 감사 API 공존 |
| \`src/dynamics/world.cpp\` / \`.h\` | 202,949 / 82,352 bytes |
| \`sandbox/rigidBodyDemo.cpp\` / \`rigidBodyDemoUi.cpp\` | 143,016 / 181,615 bytes |
| \`tests/fluid/fluid_field_test.cpp\` | 240,176 bytes — 종합 회귀 테스트 |
| 열린 [#322](https://github.com/buddas-bible/zonai/pull/322) | aperture-aware MAC sampling, Draft/미병합 (10 files) |
| 열린 [#326](https://github.com/buddas-bible/zonai/pull/326) | small Cut-cell pressure pooling + 진단, Draft/미병합 (15 files, 357 commits) |
| 열린 [#318](https://github.com/buddas-bible/zonai/pull/318) | S0–S9 로드맵 문서, Draft/미병합 |
| [#324](https://github.com/buddas-bible/zonai/issues/324) | disappearing Cut-cell 압력 스파이크 분석/수정 미완료 |
| [#327](https://github.com/buddas-bible/zonai/issues/327) | branch 정리 감사; 삭제 여부는 별도로 재확인 |

주요 코드·테스트 참조:

- [fluidField2.h](../../src/fluid/fluidField2.h), [fluidField2.cpp](../../src/fluid/fluidField2.cpp)
- [broadPhase.h](../../src/collision/broadphase/broadPhase.h), [rot2.h](../../src/math/rot2.h)
- [rigidBodyDemo.h](../../sandbox/rigidBodyDemo.h), [rigidBodyDemo.cpp](../../sandbox/rigidBodyDemo.cpp)
- [fluid_field_test.cpp](../../tests/fluid/fluid_field_test.cpp), [wind_step_benchmark.cpp](../../tests/sandbox/wind_step_benchmark.cpp)
- [cmake.yml](../../.github/workflows/cmake.yml), [coding-style.md](../coding-style.md)

**주의:** 이 표는 파일 크기와 표면적 API의 1차 감사이며, 실행 시간, 줄 수, 실제 코드 중복량, CI 성공 횟수나 삭제 가능한 코드 비율을 뜻하지 않는다.

## 3. 유지 / 통합 / 삭제 후보 등록부

| ID | 경로 / 실제 기호 | R0 분류 | 관찰 내용 및 후속 결정 조건 |
| --- | --- | --- | --- |
| K01 | \`src/dynamics/\`, \`src/collision/\`의 World·Contact·Joint·Island·CCD·DynamicTree | **유지·감사** | 핵심 알고리즘 재구현 금지. 내부 순회, 동률 선택, 이벤트/Impulse 캐시의 결정론을 각각 검증 |
| K02 | \`fluidField2\`의 MAC U/V, Pressure, Tracer, Cut-cell, swept flux, feedback | **유지·안정화** | 기반 기능 삭제 금지. 먼저 단방향/고정 장애물 경로를 검증하고 TwoWay는 별도 합격 |
| K03 | 관측 가능한 압력·체적·운동량·Tracer 수지 | **유지하되 계측 경계 정리** | 장기 안정성 판단에 필수. 벽시계/출력은 계산 결과를 변경하지 않아야 함 |
| C01 | \`fluidField2.h\`: \`setAnalyticCutCellCachedIc0Enabled\`, \`setAnalyticCutCellActiveIc0SweepEnabled\`, \`setAnalyticCutCellSplitIc0CoefficientsEnabled\` | **경로 통합 후보** | 참조 경로와 최적화 경로 비교 목적. 결과·성능·결정론을 증명한 뒤 정식 구현만 유지 |
| C02 | \`setAnalyticCutCellActivePcgVectorsEnabled\`, \`setAnalyticCutCellCachedSchurEdgesEnabled\` | **경로 통합 후보** | 내부 최적화 선택을 게임용 API에 계속 노출할 이유가 있는지 감사 |
| C03 | \`setAnalyticCutCellIndexedBoundaryFacesEnabled\`, \`setAnalyticCutCellIndexedBoundarySyncEnabled\`, \`setAnalyticCutCellBoundedCellAreasEnabled\`, \`setAnalyticCutCellBoundedFaceClippingEnabled\` | **경로 통합 후보** | 물리적으로 같아야 하는 참조/가속 경로. 먼저 Geometry 정확성·동률·계산 순서 대조 |
| C04 | \`setAnalyticCutCellMovableSchurFacesEnabled\`, \`setAnalyticCutCellCurrentGeometryReuseEnabled\`, \`setAnalyticCutCellMovingSweptFacesEnabled\`, \`setAnalyticCutCellBoundedSweptFacesEnabled\` | **경로 통합 후보** | 경계/이력/캐시 무효화와 연관. 무조건 삭제 금지 |
| C05 | \`setAnalyticCutCellIncompleteCholeskyEnabled\`, \`setCoupledPressureWarmStartEnabled\`, Jacobi 및 Schur PCG | **모델/솔버 정책 보류** | 단순 benchmark toggle과 달리 수렴 및 실패 특성이 다름. S2 안정화 전 일괄 단일화 금지 |
| C06 | \`setCutCellProjectionEnabled\`, Shape별 analytic enable, \`setDisjointAnalyticCutCellEnabled\`, \`setSweptGeometryCorrectionEnabled\`, \`setAdaptiveSweptSamplingEnabled\`, \`setConservativeAdvectionEnabled\`, \`setSweptTracerRemapEnabled\` | **물리 모델 정책 분류** | 기능 자체/물리 모델 선택일 수 있으므로 최적화용 토글과 구분. 지원 시나리오와 기본값 확정 전 삭제 금지 |
| D01 | \`broadPhase.h\`: \`pairSet_std_\`와 \`#include <unordered_set>\` | **저위험 제거 후보** | 실제 Pair 추적은 \`hashSet pairSet_\` 사용. 검색된 \`pairSet_std_\` 참조는 선언·감사 문서뿐. 독립 bench \`tests/collision/broadphase/hash_set_pair_benchmark.cpp\` 보존 조건으로 전체 참조 검사 후 제거 |
| D02 | \`fluidField2::setCoupledPressureWarmStartProbeEnabled\` | **진단 전용 분리 후보** | 실제 Warm Start 설정과 단순 잠재 효율 probe를 구분해 public 게임 API 축소 |
| D03 | \`fluidField2::setPrePressureSnapshotObserver\`, \`replayFrozenCutCellPressure\`, \`setFaceStageAuditTarget\` | **테스트 전용 분리 후보** | 압력 A/B와 Stage 문제 재현에 필요. 기존 [#324](https://github.com/buddas-bible/zonai/issues/324) 원인 검증 전에는 제거 금지 |
| D04 | \`sandbox/rigidBodyDemo.h\`: \`setWindCachedIc0Enabled\`, \`setWindActiveIc0SweepEnabled\`, \`setWindCachedSchurEdgesEnabled\` 등 | **일반 UI에서 축소 후보** | 벤치마크 스위치와 사용자 Wind/Obstacle/Tracer 설정 분리. 진단 화면·CLI는 필요한 만큼 보존 |
| D05 | \`tests/fluid/fluid_field_test.cpp\`, Wind benchmark, \`.github/workflows/cmake.yml\` | **책임 분리 후보** | 비교용 benchmark와 필수 correctness/determinism 회귀를 분리. 기존 회귀 시나리오를 무검증 삭제 금지 |
| E01 | Draft #322의 aperture-aware sampler | **실험 격리** | sampler 값 감소만으로 실제 압력·강체·질량 안정성 증거가 되는 것은 아님 |
| E02 | Draft #326의 small-cell pressure merge, temporal isolated volume-source relay, tracer remap, tangent-face variants, audit oracle | **실험 격리** | 같은 상태에서 장기·보존·힘·Tracer·비트 결정론 검증 전 \`master\`에 일괄 병합 금지 |

### 삭제 허용 규칙

1. 게임용 필수 API인지, 내부 계산 최적화 비교인지, 계측인지 먼저 분류한다.
2. 참조/호출/테스트/문서/CLI/샌드박스/UI 사용처를 전수 확인한다.
3. 현재 계산 결과와 예산을 **삭제 전** 수집한다. 필요한 비교 경로는 독립 시험 장면 또는 branch/commit에 보존한다.
4. Pure cleanup PR에서는 물리 수식, 순회 순서, FP 정밀도, 기본값을 변경하지 않는다. 삭제 전후 동일 입력·상태의 비트 동일성 확인을 목표로 한다.
5. 안정화·알고리즘 변경 PR에서는 물리 결과 변경을 명시하고 수치·보존·성능·결정론을 새 기준으로 비교한다.
6. 실험 버그를 단순히 OFF 처리한 것을 정식 수치 해결로 선언하지 않는다.
7. 브랜치는 최신 머지베이스·patch-equivalence·참조를 다시 검사하기 전 삭제하지 않는다.

## 4. 결정론 감사 위험 지점

| 영역 | 확인한 근거 | 다음 감사 |
| --- | --- | --- |
| Math | \`rot2::FromRadians\`가 \`std::cos/std::sin\` 사용. World 및 여러 Joint에 \`std::atan2\` 호출 존재 | 정식 수학 API, 컴파일러 FP 정책, trig 경로의 Windows/Linux bitwise 검사. 렌더/테스트 전용 호출과 권위 물리 호출 구분 |
| BroadPhase | \`broadPhase::pairContext\`가 32개 후보 배치 후 callback 호출. \`FindPairs\`가 tree traversal 결과를 전달 | tree 동률·재빌드·ID 재사용·후보 순서·Contact 생성 순서·hashSet 사용 영향 감사 |
| World/Island/Joint | \`world::Step(dt, subStepCount)\`와 \`island2.cpp\`의 활성 Body 순회 존재 | graph 구축, Contact/Joint solver 순서, sleep/CCD, 각종 persistent Impulse와 이벤트 결정론 |
| Fluid | PCG IC(0) 활성 셀·캐시·이력·pressure/source/tracer 저장 상태 | Reduction 순서, 종료 조건, 불연속 topological change, geometry 및 source/feedback의 재현성 |
| Sandbox Scheduler | World/Fluid substep 분리, Wind demo의 Force/Drag/pressure feedback | 고정 Tick 입력과 명시적 Step 순서; FPS/벽시계가 권위 물리에 영향 주지 않는지 검사 |
| CI | Windows Debug, Ubuntu Debug, Windows Release 선택 회귀·정보성 benchmark | **현재는 플랫폼 간 물리 상태 해시 동일성 Gate 없음**. 새 CI job 설계 |

확인할 결합 실패: [#324](https://github.com/buddas-bible/zonai/issues/324)의 거의 닫힌 V(17,13) 면 \`-28.13385 m/s\` 및 #326 진단 기록의 moving Capsule U(26,4) \`-17.9502182 m/s\`. 재현성 여부와 수치적 안정성은 각각 독립 검사한다.

## 5. R1 결정론 Replay 테스트 계약(구현 전 합의)

**입력:** engine version, scene identifier, canonical configuration, fixed \`dt\`, world/fluid substeps, PRNG seed, \`tick -> input commands\` (생성/파괴/힘/정적 수정 순서 포함).

**권위 상태:** Body/Shape/Contact/Joint ID+generation, pose/velocity/sleep, broadphase/solver 결과에 영향을 주는 이력 및 cache, impulse/manifold, 모든 fluid U/V/pressure/tracer/occupancy/aperture/swept state, response/forces, 외부 이벤트와 다음 Tick에 영향을 주는 RNG. 무엇을 authoritative로 직렬화하는지 필드별 명세를 남긴다.

**직렬화:** 포인터/주소/패딩/벽시계/할당기 내부 주소를 해싱하지 않는다. canonical 순서+명시적 endian+필드별 float 원시 비트를 사용한다. NaN/Inf는 안정성 실패로 취급하고 비트 비교에서 숨기지 않는다. debug/observer/timing ON/OFF로 권위 상태가 달라지면 실패.

**비교:** 동일 바이너리 반복, 동일 OS Debug/Release (동일 엔진 계약 내 동등성이 필요한지 분리 표기), Windows vs Linux x64, 가능하면 동일 소스의 MSVC/GCC/Clang. 첫 상이한 \`tick/substep/subsystem/object/field\`와 양쪽 비트를 출력한다. 최종 hash만 일치해도 중간 Tick 불일치가 있으면 실패.

**초기 고정 장면:**

1. No-contact: 힘/중력/회전/적분, Body 생성·삭제·ID 재사용.
2. Contact: 다수 접촉, warm cache, restitution, friction, sleep/wake.
3. Joint: 다양한 Joint, 2개 이상 island, 순서 변경·제약 반복.
4. BroadPhase: 동일 AABB SAH/tie, moved/rebuild, pair lifecycle, ShapeCast/CCD.
5. Fluid: 빈 격자, 정적 장애물, Wind source, tracer 및 fluid substeps.
6. Fluid coupling: 단방향 ReceiveOnly → ObstacleOnly → 명시적 TwoWay, 복합 Shape.
7. Cut-cell: 극소 opening과 움직이는 Capsule의 first-bad-tick 테스트, conservation/solver failure를 결정론과 별개로 검사.

**첫 Gate:** 동일 바이너리에서 각 Tick의 정식 상태 해시 재현. 그 뒤 지원 플랫폼간 bitwise Gate를 확장한다. 실패한 환경을 통과로 표시하지 않는다.

## 6. 안정성·성능 기준선 수집

최초 R1/R2 전에 다음 조건을 고정한 CSV/결과 및 성능 측정 장면을 정한다.

- World only: Contact/CCD/Joint/Query lifecycle, 1/4/8 world substeps, 정지/고속/다중 island.
- Fluid only: \`h=0.5/0.25m\`, Fluid substeps \`1/2/4/8\`, tracer, static Shape, moving obstacles.
- Coupled: TwoWay ON/OFF, Drag 0/양수, 압력/체적/운동량/토크 장부, step/PCG status.
- CPU time: 동일 러너에서 p50/p95, 할당량, solver 반복; 서로 다른 CI runner의 절대 속도를 정식 regression gate에 바로 사용하지 않는다.
- 기존 \`tests/fluid/\`, \`tests/sandbox/wind_*\`, \`tests/dynamics/\`, \`tests/collision/\` 회귀를 기준선으로 유지.
- 현재 \`.github/workflows/cmake.yml\`의 Windows/Ubuntu Debug, Windows Release selective tests 및 정보성 benchmark는 유지한다. 실제 통과 여부는 각 PR 실행 결과를 따로 기록한다.

## 7. 분할 PR 계획 / 선행 조건

| 단계 | 예상 원자 PR | 안전 조건 |
| --- | --- | --- |
| **R0** | 이 문서: 삭제·보존·수정 등록부, 위험·장면·증거 계약 | 문서만; 코드·브랜치·기본값 무변경 |
| **R1a** | canonical authoritative state serializer / structured diff | 필드 소유권·정렬·float bits 명시 |
| **R1b** | 고정 입력 Replay와 golden scene runner | 동일 실행 파일 반복 및 최초 불일치 추적 |
| **R1c** | Win/Linux·컴파일러 상태 비교 CI | 동일 스키마 출력, 지원 플랫폼 matrix 기록 |
| **R2a** | deterministic FP/trig math 선택과 검증 | math 단위테스트 + World/Joint bitwise; 예상 물리 변경은 분리 |
| **R2b** | BroadPhase/Island/Contact/Joint ordering, tie, cache 필요 수정 | first-divergence 증거, broadphase/event regression |
| **R2c** | 제거 검증된 \`pairSet_std_\` 등 독립 저위험 dead state | 전체 검색·benchmark 분리·bitwise 회귀 |
| **R3** | \`fluidField2\` Geometry/Advection/Pressure/Coupling 책임 분리 | 수식·연산 순서 변경 없는 구조 PR, 동일 상태 비트 보존 |
| **R4** | IC0/PCG/cache/performance reference paths 각자 축소 | 명시적 winner, 비트/정확성/속도/결정론 검증 후 하나씩 제거 |
| **R5** | moving Cut-cell 및 TwoWay pressure/source 안정화 | #324/#326 증거 활용, conservation/long-run/bitwise/성능 통과 |
| **R6** | Sandbox Wind scheduler·Body policy·UI/진단 경계 정리 | 헤드리스 vs Sandbox 동일 Tick, 기존 UI 주요 기능 유지 |
| **R7** | correctness / deterministic / performance CI 및 테스트 파일 재편 | 테스트 수와 적용 범위 감소 금지; 벤치 정보성 유지 |
| **R8** | API/Docs/실험 브랜치 단계별 정리 | commit/issue 참조 보존, patch-equivalence 및 승인된 삭제만 |

R1은 R2의 전제이며, R3 구조 분할과 R4 경로 삭제는 별도 PR이다. R5 해결 전 #326 연구 구현을 \`master\`에 통째로 병합하지 않는다.

## 8. R0 완료/미완료 구분 및 다음 작업

**이 1차 감사에서 완료한 것**

- master + Draft #318/#322/#326의 상태를 비교하여 정식/실험의 경계를 기록.
- 실행 파일/헤더의 크기 및 실제 옵션 API·사용처 일부를 확인하고 후보를 개별 등록.
- 정식 계약과 후속 원자 PR의 입력·출력·회귀 Gate 정의.

**R0에서 후속 확인이 필요한 것**

- 후보별 전체 호출 그래프/외부 API 영향/실제 삭제 가능 라인 수.
- CI baseline 실행 결과 및 scene-specific golden record 수집.
- 모든 정식/실험 토글의 전체 조합 및 CMake Feature 정의 전수 조사.
- 다른 프로젝트가 \`zonai\` 공개 헤더를 사용하는 경우 호환 영향 조사.

**다음 구현:** R1a canonical snapshot schema + 최소 World 상태 직렬화/해시 테스트를 먼저 수행한다. Code cleanup 전 불변식 확보가 목적이며, 그 다음 R1b Replay → R1c 플랫폼 비교를 진행한다.

> 변경 검증 규칙: 이 문서-only PR의 CI 성공은 구현의 결정론이나 유체 안정성이 해결됐다는 증거가 아니다.
