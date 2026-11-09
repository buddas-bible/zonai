# zonai 환경 물리 로드맵 — 작업 상태 / 다음 대화 인수인계

> 우선 읽을 문서: [환경 물리 상세 사양](environment-simulation.md)
> **주의:** 여기의 상태는 기록 당시의 snapshot이다. 실제 GitHub master/열린 PR/CI와 비교해 확인한 후 작업한다. 아직 이 문서 자체는 제안안이며 코드 구현 승인이 아니다.

## 0. 상태 기록

- 상태 기록일: 2026-11-09 (Asia/Seoul)
- 확인한 master SHA: **d2d55482a0c07784cb9737d2cf82301b34543169** (PR #317 merged)
- 기준 CI: PR #317 run 42564740815; Ubuntu 112/112, Windows Debug 115/115, Windows Release 52/52 Passed.
- 최종 관련 PR: #315 previous-pressure probe merged; #316 guarded Warm Start trial merged; #317 accepted Warm Start World differential merged.
- Warm Start 기본값: **OFF**, 실험으로만 설정 가능. Warm 실패 후 같은 solve에서 Cold 재시도 경로는 **미구현**.
- 기능 개발 상태: 기존 MAC Wind + cut-cell/analytic Schur PCG + 다수 Joint/Ragdoll/자동차/Chain/Spline 데모 구현. Water 자유 수면, 부력/수면 항력, 침식/퇴적, Cloth/Soft Body, GPU Fluid는 이 로드맵상 **미착수**.
- 이번 로드맵 변경 범위: **문서만**. 코드, 기본 연성 옵션, 엔진 런타임/성능을 바꾸지 않는다.
- **진행 중:** [PR #319](https://github.com/buddas-bible/zonai/pull/319), `test/fluid-coupling-spike-audit`는 S0a의 **첫 계측 하위 작업**. Side-effect-free Cut-cell/Drag 진단, 고정 cadence 실험과 CSV, 회귀 테스트를 추가했으나 **Draft, 병합 전, 첫 CI 42571092635는 Windows 전용 단계의 Ubuntu 실행 설정 오류로 실패(물리 테스트 112/112 통과). 수정 CI **42571477840**는 Ubuntu 전체 성공, Windows 검증 중**. 상세 `docs/fluid-coupling-audit.md`는 PR #319 브랜치에서 확인.

## 1. 단계별 상태

| 단계 | 상태 | 메모 |
| --- | --- | --- |
| S0 재현·진단 baseline | **진행 중 — PR #319 Draft** | 먼저 동일 시간 분할 + Drag 전후 진단, 실제 튐 원인 분리는 후속 |
| S1 Body별 4종 Interaction | 미착수 | 정책/수명 + Static/Kinematic/Compound Body |
| S2 압력·Drag 수치 안정화 | 부분 기반 존재 | PCG/IC0/Warm Start 구현; Warm→Cold retry 미착수 |
| S3 Wind + 돛·Joint 실험 | 미착수 | 기존 일반 Joint와 Wind를 새 gameplay scene으로 연결 |
| S4 2D 수면·물살·부력 | 미착수 | Water 독립 최소 모델 필요 |
| S5 Wind + Water 뗏목 MVP | 미착수 | S3, S4의 합성 목표 |
| S6 지형·침식·퇴적 | 미착수 | 기존 지형 충돌/편집만 참고 |
| S7 Cloth/Soft Body 연성 | 미착수 | solver/particle 아직 없음 |
| S8 GPU prototype | 미착수 | CPU 측정·자료 분리부터 시작 |
| S9 대규모/3D | 장기 | 별도 로드맵 필요 |

## 2. 다음 원자 PR — S0a (우선 추천)

### 제목
test/bench(fluid): isolate wind two-way coupling spikes at fixed step cadence

### 진행 상황과 범위 경계
- PR #319은 **1차 계측**: World 8, Fluid 1/4, h=0.5/0.25, coupling ON/OFF, drag=0/3 (source ON), Windows Release 프레임별 CSV. 현재 물리 계산·기본 스케줄은 변경하지 않음.
- 미완료: Fluid 2/8, Static 다수/경계에 걸친 장애물, Cut-cell swept mismatch 연관 분석, 압력 단계별 최대 속도 변화, 사용자 관찰한 이상 프레임의 실제 재현, 원인별 독립 해결 PR. PR #319이 통과하더라도 **S0 전체 완료로 표시하지 말 것**.
- 다음 대화는 먼저 PR #319 수정 CI run **42571477840**의 테스트/Windows CSV를 확인하여 실패·스파이크를 분석하고 PR을 수정/유지/병합 판단한다.

### 이번 PR에서 할 일
1. 기존 Wind Sandbox World/Fluid의 호출 횟수와 입력을 동일하게 고정한 reference와 experiment scene을 구성.
2. 동일한 World=8, Fluid=1/2/4/8 조건에서 **경계 포함, 반력 적용, Drag 적용, 시간 스케줄**을 분리해 비교. 필요하면 새 실험용 private 옵션을 추가하되 public 기본 옵션은 변경하지 않는다.
3. 각 frame에서 max fluid velocity change, coupledStatus, residual, coupledIterations, pressure/drag force+torque, dynamic body pose/velocity, cut-cell geometry/swept mismatch를 진단 CSV에 수집.
4. 물체의 이동/회전·질량·Static Obstacle 수와 h=0.5/0.25 조건을 재현 가능한 소규모 시나리오로 고정.
5. 튐의 최초 프레임과 연관된 solver/geometry/force/visual 값을 구분해서 기록. 재현하지 못해도 조사 범위와 음성 결과를 남긴다.

### 이번 PR에서 하지 말 것
- 유체 수치해법을 근거 없이 교체하지 않는다.
- 임의로 힘이나 속도를 clamp하지 않는다.
- 모든 Body를 TwoWay로 켜거나 기본 모드를 바꾸지 않는다.
- 새 수면/뗏목/GPU/침식 구현을 묶지 않는다.

### 합격 기준
- 기존 테스트/Windows·Ubuntu Debug/Windows Release CI를 통과.
- ON/OFF 비교 중 다른 변수를 한꺼번에 바꾸지 않는 재현 실험.
- 진단 값은 side-effect 없는 opt-in이며 OFF일 때 physics가 일치.
- 결과가 비정상이면 고친 척 하지 않고 로그 및 단계 S2의 구체적 수정 질문으로 넘김.

### 다음 후보
- S2a Warm Start 실패 시 Cold PCG retry (위 진단과 별개로 독립 검증 가능).
- S1a per-Body Interaction policy와 ID 수명.
- S1b compound Shape aggregation.

## 3. 새 대화에서 반드시 지킬 확인 절차

1. GitHub 최신 master SHA, 관련 열린 PR·CI 확인.
2. [상세 로드맵](environment-simulation.md)과 본 상태 파일을 읽고 오랫동안 갱신되지 않은 정보가 없는지 확인.
3. src/fluid/fluidField2.h/.cpp, sandbox/rigidBodyDemo.h/.cpp, tests/fluid, tests/sandbox의 현행 구현과 비교.
4. **구현 후보 하나**를 골라 범위·변경 파일·불변식·실험·합격 조건을 정의.
5. 수정/회귀/CI/PR을 진행하되 안정성/기본 동작을 무검증 변경하지 않음.
6. 결과를 본 파일의 완료 항목, 커밋, CI, blocker, 다음 작업에 즉시 반영.

## 4. 결정과 미결정 상태

- 로드맵 v0.1: **사용자 검토용 제안**. 코드 변경을 승인한 것으로 취급하지 않는다.
- Body별 모드(Disabled, ReceiveOnly, ObstacleOnly, TwoWay), CPU gameplay + opt-in GPU 시각 디테일, Wind/Water 분리는 첫 방향 제안.
- 미결정: 정확한 Body registry 위치; air/water per-body 정책 구성; Drag/pressure 에너지 장부; GPU gameplay authoritative 시점; 프레임 예산; 2D vs 2.5D erosion 공유.
- 실제 구현 결과가 로드맵과 다르면 '완료'로 억지 표기하지 말고 문서의 결정/불변식과 구현 범위를 갱신한다.

## 5. 새 대화 시작 문장

zonai 환경 물리 개발을 이어가자. docs/roadmaps/environment-status.md와 environment-simulation.md 및 진행 중인 PR #319를 읽고 최신 master/CI(run 42571092635) 상태를 확인해. PR #319의 Windows Release --coupling-spike-audit CSV와 테스트 결과를 먼저 분석하고, 실패를 수정하거나 원인 후보를 분리해. PR #319는 진단 첫 하위 작업일 뿐 S0 전체 완료는 아니다. 검증 결과와 다음 원자 작업을 상태 문서에 기록해.
