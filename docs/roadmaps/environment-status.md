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
- **S0 1차 완료:** [PR #319](https://github.com/buddas-bible/zonai/pull/319) merged, master SHA **5842f8c58b3e3fbceff34747c9d5b5464db62f90**, Ubuntu 112/112, Windows Debug 115/115, Release 52/52 passed (CI 42571477840). Baseline flow/Drag diagnostics and matched cadence CSV landed.
- **실측 발견:** Windows Release h=0.25m, World 8/Fluid 4, TwoWay ON/Drag 0의 측정 frame 6에서 열린 MAC face 최대 속도 **28.13385m/s** 단일 spike. 앞뒤 frame 0.595/0.712m/s; PCG **수렴**, iterations 61 (보통 46~47), 상대 잔차 8.20e-6, Body 속도 연속. 즉 Drag 주입이 **같은 프레임의** 해당 급등 원인은 아님. 확정 원인은 미상.
- **S0 2차 완료:** [PR #320](https://github.com/buddas-bible/zonai/pull/320) merged, master **1fc3be3cfd4d31c2d4435719131bc13d58fd08e1**, Ubuntu 112/112, Windows Debug 115/115, Release 52/52 통과(CI 42585749190). 문제 face는 V(x=17,y=13), Fluid substep=3, signed velocity=-28.13385m/s, open fraction=**0.00006395081**(0.0064%), 해당 면의 |velocity × open| ≈ 0.0018m/s. 별도로 전체 field의 최대 open-weighted face speed는 0.6536318m/s. 이는 국소 거의 닫힌 Cut-cell 현상이지 전역 유체 속도 급등으로 단정할 수 없음.
- **S0 3차 진행 중:** [PR #321](https://github.com/buddas-bible/zonai/pull/321), `test/fluid-aperture-sampling-contamination` Draft, CI **42587916950**. 기존 `sampleVelocity`/Semi-Lagrangian의 raw MAC 속도 보간이 거의 닫힌 face에서 이웃 실제 유체 셀 샘플에 얼마나 전파되는지 읽기 전용으로 측정. 이번 PR은 기존 물리 모델 변경 없음.

## 1. 단계별 상태

| 단계 | 상태 | 메모 |
| --- | --- | --- |
| S0 재현·진단 baseline | **1·2차 병합, 3차 PR #321 검증 중** | 거의 닫힌 V면(0.0064%)의 raw 보간 전파 여부 조사 |
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
- PR #319 (1차 계측) **병합**: World 8, Fluid 1/4, h=0.5/0.25, coupling ON/OFF, drag=0/3 (source ON), Windows Release 프레임별 CSV. 기본 물리 계산과 스케줄은 변하지 않음.
- PR #320 **병합 완료**: 이상 face의 위치, opening, raw/weighted 속도 확인. V면(17,13) 개방율 0.00006395081.
- PR #321 **Draft**: peak face 및 실제 유체가 남은 이웃 셀에서 현재 bilinear sample값을 같은 Fluid substep에 로깅. 새로운 계산식 도입 전 외부 쿼리의 오염 여부 검증.
- **S0 미완료**: Fluid 2/8 및 static 많은 장면, 경계 걸침, moving cut-cell swept mismatch 연관 분석, 국소 face velocity/시각 tracer 관계, 사용자 장면 장기 재현 및 알고리즘 원인 수정.
- 새 대화의 첫 작업: PR #321 CI run **42587916950** Windows Release `--coupling-spike-audit`의 **h=0.25 / Fluid=4 / coupled=1 / drag=0 / frame=6** 결과를 확인. `peak_adjacent_cell_normal_speed`와 해당 cell fluid fraction을 분석해 raw face velocity가 실제 유체 영역 샘플에 전파되는지 판정. 그 뒤에만 opt-in aperture-aware sample 실험을 설계. Clamp로 숨기지 않는다.

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

zonai 환경 물리 개발을 이어가자. docs/roadmaps/environment-status.md 및 environment-simulation.md와 병합된 PR #319/#320, Draft PR #321을 코드·CI로 검증하라. PR #321 CI 42587916950의 Windows Release CSV에서 h=0.25/Fluid=4/TwoWay ON/Drag=0/frame 6의 peak face V(17,13) 개방율 0.00006395081 및 인접 실제 유체 셀의 `sampleVelocity` 결과를 분석하라. 현재 물리적 근본 원인은 미확정이므로 무작정 clamp 또는 속도 수정하지 말고 원인에 맞는 차등 테스트부터 진행하라. S0 전체 완료로 표시하지 말 것.
