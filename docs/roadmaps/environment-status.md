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
- **S0 3차 완료:** [PR #321](https://github.com/buddas-bible/zonai/pull/321) merged, master SHA **21046fc26fd247dc1292547d0d8a807e3e12f8e8**, Ubuntu 112/112, Windows Debug 115/115, Release 52/52 통과(CI **42588105495**). 문제 V(17,13) 면 원시 속도 **-28.13385m/s**가 인접 **유체 점유율 0.7617487**인 셀의 legacy `sampleVelocity` normal component **14.17676m/s**로 전달됨을 실측. 면 유량 관점의 `velocity * open`는 -0.001799183m/s. 즉 작은 aperture의 국소 원시속도가 raw bilinear 샘플링에 혼합되는 것은 확인됨. 이 값만으로 실제 Body 힘 폭주/전체 tracer 불안정은 입증되지 않음.
- **S0 4차 진행 중:** [PR #322](https://github.com/buddas-bible/zonai/pull/322) Draft, `experiment/fluid-aperture-aware-sampling`, 최신 CI **42596530093** 검증 중. 기존 압력/Schur PCG, Flux, 보존형 Tracer, stored MAC face velocity, 기본 OFF 동작 보존. 정규화 보간 `sum(weight*open*v)/sum(weight*open)` 선택형으로 public velocity sample 및 Semi-Lagrangian backtrace에서 사용. Sandbox UI, Unit/World 차등 회귀, matched CSV에 더해 **동일 유체 snapshot의 legacy vs aperture sample을 직접 비교하는 계측**을 추가함. 최신 CI 완료 전 병합 금지.
- **PR #322 최초 Release 비교(검증 완료 run 42594468337):** h=0.25, Fluid=4, TwoWay ON, Drag=0에서 원시 face 속도 peak 기존 28.13385m/s → 실험 2.93254m/s, 양쪽 PCG 실패 0. 단 두 장면은 향후 Body trajectory/face aperture가 다르며 실험 peak 인접 cell fluid fraction은 2.2e-7뿐이므로 **14.18→2.93m/s가 동일 위치의 보간 개선이라는 주장은 성립하지 않음**. 이에 동일 snapshot 0.1 이상 유체 cell의 paired query 계측을 추가했고 run 42596530093 결과 필요. CPU 비용 상승 가능성이 있어 전체 프레임 프로파일 추가 고려.

## 1. 단계별 상태

| 단계 | 상태 | 메모 |
| --- | --- | --- |
| S0 재현·진단 baseline | **1~3차 병합, 4차 PR #322 검증 중** | OFF 기본의 aperture-aware 속도 샘플러 실험 및 차등 테스트 |
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
- PR #321 **병합**: 문제 V-face 인접 유체 셀 76.17%에서 기존 raw normal sample 14.17676m/s 측정.
- 진행 중 PR #322 **Draft**: 정규화된 `sum(weight*open*velocity)/sum(weight*open)` 보간 선택 옵션, 전면 개방 또는 Cut-cell OFF 결과 일치, 좁은 면 보간 영향/이류/Drag/World 검증. Windows Release `--aperture-sampling-differential`은 h=0.5/0.25, Fluid=4, TwoWay ON, Drag 0/3, sampler 0/1 동일 초기 조건 비교. 개선이 확인되지 않아도 기본값 OFF로 유지.
- **S0 미완료**: Fluid 2/8 및 static 많은 장면, 경계 걸침, moving cut-cell swept mismatch 연관 분석, 국소 face velocity/시각 tracer 관계, 사용자 장면 장기 재현 및 알고리즘 원인 수정.
- 새 대화의 첫 작업: PR #322 CI **42596530093**의 Ubuntu/Windows/Release 검증을 완료하고 `--aperture-sampling-differential` CSV에서 `same_field_pair_valid=1`인 h=0.25/Fluid4/TwoWay ON/Drag0/frame6, face V(17,13) 기록의 legacy/aware 보간값과 유체 점유율을 실제 동일 데이터로 비교. 회귀·비용·PCG/Body 상태를 기록하고 Opt-in PR 병합/보류 판단. 압력 급등 수치의 직접 원인 미해결, S0 완료 아님.

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

zonai 환경 물리 개발을 이어가자. master와 병합 PR #319~321, Draft PR #322, 계획 PR #318의 docs/roadmaps/environment-status.md, docs/fluid-coupling-audit.md를 확인해. 최신 PR #322 CI 42596530093을 먼저 검증하고 Windows Release --aperture-sampling-differential CSV의 same_field_pair_valid=1인 h=0.25m/Fluid4/TwoWay ON/Drag0/frame6에서 같은 Cut-cell 유체 상태의 raw/aware normal velocity와 fluidFraction을 비교하라. 최초 matched run 42594468337에서 max raw face 28.13→2.93m/s 관측했지만 다른 Body 궤적과 주변 fraction이므로 원인 확정 금지. 테스트·성능·기본 OFF 보존을 검증한 후 PR 병합/보류하고 결과를 기록하라.
