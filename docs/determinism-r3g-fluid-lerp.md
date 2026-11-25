# R3g — MAC 보간 결정론 적용 및 비교

기반: [R3f #347](https://github.com/buddas-bible/zonai/pull/347), HEAD `6904f042f2c1f8cc8b0bee3dcf99c2c6be015591`.

## 사전 검증

[R3f 결정론 CI](https://github.com/buddas-bible/zonai/actions/runs/45341391001): 엔진 수학 1,434개 모두 raw-bit 일치, 이전 Fluid 312,474행 중 48,159행 불일치 유지(기존 std::lerp 경로). [전체 CMake](https://github.com/buddas-bible/zonai/actions/runs/45341390904) 양 OS 성공.

## 변경

- `fluidField2::sampleComponent`의 세 std::lerp 호출만 검증된 `deterministicLerp`로 교체한다. 기존 clamped grid 좌표, 역추적, Cut-cell, pressure solve, tracer transport, step cadence 유지.
- R3e 관측용 보간도 같은 함수로 이관하고 실제 이류 출력과 정확히 일치해야 통과하도록 한다.
- 독립 Field 및 비관측 Control Field의 상태 길이, 모든 반력(힘/토크), 유한 반력도 확인한다. 기존 trace 형식/행 이름 유지.
- 기존 strict engine math 및 Fluid 비교는 완화하지 않는다.

## 비교 계획

1. 기존 Windows/Ubuntu Debug 추적 312,474행 비교, 첫 불일치와 수 기록. 기존 32 Tick sampled/analytic 장면을 유지한다.
2. 전체 CMake 회귀 확인. 작은 입력에서의 동일성은 더 큰 scene, 타 CPU, Release 전체, World-Fluid 결합 결정론까지 보증하지 않는다.
3. 별도의 R3g 한정 CI `fluid-lerp-ab.yml`로 **동일 Ubuntu runner**에서 R3f 기준 SHA와 후보를 Release 빌드한다. 기존 sandboxWindStepBenchmark의 8개 timing case를 번갈아 세 번 실행, median of medians 비교. 기존 coupled audit 192행의 최대 절대 변화, 비유한 값 및 PCG 실패도 기록한다. 시간 측정은 strict bit gate에 포함하지 않는다.
4. 위 A/B는 실제 World/Fluid interleave의 제한된 궤적에서의 비교이며 수치 안정성 증명이 아니다. 타이밍의 변동성과 pressure iteration 변화도 고려한다. 장기 유지할 기준은 별도로 결정하며, R3g 전용 역사적 baseline workflow는 최종 통합 시 정리한다.

## 수용 기준

- 보간 원인 제거를 strict trace로 확인하고, 다른 최초 차이가 남으면 별도 격리한다.
- 기존 회귀 및 수학 gate 유지. A/B에서 안정성이나 성능 악화가 크면 원인 확인 후 적용 결정.
- master 병합 금지, 실험적 Fluid #326 분리. R3g draft는 R3f 위에 쌓아 리뷰한다.
