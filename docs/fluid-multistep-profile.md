# Fluid / World 다중 substep 시간 프로파일 (Part 13y)

`fluidField2::setStepTimingEnabled(true)`는 기본 OFF인 벽시계 시간 계측을 켠다. 일반 시뮬레이션에서는 `steady_clock`을 호출하지 않는다. `getStepTiming()`은 마지막 유효한 Fluid Step을 ms 단위로 분해한다. 선택한 압력 전처리기, 유체 방정식, rigid reaction과 보존 규칙은 변경하지 않는다.

- `occupancy`: Body center-mask, `geometry`: Cut-cell 체적과 공유 face, `swept`: 이동 고체 face flux 및 선택한 reconciliation, `remap`: 재노출 face 보정.
- `boundaries`: 이류 전후 경계 적용과 관련 진단, `advection`: 속도장 이류, `pressure`: PCG/Jacobi를 포함한 전체 투영과 강체 feedback, `tracer`: 추적 물질 수송, `other`: 원래 진단과 history snapshot.
- `total`과 `accounted()`는 각각 전체 경과시간과 단계별 합이다. 시계 호출 비용과 CPU 부하로 둘은 정확히 같지 않을 수 있다. 초기화/계측 OFF 시 이전 측정값을 0으로 지운다.

Sandbox Wind는 **유체 단계별 시간 측정** 체크박스를 제공한다. 기존 World 8 substeps와 Fluid 1·2·4·8회 교대 처리에서 마지막 프레임의 Fluid 구간 시간을 `accumulate`로 합산한다. `Fluid+연동 - Field total`은 World 장애물 snapshot 생성·Drag 및 Force 적용 등의 호스트 비용을 근사한 값이며, 측정 자체의 비용도 포함한다. `World ms`는 World::Step 시간만 표시한다.

`sandboxWindStepBenchmark`는 headless 실제 World+Fluid Sandbox를 사용한다. 0.5m (24×14) 및 0.25m (48×28) 격자에 Circle/Capsule/Polygon, Cut-cell 연립 압력, 소스·Drag·Tracer를 적용하며, World 8 substeps에서 Fluid 1·2·4·8회를 시험한다. 각 경우 4 warmup 프레임 이후 12프레임의 단계별 **중앙값**을 CSV로 출력한다. Windows Release CI에 정보 제공용 실행을 추가하며, CI 시간에 성능 합격/불합격 한계를 두지 않는다.

이 프로파일은 CPU Release/CI 환경의 선택된 Sandbox 장면이며 실제 GPU 렌더링과 OS 프레임 처리 비용은 제외한다. 프레임마다 마지막 Fluid 구간의 연립 PCG 상태만 로그에 남기므로, 모든 substep의 수렴을 증명하는 지표는 아니다. 계측을 켜면 그 자체의 오버헤드가 발생한다.
