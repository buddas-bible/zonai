# R3e — MAC 이류 첫 비트 불일치의 역추적·보간 진단

> 기반: [R3d PR #345](https://github.com/buddas-bible/zonai/pull/345), 기준 HEAD \`59e4c98acd91739d7835fe6380af995e46549f40\`. 이전 PR은 계속 Draft로 남기며, \`master\`와 생산 유체 솔버는 바꾸지 않는다.

## R3d 결과

[결정론 CI](https://github.com/buddas-bible/zonai/actions/runs/45270369145)에서 Windows/Ubuntu \`312,352\`개 유체 상태 행을 비교했다.

- 최초 불일치: \`sampledCutCell.tick.1.stage.06-advection.u[71]\`.
- Ubuntu \`3f0ccd7a/4\` vs Windows \`3f0ccd7b/4\`, 1 ULP.
- 동일 장면의 00~05단계(초기, 점유, 기하, swept, remap, 이류 전 경계)는 일치했다.
- 따라서 관측된 장면에서 최초 원인은 압력 투영 **이전의 이류 계산**이다. 이 사실만으로 \`std::lerp\`가 원인이라고 단정하지 않는다.

## R3e 변경: 관측 전용 재구성

코드 변경 파일: \`tests/determinism/fluid_state_determinism_test.cpp\` **하나**. 기존 \`fluidField2::step\`, \`sampleFrom\`, \`sampleComponent\`, production \`std::lerp\` 및 CI의 strict 비교는 일절 수정하지 않는다.

R3d에서 차이가 처음 확인된 16×12 유체 격자의 \`u[71] = U(3,4)\`를 대상으로 Tick 1의 **Stage 05** 입력을 그대로 읽어 다음을 재계산·기록한다.

1. 타깃 U Face world 위치 \`point\` 및 그 지점의 속도 \`current.x/y\`.
2. \`previous = point - dt * sampleFrom(u,v,point)\` 역추적 좌표.
3. 역추적 좌표의 staggered-grid 변환 \`destinationGrid\`, 4개 보간 샘플 \`a,b,c,d\`, 셀 인덱스, \`tx/ty\`.
4. 원래 C++20 \`std::lerp\`와 동일한 순서의 \`bottom\`, \`top\`, 최종 U 보간 결과.
5. 생산 이류 Step 이후 Stage 06의 실제 \`u[71]\`와 관측 전용 미리 계산한 값의 raw-bit 일치 여부.

기존 10개 단계 전체 격자 Snapshot, 32 Tick 본 상태, 다른 Field와 콜백 없는 Control Field의 동일성 검증은 모두 유지한다. 새 Probe 행을 Stage 06 행 **앞에** 배치하므로 Windows/Ubuntu의 최초 비교 실패가 역추적 좌표인지, 보간 가중치인지, \`std::lerp\`의 중간값인지 더 구체적으로 나타난다. 디버깅 계산을 솔버에 피드백하지 않으며 어떤 성공 판정도 오차 허용치로 바꾸지 않는다.

## CI 검증 기준 및 한계

- Ubuntu/Windows에서 \`fluidStateDeterminismTests --emit\` 성공, 동일 프로세스의 두 probe와 unobserved control 결과 일치.
- Windows/Ubuntu 전체 CMake Debug 및 선택적 Release 회귀 확인.
- 기존 \`Strict Fluid MAC, Cut-cell history and pressure feedback bits\`가 **한 비트라도 다르면 실패하는 방식 유지**, 첫 차이를 raw bits로 출력.
- 외부 \`libm\` 연구용 별도 red 검사는 차단하지 않는다.
- \`previewMatchesActual = 0\`이라면 테스트 재계산과 생산 이류 계산의 반올림/실행 조건부터 조사해야 한다. Probe의 중간값을 그대로 생산 경로의 사실로 간주해서는 안 된다.
- 수렴·성능·다른 Cut-cell/PCG·World↔Fluid 결합 전체를 이 장면 하나만으로 보증하지 않는다.

결과에서 최초 차이를 일으키는 연산이 검증되면 후속 별도 수정에서 결정론적 보간의 정확성·성능·전체 회귀를 비교하고 수용 여부를 판단한다.

## 실제 R3e 첫 실행 증거 (원시 CI artifact 대조)

- CI: https://github.com/buddas-bible/zonai/actions/runs/45272708528
- 양 플랫폼 standalone Fluid 추적 생성 성공. 기존 World/Contact/Joint/Spline 권위 비교와 engine-owned math 388개 일치. Fluid strict 비교는 실패 (물리 차이이므로 정상적으로 적색).
- 전체 312,474행 중 48,159행 차이. 이전 기록 순서에서는 첫 차이가 `sampledCutCell.tick.1.advectionProbe.result.u`였으므로, 계산 순서를 따라 출력하도록 테스트의 행 순서를 바로잡아 재검증한다.

| 구간 | Ubuntu 원시 비트 | Windows 원시 비트 |
| --- | --- | --- |
| 현재 속도 X | `3f12fd58` | `3f12fd58` |
| 현재 속도 Y | `3dffa54c` | `3dffa54c` |
| 역추적 X | `bfa13994` | `bfa13994` |
| 역추적 Y | `bec110b0` | `bec110b0` |
| 최종 보간 tx | `3f763360` | `3f763360` |
| 최종 보간 ty | `3f7ddea0` | `3f7ddea0` |
| U stencil a,b,c,d | `00000000,3e110d4d,3d094d24,3f12fd58` | 동일 |
| 첫 번째 `std::lerp` (bottom) | `3e0b7fe4` | `3e0b7fe4` |
| **두 번째 `std::lerp` (top)** | **`3f0db109`** | **`3f0db10a`** |
| 최종 U 보간 | `3f0ccd7a` | `3f0ccd7b` |
| 실제 Stage-06 U(3,4) | `3f0ccd7a` | `3f0ccd7b` |

- `sourceUMatches`, `sourceVMatches`, `reconstructedMatches`, `previewMatchesActual` 네 플래그가 **Ubuntu/Windows 모두 1**이었다. 진단용 재계산과 실물리 이류 출력을 비트 단위로 교차 확인했다.
- 결과적으로 이 장면의 첫 차이는 역추적 좌표·보간 입력이 아니라 **동일한 입력에 대한 두 번째 `std::lerp(c,d,tx)` 출력**에서 발생한다.
- 위 결과로 `std::lerp` 호출의 플랫폼 의존 비트 차이는 증명됐지만, 그 아래의 라이브러리 알고리즘과 컴파일러 코드 생성 중 정확한 책임은 아직 구분하지 않았다.

### 후속 R3f 수용 단위

현재 `fluidField2::sampleComponent()`의 세 `std::lerp`를 곧바로 바꾸지 말 것. 먼저 C++20 정의역/NaN·Inf/정확한 끝점/부호와 유효 범위 안전성을 정의한 engine-owned 보간 함수를 별도 순수 테스트로 검증한다. Windows/Ubuntu raw-bit 비교, 기존 sampled/analytic 장면 32 Tick, 전체 CTest, 물리 변화·성능 비교를 통과해야만 실제 Fluid 호출부 이관을 검토한다. 기존 strict 게이트·실험용 Fluid #326 분리는 유지한다.
