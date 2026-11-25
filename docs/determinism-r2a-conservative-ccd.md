# R2a-3 — Collision/CCD의 보수적 길이 상한 정리

> 기준: 미병합 Draft [R2a-2 PR #333](https://github.com/buddas-bible/zonai/pull/333), 커밋 `7fa848ac`. 이 문서는 Collision/CCD의 **첫 번째 상한 교체**를 설명한다. 기존 R1a/#329 → R1b/#330 → R1c/#331 → R2a-1/#332 → R2a-2/#333 이후 병합한다.

## 배경과 안전 구분

Windows/MSVC와 Ubuntu/GCC에서 일반 `std::hypot(double)`의 결과가 실제로 다를 수 있음을 R2a-1에서 확인했다. R2a-2는 `deterministicHypot(double,double)`을 제공하고 Spline 기하만 교체했다.

Collision/CCD에서 길이는 두 가지 의미로 사용된다. **정규화**는 현재 길이를 가능한 정확하게 구하는 것이 목적이다. 반면 **후보 AABB의 반경과 CCD 최대 이동 속도**는 값을 조금이라도 과소평가하면 접촉 후보를 놓치거나 진행 간격을 과대평가할 수 있다. 따라서 두 용도를 명확히 분리한다.

- `deterministicHypot(x,y)`: 기존 R2a-2의 배율 정규화 계산. 회전 표현의 정규화 길이에 사용한다.
- `deterministicHypotUpperBound(x,y)`: 일반 계산 결과가 0이거나 비유한 경우를 제외하고 **양의 binary64 표현을 16 ULP 위쪽으로 이동**시켜 후보 상한의 수치적 여유를 확보한다. 비트 증가 시 양의 무한대를 넘어 NaN이 되지 않도록 포화한다. 이는 고정된 FP 환경에 따른 방어적 여유이며, **모든 실수 입력에 대한 수학적 상한 증명은 아니다.**
- 기존 회전 Sweep AABB의 별도 float pad와 `std::nextafter` 외향 확장은 **그대로 보존**한다. 기존 TOI의 1.001 speed margin도 유지한다.
- 0 입력에서 양의 0을 반환해 정지 상태를 가짜 이동으로 만들지 않는다. Inf/NaN 정책은 기존 R2a-2와 동일하게 유지한다.

## 실제 변경

| 파일 | 변경 | 이유 |
| --- | --- | --- |
| `src/math/deterministicMath.h` | `deterministicHypotUpperBound` 추가 | 상한 사용을 호출부에 명시 |
| `src/collision/rotatingSweepBounds2.h` | 회전 정규화 두 곳은 `deterministicHypot`; vertex 반경은 `deterministicHypotUpperBound` | 정규화와 후보 경계의 책임 분리 |
| `src/collision/timeOfImpact2.cpp` | 병진 속도·반경 상한 두 곳 교체 | Lipschitz speed bound의 과소평가 위험 완화 |
| `src/collision/chainSegmentCastOwnership2.h` | 움직이는 형상 반경과 병진 속도 두 곳 교체 | Chain 재시도 시간 상한 보호 |
| `src/dynamics/world.h` | Shape Sweep 반경·타깃 회전 스케일 두 곳 교체 | BroadPhase 후보 누락 방지 |
| `tests/math/deterministic_hypot_test.cpp` | 유한/극단/서브노멀 20,020개 입력 범위에서 `std::hypot` 이상, 최대 20 ULP 여유 및 부호/순서 대칭 검사 | 보정값과 정확도 회귀 |
| `tests/collision/rotating_sweep_bounds_test.cpp` | 좁은 Capsule과 원점 0/4096/65536, 양·음의 회전 4개를 조합해 1025개 포즈별 AABB 포함 검사 | 수치적 경계와 회전 진행 검사 |
| `tests/determinism/fp_math_audit_test.cpp` | `zonai.hypotUpper`의 16개 바이너리 비트 샘플 추가 | Ubuntu/GCC vs Windows/MSVC 직접 비교 |

`world.h`의 두 호출은 기존의 근사값 상한으로 처리되는 부분에 한정한다. World의 Contact·Joint solver 및 유체 계산은 변경하지 않는다.

## 검증 조건 및 제한

1. Windows/Ubuntu Debug CTest 전체 통과. 회전 Sweep, TimeOfImpact, Chain 재시도, World ShapeSweep/ShapeCast 기존 회귀 **허용 오차는 높이지 않는다**.
2. Windows Release 기존 테스트와 정보성 성능 샘플 재확인.
3. R1c의 61 Tick World Body Replay 스냅샷 동일성 유지.
4. 새 `zonai.hypotUpper` 교차 플랫폼 16개 비트 샘플 일치 여부 확인.
5. 신규 상한이 기존 `std::hypot` 오라클 결과보다 작지 않은지 폭넓게 검사한다. 그러나 이는 테스트된 입력의 경험적 검증이지 형식적 상한 증명이 아니다.
6. 원래의 비결정적 `libm.hypot` 2개 및 진단용 `libm.acos` 1개 샘플은 숨기지 않는다. 따라서 **상속된 strict FP audit는 계속 적색일 수 있으며, 병합 준비가 완료됐다고 표시하지 않는다**.
7. 새 결과가 속도, 충돌 검출 횟수, 압력 안정성을 개선했다고 주장하지 않는다. 별도 장기 회귀 및 벤치마크가 필요하다.

## 다음 작업

Spline Joint의 `hypot` 사용은 실제 Constraint 분기와 연결되므로 별도 PR에서 다룬다. Fluid Cut-cell 수치 문제는 독립된 검증으로 분리한다. 이후 기존 라이브러리 함수의 권위 있는 호출을 모두 이관했다는 증거가 확보되면, raw libm 연구 샘플과 정식 결정론 Gate를 목적에 맞춰 분리하되 발견된 3건의 차이를 지우거나 허용 오차로 은폐하지 않는다.
