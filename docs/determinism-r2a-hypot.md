# R2a-2: deterministicHypot (binary64) — Math + Spline first migration

> 의존성: R1a [#329](https://github.com/buddas-bible/zonai/pull/329) → R1b [#330](https://github.com/buddas-bible/zonai/pull/330) → R1c [#331](https://github.com/buddas-bible/zonai/pull/331) → R2a-1 [#332](https://github.com/buddas-bible/zonai/pull/332) → **R2a-2**.
>
> 해당 단계는 `master`가 아닌 #332의 최신 커밋을 기반으로 하는 stacked Draft PR이다. 게임 물리 공식과 스텝 구조를 재설계하지 않는다.

## 왜 별도 hypot인가

[R2a-1](determinism-r2a-fp-contract.md)의 Windows/MSVC ↔ Ubuntu/GCC 감사에서 `std::hypot(double)`가 **2개 샘플에서 비트 단위로 불일치**했다. 원시 stdlib 결과의 허용 오차 비교로 통과시키면 플랫폼 차이를 가리는 문제가 있으므로, 공통 계산 경로를 도입해 실제 필드에 적용한다.

그러나 단순 `sqrt(x*x+y*y)`는 입력이 정상적인 유한값이더라도 제곱의 중간 오버플로 또는 언더플로가 생길 수 있다. 그래서 `src/math/deterministicMath.h`에 배율을 정규화한 구현을 추가한다.

```text
major = max(abs(x), abs(y))
minor = min(abs(x), abs(y))
hypot = major * sqrt(1 + (minor / major)^2)
```

- `major=0`이면 **양의 0**을 반환.
- 한 인자가 무한대면 양의 무한대, 무한대가 없고 NaN이 있으면 NaN 반환.
- 피연산자의 순서와 부호를 바꿔도 정상적인 결과의 비트가 같도록 정규화한다.
- `std::hypot` 구현을 호출하지 않으며, 라이브러리 `sqrt`의 x64 명령 정확도와 컴파일러 FP 정책은 여전히 **실제 Windows/Ubuntu 비트 감사 대상**이다.
- 부동소수점 exception flag까지 std::hypot과 동일하다고 주장하지 않는다. 극단적으로 작은 상대 비율은 중간 제곱이 언더플로해도 표현 가능한 결과의 반올림에 무시할 수 있는 수준이며, 실제 정확성은 테스트로 확인한다.
- 새로운 API는 binary64 2인자 전용. 사용 범위와 값 정책이 확정되기 전 float·3인자 overload를 임의로 추가하지 않는다.

## 이번 PR의 변경 범위

| 파일 | 변경 |
| --- | --- |
| `src/math/deterministicMath.h` | 작은 inline 공통 `deterministicHypot(double,double)` 추가 |
| `src/geometry/catmullRomSpline2.cpp` | Spline 기하 모듈의 `std::hypot` 호출 10곳을 새로운 함수로 교체 |
| `tests/math/deterministic_hypot_test.cpp` | 20개 극단/경계 사례 + 고정 PRNG 생성 20,000개 사례, 순서·부호 대칭, std::hypot 대비 4 ULP 이내, overflow/NaN/Inf 확인 |
| `tests/determinism/fp_math_audit_test.cpp` | 기존 `libm.hypot` 원시 결과는 그대로 유지하면서 별도의 `zonai.hypot` 16개 결과를 추가 기록 |
| `tests/CMakeLists.txt` | `deterministicHypotTests` CTest 추가 |

이번 단계에서 **아직 교체하지 않는 사용처**:

- `src/collision/rotatingSweepBounds2.h`, `src/collision/chainSegmentCastOwnership2.h`, `src/collision/timeOfImpact2.cpp` 및 `src/dynamics/world.h`: 보수적 충돌/CCD 상한과 관련. 새로운 길이 함수가 원래 stdlib보다 1 ULP 작아지는 경우 충돌 후보 상한이 줄어들 수 있으므로 **다음 범위에서는 nextafter 등 바깥쪽 보정과 시간·형상별 회귀를 함께 검토**한다.
- `src/dynamics/joints/splineRailJointConstraint2.cpp`: Joint solver의 Arc length·Constraint 분기와 직접 관련되므로 물리 응답 테스트와 별도의 결과 비교 뒤 변경한다.
- `src/fluid/fluidField2.cpp`: 움직이는 Cut-cell 압력 문제 [#324](https://github.com/buddas-bible/zonai/issues/324) 분석 기준선이 깨지지 않도록 후속 독립 PR로 교체한다.

## 합격 조건

1. `deterministicHypotTests` Windows Debug/Release 및 Ubuntu Debug 성공. 배율 극단·서브노멀 처리·대칭·기본 정밀도를 검사.
2. 기존 Geometry/Spline·Joint/World·Fluid 포함 전체 CTest 통과 및 허용 오차를 임의로 높이지 않음.
3. 교차 플랫폼 `zonai.hypot` fingerprint가 비트 동일. 처음 발견한 `std::hypot` 플랫폼 불일치 샘플을 그대로 기록해, 검증할 대상이 바뀌었다고 결과를 숨기지 않음.
4. 기존 R1c 61-Tick World Replay 비교 통과.
5. 성능 차이는 별도 고정 장면 벤치마크에서 확인. 임의의 성능/안정성 개선 수치를 보고하지 않음.
6. 엔진 전체 결정론은 여전히 미완료. 이번 결과는 **Math 공통 hypot 및 Spline 첫 사용처**에 한정.

### 알려진 CI 적색 상태

R2a-1의 FP audit는 `libm.hypot` 2건과 **엔진에서 현재 직접 사용하지 않는** `libm.acos` 1건까지 동일한 strict gate에서 비교하므로, 이 PR은 새로운 함수만 통과하더라도 전체 FP audit가 계속 적색일 수 있다. 이를 통과했다고 선언하거나 허용 오차로 덮지 말고, 관련 호출의 이관과 테스트의 "권위 있는 Math" 범위 확정을 후속 PR로 진행한다. **실험용 `std::acos` 불일치를 외면한다는 뜻은 아니다.**

## 다음 작업

R2a-2 후속에서 실제 캐시/보존 경계용 부동소수점 범위를 검사하고 Collision/CCD 호출부를 보수적 반올림과 함께 이관한다. 이후 Spline Joint 및 Fluid를 각각 독립 회귀로 이관하고, 표준 삼각함수에 대해 더 넓은 샘플링과 결정론 함수 필요성을 판단한다.
