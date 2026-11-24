# R2a-5 — 고정 연산 순서의 atan2와 Spline Rail Joint

> 기준: 미병합 Draft [#335](https://github.com/buddas-bible/zonai/pull/335)의 `2bb45198`. 기존 [#329](https://github.com/buddas-bible/zonai/pull/329) → [#330](https://github.com/buddas-bible/zonai/pull/330) → [#331](https://github.com/buddas-bible/zonai/pull/331) → [#332](https://github.com/buddas-bible/zonai/pull/332) → [#333](https://github.com/buddas-bible/zonai/pull/333) → [#334](https://github.com/buddas-bible/zonai/pull/334) → #335 → **이 작업** 순서로만 통합한다. `master`는 변경하지 않는다.

## 1. 출발점 — 실제 플랫폼 불일치

[R2a-4 진단 CI #45229486240](https://github.com/buddas-bible/zonai/actions/runs/45229486240)에서 R1c의 표준 World Replay 61/61개는 비트 단위로 일치했지만, 닫힌 Spline Rail Tick 2의 `body[2].angularVelocity`가 Ubuntu/GCC `0x3fe74256` ↔ Windows/MSVC `0x3fe74257`로 1 ULP 달랐다.

특정 Spline Rail의 `std::atan2(float)` 입력이 동일해도 반환 값이 달랐음을 확인했다. 정확한 반례는 다음과 같다.

| 입력 (y, x; raw float bits) | Ubuntu stdlib 결과 | Windows stdlib 결과 |
| --- | --- | --- |
| `0x3c216d9a`, `0x3f7ffcd3` | `0x3c216e45` | `0x3c216e44` |
| `0xbd0ffd6b`, `0x3f7fd77e` | `0xbd100503` | `0xbd100504` |
| `0x3f7d908f`, `0xbe0ce895` | 진단에서 불일치 확인 | 진단에서 불일치 확인 |

원인은 단지 일반 길이의 `std::hypot`만이 아니었다. `std::atan2`도 실제 Joint 물리 응답에 기여하는 입력에서 플랫폼별 차이가 있다.

## 2. 공통 함수

`src/math/deterministicMath.h`에 `deterministicAtan2(float y, float x)`를 별도 공개 함수로 추가한다.

- float 입력은 먼저 **정확히 표현 가능한 double**로 승격한다. `min(|x|,|y|)/max(|x|,|y|)`로 비율을 [0,1]에 제한해 overflow 위험을 회피한다.
- 입력 비율이 `tan(pi/8)`보다 크면 `atan(a) = pi/4 + atan((a-1)/(a+1))` 항등식을 사용해 다항식의 정의역을 약 ±0.4142로 줄인다.
- 그 구간에서 31차 홀수 다항식 `z(1-z²/3+z⁴/5-...-z³⁰/31)`을 고정된 Horner 순서로 계산. 절단 오차의 이론적 상한은 감소하는 교대급수의 다음 항 `|z|^33/33`, 대략 `7e-15` 라디안이다. **이는 double 반올림 오차를 포함하는 총 오차의 보증은 아니다**.
- 대칭성과 사분면 규칙으로 [-π,+π] 반환 후 float로 한 번 변환한다. `std::atan`, `std::atan2`, OS libm은 호출하지 않는다.
- IEEE-754 NaN, ±0, ±∞의 반환 규칙을 분명히 한다. 모든 CPU/FTZ-DAZ 상태에 대한 결정론 보증은 **실제 CI에서 확인하는 범위에 한정**한다.
- CMake에서 이미 추가한 비결합 FP 정책 `/fp:precise`, `-ffp-contract=off`를 유지한다. `-ffast-math`와 FMA 병합은 금지한다.

Box2D도 [결정론 문서](https://box2d.org/posts/2024/08/determinism/)와 [b2Atan2 API](https://box2d.org/documentation/group__math.html)에서 OS 독립 atan2 근사를 사용한다. 다만 이 PR은 Box2D의 짧은 근사식 대신, Spline Joint의 각도·곡률 회귀에 대한 정확도를 우선하여 **더 긴 구간 축소/다항식**을 채택한다.

## 3. 실제 적용 범위

`src/dynamics/joints/splineRailJointConstraint2.cpp`의 2개 `std::atan2` 호출만 `deterministicAtan2`로 교체한다.

- 곡선 접선의 목표각 `atan2(tangent.y, tangent.x)`.
- Body 상대 회전의 실제각 `atan2(rotation.s, rotation.c)`.

기존 `std::remainder`와 기타 Joint의 `std::atan2`, World 조회 계산은 이번 PR에서 변경하지 않는다. 이들 함수의 실제 불일치가 확인되면 각각 별도 회귀/치환을 검토한다. Warm Start, Motor, Spring, Limit, arc-length, substep, Collision, CCD, Fluid 계산은 그대로 둔다.

## 4. 검증 및 실패 해석

- 신규 `deterministicAtan2Tests`: 36개 signed-zero/무한대 특수 조합, 서브노멀/최대·최소 유한값, 앞서 검증된 3개 직접 반례, 축·사분면·대각선 경계, 고정 PRNG로 생성한 **50,000개 raw-bit float 쌍**에 대해 `std::atan2f` 대비 최대 **2 ULP 이내의 정확도**를 확인. 이 ULP 제한은 오직 정확도 검사이며 플랫폼 결정론 기준을 완화하지 않는다.
- `fpMathAuditTests`: `zonai.atan2f` 16개, `zonai.railAtan2` 3개, 고정 입력의 `zonai.atan2Wide` 256개를 Windows/Linux 양쪽에서 수집해 완전 비트 비교. 앞서 발견된 `libm.railAtan2` 3건도 계속 유지.
- `splineRailDeterminismTests`: 기존 240-Tick × 2개의 Open/Loop 장면을 유지하며, 새로운 읽기 전용 `diagnostic.zonai...Atan2`를 기록. **실제 물리 필드 불일치와 표준 라이브러리 진단 불일치를 CI에서 구분**하되 둘 다 원시 로그로 남긴다.
- 기존 전체 CTest 및 R1c 61-Tick World Replay는 동일한 엄격도로 유지. 신규 CI에서 차이가 남으면 첫 `scene/tick/field`와 raw bit를 보존한다.
- 두 플랫폼에서 새 함수의 출력이 같더라도 **엔진 전체 결정론은 보증되지 않는다**. 실제 물리 상태가 일치하는지도 별도 판정한다.
- 기존 `libm.hypot` 2건, `libm.acos` 1건, `libm.railAtan2` 3건은 **기존 엄격 FP 진단 비교에서 보존**하므로 전체 CI는 계속 적색일 수 있다. 검증 실패를 허용 오차로 가리거나 성공으로 보고하지 않는다.
- 성능 측정 없이 `atan2`보다 빠르다고 주장하지 않는다. Spline Rail 실전 속도와 정확도 회귀를 통과한 후 최적화한다.

## 5. 후속

실제 Spline Joint의 권위 필드가 맞지 않으면 Substep 내 `std::remainder`, `std::atan2`와 Impulse/warm-start 캐시를 추가 추적한다. 만약 수학 함수 치환으로 물리 상태가 전부 일치하면 남는 라이브러리 진단 차이는 연구용으로 보존하면서 전체 엔진에서 사용 중인 각도 함수와 Fluid/CCD 내부 상태를 더 넓게 감사한다.
