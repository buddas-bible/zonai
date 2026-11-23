# R2a-1: 부동소수점 계약 명시 및 플랫폼 간 Math 감사

> Base: R1c Draft [#331](https://github.com/buddas-bible/zonai/pull/331) / commit `e419809c`. R1a [#329] → R1b [#330] → R1c [#331] → **R2a-1** 순서로 통합한다.
>
> 목표: zonai **전체**가 플랫폼 간 비트 단위로 결정론적인 물리 계산을 수행하도록 하기 위한, 검증 가능한 빌드 정책과 첫 Math 차이 추적. 이번 작업은 수치해법 변경이나 libm 대체 구현이 아닌 **정책·계측·CI**에 집중한다.

## 1. 검사에서 확인된 실제 위험 경로

| 경로 | 현재 코드 및 위험 | 처리 계획 |
| --- | --- | --- |
| `src/math/rot2.h` | `rot2::FromRadians`가 `std::cos(float)`, `std::sin(float)` 사용. 외부 math library의 플랫폼별 반환 raw bits 보장 부재 | 재현성 검사 후 필요 시 결정론적 `cosSin` 구현으로 별도 PR 교체 |
| `src/dynamics/joints/*Constraint2.cpp` | Revolute/Motor/Weld/Prismatic/SplineRail 등에서 `std::atan2(float)` 사용 | Constraint 계산이므로 다음 R2a-2에서 공통 결정론적 `atan2` 도입 검토 |
| `src/dynamics/world.cpp`, `src/dynamics/splineRailJointWorld.cpp` | Current angle 등을 `std::atan2`로 계산 | 관찰값만인지 후속 Solver에 반영되는지 별도로 구분 |
| `src/fluid/fluidField2.cpp` | `std::atan2(double)`, `std::asin`, `std::acos`, `std::sqrt`, `std::hypot` 등이 기하·Cut-cell/유체 계산에 존재 | double geometry 결과를 포함한 별도 유체 결정론 검사 필요 |
| `src/collision/*` 및 `src/geometry/*` | `std::hypot(double)`, `std::sqrt` 등 | 극한값/분기 동률/최초 차이를 별도 테스트로 확장 |
| 기존 CMake | FP contraction 제어가 명시되어 있지 않았음 | 명시적 정책 도입, 엔진 헤더를 인라인 계산하는 consumer에도 전파 |
| `src/math/vec2.h` | Dot/Cross 및 rotation 합산 시 곱셈 + 덧셈/뺄셈이 FMA로 합쳐질 수 있음 | contraction 금지로 검사, runtime 환경 별도 추적 |

## 2. 이번 변경의 FP 계약

- MSVC: `/fp:precise`를 `zonai` 타깃의 `PUBLIC` 옵션으로 명시. Visual Studio 2022+의 precise 기본 정책은 contraction OFF. `/fp:fast` 또는 `/fp:contract`를 상위 게임 타깃에서 뒤에 추가하면 계약 위반 위험이 있다.
- GCC/Clang: `-fno-fast-math -ffp-contract=off`를 `PUBLIC`으로 전파해 실제 physics 함수와 헤더 인라인 사용자의 연산 정책을 통일.
- 이번 변경은 `/fp:strict`를 무조건 사용하거나 모든 수학을 고정소수점으로 대체하지 않는다. 기본 FP rounding mode `FE_TONEAREST`가 유지되는지 테스트한다.
- 플래그만으로 다중 스레드 Reduction/순회 순서, OS libm 차이, CPU SIMD/FTZ-DAZ 정책, debug/release의 비트 일치나 모든 플랫폼을 자동으로 보장하지는 않는다.

공식 근거:
- Microsoft: [Specify floating-point behavior](https://learn.microsoft.com/en-us/cpp/build/reference/fp-specify-floating-point-behavior?view=msvc-170)
- Box2D: [Determinism](https://box2d.org/posts/2024/08/determinism/)
- Clang: [Users Manual](https://clang.llvm.org/docs/UsersManual.html)

## 3. 신규 `fpMathAuditTests`

테스트 전용 `tests/determinism/fp_math_audit_test.cpp`:

1. IEEE-754 binary32/64 기초 확인, round-to-nearest 환경 검사.
2. `1+2^-23`의 곱과 `-(1+2^-22)` 합산이 FMA로 contraction되지 않았는지 1-bit 수준으로 검사.
3. `rot2::FromRadians`, 회전 벡터, `std::atan2(float/double)`, `std::hypot(double)`, `std::sqrt(float)`, `std::asin(double)`, `std::acos(double)` 결과를 **원시 float/double 비트**로 `TSV`에 기록.
4. 입력은 16개 각도 및 경계값을 raw IEEE-754 패턴에서 만들며 런타임 문자열 float 파싱은 하지 않는다.

실행 명령:

```text
fpMathAuditTests
fpMathAuditTests --emit <path.tsv>
```

## 4. CI 설계

[R1c CI](../.github/workflows/determinism.yml)를 재사용해 동일 엔진 커밋/도구 체인으로 Ubuntu와 Windows에 새 테스트 실행 파일을 빌드한다.

- 두 플랫폼이 각각 `fp-ubuntu.tsv`, `fp-windows.tsv`를 올린다.
- 기존 World 61 Tick 비교를 **그대로 필수 유지**한다.
- 새 Math fingerprint 비교는 각 줄의 원시 비트가 하나라도 다르면 CI를 **실패**시킨다. 최초 `operation/sample` 및 양쪽 비트값을 `fp-comparison.txt`로 보존한다.
- 차이가 나는 경우 바로 억지로 허용하지 않는다. `rot2`나 Joint `atan2`에 필요한 결정론 함수를 채택하되, 물리 정확성·기존 테스트·성능은 별도로 검증하고 좁은 후속 PR에서 교체한다.

## 5. 기존 기능과 구분

이 작업은 **정적 분석에서 발견한 위험 경로와 최소 입력 범위의 실제 raw-bit 비교**만 검사한다. 관측되는 Math fingerprint가 일치해도 전체 엔진 결정론 통과 증거가 아니며, 다음 단계에서 Body/Shape/Contact/Joint/CCD 내부 권위 상태를 포함하는 테스트를 확장해야 한다.

유체의 moving Cut-cell 압력 스파이크는 이 빌드 정책과 무관하게 남아 있는 **별개의 수치 안정성 문제**다. 기존 [#324](https://github.com/buddas-bible/zonai/issues/324) / 실험 [#326](https://github.com/buddas-bible/zonai/pull/326) 기록은 보존한다.
