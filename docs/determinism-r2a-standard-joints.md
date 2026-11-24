# R2a-6 — 표준 Joint의 atan2 결정론과 각도 나머지 검증

> 기준: 미병합 [R2a-5 PR #336](https://github.com/buddas-bible/zonai/pull/336) `e5d2a9f`. **R1a #329 → R1b #330 → R1c #331 → R2a-1 #332 → R2a-2 #333 → R2a-3 #334 → R2a-4 #335 → R2a-5 #336 → 이 PR** 순서로만 병합한다. `master`는 변경하지 않았다.

## 목표와 확인한 위험

R2a-5에서 Spline Rail의 플랫폼별 `std::atan2(float)` 1 ULP 차이가 실제 Body 각속도로 전파되는 현상을 재현했다. 고정된 연산 순서의 `deterministicAtan2`로 Spline 제약 계산을 교체한 뒤, 관측 가능한 두 Spline 레일 장면의 56,394개 행에서 물리 상태 차이가 0이 됐다.

이번 단계는 동일한 원인이 될 수 있는 다른 **4개 표준 Joint 솔버의 각도 계산**과 Spline Rail의 초기화·재설정 각도를 같은 함수로 통일한다.

## 실제 변경

- `src/dynamics/joints/revoluteJointConstraint2.cpp`: 회전 범위 제약/추측적 제한 속도를 위한 1개 `atan2`.
- `src/dynamics/joints/prismaticJointConstraint2.cpp`: 횡방향·회전 오차를 위한 1개 `atan2`.
- `src/dynamics/joints/weldJointConstraint2.cpp`: Weld 상대 회전 스프링/편차를 위한 1개 `atan2`.
- `src/dynamics/joints/motorJointConstraint2.cpp`: Angular spring torque 계산을 위한 1개 `atan2`.
- `src/dynamics/splineRailJointWorld.cpp`: 레일 생성 시 tangent offset, rotation mode 변경, 기준 회전 재설정에서 총 6개 `atan2`. Solver와 초기화 계산이 다른 math 경로를 사용하지 않도록 한다.
- `src/dynamics/world.cpp`: Revolute/Prismatic/Weld 각도 조회용 3개 `atan2`. 기존과 동일한 현재 상대 Rotation에서 결정론적 값을 계산한다. 조회 과정은 물리 상태를 변경하지 않는다.
- **다른 Joint, Fluid, Sandbox와 CCD 계산식·Impulse 식·오차 임계값·Step/서브스텝 순서를 변경하지 않는다.**

## 신규 표준 Joint 회귀

`tests/determinism/standard_joint_determinism_test.cpp` (`standardJointDeterminismTests`):

- Revolute (각도 limit + motor), Prismatic (직선 limit + spring + motor), Weld (선형·각도 spring), Motor (속도 motor + 각도 spring)의 **4개 독립 장면**.
- 각 장면에 두 개의 독립 World를 구성하고 `dt=1/60`, 4 substeps, Tick 0~180을 실행. Tick 40/80/120에서 동일한 외력과 Impulse를 가해 warm-start 및 과도 응답 경로를 반복한다.
- 기존 R1a의 공개 World/Body 스냅샷을 필드별 전체 바이트 비교하고, Joint별 공개 `currentAngle`, `currentTranslation`, `lateralError`, `force`, `torque`, `motorTorque`, `motorForce`, `springForce`, `anchor` 등도 추가 비교한다.
- `--emit output.tsv`로 장면·Tick·필드명·원시 비트를 기록. Ubuntu/GCC와 Windows/MSVC가 생성한 TSV 파일을 CI에서 직접 비교해 **첫 불일치 장면/Tick/필드**를 기록하고 검사 작업 실패 처리한다.
- `worldToken`, 포인터, 구조체 padding을 기록하지 않는다.

이 테스트가 일치하더라도 Body/Joint 내부 **persistent solver impulse cache 및 모든 다음 Tick 영향 상태를 직접 직렬화한 것은 아니다**. 공개 반력과 월드 관측 결과만 비교하는 첫 단계다. 사용하지 않은 Body/Shape/Contact/Fluid 내부 필드도 검사하지 않는다.

## 각도 remainder 감사

Spline Rail은 `std::remainder(angle, 2π)`를 사용한다. 이 함수는 tie-to-even과 signed zero 및 큰 각도 처리 정책이 있어, `fmod`, 단순 감산 또는 `std::round`를 사용한 교체는 원본 동작을 바꿀 위험이 있다.

이번 PR은 **치환하지 않고**, `fpMathAuditTests`에 ±0, ±π, ±2π, ±3π, ±π/2, ±10, ±1000 등 16개 finite 입력을 추가했다. 두 플랫폼의 원시 비트를 엄격 비교하여 실제 차이가 있는지 판별한다. 차이가 없다고 판정되더라도 모든 실수 입력의 플랫폼 간 일치를 증명하지는 않는다.

## 검증 및 후속

1. 기존 Windows/Ubuntu 전체 CTest (Joint 정확도·개별 물리 회귀, Spline, Fluid 포함) 통과.
2. 신규 Standard Joint 두 독립 World의 각 Tick 바이트 동일성.
3. Ubuntu와 Windows TSV 4개 Joint 장면의 원시 비트 완전 비교.
4. 기존 Spline Joint와 R1c 61개 World Replay 스냅샷 **bitwise 동일성 유지**.
5. `libm.angleRemainder` 신규 샘플과 기존 표준 라이브러리의 실제 불일치 샘플을 별도 기록. 기존 `libm.hypot` 2건, `libm.railAtan2` 3건, `libm.acos` 1건은 숨기지 않는다. 따라서 기존 FP 연구용 전수 비교는 여전히 적색일 수 있다.
6. 실패한 Joint 장면이 있으면 초기 입력/서브스텝/Impulse 캐시로 원인을 좁힌 다음 분리 PR에서 수정한다. 테스트를 epsilon 비교로 대체하지 않는다.

**향후 내부 상태 검사:** World의 read-only snapshot을 확장하여 Joint 종류별 persistent impulse, warm-start timing, referenceAngle, accumulated spring/motor/limit impulse와 generation을 안정적으로 추적할 설계를 수립해야 한다. Production 전체에 Debug getter를 무분별하게 추가하지 않는다.

현재 작업은 Fluid Cut-cell 압력 스파이크(#324)를 해결하지 않는다.
