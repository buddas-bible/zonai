# R2a-4 — Spline Rail Joint 호 길이·제약조건의 결정론적 norm

> 기준: Draft [#334](https://github.com/buddas-bible/zonai/pull/334), commit `1e40351`. 이 작업은 **#334 이후에만 병합할 수 있는 stacked Draft PR**이다. 기존 `master`에는 병합하지 않는다.
>
> 목표: Spline Rail Joint에서 `std::hypot(double)`가 담당하던 일반 유클리드 길이를 공통 `deterministicHypot(double,double)`로 일관화하고, 실제 월드의 Joint 물리 응답을 플랫폼별 매 Tick 비트 수준으로 관측한다. 보수적 Collision/CCD 상한 함수는 여기에 사용하지 않는다.

## 1. 변경한 계산 경로

`src/dynamics/joints/splineRailJointConstraint2.cpp`의 총 **5곳**이다.

| 계산 위치 | 이전 | 변경 | 물리적 의미 |
| --- | --- | --- | --- |
| `speedAt` | `std::hypot` 1곳 | `deterministicHypot` | 1차 미분 벡터의 길이. Adaptive Simpson arc-length 적분과 종료 분기에 영향 |
| `regular` | `std::hypot` 3곳 | `deterministicHypot` | 접선 길이 계수의 scale. 극소속도/특이 곡선의 수용 여부에 영향 |
| `getSplineRailFrame` | `std::hypot` 1곳 | `deterministicHypot` | 접선 정규화·곡률·거리·제약 행렬과 solver 분기에 영향 |

제약 Solver의 warm-start Impulse, bias/softness, Motor·Spring·Limit 공식, 시뮬레이션 서브스텝 순서는 바꾸지 않는다. 기존 arc-length 적분 허용 오차와 최대 재귀 깊이, 곡선 regular 판정 임계값을 **그대로** 보존한다. 오차를 높이거나 문제 입력을 무조건 받아들이도록 우회하지 않는다.

### 왜 `deterministicHypotUpperBound`를 쓰지 않는가?

이 Joint 경로는 보수적인 충돌 후보 반경이 아니라 **실제 접선 길이**가 필요하다. 임의로 16 ULP 확장된 상한을 쓰면 arc-length와 곡률·모터 속도에 체계적인 편향을 주므로 R2a-3의 상한 함수를 사용하지 않는다.

## 2. 기존 회귀 활용

기존 Spline Rail Joint 전용 테스트는 이미 선형/곡선, Motor, Spring, Limit, 틀어짐, Open/Closed Rail, Loop Seam, 바디의 동적 재질량, 곡선 편집 및 재투영, 추적 분기 유지 등을 검사한다.

- `splineRailJointTests`, `splineRailJointWorldTests`, `splineRailSpeedTests`, `splineRailAngularSpeedTests`
- `splineRailRebaseTests`, `splineRailReprojectTests`, `splineRailRotationEnableTests`, `splineRailRotationRebaseTests` 등
- Sandbox Rail Inspector와 Terrain Test, Catmull-Rom Geometry 관련 회귀 테스트는 그대로 실행한다.

기존 테스트의 허용 오차·합격 기준은 변경하지 않는다.

## 3. 신규 `splineRailDeterminismTests`

`tests/determinism/spline_rail_joint_determinism_test.cpp`에 추가한다.

- 매번 독립 World 두 개를 초기화해 **열린 5점 곡선**, **닫힌 4점 순환 레일**을 240 Tick, `dt=1/60`, 4 substeps로 진행.
- 열린 곡선은 Motor → Spring → Motor 전환과 Limit, 닫힌 레일은 Follow-Tangent 회전과 Motor 방향 역전 시험.
- 매 Tick 공개 World/Body 스냅샷(R1a)에 Spline Rail 데이터의 `segmentIndex`, `parameter.t`, `length`, `referenceDistance`, `currentTranslation`, `currentSpeed`, `currentAngularSpeed`, `currentAngle`, `lateralError`, 접점·접선·법선·반력·토크·Motor·Spring Force를 명시적인 원시 binary32/64 비트로 추가.
- 동일 빌드의 두 World 결과는 **해시가 아니라 전체 bytes**를 비교. 추적 실패, 비유한 값, 최초 차이는 실제 테스트 실패로 보고.
- `--emit spline-<platform>.tsv`로 각 Tick/필드 raw-bit 관측값을 내보낸다. 주소, 핸들의 `worldToken`, padding을 비교하지 않는다.
- **정식 내부 전체 상태가 아님**: Spline Rail의 persistent impulse/상세 추적 이력, 모든 Body/Shape/Contact/Joint 및 Fluid 내부 값은 아직 직접 직렬화하지 않는다. 공개 관측 데이터에 한정된 결정론 검사다.

## 4. CI 교차 플랫폼 Gate

기존 `.github/workflows/determinism.yml`에서 Ubuntu/GCC와 Windows/MSVC의 `splineRailDeterminismTests`를 각각 빌드·실행하고 TSV를 artifact로 업로드한다.

- 두 파일의 **모든 줄**을 직접 비교하며, 최초 `scene/tick/field`와 두 원시 비트 문자열이 다르면 비교 작업을 실패시킨다.
- 별도 `spline-comparison.txt` 로그는 비교 성공·실패 모두 보관한다.
- 기존 R1c `worldReplayTests`의 **61개 스냅샷 비교**를 유지한다.
- 기존 연구용 표준 라이브러리 `libm.hypot` 2건과 `libm.acos` 1건의 교차 플랫폼 **비트 차이를 숨기거나 허용 오차로 바꾸지 않는다**. 따라서 기존 strict FP 전체 Gate는 적색으로 남을 수 있으며, 이때 새 Spline Gate의 성공/실패를 **구별해서** 보고한다.

## 5. 실제 첫 교차 플랫폼 불일치 (2026-11-24 KST)

첫 [R2a-4 CI #45227670332](https://github.com/buddas-bible/zonai/actions/runs/45227670332)에서 기존 R1c World Replay는 61/61 상태가 비트 동일했지만 **새로운 Spline Rail 검사에서 실제 Joint 물리 불일치를 처음 발견**했다.

- 열린 레일의 물리 관측값은 두 플랫폼에서 일치한 반면, 닫힌 레일은 Tick 0·1의 관측 상태가 일치하다가 **Tick 2의 `body[2].angularVelocity`에서 처음 달라졌다**. Ubuntu/GCC `0x3fe74256`, Windows/MSVC `0x3fe74257` (1 ULP).
- 동일 Tick 2의 Body `rotation.c/s`, `rail.tangent.x/y` 등 입력 관측값은 일치하지만 `rail.currentAngle`은 Ubuntu `0xbdd2c73e`, Windows `0xbdd2c74f`로 달랐고, 반력·토크와 이후 Body 회전으로 차이가 전파된다.
- 읽기 전용 `atan2` 및 `remainder` 진단을 추가한 [재검증 #45228590416](https://github.com/buddas-bible/zonai/actions/runs/45228590416)에서는 **닫힌 레일 Tick 1의 동일 회전 입력에 대한 `std::atan2(float)` 출력이 이미 1 ULP 달랐음**을 확인했다: Ubuntu `0x3c216e45`, Windows `0x3c216e44`. 원시 입력은 `y=0x3c216d9a`, `x=0x3f7ffcd3`로 양쪽 동일했다.
- 열린 레일에서도 Tick 4의 동일 접선 입력(`y=0xbd0ffd6b`, `x=0x3f7fd77e`)에 대한 `atan2` 진단 출력이 1 ULP 달랐다. 또한 닫힌 레일 Tick 2의 tangent `atan2` 차이를 확인했다.
- 이는 **실제 라이브러리 `atan2`의 플랫폼별 차이를 입증**한다. 다만 최초 Body 각속도 차이의 원인이 오직 이 함수 한 곳인지, 숨겨진 Joint Impulse/서브스텝 중간값도 기여하는지는 아직 분리되지 않았다. 해결됐다고 선언하지 않는다.
- 두 범주의 차이를 분리하기 위해 CI 비교기를 바꿔 **첫 권위 관측 상태 불일치와 첫 libm 진단 불일치를 모두 출력하고 엄격히 실패**하도록 했다. 정확한 3쌍의 입력/출력 비트도 FP 감사에 등록했다.

**후속 R2a-5:** 현대 Box2D의 결정을 재검토하여 정확도·범위·비트 동일성을 검증한 공통 `atan2` 및 각도 정규화 경로를 별도의 작은 PR로 구현하고, 우선 Spline Rail Follow-Tangent의 두 `std::atan2` 호출에 제한하여 적용한다. 이후 Warm Start/내부 Solver 캐시와 Substep 차이를 추가 스냅샷으로 검사한다. `std::remainder` 등 다른 libm 함수는 먼저 동일 입력의 실제 차이를 확인한다.

## 6. 합격 조건과 후속 단계

1. Windows/Ubuntu Debug의 전체 CTest 통과(기존 수치 허용 오차 불변).
2. 기존 Windows Release 선택적 회귀·정보성 성능 샘플 통과.
3. Spline Open/Loop의 241×2 Tick 샘플에서 모든 기록 필드의 정확한 비트 일치 확인. 실패하면 첫 divergence를 근거로 Math/Joint의 구체적인 연산을 분석하며, epsilon 우회는 허용하지 않는다.
4. 기존 R1c World Replay 정확한 바이트 일치 유지.
5. 이전 R2a-1 strict `libm` 감사 차이가 여전히 존재할 경우 **전체 PR을 녹색 또는 병합 준비 완료라고 선언하지 않는다**.

이 단계가 통과하더라도 엔진의 완전 결정론이 보장된 것은 아니다. 다음에는 남아 있는 Fluid의 `std::hypot` 및 Cut-cell 기하 함수를 **압력 안정성·체적 보존 테스트와 분리**하여 검증하고, `std::atan2` 등 다른 초월함수의 결정론 검사 및 내부 Solver 캐시 스냅샷 확장을 별도 PR로 진행한다.
