# R2a-7b — Joint 서브스텝 중간 상태와 최초 불일치 지점

> [R2a-7 PR #338](https://github.com/buddas-bible/zonai/pull/338) `3153b4f9` 기반 미병합 stacked Draft. 병합 순서 #329→#330→#331→#332→#333→#334→#335→#336→#337→#338→R2a-7b.

## 실제 엔진의 단계

`world::Step`의 Joint constraint는 **Step 단위로 한 번 Prepare**하고, 각 서브스텝에서 **Velocity Integration → Warm Start → Bias Solve → Position Integration → Relax Solve**를 수행한다. 모든 서브스텝이 끝난 뒤 restitution·CCD 처리를 마치고 **JointSim에 한 번 Store**한다. R2a-7b는 이 순서를 옮기거나 재해석하지 않고 그대로 관측한다.

4 substeps에서 관측 지점은 다음과 같다.

```
Prepare (Step당 1회, substep=-1)
  ├─ substep 0: IntegratedVelocity → WarmStart → SolveBias → IntegratedPosition → SolveRelax
  ├─ substep 1: IntegratedVelocity → WarmStart → SolveBias → IntegratedPosition → SolveRelax
  ├─ substep 2: IntegratedVelocity → WarmStart → SolveBias → IntegratedPosition → SolveRelax
  └─ substep 3: IntegratedVelocity → WarmStart → SolveBias → IntegratedPosition → SolveRelax
Store (Step당 1회, substep=4)
```

**한 Tick당 총 22개 관측 지점**이며, 기존 공개 World/Body 및 Joint 영속 캐시의 Tick 경계 스냅샷도 유지한다. Store 이전의 transient Constraint impulse와 Store 이후의 영속 cache 값을 별도로 기록한다.

## 계측 구조 및 프로덕션 무영향

- `ZONAI_BUILD_TESTS=ON`에서만 CMake가 `ZONAI_TEST_SUBSTEP_TRACE=1`을 `zonai` 라이브러리와 테스트 타깃에 **PUBLIC 전파**한다. 이는 World의 조건부 레이아웃이 양쪽 컴파일 단위에서 일치하기 위해 필요하다.
- `ZONAI_BUILD_TESTS=OFF`에서는 테스트 observer 필드 및 모든 callback 호출이 **전처리 시 완전히 제거**된다.
- `world.h`의 테스트 구조체 friend 및 조건부 callback 포인터만 추가. Solver의 Impulse 공식·Warm Start 조건·Island 순서·Step 데이터/서브스텝 계산·Contact/CCD/Fluid는 변경하지 않는다.
- `world.cpp`의 callback은 각 기존 단계 **완료 이후**에 const `World` 및 const `jointConstraint` span으로 실행한다. 현재 서브스텝의 임시 `Body State`를 포함한 숫자값을 읽기만 한다.
- 사용자 노출 API는 추가하지 않는다. Test-only 구조체 `jointSubstepTraceAccess`는 관측 callback을 설치·해제하며 trace 저장은 테스트 측에서 수행한다.
- 테스트 장면마다 **세 번째 Observer 없는 World**를 독립 실행해, 계측된 World의 공개 상태와 영속 cache가 정확히 같은지 Tick별로 검사한다. 따라서 같은 callback을 설치한 두 World끼리만 같은 상태가 되는 오류를 피한다.

## 실제 기록 필드

- 모든 단계: `stage`, `subStepIndex`, 해당 Step의 활성 Joint Constraint 수.
- 추적 Body별: 임시 `bodyState.linearVelocity.{x,y}`, `angularVelocity`, `deltaPosition.{x,y}`, `deltaRotation.{c,s}`.
- 해당 Joint별 transient constraint: 종류별 선형·회전·모터·스프링·limit 누적 Impulse, effective angular mass, 회전/축·anchor, Spline tracker의 구간/거리 offset·각도 반력 등 **명시적으로 명명된 필드**.
- `Store` 단계: R2a-7의 **persistent `jointSim` 캐시** 전체와 복사된 `subStepTime`.
- 크기·주소·variant padding·World lifetime token이나 무작위 자료구조 순서는 비교하지 않는다. IEEE float/double의 원시 비트를 그대로 기록한다.

## 비교 및 수용 기준

`standardJointDeterminismTests`는 Revolute, Prismatic, Weld, Motor 4장면×180 Step, `splineRailDeterminismTests`는 Open/Closed 레일 2장면×240 Step를 실행한다.

- 단계 누락/중복, 올바르지 않은 서브스텝 번호 또는 순서, tracked Joint가 solver-active Constraint 목록에서 빠지면 즉시 실패한다.
- 동일 프로세스의 독립 World 두 개에 대해 **각 단계의 모든 snapshot bytes**를 비교하고, observer 없는 별도 World와 최종 Tick 상태도 비교한다.
- Ubuntu/GCC 및 Windows/MSVC에서 동일 고정 입력을 실행하고 두 TSV를 엄격 비교한다. 최초 차이는 **scene / Tick / substep / stage / field + raw bits**로 출력한다.
- 기존 R1c World Replay 61/61 일치 및 R2a-7까지의 영속 Impulse/공개 상태 비교를 모두 보존한다.
- 기존 표준 라이브러리 `libm.hypot` 2건, `libm.railAtan2` 3건, `libm.acos` 1건의 실제 FP 차이는 계속 실패로 표시한다. 허용 오차·skip·적색 Gate 완화는 없다.
- Windows/Ubuntu CMake Debug 전체 CTest, Windows Release 기존 선택적 검증을 유지한다.

## 제한과 다음 작업

이번 계측은 선택한 다섯 Joint 종류의 **active Constraint 및 BodyState**만 기록한다. 모든 Contact solver 내부 중간값, sleeping/waking 상태 전이, 더 많은 Joint 종류, Fluid Grid 배열을 포함하는 권위 상태 전체는 아직 아니다. 실행자가 다른 FP 환경/CPU와 LLVM 등에서 동일함도 미검증이다.

다음에는 반복 계산의 개별 iteration과 Contact/Island 내부 ordering을 분리해 비교하고, 이후 Fluid Cut-cell 압력 스파이크(#324)와 양방향 유체-강체 안정성의 독립 회귀 검증으로 이어간다.
