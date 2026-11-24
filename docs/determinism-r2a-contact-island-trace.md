# R2a-8 — Contact/Island 실행 순서와 접점 Impulse 결정론

> 미병합 [R2a-7b PR #339](https://github.com/buddas-bible/zonai/pull/339), commit `10a381c`를 기준으로 한 후속 작업이다. 병합 순서 #329 → #330 → #331 → #332 → #333 → #334 → #335 → #336 → #337 → #338 → #339 → R2a-8을 유지한다.

## 검증 목적

기존 R2a-7b는 Revolute, Prismatic, Weld, Motor, Spline Rail의 transient Joint Constraint와 임시 Body State를 **Prepare → 각 substep의 Velocity/WarmStart/Bias/Position/Relax → Store** 순서로 비교했다. 실제 Contact Solver는 이와 동시에 실행되며 `BuildIslands`의 **flat Contact ID 순서**와 접점 feature ID/Manifold를 이용해 Impulse를 적용한다. 이번 단계는 이 누락 범위를 테스트 전용으로 보강한다.

## 실제 실행 순서

`world::Step`는 UpdateCollisions/BuildIslands 이후 Contact 및 Joint Constraint를 **Step당 한 번 Prepare**하고, 각 substep에서 Velocity Integrate, Warm Start(각 Island의 Joint → Contact), Bias Solve(Joint → Contact), Position Integrate, Relax Solve(Joint → Contact)를 수행한다.

모든 substep 이후 Contact Restitution을 실행한 뒤 최종 속도 제한·CCD를 진행하고, 마지막에 Contact/Joint Impulse를 persistent cache에 Store한다.

따라서 4-substep Step의 기록 지점은 **23개**다.

```
Prepare
  substep 0: IntegratedVelocity → WarmStart → SolveBias → IntegratedPosition → SolveRelax
  substep 1: IntegratedVelocity → WarmStart → SolveBias → IntegratedPosition → SolveRelax
  substep 2: IntegratedVelocity → WarmStart → SolveBias → IntegratedPosition → SolveRelax
  substep 3: IntegratedVelocity → WarmStart → SolveBias → IntegratedPosition → SolveRelax
Restitution
Store
```

## 추가 코드

- `world.h/world.cpp`의 기존 테스트 전용 `ZONAI_TEST_SUBSTEP_TRACE` observer가 `std::span<const contactConstraint2>`와 `const islandGraph2&`를 받도록 확장했다. Observer는 read-only이고 테스트 비활성 빌드에서 완전히 제외된다.
- `joint_substep_trace.h`의 recorder에 `captureContactAndIsland` 옵션을 둔다. 기존 Joint 전용 테스트는 false이므로 결과 필드 증가 없이 Restitution 단계만 추가된다.
- Island별 Body/Contact/Joint flat 시작 인덱스와 크기, **flat ID 배열의 실제 실행 순서**, Constraint와 해당 ContactSim의 stable ID 일치 여부를 기록·검사한다.
- 각 Contact Constraint의 Normal, Friction, Restitution, Softness, pointCount, Normal/Tangent Effective Mass, Base Separation, Relative Normal Velocity, Normal/Tangent/Total/Restitution Impulse를 **접점별로 원시 비트**로 기록한다. Manifold feature ID와 Contact slot generation도 함께 기록한다.
- Store 시점에는 실제 persistent ContactSim의 point ID, local manifold, normal/tangent impulse cache를 기록한다. 별도로 각 Tick의 최종 충돌 갱신 이후 persistent ContactSim 캐시를 기록한다.
- `contactIslandDeterminismTests`는 정적 바닥과 두 구역의 동적 상자 3개로 **최소 2개 Island, 복수 Contact, 2점 Manifold, 실제 Normal/Tangent Impulse**를 생성하도록 구성한다. 결정론 테스트는 두 개의 독립 관측 World를 각 90 Tick, `dt=1/60`, 4 substeps로 계산하고, 제3의 observer 없는 World와도 최종 authoritative Body/Contact 캐시 bytes를 비교한다. Coverage가 없는 장면은 명시적으로 실패한다.
- `--emit FILE.tsv` 출력은 `scene.tick.substep.stage.field` 경로로 원시 비트를 직렬화한다. Windows/MSVC와 Ubuntu/GCC의 같은 장면을 비교하고 **최초 차이의 정확한 장면/Tick/서브스텝/단계/필드**를 보고하며 CI를 실패시킨다.

## 보존되는 조건과 제한

- solver/normal/tangent/restitution 공식, Warm Start 계산, Contact/Island 정렬, BroadPhase, CCD, Fluid 수치 업데이트는 수정하지 않는다.
- 기존 `standardJointDeterminismTests` / `splineRailDeterminismTests`는 Restitution 단계 추가 외에는 같은 필드/물리 결과를 유지하며, 제3의 observer 없는 World와 비교하는 테스트도 그대로 보존한다.
- 기존 R1c 61/61 World Replay, 기존 연구용 libc 표준 수학 함수 비트 차이 6건(2 hypot, 3 rail atan2, 1 acos)을 숨기지 않는다. Test runner 의존 허용오차는 도입하지 않는다.
- 이번에 측정하는 것은 테스트 장면의 Contact/Island 및 일부 persistent ContactSim 상태다. 모든 Contact recycling 데이터·Simplex 캐시, 모든 Joint 변형, 전체 Fluid Grid, 실제 다른 프로세서 FP 환경·컴파일러의 결정론을 증명하지 않는다.
- 현재 코드에서 **Contact Restitution은 모든 substep 이후 한 번 실행**되므로 별도 단계로 표시한다. Store 관측은 CCD 후, 최종 `UpdateCollisions` 이전의 persistent cache를 기록하며 Tick 경계 관측은 UpdateCollisions 이후다. 두 의미를 혼동하지 않는다.

## 검증 및 후속

Windows/Ubuntu 전체 Debug CTest, Windows Release 기존 선택적 테스트, Contact/Island 첫 차이 리포트, 기존 Joint trace와 61-Tick Replay 결과를 확인한다. 실패하면 첫 `field`의 raw bits를 기반으로 Contact solver 계산 또는 Island ordering의 실제 원인을 분석하고, epsilon/스킵으로 감추지 않는다.

다음은 Contact/Joint별 **Solver iteration 안쪽**과 Sleep/Wake/Island 재편성, Contact slot 재사용 및 다양한 형상·충돌 속도 범위의 회귀를 더 넓히는 것이다. Fluid Cut-cell 압력 스파이크와 양방향 coupling 안정성은 별도 정량 회귀로 다룬다.
