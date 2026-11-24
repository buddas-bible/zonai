# R2a-7 — Joint 영속 Impulse/Warm Start 캐시 비트 감사

> **이 단계는 R2a-6 [#337](https://github.com/buddas-bible/zonai/pull/337)의 커밋 `b401f9e`에서 출발한 stacked Draft PR이다.** 병합 순서는 #329 → #330 → #331 → #332 → #333 → #334 → #335 → #336 → #337 → R2a-7. `master`는 수정하지 않는다.

## 감사 배경

R2a-6까지 World/Body 및 공개 Joint 반력을 Windows/MSVC와 Ubuntu/GCC 간 비트 수준으로 비교했다. 표준 Joint 네 장면의 69,685 기록 행은 일치했고, Spline Rail 두 장면의 공개 물리 상태도 일치했다. 그러나 공개 반력은 종류별로 `impulse / subStepTime` 또는 **여러 impulse의 합**으로 계산되기 때문에, **개별 저장 캐시의 차이가 총합에서 가려질 수 있다**.

실제 다음 Step의 Warm Start 입력값은 `world::jointSims_`의 종류별 `*JointSim2` 구조체에 유지된다. 이번에는 이 영속 데이터를 **메모리 덤프가 아닌 명시적인 필드별 원시 IEEE-754 비트**로 비교한다.

## 프로덕션에 추가한 유일한 연결

`src/dynamics/world.h`에서 `zonai::determinism::test::jointCacheSnapshotAccess`를 **friend 구조체로 선언**한다.

- 테스트에만 존재하는 `tests/determinism/joint_cache_snapshot.h`가 private `jointSims_`와 stable `joints_`에 **읽기 전용**으로 접근한다.
- 구조체 레이아웃, 메서드 가상 호출, 새로운 debug getter, 엔진 내 추가 메모리·연산, Warm Start 실행 순서를 추가하지 않는다.
- 정식 physics API에 캐시를 노출하거나 운영 코드가 캐시를 수정할 수 있게 하는 방식이 아니다.
- 단, C++ friend는 컴파일 시 접근을 허용하는 장치로서 보안 경계가 아니다. 이 테스트 구조체를 소스 측에서 구현하는 코드가 바뀌면 컴파일 차원에서 보호하지 못한다.

## 기록 데이터

기존 `stateSnapshotWriter`의 canonical field encoding을 사용한다. 저장시 `World token`·포인터·padding·`std::variant` 전체 바이트를 제외한다. 추적할 `jointId`는 stable index 및 generation 순으로 정렬한다. 중복 슬롯과 무효/다른 World/삭제된 Joint 핸들은 거부한다.

| Joint 종류 | 영속 상태를 직접 기록하는 필드 |
| --- | --- |
| Revolute | 2D anchor impulse, lower/upper limit impulse, motor impulse, subStepTime, reference angle/limit/motor 조건 |
| Prismatic | 2D coupled impulse, spring/limit/motor impulse, subStepTime, 축·타깃·limit 조건 |
| Weld | 2D linear impulse, angular impulse, subStepTime, 기준 각도·강성 |
| Motor | 독립된 linear velocity/spring impulse, angular velocity/spring impulse, subStepTime, 기준 각도 |
| Spline Rail | 2D impulse, spring/limit/motor impulse, subStepTime, parameter·tracking, 누적 거리·offset, 회전 설정, 보존 반력 |

또한 stable slot의 `index1`, `generation`, Body graph index, `collideConnected` 및 World 총 Joint 수를 기록한다. `float`는 `std::bit_cast<uint32_t>`, `double`은 `std::bit_cast<uint64_t>`를 사용하며, 비유한 값은 실패 처리한다.

**범위 한계:** 위 다섯 종류의 **persistent cache 필드**만 포함한다. 임의 Joint의 전체 제어 파라미터, 단기 Constraint 객체, World Step 내부의 **각 substep 중간 속도/impulse**, Contact/Island/Broadphase/Fluid 내부 state는 아직 직접 캡처하지 않는다. 이를 영속 World 상태 전체의 직렬화라고 선언하면 안 된다.

## 검증

- 기존 `standardJointDeterminismTests` 및 `splineRailDeterminismTests`의 매 Tick 공개 상태에 **영속 Joint Cache Snapshot**을 연결한다.
- 각 장면에서 독립적인 두 World의 전체 bytes를 비교. Windows/GCC가 서로 기록한 `--emit` TSV를 strict 비교해 첫 `scene/tick/cache.joint[slot].field`와 차이 비트를 표시한다. 기존 61-Tick World Replay 및 math fingerprint는 보존.
- `jointCacheSnapshotTests` 추가: 모터가 실제 비영 Impulse를 저장하는지, 저장된 `subStepTime`이 기대값인지, motor 속도 변경 후 motor impulse만 0으로 바뀌는지, 다른 World/무효/중복/삭제된 핸들이 거부되는지, slot 재사용 시 generation이 변하는지 테스트한다.
- `tests/CMakeLists.txt`에 CTest 등록. CI의 Ubuntu/Windows Replay 양쪽에서 신규 테스트를 명시적으로 실행하고, 전체 CMake 회귀도 수행한다.
- 기존 **표준 라이브러리 연구용 원시 비트 차이 6건**은 비트 비교에서 계속 실패하도록 남긴다. 차이를 허용 오차로 조정하지 않는다. **검증이 성공하더라도 전체 엔진 결정론 또는 완전한 내부 상태 같음을 의미하지 않는다.**

## 다음 작업

R2a-7b에서 각 substep 경계의 **단기 Constraint 및 Body State 캐시**를 observer 방식으로 계측하고 실행 순서·재사용·Sleep/Wake, Joint 생성·파괴를 확대한다. 다만 필드별 직렬화를 런타임 기본 경로로 넣어 비용을 증가시키지 않고, 테스트 전용 계측의 범위를 검토한다. 그 이후 Fluid의 Cut-cell/압력 안정성 문제와 별도로 결정론 감사 범위를 확장한다.
