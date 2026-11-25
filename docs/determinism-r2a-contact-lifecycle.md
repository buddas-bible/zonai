# R2a-9 — Contact 재활용·수명주기와 Sleep/Wake Island 결정론

> 기준: 미병합 [PR #340](https://github.com/buddas-bible/zonai/pull/340) head `a75bc293`. 순차 병합 #329 → #330 → #331 → #332 → #333 → #334 → #335 → #336 → #337 → #338 → #339 → #340 → R2a-9. 다른 Fluid 작업 및 `master`는 변경하지 않는다.

## 배경

R2a-8은 90 Tick의 다중 Island 장면에서 Contact/Island의 transient 제약, Restitution, ContactSim 영속 manifold/impulse를 양 플랫폼에서 389,049행 비트 단위로 일치 확인했다. 그러나 Contact **생성·파괴·free-list 재사용**, `simplexCache2`, recycler의 기준 Pose/Anchor, Body Sleep/Wake 때문에 Island graph가 바뀌는 시점은 별도로 추적하지 않았다.

## 테스트 전용 스냅샷 확장

`jointSubstepTraceAccess::captureContactLifecycle`은 기존 테스트 전용 `world` friend 접근을 재사용하며 정식 물리 API를 추가하지 않는다.

- Active 및 Free Contact 슬롯을 **Stable Index 순서**로 검사. `generation`, `freeNext`, `contactFreeList_`, live count, recycled-this-update count 및 **free-list 순서**를 비트 명명 필드로 기록한다.
- Free 슬롯에서 `contactSim2`의 Contact ID, Manifold, Impulse, recycler flag가 지워졌는지 검증. free-list 사이클, 활성 슬롯 누락 및 Contact count 불일치는 즉시 실패한다.
- Active 슬롯의 `enableRecycling`, `recycleCacheValid`, Body/Shape 연결, simplex GJK `count`/feature index, recycler 기준 relative transform·cached rotations 및 접점별 A/B anchor·separation을 **명시적인 부동소수점 원시 비트**로 기록한다. 포인터, padding, 임의 메모리 덤프를 기록하지 않는다.
- 이 값은 **검사 대상 장면의 영속 상태만** 의미하며 BroadPhase/CCD/Fluid의 모든 캐시까지 포함하지 않는다.

## 회귀 장면

`contactLifecycleDeterminismTests`는 두 시나리오를 실행한다. 기존 `joint_substep_trace.h`의 23개 단계/Step(Prepare, 각 4 substeps × 5 단계, Restitution, Store) 계측과 필드별 strict comparison을 재사용한다. **동일한 명령을 전달한 독립적인 세 World**(2개 계측, 하나 미계측)를 매 Tick까지 실행하고, 공개 World/Body 및 추가 Contact lifecycle 스냅샷을 원시 바이트로 비교한다.

1. **`recycleReuse`** — Static 바닥과 동적 상자의 실제 Manifold를 만든다. 동일 Pose에 대한 반복 `UpdateCollisions`로 recycling이 실제 발생하는지 확인한다. 이후 shape 제거/재생성 및 Fat AABB 분리를 통해 Contact 파괴와 Free 슬롯 재사용을 수행한다. 이전 `contactId`는 `IsValid`에서 거부해야 하고, 새 Contact는 **같은 슬롯+새 generation**으로 생성돼야 한다. 생성 시 recycler 허용 여부를 snapshot하므로, 비허용 새 Contact는 narrowphase를 건너뛰면 안 된다. recycled count/Reuse epoch도 스냅샷에 포함한다.
2. **`sleepWake`** — 바닥에 닿은 동적 Box, 그 위에 맞닿은 두 번째 Box, 별도 위치의 세 번째 Box를 배치한다. `SetBodyAwake(false)`는 연결된 두 동적 Body를 함께 재우되 분리된 Box는 깨어 있어야 한다. 이때 Island의 flat Body 목록에서 잠든 Body가 사라져야 한다. `SetBodyAwake(true)`를 연결 Body 하나에 호출하면 둘 다 기상하고 다시 Island graph에 참여해야 한다. 이후 World sleep 비활성화 시 모든 non-static Body가 깨어나는지 검사한다.

## 검증 조건

- Windows/MSVC, Ubuntu/GCC 양쪽 Debug 전체 CTest와 `contactLifecycleDeterminismTests` 통과.
- 두 장면의 `--emit` TSV를 **장면/Tick/서브스텝/단계/필드/비트** 순서로 완전 비교하고 첫 차이를 바로 보고·실패 처리한다. 1 ULP라도 불일치면 실패하며 테스트 가정을 완화해 통과시키지 않는다.
- `contactIslandDeterminismTests`, 표준 Joint/Spline Rail 기존 Substep 기록, R1c 원본 World Replay 61/61 그대로 유지.
- 기존 `libm.hypot`×2, `libm.railAtan2`×3, `libm.acos`×1 표준 라이브러리 연구용 비트 차이는 변경/숨김 없이 여전히 기록하고 CI aggregate를 실패로 유지한다.
- 프로덕션의 `Contact`, `Island`, `Sleep/Wake`, `BroadPhase`, `Fluid` 알고리즘/수식에 **변경 없음**. 기존 테스트용 Friend의 검사 범위만 확장한다.
- Test-build-only observer의 비용은 `ZONAI_BUILD_TESTS=OFF`에서 컴파일되지 않는다.

## 후속 및 브랜치 정리

R2a-9 검증 이후 **#329~#341 stacked Draft PR의 실제 dependency/CI/독립 diff**를 재검토하여 순차 병합 가능성을 판단한다. Legacy `libm` 샘플과 물리 결정론 CI의 관계를 서로 구분하여 문서화하되, 실패를 무시하거나 CI를 녹색으로 꾸미지 않는다. 전혀 다른 맥락의 Fluid Cut-cell 압력 스파이크(#324)는 별도 리스크로 유지한다.
