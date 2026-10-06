# Tests

This directory contains automated tests for `zonai`.

Tests should be added alongside implementation work, with emphasis on:

- math correctness
- collision edge cases
- constraint and solver regression cases
- deterministic reproduction of previously fixed bugs

Prefer small, focused tests that make failures easy to diagnose.

현재 CTest target은 43개이며 실행 방법은 [루트 문서](../README.md)를 따른다. CI는 Windows/Ubuntu Debug 전체와 Windows Release 아래 13개를 build/run한다.

`broadPhaseLifecycleTests`, `bodyShapeContactLifecycleTests`, `contactConstraintTests`, `restitutionTests`, `islandTests`, `islandSleepCcdTests`, `worldQuerySensorTests`, `contactCacheTests`, `distanceJointTests`, `distanceJointWorldTests`, `mouseJointTests`, `mouseJointWorldTests`, `sandboxDemoTests`는 NDEBUG와 무관하게 runtime check가 유지된다. 새 회귀 검사가 Release에서도 필요하면 이 패턴을 따른다.

나머지 중 29개는 runtime assert 기반이며 Release에서 검사가 제거된다. `bodyTypeTests`는 compile-time static_assert 검사다. 특히 assert 안의 상태 변경 호출도 Release에서 생략될 수 있다. 전체 Release 통과 수를 Debug와 동등하게 해석하지 않으며 기존 테스트의 일괄 변환은 현재 개발 순서에서 보류한다.

`sandboxDemoTests`는 Sandbox를 끈 구성에서도 데모 교체·World 수명·재시작·시간 진행·입력 취소를 검사한다. ImGui target이 있는 Windows 구성에서는 같은 target에 실제 데모 Controls/Canvas draw의 headless frame 검사도 포함한다. 창/GPU/실제 OS 입력과 화면 배치를 검증하는 검사는 아니다.

Distance spring 회귀는 기존 두 Distance target과 Sandbox target에 포함한다. Implicit Euler 축 응답, 주파수·감쇠·중력 평형, Hertz 0, cache/wake와 live mode 전환을 검사한다. UI는 slider text 입력의 범위 제한, spring checkbox와 radial impulse를 실제 ImGui frame으로 확인한다. [검증 범위](../docs/distance-spring.md)를 따른다.
