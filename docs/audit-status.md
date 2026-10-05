# Zonai 감사 현황과 남은 작업

기준: 2026-10-05, master `f9c5c50fbaf8f76d2ebd861842c7db0523d46e3a`.
Part 4 이후 감사의 Box2D 기준은 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`이다. 이 문서는 저장소 이력·API·테스트·기존 보고서를 종합하며, 모든 함수를 새로 재감사한 결과는 아니다.

## 병합된 감사

아래 병합 커밋 9개가 모두 기준 master의 ancestor임을 확인했다. 완료는 각 감사의 구현된 경로와 검증 범위에 한정하며, Box2D 전체 기능 구현 완료를 뜻하지 않는다. 추가 통합 감사에는 새로운 Part 번호를 부여하지 않는다.

| 범위 | 상태와 주요 결과 | 병합 근거 / 상세 기록 |
| --- | --- | --- |
| Part 1 — Math / Geometry | 기존 감사 병합. 큰 좌표에서 polygon 면적·centroid 정밀도 보강. 이번 회차는 이력과 테스트 확인이며 함수별 재비교는 하지 않았다. | [9a13b40](https://github.com/buddas-bible/zonai/commit/9a13b401d907f8b85e4648cd167f81bb0cfe53fd) |
| Part 2 — Collision primitives / GJK / ShapeCast / TOI | 기존 감사 병합. 공용 segment distance와 GJK 회귀 검증 보강. 이번 회차는 이력과 테스트 확인이며 함수별 재비교는 하지 않았다. | [e1b3cb3](https://github.com/buddas-bible/zonai/commit/e1b3cb3a4cf5e70beddfa1e91fc9759326ca1e6b) |
| Part 3 — NarrowPhase | 기존 감사 병합. feature ID, 두 점 polygon clipping, 공용 segment distance 정리. 이번 회차는 이력과 테스트 확인이며 함수별 재비교는 하지 않았다. | [afbf7f8](https://github.com/buddas-bible/zonai/commit/afbf7f871f03a5d98fde427b13a0fb429daae5e4) |
| Part 4 — DynamicTree / BroadPhase | 재감사 병합. node/free-list/mapping, SAH·rotation·rebuild, pair lifecycle 및 brute-force 비교. | `21ffa62`, [PR #12](https://github.com/buddas-bible/zonai/pull/12), [보고서](box2d-broadphase-parity-audit.md) |
| Part 5 — Body / Shape / Contact | 재감사 병합. world lifetime token, copy/move 제한, speculative query와 수명/list 회귀 검증. | `15df78a`, [PR #10](https://github.com/buddas-bible/zonai/pull/10), [보고서](part5-body-shape-contact-audit.md) |
| Part 6 — Contact Solver | 재감사 병합. scalar solver 유지, restitution·compression·momentum·energy 및 Release 검사. | `d3cee81`, [PR #11](https://github.com/buddas-bible/zonai/pull/11), [보고서](part6-contact-solver-audit.md) |
| Island / Sleep / CCD 통합 | 재감사 병합. 연결된 sleep/wake, static 경계, substep delta/force, TOI와 sensor 경계. | `09da0ee`, [PR #13](https://github.com/buddas-bible/zonai/pull/13), [보고서](island-sleep-ccd-audit.md) |
| World Query / Sensor API | 재감사 병합. 제공 중인 Contact/Sensor 조회, 결과 제한, filter, 과거 ID와 이벤트 수명. | `4571ef0`, [PR #14](https://github.com/buddas-bible/zonai/pull/14), [보고서](world-query-sensor-audit.md) |
| Contact recycling/cache | 재감사 병합. 누적 pose 기준, 임계값·회전·fast 제외, feature impulse와 mass 갱신. | `f9c5c50`, [PR #15](https://github.com/buddas-bible/zonai/pull/15), [보고서](contact-cache-audit.md) |

기존 보고서의 “다음 Part”, “CI 대기”, “미병합 브랜치” 문장은 작성 당시의 기록이다. 현재 순서는 이 문서를 따른다. 이전 설계/계획의 미체크 항목만으로 구현이 미완성이라고 판정하지 않고 실제 source·test·merge 이력과 함께 확인한다.

## 현재 검증과 공백

이 기준에서 로컬 library/test 전체를 다시 빌드하고 Debug 38/38, Release 38/38 CTest 통과를 확인했다. 로컬 설정은 Windows MSVC, sandbox OFF다. 원격 CI는 Ubuntu Debug library/test, Windows Debug library/test와 sandbox 빌드, Windows Release 아래 8개 target을 검증한다. 각 감사의 최종 원격 결과는 해당 PR에 기록돼 있고, 이 문서 변경의 CI 결과는 별도 PR에 기록한다. Sandbox UI 실행이나 시각 검증을 새로 수행한 것은 아니다.

| 테스트 종류 | 현재 구성 | 해석 |
| --- | --- | --- |
| Release에서 유지되는 runtime check | `broadPhaseLifecycleTests`, `bodyShapeContactLifecycleTests`, `contactConstraintTests`, `restitutionTests`, `islandTests`, `islandSleepCcdTests`, `worldQuerySensorTests`, `contactCacheTests` | Windows Release CI에서 8개를 명시적으로 build/run한다. |
| runtime assert를 사용하는 기존 테스트 | 29개 source/target: Math/Geometry, Collision/NarrowPhase, 기존 tree/hashSet, Body/World 등 | Debug 검증은 유효하다. NDEBUG에서 assert가 제거되므로 Release 38/38만으로 동등한 검증을 주장할 수 없다. |
| compile-time 검사 | `bodyTypeTests`는 static_assert만 사용. 다른 일부 테스트에도 static_assert가 있음. | compile-time 검사는 Release에서도 유지된다. |

특히 `tests/collision/broadphase/hash_set_test.cpp`의 `assert( set.Add(...) )`, `assert( removalSet.Remove(...) )`는 Release에서 검사뿐 아니라 상태 변경 호출 자체도 사라진다. 이것은 검증 공백의 구체적 근거이며 엔진 알고리즘 결함으로 분류하지 않는다.

## 후속 작업 우선순위

1. **다음 작업: Part 1 Math/Geometry의 Release 검증 보강.** 기존 vec2/rot2/transform2 및 circle/segment/capsule/polygon 테스트의 유효한 회귀 기대값을 runtime check로 유지하고 static_assert는 그대로 둔다. 새 framework나 중복 테스트를 만들지 않는다. 의미 있는 mutation 하나가 Release에서도 실패함을 확인한 뒤 원복하고, Debug 전체·Release 대상·두 플랫폼 CI를 검증한다. 이 회차에서 구현한 작업은 아니다.
2. **Collision/NarrowPhase 및 나머지 기존 테스트의 Release 공백 해소.** Part 2/3 primitive와 feature 회귀부터 진행하고, hashSet 등 assert 안의 side effect를 실제 실행되는 검사로 옮긴다. CI 대상은 보강된 test에 맞춰 늘린다. 단순히 실행 수를 늘리거나 NDEBUG를 제거하는 것만으로 완료 처리하지 않는다.
3. **DynamicTree/BroadPhase의 고정 stack 경계 재현.** Part 4 보고서에 기록된 512-entry stack과 Release push 생략/assert-only 경로를, 허용된 입력에서 실제로 도달 가능한지 먼저 확인한다. 재현된 실패와 작은 regression이 있을 때만 복구 방식이나 명시적 입력 상한을 결정한다. 극단 깊이의 안전성이 현재 검증됐다고 주장하지 않는다.
4. **CCD/Sensor 보장 범위 결정 후 회귀 확대.** moving target의 전체 trajectory union은 현재 후보 bounds에 포함되지 않으며 모든 교차 sweep을 보장하지 않는다. continuous sensor hit budget은 8이다. 기존 통합 감사의 제한으로 남기고, 범위를 확대할 필요가 정해지면 좁은 실패 사례부터 검증한다.

cache의 작은 feature/normal 근사, empty-manifold refresh 지연, 16-bit generation wrap, 큰 좌표 정밀도 및 전역 이벤트 순서는 개별 보고서의 제한으로 유지한다. 여기서 새로운 정확성 결함이나 무제한 보장을 확정하지 않는다. 성능 변경은 측정 근거 없이 추가하지 않는다.

## 기능 구현과 감사의 구분

현재 source는 serial 2D engine이다. World-level spatial overlap/ray/shape query와 query filter, joint/chain, body type/enable 전환 API, persistent solver sets/constraint graph, worker/SIMD scheduling은 현재 공개 API 또는 subsystem에 없다. Primitive `ShapeCast` 구현은 World/Tree cast API 구현을 의미하지 않는다. Rolling resistance, contact hit event, custom filter/material callback도 별도 기능이다.

`src/README.md`의 개발 순서는 core utility → math → 2D이며, 3D와 다른 simulation 영역은 실제 구현을 시작할 때 추가하도록 돼 있다. 저장소에는 위 기능을 바로 다음 번호의 Part로 지정한 확정 계획이 없으므로 임의의 Part 7 기능 구현을 시작하지 않는다. 우선 위 검증 공백을 줄인 뒤 기능 요구에 따라 범위를 정한다.

## 브랜치 보존 상태

기준 remote head는 `master`와 `debug/speculative-contact-timeout` 두 개다. Part 4/5/6 및 이후 완료된 감사 브랜치는 병합 후 삭제됐으며, 그 작업 커밋은 master에 포함돼 있다.

debug tip `342ccb250a8940def074ed8f78f917f477f9654b`는 master의 ancestor가 아니다. master에 없는 커밋은 CTest `--timeout 15`를 임시 추가한 1개이며, 이번 통합 현황 작업에서 병합하거나 삭제하지 않는다. 현재 master CI의 job timeout은 10분이다. 추후 정리할 때도 보존 근거를 확인하고, 의미 없는 변경을 이력 보존만을 위해 master에 넣지 않는다.

이 문서 작업 브랜치도 최종 검증·리뷰 후 병합하고, tip의 master 포함과 동일 tree를 확인한 뒤 로컬·원격에서 삭제한다. 위 후속 작업이나 debug 변경은 이번 문서 작업에 포함하지 않는다.
