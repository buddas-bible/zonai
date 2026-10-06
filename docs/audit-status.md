# Zonai 감사 현황과 남은 작업

기준: 2026-10-06, master `8fb0ae5c6fa29fc1115be16f823f1f4a69739b12`에서 시작한 최종 통합 확인.
Part 4 이후 감사의 Box2D 기준은 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`이다. 이 문서는 저장소 이력·API·테스트·기존 보고서를 종합하며, 모든 함수를 새로 재감사한 결과는 아니다.

## 병합된 감사

기존 감사 병합 커밋 11개가 모두 기준 master의 ancestor임을 확인했다. 완료는 각 감사의 구현된 경로와 검증 범위에 한정하며, Box2D 전체 기능 구현 완료를 뜻하지 않는다. 원래 대화의 로드맵은 Part 1–10 → 최종 통합 확인 → Joint였다. Island/Sleep/CCD 통합은 Part 7/8의 구현 경로를 다루고, 조회·cache 감사는 관련 범위를 보완한다.

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
| Part 9 — World / Public API / 파일 구조 | 감사 병합. World ownership·Step·API 경계와 Joint 연결 지점 검토, template 처리 분리·정의 순서·Contact record 위치 정리. | `4097e75`, [PR #17](https://github.com/buddas-bible/zonai/pull/17), [보고서](part9-world-api-structure-audit.md) |
| Part 10 — Tests / Sandbox / Build / Docs | 감사 병합. 플랫폼 기본값, 외부 consumer의 dependency 경로/UTF-8 설정, 현재 빌드/사용 문서 정리. | `8fb0ae5`, [PR #18](https://github.com/buddas-bible/zonai/pull/18), [보고서](part10-tests-sandbox-build-docs-audit.md) |
| 최종 통합 확인 | 이번 확인. 현재 경계와 Joint 확장 요구를 연결하고 학습용 첫 Distance Joint의 설계 제안을 작성함. Joint 구현 완료를 뜻하지 않음. | [보고서](final-integration-joint-readiness.md) |

기존 보고서의 “다음 Part”, “CI 대기”, “미병합 브랜치” 문장은 작성 당시의 기록이다. 현재 순서는 이 문서를 따른다. 이전 설계/계획의 미체크 항목만으로 구현이 미완성이라고 판정하지 않고 실제 source·test·merge 이력과 함께 확인한다.

## 기본 Distance Joint

2026-10-06, `c8c8d71` 이후 첫 Joint를 구현했다. 고정 거리 scalar constraint와 Body 연결/삭제, Joint handle, collision filter, Island/wake/sleep, substep solver를 함께 연결한다. [구현·학습·검증 기록](basic-distance-joint.md)의 범위와 한계를 따른다. Spring/limit/motor는 후속 학습 단계다. Windows Sandbox 포함 Debug/Release 전체 build와 40/40 CTest 실행, 새 두 runtime target 및 9가지 mutation 회귀 검사를 확인했다. 원격 CI의 Windows Release 대상은 10개로 늘렸다. 아래 38개/8개 기록은 최종 통합 확인 당시의 기준이다.

## 선택형 Sandbox

기본 Distance Joint 이후 Playground와 Distance Pendulum을 독립 데모로 분리했다. 장면 선택·재시작은 World와 UI 상태를 새로 만들고, 공통 host가 camera·시간 진행·입력 전달을 담당한다. 데모별 A/D, Space, S와 mouse impulse 실험을 제공한다. 클릭 impulse는 Mouse Joint가 아니며 다음 구현에서 picking과 target constraint를 학습한다. [구조와 방향](sandbox-demos.md)을 따른다. Windows Sandbox 포함 Debug/Release 전체 build와 41/41 CTest 실행을 확인했다. `sandboxDemoTests`는 Release에서도 검사하며 Windows ImGui headless frame도 포함한다. 실제 창/GPU/OS 입력과 화면 배치는 별도 확인이 필요하다. 원격 Windows Release CI 대상은 11개다.

## 현재 검증과 공백

이 기준에서 Windows MSVC 기본 설정으로 Sandbox를 포함해 전체를 다시 빌드하고 Debug 38/38, Release 38/38 CTest 실행 통과를 확인했다. 외부 consumer와 library-only 구성도 build/run했다. 원격 CI는 Ubuntu Debug library/test, Windows Debug library/test와 sandbox 빌드, Windows Release 아래 8개 target을 검증한다. 각 감사의 최종 원격 결과는 해당 PR에 기록돼 있고, 이번 최종 통합 확인의 CI 결과도 해당 PR에 기록한다. Sandbox UI 실행이나 시각 검증을 새로 수행한 것은 아니다.

| 테스트 종류 | 현재 구성 | 해석 |
| --- | --- | --- |
| Release에서 유지되는 runtime check | `broadPhaseLifecycleTests`, `bodyShapeContactLifecycleTests`, `contactConstraintTests`, `restitutionTests`, `islandTests`, `islandSleepCcdTests`, `worldQuerySensorTests`, `contactCacheTests` | Windows Release CI에서 8개를 명시적으로 build/run한다. |
| runtime assert를 사용하는 기존 테스트 | 29개 source/target: Math/Geometry, Collision/NarrowPhase, 기존 tree/hashSet, Body/World 등 | Debug 검증은 유효하다. NDEBUG에서 assert가 제거되므로 Release 38/38만으로 동등한 검증을 주장할 수 없다. |
| compile-time 검사 | `bodyTypeTests`는 static_assert만 사용. 다른 일부 테스트에도 static_assert가 있음. | compile-time 검사는 Release에서도 유지된다. |

특히 `tests/collision/broadphase/hash_set_test.cpp`의 `assert( set.Add(...) )`, `assert( removalSet.Remove(...) )`는 Release에서 검사뿐 아니라 상태 변경 호출 자체도 사라진다. 이것은 검증 공백의 구체적 근거이며 엔진 알고리즘 결함으로 분류하지 않는다.

## 다음 개발 순서

1. **Part 1–10 감사 병합.** 구현된 경로와 API/파일 책임, 실제 build/use 흐름을 검토했다. Box2D 전체 기능과의 차이는 개별 보고서에 남긴다.
2. **최종 통합 확인.** ID/수명, pair ownership, Island/wake/sleep, Step/CCD/Sensor 연결을 확인하고 첫 Joint의 학습·검증 순서를 구체화했다. 더 넓은 전면 재감사를 선행 조건으로 추가하지 않는다.
3. **기본 Distance Joint 구현.** 고정 거리 제약과 Body/lifecycle/filter/island/wake/sleep/solver 통합을 구현했다. Spring/limit/motor를 추가하기 전에 Sandbox 진자에서 거리·회전·impulse의 관계를 관찰할 수 있다. 선택형 Sandbox에서 독립된 진자로 거리·회전·impulse의 관계를 관찰한다.

4. **선택형 Sandbox 구현.** 한 프로그램에서 독립 데모와 입력·설정을 선택한다. 다음은 Mouse Joint의 picking·target·soft constraint이며, 이후 spring/limit/motor → ragdoll/자동차/조나이 연결 데모로 이어간다. 천·유체·soft body·voxel/파괴·terrain은 실제 subsystem을 시작할 때 같은 데모 host에 추가한다.

Release 검증 보강은 사용자의 우선순위에 따라 보류한다. 테스트 실행 자체와 해당 변경을 확인할 회귀 검증은 계속 수행하지만, 전체 테스트 변환이 Joint 개발의 선행 조건은 아니다.

## 보류한 검증과 범위 확대

1. **Part 1 Math/Geometry의 Release 검증 보강.** 기존 vec2/rot2/transform2 및 circle/segment/capsule/polygon 테스트의 회귀 기대값을 runtime check로 유지할 수 있으나 이번 주개발에서는 보류한다. 현재 Release 통과 수만으로 Debug와 동등한 검증을 주장하지 않는다.
2. **Collision/NarrowPhase 및 나머지 기존 테스트의 Release 공백 해소.** Part 2/3 primitive와 feature 회귀부터 진행하고, hashSet 등 assert 안의 side effect를 실제 실행되는 검사로 옮긴다. CI 대상은 보강된 test에 맞춰 늘린다. 단순히 실행 수를 늘리거나 NDEBUG를 제거하는 것만으로 완료 처리하지 않는다.
3. **DynamicTree/BroadPhase의 고정 stack 경계 재현.** Part 4 보고서에 기록된 512-entry stack과 Release push 생략/assert-only 경로를, 허용된 입력에서 실제로 도달 가능한지 먼저 확인한다. 재현된 실패와 작은 regression이 있을 때만 복구 방식이나 명시적 입력 상한을 결정한다. 극단 깊이의 안전성이 현재 검증됐다고 주장하지 않는다.
4. **CCD/Sensor 보장 범위 결정 후 회귀 확대.** moving target의 전체 trajectory union은 현재 후보 bounds에 포함되지 않으며 모든 교차 sweep을 보장하지 않는다. continuous sensor hit budget은 8이다. 기존 통합 감사의 제한으로 남기고, 범위를 확대할 필요가 정해지면 좁은 실패 사례부터 검증한다.

cache의 작은 feature/normal 근사, empty-manifold refresh 지연, 16-bit generation wrap, 큰 좌표 정밀도 및 전역 이벤트 순서는 개별 보고서의 제한으로 유지한다. 여기서 새로운 정확성 결함이나 무제한 보장을 확정하지 않는다. 성능 변경은 측정 근거 없이 추가하지 않는다.

## 기능 구현과 감사의 구분

현재 source는 serial 2D engine이다. World-level spatial overlap/ray/shape query와 query filter, 다른 Joint 종류/chain, body type/enable 전환 API, persistent solver sets/constraint graph, worker/SIMD scheduling은 현재 공개 API 또는 subsystem에 없다. Primitive `ShapeCast` 구현은 World/Tree cast API 구현을 의미하지 않는다. Rolling resistance, contact hit event, custom filter/material callback도 별도 기능이다.

`src/README.md`의 개발 순서는 core utility → math → 2D이며, 3D와 다른 simulation 영역은 실제 구현을 시작할 때 추가하도록 돼 있다. 이전 현황 문서는 대화에 있던 10파트 감사 이후 Joint 개발 순서를 반영하지 못했다. 위 다음 개발 순서로 바로잡으며, 그 밖의 미구현 기능은 요구에 따라 범위를 정한다.

## 브랜치 보존 상태

기준 remote head는 `master`와 `debug/speculative-contact-timeout` 두 개다. Part 4/5/6 및 이후 완료된 감사 브랜치는 병합 후 삭제됐으며, 그 작업 커밋은 master에 포함돼 있다.

debug tip `342ccb250a8940def074ed8f78f917f477f9654b`는 master의 ancestor가 아니다. master에 없는 커밋은 CTest `--timeout 15`를 임시 추가한 1개이며, 이번 통합 현황 작업에서 병합하거나 삭제하지 않는다. 현재 master CI의 job timeout은 10분이다. 추후 정리할 때도 보존 근거를 확인하고, 의미 없는 변경을 이력 보존만을 위해 master에 넣지 않는다.

완료된 작업 브랜치는 최종 검증·리뷰 후 병합하고, tip의 master 포함과 동일 tree를 확인한 뒤 로컬·원격에서 삭제한다. 최종 통합 확인에는 Joint 구현이 없었고, 후속 기본 Distance Joint 작업도 debug 변경을 포함하지 않는다.
