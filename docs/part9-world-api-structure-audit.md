# Part 9 — World / Public API / 파일 구조 감사

기준: master `a2748cb46eac6d5773990df098234632c93b65b2`.
Box2D main `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`, 2026-10-05 fetch로 확인함.

목표는 기존 World의 책임·소유권·API 경계와 Joint 연결 지점을 확인하고 필요한 정리만 하는 것이다. 앞선 Contact/Sensor 조회 감사와 달리 World의 storage, lifecycle, Step orchestration, wake/sleep, island 연결 및 파일 배치를 함께 검토했다. 하위 collision/solver 알고리즘을 전부 새로 감사하거나 Joint를 구현한 작업은 아니다.

## 비교 기준과 판단

Box2D의 [World storage](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/physics_world.h), [Step orchestration](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/physics_world.c), [Joint lifecycle](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/joint.c), [Island linkage](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/island.c)을 확인했다.

Box2D는 World가 sparse stable entity storage와 simulation storage를 소유하고, solver sets/constraint graph에 hot state를 배치하며, Contact와 Joint를 Body 및 island에 연결한다. 이는 ownership·identity·solver 작업 집합을 구분하기 위한 설계다. Zonai의 같은-index vector와 transient union-find island는 현재 serial 엔진의 의도적인 단순화로 유지한다. Joint 추가에 persistent island, scheduler, SIMD, custom allocator가 선행 조건인 것은 아니다.

| 판단 | 대상 | 근거와 처리 |
| --- | --- | --- |
| KEEP | World ownership와 stable handles | world lifetime token, slot/generation, body/sim/state 및 contact/sim 분리가 명확함. copy/move 제한 유지. |
| KEEP | Body/Shape/Contact lifecycle | 생성·파괴·refilter·mass 갱신은 owning World에서 list/proxy/pair와 함께 처리됨. 독립 subsystem 객체로 옮기지 않음. |
| KEEP | Step의 orchestration | 현재 Contact refresh → wake/island → prepare/substep solve → CCD → impulse store/transform commit → sleep/force 소비 → 최종 Contact/Sensor update 순서를 유지함. Box2D의 실제 pass 순서와 동일하다고 주장하지 않음. |
| REFACTOR | 공개 collision callback template | callback에 무관한 기존 Contact 갱신과 새 pair 생성이 template에 섞여 있었음. `updateExistingContact`, `createContactForPair` 두 private helper로 cpp에 이동함. |
| REFACTOR | world.h / world.cpp 탐색 순서 | 기존 정의 109개를 내용 그대로 보존하고 헤더 선언 순서로 재배치함. 같은 책임 구역에 `#pragma region`을 추가함. |
| REFACTOR | contact2의 파일 위치 | NarrowPhase 기하 결과가 아니라 World의 persistent pair·slot·Body edge record임. 내용 변경 없이 `src/dynamics/contact2.h`로 이동함. |
| REFACTOR | 진단 API의 borrowed references | GetBody/GetShape/AABB/tree view를 stable handle 또는 복사 snapshot으로 오인하지 않도록 수명 주석을 추가함. API 반환 타입은 유지함. |
| DEFER | shapeGeometry / runtime shape 헤더 분리 | `collision/shape.h`에 variant, geometry 계산과 runtime record가 함께 있음. 실제 Collision/BroadPhase/World 소비자가 있어 일괄 Dynamics 이동은 부적절함. 별도 geometry/runtime 분리가 필요한지 Part 10의 header 노출 검토에서 판단함. |
| DEFER | public/internal header packaging | 현재 CMake는 모든 header를 PUBLIC FILE_SET에 넣지만 설치/export 구조는 없음. 진단 사용자를 먼저 확인해야 하므로 이번에 include tree를 전면 재편하지 않음. |
| DEFER | 반복 scratch allocation / 파일 분할 | wake/sleep visited/stack, transient islands/constraints는 현재 단순한 방식임. 측정 없는 persistent cache나 `world*.cpp` 분할을 추가하지 않음. |

## callback 경계와 보존한 의미

`UpdateCollisions`의 template에는 stable slot 순회와 사용자 callable 호출만 남긴다. `broadPhase::UpdatePairs` 자체는 기존 C++ callable 구현을 유지한다. 기존 Contact는 자기 갱신 직후 callback을 받고, 새 pair callback은 해당 Contact 생성 직후 호출된다. 전체 refresh 뒤 일괄 dispatch로 바꾸지 않았으므로 callback의 읽기 전용 query가 보는 상태와 순서가 유지된다. 새 event buffer, `std::function`, callback 복사 또는 추가 heap allocation은 도입하지 않았다.

Free slot은 건너뛰고 fat AABB가 분리되면 Contact를 파괴한다. 재활용 가능하면 anchor cache로 갱신하고, 아니면 fresh NarrowPhase로 갱신한다. 기존 callback은 실제 touching에만 호출하며, speculative manifold도 포함하는 Contact data query 규칙과 구분한다. same-body/지원하지 않는 geometry pair 제외와 BroadPhase의 filter·sensor 제외도 유지한다.

Public handle 입력의 유효성은 기존 precondition이다. stale/null/foreign handle은 IsValid로 먼저 검사해야 한다. callback 중 mutation/reentry는 여전히 지원하지 않으며 runtime lock을 새로 추가하지 않았다. Contact data는 복사 snapshot, Sensor span은 다음 Step 전까지의 transient view, Body/Shape/AABB는 vector-backed borrowed reference다.

`contact2.h`를 직접 include하는 소비자는 새 경로 `dynamics/contact2.h`를 사용해야 한다. 저장소 source/test/sandbox의 기존 직접 include는 world.h 한 곳이었으며 함께 수정했다. 타입 이름·layout·Contact key 표현과 public World 함수 signature는 유지했다. 이전 Part 5 설계/계획의 옛 경로는 작성 당시 기록으로 남긴다.

## Joint 구현 시 반드시 연결할 지점

현재 구조가 Joint를 막는 것은 아니지만, Contact 전용 경로에 solver 함수 하나만 추가해서는 올바른 Joint lifecycle이 되지 않는다. 아래 변경은 Joint 구현 범위에서 함께 수행해야 한다. 이번에는 placeholder를 넣지 않는다.

1. **Identity / ownership:** world-aware jointId, stable Joint slot/free-list/generation, persistent impulse와 simulation 데이터. Body 파괴 시 연결된 Joint도 먼저 제거하고, slot 재사용과 cross-world handle을 검증함.
2. **Body edges / filtering:** Joint의 양쪽 Body 연결과 삭제 처리를 추가함. collideConnected=false 생성 시 기존 Body pair Contact 제거, 새 pair 생성 시 Joint 연결 검사, Joint 삭제 시 해당 Body pair가 다시 BroadPhase 후보가 되도록 처리함.
3. **Island / wake / sleep:** 현재 `BuildIslands`, `WakeBodyByIndex`, `SleepBodyByIndex`, `WakeSleepingBodiesFromContacts`는 Contact만 봄. Contact가 없어도 Joint로 연결된 Body가 같은 island에서 풀리고 함께 깨고 잠들도록 확장해야 함. Static 경유로 무관한 Dynamic Body를 합치지 않는 경계도 유지함.
4. **Solver orchestration:** 같은 subStepTime, bodyState delta/COM 좌표계를 사용해 Joint prepare·warm start·solve·relax·impulse store를 연결함. Distance Joint를 위한 최소 constraint부터 시작하고 모든 제약을 가상함수 기반 hierarchy로 통합하지 않음. Contact restitution은 Contact 전용 단계로 유지함.
5. **회귀/시각 확인:** Static–Dynamic과 Dynamic–Dynamic, 비중심 anchor, collision 없이 연결된 Body의 wake/sleep, Body/Joint 삭제·재사용, collideConnected 재평가를 검증하고 Sandbox에서 연결과 반응을 확인함.

첫 Joint 후보는 Distance Joint다. Contact에서 사용한 effective mass, 누적 impulse, warm start와 soft constraint를 거리 제약으로 확장하기 좋은 작은 구현 범위다. 정확한 옵션·public definition은 구현 단계의 설계에서 결정한다.

## 검증

기존 `worldQuerySensorTests`에 callback timing 회귀를 추가했다. 기존/새 pair의 즉시 callback, callback 내부의 read-only query, move-only callable의 상태 유지, recycled/fresh touching, speculative query와 callback 구분, 분리된 Contact 삭제를 검사한다. 이 test는 추출 전 구현에서도 통과했다.

Release에서 임시로 새 pair 생성을 기존 Contact callback보다 먼저 실행하도록 바꾸자 `existing callback delayed until new pairs`로 실패했다. header를 원본 bytes로 복구하고 rebuild한 뒤 동일 test의 통과를 확인했다. Mutation은 병합하지 않는다.

기존 cpp 정의 109개의 signature/body를 기준 commit과 개별 비교해 그대로 보존됐음을 확인했다. 추가된 두 helper는 기존 header의 callback-independent 처리이며 cpp 전체 정의 111개의 순서는 헤더와 맞췄다. Contact header 이동은 동일 내용이고 상대 문서 링크 15개와 region 짝도 확인했다.

Windows MSVC, sandbox OFF에서 Debug/Release 전체 build 및 CTest 각각 38/38 통과했다. 별도 리뷰 및 Windows/Ubuntu CI의 최종 결과는 이 변경의 PR에 기록한다. 기존 Release assert 테스트의 검증 한계는 유지되며, 이번에는 일괄 변환하지 않았다.

## 다음 작업

원래 로드맵의 **Part 10 — Tests / Sandbox / Build / Docs**에서 현재 사용/빌드 흐름과 public/internal header 노출, 큰 테스트·Sandbox 파일의 관리상 문제를 확인한다. 실제 이득이 있는 정리만 하고 선택적인 테스트 보강은 주개발을 막지 않도록 보류한다. 이후 최종 통합 확인을 거쳐 Joint 구현으로 진행한다.
