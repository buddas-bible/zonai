# Part 5 Body / Shape / Contact Audit Design

## Goal

Part 5는 `body`, `shape`, `contact`의 수명과 소유권이 Box2D의 의도와 맞게 유지되는지 다시 검증하고, 실제 correctness 문제만 수정하는 감사 단계다. 구조를 미관상 갈아엎거나 Part 6 solver를 재설계하지 않는다.

현재 `master`를 기준으로 작업하며, 기존 `refactor/body-shape-contact-audit`와 `refactor/body-shape-contact-reaudit`는 참고 자료로만 사용한다. 기존 브랜치의 변경은 커밋 단위로 검증하며 통째로 merge하지 않는다.

## Success Criteria

Part 5 완료 조건은 다음과 같다.

- public body / shape / contact handle이 잘못된 world, 삭제된 slot, 재사용된 generation을 올바르게 거부한다.
- body 생성/삭제에서 연결된 shape, contact, broad-phase proxy와 stable slot 상태가 일관되게 정리된다.
- shape 생성/삭제에서 body intrusive shape list, mass data, sensor storage, broad-phase proxy, 관련 contact가 일관되게 갱신된다.
- contact 생성/삭제에서 양쪽 body intrusive contact list, pairSet, contact/contactSim stable slot이 깨지지 않는다.
- persistent contact와 touching contact의 의미가 분리되어 public query가 실제 touching contact만 노출한다.
- runtime filter 변경, transform 변경, slot 재사용 같은 파괴적인 경로에 regression test가 있다.
- 전체 테스트와 빌드가 통과한다.
- Part 6 solver 수식과 solve 순서는 변경하지 않는다.

## Scope

### Included

- `src/dynamics/id.h`
- `src/dynamics/body.h`
- `src/dynamics/bodyShape.h`
- `src/dynamics/bodySim.h`
- `src/dynamics/bodyState.h`
- `src/dynamics/world.h`
- `src/dynamics/world.cpp`
- `src/collision/shape.h`
- `src/collision/narrowphase/contact2.h`
- `src/dynamics/contactSim2.h`
- `tests/dynamics/world_test.cpp`
- 필요한 경우 Part 5 전용 추가 테스트 파일

### Excluded

- contact constraint 수식 변경
- warm start 계산 변경
- bias / relaxation / restitution 순서 변경
- joint / solver set / constraint graph 신규 도입
- 파일 구조를 미관상 재배치하는 refactor

## Design Principles

### 1. Current master is the source of truth

Part 4 DynamicTree / BroadPhase 재감사가 이미 `master`에 병합되어 있으므로 Part 5는 최신 `master`에서 새 브랜치로 진행한다. 과거 Part 5 브랜치는 설계 아이디어와 회귀 테스트 후보를 찾는 용도로만 사용한다.

### 2. Keep the current cold/hot split

현재 구조는 다음 책임 분리가 이미 잘 되어 있다.

- `body`: stable slot, generation, shape/contact ownership, sleep 관련 cold state
- `bodySim`: transform, mass inverse, force, CCD 등 simulation hot state
- `bodyState`: velocity state
- `contact2`: persistent lifetime, shape ids, body edges, generation
- `contactSim2`: manifold, cached impulse, recycling cache, solver-facing hot state

이 구조는 유지한다. 감사의 초점은 데이터 배치 변경이 아니라 수명과 갱신 순서의 correctness다.

### 3. Public handles must encode world ownership

현재 master의 public ID는 `index1 + generation`만 가지고 있어 다른 `world`의 동일 slot/generation handle을 우연히 유효하게 볼 수 있다.

Part 5에서는 public `bodyId`, `shapeId`, `contactId`에 owning world lifetime을 구분하는 opaque token을 추가하는 방향을 우선 채택한다. 구현 시 다음을 만족해야 한다.

- 각 `world` 인스턴스는 0이 아닌 고유 lifetime token을 가진다.
- `MakeBodyId`, `MakeShapeId`, `MakeContactId`가 token을 넣는다.
- `IsValid`는 token을 먼저 확인한다.
- null ID의 의미는 `index1 == 0`으로 유지한다.
- token 충돌과 world lifetime 재사용 가능성을 피한다.
- `world` copy/move 정책은 public handle 안정성과 일치해야 한다.

기존 re-audit 브랜치의 atomic token 방식은 참고하되, 필요성과 비용을 다시 검증한 뒤 최소 구현을 선택한다.

### 4. Body lifecycle invariants

#### CreateBody

- stable slot을 새로 만들거나 free-list에서 재사용한다.
- 재사용 시 generation은 이전 handle과 달라야 한다.
- `body`, `bodySim`, `bodyState`는 같은 stable index를 공유한다.
- 이전 slot의 force, velocity, transform, contact/shape head가 남지 않아야 한다.

#### DestroyBody

삭제 순서는 외부 dangling reference가 남지 않도록 다음 불변식을 지킨다.

1. 연결된 contact를 모두 제거한다.
2. 연결된 shape를 모두 제거한다.
3. shape proxy와 sensor storage가 모두 정리되어야 한다.
4. body simulation/state를 free 상태로 초기화한다.
5. generation을 증가시키고 slot을 free-list로 반환한다.

삭제 중 intrusive list를 순회할 때 현재 노드를 파괴해도 다음 노드를 잃지 않아야 한다.

### 5. Shape lifecycle and ownership invariants

Body와 shape의 관계는 현재 index 기반 doubly linked list를 유지한다.

- `body.headShapeId`가 첫 shape를 가리킨다.
- 각 shape는 `prevShapeId`, `nextShapeId`, `bodyId`를 가진다.
- head의 `prevShapeId`는 항상 null이다.
- `body.shapeCount`는 실제 연결된 shape 수와 일치한다.

#### CreateShape

- owning body가 valid해야 한다.
- shape stable slot을 초기화한다.
- body shape list에 한 번만 연결한다.
- world AABB와 fat AABB를 만든다.
- broad-phase proxy를 만든다.
- sensor이면 dense sensor storage를 연결한다.
- Dynamic body의 mass/extents를 갱신한다.

#### DestroyShape

- 해당 shape가 참여한 contact를 모두 제거한다.
- sensor storage와 pending events를 정리한다.
- broad-phase proxy를 제거한다.
- body shape list에서 안전하게 unlink한다.
- Dynamic body mass/extents를 다시 계산한다.
- generation 증가 후 free-list로 반환한다.

중간 shape 삭제, head 삭제, 마지막 shape 삭제를 각각 테스트한다.

### 6. Contact lifecycle invariants

Contact는 broad-phase pair가 유지되는 동안 존재하는 persistent object다. touching은 narrow-phase manifold의 실제 접촉 상태다.

#### CreateContact

- 같은 body의 shape pair는 생성하지 않는다.
- unsupported geometry pair는 생성하지 않는다.
- stable contact slot과 matching `contactSim2` slot을 초기화한다.
- 두 body의 intrusive contact list에 각각 하나의 edge를 연결한다.
- pairSet과 contact lifetime이 일치해야 한다.
- recycling 설정은 생성 시점 정책에 따라 고정한다.

#### DestroyContact

- 양쪽 body contact list에서 edge를 정확히 제거한다.
- head / middle / tail 삭제가 모두 가능해야 한다.
- body `contactCount`가 실제 edge 수와 일치해야 한다.
- pairSet에서 pair를 제거한다.
- contactSim cache를 free 상태로 초기화한다.
- generation 증가 후 free-list에 반환한다.

`contactKey = (contactId << 1) | edgeIndex` 규칙은 유지하며 key decode/encode의 invariant를 테스트한다.

### 7. Persistent contact vs touching semantics

Public API 의미를 다음처럼 고정한다.

- Contact capacity/count 계열은 broad-phase상 persistent contact 수를 보수적으로 사용할 수 있다.
- `GetBodyContactData`와 `GetShapeContactData`는 실제 touching manifold만 반환한다.
- speculative/persistent contact가 아직 실제 접촉하지 않는 경우 public contact data에서 제외한다.
- touching 상태가 끝나도 fat AABB overlap이 남아 contact 자체가 유지될 수 있다.

과거 re-audit 브랜치의 speculative contact public 노출 실험은 채택하지 않는다.

### 8. Runtime mutation semantics

#### SetShapeFilter

- 기존 contact는 즉시 제거한다.
- broad-phase가 다음 update에서 새 filter 기준으로 pair를 다시 평가할 수 있게 한다.
- stale pair/contact가 남지 않아야 한다.

#### SetBodyTransform

- body transform과 연결 shape의 speculative/fat AABB가 함께 갱신된다.
- tree proxy가 새 bounds를 반영한다.
- 기존 contact의 lifetime 판단과 다음 narrow-phase update가 일관되어야 한다.

## Testing Strategy

Part 5 수정은 regression test를 먼저 추가한 뒤 구현한다.

필수 테스트:

1. 다른 world의 body/shape/contact handle은 invalid다.
2. body 삭제 후 같은 slot 재사용 시 old bodyId는 invalid다.
3. shape 삭제 후 같은 slot 재사용 시 old shapeId는 invalid다.
4. contact 삭제 후 같은 slot 재사용 시 old contactId는 invalid다.
5. DestroyBody가 shape/proxy/contact를 모두 제거한다.
6. shape list의 head/middle/tail 삭제가 링크와 count를 보존한다.
7. contact list의 head/middle/tail 삭제가 양쪽 body 링크와 count를 보존한다.
8. SetShapeFilter가 기존 contact를 제거하고 이후 pair를 재평가한다.
9. persistent speculative contact는 capacity에는 포함될 수 있지만 contact data에는 노출되지 않는다.
10. touching 전환 후 contact data에 나타난다.
11. touching이 끝났지만 persistent pair가 남은 경우 contact data에서는 사라진다.
12. slot 재사용 뒤 이전 simulation/cache 값이 남지 않는다.

테스트는 기존 `tests/dynamics/world_test.cpp` 패턴을 우선 따르며, 파일이 과도하게 비대해질 경우에만 Part 5 lifecycle test를 별도 파일로 분리한다.

## Style Constraints

- lowerCamelCase 사용
- 현재 private member naming 유지
- header 선언 순서와 cpp 정의 순서를 맞춤
- 기존 `#pragma region` 사용 패턴을 유지
- 불필요한 줄바꿈을 추가하지 않음
- 주석은 "무엇"보다 Box2D 의도와 invariant의 "왜"를 간단히 설명
- C 패턴을 그대로 복사하지 않고 현재 C++20 구조 안에서 동일한 의도를 보존

## Implementation Order

1. public handle/world ownership regression test
2. world lifetime-aware ID validation
3. body lifecycle audit + tests
4. shape ownership/proxy lifecycle audit + tests
5. contact intrusive list/lifecycle audit + tests
6. filter/touching/query semantics audit + tests
7. comments/style/declaration-order cleanup only where touched
8. full test/build verification
9. Part 5 audit report 작성

## Non-goals

이번 Part 5에서 다음은 하지 않는다.

- solver architecture 재설계
- solver set / constraint graph 도입
- 새로운 collision feature 추가
- API를 Box2D와 이름까지 1:1로 맞추기
- 성능 측정 없이 자료구조를 교체하기
- correctness와 관계없는 대규모 폴더 이동

## Merge Readiness

Part 5 브랜치는 다음 조건을 모두 만족할 때만 master merge 대상으로 본다.

- 새 regression test가 모두 통과함
- 기존 test가 모두 통과함
- Debug/Release build가 통과함
- Part 6 solver 동작에 의도하지 않은 diff가 없음
- 기존 Part 5 브랜치에서 가져온 변경은 각각 필요성이 설명 가능함
- 감사 보고서에 남은 known issue가 명시되어 있음
