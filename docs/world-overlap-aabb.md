# World AABB 공간 조회

`world::overlapAABB( bounds, queryFilter, callback )`는 현재 World의 Static/Kinematic/Dynamic
DynamicTree를 모두 조회하여 Shape handle을 전달한다. 간단한 호출에는 필터를 생략할 수 있다.

```cpp
simulation.overlapAABB( { { -2.0f, -2.0f }, { 2.0f, 2.0f } },
    queryFilter{ 1, ~std::uint64_t{ 0 } },
    [&]( shapeId id )
    {
        // callback의 false 반환은 전체 조회 중단.
        return true;
    } );
```

- 현재 World의 float 좌표로 조회한다. 유효하지 않은 AABB는 결과 없이 반환한다.
- Box2D와 같이 **AABB 후보** 조회이다. fat proxy에만 닿는 경우는 제외하지만,
  각 Shape의 speculative AABB를 검사하므로 실제 geometry의 접촉 보장은 아니다.
- Shape의 category/mask와 Query의 category/mask를 양방향 검사한다. 기존
  collisionMatrix와 groupIndex는 공간 조회에 영향을 주지 않는다. 기본 필터는 category 1,
  mask 전부이며 Shape의 mask가 category 1을 거절하면 기본 조회에서도 제외된다.
- Sensor 포함, Shape당 결과 한 번, 호출 순서 미보장. callback이 false를 반환하면 중단한다.
  callback 중 World/Tree/Shape 상태 변경과 Step/재진입은 허용하지 않는다.
- 결과는 world lifetime/generation이 있는 shapeId 값이다. 보관 후 다시 사용할 때 IsValid로 확인한다.
- 기존 Tree 내부 스택 크기 제한과 단정밀도 좌표의 한계는 그대로 유지한다.
  origin-relative large-world 질의, geometry overlap 및 ray/shape cast는 별도 작업이다.

자동 테스트: `worldOverlapAABBTests`는 세 body 유형·복수 Shape·Sensor·양방향 필터·
기본 필터·collisionMatrix/group 독립성·fat 오탐·즉시 pose 갱신·삭제/slot 재사용·
world handle 구분·callback 조기 중단/비복사·잘못된 bounds를 runtime check로 확인한다.
