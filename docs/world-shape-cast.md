# World ShapeCast

`world::castShape` / `world::castShapeClosest`는 기존 ShapeCast(GJK 거리 반복)와
BroadPhase의 세 DynamicTree를 연결하는 World 공간 선형 sweep API다.

```cpp
worldShapeCastInput2 input{};
input.geometry = circle2{ {}, 0.5f };
input.transform.position = { 0.0f, 1.0f };
input.translation = { 10.0f, 0.0f };

const worldShapeCastResult2 hit = simulation.castShapeClosest( input );

simulation.castShape( input, queryFilter{},
    [&]( shapeId id, vec2 point, vec2 normal, float fraction )
    {
        return fraction; // 현재 충돌 지점까지만 이후 후보를 검사
    } );
```

- 입력 Shape는 Circle, Capsule, Convex Polygon, Segment. 도형은 `transform`의 자세를
  유지하며 `translation` 방향으로 움직인다. ChainSegment를 이동 도형으로 넣는 경우는
  현재 지원하지 않는다.
- 고정 타깃은 Circle/Capsule/Polygon/Segment/ChainSegment이며 Sensor도 탐색한다.
  Chain 타깃에는 edge 오른쪽의 앞면 조건을 적용한다. 앞면 판정은 간단한
  normal/초기 중심 기준이며 ghost-vertex 접합점의 상세 ownership 검사는 하지 않는다.
- `fraction`은 전체 translation에 대한 0~1 비율이다. 기존 `ShapeCast`의
  LINEAR_SLOP 기반 접촉 허용차 및 끝 fraction 제외 규칙을 유지한다.
- callback: -1 무시, 0 전체 종료, 반환한 양수 fraction으로 남은 sweep 단축,
  1 전체 길이 유지. 후보 탐색은 거리순이 아니다.
- 처음부터 겹치면 일반 조회에는 fraction=0, zero normal로 보고하며, closest는
  처음 겹친 후보를 건너뛴다. `canEncroach=true`는 기존 narrowphase ShapeCast의
  가까운 초기 상태에서 조금 더 접근할 수 있는 모드를 사용한다.
- QueryFilter의 양방향 category/mask를 사용하며 groupIndex / collisionMatrix는
  공간 조회에 적용하지 않는다. 반환 `shapeId`의 수명은 `IsValid`로 확인한다.
- 순수 읽기 조회이며 callback에서 World/Tree/Shape를 수정하거나 `Step`에 재진입하면 안 된다.
  복수 객체의 다른 프레임 캐싱이나 terrain 연결 지점 처리, 회전 sweep은 이 API 범위가 아니다.

`worldShapeCastTests`: 세 Body Tree, Sensor, 필터, hit 위치·normal·fraction,
초기 겹침·encroachment, callback, world handle 수명, transform 변경/파괴,
Circle/Capsule/Polygon/Chain, 유효하지 않은 입력을 Release에서도 검사한다.


## BoxCast를 사용하는 BroadPhase 후보 탐색

World ShapeCast는 이제 DynamicTree의 `BoxCast`를 사용한다.
시작 도형의 tight AABB 중심을 광선으로 이동시키고, Tree node의 AABB를
시작 도형의 half extent만큼 확장해 swept AABB가 실제 통과하는 공간만 조회한다.
대각선 경로에서는 시작과 끝 AABB의 합집합에 들어오지만 이동 경로에서는
벗어난 노드들을 좁은 단계(GJK) 전에 건너뛸 수 있다.

- Tree 자체는 AABB 후보만 판정한다. 최종 hit은 기존 ShapeCast(GJK)와
  QueryFilter/Chain 앞면 검사로 확정하므로 공개 결과 규약은 동일하다.
- callback이 반환한 현재 fraction을 세 Tree 사이에 공유하여 뒤쪽 후보를 줄인다.
- BoxCast에서도 Tree 중간에 수정하거나 Step을 재진입할 수 없다.
- 정량적인 성능 향상은 장면과 트리 형상에 따라 다르며 벤치마크 결과로 주장하지 않는다.

`dynamicTreeBoxCastTests`는 독립적인 swept-AABB 시간 구간 oracle과
무작위/대각선/정지 축/clip/이동/삭제/Rebuild 케이스를 대조한다.
기존 `worldShapeCastTests` 및 `worldSpatialQueryStressTests`도
동일한 hit 의미를 유지하는지 확인한다.


## Root sweep 영역 조기 제외

`DynamicTree::BoxCast`는 비용이 큰 노드별 slab 교차 검사 전에
현재 `maxFraction`까지의 swept-AABB 합집합과 Tree root AABB가
겹치지 않으면 즉시 종료한다. 시작 위치가 root와 겹치는 경우에는
기존 순회를 바로 수행한다. root 바깥에서 시작한 경우 이동 끝점과 비교 범위는
float 반올림으로 실제 교차를 놓치지 않도록 double 정밀도로 계산하고 경계를
보수적으로 확장한다. 시작 겹침·진행 방향·clip된 구간의 후보는 유지된다.

성능 효과는 World ShapeCast 및 Tree 벤치마크로 확인하며, 특히
Root bounds 밖의 miss가 이른 종료의 직접 대상이다. 이동 중인
도형의 실제 geometry와 GJK/solver 수학은 변경하지 않는다.
