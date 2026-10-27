# World RayCast

기존 세 DynamicTree의 slab ray traversal과 정확한 Shape geometry 교차를 연결한다.

```cpp
simulation.castRay( { 0.0f, 0.0f }, { 10.0f, 0.0f }, queryFilter{},
    [&]( shapeId id, vec2 point, vec2 normal, float fraction )
    {
        return fraction; // 현재 지점으로 clip하여 가장 가까운 hit만 유지
    } );

const worldRayCastResult2 closest = simulation.castRayClosest( { 0.0f, 0.0f }, { 10.0f, 0.0f } );
```

- `translation`은 시작점에서 끝점까지의 **변위**이고 `fraction`은 0~1이다.
- 콜백의 `-1`은 무시, `0`은 즉시 종료, `fraction`은 구간 단축,
  `1`은 기존 길이를 유지한다. 방문 순서에는 의미가 없다.
- Tree의 fat AABB는 후보 필터일 뿐이다. 최종 hit은 Body local의 정확한 Circle,
  Capsule, Convex Polygon(rounded 포함), 양면 Segment, 앞면 ChainSegment와 계산한다.
- 시작점이 solid 내부이면 hit을 반환하지 않는다. 한쪽 방향 Chain의 앞면은 edge 오른쪽이다.
- Query Filter는 AABB 조회와 같은 양방향 category/mask를 사용하고
  groupIndex와 collisionMatrix는 사용하지 않는다. Sensor를 포함한다.
- 월드/트리/도형을 조회 callback 중 변경하지 않는다. 반환 handle의 수명은 IsValid로 검증한다.
- 잘못된 ray와 영벡터는 결과 없이 반환한다. 기존 Tree의 고정 stack 및 float 좌표
  정밀도 경계를 그대로 따른다.

검증은 `worldRayCastTests`에서 원·캡슐·다각형·둥근 다각형·양면/단면
세그먼트의 교차점/법선/클리핑, 이동된 Body, 세 트리, 센서, 필터,
최근접 hit, callback 규칙, 수명, 잘못된 입력을 runtime check로 검사한다.
