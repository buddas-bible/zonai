# World OverlapShape

`world::overlapShape( geometry, transform, queryFilter, callback )`는 World 좌표로
배치한 convex query geometry와 겹치는 기존 Shape를 반환한다.

```cpp
simulation.overlapShape( capsule2{ { -1.0f, 0.0f }, { 1.0f, 0.0f }, 0.3f },
    transform2{ { 2.0f, 0.0f }, {} }, queryFilter{},
    [&]( shapeId id )
    {
        // false는 즉시 종료, true는 계속 탐색.
        return true;
    } );
```

## 범위와 정확성

- BroadPhase에서는 세 Body 유형(Static, Kinematic, Dynamic)의 기존 DynamicTree와
  실제 shape.aabb를 사용하여 후보를 걸러낸다. 이후 `ShapeDistance` GJK와
  도형 radius를 계산하여 **실제 convex geometry가 겹치거나 접촉한** 도형만 보고한다.
  접선 접촉은 float 반올림으로 생긴 미소 분리 거리도 감지하도록 기존 Sensor와 같은
  `10 * float epsilon` 허용 오차를 쓴다(임의의 물리적 간격 확장이 아님).
- Query는 Circle, Capsule, Convex Polygon, Segment와 ChainSegment(owned edge)다.
  Segment/Chain은 넓이가 0이어도 표면에 정확히 접하면 겹침으로 처리한다.
  Chain ghost vertex와 앞면/뒷면 ownership은 이 **대칭 거리 조회**에는 적용하지 않는다.
  체인 앞면 충돌이나 manifold 판정과는 별개의 공간 조회다.
- Sensor도 포함하며 query category/mask를 target의 category/mask와 양방향으로 검사한다.
  groupIndex와 collisionMatrix는 적용하지 않는다.
- callback은 `shapeId -> bool`이고 false가 반환되면 남은 Tree도 중단한다.
  결과 순서는 보장하지 않는다. Query 도중 World나 Shape를 변경하거나 Step에 재진입하지 않는다.
- invalid geometry, non-finite point/transform, 음수 radius 등은 결과 없이 반환한다.
  `polygon2`는 유효한 convex polygon으로 만들어져 있어야 한다.
- 반환 handle은 결과를 저장한 뒤 사용하기 전에 `world::IsValid`로 검사한다.

`worldOverlapShapeTests`: AABB-only 거짓 양성, 접선 원, 회전된 Body의 원·다각형,
캡슐 질의, 체인·세그먼트, 필터·센서, 3 Body Tree, 콜백 중단, 수명과 invalid input을
Debug/Release에서 검사한다.
