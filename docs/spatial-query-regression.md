# World 공간 조회 교차 검증

`worldSpatialQueryStressTests`는 SpatialQueries API를 개별 예제와 별도로 검증하는
결정적 회귀 시험이다. `world_spatial_query_stress_test.cpp`는 xorshift32 고정 seed로
108개 원 Shape를 3종 Body Tree(Static, Kinematic, Dynamic)에 분산하고,
다른 local center, 회전, 센서 및 category/mask를 사용한다.

- `overlapAABB`: 살아 있는 Shape들의 `GetShapeAABB`(speculative margin 포함)와
  `ShouldQueryShape`을 독립적으로 순회한 결과와 비교함
- `overlapShape`: 원 중심 간 거리와 radius 합에 대한 해석적인 겹침 판정과 비교함.
  직경의 수치 오차가 있는 접선 바로 인근 ±0.0002m만 oracle 불일치 판정에서 제외함
- `castRay`, `castRayClosest`: 원·광선의 2차방정식 해와 최초 양수 교차를 대조함
- `castShapeClosest`: 보고한 hit 위치에서 `overlapShape`로 같은 target과의 겹침을 재확인함
- 각 단계에 대해 콜백 중복 보고, stale shapeId, AABB 후보의 부분집합 조건,
  filter, stop callback, Body transform 갱신, 필터 변경, Shape 파괴·재생성,
  body 파괴 후 세 Tree의 조회 정확성을 검사함

테스트는 처음 배치, Transform/Filter 수정 뒤, Destroy/Recreate 뒤 세 단계로
실행한다. 총 소요 시간은 참고 정보로 출력하지만 **시간 임계값은 실패 조건으로
사용하지 않는다.** 자동 CI는 느린 runner나 Debug 구현을 최적화 성능으로 오인하면
안 되기 때문이다. 성능 비교가 필요하면 동일 하드웨어, 빌드 설정에서 반복 측정한다.

원 분석 해법의 수치 경계와 GJK 근접 접촉 반올림을 구분하기 위해 아주 가까운
접선만 제외하지만, 정확한 원/원 접선 및 Polygon/Circle 접선은 별도
`worldOverlapShapeTests`가 명시적으로 검사한다.
