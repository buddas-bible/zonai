# Sandbox · AABB 후보와 실제 도형/점 포함 비교

Sandbox의 **공간 조회 → AABB·실제 도형 겹침 비교** 장면에서는
동일한 query geometry, transform, filter를 두 World API에 전달한다.

- **BroadPhase 후보**: `world::overlapAABB`는 World 좌표 AABB로 후보를 찾음.
  후보 대상 경계는 geometry의 tight AABB가 아니라 엔진의 speculative margin이 포함된
  `shape.aabb`다. 화면에서도 **실제로 조회에 사용된 확장 AABB**를 그린다.
- **정확한 겹침**: `world::overlapShape`는 위 후보 중 GJK와 radius로 판정한 도형만 찾음
- **AABB만 겹침**: 후보 수에서 실제 겹침 수를 뺀 숫자. 동일 query/filter와 한 World를
  조회하므로 정확한 겹침은 후보의 부분집합이어야 함

초기 query는 반지름 0.5m의 원, 위치 (1.45, 1.45)m다.
상자 (-1,-1) ~ (1,1)의 우상단에서 AABB만 0.05m 겹치지만,
실제 원과 상자의 표면에는 간격이 있으므로 후보 1개 / 겹침 0개다.
(1.2, 1.2)m로 클릭 이동하면 실제 겹침이 발생한다.

캔버스 왼쪽 클릭 또는 인스펙터 좌표 편집으로 조회 위치를 변경한다.
원/캡슐/상자 프리셋, 회전, Sensor 필터, 후보 AABB 표시를 선택할 수 있다.
초록색은 실제 겹침, 주황색은 후보만 있는 Shape, 회색은 후보 밖 Shape,
청록색은 query AABB다. 대상 Body/Shape는 움직이지 않는다.

## 점 포함 모드

같은 장면에서 **조회 방식 → 점 포함**을 선택하면 `world::overlapPoint`의 결과를 확인한다.

- 조회 중심의 **한 점**을 그대로 `overlapAABB({point, point})`의 후보 범위로 사용한다.
  Query AABB의 폭과 높이가 0이므로 도형의 외곽선 대신 점 마커를 표시한다.
- 후보는 speculative margin을 포함한 Shape 저장 AABB에서 찾지만, 실제 포함은
  각 Shape의 `testShapePoint` 의미를 따른다. Circle/Capsule/Polygon 내부 또는
  경계만 인정하며, 면적이 없는 Segment/ChainSegment는 포함 판정을 하지 않는다.
- 도형 크기와 회전은 점 조회에 관여하지 않아 UI에서 비활성화하며,
  원래 설정은 도형 모드로 되돌아갔을 때 유지한다. 좌표와 Sensor 필터는 두 모드가 공유한다.
- 예를 들어 상자의 오른쪽 `(1.01, 0)`은 speculative AABB의 후보이지만 상자
  내부는 아니고, `(1.0, 0)`은 경계에 포함된다. 반대로 `(1.3, 0)`에서
  점 자체는 상자 밖이지만 반지름 0.5m 원 도형은 상자와 겹칠 수 있다.

이 데모는 충돌 solver의 manifold 또는 실시간 Contact 생성 결과가 아니라,
**World 공간 조회 API의 결과**만 시각화한다.

ChainSegment의 one-sided contact 또는 ghost vertex 소유권 판정이 아니라,
World OverlapShape의 대칭 거리 겹침 판정 결과를 시각화하는 장면이다.

`sandboxOverlapShapeDemoTests`는 Linux/Windows의 query 결과·입력·필터·reset을 검증한다.
`sandboxOverlapShapeViewTests`는 Windows headless ImGui 프레임을 검증한다.
GPU/DX11 실창에서의 입력·폰트·시각 검수는 별도 수동 확인이 필요하다.

점/도형 모드 전환, 경계 점, speculative AABB 거짓 양성, 동적·정적·센서 Shape,
도형 설정 독립성과 클릭 갱신을 기존 Sandbox 모델/Headless UI 테스트에서 추가 검증한다.
