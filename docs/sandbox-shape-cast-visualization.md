# Sandbox · ShapeCast 이동 충돌 시각화

새 **공간 조회 → ShapeCast 이동 충돌 시각화** 장면은 World ShapeCast의
선형 이동 결과를 화면에 표시하는 읽기 전용 실험이다.

- 원/캡슐/상자를 선택한다. 시작점·도착점은 좌측 Inspector의 미터 좌표로 조정하거나
  캔버스 왼쪽 클릭으로 도착점, Shift+왼쪽 클릭으로 시작점을 지정한다.
- 초록색 정적 도형(원·상자·캡슐)과 보라색 센서를 별도의 World 대상으로 둔다.
  센서 포함을 끄면 QueryFilter가 해당 category를 제외한다.
- 흰색은 시작 도형, 회색은 전체 도착 도형, 파랑은 첫 충돌 시 도형,
  주황색은 충돌점과 법선, 노랑은 현재 미리보기 위치다.
- **RayCast 비교**는 이동체 중심에서 같은 변위로 쏜 광선이다. 분홍색 교차점과
  파란색 ShapeCast 교차점을 비교해 유한 크기의 영향을 확인할 수 있다.
  선은 지나가지만 원·캡슐은 부딪히는 경로(예: y=1.1m)를 실험할 수 있다.
- **이동 AABB 후보**의 청록 사각형은 시작 AABB와 도착 AABB의 합집합이다.
  후보 개수는 BroadPhase 공간 조회 결과이며 실제 geometry 충돌 개수와 다르다.
  세 조회는 Sensor 포함 토글에 따라 같은 Query Filter를 사용한다.
  표시 체크박스는 조회 계산이나 물리 World 자체를 바꾸지 않는다.
- 재생/1프레임은 미리보기 비율만 증가시키며, 최초 충돌 위치에서 멈춘다.
  **실제 물리 Body 이동이나 `world::Step()`을 호출하지 않는다.**
- 기본 원의 충돌 위치, 드래그로 바꾼 경로, Sensor 포함/제외, 원·캡슐·상자
  전환은 `sandboxShapeCastDemoTests`에서 검증한다. Windows에서는
  `sandboxShapeCastViewTests`가 GPU/창 없이 ImGui 프레임의 렌더링을 검사한다.

`master`에 있는 World ShapeCast의 범위만 표시한다. 회전 sweep, ChainSegment를
이동 도형으로 쓰는 기능, Chain ghost vertex의 상세 corner ownership은 포함하지 않는다.
헤드리스 ImGui 테스트는 실제 DX11 창/폰트·사용자 입력 시각 검증의 대체가 아니다.
