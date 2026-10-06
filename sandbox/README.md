# Sandbox

화면의 버튼·설명·오브젝트·물리 정보는 한글로 표시한다. 빌드 시 나눔고딕과 라이선스를 실행 파일 옆 `assets` 폴더로 복사하므로 배포할 때 함께 제공한다. 내부 UI ID는 유지하고 단위·키 이름은 그대로 표시한다. ImGui가 생성하는 일부 공통 메뉴도 번역하지만 개발용 Metrics/Debugger 전체를 번역한 것은 아니다.

Windows용 Win32 / Direct3D 11 / Dear ImGui visual playground다. 빌드와 실행은 [루트 문서](../README.md)를 따른다. 물리 library가 Sandbox나 ImGui에 의존하지 않는다.

데모 목록에서 **강체 실험**, **거리 조인트 진자**, **회전축과 막대**, **서스펜션과 바퀴**를 선택한다. 전환/Reset은 현재 데모의 World·선택·설정·입력을 새로 만들고 Pause와 초기 camera로 돌아간다. Sub-steps와 공통 충돌 매트릭스는 공통 설정으로 유지한다.

Inspector는 물리량·재질·충돌 마스크·수면/CCD·센서로 나뉜다. 소속 레이어는 체크 목록으로 편집하며 개별 마스크/그룹은 고급 설정을 펼쳐 편집한다. 공통 충돌 설정에서는 필요한 레이어를 최대 8개씩 표시하고 대칭 관계를 한 번만 편집한다. 숨기는 것은 충돌 규칙을 바꾸지 않는다. 월드·마우스 조인트·물리 정보 표시 설정도 필요할 때 펼친다. [설정의 범위와 우선순위](../docs/sandbox-inspector.md)를 따른다.

- Play/Pause, Step, Reset으로 simulation을 제어한다. 물리는 1/60초 fixed step, 한 frame 최대 8 step이며 긴 frame 뒤 남은 누적 시간은 버린다. Sub-steps는 World solver의 내부 반복 단위를 설정한다.
- Object Inspector에서 pose, velocity, damping, sleep/CCD와 shape material/filter를 편집한다. Playground의 Impulse Test는 중심/중심 밖 impulse와 angular impulse를 비교한다.
- Debug Draw에서 shape/fat AABB, contact separation/impulse, COM/velocity, body type별 tree와 node label을 표시한다.
- Canvas 위 mouse wheel로 cursor 기준 zoom, 가운데 버튼 drag로 pan한다. Reset Camera로 선택한 데모의 초기 view를 복원한다.

| 데모 | 장면 | Canvas 입력 |
| --- | --- | --- |
| Playground | static 바닥/ramp, dynamic circle/box/capsule, kinematic platform | A/D 유지: circle에 힘, Space: jump, S: box 회전, 왼쪽 drag: Dynamic solid 잡기, 오른쪽 클릭: cursor 방향 impulse |
| Distance Pendulum | static anchor와 dynamic circle의 고정 거리 Joint | A/D 유지: 진자에 힘, Space: 옆으로 impulse, 왼쪽 drag: Dynamic solid 잡기, 오른쪽 클릭: cursor 방향 impulse |
| 회전축과 막대 | static 축과 dynamic 막대의 Revolute Joint | A/D: 수평 힘, Space: 위로 impulse, S: 회전, Mouse drag·cursor impulse. 접힌 각도 제한/모터 설정 |
| 서스펜션과 바퀴 | static 지지대와 dynamic 원의 Wheel Joint | A/D: 축 옆 힘, Space: 위로 impulse, S: 회전, Mouse drag·cursor impulse. 접힌 주파수/감쇠·이동 제한·회전 모터 설정, 방향 전환·제동 |

입력은 Canvas 위에서만 전달한다. UI 편집·창 focus 상실·가운데 drag 중에는 유지 입력을 취소한다. 왼쪽 drag는 Mouse Joint이며 오른쪽 클릭은 impulse 실험이다. Mouse Joint 설정에서 Hertz/damping/max force를 바꾸고 주황색 anchor/target과 point error/force를 관찰한다. Paused 상태는 다음 Step/Play까지 pose를 진행하지 않는다. 취소 뒤 재시작에는 새 press가 필요하다. [제약의 수학과 수명](../docs/mouse-joint.md)을 따른다.

`main.cpp`는 platform 자원·공통 UI·camera·입력·fixed-step loop를 연결한다. `demo`/`demoSession`은 작은 데모 계약과 전환/시간 진행을 맡는다. `rigidBodyDemo`가 World와 장면/물리 입력을 소유하고 `rigidBodyDemoUi`가 Inspector/설정/표시를 맡는다. `debugCamera`는 좌표 변환과 camera 조작, `debugDraw`는 geometry/tree 표시를 담당한다. 새 데모를 추가하는 방법과 장기 학습 방향은 [데모 문서](../docs/sandbox-demos.md)에 있다.

화면의 contact 수집은 `UpdateCollisions`를 호출하므로 순수 조회만 하는 UI는 아니다. 자동 테스트의 대체나 성능 benchmark로 사용하지 않는다. ImGui headless frame 검사는 있지만 실제 창/GPU/OS 입력과 시각 검증·device-loss 복구를 보장하지 않는다.

## Distance Joint 진자

보라색 선과 두 anchor 점은 Distance Joint다. 처음에는 rigid이며 Controls에서 Distance spring을 켜고 Hertz/damping을 조절한다. ‘진자 축 방향으로 밀기’로 축 방향 진동을 시작하고 ‘진자 옆으로 밀기’로 옆으로 민다. 목표/현재 거리, 늘어난 길이와 signed 축 방향 힘을 관찰한다. Limit을 켜면 초록색 min/빨간색 max와 허용 구간을 표시한다. Spring on/0 Hz는 스프링 힘만 끄며 limit과 motor는 유지한다. Rigid는 range와 motor를 무시하고 같은 min/max는 Target의 rigid로 돌아간다. 부드러운 경계이므로 하중 아래 작은 오차가 남는다. Reset은 rigid/2 Hz/감쇠 0.7, limit off/min 1.5 m/max 2.5 m로 돌아간다. [Spring](../docs/distance-spring.md), [limit](../docs/distance-limit.md), [motor](../docs/distance-motor.md)의 수학·조작·검증을 따른다. Body local anchor는 원점 기준이며 solver에서는 COM 기준 lever arm으로 바뀐다. CCD가 이동을 자른 frame에서는 거리 오차가 일시적으로 커질 수 있다.

## Wheel Joint 서스펜션

‘서스펜션과 바퀴’의 접힌 스프링 설정에서 3 Hz/감쇠 0.7을 바꾸고 축 변위·수직 오차·힘·바퀴 각속도를 비교한다. 스프링 off/0 Hz는 스프링 힘만 끄며 수직 제약과 자유 회전은 유지한다. 접힌 ‘이동 범위 제한’에서 준비된 -0.5~0.5 m를 켜거나 편집하면 스프링을 꺼도 경계를 유지한다. 초록은 최소, 빨강은 최대, 회색은 허용 구간이다. 제한도 끄면 중력으로 떨어진다. 청록색 점은 중립점, 화살표는 축의 양의 방향, 보라색은 바퀴와 회전 방향이다. [기본 좌표계](../docs/wheel-joint.md)와 [이동 제한의 수학·검증](../docs/wheel-limit.md)을 따른다. 접힌 [회전 모터](../docs/wheel-motor.md)는 off/목표 3 rad/s/최대 1 N·m로 시작한다. 펼쳐 사용 여부·속도·토크를 조절하고 방향 전환·제동을 비교한다. 속도 0은 제동하며 토크 0은 힘을 끈다. 다음은 모터 자동차 데모다.
