# Sandbox

Windows용 Win32 / Direct3D 11 / Dear ImGui visual playground다. 빌드와 실행은 [루트 문서](../README.md)를 따른다. 물리 library가 Sandbox나 ImGui에 의존하지 않는다.

Demo 목록에서 **Playground** 또는 **Distance Pendulum**을 선택한다. 전환/Reset은 현재 데모의 World·선택·설정·입력을 새로 만들고 Pause와 초기 camera로 돌아간다. Sub-steps는 공통 설정으로 유지한다.

- Play/Pause, Step, Reset으로 simulation을 제어한다. 물리는 1/60초 fixed step, 한 frame 최대 8 step이며 긴 frame 뒤 남은 누적 시간은 버린다. Sub-steps는 World solver의 내부 반복 단위를 설정한다.
- Object Inspector에서 pose, velocity, damping, sleep/CCD와 shape material/filter를 편집한다. Playground의 Impulse Test는 중심/중심 밖 impulse와 angular impulse를 비교한다.
- Debug Draw에서 shape/fat AABB, contact separation/impulse, COM/velocity, body type별 tree와 node label을 표시한다.
- Canvas 위 mouse wheel로 cursor 기준 zoom, 가운데 버튼 drag로 pan한다. Reset Camera로 선택한 데모의 초기 view를 복원한다.

| 데모 | 장면 | Canvas 입력 |
| --- | --- | --- |
| Playground | static 바닥/ramp, dynamic circle/box/capsule, kinematic platform | A/D 유지: circle에 힘, Space: jump, S: box 회전, 왼쪽 drag: Dynamic solid 잡기, 오른쪽 클릭: cursor 방향 impulse |
| Distance Pendulum | static anchor와 dynamic circle의 고정 거리 Joint | A/D 유지: 진자에 힘, Space: 옆으로 impulse, 왼쪽 drag: Dynamic solid 잡기, 오른쪽 클릭: cursor 방향 impulse |

입력은 Canvas 위에서만 전달한다. UI 편집·창 focus 상실·가운데 drag 중에는 유지 입력을 취소한다. 왼쪽 drag는 Mouse Joint이며 오른쪽 클릭은 impulse 실험이다. Mouse Joint 설정에서 Hertz/damping/max force를 바꾸고 주황색 anchor/target과 point error/force를 관찰한다. Paused 상태는 다음 Step/Play까지 pose를 진행하지 않는다. 취소 뒤 재시작에는 새 press가 필요하다. [제약의 수학과 수명](../docs/mouse-joint.md)을 따른다.

`main.cpp`는 platform 자원·공통 UI·camera·입력·fixed-step loop를 연결한다. `demo`/`demoSession`은 작은 데모 계약과 전환/시간 진행을 맡는다. `rigidBodyDemo`가 World와 장면/물리 입력을 소유하고 `rigidBodyDemoUi`가 Inspector/설정/표시를 맡는다. `debugCamera`는 좌표 변환과 camera 조작, `debugDraw`는 geometry/tree 표시를 담당한다. 새 데모를 추가하는 방법과 장기 학습 방향은 [데모 문서](../docs/sandbox-demos.md)에 있다.

화면의 contact 수집은 `UpdateCollisions`를 호출하므로 순수 조회만 하는 UI는 아니다. 자동 테스트의 대체나 성능 benchmark로 사용하지 않는다. ImGui headless frame 검사는 있지만 실제 창/GPU/OS 입력과 시각 검증·device-loss 복구를 보장하지 않는다.

## Distance Joint 진자

보라색 선과 두 anchor 점은 고정 거리 Joint다. `Kick pendulum`으로 옆으로 밀고 Target/Current 거리와 COM/velocity 표시를 함께 관찰한다. Body local anchor는 원점 기준이며 solver에서는 COM 기준 lever arm으로 바뀐다. Spring/limit/motor는 아직 구현하지 않았다. CCD가 이동을 자른 frame에서는 거리 오차가 일시적으로 커질 수 있다.
