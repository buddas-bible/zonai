# Sandbox

Windows용 Win32 / Direct3D 11 / Dear ImGui visual playground다. 빌드와 실행은 [루트 문서](../README.md)를 따른다. 물리 library가 Sandbox나 ImGui에 의존하지 않는다.

현재 한 장면에 static 바닥/ramp, dynamic circle/box/capsule, kinematic platform이 있다.

- Play/Pause, Step, Reset으로 simulation을 제어한다. 물리는 1/60초 fixed step, 한 frame 최대 8 step이며 긴 frame 뒤 남은 누적 시간은 버린다. Sub-steps는 World solver의 내부 반복 단위를 설정한다.
- Object Inspector에서 pose, velocity, damping, sleep/CCD와 shape material/filter를 편집한다. Impulse Test는 중심/중심 밖 impulse와 angular impulse를 비교한다.
- Debug Draw에서 shape/fat AABB, contact separation/impulse, COM/velocity, body type별 tree와 node label을 표시한다.
- Canvas 위 mouse wheel로 cursor 기준 zoom, 가운데 버튼 drag로 pan한다. Reset Camera로 초기 view를 복원한다.

`main.cpp`는 platform 자원·scene·UI·fixed-step loop를 연결한다. `debugCamera`는 좌표 변환과 camera 조작, `debugDraw`는 geometry/tree 표시를 맡는다. 현재 한 scene을 위해 registry나 별도 application framework를 만들지 않는다.

화면의 contact 수집은 `UpdateCollisions`를 호출하므로 순수 조회만 하는 UI는 아니다. 자동 테스트의 대체나 성능 benchmark로 사용하지 않는다. UI 시각 검증과 GPU/device-loss 복구는 빌드 통과만으로 보장하지 않는다.

## Distance Joint 진자

보라색 선과 두 anchor 점은 고정 거리 Joint다. `Kick pendulum`으로 옆으로 밀고 Target/Current 거리와 COM/velocity 표시를 함께 관찰한다. Body local anchor는 원점 기준이며 solver에서는 COM 기준 lever arm으로 바뀐다. Spring/limit/motor는 아직 구현하지 않았다. CCD가 이동을 자른 frame에서는 거리 오차가 일시적으로 커질 수 있다.
