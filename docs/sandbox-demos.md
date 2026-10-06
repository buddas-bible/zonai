# 데모 선택과 물리 학습 공간

2026-10-06, master `773d7af9097add1a633a473eb19937435e21a57b` 이후 Sandbox의 첫 데모 선택 구조를 구현한다. 목적은 기능을 만들 때마다 같은 프로그램에 직접 조작할 실험을 추가하는 것이다. [설계와 경계](superpowers/specs/2026-10-06-sandbox-demo-design.md)를 따른다.

## 이번 구현

| 데모 | 장면 | Canvas 위 조작 |
| --- | --- | --- |
| Rigid Bodies / Playground | 기존 바닥·ramp·circle·box·capsule·kinematic platform | A/D 유지: circle에 힘, Space: jump impulse, S: box 회전 impulse, 왼쪽 drag: Dynamic solid 잡기, 오른쪽 클릭: circle을 cursor 방향으로 밀기 |
| Joints / Distance Pendulum | 독립 Static anchor와 Dynamic bob, rigid Distance Joint | A/D 유지: 수평 힘, Space: kick, 왼쪽 drag: 진자 잡기, 오른쪽 클릭: cursor 방향으로 밀기, 보라색 선·목표/현재 길이로 제약 관찰 |

`Demo` 목록으로 장면을 고르고 Play/Pause, Step, Reset을 사용한다. 전환/reset은 paused 상태의 새 장면을 만들고 선택/contact/입력·시간·카메라를 초기화한다. Sub-steps는 공통 설정으로 유지한다. Wheel zoom/middle pan과 기존 pose/material/filter/sleep/CCD/force/Contact/Tree Inspector는 유지한다.

키 조작은 pointer가 Canvas 위에 있을 때 전달한다. UI 편집이나 focus 상실, Canvas 밖, middle pan, 전환 frame에는 held 입력을 취소한다. A/D는 매 physics step에 힘으로 적용하고 Space/S/오른쪽 클릭은 한 번의 impulse로 적용한다. Paused 상태의 impulse는 속도를 바꾸지만 pose는 다음 step에서 진행한다. 왼쪽 drag의 picking/Mouse Joint는 [후속 구현](mouse-joint.md)에 기록한다. 입력 취소는 held force와 drag Joint를 함께 비우며 새 press 없이 다시 잡지 않는다.

## 파일 책임과 학습 이유

- `sandbox/demo.h/.cpp`: 작은 데모 interface/catalog와 `demoSession`의 ownership/playback. 공통 interface는 World를 요구하지 않는다. 고정 1/60초, 최대 8 step, 긴 frame의 초과시간 폐기로 렌더 FPS와 물리 시간을 분리한다.
- `sandbox/rigidBodyDemo.h/.cpp`: 두 강체 장면의 독립 World와 입력/contacts. GUI 없이 같은 실험을 실행할 수 있다. Held force를 입력 event에서 한 번만 넣으면 render FPS에 따라 결과가 달라지므로 physics step마다 적용한다.
- `sandbox/rigidBodyDemoUi.h/.cpp`: 기존 World Inspector와 geometry/contact/tree 표시. 선택과 debug 옵션은 view instance에 있어 새 데모로 이전 handle이나 선택 index가 넘어가지 않는다.
- `sandbox/main.cpp`: platform/D3D11/ImGui, 선택·공통 재생·카메라, UI를 제외한 Canvas 입력 전달. 화면 설정과 입력 뒤에 물리를 진행하여 focus 상실 frame에 오래된 held force로 추가 step이 생기지 않게 한다.

데모 전환은 이전 World를 재활용하는 것보다 소유 instance 전체를 교체하는 방식이 단순하다. 현재 World는 lifetime token으로 이전 ID를 거부하므로 새로운 장면의 같은 slot을 이전 선택이 가리키지 않는다. 새 instance를 먼저 생성하고 이전 데모를 해제한다.

새 데모는 `demoKind`/catalog에 이름·조작 안내·초기 카메라를 넣고 factory에서 해당 instance를 생성한다. 다른 물리 방식의 데모는 `rigidBodyDemo`를 상속할 필요 없이 `demo`를 구현할 수 있다. 현재 drawing/camera는 2D이며 3D rendering까지 구현됐다는 뜻은 아니다. Editor/ECS/plugin 기반을 먼저 만들지는 않는다.

## 검증

`sandboxDemoTests`는 NDEBUG 독립 runtime check다. 두 실제 scene의 body/joint 수, 전환 후 이전 ID 거부, reset/paused/single step, held force와 press impulse, UI 입력 취소, fractional time과 최대 catch-up, World 없는 demo를 검사한다. Windows에서 Sandbox를 켜면 같은 target이 실제 ImGui view의 controls/geometry를 6개 headless frame에서 전환/reset하며 실행한다. Sandbox를 끈 구성과 Ubuntu에서는 GUI model/host 검사만 실행한다.

새 header가 없는 build 실패를 먼저 확인한 후 구현했다. Windows Sandbox 포함 전체 Debug/Release build와 각각 41/41 CTest, Sandbox OFF 구성의 Release `sandboxDemoTests`를 확인했다. 전환 pause/count, 입력 차단, fixed dt, catch-up cap, pause의 fractional time 폐기를 망가뜨리는 6개 mutation 모두 실패하고 원본 복구 후 통과했다. 원격 Windows/Ubuntu CI 결과는 이 변경의 PR에 기록한다. 기존 29 assert target의 Release 공백은 유지한다. Headless ImGui smoke는 GPU/OS input/실제 화면의 시각 검증을 대신하지 않는다.

## 사용자 목표와 다음 단계

한 프로그램에 데모별 조작·설정·관찰값을 추가하는 물리 학습 공간이 장기 목표다. Mouse Joint와 필요한 picking, [Distance spring의 Hertz·감쇠 실험](distance-spring.md)을 추가했다. 다음은 Distance limit/motor와 다른 Joint를 독립 실험으로 확인하며 ragdoll·motor 자동차·조나이 선풍기/연결 장치로 조합한다.

천·유체·soft body·voxel physics/destruction/terrain·다양한 terrain collision·오목한 object/terrain·파괴 조각의 rigid body simulation도 같은 프로그램의 데모로 학습하고 싶다는 사용자 목표를 보존한다. 각 기능의 solver/data와 2D/3D 범위는 해당 구현 단계에서 정한다. Voxel은 파괴 → 조각 분리 → collision/mass 생성 → rigid body simulation을 단계별로 확인한다. 이 목록은 해당 기능의 현재 구현 완료를 의미하지 않는다.
