# Sandbox 데모 선택·입력 설계

2026-10-06, 사용자가 한 프로그램에서 여러 물리 데모를 선택하고 각 데모의 마우스·키 조작으로 학습하는 목표를 설명했다. 공통 host와 데모의 역할, 기존 강체/Distance 진자의 분리, 그다음 Mouse Joint라는 순서를 제안한 뒤 사용자가 진행을 요청했다. 이 문서는 첫 단계의 구현 경계를 기록한다.

## 역할과 입력

`demo`는 step/input/cancel/control/draw callback을 제공하며 World를 소유하지 않는다. `demoSession`은 factory가 만든 한 데모를 소유하고 재생·고정 시간 진행·전환·reset을 처리한다. `main.cpp`는 Win32/D3D11/ImGui 초기화, 데모 선택, 카메라, Canvas 입력과 frame 렌더링을 맡는다.

첫 두 데모는 `rigidBodyDemo`의 Playground/Distance Pendulum 설정이다. 각 instance는 자신의 World, shape/body/joint handle, contact 표시와 held 입력을 소유한다. `rigidBodyDemoUi`는 기존 Inspector·실험 버튼·contact/tree/debug 표시와 선택 상태를 맡는다. Headless 모델은 GUI callback 기본 동작을 사용하므로 ImGui 없이 검사할 수 있다.

Mouse Joint 전 단계의 실제 입력 실험으로 A/D held force, Space impulse, Playground의 S angular impulse, Canvas 왼쪽 클릭의 cursor 방향 impulse를 제공한다. 기본 drag/picking은 아직 구현하지 않는다. 데모별 안내를 화면에 표시한다. World pose를 cursor로 덮어쓰는 임시 drag는 만들지 않는다.

UI 편집·popup·application focus 상실·Canvas 밖·가운데 pan·데모 전환 frame에는 데모 입력을 취소한다. Held 입력은 각 physics step에 force로 적용하며 press impulse와 구분한다. 취소는 입력 상태를 지우지만 물리 속도를 지우지 않는다. 화면 설정과 입력을 먼저 처리한 뒤 simulation을 진행한다.

전환/reset은 현재 데모를 새 instance로 교체하고 paused 상태, accumulator/step count, 선택/contact/held state와 카메라를 초기화한다. 새 instance를 만든 후 이전 데모를 해제한다. 이전 World ID는 새 World에서 유효하지 않다. Sub-step 값은 공통 프로그램 설정으로 유지한다.

1/60초 fixed step, frame delta 최대 0.25초, frame당 최대 8 step과 초과분 폐기를 유지한다. Pause 전환 시 fractional time을 버리고, single step은 accumulator를 초기화하며 현재 playback 설정을 유지한다.

## 검증과 확장 경계

새 runtime `sandboxDemoTests`는 Windows/Ubuntu에서 GUI 없이 소유권·두 장면·입력·reset·timing을 검사한다. Windows Sandbox 구성은 같은 target에 ImGui frame smoke를 추가하여 두 실제 view의 control/draw/reset을 GPU/window 없이 실행한다. 실제 OS focus/click과 최종 화면의 시각 검증은 별도다.

공통 demo interface의 World 소유 의무를 없애 천·유체·soft body·voxel의 독립 데이터를 수용할 경계를 마련한다. 현재 renderer/camera/input 좌표는 2D이며 3D 지원 완료를 의미하지 않는다. 3D rendering/input, 다른 simulation data/solver, ECS/editor/plugin loader는 실제 기능을 시작할 때 정한다. 새 데모는 catalog/factory에 추가하고 자신의 step/input/control/draw를 구현한다.

신규 이름은 lowerCamelCase, private member는 `_` suffix, cpp definition은 header 순서, 관련 함수는 `#pragma region`으로 묶는다. 기존 물리 API와 Inspector의 설정 기능을 유지하고 새 dependency를 추가하지 않는다.
