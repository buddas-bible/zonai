# Zonai

Box2D의 알고리즘과 설계 근거를 이해하며 C++20으로 구현하는 2D 물리 엔진 학습 프로젝트다. 현재 serial rigid-body simulation, collision detection, contact solver, island/sleep, CCD와 sensor를 구현했다. 구현 범위와 Box2D와의 차이는 [감사 현황](docs/audit-status.md)에 기록한다. Distance spring/limit/motor, [Revolute 회전축](docs/revolute-joint.md)/각도 제한/모터와 [Wheel 서스펜션](docs/wheel-joint.md)/[이동 제한](docs/wheel-limit.md)/[회전 모터](docs/wheel-motor.md)을 선택형 Sandbox에서 실험한다. [Mouse Joint](docs/mouse-joint.md)로 Dynamic solid를 잡아 끌 수 있다. [모터 자동차](docs/motor-car.md)에서 차체·두 바퀴·마찰·경사면 주행을 비교하며 다음은 각도 제한으로 관절을 연결하는 렉돌이다. [데모 구조와 학습 방향](docs/sandbox-demos.md)을 따른다.

학습을 위한 변경은 제약의 수학, 좌표계, 데이터 수명과 불변식을 코드·주석·작은 재현 실험으로 연결한다. Box2D 구현을 이해한 뒤 현재 C++ 구조에 필요한 부분을 적용하며, 기능 수나 생산용 기반 구조를 늘리는 것을 우선 목표로 삼지 않는다.

## 바람 / 유체 속도장

2D MAC 격자에 Semi-Lagrangian 이류와 압력 투영을 구현한 독립적인 `fluidField2`를 추가했다. Sandbox의 **유체 / 2D 바람·유체 속도장**에서 흐름을 시각화하고 기존 Dynamic Body에 국소 바람 힘을 전달한다. Part 2에서는 원형 강체 점유 셀과 moving no-through 경계, 압력 반력 및 drag의 근사 역방향 주입을 추가했다. Part 3에서는 같은 solver로 캡슐과 회전된 볼록 다각형의 점유 셀과 압력 반력을 지원한다. Part 4에서는 Drag 충격량을 실제 MAC face 질량으로 분배해 Body와 유체 사이의 순간 충격량을 쌍으로 보장하며, 압력·외벽·이류의 전체 운동량 보존과는 구분한다. Part 5에서는 압력 투영의 유체 내부 face 변화량과 강체/외벽 충격량을 별도로 장부화하고, 점유 변경·이류의 운동량 변화도 분리해 측정한다. Part 6에서는 이동한 Solid가 남긴 재노출 유체 face의 속도를 기존의 순수 유체 이웃에서 복원하고, 열린/닫힌 셀과 이산 체적 변화를 기록한다(완전한 cut-cell 체적 보존은 별도 과제). Part 7에서는 부분 점유 셀과 MAC 면 개방 비율을 4×4 sub-cell 표본으로 계산하고 이전/현재 유체 체적의 차이를 계측한다. Part 8에서는 선택적으로 부분 개방 면의 가중 압력 투영을 실행하고 경계 유량과 수치 잔차를 표시한다. Part 9에서는 연속된 점유율 snapshot의 체적 변화율을 이 연속 방정식에 넣고, 폐쇄·분리된 유체 영역의 풀 수 없는 잔차를 측정한다. Part 10에서는 연속된 Shape transform의 이동·회전을 따라 내부 MAC 공유 면의 통과 체적을 시간 표본 적분하고 압력 소스로 사용한다. 경로가 불확실하면 snapshot 방식으로 돌아간다. Part 11에서는 **선택형 보존 MAC 이류**를 추가해 공유 dual face에서 Upwind 유량을 상쇄시키고, 유효 면 운동량 변화/CFL을 계측한다. Sandbox Wind Flow에서 **이류 비교 프리셋**으로 와류를 생성하고 Semi-Lagrangian과 보존 이류를 바로 비교할 수 있다. Part 12a에서는 셀별 **수동 tracer 질량의 보존형 이류**를 추가했다. Part 12b에서는 유효한 Swept Solid 경로를 이용한 **선택형 tracer 재배치 근사**와 미해결 질량 진단을 제공한다. 움직이는 강체가 덮은 셀의 tracer는 별도 계측하며 swept interface 물질 수송은 미구현이다. 정확한 전체 유체·강체 운동량 및 질량 보존, 일반 강체 장애물, 자유 수면은 아직 후속 범위다. [수학·한계·검증](docs/fluid-field.md).

## 빌드와 테스트

CMake 4.2 이상과 C++20 compiler가 필요하다. Windows preset은 Visual Studio 2026의 C++ 도구와 Windows SDK를 사용한다. Sandbox는 Windows의 Win32/Direct3D 11용이며, Dear ImGui는 저장소에 포함돼 있다.

저장소 루트에서 Windows 기본 빌드:

```powershell
cmake --preset vs2026
cmake --build --preset debug --parallel 4
ctest --test-dir build -C Debug --output-on-failure
.\build\sandbox\Debug\zonaiSandbox.exe
```

Linux library/test 빌드:

```sh
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build --parallel 2
ctest --test-dir build --output-on-failure
```

| 옵션 | 기본값 | 용도 |
| --- | --- | --- |
| `ZONAI_BUILD_SANDBOX` | Windows ON, 그 외 OFF | Windows visual Sandbox 빌드 |
| `ZONAI_BUILD_TESTS` | ON | CTest에 등록된 회귀 테스트 빌드 |

Library만 필요하면 configure에 `-DZONAI_BUILD_SANDBOX=OFF -DZONAI_BUILD_TESTS=OFF`를 전달한다. 기존 build cache의 옵션은 그대로 유지되므로 설정을 바꾸려면 값을 명시한다. 다른 플랫폼에서 Sandbox를 ON으로 설정하면 configure 단계에서 지원 범위를 안내한다.

Windows Release 빌드는 `cmake --build --preset release --parallel 4`, 테스트 실행은 `ctest --test-dir build -C Release --output-on-failure`다. 기존 assert 기반 테스트는 Release에서 검사가 사라지므로, 전체 통과 수를 Debug와 동등한 검증으로 해석하지 않는다. 실제 Release 검사와 CI 범위는 [테스트 문서](tests/README.md)를 따른다.

## CMake 프로젝트에 포함하기

```cmake
set(ZONAI_BUILD_SANDBOX OFF CACHE BOOL "" FORCE)
set(ZONAI_BUILD_TESTS OFF CACHE BOOL "" FORCE)
add_subdirectory(path/to/zonai)
target_link_libraries(myApp PRIVATE zonai::zonai)
```

`zonai::zonai`가 include 경로와 C++20 요구 사항을 전달한다. MSVC에는 `/utf-8`도 전달하므로 consumer source/실행 문자셋은 UTF-8을 사용한다. `dynamics/world.h`, `geometry/circle2.h` 등 `src/` 기준 경로로 include한다. 현재는 source tree를 포함하는 방식이며 install/export package는 제공하지 않는다. 모든 헤더가 CMake HEADERS file set에 들어가지만, solver record나 tree node를 안정적인 외부 API로 약속하는 것은 아니다. 일반적인 사용은 World의 ID와 값 기반 API를 따르고, 진단용 const 참조는 storage 변경 전에만 사용한다.

## 저장소 구성

- `src/`: 플랫폼 UI에 의존하지 않는 물리 library.
- `tests/`: subsystem별 자동 회귀 검사.
- `sandbox/`: 물리 상태와 contact/tree를 관찰하는 Windows UI. [조작과 역할](sandbox/README.md).
- `third_party/`: Sandbox의 vendored dependency와 원본 license.
- `docs/`: 감사 근거·한계·개발 순서. [문서 목록](docs/README.md).
