# Zonai

Box2D의 알고리즘과 설계 근거를 이해하며 C++20으로 구현하는 2D 물리 엔진 학습 프로젝트다. 현재 serial rigid-body simulation, collision detection, contact solver, island/sleep, CCD와 sensor를 구현했다. 구현 범위와 Box2D와의 차이는 [감사 현황](docs/audit-status.md)에 기록한다. Rigid Distance Joint와 [spring](docs/distance-spring.md)/[limit](docs/distance-limit.md), 선택형 Sandbox 데모를 구현했다. [Mouse Joint](docs/mouse-joint.md)로 Dynamic solid를 잡아 끌 수 있다. 다음은 Distance motor와 연결 오브젝트 데모다. [데모 구조와 학습 방향](docs/sandbox-demos.md)을 따른다.

학습을 위한 변경은 제약의 수학, 좌표계, 데이터 수명과 불변식을 코드·주석·작은 재현 실험으로 연결한다. Box2D 구현을 이해한 뒤 현재 C++ 구조에 필요한 부분을 적용하며, 기능 수나 생산용 기반 구조를 늘리는 것을 우선 목표로 삼지 않는다.

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
