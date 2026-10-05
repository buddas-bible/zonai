# Part 10 — Tests / Sandbox / Build / Docs

기준: 2026-10-05, master `4097e754356a81bb56378622bd6ecd1eb705011a`. Box2D 비교 기준은 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`의 root/test/samples CMake 구성이다. 이번 범위는 build/use 흐름과 파일 책임이며 엔진 알고리즘 전체를 다시 감사하지 않는다.

## 발견과 필요한 변경

1. **외부 프로젝트에서 ImGui 경로 실패.** 상위 CMake 프로젝트가 `add_subdirectory`로 zonai를 포함하고 Sandbox를 켜면 `CMAKE_SOURCE_DIR`는 상위 source tree를 가리킨다. 최소 consumer에서 상위 `third_party/imgui/imgui.cpp`를 찾으며 generate가 실패했다. `sandbox/CMakeLists.txt`를 `PROJECT_SOURCE_DIR`로 수정해 zonai에 포함된 dependency를 사용한다. [CMake source tree](https://cmake.org/cmake/help/v4.2/variable/CMAKE_SOURCE_DIR.html), [project source directory](https://cmake.org/cmake/help/v4.2/variable/PROJECT_SOURCE_DIR.html)의 scope에 맞춘 수정이다.
2. **Windows 전용 Sandbox가 모든 플랫폼에서 기본 ON.** 이전 CI는 Ubuntu에서 OFF를 명시해 기본 configure 경로를 검사하지 않았다. 기본값을 target platform의 [WIN32](https://cmake.org/cmake/help/v4.2/variable/WIN32.html)에 맞추고, non-Windows에서 명시적 ON은 configure 단계에서 오류와 OFF 설정 방법을 안내한다. CI의 platform별 override를 제거해 Windows ON / Ubuntu OFF 기본 경로를 실제 build/run한다. 기존 cache 값은 덮어쓰지 않는다.
3. **외부 MSVC consumer의 UTF-8 경고.** library 내부에만 적용되는 directory compile option은 상위 consumer에 전달되지 않아 한글 헤더에서 C4819가 발생했다. `src/CMakeLists.txt`에서 기존 `/utf-8` 요구를 PUBLIC으로 전달하며, 같은 consumer 재빌드에서 경고가 사라짐을 확인했다. Consumer도 UTF-8 source/실행 문자셋을 사용한다.
4. **사용 문서 부재/과거 표현.** 빈 root README를 실제 configure/build/test/run 및 CMake embedding 안내로 채웠다. tests/Sandbox/third-party README의 미래형 설명을 현재 구현, 조작, vendored ImGui version/license와 검증 한계로 바꿨다. tests CMake의 첫 구현 이전 주석도 바로잡았다.

## KEEP / REFACTOR / DEFER

| 결정 | 대상과 근거 |
| --- | --- |
| KEEP | 기존 38개 executable/CTest 등록. 실패를 subsystem에 귀속시키기 쉽다. 반복된 CMake 선언을 줄이기 위해 test framework나 registry를 추가하지 않는다. |
| KEEP | `main.cpp`의 한 scene + platform/UI/fixed-step 연결, camera/draw 분리. 현재 추가 scene이나 재사용 요구가 없으므로 크기만 보고 application framework로 나누지 않는다. |
| KEEP | `shape.h`의 geometry variant와 runtime record. collision/broadphase/World 모두가 실제 소비자라 전체 Dynamics 이동은 경계를 더 흐린다. type 분리는 독립 API 요구가 생기면 결정한다. |
| KEEP | source-tree library target, C++20 usage requirement와 전체 header file set. World diagnostic API와 테스트가 내부 타입을 소비한다. file set은 안정적인 외부 ABI/API 보장과 같지 않다. |
| REFACTOR | 플랫폼 기본값, 소유 프로젝트 dependency 경로, consumer UTF-8 요구와 현재 사용 문서만 정리한다. Physics header/cpp 및 함수 순서는 변경하지 않는다. |
| DEFER | install/export package, public/internal include tree 재편, scene registry, GPU device-loss 복구와 대규모 main.cpp formatting. 실제 배포/확장 요구나 재현된 문제가 있을 때 진행한다. |
| DEFER | 기존 assert 테스트의 Release 일괄 전환, benchmark/SIMD/worker/allocator 구조. 사용자 우선순위와 측정 근거 없이 Joint 개발의 선행 조건으로 추가하지 않는다. |

Box2D는 library와 test/sample dependency를 구분하고 top-level에서 test/sample 옵션을 노출한다. Zonai도 library에 UI dependency가 없는 방향을 유지한다. 다만 현재 Windows scene을 위해 Box2D의 GLFW/OpenGL, FetchContent, benchmark/Tracy, 다중 scene 구조를 복제할 필요는 없다. 기존 옵션을 명시해 library-only embedding을 사용할 수 있다.

## 검증 범위

- 외부 CMake consumer: 수정 전 ImGui path generate 실패 → 수정 후 Debug library/Sandbox build와 World create/Step/ID 검사 실행 통과. C4819 발생 → PUBLIC UTF-8 설정 후 같은 consumer compile/run 통과, 경고 없음.
- library-only embedding: Sandbox/tests OFF의 Release configure/build 및 consumer 실행 통과.
- 플랫폼 계약: Windows host의 configure harness에서 WIN32를 FALSE로 설정해 기본 Sandbox OFF와 명시적 ON 오류를 확인했다. 이것은 실제 Linux compiler 빌드가 아니며, 실제 Linux 기본 빌드는 Ubuntu CI로 확인한다.
- 새 Windows 기본 configure: Sandbox를 포함한 Debug/Release build와 각 CTest 38/38 실행 통과. 최종 Windows/Ubuntu CI는 해당 PR에 기록한다.
- 8개 Release runtime check와 29개 assert 기반 / 1개 compile-time 테스트의 차이는 [현황](audit-status.md)과 [tests 문서](../tests/README.md)에 유지한다. 전체 Release 실행 수를 동등한 coverage로 해석하지 않는다.

Sandbox UI 실행/시각 검증, GPU 장애 복구 또는 새 cross-platform renderer 검증은 수행하지 않았다. Contact 표시 수집은 `UpdateCollisions`를 호출하며 순수한 snapshot 조회가 아니므로 benchmark로 취급하지 않는다.

## 다음 작업

Part 1–10의 구현된 범위와 알려진 한계를 마지막으로 연결해 확인하고 Distance Joint로 넘어간다. 이 통합 확인은 또 다른 전면 재감사가 아니라 ID/Body edge/filter/island/wake/sleep/solver 연결 지점과 필요한 첫 Joint 회귀 조건을 확정하는 단계다. Part 9의 연결 지점 목록을 출발점으로 사용한다.
