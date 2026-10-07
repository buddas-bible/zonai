# Joint 폴더 구조 정리 설계

기준: master `9b05c61b7a98d5c1451ba481cae9e67ffb01b094`.

## 목적

현재 `src/dynamics`에 Body, Contact, Island, World와 함께 Distance/Mouse/Revolute/Wheel Joint 전용 파일이 평평하게 섞여 있다. Joint 종류가 늘면서 탐색성이 떨어졌으므로 Joint 전용 타입과 solver 파일만 `src/dynamics/joints` 아래로 모은다.

이번 작업은 파일 위치와 include 경로만 정리하며 물리 동작, public API 이름, solver 수식과 실행 순서는 바꾸지 않는다.

## 이동 대상

다음 파일만 `src/dynamics/joints/`로 이동한다.

- `joint2.h`
- `distanceJoint2.h`
- `distanceJointSim2.h`
- `distanceJointConstraint2.h/.cpp`
- `mouseJoint2.h`
- `mouseJointSim2.h`
- `mouseJointConstraint2.h/.cpp`
- `revoluteJoint2.h`
- `revoluteJointSim2.h`
- `revoluteJointConstraint2.h/.cpp`
- `wheelJoint2.h`
- `wheelJointSim2.h`
- `wheelJointConstraint2.h/.cpp`

## 이동하지 않는 파일

- `body.h`, `bodySim.h`, `bodyState.h`: Joint 전용이 아닌 공용 dynamics 상태
- `constraintSoftness2.*`: Contact와 여러 Joint가 공통으로 사용
- `contactConstraint2.*`: Contact solver 전용
- `island2.*`: Contact와 Joint 모두의 연결/수면 단위
- `world.*`: World ownership/orchestration
- `id.h`: Body/Shape/Contact/Joint public handle 공용

## 의존성 규칙

- Joint 전용 파일 내부 include는 `dynamics/joints/...`를 사용한다.
- World 및 Sandbox/Test의 Joint 직접 include도 새 경로를 사용한다.
- `joints` 내부에서 공용 dynamics 파일을 참조하는 것은 허용한다.
- 공용 dynamics 파일이 특정 Joint 구현 파일에 불필요하게 의존하도록 만들지 않는다.
- 새 umbrella header나 alias compatibility header는 만들지 않는다.

## 테스트 및 CMake

현재 source/test CMake가 glob 또는 재귀 수집을 사용하는지 먼저 확인한다. 재귀 수집이 아니라면 새 하위 디렉터리 파일이 빌드되도록 source list를 수정한다.

기존 Joint 테스트의 논리와 기대값은 변경하지 않고 include 경로만 수정한다. 파일 이동으로 인한 기능 테스트 추가는 하지 않는다.

## 완료 조건

1. 기존 Joint 전용 파일이 `src/dynamics` root에 남아 있지 않음.
2. 저장소 source/test/sandbox에서 옛 `dynamics/<joint-file>` include가 남아 있지 않음.
3. 기능 코드 diff는 경로/include 변경 외에 없음.
4. Windows/Ubuntu GitHub Actions 전체 build/CTest 통과.
5. 검증 후 PR을 `master`에 병합.

## 비목표

- Joint API 재설계
- Joint solver 통합/가상화
- `constraintSoftness2` 이동
- `world.cpp` 분할
- 새로운 Joint 종류 추가
- Solver 성능/수식 변경
