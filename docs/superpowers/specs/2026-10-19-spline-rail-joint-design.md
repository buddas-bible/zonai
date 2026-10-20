# 편집 가능한 spline 경로와 spline 레일 조인트 설계

## 목표와 완료 범위

기존 `splinePath`의 키네마틱 경로 이동을 유지하고 제어점 편집을 연결한다. 별도 `splineRail` 데모와 엔진 조인트를 추가하여 다이나믹 물체가 곡선을 따라 중력·외력·충돌에 반응하게 한다. 기존 prismatic 레일과 스프링·리미트·모터의 설정 의미를 맞춘다.

첫 구현의 레일은 열린 centripetal Catmull–Rom 경로다. 유한한 레일 양 끝은 물리적 정지점이며, 곡선을 벗어나 직선으로 연장하지 않는다. 닫힌 레일의 다회전 좌표·순환 리미트는 이번 구현에 포함하지 않는다. 기존 닫힌 spline geometry API는 유지한다.

완료에는 엔진 solver/World 연결, 두 회전 모드, 두 데모의 spline 편집, 수명·상태 보존 테스트, Debug·Release 전체 빌드·테스트가 모두 포함된다. 경로를 따라 transform을 강제 설정하는 기능을 다이나믹 조인트의 구현으로 사용하지 않는다.

## 설계 선택

전용 `splineRailJoint`를 기존 World 조인트 체계에 등록한다. 기존 prismatic의 softness·축 방향 spring/motor/limit 정책을 참고하고 공용 수학 도구를 재사용한다. 곡선 투영·곡률과 회전 추종에 필요한 Jacobian은 전용 solver에서 계산한다.

직선 조인트의 축과 anchor를 매 프레임 갱신하는 방식은 구현량은 작지만 곡률에 따른 회전 결합, 연속 경로 좌표 및 warm start의 의미가 불명확해진다. 이번 기능의 solver 구현으로 선택하지 않는다.

## 좌표와 공개 계약

- Body A가 레일을 소유하고, 제어점은 A의 로컬 좌표다. Body B의 `localAnchorB`가 레일에 붙는다. A도 움직일 수 있으며 반력·반토크가 양쪽 Body에 적용된다.
- 생성 설정은 제어점을 참조해서 받지만 World는 검증 후 복사하여 소유한다. 호출자의 배열 변경·재할당은 조인트를 바꾸지 않는다.
- `s`는 첫 제어점에서 현재 투영점까지 잰 곡선 길이(m)다. `currentTranslation = s - referenceDistance`로 정의한다. `referenceDistance`는 생성 시 지정하는 유한한 기준 거리이며 기본값은 0m다.
- `referenceDistance`는 편집 후에도 같은 수치로 유지한다. 따라서 경로가 바뀌어도 스프링 목표·리미트의 단위와 기준을 자동으로 재해석하지 않는다.
- 기본 회전 모드는 `fixed`다. Body B-A의 상대각을 `referenceAngle`로 유지한다. Body의 회전 관성을 끄거나 fixedRotation을 설정하지 않는다.
- 선택 회전 모드는 `followTangent`다. 생성/모드 진입 당시의 자세를 기준으로 이후 접선 방향 변화만큼 회전하도록 한다. 진입 시 현재 상대각과 접선각의 차이를 offset으로 저장한다. `angularHertz`·`angularDampingRatio`의 각도 spring-damper로 추종한다. 노말은 접선의 왼쪽 수직 방향이다.
- 회전 모드 전환은 물체 transform을 덮어쓰지 않는다. fixed로 돌아가면 원래 `referenceAngle`을 목표로 한다. angular tuning 0Hz는 weld와 같은 hard 추종 정책으로 정의한다.

공개 타입은 기존 명명 규칙을 따른 `splineRailJointDef`, `splineRailJointData`, `splineRailRotationMode`다. 주요 설정은 다음과 같다.

```cpp
bodyId bodyA, bodyB;
std::span<const vec2> points;
vec2 localAnchorB{};
double referenceDistance = 0.0;
float referenceAngle = 0.0f;
splineRailRotationMode rotationMode = splineRailRotationMode::fixed;
float angularHertz = 3.0f;
float angularDampingRatio = 0.7f;
bool enableSpring = false;
float hertz = 0.0f;
float dampingRatio = 0.7f;
float targetTranslation = 0.0f;
bool enableLimit = false;
float lowerTranslation = 0.0f, upperTranslation = 0.0f;
bool enableMotor = false;
float motorSpeed = 0.0f, maxMotorForce = 0.0f;
bool collideConnected = false;
```

World API는 다음과 같이 기존 조인트의 형식에 맞춘다.

```cpp
jointId createSplineRailJoint( const splineRailJointDef& definition );
void setSplineRailJointSpring( jointId id, bool enableSpring, float hertz, float dampingRatio, float targetTranslation );
void setSplineRailJointLimit( jointId id, bool enableLimit, float lowerTranslation, float upperTranslation );
void setSplineRailJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce );
void setSplineRailJointRotation( jointId id, splineRailRotationMode mode, float angularHertz, float angularDampingRatio );
bool setSplineRailJointPoints( jointId id, std::span<const vec2> points );
splineRailJointData getSplineRailJointData( jointId id ) const;
std::span<const vec2> getSplineRailJointPoints( jointId id ) const;
```

위 points 조회 span은 조인트의 다음 편집 또는 파괴 전까지 유효하다. snapshot에는 제어점 복사 대신 현재 parameter·translation·anchor·tangent·normal·lateral/angular error·설정·마지막 substep 반력/토크를 담는다.

## 기존 레일과 동일한 설정 의미

| 설정 | spline 레일에서의 의미 |
| --- | --- |
| enableSpring / hertz / dampingRatio | 레일을 따라 targetTranslation으로 복원. 0Hz는 축 방향 spring 힘만 끔 |
| targetTranslation | 기준 거리에서 잰 부호 있는 목표 곡선 거리(m) |
| lower/upperTranslation | 같은 좌표계의 이동 리미트. lower==upper는 위치 고정 |
| motorSpeed / maxMotorForce | 접선 방향 상대 속도(m/s)와 최대 힘(N). 0속도 모터는 제동 |
| collideConnected | 기존 조인트와 같은 연결 Body 충돌 필터 |

노말 방향은 레일의 hard 제약이다. 이번의 축 방향 스프링을 노말 서스펜션으로 바꾸지 않는다.

물리적 끝점 범위 `[-referenceDistance, length-referenceDistance]`와 사용자 리미트를 함께 지킨다. 사용자 리미트가 켜진 경우 두 범위가 겹치지 않는 생성·설정·경로 편집은 거절한다. 스프링 목표가 끝점 밖에 있는 것은 허용하고 끝점 제약이 이동을 막는다. 유한한 레일의 끝점 차이는 UI에 표시한다.

## 곡선 추적과 solver

1. 생성 시 Body B anchor를 A 로컬로 변환해 곡선의 초기 투영점을 구한다. 최소거리 동률은 기존 parameter 순서로 결정한다.
2. 이후에는 이전 parameter와 접선 방향 속도로 예측한 경로 위치를 이용해 연결된 인접 구간을 추적한다. 교차점에서 먼 구간으로 전역 최근접점을 다시 선택하지 않는다. 경로 편집 때만 새 곡선에 재투영한다.
3. 매 substep과 제약 반복에서 현재 Body delta 상태를 반영해 anchor·투영점·접선·노말·곡률을 갱신한다. 노말 제약과 선택한 회전 제약을 풀고, 기존 레일과 같은 개념 순서로 축 spring/motor/limit를 푼다.
4. 회전 추종은 단순히 목표각만 바꾸지 않는다. 접선각 변화율과 곡선 거리 변화율의 곡률 결합을 Jacobian에 포함한다. A의 이동·회전, COM과 anchor 사이의 팔 길이도 반력·토크에 반영한다.
5. 직선에서는 곡률이 0이므로 기존 prismatic 제약으로 환원되어야 한다. 기존 prismatic solver 자체는 변경하지 않는다.
6. warm start는 조인트별 누적 impulse를 보관한다. substep 시간·모드·경로·spring/motor/limit 변경으로 의미가 달라진 cache는 비우고 연결된 non-static component를 깨운다. 추적 분기가 바뀌면 이전 프레임의 축 impulse를 그대로 쓰지 않는다.
7. 투영·길이·곡률 계산이 실패하거나 투영의 미분이 특이해진 substep은 해당 조인트의 cache/반력을 0으로 하고 impulse 적용을 건너뛴다. NaN·Inf를 Body state로 전파하지 않는다. snapshot에 유효한 추적 상태 여부를 노출한다.

제어점은 열린 geometry 유효성에 더해 레일 방향이 정의되는 regular curve여야 한다. 0 접선/cusp 또는 수치적으로 불안정한 방향을 갖는 후보는 생성·편집에서 거절한다. 접선에 따라 자세를 정하는 splinePath 편집에도 같은 방향 유효성 조건을 적용한다. regularity 확인은 조밀한 샘플링만으로 대신하지 않고 구간별 미분 다항식의 최소 속도/근을 검사한다. 기하·투영의 오차 기준은 좌표 크기와 float 표현 한계를 함께 고려하며 solver의 물리적 이탈 허용값과 분리한다.

## 경로 수정과 두 데모

- `splinePath`는 현재 속도 1m/s와 접선 방향 회전·리셋 동작을 유지한다. constexpr 제어점은 인스턴스별 편집 가능한 배열로 바꾼다.
- `splineRail`은 별도 데모로 등록한다. Body B는 Dynamic이고 spring·limit·motor·회전 모드를 기존 레일 UI와 같은 명칭/단위로 조절한다. 접선·노말과 현재 anchor를 표시한다.
- 두 데모에 기존 spline terrain과 같은 제어점 드래그, 곡선 10px 이내 Ctrl+클릭 삽입, 최소 2점을 지키는 삭제를 연결한다. 실제 클릭 위치를 해당 곡선 구간의 두 제어점 사이에 삽입한다. 카메라 배율을 반영한다.
- 편집 gesture를 소비한 입력은 같은 누름에서 mouse joint를 만들지 않는다. 범위 밖 삽입과 잘못된 후보는 상태/Undo 기록을 바꾸지 않는다.
- 기존 한 단계 Undo/Redo와 gesture 단위 기록 정책을 재사용한다. 두 데모가 terrain의 시험·진단 상태를 불필요하게 갖지 않도록 편집 geometry/기록 부분만 재사용한다.
- 모든 후보는 검증 후 한 번에 적용한다. 조인트의 points는 복사해서 교체하고 joint/body/shape handle, 질량·밀도·마찰·반발, 선택 상태와 선형·각속도를 유지한다. 재투영과 cache 무효화 후 다음 solver가 이탈을 물리적으로 교정한다. 편집 자체가 물체 transform을 옮기지 않는다.
- splinePath 편집 시에는 편집 전 진행 길이 비율을 유지해 새 경로의 목표 위치를 정하고, 다음 kinematic update가 이를 반영한다. 정지/일시정지 중에는 진행 시간이 늘지 않는다. 리셋은 편집된 현재 경로의 시작으로 돌아간다.
- 포인터 수명은 World의 소유 복사로 보호한다. setter 실패 시 기존 geometry와 물리 상태를 유지한다.

## 파일 경계와 엔진 연결

- 새 `src/dynamics/joints/splineRailJoint2.h`, `splineRailJointSim2.h`, `splineRailJointConstraint2.h/.cpp`: 공개 설정/snapshot, 소유 geometry·solver cache, prepare/warmStart/solve.
- 새 `src/dynamics/splineRailJointWorld.cpp`: create/set/query와 경로 교체. 기존 `pogoJointWorld.cpp` 등 분리 패턴을 따른다.
- `src/dynamics/world.h/.cpp`: joint variant, graph/lifetime, prepare/solve/impulse 저장, destroy/cache/wake 처리에 새 종류를 연결한다. 기존 조인트를 재구성하거나 handle 형식을 바꾸지 않는다.
- `sandbox/demo.h/.cpp`, `rigidBodyDemo.h/.cpp`, `rigidBodyJointDemo.cpp`, `rigidBodyDemoUi.cpp`와 기존 편집 관련 구현: 새 데모, 경로 소유 배열과 입력/UI 연결. 필요 시 spline 전용 데모 구현 파일을 하나 추가하되 다른 데모를 일괄 이동하지 않는다.
- 새 solver/World 회귀 테스트와 기존 sandbox 데모/UI 테스트, `tests/CMakeLists.txt`: 기능 단계별 테스트 등록.
- C++20, 기존 lowerCamelCase/region·한국어 이유 주석·CRLF를 지킨다. 새 의존성, 무관한 리팩토링과 줄바꿈 변경은 추가하지 않는다.

## TDD 단계와 완료 증거

1. **직선 동등성**: 긴 수평/경사 spline을 기존 prismatic와 같은 기준점·각도·anchor로 구성한다. free/spring/0Hz/limit/locked/motor/brake/combined, off-center anchor, 큰 질량, 움직이는 Body A를 비교한다. 직접 solver의 동일 조건 비교는 절대·상대 오차 1e-5를 기준으로 하고, 여러 step의 World 비교는 위치 2mm·상대각 1e-4rad를 기준으로 한다. 질량·스케일별 반력 비교는 수치 크기를 반영한 상대 오차를 명시한다. 실패를 관측한 뒤 구현하며 허용차를 넓혀 실패를 숨기지 않는다. 레일 끝점 밖을 prismatic 동등성 범위로 오해하지 않는다.
2. **World 수명과 유한 끝점**: 생성·조회·설정·삭제, Body 삭제, stale/cross-world handle, slot 재사용, sleep/wake, collision filter, substep 변경과 warm start, 끝점 정지와 리미트 교집합 실패를 검증한다.
3. **곡선**: 중력/외력과 motor에 의한 진행, 노말 이탈, 축 spring 복원/감쇠, 양 끝점, 접합점 연속성, 교차점의 분기 유지, 큰 timestep/substep 수 변화, 불량 geometry와 추적 실패 시 유한한 상태를 검증한다.
4. **회전**: fixed 상대각 유지, followTangent의 초기 offset·각도 wrap·곡률 결합·각 spring 감쇠, 동적 A의 반토크, 모드/tuning 변경 시 cache와 wake를 검증한다.
5. **편집/데모**: 두 기능 모두 드래그·10px 삽입·삭제·Undo/Redo·잘못된 후보의 원자적 실패, 카메라 배율, 입력 충돌 방지, 경로 진행 비율, 조인트 물리 상태/handle 보존을 테스트한다. 기존 경로 이동과 spline terrain 회귀 테스트를 유지한다.
6. **최종 검증**: 독립 코드 리뷰 후 Debug·Release 전체 빌드와 전체 CTest를 실행한다. RED/GREEN 및 전체 실행 결과를 저장하고 실제 통과한 개수와 미검증 범위를 보고한다. 성능은 대표 제어점 수/물체 수의 step 비용을 확인하여 solver 내부에서 매 반복마다 전체 경로를 불필요하게 검증·적분하지 않는지도 확인한다.

위 단계는 하나의 기능을 검증 가능한 순서로 나눈 것이다. 직선 단계만 구현한 상태를 전체 완료로 보고하지 않는다. PR·CI·master 병합과 브랜치 정리는 별도 요청 시 수행한다.
