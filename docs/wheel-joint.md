# 기본 Wheel Joint와 서스펜션 실험

2026-10-07, master `e0124ad` 이후 Wheel Joint의 축 이동·수직 제약·물리 스프링을 추가했다. 물체 B는 A의 로컬 축을 따라 이동하고 자유롭게 회전한다. 자동차를 조합하기 전에 서스펜션의 이동과 바퀴 회전을 독립적으로 관찰하는 단계다. 이동 범위 제한과 구동 모터는 후속 구현이다.

## API와 좌표계

`createWheelJoint(wheelJointDef)`로 서로 다른 Body 두 개를 연결한다. 적어도 하나는 Dynamic이어야 한다. `localAnchorA/B`는 물체 원점 기준이고 `localAxisA`는 A 기준의 유한한 단위 벡터다. 호출자가 단위 축을 제공하며 World에서 임의의 축을 정규화하거나 잘못된 입력을 복구하지 않는다. A가 회전하면 축도 회전한다. Shape/밀도로 질량 중심이 변해도 원점 기준 연결점은 유지한다.

두 월드 작용점이 같은 위치일 때 스프링 변위가 0이다. B-A 작용점 차이를 현재 축에 투영한 값이 `currentTranslation`(m)이며 음수와 양수를 모두 허용한다. 축의 왼쪽 수직 방향에 투영한 값은 `lateralError`(m)다. 생성 순간의 변위를 별도 rest length로 저장하지 않는다. 중립 위치를 다르게 두려면 두 로컬 작용점을 선택한다.

스프링 기본은 on/3 Hz/감쇠 비율 0.7이다. `setWheelJointSpring(id, enableSpring, hertz, dampingRatio)`는 유한한 비음수 계수를 받는다. 스프링 off나 0 Hz에서는 축 방향 힘을 끄며 **축 이동을 고정하지 않는다**. 기존 Distance Joint의 spring off가 고정 거리로 전환되는 규칙과 다르다. 같은 설정은 cache/sleep을 유지하고 실제 변경은 수직·스프링 임펄스를 함께 비워 연결된 non-static component를 깨운다.

`getWheelJointData`는 Body ID, 월드 작용점·현재 축, 변위·수직 오차, 설정과 반력을 제공한다. `springForce = springImpulse/h`는 B에 작용하는 축 방향 힘(N)이다. 전체 `force`는 수직 임펄스와 스프링 임펄스를 현재 축으로 합쳐 h로 나눈 값이다. 마지막 substep의 관찰값이며 paused/sleep에서 유지된다. 질량·pose 변경은 두 cache를 비우고 h 변경은 warm start를 버린다.

## 회전하는 축의 제약

작용점 팔 길이를 r_a/r_b, 작용점 차이를 d, 현재 A의 축을 ax, 왼쪽 수직 벡터를 ay라고 둔다. 축 수직 제약은 `C = dot(ay, d)`다. A의 회전은 작용점뿐 아니라 축 방향도 바꾸므로 A의 회전 계수에는 **d+r_a**가 필요하다.

```
s1 = cross(d+r_a, ay), s2 = cross(r_b, ay)
Cdot = dot(ay, vB-vA) + s2*wB - s1*wA
K = invMassA + invMassB + invInertiaA*s1² + invInertiaB*s2²
```

스프링은 ay 대신 ax를 사용하며 위치 오차는 `dot(ax, d)`다. 반작용을 A에 적용하고 증가한 임펄스만 누적한다. 수직·축 유효 질량은 Box2D처럼 전체 Step 시작에 한 번 준비하고, solve/warm start의 축·작용점·팔 길이는 현재 누적 회전/이동을 반영한다. 이 방식은 여러 substep을 포함한 전체 Step 동안 유효 질량의 변화에 대한 근사이며 극단적인 이동·회전의 무제한 정확성을 보장하지 않는다.

물리 스프링을 먼저 풀고 갱신된 속도로 수직 제약을 푼다. 스프링은 relaxation에서도 Hertz/감쇠 softness와 복원 bias를 유지한다. 수직 제약의 수치 안정화 bias/softness는 보정 pass에서만 사용하고 relaxation은 현재 수직 속도를 제거한다. 바퀴 각도 자체를 고정하는 식은 없다. 중심 밖 연결점은 선형·회전 운동을 결합한다. 퇴화한 유효 질량은 0으로 처리하고 오래된 해당 임펄스를 적용하지 않는다.

비교 기준은 최신 확인한 Box2D main [`ac7c751`의 wheel_joint.c](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/wheel_joint.c)와 [공식 Wheel Joint 설명](https://box2d.org/documentation/group__wheel__joint.html)이다. Box2D의 local frame 대신 Zonai의 origin anchor/localAxisA를 사용하며 현재 구현은 기본 축·스프링 범위다. 새 solver set이나 범용 제약 프레임워크는 만들지 않았다.

## Sandbox에서 비교하기

**조인트 / 서스펜션과 바퀴**는 정적 지지대와 동적 원을 연결한다. 지지대 원점 아래 1.3 m가 스프링 중립점이며 바퀴는 그보다 0.3 m 아래에서 시작한다. 중력 아래 진동하고 감쇠하여 평형 변위에 머물지만 바퀴 회전은 자유롭다. 청록색은 중립점/축의 양의 방향, 보라색은 실제 바퀴와 회전 방향, 회색 선은 이동 축이다. 지지대–바퀴의 보라색 연결선은 장치 표시이며 변위 0은 별도의 청록색 점이다.

‘서스펜션 스프링’은 기본 접혀 있다. 펼쳐 사용 여부·주파수 0~10 Hz·감쇠 비율 0~2를 조절한다. 직접 숫자 입력도 이 범위를 지킨다. API 자체는 이 UI 범위로 제한하지 않는다. 변위·축 옆 오차·스프링 힘·상대 각속도를 관찰하며 주파수와 감쇠가 진동에 주는 영향을 비교한다.

Canvas 위 Space는 위로 임펄스, S는 회전 임펄스, A/D는 수평 힘이다. 버튼은 현재 축 방향 임펄스와 회전 임펄스다. 왼쪽 drag는 Mouse Joint, 오른쪽 클릭은 커서 방향 임펄스를 사용한다. 입력 capture는 Mouse Joint만 제거하고 Wheel Joint는 유지한다. 스프링 off/0 Hz에서 중력으로 떨어지는 것은 자유 이동 동작이며 Reset은 새 World와 on/3 Hz/0.7을 복원한다. 지지대의 Inspector 회전을 바꾸면 축도 함께 움직인다.

## 검증과 다음 단계

NDEBUG 독립 `wheelJointTests`는 자유 축 이동/회전, 선형 운동량, A 회전의 d+r_a·유효 질량, 현재 A/B 회전과 warm 반작용, 중심 밖 연결점/COM, spring→수직의 최신 속도, 물리 spring relaxation bias, 수직 relaxation, 0 Hz/off/h/퇴화 cache를 검사한다. World는 substep 1/4의 중력 평형·반력·회전과 mass/COM/pose/cache/no-op/wake·삭제/ID 재사용·충돌 제외·Kinematic의 이동/회전 축을 검사한다.

Sandbox model은 키/물리 진행/축 오차·평형·자유 회전/Mouse/Reset을 확인한다. Windows ImGui headless는 네 데모의 전환/그리기와 접힌 스프링 toggle·실제 두 버튼·Hz/감쇠 숫자 clamp를 실행한다. CTest는 47개, Windows Release CI의 runtime 대상은 17개다. 기존 assert 기반 Release 공백과 실제 OS/GPU/창 시각 검증은 별도다. 최종 build·mutation·리뷰·원격 CI 결과는 작업 PR에 기록한다.

Sandbox ON 전체 Debug/Release와 Sandbox OFF 전체 Release 빌드 및 각각 CTest 47/47이 통과했다. A의 d+r_a 누락, 오래된 A축/B작용점, 물리 스프링의 relaxation bias 누락, 수직 제약의 relaxation bias 적용을 각각 주입한 다섯 mutation을 모두 검출한 뒤 원본을 복구했다. 독립 읽기 전용 검토에서 계산·수명·UI의 기능 문제는 없었고 유효 질량의 준비 시점 설명을 실제 Step 흐름에 맞춰 바로잡았다.

후속 [최소·최대 이동 범위](wheel-limit.md)를 추가하여 스프링을 끈 상태에서도 경계와 별도의 제한 힘을 비교한다. 현재 전체 force에는 제한 반력도 포함한다. 이 문서의 기본 구현 당시 범위와 검증 기록은 유지하며 최신 동작은 제한 기록을 따른다. 후속 [회전 모터](wheel-motor.md)는 상대 각속도와 최대 토크를 추가하고 coupled cache에 모터도 포함한다. 다음은 자동차 데모다. Ragdoll·조나이 선풍기/연결 장치와 같은 Sandbox의 천·유체·soft body·voxel·파괴·terrain 학습 목표도 유지한다.
