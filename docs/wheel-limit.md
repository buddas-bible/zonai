# Wheel Joint의 이동 범위 제한

2026-10-07, master `96d5958`의 기본 Wheel Joint에 부호 있는 최소·최대 변위를 추가했다. 서스펜션을 끄거나 0 Hz로 설정해도 경계는 유지되고 바퀴 회전은 자유롭다. 회전 모터는 다음 단계다. 기본 좌표계·물리 스프링·회전하는 축은 [기본 Wheel 기록](wheel-joint.md)을 따른다.

## API와 관찰값

`wheelJointDef`의 `enableLimit`은 기본 false, `lowerTranslation`/`upperTranslation`은 기본 0 m다. 두 값은 유한하며 `lower <= upper`여야 한다. 생성 이후 `setWheelJointLimit(id, enableLimit, lower, upper)`로 함께 바꾼다. 호출자가 순서를 지키며 API에서 임의의 범위를 정렬하거나 복구하지 않는다. 제한을 끈 상태에서도 같은 전제조건을 적용한다.

변위는 월드 B-A 연결점 차이를 현재 A의 이동 축에 투영한 값이다. 청록색 중립점에서 축의 양의 방향이 양수이며 하한·상한 모두 음수 또는 양수일 수 있다. `lower == upper`이면 해당 변위를 유지한다. Distance Joint의 같은 min/max가 기존 목표 거리로 돌아가는 규칙과 다르다.

`getWheelJointData`는 제한 설정과 별도의 `limitForce = (lowerImpulse-upperImpulse)/h`를 제공한다. B에 작용하는 힘(N)이며 양수는 현재 축 방향이다. 기존 `springForce`는 스프링만의 힘을 유지하고 전체 `force`에는 스프링·제한·수직 반력을 합친다. 마지막 substep의 누적 임펄스/h를 현재 축으로 표시하는 관찰값이며 paused/sleep에서 유지된다.

같은 제한 설정은 cache/sleep을 유지한다. 실제 제한 또는 스프링 설정 변경은 수직·스프링·양쪽 경계의 coupled cache를 함께 비우고 연결된 non-static component를 깨운다. 질량·pose 변경은 공통 reset, substep h 변경은 prepare에서 cache를 버린다. 삭제·ID 재사용·충돌 제외·Island는 기존 Joint 경로를 공유한다.

## 한쪽 방향 경계의 수학

현재 축 ax, 연결점 차이 d, 팔 길이 r_a/r_b에 대해 `translation = dot(ax,d)`, `a1 = cross(d+r_a,ax)`, `a2 = cross(r_b,ax)`다. 스프링과 같은 축 유효 질량을 사용하며 전체 Step 시작에 한 번 준비하는 근사를 유지한다. 현재 누적 회전·이동은 solve/warm start에 반영한다.

```
C_lower = translation-lower, C_upper = upper-translation
direction = +1 (lower), -1 (upper)
Cdot = direction * (dot(ax,vB-vA) + a2*wB - a1*wA)
newImpulse = max(0, oldImpulse + deltaImpulse)
P_axis = direction * (newImpulse-oldImpulse)
```

경계 안에서는 C/h를 bias로 사용해 남은 거리만큼 접근하도록 허용하고 바깥으로 통과하는 속도만 막는다. 위치가 경계를 위반한 경우 수치 안정화 bias/softness는 보정 pass에서만 적용한다. relaxation은 위반 위치의 bias를 새로 넣지 않는다. 누적 임펄스의 비음수 제한은 경계가 안쪽 복귀를 잡아당기는 일을 막는다. 매 반복은 누적값 전체가 아닌 증가량만 적용한다.

Box2D처럼 **스프링 → 하한 → 상한 → 수직 제약** 순서로 최신 속도를 읽는다. 물리 스프링은 relaxation에서도 복원 bias를 유지하며 경계는 스프링을 꺼도 작동한다. 중심 밖 연결점에서 제한이 만든 회전은 뒤의 수직 제약에 반영한다. Warm start는 `spring+lower-upper`의 축 임펄스와 수직 임펄스를 합쳐 양쪽에 반작용을 적용한다. 바퀴 각도를 잠그는 식은 없다.

비교 기준은 Box2D main [`ac7c751`의 wheel_joint.c](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/wheel_joint.c)와 [공식 Wheel 설명](https://box2d.org/documentation/group__wheel__joint.html)이다. 기존 Revolute limit의 작은 local lambda 패턴을 재사용했으며 새 범용 제약 계층은 만들지 않았다. 부드러운 경계이므로 하중 아래 작은 위치 오차가 남을 수 있다.

## Sandbox 실험과 검증

‘서스펜션과 바퀴’는 기존 on/3 Hz/감쇠 0.7의 스프링으로 시작한다. 제한은 off이며 편집용 범위 -0.5~0.5 m를 준비한다. ‘이동 범위 제한’은 기본 접혀 있고 최소·최대 변위를 -1~1 m 안에서 편집한다. 직접 숫자 입력도 UI 범위와 lower<=upper를 지킨다. API는 이 UI 범위로 제한하지 않는다.

제한을 켜면 초록색 최소 경계·빨간색 최대 경계와 회색 허용 구간을 현재 A축에 표시한다. 스프링 힘과 제한 힘을 따로 관찰한다. 스프링을 끈 뒤 중력으로 하한에 머무는 경우, 축 방향 버튼/Space로 상한에 접근하는 경우, 중립점을 범위 밖에 두어 두 힘이 서로 버티는 경우를 비교한다. S/회전 버튼은 자유 회전을 확인한다. Mouse/키/Reset은 기존 입력 흐름을 유지하며 Reset은 제한 off/-0.5~0.5 m로 돌아간다.

NDEBUG 단위 검사는 양쪽 부호·예측 경계·안쪽 복귀·누적 증가량·위반 위치의 bias/relaxation·같은 경계·현재 A축/이동·중심 밖 최신 속도·스프링 결합·warm sum/h/off/퇴화 cache를 확인한다. World는 substep 1/4의 spring off/0 Hz·중력 반력·두 힘의 평형·같은 경계·Kinematic 축·설정/no-op/wake·질량/pose/ID 재사용을 검사한다. Sandbox는 두 경계·중력·자유 회전·Reset과 ImGui headless 접기/toggle/숫자 clamp를 실행한다. 전체 CTest 수는 기존 47개, Windows Release runtime은 17개를 유지한다. 실제 OS/GPU/창 시각 확인과 기존 assert 기반 Release 공백은 별도다.

Sandbox ON 전체 Debug/Release 및 OFF 전체 Release 빌드와 각각 CTest 47/47이 통과했다. 상한 부호 반전·unilateral clamp 제거·경계 예측 누락·limit warm start 누락·제한을 수직 제약 뒤로 이동하는 다섯 mutation을 모두 검출하고 원본 byte를 복구했다. 독립 읽기 전용 리뷰의 0 Hz 주석을 수정하여 남은 지적이 없으며 원격 최종 head CI 결과는 작업 PR에 기록한다.

다음은 Wheel의 목표 상대 각속도·최대 토크를 갖는 회전 모터다. 제한·스프링·모터를 각각 확인한 뒤 자동차로 조합하며, ragdoll·조나이 연결 장치 및 같은 Sandbox의 천·유체·soft body·voxel·파괴·terrain 학습 방향도 유지한다.
