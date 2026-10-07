# Wheel Joint의 회전 모터

2026-10-07, master `9fc1c00`의 Wheel 서스펜션과 이동 제한에 목표 상대 각속도와 토크 한도를 추가했다. 바퀴 하나의 구동·역회전·제동을 관찰한 뒤 자동차로 조합하는 학습 단계다. 축 이동·스프링·경계의 좌표계는 [기본 Wheel](wheel-joint.md)과 [이동 제한](wheel-limit.md)을 따른다.

## API와 단위

`wheelJointDef`의 기본은 `enableMotor=false`, `motorSpeed=0`, `maxMotorTorque=0`이다. 생성 이후 `setWheelJointMotor(id, enableMotor, motorSpeed, maxMotorTorque)`로 함께 바꾼다. 목표는 `wB-wA`의 유한한 rad/s, 토크 한도는 유한한 비음수 N·m다. 양의 상대 속도는 B가 A보다 반시계 방향으로 회전하는 상태다. 속도 0은 제동하고 토크 0은 모터 힘을 끈다. 각도나 절대 회전 속도를 고정하는 제약은 아니다.

`getWheelJointData`는 모터 설정과 `motorTorque=motorImpulse/h`를 별도로 제공한다. B에 작용하는 부호 있는 토크(N·m)이며 마지막 substep의 관찰값이다. paused/sleep에서 유지된다. 기존 `force`는 선형 수직·스프링·제한 반력만 합치며 `springForce`와 `limitForce`의 의미도 유지한다.

같은 모터 설정은 cache/sleep을 유지한다. 실제 모터·스프링·제한 변경은 수직·스프링·양쪽 경계·모터의 결합된 cache를 함께 비우고 연결된 non-static component를 깨운다. 중심 밖 연결점에서는 회전과 선형 제약이 결합하기 때문이다. 질량·질량 중심·pose 변경은 공통 reset을 사용하고 h 변경은 prepare에서 warm start를 버린다. 삭제·ID 재사용·충돌 제외·Island는 기존 Joint 경로를 공유한다.

## 상대 각속도 제약

```
motorMass = 1 / (invInertiaA + invInertiaB)
deltaImpulse = motorMass * (motorSpeed - (wB-wA))
newImpulse = clamp(oldImpulse + deltaImpulse, -maxMotorTorque*h, maxMotorTorque*h)
appliedImpulse = newImpulse-oldImpulse
wA -= invInertiaA*appliedImpulse
wB += invInertiaB*appliedImpulse
```

두 역관성이 모두 0이면 유효 질량과 모터 cache를 0으로 둔다. 토크×substep 시간으로 **누적 각임펄스**를 제한하므로 반복 횟수나 substep 수가 토크를 부풀리지 않는다. 양쪽 Dynamic에는 반대 각반작용을 주고 Kinematic의 지정 속도는 바꾸지 않는다. Warm start에도 양쪽 각반작용을 포함하며 이전 모터 임펄스는 현재 한도 안에서만 재사용한다.

Box2D처럼 **모터 → 스프링 → 하한 → 상한 → 수직 제약** 순서로 갱신된 속도를 읽는다. 중심 밖 연결점에서는 모터가 만든 회전이 뒤의 선형 제약에 반영된다. 위치 bias는 모터에 넣지 않으며 보정과 relaxation 모두 같은 상대 속도 제약을 푼다. 토크가 작거나 다른 제약·하중과 결합하면 목표 속도에 도달하지 못할 수 있다.

비교 기준은 확인한 Box2D main [`ac7c751`의 wheel_joint.c](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/wheel_joint.c)와 [공식 Wheel API](https://box2d.org/documentation/group__wheel__joint.html)다. 기존 Zonai의 origin anchor/localAxisA·Step 시작 유효 질량 근사와 작은 solver 구조를 유지했다.

## Sandbox 실험

**조인트 / 서스펜션과 바퀴**는 스프링 on/3 Hz/감쇠 0.7, 제한 off/-0.5~0.5 m를 유지한다. 모터도 off이며 편집용 목표 3 rad/s·최대 토크 1 N·m를 준비한다. ‘회전 모터’는 기본 접혀 있다. 펼쳐 모터를 켜고 목표 -10~10 rad/s와 최대 토크 0~5 N·m를 편집한다. 직접 숫자 입력도 이 UI 범위를 지키며 API는 UI 범위로 제한하지 않는다.

‘회전 방향 바꾸기’는 목표의 부호를 바꾸고 ‘제동하기’는 모터를 켜서 목표를 0으로 바꾼다. 최대 토크 0에서는 제동하지 않는다. 현재 상대 각속도와 실제 토크를 목표·한도와 비교한다. 작은 토크로 가속 시간을 관찰하고, 제한과 스프링을 함께 켜서 축 이동과 회전이 독립적으로 조절되는지 확인한다. S/회전 버튼의 충격 뒤에는 모터가 다시 목표 속도를 향한다. Mouse·키·Reset과 기존 표시/충돌 UI를 유지하며 Reset은 모터 off/3 rad/s/1 N·m와 반력 0으로 돌아간다.

## 검증 범위

NDEBUG에서도 실행되는 단위 검사는 양쪽 각반작용·각운동량·정/역회전·제동·토크 0/off·누적 clamp·warm/h/퇴화 cache와 중심 밖 연결점의 최신 속도 순서를 확인한다. World는 substep 1/4의 실제 토크 한도·가속·목표 속도·스프링/제한 결합, Kinematic 상대 속도, no-op/wake·질량/pose/설정 변경·삭제/ID 재사용을 검사한다. Sandbox는 구동·역회전·Reset과 실제 ImGui headless 접기/toggle/버튼/속도·토크 직접 입력 clamp를 실행한다.

다섯 오류 변형 검사를 검출했다: 토크 예산의 h 누락, 누적 clamp 누락, A 반작용 부호 반전, motor warm start 누락, 모터를 제한 뒤로 이동. 원본 byte를 복구했다. 전체 CTest 47개와 Windows Release runtime 17개를 유지하며 최종 빌드·CI 결과는 작업 PR에 기록한다. 실제 OS/GPU/창의 시각 확인과 기존 assert 기반 Release 공백은 별도다.

후속 [모터 자동차](motor-car.md)에서 두 바퀴를 차체에 연결하여 하중·주행·제동과 마찰/경사면을 비교한다. 이 문서의 개별 모터 수학과 검증 기록은 유지한다. 다음은 관절 제한 렉돌이다. Ragdoll·조나이 선풍기/연결 장치, 같은 Sandbox의 천·유체·soft body·voxel·파괴·terrain 학습 방향도 유지한다.
