# 회전 조인트의 속도 모터

2026-10-07, master `c2d46eb`의 기본 회전축·각도 제한에 목표 각속도와 최대 토크를 추가했다. 연결점을 유지하면서 B의 A에 대한 상대 각속도를 조절한다. transform이나 각속도를 목표 값으로 직접 덮어쓰지 않고, 제한된 각임펄스로 가속·제동한다.

## API와 관찰값

`revoluteJointDef`에 `enableMotor`, `motorSpeed`, `maxMotorTorque`를 추가했다. API 기본은 off/0/0이며 기존 자유 회전을 유지한다. 속도는 rad/s, 토크는 N*m다. 유한한 속도와 유한한 비음수 최대 토크를 precondition으로 요구한다. 양의 속도는 B가 A에 대해 반시계로 회전하는 방향이다. 속도 0은 제동이고 최대 토크 0은 모터 힘을 끈다. World API는 Sandbox 슬라이더 범위로 제한하지 않는다.

`setRevoluteJointMotor(id, enableMotor, motorSpeed, maxMotorTorque)`로 세 값을 함께 바꾼다. 실제 변경은 연결점·하한·상한·모터 cache를 모두 비우고 연결된 non-static component를 깨운다. 같은 설정은 cache와 sleep을 유지한다. 각도 제한 변경도 모터 cache를 비운다. Box2D의 개별 setter 대신 기존 Zonai Distance setter의 규칙을 따른다. 질량·관성·pose 변경과 substep 시간 변경에도 오래된 모터 임펄스를 적용하지 않는다.

조회에 설정과 `motorTorque`를 추가했다. **마지막 substep의 motorImpulse / h**이며 B에 작용하는 모터 토크다. 기존 `torque`는 계속 각도 제한만의 `(lowerImpulse - upperImpulse) / h`다. Box2D의 일반 torque query처럼 모터를 합산하는 값으로 바꾸지 않았다. 연결점 임펄스가 만드는 `r x P`도 두 값에 포함하지 않는다. 각도 경계에서 모터와 제한 토크가 반대로 작용하는 현상을 따로 관찰할 수 있다. Paused/sleep 상태는 마지막 결과를 유지한다.

## 속도 제약과 토크 한도

```
w = wB - wA
angularMass = 1 / (invInertiaA + invInertiaB)
delta L = angularMass * (motorSpeed - w)
max L = maxMotorTorque * h
accumulated L = clamp(old L + delta L, -max L, max L)
applied L = accumulated L - old L
```

두 물체에 반대 방향 각임펄스를 적용한다. 누적값을 제한하므로 solver 반복 횟수가 늘어도 한 substep의 토크 한도를 넘지 않는다. 속도 제약에는 위치 bias나 사용자 스프링을 추가하지 않는다. biased/relaxation pass 모두 같은 모터 식을 쓰며, 회전 역질량이 둘 다 0이면 모터 임펄스를 비운다. Warm start는 현재 한도에 맞춘 모터 cache와 `lower - upper`를 합쳐 적용한다.

Box2D처럼 **모터 → 하한 → 상한 → 연결점** 순서로 풀고 직전 제약이 갱신한 속도를 읽는다. 모터는 목표 속도를 추구하지만 각도 제한이 바깥 회전을 막는다. 토크가 충분하지 않으면 하중 아래 목표 속도에 도달하지 못한다. 중심 밖 연결점은 선형·회전 운동을 결합하며 반복 횟수와 하중에 따라 작은 연결점/속도/경계 오차가 남을 수 있다.

비교 기준은 Box2D main [`ac7c751`의 revolute_joint.c](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/revolute_joint.c)와 [공식 Revolute API 설명](https://box2d.org/documentation/group__revolute__joint.html)이다. Spring, local frame/solver set 구조는 이 작업에서 추가하지 않는다. 각도는 기존처럼 wrap되며 회전 횟수를 누적하지 않는다.

## Sandbox 실험

‘조인트 / 회전축과 막대’는 모터와 제한을 끈 자유 회전으로 시작한다. **회전 모터**는 기본 접혀 있으며 목표 2 rad/s, 최대 10 N*m가 준비돼 있다. 펼쳐 모터를 켜고 속도 -5~5 rad/s, 토크 0~50 N*m를 조절한다. 직접 숫자 입력도 범위를 지킨다. ‘회전 방향 바꾸기’는 속도 부호를 바꾸고 모터 사용 여부는 유지한다. ‘제동하기’는 모터를 켜고 속도를 0으로 만든다. 최대 토크가 0이면 제동력도 없다.

현재 상대 각속도와 실제 모터 토크를 관찰한다. 토크 10에서 중력 하중에 걸릴 수 있으므로 20으로 올려 연속 회전을 비교한다. 각도 제한을 함께 켜면 경계에서 모터가 힘을 주지만 막대는 멈추며, 제한 토크가 반대로 작용한다. 방향을 바꾸면 경계에서 안쪽으로 돌아온다. 기존 impulse 버튼·키·Mouse Joint는 유지하고 Reset은 모터 off/2 rad/s/10 N*m로 돌아간다.

## 검증 범위와 다음 단계

기존 NDEBUG 독립 Revolute 단위 검사는 상대속도·양쪽 반작용·각운동량, 누적 토크 한도, 제동/0 토크, 제한과 순서, warm start의 부호/한도, h 변경·off·고정 회전 cache를 다룬다. World는 substep 1/4에서 가속 한도·목표 속도·외부 하중·제동·역회전·경계의 상쇄 토크·설정/질량/pose cache·sleep/wake·slot 재사용과 움직이는 Kinematic 축을 검사한다.

Sandbox model은 중력 아래 중심 밖 막대가 실제로 회전하면서 연결점과 토크 한도를 유지하는지 확인한다. Windows ImGui headless 검사는 기본 접기·toggle·역회전·제동·숫자 입력 clamp를 실행한다. 실제 OS/GPU/창 시각 검증은 별도다. 전체 CTest 45개와 Windows Release runtime 15개를 유지하며, 기존 assert 기반 Release 검증 공백은 이 작업의 범위가 아니다. 최종 빌드·mutation·리뷰·원격 CI 결과는 작업 PR에 기록한다.

최종 로컬 검증은 Sandbox ON 전체 Debug/Release build와 각각 45/45 CTest, Sandbox OFF 전체 Release build와 45/45 CTest 통과다. 토크의 h 누락, 누적 한도 제거, A의 반작용 부호 반전, warm start의 모터 누락, 모터를 제한 뒤로 옮긴 다섯 mutation을 모두 검사에서 잡았다. 원본을 byte 단위로 복구한 뒤 다시 통과했으며 독립 최종 리뷰의 남은 지적은 없다.

다음 권장 단계는 **Wheel Joint의 서스펜션·바퀴 회전 실험**이다. 회전 모터와 이동 축·스프링을 결합하는 작은 실험을 확인한 뒤 motor 자동차 데모로 이어갈 수 있다. Ragdoll·조나이 선풍기/연결 장치와 천·유체·soft body·voxel·terrain을 같은 Sandbox에서 학습하는 목표도 유지한다.
