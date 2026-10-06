# Distance Joint motor

기존 Distance spring/limit에 목표 축속도와 최대 힘을 추가했다. 회전 모터가 아니라 두 작용점 사이의 거리를 늘이거나 줄이는 모터다. HyruleEngine/원본 Phyzzle에서 확인한 계산 단계·수학 표기·이유 주석은 [코딩 스타일](coding-style.md)을 따른다.

## 모드와 API

`distanceJointDef`와 조회 결과에 `enableMotor`, `motorSpeed`, `maxMotorForce`를 추가했다. 기본값은 false/0/0으로 기존 시뮬레이션을 유지한다. `setDistanceJointMotor(id, enableMotor, motorSpeed, maxMotorForce)`로 실행 중 변경한다. 속도는 유한한 값, 최대 힘은 유한한 비음수이며 유효한 Distance Joint ID가 필요하다. 기존 API처럼 Debug assert로 사전 조건을 검사하며 Release 입력 복구 API를 추가하지 않는다.

| 설정 | 동작 |
| --- | --- |
| Spring off | `length`의 고정 거리. 모터 설정을 보관하지만 힘을 주지 않는다. |
| Spring on, Hertz 0 | 스프링 힘 없이 모터·limit을 각각 사용할 수 있다. |
| Spring on, Hertz > 0 | Spring → motor → lower → upper 순서로 계산한다. |
| Limit on, min = max | 기존처럼 `length`의 고정 거리로 돌아가며 모터도 무시한다. |
| Motor speed > 0 / < 0 | 거리를 늘임 / 줄임. A에서 B로 향하는 축의 상대속도다. |
| Motor speed = 0 | 최대 힘 범위에서 축 방향 운동을 제동한다. |
| Max motor force = 0 또는 Motor off | 모터 임펄스만 0. 다른 spring/limit은 유지한다. |

```cpp
distanceJointDef definition{};
definition.bodyA = anchor;
definition.bodyB = bob;
definition.length = 2.0f;
definition.enableSpring = true;
definition.hertz = 0.0f;
definition.enableLimit = true;
definition.minLength = 1.5f;
definition.maxLength = 2.5f;
definition.enableMotor = true;
definition.motorSpeed = 1.0f;
definition.maxMotorForce = 10.0f;
const auto joint = simulation.createDistanceJoint( definition );
simulation.setDistanceJointMotor( joint, true, -1.0f, 10.0f );
```

같은 설정은 캐시와 sleep을 유지한다. 모터 설정 변경은 서로 같은 축을 사용하는 네 임펄스를 비우고 연결된 non-static component를 깨운다. Spring/limit 변경, Body mass/pose 변경도 모터 캐시를 비운다. Static anchor를 공유하는 다른 island는 깨우지 않는다. Substep 시간이 달라지면 이전 모터 임펄스를 사용하지 않는다.

## 계산과 힘

축 방향 유효 질량과 작용점의 상대속도는 기존 Distance 제약을 사용한다. `v_r = v_b + w_b × r_b - v_a - w_a × r_a`에서 축 성분 `v`를 구하고, `deltaImpulse = axialMass * (motorSpeed - v)`를 계산한다. Spring이 바꾼 최신 속도를 읽으며 회전 효과도 포함한다. 모터에는 위치 오차 bias가 없고 보정·완화 두 pass에서 같은 식을 사용한다.

최대 힘 F와 substep 시간 h를 곱한 `maxImpulse = F * h`로 **누적 모터 임펄스**를 양방향 제한한다. 매 반복에서의 증가량만 제한하면 반복 횟수만큼 총 힘이 커질 수 있다. 제한한 새 누적값과 이전 값의 차이만 두 Body에 반대 부호로 적용한다. 같은 h의 warm start도 제한 안의 값만 재사용한다.

Warm start 합은 `impulse + lowerImpulse - upperImpulse + motorImpulse`다. `axialForce`는 이 합/h, `motorForce`는 motorImpulse/h로 마지막 solved substep의 값을 보여준다. 모터가 경계를 미는 동안 모터 힘과 limit 힘이 상쇄되어 합력 0일 수 있다. Pause/sleep에서는 마지막 결과를 유지한다. 양의 모터 힘은 거리를 늘리는 방향이다.

모드 우선순위, 누적값 제한, spring→motor→limit 순서는 현재 [Box2D Distance Joint](https://github.com/erincatto/box2d/blob/main/src/distance_joint.c)의 의도를 따른다. Zonai는 기존 묶음 setter·명시적 cache 초기화·component wake 정책을 유지한다. 최신 Box2D의 spring force range나 다른 Joint 기능을 이번 모터 단계에 추가하지 않았다.

## 진자에서 실험하기

거리 조인트 진자는 기존처럼 rigid로 시작한다. ‘거리 스프링’을 켜고 주파수를 0으로 낮춘 뒤 접힌 ‘거리 모터’를 열어 ‘모터 사용’을 켠다. 기본 속도 1 m/s와 최대 힘 10 N으로 거리가 늘어나는 모습을 보고, 속도를 음수로 바꾸거나 0으로 바꾸어 줄이기·제동을 비교한다. ‘거리 제한’을 켜면 1.5–2.5 m 경계와 모터 힘을 관찰한다. 주파수를 올리면 스프링의 힘과도 비교할 수 있다.

목표 축속도는 -5–5 m/s, 최대 힘은 0–50 N이며 직접 입력도 같은 범위로 제한한다. 고정 거리 모드에는 모터 비활성 이유를 표시하고 설정값은 보관한다. Reset은 모터 off/1 m/s/10 N과 기존 spring·limit 기본값으로 돌아간다. 기존 표시 기본값과 마우스·키 조작을 유지한다.

## 검증과 범위

기존 두 Distance runtime test와 Sandbox test에서 힘 상한·운동량·양방향 속도·제동·편심 torque·spring 이후 최신 속도·limit 순서·warm start·시간/질량/pose/설정 변경·wake/no-op·force 조회와 Reset을 검사한다. World 실험은 substep 1/4에서 비교한다. ImGui headless 검사는 모터 checkbox와 두 slider의 직접 입력 제한을 실제 frame에서 실행한다. 새 target을 만들지 않아 전체 CTest 43개와 Windows Release runtime 13개를 유지한다.

위치 덮어쓰기나 단단한 limit을 추가하지 않았다. 기존 limit의 softness, 일치하는 작용점의 방향 부재, CCD clipping, 극단적인 유한 계수의 overflow 범위는 기존 구현과 같다. Headless UI 검사는 실제 창/GPU/OS 입력과 시각 검증을 대신하지 않는다. 최종 빌드·회귀·리뷰·CI 결과는 작업 PR에 기록한다.

다음은 **Revolute Joint**로 회전 축을 학습하고, 이후 각도 제한·회전 모터와 ragdoll·자동차 등의 조합 실험으로 이어간다.
