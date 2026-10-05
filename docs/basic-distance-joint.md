# 기본 Distance Joint — 구현과 학습 기록

2026-10-06, master `c8c8d718ea5caec69fd0c2abd054280e33ff0fb8`에서 시작했다. [검토한 설계](superpowers/specs/2026-10-06-distance-joint-design.md)의 기본 고정 거리 제약과 World 통합을 구현한다. Box2D 비교 기준은 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`의 `distance_joint.c`, `joint.c`, `body.c`, `solver.c`, `sensor.c`다. 사용자의 다음 작업 요청에 따라 구현했으며 spring/limit/motor는 다음 단계다.

## 거리 제약의 수학

Body origin 기준 local anchor를 질량 중심 기준으로 바꾼다. `rA = Rotate(qA, localAnchorA - localCenterA)`, B도 같다. World anchor 간 벡터는 `d = centerB - centerA + rB - rA`, 축은 `u = Normalize(d)`다.

`C = |d| - length`가 거리 오차이며, `Cdot = Dot(u, vB - vA + Cross(wB,rB) - Cross(wA,rA))`가 두 anchor 사이 축 방향 상대속도다. 회전까지 포함한 역 effective mass는 다음과 같다.

```text
K = invMassA + invMassB
  + invInertiaA * Cross(rA,u)^2
  + invInertiaB * Cross(rB,u)^2
axialMass = K > 0 ? 1/K : 0
deltaImpulse = -massScale * axialMass * (Cdot + bias)
             - impulseScale * accumulatedImpulse
P = deltaImpulse * u
vA -= invMassA * P; wA -= invInertiaA * Cross(rA,P)
vB += invMassB * P; wB += invInertiaB * Cross(rB,P)
```

Contact는 밀어낼 수만 있으므로 normal impulse를 0 이상으로 제한한다. Distance Joint는 당기기(음수)와 밀기(양수)를 모두 허용한다. 이 차이는 scalar 테스트의 momentum/relative-velocity/impulse 부호 검사로 확인한다. Kinematic 속도는 Cdot에 들어가지만 inverse mass/inertia는 0이므로 제약이 그 속도를 바꾸지 않는다.

Box2D rigid constraint처럼 bias pass는 거리 오차를 줄이고, 적분 후 relax pass는 bias 없이 상대속도를 줄인다. Prepare는 full step 시작에 한 번 수행하고 solve는 누적 deltaPosition/deltaRotation으로 현재 축과 lever arm을 다시 계산한다. 기본 60 Hz/damping 2, `min(60, 0.25/h)`는 rigid 제약의 수치 안정화 계수다. 물리적인 spring 옵션과 구분한다. 동일한 softness 계산을 공통 파일로 옮겼고 기존 Contact alias/wrapper는 유지한다.

## World 연결과 불변식

- `jointId`는 World lifetime token, slot+1, 16-bit generation으로 null/foreign/stale handle을 구분한다. 같은 World의 서로 다른 유효 Body 두 개이며 하나 이상 Dynamic이어야 한다. 유한한 local anchor와 양수 length가 전제조건이고 length는 `LINEAR_SLOP` 이상으로 제한한다.
- `joint2`는 수명과 두 Body의 intrusive edge를, 같은 slot의 `distanceJointSim2`는 anchor/length/누적 impulse를 보관한다. Body 삭제는 Joint → Contact → Shape/proxy 순서다. 삭제한 slot은 sim을 비운 뒤 free-list에 반환한다.
- `collideConnected=false`인 같은 Body 쌍의 Joint가 하나라도 있으면 기존 Contact를 제거하고 새 Contact와 CCD 후보를 차단한다. 작은 Joint list를 조회하며 positive shape group도 이 결정을 덮어쓰지 않는다. 차단된 후보는 pairSet에 등록하지 않는다. 차단 Joint 삭제 뒤 양쪽 live proxy를 touch하여 정지한 쌍도 다시 찾는다. 다른 차단 Joint가 남으면 계속 차단한다.
- Island와 wake/sleep은 Contact와 Joint를 함께 따라간다. Static은 endpoint이지만 union/wake 경유점은 아니므로 공통 Static anchor가 서로 다른 Island를 연결하지 않는다. Contact 없는 Joint chain도 함께 깨고 잠든다. Joint 생성/삭제는 non-static endpoint의 component만 깨우고, Static pose 변경은 모든 연결된 이웃을 깨운다.
- 각 Island에서 Joint warm start/bias/relax를 Contact보다 먼저 수행한다. Contact 순서와 restitution 단계는 유지한다. Joint impulse는 mass/COM/pose 변경 때 초기화하고 h 변경 시 준비 단계에서 버린다. `Step(0)`은 collision/sensor를 갱신하며 Joint solve와 force 소비를 수행하지 않는다.
- Box2D의 discrete sensor overlap은 shape filter만 사용하므로 Joint 차단을 적용하지 않는다. Continuous sensor 후보는 Body/Joint filter를 통과해야 한다. 두 경로를 별도로 테스트한다.

## 사용과 관찰

```cpp
zonai::world simulation;
const auto anchor = simulation.CreateBody();
const auto bob = simulation.CreateBody( zonai::bodyType::Dynamic, { { 0.0f, -2.0f }, {} } );
(void)simulation.CreateShape( bob, zonai::circle2{ {}, 0.3f } );
zonai::distanceJointDef definition{};
definition.bodyA = anchor;
definition.bodyB = bob;
definition.length = 2.0f;
const auto joint = simulation.createDistanceJoint( definition );
simulation.Step( 1.0f / 60.0f, 4 );
const auto data = simulation.getDistanceJointData( joint ); // World anchors와 target/current length의 값 복사
simulation.destroyJoint( joint );
```

Sandbox의 보라색 진자는 고정 거리 제약을 표시한다. `Kick pendulum`으로 옆으로 밀고 Target/Current 길이, anchor, COM, velocity를 함께 관찰할 수 있다. 이후 localAnchor를 COM에서 옮겨 같은 impulse가 선속도와 각속도로 나뉘는 이유를 비교하면 된다. 이번 검증은 Sandbox build이며 UI 실행/시각 검증은 수행하지 않았다.

## 검증

새 `distanceJointTests`, `distanceJointWorldTests`는 NDEBUG와 무관한 runtime check다. Scalar의 질량·운동량·인장/압축·COM/토크·kinematic·warm start/h 변경·zero axis와 World의 handle 재사용/World 재생성·storage growth·edge 삭제·Body cascade·multiple blockers·stationary pair·positive group·mixed graph·static 경계·sleep/wake·substep 거리·mass/pose/h reset·Step(0)·solid/sensor CCD를 검사한다.

Scalar header와 World API가 없는 상태에서 각각 build 실패를 먼저 확인한 뒤 구현했다. Release에서 부호 clamp, COM 변환, h cache, mass/pose reset, CCD filter, stationary proxy touch, Joint wake, Island union을 하나씩 잘못 바꾸는 9가지 실험도 모두 새 테스트가 거부했다. 실험 변경은 원본으로 복원했다. Reset 비교는 같은 slot 순서로 재생성한 Joint와 기존 Joint의 상태가 정확히 같아야 한다는 검사다. 두 제약을 사용해 독립된 단일 축 solve가 cached impulse를 상쇄하는 경우도 피한다.

Windows MSVC에서 Sandbox 포함 전체 Debug/Release build와 CTest 40/40 실행을 확인한다. 기존 assert 기반 29 target의 검사가 Release에서 제거되므로 두 설정의 전체 통과 수를 동등한 검증으로 해석하지 않는다. Windows Release CI는 runtime check가 유지되는 기존 8 target과 새 Joint 2 target, 총 10개를 명시적으로 build/run한다. 원격 Windows/Ubuntu 결과는 PR에 기록한다. 독립 리뷰에서 공통 Static anchor를 통한 Joint 생성/삭제의 불필요한 wake 전파를 찾았고, 실패하는 회귀 검사를 먼저 추가한 뒤 lifecycle 호출에서 Static endpoint를 제외했다. Static pose 변경의 이웃 wake는 유지한다.

## 한계와 다음 학습 단계

고정 거리 제약은 반복 수·substep 수에 따른 오차가 있으며 정확한 기하학적 투영을 보장하지 않는다. 완전히 겹친 두 anchor는 축이 0이어서 방향성 correction을 선택하지 못하지만 유한한 상태를 유지한다. Body/Joint handle의 16-bit generation wrap 한계는 기존 정책과 같다.

CCD는 Joint solve 뒤 개별 Body의 이동을 자를 수 있다. 그 frame에서는 연결된 거리도 어긋날 수 있고 다음 solve에서 회복한다. 연결된 Island 전체를 TOI로 다시 푸는 기능은 이번 범위에 없다. 기존 moving-target bounds, sensor budget, bullet/bullet 제한도 유지된다.

다음 구현은 **Distance Joint spring의 Hertz/damping**이다. 먼저 현재 rigid bias/relax와 spring softness의 차이를 설명하고, 주파수·감쇠에 따른 응답을 작은 진자 실험으로 비교한다. Limit/motor는 그 뒤 같은 축 Jacobian을 활용하는 별도 학습 단계로 진행한다.
