# Distance Joint spring

2026-10-06, master `52d35727274c05d7575bfac2d74ae7fe8c540026` 이후 구현했다. 기존 scalar Distance constraint에 spring 모드만 추가한다. Box2D 비교 기준은 다시 fetch한 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`의 `src/distance_joint.c` prepare/solve와 `src/solver.h` softness다. 고정 거리 수학·수명·collision/Island 연결은 [기본 Distance Joint](basic-distance-joint.md)를 따른다.

## Rigid와 spring의 차이

| 설정 | 거리 축의 동작 |
| --- | --- |
| `enableSpring = false` (기본값) | 기존 고정 거리 제약. bias pass의 수치 안정화와 적분 뒤 bias 없는 relax를 사용한다. |
| `enableSpring = true`, Hertz > 0 | `length`가 자연 길이인 양방향 spring. 두 pass 모두 물리 spring의 softness/bias를 사용한다. |
| `enableSpring = true`, Hertz = 0 | 거리 축 impulse를 적용하지 않는다. Rigid로 돌아가지 않으며 이전 impulse도 warm start하지 않는다. |

Hertz 0의 의미는 Box2D Distance spring을 따른다. Mouse Joint의 Hertz 0 point velocity 제약과는 다르다. Joint 자체는 남으므로 연결 관계·collision filter·Island는 유지된다.

축 단위벡터 n, COM lever arm rA/rB에 대해 유효 질량은 `mEff = 1 / (invMassA + invMassB + invInertiaA * cross(rA,n)^2 + invInertiaB * cross(rB,n)^2)`다. 회전·질량 분포가 거리 축 응답에 함께 참여한다. 완전히 겹친 anchor는 방향을 정할 수 없어 correction이 0이다.

각주파수 `ω = 2π * Hertz`, 감쇠비 `ζ`라 하면 강성 `k = mEff * ω²`, 감쇠 계수 `d = 2 * ζ * mEff * ω`다. Hertz가 클수록 축 진동이 빨라지고, ζ = 0은 감쇠 없는 진동, ζ = 1은 축의 선형 모델에서 임계 감쇠다. 유한 step의 수치 감쇠와 큰 회전 때문에 실제 장면이 항상 이 모델의 정확한 궤적을 따르지는 않는다.

substep h에 공통 softness 계산을 사용한다. `a1 = 2ζ + hω`, `a2 = hω*a1`, `a3 = 1/(1+a2)`에서 biasRate = ω/a1, massScale = a2*a3, impulseScale = a3다. 축 오차 C와 상대 축속도 v에 대한 increment는 `Δλ = -massScale*mEff*(v + biasRate*C) - impulseScale*λ`다. Spring relax에서도 이 식을 유지해야 실제 spring 강성·감쇠가 rigid velocity constraint로 바뀌지 않는다. 인장과 압축을 모두 허용하며 force cap은 없다.

## API와 관찰

```cpp
distanceJointDef definition{};
definition.bodyA = anchor; definition.bodyB = bob; definition.length = 2.0f;
definition.enableSpring = true; definition.hertz = 2.0f; definition.dampingRatio = 0.7f;
const auto joint = simulation.createDistanceJoint( definition );
simulation.setDistanceJointSpring( joint, true, 3.0f, 1.0f );
const auto data = simulation.getDistanceJointData( joint );
```

typed getter/setter는 유효한 Distance Joint ID를 받는다. Hertz와 dampingRatio는 finite이며 0 이상이어야 한다. 기존 API와 같이 assert precondition이며 Release에서 잘못된 입력을 복구하는 API는 아니다. 변경은 cached impulse를 지우고 두 non-static endpoint의 연결 component를 깨운다. 같은 값은 sleep을 깨우지 않는다. Static anchor를 통해 별도 component를 깨우지 않는다. Body mass/pose 또는 h 변경의 기존 cache 무효화도 유지한다.

`data.axialForce`는 **마지막 solved substep**의 scalar impulse/h이며 단위는 N이다. 음수는 인장, 양수는 압축이다. 생성·tuning 변경 뒤에는 0이고, paused/sleeping 상태에서는 마지막 결과를 보관한다. `currentLength - length`로 늘어남을 관찰한다. 중심에 매단 bob의 중력 평형에서는 늘어남이 `g/ω²`, 힘의 크기가 mg에 가까워진다.

Distance Pendulum은 처음에는 rigid다. Controls에서 Distance spring을 켜고 Hertz/damping을 조절한다. `Radial kick`은 바깥쪽 축속도 2 m/s에 해당하는 impulse로 spring 진동을 시작하며 기존 `Kick pendulum`은 옆으로 민다. Mouse drag로 잡아 늘이거나 Inspector에서 pose를 바꿀 수 있다. Reset은 rigid/2 Hz/감쇠 0.7로 돌아간다. Purple line은 현재 두 anchor를 연결한다. UI 숫자 직접 입력도 Hertz 0–30, 감쇠 0–2로 제한한다.

## 검증 범위

기존 runtime `distanceJointTests`, `distanceJointWorldTests`, `sandboxDemoTests`에 회귀를 추가했다. 검사식은 NDEBUG에서도 유지된다. 축 impulse의 독립 implicit Euler 기대값과 momentum, spring relax, 감쇠 증가, Hertz 0의 free axis/cache, 중력 평형 늘어남·힘, 2/4 Hz의 진동 시간, 감쇠 없는 overshoot와 임계 감쇠 정착을 검사한다. Substep 1/4, 실시간 mode/tuning과 no-op sleep, mass/pose/h 변경의 cache 무효화, demo Reset도 포함한다.

실제 ImGui headless frame에서 두 spring slider의 음수/상한 초과 text 입력, spring checkbox 왕복, Radial kick을 검사한다. 실제 native 창·GPU·OS 입력·시각 배치 검증은 아니다. 전체 CTest는 43개, 원격 Windows Release runtime 대상은 기존 13개다. 나머지 29개 assert 기반 테스트의 Release 검증 공백은 유지되며 전체 통과 수를 Debug와 동등하게 해석하지 않는다. 최종 build/CTest·mutation·원격 CI 결과는 이 작업 PR에 기록한다.

Limit/motor/force bounds는 아직 없다. CCD clipping의 일시적인 거리 오차와 기존 coincident-anchor 한계도 유지한다. 다음은 **Distance limit**의 min/max, unilateral impulse와 spring/rigid의 상호작용을 같은 진자 실험에서 학습하는 단계다. 그 뒤 motor 및 revolute/wheel 등 필요한 Joint를 연결 데모로 확장한다.
