# Mouse Joint — 점을 잡아 끄는 제약

2026-10-06, master `d4420e9b30d8680fc89b23f27db571588535bb80` 이후 구현한다. 목적은 데모에서 물체를 조작하면서 local/world 좌표, COM, effective mass와 soft constraint의 응답을 학습하는 것이다. Body pose를 cursor로 덮어쓰지 않는다.

## 수학과 Box2D 근거

클릭 시 `localAnchorB = InverseTransformPoint(bodyTransform, target)`를 저장한다. Prepare에서 `r = Rotate(q, localAnchorB - localCenter)`, `deltaCenter = center - target`를 구한다. Solve에서 누적 deltaRotation/deltaPosition을 반영해 현재 점의 오차 `C = deltaCenter + deltaPosition + r`와 속도 `Cdot = v + Cross(w,r)`를 계산한다.

```text
K = [ invMass + invI*r.y²      -invI*r.x*r.y       ]
    [ -invI*r.x*r.y            invMass + invI*r.x² ]
M = inverse(K)
deltaP = -massScale*M*(Cdot + biasRate*C) - impulseScale*P
P = clampLength(P + deltaP, h*maxForce)
v += invMass*(P - oldP); w += invI*Cross(r, P - oldP)
```

x/y는 중심 밖의 점에서 회전으로 결합된다. 축별 scalar 두 개로 풀면 이 결합을 잃는다. Force는 N이고 impulse는 N·s이므로 substep 시간 `h`를 곱한 누적 vector 전체를 제한한다. `hertz`/`dampingRatio`는 기존 `makeConstraintSoftness`로 변환하며, spring은 두 solver pass 모두에서 위치 bias를 유지한다. Rigid Distance Joint의 bias/relax와 다르다. Hertz 0은 positional spring 없이 점 속도를 감쇠하고, maxForce 0은 pull을 끈다.

현재 Box2D 기준 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`의 `samples/sample.cpp`는 kinematic mouse body와 Motor Joint linear spring으로 drag를 구성한다. 이 점 spring의 Jacobian/softness/vector force 제한은 `src/motor_joint.c`와 대조했다. 별도 Mouse Joint의 작은 one-body 형태는 [Box2D v3.1.1 mouse_joint.c](https://github.com/erincatto/box2d/blob/v3.1.1/src/mouse_joint.c)와 비교했다. Zonai는 Static A와 world target만 지원하고 B에 반작용을 적용한다. Box2D v3.1.1의 별도 angular friction은 추가하지 않으며, 현재 Motor Joint 전체나 moving A 지원을 구현한 것은 아니다.

## World와 수명

`createMouseJoint`는 Static A, Dynamic B와 유한한 world target/비음수 tuning을 받는다. A는 수명/graph 연결용이며 A의 pose는 target 좌표계가 아니다. 클릭한 점은 Body origin 기준으로 저장하고 solver에서는 COM 기준으로 바꾼다. `setMouseJointTarget`은 target을 바꿀 때 B의 component를 깨운다. `setMouseJointTuning`은 계수를 바꿀 때 cached impulse를 비우고 깨운다. 값이 같으면 sleep을 불필요하게 해제하지 않는다. 각 종류의 data/tuning 함수에는 해당 종류의 유효한 Joint ID를 전달한다.

공통 `joint2`의 cold edge를 Island/wake/lifecycle에 사용한다. 같은 slot의 hot sim과 transient constraint는 Distance/Mouse variant로 분기하며 기존 순서와 substep 구조를 유지한다. Body 삭제, generation/world token, free-list와 collision filter를 공유한다. Mouse Joint는 기존 ground Contact를 차단하지 않는다. Mass/COM/pose 변경은 두 종류의 cached impulse를 초기화하고, h 변경은 prepare에서 warm cache를 버린다. `getMouseJointData().force`는 마지막 solver의 누적 impulse/h 값이다.

## 조작과 관찰

Playground와 Distance Pendulum에서 **왼쪽 드래그**로 Dynamic solid를 잡는다. 센서·Static·Kinematic·면적 없는 segment는 잡지 않는다. 오른쪽 클릭은 이전 cursor 방향 impulse 실험이다. 주황색 선과 점은 현재 anchor/target이고, 설정에는 Hertz/damping/max force와 현재 point error/force가 나온다. Paused 상태는 target/Joint만 갱신하며 실제 pose는 Step/Play에서 진행한다.

버튼 해제·UI capture·창 focus 상실·Canvas 밖·middle pan·Play/Pause 전환·데모 교체/Reset은 drag를 취소한다. 취소 후 버튼을 계속 누르는 것만으로 다시 잡지 않으며 새 press가 필요하다. 삭제된 Body의 Joint ID는 World가 무효화하고 다음 입력/cancel에서 안전하게 비운다. 물리 속도는 취소 때문에 0으로 바꾸지 않는다.

Picking은 데모가 알고 있는 작은 shape 목록을 역순으로 검사해 나중에 그린 Dynamic solid를 우선한다. `world::testShapePoint`는 point를 local 좌표로 옮겨 실제 circle/capsule/convex polygon을 검사한다. Rounded polygon의 corner는 edge 거리로 검사한다. World overlap/ray/cast query와 spatial query filter는 이번 범위에 추가하지 않는다. 큰 장면이 실제로 필요해지면 후보 수집을 Tree query로 바꾼다.

## 검증과 한계

`mouseJointTests`는 2축/비대각 mass, COM/torque, force budget/warm clamp, h cache, Hertz 0와 singular mass를 runtime check로 검사한다. `mouseJointWorldTests`는 실제 이동·회전·point convergence, wake/cache reset, stale/foreign ID, mixed Distance/Mouse storage/solver/Body cascade와 회전·rounded geometry picking을 검사한다. `sandboxDemoTests`는 drag begin/move/release/cancel/reset/Body 삭제를 검사하며 Windows에서는 active drag의 실제 ImGui control/draw frame도 실행한다. 새 API가 없는 build 실패부터 확인했다.

Windows Sandbox 포함 전체 Debug/Release build와 각각 43/43 CTest, Sandbox OFF Release의 세 Mouse/Sandbox runtime target을 확인했다. COM·vector force 제한·warm torque·h cache·target wake·mass/pose cache reset·local picking·drag 취소를 망가뜨린 8개 mutation 모두 검사에서 실패했고, 원본 복구 후 통과했다. 최종 리뷰는 Ctrl+click 숫자 입력이 slider 범위를 벗어날 수 있음을 찾았다. 실제 ImGui SliderFloat text 경로에서 `-1`이 저장되는 Release 실패를 재현하고, 세 slider를 AlwaysClamp로 수정해 음수와 상한 초과 입력을 모두 검사한다. 최종 검토/원격 Windows·Ubuntu CI 결과는 PR에 기록한다. Headless ImGui frame은 native 창/GPU/OS focus/input/화면 배치를 대신하지 않는다. 기존 29 assert 기반 target의 Release 공백도 유지한다. CCD는 Joint 이후 개별 Body 이동을 제한할 수 있어 일시적인 점 오차가 생길 수 있다. 고정 prepare mass와 큰 회전의 오차, 16-bit generation wrap은 기존 범위의 한계다.

다음은 **Distance Joint spring의 Hertz/damping**이다. 이번 Mouse point spring과 rigid Distance 안정화의 차이를 진자 데모에서 비교한 뒤 limit/motor와 다른 Joint, ragdoll/자동차/조나이 연결 장치로 확장한다.
