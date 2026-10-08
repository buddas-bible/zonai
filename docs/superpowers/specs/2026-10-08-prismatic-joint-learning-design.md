# Prismatic Joint — 단계별 학습형 완성 설계

기준: master `927127aeed76822c5d22eaf98756dfad6195c2e4`, Box2D 비교 기준 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`.

## 목표

Prismatic Joint를 한 번에 완성품으로 추가하지 않고, **기본 제약 → 이동 제한 → 선형 모터 → 스프링** 순으로 기능을 단계별 구현한다. 각 단계는 독립적으로 테스트 가능하고 Sandbox에서 눈으로 확인할 수 있어야 하며, 단계가 끝날 때마다 수식과 Zonai 코드의 대응 관계를 설명해 학습에 사용한다.

최종 목표는 Box2D 계열의 Prismatic Joint와 같은 핵심 의미를 갖는 완성형이다.

```text
허용: A의 로컬 축을 따른 B의 병진 운동
차단: 축에 수직한 병진 운동
차단: A와 B의 상대 회전
추가: 축 방향 spring / limit / motor
```

Wheel Joint와 비슷해 보이지만 핵심 차이는 회전 자유도다.

```text
Wheel Joint
- 축 방향 이동: 허용
- 축 수직 이동: 차단
- 상대 회전: 허용

Prismatic Joint
- 축 방향 이동: 허용
- 축 수직 이동: 차단
- 상대 회전: 차단
```

이번 작업의 목적은 단순 API parity가 아니라, 기존 Contact / Distance / Revolute / Wheel에서 배운 effective mass, Jacobian, accumulated impulse, warm start, bias/relax, unilateral limit, motor clamp, softness를 하나의 Joint에서 다시 연결해 이해하는 것이다.

## 진행 단위

각 단계는 가능하면 작은 PR로 끝낸다. 각 PR은 다음 순서를 지킨다.

1. 수학/행동을 고정하는 테스트를 먼저 추가한다.
2. 실패 원인이 새 기능 부재임을 확인한다.
3. 최소 구현으로 GREEN을 만든다.
4. World lifecycle / island / wake / sleep / stale handle 회귀를 확인한다.
5. 같은 단계의 Sandbox 데모를 추가한다.
6. Debug / Release / Ubuntu / Windows CI를 확인한다.
7. 해당 단계의 원리를 문서/설명으로 정리한 뒤 병합한다.

master에는 solver에 연결되지 않은 반쪽 API나 UI만 먼저 병합하지 않는다.

---

# 1단계 — 기본 Prismatic Joint

## 의미

기본 Prismatic은 두 Body 사이 자유도 3개 중 2개를 제거한다.

```text
2D relative DOF
- 축 방향 translation : 남김
- 축 수직 translation : 제거
- relative angle       : 제거
```

레일 위 서랍으로 생각하면 된다.

```text
                허용
        <---------------->

A  ==============================  localAxisA
                  [ B ]

                  ↑ 수직 이동 금지
                  ↻ 상대 회전 금지
```

## 공개 타입

첫 단계에서는 아직 limit/motor/spring 필드를 노출하지 않는다. 구현되지 않은 옵션이 master API에 존재하지 않게 한다.

```cpp
struct prismaticJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};

    vec2 localAnchorA{};
    vec2 localAnchorB{};
    vec2 localAxisA{ 1.0f, 0.0f };

    float referenceAngle = 0.0f;
    bool collideConnected = false;
};

struct prismaticJointData
{
    bodyId bodyA{};
    bodyId bodyB{};

    vec2 anchorA{};
    vec2 anchorB{};
    vec2 axis{};
    float currentTranslation = 0.0f;
    float lateralError = 0.0f;
    float currentAngle = 0.0f;

    vec2 force{};
    float torque = 0.0f;
    bool collideConnected = false;
};
```

`localAnchorA/B`는 Body origin 기준으로 유지한다. solver prepare에서 COM 기준 lever arm으로 변환한다. `localAxisA`는 A와 함께 회전하는 유한한 단위 벡터이며 Wheel Joint와 같은 validation 규칙을 사용한다.

`referenceAngle`은 Revolute와 같은 의미다.

```text
angularError = (angleB - angleA) - referenceAngle
```

따라서 두 Body가 이미 30도 상대 각도를 가진 상태를 그대로 잠그고 싶다면 그 각도를 referenceAngle로 넣는다. 기본값 0은 두 Body의 방향을 같게 유지한다.

## World 통합

새 파일은 현재 Joint 폴더 책임을 따른다.

```text
src/dynamics/joints/
├─ prismaticJoint2.h
├─ prismaticJointSim2.h
├─ prismaticJointConstraint2.h
└─ prismaticJointConstraint2.cpp
```

`world.h`의 `jointSims_` variant에 `prismaticJointSim2`를 추가한다. 새 별도 ownership 체계는 만들지 않는다. 기존 `joint2` cold slot, generation, free-list, Body intrusive joint edge, collideConnected, island/wake/sleep, Body 삭제 경로를 그대로 공유한다.

World API는 기존 Joint naming 패턴을 따른다.

```cpp
jointId createPrismaticJoint( const prismaticJointDef& definition );
prismaticJointData getPrismaticJointData( jointId id ) const;
```

삭제, validity, joint count는 기존 공용 API를 재사용한다.

생성 precondition은 다음과 같다.

- owning World의 valid한 서로 다른 Body
- 최소 한쪽은 Dynamic
- finite localAnchorA/B
- finite unit localAxisA
- finite referenceAngle
- 기존 Joint와 같은 foreign/stale handle 규칙

## persistent sim 데이터

1단계 `prismaticJointSim2`는 다음 정보를 유지한다.

```text
jointId / bodyIdA / bodyIdB
localAnchorA / localAnchorB
localAxisA
referenceAngle
impulse.x = 축 수직 제약의 누적 impulse
impulse.y = 상대 회전 제약의 누적 impulse
```

핵심은 scalar 두 개를 서로 무관하게 푸는 것이 아니라 **2x2 block constraint**로 묶는 것이다. Box2D도 point-to-line 제약과 angular lock을 block solver로 함께 풀어 더 단단한 레일 제약을 만든다.

## 기본 제약 수학

현재 world axis를 `a`, 그 왼쪽 수직 벡터를 `p`라고 한다.

```text
a = rotate(qA, localAxisA)
p = leftPerp(a)
```

anchor 차이는 다음이다.

```text
d = (centerB - centerA) + rB - rA
```

차단해야 할 위치 오차는 두 개다.

```text
C1 = dot(p, d)                         // 레일 옆으로 벗어난 거리
C2 = (angleB - angleA) - referenceAngle // 상대 회전 오차
```

축 방향 값

```text
translation = dot(a, d)
```

은 **1단계에서는 제약식에 들어가지 않는다.** 이것이 허용된 자유도다.

velocity constraint는 대략 다음 형태다.

```text
Cdot1 = dot(p, vB + wB×rB - vA - wA×rA)
        + 축 자체가 A와 함께 회전하는 효과

Cdot2 = wB - wA
```

Jacobian 두 행을 묶으면 2x2 effective mass가 생긴다.

```text
K = J * invM * J^T

[ k11 k12 ]
[ k12 k22 ]
```

solve에서는

```text
deltaLambda = -K^-1 * ( Cdot + bias )
```

형태로 수직 선형 impulse와 angular impulse를 동시에 구한다.

Zonai에서는 별도 범용 matrix 클래스를 새로 만들지 않는다. 이 Joint 한 곳에서 필요한 작은 2x2 계수와 solve helper를 `prismaticJointConstraint2` 내부에 두고, 다른 실제 소비자가 생길 때 공용화 여부를 다시 판단한다.

## bias / relaxation

기존 Joint 흐름을 그대로 따른다.

```text
prepare
→ warm start
→ solve(useBias=true)
→ position integrate
→ solve(useBias=false)
```

bias pass는 lateralError와 angularError를 줄인다. relaxation pass는 이번 step에서 위치 보정을 위해 넣었던 push velocity를 제거한다.

물리 스프링은 아직 없으므로 `useBias=false`에서는 위치 오차 bias를 넣지 않는다.

## 1단계 테스트

Constraint 단위 테스트:

- 축 방향 속도는 그대로 남는다.
- 축 수직 속도는 제거된다.
- 상대 각속도는 제거된다.
- 축 방향 위치 오차는 보정하지 않는다.
- lateral 위치 오차는 bias pass에서 줄어든다.
- angular 위치 오차는 bias pass에서 줄어든다.
- relaxation은 위치 bias를 추가하지 않는다.
- unequal mass/inertia에서 impulse 부호와 momentum이 일관된다.
- 비중심 anchor에서 선형/각운동 coupling이 발생한다.
- localCenter가 0이 아닌 Body에서도 origin anchor가 올바르게 COM lever arm으로 변환된다.
- static/kinematic endpoint에서도 finite하다.

World 테스트:

- create/query/destroy
- joint slot generation 재사용
- Body 삭제가 Joint도 제거
- collideConnected=false collision 차단 및 삭제 후 재평가
- island 연결 / wake / sleep
- pose/mass 변경 시 cached impulse reset
- step 크기 변화 시 warm start 정책 유지

## 1단계 Sandbox

새 데모: **Prismatic — 레일 위 상자**.

```text
===============================
          [ slider ]
===============================
```

관찰 포인트:

- A/D 또는 버튼으로 축 방향 impulse를 주면 움직임
- 축 수직 impulse를 줘도 레일에서 벗어나지 않음
- angular impulse를 줘도 상대 회전이 유지됨
- Mouse Joint로 비스듬히 잡아당겨도 허용 자유도만 남음
- axis, anchor, lateral error, current translation, relative angle을 debug draw/UI에 표시

이 단계에서 사용자는 **“Prismatic은 축 방향을 푸는 Joint가 아니라, 나머지 두 자유도를 잠그고 축 방향 하나를 남기는 Joint”**라는 개념을 이해하는 것을 목표로 한다.

---

# 2단계 — Translation Limit

## 의미

기본 Prismatic에서 남겨둔 축 방향 자유도에 범위를 건다.

```text
lower                           upper
  |-------------------------------|
                [ B ]
```

공개 타입에 다음을 추가한다.

```cpp
bool enableLimit = false;
float lowerTranslation = 0.0f;
float upperTranslation = 0.0f;
```

World setter:

```cpp
void setPrismaticJointLimit( jointId id, bool enableLimit, float lowerTranslation, float upperTranslation );
```

`lower <= upper`를 보장한다. 같으면 축 방향 특정 위치를 유지하는 양방향 고정처럼 동작한다.

## unilateral constraint

하한과 상한은 방향이 반대인 두 unilateral constraint로 분리한다.

```text
lower: translation - lower >= 0
upper: upper - translation >= 0
```

따라서 persistent impulse도 분리한다.

```text
lowerImpulse >= 0
upperImpulse >= 0
```

최종 축 방향 limit impulse는

```text
axialImpulse = lowerImpulse - upperImpulse
```

가 된다.

이 구조는 Contact normal impulse와 연결해서 설명한다. Contact가 “침투하지 말라”는 한 방향 제약이라면 lower/upper limit도 각각 한 방향으로만 미는 제약이다.

Box2D 의도대로 predictive limit를 사용해 한 step 뒤 경계를 크게 넘을 속도를 미리 제한한다. 이미 경계를 넘은 음의 separation은 bias pass에서 보정하고, 경계 안쪽의 양의 separation은 `separation / h` 항으로 접근 속도만 제한한다.

## 2단계 테스트

- limit disabled이면 1단계와 동일
- lower 안쪽으로 접근은 허용, 넘으려는 속도는 제한
- upper도 반대 부호로 동일
- lower/upper impulse는 음수가 되지 않음
- equal limit은 해당 translation 유지
- cached lower/upper impulse reset 조건
- runtime setter가 non-static component를 깨움

## 2단계 Sandbox

1단계 데모에 초록 lower / 빨강 upper marker와 허용 구간을 그린다.

Preset:

```text
Free
Limited
Locked position
```

상자를 양쪽으로 밀어 경계에서 멈추는지 직접 확인한다.

---

# 3단계 — Linear Motor

## 의미

축 방향 현재 속도를 목표 속도로 맞추는 velocity constraint다.

```text
motorSpeed > 0
-------------------->
       [ slider ]
```

공개 타입 추가:

```cpp
bool enableMotor = false;
float motorSpeed = 0.0f;
float maxMotorForce = 0.0f;
```

World setter:

```cpp
void setPrismaticJointMotor( jointId id, bool enableMotor, float motorSpeed, float maxMotorForce );
```

Data query에는 실제 `motorForce`를 노출한다.

## 수학

축 방향 상대 속도를 `Cdot`이라고 하면

```text
Cdot = currentAxialSpeed - motorSpeed
```

이고 필요한 impulse는

```text
deltaImpulse = -axialMass * Cdot
```

이다.

하지만 모터는 무한한 힘을 내면 안 된다.

```text
maxImpulse = maxMotorForce * h
newImpulse = clamp(oldImpulse + deltaImpulse, -maxImpulse, +maxImpulse)
applyImpulse = newImpulse - oldImpulse
```

여기서 `oldImpulse + delta`를 clamp하고 **차이만 적용**하는 이유를 기존 Contact warm-start/accumulated impulse와 연결해서 설명한다.

`motorSpeed = 0`이면 축 방향 brake처럼 동작한다. `maxMotorForce = 0`이면 모터가 켜져 있어도 힘을 내지 않는다.

Motor와 limit는 함께 켤 수 있다. 모터가 경계 밖으로 밀려고 하면 limit가 막는다.

## 3단계 테스트

- 목표 axial speed로 수렴
- positive / negative speed 부호
- zero speed brake
- maxMotorForce clamp
- accumulated impulse와 delta 적용
- limit와 동시에 켠 상태에서 경계 우선 유지
- setter 변경 시 motor impulse reset

## 3단계 Sandbox

Preset:

```text
Free
Limit
Motor +
Motor -
Brake
Motor + Limit
```

왕복 액추에이터처럼 관찰할 수 있도록 reverse 버튼을 제공한다. UI에는 current speed, target speed, max force, actual motor force를 표시한다.

---

# 4단계 — Spring + Target Translation

## 의미

축 방향 특정 translation을 목표로 하는 물리 스프링을 추가한다.

```text
             targetTranslation
                    |
                    v
===================[ ]================
          [ slider ]
```

공개 타입 추가:

```cpp
bool enableSpring = false;
float hertz = 0.0f;
float dampingRatio = 0.7f;
float targetTranslation = 0.0f;
```

World setter:

```cpp
void setPrismaticJointSpring(
    jointId id,
    bool enableSpring,
    float hertz,
    float dampingRatio,
    float targetTranslation );
```

Box2D 비교 기준도 Prismatic spring에 `targetTranslation`을 사용한다. 단순히 anchor가 처음 겹친 위치만 목표로 하도록 숨기지 않고 공개 의미를 명확히 유지한다.

## softness

오차는

```text
C = currentTranslation - targetTranslation
```

이다.

물리 spring은 rigid position correction과 다르게 relaxation에서도 존재해야 한다. Contact의 임시 penetration push velocity와 달리 실제 스프링은 다음 물리 상태에 남아야 하는 힘이기 때문이다.

기존 `constraintSoftness2`를 재사용해

```text
biasRate
massScale
impulseScale
```

을 만들고 대략

```text
deltaImpulse = -massScale * axialMass * (Cdot + bias)
               - impulseScale * oldSpringImpulse
```

형태로 푼다.

이 단계에서 사용자는 Contact에서 배운 softness가 “충돌 전용 특수 공식”이 아니라 **위치 오차를 안정적인 velocity constraint로 바꾸는 일반 도구**임을 연결해 이해하는 것을 목표로 한다.

Spring / limit / motor는 동시에 사용할 수 있다. solve 순서는 Box2D 의도를 따라 축 방향 옵션을 먼저 처리하고 마지막에 block constraint로 rail/angle을 유지하는 방향을 기본으로 한다.

```text
motor
→ spring
→ lower limit
→ upper limit
→ perpendicular + angular block constraint
```

실제 구현 시 Box2D 기준 커밋의 prepare/solve 부호와 순서를 다시 대조하고 테스트로 고정한다.

## 4단계 테스트

- target translation 양/음 방향 복원
- hertz=0에서 spring force 비활성 의미 확인
- dampingRatio 0 / 1 / >1 finite
- spring impulse warm start
- relaxation에서도 물리 spring 유지
- spring + limit
- spring + motor
- spring + motor + limit
- setter 변경 시 관련 cached impulse reset

## 4단계 Sandbox

최종 Prismatic 데모 preset:

```text
Free
Limit
Motor
Spring
Spring + Limit
Motor + Limit
Combined
```

Debug/UI 표시:

- world axis
- anchor A/B
- current translation
- target translation
- lower / upper limits
- lateral error
- relative angle
- perpendicular force
- axial spring force
- limit force
- motor force
- angular reaction torque

최종 데모는 기능을 예쁘게 보여주는 샘플보다 **solver를 관찰하는 학습 도구**가 우선이다.

---

# Solver / lifecycle 공통 원칙

## 기존 Joint architecture 유지

Prismatic만을 위한 별도 graph, ID, allocator, island path를 만들지 않는다.

현재 구조를 확장한다.

```text
joint2 cold slot
    +
jointSims_ variant<..., prismaticJointSim2>
    +
Body jointEdge intrusive list
    +
island joint IDs
    +
world Step joint prepare/solve dispatch
```

`joint2.h`는 타입 enum을 들고 있지 않고 `jointSims_` variant가 runtime type을 표현하는 현재 설계를 유지한다.

## cached impulse reset

다음 변경에서 해당 persistent impulse를 비운다.

- 관련 option enable 상태 변경
- limit 값 변경
- motor tuning 변경
- spring tuning/target 변경
- Body mass/localCenter 변경
- Body pose를 직접 변경해 geometry 기준이 바뀜
- 기존 Joint warm start 정책상 step/substep 시간이 호환되지 않음

무관한 옵션의 impulse까지 매번 전부 비우지는 않는다. 예를 들어 motor speed 변경 시 기본 rail block impulse를 불필요하게 버리지 않는 방향을 우선한다. 다만 기존 Joint 공통 reset helper가 전체 초기화를 전제로 한다면 일관성을 우선하고 테스트로 동작을 명시한다.

## degenerate effective mass

2x2 block matrix가 singular하거나 축 방향 effective mass가 0이면 NaN/Inf를 만들지 않는다. 해결 가능한 성분만 0으로 안전하게 처리한다. 두 endpoint가 반응할 수 없는 조합은 create precondition에서 막되, solver 내부도 finite 방어를 유지한다.

## CCD

Prismatic 전용 TOI solver는 추가하지 않는다. 기존 Joint들과 동일하게 main step에서 제약을 풀고 CCD가 이후 pose를 자를 수 있는 현재 한계를 공유한다. 고속 회귀에서는 finite state와 다음 step 회복을 확인하되 이번 범위에서 graph-wide TOI joint solve를 새로 설계하지 않는다.

---

# 학습 설명 규칙

각 단계 구현 후 설명은 같은 형식을 사용한다.

## 1. 이번 단계에서 자유도 무엇을 없앴는가

예:

```text
기본 Prismatic
3 DOF → 1 DOF
```

## 2. 수식의 각 항이 실제로 무엇인가

추상 기호만 보여주지 않고

```text
rA = COM에서 anchor까지의 팔 길이
p  = 레일에 수직한 방향
C  = 지금 잘못된 위치량
Cdot = 지금 잘못 움직이는 속도량
K  = 같은 impulse가 속도를 얼마나 바꾸는가
lambda = 그 잘못을 없애기 위해 필요한 impulse
```

처럼 먼저 설명한다.

## 3. 기존 구현과 연결

- basic block constraint ↔ Revolute point/angle 제약
- limit ↔ Contact unilateral impulse
- motor ↔ Revolute/Wheel motor accumulated clamp
- spring ↔ Distance/Wheel softness와 Contact bias 이해

## 4. 코드 변수와 식 대응

실제 Zonai 코드 이름을 식 옆에 둔다.

```text
p                  → perpA
cross(rA + d, p)   → s1
cross(rB, p)       → s2
K^-1               → blockMass
lambda.x           → impulse.x
lambda.y           → impulse.y
```

정확한 이름은 구현 시 기존 naming/style과 Box2D 대조 후 확정한다.

## 5. Sandbox에서 무엇을 보면 되는가

각 단계마다 정상 동작의 시각적 징후와 이상 증상을 함께 설명한다.

예:

```text
정상: 세게 위로 밀어도 slider가 rail에서 거의 벗어나지 않음
이상: anchor가 좌우로 흔들리거나 body가 상대 회전함
```

---

# 테스트와 병합 기준

각 단계는 최소 다음을 통과해야 한다.

- 새 constraint focused test
- 새/확장 World lifecycle test
- Sandbox model test
- 필요한 경우 ImGui UI regression test
- 전체 Debug CTest
- Windows Release lifecycle tests
- Ubuntu CI
- Windows CI

PR diff에서 unrelated formatting/refactor가 없는지 확인한다. 기존 Contact solver 순서, restitution, CCD, BroadPhase 알고리즘은 Prismatic 구현에 필요한 연결 외에는 변경하지 않는다.

단계별 PR이 모두 병합된 뒤 최종 Prismatic 구현을 Box2D 기준 커밋과 다시 대조해 누락된 public behavior가 있는지 한 번만 최종 점검한다.

# 완료 정의

다음이 모두 만족되면 Prismatic Joint를 완성형으로 본다.

1. rail 방향 한 자유도만 남기는 기본 제약이 안정적으로 동작한다.
2. lower/upper translation limit가 작동한다.
3. 선형 motor와 force clamp가 작동한다.
4. target translation spring과 damping이 작동한다.
5. spring / limit / motor를 동시에 사용할 수 있다.
6. World lifecycle, collideConnected, island, wake/sleep, generation 재사용이 기존 Joint와 동일한 의미를 가진다.
7. 각 단계의 Sandbox 데모와 최종 preset이 존재한다.
8. 각 기능의 원리와 코드 대응을 학습용 설명으로 남긴다.
9. 전체 CI가 통과한다.
