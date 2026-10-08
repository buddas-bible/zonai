# Prismatic Joint — 단계별 학습형 완성 설계

기준: master `927127aeed76822c5d22eaf98756dfad6195c2e4`, Box2D 비교 기준 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`.

## 목표

Prismatic Joint를 한 번에 완성품으로 추가하지 않고 **기본 제약 → 이동 제한 → 선형 모터 → 스프링** 순으로 구현한다. 각 단계는 독립적으로 테스트 가능하고 Sandbox에서 눈으로 확인할 수 있어야 하며, 단계가 끝날 때마다 수식과 Zonai 코드의 대응 관계를 설명해 학습에 사용한다.

최종 의미는 다음과 같다.

```text
허용: A의 로컬 축을 따른 B의 병진 운동
차단: 축에 수직한 병진 운동
차단: A와 B의 상대 회전
추가: 축 방향 spring / limit / motor
```

Wheel Joint와의 핵심 차이는 회전 자유도다.

```text
Wheel Joint      축 이동 O / 수직 이동 X / 상대 회전 O
Prismatic Joint  축 이동 O / 수직 이동 X / 상대 회전 X
```

목적은 API parity 자체가 아니라 기존 Contact / Distance / Revolute / Wheel에서 배운 effective mass, Jacobian, accumulated impulse, warm start, bias/relax, unilateral limit, motor clamp, softness를 하나의 Joint에서 다시 연결해 이해하는 것이다.

## 진행 단위

각 단계는 작은 PR로 끝낸다.

1. 행동을 고정하는 테스트를 먼저 추가한다.
2. 새 기능 부재 때문에 RED가 되는지 확인한다.
3. 최소 구현으로 GREEN을 만든다.
4. World lifecycle / island / wake / sleep / stale handle 회귀를 확인한다.
5. 같은 단계의 Sandbox 데모를 추가한다.
6. Debug / Release / Ubuntu / Windows CI를 확인한다.
7. 원리와 코드 대응을 설명한 뒤 병합한다.

master에는 solver에 연결되지 않은 반쪽 API나 UI만 먼저 병합하지 않는다.

---

# 1단계 — 기본 Prismatic Joint

## 자유도

기본 Prismatic은 2D 상대 자유도 3개 중 2개를 제거한다.

```text
축 방향 translation : 허용
축 수직 translation : 차단
relative angle       : 차단
```

레일 위 서랍으로 생각한다.

```text
                허용
        <---------------->
A  ==============================  localAxisA
                  [ B ]
                  ↑ 수직 이동 금지
                  ↻ 상대 회전 금지
```

## 공개 타입

첫 단계에서는 아직 limit/motor/spring 필드를 노출하지 않는다.

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

`localAnchorA/B`는 Body origin 기준이고 prepare에서 COM 기준 lever arm으로 바꾼다. `localAxisA`는 A와 함께 회전하는 유한한 단위 벡터이며 Wheel Joint와 같은 validation 규칙을 사용한다.

`referenceAngle`은 Revolute와 같은 의미다.

```text
currentAngle = (angleB - angleA) - referenceAngle
```

즉 `currentAngle`은 raw 상대각이 아니라 **referenceAngle을 뺀 현재 각도 오차**다. 기본값 0은 두 Body의 방향을 같게 유지한다.

`force`는 B에 작용하는 마지막 substep 반력이다. 1단계에서는 축 수직 성분만 존재하고, 이후 limit/motor/spring이 추가되면 축 방향 성분도 합쳐진다. `torque`는 상대 회전을 막는 angular impulse / substep time이다.

World API:

```cpp
jointId createPrismaticJoint( const prismaticJointDef& definition );
prismaticJointData getPrismaticJointData( jointId id ) const;
```

삭제, validity, count는 기존 공용 Joint API를 재사용한다.

## 구조와 lifecycle

새 파일:

```text
src/dynamics/joints/
├─ prismaticJoint2.h
├─ prismaticJointSim2.h
├─ prismaticJointConstraint2.h
└─ prismaticJointConstraint2.cpp
```

`jointSims_` variant에 `prismaticJointSim2`를 추가한다. 별도 graph/allocator/ID 계층은 만들지 않고 기존 `joint2` cold slot, generation, free-list, Body intrusive joint edge, collideConnected, island, wake/sleep, Body 삭제 경로를 그대로 공유한다.

생성 precondition:

- owning World의 valid한 서로 다른 Body
- 최소 한쪽은 Dynamic
- finite localAnchorA/B
- finite unit localAxisA
- finite referenceAngle
- 기존 Joint와 같은 foreign/stale handle 규칙

1단계 persistent sim 데이터:

```text
jointId / bodyIdA / bodyIdB
localAnchorA / localAnchorB
localAxisA
referenceAngle
impulse.x = 축 수직 제약 누적 impulse
impulse.y = 상대 회전 제약 누적 impulse
```

## 기본 2×2 block constraint

현재 world axis를 `a`, 왼쪽 수직을 `p`라고 한다.

```text
a = rotate(qA, localAxisA)
p = leftPerp(a)
d = (centerB - centerA) + rB - rA
```

차단할 위치 오차:

```text
C1 = dot(p, d)
C2 = (angleB - angleA) - referenceAngle
```

허용할 값:

```text
translation = dot(a, d)
```

이 translation은 1단계 제약식에 넣지 않는다.

Box2D 형태를 따라 축 수직 제약과 angular lock을 하나의 block으로 푼다. `pB - xA = rA + d`이므로

```text
sA = cross(rA + d, p)
sB = cross(rB, p)

J1 = [ -p, -sA,  p,  sB ]
J2 = [  0,  -1,  0,   1  ]
```

velocity error는

```text
Cdot1 = -dot(p, vA) - sA*wA + dot(p, vB) + sB*wB
Cdot2 = wB - wA
```

이고 effective mass matrix는

```text
k11 = invMassA + invMassB + invIA*sA^2 + invIB*sB^2
k12 = invIA*sA + invIB*sB
k22 = invIA + invIB

K = [ k11 k12 ]
    [ k12 k22 ]
```

이다. bias vector를 `b`라 하면

```text
deltaLambda = -K^-1 * ( Cdot + b )
```

로 두 impulse를 동시에 구한다.

```text
P = deltaLambda.x * p

vA -= invMassA * P
wA -= invIA * ( sA*deltaLambda.x + deltaLambda.y )

vB += invMassB * P
wB += invIB * ( sB*deltaLambda.x + deltaLambda.y )
```

이 block solve가 Prismatic의 기본 핵심이다. 수직 이동과 회전은 비중심 anchor에서 서로 결합되므로 두 scalar를 독립적으로 푸는 것보다 Box2D 의도와 강성이 잘 보존된다.

별도 범용 matrix 클래스를 만들지 않는다. 이 Joint에서 필요한 작은 2×2 계수/solve helper를 `prismaticJointConstraint2` 안에 두고 두 번째 실제 소비자가 생길 때 공용화를 검토한다.

## bias / relaxation

기존 Joint 흐름을 그대로 따른다.

```text
prepare
→ warm start
→ solve(useBias=true)
→ position integrate
→ solve(useBias=false)
```

bias pass는 lateralError와 currentAngle을 줄인다. relaxation은 위치 보정을 위해 넣은 push velocity를 제거한다. 물리 spring은 아직 없으므로 `useBias=false`에서는 위치 오차 bias를 넣지 않는다.

## 1단계 테스트

Constraint:

- 축 방향 속도는 남는다.
- 축 수직 속도는 제거된다.
- 상대 각속도는 제거된다.
- 축 방향 위치 오차는 보정하지 않는다.
- lateral/angular 오차는 bias pass에서 줄어든다.
- relaxation은 위치 bias를 추가하지 않는다.
- unequal mass/inertia에서 impulse 부호와 momentum이 일관된다.
- 비중심 anchor에서 선형/회전 coupling이 발생한다.
- non-zero localCenter에서도 COM lever arm이 정확하다.
- static/kinematic endpoint에서도 finite하다.

World:

- create/query/destroy
- generation slot 재사용
- Body 삭제 시 Joint 제거
- collideConnected=false 차단 및 Joint 삭제 후 재평가
- island / wake / sleep
- pose/mass 변경 시 cached impulse reset
- step/substep 변화의 기존 warm-start 정책

## 1단계 Sandbox

새 데모: **Prismatic — 레일 위 상자**.

- 축 방향 impulse: 이동해야 함
- 축 수직 impulse: 레일에서 벗어나지 않아야 함
- angular impulse: 상대 회전하지 않아야 함
- Mouse Joint로 비스듬히 당겨도 한 자유도만 남아야 함
- axis, anchor, lateralError, currentTranslation, currentAngle을 표시

학습 목표는 “Prismatic은 축 방향을 강제로 움직이는 Joint가 아니라, 나머지 두 자유도를 잠가 축 방향 하나를 남기는 Joint”라는 점이다.

---

# 2단계 — Translation Limit

공개 타입 추가:

```cpp
bool enableLimit = false;
float lowerTranslation = 0.0f;
float upperTranslation = 0.0f;
```

World setter:

```cpp
void setPrismaticJointLimit( jointId id, bool enableLimit, float lowerTranslation, float upperTranslation );
```

`lower <= upper`를 보장하며 같으면 해당 translation을 유지한다.

하한과 상한은 서로 반대 방향의 unilateral constraint다.

```text
lower: translation - lower >= 0
upper: upper - translation >= 0

lowerImpulse >= 0
upperImpulse >= 0
axialLimitImpulse = lowerImpulse - upperImpulse
```

Contact normal impulse와 같은 방식으로 한 방향으로만 미는 제약이라는 점을 연결해 설명한다. Box2D 의도대로 predictive limit를 사용해 한 step 뒤 경계를 넘을 속도를 미리 제한하고, 이미 경계를 넘은 오차는 bias pass에서 보정한다.

테스트:

- disabled면 1단계와 동일
- lower/upper 접근 방향 및 부호
- impulse 비음수 clamp
- equal limit
- setter와 cached impulse reset
- limit 변경 시 component wake

Sandbox:

```text
Free
Limited
Locked position
```

lower는 초록, upper는 빨강, 허용 범위는 회색으로 표시한다.

---

# 3단계 — Linear Motor

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

Data에는 `motorForce`를 노출한다.

축 방향 현재 상대 속도를 `axialSpeed`라 하면

```text
Cdot = axialSpeed - motorSpeed
deltaImpulse = -axialMass * Cdot

maxImpulse = maxMotorForce * h
newImpulse = clamp(oldImpulse + deltaImpulse, -maxImpulse, +maxImpulse)
applyImpulse = newImpulse - oldImpulse
```

`motorSpeed = 0`은 brake, `maxMotorForce = 0`은 힘 없음이다. Motor와 limit는 동시에 켤 수 있고 경계에서는 limit가 모터의 이동을 막는다.

이 단계에서는 누적 impulse 전체를 다시 적용하지 않고 **clamp된 새 누적값과 이전 누적값의 차이만 적용하는 이유**를 Contact/Revolute/Wheel과 연결해 설명한다.

테스트:

- positive/negative target speed
- zero-speed brake
- force clamp
- accumulated/delta impulse
- motor + limit
- setter reset/wake

Sandbox preset:

```text
Free
Limit
Motor +
Motor -
Brake
Motor + Limit
```

current speed / target speed / max force / actual motor force를 표시한다.

---

# 4단계 — Spring + Target Translation

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

Box2D 비교 기준의 Prismatic spring도 `targetTranslation`을 사용한다. 목표를 숨은 초기값으로 두지 않고 공개 의미를 유지한다.

오차:

```text
C = currentTranslation - targetTranslation
```

기존 `constraintSoftness2`를 사용해 대략

```text
deltaImpulse = -massScale * axialMass * (Cdot + bias)
               - impulseScale * oldSpringImpulse
```

형태로 푼다.

Contact penetration bias와 달리 **물리 spring은 relaxation에서도 유지**한다. 이것은 임시 위치 보정 속도가 아니라 실제 물리 힘이기 때문이다.

Spring / limit / motor는 동시에 사용할 수 있다. solve 순서는 Box2D 기준을 따라 다음을 기본으로 한다.

```text
motor
→ spring
→ lower limit
→ upper limit
→ perpendicular + angular block constraint
```

구현 시 비교 기준 커밋의 부호/순서를 다시 대조하고 focused test로 고정한다.

테스트:

- positive/negative target 복원
- hertz=0 의미
- damping 0 / 1 / >1 finite
- spring warm start
- relaxation에서도 spring 유지
- spring + limit
- spring + motor
- spring + motor + limit
- setter reset/wake

최종 Sandbox preset:

```text
Free
Limit
Motor
Spring
Spring + Limit
Motor + Limit
Combined
```

표시 정보:

- axis / anchor A/B
- current translation / target translation
- lower / upper limits
- lateral error / current angle
- perpendicular force
- spring force / limit force / motor force
- angular reaction torque

---

# 공통 lifecycle / solver 원칙

Prismatic만을 위한 별도 graph, ID, allocator, island path를 만들지 않는다.

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

`joint2.h`에 새 type enum 계층을 추가하지 않고 현재 variant 기반 runtime type 표현을 유지한다.

cached impulse는 관련 설정/geometry가 바뀔 때 비운다.

- option enable 변경
- limit 값 변경
- motor 설정 변경
- spring tuning/target 변경
- Body mass/localCenter 변경
- Body pose 직접 변경
- 기존 Joint 정책상 호환되지 않는 step/substep 변화

가능하면 변경된 기능의 impulse만 비우되, 기존 공용 reset helper가 전체 초기화를 전제로 하면 일관성을 우선하고 테스트로 명시한다.

2×2 matrix가 singular하거나 axial effective mass가 0이면 NaN/Inf를 만들지 않는다. create 단계에서 반응 불가능한 endpoint 조합을 막고 solver 내부도 finite 방어를 유지한다.

Prismatic 전용 TOI solver는 추가하지 않는다. 기존 Joint와 같은 CCD 한계를 공유하며 고속 회귀에서는 finite state와 다음 step 회복만 확인한다.

# 학습 설명 형식

각 단계가 끝날 때 같은 순서로 설명한다.

1. 이번 단계에서 어떤 자유도를 없애거나 제어했는가
2. 수식의 각 항이 물리적으로 무엇인가
3. 기존 Contact/Revolute/Wheel/Distance와 무엇이 같은가
4. 실제 Zonai 변수와 수식이 어떻게 대응하는가
5. Sandbox에서 무엇을 보면 정상/비정상을 판단할 수 있는가

예를 들어 기본 단계에서는 다음처럼 연결한다.

```text
rA = COM에서 anchor까지의 팔 길이
p  = 레일 수직 방향
C  = 잘못된 위치량
Cdot = 잘못 움직이는 속도량
K  = 같은 impulse가 속도를 얼마나 바꾸는가
lambda = 잘못을 없애는 데 필요한 impulse
```

그리고 실제 변수 대응을 함께 보여준다.

```text
p                  → perpA
cross(rA + d, p)   → sA
cross(rB, p)       → sB
K^-1               → blockMass
lambda.x           → impulse.x
lambda.y           → impulse.y
```

정확한 코드 이름은 구현 시 기존 naming/style과 Box2D 대조 후 확정한다.

# 테스트와 병합 기준

각 단계 최소 기준:

- focused constraint test
- World lifecycle test
- Sandbox model test
- 필요한 경우 ImGui UI regression
- 전체 Debug CTest
- Windows Release lifecycle tests
- Ubuntu CI
- Windows CI

PR diff에서 unrelated formatting/refactor를 허용하지 않는다. Contact solver 순서, restitution, CCD, BroadPhase 알고리즘은 Prismatic 연결에 필요한 범위 밖에서는 변경하지 않는다.

모든 단계가 병합된 뒤 Box2D 비교 기준과 최종 대조해 public behavior 누락을 한 번 점검한다.

# 완료 정의

1. rail 방향 한 자유도만 남기는 기본 제약이 안정적이다.
2. lower/upper translation limit가 작동한다.
3. linear motor와 force clamp가 작동한다.
4. target translation spring과 damping이 작동한다.
5. spring / limit / motor를 동시에 사용할 수 있다.
6. lifecycle, collideConnected, island, wake/sleep, generation 의미가 기존 Joint와 일치한다.
7. 각 단계 Sandbox 데모와 최종 preset이 존재한다.
8. 각 기능의 원리와 코드 대응을 학습용으로 설명한다.
9. 전체 CI가 통과한다.
