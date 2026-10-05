# 기본 Distance Joint — 학습용 첫 구현 설계 제안

상태: 구현 전 검토 제안. 기준 master `8fb0ae5c6fa29fc1115be16f823f1f4a69739b12`, Box2D `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`. 사용자 설계 승인 또는 구현 완료를 뜻하지 않는다.

## 목표와 선택

두 Body의 anchor 사이 길이를 유지하는 양방향 제약을 구현하며 effective mass, COM 기준 lever arm, warm start, accumulated impulse, bias/relax를 학습한다. Contact 없이 연결된 Body도 올바르게 풀리고 함께 깨거나 잠들며, Joint/Body 삭제와 collideConnected의 재평가가 안전해야 한다.

선택지는 ① 기본 rigid distance + 완전한 lifecycle/graph 연결, ② 처음부터 spring/limit/motor 전부, ③ generic Joint hierarchy부터 구축하기다. **①을 제안한다.** 수학과 ownership 연결을 함께 관찰할 수 있고 첫 제약과 무관한 옵션·추상화를 줄인다. Spring/limit/motor는 기본 거리 제약을 이해하고 검증한 뒤 같은 흐름에 추가한다.

## 범위와 공개 입력

`distanceJointDef`는 `bodyIdA`, `bodyIdB`, Body origin 기준 localAnchorA/B, 목표 length(기본 1m), collideConnected(기본 false)를 값으로 가진다. World는 다음 API를 추가한다. 기존 API 명칭은 그대로 유지하고 새 함수는 lowerCamelCase를 사용한다.

```cpp
jointId createDistanceJoint( const distanceJointDef& definition );
void destroyJoint( jointId id );
bool IsValid( jointId id ) const noexcept; // 기존 overload 규약 유지
distanceJointData getDistanceJointData( jointId id ) const;
std::size_t getJointCount() const noexcept;
```

Data query는 body handle, world anchor A/B, 목표 length, 현재 length, collideConnected의 복사 snapshot이다. Solver impulse를 외부에서 직접 수정하는 API, mutable record 참조, spring/motor/limit setter는 추가하지 않는다. 정의 변경은 첫 버전에서 destroy/create로 수행한다.

입력은 owning World의 valid한 서로 다른 Body handle, finite anchor/length여야 하며 length > 0을 precondition으로 둔다. Box2D처럼 실제 목표 length는 기존 `LINEAR_SLOP` 이상으로 clamp한다. Static–Dynamic, Kinematic–Dynamic, Dynamic–Dynamic을 지원하고 최소 한쪽은 Dynamic이어야 한다. Static–Static/Kinematic–Kinematic처럼 반응할 Dynamic이 없는 pair는 첫 버전의 precondition 밖이다. stale/null/foreign ID query/mutation은 기존 World와 같은 precondition이며 `IsValid`는 안전하게 false를 반환한다.

## Identity와 Body 연결

- jointId는 index1, 16-bit generation, 64-bit worldToken 및 value equality를 사용한다. null은 index1=0이다. generation wrap 한계는 기존 Body/Shape와 같다.
- World가 `joint2` cold record, 같은 stable index의 `distanceJointSim2`, free-list와 live count를 소유한다. Cold record는 slot/generation/free-list, 양쪽 `jointEdge2`, collideConnected를 가진다. Sim은 local anchor, 목표 length와 persistent axial impulse를 가진다. 둘을 별도의 dense solver set으로 옮기지 않는다.
- Body에 headJointKey/jointCount를 추가한다. Contact와 같은 `(slot << 1) | edgeIndex` intrusive 양방향 연결을 사용하고 head/middle/tail 삭제, 여러 Joint, vector 증가 시 stale 참조를 검증한다. Key 표현 범위에 맞는 slot 상한을 확인하고 signed shift overflow를 허용하지 않는다.
- 생성 시 연결된 graph를 깨우고 차단 Body pair의 기존 Contact를 제거한다. DestroyBodyByIndex는 Joint → Contact → Shape/proxy 순으로 제거한 뒤 Body slot을 반환한다. Joint 삭제는 양쪽 edge를 먼저 unlink하고 sim을 비운 뒤 free-list에 반환하며 남은 component를 깨운다. 삭제 중 필요한 next key/body index는 mutation 전에 저장한다.

## Collision / Island / wake / sleep

Body pair collision 허용 여부는 작은 Joint list를 순회하며 같은 pair에 collideConnected=false가 하나라도 있으면 false다. Contact 생성과 CCD candidate는 이 정책을 조회한다. 차단된 후보는 pairSet에 등록하지 않는다. 차단 Joint 삭제 후 양쪽 Body의 live shape proxy를 TouchProxy로 표시해 이동 없는 pair도 다음 update에서 다시 평가한다. 다른 차단 Joint가 남으면 계속 차단한다. category/mask/group 정책과 기존 sensor 제외는 유지한다.

Box2D 기준에 맞춰 discrete Sensor overlap은 기존 shape filter를 유지하고 Joint로 차단하지 않는다. Continuous Sensor 후보는 CCD의 Body filter를 따른다. 각 경로의 차이를 test와 주석으로 남긴다. Body 파괴 시에는 남은 proxy를 불필요하게 query하지 않도록 삭제 흐름의 조건을 확인한다.

BuildIslands는 Joint sim span을 추가로 받아 active Joint의 awake non-static endpoint를 union한다. island2에는 jointStart/jointCount, islandGraph2에는 flat jointIds를 추가한다. 같은 Static에 매달린 서로 무관한 Dynamic은 union하지 않는다. Static endpoint Joint는 non-static endpoint의 island에 한 번 포함한다. 양쪽이 잠들면 이번 solver 작업 집합에서 제외한다.

WakeBodyByIndex/SleepBodyByIndex는 Contact + Joint graph를 순회한다. Step 전 awake endpoint에서 sleeping endpoint로 wake를 전파해 active Joint가 경계를 가로지르지 않게 한다. Static pose 변경은 Joint 이웃을 깨우되 Static을 다른 Dynamic으로 이어지는 bridge로 쓰지 않는다. 자동 sleep은 Joint를 포함한 island 단위이며 생성/삭제/Body pose 및 mass 변경은 해당 graph를 깨운다.

## 거리 제약의 수학과 Step 연결

Body origin의 local anchor를 곧바로 회전 lever arm으로 쓰지 않는다. 처음 prepare할 때 `rA = rotationA * (localAnchorA - localCenterA)`로 COM 기준 world lever arm을 만들고 B도 동일하게 처리한다. 아래 식은 양방향 제약의 부호와 비중심 anchor의 회전 효과를 보여준다.

```text
d = (centerB - centerA) + (rB - rA)
u = normalize(d), C = |d| - length
Cdot = dot(u, vB - vA + cross(wB, rB) - cross(wA, rA))
K = invMassA + invMassB + invIA*cross(rA,u)^2 + invIB*cross(rB,u)^2
axialMass = K > 0 ? 1/K : 0
deltaImpulse = -massScale*axialMass*(Cdot + bias) - impulseScale*accumulatedImpulse
P = deltaImpulse*u
vA -= invMassA*P, wA -= invIA*cross(rA,P)
vB += invMassB*P, wB += invIB*cross(rB,P)
```

Contact normal impulse처럼 0 이상으로 clamp하지 않는다. 길이를 줄이는 힘과 늘리는 힘이 모두 필요하기 때문이다. Axis normalize는 길이 0에서 0 vector를 반환하도록 안전하게 처리한다. 겹친 anchor는 finite하지만 방향이 없으면 correction을 만들 수 없다는 한계도 명시하고 NaN 여부를 검사한다.

Box2D의 rigid 경로처럼 prepare 때 초기 axialMass를 계산하고, solve마다 누적 deltaPosition/deltaRotation으로 현재 anchor/separation을 갱신한다. Kinematic의 입력 velocity는 Cdot에 반영하되 invMass/invInertia가 0이므로 impulse로 바꾸지 않는다. Static state는 read-only identity/zero를 사용한다.

수치 안정화 계수는 Box2D 기본 constraintHertz=60Hz, dampingRatio=2를 출발점으로 하며 `min(60Hz, 0.25/subStepTime)`으로 제한한다. 기존 MakeContactSoftness 수학을 공유하되 Contact 전용 명칭 때문에 Joint가 Contact solver에 의존하지 않도록 `constraintSoftness2`/`makeConstraintSoftness`를 작은 공용 dynamics 파일로 옮기고 기존 공개 이름에는 compatibility alias/wrapper를 남긴다. 이 정리는 두 실제 소비자의 동일 계산을 공유하기 위한 것이다. 물리 spring 옵션은 추가하지 않는다.

World Step은 기존 Contact 순서를 보존하면서 island별 Joint prepare, 각 substep의 Joint warm start → bias solve → delta 적분 후 bias 없는 relax, 최종 impulse store를 연결한다. Joint pass는 같은 phase의 Contact pass 앞에서 실행한다. Relax는 bias=0, massScale=1, impulseScale=0으로 correction이 남긴 velocity를 풀며 accumulated impulse를 저장한다. Warm start는 substep마다 현재 axis/anchor로 적용한다. Contact restitution은 Joint에 적용하지 않는다.

Prepare에서 최신 mass/localCenter를 읽는다. Body pose/shape mass 변경 때 관련 Joint impulse를 비워 오래된 geometry/mass의 impulse를 쓰지 않는다. Time step이나 substep 크기가 달라지면 이전 Step의 Joint warm-start impulse도 비운다. 기존 Contact의 warm-start 정책까지 함께 변경하지 않는다. Step(0)은 collision/sensor 갱신만 수행하고 Joint solve나 force 소비를 추가하지 않는다.

CCD가 이후 Body delta를 자르면 이미 푼 Joint의 거리 오차가 다시 생길 수 있다. TOI에서 전체 연결 graph를 재해석하는 새 CCD solver는 추가하지 않는다. 첫 수학 회귀는 CCD를 끈 낮은 속도에서 분리해 확인하고, CCD filter 적용과 유한성·다음 step의 회복을 별도 경계 테스트로 확인한다. 정확한 거리 고정이나 Box2D 전 기능 parity를 약속하지 않는다.

## 검증과 학습 순서

1. **Scalar 제약:** 정지 평형, 중심 anchor의 축 방향 속도 제거, unequal mass의 총 momentum, 비중심 anchor의 angular response, offset localCenter, Kinematic 입력, 0-axis finite, compression/tension 양쪽 부호. Prepare/warm/solve/relax 단계를 주석과 검사에 대응한다.
2. **World 통합:** Contact 없는 Dynamic pair의 같은 island와 wake/sleep, Static bridge 제외, Static pose 변경, create/delete, 여러 Joint의 unlink 순서, Body 삭제, slot 재사용, null/stale/foreign ID와 World 재생성.
3. **Filter:** 기존 Contact 즉시 제거, 양수 groupIndex에서도 Joint 차단, stationary Joint 삭제 후 Contact 재생성, 여러 차단 Joint 중 하나만 삭제, CCD pair 차단, discrete/continuous Sensor 정책 차이.
4. **시간/상태:** substeps 1/4/8, 긴 실행에서 길이 오차와 finite state, force 한 번 소비, mass/pose 변경과 impulse reset, Step(0), step 크기 변화. 오차 기대값은 분석 가능한 경우 직접 식으로 정하고 장기 수렴 tolerance는 실행 근거를 남긴다.
5. **Sandbox:** 기존 한 scene에 anchor와 거리선, Static pendulum 또는 Dynamic pair, 목표/현재 length 표시와 impulse 실험을 작게 추가한다. Scene registry는 만들지 않는다. UI 시각 검증 여부는 자동 테스트와 구분해 기록한다.

새 검사는 NDEBUG와 무관한 runtime check를 사용하고 Windows Release CI에도 등록한다. 기존 38개 Debug 회귀, 기존 Release runtime 8개, Ubuntu library/test 및 Windows Sandbox build를 함께 확인한다. Physics 구현은 수학과 graph/lifecycle 통합을 모두 검증한 뒤 병합한다. 중간에 solver에 연결되지 않은 Joint API를 master에 완료 기능처럼 병합하지 않는다.

새 struct/helper는 lowerCamelCase, 기존 private-member suffix `_`, header와 cpp 순서, 간결한 줄 구성 및 `#pragma region`을 따른다. 주석은 Box2D rationale, 좌표계, impulse 부호와 불변식을 설명하고 이미 보이는 코드의 동작을 반복하지 않는다.
