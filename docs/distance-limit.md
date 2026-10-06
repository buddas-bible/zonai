# Distance Joint limit

2026-10-06, master `83afedfb1b0af84565a6a1f8faf33e1e223ecd95` 이후 기존 Distance spring에 최소·최대 거리 제한을 추가했다. Box2D를 다시 fetch한 비교 기준은 `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`의 `src/distance_joint.c` prepare/warm/solve와 `include/box2d/types.h` mode 계약이다. [Spring 수학](distance-spring.md)과 기존 Joint lifecycle/COM/filter/Island 연결을 유지한다.

## Mode와 API

| 설정 | 실제 제약 |
| --- | --- |
| Spring off | Rigid `length`. Limit 설정을 보관하지만 적용하지 않는다. |
| Spring on, limit off | 기존 spring. Hertz 0이면 자유 거리 축이다. |
| Spring on, limit on, min < max | Spring과 두 unilateral limit. Hertz 0에서도 limit은 유지된다. |
| Spring on, limit on, min = max | Box2D처럼 **`length`의 rigid 제약**으로 돌아간다. 같은 min/max 값으로 고정하는 동작이 아니다. |

```cpp
distanceJointDef definition{};
definition.bodyA = anchor; definition.bodyB = bob; definition.length = 2.0f;
definition.enableSpring = true; definition.hertz = 0.0f; // limit만 관찰
definition.enableLimit = true; definition.minLength = 1.5f; definition.maxLength = 2.5f;
const auto joint = simulation.createDistanceJoint( definition );
simulation.setDistanceJointLimit( joint, true, 1.0f, 3.0f );
const auto data = simulation.getDistanceJointData( joint );
```

Def의 limit 기본값은 off, min 0, max 100000 m다. Create/setter는 finite이며 0 <= min <= max인 입력을 받으며 양쪽 값을 LINEAR_SLOP 이상으로 제한한다. Box2D의 range setter는 뒤집힌 값을 정렬하지만 zonai는 기존 API 방식대로 순서를 assert precondition으로 요구한다. 유효한 Distance Joint ID가 필요하다. Release에서 잘못된 입력을 복구하는 API는 아니다.

정규화된 설정이 같으면 sleep/cache를 유지한다. Range/enable 또는 spring tuning 변경은 scalar spring/rigid impulse와 lower/upper impulse를 모두 지우고 두 non-static endpoint의 component를 깨운다. Static anchor의 다른 component는 깨우지 않는다. Mass/pose/h 변경도 세 cache 모두 무효화한다. Body 삭제·Joint slot reuse·collision filter는 기존 경로를 공유한다.

## 한쪽 방향 제약의 수학

Lower separation은 `lengthNow - minLength`, 축속도는 v다. Upper separation은 `maxLength - lengthNow`, 축속도는 -v다. 각 누적 impulse는 **0 이상**으로 clamp한다. Lower는 축 방향으로 밀어 압축을 막고 upper는 반대 방향으로 당겨 늘어남을 막는다. Spring의 signed impulse와 합친 warm impulse는 `springImpulse + lowerImpulse - upperImpulse`다. Spring → lower → upper 순서로 풀며 각 제약은 직전 제약이 바꾼 속도를 사용한다.

범위 안의 separation C > 0에는 `bias = C/h`를 적용한다. 한 step에 남은 간격을 넘는 축속도만 막는 speculative 제약으로, 내부의 느린 자유 이동을 고정하지 않는다. 이미 범위를 넘은 C <= 0에서는 bias pass에 기존 rigid 안정화 softness를 사용하고, relax pass는 bias 없이 속도만 제한한다. 공통 axial mass와 COM lever arm 때문에 두 Body의 운동량과 회전도 같은 Jacobian으로 연결된다.

Limit은 위치를 강제로 범위 안에 덮어쓰지 않으며 **완전히 단단한 경계는 아니다**. 안정화 Hertz `fLimit = min(60, 0.25/h)` 때문에 하중 아래 작은 오차가 남는다. 중심 anchor의 정적인 축 모델에서 spring 자연 길이 L0가 경계 Lb 밖에 있고 중력이 없으면 평형은 `L = (fSpring²*L0 + fLimit²*Lb)/(fSpring² + fLimit²)`다. 두 강성 `k = mEff*(2πf)²`의 힘이 상쇄되는 위치다. 예를 들어 2 Hz spring, max 2 m, 자연 길이 3 m이면 1/60초 한 substep에서 약 2.01747 m, 네 substep에서 약 2.00111 m다. 이 수치 관계를 회귀 검사한다.

`axialForce`는 세 impulse의 signed 합/h로 **마지막 solved substep의 합력**을 나타낸다. Negative는 tension, positive는 compression이며 개별 spring/limit 힘을 뜻하지 않는다. 두 힘이 상쇄되면 합력 0도 가능하다. Sleep/paused 상태는 마지막 결과를 유지한다.

## 진자 실험과 검증

Distance Pendulum은 rigid로 시작한다. Spring과 Limit을 켜고 Hertz를 0으로 바꾸면 limit만 관찰할 수 있다. `Radial kick`, Mouse drag, Inspector pose로 안쪽/바깥쪽 이동을 비교한다. Min/Max text 입력도 서로의 경계를 넘지 않도록 제한한다. Green 점은 min, red 점은 max이며 회색 선은 현재 축의 허용 구간이다. 같은 min/max는 Target 길이의 rigid 모드로 표시한다. Reset은 limit off, min 1.5 m/max 2.5 m, rigid/2 Hz/감쇠 0.7로 돌아간다.

기존 runtime 두 Distance target과 Sandbox target에 lower/upper 부호, 운동량, speculative 이동, interior 자유 이동, boundary에서 안쪽으로 복귀, bias/relax, warm sum/h 무효화, mode 우선순위, 중력 평형 힘, spring과 limit의 강성 평형, cache/wake/no-op와 Reset 회귀를 추가했다. World 응답은 substep 1/4를 비교한다. 기존 spring의 중력 기대값도 실제 World gravity로 바꿨다. ImGui headless frame은 네 Distance slider와 세 Mouse slider의 text 입력, checkbox/radial actions를 실행한다. 실제 native 창/GPU/OS 입력·시각 배치 검증은 아니다.

전체 CTest 43개와 Windows Release runtime CI 대상 13개는 유지한다. 기존 29개 assert target의 Release 검증 공백도 유지한다. 최종 build/CTest·mutation·독립 리뷰·원격 CI 결과는 작업 PR에 기록한다. Coincident anchor의 방향 부재, CCD clipping, 극단적인 finite 계수의 overflow 등 기존 제한을 확대한 구현은 아니다. 다음은 **Distance motor**의 목표 축속도와 최대 힘을 학습하고 spring/limit과 함께 실험하는 단계다.
