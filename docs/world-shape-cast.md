# World ShapeCast

`world::castShape` / `world::castShapeClosest`는 기존 ShapeCast(GJK 거리 반복)와
BroadPhase의 세 DynamicTree를 연결하는 World 공간 선형 sweep API다.

```cpp
worldShapeCastInput2 input{};
input.geometry = circle2{ {}, 0.5f };
input.transform.position = { 0.0f, 1.0f };
input.translation = { 10.0f, 0.0f };

const worldShapeCastResult2 hit = simulation.castShapeClosest( input );

simulation.castShape( input, queryFilter{},
    [&]( shapeId id, vec2 point, vec2 normal, float fraction )
    {
        return fraction; // 현재 충돌 지점까지만 이후 후보를 검사
    } );
```

- 입력 Shape는 Circle, Capsule, Convex Polygon, Segment. 도형은 `transform`의 자세를
  유지하며 `translation` 방향으로 움직인다. ChainSegment를 이동 도형으로 넣는 경우는
  현재 지원하지 않는다.
- 고정 타깃은 Circle/Capsule/Polygon/Segment/ChainSegment이며 Sensor도 탐색한다.
  Chain 타깃에는 edge 오른쪽의 앞면 조건을 적용한다. 앞면 판정은 간단한
  normal/초기 중심 기준이며 ghost-vertex 접합점의 상세 ownership 검사는 하지 않는다.
- `fraction`은 전체 translation에 대한 0~1 비율이다. 기존 `ShapeCast`의
  LINEAR_SLOP 기반 접촉 허용차 및 끝 fraction 제외 규칙을 유지한다.
- callback: -1 무시, 0 전체 종료, 반환한 양수 fraction으로 남은 sweep 단축,
  1 전체 길이 유지. 후보 탐색은 거리순이 아니다.
- 처음부터 겹치면 일반 조회에는 fraction=0, zero normal로 보고하며, closest는
  처음 겹친 후보를 건너뛴다. `canEncroach=true`는 기존 narrowphase ShapeCast의
  가까운 초기 상태에서 조금 더 접근할 수 있는 모드를 사용한다.
- QueryFilter의 양방향 category/mask를 사용하며 groupIndex / collisionMatrix는
  공간 조회에 적용하지 않는다. 반환 `shapeId`의 수명은 `IsValid`로 확인한다.
- 순수 읽기 조회이며 callback에서 World/Tree/Shape를 수정하거나 `Step`에 재진입하면 안 된다.
  복수 객체의 다른 프레임 캐싱이나 terrain 연결 지점 처리, 회전 sweep은 이 API 범위가 아니다.

`worldShapeCastTests`: 세 Body Tree, Sensor, 필터, hit 위치·normal·fraction,
초기 겹침·encroachment, callback, world handle 수명, transform 변경/파괴,
Circle/Capsule/Polygon/Chain, 유효하지 않은 입력을 Release에서도 검사한다.


## BoxCast를 사용하는 BroadPhase 후보 탐색

World ShapeCast는 이제 DynamicTree의 `BoxCast`를 사용한다.
시작 도형의 tight AABB 중심을 광선으로 이동시키고, Tree node의 AABB를
시작 도형의 half extent만큼 확장해 swept AABB가 실제 통과하는 공간만 조회한다.
대각선 경로에서는 시작과 끝 AABB의 합집합에 들어오지만 이동 경로에서는
벗어난 노드들을 좁은 단계(GJK) 전에 건너뛸 수 있다.

- Tree 자체는 AABB 후보만 판정한다. 최종 hit은 기존 ShapeCast(GJK)와
  QueryFilter/Chain 앞면 검사로 확정하므로 공개 결과 규약은 동일하다.
- callback이 반환한 현재 fraction을 세 Tree 사이에 공유하여 뒤쪽 후보를 줄인다.
- BoxCast에서도 Tree 중간에 수정하거나 Step을 재진입할 수 없다.
- 정량적인 성능 향상은 장면과 트리 형상에 따라 다르며 벤치마크 결과로 주장하지 않는다.

`dynamicTreeBoxCastTests`는 독립적인 swept-AABB 시간 구간 oracle과
무작위/대각선/정지 축/clip/이동/삭제/Rebuild 케이스를 대조한다.
기존 `worldShapeCastTests` 및 `worldSpatialQueryStressTests`도
동일한 hit 의미를 유지하는지 확인한다.


## Root의 바깥 방향 이동 조기 제외

`DynamicTree::BoxCast`는 비용이 큰 노드별 slab 교차 검사 전에
시작 도형의 AABB가 Tree root 바깥에 있고 **그 축에서 root로부터 멀어지는**
경우에만 즉시 종료한다. 예를 들어 root 위쪽에서 수평으로 이동하거나,
root 오른쪽에서 더 오른쪽으로 이동하는 sweep은 충돌 후보가 될 수 없다.

이 검사는 기존 AABB와 이동 벡터의 **부호만** 사용한다. 이동 끝점을
float로 더하거나 곱해서 경계 부근의 충돌을 잘못 버리는 위험이 없다.
root 쪽으로 이동하지만 clip 분율 안에는 도달하지 못하는 경우는 기존 slab
검사에 맡긴다. 추가 분기 자체의 실행 비용도 실제 벤치마크로 확인한다.

성능 효과는 World ShapeCast 및 Tree 벤치마크로 확인하며, 특히 root
바깥의 횡방향/멀어지는 miss가 직접 대상이다. 이동 중인 도형의 실제
geometry와 GJK/Solver 수학은 변경하지 않는다.


## 회전과 이동을 함께 사용하는 TOI 기반 sweep

기존 `world::castShape`는 도형 자세가 고정된 **선형** 조회로 유지한다.
새 `world::castShapeSweep` / `castShapeSweepClosest`는 상대 회전이 필요한
경우에만 별도로 사용한다.

```cpp
worldShapeSweepInput2 input{};
input.geometry = circle2{ { 2.0f, 0.0f }, 0.3f };
input.transform = {};
input.translation = {};
input.rotationRadians = 1.0f; // 로컬 원점 기준 양의 라디안 회전

const worldShapeCastResult2 hit = simulation.castShapeSweepClosest( input );
```

- 시작 자세는 `transform`, 마지막 자세의 위치는 `transform.position + translation`,
  회전은 `transform.rotation * rot2::FromRadians(rotationRadians)`로 정한다.
  기존 `GetSweepTransform`처럼 sin/cos 보간 뒤 정규화하므로 일반적인
  고정 각속도 보간과 정확히 같지는 않다. fraction은 이 보간 구간의 비율이다.
- 지원 범위는 **회전량 -π/2 ~ +π/2**이며 0 이동/0이 아닌 회전도 허용한다.
  더 큰 각도, 비정상적인 회전값, 음수 radius 등 잘못된 입력은 조회하지 않는다.
  회전량이 정확히 0이면 기존 선형 ShapeCast로 위임한다.
- 회전 도중 AABB는 시작/종료 AABB의 합집합만으로는 충분하지 않다.
  로컬 점과 radius의 최대 원점 거리를 계산하고 시작·끝 위치를 포함하는
  반경 확장 AABB로 세 유형의 Tree에 **Query**한다. 이 범위는 보수적인
  후보 집합일 뿐 실제 충돌 여부를 뜻하지 않는다.
- 좁은 단계는 기존 `TimeOfImpact`의 static A와 이동/회전하는 B sweep을
  사용한다. Sensor, QueryFilter, Chain 타깃의 앞면 정책은 유지하고,
  callback은 -1 무시 / 0 종료 / 양수 fraction clip을 지원한다.
  Chain의 시작점 앞면 판정과 hit normal을 직접 확인하며, CCD 전용
  최소 이동량 heuristic은 World 조회에서 사용하지 않는다.
  따라서 짧은 이동으로 처음 접촉하는 경우도 조회 대상으로 남긴다.
  순서 보장은 없으며 최초 fraction 0 hit은 일반 조회에만 보고하고
  closest에서는 건너뛴다.
- `TimeOfImpact`는 swept separating-axis 기반 **보수적 진행 추정**이다.
  회전 중 발생하는 일부 glancing contact를 놓칠 수 있으며
  `Failed` 상태는 hit으로 보고하지 않는다. 이 API는 모든 회전 충돌을
  빠짐없이 열거하는 완전한 sweep oracle이 아니다. Chain 고스트 정점
  ownership의 완전한 처리도 범위 밖이다.
- `worldShapeCastTests`에는 중간 회전 충돌, 음의 회전, 회전·이동,
  0 회전 위임, 시작 겹침, 필터/센서, handle 수명, 입력 검증이 포함된다.

공식 Box2D에서 TOI와 선형 ShapeCast를 별도 알고리즘으로 구분하는 것과
같이 구현하며, 기존 GJK 선형 ShapeCast의 공개 의미를 변경하지 않는다.
참고: https://box2d.org/documentation/group__distance.html


## Sandbox 회전 조회 시각화

공간 조회 > ShapeCast Sandbox에서 기존 선형 조회를 유지하면서
상대 회전 각도(-90~90도)를 조절할 수 있다.

- 각도 0도: 기존 `world::castShapeClosest` 및 `canEncroach` 제어
- 각도 0도가 아님: `world::castShapeSweepClosest` (TOI). `canEncroach`는 회전 모드에서 비활성화
- 시작점과 도착점을 같게 설정하면 순수 회전 조회를 시험할 수 있다.
  원은 회전해도 외형이 같으므로 노란 방향선으로 미리보기 자세를 표시한다.
- 흰색은 시작, 회색은 종료, 노란색은 현재, 파란색은 첫 충돌 자세이며
  충돌 지점과 법선은 주황색으로 표시한다. 자세는 엔진의
  `GetSweepTransform`과 동일한 보간을 사용한다. 회전 중 로컬 support 점들의
  곡선 궤적은 연파랑 선으로 16구간 샘플링해 표시한다(충돌 계산용 경로가 아닌
  순수 디버그 시각화).
- 청록색 AABB는 회전 도중 모든 자세를 포함하도록 로컬 원점 반경으로
  확장한 보수적 후보 범위이다. 회전이 없으면 기존 선형 swept AABB를 표시한다.
- 분홍색 중심 RayCast는 **회전을 반영하지 않는 비교용 조회**이며
  회전 ShapeCast 결과로 오인하면 안 된다.
- 미리보기는 실제 Body 물리를 변경하지 않으며 기존 demoSession 재생/일시정지/
  한 프레임 실행, Canvas 클릭과 Shift+클릭 입력을 재사용한다.

`sandboxShapeCastDemoTests`는 회전 World TOI 결과, 중간 자세와 보수적 후보 범위를
확인하고 `sandboxShapeCastViewTests`는 Headless ImGui 회전 UI/렌더링을 검사한다.
실제 Win32/DX11 화면의 DPI·입력 조작은 별도 수동 확인 대상이다.


## 회전 ShapeCast 충돌 누락 진단 (분석용 회귀 테스트)

`worldRotatingShapeSweepOracleTests`는 World에서 static Circle 타깃을 이동하며
600개 고정 seed(0xA91F35D7)의 회전 Circle query를 실행한다.
회전만 하는 도형, 회전·병진을 결합한 도형, 충분히 멀리 떨어진 miss를 섞는다.

- 테스트의 별도 oracle은 회전 시작/끝 `sin/cos`의 **정규화 선형 보간**
  (`GetSweepTransform`과 같은 시간 정의)으로 moving Circle의 중심을 독립적으로
  계산한다. 두 Circle 중심 사이 거리와 radius 합을 직접 비교하며
  `ShapeDistance` / `TimeOfImpact` / `ShapeCast`는 oracle에서 사용하지 않는다.
- `t = 0..1`을 512구간으로 균등 샘플링한다. 시작/끝 표면 간격이
  0.04m보다 크고, 중간 최소 간격이 -0.04m보다 작은 충돌은
  **robust middle hit**으로 분류한다. 샘플링 사이 아주 짧게 생겼다 사라지는
  접촉은 발견하지 못할 수 있으므로, 이 oracle도 완전한 연속 충돌 증명은 아니다.
- TOI가 보고한 hit의 handle, fraction, finite hit 데이터와 수치적으로
  불가능한 양성 보고(보고 시점의 간격이 0.06m 이상)를 **CI에서 실패**로 검사한다.
  모든 시점의 간격이 0.08m보다 큰 확실한 miss도 반드시 miss로 보고해야 한다.
- **robust middle hit 누락은 CI에서 실패**로 검사한다.
  문제 #221을 수정한 뒤 두 독립 oracle에서 검증된 접촉을 필수 회귀 조건으로 유지한다.
  누락 개수와 재현 가능한 첫 사례(시작 좌표/회전/이동/타깃)를 로그로 남긴다.
  CI에서는 Ubuntu/Windows Debug 테스트 완료 후 진단 실행 파일을 직접 호출하여
  `cases/robust_middle/toi_missed` 통계를 성공 로그에도 노출한다.
  Windows Release 대상에도 동일 테스트를 포함한다.
  예상 값이나 개선 비율을 CI 환경·테스트 수정 전후 결과 없이 단정하지 않는다.
- `--csv`는 case별 간격과 TOI 결과를 출력하며, `--strict`는
  robust middle hit 누락이 하나라도 있으면 실패한다.
  현재 CI의 두 oracle CTest는 모두 `--strict`로 실행한다.

추가로 `worldRotatingBoxSweepOracleTests`는 1,200개 고정 seed
(0x57B0A923) 사례에서 오프셋된 상자가 회전하는 경로와 정적 원의
충돌을 비교한다. 앞의 600개는 상자(1.04×0.24m)와 원(반지름 0.33m),
나머지 600개는 **더 얇은 상자(0.9×0.03m)와 작은 원(반지름 0.04m)**으로
아주 짧은 접촉 구간을 시험한다. 얇은 경우의 robust 최소 침투 기준은
-0.01m이며 시작/종료 분리 기준은 동일한 +0.04m이다.
각 시점의 타깃 원 중심을 상자 로컬 좌표로 역회전하여
**point-to-oriented-box 거리**와 원 반지름 차이를 계산하는 독립 기하
oracle을 사용한다. 결과는 broad와 thin의 누락 건수로 구분해 기록한다.

두 테스트 모두 512구간 샘플과 `--csv` 및 `--strict` 옵션을 사용한다.
시작/종료 분리 기준은 +0.04m이고 중간 침투 기준은 원-원 및
일반 상자에서 -0.04m, 얇은 상자에서 -0.01m이다.
CI 성공 로그에서 원-원과 상자-원 누락 건수를 각각 출력하며,
Windows Release에도 두 테스트를 포함한다.

이 테스트는 **회전 TOI 정확도 진단**에 국한한다. Polygon/Chain 타깃의
ghost vertex ownership이나 높은 각속도, 변화하는 양쪽 Body의 전역 연속
충돌 판정까지 검증하는 것은 아니다. 향후 개선은 같은 입력·seed의
기준 로그 및 성능을 비교한 뒤 별도 PR로 도입한다.


## 회전 중간 접촉 누락 #221 수정

기존 TOI는 분리축이 **끝 시점**에서도 충분히 떨어져 있으면 `Separated`로
종료할 수 있다. 하지만 회전하는 물체에서는 처음/끝이 분리돼도 중간
접촉이 가능하다. 진단 PR #220의 얇은 상자/작은 원 index 664에서
이 문제가 재현되었다.

World의 회전 `castShapeSweep`는 이제 `toiInput2::detectTransientContacts`를
켜고, 기존 TOI가 `Separated` 또는 `Failed`일 때만
**거리 변화 상한 기반의 보수적 전진**으로 재확인한다.

- 평행이동의 최대 속도는 `|c2-c1|`이다. nlerp의 각속도
  `|cross(q1,q2)| / |(1-t)q1+t*q2|²`는 분모의 구간 최솟값으로
  상한을 계산한다. (정확히 단위 길이인 회전끼리는 `2*tan(|Δθ|/2)`와 같다.)
  작은 회전 정규화 오차도 고려하며, 각 support 점에서 localCenter까지의
  최대 거리를 곱해 이동 가능한 core 거리의 상한을 만든다.
- 현재 GJK core 분리 거리 `d`와 접촉 목표 `target + tolerance`에 대해
  `(d-target-tolerance)/maxRelativeSpeed`보다 더 멀리 진행하지 않는다.
  이는 **고정 간격 시간 샘플링이 아니라**, 어떤 점도 건너뛰지 않도록
  이동 거리의 상한으로 전진폭을 정하는 방법이다.
- 충돌을 찾으면 그 시점의 witness points/normal로 World hit을 보고한다.
  256회 내에 float 분율 진전이 불가능하거나 판정이 끝나지 않으면
  기존 TOI 결과를 유지하며, 모든 물리적 sweep의 완전한 검출을 약속하지 않는다.
- 기존 CCD의 `TimeOfImpact` 호출은 이 기능을 켜지 않아 기본 알고리즘,
  반복 한도, Solver 경로 및 성능 의미를 바꾸지 않는다.
  World에서 회전량이 0인 기존 GJK 선형 ShapeCast에도 영향이 없다.

`worldRotatingBoxSweepOracleTests`는 고정 seed 0x57B0A923의 index 664를
**필수 충돌 회귀 테스트**로 확인하며, 전체 1,800개 샘플의 diagnostic
미검출 건수도 기존과 같은 CI 출력에서 확인한다.


## 보수적 재검사 프로파일 (Release)

`worldRotatingToiBenchmark`는 회전 상자 1,200개 고정 seed 입력
(원본 `worldRotatingBoxSweepOracleTests`와 동일한 입력·생성 순서)을 사용한다.
반드시 똑같은 TOI에 대해 `detectTransientContacts=false`와 `true`를 비교한다.

- `toiDiagnostics2`는 선택적 출력 포인터이며 일반 `TimeOfImpact(input)`/CCD 경로는
  null을 사용한다. 기본 분리 거리 GJK 호출 수, 보수적 재검사 시도 수,
  재검사 GJK 호출 수, 복구된 hit 수, 최대 반복 도달 수를 각각 기록한다.
  스레드 전역 카운터나 모든 프레임 누적 상태를 추가하지 않는다.
- `--verify`는 프로파일 on/off가 같은 결과를 내는지, 기존 TOI가 찾은 hit이
  보존되는지, 664번 반례를 복구하는지, World API 결과와 직접 TOI 결과가
  일치하는지를 검증한다. 일반 CTest와 Windows Release 검증 대상이다.
- `--csv`는 총 호출 수·보수적 재검사 빈도·추가 GJK 횟수,
  7회 측정 중 중앙값 `ns/call` 및 guarded/legacy 비용 비율을 출력한다.
  매 측정 전 3번의 반복 실행을 한 묶음으로 사용하고 warm-up을 수행한다.
  **벤치마크 시간이나 속도 비율로 CI 통과 여부를 결정하지 않는다.**
- 출력은 동일한 입력을 `all_direct_candidates` 1,200개,
  `nearby_inputs` 800개, `far_inputs` 400개로 나눈다. 먼 입력은
  실제 World의 BroadPhase가 대부분 거르는 경로이므로 직접 TOI 호출의
  전체 비용을 실제 프레임 저하로 해석하면 안 된다. 가까운 입력도 BroadPhase
  통과를 시뮬레이션한 결과가 아니라 생성 단계에서 지정한 그룹이다.
- 수치는 **개별 TOI narrowphase 호출당 비용**이지 전체 World broadphase의
  프레임 시간이나 실게임 성능을 뜻하지 않는다. CPU·Windows runner·scene 구성이
  달라지면 재측정해야 한다. 거리 계산 수와 실제 Wall-clock 비용도 구분한다.

기존 GJK 선형 ShapeCast와 엔진의 CCD·Solver 동작은 이 계측 작업 범위 밖이다.
