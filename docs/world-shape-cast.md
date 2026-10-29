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
  Chain 타깃에는 edge 오른쪽의 앞면 조건을 적용한다. Circle 조회에서는
  hit 시점의 실제 원 중심으로 `CollideChainSegmentCircle`의 ghost-vertex
  Voronoi ownership도 확인하여 접합부의 이웃 edge 중복 hit을 억제한다.
  실제 parent Chain의 이전 edge가 같은 endpoint를 담당하는 경우에는
  이전 edge를 우선하여 중복 판정을 없앤다. 독립 ChainSegment의 ghost만으로
  이전 edge의 존재를 가정하지 않는다. Capsule·Polygon 조회는 hit 자세에서
  `CollideChainSegmentCapsule` / `CollideChainSegmentPolygon`의 인접 edge 법선
  분류와 clipping을 확인한다. 일반 Segment의 앞면 검사는 기존 의미를 유지한다.
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

`worldChainShapeCastOwnershipTests`: 두 이웃 ChainSegment가 만나는 직선/굽은
접합부(정방향/역방향 굴곡)와 회전한 정적 Body에서 원의 선형·회전 sweep
결과를 비교한다. 직선 seam과 접점 양쪽 ±0.12m에서는 단일 소유 edge를
확인한다. 꺾인 코너의 공유 꼭짓점에서는 서로 다른 실제 접촉 법선이라면
인접 edge 둘 다 보고할 수 있으므로 이를 단순 중복 hit으로 간주하지 않는다.
local offset Circle의 회전 및 뒷면 접근 무시도 Ubuntu·Windows Debug/
Windows Release에서 확인한다.
실제 body solver의 contact manifold 동작 자체는 변경하지 않는다.

`worldChainConvexCastOwnershipTests`: Capsule/Box/Triangle을 이동하는 도형으로
사용해 평탄·오목·볼록 Chain seam, 회전된 정적 지형, 선형/회전 조회,
로컬 오프셋, 뒷면 접근 및 퇴화 Capsule을 검증한다. 보고된 각 hit fraction에서
해당 Chain edge의 narrowphase manifold가 실제 존재하는지 독립적으로
재확인한다. Capsule/Polygon은 하나의 물체가 두 인접 edge에 실제로 동시에
접촉할 수 있으므로 무조건 callback을 하나로 제한하지 않는다.

`worldChainRotatingConvexOracleTests`는 연속된 평탄 Chain 두 edge를
회전한 Static Body에 배치하고, 회전축에서 떨어진 Capsule/Box/Triangle
1080개 입력을 독립적인 앞면 평면 접촉 oracle과 비교한다. 도형 core의
모든 vertex(캡슐은 양 끝점)를 시간에 따라 double 기반 nlerp로 변환하고
가장 높은 y 좌표와 geometry 반경으로 지형 앞면까지의 signed gap을
직접 계산한다. GJK/ShapeDistance/TOI/Collider/World sweep transform은
oracle의 충돌 정답 계산에 사용하지 않는다.

- 512개 시간 구간에서 시작·종료가 각각 3.5cm 이상 분리되고, 중간에는
  4cm 넘게 통과하며, 중심이 Chain 앞면을 유지하는 경우만 robust
  transient contact로 인정한다. 모든 정점이 chain 전체 span 안쪽에
  있는 경우만 검사하므로 무한 평면의 접촉과 실제 연속 edge의 접촉이 같다.
- 공유 vertex를 가로지르는 경로와 좌·우 edge 내부, 회전 방향 ±, 소량 병진,
  멀리 떨어진 앞면/뒷면 경로를 함께 검증한다.
- `--strict`는 이 강한 양성 사례의 World hit 누락을 실패로 처리한다.
  `--csv`는 모든 입력에 대한 판정·fraction을 출력한다.
  일반 곡면/오목·볼록 seam 소유권이나 모든 glancing contact의 완전성을
  증명하지는 않으며, 해당 경우는 별도 oracle이 필요하다.
- 이 oracle에서 기존 회전 TOI의 transient 보수적 진행이 접촉 직전에
  float fraction의 1 ULP 이하로 줄어 **명백한 Box/Triangle 중간 충돌을
  누락하는 문제**가 발견됐다. 진행폭이 float 시간으로 더 이상 표현되지
  않을 때 다음 표현 가능한 fraction에서 실제 GJK 거리를 재검사한다.
  판정 거리·`LINEAR_SLOP`·접촉 임계값은 변경하지 않으며,
  기존 CCD 기본 TOI는 transient 옵션이 꺼져 있어 변경되지 않는다.


## 굽은 Chain 접합부 독립 oracle

`worldChainBentRotatingOracleTests`는 평탄 접합부에 한정된 이전 oracle을
오목·볼록 Chain으로 확장한다. Static Chain은 `(-2,0)→(0,0)→(2,±0.6)`이며
Body 자체도 회전한다. Capsule·Box·Triangle은 두 edge 내부와 공유 꼭짓점
인접 구간을 통과하도록 배치한다.

- 독립 double nlerp로 도형의 core vertex와 centroid를 계산한다.
  움직이는 core와 **유한한** 두 segment 사이의 거리는 선분 교차와
  선분-선분 거리로 직접 구한다. GJK/ShapeDistance/TOI/Manifold 및
  엔진 sweep 변환은 oracle의 정답 계산에 사용하지 않는다.
- 512개 시간 표본에서 실제 edge **내부**의 앞면을 깊게 통과하고,
  centroid가 그 edge 앞면에 있으며, 시작과 끝에는 두 edge 모두에서
  떨어져 있는 경우만 확실한 양성 접촉으로 분류한다.
  무한 연장선이나 ghost vertex만 닿는 사례는 양성에서 제외한다.
- 두 굴곡 방향과 각 edge, ±1.48rad 회전, 로컬 오프셋, 명백한
  앞·뒷면 분리 경로를 포함한 **2,880개 입력**을 분류한다.
  `--csv`는 사례별 결과를 출력한다.
- 첫 검증에서 robust 접촉 **916건** 중 공유 seam 인접 **180건은 누락 0건**,
  명백한 비충돌 **1,920건은 오검출 0건**이었다.
  다만 열린 Chain의 첫 edge 바깥쪽에서 출발하는 Box **4건**은 누락됐다.
  첫 TOI는 약 `fraction=0.2046`에서 Hit을 반환하지만 그 자세에서는
  ghost 영역의 manifold가 0점이라 World가 해당 후보를 버린다.
  약 `fraction=0.3066`에서는 앞면 manifold가 2점 존재한다.
  즉 **소유권 필터 이후 TOI 재탐색**이 필요한 별개 문제다.
- 선행 TOI의 비소유 ghost hit을 World가 거절한 뒤에는 이동속도
  상한과 실제 유한 선분-도형 GJK 거리로 접촉 불가능한 시간 구간을
  제거한다. 남은 구간은 **시간상 이른 왼쪽 구간부터** 깊이 우선 탐색한다.
  먼저 발견한 샘플을 바로 확정하는 넓이 우선 방식과, 전체 시간 범위에
  접촉 여부의 단조성을 가정하는 이분 탐색을 사용하지 않는다.
  반환 hit의 법선·점은 실제 `CollideChainSegmentCapsule/Polygon`
  manifold에서만 계산한다.
- 재탐색은 최대 깊이 9, 최대 방문 구간 1024개이며, 마지막에는 선택한
  유효 샘플 근처만 최대 24회 추가 이분 탐색한다. 한도 소진 여부와
  방문 구간/GJK/Manifold 호출 수는 `worldShapeSweepStats2`의 선택적
  `chainRetry*` 필드로 기록한다. 모든 임의의 짧은 접촉을 증명하는
  무한 정밀도 탐색은 아니며 CCD 및 Solver의 TOI 경로는 변경하지 않는다.
- `--strict`는 열린 끝점에서 재현됐던 4건을 포함하여 robust 접촉
  **916건 모두 누락 0**과 확실한 비충돌의 오검출 0을 요구한다.
  기존 4건은 8192분할 narrowphase 기준과 반환 fraction의 차이가
  0.003 이하인지 비교하고, 방문 구간과 manifold 호출의 비용 상한을
  별도로 단언한다. 재현 입력은 제거하지 않았다.
- 이 oracle은 명확한 유한 선분 접촉의 **존재**를 검증한다.
  speculative contact, 접합점의 정확한 법선, 모든 grazing contact와
  여러 hit의 순서까지 증명하지는 않는다.

`worldChainRetryTemporalTests`는 긴 로컬 오프셋 Box가 회전하면서
동일한 Chain 앞면에 **두 번 접촉하고 중간에는 떨어지는** 경로를
평탄/회전된 정적 Body와 짧은 접촉 구간까지 검증한다.
16,384분할의 독립 narrowphase 시간순 참조로 실제 접촉 창 두 개를 확인하고
재탐색 반환 fraction이 **첫 창의 시작과 0.006 이내**인지 비교한다.
탐색 구간 최대 1024개, manifold 호출 최대 방문 구간+24회, 한도 소진 0도 검사한다.

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
  Chain의 시작점 앞면 판정과 hit normal을 직접 확인한다. 이동 원이 Chain
  타깃인 경우에는 해당 TOI 자세를 narrowphase의 ghost-vertex Voronoi
  판정으로 다시 확인한다. CCD 전용
  최소 이동량 heuristic은 World 조회에서 사용하지 않는다.
  따라서 짧은 이동으로 처음 접촉하는 경우도 조회 대상으로 남긴다.
  순서 보장은 없으며 최초 fraction 0 hit은 일반 조회에만 보고하고
  closest에서는 건너뛴다.
- `TimeOfImpact`는 swept separating-axis 기반 **보수적 진행 추정**이다.
  회전 중 발생하는 일부 glancing contact를 놓칠 수 있으며
  `Failed` 상태는 hit으로 보고하지 않는다. 이 API는 모든 회전 충돌을
  빠짐없이 열거하는 완전한 sweep oracle이 아니다. Chain 고스트 정점
  ownership은 Circle 및 Capsule/Polygon의 narrowphase 유효 접촉으로
  제한한다. TOI가 놓치는 중간 접촉까지 보장하지 않는다.
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

추가로 `worldRotatingCapsulePolygonOracleTests`는 기존 원·축 정렬 상자 테스트가
검사하지 않은 **로컬 오프셋 Capsule과 비축정렬 삼각형 Polygon**의 회전 조회를
고정 시드 900개 입력에서 정적 원과 비교한다. Capsule은 원 중심에서 이동한
선분까지의 최단 거리에서 두 radius를 빼고, 삼각형은 CCW edge의 안팎 판정과
세 선분 최단 거리로 부호 있는 간격을 계산한다. 따라서 엔진의 GJK/TOI/ShapeDistance
또는 sweep transform 함수를 기하 판정에 사용하지 않는다.

- 시작·종료와 중간 시점의 회전은 World의 nlerp 시간 규약만 독립 재현하며,
  샘플링 간격은 1/512이다. 제자리 회전, 병진+회전, 음의 회전과 명백한 miss를 포함한다.
- 양 끝 간격이 +0.04m 이상이고 중간에 -0.04m보다 깊은 사례를 robust contact로
  분류한다. 양성의 hit fraction 기하 간격은 0.06m 이내인지, 명백한 miss는
  miss인지 확인한다. `--strict`는 robust 누락 시 실패한다.
- 첫 접촉 fraction의 정밀도나 모든 중간 접촉의 완전성은 샘플링만으로 보장할
  수 없다. Chain ghost vertex/adjacent edge는 별도 검증 범위이다.

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


## Sandbox 시작 회전과 상대 회전

Sandbox ShapeCast에는 서로 독립적인 **시작 회전(-180° ~ +180°)**과
**상대 회전(-90° ~ +90°)** 조절 항목이 있다.

- 시작 회전은 `worldShapeCastInput2::transform.rotation`에 지정하며,
  원·캡슐·상자의 **처음 놓인 자세**를 정한다.
- 상대 회전은 그 자세에서 이동하는 동안 추가로 회전할 각도다.
  예: 시작 45°, 상대 +30° → 시작 45°, 종료 75°.
- 상대 회전이 **0°**이면 시작 각도가 0°가 아니더라도 기존
  `world::castShapeClosest` (GJK 선형 ShapeCast)를 이용해 도형의
  **초기 방향을 그대로 유지하며 이동**한다.
  상대 회전이 0°가 아니면 동일한 초기 `transform`을 사용하여
  `world::castShapeSweepClosest` (TOI)로 조회한다.
- 시작 자세·종료 자세·미리보기 및 AABB는 이 두 설정을 반영하며,
  시작 각도를 바꾸면 충돌 결과와 미리보기 fraction이 갱신된다.
  원은 회전해도 외형이 같아 로컬 축 표시로 방향을 확인할 수 있다.
- **도형 로컬 오프셋**은 회전축인 `transform.position`에 대한 Shape 내부
  좌표의 변위다(기본 0, Sandbox 조작 범위 각 축 ±3m).
  원은 `circle.center`, 캡슐은 두 끝점, 상자는 정점과 centroid를 함께 이동한다.
  따라서 시작점/도착점을 같게 두어도 오프셋된 도형은 회전 중 원호를 따라 움직인다.
  자세·AABB·World TOI 조회는 모두 같은 offset geometry를 사용한다.
  비교용 RayCast는 회전축이 아닌 **초기 도형 중심**에서 병진 방향으로 쏘며
  원호 회전은 반영하지 않는다.
  로컬 오프셋은 Body의 `sweep.localCenter`를 변경하는 기능이 아니다.
  Sandbox 값의 범위 제한은 World API의 제약이 아니다.
- 시작 각도의 범위 제한은 Sandbox 조작을 위한 것이며 World API에는
  별도의 시작 회전 ±180° 제약이 없다. World 회전 sweep에서 제한되는 것은
  **상대 회전량의 크기 ±90°**다.

`sandboxShapeCastDemoTests`와 `sandboxShapeCastViewTests`는 시작 회전만 있는 선형 조회,
시작+상대 회전 조합, 로컬 오프셋 원의 순수 회전 충돌, 세 도형의 오프셋 좌표,
중간 AABB, 범위 밖 입력 및 ImGui 렌더링을 확인한다.


## 다중 도형 World 회전 ShapeCast 계측

Sandbox의 ShapeCast 시각화에서 **다중 도형 장면(40개 추가)**을 켜면
기존 네 대상 외에 Static / Dynamic / Kinematic Body가 교차 배치되고,
Circle / Polygon / Capsule이 혼합된 40개 도형을 생성한다.
체크박스를 끄면 해당 Body·proxy를 삭제해 원래 장면으로 돌아간다.
미리보기는 Physics Step이 아닌 공간 조회이며 테스트에서는 기본 장면을 유지한다.

회전 조회 UI에는 마지막 `castShapeSweepClosest` 호출의
`worldShapeSweepStats2` 수치가 표시된다:
- `treeCandidates`: 세 DynamicTree의 AABB 후보 중 callback에 들어온 개수
- `narrowphaseCandidates`: QueryFilter와 tight shape AABB를 통과한 후보 수
- `toiCalls`: 실제 `TimeOfImpact` 호출 수 (Chain의 초기 overlap 조회는 제외)
- `toi.distanceCalls`, `toi.transientAttempts`, `toi.transientDistanceCalls`,
  `toi.transientHits`: TOI 단계의 기본 GJK, 재검사 호출, 추가 GJK, 복구 수

이 계측은 `worldShapeSweepStats2*`를 선택적으로 전달할 때만 수행한다.
평소 호출은 null을 전달하여 기존 결과와 비용을 유지한다.
회전이 0인 GJK 기반 선형 ShapeCast는 이 회전 전용 계측에 포함되지 않는다.

`worldRotatingSceneBenchmark`는 Sandbox보다 큰 **324개 Body·Shape (18×18)**
Fixture에서 세 Body Tree, Sensor, 여러 geometry의 실제 World 조회를 수행한다.
긴 이동·짧은 이동·먼 경로·제자리 회전의 네 시나리오를 32회씩 조회한다.

- `--verify`는 계측 활성화 전후의 결과가 일치하는지,
  후보 수·TOI·GJK 카운터가 일관되는지 확인한다. CTest에 포함한다.
- `--csv`는 World 전체 `castShapeSweepClosest`와 **후보 전용
  `overlapAABB`**의 Release 중앙값을 각각 출력한다.
  이 후보 전용 조회는 회전 도형의 원점 기준 원형 보수 경계를 사용하지만,
  World 전체 조회의 모든 전처리·callback clipping과 동일한 알고리즘은 아니다.
  따라서 두 시간의 차이를 정확한 TOI 시간으로 해석하지 않는다.
- 양쪽 모두 7회 측정 중앙값, 4회 반복, warm-up을 사용한다.
  CI의 성능 값은 정보용이고 시간 임계값으로 테스트를 실패시키지 않는다.
  Tree 후보 개수 및 실제 TOI / 보수적 재검사 호출 횟수를 함께 비교한다.

이 검사는 공개된 충돌 결과를 변경하거나 최적화하지 않고
**BroadPhase에 의해 제외되는 도형과 실제 TOI 비용을 분리해 관찰**하는 단계다.


## 회전 ShapeCast의 현재 fraction에 따른 후보 제외 (#225)

`castShapeSweepClosest`는 가장 먼저 발견한 충돌이 시간 `t`에 있다면,
더 가까운 충돌을 찾기 위해 `[0,t)` 안에서만 TOI를 계산하면 된다.
기존 회전 World 조회는 `maxFraction`을 TOI에 전달하면서도 **후보 AABB는
전체 `[0,1]` 구간**으로 검사하여 불필요한 NarrowPhase 호출이 있었다.

이제 기존 DynamicTree Query의 전체 AABB 순회는 그대로 유지하되,
callback에서 `maxFraction < 1`일 때만 시작 origin에서 해당 fraction까지의
축별 선형 보간 구간과 moving shape의 최대 로컬 원점 반경을 합친
**보수적 clipped AABB**를 이용해 target의 tight AABB를 검사한다.

- 원점의 보간은 기존 `GetSweepTransform`의 `c1*(1-t)+c2*t`를 double에서
  재계산하며, float 연산 오차를 고려한 크기 비례 margin과 바깥 방향
  `nextafter`를 적용한다. 극단적인 수치로 bounds가 유효하지 않을 경우
  최적화만 생략하여 원래 후보 조회를 유지한다.
- `maxFraction`이 갱신될 때만 clipped AABB를 재계산한다.
  아직 충돌을 찾지 못했거나 callback이 `1`을 반환하면 기존 전체 범위로 조회한다.
  현재의 callback fraction 반환 규약, 초기 overlap, chain 앞면 검사를 변경하지 않는다.
- `worldShapeSweepStats2::clippedCandidates`는 필터/tight AABB를 통과한 뒤
  clipped AABB로 제외된 후보 수다. Tree의 탐색 비용 자체를 줄이는 것은 아니므로
  `treeCandidates`는 같은 장면에서 그대로일 수 있다.
- 기존 GJK 선형 ShapeCast, CCD 및 Solver에는 이 추가 필터를 적용하지 않는다.

`worldShapeCastTests`는 먼저 방문한 Static hit보다 뒤에 있는 Dynamic 도형을
TOI 이전에 배제하면서, 반대로 이후 Tree에 놓인 **더 앞선** 충돌은 계속 찾는지
검증한다. 기존 1,800개 독립 회전 TOI oracle, Sandbox 모델·View 테스트,
324개 다중 도형 `worldRotatingSceneBenchmark`도 함께 확인한다.
Release 결과는 이전 PR #224와 **동일한 seed/scenario의 NarrowPhase·TOI 호출 수 및
후보 제외 수**를 우선 비교하고, 시간은 CI runner 편차를 감안하여 해석한다.


## 회전 조회의 원점 간 거리 상한 필터 (실험)

순수 회전 또는 짧은 병진 이동의 경우 이미 축별 AABB 후보 검사를 통과했어도
실제로는 **회전 궤적의 외접 원에 닿을 수 없는** 후보가 있다.
World의 회전 ShapeCast는 각 도형의 proxy support 점과 radius가 모두
들어가는 **로컬 원점 중심의 원**을 계산하고, 움직이는 원점의
`[0,maxFraction]` 선형 이동 구간과 정지한 target 원점 사이의 최소 거리가
두 반경 합보다 **확실히 클 때만** TOI 이전에 후보를 제외한다.

- 움직이는 origin은 sweep의 `c1`에서 `c2`까지 선형 보간된다. 회전 중인
  모든 support 점은 그 원점으로부터 최대 proxy 반경 안에 있다.
  target의 모든 support 점도 해당 원점 중심의 최대 반경 안에 있으므로
  원점 간 최소 거리로 **충돌 불가능함을 증명한 후보만** 버릴 수 있다.
- 보수적 계산에는 double과 float 반올림 여유를 사용하고
  target Body 회전값의 길이까지 반영한다. 특히 TOI의 core contact 임계값
  `max(LINEAR_SLOP, radiusA + radiusB - LINEAR_SLOP) + 0.25 * LINEAR_SLOP`이
  실제 도형의 radius 합을 초과하는 부분을 추가로 허용한다. 따라서 작은
  원끼리 실제 표면이 약간 떨어져 있어도 TOI가 `Hit`으로 판정하는
  근접 접촉을 기존과 동일하게 보존한다. 계산이 유효하지 않으면
  해당 후보를 기존 TOI로 전달한다.
- 축별 AABB와 fraction clipping은 그대로 두고, 후보별 support radius
  계산 뒤 `TimeOfImpact` 호출만 줄인다. 추가된
  `worldShapeSweepStats2::distancePrunedCandidates`는 제외된 후보 수를 나타낸다.
- 중심 원의 상한은 정확한 회전 도형 외형보다 커질 수 있다.
  특히 로컬 원점에서 멀리 떨어진 Polygon은 필터 이점이 제한적이다.
  모든 장면에서 성능이 좋아진다고 가정하지 않으며,
  실제 통계 및 Release 시간을 기준으로 적용 여부를 결정한다.

`worldShapeCastTests`는 AABB 모서리에서 순수 회전/짧은 병진의
허위 후보를 버리는지와 실제 충돌을 남기는지 확인한다. 기존 1,800개
회전 TOI oracle 및 Sandbox/World 벤치마크도 회귀 조건으로 유지한다.


## 회전축이 도형 중심과 다른 ShapeCast의 AABB 축소

기존 World 회전 조회는 도형 local 원점으로부터 가장 먼 support 점과 radius를
기준으로 **원점 중심 외접 원**의 AABB를 사용했다. 원점에서 떨어진 원·캡슐·상자가
90도 이하로 회전하면 실제 원호가 지나가지 않는 반대 사분면도 후보가 될 수 있다.

회전량이 ±90도 이내인 World `castShapeSweep`는 이제 각 local support point가
시작/끝 회전 사이에 그리는 원호의 축별 최솟값·최댓값을 계산한다.
각 원호가 ±X 또는 ±Y축 방향의 극값을 통과하는지도 검사하고,
병진 원점 시작/끝 구간 및 geometry radius를 더해 보수적인 AABB를 만든다.

- `GetSweepTransform`의 정규화된 선형 회전 보간(nlerp)은 이 제한 구간에서
  시작·종료 방향 사이의 짧은 원호를 단조롭게 지난다.
- 축별 범위는 서로 다른 시각의 극값을 조합한 상한이며, 실제 이동 도형의
  경계를 넘어서 안쪽으로 잘라내지 않는다. float 연산 오차 여유와
  `nextafter`를 추가한다.
- 새 계산이 실패하면 기존 외접 원 AABB를 사용한다. 계산에 성공하면
  두 보수적 AABB의 교집합을 사용하여 원호 경계 여유로 확대된 부분도 안전하게 줄인다. GJK/TOI, callback, Filter, Chain 처리는 변경하지 않는다.
- 420개 고정 난수 원·캡슐·상자에 대해 시작 각도, 회전 방향, 병진,
  local offset을 변경한 뒤 sweep당 257개 pose가 모두 새 AABB 안에 있는지 검증한다.
  단일 회전의 반대 사분면 후보 제외 및 양방향 실제 World hit도 별도로 검사한다.
- 추가 support 점 계산이므로 도형의 로컬 오프셋이 작거나 Tree 후보가 적을 때는
  비용이 늘 수 있다. **후보 수/총 소요시간은 별도 벤치마크로 판단**해야 한다.


`worldRotatingSceneBenchmark`에는 로컬 원점에서 3m 떨어진 Circle이
짧은 병진과 ±1.4rad 회전을 수행하는 `offset_pivot` 시나리오(32회)를
추가한다. 기존 `candidate_only_hits`는 외접 원을 사용한 참고용 후보
조회이고, `tree_candidates`는 실제 World의 원호 AABB를 사용한 Tree callback 수다.
서로 다른 필터/조회 단계의 카운터이므로 정확한 후보 제거율로 혼동하지 않는다.


추가 원호 경계 계산은 shape local 정점들의 축별 AABB가 회전축(원점)을
포함하지 않을 때만 수행한다. 원점 중심의 Box·Capsule·Circle처럼
정점 경계가 원점을 포함하면 기존 외접 원 AABB를 그대로 사용해
짧은 이동·회전에서 추가 삼각함수/원호 경계 계산 비용을 피한다.
이는 후보 제외의 선택적 최적화일 뿐, 조회 정확성 계약은 같다.
