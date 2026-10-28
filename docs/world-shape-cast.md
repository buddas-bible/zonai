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
  `GetSweepTransform`과 동일한 보간을 사용한다.
- 청록색 AABB는 회전 도중 모든 자세를 포함하도록 로컬 원점 반경으로
  확장한 보수적 후보 범위이다. 회전이 없으면 기존 선형 swept AABB를 표시한다.
- 분홍색 중심 RayCast는 **회전을 반영하지 않는 비교용 조회**이며
  회전 ShapeCast 결과로 오인하면 안 된다.
- 미리보기는 실제 Body 물리를 변경하지 않으며 기존 demoSession 재생/일시정지/
  한 프레임 실행, Canvas 클릭과 Shift+클릭 입력을 재사용한다.

`sandboxShapeCastDemoTests`는 회전 World TOI 결과, 중간 자세와 보수적 후보 범위를
확인하고 `sandboxShapeCastViewTests`는 Headless ImGui 회전 UI/렌더링을 검사한다.
실제 Win32/DX11 화면의 DPI·입력 조작은 별도 수동 확인 대상이다.
