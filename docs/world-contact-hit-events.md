# World Contact Hit 이벤트

`world`는 Contact Begin/End 외에, **실제 Solver가 높은 속도의 접근을 처리한 Contact**의 충격을 조회할 수 있다.
접점 위치/법선, 충돌 전 상대 접근 속도, 최종 누적 법선 임펄스를 하나의 snapshot으로 제공한다.

```cpp
simulation.setShapeHitEventsEnabled( shapeIdToWatch, true );
simulation.setContactHitSpeedThreshold( 1.0f ); // m/s, 기본 1.0

simulation.Step( 1.0f / 60.0f );

for( const contactHitEvent2& hit : simulation.getContactHitEvents() )
{
    if( simulation.IsValid( hit.contact ) )
    {
        // hit.point / hit.normal: world 좌표
        // hit.approachSpeed: 충돌 전 normal 접근 속도의 양수 크기(m/s)
        // hit.normalImpulse: Solver의 최종 누적 normal impulse
    }
}
```

## 생성 조건 및 순서

- Shape별 `setShapeHitEventsEnabled(id, true)` 옵션이며 기본값은 false.
  두 고체 Shape 중 **어느 한쪽만 켜도** 보고한다. Begin/End 구독과 독립적이다.
- 양수 `approachSpeed`가 `getContactHitSpeedThreshold()`보다 **클 것**,
  해당 접점에 Solver가 양의 `normalImpulse`를 남길 것,
  그리고 마지막 `UpdateCollisions` 이후 그 Contact가 여전히 유효하고
  **실제로 touching** 상태일 것이 조건이다.
- `relativeNormalVelocity`는 Contact Constraint 준비 때 저장한 접근 속도다.
  힘/중력 적분 전 값이므로 Step 중간에 가속되어 발생하는 접촉은
  보고되지 않을 수 있으며, Continuous Collision 단계에서만 발생한 TOI도
  이번 이벤트의 대상이 아니다.
- 하나의 Contact에 두 접점이 있다면 조건을 충족하는 접점 가운데
  **충돌 전 접근 속도가 가장 큰 하나**를 보고한다.
  다른 Contact는 각각 별개의 Hit를 만들 수 있다.
- 법선은 Shape A에서 B 방향이며, 점은 Shape A의 COM-relative 접점을
  이번 Step의 delta transform으로 옮긴 근사 world 위치다.
  `normalImpulse`는 Solver 최종 누적 임펄스이며
  새로 추가된 임펄스만 뜻하지 않는다. 거리 단위는 m, 속도는 m/s다.
- `Step(0)`과 이벤트를 발생시키지 않은 Step은 이전 Hit 배열을 비운다.
  반환 `span`은 다음 Step 전까지만 유효하므로 보존하려면 복사한다.
  Contact나 Shape가 다음 Step 전에 삭제될 수 있으므로 `IsValid`로 확인한다.
- 이 Hit는 Begin/End와 **독립적으로 매 Step 발생할 수 있다**.
  두 Body가 이미 접촉 중이어도 새로운 고속 접근이 있으면 다시 보고할 수 있다.
  마찰 impulse, 연속 충돌/TOI impulse, 힘/에너지의 총량을 제공하는 PostSolve API는 아니다.

## 검증

`worldContactHitEventTests`는 기본 구독/threshold, 저속 제외, 고속 충돌의 실제
충돌점·법선·임펄스, 반대쪽 Shape 구독, Begin/End 구독 독립성,
Step(0) 후 이벤트 소멸, Body 삭제, Sensor 격리를 Debug와 Release에서 검사한다.

이벤트는 오디오·파티클·충돌 강도 표시 등의 게임 로직에 사용할 수 있다.
Solver의 수학이나 restitution 계산 순서를 변경하지 않는다.
