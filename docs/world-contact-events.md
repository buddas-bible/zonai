# World Contact Begin / End 이벤트

`world`는 기존 `GetBodyContactData`와 `UpdateCollisions`의 현재 접점 조회에 더해,
**마지막 Step의 실제 Contact 시작/종료**를 Shape pair 이벤트로 제공한다.
`GetSensorBeginEvents`/`GetSensorEndEvents`와 달리 일반 **고체 충돌 Contact**만 대상이다.

```cpp
const shapeId box = simulation.CreateShape( body, MakeBox( { 0.5f, 0.5f } ) );
simulation.SetShapeContactEventsEnabled( box, true );

// 물리 Step 또는 정적 배치 검사 Step(0)
simulation.Step( 1.0f / 60.0f );

for( const contactBeginEvent2& event : simulation.GetContactBeginEvents() )
{
    if( simulation.IsValid( event.shapeA ) && simulation.IsValid( event.shapeB ) )
    {
        // 처음 실제 접촉한 두 Shape
    }
}

for( const contactEndEvent2& event : simulation.GetContactEndEvents() )
{
    // Shape/Body가 이미 파괴됐을 수 있으므로 IsValid 확인
}
```

## 의미 및 수명

- Contact 이벤트는 **opt-in**이다. `SetShapeContactEventsEnabled(id, true)`를
  일반 Shape **한쪽만 켜도** 양쪽 사이의 Begin/End를 보고한다.
  기본값은 false이고 Sensor overlap은 기존 별도 API만 사용한다.
- `Step()`이 마지막으로 갱신한 Contact manifold를 비교한다.
  `IsTouchingManifold` 기준의 실제 접촉만 Begin 대상이며,
  Contact가 살아 있고 `manifold.pointCount > 0`이더라도
  **모든 점의 separation이 양수인 speculative 상태**는 비접촉이다.
- 이전 Step에서도 실제 접촉했다면 Begin을 반복하지 않는다.
  떨어짐/비접촉이 되면 End를 한 번 보고하고 재접촉하면 Begin을 다시 보고한다.
  이벤트 구독을 끄는 경우도 다음 Step에 End가 보고된다.
- Shape/Body 삭제, filter 변경, 충돌을 차단하는 Joint 생성 등으로 Contact가
  **Step 사이에서 파괴되어도** 마지막으로 보고했던 접촉의 End를 예약하고
  다음 Step에서 내보낸다. 삭제 후 같은 Shape 슬롯을 재활용하더라도 과거의
  `generation`이 들어간 Shape handle을 반환한다.
  구형 `shapeId`는 `IsValid`가 false여도 이벤트 값 자체를 비교할 수 있다.
- 두 이벤트 getter의 `span`은 다음 `Step` 전까지 유효하다.
  사용자 코드에서 보존하려면 복사한다. 상태 변경은 다음 Step의 이벤트를 예약할 뿐,
  **이미 공개한 이벤트를 즉시 수정하지 않는다.**
- 이벤트는 최종 상태 관점이므로 **한 Step 도중 잠깐 생겼다가 사라진 접촉**은
  보고하지 않을 수 있다. 이는 TOI 접촉 통과나 Solver의 중간 상태에 대한
  모든 접촉 이력 기록 API가 아니다. Contact callback, Sensor 이벤트 및
  현 접촉 정보 조회는 계속 별개로 제공한다.
- 기존 Contact solver와 BroadPhase, `UpdateCollisions` 콜백 실행 순서에는
  영향을 주지 않는다. 두 Shape의 callback 구독 상태를 검사하고 이전 Step
  보고 상태가 바뀐 pair만 이벤트에 기록한다.

## 검증

`worldContactEventTests`는 Debug/Release의 runtime check로 다음을 검사한다.

- 기본 비구독, Shape 한쪽 구독, 반복 Step에서 Begin 중복 없음
- 양수 separation의 speculative Contact는 End, 다시 침투하면 Begin
- 마지막 구독 해제와 다른 Shape 구독으로 전환
- 필터 변경에 따른 Contact 파괴/복구 및 정지 proxy의 재탐색
- Shape 파괴/동일 슬롯 재생성 시 구세대 End와 신세대 Begin
- Body 파괴에 대한 지연 End, 후속 Step의 이벤트 초기화
- Sensor가 일반 Contact 이벤트로 섞이지 않음

실제 Win32/DX11 Sandbox에서의 이벤트 시각화는 이 API 구현 범위에 포함하지 않는다.
