# 기본 회전 조인트와 회전축 실험

2026-10-07, master `ba50e1b` 이후 기본 Revolute Joint를 추가했다. 두 물체의 연결점은 일치시키고 상대 회전은 허용한다. 거리 조인트가 한 방향의 거리를 제한했다면, 회전 조인트는 연결점의 x/y 이동을 함께 제한한다. 각도 제한·회전 모터·각도 스프링은 다음 단계다.

## API와 수명

`world::createRevoluteJoint(revoluteJointDef)`로 생성하고 `getRevoluteJointData`로 관찰한다. Body A/B와 두 로컬 연결점을 지정하며, 적어도 한 물체는 Dynamic이어야 한다. 연결점은 물체 원점 기준이다. Shape 추가나 밀도 변경으로 질량 중심이 이동해도 실제 연결 위치는 유지한다.

조회 값은 두 월드 연결점, B의 A에 대한 상대 각도(rad), 마지막 substep의 B에 작용한 반력(N)이다. 각도는 -pi부터 pi까지이며 회전 횟수를 누적하지 않는다. 반력은 누적 임펄스를 substep 시간으로 나눈 값이다. 토크나 모터 힘을 보고하는 API는 아직 없다.

기존 Joint 슬롯·ID·Body 연결·충돌 제외·Island·wake/sleep·삭제 경로를 공유한다. `collideConnected`는 기본 false이며 제거하면 해당 충돌 제외가 해제된다. Body 삭제는 연결된 Joint도 제거한다. 질량/관성/pose 변경 시 이전 임펄스를 비우고, 시간 간격이 바뀌면 warm start를 사용하지 않는다.

## 연결점의 속도와 임펄스

solver는 질량 중심 기준의 팔 길이 `r = Rotate(localAnchor - localCenter)`를 준비한다. 반복 중에는 누적 회전으로 r을 다시 회전한다. 연결점 속도는 `v + w x r`이므로, 중심 속도만 같게 만들면 회전하는 연결점이 미끄러진다.

두 연결점의 상대속도와 위치 오차로 필요한 선형 임펄스 P를 구한다. 중심 밖의 연결점에서는 x/y 임펄스가 모두 회전에 영향을 주므로 다음 2×2 행렬을 함께 푼다. 각 물체의 역질량을 m, 역관성을 i로 표기한다.

```
k11 = mA + mB + iA * rA.y² + iB * rB.y²
k12 = -iA * rA.x * rA.y - iB * rB.x * rB.y
k22 = mA + mB + iA * rA.x² + iB * rB.x²
K = [ k11 k12; k12 k22 ]
```

현재 회전에 따라 K도 매 반복 다시 계산한다. 누적 임펄스의 증가분을 B에 적용하고 A에는 반대로 적용하며, 각속도에는 `i * (r x P)`를 반영한다. 상대 각도를 고정하는 제약은 없다. singular K는 역행렬을 0으로 처리한다.

biased pass는 위치 오차와 기존 고정 거리 조인트와 같은 수치 안정화 softness를 사용한다. 이것은 사용자가 조절하는 회전 스프링이 아니다. relaxation pass는 bias/softness를 끄고 연결점의 상대속도만 제거한다. 이전 임펄스를 적용할 때도 현재 회전의 팔 길이를 사용한다.

Box2D main [`ac7c751`의 revolute_joint.c](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/revolute_joint.c)의 point constraint 계산과 [공식 Revolute Joint 설명](https://box2d.org/documentation/group__revolute__joint.html)을 참고했다. Box2D의 solver set이나 local frame 구조 대신 Zonai의 기존 origin anchor와 serial substep 구조에 맞췄다.

## Sandbox에서 관찰하기

데모 목록의 **조인트 / 회전축과 막대**를 선택한다. 정적 원점과 막대 윗부분을 연결한다. 하늘색 점은 고정 축, 보라색 점은 막대 연결점이며, 두 점 사이 선이 위치 오차다. 화살표는 막대의 방향이다.

- Canvas 위 A/D 유지: 막대에 수평 힘.
- Space: 위쪽 임펄스. S 또는 ‘막대 회전시키기’: 회전 임펄스.
- ‘막대 옆으로 밀기’: 수평 임펄스.
- 왼쪽 drag: Mouse Joint로 잡기. 오른쪽 클릭: cursor 방향으로 밀기.
- 실험 패널: 상대 각도(degree), 연결점 오차(m), 반력(N).

정지 상태에서도 임펄스는 속도를 바꾼다. 한 step씩 진행하면 회전이 허용되면서 연결점은 축에 머무는지 확인할 수 있다. Mouse Joint를 놓거나 UI로 입력이 이동하면 마우스 연결만 제거되고 원래 회전축은 유지된다.

## 검증 범위

`revoluteJointTests`와 `revoluteJointWorldTests`는 NDEBUG와 무관한 runtime check다. 중심/중심 밖 연결점, 자유 회전과 연결점 속도, 현재 회전의 K, warm start, relaxation, 시간 간격 변경, singular K를 검사한다. World 검사는 반력 캐시·원점/질량 중심 구분·질량/pose 변경·ID 수명·연결 충돌·sleep/wake·이동하는 Kinematic 축을 다룬다.

기존 `sandboxDemoTests`에 세 번째 데모의 전환·reset·키 입력·Mouse Joint 취소를 추가한다. Windows Sandbox ON 구성은 ImGui headless frame에서 그리기와 두 막대 버튼의 실제 임펄스 적용도 검사한다. 실제 창/GPU/OS 입력의 시각 검증은 별도다. 기존 assert 기반 테스트의 Release 공백을 해결했다고 주장하지 않는다.

최종 소스로 Windows Sandbox ON 전체 Debug/Release build와 각각 45/45 CTest, Sandbox OFF 전체 Release build와 45/45 CTest를 확인했다. 연결점의 회전 속도 누락, K의 결합항 제거, relaxation에 bias 적용, warm start의 현재 회전 누락이라는 네 가지 mutation을 모두 테스트가 잡았다. 원본 복구 후 다시 통과했으며, 최종 리뷰에서 권고한 중심 밖 Dynamic–Dynamic 검사도 추가해 양쪽 각속도·연결점 속도·총 선형 운동량을 확인했다. 원격 CI 결과는 해당 PR에 기록한다.

다음 단계는 기준 상대 각도와 각도 제한을 학습하고, 이어 회전 모터를 추가하는 것이다. 자동차나 조나이 선풍기는 모터까지 확인한 뒤 같은 데모 host에 추가한다.
