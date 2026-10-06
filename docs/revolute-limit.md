# 회전 조인트의 각도 제한

2026-10-07, master `fcbad51` 이후 기본 회전축에 기준 각도와 하한·상한을 추가했다. 두 연결점을 유지하면서 허용된 각도 안에서는 회전을 자유롭게 둔다. 경계에서는 바깥쪽 움직임을 막고 안쪽 복귀는 허용한다. 각도를 transform에 강제로 덮어쓰지 않는다.

## 설정과 관찰값

`revoluteJointDef`에 `referenceAngle`, `enableLimit`, `lowerAngle`, `upperAngle`을 추가했다. API의 각도 단위는 rad다. 기본 기준 각도는 0, 제한은 off, 하한·상한은 0이다. `currentAngle`은 이제 **B의 A에 대한 상대 회전에서 기준 각도를 뺀 값**을 -pi부터 pi까지로 반환한다. 기준 각도 0인 기존 호출의 의미는 유지된다. 생성 시 지정한 기준은 이후 물체를 움직여도 유지된다.

`setRevoluteJointLimit(id, enableLimit, lowerAngle, upperAngle)`로 제한 여부와 범위를 바꾼다. 유한한 lower <= upper를 precondition으로 요구하며, World는 두 경계를 ±0.99*pi로 clamp한다. Box2D main의 wrap 경계 회피 범위를 따른다. main은 뒤집힌 두 값을 정렬하지만 Zonai는 기존 Distance API처럼 입력 순서를 요구한다. Release에서 잘못된 ID나 입력을 복구하는 API는 아니다.

두 경계가 같으면 그 각도를 유지한다. Distance Joint의 같은 min/max가 별도 `length` 모드로 돌아가는 규칙과 구분한다. 두 Body 모두 회전할 수 없으면 각도 제약은 적용하지 않지만 기존 연결점 제약은 유지한다.

설정이 실제로 바뀌면 선형·하한·상한 임펄스를 모두 비우고 연결된 non-static component를 깨운다. 같은 설정은 cache/sleep을 유지한다. 질량·관성·pose 변경과 substep 시간 변경도 cache를 무효화한다. 기존 생성/삭제·ID·충돌 제외·Island 경로를 공유한다.

조회의 `torque`는 **마지막 substep의 (lowerImpulse - upperImpulse) / h**, B에 작용하는 제한 반력 토크(N*m)다. 양수는 각도를 늘리고 음수는 줄인다. 중심 밖 연결점의 선형 임펄스가 만드는 `r x P`와 모터 토크는 이 값에 포함하지 않는다. `force`는 기존 연결점 반력이다. Paused/sleep 상태는 마지막 결과를 유지한다.

## 한 방향 회전 제약

상대 각도 a, 상대 각속도 w = wB - wA에 대해 하한의 separation은 `a - lower`, 상한은 `upper - a`다. 각도 유효 질량은 `1 / (invInertiaA + invInertiaB)`다. 두 누적 임펄스를 각각 0 이상으로 제한하여 경계를 한 방향 제약으로 만든다.

- 하한: w를 읽고 양의 임펄스로 각도를 늘린다.
- 상한: -w를 읽고 음의 임펄스로 각도를 줄인다.
- Warm start: 두 임펄스의 차이를 A/B에 반대로 적용한다.

범위 안의 C > 0에는 `bias = C/h`를 사용한다. 남은 각도를 한 substep에 넘는 속도만 제한하므로 느린 자유 회전은 유지된다. 이미 위반한 C <= 0에서는 biased pass만 softness로 위치 오차를 줄인다. relaxation은 위반 위치 bias를 끄고 바깥쪽 속도만 제한한다. 각도 안정화 계수는 기존 연결점의 수치 안정화와 같으며 사용자 회전 스프링은 아니다. 하중과 반복 횟수에 따라 작은 경계 오차가 남을 수 있다.

Box2D처럼 하한 → 상한 → 연결점의 순서로 풀고, 각 단계가 갱신한 최신 속도로 다음 제약을 푼다. 현재 A/B 누적 회전과 생성 기준을 곱한 상대 rotation에서 atan2로 각도를 구해 ±pi를 넘는 기준 각도도 처리한다. 회전 횟수나 한 substep 중 여러 바퀴의 경계 통과를 추적하지 않는다. 강한 충격·극단 계수에서 무조건 정확한 경계를 보장하는 기능은 아니다.

비교 기준은 Box2D main [`ac7c751`의 revolute_joint.c](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/revolute_joint.c)다. 공개 [3.1 설명](https://box2d.org/documentation/group__revolute__joint.html)의 reference angle 개념을 사용하되, main의 local frame 구조를 Zonai의 기존 origin anchor/referenceAngle로 표현한다. 3.1 문서의 ±0.95*pi와 main의 ±0.99*pi를 혼동하지 않는다.

## Sandbox 실험

‘조인트 / 회전축과 막대’는 기존처럼 자유 회전으로 시작한다. 기본 범위는 ±45도이며 **각도 제한**은 접혀 있다. 펼쳐 제한을 켜고 최소·최대 각도를 degree로 조절한다. 직접 숫자 입력도 전체 범위와 반대 경계를 넘지 않게 clamp한다. 기준 각도와 제한 토크를 같은 설정 안에서 관찰한다.

초록 선은 하한, 빨강 선은 상한, 회색 호는 허용 범위다. 보라색 화살표는 실제 막대 아래쪽 방향과 일치하며, A가 움직이면 기준과 경계도 함께 움직인다. S/회전 버튼·A/D·Mouse drag로 경계와 복귀를 실험한다. 같은 각도, 제한 끄기, Reset을 비교하면 각도 제약과 연결점 제약의 차이를 볼 수 있다. Reset은 제한 off/±45도로 돌아간다.

## 검증 범위와 다음 단계

기존 NDEBUG 독립 Revolute 단위/World 검사에 하한·상한 부호, 각운동량, interior 자유 회전, 안쪽 복귀, speculative 속도, bias/relaxation, 기준·현재 회전·wrap, warm signed sum, h 변경 및 고정 회전의 cache를 추가했다. World는 substep 1/4에서 지속 토크와 평형 반력, 같은 각도, 설정 no-op/wake, 질량/pose cache 및 제한 off를 검사한다. Sandbox model은 실제 중력 아래 중심 밖 막대가 경계에 도달하면서 축에 연결되는지 확인한다.

Windows ImGui headless 검사는 기본 접기와 toggle, 숫자 입력 clamp, 경계 geometry 생성을 실행한다. 실제 OS 입력/GPU/창의 시각 검증은 별도다. 전체 CTest 45개와 Windows Release runtime 15개를 유지한다. 기존 assert 기반 Release 검증 공백은 이 작업의 범위가 아니다. 최종 빌드·전체 검사·mutation·리뷰·원격 CI 결과는 작업 PR에 기록한다.

최종 로컬 검증은 Sandbox ON 전체 Debug/Release build와 각각 45/45 CTest, Sandbox OFF 전체 Release build와 45/45 CTest 통과다. 누적 임펄스 clamp 제거, 상한 부호 반전, warm start의 잘못된 합, speculative bias 제거, 현재 A 회전 누락의 다섯 mutation을 모두 검사에서 잡고 원본 복구 후 통과했다. 독립 최종 리뷰의 남은 지적은 없다.

다음은 목표 각속도와 최대 토크를 가진 **회전 모터**다. 제한과 함께 방향 전환·제동·경계에서의 반력을 학습한 뒤 자동차나 조나이 선풍기 같은 연결 데모로 진행한다.
