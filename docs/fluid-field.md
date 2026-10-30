# 2D 바람 / 유체 속도장 — Part 1

zonai의 첫 유체 subsystem은 `src/fluid/fluidField2`에 둔 2D MAC(Staggered) 격자다. 기존 `World`와 충돌 Solver를 변경하지 않으며, 별도의 시간 적분과 압력 투영을 수행한다.

## 범위와 계산

- 균일한 격자: `width × height` cell, 셀 크기 m, 좌하단 world origin.
- X 속도 `u`: 수직 face `(width+1) × height`, Y 속도 `v`: 수평 face `width × (height+1)`. 두 속도를 구분해 놓으면 cell 발산을 face flux 차이로 계산할 수 있다.
- `addVelocity(position, deltaVelocity, radius)`: 원형 영역의 국소 속도 변화. 이것은 질량 또는 힘 주입이 아니라 face 속도에 직접 더하는 실험용 source다. 실제 유체 구동 방법은 이후 단계에서 분리한다.
- `step(dt, iterations)`: 이전 속도장 역추적 + 쌍선형 보간의 Semi-Lagrangian advection, 이어서 Jacobi pressure projection. 내부 face에 압력 기울기를 빼고 외곽의 수직 face는 0으로 고정한다. 경계는 닫힌 slip wall/no-through 근사다.
- `sampleVelocity(position)`: 두 staggered component를 쌍선형 보간해 world-space 속도를 반환. 영역 바깥에서는 0.
- `getDivergenceRms()`: 현재 유체 cell의 |∇·u| RMS. 투영 반복 횟수가 제한되므로 **정확한 0은 아님**.

관계식:

```text
u* (x) = u_previous(x - dt * u_previous(x))
∇²p = (∇·u*) / dt
u_next = u* - dt * ∇p
```

압력은 일정한 기준값이 없는 Neumann 문제이므로 값 자체가 아닌 압력차만 의미를 갖는다. 반복 해법은 유한 횟수에 멈추며, 이류도 보존형 스킴이 아니어서 수치적 감쇠가 있다.

## Sandbox: 유체 / 2D 바람·유체 속도장

- `demoKind::windFlow`의 24×14, 셀 0.5m 속도장을 시각화한다. 파란 화살표는 샘플링된 속도, 노란 점은 바람 source 위치다.
- 시간에 따라 Y 방향이 진동하는 국소 wind source를 주입하고 한 물리 Step마다 격자도 진전한다.
- 세 Dynamic Body는 국소 유체 속도 `v_fluid`와 강체 중심 속도 `v_body`의 차이에 비례하는 경험적 힘 `F = dragCoefficient * (v_fluid - v_body)` (계수 단위 N·s/m)을 받는다. 따라서 강체가 속도장을 따라 가속하며, 원래 `World`의 force/velocity/Contact solver를 재사용한다.
- 바람 source On/Off, source 강도, 경험적 drag 계수, 양방향 obstacle coupling On/Off, 흐름+Body 초기화 기능을 제공한다. Drag를 0으로 설정하더라도 양방향이 활성화되면 압력 반력이 남는다.
- `Step(0)`은 유체를 전진시키거나 힘을 주입하지 않는다.

## 한계 (유체의 첫 단계)

이 구현은 **비압축성 속도장 실험**이지 공기/물 전체 물리 모델이 아니다.

- 기본 `step(dt)`는 외곽 닫힌 경계만 고려한다. Part 2의 obstacle overload는 원형 강체의 셀 경계에 한정한다. 다른 형상, 개방 경계, 보존형 양방향 운동량 전달은 아직 없다.
- 정밀한 표면적·압력 부력, 점성 응력, 실제 유체 밀도 편집, 열, 자유 수면, 연기/염료 수송은 구현하지 않았다. 거친 격자에서만 압력 토크를 집계한다.
- `addVelocity`는 속도 injection이며 보존적인 inflow boundary/source는 아니다. Drag 반력에는 Part 4의 별도 impulse API를 사용한다.
- Jacobi는 수렴이 느릴 수 있다. 큰 해상도에서 다중 격자/PCG 또는 더 정확한 수송 기법을 검토한다.
- 특히 시간이 큰 경우 격자 이류의 샘플링은 수치 확산을 낳으며, 유체와 Body가 공유하는 고정 timestep/동기화를 더 구체화해야 한다.

## Part 2 — 움직이는 원형 Solid와 압력 반력

`fluidField2::step(dt, obstacles, feedback, iterations)`는 매 물리 Step에서 원형 강체의 world-space 중심·반경·선속도·각속도를 입력받아 장애물과 압력 반력을 계산한다.

- 원의 내부에 **중심이 들어간 격자 셀**을 Solid로 마스킹한다. 실제 원 경계와 일치하지 않는 계단형 경계(격자 해상도 제한)다. 두 원이 겹친 셀은 최초 원이 소유한다.
- Solid와 접하는 MAC face의 **법선 속도**는 `v_solid(face) = v_center + omega × (face-center)`로 설정한다. 유체·유체 face만 압력 구배로 수정하며 Solid 내부는 압력 미지수에서 제외한다. 유체 셀의 `div(u)` 계산에는 Solid 경계 속도를 포함한다.
- 투영 후 각 유체·Solid 경계에서 `F ≈ p_fluid * normal_solid * cellSize * unitDepth`를 적분해 강체별 `fluidObstacleForce2{force, torque}`를 반환한다. 유체의 기준 밀도는 1kg/m³, 깊이는 1m로 고정한 학습용 단위 모델이다.
- World는 별도로 움직이는 강체이고 `fluidField2`는 ID나 Body를 보유하지 않는다. Sandbox에서 현재 Circle Shape를 world 좌표로 변환해 전달하고, 결과 압력 force/torque를 `ApplyForceToCenter`, `ApplyTorque`로 전달한다.
- 경험적 Drag `F = dragCoefficient * (v_fluid-v_body)`는 유지한다. Part 4부터 양방향 모드의 Drag는 유체 내부 MAC face의 유효 질량을 기준으로 충격량을 분배하고, 실제 주입된 양만큼의 반대 충격량을 강체에 적용한다. 압력/외벽/이류의 전체 운동량까지 보존하는 것은 아니다.
- Sandbox에서 양방향 모드를 끄면 원래 단방향 실험이 동작한다. 점유 셀과 물체별 압력 반력을 보고, 움직이는 원이 정지 유체에 어떤 흔적을 남기는지 비교할 수 있다.

이 방식은 **원형 강체의 격자화된 no-through 경계 및 압력 feedback 첫 실험**이지 완성된 양방향 CFD-강체 solver는 아니다. 질량 중심이 빠르게 이동할 때 계단형 점유 셀이 불연속적으로 바뀌며 압력에 인공적인 힘을 더할 수 있다. 압력 gauge, 유체 점성/마찰, Solid/Fluid 체적 보존과 시간 적분의 완전 동기화, 일반 polygon/chain의 보존형 cut-cell coupling은 후속 구현 대상이다. 일반 Body collision은 원래 zonai World Solver가 해결한다.

## Part 3 — Circle / Capsule / Convex Polygon 공통 장애물

`fluidShapeObstacle2`는 World handle 없이 `shapeGeometry`, `transform2`, world-space COM, 선속도와 각속도를 전달한다. 기존 `fluidCircleObstacle2` API는 호환용 overload로 남겨 두지만 동일한 일반 도형 solver로 위임한다.

- 유체 격자 셀 중심을 도형의 **로컬 공간으로 역변환**한 다음 Circle/Capsule/Convex Polygon의 내부 여부를 검사한다. Capsule은 축 선분 거리, Convex Polygon은 반평면과 모서리 거리(둥근 radius 포함)로 판정한다. `polygon2`의 2-vertex rounded core도 지원한다.
- MAC face 표면 속도는 `linearVelocity + angularVelocity × (face - centerOfMass)`를 사용한다. 압력 힘/토크는 기존 방식으로 계산하고 feedback은 obstacle 배열의 인덱스에 대응한다.
- Sandbox 바람 장면은 Circle, Capsule, Polygon Box를 함께 배치한다. 기존의 단방향 비교 모드와 압력/drag 옵션은 유지한다.
- Segment, ChainSegment, monostate는 점유 셀을 만들지 않는다. 정적 Polygon 등도 0속도를 제공하면 사용할 수 있다.
- 유체 solver가 사용하는 중심점 occupancy는 실제 표면을 잇는 cut-cell이 아니다. sub-cell 크기, 이동하는 장애물의 cell flicker, 보존형 운동량 교환 문제는 남는다.
- 복합 Body에 여러 Shape가 붙어 있을 때는 Shape 별 obstacle과 body force 합산이 필요하며 아직 Sandbox wind 장면에서는 한 Body에 한 Shape만 둔다.

다음 단계에서는 일반 강체를 위한 Shape traversal/force accumulation을 정리하고, 움직이는 장애물과 유체의 **운동량 budget 오차**를 정량화할 필요가 있다.

## Part 4 — MAC face 기반 Drag 충격량 교환

기존 구현은 Body에 가한 Drag와 유체에 주입한 속도 변화가 **서로 다른 운동량**을 의미하는 문제가 있었다. 이제 `fluidField2::applyMomentumImpulse(position, J, radius)`로 Drag의 한 Step 교환량을 직접 전달한다.

- `getFluidMomentum()`은 **내부 fluid-fluid MAC face**의 두 속도 성분에 각기 `m_face = ρ · h² · depth`를 곱해 합산한다. 실험 모델은 `ρ = 1 kg/m³`, `depth = 1 m`이므로 face 유효 질량은 `h² kg`이다. 외곽 wall와 solid에 접한 운동학 경계 face는 제외한다.
- `applyMomentumImpulse`는 지정 반경 내부의 유효 U/V face에 거리 기반 가중치를 매기고 **성분별로 가중치 합을 정규화**해 속도를 변경한다. 바깥 영역, 잘못된 입력, 유효 face가 없는 성분은 0 충격량을 반환한다. float 표현 오차를 포함한 *실제 face 운동량 증가량*을 반환한다.
- Sandbox는 Pressure Solver를 수행한 다음 Body별 Drag 목표 `J = F_drag·dt`를 계산한다. 양방향 모드에서는 `applyMomentumImpulse(-J)`가 실제 유체에 가한 충격량 `J_fluid`를 보고하고 **강체에는 `J_body = -J_fluid`만 적용**한다. 주변 유효 face가 전혀 없으면 강체 Drag도 그만큼 줄어든다.
- Body와 유체의 Drag 충격량을 합한 `J_body + J_fluid`을 Sandbox에 표시한다. 두 모드를 쉽게 비교하도록 단방향 모드는 종전처럼 유체의 Drag 피드백을 생략한다.
- Solid 내부의 속도를 그대로 Drag 샘플로 쓰면 강체 경계 속도가 읽히므로, 양방향 Drag는 Body 바깥 4방향에서 흐름을 샘플링하고 유체를 교란하는 방향 바깥쪽에 반작용을 분배한다.

### 보존 범위의 경계

여기서의 **교환 잔차가 거의 0**이라는 말은 *Drag 전달 순간*에 한정된다. 유체는 다음 Step의 이류·압력 투영으로 다시 속도가 바뀌고, 벽/움직이는 Solid의 운동학 경계도 외력을 전달한다. 현재의 압력 traction 역시 face impulse budget과 쌍으로 업데이트되지 않는 근사다. 따라서 수치적/물리적인 **전체 유체+강체 운동량 보존을 아직 주장하지 않는다**. 완전한 압력-강체 연동 및 cut-cell/flux 기반 이류는 다음 단계의 별도 과제다.

회귀 테스트는 정상 격자·단일 U face·Solid 인접 face·지원 범위 밖 입력에서 **실제 유체 운동량 변화 == 반환된 충격량**, 그리고 Sandbox의 **Body Drag + 유체 Drag ≈ 0**, One-way 모드와 Pressure 단독 모드를 각각 검사한다.

## Part 5 — 압력·강체·외벽 운동량 수지

유체에 대한 기존 `getFluidMomentum()`은 외곽 wall face와 Solid에 인접한 운동학 face를 제외한 **fluid-fluid MAC face의 dual 질량 운동량**이다. 이번 단계에서는 이 정의를 그대로 유지하면서 압력 투영 전후의 운동량 차이와 압력에 의한 강체/외벽 충격량을 분리한다.

- `getPressureMomentum()`은 마지막으로 완료된 `step()`의 `fluidPressureMomentum2`를 반환한다. `fluid`는 실제 유체 내부 face의 압력 보정 전후 운동량 차이, `solid`는 각 장애물에 반환된 압력 힘의 합 × dt, `wall`은 도메인 외곽의 pressure traction × dt다. `residual() = fluid + solid + wall`.
- **외벽을 생략하고 강체 압력 반력과 유체 운동량만 비교하면 보존 오차가 있는 것처럼 보인다.** 닫힌 압력 문제에서 외벽도 운동량을 전달하기 때문이다. 외벽의 힘은 왼쪽 `-p·h`, 오른쪽 `+p·h`, 바닥 `-p·h`, 천장 `+p·h`를 합산한다(밀도=1, 깊이=1 기준).
- `getStepMomentum()`은 `before → afterObstacles → afterAdvection → afterPressure` 스냅샷을 제공해 셀 점유 변경으로 유효 face 집합이 달라진 효과, Semi-Lagrangian 이류 효과, 압력 효과를 분리한다. 이 단계별 기록은 디버그 관측량이며 유체 질량 전체의 물리적인 운동량 총합을 뜻하지 않는다.
- `clear()`는 두 진단량도 초기화한다. Drag 충격량 주입은 압력 투영 **이후** 수행되므로 마지막 `step()`의 `afterPressure`에는 Drag 변화가 들어있지 않다. Sandbox의 Drag 장부는 별도로 확인해야 한다.

```text
Pressure residual = ΔP_fluid-fluid faces + J_solid + J_outer walls
Step fluid ΔP     = occupancy ΔP + advection ΔP + pressure ΔP
```

**한계:** 위 합이 작은 것은 동일한 압력장/계단형 마스크에서 계산한 **이산 face 수지의 대수적 일관성**을 검증한다는 뜻이다. 실제 연속체의 물리적인 운동량 보존은 아니며, moving obstacle 경계가 밀어낸 유체 체적, 격자 마스크의 불연속적 전환, 비보존형 이류, 압력–강체 속도 결합의 완전 동기화까지 해결하지 않는다. 압력 게이지 상수도 일관된 닫힌 경계 계산일 때만 상쇄된다.

유체 코어 회귀 테스트는 외벽만 있는 영역, 장애물+바람, 움직이는 장애물에서 이 세 항의 잔차와 단계별 합을 확인한다. Sandbox 테스트는 월드에 반환되는 압력 force가 계측된 Solid 충격량과 같은지, reset/단방향 토글 동작을 확인한다.

## 다음 구현 후보

1. moving obstacle의 체적 교환 및 cut-cell boundary, pressure–rigid 동기 결합의 정밀화.
2. 복합 Body의 다중 Shape 집계, static obstacle, Segment/Chain의 별도 표면 경계 표현.
3. 연기/염료(tracer) 수송, 점성(확산), 회전(vorticity) 표현.
4. 압력 solver 수렴 기준과 성능 평가, 해상도 증대.
5. Cloth/Soft Body 입자에 속도장을 샘플링해 바람 힘 전달.

## 검증

`fluidFieldTests`는 크기/유효성/경계/속도 source/영역 밖 조회/Step(0)/비압축성 투영과 반복 수렴/지속 source를 확인한다. `sandboxWindFlowDemoTests`는 기존 단방향 실험과 원형 장애물로 생성된 압력 반력/유체 교란, 토글, 초기화를 확인한다. 두 테스트는 Linux/Windows library+headless Sandbox 경로에서 실행할 수 있고 Win32/DX11 화면 확인은 별도다.
