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
- 세 Dynamic Body는 AABB 축방향의 4개 표면 근사점에서 `F_i=(dragCoefficient/4)*(v_fluid,i-(v_COM+ω×r_i))`의 경험적 저항력을 받는다. 각 힘을 작용점에 적용하므로 회전 저항 토크도 계산하며, `World`의 force/Contact solver를 재사용한다.
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
- Solid 내부의 속도를 그대로 Drag 샘플로 쓰면 강체 경계 속도가 읽히므로, Drag는 AABB 축방향의 4개 표면 근사점에서 한 셀 바깥 유체 속도를 샘플링한다. 양방향에서는 각 점의 반작용을 주변 유효 MAC face로 전달하고 실제 전달된 양을 반대 힘으로 강체의 동일 작용점에 적용한다.

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

## Part 6 — 움직이는 Solid의 셀 전이와 재노출 face 속도 복원

이전 구현은 장애물이 이동해 기존의 Solid 경계 face가 **fluid-fluid face로 열리면**, 예전 강체 운동학 경계 속도가 유체의 초기 속도로 그대로 남을 수 있었다. 특히 빠르게 움직이는 강체가 지나간 자리나 회전으로 드러난 격자에서 유체 속도가 부자연스럽게 튀는 원인이 된다.

- `fluidField2`는 직전 `solidOwner_`를 재사용 scratch `previousSolidOwner_`에 복사하고 현재 장애물을 rasterize한다. 셀 단위 `exposedCells`(solid→fluid), `coveredCells`(fluid→solid)를 집계한다.
- **이번과 이전 단계 모두 fluid-fluid였던 MAC face**만 유효한 재구성 샘플로 취급한다. 이번 단계에 막힌 face를 참조하지 않고, 이번에 처음 열린 face끼리 서로 참조하지 않으므로 순회 순서에 의존하지 않는다.
- 재노출 U/V face마다 격자 Manhattan 거리 1~4 이내에서 **가장 가까운 유효 이웃 face의 직전 속도 평균**으로 복원한다. 찾지 못하면 정지 유체(0)로 시작한다. 기존에 계속 유체였던 face는 변경하지 않는다.
- 이어서 움직이는 강체의 no-through 경계 속도를 설정하고 기존 Semi-Lagrangian advection과 Pressure projection을 수행한다.
- `getSolidTransition()`은 열린/닫힌 셀, U/V face 수, 이웃 외삽 및 0 fallback 횟수, `fluidVolumeDelta = (exposedCells-coveredCells) * h² * unitDepth(1m)`를 제공한다. 셀 크기의 정수 배만큼 형태가 그대로 이동하면 이산 점유 체적 변동이 0일 수 있다.
- 기존 `getStepMomentum()`의 `afterObstacles`에는 이 재노출 face 속도 복원도 포함되며, `occupancyDelta()`는 이전처럼 전체 Step의 전체 운동량 증가가 아니라 현재 유효 face 집합에서의 변화다.

**한계:** 이 재구성은 *오래된 강체 경계 속도의 유출을 막는 안정화 조치*이지 cut-cell 체적 보존/보존형 advection이 아니다. 이산 셀 점유 변화로 인한 체적·운동량 오류는 계측될 뿐 실제 유체 질량으로 보정되지 않는다. 움직이는 경계의 면적 가중치, swept-volume flux와 유체/강체 연성 압력 투영이 그다음 단계다.

## Part 7 — Sub-cell 점유율과 기하학적 체적 전이

기존 `solidOwner_`는 셀 중심이 강체 안에 들어가는지로 판정하므로, 직경이 셀 크기보다 작거나 경계가 셀 중간을 지나는 장애물은 실제 점유 면적과 차이가 크다. 다음 압력/이류 연산자 교체에 앞서 **부분 점유 면적과 face 개방 길이**를 별도의 기하 레이어로 계산한다.

- `getCellFluidFraction(x,y)`은 각 셀을 **4×4 균일 sub-cell 샘플**로 나눈 뒤 전체 샘플에서 강체 밖에 있는 비율(0~1)을 반환한다. Circle/Capsule/Convex Polygon의 현재 `ContainsFluidObstacle`를 재사용하므로 회전한 도형과 겹친 도형의 합집합도 동일하게 판정한다.
- `getUFaceOpenFraction(x,y)`, `getVFaceOpenFraction(x,y)`은 각 내부 MAC face의 4개 균일 표본으로 추정한 열린 길이 비율이다. **외곽 no-through wall**은 항상 0이다.
- `getCutCellTransition()`은 직전/현재 샘플 기반 **유체 체적**, `coveredVolume`(이전에 유체였지만 새로 가려진 체적), `releasedVolume`(이전에 가려졌다가 열린 체적), 부분 점유 셀과 부분 개방 face 수를 제공한다. 깊이 1m를 가정해 `volume = Σ(fluidFraction) × h² × 1m`로 계산한다.
- `volumeDelta() = fluidVolume - previousFluidVolume`이며 수치적으로 `releasedVolume - coveredVolume`과 대응한다. 이전 프레임과 같은 모양이면 신규 전이는 0이고, 도형이 격자 안에서 평행 이동하면 양 방향의 국소 체적 변화가 발생할 수 있다.
- `clear()`는 내부 셀을 완전 유체, 내부 face를 개방 상태로 되돌리고 체적 전이 **마지막 Step 진단값**을 비운다. 다음 `step()`은 전체 도메인 유체 체적을 기준으로 새로 점유한 양을 계산한다.

**중요한 설계 경계:** 이번 단계는 *Cut-cell 기하 사전 단계*다. 기존 이진 `solidOwner_`와 압력 Poisson/Jacobi, no-through face 설정, Drag/Pressure 운동량 장부는 **변경하지 않는다**. 부분 점유율을 압력에 임의로 곱하면 대칭 압력 연산자, 무발산 조건, 강체 압력 반력 장부를 동시에 깨뜨릴 수 있기 때문이다. 또한 표본 기반 비율은 정확한 원–사각형 교차 면적 계산이 아니며, `covered/released`는 두 순간의 **공간별 차이**일 뿐 이동 경계가 시간 동안 쓸고 간 실제 swept-volume flux가 아니다. 따라서 *유체 질량·체적 보존이 완성됐다*고 해석하면 안 된다.

이 단계의 회귀 테스트는 축 정렬 Box의 정확한 절반 셀/face와 면적, 1/4 cell 이동 후 covered/released 균형, 셀 중심 마스크가 놓친 작은 원, 회전된 다각형, 장애물 삭제와 초기화를 검증한다. 다음 단계에서 cut-cell flux 및 압력 연산자로 연결할 때 이 참조 값을 활용한다.

## Part 8 — 부분 개방 면의 가중 압력 투영 (선택적 실험 모드)

Part 7의 셀 유체 면적 비율과 MAC face 개방율을 **실제 압력 방정식에 사용하는 별도 Solver 경로**를 추가했다. `fluidField2::setCutCellProjectionEnabled(true)`로 선택하며 기본값은 `false`다. 기존 이진 cell-center 경로와 이전 테스트는 유지되고 Sandbox 바람 장면에서 전환할 수 있다.

- 두 셀의 `cellFluidFraction > 0`이고 face의 `openFraction = a > 0`이면 유체-유체 면으로 취급한다. 그 외에는 고체 경계로 취급한다. 셀 전체를 막지 않는 작은 Circle/Polygon도 압력 계산의 일부가 된다.
- 면 유량은 **`F = a·u_fluid + (1-a)·u_solid`**로 계산한다. 막힌 부분의 속도 `u_solid`는 실제 장애물의 각 face sub-sample에서 `v_COM + ω × r`의 법선 성분을 구해 평균한다. 완전히 막힌 면은 강체 경계 속도를 유지하고, 부분 개방 면의 유체 속도는 압력 보정을 받는다.
- 압력 방정식은 `Σ_neighbours a·(p_neighbour-p_cell) = (h²/dt)·div(F)`다. 공유한 MAC face의 동일한 `a`를 두 셀이 사용하기 때문에 이산 Laplacian의 **대칭적인 가중 edge**를 유지한다. 기존과 동일한 Jacobi 횟수(1~200)를 사용한다.
- 닫힌 도메인에서 강체의 부분 경계 속도를 표본화한 총 순유량이 0이 아닐 수 있다. 압력 Neumann 문제의 정합성을 위해 유효 셀에서 평균 발산을 빼고, **차감한 총 유량 `sourceImbalance`를 공개**한다. 이는 외부에서 실제 질량을 보충했다는 뜻이 아니라 미해결 체적/시간 결합 오차를 드러내는 값이다.
- `getCutCellPressure()`는 압력 전·후 가중 flux 발산 RMS, Jacobi 잔차, sourceImbalance와 활성 유체 셀 수를 반환한다. 이진 `getPressureMomentum()`의 강체/외벽 운동량 장부는 부분 면에 대한 정의가 맞지 않으므로 **실험 모드에서는 초기화**한다.
- 강체 압력 반력은 유체 셀 경계의 개방 면 차이로 추정한 `-p·h·(a_right-a_left, a_top-a_bottom)`을 사용한다. 여러 장애물이 있는 부분 셀의 소유자는 가장 가까운 COM으로 근사한다. 정확한 표면 위치/토크 적분은 후속 과제다.

**범위/한계:** 이 단계는 부분 face의 속도와 압력 연산자를 일관되게 가중하지만 **시간에 따라 변하는 부분 점유 체적 `V(t)`의 `dV/dt` 보존형 소스는 아직 적용하지 않는다**. 현재 Semi-Lagrangian 이류도 flux conservative 방식이 아니며, 압력 반력은 근사이므로 유체+강체의 전체 운동량/질량 보존을 보장하지 않는다. 작은 `cellFluidFraction` 및 단절된 유체 포켓에 대한 수렴 개선, 압력 상수 gauge, 반력 소유자 및 이동 표면의 swept-volume flux는 후속 단계다.

실험 모드에서는 기존 `getDivergenceRms()`가 여전히 **이진 면 속도 발산**을 계산하므로 대신 `getCutCellPressure().divergenceAfter`를 사용해야 한다. 동일하게 기존 `getFluidMomentum()`, `getStepMomentum()`은 이진 fluid-fluid face 기준으로 남아 있으니 cut-cell 전체 운동량 보존의 지표가 아니다.

## Part 9 — 움직이는 Cut-cell의 시간적 연속 방정식

Part 8의 가중 압력 투영은 모든 MAC 면의 개방율을 공간적으로 고려했지만, **장애물이 이동하면서 각 셀의 유체 체적이 변화하는 효과**는 연속 방정식에 없었다. 실험 모드에서 두 프레임의 표본 점유율 차이를 이용한 시간적 소스를 연결한다.

- 각 유체 셀의 면적 (V_i = f_i h²) (깊이 1m)에 대해 `Δf_i = f_i(t) - f_i(t-Δt)`를 계산한다. `∂f_i/∂t ≈ Δf_i/Δt`이고 유체가 차지한 개방 면의 속도만 적분해 **`div(a·u_fluid) + Δf_i/Δt = 0`**을 목표로 한다.
- 기존 Part 8의 `a u_fluid + (1-a)u_solid`을 그대로 둔 채 `Δf_i/Δt`를 더하면, 강체가 차지한 면의 운동학 경계 이동을 **중복 계산**한다. 따라서 이 압력 모드의 유량은 **`a·u_fluid`만** 사용한다. 이미 계산한 강체 속도는 완전히 가려진 face의 운동학 경계 조건/이류 근사에만 남긴다.
- 압력 방정식은 기존과 동일한 대칭 가중 연산자에 `div(a·u_fluid) + Δf_i/Δt`를 우변으로 넣는다. 각 유체 면에는 동일한 개방율 `a`가 양쪽 셀에서 적용된다.
- **첫 유효 Step의 장애물 배치**는 이전 물리 위치가 없는 초기 조건이므로 압력의 `Δf_i/Δt`를 0으로 취급한다. 반면 `getCutCellTransition()`은 초기 배치의 가려진 체적을 그대로 기록한다. `clear()` 뒤에도 시간 이력은 다시 시작한다.
- 현재 `f_i=0`인 셀에는 압력 미지수가 존재하지 않는다. 그 셀에서 소멸한 `Δf_i`는 격자 내 **가장 가까운 활성 유체 셀**에 다중 시작 BFS로 전달한다. 이는 압력계에 시간을 따라 이동한 체적원을 잃지 않기 위한 근사이며 실제 이동 경로의 swept-volume 면 플럭스가 아니다. 활성 유체 셀이 전혀 없으면 `unresolvedVolumeRate`로 별도 집계한다.
- 닫힌 외벽 또는 서로 단절된 유체 포켓에서는 각 연결 영역의 전체 소스가 0이 아니면 Neumann 압력 문제가 성립하지 않는다. **개방된 face를 따라 연결된 영역별** 우변 평균을 빼서 정합 가능한 성분을 투영하고, 각 영역별 제거량의 절댓값 합을 `compatibilityCorrection`으로 보고한다. 전체 부호 있는 불일치는 `sourceImbalance`다.
- `fluidCutCellPressure2`의 `divergenceBefore/After`는 이제 `div(a·u_fluid) + Δf_i/Δt`의 RMS(1/s)이고, `volumeRate`·`redistributedVolumeRate`·`unresolvedVolumeRate`는 m³/s 단위, `connectedRegions`와 `hasVolumeHistory`도 제공한다. `pressureResidual`은 정합성 보정을 적용한 가중 Poisson의 반복 잔차이므로, 실제 원래 연속 방정식의 미해결 부분은 `divergenceAfter`·`compatibilityCorrection`에서 별도로 판단한다.

**한계:** 이 구현은 두 공간 표본 사이의 `ΔV/Δt`를 이용한 **시간 이산 연속 방정식 근사**이다. 강체의 실제 경로 및 회전 중 쓸고 간 체적을 따라 face별 flux를 계산하지 않는다. 부분 점유율은 4×4 표본이므로 프레임 간 aliasing으로 전체 유체 체적의 비보존이 발생할 수 있고, 닫힌 도메인의 정합 보정이 이를 물리적으로 복구하지는 않는다. 현재 이류는 여전히 Semi-Lagrangian이며, 정확한 강체 압력 반력과 에너지·운동량의 전체 보존도 해결되지 않았다. **보존형 advection, swept interface flux 및 유체-강체 반력 장부를 다음 단계에서 별도로 구현**한다. 이진 Solver가 기본이고 본 경로는 실험 기능으로 남는다.

## Part 10 — MAC 공유 면의 Swept Solid Flux 시간 적분

Part 9에서는 두 프레임의 유체 셀 점유율 차이 `Δf`를 압력 연속 방정식의 소스로 사용했다. 이 접근만으로는 강체가 **그 사이 어떤 MAC 면을 가로질러 이동하거나 회전했는지**를 알 수 없다. 실험 Cut-cell 모드에서 Shape의 직전/현재 world transform으로 그 경로를 추정해 **공유 MAC face를 통과한 강체의 부호 있는 체적**을 시간 적분한다.

- `fluidField2`는 각 유효 `step()` 종료 시 Shape snapshot을 저장한다. 다음 Step에서 장애물 배열의 순서·geometry가 유지되고 시작/끝 Shape가 **모두 도메인 내부**이면, `transform.position`을 선형 보간하고 회전은 최단 각도 차이를 따라 보간한다. Center of Mass의 `linearVelocity`/ `angularVelocity`가 아니라 **실제 transform 변위**를 `Δt`로 나눈 강체 운동학 속도를 사용한다.
- 강체의 이동 거리와 각도 × 형상 반경 중 더 큰 이동 척도로 시간 샘플 수를 정한다. 한 번에 약 `h/4` 이하를 이동하도록 구간 수를 증가시키되, 최대 32구간으로 제한한다. 각 시점에서 공유 MAC U/V face의 4개 길이 표본마다 `ContainsFluidObstacle()`로 solid 점유를 판정한다.
- 통과 체적 근사는 **`Q^solid_face ≈ ∫_step ∫_face χ_solid(x,t) · [v_origin + ω×(x-origin)]·n dA dt`**다. 단위 깊이 1m를 가정한 `m³`이며, 모든 내부 face의 일관된 단일 부호 값을 양쪽 셀이 공유한다. 겹치는 다중 Shape는 기존과 마찬가지로 먼저 포함한 obstacle owner를 사용한다.
- 각 셀에는 이웃 4개 공유 면의 **`(Q_right-Q_left+Q_top-Q_bottom)/h²`**를 유체 면적 비율 변화에 대응하는 적분 소스로 전달한다. 외부 wall에는 swept face 유량을 통과시키지 않으며, 부분 유체만 압력을 푼다. 완전히 가려진 셀의 소스 전달·연결 영역별 Neumann 정합 보정은 Part 9 방식을 유지한다.
- Shape가 생성·삭제되거나 geometry가 바뀌거나 외벽에 겹치거나 넘어가거나 첫 snapshot이면 **기존 `Δf` 방법으로 fallback**한다. 외벽을 통과하는 강체를 내부 유량으로 잘못 상쇄시키지 않기 위함이다. 동일 형상의 장애물은 배열 인덱스가 Step마다 유지된다는 계약이 필요하다.
- `getSweptFaceFlux()`는 실제 swept 경로 사용 여부, 시간 샘플 수, 모든 face의 **절댓값 통과량 합**(같은 물체가 여러 face를 지나면 여러 번 집계), **경로 적분 결과와 snapshot `Δf·h²`의 셀별 절댓값 차이 합**을 제공한다. `getCutCellPressure().volumeRate`는 여전히 snapshot 유체 체적 총량 변동을 뜻하지만, 실제 압력 우변에 쓰이는 **지역 소스**는 경로가 유효한 경우 위 공유 면 적분에서 나온다.
- 회귀 테스트는 최초 배치, 빠른 평행 이동, 회전만 하는 얇은 Polygon, 정지 Shape, 기하 변경에 따른 fallback, 외벽 통과 fallback, `clear()`를 확인한다.

**주의:** 이는 공간–시간 표본 적분을 이용한 **MAC 내부 공유 면의 Swept Solid Flux 근사**이며 정확한 연속 도형의 교차 면적이나 진정한 4D swept interface reconstruction이 아니다. 시간 샘플 수 상한과 4개 face 표본에 따른 aliasing이 존재하며, 특히 완전히 한 셀 안에서 움직여 **MAC 면을 지나지 않는 작은 장애물**은 여전히 식별하기 어렵다. 도메인 벽과 Shape 추가·삭제는 경로 적분에서 제외한다. 현재 **Semi-Lagrangian 이류는 보존형으로 전환하지 않았고**, fluid/solid pressure reaction도 단순 면적 차이 근사다. 외벽/연결된 셀의 합 보존이 곧 실제 유체 질량·운동량 보존을 의미하지 않는다.

## Part 11 — Flux-form MAC dual-face 이류와 Sandbox 비교

Part 1~10의 속도 이류는 MAC face를 이전 위치로 역추적해 샘플링하는 Semi-Lagrangian 방식이었다. 안정적이지만, **내부 face 운동량이 인접한 face로 얼마씩 교환되었는지**에 대한 보존적인 pair flux가 없었다. 기본 이류 방식은 바꾸지 않고 `setConservativeAdvectionEnabled(true)`로 **1차 upwind flux-form MAC 이류**를 선택하도록 했다.

- MAC U 내부 face마다 `h×h`의 U dual control volume, V face마다 V dual control volume을 둔다. 저장한 `u`, `v`는 각각의 수송 성분이고 MAC 속도를 보간해 **dual edge의 실제 이류 속도**를 구한다. 예를 들어 U dual의 x 방향 전송 속도는 인접 두 U 속도의 평균, y 방향은 상단 V 두 face의 평균이다. V dual도 x·y를 서로 바꾸어 대칭적으로 구성한다.
- 인접한 두 dual control volume이 공유하는 edge에는 **동일한 부호의 upwind flux** `a_edge * transportSpeed * (speed >= 0 ? leftValue : rightValue)`를 적용한다. `a_edge = min(a_face1, a_face2)`이며 `a_face`는 실험 Cut-cell 모드의 기하 개방율, 기존 모드에서는 0/1 이진 유체 face다. **현재 형상에서 완전히 막힌 face는 계산에서 제외**한다.
- 각 face의 유효 질량을 `rho * h² * a_face` (rho=1, unit depth)로 근사하고, `newVelocity = velocity - dt/h * (outgoingFlux - incomingFlux) / a_face`로 갱신한다. 양쪽 face에서 공통 내부 flux를 사용하므로 **고정된 현재 개방율을 가진 닫힌 dual grid에서 유효 face 운동량 합이 수치 오차 내 보존**된다. 외곽 dual domain의 flux는 0이다. 이는 물리적 원 도메인의 정확한 cut volume 질량이 아니다.
- 각 Step 시작 시 활성 U/V 면의 최대 속도에서 `maxCfl = speedMax * dt / h`를 계산하고, 목표 `CFL <= 0.45`에 맞춰 최대 64회 균등 substep으로 나누어 수행한다. 상한을 넘어가면 `cflLimited` 진단이 true가 되며 이 경우 안정성을 보장하지 않는다.
- `getAdvectionStats()`는 압력 투영 직전의 **이류 단계만** 평가한다. `before`, `after`는 가중 dual face 운동량(N·s), `delta()`는 이류로 인한 변화, `conservative`, `maxCfl`, `substeps`, `cflLimited`는 선택한 알고리즘의 실제 실행 정보다. 기존 Semi-Lagrangian 모드에서도 동일한 면 운동량 합을 구해 비교할 수 있다. `clear()`는 진단값을 초기화하되 모드 선택은 유지한다.
- **Sandbox 사용법:** `유체 / 2D 바람·유체 속도장`에서 **보존형 MAC 이류 (실험)**를 토글한다. **이류 비교 프리셋 (와류 + 외력/강체 OFF)** 버튼은 유체/강체를 리셋하고 8방향 원형 접선 속도 와류를 주입하며 바람 소스·Drag·양방향 강체 연성을 끈다. 재생/일시정지하면서 속도 화살표를 보고, 전·후 운동량/CFL을 비교한다. **현재 흐름에 와류 추가** 버튼으로 실행 중 반복 주입도 가능하다. 기존 강체 연성을 다시 켜면 Circle/Capsule/Polygon과 Cut-cell 조합을 비교할 수 있다.
- 핵심 회귀: 동일 초기 속도의 Semi-Lagrangian/보존 이류 시작 운동량 비교, 빈 격자에서 CFL 다중 substep 보존, 정적 Circle의 부분 면 가중 운동량 수지, 두 모드 토글과 초기화, Sandbox 와류 프리셋·강체 실험.

**한계:** 이 단계의 보존량은 **MAC staggered dual face에 정의한 가중 운동량**이며 유체 전체의 정확한 운동량·질량 보존이 아니다. 현재 점유 면적을 이류 중 고정시키므로 강체가 움직여 생긴 swept interface의 유체 질량/운동량 수송과 같은 공간–시간 격자를 공유하지 않는다. 압력 투영, 이산 마스크 전이, drag, 실제 외벽 반력 및 Semi-Lagrangian 모드에는 이 보존 보장이 적용되지 않는다. 1차 upwind의 수치 점성, dual 격자 외곽 0-flux 근사, 작은 부분 개방 면, 매우 큰 CFL(64 substep 상한)에서의 오차가 남는다. 추후에는 **conservative volume/mass tracer, 실제 Cut-cell geometry의 dual mass, swept flux와 유체 이류 통합, 표면 pressure reaction 수지**를 진행한다.

## Part 12a — 보존형 수동 tracer 질량 수송

Part 11의 가중 MAC face 운동량과는 별개로, **셀 중심 수동 tracer 질량**을 추적한다. 이는 움직이는 강체와의 완전한 물질 수송까지 해결한 Part 12 전체가 아니다.

- `setTracerConcentration(x,y,c)`는 농도 `c` (kg/m³, 단위 깊이 1m)를 현재 셀의 유체 면적 비율과 `h²`로 곱해 질량으로 저장한다. 비활성 상태에서 기존 속도·압력 경로의 계산은 변경하지 않는다.
- 압력 투영 후의 MAC U/V 속도와 현재 면 개방율에 **1차 upwind shared-face flux**를 적용한다. 한 면에서 뺀 양을 반대편에 더하므로 닫힌 격자 전체 질량의 합이 보존된다.
- CFL 0.45 목표, 최대 64 substep. 작은 cut-cell과 큰 CFL에서 각 donor의 합계 유출을 보유 질량으로 제한해 음수 물질량을 방지한다. 이 제한은 국소 정확도를 낮출 수 있다.
- `getTracerConcentration()`, `getTracerMass()`, `getTracerStats()`가 현재 농도, 총 보관 질량, 이번 Step의 before/after 질량, 갇힌 질량, 최대 CFL과 substeps를 보고한다. `clear()`는 tracer를 비운다.
- 테스트는 빈 폐쇄 격자/장시간 수송, 고 CFL, static fractional cut-cell, 비정상 입력, moving obstacle의 완전 점유와 재노출을 포함한다.
- Sandbox **유체 / 2D 바람·유체 속도장**에서 **수동 추적 물질 주입** 버튼으로 두 셀에 초기 tracer를 배치한다. 패널의 총 질량·막힌 셀 보관 질량·수송 전후 변화·CFL/substep을 확인하며, **흐름+Body 초기화** 또는 이류 비교 프리셋은 tracer를 지운다. 현재 시각화는 숫자 진단이며 tracer 컬러 렌더링은 별도 과제다.

**한계:** 움직이는 장애물이 완전히 점유한 셀에는 tracer 질량을 삭제하지 않고 남겨서 `trappedMass`로 별도 표시한다. 이 질량은 다시 열린 셀에서 나타나므로 **swept interface 물질 수송을 구현했다고 볼 수 없다**. 또한 현재 cut-cell 비율은 표본 근사이며 tracer 질량 합계가 실제 유체 총 질량·운동량을 의미하지 않는다. Moving swept-flux와 물질 수송의 결합, 정확한 dual mass, 압력/강체 운동량 장부는 후속 단계로 남긴다.

## Part 12b — Swept Solid 방향 기반 tracer 재배치 (선택형)

Part 12a에서는 움직이는 Solid가 부분 점유율을 낮추거나 셀을 완전히 막아도 tracer 저장 질량을 그대로 두었다. Part 12b에서는 **유효한 연속 Shape 이동 경로가 있을 때만** 감소한 유체 공간의 tracer 일부를 다른 열린 셀로 옮기는 근사 보정을 추가한다.

- `setSweptTracerRemapEnabled(true)`로 명시적으로 활성화한다(기본 false). 실험 Cut-cell 압력 모드와 Part 10의 유효한 Swept Solid Flux 이력이 필요하다. 기존 수동 tracer 수송과 압력 Solver는 유지한다.
- 각 셀에서 `oldFraction > newFraction >= 0`인 경우 `displacedMass = oldTracerMass * (oldFraction-newFraction)/oldFraction`을 계산한다. 공유 MAC 면의 기존 **부호 있는 swept solid flux가 셀에서 바깥쪽을 가리키면**, 현재 유체가 남아 있는 인접 셀로 해당 질량을 방향 가중 분배한다. 이동한 질량을 donor에서 빼는 동시에 recipient에 같은 양을 더한다.
- 방향성 있는 인접 목적지가 없으면 완전히 막힌 셀에서는 Part 9의 **가장 가까운 활성 유체 셀**, 일부 열린 셀에서는 새로 열린 인접 유체 셀을 사용한다. 대상이 전혀 없으면 이동시키지 않고 `unresolvedDisplacementMass`로 기록한다.
- `getTracerStats()`에 `usedSweptRemap`, `sweptRemappedMass`, `fallbackRemappedMass`, `unresolvedDisplacementMass`가 추가됐다. 기존 `trappedMass`는 remap과 tracer 이류 후 완전히 막힌 셀에 남은 질량이다.
- Sandbox Wind Flow에서 **이동 강체의 tracer 재배치 (실험)**을 켜면 기존 수동 tracer 주입과 함께 재배치·미해결량을 확인할 수 있다. Core 회귀에서 이동하는 Box, 회전만 하는 Box, 최초 배치/기하 변경/정지, 기본값 호환 및 총 질량 보존을 검사한다.

**수치적 한계:** Swept flux는 강체의 이동을 표본화한 **방향 근거**일 뿐, 유체-고체 경계의 정확한 물질 유량은 아니다. 셀의 감소 면적은 별도의 시작/끝 4×4 표본에서 추정하고, 직접 인접하지 않은 fluid 셀로 fallback하면 경로를 건너뛰는 인공적인 이동이 생길 수 있다. 빠른 이동/회전, 작은 Cut-cell, 다중 Shape의 정확한 질량·체적 대응은 보장하지 않는다. **총 tracer 질량의 대수적 보존과 실제 물리적 이동 경계 수송의 정확도는 다르다.** 장차 표면 교차 및 공간–시간 체적 재구성이 필요하다.

## Part 12c — Swept face 통과 체적 기반 tracer 질량 이송

Part 12b는 `sweptU/VVolume`을 이송 **방향의 가중치**로만 사용해, 조금만 열린 공유 면에서도 셀의 모든 감소 체적 질량을 직접 이동시킬 수 있었다. 선택적 `setSweptTracerRemapEnabled(true)` 모드에서 실제 표본화된 **부호 있는 통과 체적**을 질량 이송량 계산에 사용한다.

- 직전 셀 유체 비율 `f_old`과 저장된 tracer 질량 `M_old`으로 이전 농도 `c_old = M_old / (f_old h²)`를 계산한다(단위 깊이 1m). 감소한 유체 공간에 속했던 질량의 상한은 `M_displaced = M_old (f_old-f_new)/f_old`다.
- 셀의 좌·우·하·상 면에 기록된 swept solid 체적 `Q`에 바깥 방향 부호를 맞춘 뒤 `max(0,Q_out) c_old`를 **각 이웃으로 보낼 요청 질량**으로 계산한다. 이웃 셀의 현재 유체 비율이 0이면 해당 면을 사용하지 않는다.
- 모든 요청량의 합이 `M_displaced`를 초과하면 공통 비율로 제한한다. 출발 셀에는 보낼 양을 차감하고 해당 이웃 셀에는 같은 양을 가산하므로 질량은 보존되며, 공유 면별 이송량이 상대 통과 체적 비율에 따른다.
- 요청량이 감소 체적 질량보다 적으면 남은 질량은 **완전 차폐 셀**에서 최근접 열린 셀로, **부분 개방 셀**에서 새로 열린 인접 셀이 있는 경우에만 보조적으로 이동한다. 적절한 목적지가 없으면 원래 셀에 보관하고 `unresolvedDisplacementMass`로 남긴다.
- `getTracerStats()`의 `sweptRequestedMass`는 면별 통과 체적 × 이전 농도의 총 요청 질량, `sweptLimitedMass`는 감소 체적 상한으로 잘라낸 질량이다. 기존 `sweptRemappedMass`, `fallbackRemappedMass`, `trappedMass`와 분리해 Sandbox Wind Flow에서 확인한다. 테스트는 요청·제한·직접 이송·보조 이송·미해결량을 하나의 수지로 검사한다.

**한계:** `sweptU/VVolume`은 완전한 시간 연속 cut-cell 표면의 체적 변화가 아니라 **MAC 면 4표본 × 최대 32 시간 구간 적분**이다. 시작·끝 셀 유체 비율은 또 다른 4×4 표본이므로 이산 기하학적 불일치가 있고, 위 질량 수송은 이를 완전히 복구하지 않는다. 이 단계는 tracer의 대수적 총질량 보존과 swept 체적 기반 이송 **정량화**를 개선하지만, 압력-강체 표면 반력이나 정확한 전체 유체 질량·운동량 보존을 뜻하지 않는다.

## Part 12d — 공유 MAC 면의 국소 Swept 체적 일치 보정

기존 Part 10의 swept 면 적분은 시간·공간 표본화 방식이므로, 각 셀에 누적된 면 체적의 **signed divergence**가 시작/끝 4×4 cut-cell 표본에서 구한 유체 공간 변화와 다를 수 있다. Part 12d에서는 해당 불일치를 억지로 없애거나 전역에 확산하는 대신, **서로 부호가 반대인 인접 셀의 잔차만** 공유 face에서 상쇄하는 실험 기능을 추가했다.

- `setSweptGeometryCorrectionEnabled(true)`는 기본 false이며 Cut-cell 모드와 유효한 연속 Shape swept 이력을 함께 요구한다. 첫 snapshot, 기하 변화, 외벽 교차 등 불확실한 경로는 기존 fallback으로 처리한다.
- 셀 `i`의 discrepancy 체적은 `e_i = (f_new - f_old) h² - (Q_right - Q_left + Q_top - Q_bottom)`이다. 면 오른쪽/위쪽의 두 셀에 `e_a > 0`, `e_b < 0`이라면 공유 면 체적에 `min(e_a, -e_b)`를 더하고, 반대 부호라면 그만큼 뺀다. 이렇게 양쪽 셀의 불일치가 동시에 줄어든다.
- 기존 sampled swept flux가 있거나, 두 셀 중 하나의 기하 점유율이 실제로 변한 공유 면만 사용한다. 외벽 face는 수정하지 않으며, 기존에 통과 방향이 기록된 유량을 반대로 뒤집지 않는다. 최대 4회 순회하고 **인접하지 않은 오차**나 전체 체적의 비영합은 고치지 않는다.
- `getSweptFaceFlux()`의 `rawGeometryMismatch`는 보정 전 L1 체적 오차, `geometryMismatch`는 압력 소스에 사용할 최종 체적 오차다. `correctionAbsoluteFlux`는 공유 면을 수정한 체적 절댓값 누적치(같은 면 반복 가능), `correctedFaces`는 수정 **횟수**다.
- 보정된 `sweptUVolume_/sweptVVolume_`은 이미 구현된 Cut-cell 연속방정식 소스와 Part 12c의 tracer 체적 기반 이송에 공통으로 전달된다. 폐쇄 도메인에서 쌍으로 교환한 체적은 합계가 0이므로 전체 기하적 불일치를 은폐하지 않는다.

**한계:** 이는 두 가지 서로 다른 격자 표본 간 *불일치 완화*이며 실제 움직이는 연속 계면의 정확한 체적/압력 반력 해석이 아니다. 부호가 반대인 인접 오차가 없는 경우에는 차이가 그대로 남는다. 강체 압력 반력과 함께 전체 유체 질량/운동량 보존이 완성되었다는 의미는 아니다.

## Part 13a — Sandbox 강체·Fluid Substep 동기화 + 이진 MAC 압력-강체 연립 투영 (실험)

원래 Sandbox는 한 외부 프레임에서 `fluidField2::step(dt)`를 1회 호출해 Pressure/Drag를 강체에 가한 뒤 `World::Step(dt, subStepCount)`를 호출했다. Fluid가 사용한 강체 경계 형상과 속도는 World 하위 Step 동안 갱신되지 않아, 빠르게 움직이는 강체에서는 압력 피드백이 지연될 수 있었다.

- `setWindSynchronizedCouplingEnabled(true)`를 Wind Flow Sandbox에서 기본으로 사용한다. **강체·유체 양방향 ON**, `dt > 0`, `subStepCount > 1`일 때만 `subDt = dt/subStepCount`로 나눈다. 매 하위 구간에 `World`의 현재 강체 Pose/속도를 읽고 `Fluid::step(subDt, obstacles, feedback)` → 압력 반력·Drag 적용 → `World::Step(subDt, 1)`을 반복한다.
- 원래의 `World::Step(dt, subStepCount)`와 단일 Fluid Step 경로는 동기화 옵션 OFF 또는 단방향일 때 유지한다. Sandbox UI에서 토글하고 실제 실행한 Fluid 구간 수를 확인할 수 있다.
- 동기화 모드의 `getWindPressureForces()`는 **외부 Step 동안 적분한 압력 충격량을 원래 `dt`로 나눈 평균 힘·평균 토크**를 나타낸다. `getWindDragBodyImpulse()`와 `getWindDragFluidImpulse()`는 하위 Step 충격량의 **합**을 유지한다.
- `fluidField2::getPressureMomentum()`과 `getCutCellPressure()`는 **마지막 Fluid 하위 Step**의 진단값이므로, 외부 Step 전체의 평균 압력 반력과 직접 대조하면 안 된다. 이전 단일 Step pressure-ledger 회귀는 동기화 옵션 OFF로 검사하며, 장시간 안정성·양방향·단방향 호환 회귀를 추가한다.

- `fluidShapeObstacle2` 끝에 `inverseMass`·`inverseInertia`를 기본 0으로 추가했다. 이진 MAC 투영에서 양방향 동기화 옵션이 ON이면 `subStepCount=1`에서도 Sandbox가 실제 Dynamic Body 질량·관성의 역수를 설정하고, 유체 자유 face 압력 기울기와 고체 경계 압력 반력이 같은 SPD Schur complement(`A = L + B M_body^-1 B^T`)에 들어간다. 이 압력은 preconditioned CG로 반복 계산하고 기존 pressure face force/torque 그대로 World에 전달한다. 기본 0인 독립 Fluid API 및 기존 Jacobi 경로는 유지된다.
- **한계:** 아직 Cut-cell 가중 투영에는 강체 유효질량 연립항이 적용되지 않는다. Cut-cell 모드에서는 안정성이 검증되지 않은 명시적 다중 반력 주입을 피하기 위해 동기화 옵션이 ON이어도 기존 단일 Fluid Step + World Substep 경로를 유지한다. 단일 압력 gauge, 폐쇄된 단절 영역 정합, 정확한 다중 Shape force ownership, 점성/Drag 합동 투영도 미완성이다. 이 새로운 경로가 추가질량 불안정성 완전 해결이라는 뜻은 아니며, 장시간 압력-운동량·에너지 회귀로 증명해야 한다.

**안전 게이트:** 위의 동기화 실행은 **이진 MAC 압력 모드에서만** 사용한다. Cut-cell 가중 압력 모드는 아직 암시적 Body 유효질량 항을 포함하지 않으므로, 원래 단일 호출 방식으로 남겨둔다. Cut-cell의 안정성은 별도 결합 압력 구현·검증이 필요하다.

**범위와 한계:** 이 변경은 **Sandbox의 시간 배치**만 변경하고 `World`의 충돌/Joint solver, `fluidField2`의 압력 행렬, 접촉 계산과 대수적 운동량 교환은 변경하지 않는다. `World::Step(dt/N, 1)` N회는 `World::Step(dt, N)` 1회와 Solver 관점에서 완전히 동일하지 않을 수 있다. 시간 동기화는 프레임 지연과 큰 장애물 점유 변화는 줄이지만 **추가질량(added-mass) 불안정성, coarse-grid 압력 반력 스파이크, Cut-cell 표면 적분 오차를 자체적으로 해결하지 않는다.** 장기적으로는 World/Fluid를 함께 호출하는 공통 Physics Scheduler와 강체 유효 질량이 포함된 implicit 압력 결합이 필요하다.

## Part 13b — Fluid 계산 횟수 분리, 가변 Field, Static 장애물

- **비용 분리:** World의 `subStepCount`는 Contact/Joint 정확도에 사용하고 Fluid 압력·기하 갱신·이류 횟수는 `setWindFluidSubstepCount(n)`(기본 1회, 1~32)로 별도로 설정한다. 이진 MAC 질량 결합이 활성화되면 실제 Fluid 횟수는 `min(n, worldSubSteps)`이며 한 Fluid 구간마다 여러 World substep을 묶어 `World::Step(groupDt, groupSubsteps)`를 수행한다. World=8 / Fluid=1이라면 PCG 압력은 1회, World는 8 substep이다. World=8 / Fluid=2라면 PCG 2회, World 4+4 substep이다. 불균등 분할은 World substep당 dt를 동일하게 유지해 그룹별 시간으로 합산한다.
- **절충:** Fluid 계산 횟수 1은 연산 비용을 줄이지만 강체 움직임과 Fluid 경계의 재샘플링 간격은 늘린다. 결과 정확도가 World substeps에만 의존하지 않으므로 매우 빠른 장애물·강한 유동·격자 이동이 큰 경우 Fluid 횟수를 늘리거나 시간 간격을 줄여 비교해야 한다. Cut-cell 실험 모드에서는 아직 강체 질량 결합이 없어 기존 Fluid 1회 경로를 유지한다. 기존 동기화 옵션을 끈 단방향/레거시 경로도 Fluid 1회다.
- **Field 크기:** Sandbox의 가로/세로 셀 수(각 8~128), 셀 크기(0.25~2m)를 편집하고 적용하면 원점 기준 중앙에 맞춘 새 `fluidField2`를 생성한다. 크기 변경은 기존 Fluid 속도·압력·tracer 해를 초기화하고 World의 Static 장애물 및 모드 설정은 유지한다. 소스 위치는 변경된 Field 왼쪽 상대 위치에서 계산한다. 실제 물리 격자 크기는 `width * height`와 `cellSize`가 함께 결정하며, 셀 수/압력 반복/장애물 수가 비용을 지배한다.
- **Static 장애물:** Sandbox에서 Static Box 또는 Circle을 좌표·반크기·반경으로 최대 24개 생성하고 Inspector에서 위치·회전을 조정하며 삭제할 수 있다. 이는 별도 그래픽 표시용 물체가 아닌 실제 `World::CreateBody(Static)`과 `CreateShape`로 생성되므로 강체 충돌에도 참여한다. Fluid는 기존 동적 3개에 추가한 모든 Static을 장애물 스냅샷으로 받고 Static의 역질량은 0이다. **Dynamic 양방향 옵션을 끄더라도 Static은 유체를 막는다.**
- 기존 `getWindPressureForces()`는 Dynamic 강체 세 개의 진단을 유지한다. 나머지 Static 경계 반력은 `fluidField2`가 내부적으로 계산하지만 현재 Sandbox UI에는 별도 배열로 제공하지 않는다.
- **검증:** Fluid 1/2/3회와 World 8/3 substep 분리, 기존 결합 압력 사용, Field 재구축 후 옵션과 Static의 유지, 장애물의 점유 갱신 및 삭제, 단방향에서도 Static 유체 경계 유지 등을 GUI 없는 Sandbox 테스트로 검사한다.

## Part 13c — 결합 PCG 수렴 판정과 실패 경로

- `fluidPressureMomentum2::coupledStatus`는 `notRequested`, `converged`, `noBoundary`, `iterationLimit`, `numericalBreakdown`, `invalidPressure` 중 하나다. `coupledIterations`·`coupledIterationLimit`과 초기/최종 RHS 잔차 RMS 및 상대 잔차를 함께 제공한다. 이는 **보정된 연립방정식 `A*p=b`의 잔차**이며 실제 Fluid 발산 RMS나 World/Fluid 전체 에너지 오차가 아니다.
- PCG의 반복 잔차만 신뢰하지 않고 종료 시 `b-A*p`를 재계산한다. 목표는 상대 제곱 잔차 `1e-10`(상대 노름 `1e-5`) 또는 절대 제곱 `1e-18`이고, 반복 횟수 한도에 도달한 경우 실제 잔차로 수렴 여부를 판정한다. `coupledStatus=converged`인 경우에만 `usedCoupledProjection=true`다.
- 경계 면 자체가 없다면 `noBoundary`로 기록하고 기존 Jacobi 경로를 사용한다. 하지만 **실제 결합 Solver가 수렴하지 않거나 수치 실패하면** 질량을 무시하는 Jacobi 반력을 Dynamic Body에 주입하지 않도록 그 Step의 압력을 0으로 두고 반력을 생략하며 `coupledSolveFailed=true`로 기록한다. 이는 폭주 위험을 낮추기 위한 **오류 격리 정책**일 뿐, 해당 Step의 비압축성을 만족시키거나 물리적 안정성을 보장하는 해결책이 아니다.
- Sandbox는 PCG 상태·반복 횟수·실제 잔차를 표시한다. 테스트는 1회 반복으로 강제한 미수렴이 더 이상 성공이 아님을 확인하고, 200회 이내 정상 수렴·경계 없는 Dynamic Body·진단 초기화·120프레임 연성 안정성의 실제 수렴을 검증한다.

## Part 13d — 결합 압력의 MAC 경계 속도 동기화

- PCG 압력 Solver는 내부 fluid-fluid face를 `-dt/rho * grad(p)`로 보정하며, 강체에는 동일 압력의 face traction `F, τ`를 반환한다. 이전 구현은 이후에도 solid에 접한 MAC face에서 **압력 적용 전** 강체 법선 속도를 유지했다.
- 이번 단계는 **수렴한** 이진 MAC 결합 압력의 traction으로 예상되는 강체 속도 변화 `Δv = dt·inverseMass·F`, `Δω = dt·inverseInertia·τ`를 적용한 `v_surface = v_initial + Δv + (ω_initial + Δω)×(point−COM)`를 solid face에 다시 기록한다. 내부 fluid-fluid face의 투영 결과와 압력 행렬은 바꾸지 않는다. 완전히 solid에 가려진 내부 face도 기존과 마찬가지로 지정되지만 진단 개수에는 fluid-solid 경계 면만 포함한다.
- `getPressureMomentum().synchronizedBoundaryFaces`와 `boundaryNormalChangeRms`는 **마지막 Fluid Step**에서 압력 반력으로 재설정한 경계 법선 face 개수와 기존 face 속도에서 예측 face 속도로의 평균 변경량(m/s)이다. 이 값은 오류량이나 전체 World–Fluid 운동량·에너지 잔차가 아니다.
- World에서는 이어서 `World::Step`을 실행한다. **Drag·중력·접촉·Joint·감쇠**와 강체 위치 적분으로 실제 다음 구간 속도와 형상이 달라질 수 있으므로, 그 상태는 다음 Fluid 호출에서 다시 샘플링한다. 단일 Fluid 구간(Fluid 1회/World 8 Substep) 사이에는 여전히 지연이 남는다. 정확한 전 과정 연립 통합으로 간주하지 않는다.
- 정상 PCG에서는 이동·회전 Polygon의 모든 이진 fluid-solid face의 법선 속도를 독립적으로 검사하고, 실패 PCG에서는 적용 전 경계 속도가 유지되는지 검사한다. Sandbox에서는 Drag/중력/충돌이 없는 단일 Step의 World 속도 증분이 실제 Fluid 압력 반력 `dt·M^-1·F, dt·I^-1·τ`와 맞는지 테스트한다. Cut-cell 경로 및 PCG 실패 경로에는 이번 동기화가 적용되지 않는다.

## Part 13e — 유체·강체 압력 단계의 운동 에너지/운동량 정량 감사

- `getFluidKineticEnergy()`는 이진 MAC에서 유체 셀 두 개 사이에 있는 U/V face에 각각 유효 질량 `h² kg`을 부여해 `0.5*h²*(sum(u²)+sum(v²))`를 합한 **부분 운동 에너지(J)**다. 고체에 인접한 MAC face, 외벽, 완전한 moving cut-cell dual volume은 제외한다. 앞서 사용한 `getFluidMomentum()`과 동일한 face 선택으로 비교할 수 있다.
- `fluidPressureMomentum2::energy`는 **압력 투영 직전과 직후**의 자유 face 에너지 및 입력 `fluidShapeObstacle2`의 역질량/역관성과 압력 반력 `F, τ`로 예측한 강체 운동 에너지를 구분한다. 압력으로 예측한 `v' = v + dt * invMass * F`, `ω' = ω + dt * invInertia * τ`를 사용한다. 정적 장애물 및 역질량 0의 운동학적 강체는 강체 운동 에너지 장부에서 제외한다. 압력 해 실패 시에는 해당 Step의 반력이 0이므로 강체 예측 에너지는 바뀌지 않는다.
- **중요:** `energy.totalDelta()`는 *부분 장부의 압력 단계 에너지 변화*이지 **전체 유체–강체 시스템의 총 에너지 보존 증명**이 아니다. 이류·변형 경계에서 바뀐 유체 자유 face 선택, Drag, 외부 source, 벽으로 전달된 일, missing interface dual volumes를 포함하지 않는다. 양수 변화만으로 에너지가 생성됐다고 단정해서는 안 된다.
- GUI 없이 **3개 질량(0.25/2/12kg)·3개 회전 관성(0.08/1/8kg·m²) 중 대응한 3쌍·2개 해상도(h=0.5/0.25m)**를 36회 연속 압력 교환하며 자유 MAC face 에너지, 압력 예상 강체 에너지, discrete fluid+solid+wall 운동량 잔차, 최대/마지막 에너지와 양의 압력 단계 에너지 변화 누적량을 검증·출력한다. 이 실험은 geometry를 고정한 solver-only 시나리오로 swept solid 체적 보존의 증명은 아니다.
- Sandbox에서는 wind source/Drag/gravity 없는 World8/Fluid1의 60프레임 실제 강체 이동·회전을 반복하고 강체/Fluid 부분 운동 에너지를 검사한다. World 자체 속도에서 독립적으로 계산한 강체 운동 에너지가 Fluid의 압력 예측값과 일치하는지도 별도 검증한다. 현 단계의 폭주 안전성 회귀는 **느슨한 상한**이며, 이를 전체 에너지 보존의 성립으로 해석하지 않는다.

## Part 13f — 회전 표면 속도의 4점 Drag (Sandbox)

- 중심 속도만으로 평가하던 경험적 Drag는 순수 회전의 저항 토크가 0이었다. 현재는 AABB 좌/우/상/하 4개 표면 근사점에서 `v_surface=v_COM+ω×r`를 사용하고, 바깥 한 셀의 유체 속도를 참조해 각 `dragCoefficient/4`의 힘을 계산한다.
- 모든 대상의 상대 속도를 먼저 샘플링하여 다른 강체 순회·유체 impulse 적용 순서로 유량 읽기가 달라지지 않게 한다. 양방향에서는 유체에 실제 전달한 선형 충격량의 반대값만 `World::ApplyForce(body,force,point)`에 넣는다. 강체의 토크는 이 작용점에서 계산되며, 현재 교환 장부는 **선운동량만** 계측한다.
- 회귀 검증은 소스 없는 순수 회전 Box의 감쇠, 대칭력의 COM 이동 없음, 양방향 Drag 선운동량 잔차를 비교한다.

**한계:** 표면은 실제 Circle/Capsule/Polygon 표면 적분이 아닌 AABB 네 점이다. 유체에 분산된 각운동량의 총합은 측정하지 않으므로 전체 각운동량 보존이나 실제 유체 점성 저항 계수를 보증하지 않는다.

## Part 13g — 선택형 Cut-cell 압력·Body 유효질량 결합 PCG

- `setCutCellCoupledProjectionEnabled(true)`를 새로 제공한다. 기본 `false`여서 종전 Cut-cell Jacobi와 이진 MAC PCG 동작은 바뀌지 않는다. Cut-cell을 켜고 `fluidShapeObstacle2.inverseMass / inverseInertia`가 양수인 장애물을 전달했을 때만 결합 경로를 사용한다. 현재 Wind Sandbox는 Cut-cell Body 역질량을 주입하지 않으므로 해당 경로를 아직 자동 활성화하지 않는다.
- 기존 PCG의 이산 Schur 연산자 `A = L + B M_body^-1 B^T`를 재사용하며, Cut-cell에서 `L`은 각 열린 face의 길이 비율을 사용한다. `B`는 **기존 Cut-cell 압력 feedback과 똑같은** 셀 개방율 차이의 힘 `-h*(r-l, t-b)`와 셀 중심 COM 레버암에 의한 토크 계수다. 연결된 유체 영역별 RHS 평균 제거는 기존 Cut-cell 단계에서 수행한 값을 그대로 사용한다.
- `getCutCellPressure()`에 결합 PCG 사용 여부, 실패 여부, 상태, 반복 수 및 상대 잔차를 별도 표시한다. 압력 PCG 실패는 Jacobi 반력으로 대체하지 않고 해당 Step의 압력을 0으로 둔다. 결합 경로의 `pressureResidual`은 전체 Schur 연산자의 실제 잔차를 원래 표시 단위로 환산하며, `divergenceAfter`는 **강체의 예측 변형률/체적 응답을 포함하지 않은 유체측 개방 flux 진단**이므로 두 값은 같지 않을 수 있다.
- Cut-cell의 binary momentum ledger는 계속 비워둔다. `clear()`는 진단값을 초기화하지만 선택된 실험 옵션은 유지한다.

**현재 한계:** 이 변경은 Cut-cell의 **압력 계수와 Body 유효질량을 같은 이산 행렬에서 계산하는 첫 단계**다. 현재 `B`는 정확한 실제 cut surface traction이 아니라 근사 셀 압력 반력이며, 움직이는 인터페이스의 swept 체적과 Body 속도 변화를 연속 시간으로 완전 연립한 모델이 아니다. 단절된 유체 영역 간 gauge·반력 배분, 외벽 및 미분할 dual volume의 운동량, pressure-predicted cut-face 법선 속도 동기화도 완성되지 않았다. 복합 Shape의 정확한 ownership과 end-to-end 보존 검증 전에 일반 Sandbox 모드의 기본 경로를 이 Solver로 교체하면 안 된다.

## Part 13h — Cut-cell 압력 반력 이후 Solid face 속도 동기화

- Part 13g의 **선택형 Cut-cell 연립 PCG가 수렴했을 때만** `synchronizeCutCellBoundaries()`를 호출한다. 이진 MAC 경로와 기존 Cut-cell Jacobi, PCG 실패 시에는 적용하지 않는다.
- 앞선 Cut-cell 기하 갱신에 사용한 **각 U/V face의 4개 sub-face 표본**을 다시 순회한다. 같은 Shape 첫 점유자 규칙과 같은 위치에서 `v'_COM = v_COM + dt·inverseMass·F_pressure`, `ω' = ω + dt·inverseInertia·τ_pressure`로 계산한 강체 표면 속도의 법선 성분을 평균한다. 고정/정적 장애물은 역질량과 역관성이 0이므로 원래 속도를 유지한다.
- 완전히 차단된 face는 계산된 `uSolidVelocity_/vSolidVelocity_`와 함께 MAC `u_/v_`를 갱신한다. **부분 개방 face에서는 막힌 부분의 Solid 법선 속도만 갱신하고 열린 부분의 유체 법선 속도는 변경하지 않는다.** 압력으로 투영한 유체 속도를 다시 덮어쓰지 않기 위해서다.
- `getUFaceSolidVelocity()/getVFaceSolidVelocity()`에서 해당 면의 막힌 부분 평균 속도를 읽을 수 있다(외곽 face/범위 밖은 0). `getCutCellPressure()`의 `synchronizedBoundaryFaces`, `synchronizedPartialFaces`, `boundaryNormalChangeRms`는 마지막 **수렴한 연립 압력 단계**가 갱신한 면 개수와 원래 Solid face 속도에 대한 RMS 변화량이다. `clear()`는 이 값도 초기화한다.
- 회귀 테스트는 Circle이 이동·회전할 때 가려진 sub-face에서 독립 계산한 예측 속도와 저장된 U/V Solid 경계값이 일치하는지, 일부만 열린 face와 완전 차단 face, PCG 실패/clear를 검사한다.

**보존 범위:** 이번 동기화는 압력 단계 말미의 **경계 속도 저장 상태**를 고치는 것이다. 다음 Step의 `updateCutCellGeometry()`는 World의 실제 최신 속도·형상을 다시 사용하며, 현재 `prepareCutCellVolumeChange()`의 Swept Volume/Cell Fraction을 이 예측 속도로 다시 적분하지 않는다. 따라서 `divergenceAfter`는 기존 fluid aperture flux 진단이고, 이 변경만으로 moving-interface 연속성, 전체 유체–강체 운동량·각운동량·에너지 보존을 증명하지 않는다. 또한 각 sub-face 재검사는 선택 모드에서만 수행하지만 추가적인 격자 스캔 비용이 있다.

## Part 13i — Cut-cell 압력 반력과 예측 체적 유량의 Adjoint 검사

Part 13h까지는 강체 반력을 셀 중심 `-h(r-l,t-b)\,` 근사로 만들고, 실제 경계 속도는 4개 sub-face의 점유 Shape와 그 위치에서 계산했다. **같은 연립 압력해인데도 토크 레버암과 Shape 소유자가 일치하지 않아**, 압력 행렬의 `B M_body⁻¹ Bᵀ`가 경계 유량 변화에 대응하지 않을 수 있었다.

- **결합 Cut-cell 모드에 한정**해서 압력 연립행렬 `B`와 압력으로 강체에 돌려주는 힘/토크를 동일한 U/V sub-face 4점으로 정의한다. 내부 공유 face의 막힌 표본마다 왼쪽/아래 셀에는 `+h/4\cdot n`, 오른쪽/위 셀에는 `-h/4\cdot n`의 계수를 부여하고, 해당 점에서 `(point - COM) × F`로 토크를 구한다. 열린 유체 face에는 기존 aperture 가중 `L`을 유지한다.
- 같은 표본 규칙을 static/kinematic 표면 압력 반력 진단에도 사용한다. 반력 장부에는 남기지만 `inverseMass=0`, `inverseInertia=0`인 경계는 Schur Body 응답 항에 기여하지 않는다. **동적 경계가 실제로 존재할 때만** 결합 PCG를 수행한다.
- PCG가 수렴한 Cut-cell 압력은 해 `x`에서 임의의 전역 평균을 빼지 않고 사용한다. 근사 Cut-cell의 `B`는 완전한 pressure gauge nullspace를 보장하지 않으므로, 수렴 뒤 상수 압력을 빼면 `A*p=b`가 깨질 수 있다. 이진 MAC gauge 동작은 변경하지 않는다.
- 따라서 압력해에서 강체가 받는 `Δv=dt·M⁻¹·Bᵀp`와 face의 고체 부분 평균 속도 변화가 동일한 `B`의 기하·소유자를 따른다. **기존 Cut-cell Jacobi와 이진 MAC 경로는 변경하지 않는다.**
- `pressureUSolidFlux_/pressureVSolidFlux_`에는 `(blockedFraction) × (v_predicted - v_initial)`을 저장한다. `getCutCellPressure().predictedContinuityRms`는 투영된 열린 유체 flux + 기존 swept/점유 체적 source + **압력으로 예측한 추가 blocked face flux**, 그리고 분리 유체 영역별 호환성 평균 보정을 합친 연속방정식 잔차 RMS(1/s)이다. `predictedSolidVolumeL1`은 각 셀의 예측 추가 체적 변화 절댓값 합(m² × 단위 깊이 1m)이다. 실제 geometry를 이동/변경하는 값이 아니라 `dt·Δv` 선형화 측정치다.
- 결합 `pressureResidual`에도 `dt/h²`를 적용해 PCG `b-Ap`를 연속방정식 단위(1/s)로 반환한다. 기존 legacy Jacobi 잔차 표시 방식은 변경하지 않는다.
- 회귀 테스트는 고정된 영역에서 질량·관성 3종, 격자 크기 2종, 유체 시간 간격 2종의 12개 조건을 비교하고, 예측 경계 flux의 연속성 잔차가 PCG 행렬 잔차와 일치하는지 검증한다.

**제한:** 이 경로는 현재 고정된 Cut-cell geometry에서 압력 단계 동안 고체의 예측 속도 변화가 유발할 *1차 체적 flux*를 포함한다. **실제 형상의 다음 위치를 즉시 변형하거나**, swept geometry 적분, tracer 이송, 유체 전체 질량·에너지·각운동량 보존을 구현한 것은 아니다. 부분 개방 면의 4점 근사, 도메인 경계와 접촉하는 형상, 분리된 fluid pocket의 gauge/호환성 등은 추가 검증이 필요하다. 이 단계도 기본 OFF인 결합 Cut-cell 실험 모드에만 적용한다.

## Part 13j — Sandbox에서 Cut-cell 강체 양방향 연성 선택 실행

- `rigidBodyDemo::setWindCutCellCoupledProjectionEnabled(true)`와 Wind Sandbox 체크박스 **Cut-cell 질량·관성 연성 PCG (실험)**을 추가한다. 기존 Cut-cell 가중 Jacobi/이진 MAC은 수정하지 않고, 옵션의 기본값은 false다.
- `Cut-cell 압력 투영 + Cut-cell 연성 PCG + 강체·유체 양방향 + 강체 질량 압력 결합(동기화)`가 모두 켜져야 World의 **Dynamic Body 역질량/역관성**을 `fluidShapeObstacle2`에 넣는다. 이 경우에만 사용자가 선택한 Fluid 계산 횟수(1–32회, World substep 이하)만큼 World와 Field를 교대로 갱신한다. 실제 이동된 World pose가 다음 Fluid interval의 `previousObstacles_`와 비교되어 swept geometry에 들어가며, 압력 단계의 예측 체적 유량을 다음 구간의 실제 geometry에 **추가로 더하지 않는다**.
- 실험 Cut-cell 결합 경로는 200회의 PCG 한도, 나머지 경로는 기존 80회 기본 한도를 사용한다. PCG가 수렴하지 않으면 해당 interval의 압력 반력은 생략되며, UI에서 상태·반복 수·예측 연속방정식 RMS·동기화 face 수·추가 체적 L1을 확인할 수 있다.
- `configureWindField()`로 격자를 재생성해도 옵션은 보존하고 체적 이력은 초기화한다. `resetWindFlow()`도 옵션은 보존한다. 양방향 또는 동기화를 해제하면 Cut-cell의 legacy weighted 경로로 돌아간다.
- `sandboxWindFlowDemoTests`는 Circle의 실제 선속도·각속도 변화, World4/Fluid2 동기화, 다음 구간의 swept geometry 이력, Field 리사이즈와 양방향/동기화 비활성화를 검사한다.

**물리 모델 한계:** 이 Sandbox 연동은 Cut-cell 연속방정식의 swept geometry와 pressure-predicted 속도의 시간 순서를 실제 움직이는 강체로 검증하기 위한 *실험 환경*이지, 아직 전체 유체–강체/Tracers의 질량·각운동량·에너지 완전 보존을 입증하거나 실제 움직이는 표면의 정확한 연속 적분을 제공하지 않는다. 특히 Body contact/joint/drag가 World 다음 구간에서 최종 속도를 바꾸므로 압력 예측치와 실제 위치 변화는 일치하지 않을 수 있다. Field substep 횟수가 늘면 가격이 증가한다.

## Part 13k — World 이동 Cut-cell의 Swept 체적 이중 계상 감시

- Windows/Ubuntu CTest의 Wind Sandbox 통합 검증에서 `World4/Fluid2`를 36프레임 실행하며 동적 Circle 병진·Box 회전과 소스 없는 유체 와류를 동시에 사용한다.
- 각 Fluid 구간의 `volumeRate * fluidDt`와 별도 `getCutCellTransition().volumeDelta()`를 직접 대조한다. 압력 단계에서 보고한 `predictedSolidVolumeL1`은 별도 **선형화 진단값**으로 유지하며 다음 구간의 실제 형상 점유 변화율에 더하지 않는다. 정상 swept face 이력이 반복해서 만들어지는지, 결합 PCG가 실제 사용되는지, 관찰값과 Body 속도가 유한한지도 검사한다.
- 회전 Drag는 별도 단방향 감쇠와 양방향 선운동량 교환 테스트로 검증한다. Drag torque를 World의 작용점에 적용하지만 유체 각운동량 장부는 아직 없다.

**주의:** 이 검증은 형상 snapshot과 solver source 사이의 *중복 유량 방지* 회귀 테스트다. 장기 전체 물리 에너지 보존, swept surface의 정확한 연속 적분, 자유 수면/물의 체적 보존까지 보증하지 않는다.

## Part 13l — 선택형 Swept 면 시간 적응 표본

- 기존 moving-solid swept face quadrature는 총 이동량/격자 크기로 1–32개 시간 표본을 한 번 선택했다. 이 적분은 공유 MAC 면에 4개 공간 표본을 사용하므로 보존적인 내부 면 유량이지만, **셀 sub-cell 점유율의 시작·종료 snapshot 변화와 시간 적분 flux divergence 사이의 오차**가 생길 수 있다.
- `setAdaptiveSweptSamplingEnabled(true)`를 켜면 먼저 기존 시간 표본으로 적분하고, **셀별 `|ΔV_snapshot - div(sweptFaceVolume)|`의 L1 합**을 측정한다. 오차가 충분히 크면 시간 표본을 최대 3차례 2배씩(상한 64) 늘려 다시 적분한다. 오차가 실제로 줄어든 표본만 채택한다. 개선이 중단되면 더 이상 반복하지 않는다. 기본값은 false이며 기존 결과·비용은 그대로다.
- 같은 공유 면에 대해 하나의 swept U/V 값을 사용하는 규칙을 그대로 유지한다. 표본 재계산은 **실제 body 경로 속도의 시간 적분**일 뿐, 압력 단계의 예측 `predictedSolidVolumeL1`을 이력이나 swept flux에 다시 더하지 않는다.
- 정지한 동일 geometry는 1개 시간 표본으로 결과를 표시하지만 실제 면 적분 그리드 순회를 생략하고 모두 0으로 유지한다. 움직이는 장면에서는 사전 보수적 bounding radius로 원거리 Shape의 `Contains` 검사도 생략한다.
- `fluidSweptFaceFlux2.initialTimeSamples`, `refinements`, `initialGeometryMismatch` 진단을 추가했다. `geometryMismatch`는 최종 채택한 적분(그리고 기존 선택형 국소 보정 이후)의 L1 오차다. Sandbox는 checkbox 및 비교 통계를 표시하고 Field 리사이즈 시 옵션을 보존한다.
- 별도 유체 회귀 테스트는 다각형의 다양한 병진·회전, 정적 장애물의 적분 생략, `clear()` 이후 설정 유지, 시간 세분화로 인한 오차 비증가를 확인한다.

**제한:** 시간 표본을 늘린다고 4개 공간 표본의 cut interface 근사나 discrete snapshot 자체의 aliasing이 없어지는 것은 아니다. 회전·고속 이동에서 비용이 증가하며 오차가 개선되지 않을 경우 원래 표본을 채택한다. 이것은 moving-boundary의 완전한 기하 보존 또는 전체 FSI 에너지·질량 보존 증명이 아니다.

## Part 13m — Cut-cell 공간 표본 해상도 통합 (4×4 또는 8×8)

- `setCutCellSampleCount(4|8)`을 추가한다. 기본 4는 기존 4×4 cell 점유·공유 MAC face당 4점의 이산화 및 성능을 유지한다. 8을 사용하면 셀은 8×8=64점, 각 U/V face는 8점으로 계산한다. 유효 범위를 벗어난 입력은 4(8 미만) 또는 8(8 이상)로 정규화한다.
- 중요한 점은 **같은 공간 표본 수를 반드시 전체 Cut-cell 체인에서 공유**한다는 것이다. `updateCutCellGeometry`의 cell 면적·면 개방율/solid 표면 속도, 결합 PCG `B, Bᵀ`의 blocked sub-face 힘·토크, `synchronizeCutCellBoundaries`의 pressure-predicted blocked normal flux, 그리고 `updateSweptSolidFlux`의 이동 물체 시간 적분을 동일한 표본 단위로 맞춘다. 다른 값들을 혼용하면 면 질량·반력과 연속방정식이 맞지 않는다.
- **작동 중 공간 표본 수를 바꾸면** 이전 형상/체적 이력과 swept 진단을 무효화한다. 첫 새 프레임에는 이전 4×4 표본을 새로운 8×8 점유율에서 차분해 가짜 체적 source를 만들지 않는다. 이후 새 기준의 연속 스냅샷에서만 swept 적분한다. `clear()` 및 Sandbox Field 재설정에서는 선택된 8×8 옵션을 유지한다.
- Sandbox는 8×8 체크박스와 현재 공간 표본 수를 표시한다. 유체 회귀 테스트는 분석적 원 면적과 비교한 고정 fixture 정확도, 4→8 전환 시 이력 초기화, 실제 이동 후 swept 이력 정상 생성, 8점 PCG의 pressure–blocked-flux adjoint/연속방정식 잔차를 검사한다.

**한계:** 8×8은 4×4보다 셀 점유 면적의 `Contains` 호출이 최대 4배 늘고, cut-face의 샘플 및 Schur boundary 수가 늘어난다. 모든 형상·위치에서 분석적 기하 오차가 반드시 작아지는 것은 아니며, 이 단계는 *선택 가능한 균일 공간 이산화*다. 적응형 쿼드트리 기하·정확한 면적 분할·이동 표면의 보존적 이류·전체 FSI 질량/에너지 보존은 후속 작업이다.

## 다음 구현 후보

1. 정확한 moving interface surface 적분, 실제 cut-cell dual mass와 swept interface 연동형 이류, pressure–rigid 반력 수지 정밀화 및 성능 측정.
2. 복합 Body의 다중 Shape 집계, static obstacle, Segment/Chain의 별도 표면 경계 표현.
3. 연기/염료(tracer) 수송, 점성(확산), 회전(vorticity) 표현.
4. 압력 solver 수렴 기준과 성능 평가, 해상도 증대.
5. Cloth/Soft Body 입자에 속도장을 샘플링해 바람 힘 전달.

## 검증

`fluidFieldTests`는 크기/유효성/경계/속도 source/영역 밖 조회/Step(0)/비압축성 투영과 반복 수렴/지속 source를 확인한다. `sandboxWindFlowDemoTests`는 기존 단방향 실험과 원형 장애물로 생성된 압력 반력/유체 교란, 토글, 초기화를 확인한다. 두 테스트는 Linux/Windows library+headless Sandbox 경로에서 실행할 수 있고 Win32/DX11 화면 확인은 별도다.
