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
- `addVelocity`는 속도 injection이며 보존적인 inflow boundary/source는 아니다.
- Jacobi는 수렴이 느릴 수 있다. 큰 해상도에서 다중 격자/PCG 또는 더 정확한 수송 기법을 검토한다.
- 특히 시간이 큰 경우 격자 이류의 샘플링은 수치 확산을 낳으며, 유체와 Body가 공유하는 고정 timestep/동기화를 더 구체화해야 한다.

## Part 2 — 움직이는 원형 Solid와 압력 반력

`fluidField2::step(dt, obstacles, feedback, iterations)`는 매 물리 Step에서 원형 강체의 world-space 중심·반경·선속도·각속도를 입력받아 장애물과 압력 반력을 계산한다.

- 원의 내부에 **중심이 들어간 격자 셀**을 Solid로 마스킹한다. 실제 원 경계와 일치하지 않는 계단형 경계(격자 해상도 제한)다. 두 원이 겹친 셀은 최초 원이 소유한다.
- Solid와 접하는 MAC face의 **법선 속도**는 `v_solid(face) = v_center + omega × (face-center)`로 설정한다. 유체·유체 face만 압력 구배로 수정하며 Solid 내부는 압력 미지수에서 제외한다. 유체 셀의 `div(u)` 계산에는 Solid 경계 속도를 포함한다.
- 투영 후 각 유체·Solid 경계에서 `F ≈ p_fluid * normal_solid * cellSize * unitDepth`를 적분해 강체별 `fluidObstacleForce2{force, torque}`를 반환한다. 유체의 기준 밀도는 1kg/m³, 깊이는 1m로 고정한 학습용 단위 모델이다.
- World는 별도로 움직이는 강체이고 `fluidField2`는 ID나 Body를 보유하지 않는다. Sandbox에서 현재 Circle Shape를 world 좌표로 변환해 전달하고, 결과 압력 force/torque를 `ApplyForceToCenter`, `ApplyTorque`로 전달한다.
- 기존 경험적 Drag `F = dragCoefficient * (v_fluid-v_body)`는 유지하되, 양방향 모드에서 `-F * dt`의 근사치를 강체 주변 유체의 속도 source로 되돌린다. source가 보존형 격자 flux가 아니므로 **엄밀한 작용-반작용이나 운동량 보존을 보장하지 않는다.**
- Sandbox에서 양방향 모드를 끄면 원래 단방향 실험이 동작한다. 점유 셀과 물체별 압력 반력을 보고, 움직이는 원이 정지 유체에 어떤 흔적을 남기는지 비교할 수 있다.

이 방식은 **원형 강체의 격자화된 no-through 경계 및 압력 feedback 첫 실험**이지 완성된 양방향 CFD-강체 solver는 아니다. 질량 중심이 빠르게 이동할 때 계단형 점유 셀이 불연속적으로 바뀌며 압력에 인공적인 힘을 더할 수 있다. 압력 gauge, 유체 점성/마찰, Solid/Fluid 체적 보존과 시간 적분의 완전 동기화, 일반 polygon/chain의 보존형 cut-cell coupling은 후속 구현 대상이다. 일반 Body collision은 원래 zonai World Solver가 해결한다.

## 다음 구현 후보

1. 보존형 fluid-solid coupling과 압력/drag 운동량 budget, 경계 셀 정확도 검증.
2. 일반 Polygon/Capsule/Chain 및 static obstacle 적용, 반응 가능한 회전 torque.
3. 연기/염료(tracer) 수송, 점성(확산), 회전(vorticity) 표현.
4. 압력 solver 수렴 기준과 성능 평가, 해상도 증대.
5. Cloth/Soft Body 입자에 속도장을 샘플링해 바람 힘 전달.

## 검증

`fluidFieldTests`는 크기/유효성/경계/속도 source/영역 밖 조회/Step(0)/비압축성 투영과 반복 수렴/지속 source를 확인한다. `sandboxWindFlowDemoTests`는 기존 단방향 실험과 원형 장애물로 생성된 압력 반력/유체 교란, 토글, 초기화를 확인한다. 두 테스트는 Linux/Windows library+headless Sandbox 경로에서 실행할 수 있고 Win32/DX11 화면 확인은 별도다.
