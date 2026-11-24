# R3c — Fluid MAC/Cut-cell 권위 상태의 크로스 플랫폼 원시 비트 추적

> R3b integration candidate [#343](https://github.com/buddas-bible/zonai/pull/343) 위에 쌓인 별도 Draft 검증 단계. 기존 MAC 압력/이류/강체 상호작용 수식과 스텝 순서는 변경하지 않는다.

## 목적

R3a/R3b의 strict World/Contact/Joint 비교가 성공했어도 Fluid의 권위 있는 격자·기하 이력은 비교되지 않았다. Production Fluid에는 플랫폼 간 차이를 일으킬 수 있는 \`std::hypot\`, \`std::asin\`, \`std::atan2\`, \`std::remainder\` 호출도 남아 있다. 수치 모델 교체나 근거 없는 연산 교체보다 먼저 실제 Fluid 계산의 **최초 비트 차이**를 관측한다.

## 변경

- \`fluidField2\`에 테스트 전용 friend \`fluidStateTraceProbe2\`만 추가했다. 정상적인 Step, 데이터 배치, 외부 공개 API 및 시뮬레이션 식은 그대로다.
- \`fluidStateDeterminismTests\`에서 고정된 32 Tick, 1/60s, 동일한 입력/장애물 이동을 두 개의 독립 Field에 재생하고 각 Tick의 원시 비트를 비교한다.
- Sampled Cut-cell 및 단일 analytic Circle Cut-cell 두 경로를 각각 시험한다. Fractional cell, 유동, 압력 경계 힘이 출력에 포함된다.
- Raw row-major \`u/v\`, \`pressure\`, \`tracerMass\`, 정적/직전 solid owner, 현재/직전 fluid fraction, face open fraction, solid face velocity, pressure solid flux, swept volume과 cell volume rate, obstacle geometry history, 압력 진단과 Body feedback의 실수 비트가 TSV로 기록된다.
- 메모리 패딩/미사용 capacity, wall clock/CPU 시간, transient PCG scratch는 snapshot에서 제외한다. 진단용 수치는 생성 시점을 분리해서 해석해야 한다.
- Windows·Ubuntu가 **각자 자체적으로 Field를 계산**해 \`fluid-*.tsv\`를 생성하고 strict \`compare\` job에서 모든 행을 비트 단위로 비교한다. 첫 차이의 Tick/field/raw bits와 불일치 수를 \`fluid-comparison.txt\`로 보관한다. 오차 허용치나 skip은 없다.

## 검증 조건

1. 같은 실행 파일 안의 독립적인 Field 두 개가 동일 Tick에서 같은 비트를 만들어야 한다.
2. CMake Ubuntu/Windows Debug CTest 및 기존 Windows Release 검사에 회귀가 없어야 한다.
3. Cross-OS 비교 결과가 같다면 이 **두 테스트 장면에 한해** 결정론적이며, 다르다면 첫 raw-bit divergence를 다음 패치의 수정 대상으로 삼는다.
4. 기존 R3a 외부 libm 연구용 적색 검사는 계속 분리한다. Fluid 권위 상태는 진단으로 강등하지 않으며, 한 비트라도 달라지면 required physics job을 실패시킨다.

## 명시적 한계

- 현재는 Fluid 독립 Field 테스트이며, Sandbox 전체 World↔Fluid 양방향 결합/World Body state를 동시에 추적하지 않는다.
- 단일 움직이는 Circle과 Sampled/Analytic Cut-cell만 시험한다. Capsule, Polygon, multiple-body Schur, PCG Warm Start, 회전 swept geometry, 재구성된 소규모 Cut-cell pressure spike는 후속 fixture가 필요하다.
- 관측 대상은 **매 Tick 종료 상태**이다. 최초 차이가 압력 전/후 어느 단계인지 파악하려면 pre-pressure frozen snapshot과 stage trace를 추가해야 한다.
- CI가 green이어도 다른 CPU·컴파일러·아키텍처에서의 비트 일치나 전체 Fluid 결정론을 보증하지 않는다.

## 이후 판단

R3c 비교 결과를 확인한 뒤, 실제 Bit divergence가 발생하는 Fluid 경로에서만 \`deterministicHypot\`, \`deterministicAtan2\` 등 기존 수학 helper 재사용을 검토한다. LBM 같은 유체 모델 교체는 별도 성능/안정성/결정론 실험 후 판단한다.