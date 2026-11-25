# R3h — 확대된 Fluid 기하 및 압력 결합 결정론 검사

기반: [R3g #348](https://github.com/buddas-bible/zonai/pull/348), `cda3cecb5f877a5218134c18ad8296b578881eda`.

## 확인된 R3g 결과

[Strict CI 45347667841](https://github.com/buddas-bible/zonai/actions/runs/45347667841)에서 기존 Fluid 312,474행 전체와 엔진 수학 1,434개가 Windows/Ubuntu 간 일치했다. 이전 48,159개 Fluid 불일치가 0개로 줄었다. [전체 CMake](https://github.com/buddas-bible/zonai/actions/runs/45347667816) 양 OS 성공. [Release A/B](https://github.com/buddas-bible/zonai/actions/runs/45347667921) 완료. 이 결과는 기존 두 장면에 한정되며 외부 libm 진단의 실패와 극소 개방 면의 속도 급등 문제는 남는다.

## 추가 장면

기존 두 Circle 장면은 수정하지 않는다. 24×16 격자(h=0.5m), dt=1/60, 12 Tick의 네 장면을 추가한다.

| Scene | 검증 범위 |
|---|---|
| sampledMixed | Circle/Capsule/Box/Rounded Box의 표본 기하, 회전·병진, swept history |
| analyticCapsule | 단일 회전 Capsule 분석 기하 |
| analyticPolygon | 단일 회전 Box 분석 기하 |
| coupledDisjoint | 분리된 네 도형 분석 기하, 움직이는 두 장애물에 대한 Schur-PCG/IC(0) 압력 반력 재입력 |

각 장면의 처음 두 Tick은 기존 10단계 관측 훅으로 기록한다. 두 번째 Tick에 swept history를 포함한다. 매 Tick의 MAC/geometry/history, PCG 상태·반복수·잔차·경계 예측, 장애물 pose/velocity 및 feedback bits를 관측하지 않은 독립 Field와 비교한다. 유한값, 실제 partial cell/flow/swept history, 분석 기하의 비표본 분율, 실제 Schur/IC0 수렴 및 비영 반력의 경로 실행 여부를 검증한다. PCG 실패 자체는 기록하며 최소 한 번의 수렴 경로를 요구한다.

압력 반력을 다음 Tick의 장애물 속도에 재입력하지만, 이 테스트는 **실제 World::Step/Drag/Joint 결합이 아니다**. World 결합 전체는 별도 후속 검사 대상이다.

## CI 및 수용 기준

기존 strict Fluid bit gate의 허용치와 실패 조건은 바꾸지 않는다. 각 장면의 총 비교 행, 불일치 행, 첫 불일치 raw bits를 출력한다. 새 장면에서 다른 연산의 문제가 드러나면 원인을 분리한 뒤 최소 변경한다. R3h는 생산 물리 코드 변경 없이 범위 확대만 수행한다. 전체 CMake 회귀와 기존 312,474행의 일치를 유지해야 한다.

R3g의 역사적 A/B workflow는 최종 스택 통합 시 정리 대상이며, 이 변경에서는 수정하지 않는다. master 및 실험 PR #326은 별도로 유지한다.
