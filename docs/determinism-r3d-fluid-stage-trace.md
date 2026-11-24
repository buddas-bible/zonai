# R3d — 최초 Fluid 크로스 플랫폼 비트 불일치 단계 분리

> 기반: [R3c PR #344](https://github.com/buddas-bible/zonai/pull/344), 기준 HEAD `767a95d1`. 이 브랜치는 R3c 위에 쌓이며 `master`와 기존 유체 솔버 수식은 변경하지 않는다.

## R3c에서 재현된 문제

- 동일한 고정 입력을 Ubuntu와 Windows에서 각자 실행한 결과, `227,154` raw-bit 행 중 `34,577`개에서 차이가 났다.
- 첫 차이: `sampledCutCell.tick.1.u[2]`. Ubuntu `bd82f5af/4`, Windows `bd82f5b8/4`.
- [CI 실행 45268947458](https://github.com/buddas-bible/zonai/actions/runs/45268947458)의 World/Contact/Joint/Spline 등 기존 물리 비교는 동일하지만 Fluid 자체는 일치하지 않았다.
- 마지막 Tick 상태만으로는 다른 결과가 최초 생성된 계산 단계를 판단할 수 없다.

## 변경: 필드 복사 없이 선택적 단계 관측

테스트 전용 friend가 `fluidField2`의 private 콜백을 설치할 때만 열 단계에서 읽기 전용 필드 상태를 직렬화한다. 콜백은 모든 일반 Step에서 기본적으로 비어 있으며 프로덕션 계산식을 변경하지 않는다.

| 단계 | 관측 지점 |
| --- | --- |
| 00-entry | 장애물 점유율 갱신 직전 |
| 01-occupancy | 이진 장애물 점유율 갱신 직후 |
| 02-geometry | Cut-cell 점유 면적/개방률 계산 직후 |
| 03-swept | 시간적 swept flux 계산/보정 직후 |
| 04-remap | 새로 열린 MAC face 재구성 직후 |
| 05-firstBoundary | 이류 전 막힌 face 경계 속도 적용 직후 |
| 06-advection | MAC 이류 계산 직후, 경계 재적용 전 |
| 07-secondBoundary | 이류 후 경계 재적용 직후, 압력 투영 전 |
| 08-pressure | 압력 투영/반력 계산 직후 |
| 09-tracer | Tracer 수송 직후 |

두 장면(`sampledCutCell`, `analyticCircle`) 각각 첫 Tick에만 10개 단계의 원시 상태를 TSV로 기록한다. 이전 R3c 매 Tick 상태·반력 비교는 그대로 유지한다. 추가로 압력 RHS의 재현성을 관측하기 위해 `divergence_` 배열도 기록한다.

- 동일한 프로세스의 두 개 독립적인 측정 Field가 모든 단계에서 비트 단위로 일치해야 한다.
- 별도의 **콜백 없는 세 번째 Field**가 매 Tick 동일한 물리 상태를 내야 한다. 관측이 계산을 바꾸는 회귀를 차단한다.
- Windows와 Ubuntu는 기존 CI `fluid-ubuntu.tsv`/`fluid-windows.tsv`를 동일하게 생성하고 strict comparator가 **한 행이라도 다르면 실패**한다. 최초 차이는 `scene.tick.stage.field`로 출력된다.

## 수용 기준

1. Windows/Ubuntu 테스트 자체의 생성과 로컬 반복 재현은 성공.
2. 기존 전체 CMake Debug/선택적 Release 회귀 유지.
3. 단계별 비교가 실제 최초 차이(기하, swept, 이류, 압력 중 하나)를 지목.
4. 원인을 격리하기 전 `libm` 연산을 추측으로 교체하거나 strict gate를 완화하지 않는다.

## 남은 한계

이번 단계 관측은 두 기존 소규모 Circle 장면과 Tick 1에만 추가된다. PCG Schur 결합, Capsule·Polygon, 자유수면, 전체 World↔Fluid 양방향 결합, 극소 Aperture 압력 발산 장면은 아직 독립 검증이 필요하다. 결과가 Bitwise 동일하더라도 지원하는 모든 플랫폼에서 완전 결정론을 보증하는 것은 아니다.
