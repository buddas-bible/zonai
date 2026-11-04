# Cut-cell Schur Laplacian face 연결 캐시 (Part 13ac)

## 동기

PR #294 이후 IC(0) 활성 셀 순회가 적용되어 반복 비용이 감소했다. 하지만 matrix-free Schur 연산자 `A = L + B M_body^-1 Bᵀ`의 `L` 계산은 PCG 반복마다 모든 U/V 내부 face를 순회해 `cutCellUOpen`/`cutCellVOpen`과 두 셀 인덱스를 다시 계산한다. 이번 PR은 변하지 않는 **L의 nonzero MAC face와 개방율만** coupled pressure solve마다 사전 구축한다. 강체 결합 `B M^-1 Bᵀ`의 계산 및 기하 모델과 압력 수렴 조건은 바꾸지 않는다.

## 설계

- 기본 ON인 `setAnalyticCutCellCachedSchurEdgesEnabled(bool)`은 분석적 Cut-cell 및 유효질량 연립 압력 경로에서 작동한다. OFF는 기존 matrix-free 격자 직접 순회 방식이다. 일반 이진 MAC 경로·샘플링 Cut-cell 경로에는 새 캐시를 적용하지 않는다.
- 기존 `coupledPressureScratch`에 `coupledPressureEdge { first, second, weight }` 배열을 둔다. U 내부 face의 x/y 행 우선 순회를 먼저, V 내부 face의 x/y 행 우선 순회를 나중에 수행해 기존 누산 순서를 유지한다. 개방율 0인 face는 저장하지 않는다.
- 이후 PCG의 모든 `multiply()`에서 같은 순서로 `d = weight*(p[first]-p[second])`를 양옆 셀에 `+d/-d` 적용한다. 기존 `faces` 벡터 및 두 차례 Body Schur 응답 집계는 그대로 진행한다. 마지막 실제 잔차 검증도 같은 연산자로 계산한다.
- **각 solve마다** 엣지를 다시 수집하고 scratch의 `std::vector` capacity만 재사용한다. 격자 크기 변경 또는 움직이는 장애물 뒤에 이전 단계 가중치를 재사용하지 않는다.
- 선택적 압력 프로파일에는 `cachedLaplacianEdges`를 추가한다(해당 solve에 저장한 nonzero face 수). 타이밍 OFF이거나 직접 순회 경로이면 0이며 Sandbox의 Fluid substep 누적 진단도 합산된다.

## 검증

- `fluidFieldTests`: h=0.5/0.25, 두 원의 교차·분리·포함, 4회 이동하는 장애물에서 캐시 vs 직접 순회를 비교한다. PCG 반복수, 실제 상대 잔차, 압력 연속방정식 잔차, 강체별 force/torque, 유체 속도·운동량을 대조한다.
- `sandboxWindFlowDemoTests`: 실제 World 8/Fluid 4 substep 및 Circle/Capsule/Polygon/풍력 연동으로 6프레임의 Body 위치·속도와 pressure 수렴을 비교하고, 필드 리사이즈 후 비교 모드가 유지되는지 검사한다.
- `fluidPreconditionerBenchmark`: 기존 Jacobi·IC(0) 세 경로 외에 **활성 셀 IC(0) + Schur face 캐시**를 추가, 고정 장면에서 동일 PCG 전처리기끼리 수렴했을 때만 전체 Schur 및 `A*v` 시간의 비율을 기록한다.
- `sandboxWindStepBenchmark --csv`는 Schur face 캐시 기본 ON, `--schur-reference --csv`는 캐시만 OFF하여 같은 World/Fluid 장면으로 비교한다. 기존 `--ic0-reference`/`--ic0-dense`는 이전 PCG 비교를 보존하기 위해 Schur face 캐시를 끈다. Windows Release CI는 두 연산자를 동일 runner에서 측정하며 실행시간에 합격/실패 게이트를 두지 않는다.

## 판단 기준

캐시 생성은 초기 `assembly` 시간을 늘리고 메모리를 소모한다. 반복 연산 `multiply`뿐 아니라 전체 Schur 및 Field 시간까지 이득이 있는지 확인한다. 0 개방율 face를 생략하더라도 유효 Laplacian 항의 덧셈 순서는 유지되며, 시뮬레이션의 실제 압력 결과는 수치 회귀로 검증한다. CI runner와 사용자 PC의 FPS는 구분해야 한다.
