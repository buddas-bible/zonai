# Cut-cell IC(0) 활성 셀 순회 (Part 13ab)

## 배경과 목적

PR #293의 Windows Release 실행(48×28, World 8 / Fluid 4)에서 캐시 기반 IC(0) 전처리기 적용은 2.873ms, 전체 Schur는 5.748ms였다. 기존 cached IC(0) 코드는 이웃 계수/역피벗을 미리 계산하지만, PCG 반복에서 여전히 매 전진·후진 대입마다 격자 전체 x/y를 순회하고 `cellFluidFraction`을 확인한다. 이번 변경은 **물리 계산을 유지한 채 활성 셀 스케줄을 사전 구성**한다.

## 구현

- `setAnalyticCutCellActiveIc0SweepEnabled(bool)` 기본 true. `setAnalyticCutCellCachedIc0Enabled(false)`는 원래 조회/나눗셈 IC(0) 경로, `cached=true`+`active=false`는 PR #293의 밀집 Cached IC(0) 순회, `cached=true`+`active=true`는 새 활성 셀 순회다. Jacobi 선택은 이 옵션과 독립이다.
- 각 결합 압력 solve에서 피벗을 만들 때 유체 fraction이 양수인 셀 인덱스를 행 우선으로 `incompleteActiveCells`에 추가한다. 이 리스트를 뒤집어 읽으면 후진 대입 순서와 정확히 일치한다.
- 서쪽/남쪽 face의 계수는 기존대로 계산한다. 면이 열려 있으면 해당 이웃 셀의 동쪽/북쪽 계수도 동시에 기록한다. 위상·결합 강체가 달라지는 매 step마다 새 scratch 값으로 다시 계산한다.
- 활성 셀을 순서대로 방문하므로 반복 중에는 매 셀의 2D 좌표, 유체 여부, 외벽 조건을 검사하지 않는다. 실제 열린 연결을 통해서만 이웃을 참조하여 경계에서 배열 범위를 벗어나지 않는다.
- 원래 Schur 행렬 연산자, IC(0) 정규화/피벗식, `b-A*x` 실제 잔차 검증, PCG 반복 상한과 fail-closed 압력 반력 정책은 변경하지 않는다.

## 검증

- `fluidFieldTests`: 격자 0.5m/0.25m, 두 원의 교차·분리·포함, 매 step 움직이는 장애물에서 dense vs active 결합 압력 수렴/반복횟수/실제 잔차/연속방정식 잔차/강체 힘과 토크/유체 운동량과 속도를 비교한다. `clear` 이후 옵션 유지·진단 초기화도 확인한다.
- `sandboxWindFlowDemoTests`: 실제 World 8/Fluid 4 substep에서 Circle/Capsule/Polygon을 대상으로 dense/active 모드의 동기화된 강체 운동을 비교한다. 리사이즈 후 모드 보존도 확인한다.
- `fluidPreconditionerBenchmark --csv`: Jacobi, 원래 IC(0), 기존 dense cached IC(0), active cached IC(0)의 cold-start 중앙값을 출력. 둘 다 수렴한 경우만 dense 대비 속도 향상 비율을 계산한다.
- `sandboxWindStepBenchmark --csv` 기본 active, `--ic0-dense --csv`는 기존 dense cached, `--ic0-reference --csv`는 원래 IC(0). Windows Release CI의 동일 러너에서 각각 실행하며 **절대 성능 gate는 걸지 않는다**.

## 유의점

활성 셀의 인덱스 리스트와 동/북 계수는 각 압력 풀이의 factorization 비용과 scratch 메모리를 소폭 늘린다. 전처리기 한 번의 속도만이 아닌 Schur 전체, Field Step 전체, 셀 크기/Fluid substep 개수에 따른 비용으로 최적화 유효성을 판단한다. CI runner 실측은 환경 의존적이며 게임 실제 FPS를 보증하지 않는다.
