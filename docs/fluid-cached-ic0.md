# Cut-cell Schur IC(0) sweep 계수 캐싱 (Part 13aa)

## 변경 이유

PR #292에서 Windows Release 실제 World+Fluid 48×28 격자, Fluid 4회/World 8 substep 프로파일을 분해한 결과, Schur solve 7.661ms 가운데 IC(0) 전처리기 *적용* 4.757ms로 큰 비중을 차지했다. 여기에는 매 반복 이웃 face 개방율 조회(`cutCellUOpen`/`cutCellVOpen`)와 `pivot` 나눗셈이 포함되어 있었다. 정확성 유지를 위해 압력 Schur 연산자, PCG 수렴 허용오차·반력 계산, IC(0) 피벗 구성식은 수정하지 않는다.

## 구현

- `setAnalyticCutCellCachedIc0Enabled(bool)`은 기본 ON이다. OFF로 설정하면 기존의 reference 전진/후진 대입을 선택해 같은 물리 장면에서 차이를 검증할 수 있다. `setAnalyticCutCellIncompleteCholeskyEnabled(false)`는 여전히 Jacobi 경로이며 캐시 설정과 독립이다.
- 매 `solveCoupledPressure` 안에서 IC(0) 피벗을 구성하는 순간, 활성 격자 셀별로 서쪽/남쪽 MAC face의 유효 개방율, 열린 면 가중치를 이웃 피벗으로 나눈 계수, 자신의 역피벗을 함께 저장한다.
- 전진 대입에서 `westOverPivot * incompleteForward[west]`, `southOverPivot * incompleteForward[south]`를 쓰고, 후진 대입에서 동쪽/북쪽 셀의 공유 face 가중치를 읽은 후 `inversePivot`을 곱한다. 반복 중에는 개방율 검사 함수와 피벗 나눗셈을 호출하지 않는다.
- 캐시는 매 압력 풀이마다 새로 구성하며 이전 Frame/Fluid substep의 형상·피벗을 재활용하지 않는다. 기준 경로는 원래 계산 순서를 그대로 유지한다.
- 나눗셈을 미리 수행하면서 부동소수점 계산 순서가 일부 변경된다. 양쪽 모두 **동일한 실제 잔차 검사, PCG iteration limit, fail-closed 정책**을 통과해야 하고, 반력·속도와 연속 방정식 오차는 허용 범위 내에서 비교한다.

## 검증 및 비교

- `fluidFieldTests`: h=0.5,0.25, 두 Circle의 overlap/separation/containment를 각각 4회 이동시켜 기존 vs 캐시 경로의 Schur 수렴, 실제 잔차, Fluid 속도와 운동량, 개별 Body 압력 반력 및 impulse audit를 비교한다. `clear()` 후 모드 유지도 확인한다.
- `sandboxWindFlowDemoTests`: 실제 World 8 substep/Fluid 4회로 움직이는 Circle·Capsule·Polygon 장면에서 6프레임 동안 기준/캐시 경로의 World body transform과 velocity를 비교하고, field resize가 설정을 보존하는지 검사한다.
- `fluidPreconditionerBenchmark`: Jacobi, IC0-reference, IC0-cached를 각각 동일 cold-start 조건에서 5회씩 측정한다. 모든 비교 대상이 수렴한 경우에만 전체 Schur 시간과 전처리 적용 시간의 속도 향상 비율을 출력한다. 실패한 Jacobi와의 시간을 성공적인 성능 개선으로 보고하지 않는다.
- `sandboxWindStepBenchmark --csv`는 캐시 경로, `--ic0-reference --csv`는 같은 World/Fluid 실험의 기준 경로다. Windows Release CI에서 두 실행을 모두 정보 제공용으로 기록한다.

## 측정 주의

CI runner별 실행시간과 실제 게임의 FPS는 다르다. 캐싱으로 준비 단계(`factorization`)의 비용과 scratch 메모리가 늘 수 있으므로, **전처리기 적용 시간과 전체 Schur/Field 시간을 함께** 비교해야 한다. CI에는 절대 속도 통과/실패 기준을 추가하지 않는다.
