# Cut-cell 연립 Schur PCG 단계별 성능 진단 (Part 13z)

`setCutCellPressureTimingEnabled(true)`는 Cut-cell 연립 압력 풀이의 내부 경과시간(ms)을 별도로 수집한다. 기본 OFF. `getCutCellPressure().coupledTiming`은 마지막 Fluid Step의 Schur solve에 대한 진단이고, 해당 Fluid Step에서 결합 압력을 요청하지 않았다면 모든 필드가 0이다. 기존 `coupledSolveMilliseconds`와 같은 전체 구간을 다루며, 전처리기 및 행렬·힘·수렴 허용오차는 변경하지 않는다.

측정 항목: `assembly`(RHS/diagonal + 강체 경계 face clipping/추출), `factorization`(IC(0)의 incomplete LDLᵀ 준비; Jacobi는 계측 오버헤드만), `multiply`(PCG 반복 안의 matrix-free A*v), `preconditioner`(Jacobi 또는 IC(0) 적용), `vector`(R/Z 방향 벡터 갱신, 내적, 수렴 판정), `validation`(종료 시 별도 진짜 잔차 b-A*x 검증), `output`(float 압력/게이지 검증 및 기록). `matrixProducts`는 PCG 반복에 사용된 A*v 횟수이고, `preconditionerApplications`는 처음 1회와 반복마다의 실제 적용 횟수다. `validation`의 추가 A*v는 `matrixProducts`에 포함하지 않는다.

`total`은 Schur solve의 내부 전체 wall-clock 시간이며 `accounted()`는 단계별 합계다. 단계 전환 시계 오버헤드로 부동소수점 오차가 있을 수 있으므로 합계가 완전히 같을 필요는 없다. 수치적으로 실패하거나 iteration limit에 도달해도 **그 시점까지의 실제 수행량**을 보존하며, 성공·실패 상태는 별도의 `coupledStatus`와 `hasAnalyticImpulseBudget`을 사용한다. 타이밍 데이터는 과학적 계측용이며 결과 수렴 여부 판정에 사용하지 않는다.

Sandbox의 **유체 단계별 시간 측정** 옵션은 내부 Cut-cell Schur 시간 계측도 함께 켜고, 프레임에서 교대로 실행된 Fluid substep의 내부 시간을 `getWindCoupledTiming()`으로 모두 합산한다. UI는 이 값을 기존 전체 압력 투영(`getWindFieldTiming().pressure`)의 일부로 보여준다. Field를 리사이즈하거나 비활성화할 때 stale 통계를 지운다.

기존 `fluidPreconditionerBenchmark --csv`와 `sandboxWindStepBenchmark --csv`를 확장해 행렬 구성·IC(0)·A*v·전처리 적용·벡터·진짜 잔차 측정값과 연산 횟수를 출력한다. Windows Release CI는 기존 벤치마크를 정보 제공 전용으로 실행하고 속도 수치로 CI 합격·실패를 결정하지 않는다. **수렴하지 않은 Jacobi와 수렴한 IC(0)의 실행 시간 비율을 성능 개선으로 간주하지 않는다.**

본 변경은 관찰용 프로파일만 추가한 것으로, 빠른 알고리즘/메모리 레이아웃 최적화는 측정 결과를 보고 별도 PR에서 검증한다. 계측이 켜진 실행은 시계 호출 비용으로 인해 순수 실행시간보다 느릴 수 있고, CI 환경은 사용자 실제 PC의 FPS를 대변하지 않는다.


## 이전 압력 초기값 타당성 계측 (실험)

`setCoupledPressureWarmStartProbeEnabled(true)`와 압력 타이밍을 모두 켜면, 매
Cut-cell Schur solve에서 **이전 Step의 압력**을 현재 연산자에 적용한다.
`warmStartRhsSquared`는 새 RHS의 제곱노름 `||b||²`,
`warmStartGuessSquared`는 이전 압력을 초기값으로 가정한
`||b-A*p_previous||²`, `warmStartProbes`는 성공한 계측 횟수다.
각 값은 substep별로 더할 수 있으므로 Sandbox의 프레임 합산에도 적합하다.
`sqrt(guessSquared/rhsSquared)`가 1보다 작으면 이전 압력이 0 초기값보다
작은 잔차를 만들었다는 뜻이지만, **PCG 반복 감소 또는 성능 향상을 보장하지는 않는다.**
0보다 큰 새 RHS 제곱노름과 유한한 값만 분석한다.

이 모드는 실제 PCG의 x, residual, 수렴 판정, 압력, 힘/토크를 변경하지 않는다.
추가 A*p 연산은 오직 옵트인 계측에만 사용되며 정규 PCG 반복 횟수에는 세지 않는다.
대신 Schur `assembly`와 `total` 시간에 추가 비용이 포함되므로
`--pressure-reuse-probe`의 프레임 시간은 일반 벤치마크와 속도 비교에 쓰면 안 된다.
기본값은 OFF이며, 압력 타이밍이 꺼져 있으면 추가 A*p도 수행하지 않는다.

Windows Release CI의
`sandboxWindStepBenchmark --pressure-reuse-probe`는 실제 World 8 substeps와
Fluid 1/2/4/8 substeps를 각각 측정하고 중앙 프레임별 잔차 비율과
probe 횟수를 마지막 CSV 열에 기록한다. `fluidFieldTests`에서는
회전/이동 강체의 연속 스냅샷과 두 격자 크기에서 probe ON/OFF의
PCG 반복, 압력 잔차, 강체 반력 및 운동량의 정확한 일치를 검증한다.


## 실제 PCG Warm Start (선택형 성능 실험)

`setCoupledPressureWarmStartEnabled(true)`는 analytic Cut-cell Schur PCG에서
직전 Fluid Step의 `pressure_` (float)을 현재 압력 연산자에 대입한다.
기본 OFF이며 기존 zero-initialized PCG는 변함없이 남겨둔다.
현재 `b`와 `A = L + B M^-1 B^T`를 조립한 다음 정확히 한 번 `A*p_previous`를
계산하고, `||b-A*p_previous||² < 0.25 ||b||²`이며 모든 값이 유한할 때만
이전 압력을 초기 `x`로 채택한다. 그렇지 않으면 기존 0 초기값으로 진행한다.

채택 시 `r=b-A*x`와 `x=p_previous`로 시작하지만 **종료 기준은 원래
`max(1e-18, ||b||²*1e-10)`을 그대로 사용**한다. 종료 시 별도
`b-A*x` 검증, 강체 압력 반력/운동량 경로도 유지한다.
`coupledInitialResidualRms`와 `coupledRelativeResidual` 역시 기존
Cold RHS 기준으로 표시한다. 공통 압력 계수 캐시와 압력 solver 전체를
매 Step 새로 준비하므로 오래된 factorization을 재사용하지 않는다.
기본 동작에는 추가 행렬 곱과 수치 변경이 없다.

계측 중에는 `warmStartAttempts` / `warmStartAccepted`가 누적되며,
기존 `--pressure-reuse-probe`와 함께 켜면 잔차 비율도 별도 기록한다.
Windows Release의
`sandboxWindStepBenchmark --pcg-warmstart-sweep`는 실제 World 서브스텝 8회와
Fluid 서브스텝 1/2/4/8회에서 cold/warm의 PCG 반복, 수렴 횟수, 전처리
호출, Pressure/Field/Fluid 시간을 출력한다. 추가 `A*x` 비용 때문에
반복 수가 줄어도 반드시 전체 시간 개선으로 이어지는 것은 아니다.

이 경로는 현재 **옵트인 검증용**이다. 장기적인 고속·고충격 동적
강체, 셀 점유 급변, 다중 분리 유체 영역, Warm Start 실패 시 안정적인
Cold retry 등 회귀 테스트가 충분히 쌓이기 전에는 기본 ON으로
전환하지 않는다.


### World/Fluid에서 실제 채택 경로 회귀 검증

`sandboxWindFlowDemoTests`의 `checkWorldPressureWarmStartDifferential`은
같은 초기 상태의 Cold/Warm 장면을 각각 12프레임 실행한다. World 8회,
Fluid 4회 서브스텝을 교대하고, h=0.5m와 h=0.25m를 모두 포함한다.
기존 pressure diagnostic과 달리 **채택된 Warm Start 경로가 실제로
실행되는지**를 검사하며, 프레임 내 Warm Start 시도 횟수(4회),
양쪽 최종 수렴, 잔차, 압력 연속성, body별 압력 force/torque,
World의 body 위치/선형 속도를 대조한다.

또한 Warm Start 채택이 1회 이상 발생하고, 전 구간 PCG
`matrixProducts` 총합이 Cold 대비 감소해야 통과한다. 벽시계(ms)는
CI 런너 편차 때문에 회귀 조건으로 삼지 않고 Release 벤치마크에서
별도로 비교한다. 빠른 물체/급격한 점유 변화에서는 단순히 이전 압력을
재사용할 수 없으므로, Field 레벨의 기하 변화 회귀는 Warm Start
거부 시 원본 Cold 반복 횟수를 보존하는 조건을 계속 검증한다.
