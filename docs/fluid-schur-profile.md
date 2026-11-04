# Cut-cell 연립 Schur PCG 단계별 성능 진단 (Part 13z)

`setCutCellPressureTimingEnabled(true)`는 Cut-cell 연립 압력 풀이의 내부 경과시간(ms)을 별도로 수집한다. 기본 OFF. `getCutCellPressure().coupledTiming`은 마지막 Fluid Step의 Schur solve에 대한 진단이고, 해당 Fluid Step에서 결합 압력을 요청하지 않았다면 모든 필드가 0이다. 기존 `coupledSolveMilliseconds`와 같은 전체 구간을 다루며, 전처리기 및 행렬·힘·수렴 허용오차는 변경하지 않는다.

측정 항목: `assembly`(RHS/diagonal + 강체 경계 face clipping/추출), `factorization`(IC(0)의 incomplete LDLᵀ 준비; Jacobi는 계측 오버헤드만), `multiply`(PCG 반복 안의 matrix-free A*v), `preconditioner`(Jacobi 또는 IC(0) 적용), `vector`(R/Z 방향 벡터 갱신, 내적, 수렴 판정), `validation`(종료 시 별도 진짜 잔차 b-A*x 검증), `output`(float 압력/게이지 검증 및 기록). `matrixProducts`는 PCG 반복에 사용된 A*v 횟수이고, `preconditionerApplications`는 처음 1회와 반복마다의 실제 적용 횟수다. `validation`의 추가 A*v는 `matrixProducts`에 포함하지 않는다.

`total`은 Schur solve의 내부 전체 wall-clock 시간이며 `accounted()`는 단계별 합계다. 단계 전환 시계 오버헤드로 부동소수점 오차가 있을 수 있으므로 합계가 완전히 같을 필요는 없다. 수치적으로 실패하거나 iteration limit에 도달해도 **그 시점까지의 실제 수행량**을 보존하며, 성공·실패 상태는 별도의 `coupledStatus`와 `hasAnalyticImpulseBudget`을 사용한다. 타이밍 데이터는 과학적 계측용이며 결과 수렴 여부 판정에 사용하지 않는다.

Sandbox의 **유체 단계별 시간 측정** 옵션은 내부 Cut-cell Schur 시간 계측도 함께 켜고, 프레임에서 교대로 실행된 Fluid substep의 내부 시간을 `getWindCoupledTiming()`으로 모두 합산한다. UI는 이 값을 기존 전체 압력 투영(`getWindFieldTiming().pressure`)의 일부로 보여준다. Field를 리사이즈하거나 비활성화할 때 stale 통계를 지운다.

기존 `fluidPreconditionerBenchmark --csv`와 `sandboxWindStepBenchmark --csv`를 확장해 행렬 구성·IC(0)·A*v·전처리 적용·벡터·진짜 잔차 측정값과 연산 횟수를 출력한다. Windows Release CI는 기존 벤치마크를 정보 제공 전용으로 실행하고 속도 수치로 CI 합격·실패를 결정하지 않는다. **수렴하지 않은 Jacobi와 수렴한 IC(0)의 실행 시간 비율을 성능 개선으로 간주하지 않는다.**

본 변경은 관찰용 프로파일만 추가한 것으로, 빠른 알고리즘/메모리 레이아웃 최적화는 측정 결과를 보고 별도 PR에서 검증한다. 계측이 켜진 실행은 시계 호출 비용으로 인해 순수 실행시간보다 느릴 수 있고, CI 환경은 사용자 실제 PC의 FPS를 대변하지 않는다.
