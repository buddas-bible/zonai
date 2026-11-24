# R3a — 결정론 CI 게이트 분리와 Draft PR 통합 준비

> 2026-11-24 기준. `master`: `3f6dc69545d62d7c5a0ebf7ebe83a67454b8ea8d`. R2a-9 [#341](https://github.com/buddas-bible/zonai/pull/341) `2c55d444`에서 분기한 후속 stacked Draft이다. **이 PR은 병합하지 않고 검증 체계를 분리**하는 작업이다.

## 왜 분리하는가?

R2a-9에서는 Ubuntu와 Windows에서 World Replay 61/61, 표준 Joint 서브스텝 641,266행, Contact/Island 389,049행, Contact lifecycle 217,173행을 **원시 비트로 일치** 확인했다. Spline Rail transient/영속 물리 상태도 비교된 테스트에서 차이 0이었다. 그러나 이전 workflow는 다음 두 종류를 모두 동일한 `compare` job에서 실패로 기록했다.

1. **권위 있는 물리 결과 및 engine-owned math:** 실제 Body/Contact/Joint 내부 상태, Spline Rail solver state, `zonai.*`, `rot2.*`, `sqrtf`, `fp.mulAddSeparate` 등. 하나라도 차이 나면 **반드시 fail**해야 한다.
2. **원본 외부 수학 라이브러리 연구용 probe:** `libm.*` fingerprint 및 Spline Rail `diagnostic.tangentAtan2`, `diagnostic.relativeAtan2`, `diagnostic.angleRemainder`. 이 값들은 **그 자체로** 해당 Spline replay의 solver state가 아니며, 플랫폼 간 차이 발생 원인을 관찰하기 위한 비교이다. 지금까지 `libm.hypot`×2, `libm.railAtan2`×3, `libm.acos`×1, Spline read-only 진단×222 차이가 확인되었다.

같은 job의 빨간 상태만으로는 **실제 solver state diverged**와 **진단용 libm 샘플만 다름**을 구분하기 어려웠다.

## 새로운 workflow 계약

`.github/workflows/determinism.yml`:

| Job | 목적 | 실제 차이 발생 시 |
| --- | --- | --- |
| `ubuntu-trace` / `windows-trace` | 동일 입력/동일 커밋으로 각 환경에서 TSV·Replay 생성 | 실행·테스트 실패 시 fail |
| **`compare`: Required strict engine physics and math bits** | World Replay 61개, Contact lifecycle/Island, 표준 Joint, Spline Rail의 **비진단 필드 전체** 및 `libm.`이 아닌 엔진 수학 fingerprint | **1비트 차이여도 fail** |
| **`external-libm-audit`: Strict external libm research fingerprint (known divergences)** | 양 OS에서 직접 호출한 `libm.*`, Spline read-only 표준 라이브러리 각도 진단 | **1비트 차이여도 fail** (기존 6 + 222 차이로 현재 red 예상) |

권위 결과 comparator에서 Spline `diagnostic.zonaiTangentAtan2`, `diagnostic.zonaiRelativeAtan2` 및 다른 알 수 없는 `diagnostic.*`는 계속 **strict** 검증한다. 외부 함수의 세 구체 필드만 진단으로 분류한다. FP는 **`libm.` prefix만 연구용**으로 분리하고 나머지는 모두 strict로 남긴다. 오차 허용치·미검사·성공으로 위장하는 `continue-on-error`를 사용하지 않는다.

기존 로그와 비교 보고서에는 외부 차이를 **경고 및 개수**로 남기고, 별도 `external-libm-audit` job이 **실패 상태와 독립 Artifact**를 보존한다. 따라서 workflow 전체는 여전히 red일 수 있지만, 실물리 실패인지 연구용 libc 진단 차이인지 **Job 수준에서 정확하게 판단**할 수 있다.

### 이 분리는 전체 엔진 결정론 보증이 아니다

표준 라이브러리가 실제 생산 코드에서 **여전히 사용되기 때문**이다. 현재 추가로 확인한 예:

- `src/math/rot2.h`: `std::cos(float)`, `std::sin(float)`. 이 경로는 생산 엔진에 있으므로 `rot2.cos/sin` probe가 필수 strict gate에 포함된다.
- `src/fluid/fluidField2.cpp`: `std::hypot(double)` 호출 다수, `std::asin(double)`, `std::atan2(double)`, `std::atan2(float)`, `std::remainder(float)`. 유체 cut-cell pressure/coupling 등 **아직 World replay에서 직렬화하지 않은 경로**는 실제 플랫폼 차이가 전파될 가능성을 배제할 수 없다.
- `libm.acos`의 현재 probe 차이만으로 생산 엔진에서 직접 호출됨이 입증된 것은 아니다. 확인된 코드 기준 `src/fluid/fluidField2.cpp`에는 `std::acos` 직접 호출이 없었다.

따라서 후속 작업은 **유체 경계/압력 권위 상태와 실제 유체 수학 호출의 FP 감사**이다. 현재 외부 함수 차이를 테스트에서 완전히 제거하거나 오차 범위를 늘려 통과시키지 않는다.

## PR #328~#341 의존성 및 병합 준비

GitHub compare API로 검증:

- **#328 R0**: `master` 기반 문서 감사 (`docs/README.md`, `docs/engine-stability-refactor-r0.md`).
- **#329 R1a**: 동일 `master`에서 분기. #328을 포함하지 않는다. 서로 다른 파일을 수정하므로 현재 관측된 직접 파일 충돌은 없지만, **부모·자식 관계도 아니다**. `#328→#329`라고 Git 히스토리를 오해하면 안 된다.
- **#330~#341**: 각 후속 브랜치가 바로 앞 브랜치의 커밋을 포함하는 **선형 스택**. 모든 PR은 `master`를 base로 표시해 누적 diff가 보인다. 먼저 코드상 delta를 개별 비교한 뒤 순서대로 통합 단위 재구성이 필요하다.
- **#335와 #336**: R2a-4 Spline 실물리 divergence가 R2a-5에서 수정됐다. 따라서 #335를 **독립 통합 완료 상태로 간주해서는 안 된다**. 변경 사항을 #336의 회귀 수정과 하나의 검증된 통합 단위로 결합하거나, 다른 PR에 의존한 실패 기간을 방지하는 순차 검증 계획이 필요하다.
- **#332 이후 적색 CI**는 이전에는 실물리 상태·표준 라이브러리 probe를 혼합했다. 이 PR부터 Job 단위로 해석 가능하지만 **이전 PR head는 자체 workflow/검증 상태를 다시 확인**해야 한다.
- 별개의 유체 small cut-cell 실험 [#326](https://github.com/buddas-bible/zonai/pull/326)은 이 스택과 혼합하거나 임의 병합하지 않는다.

### 권장 통합 절차

1. R0 (#328) 문서 변경을 검토·확정한 뒤 R1a (#329)와의 분기 관계를 해결한다. 필요하다면 #329에 R0을 rebasing하여 선형 통합 기준을 확보한다.
2. R1a/R1b/R1c (#329~#331)를 최소 단위로 검증하고, 이후 #332~#334의 FP/기하 결정론 변경을 자체 회귀와 함께 검토한다.
3. #335+#336은 **물리 불일치 재현 및 수정이 함께 있는 단위**로 관리한다. 이후 Joint/Contact 테스트(#337~#341)를 차례대로 통합한다.
4. 본 R3a의 엄격한 **엔진 계산 검사**가 반드시 green인지, **외부 연구용 job**이 왜 red인지 구분하여 리뷰한다. 외부 수학 호출 중 실제 엔진 권위 상태에 영향을 주는 경로는 별도 회귀/수정하기 전까지 **플랫폼 결정론 미검증**으로 남긴다.
5. 각각의 병합 후 base/diff/CI를 재평가한다. 브랜치를 무조건 모두 삭제하거나 현재 Draft들을 master로 일괄 병합하지 않는다.

## 이번 PR 검증 조건

- Windows/Ubuntu CMake Debug 전체 CTest 및 Windows Release 타깃 검증 유지.
- `compare` job은 기존 World/Joint/Contact/Spline 모든 권위 필드와 engine-owned math를 raw-bit로 strict gate.
- `external-libm-audit`은 독립적으로 기존 불일치를 **실패·비트 샘플·개수/파일**로 보존.
- **새로 발견되는 물리 상태 또는 deterministic zonai math의 1 ULP 차이도 통과 불가**.
- `master`, Fluid solver 및 이전 미병합 PR 본문 코드를 변경하지 않음.
