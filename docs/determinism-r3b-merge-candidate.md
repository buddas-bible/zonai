# R3b — R0 감사 이력과 결정론 검증 스택의 통합 후보

> 2026-11-24 (Asia/Seoul). 이 문서는 **병합 준비 검토용**이다. `master`는 변경하지 않았으며, 기존 Draft PR #328~#342를 임의로 병합·닫거나 브랜치를 삭제하지 않았다.

## 1. 왜 별도 통합 후보가 필요한가?

- [PR #328](https://github.com/buddas-bible/zonai/pull/328)은 `master` `3f6dc695`에서 출발한 R0 **문서 전용** 감사 브랜치다.
- [PR #329](https://github.com/buddas-bible/zonai/pull/329)는 같은 기준에서 별도로 출발한 R1a이고, 이후 **#330~#342**가 부모 커밋을 포함하는 직선형 스택을 형성한다.
- #328은 #329~#342에 포함되지 않아, 후속 PR을 번호순으로 병합하는 것만으로 R0 감사 문서의 이력과 내용이 자동 포함되지 않는다.
- [PR #335](https://github.com/buddas-bible/zonai/pull/335)는 Spline Rail에서 **실제 권위 물리 상태의 크로스 OS 비트 불일치**를 재현했으며, [PR #336](https://github.com/buddas-bible/zonai/pull/336)의 결정론적 `atan2` 수정 이후 관측된 Spline 상태가 비트 단위로 일치했다. **#335를 수정 없는 독립적인 완성 단계로 병합하지 않는다.**

## 2. 이번 통합 후보의 정확한 조합

새 브랜치 `integration/determinism-r0-r3a-candidate`는 [#342 head](https://github.com/buddas-bible/zonai/commit/e227de6527427b3b503e52c03756fd45d2536e4b) `e227de6527`에서 출발했다. Git의 **두 부모를 갖는 merge commit**으로 [#328 head](https://github.com/buddas-bible/zonai/commit/bbf2b0c2d1788759cf856b461a033012d21c0ea6) `bbf2b0c2d1`을 합쳤다. 이때 메인 스택의 기존 소스 트리를 유지하고 R0 문서 두 Blob만 원본 내용 그대로 적용했다.

| 항목 | 검증 결과 |
| --- | --- |
| 조합 브랜치에서 #342까지의 이력 | 모두 조상 커밋으로 보존 |
| 조합 브랜치에서 #328 R0 이력 | 두 번째 부모로 조상 커밋 보존 |
| `docs/README.md` | 원본 #328 Blob SHA `db203dd915d64bea5fb27166bd21a4fb88fc8624` |
| `docs/engine-stability-refactor-r0.md` | 원본 #328 Blob SHA `7ce2d7c5f86f97b1e1bc95e23e4c8ece0765cd61` |
| #342 대비 R0 통합 변경 파일 | **정확히 위 2개 문서**, 프로덕션/테스트/CI 코드 변경 없음 |
| 기존 R3a 결정론 워크플로 Blob | 그대로 보존 |

이후 이 검토용 문서만 추가한다. `master`에는 영향이 없으며, #328과 #329의 기존 커밋 이력을 **재작성·강제 푸시하지 않았다**.

## 3. Spline 수정의 원자적 병합 단위

R2a-3 head `1e40351de9c2ad827c7d3ff64ddc7ecac192af1d` 이후 두 범위를 하나의 통합 검토 단위로 취급한다.

- **R2a-4 / #335**: Spline Rail의 실제 World/Body와 Joint 캐시를 관측해 플랫폼 간 차이를 재현·분리함. 주요 변경은 Spline Joint Constraint와 결정론 테스트/CI다.
- **R2a-5 / #336**: R2a-4에서 발견한 `std::atan2` 원인을 `deterministicAtan2`로 대체하고 실물리 상태의 관측 차이를 제거함. #335에서 변경한 동일 Spline Constraint·테스트 파일을 **다시 수정**한다.
- 따라서 검토 시 `#334 head → #336 head`의 **합쳐진 전체 diff**를 필수로 확인한다. `#335` 단독의 적색 물리 회귀를 합리화하여 먼저 적용하지 않는다.
- #337~#342는 이 수정 이후의 코드 상태에서 검증됐고, 후속 Joint/Contact/Island, 수명주기 회귀와 외부 `libm` 연구용 게이트 분리까지 포함한다.

## 4. 병합 수용 조건 (실제 확인하기 전에는 미완료)

1. 이 통합 후보의 Windows·Ubuntu **전체 CMake Debug CTest**와 Windows Release 선택적 검사 확인.
2. 동일 HEAD에서 **R1c World Replay 61/61**, Contact Lifecycle, Contact/Island Substep, 표준 Joint, Spline Rail 권위/중간/영속 캐시 원시 비트 일치.
3. **엔진/물리 strict CI job이 성공**하고, 외부 `libm` 연구용 CI job이 **기존 비트 차이를 빠짐없이 실패·보고·artifact 저장**하는지 구분 검증. 외부 Job 실패를 성공으로 꾸미거나 무시하지 않는다.
4. R0 감사 본문의 오래된 PR/시각 정보를 현재 코드 기준의 완료 상태로 오해하지 않기: 과거 감사 기록이고 기능 완료 선언이 아니다.
5. 개별 PR의 변경을 재검토하고, 최종 통합 방식(원본 PR 순차 병합 vs 이 통합 PR을 단일 병합 단위로 사용)을 결정한다. **둘 모두 병합해 중복 이력을 만들지 않는다.**
6. Fluid 권위 그리드 결정론, `rot2::FromRadians`의 실사용 경로, 비지원 compiler/CPU/FP환경과 Cut-cell 압력 스파이크는 별도 미해결 작업으로 유지한다.

## 5. 기존 Draft 보전 정책

- #328~#342는 여전히 원본 감사·구현 근거를 보유하는 **열린 Draft**로 남겨둔다.
- 통합 후보는 **검증용**이고 이전 PR을 자동으로 닫거나 대체하지 않는다.
- 모든 Diff/테스트/문서를 확인한 뒤 실제 병합 전략을 정하기 전까지 `master`에 일괄 병합하지 않는다.
- 별도의 [Fluid small-cell 실험 PR #326](https://github.com/buddas-bible/zonai/pull/326)은 혼합하지 않는다.
