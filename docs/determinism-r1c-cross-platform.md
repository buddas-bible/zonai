# R1c — Windows / Ubuntu 비트 단위 World Replay 비교 CI

> 의존: [R1a #329](https://github.com/buddas-bible/zonai/pull/329) → [R1b #330](https://github.com/buddas-bible/zonai/pull/330) → **R1c**.
> 기존 `master`가 아닌 R1b의 최신 커밋에서 시작하는 **stacked Draft PR**이다. 병합 순서를 반드시 지킨다.
> 이 단계에서는 `src/`, `sandbox/`, 물리 수치 알고리즘과 기본 설정은 수정하지 않는다.

## 1. 목표

서로 다른 플랫폼이 각자 생성한 파일을 자기 환경에서만 검증하는 R1b를 확장해, **완전히 같은 기록 입력으로 Windows MSVC 및 Ubuntu GCC가 생성한 매 Tick의 World/Body 관측 스냅샷을 직접 비트 비교**한다.

워크플로: [`.github/workflows/determinism.yml`](../.github/workflows/determinism.yml).

## 2. 3개 CI 작업

1. **ubuntu-trace** — Ubuntu 24.04 x64/GCC Debug에서 `worldReplayTests`만 빌드. 고정 테스트 장면에 대해 `--record-input`으로 `shared.zin`을 생성, `--replay`로 `ubuntu.ztr` 생성, `--verify`로 해당 플랫폼 내 검증. 원본 입력과 Linux Trace/비교 실행 파일을 별도 artifact로 업로드한다.
2. **windows-trace** — Ubuntu가 업로드한 **동일한 `shared.zin` 파일을 다운로드**한다. Windows x64/MSVC Visual Studio 2026 Debug에서 독립 빌드한 `worldReplayTests`로 해당 입력을 재생해 `windows.ztr`을 만들고 `--verify`를 실행한다. Windows가 별도로 예제 입력을 재생성하지 않는다.
3. **compare** — 두 Trace를 다운로드한다. Ubuntu에서 빌드한 동일 `worldReplayTests --compare ubuntu.ztr windows.ztr`을 사용해 **입력 바이너리가 동일한지 먼저 검사하고, 그 뒤 Tick 0부터 모든 직렬화 필드의 전체 바이트를 비교**한다. 첫 불일치 시 비영(非0) 종료하여 필수 CI가 실패하고 `comparison.txt`를 artifact로 남긴다.

업로드 artifact:

| 이름 | 포함 항목 |
| --- | --- |
| `r1c-shared-input` | `shared.zin` |
| `r1c-ubuntu-trace` | `ubuntu.ztr`, Ubuntu `worldReplayTests` 바이너리 |
| `r1c-windows-trace` | `windows.ztr` |
| `r1c-comparison-report` | `comparison.txt` (통과 또는 최초 실패 정보) |

R1c는 `pull_request` (대상 `master`)와 `workflow_dispatch`에 의해 실행된다. 이 PR은 기존 R1a/R1b가 미병합이어서 `master`를 임시 대상으로 열고, 실제 PR diff에 선행 변경도 포함되므로 **그대로 병합하면 안 된다**.

## 3. 합격/실패 해석

- **비교 작업 성공**: 현재 **R1a의 제한된 공개 World/Body Snapshot v1 + R1b의 60-Tick 고정 장면**이, 해당 CI 환경의 Windows/MSVC와 Ubuntu/GCC에서 매 Tick 비트 단위로 일치했다는 근거.
- **비교 작업 실패**: 입력 바이트가 다른지, Tick/필드 raw bits가 처음 달라지는지 기록한다. 실패를 허용 오차 비교, `continue-on-error`, `if: always()` 비교 스킵, 예외 정규화 등으로 **녹색으로 바꾸지 않는다**.
- **다른 CI 잡 실패**: 컴파일/파일 생성/아카이브 다운로드 문제인지, 실제 비트 불일치인지 작업 이름과 로그로 구분한다.
- **기능 전체에 대한 판정 불가**: 결과가 같아도 모든 Contact/Joint/CCD/DynamicTree 내부 이력, Body pending forces, shape geometry, Fluid 압력·속도·Tracer/Cut-cell·warm start 등은 현재 Snapshot v1에 포함되지 않는다. 나아가 플랫폼 계약은 컴파일러 버전·FP 정책·CPU 기능·Debug/Release별로 확대·고정해야 한다.

**핵심:** R1c가 적색으로 나오더라도 이 단계에서는 곧바로 물리 계산을 임의로 변경하지 않는다. 먼저 실제 불일치 위치를 수집하고 R2에서 관련 수학 함수·연산 순서·상태를 제한된 PR로 수정한다.

## 4. 검증 범위와 다음 단계

현재의 게임용 엔진 목표는 **플랫폼 간 권위 물리 상태 전체의 완전한 비트 결정론**이다. 따라서 다음 작업은 R1c 결과를 확인해 첫 divergence를 재현하고, 동시에 Snapshot v1의 누락 필드를 점진적으로 확장하는 것이다.

1. R1c 결과가 실패하면 첫 Tick과 필드, 플랫폼 도구 버전, 재생 파일을 회귀 증거로 보존.
2. R2a: 컴파일러 부동소수점 설정·수학 함수와 물리 연산 순서에서 해당 divergence의 원인을 규명.
3. R2b: Contact/Joint/CCD/BroadPhase 등 내부 권위 상태를 독립 스냅샷으로 확장.
4. R2 및 S2: Fluid 압력·장벽·연성의 수치적 안정성은 결정론과 별개로 검증. Issue [#324](https://github.com/buddas-bible/zonai/issues/324) 미해결.

GPU 렌더와 정보성 벤치마크는 현 단계 물리 상태에 영향을 주지 않는 비권위 데이터로 간주한다.
