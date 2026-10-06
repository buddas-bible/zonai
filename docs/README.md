# Documentation

현재 감사 완료 범위와 다음 작업은 [감사 현황](audit-status.md)에서 확인한다. 개별 보고서의 다음 작업/미병합/CI 대기 문구는 작성 당시의 기록이며, 최신 진행 순서는 감사 현황을 따른다.

감사 상세 기록:

- [Part 4 — DynamicTree / BroadPhase](box2d-broadphase-parity-audit.md)
- [Part 5 — Body / Shape / Contact](part5-body-shape-contact-audit.md)
- [Part 6 — Contact Solver](part6-contact-solver-audit.md)
- [Part 9 — World / Public API / 파일 구조](part9-world-api-structure-audit.md)
- [Part 10 — Tests / Sandbox / Build / Docs](part10-tests-sandbox-build-docs-audit.md)
- [최종 통합 확인 / Joint 진입 조건](final-integration-joint-readiness.md)

- [Island / Sleep / CCD 통합](island-sleep-ccd-audit.md)
- [World Query / Sensor API](world-query-sensor-audit.md)
- [Contact recycling/cache](contact-cache-audit.md)

선택형 Sandbox: [설계](superpowers/specs/2026-10-06-sandbox-demo-design.md), [데모 추가와 학습 방향](sandbox-demos.md). Mouse Joint: [수학·수명·조작·검증](mouse-joint.md). Distance spring: [Hertz·감쇠·진자 실험](distance-spring.md). 다음 구현은 Distance limit이다.

첫 Joint: [기본 Distance Joint 설계](superpowers/specs/2026-10-06-distance-joint-design.md), [구현·학습·검증 기록](basic-distance-joint.md).

기존 설계와 실행 계획은 `superpowers/specs`와 `superpowers/plans`에 보존한다. 구현 세부 사항은 코드 가까이에 두고, 이 디렉터리에는 근거·결정·검증 범위와 제한을 남긴다.
