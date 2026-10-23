# Spline rail rotation inspector implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline and request a fresh review at the end.

**Goal:** Connect existing rotation APIs to the existing spline rail inspector.
**Architecture:** Direct ImGui controls in rigidBodyDemoUi.cpp call existing world APIs, re-query after changes so the reference and status are current. Keep physics/model unchanged. Add a separate headless ImGui test source to sandboxDemoTests using its existing initialized context.
**Tech Stack:** C++ / Dear ImGui / CMake / CTest.
**Spec:** Approved conversation design: enable toggle, mode-specific radians reference editing, current-pose reference button, currentAngle/currentAngularSpeed display; retain mode/tuning while disabled, selection/playback/spline history, no new demo.

## Constraints and review focus
- Free mode keeps reference/mode/tuning editable and does not implicitly re-enable rotation.
- Switching modes must re-query the newly captured reference before editing it.
- Reference input is finite and unbounded like the existing API; no per-frame writes or automatic normalization.
- Rebase button must respect tracking failure and refresh displayed values after success.
- Preserve physical state and geometry history; use existing widget sizes and CRLF style.

## Task 1: TDD inspector controls
- [x] Add tests/sandbox/spline_rail_inspector_test.cpp to the existing UI test target/context.
- [x] Observe actual widget activation RED for rotation enable; connect minimal checkbox and confirm GREEN.
- [x] Add RED reference edit/rebase tests, connect existing APIs, refresh state and display rate/error.
- [x] Verify both modes, hard/spring controls, free/active state, failure and no-interaction no-op.

## Task 2: State preservation and verification
- [x] Exercise actual controls with playing/paused sessions and pending Undo/Redo; verify physical/selection/editor state.
- [x] Fresh read-only review and fix actionable findings.
- [x] Full Debug/Release build and CTest, whitespace/CRLF checks.
- [x] Leave uncommitted pending separate integration request.

Evidence: independent RED failures for toggle, reference input, rebase button and displayed error/rate; all GREEN after direct existing-API wiring. Test harness opens the collision-material section before its selection-preservation probe.

Final evidence: full Debug/Release builds succeeded, CTest 85/85 in each configuration. Fresh review found no actionable findings; new tests drive real ImGui widget activation and text entry in the existing headless context. Native desktop mouse/focus routing and rendered-window visual inspection were not automated. CRLF and whitespace checks passed with no new compiler warnings.
