# Spline rail reprojection inspector implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline, test-driven-development and one fresh final review.

**Goal:** Expose the existing bounded global reprojection API in the existing spline rail inspector.
**Architecture:** Keep the 0.5m editable limit and last request result in rigidBodyDemoUi. Invoke world::reprojectSplineRailJointWithinDistance only on explicit button activation; refresh data before existing controls. No new physics/model API or demo.
**Tech Stack:** C++ / Dear ImGui / CMake / CTest.
**Spec:** Approved conversation design: distance input defaults to 0.5m; explicit reprojection is available during tracking failure, does not teleport bodies, preserves controls/editor/playback and reports acceptance or rejection. Failure preserves tracking/cache/sleep; changed success uses existing reset/wake policy; identical success is a no-op.

## Constraints and review focus
- Inputs must reach the bounded global API unchanged, including exact zero and rejection of negative/nonfinite values; never substitute unbounded projection.
- Last-result text describes the last explicit request, not continuous validity; no automatic retries during rendering or input editing.
- Refresh distance/speed/rotation output in the same frame after the action.
- Invalid tracking must not disable recovery. Crossings retain their branch during idle frames and use global selection only on activation.
- UI state belongs to each demo view and starts fresh when that view is recreated. Geometry undo/redo excludes these controls.

## Task 1: TDD UI action
- [x] Add actual button RED for changed projection and default 0.5m rejection; run sandboxDemoTests and inspect the expected assertion.
- [x] Add minimal UI input, action and last result, then focused GREEN.
- [x] Cover actual input editing, < / == / > boundaries including nextafter, zero, negative and nonfinite rejection, no automatic retries and fresh-view defaults.
- [x] Cover failed tracking recovery, global crossing selection, no-op and failure state preservation and same-frame display.
- [x] Cover physical state/settings/selection/playback/history preservation and successful resume.

## Task 2: Validation
- [x] Document UI semantics.
- [x] Fresh read-only review; resolve material findings with regression tests.
- [x] Full Debug/Release build and CTest; expected 85/85 in each configuration.
- [x] CRLF, whitespace and final diff checks. Leave changes uncommitted for separate integration request.

Evidence: actual activation failed RED at the changed-projection assertion, then passed after minimal UI wiring. Extended focused tests passed for adjacent-float 0.5m boundaries, edited finite/zero/negative/nonfinite inputs, loaded no-op and failed request cache/sleep, fresh-view defaults, tracking recovery and resumed motor movement at substeps 1/4/8, crossing branch selection, origin/settings and selection/playback/history preservation.
Fixture correction: SetBodyTransform clears joint impulses by design, so loaded-cache preservation is tested without teleport; boundary teleport cases check zero-cache preservation and wake behavior. No production workaround was needed.

Final verification: Debug and Release full builds succeeded and CTest passed 85/85 in each configuration. No compiler warnings/errors matched either completed build log. Fresh read-only review reported no actionable findings. CRLF and git diff --check passed. Real ImGui controls were driven headlessly; native desktop mouse/visual inspection was not performed. Changes remain uncommitted on feature/spline-rail-reprojection-inspector.
