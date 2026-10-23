# Spline rail translation inspector implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline and request one fresh review.

**Goal:** Connect the existing translation rebase API and actual rail speed query to the existing inspector.
**Architecture:** Add one button using rebaseSplineRailJoint, refresh its data before distance/speed and later target/limit sliders. Reuse the existing headless ImGui test harness in spline_rail_inspector_test.cpp. No physics/model/API or demo additions.
**Tech Stack:** C++ / Dear ImGui / CMake / CTest.
**Spec:** Approved conversation design: reset current translation to zero while shifting target/limits together, preserve future motion, physical/cache/sleep/rotation/editor/selection/playback state; failure leaves old values.

## Constraints and review focus
- Do not manually zero data, clear impulses or wake bodies; let the existing validated API commit the change.
- Tracking failure and numerical rejection must preserve visible and stored values.
- Refresh distance, speed, target, bounds and slider endpoints in the same frame.
- Display actual speed independently of motor target, including negative values and invalid-tracking zero.
- Repeated idle drawing and repeated zero reset must preserve exact state and pending Undo/Redo.

## Task 1: TDD UI wiring
- [x] Add real button activation RED, connect API with refresh, confirm GREEN.
- [x] Add actual-speed display RED, add live m/s text, confirm GREEN.
- [x] Check target/limit/slider bounds and pending history/selection/playback preservation.
- [x] Compare subsequent motion to an unre-based independent scene; test invalid tracking and numerical failures.

## Task 2: Complete
- [x] Document UI semantics and limitations.
- [x] Fresh read-only review, resolve actionable findings.
- [x] Full Debug/Release build/CTest and CRLF/whitespace checks.
- [x] Leave uncommitted pending separate integration request.

Evidence: actual button RED and signed speed display RED, then focused GREEN. Motion equivalence checks cover four angular configurations x three axial controls x substeps 1/4/8, with 240 post-button steps and motor reversal. Failure fixtures cover stored tracking failure and collapsed float endpoint bounds.

Final verification: Debug and Release full builds succeeded; CTest passed 85/85 in each configuration. No compiler warnings/errors matched the build logs. Fresh read-only review found no actionable issues. Changes remain uncommitted.
