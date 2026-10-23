# Spline rail rotation enable implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline and request one fresh review at the end.

**Goal:** Disable and re-enable the angular constraint independently of rotation mode and tuning.
**Architecture:** Carry default-true enableRotation through def/sim/data and a dedicated setter using resetSplineRailJoint. Solve only the normal row when disabled and discard angular warm-start impulse. Preserve existing enabled solver arithmetic.
**Tech Stack:** C++ / CMake / CTest.
**Spec:** User-approved design in this conversation: free rotation, stored-reference reactivation, moving/curved/seam rails and unchanged axial motor/spring/limit; changed flag clears caches/wakes, exact no-op preserves sleep. No new demo.

## Constraints and review focus
- Noncentral anchor reaction torque must remain physical; disabling rotation does not zero all torque.
- Store mode, both references and angular tuning unchanged across toggles.
- Angle and angular-speed queries retain their stored-reference meaning while disabled.
- Preserve default-true behavior, closed winding/branch, geometry editing and failure policy.
- Validate stale angular warm-start, degenerate effective mass, sleep and exact body-state preservation.

## Task 1: TDD free rotation and reactivation
- [x] Add def/sim/data flag and public setter declaration; register focused tests.
- [x] Observe free-rotation RED; implement disabled normal-only solver and warm-start handling; confirm GREEN.
- [x] Observe toggle RED; implement changed/no-op setter using existing reset; confirm GREEN.
- [x] Add curve/seam/moving rails, axial controls, noncentral anchor and preservation tests.

## Task 2: Verify
- [x] Document semantics, including retained reaction torque and queries.
- [x] Fresh read-only review; address actionable findings.
- [x] Full Debug/Release builds and CTest; CRLF/whitespace checks.
- [x] Leave uncommitted pending separate integration request.

Evidence: free-spin RED, then runtime-toggle RED, followed by focused GREEN. Expanded tests passed for all three saved angular configurations and substeps 1/4/8. Ruling: geometry-edit travel uses 1e-6 m tolerance, consistent with existing projection precision; observed delta was 1e-9 m.

Final verification: full Debug and Release builds succeeded; CTest 85/85 each. Fresh review found no actionable issues. No new compiler warnings, CRLF and whitespace checks passed; no demo changes.
