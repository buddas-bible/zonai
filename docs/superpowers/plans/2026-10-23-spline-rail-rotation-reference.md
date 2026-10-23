# Spline rail rotation reference implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline; request a fresh review at the end.

**Goal:** Set the selected spline rail rotation reference without moving bodies.
**Architecture:** Select referenceAngle for fixed mode or tangentOffset for followTangent. Reuse resetSplineRailJoint on a changed reference; expose tangentOffset in the existing data query.
**Tech Stack:** C++ / CMake / CTest.
**Spec:** User-approved design in this conversation: finite radians, unchanged mode/tuning/travel/body state, exact same reference preserves sleep/cache, invalid input fails atomically. No new demo.

## Constraints and review focus
- Preserve existing mode transitions, creation, rebase, numerical angle-error calculation and CRLF style.
- Test both active and inactive references, transformed/moving rails, curve seams, hard and spring response.
- Test NaN and both infinities, loaded cache, sleeping bodies, repeated settings and wrapped angular errors.

## Task 1: Reference API and query (RED then GREEN)
- [x] Add tests/dynamics/spline_rail_rotation_reference_test.cpp and its tests/CMakeLists.txt target.
- [x] Declare world::setSplineRailJointRotationReference(jointId,float) and data.tangentOffset; use a false-return stub to observe an assertion failure for valid input.
- [x] Implement finite-input guard, active-reference selection, exact no-op and existing reset; expose stored tangentOffset in getSplineRailJointData.
- [x] Confirm focused tests pass for fixed/follow/hard/spring, state preservation and failure cases.

## Task 2: Verify and document
- [x] Document the setter and query in docs/closed-spline-rail.md.
- [x] Review the complete diff with a fresh reviewer, fix actionable findings.
- [x] Full Debug and Release builds and CTest; diff whitespace and CRLF checks.
- [x] Implementation completed; the user subsequently requested commit, PR, CI, merge and branch cleanup.

Evidence: RED query exposure failed, then RED finite-angle assertion failed; focused Debug GREEN passed. Scope stays uncommitted.

Verification: Debug and Release full builds succeeded, CTest 83/83 each; review found no actionable issues; CRLF and whitespace checks passed.
