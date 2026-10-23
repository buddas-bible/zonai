# Spline rail angular error speed implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline; request one whole-branch review.

**Goal:** Expose currentAngularSpeed as the instantaneous rate of currentAngle in rad/s.
**Architecture:** Extend splineRailJointData and its const world query. Use the existing solver angular row: relative angular velocity for fixed mode, curvature and the accumulated-distance row for tangent mode. Keep solver and currentSpeed unchanged.
**Tech Stack:** C++ / CMake / CTest.
**Spec:** User-approved design in the conversation: straight/curved/seam/moving rails, angle finite-difference agreement, invalid tracking yields zero, read-only query preserves all state. No demo.

## Constraints and review focus
- Preserve existing angle wrapping, creation/mode/reference policies, and CRLF style.
- Validate off-center anchors and centers of mass, lateral projection correction and open-end extension.
- Validate forward/reverse seam travel, both rotation modes and hard/spring tuning.
- Repeated query must preserve sleep, cache, branch/winding, pose and velocity, and future simulation.
- Stored solver tracking failure and current invalid geometry must both yield zero.

## Task 1: TDD query field
- [x] Add tests/dynamics/spline_rail_angular_speed_test.cpp and register it in tests/CMakeLists.txt.
- [x] Add the default-zero data field and observe RED on nonzero straight relative angular velocity.
- [x] Implement the existing angular-row velocity in getSplineRailJointData and confirm GREEN.
- [x] Add differential, seam and state preservation checks; confirm focused GREEN.

## Task 2: Complete verification
- [x] Document instantaneous rate and invalid-tracking policy in docs/closed-spline-rail.md.
- [x] Request fresh read-only review and resolve actionable findings.
- [x] Full Debug/Release builds and CTest, whitespace/CRLF checks.
- [x] Leave implementation uncommitted for a separate integration request.

Evidence: RED straight relative spin, then RED curved angle differentiation without curvature correction; focused GREEN passed. Ruling: use 0.1 ms central differences at the symmetric seam; 1 ms truncation error was 0.015 rad/s and shrank below the unchanged 0.004 rad/s tolerance. Other differences retain 1 ms.

Final evidence: full Debug and Release builds succeeded; CTest 84/84 in each configuration. Reviewer found no actionable issues and independently passed the focused Debug test. No solver or demo changes; no new compiler warnings; CRLF and whitespace checks passed.
