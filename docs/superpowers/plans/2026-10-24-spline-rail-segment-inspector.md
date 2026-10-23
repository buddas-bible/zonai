# Spline rail segment reprojection inspector implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline and one fresh final review.

**Goal:** Expose bounded segment reprojection alongside existing global reprojection in the existing inspector.
**Architecture:** Store global/selected mode and a zero-based segment index only in rigidBodyDemoUi. Derive the segment list from world geometry. Display one-based segment numbers; route explicit action to the existing matching overload. No physics/model API or demo changes.
**Tech Stack:** C++ / Dear ImGui / CMake / CTest.
**Spec:** Approved conversation: global by default, segment dropdown, shared allowance/button, no fallback on segment failure, no automatic projection when selecting. Invalid selection after geometry edits requires reselection; ordinary connected tracking resumes after explicit projection. Preserve editor/playback/settings and existing success/no-op/failure cache/sleep policy.

## Review focus
- Use world points and isLoop for current segment count; do not silently clamp invalid selected index after edits.
- Selection is a current geometry segment number, not a persistent segment identity across insertion/removal.
- Global mode ignores an old invalid selected index. Recovery remains available with invalid tracking.
- Segment dropdown is separate from control-point selection. Same-frame action uses the newly selected mode/index.
- Pass allowance unchanged to the correct bounded overload; preserve last explicit result during selection and idle drawing.

## Task 1: RED and GREEN
- [x] Add actual mode/segment popup activation test at a crossing; expect later branch on button; run sandboxDemoTests and inspect RED.
- [x] Implement minimal selectors and bounded overload dispatch; focused GREEN.
- [x] Cover both crossing branches, global fallback only by explicit mode selection and continued connected tracking.
- [x] Cover segment distance below/equal/above 0.5m including a closer other segment; no automatic fallback.
- [x] Cover actual geometry editing/undo/redo, invalid selection and reselection, playback/selection/control state preservation and failure cache/sleep.

## Task 2: Complete
- [x] Document semantics and run fresh final read-only review.
- [x] Full Debug/Release builds and CTest: expected 85/85 each.
- [x] Check CRLF/whitespace and final diff; leave uncommitted for separate integration request.

Evidence: actual target list failed RED because the widget was missing; selectors and bounded overload dispatch made the crossing test GREEN. Extended focused tests passed for both crossing branches/global selection, adjacent-float distance boundaries, closer-neighbor rejection, loaded failure/no-op cache and sleep, substeps 1/4/8 recovery and connected travel, real editor deletion/undo/redo/reselection, playback/shape/control selection/history and settings/origin preservation. Current geometry segment numbering across edits follows the approved design; no persistent segment IDs are added.

Final verification: Debug and Release full builds succeeded; CTest passed 85/85 in each configuration. Fresh read-only reviewer found no actionable findings. World/model segment-count distinction, new-view defaults and both-mode nonfinite/negative/zero input checks also passed. CRLF and git diff --check passed. Tests drove real ImGui controls headlessly; native desktop mouse/visual inspection was not performed. Changes remain uncommitted on feature/spline-rail-segment-inspector.
