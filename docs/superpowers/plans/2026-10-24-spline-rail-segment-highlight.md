# Spline rail selected segment highlight implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline and one fresh final review.

**Goal:** Show the selected reprojection segment on the existing spline rail canvas.
**Architecture:** In rigidBodyDemoUi::draw, read current joint geometry and bodyA transform. Sample only [selected segment, 0..1] using existing 0.01m adaptive sampling, then draw a thick bright curve, endpoint markers and one-based segment label. Reuse existing selection state. No new demo, physics/model API, or persistent geometry cache.
**Tech Stack:** C++ / Dear ImGui / CMake / CTest.
**Spec:** Approved conversation: highlight only a valid selected segment in selected mode; hide in global mode or on invalid selection/sampling failure. Follow edits and Undo/Redo. Drawing must not reproject, wake, step, or modify physics/control/editor state.

## Review focus
- Use actual world geometry and isLoop, not a stale editor span; transform from local rail coordinates through bodyA exactly once.
- Hide stale highlight after deletion; Undo/Redo refreshes without changing the selected segment number.
- Partial sampling must include only the chosen segment and both endpoints; use the existing 1cm error policy.
- Drawing must not alter stored tracking branch, failed tracking state, cache/sleep, origin/settings, selection, playback or edit history.
- Canvas overlay must distinguish crossing branches; existing path animation and base rail rendering stay intact.

## Task 1: RED and GREEN
- [x] Drive actual segment list; capture real ImGui draw vertices and fail RED when selected highlight is absent.
- [x] Add minimal read-only overlay and confirm GREEN.
- [x] Verify current world geometry, endpoints/label/interior curve and both crossing branches, including translated/rotated rail coordinates.
- [x] Verify global/invalid suppression, real edits and Undo/Redo, repeated drawing and physics/editor/playback/history preservation.

## Task 2: Complete
- [x] Document semantics and request fresh final read-only review.
- [x] Run full Debug/Release builds and CTest (85/85 each expected).
- [x] Check CRLF/whitespace and leave uncommitted for separate integration request.

Evidence: actual segment selection produced no highlight and failed RED; the read-only draw overlay made it GREEN. Focused Debug sandboxDemoTests passed with direct ImGui mesh checks for endpoints/marker size/thick stroke/label, independently calculated cubic interior points and ninety-degree transform, separate crossing branches and world/editor geometry distinction, real drag/delete/undo/redo, invalid/global hiding, loaded cache/sleep/settings/origin/playback/history preservation, failed tracking and failed sampling.
Fixture correction: texture-antialiased lines emit mesh edges beyond the visible centerline; drawing tests allow the stroke mesh footprint while still distinguishing the cubic from its chord. Production sampling/error policy was unchanged.

Final verification: Debug and Release full builds succeeded; CTest passed 85/85 each. Neither completed build log matched compiler warning/error patterns. Fresh read-only review found no actionable findings. CRLF and git diff --check passed. Drawing validation used real ImGui meshes headlessly; native desktop visual inspection was not performed. Changes remain uncommitted on feature/spline-rail-segment-highlight.
