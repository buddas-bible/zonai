# Spline rail Alt-click segment selection implementation plan

> **For agentic workers:** Use superpowers:executing-plans inline and one fresh final review.

**Goal:** Select a reprojection segment directly with Alt+left click within 10 screen pixels of the real curve.
**Architecture:** Add a modifier flag to the existing sandbox demoInput and route KeyAlt from the host. The UI view overrides handleInput/cancelInput to consume only the selection mouse gesture. Query current world spline in rail-local coordinates through existing ClosestPoint with the same 0.01px distance accuracy as Ctrl insertion. Update existing scope/index only on a valid hit; existing list, overlay and explicit reproject button remain consumers. No new physics API or demo.
**Tech Stack:** C++ / Dear ImGui / CMake / CTest.
**Spec:** Approved conversation: 10px inclusive, zoom independent; out-of-range or failed query leaves scope/index unchanged; successful hit selects segment mode and updates list/highlight without reprojection. Existing nearest-point tie policy. Ctrl insertion wins over Alt. Consume Alt-started left gesture until release, not keyboard/right inputs. Focus/pause cancellation clears pending gesture.

## Review focus
- Use current world geometry/isLoop and inverse bodyA transform; distance is to the curve, not control polygon or body origin.
- Reject invalid scale/radius/query without accidental drag or changes to mode/result/history; keep last explicit reproject result.
- Ctrl+Alt starts normal insertion; pressing/releasing modifiers during a held gesture must not switch its purpose.
- Non-rail demos and ordinary point/body drags retain existing behavior; unrelated simultaneous inputs still reach the model.
- No query writes, wakeups, implicit reproject, origin/settings/branch changes, editor selection changes, history entries or physics steps.

## Task 1: RED and GREEN
- [x] Add input contract flag and real ImGui mouse/modifier event test; run sandboxDemoTests and observe missing selection RED.
- [x] Wire host flag, UI input routing/gesture cancellation and closest-point selection; focused GREEN.
- [x] Verify list/overlay/button linkage, inclusive 10px boundaries at multiple zooms, curve interior and rail transforms, crossing/tie policy.
- [x] Verify click/hold/modifier conflict and focus/pause cancellation, other demos and existing drag/insertion behavior.
- [x] Verify current geometry edits/undo/redo and loaded sleep/cache/settings/origin/branch/editor/playback/history preservation on hit/miss/failure.

## Task 2: Complete
- [x] Document controls and fresh read-only final review.
- [x] Full Debug/Release builds and CTest (85/85 each expected).
- [x] CRLF/whitespace/final diff checks; leave uncommitted for separate integration request.

Evidence: real ImGui modifier/mouse events initially failed the inspector selection assertion RED. Corrected fixture event ordering (modifiers before button press, matching real Alt-click) and reconfirmed RED with UI selection disabled, then GREEN after restoring/rebuilding the handler. Focused Debug sandboxDemoTests passed at 9/10/11 pixels across zooms 10/20/55/60/80/160/240 in global/selected modes, cubic-vs-chord/transformed hits, crossing/ties/world geometry, list/highlight/button linkage, modifier holds/Ctrl priority/body and point drags, focus/pause/other demos and simultaneous keyboard input, loaded state/failure and geometry undo/redo preservation.
Fixture notes: explanatory copy avoids a pre-existing selector-label absence assertion collision. Restoring the RED probe source required updating its timestamp to rebuild the restored GREEN implementation. No physics or numerical workaround was needed.

Final verification: Debug and Release full builds succeeded; CTest passed 85/85 each. Neither completed build log matched compiler warning/error patterns. Fresh read-only reviewer found no actionable findings. CRLF and git diff --check passed. Real ImGui input events were forwarded through demoSession and draw meshes inspected headlessly; native OS mouse/visual inspection was not performed. Changes remain uncommitted on feature/spline-rail-alt-selection.
