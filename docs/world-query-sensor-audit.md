# World Query / Sensor API re-audit

Base: Zonai master `09da0eea998c2e12f26c5c645df846093c2ff4a8` after Island/Sleep/CCD.
Reference: Box2D main `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`, fetched 2026-10-05.
Compared `src/sensor.c`, `src/shape.c`, `src/body.c`, `src/physics_world.c` and public headers.
Source: [Box2D sensor update](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/sensor.c).

## Findings and changes

No production correctness defect was confirmed in the reviewed existing APIs. Zonai does not yet expose World spatial overlap/ray/shape queries or custom query filters; this audit does not add those features or infer Box2D query callback semantics for absent APIs.

- Body Contact capacity includes persistent zero-point candidates. Shape capacity conservatively includes sibling-shape contacts, with sensors returning zero. Data queries skip zero-point manifolds and unrelated shapes; pointCount > 0 includes speculative constraints, matching Box2D's contact-touching flag. Empty/short spans return the written count without modifying the unused tail. Contact data is copied, including world-space geometry and solver impulses.
- `UpdateCollisions` is a synchronous void callback for actual touching manifolds (separation <= 0), distinct from speculative Contact data queries. World mutation or Step/UpdateCollisions reentry during this traversal is unsupported; the header now says so. DynamicTree Query forwards stable proxy IDs for AABB candidates and stops its tree traversal when callback returns false. Tree mutation during a callback is unsupported; it provides no shape-level mask/group filtering.
- Sensor update queries Static/Kinematic/Dynamic trees, excludes same-body shapes, requires both shapes' event opt-in, applies bilateral 64-bit masks and signed group precedence, then uses GJK with radii to reject geometric false positives. Sensor-sensor overlaps are supported, independently of solid contact body-type exclusions.
- Overlaps and event IDs are snapshots of the last Step, including Step(0). Transform/filter/opt-in changes affect the next sensor update. Like Box2D's public sensor-data contract, a destroyed visitor can remain in the snapshot until then; returning the historical generation is intentional. The header now specifies IsValid checks, bounded output, span lifetime and copying when retaining events.
- Generation participates in sort/diff, so slot reuse produces old-generation End and new-generation Begin. Dense sensor removal fixes the moved sensor's shape index and queues historical End events for the next Step, even if no sensors remain. Pending events are published once; published event spans survive ordinary mutations until the next Step or world destruction.
- Replaced the disabled sensor path's temporary empty vector/move with `overlaps.clear()`. This retains capacity while disabled and preserves the same End-event behavior with fewer statements. No new cache, event layer or query abstraction was needed.

## Verification

`worldQuerySensorTests` uses runtime checks in Debug and Release. It covers conservative contact capacities; empty/short output and unused tails; sibling/zero-point/speculative contact selection; actual-touch callbacks and copied snapshots; all three sensor trees; same-body and geometry exclusions; 64-bit bilateral masks and positive/negative groups; sensor-sensor opt-in and stationary refilter; opt-out/event clearing; reused visitor generations; destroyed sensor/body IDs; pending End events with no live sensors; dense sensor swap; and tree early termination before/after rebuild.

Mutation checks removed the overlap generation comparison and the same-body exclusion separately. The new test failed with `old generation end lost` and `sensor all-tree query or same-body exclusion`. Exact production files were restored after each mutation, and the target passed again. No mutation is committed.

Full local Debug/Release build results, independent review and Windows/Ubuntu CI are recorded in the PR. Windows Release CI now includes worldQuerySensorTests.

## Limits and next scope

Handles passed as query inputs must already be valid; existing assertion-based API preconditions remain unchanged. The engine is single-threaded and has no mutation-safe callback lock. Sensor ordering follows current dense sensor iteration and sorted visitor slots; no global event ordering contract is added. Per-Step overlap allocation remains intentional until measured cost justifies persistent double buffers. The existing float sensor tolerance uses <= rather than Box2D's strict < at exactly 10*FLT_EPSILON; this audit preserves the established boundary.

Next recommended scope: contact recycling/cache re-audit (transform/feature validity, threshold boundaries, impulse retention and invalidation). Keep the unrelated debug-timeout branch preserved.
