# Contact recycling/cache re-audit

Base: Zonai master `4571ef022bec0fbe6bf862c044aa6aa3938d6a6a` after World Query/Sensor.
Reference: Box2D main `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`, fetched 2026-10-05.
Sources: [recycling](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/physics_world.c#L530), [feature impulse matching](https://github.com/erincatto/box2d/blob/ac7c751eaeddbabdc1c4d41ae4f3a25d78627790/src/contact.c#L636).

## Findings and rationale

No production correctness defect was confirmed in the reviewed paths. Preserve the existing implementation, add Release-active boundary tests and cite/explain Box2D's recycling rationale in world.cpp and contactSim2.h.

- Contact creation snapshots both bodies' recycling permission; later body toggles affect new contacts. Refilter/destruction removes the Contact and pair key, and slot reuse initializes simulation/cache/impulses anew with a new generation.
- Recycling requires a valid cache, positive world distance and a permitted pair; fast bodies bypass it. Both absolute body rotations must have cosine > 0.98, and relative translation length plus non-static maxExtent * abs(relative sine) must be strictly below tolerance. Static extent is omitted as in Box2D. Equality runs fresh narrowphase. Empty manifolds use min(world distance, speculative distance).
- Relative pose, absolute rotations and A/B origin-local anchors are saved only after fresh narrowphase. Recycled updates do not rebase them, so accumulated motion eventually forces refresh. Separation is fresh separation plus current anchor displacement projected on the current A-frame normal; public points use the current anchor midpoint. This is an approximation retaining existing features to reduce jitter, not a fresh geometric result at each movement.
- Inverse mass/inertia is updated even on the recycled path; density/center-of-mass changes cannot leave solver coefficients stale. Body-origin anchors remain valid when localCenter changes. Constraint preparation mixes current shape friction/restitution every Step, unlike Box2D's cached material path, which can skip material refresh during recycling.
- Fresh manifold replacement zeroes impulses, then restores normal/tangent impulse only for matching point IDs. Unmatched IDs start at zero; two-point polygon feature changes are covered. Circle and polygon-circle single-point manifolds intentionally use a constant ID, as in Box2D: geometric movement alone does not imply a changed point ID. Box2D's unmatched impulse redistribution experiment is disabled; no redistribution is added here.
- Cache-local geometry differs from Box2D's world normal and center-relative solver anchors. Zonai transforms the cached normal with current A rotation and updates public midpoint coordinates; those differences preserve its local-manifold contract. Bounds separation and refilter still precede cache reuse. Existing stable-slot fields and one cache-valid flag suffice; no extra invalidation version, cache layer or tuning API is needed.

## Verification

`contactCacheTests` uses runtime checks in Debug and Release: new cache initialization, exact and cumulative distance, separation/midpoint drift, zero distance, empty manifold tighter tolerance, independent extent-scaled rotation and shared absolute rotation, pair-created body settings/refilter generations, fast-body exclusion, recycled and fresh matching normal/tangent impulses, unmatched two-point polygon IDs, new-pair impulse reset, and density changes with stopping impulse 8*pi for a unit circle of density 4 and approach speed 2.

Mutation: changed >= to > at the relative motion threshold, producing `threshold equality or accumulated motion recycled`. Removed recycled mass updates, producing `recycled contact retained stale effective mass`. A velocity-only mass check initially missed that mutation because the wrong mass was used consistently in solve/application; the final test also checks the independently derived stopping impulse. Source was restored exactly and tests passed again; no mutation is committed.

Full local Debug/Release builds/tests, independent review and final-head Windows/Ubuntu CI are recorded in the PR. Windows Release CI now includes contactCacheTests.

## Limits and next scope

Recycling intentionally tolerates small feature/normal approximation and empty-manifold delay within its displacement budget. Arbitrarily increasing the user distance increases that approximation; no new geometric validity guarantee is claimed. Coverage is bounded, not exhaustive feature-order/shape fuzzing, angular inertia mutation coverage or large-world precision validation. Geometry replacement, chain cache, custom materials and mutation-safe callback support are absent APIs.

Next recommended scope: a phase-wide audit ledger/consolidation to establish which planned engine features and audits remain, then choose the next unfinished phase from repository evidence. Preserve the unrelated debug-timeout branch until its history is intentionally handled.
