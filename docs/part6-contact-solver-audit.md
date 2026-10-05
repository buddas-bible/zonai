# Part 6 Contact Solver re-audit

Base: Zonai master `15df78a72b5f1b2af1abb8c8d879f06bd5a77934` after Part 5.
Reference: Box2D main `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`, fetched 2026-10-05.

## Findings

The preserved Part 6 branch contains restitution/stack regressions, not an outstanding solver fix. Current master already performs two restitution sweeps and tracks compression/restitution impulses. The original one-sweep assumption in draft PR #11 is obsolete.

No additional solver correctness defect was identified in the audited paths. Keep the existing scalar solver and API. The necessary changes are runnable Release checks, inherited regression integration, and a comment explaining coupled restitution sweeps.

## Function and stage comparison

- `MakeContactSoftness`: matches Box2D's Hertz/damping formula; zero Hertz/time uses the rigid default. World preparation caps Hertz against substep duration and doubles it for static contacts, using Box2D's default 30 Hz, damping ratio 10, push speed 3 and restitution threshold 1.
- `PrepareContactConstraint`: converts the shape-A-local manifold to world normal/COM anchors; computes normal and tangent effective masses; samples pre-force relative normal velocity; restores cached normal/tangent impulses. Per-step compression and restitution accumulators start at zero.
- `WarmStartContactConstraint`: applies cached normal/tangent impulses and includes applied warm-start normal impulses in the accumulated compression, matching current Box2D.
- `SolveContactConstraint`: recomputes point velocities sequentially and separation from accumulated delta translation/rotation. Positive separation uses speculative bias, penetration push uses softness and bounded correction, accumulated normal impulses stay nonnegative. Relax removes bias before Coulomb friction, with tangent impulse bounded by friction times normal impulse.
- `ApplyRestitutionContactConstraint`: threshold plus actual compression arms restitution. Approach impulse is separated from bounce impulse, and Poisson allowance prevents consuming the same compression budget repeatedly. Unarmed points retain speculative constraints. Zero restitution skips the stage, matching Box2D's default propagation-disabled behavior.
- `StoreContactImpulses`: stores the final normal/tangent impulses in persistent ContactSim for point-ID matching in the next collision update. Hit events, rolling resistance and total-impulse public telemetry are outside the existing Zonai API and are not added speculatively.
- `world::Step`: prepare once, then integrate velocity/warm start/push/integrate delta position/relax for each substep; restitution and impulse storage run after all substeps. Stable contact/body slots and island spans remain unchanged.

Zonai retains eight scalar push/relax sweeps per substep; current Box2D uses one of each within its graph/color/SIMD scheduler. This is a convergence/cost difference, not proof of a correctness defect. No sweep-count or scheduler tuning is included here. Two restitution sweeps match current Box2D's default. Rolling resistance, restitution propagation, graph scheduling and SIMD are feature/performance differences, not unused scaffolding to add during this audit.

Existing production variable names and function definition order are preserved. No unrelated Part 4/5 algorithm or debug-timeout change is introduced.

## Verification

- Existing impact, elastic drop/apex/spin and 12-body stack tests pass on current master after bringing the branch forward.
- Converted contactConstraintTests and restitutionTests from assert-only checks to runtime checks with source-location diagnostics, active in Release.
- Added equal-mass dynamic/dynamic tests at restitution 0, 0.5 and 1: momentum, expected bounce velocity, energy ceiling and unchanged compression budget across eight repeated restitution calls.
- Mutation check: temporarily reducing world restitution sweeps from two to one makes the impact regression fail. Restored the exact original implementation and reran successfully. The mutation is not committed.
- Local Debug and Release full builds/tests: 35/35 pass in each configuration. Final independent review and CI are recorded in PR #11.
- Windows Release CI now runs contactConstraintTests and restitutionTests alongside Part 4/5 runtime lifecycle checks.

Next recommended scope: Island / Sleep / CCD integration audit, with contact activation, wake/sleep propagation, substep deltas and TOI boundaries verified separately.
