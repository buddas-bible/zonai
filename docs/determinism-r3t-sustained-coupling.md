# R3t — sustained in-domain pressure/drag and anchored assembly

Base: [R3s #360](https://github.com/buddas-bible/zonai/pull/360), `2176faeb74c8bd39a70dd32f0319c614f1fa5aee`.

## Verified baseline

[R3s strict CI](https://github.com/buddas-bible/zonai/actions/runs/45410737830): 5,659,220/5,659,220 Fluid trace rows exact, including 1,934,160 continuation rows; 25,596/25,596 engine math samples exact. All required physics gates passed; external libm research remains red. [Full CMake](https://github.com/buddas-bible/zonai/actions/runs/45410737801) passed both OS.

Independent inspection of Ubuntu raw trace artifact 14926069236 (the corresponding Windows rows compared exactly in CI):
- All 361 end-of-frame poses including initial state: circle, capsule and box geometric AABB inside the 12×7 m field. Approximate minimum margins from decoded poses: circle 0.966839 m (tick 211), capsule 0.140935 m (tick 360), box 0.039141 m (tick 360). These are reconstructed geometric extents, not broad-phase padded AABBs or proof of continuous containment between samples.
- Each of 1,440 intervals used coupled projection and reported converged status; max 31 PCG iterations. Each of 360 advanced frames had nonzero pressure feedback for each body and nonzero aggregate drag impulses on both sides.
- Post-projection max face speed 6.04074955 m/s (tick 321 interval 0), max continuity RMS 0.0252274293 1/s (tick 313 interval 3). Maximum aggregate body+fluid drag impulse residual norm 5.76e-9. These are observations, not stability thresholds or full FSI momentum conservation.

## Changes

- Turn coverage into assertions in the existing continuous 360-tick replay. Recompute actual geometric AABBs with ComputeShapeAABB (not padded broad-phase AABB), require all visible shapes within field at every interval entry and frame end; require coupled projection converged and partial cells present at every interval; require nonzero pressure exchange for every visible body and aggregate drag on both sides at every advanced frame. Report min margin, max face speed, max continuity RMS, interval and exchange counts. Preserve all existing trace keys/row counts.
- Append worldFluidAnchoredAssembly: 120 ticks (2 simulated seconds), original Sandbox three dynamic obstacles and same pressure/drag settings. Circle and capsule Weld-connected at initial midpoint; circle additionally attached to a shapeless static anchor at its center by a limited passive Revolute Joint (±0.35 rad). Box remains free. The static anchor is part of World, not a Fluid obstacle. Real solver constraints, no position clamping, artificial reset or new force model.
- Capture both persistent Joint caches, both public reactions, anchor Body in World snapshot, and both transient constraints in first two ticks. Require nonzero reactions from both joints at least once and the sustained exchange checks. Compare with independent unobserved control each frame. The reaction includes initial motion and constraints; not an isolated measurement of fluid force.
- Pin all 12 previous trace segment row counts, require new assembly trace presence. Engine math unchanged. No production code or numerical tolerance changes.

## Acceptance and limitations

Review both producer jobs, strict comparison, full CMake, and coverage summaries. If a coverage assertion fails, inspect that tick/interval before modifying any physics; do not silently weaken coverage. The new fixture has two joints and no intentionally forced collision. The six-second original and two-second assembly do not establish hours-long stability; checks sample interval boundaries, aggregate drag does not prove nonzero per-body drag, and coupled status is not a general energy/momentum conservation proof. Pressure/velocity spikes and more complex contact-rich assemblies require dedicated validation. No master merge or experimental #326 changes.
