# R3o — extended World/Fluid replay and wind-source audit

Base: [R3n #355](https://github.com/buddas-bible/zonai/pull/355), `a1d8e45f4e7bf7c4872d6bed12265329e27985a5`.

## Baseline and scope decision

[R3m strict CI](https://github.com/buddas-bible/zonai/actions/runs/45405137059) verified 2,069,914 identical Fluid/World-fluid rows and 11,728 identical engine math rows on Windows and Ubuntu. The R3n Joint/Fluid trace comparison [45406055464](https://github.com/buddas-bible/zonai/actions/runs/45406055464) is still pending at implementation time; Ubuntu trace generation succeeded. Do not assume the new Joint scenes are cross-platform exact. Extend only the already validated R3m two-way drag scenario, preserve all existing scenes.

## Extended replay

- Append `worldFluidDrag180`: 180 fixed ticks (3 simulated seconds), 8 World and 4 Fluid substeps per tick, 720 actual Fluid/World intervals. Reuse existing 24×14 h=0.5 m coupled analytic cut-cell, pressure and drag scene and same initial state. No new production algorithm, no reset or special forcing.
- Keep per-tick full raw field, geometry/history, World observations and exchange traces, and exact independent unobserved control comparison. Retain all previous short scene rows, including the new R3n fixtures.
- Add compact source-region sampled velocity after source injection and before geometry (phase 0) and continuity, maximum face speed, PCG status and iterations after tracer (phase 9) for each of the 720 intervals. Existing first-two-tick detailed traces remain. A source-region sample includes prior flow and interpolation; it is not a direct observation of source sine. Require finite flow and no reported coupled solve failure at every interval; identify tick/interval in pressure or flow errors. No arbitrary velocity limit imposed.
- Independently reconstruct the actual Sandbox float time recurrence (dt=1/60, World 8, Fluid 4, strength 8) in FP audit. Record time/angle, bounded engine sine candidate and resulting candidate Y increment as 2,880 additional strict math rows (14,608 expected total). Preserve actual std::sin and resulting Y increment as 1,440 external diagnostic rows. This is a separate reproduction, not captured production locals; no production source replacement. Require positive and negative phases and approximately three seconds elapsed.

## Acceptance

All existing physics gates remain unchanged. Check both platforms, all short scenes, extended scene first differing field, source diagnostic, and full CMake. If extended replay fails locally (PCG, nonfinite or observer-control), investigate that failure before claiming cross-platform coverage. If source libm differs, determine whether and when physical state diverges before changing production behavior. This is a first medium-duration test, not hours-long stability, not a guarantee of bounded velocity or pressure, and not full-range rotation, all Joint kinds or extra architectures. Continue longer coverage after identifying issues.

Master and experimental #326 untouched. Historical temporary R3l A/B still triggers on stacked PRs; it is not a new performance claim for this test-only change.
