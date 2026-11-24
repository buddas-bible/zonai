# R3s — continuous 360-tick World/Fluid replay

Base: [R3r #359](https://github.com/buddas-bible/zonai/pull/359), `0185dd05e5ab4382e9fea9c1cfdf42ae6fdca0e6`.

## Verified baseline

[R3r strict CI](https://github.com/buddas-bible/zonai/actions/runs/45409519060): both producers and required strict comparison passed; all eleven Fluid scenes (3,725,060 rows), all 25,596 engine math samples (including the 1,440-interval source recurrence) exact. Spline physics exact, 222 separate diagnostic differences; external libm research remains red. [R3q full CMake](https://github.com/buddas-bible/zonai/actions/runs/45408526495) passed both OS. [R3r Release A/B](https://github.com/buddas-bible/zonai/actions/runs/45409519117) succeeded; reported paired physical metrics identical, no nonfinite faces or PCG failure. At h=0.25/8 Fluid substeps, field median 9.6166→9.5645 ms, swept 0.0588→0.0583 ms. Short benchmark does not measure large-angle throughput.

## R3s scope

- Extend actual R3o/R3p two-way pressure+drag Sandbox scene from 180 to 360 ticks (6 simulated seconds, 1,440 Fluid intervals and 2,880 World substeps). Same scene, geometry, field, source, and initial state; no reset at tick 180.
- Preserve the first 180 ticks' original `worldFluidDrag180` trace keys and all preceding ten scene schemas. Label ticks 181–360 `worldFluidDrag360Continuation` for per-scene CI reporting. These are two trace segments of one simulation, not two independent scenes.
- Record full World/Fluid snapshots every tick; compare with independent unobserved control. Keep existing per-interval source-region velocity and projection diagnostics. Add all ten Fluid phases in all four intervals at ticks 221–223 near the old bounded angle crossing (approximately tick 222 based on the R3r source recurrence). This is a chosen diagnostic window, not a direct capture of the private source clock.
- Retain finite-state, no coupled-solve-failure, pressure/drag exercise and phase-sequence checks. Report frame context on observer/control divergence. No arbitrary face-speed or pressure caps, no stability thresholds relaxed.
- Stream the strict TSV comparison instead of reading both entire traces into memory. Preserve exact line/key/value comparisons and missing-row detection. Require all eleven historical per-scene row counts and presence of the continuation; no tolerance added. Existing 25,596 engine math rows unchanged.

## Acceptance

Both producer jobs, strict physical comparison and full CMake must pass. Review the continuation's first mismatch and its stage context if any. A local pressure failure or nonfinite value is a blocker, not a reason to loosen the gate. Six seconds is a medium-duration test, not long-run stability certification; some bodies may move out of the field, so passing alone does not prove persistent coupled pressure coverage over arbitrarily long runs. Next audit actual sustained in-field coupling, contacts/Joint networks and remaining libm paths; then extend duration, Release and architecture coverage. Master and experimental #326 unchanged. Temporary inherited R3r A/B may run again because the stacked PR targets master; it is not a new production performance comparison for R3s.
