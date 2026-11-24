# R3w — fine-grid near-closed face replay

Base: [R3v #363](https://github.com/buddas-bible/zonai/pull/363), `b88827dae07351e1e67ae621eca969b186536fed`.

## Verified baseline

[R3v strict CI](https://github.com/buddas-bible/zonai/actions/runs/45415515508) passed all 7,639,425 Fluid rows (including R3u contact assembly 870,485 and stability diagnostic 206,880) and 25,596 engine math samples exactly. All required gates passed. R3v's 360-tick scene: 1,440 valid pressure budgets, max projected speed 6.04074955 m/s, max pressure velocity delta 0.916818619 m/s. Anchored scene: 480 valid budgets, max projected speed 4.15207386 m/s, max pressure velocity delta 0.0834648982 m/s. These are measured maxima, not stability limits.

## Replay contract

- Reuse the settings of the historical `auditFrozenPressure` in tests/sandbox/wind_step_benchmark.cpp: 48x28 grid, h=0.25, 8 World/4 Fluid substeps, dt=1/60, analytic/disjoint coupled pressure, strength 8, drag 0, seeded tracer and the same initial body velocities. Explicitly enable synchronization and source. Run 16 ticks (4 warmup + 12 measured) without resets. The historical V(17,13) case is interval index 43 (tick 11, zero-based interval 3); the historical global peak and this fixed face need not be the same event.
- Record the raw target V and effective opening at every phase; capture all R3v phase 4–8 summaries, adjacent pressure rows and fixed target face audit after phase 9. Find the current global post-projection maximum over all 64 intervals rather than assert a historical speed or location. Both the fine-grid peak and the historical target's speed before/after projection and opening are reported.
- Freeze a copy via existing pre-pressure observer at index 43. Clear callbacks on two independent copies. Replay default and uncached Schur traversal on the identical state. Require default projected raw U/V/pressure/divergence/volume-rate bits to equal live phase 8; require both PCG solves to converge. Emit raw outputs, partial energy/momentum and pressure/feedback diagnostics for each variant. Cached versus uncached output differences are diagnostic, not required to be zero; each variant must compare exactly between platforms. No post-replay tracer transport or World integration is implied.
- Preserve all 14 historical segment counts, add nearClosedFace trace presence. No production code, threshold, solver or tolerance changes. No new same-platform unobserved control for this short stress fixture; prior observer/control scenes remain intact.

## Acceptance / limitations

Both producers, strict comparison, and CMake regression must pass before claiming a reproduced spike. Inspect the actual peak, aperture, projection delta, row RHS/Laplacian/Schur and residual. A converged PCG or cache/reference agreement is not sufficient evidence of stable physics. The historical 28 m/s observation is not assumed to recur under current deterministic math. If a different face or interval becomes the maximum, freeze that event next. The fixed-face audit is a reconstruction from production pressure fields, not the complete matrix. No pooling, pressure/velocity clamping, source modification or experimental #326 changes. Master unchanged.
