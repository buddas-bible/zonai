# R4m — forward correction-budget comparison

Base: [R4l #379](https://github.com/buddas-bible/zonai/pull/379), d16d007132cc3d9a5312aa2ef37a43572e029b17.

[R4k strict CI](https://github.com/buddas-bible/zonai/actions/runs/45435740078) passed 7,827,695 Fluid rows. [R4l strict CI](https://github.com/buddas-bible/zonai/actions/runs/45436513891) passed 7,827,867 Fluid rows including both 172-row forward scenes, and 25,596 engine math samples, all with zero raw-bit mismatches. External stdlib research gate still reports 33 mismatches. Ubuntu CMake passed for both; Windows CMake pending when checked.

| Interval | Full-response measured gap (m²) | Cadence measured gap (m²) | Cadence modeled gap (m²) |
| --- | ---: | ---: | ---: |
| 43 | 5.34744552e-6 | 3.65655738e-7 | 2.22127059e-7 |
| 44 | 5.76412009e-6 | 5.22081802e-7 | 4.17943746e-7 |
| 45 | 6.2543588e-6 | 9.30762715e-7 | 4.5860308e-7 |
| 46 | 6.37078912e-6 | 5.76694979e-7 | 2.9155169e-7 |

Cadence significantly reduces actual measured volume discrepancy. However interval43 retains negative modeled volume 2.3292307e-11 m², and interval44 correction stalls. All four have zero unsupported cells, peaks below 0.698 m/s. Neither is ready for production.

## Experiment

Use the exact same four R4l forward trial starts, gamma=0.75, 64 midpoint panels with endpoint refinement, frozen incoming MAC velocity, and independent actual two-substep World replay. Compare zero, one, and three allowed nonlinear defect correction passes. Keep the initial projected solve and all residual/graph validity checks. The zero budget deliberately tests the linear projection without nonlinear correction; it does not relax the physical acceptance criteria.

Preserve every old trace row and add two 172-row scenes plus 36 operation-count rows: initial PCG iterations, cumulative correction PCG iterations, and defect geometry evaluations (including rejected backtracking trials) for all three cadence budgets. Expected total 7,828,247 Fluid rows. These counts are not elapsed-time benchmarks and exclude common quadrature and other diagnostics. Compare actual geometry closure, negative modeled volume, unsupported cells, speed and forces; prefer the simpler candidate only if physically adequate. Do not promote a candidate solely on bit equality or lower counts.

Both platform producers, strict comparison, full CMake and result review required. Frozen-velocity, unconstrained research only; no corrected endpoint MAC, forward advection, tracer or persistent forward integration claim. No production code, defaults, tolerances or master changes.
