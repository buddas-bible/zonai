# R4o — coverage-triggered endpoint quadrature

Base: [R4n #381](https://github.com/buddas-bible/zonai/pull/381), 8a835d2d64d6dfd843dd532120e35baa8b458aa2.

[R4n Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45438977208/job/138604468239) passed. Windows and strict comparison were pending at preparation. R4m both CMake jobs passed.

| Interval | 64 midpoint measured gap (m²) | 32 midpoint | 16 midpoint |
| --- | ---: | ---: | ---: |
| 43 | 6.67796701e-7 | 6.67798457e-7 | 6.67801061e-7 |
| 44 | 5.50788035e-7 | 5.50786177e-7 | 5.50778543e-7 |
| 45 | 8.46897839e-7 | 8.46901109e-7 | 8.46907079e-7 |
| 46 | 9.79310714e-7 | 9.79311116e-7 | 9.79308154e-7 |

All three budgets have zero negative modeled volume and unsupported cells in the tested intervals; each needs 192 initial PCG iterations. Reducing from 64 to 16 uniform midpoints with fixed endpoint refinement halves weighted assembly calls (96 to 48), without a substantial change in the observed errors. This is not a runtime measurement or a production acceptance.

## Experiment

Keep 16 uniform midpoints, gamma=0.75, zero nonlinear correction and the same four frozen forward trials. Reuse the existing uniform midpoint coverage scan. If any changed-volume cell has no midpoint support, retain all 16 dyadic endpoint refinement levels; otherwise use the uniform rule without endpoint refinement. No support deletion, aperture floor, retry fallback, or relaxed graph/residual check. A coverage scan alone is not a guarantee of correct aperture quadrature, particularly for transient cells with identical endpoint volumes. Compare actual measured closure, peak, loads, negative volume and support with the fixed-refinement 16-midpoint baseline; reject unsuitable candidates.

Retain all existing trace rows; add 188 exact rows (43 existing-style diagnostics plus four coverage/cost scalars per interval). Expected 7,828,815 Fluid rows. Report selected depth, missed cells, midpoint coverage checks and weighted panels, with 16 or 48 weighted assemblies. No claim about elapsed time or total geometry work. Both platform producers, required strict comparison, full CMake and numerical review required.

Frozen-velocity, unconstrained research only. No production code, default, tolerance or master changes.
