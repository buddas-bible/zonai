# R4r — forward pressure warm start

Base: [R4q #384](https://github.com/buddas-bible/zonai/pull/384), 62f1b8e353093a784421fe39a1e15bf94f9a4580.

[R4q strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45444698213) passed 7,829,207 Fluid rows and 25,596 engine math samples with zero bit mismatches. External stdlib diagnostics report 33 mismatches. Ubuntu CMake passed, Windows CMake pending when checked. Release timing passed 100 samples.

| Interval | Strict actual gap (m²) | Residual12 actual gap | Residual10 actual gap | Strict linear ms | Residual10 linear ms |
| --- | ---: | ---: | ---: | ---: | ---: |
| 43 | 6.67801061e-7 | 6.67757814e-7 | 6.67534911e-7 | 5.125 | 3.763 |
| 44 | 5.50776758e-7 | 5.50760101e-7 | 5.51539324e-7 | 4.909 | 3.607 |
| 45 | 8.4691009e-7 | 8.46923042e-7 | 8.47471299e-7 | 4.939 | 3.606 |
| 46 | 9.79307535e-7 | 9.79297266e-7 | 9.79523285e-7 | 4.963 | 3.634 |

All three variants retain zero negative modeled volume and unsupported cells on these four frozen trials. PCG iterations are 192/166/140 respectively. Residual10 diagnostic totals are 5.442–8.236 ms. The tested closure changes are small, but these results are not production tolerance acceptance.

## Experiment

Use the same adaptive16 quadrature, gamma=0.75, zero correction and relative squared target 1e-10. Compare cold initialization to a pressure seed from the already computed lagged full-response field, not from the current forward answer. Validate dimensions and finiteness, mask inactive cells, remove each current component mean, and recompute the residual from the forward matrix. No old search direction or cached matrix is reused. Keep the stopping threshold based on the original RHS squared norm rather than the seed residual; retain independent true residual, graph, energy and volume identity checks. Report seed relative squared residual, actual closure, loads, speed, support, modeled negative volume and iteration count. This is an initial-guess experiment, not a persistent forward solver or production warm-start change.

Keep all earlier trace rows; require 200 new exact rows (50 per interval), total 7,829,407. Expand the independent Release timing experiment to six variants and six repeats with balanced rotating order (144 measurements). Compare against cold residual10 at the same tolerance and reject any physically inadequate result. No elapsed time enters the authoritative trace. Both producers, strict comparison, CMake and numerical review required. No production changes or master merge.
