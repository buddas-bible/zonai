# R4aa — relaxed geometry feedback

Base: [R4z #393](https://github.com/buddas-bible/zonai/pull/393), 572c559738dfb40f1b23fba1a579e130bcfbcdef.

[R4z Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45463803551/job/138693200875) and Release timing passed; Windows replay and strict comparison were pending at preparation.

| Pass | Negative cells | Actual volume L1 (m²) | Geometry update L1 (m²) | Force update max (N) |
| --- | ---: | ---: | ---: | ---: |
| 1 | 0 | 5.80005561e-7 | 4.49339495e-7 | 0.00419402122 |
| 2 | 0 | 4.89468673e-7 | 4.8007314e-7 | 0.0164510123 |
| 3 | 0 | 6.17991956e-7 | 4.97768269e-7 | 0.016524978 |

No unsupported cells or uncovered faces; each solve required 195 PCG iterations. The force and geometry updates do not decrease monotonically. Center component updates of 1.1920929e-7 m suggest pose quantization may contribute; the cause has not been isolated.

## Comparison

Keep the original full-update three-pass experiment and all sixteen committed intervals unchanged. Independently restart the matched interval57 replay for load relaxation factors 0.5 and 0.25, six passes each. For each pass solve on the estimated geometry with the existing weighted compatibility, gamma=0 and strict residual target. Replay the original start World with appliedLoad = estimateLoad + relaxation * (solvedLoad - estimateLoad), using binary32 load arithmetic. Each next geometry and estimate load must correspond to that replay. The full-update path copies solvedLoad directly, preserving existing arithmetic. Measure actual closure, negative volume, support, peak, full fixed-point load residual, actual applied update and remaining unapplied load. A smaller applied update by itself does not imply convergence; the pressure ledger comes from the full solve and is not dynamically consistent with a partially applied load unless the unapplied difference is negligible. No state replacement, fallback or convergence assumption.

Add 25 exact scalars per relaxed pass, 300 total, expected 7,919,849 Fluid rows. Existing full-update diagnostics and timing variants retained. Both producers, strict comparison, full CMake and numerical review required. If the residual does not shrink meaningfully, stop tuning relaxation and investigate geometry precision or a simpler coupling strategy. No production changes, tolerance adjustments or master merge.
