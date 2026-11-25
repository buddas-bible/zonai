# R4n — forward quadrature budget

Base: [R4m #380](https://github.com/buddas-bible/zonai/pull/380), dc89a5334bc27e3fe35f35e37eba64787b95be38.

[R4m required strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45437746301) passed 7,828,247 Fluid rows and 25,596 engine math samples with zero mismatches. External stdlib diagnostics still report 33 differences. R4l full CMake passed; R4m Ubuntu passed, Windows pending when checked.

| Interval | Zero correction measured gap (m²) | One correction | Three corrections |
| --- | ---: | ---: | ---: |
| 43 | 6.67796701e-7 | 3.27832278e-7 | 3.65655738e-7 |
| 44 | 5.50788035e-7 | 5.22081802e-7 | 5.22081802e-7 |
| 45 | 8.46897839e-7 | 7.36572811e-7 | 9.30762715e-7 |
| 46 | 9.79310714e-7 | 5.81108796e-7 | 5.76694979e-7 |

Zero correction retained nonnegative modeled volume in all four trials, unlike one and three corrections at interval43 (7.38053656e-11 and 2.3292307e-11 m² negative respectively). All candidates have zero unsupported cells, peaks below 0.698 m/s. Zero correction uses 192 initial PCG iterations per interval, without the additional 185–194 iterations of one correction or 382–577 of the three-pass budget. Three passes do not consistently improve measured closure. Prefer zero correction for the next controlled cost experiment, not as a production decision.

## Experiment

Keep gamma=0.75, zero nonlinear correction, identical four frozen incoming MAC states, independent zero-load prediction and actual two-substep loaded World replay. Compare 16/32/64 uniform midpoint budgets, all with 16 dyadic refinement levels at each endpoint. No aperture floors or support fallback. Preserve all existing trace rows and require two new 172-row scenes plus 36 operation-count rows (midpoint coverage checks, weighted panels, initial PCG iterations), total 7,828,627 Fluid rows. Weighted panel counts are 48/64/96; they count quadrature assembly calls, not wall time or all geometry rebuilds.

Review measured closure, negative modeled volume, support, force/torque, and interval peaks against the 64-sample reference. Any support/solver failure blocks acceptance. Fewer panels alone do not demonstrate practical real-time performance; the experiment retains the full production-independent quadrature machinery. Both OS producers, required strict comparison, full CMake and numerical review required. No forward advection, persistent integration, tracer, endpoint MAC reconstruction or constraint claim. No production code, tolerances, defaults, or master changes.
