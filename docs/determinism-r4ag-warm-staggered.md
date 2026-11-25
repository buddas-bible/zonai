# R4ag — optimized staggered continuation

Base: [R4af #399](https://github.com/buddas-bible/zonai/pull/399), 0daf13274e670bdf604b3b7996ecaefd0ef10582.

[R4af required strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45471362075) passed 8,008,972 Fluid rows and 25,596 engine math samples, zero mismatches. External libm retains 33 mismatches. Release timing and Ubuntu CMake passed; Windows CMake pending when checked. R4ae both CMake platforms passed.

| Matched variant | PCG iterations | Negative cells over 16 intervals | Maximum force component delta (N) |
| --- | ---: | ---: | ---: |
| Fixed cold strict | 194–195 | 0 | 0 |
| Fixed warm strict | 167–182 | 1 | 2.98023224e-8 |
| Fixed warm residual10 | 106–119 | 0 | 1.63316727e-5 |

Cold strict fast operator was numerically equal to the reference on all matched states. Warm strict produced a negative volume of 7.35450121e-17 m² at interval57; do not equate stricter residual tolerance with positivity. Warm residual10 maximum actual volume gap was 2.87267654e-7 m², maximum interval velocity difference 1.1061772e-5 m/s. These are matched-state results, not a trajectory guarantee. At interval57, median linear/pressure costs for cold strict were 4.875/8.142 ms and warm residual10 2.836/6.071 ms (same benchmark job). This supports choosing warm residual10 for a bounded continuation trial.

## Independent continuation

Use the same interval43 bootstrap, precise disjoint capsule geometry, source/advection schedule, lagged pressure coupling and endpoint reconstruction as R4ad. Solve each interval using the fast fixed operator and previous accepted pressure, with relative squared residual 1e-10 (relative norm 1e-5). Carry its own pressure, reconstructed velocities, actual loaded body poses/velocities and newly solved load into the next interval. Preserve the entire original strict trajectory and its matched probes. The optimized trajectory never runs the matched probes or borrows the original trajectory after bootstrap. No fallback or clamping; negative volume remains explicit diagnostic evidence, unsupported geometry remains a hard failure. No production promotion or claim of momentum conservation.

Record the full endpoint state and existing per-interval diagnostics plus true and seed squared residual relative to RHS. Add 88,112 state/diagnostic rows, 256 endpoint rows and 64 reconstruction budget rows (88,432 total; expected 8,097,404 Fluid rows). Repeat the optimized trajectory three times in Release with exact replay equality and require 48 additional timing samples. Compare full diagnostic costs within the same job, noting fixed execution order and test overhead. Retain the prior timing samples. Both producers, strict comparison, CMake and physical review required. No master/default changes.
