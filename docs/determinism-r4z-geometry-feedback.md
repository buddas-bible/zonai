# R4z — bounded actual-geometry pressure feedback

Base: [R4y #392](https://github.com/buddas-bible/zonai/pull/392), 2fabe9fb1aab187c0bb5eccd130be135ad27517e.

[R4y strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45462331531) passed 7,919,489 Fluid rows and 25,596 engine math samples with zero mismatches. External libm research retains 33 mismatches. R4w and R4x full CMake passed. R4y Ubuntu CMake and Release timing passed, Windows CMake pending when checked.

| Interval57 diagnostic | Original coupled | Actual geometry uniform | Actual geometry weighted |
| --- | ---: | ---: | ---: |
| Target ledger (m²) | -2.55469957e-9 | -9.86002714e-11 | 5.76277807e-17 |
| Negative cells | 1 | 1 | 0 |
| Actual volume L1 (m²) | 8.4300514e-7 | 1.31828642e-7 | 1.31828642e-7 |
| Interval peak (m/s) | 0.857305074 | 0.857303502 | 0.857303519 |

Both fixed-geometry trials used 195 PCG iterations. Weighted compatibility avoids removing volume from fully closed endpoint cells. Its success is conditional on the already known loaded geometry; its newly computed pressure feedback was not applied to World. The force component difference from the original applied load is about 0.004194 N. No physical coupled solution is established.

## Bounded feedback experiment

Keep the original sixteen-interval trajectory unchanged. At interval57 only, initialize the geometry estimate and load with the original coupled trial's loaded endpoint. Solve the weighted fixed-geometry model on that estimate (adaptive16, gamma=0, cold initialization, squared relative tolerance 1e-16), then use its newly computed pressure feedback to replay the same initial free-body World with two substeps. Rebuild the actual loaded geometry and use that endpoint as the next estimate. Run exactly three full-load updates, with no accumulated body force, relaxation, fallback, or accepted state replacement. Neither a small residual on estimated geometry nor a nonnegative ledger establishes self-consistency on the newly loaded geometry.

Record the estimated and actual target volumes, negative volume/count, ledger closure on both geometries, geometry change, support, interval peak, true residual, iterations and force/torque/pose update magnitudes. Check whether the fixed-point update converges or oscillates, especially near the closed cell. No convergence assumption or production promotion; a nonanalytic geometry or solver failure is a research blocker. This does not establish chronological donor feasibility or coupled momentum conservation.

Add 20 exact scalars for each of three passes (60 total), expected 7,919,549 Fluid rows. Preserve existing trace rows and benchmark variants. Both producers, required strict comparison, full CMake and numerical review required. No production code, tolerance, or master changes.
