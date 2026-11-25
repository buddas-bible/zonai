# R4af — fixed-geometry pressure cost

Base: [R4ae #398](https://github.com/buddas-bible/zonai/pull/398), 79c3ce1883468e1205ce83a85705b9a61b03559b.

[R4ae strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45469871975) passed 8,008,348 Fluid rows and 25,596 engine math samples with zero mismatches. External libm retains 33 mismatches. R4ad both CMake platforms passed; R4ae Ubuntu CMake passed, Windows pending when checked.

[R4ae Release timing](https://github.com/buddas-bible/zonai/actions/runs/45469871975/job/138714604145) passed 48 samples and three identical trajectory traces. Diagnostic total at interval43 was 13.162 ms; subsequent median totals ranged 10.174–11.430 ms. Interval pressure at interval43 was 10.317 ms (quadrature 5.138 ms, linear 4.924 ms); subsequent pressure 7.146–7.222 ms, quadrature 2.000–2.026 ms and linear 4.901–4.965 ms. Endpoint reconstruction cost 2.140–2.217 ms. Linear pressure work is the largest measured subphase.

## Matched optimization

For each original staggered state, run three independent fixed-geometry pressure probes. Skip the body-response operator products during PCG when gamma is exactly zero, retaining the original B^T p traction accumulation on final pressure. Compare cold strict, previous-pressure warm strict, and previous-pressure warm relative squared residual 1e-10. The first interval uses historical pressure; subsequent intervals use the preceding accepted original staggered pressure. No probe load or pressure is committed. This is a matched-state optimization comparison, not an optimized continuation trajectory.

Require the cold strict fast operator to produce equal numerical pressure, interval velocity, feedback, ledger, PCG iteration count and true residual to the original calculation. This equality check is numerical (signed zero is not distinguished), not a cross-variant bitwise equivalence assertion. All authoritative diagnostic rows still undergo cross-platform bit comparison. Record negative volume/count without clamping, support, volume closure, residual, iterations and maximum deltas in pressure, velocity, force, torque and ledger. In particular, looser residuals may create small negative closing-cell volumes and must not be promoted solely for speed.

Retain 16 original intervals, unchanged accepted trajectories and original timing variants. Add 13 scalars per variant per interval, 624 rows, expected total 8,008,972 Fluid rows. Release benchmark records 144 additional matched pressure samples; exclude probe time from the original staggered diagnostic total. Timing still includes research fixture and trace overhead, not shipping frame cost. Require both producers, exact comparison, CMake and timing review. Select an optimized path only after assessing numerical and physical results. No production default or master changes.
