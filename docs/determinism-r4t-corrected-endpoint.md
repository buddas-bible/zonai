# R4t — actual corrected endpoint handoff

Base: [R4s #386](https://github.com/buddas-bible/zonai/pull/386), eeab042b8c78e6d9c20da64364cc23d79d6b3c8b.

[R4s required comparison](https://github.com/buddas-bible/zonai/actions/runs/45448750399) passed 7,829,611 Fluid rows and 25,596 engine math samples with zero mismatches. External libm diagnostics retain 33 mismatches. R4r and R4s both CMake platforms passed. R4s Release timing passed 196 samples.

| Interval | Lagged seed PCG | Previous forward seed PCG | Lagged seed linear ms | Previous forward seed linear ms |
| --- | ---: | ---: | ---: | ---: |
| 43 | 105 | 105 | 2.712 | 2.699 |
| 44 | 90 | 108 | 2.324 | 2.772 |
| 45 | 102 | 109 | 2.630 | 2.804 |
| 46 | 102 | 106 | 2.632 | 2.731 |

History seeding keeps comparable measured closure and zero negative modeled volume or unsupported cells, but is slower on all three intervals that actually use the previous forward result. Retain R4r lagged seed for the next matched-state audit; stop cache micro-optimization. This comparison does not establish how pressure history will behave in a persistent forward integrator.

## Corrected endpoint audit

The accepted forward interval solution is defined on the predicted interval-averaged graph. The actual loaded World endpoint differs. Retain mean apertures and solved interval velocities for the R4r trial only. On the rebuilt actual corrected geometry, compare two explicit hypothetical handoffs: copying supported interval velocity and preserving each supported face's interval transfer by dividing by actual endpoint opening. Separately evaluate the signed interval ledger against actual volume change. Report transfer mismatch, cell volume L1/max error, velocity peaks, smallest open fraction, closed-face lost transfer and unsupported open faces. Unsupported faces are reported, not filled; closed transfers remain visible in the ledger. The endpoint-as-constant-in-time transfer test is not an instantaneous divergence test. Neither representation is projected or committed, no extra pressure loads are applied, and there is no endpoint velocity reconstruction acceptance yet.

Add 14 scalar rows per interval, total 56 new rows and expected 7,829,667 Fluid rows. Preserve earlier diagnostic outputs and the existing benchmark variants. Both platform producers, required strict comparison, CMake and numerical review required. Use results to decide whether a separate endpoint velocity reconstruction is viable before implementing persistent forward advection/World coupling. No production code, tolerances, or master changes.
