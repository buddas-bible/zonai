# R4p — forward predictor cost research

Base: [R4o #382](https://github.com/buddas-bible/zonai/pull/382), 984041fcb7f09ef9ad88b45b39bd872088c13431.

[R4o strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45439845500) passed 7,828,815 Fluid rows and 25,596 engine math samples with zero mismatches. Both OS CMake jobs passed in [R4o](https://github.com/buddas-bible/zonai/actions/runs/45439845969) and R4n. External libm research retains 33 mismatches.

| Interval | Fixed16 actual gap (m²) | Adaptive16 actual gap (m²) | Adaptive weighted panels |
| --- | ---: | ---: | ---: |
| 43 | 6.67801061e-7 | 6.67801061e-7 | 48 |
| 44 | 5.50778543e-7 | 5.50776758e-7 | 16 |
| 45 | 8.46907079e-7 | 8.4691009e-7 | 16 |
| 46 | 9.79308154e-7 | 9.79307535e-7 | 16 |

Only interval43 needs endpoint refinement (one missed changed-volume cell); all four adaptive trials have zero negative modeled volume and unsupported cells, with 192 initial PCG iterations each. Weighted assemblies total 96 instead of 192 for fixed16, or 384 for fixed64. This count excludes the coverage scan and does not establish elapsed-time savings.

## Measurement

Add opt-in fluidStateDeterminismTests --benchmark-forward. Use the existing frozen four-interval fixture, gamma=0.75, zero nonlinear corrections. Warm up via the existing comparisons, then measure three repeats per interval of fixed64, fixed16 and adaptive16, rotating execution order. Assert repeated exact diagnostic outputs match within each variant. Record steady-clock timings for coverage/quadrature, graph assembly and initial solve, and whole diagnostic trial including World copies, replay, audits, logging and trace formatting. No timings enter authoritative trace rows or numeric decision logic. Normal deterministic coverage remains 7,828,815 Fluid rows.

An independent Ubuntu Release research CI job builds and runs the option, validates 36 timing samples and prints per-interval/variant medians; it uploads raw evidence. These are runner-dependent diagnostic timings, not a production frame-time benchmark. No speed threshold. Examine measured costs before further optimizing or adopting the frozen model. No production source, defaults, tolerances, or master changes.
