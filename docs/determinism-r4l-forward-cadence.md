# R4l — forward predictor cadence comparison

Base: [R4k #378](https://github.com/buddas-bible/zonai/pull/378), 11e5265233a4049a8986e0234f1b83b5a7c1b04a.

[R4k Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45435740078/job/138592910838) passed. Windows and strict comparison were still running when this experiment was prepared. Actual corrected endpoint volume closure is substantially worse than the modeled closure:

| Interval | Predicted endpoint gap (m²) | Actual loaded endpoint gap (m²) | Modeled corrected gap (m²) |
| --- | ---: | ---: | ---: |
| 43 | 2.24512625e-5 | 5.34744552e-6 | 2.40143585e-7 |
| 44 | 2.36082181e-5 | 5.76412009e-6 | 8.34741838e-8 |
| 45 | 2.44353042e-5 | 6.2543588e-6 | 4.1617664e-7 |
| 46 | 2.3833853e-5 | 6.37078912e-6 | 1.91267673e-7 |

No unsupported cells in these four trials. Interval43 has 4.31351791e-11 m² negative modeled volume; interval46 defect correction stalls on pass3. Interval peaks remain below 0.698 m/s. Do not promote the model.

## Controlled comparison

Reuse precisely the same four captured lagged full-response starts, zero-load endpoint prediction, frozen incoming MAC velocity, quadrature, solver tolerances and independent two-substep World replay. Evaluate both responseScale=1 and responseScale=0.75. The latter matches the free-body translational displacement coefficient for two semi-implicit Euler substeps under constant force; it is not a proof of coupled pressure correctness. Keep all 172 R4k rows unchanged and add 172 forwardCadence rows, expected total 7,827,867 Fluid rows.

Compare actual measured raw/projected volume error, modeled defect, negative volume, unsupported cells, force and torque, and interval peak. Check whether cadence scaling closes the measured/model gap without worsening other metrics. A remaining negative volume or stalled solve is not waived. Require both platform producers, exact comparison, full CMake and numerical review; if inadequate, reconsider simpler stable coupling before further solver complexity.

This is frozen-velocity research only; no forward advection, tracer, constraints, corrected endpoint velocity or performance claim. No production code, default, tolerance or master changes.
