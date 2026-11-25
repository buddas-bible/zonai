# R4s — previous forward pressure seed

Base: [R4r #385](https://github.com/buddas-bible/zonai/pull/385), 33b8ee1a43c464c3fb55f7d30d37109ebd330615.

[R4r strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45446128764) passed 7,829,407 Fluid rows and 25,596 engine math samples with zero mismatches. External stdlib diagnostics retain 33 mismatches. R4q full CMake passed; R4r Ubuntu CMake passed, Windows pending at preparation. R4r Release timing passed.

| Interval | Cold PCG | Lagged seed PCG | Cold linear ms | Lagged seed linear ms | Lagged seed actual gap (m²) |
| --- | ---: | ---: | ---: | ---: | ---: |
| 43 | 140 | 105 | 3.565 | 2.706 | 6.67684136e-7 |
| 44 | 140 | 90 | 3.560 | 2.332 | 5.52115701e-7 |
| 45 | 140 | 102 | 3.553 | 2.631 | 8.47314528e-7 |
| 46 | 140 | 102 | 3.557 | 2.637 | 9.79588108e-7 |

All four lagged-seed trials retain zero negative modeled volume and unsupported cells, with comparable closure to cold initialization. Seed squared residual relative to RHS is approximately 0.015–0.019. Diagnostic totals 4.653–8.099 ms remain runner-dependent, not production frame times.

## Experiment

Compare the R4r lagged seed to a sequential binary32 pressure cache from the previous independent forward trial. Bootstrap interval43 with the same lagged seed, then reuse only the immediately preceding forward pressure for intervals44–46. No current answer is used as its own seed. Keep adaptive16, gamma=0.75, zero correction, RHS-relative squared target 1e-10, graph masking, gauge projection and independent residual checks unchanged. Report seed source explicitly. The underlying states still come from the same lagged trajectory; no forward state commit, forward advection or persistent forward integrator is claimed.

Add 204 exact rows (51 per interval), expected total 7,829,611. Retain previous rows. Extend Release timing to seven variants and seven repeats (196 samples); each measured history trial receives the same saved prior seed, and its returned pressure must match the normal trial. Report actual closure, modeled negative volume, support, loads, peak, PCG work and timing. No production code or tolerance changes. Both platform producers, strict comparison, CMake and numerical review required. If seed history proves adequate, prioritize actual forward state/endpoint consistency over further micro-optimization.
