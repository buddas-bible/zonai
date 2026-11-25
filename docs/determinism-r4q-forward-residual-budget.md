# R4q — forward PCG residual sensitivity

Base: [R4p #383](https://github.com/buddas-bible/zonai/pull/383), 262d1da1ffa400a893b2b7b29a828acd0c2284a6.

[R4p Release timing job](https://github.com/buddas-bible/zonai/actions/runs/45444331039/job/138623804521) passed all 36 timing samples on Ubuntu 24.04, GCC 13.3, AMD EPYC 9V45 runner (4 visible CPUs). Other R4p CI jobs were still running at preparation.

| Interval | Fixed64 diagnostic total ms | Fixed16 total ms | Adaptive16 total ms | Adaptive16 linear ms |
| --- | ---: | ---: | ---: | ---: |
| 43 | 14.774 | 9.732 | 9.746 | 5.163 |
| 44 | 14.642 | 9.720 | 7.118 | 5.177 |
| 45 | 14.734 | 9.686 | 7.101 | 5.163 |
| 46 | 14.622 | 9.705 | 7.191 | 5.175 |

Adaptive coverage reduces quadrature median from 9.153–9.246 ms to 1.618–4.259 ms. Graph assembly and initial linear solve remain about 5.2 ms. This is diagnostic runner timing, not a frame-time benchmark. The present cost is too high to assume a practical multi-substep game solver.

## Controlled sensitivity test

Keep adaptive16 quadrature, gamma=0.75, zero nonlinear corrections, frozen trial inputs and actual two-substep loaded World replay unchanged. Compare the existing relative squared residual target 1e-16 (relative norm 1e-8) against explicitly experimental 1e-12 and 1e-10 (relative norms 1e-6 and 1e-5). Retain the original absolute floor and independent true residual recomputation, matrix/volume identity and graph checks. No production tolerance changes; the strict reference is retained. This is a sensitivity comparison, not an acceptance waiver. Report actual geometric closure, modeled negative volume, unsupported cells, speed, loads, relative true residual and iteration count. Reject any inadequate candidate even if faster.

Retain all prior trace rows, add 196 rows for each of two experimental targets, expected total 7,829,207 Fluid rows. Expand the independent Release timing audit to five variants and five repeats with balanced rotated ordering, 100 timing samples, identical repeat outputs checked. All timing stays outside the exact trace. Both producers, strict comparison, full regression and numerical review required. No production integration, frame-rate guarantee, or master changes.
