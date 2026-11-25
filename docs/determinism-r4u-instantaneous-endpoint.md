# R4u — instantaneous corrected endpoint reconstruction

Base: [R4t #387](https://github.com/buddas-bible/zonai/pull/387), b6d7e855a5962cf9bc39e5cf702a990d53f7a445.

[R4t Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45450922075/job/138647465176) passed, as did the Release timing job. Windows replay, strict comparison and full CMake were pending when checked.

| Interval | Ledger gap (m²) | Velocity-copy gap | Flux-preserving gap |
| --- | ---: | ---: | ---: |
| 43 | 6.67684136e-7 | 3.41464015e-5 | 6.67745126e-7 |
| 44 | 5.52115701e-7 | 3.54007709e-5 | 5.52115701e-7 |
| 45 | 8.47314528e-7 | 3.75081002e-5 | 8.47314528e-7 |
| 46 | 9.79588108e-7 | 3.96057274e-5 | 9.79588108e-7 |

Both endpoint representations have global peaks below 0.698 m/s and zero unsupported open faces in these four trials. Interval43 loses 4.4011795e-11 m² through closed faces in the hypothetical endpoint representation. Flux preservation matches integrated transfer closely, but neither this observation nor the global speed peak establishes instantaneous continuity.

## Fixed-body endpoint projection audit

Use the existing R4r trial and actual corrected World geometry and solid velocities. Compute instantaneous open-fluid flux plus blocked-solid flux from the raw geometric aperture and blocked-segment average solid normal velocity. No historical volume-difference source is substituted. Report masked geometric openings and inactive solid divergence separately. Compare both supported velocity-copy and flux-matched seeds. Build the actual endpoint open-face Laplacian in stable U-then-V row-major order, remove each component's mean and report the removed compatibility residual. Independently solve a fixed-body velocity correction with projected Jacobi PCG at squared relative target 1e-10 and absolute floor 1e-24. Recompute the true residual, round reconstructed velocities to binary32 and independently measure raw instantaneous RMS/max divergence, peak and aperture-weighted correction norm. Unsupported faces flag a failed handoff; do not invent their velocity or accept a projection. No extra body pressure force, no live state commit, no assertion that the reconstruction is momentum conservative.

Add 26 rows per interval (six common plus ten per seed), 104 new exact rows; expected total 7,829,771 Fluid rows. Preserve all earlier rows and existing timing variants. Both platform producers, required strict comparison, full CMake and numerical review required. This is a diagnostic additional solve, not an approved production path. Its numerical error, velocity change, cost and separation from the interval ledger must be reviewed before a persistent forward trajectory. No production code, tolerances, defaults or master changes.
