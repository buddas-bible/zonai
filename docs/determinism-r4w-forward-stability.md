# R4w — extended forward stability and momentum audit

Base: [R4v #389](https://github.com/buddas-bible/zonai/pull/389), 5ceb2d9975f86576d7cd7d11f2d4d11ab6ecc3ff.

[R4v required strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45458699969) passed 7,852,027 Fluid rows and 25,596 engine math samples with zero mismatches, including all 22,256 new continuation diagnostics. External libm research has 33 mismatches. Ubuntu CMake and Release timing passed; Windows CMake was pending when checked.

| Interval | Actual volume L1 (m²) | Reconstructed copy RMS (/s) | Reconstructed copy peak (m/s) | Interval PCG |
| --- | ---: | ---: | ---: | ---: |
| 43 | 6.67968298e-7 | 2.62854092e-8 | 0.653613806 | 92 |
| 44 | 7.23280291e-7 | 3.02000513e-8 | 0.668193281 | 110 |
| 45 | 7.07532867e-7 | 3.09566843e-8 | 0.682768583 | 106 |
| 46 | 8.5527502e-7 | 2.89742156e-8 | 0.697339833 | 107 |

Zero negative modeled volume and unsupported cells/faces in these four trials. This is limited free-body evidence, not general stability or momentum conservation.

## Experiment

Extend the same forward continuation from four to sixteen consecutive intervals (43–58), keeping the initial snapshot, source cadence, start-geometry advection, predictor, pressure tolerance and reconstructed velocity-copy commit unchanged. Preserve prior trace keys and values. The first four intervals must remain reproducible. Unexpected constraint, geometry, support or solver failures are blockers; do not relax checks or introduce a silent fallback.

Add eight diagnostic scalars per committed interval. Measure the velocity reconstruction alone using unit-density/unit-depth MAC face weights h² times actual endpoint open aperture. Accumulate the signed x/y change in this momentum proxy and pre/post kinetic energy proxies using the actual rounded binary32 reconstructed velocity. Separately record actual summed body linear momentum change (mass times measured velocity difference) and applied interval pressure impulse (dt times summed feedback). The proxy does not represent an exact cut-cell dual mass or conservative momentum transport; body momentum change can include other World forces. Do not combine these terms into a purported closed total momentum budget. Use them to identify the scale and sign of reconstruction effects and inform a later physical budget.

Expected coverage: 88,384 forwardPersistent, 224 forwardPersistentTransfer, 416 forwardPersistentEndpoint and 128 forwardPersistentMomentum rows, or 66,896 additional exact rows and 7,918,923 Fluid rows overall. Require both platform producers, strict comparison, full CMake and review of all sixteen intervals, including topology changes, negative modeled volume, instantaneous divergence, peak, body impulse and correction proxy. Sixteen intervals are still a short-window test, not long-duration validation. No production changes, tolerance adjustments, tracer, constraints or master merge.
