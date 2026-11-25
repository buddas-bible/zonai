# R4v — persistent forward free-body/MAC experiment

Base: [R4u #388](https://github.com/buddas-bible/zonai/pull/388), 00d3e36f2e9818ccb1199b322eb93e6ac9d5418b.

[R4u required strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45451950228) passed 7,829,771 Fluid rows and 25,596 engine math samples, zero mismatches. R4t and R4u both Windows and Ubuntu CMake passed. External libm research still has 33 mismatches.

| Interval | Copy seed instantaneous RMS (/s) | Reconstructed copy RMS (/s) | Reconstructed flux RMS (/s) | Copy PCG |
| --- | ---: | ---: | ---: | ---: |
| 43 | 5.23855165e-4 | 2.67005813e-8 | 3.06789769e-8 | 133 |
| 44 | 5.30988611e-4 | 2.77597279e-8 | 3.2247393e-8 | 136 |
| 45 | 5.35426271e-4 | 2.78724808e-8 | 3.03907516e-8 | 134 |
| 46 | 5.37148976e-4 | 2.85590526e-8 | 3.08483288e-8 | 136 |

Both seeds are supported, peaks below 0.698 m/s; flux seed needs 128 PCG iterations in each trial. Select the reconstructed velocity-copy seed for a first continuation because its initial instantaneous RMS is smaller. This is not a demonstrated momentum-conserving selection or cost optimum.

## Independent forward trajectory

Start from the historical interval43 pre-pressure snapshot, whose source/advection is already applied. Bootstrap the pressure seed from its historical projection, then retain only the actual loaded World poses/velocities and reconstructed binary32 MAC, interval pressure and actual geometry for the next step. Advance the source schedule and velocity advection on the committed start geometry for intervals44–46, with no extra reference pressure solve or source applied twice. Use adaptive16, gamma=0.75, zero nonlinear correction and squared relative target 1e-10. Each interval reconstructs a fresh free-body World from the carried poses/velocities and independently predicts and replays its loaded endpoint with two World substeps. No constraints are allowed. The instantaneous fixed-body velocity reconstruction changes fluid velocity only; it does not apply a second pressure load to bodies. Close blocked faces with the actual solid speed. Reject unsupported endpoint faces or corrected volume cells instead of silently filling them.

Trace 50 forward diagnostics plus 18 body pose/velocity scalars and four full arrays (U/V, interval pressure, endpoint fraction) per interval on the fixed 48x28 fixture: 22,096 forwardPersistent rows. Add 56 forwardPersistentTransfer and 104 forwardPersistentEndpoint rows. Total expected 7,852,027 Fluid rows. Existing trace rows and timing variants unchanged. Compare closure, negative modeled volume, supported faces, instantaneous RMS, peak, pressure feedback and actual trajectory. Required strict comparison and both CMake platforms plus numerical review are mandatory; a test failure is a research blocker, not grounds to loosen the existing gate.

This is a staged, test-only forward continuation, not production World persistence: fresh World reconstruction is valid only for the tested unconstrained bodies. Advection is a split update on start geometry rather than full space-time momentum transport. The instantaneous correction does not conserve total coupled momentum by construction; tracer, drag, joints, contact, long-duration and real-time cost are not verified. No production code, defaults, tolerances or master changes.
