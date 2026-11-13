# S2c small Cut-cell pressure pooling — CI evidence and next gate

Observed on PR #326 commit `c260625f8e74b5eb6c296ac9686d5d63f2fd3b57`, workflow [43268224810](https://github.com/buddas-bible/zonai/actions/runs/43268224810), Windows Release job. Ubuntu and Windows jobs completed successfully. **Research only: do not enable the model by default or merge as a validated physical fix.**

## Frozen-state threshold sweep

CSV source: `sandboxWindStepBenchmark --small-cell-merge-threshold-audit`. Same pre-projection state, V(17,13), h=0.25 m, World 8 / Fluid 4, two-way ON and Drag 0. Threshold is the opt-in minimum fluid fraction.

| Threshold | Merged cells | PCG iterations | Target V (m/s) | Max raw face speed (m/s) | Grouped continuity RMS (1/s) | Individual continuity RMS (1/s) | Tracer mass |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0 | 0 | 61 | -28.1338482 | 28.1338482 | N/A | 7.19411446e-8 | 0.5 |
| 1e-9 | 0 | 61 | -28.1338482 | 28.1338482 | N/A | 7.19411446e-8 | 0.5 |
| 1e-8 | 1 | 49 | -0.51022017 | 0.653637052 | 7.39733608e-8 | 5.17688808e-4 | 0.5 |
| 1e-6 | 1 | 49 | -0.51022017 | 0.653637052 | 7.39733608e-8 | 5.17688808e-4 | 0.5 |
| 1e-4 | 1 | 49 | -0.51022017 | 0.653637052 | 7.39733608e-8 | 5.17688808e-4 | 0.5 |
| 1e-2 | 3 | 56 | -0.51022017 | 0.653637111 | 6.09995752e-8 | 5.18036075e-4 | 0.5 |

All six sweep rows reported PCG status 1. The reduced operator enforces *group* continuity. It explicitly does not enforce the same pointwise continuity in a sliver: the increase in individual continuity is material and cannot be presented as an individual-cell improvement. Conserved global tracer mass is not proof of acceptable local tracer transport. Virtual internal exchange and geometry displacement require independent interpretation.

## Continuous World comparison

The `--small-cell-world-feedback-audit` run tested two separately evolved trajectories at fixed parameters, not a common frozen snapshot. Drag=0 reported last merged frame 15 and sum of merged cells over last substeps 2; Drag=3 reported last merged frame 21 and sum 3. Different trajectories must not be interpreted as a paired same-state face-speed correction. The output has columns for pressure force/torque, Body position/speed, continuity, pressure impulse, and PCG status, so acceptance requires explicit divergence curves and tolerance definitions rather than a simple pass flag.

## Tracer and geometry diagnostic

The `--small-cell-local-tracer-audit` tracer-seeded experiment returned global mass 0.00731756771 kg in the printed frame-5 and frame-19 cases. In the swept-remap frame-5 baseline, unresolved tracer was 1.44328651e-5 kg with 2 unresolved sources; the worst source at (18,13) retained ~1.44238064e-5 kg without a usable outward swept transfer although its neighboring final faces were open. This is a **local redistribution diagnostic**, not proof of lost global mass. In the swept-remap frame-19 baseline unresolved mass rose to 6.81927268e-5 kg with 3 sources; the worst cell (18,13) had geometry-displaced mass 6.6337292e-5 kg, swept transfer 6.23862805e-6 kg, and later MAC net outward transport 6.95686689e-5 kg. Swept remap, tracer transport and endpoint geometry flux are distinct stages; adding their totals blindly would double-count.

## Required next atomic work

1. Add a **read-only per-stage local tracer/volume closure test** for the same source cell and its connected neighbors, including negative inward and outward swept-face signs. Assert that each stage's mass ledger balances separately: previous mass, swept transfer, fallback, unresolved retained/trapped mass, MAC flux and final mass. Never declare unresolved tracer automatically lost.
2. Compare `snapshot_delta_m3`, `pressure_source_delta_m3`, `swept_divergence_m3`, and `mac_endpoint_out_m3` at matching captured geometry, distinguishing endpoint area flux from swept-volume transfer.
3. Require opening/closing, reopened face, fully disconnected cell, all four directions, varying h and World/Fluid cadence. Record baseline and pooled group continuity **and** individual continuity, local tracer displacement, pressure force/torque, body trajectory and runtime.
4. No forced velocity/pressure clamp, no default threshold change, no merging #322 sampling into this experiment. Retain #326 as Draft until conservation and dynamic feedback acceptance is justified.

## 2026-11-13: applied local tracer transfer closure (experimental branch)

The first portion of gate 1 now has executable regression coverage in PR #326:

- Commit `a26cf59`: four-direction moving Cut-cell closure/reopening differential, h=0.5/0.25 m, Fluid substeps 1/4, with pressure pooling setting 0/1e-4. Ubuntu/Windows workflow [43302725792](https://github.com/buddas-bible/zonai/actions/runs/43302725792) passed.
- Commit `195ee87`: opt-in post-Swept and post-MAC `tracerStageObserver2` exposes actual stored intermediate cell mass; no change to solver or advection. Ubuntu/Windows workflow [43304245269](https://github.com/buddas-bible/zonai/actions/runs/43304245269) passed.
- Commits `de682eb`, `27506cb`, `7f60ce0`: optional `tracerTransferObserver2` records every *applied*, directed, positive-kg Swept-face, fallback and donor-limited MAC transfer. Regression independently reconstructs the post-Swept and post-MAC mass of **every cell** by accumulating outgoing and incoming events against the pre-Step snapshot. It separately checks each stage's global signed delta and zero influence on the unobserved solver path. Workflow [43357224077](https://github.com/buddas-bible/zonai/actions/runs/43357224077) is the corresponding CI run; check both job conclusions before acceptance.

`unresolvedDisplacementMass` is **not** emitted as a transfer event because the unmatched tracer remains stored at its original cell. A remap fallback may send mass to a nonadjacent destination; only Swept/MAC face events must connect direct neighbors. The callbacks are optional observers, not a conservative space-time reconstruction, and should never be used to modify the field during stepping. Default tracer and pressure behavior remain unchanged.

### Real World8/Fluid4 applied-transfer ledger — 2026-11-13

`sandboxWindStepBenchmark --small-cell-applied-transfer-audit` runs the existing real `rigidBodyDemo::step` World8/Fluid4 path at h=0.25m, two-way ON, Drag=0, 20 frames with tracer seeded after frame 4. It measures **all** 48×28 cells at each Fluid step, and independently sums applied Swept, fallback, and donor-limited MAC transfer events into each cell. It also checks net transport through the fixed source-plus-W/E/S/N stencil around **(18,13)**; transfers within the stencil cancel, whereas crossings of the stencil boundary change its mass. Substep stage snapshots must match this signed event ledger, and the unmatched swept displacement must remain accounted as stored mass rather than a transfer.

Code commits: `bf9de6a` (sandbox observers), `a7090fd` (actual World audit), `05ab16d` (Windows Release CI), `456a4ea` (Ubuntu Debug CI) and `863b71b` (require actually pooled substep). The original Windows Release audit **passed** on workflow [43359225633](https://github.com/buddas-bible/zonai/actions/runs/43359225633); Ubuntu and Windows jobs both concluded success for that workflow. The CI added to the later code commits should be checked independently.

The first successful Windows audit emitted 120 per-Fluid-step records (2 pressure variants × 15 post-seed World frames × 4 Fluid steps). The test observed nonzero actual Swept and MAC movement, unresolved displaced material, and pressure pooling in the experimental branch at eight Fluid steps, including frame 10/substep 2 and frame 15/substep 3.

| Source cell (18,13), last Fluid substep | Baseline (threshold 0) | Experimental (threshold 1e-4) |
| --- | ---: | ---: |
| Frame 5 unresolved displaced tracer, kg | 1.44328653496e-5 | 1.44328653496e-5 |
| Frame 19 unresolved displaced tracer, kg | 6.81927225801e-5 | 6.82126703029e-5 |
| Frame 19 source stored tracer, kg | 0.00140467413087 | 0.00140350152753 |
| Frame 19 applied Swept face transfer, whole field, kg | 1.1248976679e-5 | 1.12651441627e-5 |
| Frame 19 source snapshot volume change, m³ | -0.000813487917185 | -0.000813778489828 |
| Frame 19 endpoint MAC outward volume, m³ | 0.000818215339838 | 0.000818503360164 |

All quantities refer to **one Fluid substep**, not cumulative frame totals. The field-wide applied Swept transfer and one-cell unresolved mass represent different sums and must not be equated. Likewise the snapshot, pressure-source and swept-volume entries are unit-depth volume diagnostics, not tracer kilograms.

Crucially, the frame-5 baseline source *gains* mass from 0.0019484740762 kg before remap to 0.00228605529247 kg after Swept and 0.00242990565431 kg after MAC despite unresolved displacement there: inflow from other donors exceeds its own local exported amount. At frame 19 the same cell goes from 0.0014802727017 → 0.00147424279978 → 0.00140467413087 kg; later MAC advection is independently measured, not counted as successful Swept evacuation. The tracer mass balance and physical appropriateness of the displacement are therefore **different questions**.

The experimental pressure pool changes local tracer evolution modestly after pressure pooling activates, but **does not eliminate the unresolved swept-remap discrepancy**. No physically accepted replacement model has been established here.

### Experimental connected-residual tracer routing — 2026-11-13

**Research-only, default OFF.** New `setConnectedResidualTracerRemapEnabled(true)` applies only when the existing opt-in Swept tracer remap is active, a valid moving-boundary history exists, and the shrinking donor is **still partially open**. The normal outward-swept transport and its volume-derived displacement cap run unchanged. For the *remaining unmatched tracer mass*, the experiment offers direct neighbor recipients with positive current fluid fraction **and positive current shared-face aperture**, distributing the unmatched mass in proportion to endpoint apertures. If there is no such face, the mass is retained and reported as unresolved: do not fall through to legacy gain-only fallback through a closed face. Completely covered donors continue the pre-existing nearest-fluid fallback, which is **not** a claim of face-local conservative transport.

This is a hypothesis for local tracer routing, **not a fluid volume reconstruction**. It does not change Cut-cell pressure, Body feedback, the swept geometry source, or MAC velocity; the new transport is logged separately as `connectedResidual` events and `connectedResidualRemappedMass` / `connectedResidualSources`. The baseline choice, initial data and existing `fallback` behavior are untouched when the option is OFF. No pressure/speed clamps or default changes were made.

Evidence/commits: `5c3a4d2` (opt-in API), `fface935` (aperture-weighted residual transfer), `544e3d4` (sandbox control), `ea1e82e` (real World8/Fluid4 A/B), `e4744f7` (four directions, h=0.5/0.25m, Fluid steps=1/4, mass + closed-face + pressure tests), `a009247` (Windows Release/Ubuntu Debug CI), `0561c7f` (no gain-fallback across closed faces in experimental partial cells).

The earlier Ubuntu Debug workflow [43361541611](https://github.com/buddas-bible/zonai/actions/runs/43361541611) successfully ran the new connected-remap comparison, including actual connected transfer. At identical World8/Fluid4 pressure settings (pooling OFF for both alternatives), the original tracer scheme reported unresolved mass of ~1.4432866e-5 kg at frame 5/substep 3 and ~6.8192834e-5 kg at frame 19/substep 3. The new option reported zero unresolved mass at both snapshots. However **source (18,13) stored mass after MAC** changed from ~0.0014046745 kg (baseline) to ~0.0005070626 kg (new mode) at frame 19/substep 3. This is a large redistribution difference, not proof of more physically correct fluid motion. The original run applied ~3.0e-4 kg of connected transfer over its *printed* experimental records, which are incomplete for zero-unresolved frames; do not report that as a total simulation transfer. Later CI for the strict no-closed-fallback commit: [43361806369](https://github.com/buddas-bible/zonai/actions/runs/43361806369); check job conclusions separately.

### Applied connected-transfer face-capacity probe — 2026-11-13

Added `sandboxWindStepBenchmark --small-cell-connected-capacity-audit` on the **real** h=0.25 m, World8/Fluid4, two-way ON, Drag=0 trajectory (code `020d320`, CI `e8fc99c`, scope fix `de05465`). This is read-only and does not change pressure, Body reaction, MAC velocity, swept remap or the experimental connected-residual distribution.

At the moment each *applied* connected-residual transfer is recorded, the audit reads that donor's last snapshot fraction, original stored tracer mass, endpoint face aperture, signed solid Swept outward volume and signed endpoint MAC outward volume from the **same Fluid substep**. It reports the positive MAC/Swept volume multiplied by donor pre-remap concentration **separately** as kilogram-equivalent comparison scales. These are **not time-centered flux integrals, conservation caps or quantities that should be added**: a moving-solid swept term and a later MAC transport stage are different mechanisms. The event data also classify transport against the sign of the endpoint MAC face and transport through nearly closed apertures.

Workflow [43372382516](https://github.com/buddas-bible/zonai/actions/runs/43372382516) concluded **success on both Ubuntu Debug and Windows Release**, including the new face-capacity audit. The Ubuntu Debug log reported **59 emitted substep capacity records** (only substeps with applied connected transfers) and **966 directed transfer events**:

| Read-only risk metric | Observed value |
| --- | ---: |
| Records with endpoint MAC volume inward/zero on the chosen face | 341 events |
| Connected residual transferred by those events | 0.000172069171421 kg |
| Events with aperture below 0.001 | 2 |
| Minimum aperture used for a positive connected residual transfer | 0.0000572269055 (frame 10, Fluid substep 3) |
| Transfer mass on *any* aperture below 0.01, summed over printed events | 2.004276986441e-8 kg |
| Largest **single-event** connected tracer transfer / positive endpoint MAC kg-equivalent | 94.8656007 (among events with a positive reference) |

**Windows Release cross-check** from the same successful workflow: 59 emitted substeps, 959 directed transfer events, 340 events with no positive endpoint-MAC outward volume (total 0.000172069181869 kg), 2 transfers using a face aperture below 0.001, minimum open fraction 0.0000572343561 at frame 10 / Fluid substep 3, and largest event-to-positive-endpoint-MAC-equivalent ratio 94.865268 at frame 9 / Fluid substep 1. These are closely aligned with Ubuntu Debug but **not bitwise-identical**; do not assert identical event counts across different build configurations.

The minimum observed open fraction is **0.00572%**. The 94.9× value is a diagnostic **event-level** ratio, not proof of a broken physical conservation law. A printed sum of all per-event MAC/Swept equivalent masses may repeat a face if multiple transfer events share it; **do not interpret such an aggregate as unique available flow capacity**. Actual MAC tracer transport occurs later and is already independently accounted for in its own stage. The 341 opposing-direction events warrant interpretation against the moving obstacle's local displacement; a negative endpoint MAC flux alone does not prove an invalid geometry-driven remap.

The first submitted capacity audit failed compilation because a coverage flag was declared inside the variant loop but referenced after it; `de05465` fixed the scope. It is the corrected commit that ran the passing Ubuntu audit.

**Decision:** preserve the original default-OFF experiment; do not turn the proxy comparison into a hard speed/mass clamp. Next isolate the actual two tiny-face events by source/destination and compare unique-face grouped transfer, the corresponding swept time integral and MAC endpoint flux, then consider a time-integrated open-aperture physical capacity model with independent geometry sampling. Inspect both trajectory and timing before accepting a change.

### Grouped physical-face transfer and time-mean open-aperture quadrature — 2026-11-13

This follow-on uses the **same real h=0.25m, World8/Fluid4, two-way ON, drag 0** moving-obstacle replay, with connected tracer rerouting enabled **only for the experimental variant**. The added diagnostic `setSweptApertureAuditEnabled(true)` samples open geometry independently at exactly the midpoint time nodes selected by the existing Swept solid-flux integrator; disjoint analytic shape-face clipping is used where available, otherwise the existing subface sampling convention. The measured `timeMeanOpen` is the quadrature estimate of `(1/dt)∫ A_open(t)/h dt`, **not** `∫ A_open(t) u(t) dt`. Multiplying that average aperture by the *endpoint* MAC velocity defines an intentionally labeled **constant-endpoint-speed proxy**, not an exact time-integrated mass-flux capacity. The swept solid-normal volume is another independent signed integral, not interchangeable with this open-fluid cross section.

The new `--small-cell-grouped-face-audit` sums applied `connectedResidual` events into unique **physical U/V faces** (both directed signs retained). MAC and Swept reference data are read only **once per physical face and Fluid substep**, using the appropriate positive/negative donor concentration, so duplicate callbacks cannot inflate the reference values. The complete whole-grid per-stage tracer closure from previous audits remains active. New code commits: `6ded9c4` (opt-in geometry API), `820c353` (independent midpoint geometry quadrature), `dd19675` (sandbox setting), `64f8a8f` (unique-face group and detailed risk CSV), `8a1d871` (four-direction two-grid-size regression), `3232959` (CI), `6247446` (explicit header). Workflow [43377590818](https://github.com/buddas-bible/zonai/actions/runs/43377590818) **Ubuntu Debug passed both the normal suite and new grouped audit**; confirm the Windows job separately.

The successful Ubuntu Debug log contains **59 Fluid substeps with connected transfers, 966 callback events but only 727 unique face/substep groups**; 192 of these physical faces had multiple callbacks. The grouped gross tracer transfer was ~0.0027214295955 kg, equal to the independently recorded actual event ledger. The following are *not unique global fluid capacities*: grouped endpoint MAC kg-equivalent over time ~0.00340362244, grouped constant-endpoint-speed/time-mean-aperture proxy ~0.00340892749, and grouped positive outward Swept solid kg-equivalent ~0.000164709834. They belong to different mechanisms and cannot be added or used as global conservation acceptance bounds.

**Both extreme endpoint-aperture faces occurred at frame 10, Fluid substep 3**:

| Unique face | Applied connected transfer | Endpoint open fraction | Swept-time mean open fraction | Solid Swept signed volume | Endpoint MAC signed volume |
| --- | ---: | ---: | ---: | ---: | ---: |
| V(17,13), bottom→top | 2.93198744446e-10 kg | 6.39424833935e-5 | 0.00565211242065 | -8.64491885295e-5 m³ | -1.87414818432e-6 m³ |
| U(18,13), right→left | 1.031502564e-9 kg | 5.72269054828e-5 | 0.00508011784405 | +5.98237034865e-4 m³ | +1.67217895826e-6 m³ |

Signed volumes above use **positive canonical right/up** orientation (independent of the event's direction). Both events transport tracer **opposite the measured endpoint MAC normal flux and solid Swept normal flux**. Importantly, their average apertures during the step were ~80–90× larger than the tiny endpoint aperture: endpoint-only thresholds would misclassify these as nearly closed throughout the whole interval. Their absolute transferred masses are small. **Neither fact validates the direction of transport.**

A separate much larger per-event discrepancy is **U(17,12), right→left at frame 9/substep 1**: endpoint aperture 0.68497616, time mean 0.68529046, applied ~2.87497474122e-6 kg, endpoint-MAC donor kg-equivalent ~3.03057664596e-8 kg, giving grouped ratio ~94.82×. This is **not a tiny-aperture failure**; the endpoint MAC flux magnitude is small relative to a geometry-only residual transfer. Since the MAC velocity is an endpoint observation, this ratio is a high-priority **model discrepancy flag**, not an exact physics-law violation.

**Windows Release independent cross-check:** workflow [43377590818](https://github.com/buddas-bible/zonai/actions/runs/43377590818) finished **success on both Ubuntu and Windows**, including the new grouping test and unchanged general suite. Windows Release observed 727 unique face/substep groups, 959 events and 190 multi-event faces (Ubuntu: 727/966/192). Grouped gross connected remap was 0.00272142933497 kg. The same two tiny physical faces recur at frame 10/substep 3: V(17,13), endpoint open ~6.39508e-5, time-mean open ~0.00565212, transferred ~2.93237e-10 kg; U(18,13), endpoint open ~5.72344e-5, time-mean ~0.00508013, transferred ~1.03164e-9 kg. The high-ratio face U(17,12) at frame 9/substep 1 transferred ~2.874975e-6 kg at time-mean opening ~0.68529046, ~94.82× its *positive endpoint MAC donor kg-equivalent* proxy. Platform floating-point/build differences cause small event-count differences, not a changed physical conclusion.

The *default OFF* path is unchanged. The next physical acceptance task should derive or independently sample the **full space-time open-fluid volume flux** and distinguish interface displacement from fluid advection, rather than imposing a universal fraction cutoff or a hard velocity/pressure clamp.

### Remaining S2c acceptance gate

1. **Physical acceptance, not just mass conservation:** compare the new endpoint-aperture transfer with the *time-integrated open-face* fluid capacity, moving-solid surface flux, pressure source and local concentration. Identify false evacuation through tiny apertures and unphysical redistribution into inflowing neighbors. Treat total tracer mass as necessary but insufficient.
2. Extend controlled tests to **almost closed faces, fully disconnected pockets, newly reopened faces**, nonlocal fully covered fallback, rotation, h/cadence variation and Body force/trajectory differences. The new experiment handles **partial** donors only. Verify no change to tracer-disabled and default-off paths.
3. On the same actual World8/Fluid4 source (18,13), independently compare baseline/pressure-pool and baseline/connected-tracer alternatives, plus combined alternatives **only after** each independent effect is understood. Compare same-state pressure continuity and separate continuous histories. Never add endpoint MAC kg-equivalent flux to Swept tracer kg and call it a conservative space-time flux.
4. Measure normal-step overhead (feature disabled) and optional observer/remap overhead enabled. Keep **both experimental controls OFF by default**, PR #326 Draft, and PR #322 sampling independent. No solver merge until supported by physical and performance acceptance evidence.

