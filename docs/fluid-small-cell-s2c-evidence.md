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

### Time-interpolated moving-face MAC proxy — 2026-11-13

A full **measured** time history of the MAC fluid velocity is **not available** within one current Fluid Step: advection and pressure produce a new end-of-step field. It would therefore be incorrect to call an endpoint MAC velocity or a linearly reconstructed velocity an exact time-integrated fluid flux. This S2c experiment adds an explicitly **hypothetical interpolation model** for *diagnosis only*, with no effect on the authoritative physics solver, Tracer transport, or Body feedback:

```text
A0 = average_t(open_fraction(t))
A1 = average_t(alpha * open_fraction(t)), alpha = (t-t_start)/dt
u(alpha) = (1-alpha)*u_at_step_entry + alpha*u_after_pressure
V_linear_proxy = cellSize*dt * [u_at_step_entry*(A0-A1) + u_after_pressure*A1]
```

The prior midpoint open-face geometry sampling accumulates the extra first moment `A1` at the **same swept-geometry time nodes**, while the diagnostic stores true MAC entry velocities only when enabled. The proxy is evaluated after pressure projection, so `u_after_pressure` is the projected MAC endpoint. Its sign is canonical +U/+V; sign reversal for a donor is applied by the comparison test, not by the field query. The proxy is not a flux through a moving solid boundary; `solidSweptOutwardVolume` remains separate. Units are unit-depth m³. Multiplying this proxy by *pre-Swept source tracer concentration* yields a **comparison scale** in kg, not an independently verified transported tracer mass. Both velocity interpolation and donor concentration history remain assumptions.

Code: `5cb8eba` (API and opt-in scratch), `84d9464` (entry-velocity snapshot, first open-area moment, proxy volume query), `eace805` (unique-face directional A/B output), `14fd2e8` (validity/clear regressions), `feafa61` (Ubuntu/Windows CI). A further independent controlled test `0b59615` checks the identity `A(t)=1 => V_linear_proxy=cellSize*dt*(u_at_entry+u_after_pressure)/2` at a face untouched by a moving obstacle. The corresponding new CI should be checked separately.

Workflow [43379203133](https://github.com/buddas-bible/zonai/actions/runs/43379203133) **passed on both Ubuntu Debug and Windows Release**, including the time-interpolated audit. The subsequent identity-test code CI is [43379962325](https://github.com/buddas-bible/zonai/actions/runs/43379962325); check its final platform jobs separately. It emitted 59 Fluid substeps with actual connected residual transfers, representing the same 727 unique face/substep groups from the earlier A/B scenario. For comparison, on **only those faces selected by the experimental tracer remap**, the sums over those substeps were:

| Read-only metric | Ubuntu Debug |
| --- | ---: |
| Actual experimental connected tracer remap | 0.0027214295955 kg |
| Endpoint MAC face/positive donor kg proxy | 0.00340362243952 kg |
| Constant-endpoint-speed/time-mean-aperture kg proxy | 0.00340892748618 kg (previous audit) |
| **Linear-velocity/time-varying-open-aperture kg proxy** | 0.00338656638747 kg |
| Face/substep groups transporting mass against the signed linear proxy | 311 |
| Total mass on those opposing-direction groups | 0.00017206917142 kg |
| Largest single-group ratio to a positive linear proxy | 29.0476× (but transfer only **7.36e-13 kg**) |

Avoid comparing those summed proxies as a physical field-wide capacity: they include *only selected connected residual faces*, ignore other advection routes and assume constant donor concentration across each interval. The largest ratio occurs on a near-zero-mass event, so is not by itself evidence of material transport error.

A more significant discrepancy persists at **U(17,12), frame 9 / Fluid substep 1**: connected transfer ~2.87497474e-6 kg, endpoint MAC donor proxy ~3.03058e-8 kg (~94.82×), and linear-velocity proxy ~1.40779e-7 kg (**20.42×**). Its endpoint and time-mean apertures are ~0.685, so a tiny aperture is *not* the cause. At frame 10 / Fluid substep 3, V(17,13) and U(18,13) both have endpoint opening ~6e-5 but time-mean opening ~0.005; their extremely small connected transfers ~2.93e-10 and ~1.03e-9 kg remain opposite the linear-interpolated proxy direction. Endpoint aperture alone is therefore a poor physical filter, and changing the flux proxy alone does not justify transporting tracer against it.

**Windows Release cross-check:** 59 Fluid substep records, 727 unique physical-face groups, actual connected residual transfer ~0.002721429335 kg, linear-speed proxy ~0.003386566390 kg, endpoint proxy ~0.003403622428 kg, and 311 face/substep groups with transport opposite the signed linear proxy (~0.000172069182 kg). The largest recorded group ratio was ~29.11× but involved nearly zero actual transported mass. U(17,12) at frame 9/substep 1 still transferred ~2.87497518e-6 kg versus linear proxy ~1.40779170e-7 kg (~20.42×). V(17,13) and U(18,13) at frame 10/substep 3 remained opposite the signed linear flux. Thus the qualitative observation is reproduced under different platform/build floating-point behavior, but event bits and peak near-zero ratios are not identical.

**Next gate:** independently resolve intermediate MAC velocities using actual **smaller Fluid substeps** and record per-substep geometry and face flux along matched prescribed obstacle trajectories, without assuming linear velocity between two endpoints. The reference must distinguish geometry-only displacement, moving-solid swept flux, fluid flow, and tracer advection. No transport cap, tuning constant, or solver merge should be introduced merely to reduce diagnostic ratios.

### Prescribed moving-obstacle, solved substep reference — 2026-11-13

A **new, independent controlled experiment** now uses `sandboxWindStepBenchmark --small-cell-prescribed-substeps-audit`. Unlike the previous single World8/Fluid4 A/B scenario, it explicitly prescribes the **same translation path** for one kinematic analytic circle: radius 0.58m, start (-0.38,-0.10)m, finish (-0.14,+0.06)m in 1/30s, h=0.25m on a 32×32 grid. The starting obstacle is **motionless during initialization**, and each cadence receives the same two velocity injections *after* that initialization. No Body/World integration is performed during the comparison. Pressure pooling and connected-residual Tracer remap remain disabled.

For Fluid substep counts **1, 2, 4, 8, 16**, the harness runs actual smaller `fluidField2::step` solves at interpolated scripted obstacle poses and the same prescribed rigid velocity. Ten interior MAC U/V faces are monitored, including a far-field control. Each real substep records: (1) the accepted swept midpoint open-aperture time average, (2) the signed MAC **endpoint** volume from `getCutCellVolumeBudget`, (3) `getSwept[UV]LinearVelocityProxyVolume` based on **that substep's actually solved entry/exit MAC velocities**, and (4) the independent signed solid Swept normal volume. It asserts identical final geometry within tolerance, finite flux, physical `0≤open≤1`, consistent geometric open-time accumulation across cadences, and a nontrivial velocity/geometry resolution difference. The 16-substep run is an **internal numerical reference**, *not* an exact continuous Navier–Stokes solution or proof of temporal convergence.

Code commits: `795133f` (reference), `7b27b3a` (Ubuntu Debug/Windows Release CI), `75ef36c` (ensure zero initial solid boundary velocity and identical starting states). In [CI 43382460461](https://github.com/buddas-bible/zonai/actions/runs/43382460461), **both Ubuntu Debug and Windows Release succeeded**, including the new prescribed-trajectory audit and existing suite.

The initial **Ubuntu Debug** read-only CSV reported the following 10-face RMS differences in integrated linear-proxy fluid volume from the 16-substep run (unit-depth m³), for this one deterministic trajectory:

| Actual Fluid substeps | RMS difference from 16-substep run | RMS difference in open-fraction time integral (s) |
| ---: | ---: | ---: |
| 1 | 0.00923168251 | 0.000543266639 |
| 2 | 0.00541468917 | 0.000380363122 |
| 4 | 0.00255670464 | 0.000329403940 |
| 8 | 0.00131424301 | 0.000329403940 |
| 16 | 0 (reference by definition) | 0 |

For example, **U(17,16)** moved from coarse 1-substep integrated linear proxy 0.00194710538m³, to 2:0.00662258757, 4:0.0162076353, 8:0.0179972764, and 16:0.0197106356m³; its **coarse endpoint MAC volume was 0** despite time-mean partial opening (open-fraction time integral ~0.0100000s, vs ~0.0114583s in the fine run). The *same* final obstacle pose does **not** imply equal interior fluid velocity histories. A second face U(18,16) was always open (time integral ~0.0333333s) but its coarse linear proxy 0.0244184m³ still differed from the 16-substep 0.0409870m³: the temporal error cannot be attributed entirely to aperture aliasing.

**Windows Release cross-check:** All 50 per-face/cadence samples completed. The same ordered five-cadence RMS signed-linear-flux differences from the 16-substep run were **0.00923168253, 0.00541468889, 0.00255670542, 0.00131424330 and 0 m³**, compared with Ubuntu's 0.00923168251, 0.00541468917, 0.00255670464, 0.00131424301 and 0. The geometry open-time RMS values matched to the displayed precision. U(17,16) 16-substep linear flux was 0.0197106363 m³ on Windows versus 0.0197106356 m³ on Ubuntu; the single-step endpoint volume was **zero on both**. This is a reproducible **numerical resolution effect** with very small platform differences. No claim is made that 16 is converged or that the coarse/fine pressure solutions have identical physical trajectories.

**Limits / next validation:** This experiment deliberately isolates prescribed **translation** and does not yet reconstruct instantaneous MAC velocities at arbitrary continuous times or model rotation, multi-obstacle interaction, deforming joints or truly coupled World trajectories. Intermediate velocities are solved at **substep endpoints**, but the open-aperture×velocity integrand inside each fine substep still uses a **linear interpolation**. Increasing 16→32/64 and repeating at different h/cadences is necessary before claiming convergence; a 16-step self-reference has exactly zero error *by definition*. The strong observed dependence on cadence argues against adopting any universal tracer cap from the coarse endpoint or linear-speed proxy. Retain default OFF/PR Draft.

### 32/64-step reference with independent translating circle and rotating capsule — 2026-11-13

**Implementation and reproducible command:** `sandboxWindStepBenchmark --small-cell-prescribed-convergence-audit` in commit `910033f`, CI wired on Ubuntu Debug and Windows Release by `db50ced`. The original `--small-cell-prescribed-substeps-audit` keeps its historical 1/2/4/8/16-step, 16-step-reference behavior. The extended command runs **1/2/4/8/16/32/64 real Fluid solves** per scenario, reinitializing the field separately for each cadence with the same prescribed geometry and the same two post-prime fluid velocity injections. All runs use h=0.25m, a 32×32 domain and total time 1/30s.

- **Translation control:** Circle of radius 0.58m, from (-0.38,-0.10)m to (-0.14,+0.06)m at constant translational velocity, no rotation
- **Pure rotation:** Elongated capsule with local segment endpoints (-0.63,0)/(+0.63,0)m and radius 0.19m, COM held at (-0.20,-0.12)m, angle **0.18→1.28 radians** (about 63°), nonzero prescribed angular velocity, no COM translation
- The static prime step is identical in physical initial conditions for each cadence. Within each substep the program prescribes the correct intermediate position, orientation, COM velocity and angular velocity, then **actually solves** `fluidField2::step`. It checks the final geometry and orientation match at all cadences, plus the validity and the separate bookkeeping of integrated temporal aperture, signed fluid MAC volume and moving-solid Swept volume.

The new mode writes `prescribed_convergence` face rows and `prescribed_convergence_summary` rows, both labeled by motion type. Each summary computes RMS difference against **the 64-substep run on exactly the same scenario**, RMS consecutive-refinement difference, RMS integrated open-aperture discrepancy and number of face-integral sign disagreements. The maximum resolution is an *internal numerical reference*, **not a proven converged or analytic solution**. The flux within each *individual* solved substep still uses the separately documented linear entry/exit MAC velocity proxy and moving-shape midpoint aperture quadrature.

**Validation:** [Workflow #43384225037](https://github.com/buddas-bible/zonai/actions/runs/43384225037) passed **all Ubuntu Debug and Windows Release jobs**, including existing unit tests and the new high-resolution trial. The following values are RMS over the same ten monitored U/V faces, measured in unit-depth m³. Windows differences are confined to small floating-point roundoff.

| Actual Fluid substeps | Translation RMS vs 64 (Ubuntu) | Rotation RMS vs 64 (Ubuntu) | Translation RMS vs 64 (Windows) | Rotation RMS vs 64 (Windows) |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 0.00903558820 | 0.03539788717 | 0.00903558905 | 0.03539787449 |
| 2 | 0.00519515701 | 0.03085913940 | 0.00519515735 | 0.03085912648 |
| 4 | 0.00238955000 | 0.01264510102 | 0.00238955147 | 0.01264508909 |
| 8 | 0.00101694578 | 0.00486081036 | 0.00101694619 | 0.00486080582 |
| 16 | 0.00098541409 | 0.00198642210 | 0.00098541494 | 0.00198641974 |
| 32 | 0.00038555077 | 0.00113318917 | 0.00038555056 | 0.00113319777 |
| 64 | 0 (by definition) | 0 (by definition) | 0 (by definition) | 0 (by definition) |

The rotation capsule at **V(17,16)** is especially informative: at 1 Fluid step the integrated linear-proxy MAC volume is about **-4.517277e-4m³**, but at 64 steps it is about **+7.338015e-2m³**. Its signed time-integrated solid Swept volume is recorded separately; neither is a substitute for the other. Out of ten monitored faces, **four rotate-case faces and one translation-case face disagree with the 64-run flux sign at one substep**, whereas by 32 substeps that count is zero on both platforms. The comparison does **not** prove that the sign in the highest-resolution run is physically exact.

The geometry aperture quadrature itself also changes with cadence (for example, the ten-face RMS *open-time* difference between 32 and 64 steps remains ~1.36563659e-4 seconds). Thus the discrepancy is not solely velocity time integration: geometric midpoint quadrature, active-face switching and pressure projection all contribute possible temporal error. The circle and capsule are **different geometries and motions**, so this experiment does not establish a universal claim that rotation is intrinsically less stable than translation.

**Decision:** No solver change, no hard tracer transfer capacity clamp and no automatic change in the engine's Fluid Substep count. Small-cell pressure pooling, connected residual Tracer remap and temporal aperture instrumentation remain OFF by default; PR #326 stays Draft. Before physical acceptance, test further 64→128 refinement where practical, different h/rotation rates, and actual coupled World trajectories. Reconcile the sampled relative solid/fluid flux with stored tracer redistribution at the source cell, not just the signed MAC-only reference.

### 128-step temporal and h=0.25/0.125m matched-face refinement — 2026-11-13

The new read-only `sandboxWindStepBenchmark --small-cell-prescribed-spatial-audit` compares the **same prescribed translation and pure rotation trajectories**, the same physical initial velocity injections, and **real 16/32/64/128 Fluid solves** at **h=0.25m (32×32)** and **h=0.125m (64×64)**. It does not change the production fluid timestep or solver. Commits: `98e3d33` (reusable reference with spatially aligned face aggregation, row-by-row temporal/spatial CSV) and `d65b7c2` (Ubuntu/Windows CI). The old 1..16 and 1..64 commands remain unchanged.

**Essential spatial comparison rule:** Each monitored face represents a fixed **0.25m physical line segment** in world coordinates. At h=0.125m, **two adjacent U or V subfaces** cover that same segment. The benchmark **sums** actual sampled fluid flux and moving-solid Swept volume over the two fine faces, but **length-averages** the open-fraction time integral. Thus a half-length face's standalone flux is not incorrectly compared to a full-length coarse face. Initial geometry/velocity sources are specified in world space and all candidates of one grid size must end at the same obstacle pose and Cut-cell occupancy. This comparison includes the expected spatial-discretization effect of sampling the same world-space initial source at different h.

The high-resolution output distinguishes:

- `prescribed_spatial_temporal_summary`: RMS integral-fluid-flux error of 16/32/64-step solutions against **128 steps at the same h**, plus adjacent-step RMS, open-time RMS, and face-flow sign disagreement count
- `prescribed_spatial_cross_summary`: RMS fluid-flux difference, open-time difference and sign mismatches **between h=0.25 and h=0.125** at each *matched cadence*, over the exact same ten physical 0.25m faces
- Fine-grid flux is summed without reusing the same physical face twice; the coarse-segment open-time is a length average. The calculated flux inside each actual solved substep **still uses the same documented entry/exit linear MAC velocity assumption**. Neither 128 substeps nor h=0.125m constitutes an analytic solution.

**Cross-platform verified:** [Workflow #43388443571](https://github.com/buddas-bible/zonai/actions/runs/43388443571) finished **success for both Ubuntu Debug and Windows Release**, including the high-resolution time-space audit, baseline suite and lifecycle tests.

| Motion | h (m) | RMS fluid flux 32 vs 128 (m³) | RMS 64 vs 128 (m³) |
| --- | ---: | ---: | ---: |
| Translation circle | 0.25 | 0.000384344348 | 0.000286105648 |
| Translation circle | 0.125 | 0.000616705984 | 0.000252585469 |
| Pure rotating capsule | 0.25 | 0.001033941631 | 0.000507747638 |
| Pure rotating capsule | 0.125 | 0.000626045508 | 0.000481830656 |

At each h, the finer 64-step approximation gets closer to 128 than 32 in the reported RMS, but **the 64→128 difference is still nonzero**. The following compares **two different spatial resolutions at equal real Fluid substeps**, rather than incorrectly treating high temporal resolution as a space-convergence proof:

| Motion | h=0.25 vs 0.125 RMS at 16 steps (m³) | At 64 steps | At 128 steps |
| --- | ---: | ---: | ---: |
| Translation circle | 0.001365246942 | 0.000755847246 | 0.000559086157 |
| Pure rotating capsule | 0.003348989255 | 0.001876010446 | **0.002211549749** |

**Windows Release independent numerical check:** It reproduced all sixteen time-refinement summary rows and eight matched-grid summary rows with only minor floating-point variation. At 128 Fluid substeps, the Windows translation h=0.25→0.125 physical-face RMS was **0.000559083849m³** (Ubuntu 0.000559086157), and rotation RMS was **0.002211554714m³** (Ubuntu 0.002211549749). The rotation discrepancy also **increased** compared with the Windows 64-substep h-to-h RMS of **0.001876016573m³**. At fixed h=0.125m the Windows 64→128 temporal differences were translation **0.000252586382m³**, rotation **0.000481837450m³**, matching Ubuntu's ~0.000252585469 and ~0.000481830656. Both jobs fully succeeded; differences are small build/float effects, not bitwise identity.

**Important negative result:** The rotating capsule's spatial difference **increased** between the 64-step and 128-step comparisons. At 128 steps, **V(16,16)** contributed the largest single matched-face difference: h=0.25 integrated linear MAC ~0.0254583m³ versus h=0.125 ~0.0318628m³ (difference ~-0.0064045m³). Because the metric includes discretization, nonlinearity, and geometry quadrature, it does **not** prove a specific solver defect. However, it rules out claiming space convergence from finer temporal stepping alone.

**Decision:** The primary S2c physical acceptance criterion remains unsatisfied. Do not convert endpoint or time-linear MAC observations into a global tracer-transfer cap. Before any default-on change, investigate why rotating interfaces produce a non-monotonic cross-grid response, test h=.0625m if justified by cost and relevance, and compare the **same sampled World/Fluid moving-body state** under different fluid cadences. S2c pressure pooling and connected-residual remap remain default OFF, PR #326 remains Draft, and PR #322 stays independent.

### V(16,16) pure-rotation pressure-stage split and geometric exposure — 2026-11-13

A **new read-only, pressure-stage-specific experiment** `sandboxWindStepBenchmark --small-cell-rotation-pressure-split-audit` (commit `1de7410`, corrected CSV formatter `cdec94f`, cross-platform CI wiring `8d7ed39`) replays the earlier worst physical face **V(16,16)** in a fixed pure-rotation capsule path. It solves **64 and 128 actual Fluid steps**, once on 32×32 h=0.25m and once on 64×64 h=0.125m. The physical segment is **one coarse V face** or **two adjacent fine V faces**, always representing the same 0.25m world-space line. Solid displacement/pose, initial velocity injections and total duration are prescribed identically at each grid/cadence; the pressure-pool and connected-remap experiments are both OFF.

At **every** Fluid substep, the existing `setPrePressureSnapshotObserver` captures the unchanged field's pressure-input MAC face volume and the optional time-linear aperture×MAC velocity proxy **after advection/boundary processing but before pressure projection**. Following the normal pressure solve, the same two values are read again from the actual Field. We sum the two half-width fine faces' signed volumes exactly once and length-average their aperture samples. The differences `post - pre` are explicitly **pressure/projection-stage changes**, not new Swept volume; they also include any normal pressure-stage boundary synchronisation. We separately accumulate the existing signed solid Swept volume, length-averaged open-time and endpoint aperture over the *same physical segment*. The test checks each signed ledger closes at the individual run and cross-grid levels, and never alters the pressure operator.

For a matched coarse-minus-fine signed **time-linear-proxy fluid volume** `Δpost`, the test also provides the exact algebraic split

`Δpost = Δpre + Δprojection = (E_coarse - E_fine) * uEffectiveFine + E_coarse * (uEffectiveCoarse - uEffectiveFine) + Δprojection`,

where `E = physicalFaceLength * (integrated mean open fraction over time)` is the shared-face aperture exposure and `uEffective = preProxyVolume / E` is the exposure-weighted **pre-projection** velocity. This is an identity for the measured quantities, **not a causal diagnosis**: pre-projection velocities at later substeps already contain previous pressure projection and prior velocity advection. The linear speed assumption remains in each individual substep proxy, so neither the proxy nor its decomposition proves the exact physical space-time flux.

**Cross-platform validation:** [CI #43392265568](https://github.com/buddas-bible/zonai/actions/runs/43392265568) completed **success on both Ubuntu Debug and Windows Release**, including the new rotating-face pressure-stage audit. The target V(16,16) Ubuntu results are:

| Metric, signed coarse minus fine | 64 Fluid steps | 128 Fluid steps |
| --- | ---: | ---: |
| Geometry aperture exposure contribution (m³) | -0.000211261707 | -0.000871465018 |
| Pre-projection effective velocity contribution (m³) | -0.004778846030 | -0.005632295548 |
| Current projection-stage contribution (m³) | +0.000025341898 | +0.000099265404 |
| **Total time-linear-proxy fluid volume difference (m³)** | **-0.004964765839** | **-0.006404495162** |
| Endpoint MAC volume: pre-projection difference (m³) | -0.006098198200 | -0.006883569619 |
| Endpoint MAC volume: pressure-stage difference (m³) | +0.000163801884 | +0.000146038997 |
| Solid Swept volume coarse / fine (m³) | 0.060679928807 / 0.060580896934 | 0.061029051081 / 0.060355983846 |
| Aperture open-time coarse / fine (s) | 0.009375000489 / 0.009440104659 | 0.009244792149 / 0.009505208829 |

At 128 steps, the largest signed magnitude in the exact decomposition is the **pre-projection velocity term** (-0.0056323m³). The measured **current** pressure-stage difference is a much smaller positive offset (+0.0000993m³), reducing the final coarse-minus-fine discrepancy. This **does not rule out past pressure history** or geometry-driven state divergence as the source of the pre-projection velocity difference. The geometric exposure term (-0.0008715m³) is meaningful but not the only contributor. The solid swept signed volume remains independently measured; **do not add it to the MAC fluid proxy as if it were a single-face mass transfer**.

The high-difference period is concentrated after the capsule reaches ~0.74 rad. Around **0.91–0.94 rad** (128-step indices ~84–88), the two resolutions can report the **same mean-open fraction (0.5)** and **zero signed Swept face-volume difference for the individual step** while the **pre-projection fluid transport still differs**, up to roughly -0.00033846m³ for one substep. This localizes the issue beyond a *current-step aperture-only* explanation. The sign and size can vary across substeps and pressure updates.

**Windows Release independent cross-check:** The same **384 per-substep traces** and **two cross-grid ledgers** passed on Windows. The measured signed coarse-minus-fine time-linear-proxy volume differences at 64 and 128 substeps were **-0.004964761113m³** and **-0.006404509368m³** (Ubuntu: -0.004964765839 and -0.006404495162). At 128 substeps, Windows decomposed the total as geometry **-0.000871465163m³**, pre-projection effective velocity **-0.005632309358m³**, and current pressure/projection-stage change **+0.000099265152m³**. The two platform results are extremely close, not bit-for-bit identical. The Windows 128-step solid Swept volumes were 0.061029051140m³ coarse and 0.060355983875m³ fine. This supports the same diagnosis: a majority of the observed coarse-vs-fine difference is present **before each current pressure projection**, while accumulated prior pressure/geometry/advection histories remain plausible contributors.

**Decision / follow-up:** Keep the new observer and the experimental tracer path OFF by default. The next useful counterfactual is to hold a *single identical pre-projection fluid state/geometry* fixed while varying only the pressure discretization/face sampling, then separately test propagation of previous-step velocity differences. This would distinguish a **direct projection-operator discrepancy** from **indirect accumulated solver history**. Also return to the actual World8/Fluid4 disappearing-cell tracer source with this independent face staging; do not insert a global tracer cap before a physically accepted space-time flux bound.

### Remaining S2c acceptance gate

1. **Physical acceptance, not just mass conservation:** compare the new endpoint-aperture transfer with the *time-integrated open-face* fluid capacity, moving-solid surface flux, pressure source and local concentration. Identify false evacuation through tiny apertures and unphysical redistribution into inflowing neighbors. Treat total tracer mass as necessary but insufficient.
2. Extend controlled tests to **almost closed faces, fully disconnected pockets, newly reopened faces**, nonlocal fully covered fallback, rotation, h/cadence variation and Body force/trajectory differences. The new experiment handles **partial** donors only. Verify no change to tracer-disabled and default-off paths.
3. On the same actual World8/Fluid4 source (18,13), independently compare baseline/pressure-pool and baseline/connected-tracer alternatives, plus combined alternatives **only after** each independent effect is understood. Compare same-state pressure continuity and separate continuous histories. Never add endpoint MAC kg-equivalent flux to Swept tracer kg and call it a conservative space-time flux.
4. Measure normal-step overhead (feature disabled) and optional observer/remap overhead enabled. Keep **both experimental controls OFF by default**, PR #326 Draft, and PR #322 sampling independent. No solver merge until supported by physical and performance acceptance evidence.

