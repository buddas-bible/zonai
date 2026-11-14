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

### Same frozen pre-projection state: Jacobi 40/120/200 iteration control — 2026-11-13

**Purpose:** The previous `V(16,16)` rotating-capsule investigation found that, at h=0.25m vs h=0.125m, most of the measured signed fluid-face transport discrepancy already existed **before the current pressure projection**. That difference can contain *past* pressure, advection and geometry effects. A pressure-stage split alone cannot identify whether the **current Jacobi iteration budget** is the cause. This new test isolates precisely the current projection solve from a fixed input, without claiming identical grid-discretized states across two different h.

**Implementation:** `replayFrozenCutCellPressure(int pressureIterationsOverride = -1)` accepts an optional diagnostic iteration budget in **[1,200]** on a one-use copy captured by the existing `setPrePressureSnapshotObserver`. The original no-argument replay retains the frozen Step iteration budget; invalid overrides return `nullopt` without consuming a copy. This overload does **not** change production `fluidField2::step`, pressure operator, warm start, geometry, advection or coupling. A unit regression verifies default-versus-explicit identical-budget replay equivalence, finite 200-iteration output and rejection of invalid values.

The command `sandboxWindStepBenchmark --small-cell-frozen-pressure-iterations-audit` reproduces the previous pure capsule rotation with normal **120-iteration live** Fluid steps at 64 or 128 temporal subdivisions, on both h=0.25 and h=0.125. At exact fractions **11/16**, **12/16** and **16/16** of the total prescribed rotation, the observer receives the *actual* pre-projection state, then independently deep-copies it three times and applies **40, 120 or 200 Jacobi iterations** on those copies. The copies share identical velocity, geometry, swept volume, tracer state, pressure history *up to this moment*, pressure source and obstacle pose within each h/cadence. Their only intentional difference is the pressure iteration budget. The live Fluid field is unchanged and must agree numerically with the independently replayed 120-iteration result. Tests assert all cross-grid `post = pre + current projection` face ledgers close and both the projected face flux and iteration sensitivity are exercised.

Code: `24a049b` (optional copy-only API), `10bb3e6` (bounded replay), `44c4074` (unit coverage), `9ecf6e1` (frozen rotating control) and `66c5d06` (Ubuntu/Windows CI). [Workflow #43413289439](https://github.com/buddas-bible/zonai/actions/runs/43413289439) **passed on both Ubuntu Debug and Windows Release**, including the new frozen-pressure comparison, unit replay semantics and existing complete regression suite.

**Ubuntu Debug target physical face results** (signed coarse h=0.25 minus fine h=0.125, one corresponding **current** Fluid Substep, unit-depth m³):

| Actual Fluid substeps | Rotation fraction | Before pressure, identical across replay budgets | After replay at 40 iterations | After replay at 120 iterations | After replay at 200 iterations |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 11/16 (~0.936rad) | -0.000639748684 | -0.000525885351 | -0.000527974308 | -0.000528546388 |
| 64 | 12/16 (~1.005rad) | -0.000256405489 | -0.000257876420 | -0.000258971562 | -0.000259045354 |
| 64 | 16/16 (1.280rad) | -0.000322752719 | -0.000325398397 | -0.000324764733 | -0.000324705819 |
| 128 | 11/16 (~0.936rad) | -0.000220951061 | -0.000220408325 | -0.000220680543 | -0.000220777494 |
| 128 | 12/16 (~1.005rad) | -0.000131374317 | -0.000131987686 | -0.000132014943 | -0.000131999871 |
| 128 | 16/16 (1.280rad) | -0.000172087672 | -0.000173800677 | -0.000174259171 | -0.000174370879 |

At 128 steps, 11/16 rotation, the pressure-only iteration sweep changes the final coarse-minus-fine volume by ~3.69e-7m³ between 40 and 200 iterations, while the **fixed pre-pressure** difference is ~-2.21e-4m³. Thus the larger input discrepancy persists across the chosen projection budgets. At 64 steps, 11/16, current pressure *does* shift the difference by ~+1.12e-4m³, so neither pressure projection nor its iterative convergence should be declared irrelevant. The opt-in replay data also reveal nonzero pressure residual at 200 Jacobi iterations: finite targeted face-flux sensitivity does **not** establish global convergence of the pressure system.

**Windows Release cross-check:** The same **36 frozen pressure replay rows** and **18 matched physical-face cross-grid comparison rows** were generated, both CI jobs completed successfully. At 128 Fluid steps, rotation 11/16, Windows preserved the common pre-projection coarse-minus-fine difference **-0.000220951217m³** across all three replay budgets; the post-projection differences were **-0.000220408473 / -0.000220680714 / -0.000220777649m³** for 40/120/200 iterations, respectively. This agrees with Ubuntu's -0.000220951061 pre and -0.000220408325/-0.000220680543/-0.000220777494 post within ordinary cross-platform float variation. At 64 Fluid steps, rotation 11/16, Windows recorded -0.000639748498m³ pre and post -0.000525885072/-0.000527974028/-0.000528546140m³. The pressure-only results themselves are iteration sensitive, and a larger iteration count can reduce the reported pressure residual, but this finite budget sweep **does not** certify a converged pressure operator.

**Important scope:** "same state" means the three 40/120/200 copies **inside one fixed grid and cadence**. A coarse-grid field and a fine-grid field are different discrete states with different pressure systems; they cannot be declared identical merely because their world-space obstacle path is the same. This test isolates **current iteration-budget sensitivity**, not past pressure history or the entire difference between distinct spatial pressure operators. Further acceptance work should compare a matched cross-grid prepressure field via a justified prolongation/restriction operator, and then isolate previous-step state divergence with short controlled restarts. Keep the original coupled World/Fluid path, default physics and PR Draft status unchanged.

### Matched continuous-world MAC state and short controlled restarts — 2026-11-14

**Motivation:** The previous rotating-face snapshot trial showed that changing Jacobi iterations alone could not erase the existing coarse/fine pre-projection velocity difference. But the **normal** h=0.25 and h=0.125 experiments do not have equal discrete velocity states after dozens of advection/pressure steps. To distinguish fresh pressure/geometry discretization from **previous-step history**, the new experiment freezes each actual pre-projection Step and performs an explicit diagnostic reset to the **same continuous velocity field** at every currently-open MAC face, while leaving the scripted capsule pose and solid boundary velocity history in place.

**Implementation and isolation:**
- `setFrozenMacVelocityForAudit(std::function<vec2(vec2)>)` accepts a complete finite world-space vector field **only on a replay-ready frozen copy**, and changes only **open interior U/V MAC face components**. Closed faces retain prescribed solid boundary velocities, and a nonfinite function result rejects the entire write atomically. This feature is not called in production `step()`. The synthetic field uses `u(x,y)=0.30-0.08(y+0.12)` and `v(x,y)=0.42+0.12(x+0.20)` in m/s, so `∂u/∂x+∂v/∂y=0` on the continuous plane. The fields are sampled at each grid's **actual staggered world-face locations**: this is not an exact restriction/prolongation of a *discrete* pressure or velocity solution, and discretized aperture/source differences remain.
- `replayFrozenCutCellPressure(iterations, finishStepForRestart=true)` explicitly calls the same tracer-transport tail and commits the **current** obstacle pose as the previous pose for the *next* moving step. Its default `finishStepForRestart=false` preserves existing **pressure-only** semantics. All of this runs on user-created copied snapshots; there is no extra allocation on the ordinary Step path. Dedicated unit coverage asserts invalid synthetic fields leave the frozen state unchanged, ordinary/nonfrozen fields reject writes, a completed replay is one-use, and the copied **following real moving step** reproduces the untouched live field and tracer mass.
- `sandboxWindStepBenchmark --small-cell-matched-world-restart-audit` replays the same pure rotating capsule and two initial world-coordinate velocity injections as the preceding pressure forensic. For each h=0.25/0.125 and 64/128 real Fluid Steps, it captures pre-pressure snapshots at 11/16 and 12/16 of the rotation, creates two independent copies, and **finishes the currently frozen step at 120 iterations**: an untouched **evolved/history** copy and a **matched-world/manufactured** MAC-field copy. Both then advance **1, 2, and 4 real additional Fluid steps** on exactly the same prescribed future obstacle motion. The natural copy must reproduce the actual live Step. Every row groups one h=0.25 V(16,16) face or its **two h=0.125 neighboring V faces** representing the same physical 0.25m segment, and independently checks that the two restart options experience the **same** future time-mean open aperture and signed solid Swept face volume.

**Code:** `36dfd36` (explicit diagnostic API), `76c2c21` (atomic manufactured velocity + completed frozen-step tail), `20ed09b` (unit/next-Step identity), `230094b` (controlled synthetic/natural fork and short propagation), `a6d063a` (standard header), `3a2de88` (dual-platform CI). [Workflow #43554055856](https://github.com/buddas-bible/zonai/actions/runs/43554055856) **passed Ubuntu Debug**, including new and pre-existing suite. Windows Release cross-check is recorded separately after it completes.

**Ubuntu Debug, 128 Fluid steps, rotation angle ~0.93625rad, V(16,16):**

| Signed coarse-minus-fine current face volume (m³) | Frozen pre-pressure | After normal 120-iteration pressure |
| --- | ---: | ---: |
| Evolved state with actual preceding solver history | -0.000220074113 | -0.000219533076 |
| Same continuous affine MAC velocity field sampled at both grids | **-0.000000244140** | **-0.000117858443** |

The selected synthetic **pre-pressure** face fluxes are almost equal (coarse 0.00001494140675m³, fine 0.00001518554715m³), as expected for a physical segment having matched aperture and smoothly varying common input velocity. The respective **post-pressure** fluxes are 0.000303880638 vs 0.000421739081m³, with cross-grid difference -0.000117858443m³. Therefore the original accumulated velocity difference accounts for *part* of the original large coarse/fine discrepancy, **but a substantial new grid-dependent discrepancy is generated during projection from the matched continuous velocity field**. This projection difference combines different geometry/source discretizations, pressure operators and boundary treatment; it is **not** an exact pressure-kernel-only error measurement. Conversely, the evolved pre-pressure difference is not exclusively caused by pressure history: old geometry and advection contribute.

**Short forward propagation (Ubuntu, 128 steps, same 11/16 checkpoint)**:

| Additional actual Fluid steps after frozen projection | Evolved coarse-minus-fine (m³) | Matched-world coarse-minus-fine (m³) | Difference of gaps (m³) |
| ---: | ---: | ---: | ---: |
| 0 | -0.000219533076 | -0.000117858443 | -0.000101674633 |
| 1 | -0.000104509951 | -0.000010776365 | -0.000093733586 |
| 2 | -0.000104233255 | -0.000011874829 | -0.000092358426 |
| 4 | -0.000069634061 | +0.000019824941 | -0.000089459002 |

The geometric/swept phase at each horizon is equal between **evolved** and **synthetically matched** runs of the same grid and scripted motion. Their differences persist for the next four steps. This is a **short deterministic counterfactual**, not a statement about eventual asymptotic convergence or which restarted velocity field is physically correct. In other rotation checkpoints (e.g., 12/16), the manufactured cross-grid signed gap can switch sign, so do not generalize a single ratio into an across-the-board error reduction or a new tracer cap.

**Acceptance consequence:** S2c still lacks an independently validated *physical* space-time tracer flux bound. No production settings or pressure/Tracer equations have changed; pressure pooling and connected-residual routing remain OFF, and PR #326 remains Draft. Next investigate the **matched-world pressure stage** with a controlled geometry/source alignment or a convergence-tested reference projection (rather than only its signed target-face flux), then carry the diagnostic back to the coupled World8/Fluid4 small-cell tracer disappearance. Measure both production and opt-in probe overhead before considering defaults.

### Zero-open-MAC frozen pressure control — 2026-11-14

The prior **matched continuous-world affine** checkpoint still produced a significant
h=0.25 vs h=0.125 **post-projection** V(16,16) flux difference. This by itself
cannot assign error to the pressure operator: the two grids have different
discrete moving-boundary geometry and pressure volume sources. The
`--small-cell-zero-mac-pressure-audit` trial adds a third frozen copy at each
11/16 and 12/16 pure-capsule rotation checkpoint, for 64 and 128 actual Fluid
substeps, on both grids.

- **Natural**: preserve the actual pre-projection history; complete the copied
  step and advance up to four real subsequent Fluid steps (existing test).
- **Affine**: replace only *open interior* MAC face velocities with the same
  divergence-free world-coordinate affine field on both grids (existing test).
- **Zero-open-MAC**: on another copy of the same frozen Step, set only *open
  interior* face velocities to zero. Retain the moving capsule's swept geometry,
  volume-rate source and **closed-face moving-solid velocities**. Execute the
  ordinary 120-iteration pressure projection once, read-only with respect to
  the original field. This is **not** a stationary-wall or zero-total-RHS case.

The new per-grid CSV reports pre-pressure/post-pressure *signed physical-face
MAC volumes*, their difference, pressure residual, signed solid swept volume
and open-time exposure. The cross-grid CSV reports:

`affineProjectionGap = (affinePost_coarse - affinePre_coarse) - (affinePost_fine - affinePre_fine)`

`zeroProjectionGap = (zeroPost_coarse - zeroPre_coarse) - (zeroPost_fine - zeroPre_fine)`

`velocityDependentGap = affineProjectionGap - zeroProjectionGap`.

These are identities for two **different discrete grids**, not an exact
projection truncation-error estimate, a pressure-operator-only difference or
a physical space-time tracer-flux bound. In particular, changing open MAC
velocities also changes their divergence contribution to each pressure RHS.
The zero-input response can retain wall-motion, geometry and numerical
effects. The test asserts finite pressure, equal geometry/swept ledgers
across cloned inputs, full checkpoint coverage, and closure of the comparison
algebra. The zero-pressure replay does not execute tracer transport; tracer
independence is not inferred from a new tracer-mass regression. The original
benchmark command retains its exact behavior when the new flag is absent.

**Validated:** `98bf088` (opt-in frozen zero-MAC probe), `2923837`
(Ubuntu Debug/Windows Release CI), [workflow 43555705311](https://github.com/buddas-bible/zonai/actions/runs/43555705311)
**success on both platforms**, including full existing tests and the new
audit. Ubuntu and Windows printed the following signed physical-face
coarse-minus-fine projection changes identically to the shown precision
(unit-depth m³; negative/positive are canonical face directions):

| Fluid steps | Rotation checkpoint | Affine projection gap | Zero-open-MAC projection gap | Affine minus zero projection gap |
| ---: | ---: | ---: | ---: | ---: |
| 64 | 11/16 | -0.000236343585329 | -0.000236558181464 | +0.000000214596135 |
| 64 | 12/16 | +0.000096197078210 | +0.000097969241583 | -0.000001772163373 |
| 128 | 11/16 | -0.000117614302942 | -0.000117721445789 | +0.000000107142847 |
| 128 | 12/16 | +0.000058726451396 | +0.000059612253685 | -0.000000885802290 |

On this prescribed rotating capsule, the cross-grid response to the
**zero-open-MAC** pressure input accounts for nearly all of the affine
projection gap at the sampled checkpoints. The added velocity-dependent
projection gap is only about 0.09–1.8% of the zero-MAC gap in magnitude.
For 128 steps at checkpoint 11/16, the two grids even report the same
**target physical-face** time-mean aperture exposure (0.000130208340124 s)
and solid swept volume (0.00028198203654 m³), while their zero-input
post-pressure MAC face volumes are 0.000288905187366 m³ (coarse) and
0.000406626633155 m³ (fine). Equality of one target face's geometric
ledger is **not** equality of the full surrounding pressure discretization,
cell-volume source, connected region or boundary conditions.

**Limitations:** The replay uses a prescribed kinematic capsule and the
ordinary **120-iteration Cut-cell Jacobi** projection, not the fully coupled
Schur PCG/Body trajectory of World8/Fluid4. Raw pressure residuals on the
zero-input checkpoints remain ~0.0303–0.0464; this is not a
convergence-certified reference pressure solution. The matched fields
differ in discrete grid geometry, pressure operator, compatible source
correction and potentially wall conditions. No production stabilization or
physical tracer-capacity bound follows from this control. Defaults,
pressure/Schur implementations, Body forces, tracer routing and clamps
are unchanged; pressure pooling and connected residual remap stay Draft/OFF.

**Next:** perform a **same-frozen-zero-MAC Jacobi iteration sweep** with
pressure residual and face-flux convergence controls; audit cell-local
moving-volume source and connected-region compatibility around V(16,16).
Then apply the resulting stage separation to the actual coupled
World8/Fluid4 disappearing V(17,13) tracer scene and measure optional
diagnostic overhead before deciding on any physical correction.

### Frozen zero-open-MAC Jacobi iteration sensitivity and independently reconstructed source ledger — 2026-11-14

**Question:** Could the h=0.25 vs h=0.125 rotating V(16,16) difference
persist merely because the *current* 120-iteration Jacobi pressure projection
is not sufficiently converged? And, when all OPEN interior face velocities
are explicitly set to zero, does the source that drives pressure really
come from moving-volume geometry rather than hidden MAC advection?

**Implementation:** `--small-cell-zero-mac-convergence-audit` builds on
the previous frozen-state control without changing production code.
At the same 64/128 actual Fluid-step trajectories, and at 11/16 and
12/16 rotation checkpoints, it takes four **independent frozen copies**,
zeros only open interior U/V velocities, preserves closed-face prescribed
wall velocities and geometry, and replays pressure at exactly
**40/80/120/200 Jacobi iterations**. Each copy uses the exact same
moving-solid pressure source and pressure compatibility correction.
For each physical 0.25m target V face, the fine result is the sum of
two half-width faces. The original untouched live 120-iteration Step
and the earlier zero-open-MAC 120-iteration replay remain reference
checks. The `replayFrozenCutCellPressure` API's existing [1,200]
diagnostic iteration limit is unchanged.

The audit separately walks the **public fluid-cell and open-face graph**
to count connected fluid components and accumulate the stored
`pressureSourceDeltaVolume` for each component. It compares the
independently reconstructed global signed source + unresolved correction
against `sourceImbalance`, and the sum of per-region absolute sources
+ absolute unresolved correction against `compatibilityCorrection`.
This covers one source equation with the internal OPEN-face velocity
explicitly zero; it is **not** independent reconstruction of the
pressure matrix itself. For the cells on each side of the measured V face,
it separately records source volume, snapshot volume, Swept solid
divergence, post-pressure MAC divergence and the remaining local
continuity budget. The local cell collections are **not identical
world-space physical control volumes** between h=.25 and h=.125, so their
volume totals must not be directly interpreted as a convergence rate.
The test rejects changed geometry/sources between iteration budgets,
nonfinite results, missing checkpoints, unmatched face grouping, or a
120-iteration replay that differs from the prior zero-MAC control.

**Validated first code CI:** [workflow 43564705365](https://github.com/buddas-bible/zonai/actions/runs/43564705365)
**success on Ubuntu Debug and Windows Release**. Both platforms
printed the following signed target-face **coarse minus fine**
post-projection MAC volumes (m³) to the shown precision:

| Actual Fluid substeps | Rotation | Jacobi 40 | Jacobi 80 | Jacobi 120 | Jacobi 200 |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 64 | 11/16 | -0.000192770186 | -0.000225416831 | -0.000236558181 | -0.000243831749 |
| 64 | 12/16 | +0.000146485997 | +0.000109661687 | +0.000097969242 | +0.000091041195 |
| 128 | 11/16 | -0.000096063010 | -0.000112253495 | -0.000117721446 | -0.000121248582 |
| 128 | 12/16 | +0.000084176912 | +0.000065553844 | +0.000059612254 | +0.000056065249 |

At 128 steps, rotation 11/16, the h=.25 pressureResidual
decreased from 0.0967633 (40 iterations) to 0.0433588 (120)
to **0.0342637 (200)**; h=.125 decreased from 0.1229897
to 0.0400728 to **0.0240927**. The existing reported
`pressureResidual` for noncoupled Jacobi is a legacy residual
quantity, **not** a certified true relative PCG residual or
physical transport conservation error. In the same checkpoint
the coarse-minus-fine face-volume gap still moved by
**-3.52713608e-6m³ between 120 and 200** (~2.9% of the
200-iteration signed gap magnitude). At 64 steps/11/16 the
120→200 gap shift was **-7.27356760e-6m³**. Hence 200 iterations
are **not a demonstrated converged reference** and the
previous finite-120 cross-grid result cannot be attributed solely
to geometry or exclusively to old velocity history.

The **frozen zero-MAC moving-volume source does not vanish**.
For the local lower/upper cells adjoining the target face at
128 steps/11/16, the independently queried (unit-depth)
pressure-source volumes were approximately **0.000768736791m³**
on h=.25 and **0.000781493087m³** on h=.125. Their local
snapshot changes happened to sum to zero at this checkpoint,
while the separate signed Swept divergences were about
0.001084959491m³ and 0.000781493069m³, respectively.
These are **different geometric quantities** and may not be
substituted for a measured time-integrated fluid flux. Per-grid
source, snapshot, Swept and fluid-component topology were
invariant across the four Jacobi budgets; all reported cases
had one connected active fluid region.

**Numerical source-ledger caveat:** full-grid connected sums
nearly cancel, making their final net source of order
1e-7m³/s. The independent source/compatibility reconstruction
differed from the solver's stored float ledger by up to
approximately **3.74e-7m³/s** in the first successful
Ubuntu/Windows run. That small **absolute** difference is
not negligible relative to the much smaller signed net source.
A follow-on assertion correction (`3302f73`) narrows the
acceptance to `1e-6 + 1e-5 * max(abs(sourceRate),
regionAbsoluteSourceRate)` m³/s, rather than scaling it to
1m³/s; its own CI must pass before calling that stronger
criterion verified. Do not claim exact component-source
closure from the earlier wide-tolerance run.

**Scope/decision:** This remains one isolated **diagnostic** of a
prescribed kinematic rotating capsule using the **noncoupled
Jacobi** Cut-cell projection, not a production Schur PCG solver
change or a validation of the real World8/Fluid4 disappearing
tracer. Neither pressure pooling nor connected-residual tracer
redistribution is accepted. No default changes, speed clamps,
new fluid forces or tracer-capacity fixes are included.

**Next atomic S2c task:** obtain a genuinely convergence-tested
pressure reference for the same frozen state (bounded independent
reference solve, with residual defined against the assembled
discrete equation rather than an iteration count alone);
then repeat geometry/source/velocity-stage comparison in
the real coupled World8/Fluid4 near-closing V(17,13)
case. Avoid comparing one target face's open-time
alone as proof that two full pressure systems share their
surrounding geometry and compatibility constraints.

### Independent compatible-residual pressure reference probe — 2026-11-14

Previous same-state 40/80/120/200 Jacobi measurements did not converge
sufficiently to certify the coarse/fine target V(16,16) pressure difference.
The new opt-in `--small-cell-zero-mac-reference-audit` repeats the same
prescribed capsule rotation at 128 *real* Fluid steps and captures frozen
11/16 and 12/16 checkpoints at h=0.25/0.125. On separate frozen copies,
only OPEN interior MAC velocities are set to zero; moving-solid geometry,
closed-face prescribed velocities and pressure-source history are retained.
The original noncoupled Cut-cell Jacobi solver is replayed at
**200 / 800 / 1600 / 3200 iterations**. The frozen-copy override accepts
iterations above 200 only for obstacles without inverse mass/inertia; the
live Step and Schur PCG iteration budget remain unchanged.

For each open-face-connected fluid region, let S_i be its stored
pressure-source volume, F_i its *post-projection endpoint MAC net
outward* volume, and mean(S)_region the source volume summed over that
region divided by its active-cell count. Independently reconstruct

`compatibleRms = sqrt(sum_active((F_i + S_i - mean(S)_region)^2)/N) / (h*h*dt)`

in 1/s, and separately report the RMS of the component source mean as
an **incompatible Neumann source floor**, not a solver error.
This uses the implemented discrete source and post-projection MAC
operator, **not** an exact time-integrated physical fluid-face flux or a
new independent direct linear solver. Full domain and local moving-wall
geometry can still differ across coarse and fine grids.

The experiment checks finite output, component/source identity across
the copied budgets, zero initial open-MAC face flux, replay safety and
cross-grid face pairing. It reports `reference_criteria_met=1` only
when both grids reach compatibleRms below **1e-3 / s** and the selected
face gap is within **1e-6 m³** of the 3200-iteration gap. These thresholds
are diagnostic hypotheses, *not production acceptance*. If the criteria
are not met, the result remains **unconverged** even at 3200 iterations.

Implementation: `eef4d99` (frozen-only iteration guard),
`b0eca6f` (API contract), `211dcaf` (unit bound/one-use regression),
`d673f5b`, `67701a6` (independent residual benchmark),
`112ec36` (Ubuntu Debug and Windows Release CI).
[Workflow 43566519525](https://github.com/buddas-bible/zonai/actions/runs/43566519525)
must be checked for both platform results and actual residual/face-volume
values before declaring convergence.

**Next:** if the discrete reference converges, distinguish remaining
spatial pressure/geometry discrepancy from the previous history and
bring the matched-stage analysis back to real coupled World8/Fluid4
near-closing V(17,13). If not, test an independently validated pressure
reference solve before changing physical tracer transport or defaults.

### High-iteration frozen reference outcome — 2026-11-14

[CI 43566519525](https://github.com/buddas-bible/zonai/actions/runs/43566519525)
**passed on both Ubuntu Debug and Windows Release**. The new
`--small-cell-zero-mac-reference-audit` emitted 16 per-grid replay
records and 8 cross-grid records. The displayed values agree exactly
across those two build configurations; this is not a guarantee of
bitwise equality across other CPUs or future compilers.

At 128 real Fluid steps, checkpoint **11/16** (same prescribed rotating
capsule geometry, one target physical V segment), the independently
reconstructed **component-compatible continuum-form continuity
residual** was:

| Jacobi iterations | h=0.25 compatible RMS (1/s) | h=0.125 compatible RMS (1/s) | Coarse minus fine V-face volume (m³) |
| ---: | ---: | ---: | ---: |
| 200 | 0.137054977421 | 0.192741665056 | -0.000121248581872 |
| 800 | 0.0956050610453 | 0.0644154321626 | -0.000123136993711 |
| 1600 | 0.0926427767761 | 0.0475187080614 | -0.000123192190098 |
| 3200 | **0.0925685450706** | **0.0383750523** | **-0.000123227580358** |

At checkpoint **12/16**, the same 3200-iteration residuals were
0.0650400081773 (h=0.25) and 0.00950959280973 (h=0.125) 1/s;
the coarse-minus-fine V-face volume was +0.0000541209271464 m³.
**All cross-grid rows reported `reference_criteria_met=0`**.
Specifically, although the 1600→3200 face gap changed by only
~3.54e-8 m³ (checkpoint 11/16) and ~4.32e-9 m³ (checkpoint
12/16), both compatible RMS values remained above the diagnostic
1e-3/s criterion. The target face appears nearly iteration-stable,
**but the full discrete pressure solution is not certified converged**.

The independently reconstructed component-source floor was zero
to within 2.86e-9/s for the shown cases, and each field reported
one open-face-connected fluid region. Therefore the large residual
is not explained *solely* by removing a nonzero region-average
Neumann source in this experiment. The h=0.25 residual decreases
very little between 1600 and 3200 iterations; potential causes
include slow modes on the cut-cell graph, insufficient iterative
precision, or discretization/assembly inconsistency. This test
**does not identify which explanation is correct**.

This is not an independent matrix-inversion reference: it exercises
the existing noncoupled Jacobi operator and evaluates its resulting
endpoint MAC/source continuity separately. A genuinely
convergence-certified pressure solution has **not** been obtained.
No production solver, pressure pooling, connected-residual tracer,
two-way Body feedback or default was changed.

**Next atomic experiment:** locate the cells and connected faces
responsible for the largest *compatible* residual at 200/1600/3200
using identical frozen inputs; distinguish slow Jacobi modes from
a non-reducible residual floor and identify whether these cells
are near vanishing Cut-cell apertures. Only then evaluate a
separate higher-precision/independent pressure reference and
return to the actual coupled World8/Fluid4 near-disappearing
V(17,13) tracer scene.

### Per-cell localization of the unconverged frozen Jacobi residual — 2026-11-14

**Question:** The zero-open-MAC frozen pressure reference remained above
the diagnostic compatible-continuity threshold after 3200 Jacobi iterations,
especially h=0.25 at rotation checkpoint 11/16. Is the residual supported
mainly in tiny Cut-cells/near-closing open faces, near the target physical
V(16,16), or in remote connected fluid cells? Do the dominant sites
change from 200 to 3200 iterations?

The new **opt-in** `--small-cell-zero-mac-residual-map-audit` executes
the *same* 128-step prescribed rotating-capsule frozen reference at
h=0.25 and 0.125 with checkpoints 11/16 and 12/16. It reuses the existing
200/800/1600/3200 Jacobi replays and exact connected-component
compatibility-source subtraction; no change to production projection.
For every active cell it reconstructs the signed
`(postPressureMacNetVolume + pressureSourceVolume - componentMeanSourceVolume) / (h² dt)`
(1/s) using the public per-cell volume ledger.

Each grid/checkpoint/iteration reports the **top eight signed residual
cells**, their integer coordinates and cell world-space center, current
fluid fraction, smallest strictly positive surrounding U/V open
fraction, largest opening, number of open faces, moving-volume pressure
source, distinct snapshot/Swept volumes, endpoint MAC net outward
volume and Manhattan distance from either cell adjoining the target
physical face. To separate a local spike from a globally spread slow
error, it also reports **squared residual shares** for:

- top eight cells overall;
- cells with current fluid fraction <0.01;
- cells with at least one strictly positive open face below 0.01;
- cells within three cell-index Manhattan steps of the target physical
  V-face-adjoining cells.

These sets **can overlap** and their shares must not be summed.
A tiny fraction or positive aperture is a geometric classification,
not automatically the cause of solver failure. Cross-grid cell indices
are not identical world-space control volumes. All reported
"residual energy shares" are mathematical squared-continuity-error
shares, **not fluid mechanical energy**. The test checks that the
independently summed per-cell squared residual reproduces the
previous component-compatible global RMS, rejecting missing or nonfinite
cells. The original `--small-cell-zero-mac-reference-audit` output is
unchanged when localization is not requested.

Implementation `401acdf`; Ubuntu/Windows CI `b396d56`.

**Follow-up parity hypothesis:** Initial Ubuntu 3200-iteration results
showed top-8 shares of only **0.841%** (h=.25, checkpoint 11)
and **0.465%** (h=.125, checkpoint 11); the top cells had
**fluid fraction 1, four fully open faces, zero local moving-volume
source**, and the top eight all shared the same grid-index
`(x+y) mod 2` parity in each case. Thus the residual is broadly
distributed, not concentrated in a particular vanishing-aperture cell.
These observations are consistent with, but alone do **not prove**,
an undamped bipartite/checkerboard Jacobi mode on the connected
MAC pressure graph.

The additional code commit `05b3fd6` measures the signed full-domain
checkerboard projection of the per-cell continuity residual,
`mean((-1)^(x+y) * residual)`, and its **squared RMS share**
`(sum((-1)^(x+y)*r_i))² / (N * sum(r_i²))`, bounded [0,1].
On the residual-map flag **only**, each selected frozen geometry is
replayed at 199/200, 799/800, 1599/1600 and 3199/3200 Jacobi
iterations. Independent copies compare the odd-vs-even target-face
flux and full-grid compatible residual; the older default reference
command still uses only 200/800/1600/3200. The parity audit must
pass both-platform CI and show strong projection or oscillation
before assigning causality. It is a diagnostic, **not** a solver
change or an accepted physical Cut-cell correction.
[Workflow 43567697554](https://github.com/buddas-bible/zonai/actions/runs/43567697554)
must be verified and its CSV inspected before attributing the residual
plateau to local aperture geometry, Jacobi slow modes, or matrix
inconsistency. Neither pressure pooling nor tracer remap is enabled.

### Frozen Jacobi checkerboard eigenmode confirmed by odd/even control — 2026-11-14

**Validated:** opt-in residual-map code `401acdf`,
[workflow 43567697554](https://github.com/buddas-bible/zonai/actions/runs/43567697554)
**Ubuntu Debug and Windows Release success**. Follow-on odd/even and
full-grid checkerboard projection `05b3fd6`,
[workflow 43568312090](https://github.com/buddas-bible/zonai/actions/runs/43568312090)
**Ubuntu Debug and Windows Release success**; all published CSV values
below match across the two runners to displayed precision.

The 3200-iteration **compatible continuity residual** decomposes as:

| Grid h | Rotation checkpoint | RMS (1/s) | Squared residual in signed checkerboard projection | Top-eight-cell squared share | Tiny fraction / positive aperture <1% squared share |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.25m | 11/16 | 0.0925685451 | **98.8489555%** | 0.8414542% | 0% / 0% |
| 0.25m | 12/16 | 0.0650400082 | **98.8212793%** | 0.8413770% | 0% / 0% |
| 0.125m | 11/16 | 0.0383750523 | **89.1598984%** | 0.4653814% | 0% / 0% |
| 0.125m | 12/16 | 0.0095095928 | **11.4411598%** | 0.7174071% | 0% / 0% |

All top-eight cells at the coarse 11/16 3200-iteration snapshot had
`fluidFraction=1`, four open faces with `aperture=1`, and zero local
moving-volume source. A representative highest residual cell was
(x=1, y=30), world-center (-3.625m, 3.625m), far from the target V
face. The fine-grid largest residual at the same checkpoint was
(x=62, y=2), world-center (3.8125m, -3.6875m), likewise far from
the moving capsule. The tiny-cell categories contributing 0% directly
**do not imply** that moving Cut-cell geometry did not excite the
global error mode; they show the *remaining residual* is not localized
on those cells.

**Independent odd/even control:** At h=.25, 11/16, the signed
checkerboard component changes from **+0.0920342600/s** after 3199
Jacobi iterations to **-0.0920342506/s** after 3200. The compatible
RMS stays practically unchanged (0.0925685630/s versus
0.0925685451/s). Their target V-face volumes differ by
**+3.89354945e-7 m³** (even minus odd). At h=.25, 12/16,
the parity component flips from -0.0646555831/s to
+0.0646555519/s, and target-face volume changes
by -2.73901983e-7 m³. The fine h=.125 11/16 snapshot has
89.16% of its residual in the checkerboard pattern at 3200;
at 12/16, only 11.44%: other slowly converging components
remain. Thus an apparent near-plateau in total residual
is **not an accepted convergence limit**.

**Mechanism:** The ordinary noncoupled projection is an undamped
Jacobi update on the open-face-weighted orthogonal cell adjacency
graph. For every active degree-positive cell i, its homogeneous
iteration satisfies `e_next[i] = sum_j(a_ij * e[j]) / sum_j(a_ij)`.
On a bipartite 4-neighbor grid the checkerboard vector
`e[i] = (-1)^(x_i+y_i)` obeys `e_next[i] = -e[i]` because every
neighbor has opposite parity. This is the **undamped -1 eigenmode**,
so adding further even iteration counts cannot remove its
component. This same high-frequency undamped-Jacobi issue is
described in Gilbert Strang's MIT *Iterative Methods*, section 6.2:
https://math.mit.edu/classes/18.086/2006/am62.pdf .
This reasoning assumes the actual face graph/pressure rows follow the
stated noncoupled Jacobi discretization; the measured >98% coarse
projection and odd/even sign flip directly corroborate it here.

**Engineering decision:** This is a reproducible numerical solver
issue in the **frozen prescribed-kinematic, noncoupled Jacobi** control.
It is not proof that the real **coupled Schur PCG**
World8/Fluid4 tracer problem has the same mode. Do not interpret
the 3200-step solution as a spatially converged pressure reference,
and do not enable pressure pooling, connected-residual tracer
redistribution, velocity clamps or a default Jacobi replacement.
The existing code/CI adds diagnostic observation only, preserving
production defaults and PR #326 Draft status.

**Next atomic S2c experiment:** on identical frozen inputs, compare
the legacy undamped Jacobi with a **diagnostic-only damped Jacobi
control (e.g. omega=2/3)** and/or a separately residual-checked
pressure solve. Verify full-grid compatible RMS and target flux at
identical iteration/work budgets, including fine h=.125 at 12/16
where the checkerboard share is low. Any proposed production change
requires separate regressions and costs; return afterwards to the
actual World8/Fluid4 V(17,13) coupled pressure/tracer scene.

### Frozen damped-Jacobi A/B against the -1 checkerboard mode — 2026-11-14

**Question:** Does the 3200-iteration stagnation actually come from
the undamped Jacobi checkerboard mode rather than an irreducible moving
Cut-cell pressure-source error? Is a locally damped solve sufficient to
establish a globally converged discrete reference?

**Experiment:** New opt-in
`--small-cell-zero-mac-damped-jacobi-audit` reuses the **same**
pre-pressure frozen rotating capsule (h=0.25/0.125, 128 actual Fluid
steps, rotation checkpoints 11/16 and 12/16). Copies start from
identical zero-open-MAC fields, moving-solid swept geometry, prescribed
closed-wall velocities and source history. On one copy, the original
Jacobi update uses **omega=1**; on another, pressure uses the weighted
Jacobi formula

`p_new = p_old + (2/3) * (p_jacobi - p_old)`.

Each copy replays **200/800/1600/3200** iterations. The diagnostic
`replayFrozenCutCellPressure(..., jacobiRelaxation)` accepts
finite `0 < omega <= 1` only. Nonunit omega is rejected for any
inverse-mass or inverse-inertia obstacle; the normal world Step always
calls `projectCutCell` with omega=1. Invalid omega is rejected before
consuming the snapshot. A unit regression verifies that explicitly
passing omega=1 reproduces the old copy's velocity and that damped
snapshots remain one-use. This does **not** change production
iterations, Schur PCG, dynamics, tracer transport or defaults.

The independently reconstructed, component-compatible per-cell
continuity RMS is measured in **1/s**, with a separate squared
checkerboard projection share. Both paths are checked for identical
pressure source, component count and compatibility floor.
The cross-grid comparison is the signed endpoint MAC volume on one
physical V(16,16) face. The diagnostic column
`damped_reference_criteria_met` only checks *both* fields'
`compatibleRms < 1e-3/s`; it is **not** a full independent
convergence certificate, and a 3200-iteration reference cannot be
accepted merely because the target face appears stable.

**Validated on both platforms:**
[workflow 43569382849](https://github.com/buddas-bible/zonai/actions/runs/43569382849)
**success on Ubuntu Debug and Windows Release**, including full existing
regression jobs and both new damped-Jacobi A/B steps. All 16 per-grid
damped comparison rows and eight cross-grid rows match between platforms
to their displayed precision; this does not establish bitwise identity
across all build environments.

| Grid h | Rotation | Iterations | Legacy compatible RMS (1/s) | Damped compatible RMS (1/s) | Legacy checkerboard squared share | Damped checkerboard squared share |
| ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 0.25m | 11/16 | 3200 | 0.0925685451 | **0.000132951893** | 98.8489555% | 0.0000191235% |
| 0.25m | 12/16 | 3200 | 0.0650400082 | **0.000088577782** | 98.8212793% | 0.000130344% |
| 0.125m | 11/16 | 3200 | 0.0383750523 | **0.001083539843** | 89.1598984% | 0.0000036542% |
| 0.125m | 12/16 | 3200 | 0.0095095928 | **0.001596167722** | 11.4411598% | 0.0000002751% |

The coarse 11/16 RMS fell by roughly **696-fold**, and the fine
11/16 RMS by about **35-fold**. The checkerboard component is nearly
suppressed in every sampled damped state, confirming the -1-mode
interpretation of the legacy undamped Jacobi plateau. Crucially,
**neither fine-grid 3200-iteration state** satisfies the provisional
`1e-3/s` compatible-RMS threshold (0.0010835 and 0.0015962).
Damped Jacobi also has slow, non-checkerboard convergence; no
certified pressure reference or fully spatially converged coarse/fine
physical flux follows. At 11/16, the 3200 coarse-minus-fine target
volume changed from **-0.000123227580358 m³** (legacy) to
**-0.000123452339551 m³** (damped), but this
near-face agreement does not prove full-grid convergence.

**Scope/next:** Leave omega=1 in the real engine, with omega=2/3
copy-only and opt-in. A separate residual-checked pressure reference
(faster elliptic solver or an independently assembled linear solve)
is still needed to distinguish remaining slow modes and spatial
pressure discretization. Then transfer the same-state analysis to the
**real coupled World8/Fluid4 V(17,13)** and separately measure
normal-step performance before accepting any stabilization.
Neither experimental pressure pooling nor tracer remap is accepted.

### Independently assembled frozen pressure PCG reference — 2026-11-14

The damped-Jacobi A/B experiment suppressed the bipartite checkerboard
mode but did **not** meet the fine-grid residual threshold by 3200
iterations. The next diagnostic is implemented as
`--small-cell-zero-mac-independent-pcg-audit` in
`tests/sandbox/wind_step_benchmark.cpp`. It deliberately leaves all
production `fluidField2::step`, Schur PCG, body dynamics and tracer
transport unchanged.

For the **same** h=0.25/0.125, 128 real Fluid-step rotating-capsule
snapshots at checkpoints 11/16 and 12/16, the command preserves the
zero-open-MAC frozen input and compares both legacy omega=1 and
frozen-only damped omega=2/3 Jacobi at 200/800/1600/3200 iterations.
In addition, it constructs a **separate double-precision Neumann graph
system** from the public post-pressure `getCutCellVolumeBudget`
`pressureSourceDeltaVolume` and U/V face open fractions, preserving
the original frozen moving-volume source and the original
**compatibility traversal**, including zero-fluid cells reached across
positive-open faces. Those zero-fluid cells have no pressure unknown;
the existing Jacobi matrix still includes their edge weight in the
active neighbor's diagonal (an anchored, Dirichlet-like row). An
independent reference therefore must not assume that every positive
aperture links two active pressure unknowns, or force a mean-zero gauge
on an anchored component.

The independently assembled **volume-form operator** for the
zero-initial-open-MAC case is
`(Aq)_i = sum_open_faces(a_ij * (q_i - q_j))`,
`b_i = -(sourceVolume_i - mean(sourceVolume)_connected_region)`,
and `q = dt² * pressure`. The pressure-corrected endpoint MAC volume
on an oriented V face is
`a * (q_lower - q_upper)`, summed across both fine-grid segments
when applicable. This is a **discrete pressure reference**, not
an exact time-integrated moving-wall transport reference.
Fully covered cells are not pressure unknowns. A constant
pressure gauge is projected out **only** for unanchored components;
positive apertures to zero-pressure inactive neighbors are preserved
as diagonal edges. The compatibility-source mean is reconstructed
using the production graph's full traversal count (which may include
inactive cells), distinct from the unknown count.

The standalone solver uses double-precision diagonal-preconditioned
conjugate gradients up to 10,000 iterations. Its **explicit** residual
`Aq-b` is recomputed after the solve (not trusted solely from
the recurrence); the diagnostic convergence threshold is
`sqrt(sum_active((Aq-b)_i²)/N)/(h²dt) < 1e-5/s`.
Nonfinite/invalid apertures, truly out-of-grid adjacency,
changed compatibility region counts or pressure-source ledgers, and
a nonconverged reference are treated as test failures.
The diagnostic also prints the count and summed aperture of
active-to-inactive positive-open faces and the number of pressure
unknown components anchored by them. The resulting physical-face
volume is compared independently against the **3200-iteration
damped-Jacobi** output, and both coarse/fine pressure-reference
faces are compared at the same world time.

The **first CI** [43572701343](https://github.com/buddas-bible/zonai/actions/runs/43572701343)
found a nonzero aperture connecting an active and fully covered cell,
and correctly rejected the initial *pure Neumann unknown graph*
assumption before emitting a false reference. The frozen Jacobi
assembly, however, includes that face in the active diagonal and
holds the inactive pressure at zero. Commit `c921a46` adapts only
the independent benchmark to reproduce this actual mixed
anchored/Neumann discrete operator and the production compatibility
graph traversal. This discrepancy in the originally assumed geometry
graph is worth investigating separately; it is **not** by itself
proof that a physical fluid leak occurred. Its corrected CI results
are required before validating the reference.

Commits: `f0557f6` (standalone independent graph PCG),
`1e5b314` (same-frozen snapshot comparison and CSV),
`df7b359` (Ubuntu Debug / Windows Release CI).
CI and actual values must be inspected before drawing any
convergence or physics conclusions.

**Limitation:** Both solvers still use the *same Cut-cell geometric
sampling, finite-volume pressure-source ledger and endpoint MAC
aperture model*. A successful double-precision residual only
certifies convergence to that **discrete equation**. It is not
proof of physically correct space-time tracer capacity, nor
evidence that the real **coupled World8/Fluid4 near-closing
V(17,13)** pressure/Body dynamics share this noncoupled Jacobi mode.
No pressure-pooling or connected-residual-tracer variant is accepted.

### First cross-platform independent pressure reference outcome — 2026-11-14

The duplicate local `componentCells` declaration in the initial anchored
graph split prevented compilation; `fe1cedf` removes it. In
[workflow 43573744077](https://github.com/buddas-bible/zonai/actions/runs/43573744077),
**Ubuntu Debug completed successfully** and the independently assembled
double-precision PCG reached the explicit volume-form pressure residual
threshold `< 1e-5/s` at all four frozen checkpoints. Windows must be
checked separately before asserting matching cross-platform results.

| Grid h | Rotation | Independent PCG iterations | Explicit RMS (1/s) | PCG physical V-face volume (m³) | Damped Jacobi 3200 V-face volume (m³) |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.25 | 11/16 | 161 | 0.00000955518 | 0.000287553307 | 0.000290708445 |
| 0.25 | 12/16 | 126 | 0.00000880323 | 0.000419161684 | 0.000419163353 |
| 0.125 | 11/16 | 330 | 0.00000972744 | 0.000424229862 | 0.000414160785 |
| 0.125 | 12/16 | 326 | 0.00000967737 | 0.000199157174 | 0.000364916480 |

All fields have one compatibility-connected fluid region and one active
pressure group. At h=.25/11 there are **three** open active-to-inactive
faces (apertures summing to 1); at h=.25/12 there are **none**;
at both h=.125 checkpoints there are **four** (sum 1).
The pressure group is anchored in the three cases with such faces.
A zero-fluid cell in the compatibility traversal is *not* a solved
pressure unknown, so this boundary condition matters for the
reference linear system.

The huge **0.00016575930649m³** PCG-minus-damped
target-face difference at h=.125/12 is real in the emitted
Ubuntu benchmark, despite the damped 3200-step RMS appearing small
(`0.00159617/s`). The target face and local pressure response
cannot be certified merely from a small global continuity RMS or
endpoint-flux change between two Jacobi budgets. At h=.25/12,
where no active-to-inactive face is present, the PCG-minus-damped
face difference is only **-1.67e-9m³**. These four cases suggest that
the anchored inactive-cell edges may matter, but they do **not**
establish causality: resolution, geometry, pressure conditioning and
checkpoint differ simultaneously. Do not mistake this **discrete
PCG/finite-Jacobi** difference for an actual space-time tracer leak.

**Next atomic investigation:** on the *same frozen* h=.125/12
snapshot, run a separately labeled independent pressure-operator
counterfactual that removes only active-to-inactive aperture
anchors, preserving other open-open weights and reporting any
necessary per-Neumann-group compatibility rebalancing. Compare
explicit true residual, target face and cross-grid difference;
do **not** alter the engine's Cut-cell geometry, pressure
defaults or connected tracer transport. Then return to the
World8/Fluid4 V(17,13) fully coupled scene.

### Anchored inactive-neighbor edge counterfactual — 2026-11-14

The converged independent pressure-reference control revealed a much
larger-than-expected target-face deviation from finite 3200-iteration
damped Jacobi on h=0.125 at checkpoint 12/16. Three of four tested
snapshots retain positive aperture edges from an active pressure cell to
an inactive, zero-pressure neighbor. The next opt-in test
`--small-cell-zero-mac-anchor-audit` isolates this particular discrete
boundary/conditioning hypothesis without altering the engine.

It evaluates two **independently assembled double-precision PCG**
systems on the exact same frozen source, open MAC input and moving
geometry:

1. **Actual discrete reference:** preserve all positive open-face
   weights (including active-to-inactive, with the inactive pressure
   unknown pinned to zero), and preserve the solver's original
   full-graph source compatibility correction
2. **Counterfactual pressure operator:** remove only positive
   active-to-inactive pressure edges *after* computing the original
   full-graph compatibility source; preserve every active-to-active
   face, source volume and frozen physical geometry, then separately
   report any additional active-pressure-component mean-source
   subtraction required to make the newly unanchored Neumann system
   solvable

Both calculations independently recompute `Aq-b` RMS and must meet
the `1e-5/s` PCG diagnostic criterion. Their target physical V-face
volumes and coarse-minus-fine gaps are compared at identical 11/16 and
12/16 checkpoints, including a **no-anchor control** (h=0.25 at
12/16), which must reproduce the actual reference within
`1e-9m³` if there is no operator change. The CSV reports
`compatibilityRebalanceL1Volume` to make any changed constraint
explicit rather than treating the two matrices as identical.

This is an intentional *pressure-matrix-only counterfactual*, **not
a production proposal to delete geometric face apertures**.
Deleting an active-to-inactive edge from the pressure operator
does not physically close a moving fluid channel or establish
correct time-integrated face capacity. The exact geometric origin of
those inconsistent positive-open edges still requires investigation.
Do not interpret it as a confirmed physical leak or apply the
counterfactual to World8/Fluid4 yet.

Implementation `92abb9a`, Ubuntu/Windows CI `628e366`.
CI conclusions and emitted CSV are required before assigning causal
influence or accepting any change.

### Same-snapshot anchored-edge causality control — 2026-11-14

The initial new
[workflow 43574557305](https://github.com/buddas-bible/zonai/actions/runs/43574557305)
**passed on Ubuntu Debug**, including all existing regressions and
the separately assembled anchor counterfactual. Windows must be
checked for a completed success and matching CSV before declaring
cross-platform validation.

| Grid h | Checkpoint | Active-to-inactive positive faces | Actual operator PCG V(m³) | Omit-anchor PCG V(m³) | Damped Jacobi 3200 V(m³) |
| ---: | ---: | ---: | ---: | ---: | ---: |
| 0.25 | 11/16 | 3 (sum a=1) | 0.000287553307 | 0.000290717179 | 0.000290708445 |
| 0.25 | 12/16 | 0 | 0.000419161684 | 0.000419161684 | 0.000419163353 |
| 0.125 | 11/16 | 4 (sum a=1) | 0.000424229862 | 0.000414197073 | 0.000414160785 |
| 0.125 | 12/16 | 4 (sum a=1) | **0.000199157174** | **0.000364877017** | **0.000364916480** |

On the **exact same frozen** h=.125/12 source, removing only
active-to-inactive edges from the standalone pressure matrix changes
the converged PCG V-face volume by **+0.000165719843204m³**,
almost all of the previously observed **+0.00016575930649m³**
damped-Jacobi versus anchored-PCG discrepancy. The new no-anchor
reference is within **3.94633e-8m³** of the 3200-step damped
Jacobi, though the finite-iteration damped state is not itself a
certified converged solution. At h=.25/12, where no such anchor exists,
the two independently solved references have **identical** target
face volume.

Both independent PCG variants passed the explicit full-operator
residual threshold `1e-5/s` at all sampled checkpoints.
The additional active-group Neumann compatibility correction was
zero or within **3.5e-14m³** of zero. Therefore the dramatic V-face
difference is not caused by an arbitrary change in source
compatibility in this experiment. The converged true operator
includes an anchored inactive-cell pressure row, while the finite
damped Jacobi behaves very close to the counterfactual **unanchored**
operator at this sampling horizon. This isolates a strong causal
**discrete pressure boundary-condition sensitivity** rather than
proving an unphysical fluid mass leak.

The operator-only control does **not** change fluid geometry, face
apertures, source history, coupled Body dynamics, actual solver,
tracer routing or defaults. Simply deleting pressure anchors in
production would be premature. **Next:** print the exact grid and
world positions of positive-open faces adjacent to inactive cells,
check shared-face/active-fraction classification and moving-solid
sampling (including strict boundary/tangent cases), and verify whether
these pressure anchors are intended boundary conditions or artifacts
of inconsistent geometry before proposing any correction.
Then reproduce this separation in the actual World8/Fluid4
coupled V(17,13) case.

### Exact anchored-face coordinate ledger — 2026-11-14

After isolating the anchored-pressure-edge sensitivity, the same
`--small-cell-zero-mac-anchor-audit` additionally emits one
`zero_mac_anchor_face` row for **each unique U/V face** with positive
endpoint opening and precisely one zero-fluid-fraction neighbor.
Each row records face orientation, integer and world-space face
position, endpoint opening, **separate swept time-mean opening**,
the two adjacent current cell fractions and stored pressure-source
volumes, and the prescribed solid face velocity. The independent
face scan must match the pressure assembly's exact anchor count and
aperture sum (absolute tolerance `1e-7`). In particular, a boundary
may have a nonzero swept time-mean exposure while its endpoint face
is closed, or vice versa: these quantities must not be conflated.

Commits `b89de78`, `66d729a` add the read-only position
and face-ledger checks. They do not change production Cut-cell
geometry or declare positive-open/covered-cell adjacency
automatically invalid: actual obstacle shape sampling and
boundary-point classification remain to be investigated based on
the measured coordinates.

### Single rotating capsule: sampled-cell versus sampled-face witness — 2026-11-14

The frozen zero-MAC reference uses **one** rotating Capsule, not a disjoint multi-Shape
set. In `fluidField2.cpp`, `BuildDisjointAnalyticShapes` returns empty for fewer
than two obstacles; the individual analytic Capsule Cut-cell option is OFF in this
benchmark. Consequently, `updateCutCellGeometry` uses its fallback **4×4 midpoint
cell occupancy** and **four midpoint points per MAC face**, rather than analytic
Capsule rectangle areas. These two quadratures need not have identical support:
a positive-open face can border a cell classified as exactly zero fluid because
all sixteen cell samples miss a narrow, real fluid wedge.

The previous coordinate-ledger run records positive openings of 0.25 or 0.5
next to zero-fraction cells. This is evidence of **discrete support
inconsistency**, not proof of negative physical volume, an incorrect exact
Capsule intersection routine, or a defective pressure PCG solve.

To test the hypothesis independently, the opt-in
`--small-cell-zero-mac-anchor-audit` now evaluates the known capsule as a
point-to-rotated-segment distance, not by calling the engine's clipping or
occupancy routines. The first independent midpoint evaluation uses **the
same N=4** and must reproduce every stored face opening and both cell
fractions within `1e-6`. A separate N=96 face/cell midpoint estimate is
reported in the same CSV for each unique anchored physical face. This
distinguishes `N=4` quadrature artifacts from large-scale genuine blocked
geometry, without altering the solver or the pressure-operator
counterfactual.

Implementation: `0f490ce5` (high-resolution geometry witness) and
`f83f821b` (enforced independent coarse-oracle match), on PR #326.
CI references: [first oracle run](https://github.com/buddas-bible/zonai/actions/runs/43576922437)
and [coarse-oracle assertion run](https://github.com/buddas-bible/zonai/actions/runs/43577219383).
Verify both Windows and Ubuntu conclusions and the new face rows before
assigning a quantitative correction; this section documents the method, **not**
an accepted numerical fix.

**Next decision:** after checking the CSV, compare the same prescribed capsule
and timestamps with the existing opt-in analytic-Capsule geometry, and
independently report cell fractions, face apertures, pressure anchors, source
volumes, and target-face convergence. Geometry alternatives change the
discrete fluid state, so distinguish that experiment from a same-frozen-state
pressure-only A/B. Do not simply erase anchored pressure faces, clamp
velocities or enable either experimental pressure pooling or tracer rerouting.

### Prescribed rotating Capsule: sampled versus analytic geometry A/B — 2026-11-14

The independent coarse-midpoint oracle in the previous section passed
Ubuntu Debug CI at [run 43577219383](https://github.com/buddas-bible/zonai/actions/runs/43577219383).
It matches the engine's four face points and 4×4 cell occupancy without using
the engine's clipping routines. Its separate 96×96 midpoint witness shows
positive fluid area where the baseline discrete cell is entirely inactive.
For example, h=0.125m at checkpoint 12/16:

| Covered-by-N=4 cell | Discrete fluid fraction | Independent N=96 estimated fluid fraction | Adjacent open face |
| --- | ---: | ---: | --- |
| (30,29) | 0 | 0.00716146 | U(31,29), sampled open 0.25 |
| (32,32) | 0 | 0.02832031 | U(33,32), sampled open 0.25 |
| (31,34) | 0 | 0.01019965 | U(31,34), sampled open 0.25 |
| (32,32) | 0 | 0.02832031 | V(32,32), sampled open 0.25 |

These values are **point-sampled area estimates**, not analytic exact volumes
or an accepted repair. They demonstrate that cell and face N=4 support
decisions can disagree along the same continuous Capsule boundary.
This supplies a concrete pressure-anchor mechanism without establishing
whether analytic geometry resolves the real coupled World8/Fluid4 spike.

The next standalone, opt-in --small-cell-capsule-geometry-audit
(commit 23158412, CI wiring 9258ddbc) runs the **same prescribed rotation**
at h=0.25/0.125, 128 Fluid substeps, checkpoints 11/16 and 12/16,
with baseline sampled geometry and the existing analytic-Capsule mode
separately. Each pre-pressure snapshot is copied and given zero-open-MAC
input for an independently assembled double-precision reference PCG;
the original Fluid/pressure solver is not modified. CSV includes active
fluid cells, active-to-inactive pressure anchors and aperture sums,
sampled-versus-analytic fractions and face apertures on affected faces,
source-volume differences, pressure true residual and target-face volume.

**Interpretation gate:** geometry, pressure source and previous Fluid states
may differ between the two separately evolved variants, even though
the prescribed solid path and zero-MAC diagnostic input match.
This is a **geometry-policy sensitivity control**, not a same-snapshot
pressure-matrix A/B and not proof of mass preservation or physical stability.
Review Windows and Ubuntu CSV, including any analytic-mode anchors or
reference PCG failures, before deciding on a numerical change.
Keep experimental pressure pooling and connected tracer rerouting OFF,
and PR #326 Draft.

### Remaining S2c acceptance gate

1. **Physical acceptance, not just mass conservation:** compare the new endpoint-aperture transfer with the *time-integrated open-face* fluid capacity, moving-solid surface flux, pressure source and local concentration. Identify false evacuation through tiny apertures and unphysical redistribution into inflowing neighbors. Treat total tracer mass as necessary but insufficient.
2. Extend controlled tests to **almost closed faces, fully disconnected pockets, newly reopened faces**, nonlocal fully covered fallback, rotation, h/cadence variation and Body force/trajectory differences. The new experiment handles **partial** donors only. Verify no change to tracer-disabled and default-off paths.
3. On the same actual World8/Fluid4 source (18,13), independently compare baseline/pressure-pool and baseline/connected-tracer alternatives, plus combined alternatives **only after** each independent effect is understood. Compare same-state pressure continuity and separate continuous histories. Never add endpoint MAC kg-equivalent flux to Swept tracer kg and call it a conservative space-time flux.
4. Measure normal-step overhead (feature disabled) and optional observer/remap overhead enabled. Keep **both experimental controls OFF by default**, PR #326 Draft, and PR #322 sampling independent. No solver merge until supported by physical and performance acceptance evidence.

