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

### Prescribed Capsule geometry A/B observations — run 43578243918

[CI run 43578243918](https://github.com/buddas-bible/zonai/actions/runs/43578243918)
completed **success on Windows and Ubuntu**, including the opt-in
same-prescribed-motion geometry comparison. The double-precision
independently assembled pressure references report convergence at the
specified explicit residual tolerance (under 1e-5/s) in both modes.
The following table compares active-to-inactive **pressure anchors**;
these are not proof of a whole-fluid mass leak.

| Grid h / checkpoint | Sampled N=4 anchors (sum opening) | Analytic Capsule anchors (sum opening) | Analytic minus sampled target-face m³ |
| --- | --- | --- | ---: |
| 0.25 / 11 | 3 (1.0) | 0 (0) | +0.000113940671 |
| 0.25 / 12 | 0 (0) | 0 (0) | +0.000058012498 |
| 0.125 / 11 | 4 (1.0) | 0 (0) | -0.000015339097 |
| 0.125 / 12 | 4 (1.0) | **2 (0.00017798537)** | **+0.00024808895** |

At h=0.125/checkpoint12, the two remaining analytic-only anchors
are U(29,26) (open 0.000108855966) and V(28,26) (open
0.0000691294044). Both touch the same analytic zero-fraction cell
(28,26). They are **much smaller** than the four N=4 support
mismatches, but cannot be silently deleted or assumed to be real
fluid channels without an independent geometric tolerance study.

Windows Release and Ubuntu Debug agree on anchor counts, sums
and the target-face pressure difference to the shown precision.
They do **not** have identical analytic active-cell counts at
h=0.125/checkpoint12 (Windows 4086; Ubuntu 4080). Those near-zero
fraction classifications require a separate precision/roundoff
audit before treating the analytic model as a fully robust solution.

Crucially, removing large sampled anchors **does not imply that
the pressure answer improved**: the h=0.125/checkpoint12 target-face
reference changed by +2.48089e-4 m³, while the pressure source and
geometry also changed. The experiment compares two separately
evolved Fluid histories with the same prescribed solid path and
zeroed MAC input for each independent pressure reference.
Do not attribute the entire target-face difference to just the
anchor edges, nor generalize these isolated references to the
actual World8/Fluid4 coupled spike.

**Next narrowly scoped task:** quantify the two analytic-only tiny
anchor faces and cell (28,26) with independent double-precision
signed-distance and interval/area geometry, including tangent and
float-roundoff perturbations; compare h=0.125/checkpoint12
Windows/Ubuntu active-mask differences. Keep the default geometry
policy unchanged until conservation, topology and full two-way
Body feedback tests justify a physical fix.

### Tiny analytic Capsule corner: independent exterior-area witness — 2026-11-14

The new read-only, single-cell `capsule_corner_oracle` output in
`--small-cell-capsule-geometry-audit` tests the two tiny analytic-only
anchors remaining at h=0.125m, checkpoint 12/16. Their shared inactive
cell is (28,26); its adjacent analytic face apertures are
V(28,26) = 0.0000691294044373 and U(29,26) = 0.000108855965664.
The engine stores that cell's fluid fraction as **exactly zero**.

An independent point-to-segment capsule **signed-distance** function
uses the floating-point-transformed capsule endpoints and bisection to
find where the capsule boundary crosses the cell's bottom and right
edges. It integrates the tiny exposed lower-right wedge over its own
narrow x-range with 128 and 1024 one-dimensional midpoint slices.
It does **not** call engine face clipping, capsule area or pressure
projection routines, and does not affect the simulated field.

[CI #43579391847](https://github.com/buddas-bible/zonai/actions/runs/43579391847)
passed **Windows Release and Ubuntu Debug**, including existing tests
and the new wedge oracle.

| Diagnostic (float-derived transform, nominal radius) | Windows Release | Ubuntu Debug |
| --- | ---: | ---: |
| Corner signed clearance (m) | 7.29305747e-6 | 7.29314458e-6 |
| Independent lower-edge exposed length (m) | 8.63940600e-6 | 8.63950908e-6 |
| Independent right-edge exposed length (m) | 1.36042093e-5 | 1.36043722e-5 |
| Independent positive fluid area (m²) | **5.87661436e-11** | **5.87675485e-11** |
| Area / cell area | **3.76103319e-9** | **3.76112310e-9** |
| Engine analytic cell fluid fraction | **0** | **0** |
| `area128 - area1024` (m²) | -1.69e-23 | +1.31e-22 |

Changing the radius by **one float ULP in either direction** and
using separately constructed double-angle ideal endpoints still
leaves a positive exterior wedge on both platforms. For the actual
float endpoints, the `-1/+1 ULP` Windows-area range was
5.90065307e-11 to 5.85262472e-11 m² (Ubuntu:
5.90079384e-11 to 5.85276492e-11 m²). The independently recovered
face fractions agree with the engine's analytic clipped face openings
within the diagnostic 1e-5 bound. Thus the face is not merely an
isolated noisy positive aperture; a corresponding very small
two-dimensional open wedge also exists.

**Interpretation:** the current analytic Capsule cell-fraction path
reports zero where independently reconstructed signed-distance
geometry has a positive (but tiny) open area. This is concrete
**near-full-cell geometric support inconsistency**, rather than
proof that solver PCG failed or that pressure anchors can safely be
removed. The small nonzero cross-platform difference in the
independent wedge is consistent with differing float-transformed
endpoints, but these measurements alone do not establish the exact
source of the engine's area calculation error. A 1024-slice agreement
does not constitute a formal exact-integration proof.

In the broader same-path geometry comparison, analytic h=.125,
checkpoint12 active-cell counts were **4086 Windows vs 4080 Ubuntu**,
while the anchor count/aperture sum remained 2 / 0.000177985370101
on both. This local witness explains one shared zero-fraction
classification, **not the six-cell platform difference**.

**Next bounded experiment:** inspect the analytic Capsule covered-area
decomposition (float core vertices, polygon rectangle clip, two
half-disk cap contributions) around this precise cell, using independent
double-precision reference geometry and tracing cancellation in
`1 - coveredArea/cellArea`. Then audit all cross-platform
near-zero cells and compare source/pressure budgets. Do not introduce
a per-cell epsilon or force positive fractions without a stated
conservative volume/face topology policy, and do not change defaults
or merge experimental pressure/tracer algorithms.

### Exact analytic Capsule area-decomposition trace — 2026-11-14

After confirming a positive near-tangent open wedge by independent
point-to-segment distance integration, an on-demand
`fluidField2::getAnalyticCapsuleCellAreaAudit` now re-evaluates the engine's
**same** single-Capsule core + first/last half-circle routines for one
caller-supplied obstacle and cell. It reports component covered areas,
their raw sum, clamped covered area, signed pre-clamp remainder and stored
fluid fraction. The callback compares this result with the independently
integrated corner wedge; the normal Fluid Step does not call the audit.

Implementation commits `8d4ca170`, `d521a672`, `e7b3c21b`.
[Windows Release + Ubuntu Debug CI #43581074532](https://github.com/buddas-bible/zonai/actions/runs/43581074532)
**passed on both platforms**.

For h=0.125m, checkpoint 12/16, cell (28,26):

| Area term, m² | Reproduced engine arithmetic, both platforms |
| --- | ---: |
| Whole cell | 0.015625 |
| Core polygon covered | 0.011288604901483743 |
| First half-cap covered | 0.0043363951027066677 |
| Last half-cap covered | 0 |
| **Unclamped sum covered** | **0.015625000004190412** |
| **Signed remainder, cell minus sum** | **-4.190411906357383e-12** |
| Stored analytic fluid fraction after clamp | **0** |

The **independent positive exterior wedge** for this cell is
5.87675484957e-11 m² on Ubuntu and 5.87661436165e-11 m² on Windows.
The raw covered-area sum therefore **exceeds the geometric reference**
by 6.29579600e-11 m² (Ubuntu) or 6.29565549e-11 m² (Windows),
respectively. These values are not comparable to a normal fluid
mass balance: they are 2D geometric areas at one frozen endpoint.

The result identifies the **engine's pre-pressure, core/half-cap area
composition and final nonnegative clamp** as the mechanism making
this real tiny wedge inactive. It does not yet uniquely apportion the
component error between construction of the float
`analyticFluidCapsule::axis` / `core.vertices`, the rectangle/polygon
intersection arithmetic, and half-circle integration. In particular,
`TryGetAnalyticCapsule` computes a double-precision normalized axis
but casts it to `vec2` (float) before constructing the core's float
vertices; component area functions subsequently use double arithmetic.
Double arithmetic on rounded geometric inputs need not preserve such
an extremely small exterior sliver.

**Next discriminating test:** reconstruct the same float-transformed
endpoints in an independent **all-double** core/half-cap geometric
partition, compare each covered component and the signed remainder
with the existing on-demand audit and independent wedge, then
extend to all near-zero cells across Windows/Ubuntu. A change to
core construction also affects face clipping, moving-boundary flux
and pressure connectivity; do not turn a positive tiny cell on by
epsilon alone. Retain the baseline solver and analytic-mode default
as-is pending conservation, two-way Body feedback and performance
acceptance tests.

### Axis and core-vertex precision controls — 2026-11-15

Commit [`badb9b74`](https://github.com/buddas-bible/zonai/commit/badb9b74eee2cb878a578b809be9f59acd00da62)
extends the single-cell (28,26) oracle with an **independent** all-double
rectangular core clip and independent one-dimensional cap cross-section
integrals. For reproducibility it starts from the **same already
float-transformed Capsule endpoints** as the engine and reconstructs
both a rounded-float-axis/float-core-vertex partition and a
double-normalized-axis/double-core-vertex partition. The only differences
between these two controls are the axis and core-vertex rounding.
Cap midpoint integration is checked at 16,384 versus 65,536 slices.
No production geometry or pressure code was changed.

[GitHub Actions run 43631292578](https://github.com/buddas-bible/zonai/actions/runs/43631292578)
**passed Windows Release and Ubuntu Debug**, including new strict
same-input reproduction and positive-double-remainder checks.

For h=0.125m, checkpoint 12/16, cell (28,26):

| Independent component comparison (m²) | Windows Release | Ubuntu Debug |
| --- | ---: | ---: |
| Engine/float-core area | 0.011288604901483743 | 0.011288604901483743 |
| Double-core area | 0.011288605196665229 | 0.011288604881778193 |
| **Float minus double core area** | **-2.95181486892e-10** | **+1.97055496920e-11** |
| Engine/float-axis first cap area | approx. 0.0043363951027 | approx. 0.0043363951027 |
| Double-axis first cap area | 0.004336394744327535 | 0.004336395059209390 |
| **Float minus double first cap area** | **+3.58133721463e-10** | **+4.32518664981e-11** |
| Existing engine signed free area, before clamp | -4.19041190636e-12 | -4.19041190636e-12 |
| **All-double reconstructed positive free area** | **+5.90072365969e-11** | **+5.90124164812e-11** |
| Independent signed-distance wedge area | +5.87661436165e-11 | +5.87675484957e-11 |

The independent **float reconstruction** reproduces the engine's
core exactly and the first cap within 2.5e-13m² on both platforms.
This localizes the spurious negative remainder to geometry rounding,
rather than an entirely unrelated pressure matrix or an unverified
integration call. The independent double construction leaves a
small positive free region and agrees with the separate signed-distance
wedge within approximately 2.5e-13m².

**Cross-platform caveat:** the separate double-core and double-cap
values differ substantially between Windows and Ubuntu, and the
individual float-minus-double contributions even differ in sign for
the core. Nevertheless, the respective total exterior areas remain
close, owing to component cancellation. This test does not yet
identify the exact source of the platform-dependent endpoint
difference, and it does **not** prove the full analytic geometry
policy is stable for arbitrarily thin cells or two-way FSI.

The result suggests avoiding double arithmetic applied **after**
the capsule axis and polygon vertices have already been rounded to
float. A potential experimental production option would construct
axis, core vertices and cap partition consistently in double
precision for single-Capsule Cut-cells. That requires consistent
cell **and shared-face** aperture calculations, swept-volume/source
history, pressure graph topology, and performance tests before
any default change. Do not simply mark the cell as active with an
epsilon: the tiny positive fraction can become an ill-conditioned
pressure unknown and alter tracer/Body feedback.

**Next experiment:** add diagnostic printing of the exact
float-transformed Capsule endpoints/axis across the two compilers,
then compare a separately opted-in double-geometry policy on
identical prescribed motion (and on the frozen real World8/Fluid4
near-disappearing cell), including small-face pressure and
volume/tracer/Body conservation ledgers. Keep PR #326 Draft and
the pressure-pooling/tracer experiments disabled by default.

### Opt-in double Capsule geometry: actual field A/B and frozen-PCG gate — 2026-11-15

The earlier single-cell double-axis/core decomposition is now exercised
as an **explicitly disabled-by-default** Fluid policy,
`setAnalyticCapsuleDoubleGeometryEnabled(true)` (and the pre-existing
`setAnalyticCapsuleCutCellEnabled(true)`). Commits `0517f758`,
`d4be5de8`, `a0434669` add a double-normalized Capsule axis,
double-coordinate core vertices, and consistent double-coordinate
**cell, shared face, swept boundary and pressure/Body face** clipping.
The float-transformed Shape endpoints remain the same inputs.
Single-Capsule only: disjoint/multiple-Shape paths and defaults remain
unchanged. Policy switches invalidate obstacle and volume history.

[Windows Release and Ubuntu Debug CI #43654229820](https://github.com/buddas-bible/zonai/actions/runs/43654229820)
**both passed**, including the new three-policy prescribed-motion
`--small-cell-capsule-geometry-audit`. Each mode follows the same
specified 128-substep rotating Capsule trajectory separately, with
frozen zero-open-MAC snapshots solved by independently assembled
double-precision PCG. Geometry histories are **different states**,
not a same-state matrix A/B. The audit prints cell area and source
differences, pressure-anchor aperture sums, reference target face
volumes and explicit true PCG residual/convergence.

| h (m), checkpoint | Float analytic pressure anchors | Double analytic anchors | Float active cells (Windows / Ubuntu) | Double active cells (Windows / Ubuntu) |
| --- | ---: | ---: | ---: | ---: |
| 0.25, 11/16 | 0 | 0 | 1023 / 1023 | 1023 / 1023 |
| 0.25, 12/16 | 0 | 0 | 1023 / 1022 | 1022 / 1022 |
| 0.125, 11/16 | 0 | 0 | 4087 / 4087 | 4080 / 4080 |
| **0.125, 12/16** | **2 (opening sum 0.00017798537)** | **0** | **4086 / 4080** | **4081 / 4084** |

At the near-tangent witness cell (28,26), h=.125/checkpoint12:
the existing float analytic mode retains fluid fraction **0**;
the new double policy retains a **positive** area of
5.87661464629e-11 m² (Windows) or 5.87675394459e-11 m²
(Ubuntu), matching the independent signed-distance wedge to
within 3e-18m² or 1e-17m² respectively.
The two associated shared U/V face apertures now connect to an
**active** cell rather than a falsely covered zero-pressure node.
The next test commit `c2538137` guards the concrete
2-to-0 anchor transition and independently checks both face
apertures at this checkpoint.

The **independent frozen zero-MAC PCG** at h=.125/checkpoint12
converged under 1e-5/s true RMS in both policies and both OSes:
float analytic 414 iterations versus double analytic **217**
iterations. Target-face reference integrated volume was
0.000455333857498 m³ (Windows float) versus
0.000458961534468 m³ (Windows double);
Ubuntu 0.000455333849517 versus 0.000458961549608 m³.
Thus the double policy changed that reference volume by
approximately **+3.6277e-6 m³**. It is a *geometry and history*
difference, not a same-operator correction or proof that the
physical reference must be the double value.

**Limitations/gates:** double geometry fixes the local topology
witness and both platforms' anchor count there, but it does not
make every near-zero cell's classification monotonic: for
h=.125/checkpoint11, seven float-active cells became double-inactive.
At checkpoint12 the count/locations of newly active and newly
inactive cells differ by OS. No acceptance of global fluid
volume conservation, tracer advection, moving-wall flux, two-way
Body motion, or performance has followed from this experiment.
The disjoint multi-Shape World8/Fluid4 case is NOT covered by this
single-Capsule geometry option. Keep S0/S2 open, PR #326 Draft,
and all experimental options OFF by default.

**Next:** localize and independently integrate near-zero
**double-inactive** cells; quantify pressure-source and swept
face-volume budgets as a matched physical error, then test a
separate multi-Shape geometry policy only after it preserves
the single-Shape regression and disjoint Shape ownership rules.
Do not simply remove anchor edges, apply a per-cell epsilon,
or merge pressure pooling based on pressure iteration counts.

### Independently classified near-zero Capsule cells — 2026-11-15

Commits [`937a0d63`](https://github.com/buddas-bible/zonai/commit/937a0d63aac1c3e570b2191cd949be5f946866a7)
and [`51a612ce`](https://github.com/buddas-bible/zonai/commit/51a612ce4c78de07b1fe7354fa38b6d0a263a3d8)
extend the **three-mode prescribed-motion geometry test**, not
production physics. At each frozen checkpoint, the audit independently
rebuilds the point-to-segment signed distance to the Capsule using the
same float-transformed endpoints/radius. It classifies **all** cell
rectangles from four corner signed clearances, with a strict
1e-10m confidence margin:

- If any corner lies confidently outside a convex Capsule, an open
  area must exist; a zero stored fluid fraction is a **false zero**.
- If all four corners lie confidently inside a convex Capsule, the
  whole rectangular cell is covered; any positive stored fraction is
  a **false positive** (spurious pressure-active cell).
- Near-tangent cases inside that margin are *unknown*, not forced into
  either conclusion. No near-tangent unknown cells occurred at these
  four checkpoints on either OS.

For each reclassified or provably misclassified cell the CSV
`capsule_corner_topology_cell` logs coordinates, float/double
fractions, independent corner clearances, all four MAC openings,
previous fractions, snapshot volume changes, pressure sources and
swept-face divergence volumes. The geometry histories are separately
evolved and should not be confused with the same-state pressure
matrix-only counterfactual.

[CI #43656830145](https://github.com/buddas-bible/zonai/actions/runs/43656830145)
**passed Windows Release and Ubuntu Debug**. Summary:

| Grid / checkpoint | False zeros: float / double (Windows, Ubuntu) | False positives: float / double Windows | False positives: float / double Ubuntu |
| --- | --- | --- | --- |
| h=.25, 11/16 | 0 / 0 (both) | 0 / 0 | 0 / 0 |
| h=.25, 12/16 | 0 / 0 (both) | **1 / 0** | 0 / 0 |
| h=.125, 11/16 | 0 / 0 (both) | **9 / 2** | **9 / 2** |
| **h=.125, 12/16** | **1 / 0 (both)** | **10 / 4** | **4 / 7** |

The genuine corner wedge in cell **(28,26)** at h=.125,
checkpoint12 has positive exterior clearance
7.29305747e-6m (Windows) / 7.29314458e-6m (Ubuntu);
the float analytic path marked it as zero and the double path
restores a fraction approximately 3.761e-9. It corresponds to
the two former active-to-inactive positive-opening pressure faces.

The reverse changes **are not uniformly loss of real fluid**.
At h=.125/checkpoint11 the seven cells losing float-active status
in the double path are inside the Capsule by at least
approximately **0.026m** at their furthest corners, on both
platforms; the original float fractions were numerical artifacts
(typically 1e-16, but (27,26) was 1.52e-8 and (33,34)
was 6.25e-9). All reported false-positive cases in this
trajectory have **zero endpoint open MAC faces** and
negligible snapshot volume residuals, but their active flags
can still change the independent pressure graph.

**Remaining defect:** even the double geometry path produces
spurious fluid fractions for cells provably fully inside the
convex Capsule. It produces 2 false positives at h=.125/checkpoint11
on both OSes, and **4 Windows / 7 Ubuntu** at checkpoint12.
These counts vary across compilers although the independent
corner geometry is confidently covered, indicating the
core/half-cap area partition still retains near-full-cell
cancellation error. A double-coordinate construction alone
does **not** establish platform-stable cell masks or
pressure-source consistency.

The follow-up test commit
[`c2c2dfcf`](https://github.com/buddas-bible/zonai/commit/c2c2dfcf81c92ad3f6896bf3c00aa0b03d826bf2)
turns **zero proven-open / zero stored double-fraction** into a
cross-platform enforced regression assertion. The fully
covered but spuriously active cells remain *reported*, not
silently fixed or swept into a cell-fraction epsilon.

**Next bounded work:** separately prototype an opt-in convex
full-coverage guard for near-zero double-Capsule fractions,
based on all four corners being **strictly** inside and
consistent zero-open shared faces. Compare it against the
current double path in the same prescribed trajectory and then
the real World8/Fluid4 coupled-feedback path, with source/swept
volume, tracer transport, PCG conditioning and performance
acceptance. Do not change any default or broaden to
multi-Shape/disjoint ownership before these gates.

### Opt-in convex full-coverage guard and active-mask parity — 2026-11-15

Commits [`907a7368`](https://github.com/buddas-bible/zonai/commit/907a73681b73d698894f0f522890a77c4a60506f)
and [`91308434`](https://github.com/buddas-bible/zonai/commit/91308434ffead8574dd254495b1e8a29ba265ced)
introduce `setAnalyticCapsuleFullCoverageGuardEnabled(bool)` as an
independent **OFF-by-default** experimental policy, effective only
with single-Capsule **double** analytic geometry enabled. When that
geometry's calculated fluid fraction is strictly between 0 and 1,
all **four stored shared MAC apertures** must first be exactly closed.
Only then a separate, original-endpoint, double-precision
point-to-segment distance witness checks that **all four rectangular
cell corners lie strictly inside** the convex Capsule by a
64-double-ULP scaled arithmetic margin. By convexity this certifies
the **entire cell** is covered; the cell's spurious positive fraction
is set to 0 before moving-volume history and pressure sources are
assembled. Neither the value of the original fraction nor an
arbitrary near-zero fraction threshold decides occupancy. The actual
shared-face aperture data are untouched. Switching the option
invalidates geometry and volume histories.

Commit [`7b742662`](https://github.com/buddas-bible/zonai/commit/7b74266235b36d274e1f88e7e4febb22286f6c20)
adds a **four-mode** comparison of sampled, float analytic, double
analytic, and double analytic + guard policies. Independent signed
distance, four-face identity, geometry/pressure-source/swept ledgers,
and an independently assembled frozen zero-MAC PCG check that
corrected cells are certified fully covered and that no confidently
open cell was deleted.

[CI run #43692261160](https://github.com/buddas-bible/zonai/actions/runs/43692261160)
**passed Windows Release and Ubuntu Debug**.

| h=0.125m, checkpoint | Double active (Windows / Ubuntu) | Guarded active (both) | Corrected cells (Windows / Ubuntu) | Guarded false zeros / false positives (both) | Frozen PCG iterations (both policies) |
| --- | --- | ---: | --- | --- | ---: |
| 11/16 | 4080 / 4080 | **4078** | 2 / 2 | **0 / 0** | 214 |
| 12/16 | 4081 / 4084 | **4077** | 4 / 7 | **0 / 0** | 217 |

At h=0.25m both checkpoints require no corrections from the
already double-geometry path. The small genuine open wedge at
h=.125, checkpoint12, cell (28,26) **remains active**, and both
preexisting active-to-inactive pressure anchors remain absent in
the guarded mode. Shared face apertures compare exactly equal
between the double and guarded modes at both grid resolutions.
All independent frozen PCG checks converged.

At h=.125m/checkpoints 11 and 12, the L1 differences in
pressure-source volumes and swept divergences were **0** in
both OSes. Snapshot-volume L1 changes were at most
3.47e-17m³, consistent with removal of positive machine-rounding
remainders in fully covered cells. This is **not** validation of
moving-solid geometric mass conservation or of tracer remapping
in a two-way coupled World.

The diagnostic wall-clock profiler totals for 128 prescribed
Fluid substeps (and separate copies of the pressure solve):
- Ubuntu Debug, h=.125: `geometry` 50.903ms (double) vs
  53.139ms (guarded); `total` 5285.813ms vs 5249.578ms.
- Windows Release, h=.125: `geometry` 15.275ms (double)
  vs 14.206ms (guarded); `total` 730.267ms vs 655.409ms.

These **single-run CI timings are not a performance acceptance
or reliable proof of speedup/slowdown**: different compilers, noisy
runners and sequential trial order dominate differences that are
small compared with the pressure solve. The guarded policy scans
the grid after area and face computation, but executes its more
expensive corner witness only on positive, non-full cells with
four closed MAC faces. Repeat controlled performance runs before
broader enabling.

Commit [`c1f7f84a`](https://github.com/buddas-bible/zonai/commit/c1f7f84af993f5698de38ddc764cf5e3618ccd6a)
adds a row-major FNV-1a **binary active-mask** checksum for each
mode/checkpoint, not merely the count. In
[CI #43692996036](https://github.com/buddas-bible/zonai/actions/runs/43692996036),
**both platforms passed** and the guarded masks matched across
toolchains exactly at the two h=.125 checkpoints:

| Checkpoint | Windows Release FNV-1a 64 | Ubuntu Debug FNV-1a 64 |
| --- | --- | --- |
| 11/16 | `8787b49d7bcf8e49` | `8787b49d7bcf8e49` |
| 12/16 | `8d0ee9a33f07407c` | `8d0ee9a33f07407c` |

The unguarded double mask at 12/16 instead differed across
toolchains (`6b305b093bd54132` Windows,
`add3e65393a27a17` Ubuntu). A matching 64-bit checksum
is strong evidence for matching masks in **this specific fixed
trajectory**, not a formal universal geometry proof. Commit
[`5e132955`](https://github.com/buddas-bible/zonai/commit/5e132955a415f9bae7005680e082f7dd8550af5d)
pins these observed values as per-toolchain regression goldens.

**Remaining acceptance gates:** test moving/rotating single-Capsule
volume and tracer transport against independent swept-surface and
face-capacity references at near-disappearing cells; independently
compare the *actual* pressure source and full rigid Body response
under matched physical initial conditions. The World8/Fluid4
multi-Shape experiment currently exercises a different disjoint
analytic geometry path and is **not** covered by this single-Capsule
guard. Neither double geometry nor this guard is enabled by
default; PR #326 remains Draft pending physical, two-way coupling
and repeatable performance gates.

### Live prescribed Capsule transport and extreme face-speed witness — 2026-11-15

Commits [`3f2153a2`](https://github.com/buddas-bible/zonai/commit/3f2153a2ed9593943100e0c1423d9bd330a1d476)
and [`50ff1835`](https://github.com/buddas-bible/zonai/commit/50ff183534e7d46ffc43d920a366d036ad84c7c5)
add an independent **live fluid Step** audit,
`--small-cell-capsule-moving-transport-audit`, to both CI OSes.
Unlike the previous zero-MAC frozen PCG oracle, it evolves MAC
velocities, pressure, rotating solid geometry and conservative tracer
over 128 substeps of a 1/30-second prescribed rotating Capsule.
The two separately evolved variants differ **only** in the optional
convex full-coverage guard; both use single-Capsule
double-precision geometry, swept tracer remapping and ordinary
Cut-cell projection. The existing connected-residual tracer
experiment, small-cell pressure pooling, dynamic Body inverse
mass and coupled Schur pressure are all **disabled**.
The Capsule's motion is **kinematically prescribed**: returned
`fluidObstacleForce2` feedback is measured but **NOT**
integrated into a rigid Body trajectory. This is *not* a two-way
FSI acceptance test.

At initialization 247 fully open cells common to both variants
receive identical tracer concentrations (initial total
**1.72629459528 kg**). The test independently sums raw
double-precision tracer cell masses at both actual applied
tracer stages (`afterSweptRemap` and `afterMacAdvection`),
and at the end of each Fluid substep. It records face velocity,
pressure force/torque, current fluid volume, swept geometry
mismatch, pressure-source volume and actual tracer transfer
ledgers at every 8th substep. A 1e-6 kg scaled absolute
stage-mass check and finite physical-budget checks are enforced;
pressure-force difference is informational, not silently asserted
to be zero.

[Windows Release and Ubuntu Debug CI #43697427172](https://github.com/buddas-bible/zonai/actions/runs/43697427172)
**both passed**.

| 128-step live transport result | Windows Release | Ubuntu Debug |
| --- | ---: | ---: |
| Stages observed per mode (swept + MAC) | 128 + 128 | 128 + 128 |
| Maximum absolute tracer mass defect, either policy | 3.77476e-15 kg | 3.77476e-15 kg |
| Maximum double-summed fluid volume difference between policies | 0 m³ | 0 m³ |
| Maximum pressure-source volume difference | 2.91038e-11 m³ | 2.91038e-11 m³ |
| Maximum emitted pressure force difference | 141.0173 N | 141.0171 N |
| Maximum emitted torque difference | 41.2512 N·m | 41.2513 N·m |
| Largest single-mode force magnitude seen | 1154.153 N | 1154.153 N |
| Maximum swept geometry mismatch (either mode) | 1.275028e-4 m³ | 1.275028e-4 m³ |
| Sum of applied swept tracer remaps, unguarded / guarded | 0.082747 / 0.057826 kg | 0.081796 / 0.057826 kg |
| Sum of unresolved tracer displacement, unguarded / guarded | 0.230027 / 0.257237 kg | 0.230854 / 0.257237 kg |

The tracer remap/unresolved numbers are **sums of substep
transfer metrics**, not the final missing mass (which was
conserved to ~4e-15 kg). Their sizeable mode dependence
demonstrates why **global tracer mass conservation alone
does not establish physically correct local transport**. The
kinematically emitted pressure forces also differ; their
maxima do **not** necessarily occur at the same substep as
the maximum face speed. There is no claim that either policy
is a validated physical reference.

Commit [`b52afcab`](https://github.com/buddas-bible/zonai/commit/b52afcabd61d2eb3f4b5b388a826c7f57dc4376f)
adds `capsule_moving_velocity_hotspot` records for every
open MAC face with computed speed over 1000 m/s, logging
the location, open fraction, aperture-weighted speed,
**nominal** one-substep face volume and both adjacent
cell/source/residual ledgers.
[CI #43698532451](https://github.com/buddas-bible/zonai/actions/runs/43698532451)
**passed both OSes**. Both variants identified exactly the
same extreme face at the same time:

| h=0.125m, substep 96/128 | Windows Release | Ubuntu Debug |
| --- | ---: | ---: |
| Axis, shared face | **U(29,26)** | **U(29,26)** |
| Signed open-face velocity | **5486.3423 m/s** | **5486.2764 m/s** |
| Endpoint open fraction | 0.0001088336721 | 0.0001088349745 |
| Open-weighted face speed | 0.5970988 m/s | 0.5970988 m/s |
| Nominal one-step open face volume `h*dt*open*u` | 1.9436810e-5 m³ | 1.9436809e-5 m³ |
| Fluid fraction of left cell (28,26) | 3.7610333e-9 | 3.7611225e-9 |
| Fluid fraction of right cell (29,26) | 0.6825421 | 0.6825422 |
| Pressure-source volume on left cell | -3.1771167e-5 m³ | -3.1771167e-5 m³ |
| Left pressure residual volume | -1.975e-12 m³ | -2.095e-12 m³ |

At h=.125m the tiny left cell's fluid volume is
approximately **5.877e-11 m³**, whereas the **nominal**
one-step `open*velocity*faceLength*dt` is about
**3.3e5 times that volume**. The local pressure-source
volume magnitude is about **5.4e5 times that tiny
current cell volume**. The pressure residual of the tiny
cell is numerically small, but this does NOT imply a
physical, stable or capacity-respecting face flux.
Neither the still-present real near-tangent sliver nor
its extreme velocity is removed by the convex
full-coverage guard. The guard only removes **provably
fully covered false-positive** cells.

**Interpretation limitations:** `h*dt*open*u` uses the
endpoint open fraction and endpoint velocity; it is NOT
the exact time integral of a moving-face flux, and it
cannot by itself establish the true physical fluid
transport. The saved velocity is a discrete face
unknown, not necessarily a realized trajectory for
a fluid particle. These ratios nevertheless isolate
a numerically severe small-cell scale disparity and
show why global mass/error and PCG convergence are
insufficient as S2c acceptance tests.

**Next bounded S2c priority:** use the actual frozen
step-96 state and two adjacent cells at U(29,26) to
separate the *raw swept solid source* from its
pressure-compatibility correction, the discrete
pressure impulse, the time-averaged aperture
capacity and the tracer's **applied** neighbor
transfers. Test a mathematically conservative small-
cell pressure aggregation/flux-limiting candidate
only in a separately opted-in A/B against this
frozen state, maintaining valid Body impulse
accounting and avoiding a trivial raw-speed cap.
Then extend to actual two-way World Body motion
with comparable physical initial states and
repeatable benchmark timing. Keep PR #326 Draft
and all new options OFF by default.

### Step-96 Capsule aperture audit: spatial aliasing isolated — 2026-11-15

PR #326 commits `53063d41`, `72a977a2`, `66b161c3`, `e06fc42a` fix a **diagnostic** mismatch and add
an explicit high-resolution, single-face time-quadrature witness. Previous
`getSweptUTimeMeanOpenFraction(29,26)` reported **0** even though
the endpoint U aperture was **0.000108835**. The pressure and swept-solid
flux paths used single-Capsule analytic clipping, whereas the opt-in
time-aperture observer used spatial midpoint stencil sampling unless
`BuildDisjointAnalyticShapes` succeeded. The Capsule-only audit used the
sampled fallback. The observer now selects the same analytic primitive
clipping as the live Cut-cell geometry for single Circle/Polygon/Capsule/
RoundedPolygon, while other geometries retain their original fallback.
A read-only on-demand single-Capsule face query makes temporal resolution
checks independent of the whole-field time audit. No production
advection, pressure, Schur, solid flux or tracer model was changed.

[Ubuntu Debug CI #43714973788](https://github.com/buddas-bible/zonai/actions/runs/43714973788)
passed its 112 CTest tests and the opt-in 128-step Capsule audit.
The endpoint-geometry analytic query agrees with the stored U(29,26)
opening within 3e-12 in the printed control. The two guard modes
produced the same displayed aperture values:

| Midpoint time samples | Nonzero samples | Mean open fraction |
| ---: | ---: | ---: |
| 1 (actual accepted swept-time sample) | 1 | 0.0399289856 |
| 16 | 16 | 0.0398327543 |
| 64 | 64 | 0.0398324559 |
| 256 | 256 | 0.0398324794 |
| 1024 | 1024 | 0.0398324420 |
| 4096 | 4096 | 0.0398324535 |
| 16384 | 16384 | 0.0398325807 |

The original **zero was spatial aliasing of the narrow opening, not
time-midpoint aliasing** in this controlled single-Capsule case.
The previous accepted 1-point time sample, once geometrically
consistent, differs from the 16384-point analytic-time reference by
approximately 0.24% of the reference. This does not prove that one
time sample suffices for other solid trajectories or topology changes.

At the same U face, raw pre/post-pressure velocities remain
23.1379/5486.2764 m/s, while current endpoint aperture is
0.000108835. The *nominal endpoint* face volume is
1.94368e-5 m³ versus the **linear-endpoint velocity hypothesis**
integrated against the sampled time aperture at 1-point resolution
of **0.00358077 m³**. These are two diagnostic approximations;
**neither is an actual measured fluid-particle transport or a
conservation-certified moving-face flux**. The directly applied tracer
transfer between the two adjacent cells is zero/negligible here;
global tracer conservation alone is not physical local validation.
The pressure-only frozen replay still reproduces the observed
endpoint flux, not a physical resolution of its scale.

**Next bounded step:** separate the exact single-face space-time
solid swept integral from reconciled displacement and the
region-compatibility-adjusted pressure source at the identical
step-96 checkpoint. Investigate conservative time-consistent
moving-face/source treatment on independent frozen copies while
preserving total source and Body impulse; then verify coupled World
trajectories at multiple cadences. Keep #326 Draft and every new
physical option OFF.

### Step-96 matched-entry swept reconciliation and pressure-source ledger — 2026-11-15

PR #326 commits `ab63875a` and `9efe2be5` extend the **existing opt-in
single-face stage observer** with a raw swept-solid face volume captured
immediately after `updateSweptSolidFlux` and a reconciled swept-solid
volume captured immediately after `reconcileSweptSolidFlux`. No
reconstruction from post-pressure MAC velocities is used. The normal
Field step does not allocate or run additional diagnostics when the
observer is not selected.

Commits `d2819da5`, `a577920c`, and `269fb9df` extend the
existing `--small-cell-capsule-hotspot-ledger-audit`: clone the actual
fluid Field **immediately before the 96th fluid Step**; the original
continues with `sweptGeometryCorrectionEnabled=false`, while a
one-step-only copy enables the already implemented optional geometric
flux reconciliation. Both receive identical Capsule transforms,
velocities, step duration, and pressure iteration budget. The copied
Field is not reused as a separate 128-step history. The test asserts
identical raw swept face flux between copies and verifies the
read-only audit against a partial-open Capsule fixture.

**Verified:** [CI #43721676946](https://github.com/buddas-bible/zonai/actions/runs/43721676946),
Ubuntu Debug 112/112 CTest, Windows Debug 115/115 CTest, Windows
Release 52/52 CTest, and both opt-in 96th-step headless audits pass.
No runtime physics, defaults, tracer remap or Body trajectory changed
by the new trace itself.

| Measured quantity | Ubuntu baseline | Ubuntu corrected clone | Windows baseline | Windows corrected clone |
| --- | ---: | ---: | ---: | ---: |
| Raw U(29,26) swept solid face volume (m³) | 0.000582699140068 | 0.000582699140068 | 0.000582699140068 | 0.000582699140068 |
| Reconciled U(29,26) swept solid face volume (m³) | 0.000582699140068 | 0.000582780281547 | 0.000582699140068 | 0.000582780281547 |
| Corrected faces | 0 | 15 | 0 | 15 |
| Final swept/snapshot L1 mismatch (m³) | 9.11950e-7 | 3.22179e-7 | 9.13260e-7 | 3.28506e-7 |
| Left-cell pressure source (m³) | -3.17711674143e-5 | -3.16900259349e-5 | -3.17711674143e-5 | -3.16900259349e-5 |
| Right-cell pressure source (m³) | -0.000672970549203 | -0.000673051690683 | -0.000672970549203 | -0.000673051690683 |
| Max raw fluid-face velocity (m/s) | 5486.27637 | 5472.26855 | 5486.34229 | 5472.33496 |
| Active cells in connected hotspot region | 4077 | 4077 | 4077 | 4077 |

The chosen U face gains **8.11414793134e-8 m³** of swept-solid
flux. Its adjacent current-active pressure sources change in
opposite directions by approximately the same magnitude: this
is a conservative pairwise *source redistribution*, not a new
body force or a transported tracer-mass measurement. The
optional reconciliation substantially reduces geometric mismatch
(approximately 64–65%), but the maximum *raw* MAC velocity is
only ~0.255% lower. The huge velocity is therefore **not explained
solely by raw swept geometry mismatch** in this same-entry,
prescribed-kinematic Capsule control. The original endpoint-open
face remains O(1e-4); no claim of stabilized FSI or pressure pooling
acceptance follows.

**Connected-component and precision caveat:** the read-only audit
reconstructs each active region's pre-pressure net MAC volume plus
**current projected** pressure source, then separately sums region
imbalance and applies the unresolved-source term, matching the
discrete conservation *expression* but with double accumulation
from public per-cell budgets. The solver accumulates float
divergences and removes the per-region float mean. In this 4077-cell
region the double reconstruction of reported compatibility is
roughly 3.2–3.6e-7 m³/s; solver-reported totals range from
3.54e-7 to 8.43e-7 m³/s across OS/branch, an absolute difference
of order 1e-7 m³/s. This has **not** been independently attributed
solely to float rounding. It is tiny compared with the local
left-cell volume source (~3.17e-5 m³ each 0.000260417s), but it
demands a solver-operation-order-matched float reconstruction
before using this audit as a compatibility/conservation proof.
Do **not** claim exact pressure-source/global volume closure from
the rounded diagnostic or treat this result as a GPU/CPU parity test.

**Next S2c acceptance-gated experiment:**
1. Reconstruct `(fluxE-fluxW+fluxN-fluxS)/h + volumeRate/dt` with
   the solver's float operation order, compare per-cell and
   per-component source sums and the reported mean subtraction, and
   isolate any remaining reproducible discrepancy.
2. On the exact same frozen state, evaluate **pressure projection
   alternatives** that account for extremely small endpoint
   apertures and finite swept displacement, including
   source-conservative small-cell grouping/flux redistribution.
   Preserve summed source, effective face momentum and Body impulse;
   do not introduce arbitrary velocity clamps.
3. Run opt-in multi-grid/multi-cadence **coupled** Body, joint and
   tracer tests before considering any default change.

Keep PR #326 Draft, PR #322 independent and all new physical
strategies OFF by default.

### Step-96 pressure Neumann compatibility: reproduce solver float order — 2026-11-16

**Scope:** S2c read-only investigation, on the exact same 96/128
prescribed rotating-Capsule checkpoint as the matched swept-source
audit above. PR #326 adds:
- `getUFaceVelocityForAudit` and `getVFaceVelocityForAudit` as
  side-effect-free optional raw stored MAC component queries
  (commit `26144c71`).
- An independent connected-component region walk on the real
  **pre-pressure frozen** MAC snapshot using **the current step's
  projected** `pressureSourceDeltaVolume` (not the stale frozen
  `cutCellVolumeRate_`). Each face performs
  `float(open) * float(velocity)`, each cell computes the same
  `float((uE-uW+vN-vS)/h + volumeRate/dt)` as
  `projectCutCell`; only **then** is each float divergence
  accumulated into a double region sum. A second control independently
  uses double MAC/volume budgets as before (commit `ed650351`).
- Commit `7a40d134` asserts reconstructed region compatibility and
  total source imbalance agree with the reported float solver values
  within an absolute **1e-10 m³/s**; `d9f4924e` checks raw face
  query bounds. These latter regression gates are code-bearing
  follow-ups to the initial evidence run below.

**Cross-platform initial reproduction:** [Actions #43737915486](https://github.com/buddas-bible/zonai/actions/runs/43737915486)
passed Ubuntu Debug **112/112**, Windows Debug **115/115**,
Windows Release **52/52**, including the opt-in step-96 audit.

| Step-96 metric (m³/s) | Ubuntu original | Ubuntu geometry-corrected | Windows original | Windows geometry-corrected |
| --- | ---: | ---: | ---: | ---: |
| Solver-reported region compatibility | 7.70847691456e-7 | 8.43490852276e-7 | 3.53840732714e-7 | 5.62456989428e-7 |
| Independent **float-order** compatibility | 7.70847691456e-7 | 8.43490852276e-7 | 3.53840732714e-7 | 5.62456989428e-7 |
| float-order minus solver | **0** | **0** | **0** | **0** |
| Older double-budget minus solver | -4.07631906621e-7 | -5.22184581113e-7 | +9.37505393076e-9 | -1.99241202783e-7 |
| Independent float-order source imbalance minus solver | **0** | **0** | **0** | **0** |

The monitored connected region contains **4077 active fluid cells**.
The maximum individual cell RHS discrepancy between the
double-budget calculation and the float-order evaluation is
7.55e-6..1.91e-5 s⁻¹ across these branches/platforms. The
region sums demonstrate that the previously reported mismatch
was **due to the independent diagnostic not reproducing the
float face/cell operation order**, not evidence of an independently
different Neumann correction in this selected scenario. This
is a **discrete arithmetic identity check**, not a proof of
continuous mass conservation, physical fluid transport, solver
accuracy, or cross-platform bitwise-deterministic dynamics.
The different Ubuntu/Windows absolute region compatibility
values may also reflect differences earlier in their separately
evolved float histories; do not attribute that variation to one
specific CPU operation without a frozen binary-identical input.

**Decision:** Do not change the pressure source solver simply to
force agreement with the old double-budget diagnostic. Maintain
both diagnostic estimates, labeled with their distinct semantics.
The severe ~5.5 km/s tiny-face raw velocity persists and needs
a separate **conservative pressure/geometry discretization**
experiment on identical frozen states. Preserve summed volume
source, momentum/Body impulse, local tracer transfers, and
independent velocity/CFL acceptance criteria. No velocity clamp
or default physics change has been accepted.

### Frozen step-96 NONCOUPLED Galerkin pressure-pooling oracle — 2026-11-16

**Motivation:** the severe rotating single-Capsule U(29,26) velocity spike
(~5,486 m/s) happens in the **noncoupled Cut-cell Jacobi** path.
PR #326's existing small-cell pressure pooling is inside the coupled
Schur/PCG path and does not affect this prescribed kinematic Capsule.
Simply turning its original opt-in pooling flag ON could not test this
hotspot; a separate pressure-only oracle was necessary.

**New code and gate:** `tests/sandbox/wind_step_benchmark.cpp`
adds the explicit CLI
`--small-cell-capsule-noncoupled-pressure-oracle`. Commit
`7efea14e` builds an **independent frozen pressure operator** from the
REAL pre-pressure staggered MAC state, current per-cell pressure sources,
endpoint active-to-active aperture graph, and solver float-order
per-region Neumann compatibility. The field at step 96 of the real
128-substep prescribed-Capsule history is split into the existing
swept-geometry-correction OFF/ON *matched step-entry copies*.
Each branch is evaluated at merge-fraction thresholds `0`,
`1e-6`, and `1e-4`. Eligible *shrinking* active slivers share a
pressure unknown with their most-open directly connected neighbor
whose fluid fraction is at least the threshold; all original cell
RHSs are **summed** (`P^T b`), not dropped or clamped.
The matrix-free weighted-face operator is `P^T L P`; it solves
in volume units with `q = dt^2 pressure`, independent double
PCG with per-region Neumann gauge, Jacobi preconditioning, and a
true explicitly re-evaluated grouped residual.
The per-cell residual and target/maximum predicted MAC speed are
calculated independently from the resulting pressure potential.
Volume source equality before/after restriction is an enforced
`1e-11 m³` absolute gate. No normal-step work or solver code change.
Commit `052c666b` correctly **excludes active/inactive faces** from
this pressure oracle even when the raw geometric face opening is
positive; the real `cutCellU/VOpen` operator makes the same distinction.
Both operating systems execute the oracle in CI (workflow commit
`252b053d`).

**Cross-platform verified:** [CI #43747040464](https://github.com/buddas-bible/zonai/actions/runs/43747040464)
passed Ubuntu Debug **112/112**, Windows Debug **115/115**, Windows
Release **52/52**; the standalone pressure oracle passed on
Ubuntu Debug and Windows Release. The earlier unmasked oracle
[CI #43746904715](https://github.com/buddas-bible/zonai/actions/runs/43746904715)
also passed, but the corrected masked graph above is authoritative.

| Selected U(29,26) metric | Ubuntu original | Ubuntu pooled 1 cell | Windows original | Windows pooled 1 cell |
| --- | ---: | ---: | ---: | ---: |
| Total active pressure roots | 4084 | 4083 | 4081 | 4080 |
| PCG iterations | 135 | 133 | 135 | 133 |
| Explicit **group** residual RMS (1/s) | 9.53160e-5 | 8.99281e-5 | 9.53438e-5 | 8.99595e-5 |
| **Per-cell** continuity residual RMS (1/s) | 9.53160e-5 | **0.172104** | 9.53438e-5 | **0.172167** |
| Target U and global max predicted face speed (m/s) | 5486.275 | **23.1379** | 5486.341 | **23.1379** |
| Max open-weighted predicted face speed (m/s) | 16.2388 | 16.2414 | 16.2388 | 16.2414 |
| Pressure fluid X impulse proxy (N·s) | 0.101005 | **0.0859253** | 0.101004 | **0.0859238** |
| **Unresolved merged-child local residual L1 (m³)** | 0 | **3.1645195e-5** | 0 | **3.1645197e-5** |

The other swept-correction-ON branch also reproduces this tradeoff:
Ubuntu `5472.267 → 23.1379 m/s` and pointwise RMS
`~9.51e-5 → 0.171662 1/s`; Windows `5472.333 → 23.1379 m/s`,
pointwise RMS `~9.51e-5 → 0.171725 1/s`.
Both `1e-6` and `1e-4` thresholds select the same **one
shrinking sliver** at this snapshot. It shares the U(29,26) pressure
unknown with the neighboring larger cell, making that face's
pressure gradient zero **by construction**. Other faces are still
checked for displaced maxima. Both grouped and unpooled independent
PCG solves satisfy the **1e-4 s⁻¹ group residual target**.
The restricted signed source sum is exactly the same as the
unrestricted signed source sum at reported precision (before
the separately measured Neumann gauge correction, order 1e-11 m³).
Linux/Windows have independently evolved float histories and
slightly different numbers of positive-active cells (4084/4081);
these are not claims of bitwise cross-platform determinism.

**Critical rejection of early physical acceptance:** the dramatic
drop in *raw* speed is achieved by deleting the sliver's
**individual** continuity row, not by actually moving the displaced
fluid. Its unfulfilled local exchange `~3.16e-5 m³` is roughly
`5.4e5` times the current sliver fluid volume
(`~5.88e-11 m³`). It is a **virtual grouped residual**, not
a measured tracer transfer, swept flux, or realized internal
fluid volume. The fluid impulse proxy changes by about 15% in
X; no consistent equal/opposite Body or wall impulse was computed
for this frozen alternative. Therefore **do not enable this
pooling in production or mark FSI, mass transport, pointwise
continuity, energy, or stability acceptance as passed**.
Actual `projectCutCell` (120 Jacobi iterations) is also distinct
from the independent PCG oracle (135/133 iterations); parity of
their ~5,486m/s result is a useful control, not a general solver
equivalence proof.

**Next S2c experiment, still opt-in:** construct an explicit
*conservative internal face-exchange ledger* for the same pooled
root, on the same frozen states and the actual swept/time-open
aperture. Derive the transfer needed by each omitted child
continuity equation, enforce equal/opposite source volume and
nonnegative tracer donor limits, account for the enormous
exchange-to-sliver-volume ratio, and reconcile it with fluid/Body
pressure impulses. Test h/time/cadence and full two-way moving
Body/joint trajectories before any solver integration.
Preserve existing S0–S9 roadmap and keep PR #326 Draft,
#322 independent, runtime physics defaults unchanged.

### Frozen pressure-pool internal exchange vs time-open face and post-transport inventory — 2026-11-16

**Why:** the independent noncoupled pressure Galerkin oracle at the real rotating
Capsule **step 96/128**, U(29,26), proved it can suppress a raw
~5.5 km/s pressure velocity by merging one shrinking sliver with its
neighbor, but left **~3.16e-5 m³** of pointwise continuity
unfulfilled. That is a group-row omission, **not** an observed physical
tracer transfer or evidence that all that water is still resident in
the sliver.

**Implemented opt-in, no production/World changes:**
- Commit `893388e1` extends the existing separate
  `--small-cell-capsule-noncoupled-pressure-oracle`, not the ordinary
  `--small-cell-capsule-hotspot-ledger-audit`. For every grouped
  child (c), after the independently solved `P^T L P` equation,
  it calculates the signed child-outward **necessary internal exchange**
  `b[c] - (A_full * Pq)[c]`, gives the connected parent the exact
  opposite signed request, and reconstructs the actual face from
  neighboring cell indices.
- A **256-midpoint** independently analytic moving-Capsule face
  aperture integral over *the same* step 95→96 (not the endpoint
  aperture and not a measured (u(t))) provides a `meanOpen`.
  It prints the speed **needed** to move the required amount if velocity
  were time-constant, and a distinct **reference capacity** using
  the stored *pre-pressure* face speed as a stationary hypothetical
  proxy. These are explicitly NOT actual time-resolved volume fluxes,
  guaranteed speed bounds, or a replacement for moving-solid swept
  flux.
- A hypothetical **additional** remap, evaluated *after the normal
  step's swept + MAC tracer transport*, is limited by both the
  reference-time-open face capacity and the donor's **remaining
  endpoint cell volume**. Candidate tracer kg use the donor's
  current stored mass/volume, and are additionally limited by the
  donor's remaining kg. Both sides are recorded with equal/opposite
  signed volume and mass. No real Field state or tracer is mutated.
- Commit `0f4755a3` strengthens this to a scratch-array inventory
  test for all active cells: net hypothetical volume and mass deltas
  must each sum to zero, no donor can send more than its *original*
  post-transport inventory even when several children share a donor,
  no resulting hypothetical inventory can go negative, and the
  total requested child exchanges must equal the earlier independently
  reported `virtualInternalExchangeVolume` within 1e-8 m³.
  This check is **bookkeeping conservation**, not physical
  transported-fluid acceptance.

**First cross-platform code-verification:** [CI #43759253449](https://github.com/buddas-bible/zonai/actions/runs/43759253449)
passed Ubuntu Debug **112/112**, Windows Debug **115/115** and
Windows Release **52/52**, including the new
`capsule_hotspot_internal_transfer` rows on both platforms.
The additional scratch-inventory regression is [CI #43759839143](https://github.com/buddas-bible/zonai/actions/runs/43759839143)
(consult current job status; it was executing when this evidence was prepared).

| Step-96 U(29,26), no swept-geometry correction | Ubuntu Debug | Windows Release |
| --- | ---: | ---: |
| Grouped child→parent cell index | 1692→1693 | 1692→1693 |
| Required child→parent internal volume (m³) | 3.16451948118e-5 | 3.16451967589e-5 |
| Aperture at step START | 0.0791688242854 | 0.0791688242854 |
| Aperture at step END | 0.000108834974526 | 0.00010883367213 |
| 256-point **time-mean aperture** | 0.0398324793615 | 0.0398324850844 |
| Existing **pre-pressure** U velocity (m/s) | 23.1379013062 | 23.1378936768 |
| Constant `u` hypothetically required using time-mean opening (m/s) | 24.4057198924 | 24.4057178876 |
| Required using ENDPOINT opening held for entire step (m/s) | 8932.24203113 | 8932.34947153 |
| Pre-pressure speed × time-mean area **proxy** (m³) | 3.00013028748e-5 | 3.00012972927e-5 |
| Donor volume at step START (m³) | 3.16900877806e-5 | 3.16900877806e-5 |
| Donor volume at step END (m³) | 5.87675394459e-11 | 5.87661447282e-11 |
| Donor stored **after both tracer stages** (kg) | 0 | 0 |
| Bounded **extra post-step** hypothetical transfer (m³) | 5.87675394459e-11 | 5.87661447282e-11 |
| Remaining omitted child volume after this extra transfer (m³) | 3.16451360443e-5 | 3.16451379927e-5 |
| Net recorded pair volume and tracer kg | **0 / 0** | **0 / 0** |

With optional swept-geometry reconciliation ON in the *one-step
matched clone*, required child volume is `3.15640535679e-5 m³`
Ubuntu / `3.15640555106e-5 m³` Windows. The inferred
time-mean constant velocity is `24.3431 m/s`. Geometry is
identical, so time-open aperture and current tiny donor volume
remain essentially the same. The paired hypothetical delta remains
zero. Windows measured a **numerically negligible**
`3.2614e-231 kg` in the corrected clone's donor after transport,
not evidence of a substantial releasable tracer reserve.

**Interpretation and NON-acceptance:**
1. A shrinking sliver's initial volume `3.169e-5 m³` is close to
   the group's missing per-cell continuity `3.1645e-5 m³`.
   It should be accounted for **during** the moving-interface
   substep; it is **not** proof of an additional `3.1645e-5 m³`
   physically still inside the *end* cell. The existing tracer
   remapper has already run, and the donor carries zero (or
   numerically negligible) remaining tracer. A naive extra
   post-step remap would double-count prior transport.
2. At the observed pre-pressure speed, the time-open face proxy is
   `3.00013e-5 m³`, near but smaller than the necessary internal
   exchange. Since **true within-step fluid U(t) is unknown**, do
   NOT call this a proven physical capacity limit or declare
   `24.4 m/s` the physically correct solved speed. In particular
   aperture integrals `∫A(t)dt` and `∫A(t)u(t)dt` are different
   quantities.
3. The endpoint-only interpretation instead demands ~8.9 km/s
   *for the whole-step equivalent exchange*, explaining the severe
   discretization sensitivity without proving the cause of every
   velocity spike or physical mass conservation. The actual
   noncoupled field, grouped oracle and tracer-remap histories are
   separate numerical processes.
4. The pressure pooling still violates pointwise continuity and
   changes its independent fluid impulse proxy. This ledger does
   not calculate an equal/opposite pressure force on a movable
   Body. **Do not enable the pool in ordinary physics or claim
   stable coupled FSI, water, sail, cloth or joints.**

**Next gated work:** measure or solve an explicitly time-consistent
face flux and moving-solid source on the same split substep, preserving
the integration of open area AND velocity, and compare it against
actually applied swept-tracer exchange **before** any new transport
is added. Then test local/region continuity, Body+wall impulse
closure, positivity, closed/reopening pockets, h/substep
convergence and disabled-mode performance. Keep PR #326 Draft
and PR #322 independent, no default physics changes.

### Remaining S2c acceptance gate

1. **Physical acceptance, not just mass conservation:** compare the new endpoint-aperture transfer with the *time-integrated open-face* fluid capacity, moving-solid surface flux, pressure source and local concentration. Identify false evacuation through tiny apertures and unphysical redistribution into inflowing neighbors. Treat total tracer mass as necessary but insufficient.
2. Extend controlled tests to **almost closed faces, fully disconnected pockets, newly reopened faces**, nonlocal fully covered fallback, rotation, h/cadence variation and Body force/trajectory differences. The new experiment handles **partial** donors only. Verify no change to tracer-disabled and default-off paths.
3. On the same actual World8/Fluid4 source (18,13), independently compare baseline/pressure-pool and baseline/connected-tracer alternatives, plus combined alternatives **only after** each independent effect is understood. Compare same-state pressure continuity and separate continuous histories. Never add endpoint MAC kg-equivalent flux to Swept tracer kg and call it a conservative space-time flux.
4. Measure normal-step overhead (feature disabled) and optional observer/remap overhead enabled. Keep **both experimental controls OFF by default**, PR #326 Draft, and PR #322 sampling independent. No solver merge until supported by physical and performance acceptance evidence.

### Step-96 exact-entry substep refinement vs applied tracer face ledger — 2026-11-17

**Scope (read-only benchmark, S2c):** `sandboxWindStepBenchmark
--small-cell-capsule-step96-split-flux-audit` reuses the already
established prescribed rotating single-Capsule test (h=0.125m,
64×64, angle 0.18→1.28 rad over 128 Fluid steps, no coupled Body).
Immediately before its **96th real step**, the current step-95 Field
state is cloned. Independent clones then traverse **exactly the same
geometric angle interval** using 1, 2, 4 or 8 *actually executed*
Fluid substeps (each with its own advection, swept geometry, Cut-cell
pressure projection and applied tracer transport). The 1-substep
unmodified-tracer clone must reproduce the unmodified live step-96
stored endpoint face MAC volume, left pressure source and global tracer
kg within explicit numerical tolerances.

Two completely separate passive-dye controls are evaluated:
(1) actual historical tracer from the 95-step run and
(2) same step-95 fluid/solid/pressure state but a **controlled
1 kg/m³ tracer concentration** assigned only at the shrinking left
cell (28,26) on each copied Field before stepping. No tracer is
injected into the original Field, and neither experiment enables
small-cell Galerkin pressure pooling or connected-residual remap.

For U(29,26), the CSV `capsule_step96_split` reports independently:

- Sum of actual **post-projection stored endpoint MAC** face volume
  `Σ open_end * u_end * h * dt_small`, plus **pre-pressure** endpoint
  volume seen in the real pre-pressure callback.
- Sum of `getSweptULinearVelocityProxyVolume`, explicitly a
  *hypothetical* within-substep velocity interpolation rather than
  the exact integral of `open(t)*u(t)`.
- Sum of per-substep time-mean opening × `dt_small`
  (aperture exposure), and separately the **solid swept-face volume**,
  left/right **pressure volume sources**, left snapshot volume change,
  and accumulated left post-pressure continuity residual.
- Exactly **applied, signed** left↔right tracer kg from the existing
  `setTracerTransferObserver`, separated into sweptFace, fallback,
  MAC and connectedResidual kinds. These kg are neither face volume
  nor a pressure-induced virtual child→parent exchange.
- Global before/after tracer mass, remaining left-cell tracer mass and
  cell fluid volume, plus maximum sampled endpoint raw face speed.
  Source-sum equality and convergence across different substep counts
  are **not assumed**; actual discrete pressure and advection
  histories diverge when the cadence changes.

A cloned 1-substep path is an **identical-entry control**. Its
higher-cadence siblings are a meaningful *discrete refinement
comparison*, **not** samples of an independently known continuous
velocity history inside the original coarse step. Even 8 real
substeps do not certify physical space-time mass/energy conservation,
and the endpoint volume sum and linear proxy must not be silently
added to the already applied Swept tracer kg. Any observed
difference flags a pressure/geometry/advection coupling question,
not an accepted replacement for the current transport solver.

**Verification:** Linux Debug + Windows Release opt-in benchmark
steps are registered in `.github/workflows/cmake.yml`; check the
CI triggered by this change for compiler, parity and numerical
regression before treating the experiment as verified. The ordinary
field/World solver and all default options remain unchanged.
Keep PR #326 Draft and PR #322 separately Draft; next acceptance
requires independent actual time-consistent fluid-face flux with a
moving-interface balance and pressure/Body+wall impulse closure,
then geometry reopening, h/cadence and performance controls.

### Measured step-96 split-refinement results — 2026-11-17

**Code-bearing CI passed:** [GitHub Actions #43936620326](https://github.com/buddas-bible/zonai/actions/runs/43936620326) completed successfully on Ubuntu Debug and Windows Release, including both new `--small-cell-capsule-step96-split-flux-audit` steps, the ordinary CTests, and the existing pressure/Cut-cell audits. This run is on commit `6dd9b973` (the benchmark implementation `94a0b486` plus CI registration); subsequent documentation-only commits do not alter solver or benchmark code.

The following matched comparisons are **separately evolved actual 1/2/4/8-step Fluid histories from one identical step-95 entry**. They are not a reconstruction of a unique within-step velocity of the 1-step baseline. The face is U(29,26), with world cell size 0.125m and one 1/128-th of the prescribed 1/30s total rotation interval.

| Quantity (Ubuntu Debug, original tracer history) | 1 split | 2 splits | 4 splits | 8 splits |
| --- | ---: | ---: | ---: | ---: |
| Maximum raw U(29,26) speed (m/s) | 5486.276 | 2762.238 | 1392.036 | 704.831 |
| Summed **pre-pressure endpoint** MAC face volume (m³) | 8.1973e-8 | 1.52152e-5 | 2.07701e-5 | 2.19025e-5 |
| Summed **post-pressure endpoint** MAC face volume (m³) | 1.94368e-5 | 2.03211e-5 | 2.07561e-5 | 2.09689e-5 |
| Hypothetical velocity-linear face volume proxy (m³) | 3.58077e-3 | 4.78506e-4 | 8.34847e-5 | 3.15554e-5 |
| Time-open face exposure (s) | 1.03982e-5 | 1.03795e-5 | 1.03746e-5 | 1.03734e-5 |
| Summed left pressure-source volume (m³) | -3.17712e-5 | -3.17105e-5 | -3.16949e-5 | -3.16911e-5 |

Windows Release independently reproduced the 1→8 raw velocity change **5486.342→704.840 m/s** and the post-pressure endpoint volume sum **1.943681e-5→2.096891e-5 m³**, with negligible differences in the stable geometry and pressure-source budgets. Both platforms successfully enforced **split=1 same-state parity** against the real original Field (post-pressure face MAC volume, left pressure source, and total tracer mass). Even though max raw velocity changes substantially with finer time integration, the pressure-source and face exposure sums are comparatively steady. This is a discretization diagnostic, not a verified physical fix or a proof of monotonic convergence.

For a separate **controlled dye donor** of 1 kg/m³ in the shrinking left cell immediately before the split interval (copies only), actual applied signed left→right transfers across **this one face** were:

| Measured tracer mass (Ubuntu Debug, kg) | 1 split | 2 splits | 4 splits | 8 splits |
| --- | ---: | ---: | ---: | ---: |
| Applied Swept face transfer | 2.216863e-5 | 1.651754e-5 | 1.117561e-5 | 1.103088e-5 |
| Applied MAC face transfer | 3.59525e-11 | 5.19428e-6 | 1.031080e-5 | 1.050809e-5 |
| Signed fallback / connected-residual on this pair | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| **Swept + MAC direct pair** | ~2.216866e-5 | ~2.171182e-5 | ~2.148641e-5 | ~2.153896e-5 |

Windows Release produced essentially the same numbers (Swept 1→8: **2.216863e-5→1.103088e-5 kg**; MAC: **3.59517e-11→1.050808e-5 kg**). The **global** tracer mass remained equal before/after every tested interval at printed precision (initial 1.72632628537 kg in the controlled-donor condition, including historical dye); the shrinking left cell ended with approximately zero tracer while its geometric fluid volume was ~5.877e-11 m³. Its local dye could have moved through OTHER adjacent faces or fallback paths: direct U(29,26) pair mass **does not close the donor's complete tracer ledger**. Do not equate these kg with the ~3.16e-5 m³ omitted child continuity in the independently pooled pressure oracle.

**Findings / rejection of early physical acceptance:**

1. The available *actual* fine-step MAC and tracer histories show strong sensitivity in the split between Swept remap and MAC advection, even though the combined direct-pair transported dye is of similar order. The current pressure/transport split has not demonstrated a time-consistent single-face conservation identity.
2. The 1-step velocity-linear proxy is **0.00358 m³**, vastly larger than the actual 1-step *endpoint-based* face-volume sum **1.94e-5 m³**. The former is NOT measured material transport: it linearly invents intermediate velocity in the presence of an endpoint spike. By 8 substeps the proxy is ~3.16e-5 m³, but it is still a hypothetical interpolation model.
3. The raw hotspot velocity falls roughly with refinement (5486→705 m/s) while the left pressure-source volume and time-open aperture exposure change comparatively little. This supports **time/pressure discretization sensitivity** as a concrete area for investigation. It does **not** establish a physically correct speed, numerical convergence, admissible implicit transfer, or conservation of water, energy, pressure–Body impulses, or joints.
4. Both platforms pass the diagnostic's validity, stage count and global tracer mass gates. These gates are bookkeeping checks for *this* replay, not a high-resolution, continuous moving-interface solution or GPU/CPU parity.

**Next bounded S2c experiment:** independently close the **entire shrinking donor** tracer inventory by recording the actually applied Swept/MAC/fallback transfers on *all four faces and any nonlocal destinations* at this exact step-95 entry, separately for 1/2/4/8 substeps. Compare that with the donor's snapshot change and pressure-source budget without converting tracer kg into volume using an endpoint-only concentration or double-counting Swept transfers. Only after this full local ledger should a conservative moving-interface discretization be proposed, then test h/cadence, reopening/closing, coupled Body+wall feedback and disabled-mode overhead. Both experimental physics options stay OFF; PR #326 and #322 remain Draft.

### Step-96 full four-face donor tracer inventory closure — 2026-11-18

**Implementation:** `ad1af6e5` adds W/E/S/N + nonlocal **actual applied**
per-kind tracer-transfer bookkeeping to the *existing*
`--small-cell-capsule-step96-split-flux-audit`; `8ff51d4e`
fixes the benchmark's CSV header literal. **Cross-platform validation:**
[Actions #44148962605](https://github.com/buddas-bible/zonai/actions/runs/44148962605)
completed **success** on both **Ubuntu Debug** and **Windows Release**,
including ordinary CTests, prior S2c audits, and the new donor-closure
assertions. PR #326 remains Draft; PR #322 remains independent Draft.

**Identical-entry controls:** Step-95 Field snapshots, the exact same
step-96 physical Capsule rotation and geometry, but 1/2/4/8 genuinely
computed Fluid substeps. Both the existing tracer history and an
additional *cloned-only* 1 kg/m³ controlled donor tracer at
cell **(28,26)** are checked. No physical field, pressure equation,
World feedback or transport default changes. Source/destination cell
indices are reconstructed from the **already applied**
`fluidTracerTransfer2` events, never inferred from potential MAC or
Swept capacities. Event direction is relative to the donor cell:
W/E/S/N, with `X` for nonadjacent fallback, separately tallying
outgoing and incoming. Swept, fallback, limited MAC and the disabled
connected-residual experiment have distinct channels. The audit checks
the following on **every substep**, including after Swept remap and
after MAC transport:

```text
afterSwept = beforeSubstep - observedSweptNetOut
                          - observedFallbackNetOut
                          - observedConnectedNetOut
afterMac = afterSwept - observedMacNetOut
afterEntireInterval = beforeInterval + Σ(applied incoming kg) - Σ(applied outgoing kg)
```

On both platforms, for the controlled donor, the initial stored mass
was **3.16900877806e-5 kg** and the final donor mass was approximately
zero for all four cadence variants. The **maximum** absolute
substep/stage inventory residual on the 8-split run was approximately
**3.39e-21 kg** (Ubuntu and Windows), far below its explicit
`1e-12 + 1e-6 * initialDonorMass` kg regression tolerance.
The existing 1-split cloned path independently matches the actual
unmodified Field. The unseeded historical tracer control begins with
only subnormal rounding-level donor mass; it does not supply evidence
for a meaningful donor leak.

| Applied controlled-donor transport (Ubuntu Debug, kg) | 1 split | 2 splits | 4 splits | 8 splits |
| --- | ---: | ---: | ---: | ---: |
| Swept **East** | 2.2168626982e-5 | 1.65175436491e-5 | 1.11756104252e-5 | 1.10308792646e-5 |
| Swept **South** | 9.52140203104e-6 | 7.18722248664e-6 | 4.88571413661e-6 | 4.81820594219e-6 |
| Actual limited MAC **East** | 3.59525199076e-11 | 5.19428034257e-6 | 1.03108034342e-5 | 1.05080856409e-5 |
| Actual limited MAC **South** | 2.28150195384e-11 | 2.79104130224e-6 | 5.31795978458e-6 | 5.33291693297e-6 |
| Applied West / North, all kinds | 0 | 0 | 0 | 0 |
| Applied adjacent/nonlocal fallback, all directions | 0 | 0 | 0 | 0 |
| Applied connected-residual experiment | 0 (OFF) | 0 | 0 | 0 |

The combined **East + South Swept/MAC transfer** accounts for all
**3.16900877806e-5 kg** of the original controlled donor mass; the
remaining ~1e-22–1e-37 kg signed incoming differences are numerical
roundoff and not meaningful material recycling. Windows Release
agrees to displayed precision. The earlier one-face
U(29,26) audit excluded the real **South** transport; it was not
missing tracer or a physical volume created by the pressure-pooling
oracle. The actual operator split changes substantially as time steps
are refined: at 1 split nearly all dye is moved by **Swept**; by 8
splits Swept and MAC account for approximately half each, with
the same final donor mass. This is strong evidence of *discrete
operator/cadence sensitivity*, not proof of a physically correct flux.

**Do not equate the closed dye ledger to fluid volume/impulse
conservation.** In the 1-split control the measured signed
solid-Swept volumes on donor W/E/S/N were approximately
`[-6.0961413e-4, +5.8269914e-4, +2.5026867e-4,
-2.5512485e-4] m³`, whereas applied dye crosses
**only East and South**. Their signed geometric cancellation gives
~`-3.1771e-5 m³`, comparable to that step's pressure-source
volume. Neither the large *individual* solid-swept face volumes nor
post-pressure endpoint MAC volumes are time-integrated physical
open-fluid tracer transfers. Mass closure of the tracer bookkeeping
does not validate the pressure operator, group-only Galerkin
continuity, physical fluid inventory, or a Body+wall impulse pair.

**Updated S2c next gate:** derive a time-consistent moving-interface
*geometric conservation* and open-face **fluid** volume budget,
including measured/independently resolved temporal face velocity,
without duplicating the already applied Swept/tracer flux. Test it
at the same step-95 entry against the donor's W/E/S/N
pre-/post-pressure MAC volumes, signed swept geometry source, and
local continuity. Then expand to nearly closed/reopened pockets,
h/cadence sensitivity, and genuinely two-way World + wall impulse
closure. Do not enable pressure pooling, change default transport,
or claim fluid/FSI acceptance based solely on this tracer test.

### Step-96 independent endpoint geometry GCL and actual-substep face quadrature — 2026-11-18

**Bounded, read-only S2c experiment:** commit `28532ffc` extends the
previously validated `--small-cell-capsule-step96-split-flux-audit`.
No solver, transport, Body feedback, pressure-pooling default or
aperture-sampler PR is changed. Starting from the *identical* step-95
Field, use the same prescribed step-96 Capsule rotation at real
1/2/4/8 Fluid substeps. On each copied Field, compare the *same*
donor (28,26) and its four faces (W/E/S/N). The original historical
tracer and controlled-donor variants share the same geometrical and
MAC history; their separate applied transfer ledgers continue to
enforce the prior tracer conservation gates.

Independently re-evaluate the **existing analytic Capsule area code**
at actual starting/ending obstacle poses with
`getAnalyticCapsuleCellAreaAudit`. This is independent of the *stored
Field fraction*, **not** an independent geometry implementation.
Compare that endpoint area difference against the telescoping stored
occupancy change and the already-applied moving-solid Swept volumes,
all in unit-depth m³:

```text
dV_analytic = h^2 * (analyticFraction_end - analyticFraction_start)
dV_stored   = Σ_substeps h^2 * (storedFraction_end - storedFraction_start)
V_swept     = Σ_substeps Σ_W,E,S,N (signed solidSweptOutwardVolume)
GCL_defect  = dV_stored - V_swept
```

Read the *actual stored* MAC velocity and endpoint open fraction
at **each** substep entry and exit (the entry is the previous
evolved Field state). Alongside the pre-pressure and post-projection
MAC snapshot volumes, compare a physically dimensioned **trapezoid
proxy** formed from those **actual** endpoints:

```text
V_mac_start = Σ_substeps Σ_faces h*dt_i*(open_start * velocity_start)_out
V_mac_end   = Σ_substeps Σ_faces h*dt_i*(open_end   * velocity_end)_out
V_mac_trap  = 1/2 * (V_mac_start + V_mac_end)
P_residual  = V_mac_end + Σ_substeps pressureSourceDeltaVolume
```

The arithmetic trapezoid is **not** the true integral
`∫open(t)*u(t)*h dt` of the original one-step velocity field.
Advection, pressure projection and moving boundaries create different
intra-step paths at each cadence; only endpoint velocities are
observed. Do not identify this proxy with water mass, original-step
advection, or already applied swept/tracer transfers. The pre-pressure
endpoint volume (`donorPreMacOutwardVolume`) is also separately
recorded to reveal how advection/boundaries and pressure change the
face field within the substep.

**Cross-platform validation:** [workflow #44246966255](https://github.com/buddas-bible/zonai/actions/runs/44246966255)
completed **success on Ubuntu Debug and Windows Release**, including
the extended step-96 GCL/face-quadrature benchmark and prior regressions.
Windows Release independently measured a 1→8-split GCL defect of
**8.11383998184e-8 → 1.09195212594e-9 m³**, endpoint-trapezoid
proxy of **6.27358065645e-5 → 3.72981393595e-5 m³**, and
post-pressure discrete residual of **-1.97535535834e-12 →
5.3736811554e-9 m³**. Differences from the Ubuntu values below
are floating-point/platform noise, not a different physical conclusion.

| Actual step-95 entry / prescribed step-96 donor (28,26), m³ unless noted | 1 split | 2 splits | 4 splits | 8 splits |
| --- | ---: | ---: | ---: | ---: |
| Analytic endpoint `dV` | -3.16900281243e-5 | -3.16900281243e-5 | -3.16900281243e-5 | -3.16900281243e-5 |
| Stored telescoping `dV` | -3.16900290130e-5 | -3.16900290130e-5 | -3.16900290130e-5 | -3.16900290130e-5 |
| Summed solid Swept divergence | -3.17711674143e-5 | -3.17105441354e-5 | -3.16949299304e-5 | -3.16911209666e-5 |
| **GCL defect = stored - Swept** | **8.11384012131e-8** | **2.05151223717e-8** | **4.90091733168e-9** | **1.09195352066e-9** |
| Summed pressure source | -3.17711674143e-5 | -3.17105441354e-5 | -3.16949299304e-5 | -3.16911209666e-5 |
| Starting-state MAC endpoint quadrature | 9.37004514461e-5 | 7.05682398326e-5 | 5.31119104994e-5 | 4.28997868122e-5 |
| Pre-pressure MAC endpoint quadrature | 1.24958062097e-7 | 2.33356258099e-5 | 3.17095227470e-5 | 3.32186244581e-5 |
| Post-pressure MAC endpoint quadrature | 3.17711653192e-5 | 3.17139294361e-5 | 3.17004999999e-5 | 3.16964968331e-5 |
| **Actual-endpoint trapezoid proxy** | **6.27358083826e-5** | **5.11410846344e-5** | **4.24062052497e-5** | **3.72981418227e-5** |
| Post MAC - trapezoid proxy | -3.09646430634e-5 | -1.94271551982e-5 | -1.07057052498e-5 | -5.60164498959e-6 |
| Sum of discrete post-pressure residual volumes | -2.09504133741e-12 | 3.38530071518e-9 | 5.57006950034e-9 | 5.37586649596e-9 |
| Max absolute **per-substep** GCL defect | 8.11384012131e-8 | 1.06507428363e-8 | 1.47656464833e-9 | 2.85581336357e-10 |

The maximum analytic-vs-stored fluid fraction discrepancy was
**5.68828317782e-11**. The equal endpoint occupancy across the four
splits and very small recomputed-area discrepancy distinguish the
time-integrated swept-geometry mismatch from an evident endpoint
occupancy discrepancy in this specific witness. In this mode, the
pressure source equals the signed Swept divergence (to printed
precision), but differs from the exact endpoint occupancy change;
therefore a small pressure residual proves only the chosen
**discrete** operator relation, not geometric conservation.

The total **GCL defect** falls ~74× from 1→8 splits, while the
absolute gap between the post-projection endpoint quadrature and
the independent evolved-endpoint trapezoid proxy is still
**5.60e-6 m³** at 8 splits — more than three orders of magnitude
larger than the donor's GCL defect. Also note that the pressure
residual is **not** monotonically decreasing with time refinement.
Do **not** infer a converged physical space-time face flux or repair
the pressure solver by replacing its source with the trapezoid proxy.
Pressure, geometry and transport remain separate finite-step
operators; the difference is a falsifiable diagnostic, not proof
of which approximate quadrature equals physical advection.

**Next gate (still open):** reconstruct an independently consistent
moving-interface finite-volume flux balance for **all four** fluid
faces with a time-resolved \(A(t)u(t)\) and the moving-solid interface
source, plus local/region pressure continuity and equal/opposite
Body+wall momentum. Then test nearly closed/reopened pockets,
h/cadence and performance. Compare to the existing *actual* applied
Swept/MAC tracer kg without double-counting them. Keep both
pressure pooling (#326) and independent aperture sampler (#322)
Draft and default OFF; no physical-acceptance claim.

### Step-96 temporal aperture convergence and face-velocity non-uniqueness — 2026-11-18

**Bounded reference on PR #326 (still Draft):**
`tests/sandbox/wind_step_benchmark.cpp` (code commits
`264bee68`, `898f92a7`, `798753cb`) extends the same
`--small-cell-capsule-step96-split-flux-audit`. The experiment
reuses the **same step-95 entry field**, the **same prescribed Capsule
rotation**, and **1/2/4/8 genuinely stepped** copies. It is opt-in
read-only instrumentation; the original Fluid/World solver, pressure,
body feedback, and default tracer transport remain unchanged.
The prior 4-face donor tracer and GCL ledgers are retained.

**What the new geometry reference really measures:** For each donor
(28,26) face W/E/S/N and each actual Fluid substep, reconstruct the
rigid Capsule pose at 64 and 256 midpoint times and query the existing
analytic face clipper via
`getAnalyticCapsuleFaceOpenFractionForAudit`. Sum the independent
geometric open exposures (`seconds`) and then calculate alternative
hypothetical velocities against the **same** sampled aperture:

```text
alpha = (sample + 0.5)/samples in each actually stepped time interval
A(alpha) = analytic open fraction at the reconstructed Capsule pose
u0 = signed MAC velocity captured at Fluid substep entry
u1 = signed MAC velocity captured after its pressure projection
V_model = h * dt * midpointMean[ A(alpha) * u_model(alpha) ]
velocity models (same endpoints u0 and u1):
  u_linear(alpha) = u0 + alpha*(u1-u0)
  u_early(alpha)  = u0 + (2*alpha-alpha^2)*(u1-u0)
  u_late(alpha)   = u0 + alpha^2*(u1-u0)
```

The 64↔256 comparison tests **geometric aperture quadrature
sensitivity** *under an assumed linear face-velocity path*,
not the convergence of the actual unknown `u(t)`.
The same 256-point open geometry with *early* and *late* endpoint-
matched velocities supplies an illustrative **non-uniqueness range**:
those are all compatible with measured endpoints but none is asserted
to be the physical motion. Separately record the solver's existing
sampled `sweptU/VTimeMeanOpen`, current
`getSweptU/VLinearVelocityProxyVolume`, and the actual post-pressure
**endpoint** MAC volume budget for each face. The latter remains a
discrete operator output, not a measured time-integrated physical
flux. The outgoing sign on W and S is negative; E and N positive.
Different tracer-seeding copies must share the same geometry/flow
outputs; dye kg is **never** converted into m³ to manufacture closure.

**Read-only CSV streams:**
`capsule_step96_time_face` records per-face 64/256 aperture
exposure, 64/256 linear-velocity proxy, 256 early/late proxies,
internal sampled geometric exposure, internal linear proxy,
endpoint MAC volume and signed solid Swept volume;
`capsule_step96_time_models` sums the modeled net outward volumes
and explicitly reports the endpoint-vs-model and early-vs-late
differences. Existing `capsule_step96_gcl`,
`capsule_step96_gcl_face` and `capsule_step96_donor_balance`
remain the independently checkable geometric/discrete-pressure/
applied-tracer witnesses.

**Acceptance boundary:** These tests may support the accuracy of
time-integrated *aperture geometry* or disprove that endpoints
determine a unique time-integrated flow, but they do **not** provide
a measured true `integral A(t)*u(t) dt`. An accepted moving-interface
volume/pressure/impulse discretization still needs a temporally
consistent fluid flux history (or explicit in-step time-integral
operator) and equal/opposite Body+wall coupling. Do not replace
the pressure source or transport with any of these models, or
turn on small-cell pooling. Subsequent targeted development should
sample *actual evolved intermediate fluid velocity* at a fixed
matching geometry history and verify closed/reopened pockets,
h/cadence, 4-face local and region volume, plus joint/Body feedback.

**Relevant numerical formulation references** (motivation and
acceptance criteria, not proof that zonai implements them):
- [Integrated Space-Time Finite Volume Method and Moving Boundary
  Problems](https://www.sciencedirect.com/science/article/pii/S0021999199963246):
  conservation over space-time control volumes and GCL.
- [Improved mass conservation and reduced spurious pressure oscillations
  near moving sharp interfaces](https://www.sciencedirect.com/science/article/pii/S0021999111003524):
  GCL violation as a source of pressure artifacts.
- [Conservative and consistent implicit Cartesian cut-cell method for
  moving geometries](https://www.sciencedirect.com/science/article/pii/S0021999122001863):
  pressure/velocity coupling and local conservation in moving cut cells.

#### Measured temporal aperture quadrature — Linux Debug / Windows Release

**Validated implementation CI:** [Actions #44256868806](https://github.com/buddas-bible/zonai/actions/runs/44256868806)
**Ubuntu Debug success (35/35 job steps) and Windows Release success
(65/65 job steps)**. The benchmark, ordinary tests and previous Cut-cell
regressions passed. First attempts `264bee68` and `898f92a7`
had test-source compile errors (local shadowing and duplicate face
coordinates); final compiled code is `798753cb`. The following data
are from that **successful** run, not those failed attempts.

| Actual step-95 entry; step-96 donor four-face model (Ubuntu Debug, m³) | 1 split | 2 splits | 4 splits | 8 splits |
| --- | ---: | ---: | ---: | ---: |
| Existing internal aperture-moment **linear proxy** | 5.87313793075e-3 | 7.80767666652e-4 | 1.33638810928e-4 | 4.86594697907e-5 |
| Independent 64-time-midpoint geometric **linear proxy** | 3.93685850367e-3 | 5.37598185352e-4 | 1.03233522727e-4 | 4.48725766939e-5 |
| Independent 256-time-midpoint geometric **linear proxy** | 3.93642408811e-3 | 5.37543342808e-4 | 1.03226250660e-4 | 4.48718584830e-5 |
| 256-sample **early** changing-velocity hypothesis | 5.87706827116e-3 | 7.81991266400e-4 | 1.33602403614e-4 | 4.83957691946e-5 |
| 256-sample **late** changing-velocity hypothesis | 1.99577990505e-3 | 2.93095419216e-4 | 7.28500977067e-5 | 4.13479477714e-5 |
| Early-late hypothesis difference | 3.88128836611e-3 | 4.88895847185e-4 | 6.07523059068e-5 | 7.04782142317e-6 |
| Actual solver **post-pressure endpoint** MAC net (not time integral) | 3.17711653192e-5 | 3.17139294361e-5 | 3.17004999999e-5 | 3.16964968331e-5 |

Windows Release agrees to floating-point/platform precision: at
1 split the independent 64/256 linear proxies were
`3.93690575629e-3 / 3.93647199454e-3 m³`,
the internal proxy `5.87320802967e-3 m³`, and the early/late
spread `3.88133558103e-3 m³`. At 8 splits Windows reported
`4.48720273430e-5 m³` for independent 256, versus
`4.86596128581e-5 m³` internally. Controlled-donor tracer and
unseeded history have the same geometric/velocity model outputs
at each cadence, as expected; the dye seed changes only passive kg.

**Specific finding: the FIRST time moment of aperture is not resolved
merely because its mean appears accurate.** On the 1-split East face,
the internal one-step open exposure was
`1.03981737993e-5 s`, versus the independent 256-point
`1.03730420414e-5 s` (about **0.24%**). On South it was
`6.66627011014e-6 s` vs `6.67102022991e-6 s`
(about **0.071%**). Yet the corresponding internal **linear-time
velocity proxies** are `3.58076510621e-3 m³` (East) and
`2.29237282454e-3 m³` (South), compared with independent
256-point linear proxies of `2.40059686251e-3` and
`1.53582722559e-3 m³` respectively. Summed, the existing
internal proxy is about **49% larger** than the independently
sampled **same linear velocity hypothesis**, even though the
64→256 independent total changes by only about **0.011%**.
This separates inadequate **time weighting of aperture** from
uncertainty in the real fluid velocity history.

**Root-cause constraint from the actual code path:** In
`fluidField2::updateSweptSolidFlux` the default
`timeSamples = clamp(ceil(maximumTravel / (0.25*h)), 1, 32)`;
`adaptiveSweptSamplingEnabled_` is false in this control. The
prescribed rotating Capsule moves approximately
`0.00859 rad` per original step, with a maximal traveled distance
about `0.007 m`, below `0.25*h = 0.03125 m`. Consequently
the accepted swept/aperture quadrature is **one temporal midpoint
per actual Fluid substep**. A single midpoint can approximate the
zeroth opening moment yet still misrepresent
`∫alpha*A(alpha)dalpha`, which the internal linear-velocity
proxy uses. This is an identified **diagnostic quadrature limitation**,
not proof that physically correct pressure requires 64 or 256
samples everywhere. Do not globally raise normal-step samples
without a disabled-path performance and conservation assessment.

At 8 actual smaller fluid substeps the independent 256-point
linear proxy is **4.48718584830e-5 m³**, the internal proxy
**4.86594697907e-5 m³**, and the post-pressure endpoint
MAC operator **3.16964968331e-5 m³**. The early/late
interpolants **still differ by 7.04782142317e-6 m³** despite
the converged aperture quadrature. This is a structural
**velocity-history underdetermination**: improving the aperture
integral alone does **not** reveal the physical moving-boundary
volume flux or establish pressure/FSI conservation.

**Next bounded implementation gate:** instrument **actual**
substep-resolved, post-advection/pre-pressure and post-projection
MAC velocities on W/E/S/N at matched geometric time points, or
design the pressure/transport operator to consume and conserve
a common space-time face-volume integral. Use the above 64↔256
aperture reference to isolate pure geometric weighting, then
compare independently to the `dV_stored` and four signed
solid-Swept fluxes and actual applied tracer kg. Test a near-open,
near-closed and reopening cut cell, region closure and paired
Body+wall feedback before any solver change/merger.

### Step-96 observed MAC pressure-stage velocity and per-substep source/flux closure — 2026-11-18

**Verified prior stage witness:** commit `627139d1` extends only the
opt-in `--small-cell-capsule-step96-split-flux-audit`. GitHub Actions
[run #44268890599](https://github.com/buddas-bible/zonai/actions/runs/44268890599)
finished **success on Ubuntu Debug and Windows Release**. It records
each genuinely evolved Fluid substep's MAC W/E/S/N face opening and signed
normal velocity at (1) the previous completed-step entry, (2) the actual
post-advection/pre-pressure snapshot, and (3) after pressure projection.
For each face the stage-volume entries are `h*dt*opening*velocity` with
outward W/S signs, **not** a measured continuous-time flux integral.

For the same step-95 entry and prescribed rotating Capsule step 96,
the donor (28,26) has the following **summed per-substep endpoint**
volume budgets from the successful **Ubuntu Debug** run (unit-depth m³
except last column). These are sums across the actual 1/2/4/8 stepped
intervals, *not* one synthetic step evaluated with alternate cadence.

| Actual Fluid splits | Pre-pressure MAC outward | Post-pressure MAC outward | Applied pressure face-volume delta | Post-pressure discrete residual | Stored-minus-Swept GCL defect | Last-substep East post-pressure raw velocity (m/s) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| 1 | 1.24958062097e-7 | 3.17711653192e-5 | +3.16462072571e-5 | -2.09504133741e-12 | +8.11384012131e-8 | 5486.27636719 |
| 2 | 2.33356258099e-5 | 3.17139294361e-5 | +8.3783036262e-6 | +3.38530071518e-9 | +2.05151223717e-8 | 2762.23779297 |
| 4 | 3.17095227470e-5 | 3.17004999999e-5 | -9.0227471e-9 | +5.57006950034e-9 | +4.90091733168e-9 | 1392.03552246 |
| 8 | 3.32186244581e-5 | 3.16964968331e-5 | -1.522127625e-6 | +5.37586649596e-9 | +1.09195352066e-9 | 704.831115723 |

On the final substep for each cadence, **both East and South face raw
velocities remain around 19–24 m/s before projection but increase sharply
after projection**. The final East open fraction is about
`1.08835e-4` and South about `6.91161e-5` on Ubuntu; opening is
unchanged by the pressure stage. The same face's high raw velocity
therefore corresponds to a far smaller **open-weighted throughput** than
the unweighted m/s reading suggests. Windows Release independently
reproduces the same stage pattern and near-identical values.

**Interpretation bound:** `projectCutCell` assembles its face flux
using the current `cutCellUOpen/VOpen` and the moving-volume source
`cutCellVolumeRate` prepared from existing geometry/Swept history.
For one unsplit step, the pressure-stage correction must replace an
almost-zero pre-pressure net (`1.25e-7 m³`) with approximately
`3.177e-5 m³` post-pressure outward flux to balance that existing
source. It solves the *discrete* row to about `2.1e-12 m³`, while the
independent endpoint snapshot and applied Swept geometry disagree by
`8.11e-8 m³` (about 0.26% of the source magnitude). Thus **the
measured GCL discrepancy alone is far smaller than the face-volume
correction required by the model**. At eight smaller steps GCL improves
substantially but the final nearly closed-face raw speed remains
`~705 m/s`. This supports a mixed-time source/end-face-area numerical
conditioning hypothesis, **not** a proof that either endpoint flux or
an assumed time-interpolated velocity is the true physical flux.

**New bounded ledger:** commit `a0b85df5` records
`capsule_step96_pressure_step` separately for every actual substep and
both tracer-seeding controls. For each donor:
```text
required_pressure_correction = -(pre_pressure_mac_net + pressure_source)
applied_pressure_correction = post_pressure_mac_net - pre_pressure_mac_net
applied - required = post_pressure_mac_net + pressure_source
GCL_defect = stored_snapshot_delta - solid_swept_divergence
snapshot_residual = post_pressure_mac_net + stored_snapshot_delta
```
It checks the pressure identities **per step** and again summed across
all splits, explicitly retaining the independent GCL difference.
This is a *diagnostic* identity of the existing solver; it does not
claim physical conservation, measure intra-step `u(t)`, silently move
tracer volume, alter a pressure source, or enable either experimental
pressure pooling or aperture-aware sampling. The new per-step ledger
requires successful CI/replay inspection before it is treated as verified.

**Next acceptance gate:** compare the new per-step ledger between
unseeded and controlled-dye copies, and between Ubuntu and Windows,
particularly the last closing substep; then repeat for reopening and
near-open faces, establish a common space-time face-volume flux with
moving-interface GCL, and close corresponding region and Body/wall
momentum budgets. Do not use reduced raw velocity alone to merge #326.

### Step-96 matched closing-to-reopening Cut-cell reference — 2026-11-18

**Opt-in test-only implementation:** commit [0f107596](https://github.com/buddas-bible/zonai/commit/0f1075969995f032157124e764ea634e0e447db6)
extends the existing `sandboxWindStepBenchmark --small-cell-capsule-step96-split-flux-audit`
with `auditCapsuleStep96Reopening`. **CI**:
[Actions #44326324331](https://github.com/buddas-bible/zonai/actions/runs/44326324331)
completed **success** on Ubuntu Debug (35/35 successful job steps) and
Windows Release (65/65). No production solver, pressure source, default
tracer model, obstacle feedback, or aperture sampler was modified.
PR #326 remains experimental Draft; PR #322 stays separate and Draft.

#### Identical entry-state experimental control

1. Deep-copy the **actual** fully evolved step-95 Fluid state and, in the
   dye control only, seed donor (28,26) with 1 kg/m³ concentration.
2. Advance the clone **one real forward** step with the rotating Capsule
   to the same near-closed step-96 endpoint at every reverse cadence.
   Only after that step is complete install the diagnostic observers.
3. Reverse the prescribed Capsule **pose sequence and angular velocity**
   from original step 96 toward step 95 using 1, 2, 4 or 8 **real**
   `fluidField2::step` calls. This is **not** a mathematical time reversal
   of the dissipative fluid integrator: each reverse trial has the
   identical initial closing-state Field but subsequently follows its
   own evolved pressure/advective path.
4. At **each actual reverse substep**, record donor occupancy,
   stored fluid-volume change, signed solid Swept divergence, pressure
   source, true observed pre-/post-projection MAC net and the corresponding
   required/applied pressure correction and residual. Independently
   inspect W/E/S/N opening, signed MAC velocity and face volume at the
   actual start, pre-pressure and post-pressure stages. The new
   `capsule_step96_reopen_step` and `capsule_step96_reopen_face`
   streams are read-only and do **not** claim a continuous
   `∫ A(t)u(t)dt` measurement.
5. Observe **actual applied** donor-local tracer transfer events and
   verify `m_after = m_before + Σ(transfers_in - transfers_out)`, plus
   total tracer mass and finite obstacle feedback. Also report the
   *fixed 3×3 active-cell patch* pre-/post-pressure MAC volumes and
   post-pressure source: that patch is **not** an independently closed
   Neumann component and its summed residual is diagnostic only.

#### Observed reverse-motion numerical behaviour

The donor starts near vanishing at
`fraction ≈ 3.76103×10⁻⁹` (Windows; Linux
`3.76112×10⁻⁹`) and reopens to **0.00202816562** under
all four actual reverse cadences. The same final geometry but different
Fluid substep histories is a deliberate control.

| Reverse Fluid splits | Maximum donor pointwise pressure-volume residual, Windows Release (m³) | Maximum absolute *per-substep* GCL defect, Windows Release (m³) | Ubuntu Debug maximum donor pressure-volume residual (m³) |
| --- | ---: | ---: | ---: |
| 1 | 1.66257009175e-6 | 8.11378413346e-8 | 1.66240780106e-6 |
| 2 | 1.20469357553e-5 | 1.06510924525e-8 | 1.20469833564e-5 |
| 4 | 3.88068342748e-6 | 1.47656464833e-9 | 3.88058874888e-6 |
| 8 | 5.16728988503e-7 | 2.85581336357e-10 | 5.16712741783e-7 |

The **GCL defect decreases under refinement**, but the *pointwise*
post-pressure donor continuity residual can be orders of magnitude
larger than GCL error on reverse opening, and it **does not improve
monotonically** from 1→2→4→8 substeps. The **2-split**
reopening exhibits the largest donor pressure residual in this
witness. Do not infer that increasing only the quadrature of geometric
apertures, or using more substeps blindly, resolves the pressure row
or establishes full-space-time fluid conservation.

The actual donor transfer-event ledger agrees with donor mass to
floating-point precision (the maximum observed Windows donor mass
defect is under `7e-21 kg` in the unseeded copy), and the global
passive tracer mass gate passes. All **15 paired substep**
pressure/geometry records and **60 paired face** observations between
seeded and unseeded copies are exactly equal on both platforms.
Differences in tracer *mass* columns are intentional; do not compare
them as pressure/geometry observables.

**What is not established:** this control exercises the *noncoupled
prescribed-kinematic Capsule* with 120 pressure iterations, not a
dynamic bidirectionally coupled World/Body with equal-and-opposite
momentum closure. The large reverse pressure residual is measured
with the existing solver and may contain **incomplete iterative
convergence**, **Neumann connected-component compatibility/gauge
handling**, and/or **source/end-face geometry discretization**
effects. The current local and 3×3 patch ledgers do not attribute
that residual to a single mechanism. The pre- and post-pressure
observations are stage endpoints rather than the physical
intra-step face flux history.

**Next bounded S2c gate:** freeze the same first and last *reverse
opening* pre-pressure snapshots without changing the geometry/velocity,
then compare identical-state 120/240/480 (or otherwise bounded)
Jacobi pressure replay, per-connected-component source compatibility,
the donor/region residual and Wall/Body pressure impulse ledgers.
Keep the standard path unchanged, avoid pressure/velocity caps, and
test genuine two-way World coupling only after the kinematic
reopening witness is explained.

### S2c reopening frozen Jacobi budgets and all-region source reconstruction — 2026-11-18

**Verified test-only work:** [6bef7884](https://github.com/buddas-bible/zonai/commit/6bef7884fa9d50b8dc25364728e01f6709853d11) adds the same-state replay comparison, [2e209b6c](https://github.com/buddas-bible/zonai/commit/2e209b6ce1d8090541d6ee86d94667094a5c5ffc) corrects the legacy pressure-residual unit label, and [95353e0a](https://github.com/buddas-bible/zonai/commit/95353e0a8f6f91d7c523bd17404a5a84e05d7e6d) reconstructs *all* open-face-connected components rather than only the donor's component. The successive actual code CI runs [#44330584260](https://github.com/buddas-bible/zonai/actions/runs/44330584260) and [#44333657485](https://github.com/buddas-bible/zonai/actions/runs/44333657485) both finished **success on Ubuntu Debug (35/35) and Windows Release (65/65)**. All these changes are in the opt-in `--small-cell-capsule-step96-split-flux-audit`; default fluid physics and PR #322 remain unchanged.

#### Identical frozen pressure system

At the actual **first and last reverse-opening substeps** of each 1/2/4/8-split trial (deduplicating the 1-split case), the pre-projection Field is deep-copied and its actual MAC velocities, geometry and obstacles preserved. On independent copies, invoke the existing `replayFrozenCutCellPressure(120/240/480)`; do **not** manufacture a zero-MAC state or use a different geometry. The 120-iteration replay must match the live 120-iteration donor volume residual to `1e-9 m³`. Across budgets, assert identical donor pressure source and number of connected regions.

The production Neumann RHS is constructed from float expressions of current-open-fraction MAC flux plus moving-volume source, then averaged over each connected region. Independently reconstruct each active component by traversing only strictly positive-aperture faces. For each region, report the actual pre-projection MAC volume plus post-replay source, the corresponding mean per cell, the sum of post-pressure residuals and the mean-removed regional RMS. Record the component's representative cell ID and whether it contains the target donor. Reconstruct the *whole-domain* compatibility rate as `abs(unresolvedVolumeRate) + Σ abs(component raw RHS volume / dt)` and compare to the solver's `compatibilityCorrection`; keep their difference visible **without forcing equality** because the two assembly paths are not yet mathematically demonstrated equivalent.

The run emits three opt-in CSV families:
`capsule_step96_reopen_frozen`, `capsule_step96_reopen_region`, and `capsule_step96_reopen_compat`. The `pressureResidual` value in the legacy noncoupled Jacobi diagnostics is in its existing native units, **not** asserted to be `m³/s`.

#### Same-state Jacobi response (donor absolute pointwise volume residual, m³)

| Reverse checkpoint | 120 iters Ubuntu | 240 iters Ubuntu | 480 iters Ubuntu | 120 iters Windows | 480 iters Windows |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 split, first/last | 1.66240780106e-6 | 8.12970276265e-7 | 3.75538355307e-7 | 1.66257009175e-6 | 3.75575728057e-7 |
| 2 splits, first | 1.20469833564e-5 | 5.59880293112e-6 | 2.57446235647e-6 | 1.20469357553e-5 | 2.57451078381e-6 |
| 2 splits, last | 1.10313116799e-5 | 6.99753968267e-6 | 4.01310841289e-6 | 1.10311620794e-5 | 4.01298136800e-6 |
| 8 splits, last | 5.16712741783e-7 | 4.55392691371e-7 | 3.94888692419e-7 | 5.16728988503e-7 | 3.94899571532e-7 |

The corresponding donor component contains **4,077 active cells** in every recorded checkpoint; the number of pressure regions in the entire field varies between 4 and 7 as the geometry changes, but stays exactly the same across 120/240/480 replays of any given snapshot. The donor component's uncorrected RHS mean is on the order of `10⁻¹⁴ m³/cell` or smaller, far below its `10⁻⁵–10⁻⁷ m³` pointwise residual. Increasing Jacobi iterations reduces that residual, establishing an **iteration-sensitivity/slow-convergence contribution** at these fixed states; 480 iterations do not demonstrate convergence.

At **2-split first reopening**, the independently reconstructed donor-region source sum is about `-2.18e-11 m³`. The post-pressure sum agrees with it within roughly `1e-17 m³`, despite the substantial individual-cell residual. That distinguishes a spatially distributed residual from a newly introduced whole-region volume leak in this diagnostic reconstruction.

#### Global compatibility difference is NOT sourced by the detached 1-cell pockets

| First reverse substep | Solver compatibility rate, Ubuntu (m³/s) | Independent double volume reconstruction, Ubuntu (m³/s) | Solver compatibility rate, Windows (m³/s) | Independent reconstruction, Windows (m³/s) |
| --- | ---: | ---: | ---: | ---: |
| 1 split | 9.62059857557e-7 | 3.63215790184e-7 | 8.27734766062e-7 | 3.63215773143e-7 |
| 2 splits | 3.07760346914e-5 | 1.67638201232e-7 | 2.79350133496e-5 | 1.67638000573e-7 |
| 4 splits | 1.71406900336e-5 | 2.79398617198e-8 | 2.17695742322e-5 | 2.79404474166e-8 |
| 8 splits | 1.43019060488e-5 | 6.50846530199e-15 | 1.43183860928e-5 | 4.73107856322e-14 |

All reported components and reconstructed components have identical counts on each corresponding platform/cadence. For the **2-split first** opening there are precisely one large 4,077-cell donor component and three detached **single-cell** components; the latter each have independently reconstructed raw RHS **zero**. The recorded `unresolvedVolumeRate` is zero in both platforms. Thus the difference between the float-assembled *reported* compatibility rate and the double *independently reconstructed* compatibility rate **cannot be attributed to those zero-RHS detached components or the unresolved-source term**.

**Hypothesis to verify, not an accepted fix:** `projectCutCell` builds each `divergence_[cell]` from **float-ordered** face products, differences, cell-size division and source division before accumulating connected-region sums. The independent public budget multiplies stored face speeds/open fractions and volume source in **double**. Algebraic equivalence does not imply identical cancellation in finite precision. However, the current witness does **not** prove all the discrepancy is float rounding; additional operator/source-stage ordering and boundary-flux bookkeeping remain possibilities. No solver correction, pressure cap, altered source, or new activation threshold was applied.

**Next narrow gate:** on the *same immutable reopening pre-projection snapshot*, independently reproduce the exact `float` expression and region-wise summation order that `projectCutCell` uses, then compare its component sums with (a) reported `compatibilityCorrection`, (b) double budget reconstruction, and (c) analytic/Swept GCL. Determine whether float RHS cancellation is a substantial source of inter-platform variation before testing an opt-in double-accumulated RHS or alternate preconditioned solver. Keep the pressure equation and two-way Body/wall momentum checks separate acceptance gates.
