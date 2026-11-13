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

### Remaining S2c acceptance gate

1. Run this applied-transfer ledger on the **actual** h=0.25, World8/Fluid4 source (18,13), its connected neighbors and full domain, and compare old / pooled on a frozen state **and** across two separately evolved World trajectories. The compact four-direction regression fixture does not establish that its pressure cells were actually pooled.
2. In each Fluid substep record displaced-by-snapshot mass, swept transfer, nonlocal fallback, unresolved mass retained or trapped, and subsequent MAC boundary crossing as **separate** quantities; verify the same-source local stencil, without adding MAC export twice or equating tracer kilograms to displaced geometric cubic metres.
3. Compare actual transport and pressure-source volume on the same captured geometry; verify opening/closing, nearly zero aperture, full disconnection, body reaction/trajectory, and varying cadence. Decide on physical acceptance criteria only after those measurements.
4. Measure the opt-in observer's overhead when enabled and normal performance without the observer. Keep the pressure pooling option default OFF and PR #326 Draft. Do not merge #322's sampler into this pressure experiment.

