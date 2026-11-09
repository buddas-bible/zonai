# Wind coupling spike audit (Audit 1 / S0a)

Status: **instrumentation only, diagnostic results pending CI**. Based on master `d2d55482a0c07784cb9737d2cf82301b34543169` (2026-11-09). No fluid or rigid-body equations, default coupling mode, or pressure solver behavior were changed.

## Why

The Sandbox occasionally shows abrupt airflow motion with two-way coupling enabled. The first experiment must separate (1) pressure/cut-cell divergence, (2) the drag reaction injected **after** fluid projection, (3) changes in World/Fluid substep cadence, and (4) rigid Body speed. Merely switching the old global coupling toggle also changes obstacle presence and, previously, fluid substep count; that is **not** a controlled coupling-only comparison.

## Run

Build the `sandboxWindStepBenchmark` target and execute:

```text
sandboxWindStepBenchmark --coupling-spike-audit
```

CSV outputs one row **per measured World frame** after four warmup frames, for 0.50/0.25 m cells × 1/4 Fluid substeps × coupling ON/OFF × drag 0/3. `world_substeps` is always 8. The input wind source remains enabled at strength 8.0, with the same initial dynamic Body 0 velocity and Body 1 rotation across scenes.

The `setWindCouplingAuditEnabled(true)` Sandbox switch is strictly opt-in. It (a) records the **current** Cut-cell volume-source-aware continuity RMS and maximum absolute velocity of open MAC faces immediately after the Fluid Step and immediately after the Sandbox drag exchange, and (b) forces the *requested* Fluid step count even when the global coupling toggle is OFF, so the comparison uses the same scheduler cadence. Disabling the switch retains the original scheduling and does not introduce extra field scans. Regular simulation never calls `getFlowAudit()` implicitly.

- `pre_drag_continuity_rms` / `post_drag_continuity_rms`: maxima of frame's per-Fluid-substep measurements in 1/s; **their difference is not itself a same-substep delta**.
- `max_drag_continuity_increase`: max positive **paired** after-minus-before continuity RMS over Fluid substeps in the frame.
- `pre_drag_max_face_speed` / `post_drag_max_face_speed`: maxima of absolute U or V components at open fluid faces (m/s); not Euclidean cell-centered velocity.
- `max_drag_face_speed_increase`: max positive paired after-minus-before change in the above maximum across substeps.
- `nonfinite_faces`: sum of non-finite open-face velocities measured **after drag**; does not claim to cover fully solid faces.
- `pcg_status`, `pcg_iterations`, `pcg_relative_residual`, `pcg_failed`: **last Fluid substep's** pressure state in this World frame, not a maximum or aggregated failure count.
- `body0_speed`, `body0_omega`, `body0_x`, `body0_y`: Body 0 state **after World Step**; acceleration and torque are not directly inferred from these values without prior frame state.
- `drag_body_impulse`, `drag_exchange_residual`: magnitude of frame-integrated drag Body impulse and its Body+Fluid bookkeeping residual; not pressure traction or full energy conservation.
- `fluid_ms`, `world_ms`: Sandbox wall timings, informational only. Audit calls themselves contribute to `fluid_ms`, so this CSV must **not** be used for speed comparisons to non-audit runs.

The `fluidField2::getFlowAudit(dt)` query is side-effect-free and uses the latest stored `cutCellVolumeRate_` for Cut-cell continuity, which includes the last obstacle snapshot's movement. It does **not** include the predicted `pressureUSolidFlux_` / `pressureVSolidFlux_` correction applied for the separate post-pressure continuity estimator. In particular, `getCutCellPressure().predictedContinuityRms` and `getFlowAudit().continuityRms` are not identical physical quantities. The legacy binary projection uses `getDivergenceRms()` instead. Invalid dt yields NaN continuity, not a silently valid zero.

## Automated invariants

- Enabling audit in an already synchronized two-way World scene does not change fluid momentum, drag impulses or Body trajectories at fixed cadence (0.50/0.25 m).
- Drag=0 implies exact identity between before/after flow snapshots and zero exchanged drag impulses.
- An uncoupled scene can run the specified Fluid cadence for diagnostics; turning off audit restores the old single-fluid-step schedule.
- A standalone Field query is repeatable and does not mutate velocity/momentum; Cut-cell query is compatible with its volume-aware divergence diagnostics.

## Verified Audit 1 results (PR #319, Windows Release 42571477840)

Ubuntu 112/112, Windows Debug 115/115 and Windows Release 52/52 passed. Windows Release emitted 192 per-frame records, with 0 PCG failures and 0 non-finite open fluid faces across all 16 scenarios (two grid sizes, two Fluid counts, coupling ON/OFF, drag 0/3).

A **single-frame 28.13385 m/s maximum open MAC face velocity** was observed at **h=0.25m, World=8, Fluid=4, two-way ON, Drag=0, measured frame 6** (after four warmup frames). Neighboring measured frames 5 and 7 had respective peaks **0.5953736** and **0.7118427 m/s**. The same frame's PCG iterations rose to **61** from typical **46–47**, but the PCG relative residual was **8.197879e-6** and reported converged; Body 0 continued smoothly at approximately **0.589 m/s**. This is **a local face-speed spike, not proof of a whole-field or rendered tracer explosion**.

At the same h and Fluid count with Drag=3, maximum open face velocity across recorded frames was only **1.017758 m/s**. Post-drag continuity RMS increased by at most **0.000605896 1/s** in that case; the original Drag=0 spike appears equally in pre/post-Drag reads (no Drag exchange performed). Therefore the **28.13 m/s spike cannot have been caused by post-projection Drag injection in that same interval**. This does not rule out preceding geometry, Schur pressure, very small open faces or solver boundary synchronization. It also does not prove that drag stabilizes the solver in general: dragged bodies follow different trajectories.

These numbers were extracted from the successful Windows job logs of [run 42571477840](https://github.com/buddas-bible/zonai/actions/runs/42571477840), on the merged [PR #319](https://github.com/buddas-bible/zonai/pull/319).

### Follow-up instrumentation (PR #320)

The existing live-flow audit now also captures the **exact U/V face index, its signed velocity, open fraction**, and the **largest open-fraction-weighted face speed**. The Sandbox retains the substep and peak face across all Fluid intervals; the `--coupling-spike-audit` CSV adds `peak_face_substep`, `peak_face_is_u`, `peak_face_x`, `peak_face_y`, `peak_face_open_fraction`, `peak_face_signed_velocity`, `peak_open_weighted_max_speed`.

The distinction matters: a huge speed on a nearly blocked face may carry little area-weighted flux, while a large speed on a mostly open face is potentially more consequential. The area-weighted speed is a diagnostic, not a physical stability proof; it is not a kinetic energy norm or a spatially filtered velocity. These new measurements are read-only, and no pressure/drag solver change is included.

## Verified Audit 2 results (PR #320, Windows Release 42585749190)

[PR #320](https://github.com/buddas-bible/zonai/pull/320) merged after Ubuntu (112/112), Windows Debug (115/115), and Windows Release (52/52) success. The same 192-row CSV reproduced the identical isolated peak:

| Metric | Recorded value |
| --- | --- |
| Scenario | h=0.25m, World=8, Fluid=4, TwoWay ON, Drag=0, measured frame 6 |
| Peak Fluid substep | 3 (zero-based) |
| Grid face | **V-face (x=17, y=13)** |
| Signed face-normal speed | **-28.13385 m/s** |
| Open fraction | **0.00006395081** (~0.0064%) |
| That face's signed open-fraction-weighted normal flux speed | **~ -0.001799 m/s** |
| Maximum open-fraction-weighted face speed **anywhere in the field** | 0.6536318 m/s |
| PCG iterations / relative residual | 61 / 8.197879e-6 (converged) |

**Critical distinction:** 0.6536318 m/s is the global maximum weighted face speed, **not** the weighted speed of the -28.13385 m/s face. The anomalously fast face is almost entirely blocked, so its local unweighted normal speed does **not** imply an equally large net throughput across the whole grid. This still does not rule out corruption of interpolation-based velocity queries.

### Audit 3 — raw bilinear sampling near a nearly blocked face (current PR)

Source inspection confirms `sampleFrom` and the public `sampleVelocity` perform ordinary bilinear interpolation of raw MAC face velocities **without multiplying by or otherwise considering Cut-cell open fractions** (see `src/fluid/fluidField2.cpp` lines 1260–1290). Semi-Lagrangian velocity advection also uses `sampleFrom` for both its backtrace and reconstruction. By contrast the conservative tracer transport computes `u * openFraction` or `v * openFraction` at each face. This is a **potential propagation path**, not proof that this particular peak destabilized a Body, tracer or entire field.

The opt-in Sandbox audit now samples the exact peak face **in the same Fluid substep**, plus the centers of its two adjacent cells **if they contain fluid**. The CSV adds `peak_sampled_normal_velocity`, `peak_adjacent_cell_normal_speed`, `peak_adjacent_cell_fluid_fraction`, and `peak_face_flux_normal_velocity`. The latter is simply peak face's signed velocity × open fraction; it is not a reconstructed local pressure or material flux integral.

Next analysis: compare the 28.13 m/s face with the sampled normal velocity of actual neighboring fluid cells. If a substantial sample reaches fluid interiors, investigate a **separate opt-in, aperture-aware sampling policy** and compare it against the original CPU solver, pressure residuals, tracer transport and Body forces. Do not silently replace raw local velocity with aperture-weighted face flux: the two quantities have different physical meanings and can have different near-boundary behavior. No core equations or default sampling are changed in this audit.

## First results / next decisions

**No root cause is claimed before reading actual trace and tests.** When Windows Release CI completes, inspect spikes by frame and compare: (1) coupled vs uncoupled at matching h/steps/drag, (2) drag 0 vs 3 at matching h/steps/coupling, (3) continuity increase immediately after drag, and (4) coincident PCG failures or Body acceleration jumps. If a spike occurs before drag, investigate pressure/geometry/velocity remap; if after drag, test an explicit pre-projection force application in a separate opt-in differential PR. Pressure/Drag stage changes, solver restarts, obstacle identity and API policy changes remain **out of scope** in this PR.

The full environmental physics plan is being reviewed in PR #318, in `docs/roadmaps/environment-simulation.md` and `docs/roadmaps/environment-status.md`; since that PR is still Draft, these files are not yet expected on master.
