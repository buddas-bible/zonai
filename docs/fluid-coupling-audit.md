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

## First results / next decisions

**No root cause is claimed before reading actual trace and tests.** When Windows Release CI completes, inspect spikes by frame and compare: (1) coupled vs uncoupled at matching h/steps/drag, (2) drag 0 vs 3 at matching h/steps/coupling, (3) continuity increase immediately after drag, and (4) coincident PCG failures or Body acceleration jumps. If a spike occurs before drag, investigate pressure/geometry/velocity remap; if after drag, test an explicit pre-projection force application in a separate opt-in differential PR. Pressure/Drag stage changes, solver restarts, obstacle identity and API policy changes remain **out of scope** in this PR.

The full environmental physics plan is being reviewed in PR #318, in `docs/roadmaps/environment-simulation.md` and `docs/roadmaps/environment-status.md`; since that PR is still Draft, these files are not yet expected on master.
