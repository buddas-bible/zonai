# R3m — actual World/Fluid interleaving determinism

Base: [R3l #353](https://github.com/buddas-bible/zonai/pull/353), `ca85f0e85c0bc0bbd69ec560dc8dc05037dd350e`.

## Verified baseline

[R3l strict CI](https://github.com/buddas-bible/zonai/actions/runs/45404039536): all six standalone Fluid scenes, 1,697,236/1,697,236 raw-bit rows identical (previously 14,474 differences). Engine math 11,728/11,728; required World/Contact/Island/Joint gate passed. Separate external libm research gate remains red. [R3l Release A/B](https://github.com/buddas-bible/zonai/actions/runs/45404039709): zero nonfinite faces and PCG failures, zero paired delta in reported 192-row coupled audit metrics, h=0.25/8 substeps swept 0.0455→0.0453 ms and field 7.2798→7.2993 ms. This is a limited measurement, not a stability proof.

## R3m test scope

- Reuse the actual headless Sandbox `rigidBodyDemo::step` wind flow path (same sources as sandboxWindFlowDemoTests), without duplicating the coupling algorithm or changing production code.
- Two scenes: `worldFluidPressure` (coupled analytic pressure, drag 0) and `worldFluidDrag` (same pressure, two-way drag coefficient 3). Existing three bodies (Circle/Capsule/Box); 24×14 field, h=0.5 m, source strength 8, 12 fixed 1/60 s ticks with 8 World and 4 Fluid substeps. Fixed initial circle velocity and capsule angular velocity. Source oscillation remains the actual Sandbox source, including existing libm call.
- Capture canonical public World/Body state (no lifetime tokens), raw MAC/pressure/Cut-cell/history, deterministic Schur status/iteration count, per-frame average pressure forces/torques, Body/Fluid drag impulses. Compare every frame exactly with a second independent unobserved simulation.
- Existing read-only Fluid stage observer records phase 0 (after source injection, before geometry) and phase 9 (after tracer, before history commit/drag/World integration), all four intervals of the first two ticks. For all 12 ticks require the actual ten phases per interval, four intervals, no coupled pressure failure, finite live face/continuity and feedback, nonzero pressure and flow, actual analytic/IC0 coupled path, and nonzero Body and Fluid drag impulse only in drag scene.
- Preserve the old six scenes and strict row comparator; append new scene names to existing Fluid TSV. Include Sandbox changes in strict CI path filter. No wall-clock fields captured.

## Acceptance / next work

Verify both trace jobs and the full CMake regression; inspect per-scene first mismatch. If standalone scenes remain exact but coupled traces differ, locate the first actual source/pressure/drag/World divergence before changing code. A stage 0 mismatch can originate in the source or previous interval, not necessarily the MAC solver. No relaxation of strict gate. This is not exhaustive internal World cache coverage in the coupled scene, not a Joint-coupled fixture, and not a long-run or extra-architecture certification. General rotation outside ±float(2*pi) still uses legacy libm; direct Sandbox sin/cos and other paths require later audits. No master merge; experimental #326 untouched.
