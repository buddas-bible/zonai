# R3v — pressure-stage stability diagnostics

Base: [R3u #362](https://github.com/buddas-bible/zonai/pull/362), `fd82d900581b7234c841525cbd0f83741e2c2ae6`.

## Verified baseline

[R3t strict CI](https://github.com/buddas-bible/zonai/actions/runs/45413689799) passed 6,562,060 Fluid rows and 25,596 engine math samples exactly. [R3u Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45414877533) succeeded: 7 combined Contact/Joint/Fluid intervals, first tick 8 interval 1 (zero-based), one maximum live Contact, friction impulse observed, 240 coupled intervals and 60 exchange frames; max face speed 2.81309819 m/s, max continuity RMS 0.0932893753 1/s. Windows comparison pending at preparation.

## Diagnostic scope

- Read-only test instrumentation of the already verified 360-tick free-body scene and 120-tick anchored assembly (1,920 Fluid intervals total). Keep their existing authoritative rows unchanged. R3u contact fixture is retained but not included in this diagnostic until its cross-platform result is known.
- At phases 4–8 (remap, first boundaries, advection, second boundaries/pre-pressure, post-pressure), measure effective interior open MAC face speed, aperture-weighted speed, peak face coordinates/open fraction, counts and maximum speeds for openings <=1% and <=0.1%.
- Using the same effective aperture as the Cut-cell projection, accumulate partial dual-face momentum and kinetic energy with fixed iteration order and mass = cell area * opening (unit density/depth). The existing binary MAC pressure energy ledger is cleared by projectCutCell and is not a valid Cut-cell energy measurement.
- Copy the U/V arrays at phase 7; at phase 8 identify the fastest projected face and the largest projection velocity change independently. Record before/after at the same coordinates, effective opening, neighboring pressures, pressure difference, fractions and volume rates. Pressure range across active cells and pressure residual/predicted continuity are contextual diagnostics. Absolute pressure depends on gauge; compare pressure differences and same-face changes.
- Record the existing Cut-cell analytic impulse budget with validity flag, and its fluid/solid/wall and residual components. Separately record aggregate frame drag residual, not a per-body or per-contact accounting. Record open-face kinetic energy change across projection; it excludes body energy, wall/source work, other phases and Joint/Contact dissipation. Positive change is not by itself an instability or energy creation proof.
- Add 206,880 exact diagnostic rows: 1,920 intervals * (5*13 + 42) plus 480 frames * 3 drag residual fields. Place them under fluidStabilityAudit so historical physics segment counts do not change. Require exact count and compare both platforms; preserve existing strict gates. No arbitrary spike cutoff, pressure/velocity clamp, solver modification or tolerance change.

## Acceptance

Both platform trace producers, exact physics/math and diagnostics, and CMake regression. Inspect trace maxima, phase of amplification, aperture and pressure difference before deciding on a corrective algorithm. A converged pressure system is not automatically stable. This audit does not certify stability across all scenes or total FSI conservation. Existing near-closed-face stress/experimental #326 remains separate and unchanged; next replay a targeted near-closed-face event and compare with these observations. Master unchanged.
