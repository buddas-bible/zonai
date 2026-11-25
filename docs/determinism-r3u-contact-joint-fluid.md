# R3u — Contact, Joint and Fluid in the same solver interval

Base: [R3t #361](https://github.com/buddas-bible/zonai/pull/361), `478edbdc3bbce52120b9c6221c1ae079fa9cb703`.

## Baseline

[R3s strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45410737830): 5,659,220 Fluid rows and 25,596 math rows exact. [R3t trace run](https://github.com/buddas-bible/zonai/actions/runs/45413689799) Ubuntu and Windows producers and required strict comparison passed: 6,562,060/6,562,060 Fluid rows exact, including 902,840 anchored assembly rows; 25,596 engine math rows exact. Separate external libm research remains red. Ubuntu coverage: original 360 ticks, 1,440 converged coupled intervals, 360 exchange frames, min margin 0.0391407013 m, max face speed 6.04074955 m/s; anchored assembly 120 ticks, 480 intervals, 120 exchange frames, min margin 1.97785783 m, max face speed 4.15207386 m/s, max continuity RMS 0.00580159621 1/s. These are observations, not general stability thresholds.

## New fixture

- Append `worldFluidContactAssembly`, 60 ticks, 240 Fluid intervals, 480 World substeps. Keep the R3t Weld/limited Revolute assembly and original fluid field/source/drag settings. Move only the free box initially to (1.64, 1.0), velocity (-1.5, 0.15) m/s. Initial nominal geometric gap between capsule and box is 0.09 m. No initial penetration, artificial correction, added force or production solver change.
- Retain R3t's strict in-domain, partial-cell, converged coupled pressure, per-body pressure exchange and aggregate drag conditions. A contact-induced fallback or coverage failure is evidence to investigate, not an excuse to weaken prior tests.
- Enable existing read-only Contact/Island and Joint substep recorder for all 60 ticks in the new scene. Validate 52 stage records per advanced tick. At solveBias, require at least one positive normal impulse within the contact range of an island containing both joints, following converged Fluid projection in the same interleaved interval. Report first tick/zero-based interval, number of combined intervals, maximum contact count and whether friction impulse occurred. Friction is informative, not required.
- Include persistent contacts and contact lifecycle snapshot (generation, free list, cache fields) at frame boundaries in observed/control exact comparison. Reuse existing Joint cache, World, and Fluid state traces. Record full transient contact, island and joint fields in observed trace. The coupled-pressure interval is followed by a rigid solver step; this is interleaved coupling, not one monolithic Fluid/Contact/Joint linear system.
- Keep all previous scenes and 13 pinned historical counts, including the verified 902,840 anchored assembly rows. Require the new contact assembly in strict streaming comparator. No new math rows or tolerance.

## Acceptance

Both platform producers must succeed; required physical comparison must be exact; all previous checks and full CMake must pass. If new scene fails, identify the failed interval/coverage or first raw mismatch; do not assert collision until actual positive impulse coverage passes. Initial scene and 60-tick duration are chosen test parameters, not a complete vehicle or raft. This does not certify continuous geometry containment, long-run stability, or arbitrary contact networks. Master and experimental #326 remain unchanged.
