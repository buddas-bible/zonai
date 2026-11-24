# R3x — near-closed-cell volume source attribution

Base: [R3w #364](https://github.com/buddas-bible/zonai/pull/364), `5b7e8f16f0eb5f17842b2a8577a0cab3cd92c32d`.

## Evidence

[R3w Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45416625128) succeeded. Raw artifact 14932855592 contains 23,910 nearClosedFace rows. Windows comparison pending at implementation. [R3v strict run](https://github.com/buddas-bible/zonai/actions/runs/45415515508) verified 7,639,425 Fluid rows and 25,596 math rows exactly.

Historical index 43 (tick 11, interval 3, zero-based): V(17,13) aperture decreases from 0.0112415692 to 0.0000639549762. Velocity after remap/boundaries -0.5166502, pre-projection -0.510220349, post-projection -28.132021 m/s. Lower/upper pressures -0.166325137 and 1657.141602, jump 1657.307983. The projected delta matches -dt/h times the jump. Upper fluid fraction 1.8302887e-9, lower fraction 0.761748731. Upper cell only has east opening 5.72380814e-5 and south opening 6.39549762e-5. Upper source fraction change -5.67354728e-5, adjusted row RHS 0.200819409, Laplacian 0.200840153, Schur -2.07449338e-5, row residual 1.19872019e-9. Predicted continuity RMS 7.50726556e-8 /s; PCG pressure residual 5.30614273e-8 /s. Cached and uncached frozen outputs matched all 6,867 captured fields on Ubuntu, including feedback. This supports a nearly disappearing-cell continuity/geometry issue rather than cache or convergence failure; it does not yet distinguish physical displacement from swept integration error or relocation.

R3v raw artifact 14931003159: for the h=0.5 free-body fixture maximum post-projection speed 6.04075 m/s was fully open; maximum projected speed for opening <=0.1% was 2.80739 m/s. Maximum pressure-induced velocity change 0.916819 m/s at tick 335 interval 0, V(14,2), opening 0.0118521, lower fraction 0.000336865. Maximum analytic pressure impulse residual norm 7.60e-7 N*s (partial ledger). Anchored assembly max residual 2.92e-7 N*s; no claim of whole-system conservation.

## New same-state audit

- Use the existing frozen index 43 and replay its original swept source, existing snapshot-fraction fallback, and existing local swept geometry reconciliation on independent copies. Only test friend changes the frozen source selection or invokes the already existing reconciliation before replay. No live solver, aperture, velocity, obstacle transform, pressure algorithm or default configuration changes. The snapshot alternative is a diagnostic counterfactual, not a proposed stable method.
- Attribute target upper-cell source to own swept divergence vs own endpoint fraction difference plus individual fully-solid donor cells assigned by the actual nearest-fluid mapping. Reconstruct the assigned source in original array order and require bitwise equality to production volume source. Record all donor contributions, pre-projection fluxes, velocity/source/compatibility-adjusted divergence, pressure jump, global sums, unresolved and redistributed volume rates, geometry mismatch.
- Require both alternate PCG solves to converge and feedback finite. Record aperture, target speed, projected global peak and max velocity delta, pressure impulse ledger, partial open-face momentum/energy, continuity, PCG iterations, tracer mass and force/torque differences from baseline. A reduction in peak alone is not acceptance; source accuracy, residuals and feedback changes matter. No source discarded or clamp applied.
- Preserve 15 prior segment counts (including 23,910 stress rows counted in Ubuntu artifact); add volumeSourceAudit segment with strict bitwise cross-platform comparison. No change to existing fingerprint schema. All variants are read-only diagnostics of the live path and mutations of private copies only.

## Acceptance

Review R3w Windows and full CMake before certifying baseline. Both R3x producers, strict comparison and full regression required. Analyze actual donor/source breakdown and alternative output, including changes in feedback and continuity, before selecting a correction. Endpoint differencing and local reconciliation may both leave a severe sliver constraint; if so, investigate conservative space-time geometry/constraint treatment separately. No production change, no pooling, master and experimental #326 untouched.
