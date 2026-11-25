# R4y — actual geometry closing-cell comparison

Base: [R4x #391](https://github.com/buddas-bible/zonai/pull/391), f364aead7c74d1c972e3b47c88f0074a540677a3.

[R4x strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45460813567) passed 7,919,451 Fluid rows and 25,596 engine math samples with zero mismatches. R4w strict comparison also passed. External libm diagnostics retain 33 mismatches. R4x Ubuntu CMake and Release timing passed; Windows CMake pending when checked.

## Localized defect

Interval57 cell (17,14) starts with 7.68571113e-7 m² and closes completely in both predicted and actual loaded geometries. Integrated outflow is 7.71125812e-7 m² with zero inflow. East and north are the only supported outgoing faces, mean apertures 0.00153892309 and 0.00233071856.

The signed ledger of -2.55469957e-9 m² decomposes into predicted volume zero, linearized body-response volume -2.26925188e-9, removed compatibility volume +2.85533124e-10, and signed linear volume residual -8.5439672e-14 m². Thus the principal error is extrapolating the linearized body response below the geometric zero-volume boundary, with a smaller contribution from uniform compatibility removal. Linear solver residual alone cannot explain the deficit.

## Matched fixed-geometry comparison

Keep the original 16-interval trajectory and accepted pressure loads unchanged. At interval57 only, independently replay the interval using the same incoming trial velocities and actual corrected endpoint geometry, interpolated poses and adaptive16 quadrature. Disable additional body response (gamma=0) so the already applied World pressure is not counted twice. Compare uniform RHS compatibility removal and removal weighted by actual endpoint fluid fraction per connected component. Keep the pressure gauge projection unchanged; remove residual roundoff uniformly. A component with nonzero incompatibility and no endpoint fluid volume is rejected explicitly. Use cold initialization and relative squared residual 1e-16 in both comparisons, retain independent true residual and volume identities. No aperture floor, clamp, donor transfer adjustment or committed state change.

Report target cell ledger, compatibility and residual, all negative cells/volume, actual closure, total volume mismatch, support, peak, iteration count and unapplied load differences. The fixed-body replay is an oracle-style diagnostic using the already computed actual endpoint, not a valid standalone forward solver; neither solution's feedback is applied to World. Uniform versus weighted results isolate the effect of compatibility distribution within the fixed-geometry model. Compared with the coupled baseline, actual geometry and response operator both change, and solve tolerance is tighter. Neither variant proves chronological donor feasibility, nonnegative transport or total momentum conservation.

Add 38 exact rows, expected total 7,919,489 Fluid rows. Preserve all previous traces and timing variants. Both producers, required exact comparison, full CMake and numerical review required. No production code, tolerance or master changes.
