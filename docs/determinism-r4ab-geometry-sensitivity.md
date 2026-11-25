# R4ab — geometry sensitivity of pressure feedback

Base: [R4aa #394](https://github.com/buddas-bible/zonai/pull/394), 8ae2aa294fa02be07d4a893a09eece68c9ed2350.

[R4z required strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45463803551) passed 7,919,549 Fluid rows and 25,596 engine math samples, zero mismatches; external libm retains 33 mismatches. R4y CMake passed both platforms. [R4aa Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45464609457/job/138696048811) and Release timing passed; Windows replay, strict comparison and full CMake pending at preparation.

Half relaxation reduced the full force residual from 0.00419402122 N to 0.000373497605 N by pass4, but pass5 increased it to 0.016111549 N. The preceding geometry change was 4.44973921e-7 m² and maximum center component change 1.1920929e-7 m. Quarter relaxation reached 0.000994026661 N full residual at pass6, leaving 0.000745519996 N unapplied. Both variants retained zero negative modeled volume and unsupported cells/faces, but neither establishes convergence. Each fixed-geometry solve required 195 PCG iterations; do not increase the relaxation iteration budget without evidence.

## Matched geometry sensitivity

On the actual half-relaxation pass4 to pass5 transition, independently substitute each body's complete next endpoint pose and velocity while holding the other bodies' preceding endpoints unchanged. Also substitute all three together. Rebuild analytic geometry, use identical incoming MAC velocities, start snapshot, weighted compatibility and strict fixed-geometry solve. Compare with the original pass4 solve, not the previous applied force. Record endpoint volume L1/signed/max change and cell, topology and interval support changes, compatibility L1/signed change, mean aperture changes, pressure/load response, negative volume, closure, peak and PCG iterations/residual. The all-body probe should reproduce the pressure response of pass5, subject to its identical numerical inputs; no modified load is applied to World.

These probes isolate contributions by body, not translation versus rotation or source versus operator. A quantized pose change correlating with a pressure jump does not by itself prove that binary32 precision is the sole cause. Use the results to choose whether local geometry precision, global compatibility/source sensitivity or a simpler game-oriented coupling warrants further work. Do not tune more relaxation factors. Preserve all historical trajectories and diagnostics, no production changes.

Add 20 rows per probe, 80 total, expected 7,919,929 Fluid rows. Require both producers, strict comparison, CMake and numerical review. No master merge.
