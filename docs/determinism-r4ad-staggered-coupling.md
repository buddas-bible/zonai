# R4ad — bounded staggered pressure coupling

Base: [R4ac #396](https://github.com/buddas-bible/zonai/pull/396), 2eb3d5409829b76188ecbc9bf97f05083c46c5bf.

[R4ac Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45466646091/job/138703197736), Ubuntu CMake and Release timing passed. Windows replay, strict comparison and Windows CMake pending when checked.

Double capsule construction reduced the matched signed volume change magnitude from 1.01199475e-7 to 4.96584107e-10 m² and pressure max change from 0.0124974251 to 0.00653636456. Force max change decreased from 0.0162982978 to 0.0116665661 N, but substantial sensitivity remains. Both precise fixed-geometry solves had zero negative volume. Do not pursue unlimited geometry/pressure fixed-point iterations.

## Alternative bounded schedule

Start an independent 16-interval trajectory from the same historical interval43 snapshot, with precise disjoint capsule geometry rebuilt on the start pose. Keep the already computed incoming velocities. Bootstrap with the historical production pressure load; subsequent loads come exclusively from the preceding staggered interval. Apply the previous pressure load to a fresh free-body World reconstructed from the carried state, using two substeps. Rebuild its actual endpoint and solve the weighted fixed-geometry interval pressure (adaptive16, gamma=0, cold strict residual target 1e-16). Reconstruct the instantaneous endpoint with only the velocity-copy seed projection. Commit the rounded endpoint MAC, pressure, actual body poses and velocities, and next interval pressure load. No second World replay or fixed-point iteration. The existing split start-geometry source and advection schedule is retained.

The method explicitly delays feedback by one fluid interval. Pressure does not implicitly couple the body response within that same interval. It may be unstable for light bodies or larger time steps; it does not establish exact momentum conservation. The historical initial load is a bootstrap, not a demonstrated production startup scheme. Negative volume is reported, never clamped. Unsupported volumes/faces and solver breakdown remain explicit failures. Compare against the original trajectory while acknowledging both precision and scheduling differ. Actual game-frame cost is not measured here; two pressure projections are still required.

Trace 49 diagnostics per interval plus 5,452 MAC/pressure/fraction values, 16 instantaneous endpoint diagnostics and four reconstruction momentum/energy proxies. 88,336 additional exact rows, expected 8,008,284 Fluid rows. Preserve all prior traces and timing variants. Require both producers, strict comparison, full CMake and numerical review before further promotion. No production default or master change.
