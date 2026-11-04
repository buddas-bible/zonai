# AABB-bounded analytic Cut-cell face clipping

`fluidField2::updateCutCellGeometry` originally checked all disjoint analytic shapes for every internal U/V MAC face, even when the shape was nowhere near that face. This change adds an optional **default-on** path for scenes with **three or more disjoint analytic shapes**. It clips faces only within each shape's conservatively padded AABB grid range and accumulates blocked lengths and owner boundary velocities into reusable double-precision U/V face scratch buffers.

For each face the contributions remain in **original shape order**, preserving the numerical summation order of `ForEachAnalyticFacePortion` and the output float apertures and velocities. After accumulation the original U-then-V row-major face walk is preserved, including the strictly-positive blocked-face index list used by PR #299. The overlapping/contained two-circle ownership path, sampled fallback and binary MAC are **not** changed. Current shape transforms, velocity snapshots and AABBs are used on every fluid step; `clear()` discards scratch values.

The flag `setAnalyticCutCellBoundedFaceClippingEnabled(bool)` switches the disjoint multi-shape geometry pass between the bounded and original full-face reference paths. The reference is retained for differential validation.

The regression test uses two moving dynamic circles plus 16/48 separated immovable circles across 0.5m and 0.25m grids. For each step it compares **all U/V face apertures and solid velocities exactly**, coupled pressure convergence/iterations/true residual, per-body force/torque, fluid momentum, and the toggle's persistence over `clear()`.

`fluidPreconditionerBenchmark --face-sweep` runs matched reference/bounded scenes (0/16/48 static shapes, 0.5m/0.25m cells), outputs geometry, coupled PCG and whole step times, and checks convergence reporting. The Windows Release CI samples this as informational only. When there are zero statics, the two-shape reference path is intentionally used regardless of the toggle and should show similar timing. No performance result is claimed before paired CI measurements.
