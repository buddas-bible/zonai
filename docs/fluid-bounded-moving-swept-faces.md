# AABB-bounded moving analytic swept face clipping

The moving-only analytic swept-flux path introduced by PR #305 excludes immovable shapes from the volume-flux integral, but still visits **every internal U/V MAC face** at each interpolated time sample. This PR limits expensive face/shape intersection evaluation to **the current intermediate pose's conservatively padded AABB** for every moving shape.

The default-on `setAnalyticCutCellBoundedSweptFacesEnabled(bool)` controls this additional bounding optimization. It applies only when moving-only swept clipping is enabled and a valid analytically disjoint collection contains **three or more shapes**. Two-circle overlap/containment ownership handling, single-shape analytic geometry, sampled fallback and genuinely stationary time paths remain unchanged.

For each intermediate pose:

1. `BuildDisjointAnalyticShapes(intermediate)` still constructs the complete current-time geometry and validates disjointness. The cache for final Step geometry is **not** reused for swept integration.
2. Moving shape owners are visited in input order. For each owner, all U faces and then all V faces in a conservatively expanded shape AABB range are clipped. Their exact double-precision `interval.length * normalVelocity` values accumulate in reusable local per-face scratch vectors.
3. After all owners, accumulated doubles are cast to floats **once per face**, and added to swept volume in original U-then-V row-major order. This matches the old per-face ordered sum/convert/add operation even when several distinct moving shapes intersect the same face.
4. Adaptively refined swept time samples reuse the same buffers but reconstruct each intermediate pose and clear all face values at each sample.

All translational and rotational transform changes contribute even for kinematic bodies with zero inverse mass. The method does not depend on an obstacle's instantaneous velocity or on whether the pressure solver treats that body as movable.

**Regression:** run the existing moving-only swept differential scene twice: first against the original all-shape scan, then with moving-only enabled in both fields and AABB clipping OFF vs ON. The scene includes 16 or 48 stationary circles, moving dynamic and kinematic circles, 0.5m/0.25m grid resolutions and an adaptive swept sampling configuration. It compares swept flux totals, mismatch diagnostics, sample counts, coupled-pressure convergence and continuity, per-body pressure traction and total fluid momentum across multiple actual moving snapshots.

**Benchmark:** `fluidPreconditionerBenchmark --swept-bounds-sweep` measures a genuinely moving second Fluid step for each of 0/16/48 stationary circles and 0.5m/0.25m grid resolutions. Both benchmark modes keep moving-only integration ON; only full-grid vs AABB traversal differs. The CSV includes swept time, whole Step time, sample counts, convergence and physical diagnostic values. Windows Release CI runs this as an **informational measurement only**, with no speedup threshold. Large or closely packed moving bodies may see smaller benefit than tiny moving bodies in a large field.
