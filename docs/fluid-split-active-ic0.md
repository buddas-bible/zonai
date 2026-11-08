# Split active IC(0) coefficient streams

The coupled Cut-cell Schur pressure solve uses cached IC(0) with active-cell forward/backward sweeps. On a 0.25m World/Fluid interleave in Windows Release CI, its preconditioner applications dominate the instrumented solve (~0.68ms of ~1.18ms Schur time for one Fluid substep in the observed reference scene). The existing AoS cache stores seven doubles per cell, even though the forward pass reads only two ratios and the backward pass reads three values.

The optional, **default-ON** `setAnalyticCutCellSplitIc0CoefficientsEnabled(bool)` selects two separate arrays for the active sweep: `incompleteForwardCoefficients` (westOverPivot, southOverPivot) and `incompleteBackwardCoefficients` (east, north, inversePivot). This reduces coefficient cache footprint from 56 to 40 bytes per grid cell and avoids fetching unrelated cached values for each sweep. It changes **only the storage layout**: factorization still computes the same IC(0) pivots using the same west/south face apertures and order, and the forward/backward loops visit the same active cells in the same direction with the same arithmetic sequence.

The layout applies only when *all three* IC0 features are enabled: incomplete Cholesky, cached coefficients and active-cell sweeps. If the new toggle is OFF, the original AoS implementation runs unchanged. Dense cached sweeps, reference on-demand IC0, Jacobi, sampled geometry, and binary MAC paths are unaffected. Both split arrays are cleared/rebuilt for each coupled pressure solve; no cross-step reuse occurs.

## Forward / backward timing diagnostics

With `setCutCellPressureTimingEnabled(true)`, the split active-cell path now reports
`coupledTiming.ic0Forward` and `coupledTiming.ic0Backward` as diagnostic subsets
of `preconditioner`. They are not additional phases in `accounted()` and remain
zero on Jacobi, AoS and other paths. Timing is accumulated over all preconditioner
applications during one solve; the clock is only sampled when profiling is enabled.
`sandboxWindStepBenchmark --csv` also appends the two sweep-time columns for real
World/Fluid interleaved frames at 1/2/4/8 Fluid substeps, and the Sandbox Wind
profiling UI shows the same per-frame accumulated values. These measurements
remain subsets of preconditioner time and are reset when profiling is disabled.
`fluidPreconditionerBenchmark --ic0-split-sweep` adds both columns, allowing the
sequential forward and backward costs to be compared before considering a change
to dependency traversal or memory layout. No solver arithmetic or stopping rule
is changed by this measurement.

## Validation

The `checkActiveCellIc0SweepDifferential(true)` regression compares AoS OFF vs split ON across 0.5m and 0.25m grids and three two-circle relationships (partly overlapping, disjoint, strict containment) over four changing snapshots. It checks coupled convergence/iterations, exact residual and continuity results, exact per-body pressure traction, sampled velocities and fluid momentum. The original active-vs-dense IC0 differential is retained, along with `clear()` toggle and state checks.

The informational Windows Release `fluidPreconditionerBenchmark --ic0-split-sweep` compares AoS/split on the same CI runner at 0/16/48 stationary analytic obstacles, 0.5m/0.25m grids. It records pressure IC0 factorization, total preconditioner application time, Schur multiply time, coupled PCG solve time, whole Step median, iteration count and true residual. Each mode uses the same analytic geometry, RHS, active IC0 and cached Schur edge operator. Measurements are not performance gates, and preserving numerical behavior alone does not justify merging a slow layout.
