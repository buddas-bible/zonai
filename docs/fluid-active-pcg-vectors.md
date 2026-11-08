# Active-cell PCG vector operations

The coupled Cut-cell pressure solver already constructs `coupledPressureScratch::incompleteActiveCells` in row-major order while building active IC(0) coefficients. However, PCG still scans every grid cell for each search-direction dot product, residual update, preconditioned dot product, and search-direction update. Fully covered solid cells have zero RHS and no open Laplacian face, and do not participate in coupled pressure. Revisiting those slots on every iteration is potentially wasteful for obstacle-heavy grids.

The default-ON `setAnalyticCutCellActivePcgVectorsEnabled(bool)` reuses the **same current-solve** sorted active-cell list, **only** when cached active-cell IC(0) is enabled. A small local visitor dispatches each vector pass either through that list or through the original dense row-major range when disabled. The active pass changes the initial `r·z` and `r·r`, PCG `d·A d`, `x/r` updates and residual norm, the next `r·z`, and the search-direction update.

The active list is built anew from current `cellFluidFraction_ > 0` while factoring IC(0), exactly as the existing preconditioner does. Because it preserves ascending cell order, contributions from active cells occur in the **same sequence** as the original dense sums. Inactive cells remain zero in the initialized `x`, `residual`, `z` and `direction` vectors: they have no pressure RHS or Schur boundary face and do not connect to active cells through open face weights. The Laplacian/Schur matrix product, its zero-fill, IC(0) factorization/application, true residual recomputation on the complete matrix, convergence tolerance and final pressure/force calculation remain unchanged.

Switching to uncached IC(0), dense IC(0), Jacobi, sampled or non-cut-cell geometry implicitly retains the original dense vector passes. `clear()` preserves the setting but resets all coupled pressure scratch/history as before. No list or matrix coefficients are persisted between separate Fluid steps.

## Differential regression

`checkActiveCellIc0SweepDifferential(false, true)` compares dense vector passes with active vector passes on identical split-IC(0) / Schur operator configurations, at 0.5m and 0.25m grid spacing, with disjoint, overlapping and contained moving circles for four evolving snapshots. It checks exactly equal coupled PCG iterations, relative residual, predicted continuity, pressure residual, per-body force/torque and fluid momentum, in addition to the existing finite-pressure/true-residual gates. An explicit `clear()` assertion checks the toggle lifetime. The previous AoS/split and dense/active IC(0) tests remain unchanged.

## Performance gate

`fluidPreconditionerBenchmark --pcg-active-vectors-sweep` compares paired **full-grid vs active-cell** vector passes on one Windows Release CI runner with all other analytic Cut-cell, split IC(0), and cached Schur settings identical. Tests include h = 0.5m and 0.25m; 0/16/48 static circles; and two stationary-circle radii (0.12m thin obstacles and 0.35m substantial obstacles) to expose the active-cell fraction dependence. The CSV reports active cells, the instrumented PCG vector phase, Schur multiply, IC(0) preconditioner, total PCG and whole-Step time, iteration count and true residual, all with five-run medians. Measurements are informational; no speedup is claimed until CI completes.

Avoid merging if skipping dense zeros does not actually reduce the measured vector phase and full Step consistently or if any pressure/force differential fails.
