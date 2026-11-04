# Static-obstacle Schur pressure scaling benchmark

`fluidPreconditionerBenchmark --static-sweep` runs the existing cold-start analytic Cut-cell, cached-edge, active IC(0) pressure path with **two movable circles** and **0, 16, or 48 disjoint immovable circles**. It samples both 0.5m and 0.25m cells in a fixed 12m × 8m domain. Geometry is fixed across the five measured trials; one warm-up trial precedes them.

The CSV reports requested static obstacle count, convergence count, total Field step and coupled pressure times, Schur assembly/multiply/preconditioner times, PCG iterations, true relative residual and cached Laplacian edges. The Windows Release GitHub Actions job runs it as an *informational* benchmark, not an FPS gate.

This sweep makes the amount of static geometry explicit after PR #296 removes immovable faces from `B M_body^-1 Bᵀ` and PR #297 clears only movable body response entries during each PCG product. **It does not isolate their speedup relative to pre-change code**: geometry processing and the grid pressure operator still grow with obstacle complexity. Compare the same executable's 0/16/48 rows for scaling, and compare the same scenes against a pre-optimization commit on the same machine if attribution is needed. Timing noise, convergence and field resolution must be considered before conclusions.

The benchmark changes neither the fluid equations nor the solver convergence criteria. Its static circles are not a substitute for a future static wall/terrain workload with realistic polygons and a larger domain.
