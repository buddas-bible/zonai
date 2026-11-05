# Indexed analytic Cut-cell boundary synchronization

After a coupled Cut-cell pressure solve, `synchronizeCutCellBoundaries` updates predicted solid-normal velocities and the additional blocked-face flux used by the continuity diagnostic. Previously the disjoint analytic path scanned **every internal U/V face**, even though most apertures were fully open.

The analytic geometry pass already stores `analyticUBlockedFaces_` and `analyticVBlockedFaces_`, in U-then-V row-major order. These contain **every face with strictly positive analytic blocked length** (including intervals so small that the stored float aperture rounds to 1). The synchronization routine already skips `open >= 1`, so selecting this index is an exact superset of the faces that can affect the result. This avoids revisiting fully open faces while preserving the original face order, shape clipping, double accumulations, predicted solid velocities and pressure flux.

`setAnalyticCutCellIndexedBoundarySyncEnabled(bool)` is ON by default. OFF uses the original whole-grid scan, retained for differential testing. Both paths still rebuild disjoint analytic geometry for the current obstacle snapshot; other Cut-cell geometry and sampled paths are unaffected. `clear()` clears the existing blocked index arrays, while preserving the configured toggle.

The extended fluid field regression runs both modes with 16 or 48 separated static circles plus two moving dynamic circles, at 0.5m and 0.25m cell sizes. It checks every U/V face aperture and synchronized solid speed exactly, as well as coupled PCG convergence/iterations/residual, predicted continuity RMS, per-body traction and fluid momentum across three frames. This includes the static-to-dynamic solver's separate existing regressions.

`fluidPreconditionerBenchmark --boundary-sync-sweep` provides paired full/indexed readings for 0/16/48 static obstacles, recording total pressure stage, coupled PCG and whole step times, iteration count and relative residual on the same Windows Release runner. The timings are informational, not CI performance thresholds. End-to-end speedup may be modest because the PCG solve still dominates the pressure stage.
