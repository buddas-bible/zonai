# R4i — cadence-aware free-body displacement model

Base: [R4h #375](https://github.com/buddas-bible/zonai/pull/375), `5990c84d832078e8b02c11e179f31782a546ef97`.

## Verified evidence

[R4g required strict CI](https://github.com/buddas-bible/zonai/actions/runs/45424935835) passed 7,735,748 Fluid rows and 25,596 math rows exactly. [R4h required strict CI](https://github.com/buddas-bible/zonai/actions/runs/45425665384) passed all 2,520 new World-response rows with zero mismatches (7,738,268 total Fluid rows); [CMake](https://github.com/buddas-bible/zonai/actions/runs/45425665172) and Release A/B passed. The overall strict workflow still reports failure from the separate known external-libm research gate, not the required engine gate.

Historical interval43 midpoint32, candidate force, N=2: maximum position error from full dt² response 4.56658203e-6 m, cadence-aware error 7.55696399e-8 m. Rebuilt geometry L1 errors 6.92128378e-6 and 1.79177732e-7 m² respectively. For these unconstrained bodies, the isolated next-interval response is consistent with the semi-implicit constant-force displacement coefficient (N+1)/(2N). This does not prove a response model for contacts, joints, drag, damping, or a retrospectively corrected historical path.

## Implementation

- Parameterize only the test-only interval pressure prototype with a displacement response scale. Preserve factor 1.0 for all existing cases. For the World8/Fluid4 free-body cadence, set gamma=0.75, giving A=Lbar+gamma Bbar M^-1 Bbar^T. Use the identical gamma in the Jacobi diagonal, quadratic operator identity, integrated body-volume contribution, actual geometry perturbation and nonlinear defect correction. Keep physical inverse masses, B^T p force/torque and dt/h fluid gradient unchanged. Do not scale the actual World load or its velocity impulse.
- Repeat the complete frozen solve and all downstream diagnostics for historical interval43 midpoint32 and closing interval44 depth16. No change to obstacle path, trial velocities, geometry quadrature or linear/defect tolerances. Record both existing and scaled cases, including independent volume closure, separated transfer, simultaneous/chronological tracer diagnostics, geometric rebuild, up to three defect corrections and matched fresh World response. The old tracer diagnostics still consume the initial linear solve, not the final nonlinear corrected state.
- Put complete new trace rows under cadencePressure, preserve all existing rows. 11,630 rows per case plus seven topology fields for the known single missed cell in interval44 = 23,267 new rows. Expected total 7,761,535 Fluid rows and 25,596 engine math rows. Require both producers and strict comparison.

## Acceptance and limitations

Review peak speed, body feedback, corrected and raw residual, negative volume, uniform concentration, topology and actual World response; compare both scale values on identical frozen inputs. The displacement factor is a free-body geometric-response parameter, not an effective mass change or a fully consistent momentum projection. The interval graph is still frozen and historical geometry is not recomputed from the actual pressure-driven trajectory. This is not live coupled integration, corrected-state tracer transport, or proof of stability. Before production changes, define a consistent forward-time predictor/corrector and test repeated steps, then constrained bodies. No production src changes, new defaults, clamps, tolerance relaxations or master merge. #322/#326 untouched.
