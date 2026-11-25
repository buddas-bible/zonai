# R4h — actual World response to frozen pressure loads

Base: [R4g #374](https://github.com/buddas-bible/zonai/pull/374), `771e88230a211876b8fa517b6335a951234684f6`.

## Evidence

[R4f required CI](https://github.com/buddas-bible/zonai/actions/runs/45424189419) passed 7,735,408 Fluid/diagnostic rows and 25,596 math rows exactly. [R4g Ubuntu](https://github.com/buddas-bible/zonai/actions/runs/45424935835) passed. Index43 midpoint32 gap 6.10253066e-7 -> 3.7581679e-7 m², negative flux-only volume 3.19872643e-8 -> 0, interval peak 0.65360385 -> 0.653602225 m/s. Index44 depth16 gap 6.41733064e-7 -> 4.2770058e-7, peak 0.809383508 -> 0.668151788 m/s, negative volume removed. Index42 stalled on third pass; last accepted gap 3.37232173e-7 m². All five had lower final raw gap and no negative flux-only volume. Final Windows comparison pending at implementation. The fixed-graph candidate is still experimental.

## Actual rigid solver comparison

- For each of five frozen cases, initialize fresh Sandbox wind-flow Worlds from the actual current obstacle poses and velocities. Verify mass/inertia, zero damping and initial COM match. Gravity is zero in the existing fixture; sleep disabled. Apply zero, original production pressure load or final accepted R4g pressure load at COM plus torque, then call real World::Step(dt,N), for N=1,2,4,8. Recompute B^T p at the final accepted pressure rather than using scratch from a rejected trial. No fluid step, drag, joints or contacts; assert no constraints after stepping. The N=2 case matches World8/Fluid4 cadence.
- Measure loaded-minus-unloaded COM displacement, relative angle and velocity change. Compare dt² M^-1 F assumption against the semi-implicit constant-force substep prediction dt²*(N+1)/(2N)*M^-1 F (and angular analogue). Report measured errors; no presumption of exact equality in binary32.
- Rebuild analytic geometry for unloaded and loaded actual World outputs and two hypothetical loaded poses based on unloaded final pose plus predicted displacement. Report actual geometric volume response, full-step and cadence-aware errors, topology changes and signed volume sum. This measures the *next* isolated World interval from frozen current pose, not the historical prior interval nor a rerun of the live coupled scene. No live World cloning or contact/warm-start state restoration.
- Emit 63 fields per load comparison (3*18 body fields + 9 aggregate), 5*4*2*63 = 2,520 exact rows. Retain all existing traces and tolerances. Expected total 7,738,268 Fluid/diagnostic rows, 25,596 math rows.

## Acceptance

Both producers, strict comparison and CMake regression required. Compare measured response and actual volume errors to assess whether the cadence factor is appropriate for the free-body model. Do not extrapolate it to joint/contact-constrained effective mass or a coupled nonlinear trajectory. The experiment does not change Schur coefficients, force timing, endpoint handoff, tracer or production World. Actual candidate trajectory, constraint-aware response, conservation and performance remain open. Master and experimental #322/#326 unchanged.
