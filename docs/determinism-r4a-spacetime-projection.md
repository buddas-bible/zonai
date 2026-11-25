# R4a — interval-average coupled projection prototype

Base: [R3z #367](https://github.com/buddas-bible/zonai/pull/367), `f19d04a2dfd2a8e26f4d75d2b4b30feebba71a5b`.

## Verified evidence

[R3y required CI](https://github.com/buddas-bible/zonai/actions/runs/45418406663) passed all 7,663,714 Fluid/diagnostic rows and 25,596 math rows exactly. [R3z Ubuntu](https://github.com/buddas-bible/zonai/actions/runs/45419188235): at historical index 43, end aperture sum 0.000121193058; 64-midpoint average 0.0107398439 (88.6176495x), conditional speed bound 0.316994729 m/s versus endpoint 28.0913278. The bound fixes the prior source and predicted solid response; it is not a pressure solution. World8/Fluid1,2,4,8 peaks 1.62754583,2.29466367,28.132021,0.944099545 m/s, all intervals converged. No monotonic cadence trend. Windows strict result pending.

Space-time control-volume conservation is a relevant design principle, not a proof of this prototype: [Integrated Space-Time Finite Volume Method (JCP, 1999)](https://www.sciencedirect.com/science/article/pii/S0021999199963246), [An accurate moving boundary formulation in cut-cell methods (JCP, 2013)](https://www.sciencedirect.com/science/article/pii/S0021999112005839).

## Candidate model and implementation

Test-only, on the actual frozen index 43. For 8 and 32 midpoint poses independently:
- Reuse existing analytic geometry and zero-RHS production Schur assembly on a private field copy. Average effective openings and per-cell/per-body force and torque coefficients (COM interpolated between poses). No live state modifications. Require analytic geometry, successful coefficient assembly and support for every nonzero endpoint volume change.
- Use one trial velocity per interval-open face (frozen pre-pressure endpoint value), including faces closed at the end. This is a modeling approximation; closed-end trial values may be solid boundary values. Keep endpoint fraction differences, not donor relocation; record summed source. Use interval-active pressure rows and the connected averaged-aperture graph. Explicitly remove each component mean from RHS, report removed magnitude. Use a zero-mean pressure subspace P, unlike existing coupled production gauge handling.
- Solve P (Lbar + Bbar M_end^-1 Bbar^T) P p = P b with an independent matrix-free Jacobi PCG. Lbar weights use averaged apertures, Bbar uses averaged geometric coefficients, body inverse mass/inertia held at endpoint values. Project preconditioner and operator onto component zero-mean subspace. Require true projected residual squared <= max(1e-24, initial squared * 1e-16). Report unprojected residual separately; a small projected residual does not guarantee raw continuity.
- Reconstruct trial interval velocities with pressure gradient, calculate force and torque with the same Bbar^T. Verify the quadratic identity p^T P A P p = sum(a_bar*(dp)^2) + (Bbar^T p)^T M^-1 (Bbar^T p), within roundoff tolerance. Compare target and global interval speed, peak on endpoint-open faces, baseline force/torque deviations, source compatibility removal, iterations and residual. The identity is a discrete operator consistency test, not coupled kinetic energy conservation.
- Emit 28 fields per resolution (56 total) under spaceTimeProjection; require prior R3z counts and all preceding trace groups. Strict compare both platforms.

## Limitations / next gate

This is an independent experimental frozen pressure solve, not a production algorithm or a faithful implementation of a cited paper. It approximates time-constant face velocity and body response, uses a different source/active set/gauge from production, and does not integrate tracer, remap interval velocity to the endpoint, synchronize solid flux, or step World. A lower interval speed alone cannot justify deployment; pressure/force changes, unprojected residual, volume compatibility, opening/closing topology and advection consistency must be investigated. The endpoint-open peak is just a subset of interval velocities, not a computed endpoint velocity. Full-grid repeated geometry assembly and reference PCG are deliberately slow. If midpoint support is insufficient, fail explicitly; do not discard source. No default changes, caps, pooling, or edits to src. Master and experimental #322/#326 remain unchanged. Both producers, strict comparison and full CMake pending.
