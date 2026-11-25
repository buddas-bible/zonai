# R4ac — precise disjoint capsule geometry

Base: [R4ab #395](https://github.com/buddas-bible/zonai/pull/395), 81f8b40a4f47e86cfea29eddfd7332b472453feb.

[R4ab required strict comparison](https://github.com/buddas-bible/zonai/actions/runs/45465217904) passed 7,919,929 Fluid rows and 25,596 engine math samples, zero mismatches. External libm retains 33 mismatches. Ubuntu CMake and Release timing passed; Windows CMake was pending when checked.

R4ab identifies body1, the capsule in createWindFlow, as the sole geometry contributor to the half-relaxation pass4-to-pass5 pressure jump. Body0 and body2 substitutions have zero effect. Capsule-only and all-body probes both produce volume L1 change 4.44973921e-7 m², signed volume change -1.01199475e-7 m², compatibility L1 change 1.01199475e-7 m², pressure max change 0.0124974251 and force max change 0.0162982978 N, without cell or aperture topology changes.

## Precision comparison

The existing capsule constructs transformed endpoints, unit axis and rectangle vertices in float before integrating in double. Add a private default-disabled disjoint capsule path that constructs these derived quantities in double, reuses the existing rectangle/cap area and face clipping algorithms, and conservatively expands the float broad-phase bound. Propagate the option through disjoint geometry, boundary coefficient, synchronization and swept-geometry construction. The default path retains its original float arithmetic; single-obstacle paths are unchanged. The private option is accessed only by the test friend. No public API or runtime setting is added.

Rebuild the common start and both actual endpoint estimates of the R4ab matched comparison with precise capsule geometry. Keep the historical incoming MAC velocities, source schedule, fixed-geometry weighted compatibility, quadrature and strict residual budget. Measure each ledger and the inter-estimate volume/compatibility, pressure and load differences. No probe load is applied, no trajectory replaced. World poses, output cell fractions, face openings and final forces remain binary32; rotation normalization is unchanged. The comparison does not isolate these remaining rounding sources or guarantee convergence. Verify the existing default traces are unchanged, not merely matching across platforms.

Add 19 exact rows; expected total 7,919,948 Fluid rows. Both platform producers, strict comparison, full CMake and numerical review required. No promotion, master merge or claim of repaired pressure convergence until results are reviewed. If the sensitivity remains high, prioritize a simpler bounded game coupling rather than adding repeated full pressure solves.
