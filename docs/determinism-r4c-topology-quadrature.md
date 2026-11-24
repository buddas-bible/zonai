# R4c — endpoint-refined quadrature for closing cells

Base: [R4b #369](https://github.com/buddas-bible/zonai/pull/369), `5c89efb27e04c49698f11e551aaa0db539b86b78`.

## Evidence and blocker

[R4a required strict CI](https://github.com/buddas-bible/zonai/actions/runs/45419954708) passed 7,680,160 Fluid/diagnostic rows and 25,596 math samples exactly. [R4b Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45420654467) failed before trace upload: interval 44 changed volume missed by midpoint quadrature; Windows and required comparison skipped. Historical interval 43 with 32 midpoints: source and compatibility L1 5.34948627e-8 m² magnitude, corrected volume residual L1 1.75308644e-11 m², independent identity error 1.89912825e-19 m². Velocity-preserving handoff cell residual L1 3.54614317e-5 m²; transfer-preserving handoff peak 31.8971569 m/s. No closed-end faces at interval 43. Interval 42 solved and ledger identity passed. These are independent frozen tests, not candidate-driven simulations.

## Change

Retain the exact R4a and R4b uniform midpoint arithmetic on previously successful inputs. For interval 44:
- First reproduce the original 64-midpoint missing-source-cell support set using geometry-only evaluations. Require it to be nonempty. Record each missed cell index, endpoint fractions, refined mean fraction and total mean effective aperture.
- Subdivide the first and last uniform panels dyadically toward each endpoint, with all other panels unchanged. Use 12 and 16 refinement levels (88 and 96 panels). Each midpoint lies strictly inside the interval and carries its actual panel width; require weights sum to one. Use the same weight for effective aperture, fraction and body coupling coefficients. No arbitrary positive aperture floor, endpoint-only weight, deleted source, or frozen endpoint replacement.
- Require the missed cells to regain positive fraction and actual open-face support, and retain failure on any changed volume without support. Continue existing pressure/volume/handoff checks unchanged. The second resolution is an independent comparison, not evidence that quadrature has converged.
- Keep prior 140 planned R4b diagnostic rows, plus topologyQuadrature 55 + 8 per missed cell (second-resolution 49 outputs, two headers of three fields each, four fields per missed cell per resolution). Strict cross-platform comparison required.

## Acceptance

Inspect both resolution results: recovered geometry, volume residual, compatibility removal, pressure/force changes, closed-face transport and maximum velocity. A quadrature repair is not yet a robust topology algorithm: transient cells away from endpoints can still be missed and tiny floating-point geometry may be unreliable. If support is still missing or pressure/volume checks fail, retain the blocker. Do not reinterpret integrated transfer as endpoint velocity; R4b already demonstrated that this can reproduce the spike. A separate interval transport ledger and endpoint velocity state still need a conservative tracer and live coupling contract. No production src modifications, new defaults, clamps, pooling, or master merge. #322/#326 untouched.
