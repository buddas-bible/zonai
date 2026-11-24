# R3y — sliver flux feasibility and sampling propagation

Base: [R3x #365](https://github.com/buddas-bible/zonai/pull/365), `41ad74d75b375714763d69032d24fc5da3b25b0d`.

## Verified evidence

[R3w required CI](https://github.com/buddas-bible/zonai/actions/runs/45416625128) passed 7,663,335 Fluid rows and 25,596 math rows exactly. The historical near-closed V(17,13) spike is reproduced. [R3x Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45417474819) passed; Windows strict comparison pending at preparation.

| Source | Assigned fraction change | Pressure jump | Peak speed (m/s) |
|---|---:|---:|---:|
| Swept | -5.67354728e-5 | 1657.30798 | 28.132021 |
| Snapshot | -5.68079158e-5 | 1659.4408 | 28.1675682 |
| Corrected swept | -5.67894895e-5 | 1658.91248 | 28.158762 |

All three had zero donor relocation into the target cell. The current fraction is 1.8302887e-9, previous 5.68097457e-5. Baseline velocity divergence 0.000228551769/s; source divergence -0.0136165125/s. Endpoint differencing and existing local reconciliation do not resolve this spike. Neither is adopted.

## New audit

- For the exact same three frozen inputs, reconstruct the upper cell's four signed open-face fluxes and predicted solid fluxes from production fields. Use effective apertures, not raw open fractions with solid neighbors. Record pre/post flux, assigned source, required net flux from the compatibility-adjusted pre-projection divergence and computed solid response.
- For fixed predicted solid response and source, let A=sum(open fractions), Q=required outward face flux in velocity units. The triangle inequality gives max(|u|) >= |Q|/A. The Cauchy–Schwarz minimum of sum(a*u²) for a given observed net flux is Q²/A. These are local feasibility diagnostics, not a global pressure solution, a proposed physical limit or a guarantee when solid response changes. Record observed bound, actual peak and flux closure error separately. Double reconstruction can differ slightly from production float evaluation.
- Reconstruct the actual row L*p and Schur term using stored pressure and the assembled boundary coefficients. Aggregate the coefficients per owner before evaluating the local Schur diagonal, to include cancellation of opposite faces. Record RHS, local diagonals, row terms and residual; the local diagonal is not an estimate of the global matrix condition number.
- Compare legacy bilinear sampling with a read-only aperture-normalized candidate based on experimental [#322](https://github.com/buddas-bible/zonai/pull/322), using the current deterministic interpolation for fully open stencils. Evaluate nine points between the two adjacent cell centers, including the exact face center, on the identical baseline post-projection field. No clamping, no writes to MAC arrays, no change to advection or drag. Zero-support stencil returns zero, as in #322; this is not a general boundary interpolation guarantee.
- Preserve all historical rows. Require exactly 162 new strict diagnostic rows (3*39 local budget + 9*5 sampling). R3x source audit remains required. No production edits or solver/tolerance changes.

## Acceptance and next decision

Review R3x Windows and both R3y producers, strict comparison and full regression. If the conditional lower bound is close to the actual spike, simply capping velocities or improving PCG cannot preserve the existing local continuity constraint. Evaluate a conservative space-time or grouped constraint treatment separately; historical #326 pooling did not establish an accepted remedy. If the same-state aperture-weighted sampler reduces off-face propagation, treat it only as a candidate for limiting propagation, not a pressure stabilization fix. Validate on closing/opening topology, force/torque and residuals before any production migration. Master, #322 and #326 unchanged.
