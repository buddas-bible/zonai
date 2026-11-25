# R4g — fixed-graph geometric defect correction

Base: [R4f #373](https://github.com/buddas-bible/zonai/pull/373), `491d71f2f6297baca8eaece48927caad64f243e4`.

## Evidence

[R4e required CI](https://github.com/buddas-bible/zonai/actions/runs/45423393112) passed 7,735,223 Fluid/diagnostic rows and 25,596 engine math rows exactly. [R4f Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45424189419) succeeded: index 43 midpoint32 original geometry gap L1 2.78575635e-5 m², linear body contribution L1 2.78056679e-5, compatibility 5.34948627e-8, corrected residual 1.75308644e-11, identity error 1.9069046e-17. Applying the model free-body displacement reduced actual geometry gap to 6.10253066e-7 m², or 5.58614548e-7 after accounting for compatibility. The perturbed geometry closes one cell. Mean vs endpoint linearization errors were 5.58597358e-7 and 3.09155824e-7 m². Windows strict comparison pending. This is not actual World integration.

## Algorithm

For each of the five frozen solutions, retain the existing traces and start from the R4a pressure p. Freeze interval apertures, active graph, trial velocities, average B and inverse masses. Compute B^T p, rebuild analytic endpoint geometry using the same counterfactual dt² free-body displacement from the original endpoint (never accumulate pose perturbations across iterations). Independently calculate V_flux from previous volume and signed interval transfers and form E(p) = V_geometry(p) - V_flux(p).

- Approximate the derivative by dt² A, A=Lbar+Bbar M^-1 Bbar^T. Solve P A P delta_p = -P E / dt² using the existing matrix product and diagonal weights, with an independent reference PCG correction and true residual check. Component mean correction remains explicit; report raw and projected defects separately. No source deletion.
- Try deterministic step factors 1, 0.5, 0.25, 0.125, accepting only a reduction in both raw L1 and projected squared defect, analytic geometry and no volume defect outside the interval graph. If none is accepted, report stalled and retain the last accepted state; do not change tolerance, invent aperture support, or cap speeds. Initial unsupported geometry is an explicit failure. Up to three correction passes.
- Record four states per solution (initial and three passes), raw/projected volume errors, compatibility, negative flux-only volume, velocity peak, historical face speed, pressure force/torque changes, topology, accepted factor, trials and PCG accuracy. Require 340 new exact rows (5*4*17), all historical rows unchanged.

## Acceptance and limitations

Both producers, strict comparison and CMake regression required. Compare actual raw volume gap and speed/feedback changes. Backtracking guarantees accepted defect decrease, not physical convergence, nonnegative flux volume, bounded speed or uniform concentration. Fixed interval apertures and averaged B do not change when endpoint pose changes; the method is an approximate nonlinear defect correction, not a fully consistent space-time geometry/pressure solve. No corrected state is passed to tracer or World. The perturbation uses binary32 geometry, so precision limits may cause stalling. If correction is unstable or merely shifts the spike, reject it as a production candidate. Next investigate consistent interval path and actual World response or a simpler alternative based on observed results. No production src changes, defaults, speed/pressure clamps, or master merge. #322/#326 unchanged.
