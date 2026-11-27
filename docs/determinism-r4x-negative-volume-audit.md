# R4x — minimum interval-ledger volume audit

Base: [R4w #390](https://github.com/buddas-bible/zonai/pull/390), f74e9fd2d8b012ec236dd669eca08cbe3ef3ecde.

[R4w Ubuntu producer](https://github.com/buddas-bible/zonai/actions/runs/45460030702/job/138679780537) and Release timing passed. Windows replay, exact comparison and CMake were pending when checked. R4v full CMake passed.

All sixteen forward trials completed with zero unsupported cells/faces, reconstructed copy RMS approximately 2.63e-8–3.42e-8 /s and peak below 0.872 m/s. Actual endpoint volume L1 ranges from 5.84e-7 to 1.58e-6 m². However interval57 has negative modeled volume 2.55469957e-9 m². Do not promote the model or extend its duration before localizing that defect. A nonnegative analytic endpoint does not make a signed transport ledger physically feasible.

The reconstruction momentum proxy is negative in x and positive in y in every interval; the energy proxy increases in every interval. This is a systematic effect of reconstruction in this fixture, not a measured total momentum or energy violation. At interval57 the correction proxy is (-7.87422983e-4, 2.71175413e-4), while applied body pressure impulse is (2.19463224e-3, -6.48179451e-4). The actual body change agrees closely with its applied impulse. These are distinct diagnostics; no closed momentum budget is established.

## Localization

Retain the sixteen-interval trajectory unchanged. For every committed interval, select the minimum signed ledger volume among interval-supported cells (first row-major cell on ties). Count and sum all negative volumes independently and check equality with the existing defect diagnostic. Record start, predicted and actual volumes, signed ledger, body-response volume, removed compatibility volume and linear volume residual. The initial pressure solution's decomposition is valid here because correction budget is zero. Independently reconstruct the selected cell's net transfer from its four signed face transfers, recording start, mean and actual openings, solved interval velocity and inward transfer. Validate both local identities using the existing 1e-12 absolute identity scale; do not clamp or redistribute a negative volume. Report every interval and detailed face values on a negative-volume interval. This minimum-cell audit does not characterize every other cell or chronological sub-panel donor availability.

Add 33 scalars per interval (528 total), expected 7,919,451 Fluid rows. Preserve prior traces, numerical settings, trajectory and benchmark variants. Both producers, strict comparison, full CMake and numerical review required. Based on the decomposition, distinguish geometric response error, compatibility removal, linear solve residual and signed transport feasibility before choosing a correction. No production code, defaults, tolerances or master changes.
