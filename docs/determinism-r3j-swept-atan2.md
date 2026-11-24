# R3j — deterministic swept rotation extraction

Base: [R3i #350](https://github.com/buddas-bible/zonai/pull/350), `bb344d2f158091afc0df0e9441c2d02b481e14e8`.

## Verified evidence

[R3i strict run 45382050018](https://github.com/buddas-bible/zonai/actions/runs/45382050018) compared 1,697,236 Fluid rows, 61,148 mismatches (including reconstruction and extended stage rows). Both original Circle traces and 1,434 engine math samples remain exact; full CMake passed both OS. Independently downloaded both platform trace artifacts. In each of the four new scenes, the first reconstructed difference is a one-ULP std::atan2 output, with identical input bits and identical deterministicAtan2 candidate bits. The first non-probe difference is in Stage 3 sweptUVolume.

| Scene and tick | y bits | x bits | Ubuntu std::atan2 | Windows std::atan2 | engine candidate (both) |
|---|---|---|---|---|---|
| sampledMixed 4 end | 3dfd4d3c | 3f7e08ce | 3dfdf3b5 | 3dfdf3b6 | 3dfdf3b6 |
| analyticCapsule 3 end | 3e50f43a | 3f7a9d10 | 3e526e96 | 3e526e97 | 3e526e97 |
| analyticPolygon 2 start | 3ec9d0e7 | 3f6b45fe | 3ecf7201 | 3ecf7200 | 3ecf7200 |
| coupledDisjoint 5 end | 3e9a4d99 | 3f7418fc | 3e9cbd69 | 3e9cbd68 | 3e9cbd68 |

## Narrow migration

- Change only the two float angle extraction calls in `fluidField2::updateSweptSolidFlux` to existing `deterministicAtan2`.
- Preserve the remainder/shortest arc, time sampling, rotation construction, swept face integration, pressure, and all other physics code. The separate binary64 area atan2 remains unchanged.
- Mirror the production calls in the read-only reconstruction; keep historical candidate columns as aliases so field names and row counts remain comparable. Keep the four historical libm witnesses in the separate external FP diagnostic, with exact expected engine bits in the strict math fingerprint. No physical-state comparison is relaxed.
- Reuse the temporary A/B workflow with R3i pinned as baseline, rather than measuring the combined R3g and R3j change against R3f. Report swept, advection and field median timings, PCG failures, and 192-row coupled trajectory audit. R3g historical A/B remains archived at [45347667921](https://github.com/buddas-bible/zonai/actions/runs/45347667921).

## Acceptance

Both platform traces, full CMake and 1,438 engine math rows should pass; inspect all six Fluid scenes and first remaining mismatch if any. Compare against R3i on the same Release runner for performance and stability. This does not certify all rotations, all architectures, or actual World/Fluid/Joint cross-platform interleave. A successful narrow migration is causal evidence for the observed divergence, not a proof that every remaining libm path is safe. Do not merge until the candidate is validated; retain master and experimental #326.
