# R3r — full-range rotation migration

Base: [R3q #358](https://github.com/buddas-bible/zonai/pull/358), `eeeecbcc4aca6d028e9fa500713a49db7a8ab342`.

## Verified baseline

[R3q strict CI](https://github.com/buddas-bible/zonai/actions/runs/45408526380), job 138498096703: both platform producers and required strict comparison passed. Engine-owned math 22,716/22,716, Fluid 3,725,060/3,725,060 across all eleven scenes (including 180-tick World/Fluid and both short Joint/Fluid scenes). Contact lifecycle 217,173, Contact/Island 389,049 and standard Joint 641,266 rows exact; Spline physics exact, 222 diagnostic differences. Separate external libm research gate remains red.

## R3r changes

- Simplify `rot2::FromRadians` to call `deterministicSinCos` for every input. No standard-library sin/cos fallback, no angle clamp, no signature or c/s storage change. Retain exactly the bounded reduction and polynomial for small finite angles. Nonfinite inputs now return two canonical quiet NaNs (previous NaN payload/sign and libm side effects are not preserved).
- Require the production rotation factory to match the exact full-range candidate fixtures, including FLT_MAX, and all 4,032 exponent-stratified samples and their opposites. Require canonical invalid bits for all three existing invalid fixtures. Remove the old same-platform assertion that large-angle outputs equal the legacy float libm outputs: numerical accuracy remains independently checked against double libm and fixed high-precision references, and the strict fingerprint remains exact. Existing bounded cases and physical-state gates remain.
- Extend the source recurrence audit from 180 to 360 ticks, 1,440 intervals, approximately 6 simulated seconds. Preserve original 720 interval outputs and the three exact R3p witnesses. Check bounded compatibility before crossing 2*pi, full-range dispatch and numerical sine reference throughout, and exercise both sides of the old limit. Append 2,880 authoritative math rows (25,596 expected total), and 1,440 external diagnostic rows. This is the reconstructed source recurrence, not a new six-second World/Fluid physics scene.
- Re-pin temporary Release A/B to R3q. Existing short benchmark tests small-angle overhead only, not large-angle throughput or 6-second stability.

## Acceptance and remaining work

Review both platform math/physics traces and full CMake; compare the prior bounded results and all eleven physics scenes. Verify the 360-tick source recurrence and informational Release A/B. Then separately extend actual World/Fluid replay beyond 2*pi, identify any solver or boundary failures, and continue longer-duration tests. Full-range binary32 is supported by this candidate; correct rounding for every possible input is not proved. Other direct libm callers, target FP environment, extra architectures, and multithreaded determinism still require validation. No solver equation or source strength/time changes, no merge to master, no experimental #326 changes.
