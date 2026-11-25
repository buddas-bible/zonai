# R3k — bounded deterministic sine/cosine contract

Base: [R3j #351](https://github.com/buddas-bible/zonai/pull/351), `35812673d88a10d7eaf9e0d6ffb6c7aef2fbe29d`.

## Verified R3j evidence

[Strict CI](https://github.com/buddas-bible/zonai/actions/runs/45401846899): 14,474/1,697,236 differences, down from 61,148 at R3i with the same row coverage. Both Circle fixtures, analyticPolygon and coupledDisjoint now agree exactly; the remaining scenes are sampledMixed (5,979) and analyticCapsule (8,495). All 1,438 engine math rows agree. [Same-runner Release A/B](https://github.com/buddas-bible/zonai/actions/runs/45401846930) passed: zero nonfinite faces and PCG failures in both versions, maximum paired face-speed delta 0.00366 m/s and continuity RMS delta 2.6e-7. At h=0.25, eight Fluid substeps, swept median 0.0390→0.0397 ms, total field 7.0559→7.0726 ms. Timing is informational; existing small-face velocity peaks remain.

Downloaded and compared both R3j raw artifacts. End-of-tick rotation snapshots reveal identical input angle, position, velocity, omega, and cosine, but a one-ULP sine difference:

| Scene | Input radians bits | Ubuntu sine | Windows sine | Rounded high-precision sine |
|---|---|---|---|---|
| sampledMixed tick 8, body 1 | 3e0d4fdf | 3e0cdd29 | 3e0cdd28 | 3e0cdd28 |
| analyticCapsule tick 7, body 0 | 3e67983b | 3e65a014 | 3e65a013 | 3e65a013 |

This implicates the fixture's `rot2::FromRadians` sine evaluation; it does not prove that every remaining difference comes only from that operation. Swept intermediate rotations and other production call sites also use FromRadians. Prior end snapshots match through tick 7/6 respectively; stage traces currently stop earlier.

## R3k scope

- Introduce `deterministicSinCosBounded` for finite binary32 angles within ±float(2*pi), including signed zero. Out-of-domain/nonfinite inputs return two canonical qNaNs. This is a candidate, **not a replacement for the arbitrary-angle rot2 API**.
- Evaluate in binary64 with nearest-quadrant reduction, split pi/2 constants, Taylor sine through degree 17 and cosine through degree 16, fixed Horner order on the reduced interval. Existing noncontracting FP policy and round-to-nearest remain mandatory.
- Add 19 exact cases (including both historical failures, signed zero, subnormal inputs, quadrant boundaries), 5 invalid cases, 4,097 uniformly spaced angles and 1,024 seeded raw-bit inputs. Require exact fixtures, symmetry, finite range, norm and numerical agreement with double libm within the test tolerance. The strict fingerprint compares the candidate results bit-for-bit, without tolerance; 10,290 new authoritative fields, 11,728 total expected. Historical float sin/cos witnesses are separately retained under libm diagnostics.
- No changes to rot2, production Fluid, scene inputs, or strict comparison. Full World/Fluid coupling expansion waits until the remaining two fixtures have been resolved.

## Next acceptance

Validate the candidate independently on Windows and Ubuntu. Before replacing FromRadians, account for its arbitrary finite-angle contract and all engine call sites; bounded swept-angle use does not authorize restricting the global API. Use a separate migration candidate, repeat all six scene traces, full regression and a same-runner Release comparison. If necessary, extend stages around the first differing tick and identify additional operations. No master merge or changes to experimental #326.
