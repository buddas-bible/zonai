# R3i — Swept rotation first-divergence isolation

Base: [R3h #349](https://github.com/buddas-bible/zonai/pull/349), `bca7491856e3ab9ab179baf7dabdad9d25a7ef56`.

## Observed R3h evidence

[Strict CI 45378469444](https://github.com/buddas-bible/zonai/actions/runs/45378469444): 55,000 mismatches in 1,262,940 Fluid rows. Both original Circle fixtures remain identical (312,474 rows); math 1,434/1,434 matches. [CMake](https://github.com/buddas-bible/zonai/actions/runs/45378469384) passed on Windows and Ubuntu.

| Scene | First differing field | Ubuntu | Windows |
|---|---|---|---|
| sampledMixed | tick.4.u[2] | bc9d5adf | bc9d5ae2 |
| analyticCapsule | tick.3.u[1] | bc032154 | bc03215c |
| analyticPolygon | tick.2.stage.3.sweptUVolume[162] | 38b68616 | 38b68764 |
| coupledDisjoint | tick.5.u[2] | bcad2866 | bcad2865 |

Polygon is the earliest **observed stage-level** mismatch, not necessarily the cause of all four scenes. Prior end snapshots cannot exclude cancelled intermediate differences.

## R3i test changes

- Extend the ten-stage trace through each new scene's earliest previously failing tick (4, 3, 2, 5 respectively). Preserve original Circle traces and all per-tick checks.
- At phase 3 (after actual swept integration/reconciliation, before remap), only when the real solver reports swept history executed, reconstruct the exact float angle extraction from retained previous and current obstacle transforms. Record identical operands, standard-library atan2 start/end, remainder, angular and linear speed, first intermediate orientation using the actual selected sample count, and the existing engine deterministicAtan2 candidate outputs.
- Report first swept-reconstruction mismatch separately from first non-swept-probe trace mismatch. Every mismatch still fails the strict comparison. New diagnostic rows and extended stages change the row count; do not compare mismatch percentages as if the trace coverage had not changed.
- No production source modifications. The probe repeats expressions, not actual internal locals; agreement with actual swept output is not asserted and should not be inferred. A matching pattern across probe and stage traces is evidence to investigate, not proof that every divergence is caused by atan2. No changes to libm classification or strict physics comparison.

## Follow-up

If the same rotation operands produce different atan2 results and actual swept volume subsequently differs, try the already tested engine deterministicAtan2 in the two swept rotation extraction calls in a **separate** candidate. Retain the observed cases and compare exact stage traces, angle wrapping and boundary cases, complete CMake and coupled-scene Release A/B. If the first mismatch is elsewhere, isolate that operation instead. Do not change the Fluid model, relax the gate or claim full World/Fluid determinism.

Master, prior drafts and experimental Fluid #326 remain untouched. Historical R3g A/B is slated for cleanup/generalization at final integration.
