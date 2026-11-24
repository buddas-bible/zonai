# R3n — joint constraints in actual World/Fluid interleaving

Base: [R3m #354](https://github.com/buddas-bible/zonai/pull/354), `278d52e0f6433ebdc35f9b8d2283ff22143e1324`.

## Verified baseline

[R3m strict CI](https://github.com/buddas-bible/zonai/actions/runs/45405137059): 2,069,914/2,069,914 Fluid rows identical across Windows/Ubuntu, including 186,339 rows each for actual pressure-only and pressure+drag World/Fluid scenes. Engine math 11,728/11,728. Required World/Contact/Joint gate passed; external libm research gate remains red. [R3l full CMake](https://github.com/buddas-bible/zonai/actions/runs/45404039511) passed both OS. [R3m full CMake](https://github.com/buddas-bible/zonai/actions/runs/45405136993) is tracked separately.

## Scope

- Append two scenes to the existing eight: worldFluidRevolute and worldFluidWeld. Reuse the actual Sandbox step path, R3m field, source, initial motion, 12 ticks, 8 World substeps and 4 Fluid intervals. Enable two-way pressure and drag in both scenes.
- Connect the first two dynamic obstacles (Circle and Capsule) at their initial world midpoint using coincident local anchors. Passive Revolute allows relative rotation; hard Weld constrains relative pose. Keep the third body and all original fluid geometry. No motor or new coupling algorithm.
- Reuse public Joint reaction/anchor/angle queries, persistent Joint cache snapshot and test-only World substep recorder. Compare observed and independent unobserved frame state, including Joint cache and reaction, for all ticks.
- First two ticks record all 13 actual World solver stage records per interval (Prepare, five phases per substep, Restitution, Store), 52 records per tick, and validate stage/index order. Preserve existing Fluid phase 0/9 observations. World substep traces are appended as a separately labeled block after the frame's Fluid interval traces; displayed first mismatch is not necessarily chronological across the blocks.
- Require actual Joint creation, nonzero Joint reaction, analytic coupled pressure/IC0, finite state and nonzero pressure/drag exchange. Initial relative motion contributes to Joint reaction too; this is a combined-path test, not an isolated measurement of the force transmitted by the fluid alone.
- No production source, equations, tolerances or existing trace keys changed. Existing eight scene coverage retained; strict gate still rejects any physical bit difference.

## Acceptance and follow-up

Both trace jobs and full CMake must execute successfully; all prior scenes must remain exact. Inspect new scene results and isolate the first physical-stage divergence if any. This checks two Joint kinds under one short coupled scenario each; not every Joint, multi-body joint network, contact-rich scene, large angle, architecture or long-run stability. Bounded rotation legacy fallback and direct libm callers remain future work. No merge to master; experimental #326 untouched.
