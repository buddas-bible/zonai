# Island / Sleep / CCD integration re-audit

Base: Zonai master `d3cee81179850a97fd23d54175343d24f6558cb9` after Part 6.
Reference: Box2D main `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`, fetched 2026-10-05.

## Findings and scope

No additional production correctness change was required in the reviewed integration paths. Preserve the current scalar/stable-slot implementation; add Release-active regression coverage for its island, sleep and CCD boundaries. This is a bounded integration audit, not a proof that every continuous trajectory is detected.

- `BuildIslands` uses active non-static bodies as nodes and pointCount > 0 contacts as edges, including speculative constraints. Empty manifolds/free contact slots are ignored. Static bodies contribute constraints without joining otherwise independent islands. Deterministic body/contact slot order and prefix sums produce contiguous solver spans.
- `WakeBodyByIndex` / `SleepBodyByIndex` traverse only active contact edges, stop propagation at static nodes, and handle a mutated static body's active neighbors separately. Explicit sleep clears motion and forces. Disabling body sleep wakes its connected island; disabling world sleep wakes current non-static bodies.
- `WakeSleepingBodiesFromContacts` runs before island construction, ensuring an active non-static contact cannot cross an awake/asleep boundary. New manifold activation wakes the connected bodies; zero-point persistent pairs do not join islands.
- `UpdateIslandSleepStates` checks every member's linear/angular point speed plus delta correction speed (0.5 weighting, matching Box2D); one active/sleep-disabled member keeps the island awake. Creation's explicit initial sleeping state remains separate from automatic world sleep, as in Box2D body.c. No new world-level override is added.
- `Step` accumulates translation/rotation across substeps, uses external force through all substeps, commits the final/clipped sweep once, then clears deltas and consumes force once. Zero-duration Step updates collision/sensor state without integrating and clears transient fast/TOI flags under the existing Zonai contract.
- CCD classifies moving Dynamic bodies by maximum point motion relative to safetyFactor/minExtent. Regular fast bodies inspect static geometry; bullets also inspect kinematic/dynamic candidates and skip other bullets. Sensor geometry does not clip body motion.
- Non-bullet CCD precedes pending final proxy bounds; bullet queries run after those bounds are in the trees. Candidate TOI uses both body sweeps in Zonai. The final hit clips delta translation/rotation; only lost gravity contribution is corrected, matching Box2D's stated force/torque limitation.
- Sensor crossings are buffered until the nearest solid fraction is known and then discarded at/after that boundary. Earlier transient crossings publish begin state and disappear on a later sensor update if final geometry does not overlap.

## Intentional implementation differences and limits

Zonai rebuilds a union-find island graph each Step and traverses active contacts on wake/sleep; Box2D uses persistent islands and solver sets. The current approach uses temporary O(body/contact count) storage and repeated traversal during mutations. No duplicate graph cache, scheduler or solver-set scaffolding is justified by this correctness audit.

Zonai exposes fast classification even with CCD disabled; its existing API/tests use that diagnostic independently of clipping. Both implementations deliberately exclude bullet-bullet continuous detection and sensor-only moving shapes from solid CCD. Candidate trees store current/final fat bounds, not the union of every target trajectory; the tests below cover the supported target bounds-finalization path, not all possible crossing target sweeps. Feature expansion or broad-phase trajectory guarantees require a separately specified change.

## Verification

- Converted existing islandTests assertions to runtime checks with source-location diagnostics, active in Release.
- Added islandSleepCcdTests: three-body sleep/wake chain and unrelated island; static wake boundary and mutation; one sleep-disabled member; world sleep disable; 1/4/8 substeps with force consumption and clean next-step translation/rotation; static CCD enabled/disabled with 1/4 substeps and zero-step flag reset; dynamic/kinematic moving targets in both creation orders; sensor crossings before/after solid TOI in both candidate creation orders.
- Mutation: temporarily skipped fast non-bullet bounds finalization; the new test failed with `bullet missed moving target after bounds finalize`. Restored the exact original world.cpp before the green run. No mutation is committed.
- Full local Debug and Release builds/tests passed 36/36 in each configuration. Independent final review and Ubuntu/Windows CI are recorded in the PR.
- Windows Release CI also runs islandTests and islandSleepCcdTests alongside existing Part 4/5/6 runtime targets.

Next recommended scope: public world query / sensor API re-audit, especially filters, callbacks, event lifetime and query result limits. Keep the preserved debug-timeout branch separate until its history is intentionally archived or integrated.
