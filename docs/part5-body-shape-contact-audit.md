# Part 5 Body / Shape / Contact re-audit

Reference: Box2D main `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`, fetched 2026-10-05.
Base: Zonai master `21ffa629ef35b1b9c81cb18a7f4a29c2c14882d4` after Part 4.

## Verified corrections

- Reuse reviewed `body-shape-contact-reaudit` production changes: public body/shape/contact IDs carry an independently allocated world lifetime token. Slot/generation alone allowed foreign-world IDs; an object address alone allowed IDs from a destroyed world to revive at the same address.
- Disable world copy/move to avoid duplicating its identity. Sensor overlap/event handles carry the token, including destroyed visitor history.
- Body contact queries include speculative manifolds, matching shape queries. Box2D body.c/shape.c test its touching flag; contact.c assigns that flag's simulation state from `pointCount > 0`. This does not mean separation must be non-positive. The existing actual-touch collision callback remains separate.
- Correct the inherited audit plan's contrary speculative-query assumption and stale query comments. Use lowerCamelCase for the new token allocator.

## Reviewed invariants; no additional algorithm change needed

- body/bodySim/bodyState retain cold identity/list state, simulation properties and hot motion separately. Stable parallel slots are deliberate until solver sets are implemented.
- Shape's geometry/filter/material/proxy/list fields stay together, as in current Box2D. No unused shapeDef, contact-event flags or new abstraction is needed.
- Creation increments generation; freeing preserves it and clears simulation state. DestroyBody removes contacts, shapes and proxies before returning the body slot.
- Shape doubly linked lists repair both neighbors/head and counts. Contact keys encode contact index and edge side; destruction repairs both body lists and removes the persistent pair key.
- ContactSim reset prevents impulses and recycle anchors leaking into a reused contact slot. Warm-start persistence within the same contact remains unchanged.
- Runtime filter mutation removes affected contacts immediately and touches the proxy; restoring the filter recreates stationary pairs. Transform changes update bounds and remove/recreate pairs. Zonai does not store category bits in tree nodes, so Box2D's category-proxy recreation is unnecessary here.
- Sensor destruction swaps storage and repairs the moved sensor index. Begin/end/overlap handles keep ownership and visitor generation.

## Regression coverage and verification

`bodyShapeContactLifecycleTests` uses runtime checks active in Debug and Release: foreign body/shape/contact IDs, reconstruction at the same address, body/shape/contact slot reuse, shape middle/head/tail deletion, all six contact edge deletion orders on both bodies, cascading proxy cleanup, stale force/velocity cleanup, nonzero impulse reset, stationary refilter, transform mutation, speculative and zero-point queries, sensor begin/end/history identity.

RED: on the baseline master, the new test fails with `foreign body accepted`. The reviewed ownership changes make it pass. Existing bodyTests also contain compile-time copy/move restrictions and speculative-query regressions.

Local Debug and Release build/test: 34/34 passed before final review; final validation and CI are recorded in the pull request.

The old 16-bit body/shape generation wrap behavior is unchanged, matching Box2D's finite-generation handle convention. The world token similarly has a theoretical 64-bit wrap boundary; it is not an unlimited-lifetime proof.

Part 6 solver implementation/tests are excluded. Next: rebase the preserved contact-solver audit onto merged Part 5, reproduce its restitution regression, then review solver behavior independently.
