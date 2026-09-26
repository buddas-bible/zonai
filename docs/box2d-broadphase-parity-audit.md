# Box2D BroadPhase / DynamicTree parity audit

Reference upstream:

- Repository: `erincatto/box2d`
- Branch: `main`
- Commit: `956ce4e1e8acddd05a21102629e268ff9321ec50`
- Compared areas:
  - `src/dynamic_tree.h`
  - `src/dynamic_tree.c`
  - `src/broad_phase.h`
  - `src/broad_phase.c`
  - `src/table.h`
  - `src/table.c`
  - AABB validation helpers
  - relevant upstream dynamic-tree / hash-set tests

This audit is focused on behavior already implemented in Zonai. Missing future features are
listed separately so they are not confused with accidental omissions.

## Correctness / safety omissions found and fixed

### Empty-node sentinel AABB

Box2D empty nodes are leaf-tagged sentinels whose AABB is deliberately inverted:

- lower = `(+inf, +inf)`
- upper = `(-inf, -inf)`

This guarantees that an empty root never overlaps a valid AABB.

Zonai previously left the empty AABB zero-initialized, which allowed an empty tree at the
origin to participate in BroadPhase pair generation.

Fixed by restoring the inverted-infinity sentinel AABB. BroadPhase `TestPair` also rejects
empty nodes explicitly as an additional invariant guard.

### AABB input validation

Box2D validates finite, ordered AABBs on proxy creation and movement and constrains moved
AABB extents to the single-precision broad-phase range.

Zonai now:

- validates finite / ordered AABBs,
- validates moved AABB width / height against `1.0e5f`,
- tests inverted, infinite, and NaN AABBs.

### Proxy <-> leaf mapping

Box2D stores the shape index both in proxy user data and in the leaf, then validates that
both views agree.

Zonai now mirrors the shape index in `TreeProxy::userData` and validates:

- proxy id range,
- proxy -> node mapping,
- node is a leaf,
- leaf -> proxy id mapping,
- proxy user data == leaf shape index.

Destroy, move, validation, and proxy-AABB lookup paths check these invariants.

### Free-list validation

Box2D validates the sibling-pair free list and proxy free list, including their counts.

Zonai `Validate()` now checks:

- sibling pair index is even and in range,
- both nodes in a free pair are empty,
- free-pair traversal cannot cycle indefinitely,
- free proxy indices are in range,
- free proxies have no live node,
- live proxy count + free proxy count == proxy capacity,
- node storage count matches live proxies + free pairs.

### Fixed-stack writes

Some Box2D tree traversals check capacity before writing to a fixed stack so release builds
cannot write past the array even when assertions are disabled.

Equivalent guards were added to implemented Zonai paths:

- DynamicTree query,
- moved-flag clearing,
- BroadPhase proxy-vs-subtree traversal,
- BroadPhase subtree-pair traversal.

Rebuild stack sites for which upstream itself is still assert-only are listed under
"Upstream limitations retained" below.

### BroadPhase moved-sibling scratch

Box2D sizes moved-sibling scratch storage internally from the current dynamic tree.

Zonai previously required the caller to provide a span of sufficient size. This created an
avoidable external precondition.

BroadPhase now owns a persistent `std::vector<std::int32_t>` scratch buffer and resizes it
from the dynamic-tree node count. The vector is reused between updates.

### SAH sibling tie-break

Current Box2D resolves equal lower-bound SAH costs by comparing subtree-centroid distance
to the inserted leaf center.

Zonai was missing this tie-break. It has been restored.

### Rotation bookkeeping

The final `C <-> E` rotation candidate in Zonai selected `bestDown` / `bestUp` but did
not assign `bestDelta`.

The current code now matches all four Box2D rotation candidate updates.

### Empty-root insertion

Box2D centralizes the empty-root case inside leaf insertion.

Zonai previously duplicated the first-leaf special case in CreateProxy and MoveProxy.
The handling is now centralized in `InsertLeaf`, reducing the chance that a future
insertion path bypasses it.

### Area-ratio metric

Current Box2D defines tree area ratio as:

`sum(non-root internal-node perimeters) / root perimeter`

Zonai previously included root and leaf perimeters. The metric and tests now use the
current Box2D definition.

### Tree-node memory layout and alignment

Current Box2D deliberately uses:

- 32-byte `b2TreeNode`,
- sibling nodes at consecutive even/odd indices,
- 64-byte aligned tree-node storage,

so a sibling pair occupies one 64-byte cache-line-sized block.

Zonai now:

- keeps `TreeNode` at 32 bytes with the same 8-byte padding slot,
- compile-time checks the node size,
- stores nodes in `std::vector` with a 64-byte aligned allocator,
- tests the actual storage address alignment.

`TreeProxy` remains 16 bytes intentionally because Zonai has not yet moved category bits
into tree proxies. This is a feature difference, not an accidental layout mismatch.

### HashSet invariants

Box2D reserves key zero as the empty-slot sentinel.

Zonai keeps the same rule and now has stronger tests for:

- growth,
- deletion/probe-chain repair,
- large keys,
- repeated add/remove/contains operations against `std::unordered_set`,
- many pair-shaped keys.

BroadPhase candidate generation also asserts that candidate shape indices are non-negative
and distinct before a pair key is formed.

## Existing invariants verified as already equivalent

The following implemented behavior was rechecked against current Box2D and did not require
a correctness change:

- root index is zero and node one remains empty,
- sibling pairs begin at even indices,
- stable proxy ids are independent of movable node indices,
- internal-node AABB is the union of its two children,
- internal height is reconstructed from child heights,
- moved state propagates from children to internal nodes,
- DFS ordering state is invalidated by rotations/insertion when necessary,
- remove-leaf promotes the surviving sibling and frees the old pair,
- rebuild preserves stable proxy ids,
- partial rebuild expands moved branches and retains untouched subtrees,
- median partition fallback prevents an empty side,
- dynamic self-pair traversal collides sibling subtrees without duplicate self pairs,
- dynamic/static and dynamic/kinematic cross-seed traversal follows the same moved/overlap rule,
- static moved state is cleared after pair generation,
- dynamic and kinematic stale trees are rebuilt after pair generation,
- zero remains reserved by the custom hash set.

## Test coverage added during the audit

DynamicTree tests now include:

- 64-byte node-storage alignment,
- brute-force comparison of tree Query results against 200 proxy AABBs,
- proxy movement followed by brute-force Query comparison,
- full rebuild followed by brute-force Query comparison,
- checking that rebuild clears moved flags on every live node,
- repeated destruction with Validate after each operation,
- complete free-list reuse by reinsertion with Validate after each operation.

BroadPhase tests include the regression case that originally exposed the sentinel bug:

- Dynamic shape index 0 spanning the origin,
- empty Static tree,
- empty Kinematic tree,
- no candidate pair may be produced.

AABB tests cover invalid ordering, infinities, and NaN.

HashSet tests include reference comparison with `std::unordered_set`.

## Implemented paths that intentionally differ from current Box2D

These are not classified as omissions because the corresponding Zonai subsystem/API does
not exist yet:

- DynamicTree proxy category bits and category-mask Query pruning,
- EnlargeProxy,
- RayCast / ShapeCast / box-cast tree APIs,
- public moved-mark / gather / refit workflow used by the solver,
- tree root-bounds / byte-count / detailed TreeStats APIs,
- parallel pair tasks and worker-local pair-key arrays,
- final pair-key sorting before Contact creation,
- ShapeType-level `CanCollide` filtering,
- joint-based body collision overrides,
- user custom collision filtering.

When these features are implemented, their Box2D preconditions and validation code should
be ported at the same time as the algorithm.

## Upstream limitations retained

Current Box2D still has a few fixed-stack sites in rebuild/copy/refit code marked with
assertions or TODOs rather than full release-mode recovery. Zonai does not classify those
as missed Box2D safeguards because upstream itself does not currently provide a stronger
fallback.

Likewise, recursive validation can overflow if the tree is already pathologically corrupt;
Box2D explicitly treats that failure as validation evidence.

## Rule for future Box2D ports

When translating a Box2D subsystem, treat all of the following as part of the algorithm,
not incidental implementation detail:

1. sentinel values and sentinel geometry,
2. asserts / validation checks,
3. special-case branches,
4. duplicated state used for cross-validation,
5. memory size / alignment assertions,
6. free-list invariants,
7. scratch-buffer sizing,
8. deterministic tie-breaks,
9. debug tests that compare against a brute-force/reference implementation.

Do not port only the main mathematical path.
