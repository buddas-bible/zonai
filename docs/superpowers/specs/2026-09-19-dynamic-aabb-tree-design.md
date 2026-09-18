# Dynamic AABB Tree Design

## Goal

Implement Zonai's first Dynamic AABB Tree as an independently testable broad-phase data structure, while keeping the algorithm understandable enough to serve as the conceptual bridge from Zonai 2D to a future Zonai 3D implementation.

## Scope

This design covers only the Dynamic AABB Tree.

Included:
- AABB helpers required by the tree
- packed tree node flags compatible with the current Box2D/Box3D design direction
- stable proxy ids separate from node indices
- tree creation/destruction through ordinary C++ ownership
- proxy creation and insertion
- sibling selection using a greedy SAH-style cost
- ancestor refit
- node rotation/balancing
- proxy removal
- proxy movement by remove/reinsert
- AABB query
- validation helpers and focused tests

Excluded for this phase:
- BroadPhase pair generation
- body-type separated trees
- collision filtering/category bits
- fat-AABB policy owned by BroadPhase
- contact persistence
- speculative contacts
- ray cast / shape cast
- tree rebuild optimizations beyond the core incremental tree

## Conceptual Model

The tree is a binary BVH.

Leaf nodes represent object proxies. Internal nodes represent the union of the AABBs of their two children.

Unlike a quadtree/octree, the world is not partitioned into fixed spatial cells. Tree topology is chosen according to AABB cost so that nearby or compactly-grouped proxies tend to share ancestors.

Unlike a segment tree, hierarchy is not determined by a fixed index interval. The tree is reorganized dynamically as proxies are inserted, removed, and moved.

## AABB Policy

The DynamicTree receives AABBs from its caller and does not decide how much margin to add.

That keeps the tree generic:
- a caller may pass a tight AABB,
- BroadPhase may later pass an enlarged/fat AABB,
- MoveProxy simply replaces the leaf AABB and reinserts when requested.

## Node Representation

Keep the current packed flag direction:

- bit 31: leaf flag
- bit 30: moved flag
- bits 0..29: child-pair index for internal nodes, proxy id for leaf nodes

Internal nodes store height.
Leaf nodes store shapeIndex.

The existing `leafCount` union member is replaced with `height`.

Sibling nodes are stored as an adjacent pair beginning at an even index. This mirrors the current Box2D/Box3D data-oriented layout and lets the second child be found with `pair + 1`.

## Proxy Representation

Proxy ids are stable external handles.

A proxy stores:
- the current node index containing its leaf
- free-list linkage when unused

This deliberately separates:
- proxy id: stable identity used by callers
- node index: transient physical location inside the tree

## Ownership

`DynamicTree` owns its storage using standard C++ containers.

The implementation should preserve Box2D/Box3D's data-oriented concepts without copying C allocation APIs. Zonai remains idiomatic C++ while maintaining contiguous storage.

## Insertion

Insertion proceeds as follows:

1. Allocate a stable proxy id.
2. Create a leaf node for the proxy.
3. If the tree is empty, the leaf becomes the root.
4. Otherwise find the best sibling using greedy SAH-style cost.
5. Allocate an adjacent sibling pair.
6. Place the old sibling and the new leaf into that pair.
7. Replace the old sibling's previous location with a new internal parent.
8. Refit AABBs/heights toward the root.
9. Rotate nodes when a local alternative reduces tree cost.

The first implementation steps will intentionally stop at simpler subcases before introducing sibling search and rotation, so each structural rule is observable in tests.

## Sibling Selection

For a candidate leaf D, choose a sibling H by minimizing the growth of ancestor bounding cost.

In 2D the cost metric is based on AABB perimeter. This is the 2D counterpart of surface-area cost used by a 3D BVH.

The algorithm follows a single greedy descent with lower-bound pruning rather than exhaustively checking every leaf.

## Move

Movement is represented as remove + reinsert with a new AABB.

Fat-AABB containment checks are not part of DynamicTree itself. BroadPhase will later decide whether a movement requires calling `MoveProxy`.

## Query

AABB query traverses from the root:
- if a node AABB does not overlap the query, prune the entire subtree,
- if it is a leaf, report the proxy,
- otherwise push its two children.

This is the first consumer-facing operation that demonstrates why hierarchical AABBs are useful.

## Testing Strategy

Use TDD for each structural capability.

Tests should verify invariants, not only returned ids:
- empty tree state
- one proxy becomes root leaf
- two proxies create an internal root with two leaves
- parent AABB equals union of children
- height is correct
- proxy id remains stable while node index may change
- three-proxy insertion chooses the lower-cost branch
- removal repairs ancestry
- movement reinserts correctly
- query prunes and reports expected leaves
- validation detects no structural inconsistencies after mixed operations

## Teaching Strategy

For each implementation slice:
1. explain the invariant and why it exists,
2. show the smallest tree shape involved,
3. add a failing test,
4. implement the minimum code,
5. run tests,
6. relate the finished code back to Box2D and Box3D.

The user should be able to explain the resulting tree structure and insertion logic without relying on memorized source code.
