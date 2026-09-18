# Dynamic AABB Tree Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a tested Dynamic AABB Tree for Zonai and teach the algorithm incrementally while implementing it.

**Architecture:** The tree is a binary BVH with stable proxy ids and transient node indices. Internal nodes store AABB unions and heights; leaves store proxy identity. Sibling nodes are kept adjacent in paired slots, following the current Box2D/Box3D data-oriented direction while using C++ containers for ownership.

**Tech Stack:** C++20, CMake, CTest, existing Zonai math/collision primitives.

**Spec:** `docs/superpowers/specs/2026-09-19-dynamic-aabb-tree-design.md`

## Global Constraints

- DynamicTree does not own the fat-AABB policy.
- BroadPhase pair generation is out of scope.
- Keep stable proxy ids separate from node indices.
- Preserve the packed `flagIndex` concept.
- Use TDD for every behavior change.
- Follow existing Zonai naming and brace style.

---

### Task 1: AABB operations required by the tree

**Files:**
- Modify: `src/collision/aabb2.h`
- Modify: `tests/collision/aabb2_test.cpp`

**Interfaces:**
- Produces: `aabb2 Union(const aabb2&, const aabb2&)`
- Produces: `float Perimeter(const aabb2&)`
- Produces: `bool Contains(const aabb2&, const aabb2&)`

- [ ] **Step 1: Write failing tests**

Add cases asserting:
- union of `[-1,-1]-[1,1]` and `[2,-2]-[4,0]` becomes `[-1,-2]-[4,1]`
- perimeter cost of a width 4, height 2 box equals `12.0f`
- outer AABB contains an inner AABB
- containment fails when one edge escapes

- [ ] **Step 2: Run the AABB test**

Run:
```
ctest --test-dir build -C Debug -R aabb2Tests --output-on-failure
```

Expected: compile or assertion failure because the new helpers do not exist.

- [ ] **Step 3: Implement the helpers**

Add inline helpers in `aabb2.h` using componentwise min/max and:

```cpp
return 2.0f * ( width + height );
```

- [ ] **Step 4: Run AABB tests**

Expected: PASS.

- [ ] **Step 5: Commit**

```
git add src/collision/aabb2.h tests/collision/aabb2_test.cpp
git commit -m "feat: add dynamic tree aabb helpers"
```

### Task 2: Minimal tree representation and one leaf

**Files:**
- Modify: `src/collision/broadphase/dynamicTree.h`
- Create: `src/collision/broadphase/dynamicTree.cpp`
- Create: `tests/collision/broadphase/dynamic_tree_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Produces: `class DynamicTree`
- Produces: `int CreateProxy(const aabb2& aabb, int shapeIndex)`
- Produces: `std::size_t GetProxyCount() const`
- Produces test/debug inspectors for root/proxy invariants without exposing raw mutable storage.

- [ ] **Step 1: Write a failing empty-tree/one-proxy test**

The test creates a tree, checks count 0, creates one proxy, then verifies:
- returned proxy id is valid
- proxy count is 1
- root is a leaf
- root AABB equals the inserted AABB
- root leaf maps back to the returned proxy id

- [ ] **Step 2: Build and verify RED**

Expected: missing DynamicTree API.

- [ ] **Step 3: Implement node/proxy constants and storage**

Define:
```cpp
constexpr std::uint32_t TREE_MOVED_NODE = 1u << 30;
constexpr std::uint32_t TREE_LEAF_NODE = 1u << 31;
constexpr std::uint32_t TREE_NODE_INDEX_MASK =
    ~( TREE_MOVED_NODE | TREE_LEAF_NODE );
```

Replace the internal-node union member `leafCount` with `height`.

Add a private proxy record containing:
```cpp
std::int32_t node = -1;
std::int32_t next = -1;
```

Use contiguous C++ storage and a free list.

- [ ] **Step 4: Implement the one-leaf creation path**

If the tree is empty, store the new leaf at the root and map the proxy to root index 0.

- [ ] **Step 5: Run the new tree test and all existing tests**

Expected: PASS.

- [ ] **Step 6: Commit**

```
git add src/collision/broadphase/dynamicTree.h src/collision/broadphase/dynamicTree.cpp tests/collision/broadphase/dynamic_tree_test.cpp tests/CMakeLists.txt
git commit -m "feat: add dynamic tree root leaf"
```

### Task 3: Two-leaf tree and paired siblings

**Files:**
- Modify: `src/collision/broadphase/dynamicTree.cpp`
- Modify: `tests/collision/broadphase/dynamic_tree_test.cpp`

**Interfaces:**
- Consumes: stable proxy allocation from Task 2.
- Produces: internal root with adjacent child pair.

- [ ] **Step 1: Add a failing two-proxy structural test**

Verify:
- root is internal
- root height is 1
- root AABB equals union of both leaves
- left/right children are adjacent
- each leaf still resolves to its original proxy id

- [ ] **Step 2: Run and verify RED**

Expected: second insertion path fails.

- [ ] **Step 3: Implement sibling-pair allocation and internal-node construction**

For the special two-leaf case:
- allocate two adjacent child slots,
- copy the old root leaf into the first slot,
- place the new leaf in the second,
- replace index 0 with an internal node whose child-pair index points to the first slot,
- update proxy->node mappings.

- [ ] **Step 4: Run tests**

Expected: PASS.

- [ ] **Step 5: Commit**

```
git add src/collision/broadphase/dynamicTree.cpp tests/collision/broadphase/dynamic_tree_test.cpp
git commit -m "feat: add paired dynamic tree leaves"
```

### Task 4: Best-sibling selection and general insertion

**Files:**
- Modify: `src/collision/broadphase/dynamicTree.cpp`
- Modify: `tests/collision/broadphase/dynamic_tree_test.cpp`

**Interfaces:**
- Produces private: `FindBestSibling(const aabb2&) const`
- Produces private: ancestor refit after insertion.

- [ ] **Step 1: Add a failing three-proxy placement test**

Insert A and B close together, then C far away, and verify the resulting hierarchy chooses the lower perimeter-growth grouping.

- [ ] **Step 2: Run and verify RED**

Expected: general insertion is missing or produces the wrong grouping.

- [ ] **Step 3: Implement greedy SAH-style sibling search**

Use:
```cpp
Perimeter( Union( candidateAABB, newAABB ) )
```
plus inherited ancestor growth to choose a sibling by greedy descent.

- [ ] **Step 4: Implement general insertion and ancestor refit**

After splicing in the new parent, walk upward and recompute:
- AABB = union of children
- height = 1 + max(child heights)

- [ ] **Step 5: Run tests**

Expected: PASS.

- [ ] **Step 6: Commit**

```
git add src/collision/broadphase/dynamicTree.cpp tests/collision/broadphase/dynamic_tree_test.cpp
git commit -m "feat: add dynamic tree insertion"
```

### Task 5: Removal and stable proxy ids

**Files:**
- Modify: `src/collision/broadphase/dynamicTree.h`
- Modify: `src/collision/broadphase/dynamicTree.cpp`
- Modify: `tests/collision/broadphase/dynamic_tree_test.cpp`

**Interfaces:**
- Produces: `void DestroyProxy(int proxyId)`

- [ ] **Step 1: Add failing removal tests**

Verify:
- removing one of two proxies promotes the sibling to the root,
- removing the last proxy empties the tree,
- proxy count decrements,
- remaining proxy id is unchanged.

- [ ] **Step 2: Run and verify RED**

- [ ] **Step 3: Implement removal and pool recycling**

Remove the leaf and its parent, splice the sibling into the grandparent position, refit ancestors, then return the proxy and pair to their free lists.

- [ ] **Step 4: Run tests**

Expected: PASS.

- [ ] **Step 5: Commit**

```
git add src/collision/broadphase/dynamicTree.h src/collision/broadphase/dynamicTree.cpp tests/collision/broadphase/dynamic_tree_test.cpp
git commit -m "feat: add dynamic tree proxy removal"
```

### Task 6: MoveProxy

**Files:**
- Modify: `src/collision/broadphase/dynamicTree.h`
- Modify: `src/collision/broadphase/dynamicTree.cpp`
- Modify: `tests/collision/broadphase/dynamic_tree_test.cpp`

**Interfaces:**
- Produces: `void MoveProxy(int proxyId, const aabb2& aabb)`

- [ ] **Step 1: Add a failing movement test**

Create three proxies, move one across the scene, verify:
- the proxy id does not change,
- its leaf AABB updates,
- hierarchy refits/reinserts,
- validation still succeeds.

- [ ] **Step 2: Run and verify RED**

- [ ] **Step 3: Implement move as remove-leaf + update AABB + reinsert-leaf**

Do not add fat-AABB containment policy here.

- [ ] **Step 4: Run tests**

Expected: PASS.

- [ ] **Step 5: Commit**

```
git add src/collision/broadphase/dynamicTree.h src/collision/broadphase/dynamicTree.cpp tests/collision/broadphase/dynamic_tree_test.cpp
git commit -m "feat: add dynamic tree proxy movement"
```

### Task 7: AABB query

**Files:**
- Modify: `src/collision/broadphase/dynamicTree.h`
- Modify: `src/collision/broadphase/dynamicTree.cpp`
- Modify: `tests/collision/broadphase/dynamic_tree_test.cpp`

**Interfaces:**
- Produces a read-only query API that reports overlapping proxy ids through a callback or small callable template without exposing nodes.

- [ ] **Step 1: Add failing query tests**

Insert separated proxies and query a region overlapping only a subset.

Verify exactly those proxy ids are reported.

- [ ] **Step 2: Run and verify RED**

- [ ] **Step 3: Implement stack-based traversal**

Prune nodes whose AABB does not overlap the query AABB.

- [ ] **Step 4: Run tests**

Expected: PASS.

- [ ] **Step 5: Commit**

```
git add src/collision/broadphase/dynamicTree.h src/collision/broadphase/dynamicTree.cpp tests/collision/broadphase/dynamic_tree_test.cpp
git commit -m "feat: add dynamic tree aabb query"
```

### Task 8: Rotation and validation

**Files:**
- Modify: `src/collision/broadphase/dynamicTree.cpp`
- Modify: `tests/collision/broadphase/dynamic_tree_test.cpp`

**Interfaces:**
- Produces private local rotation logic.
- Produces test/debug validation for parent-child, AABB, height, proxy mapping, and paired-sibling invariants.

- [ ] **Step 1: Add a skewed-insertion regression test**

Insert a sequence that would create a poor tree without local rotation and assert validation plus an expected bounded height/cost improvement.

- [ ] **Step 2: Run and verify RED**

- [ ] **Step 3: Implement local cost-reducing rotations**

Evaluate legal local alternatives, apply only a rotation that lowers AABB cost, and repair parent/proxy links.

- [ ] **Step 4: Implement validation checks**

Validation verifies:
- every live proxy points to a leaf with the same proxy id,
- every internal node owns an adjacent child pair,
- internal AABB equals child union,
- height matches child heights,
- parent indices are consistent,
- root has no parent.

- [ ] **Step 5: Run the full suite**

```
ctest --test-dir build -C Debug --output-on-failure
```

Expected: all tests PASS.

- [ ] **Step 6: Commit**

```
git add src/collision/broadphase/dynamicTree.cpp tests/collision/broadphase/dynamic_tree_test.cpp
git commit -m "feat: balance and validate dynamic tree"
```
