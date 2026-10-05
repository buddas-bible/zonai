# Part 5 Body / Shape / Contact Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Audit and minimally fix zonai's Body / Shape / Contact ownership and lifetime semantics against current Box2D intent without changing Part 6 solver behavior.

**Architecture:** Keep the existing stable-slot storage and cold/hot split (`body`/`bodySim`/`bodyState`, `contact2`/`contactSim2`). Add world ownership to public handles, then prove body/shape/contact lifecycle invariants with regression tests and only change code when a test exposes a real defect. Existing Part 5 branches remain reference material only.

**Tech Stack:** C++20, CMake 4.2, Visual Studio 2026/MSVC, CTest, existing assert-based zonai tests.

**Spec:** `docs/superpowers/specs/2026-10-05-part5-body-shape-contact-audit-design.md`

## Global Constraints

- Keep lowerCamelCase and the current private-member convention.
- Keep cpp definition order aligned with header declaration order.
- Preserve the existing `#pragma region` organization where touched.
- Avoid unnecessary line breaks and aesthetic-only refactors.
- Comments should explain Box2D rationale/invariants briefly.
- Do not change contact constraint math, warm start math, bias/relaxation/restitution order, or other Part 6 solver behavior.
- Do not wholesale merge `refactor/body-shape-contact-audit` or `refactor/body-shape-contact-reaudit`.
- Preserve current generation semantics: deletion keeps the generation value; reallocation of the slot increments it.

## Review Focus

- A handle from another `world` must be rejected even when `index1` and `generation` happen to match.
- A world lifetime token must never be zero and must stay stable for the world lifetime.
- Destroying nodes while walking body shape/contact lists must not lose remaining links.
- Reused slots must not retain old simulation, manifold, impulse, proxy, or list state.
- Persistent zero-point contacts may remain allocated, but public contact-data queries exclude them. Speculative manifolds with pointCount > 0 are included, matching Box2D.

---

### Task 1: Make public handles world-aware

**Files:**
- Modify: `src/dynamics/id.h`
- Modify: `src/dynamics/world.h`
- Modify: `src/dynamics/world.cpp`
- Test: `tests/dynamics/world_test.cpp`

**Interfaces:**
- Consumes: existing `bodyId`, `shapeId`, `contactId`, `world::IsValid`, `world::MakeBodyId`, `world::MakeShapeId`, `world::MakeContactId`.
- Produces: IDs with `std::uint64_t worldToken`; stable non-zero `world::worldToken_`; world-aware `IsValid`.

- [ ] **Step 1: Add RED cross-world body/shape tests**

Create `worldA` and `worldB`, create equivalent body and shape slots in both, and assert each world's handles are valid only in the owning world:

```cpp
assert( worldA.IsValid( bodyA ) );
assert( worldB.IsValid( bodyB ) );
assert( !worldA.IsValid( bodyB ) );
assert( !worldB.IsValid( bodyA ) );
assert( !worldA.IsValid( shapeB ) );
assert( !worldB.IsValid( shapeA ) );
```

- [ ] **Step 2: Run the focused test and verify RED**

```bash
cmake --preset vs2026
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: FAIL because current public IDs contain only slot + generation.

- [ ] **Step 3: Add world ownership to public IDs**

In `src/dynamics/id.h`, add `std::uint64_t worldToken = 0;` to `bodyId`, `shapeId`, and `contactId`. Keep all `IsNull(...)` helpers based on `index1 == 0`.

- [ ] **Step 4: Add a stable non-zero world token**

In `world.h`, add:

```cpp
world();
world( const world& ) = delete;
world& operator=( const world& ) = delete;
world( world&& ) = delete;
world& operator=( world&& ) = delete;
```

and private `std::uint64_t worldToken_ = 0;`.

In `world.cpp`, add a translation-unit-local `AllocateWorldToken() noexcept` backed by `std::atomic<std::uint64_t>` using `memory_order_relaxed`, skipping token `0`, then initialize `worldToken_` in `world::world()`.

- [ ] **Step 5: Update public ID construction and validation**

Update these exact interfaces without changing their names:

```cpp
bool world::IsValid( bodyId bodyId ) const noexcept;
bool world::IsValid( shapeId shapeId ) const noexcept;
bool world::IsValid( contactId contactId ) const noexcept;
bodyId world::MakeBodyId( std::int32_t bodyIndex ) const;
shapeId world::MakeShapeId( std::int32_t shapeIndex ) const;
contactId world::MakeContactId( std::int32_t contactIndex ) const;
```

`IsValid` rejects a mismatched token before indexing storage. `Make*Id` embeds `worldToken_`.

- [ ] **Step 6: Add cross-world contact coverage**

Create one touching pair in each world, collect `contactData.id` through `UpdateCollisions`, and assert a contact handle from one world is invalid in the other.

- [ ] **Step 7: Run Task 1 tests**

```bash
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add src/dynamics/id.h src/dynamics/world.h src/dynamics/world.cpp tests/dynamics/world_test.cpp
git commit -m "fix(part5): validate public handles by world lifetime"
```

---

### Task 2: Lock down Body and Shape lifecycle invariants

**Files:**
- Modify if needed: `src/dynamics/bodyShape.h`
- Modify if needed: `src/dynamics/world.cpp`
- Test: `tests/dynamics/world_test.cpp`

**Interfaces:**
- Consumes: `CreateBody`, `DestroyBody`, `CreateShape`, `DestroyShape`, `GetBody`, `GetShape`, body/shape counts, broad-phase inspection already exposed by `world`.
- Produces: regression coverage for stable-slot reset, intrusive shape-list integrity, and proxy/contact cascade cleanup.

- [ ] **Step 1: Strengthen body slot-reuse coverage**

Before destroying a Dynamic body, assign non-default transform and velocities. Recreate a Dynamic body in the same slot and assert:

```cpp
assert( reusedBody.index1 == oldBody.index1 );
assert( reusedBody.generation != oldBody.generation );
assert( !world.IsValid( oldBody ) );
const vec2 reusedVelocity = world.GetBodyLinearVelocity( reusedBody );
assert( reusedVelocity.x == 0.0f );
assert( reusedVelocity.y == 0.0f );
assert( world.GetBodyAngularVelocity( reusedBody ) == 0.0f );
```

Also verify the recreated `body` has `shapeCount == 0`, `contactCount == 0`, and null list heads through `GetBody`.

- [ ] **Step 2: Run `worldTests`**

Expected: PASS if current reset logic is already correct. If RED, change only `CreateBody` / `DestroyBodyByIndex` fields implicated by the failing assertions.

- [ ] **Step 3: Add shape-list head/middle/tail deletion tests**

Create one body with three shapes. In separate scenarios delete the current head, a middle shape, and the final remaining shape. Verify via `GetBody`/`GetShape` that:

- head `prevShapeId == NULL_INDEX`
- surviving neighbors reference each other after middle deletion
- final deletion leaves `headShapeId == NULL_INDEX` and `shapeCount == 0`
- destroyed handles become invalid and `GetShapeCount()` matches live shapes

- [ ] **Step 4: Run and minimally repair shape ownership if RED**

Only touch `LinkShape`, `UnlinkShape`, `DestroyShapeByIndex`, mass refresh, or proxy cleanup that a failing test identifies. Keep the intrusive list design.

- [ ] **Step 5: Add DestroyBody cascade coverage**

Create a body with multiple shapes and a touching contact to a surviving body. Call `DestroyBody` and assert:

- destroyed body and shape handles are invalid
- body/shape/contact counts decrease correctly
- surviving body's contact capacity no longer contains the destroyed pair
- destroyed proxies are absent from broad-phase state exposed by current debug/query APIs

- [ ] **Step 6: Run Task 2 tests**

```bash
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/dynamics/bodyShape.h src/dynamics/world.cpp tests/dynamics/world_test.cpp
git commit -m "test(part5): cover body and shape lifecycle invariants"
```

---

### Task 3: Audit Contact lifetime and intrusive-edge bookkeeping

**Files:**
- Modify if needed: `src/collision/narrowphase/contact2.h`
- Modify if needed: `src/dynamics/contactSim2.h`
- Modify if needed: `src/dynamics/world.cpp`
- Test: `tests/dynamics/world_test.cpp`

**Interfaces:**
- Consumes: `MakeContactKey`, `GetContactId`, `GetContactEdgeIndex`, `UpdateCollisions`, `GetBodyContactCapacity`, `GetBodyContactData`, `GetContactData`.
- Produces: regression coverage that contact removal keeps both bodies observable as consistent and reused contact slots contain fresh state.

- [ ] **Step 1: Add contact-key encode/decode regression**

For representative non-negative contact indices and both edge indices, assert `GetContactId(MakeContactKey(...))` and `GetContactEdgeIndex(...)` round-trip exactly.

- [ ] **Step 2: Add head/middle/tail contact-removal scenarios**

Create one body touching three separate bodies. Capture the returned `contactData.id` values and the central body's `headContactKey`. Remove counterpart shapes/bodies in orders that exercise removal of the current head and non-head contacts. After every removal assert:

- `GetBodyContactCapacity` decreases by exactly one
- every surviving public `contactId` remains valid
- the removed `contactId` becomes invalid
- `GetBody(...).contactCount` equals public capacity
- when no contacts remain, `headContactKey == NULL_INDEX`

These externally observable invariants verify intrusive-edge bookkeeping without adding a test-only public API for private `prevKey`/`nextKey` storage.

- [ ] **Step 3: Run `worldTests`**

Expected: PASS if current `DestroyContact` is correct. If RED, modify only predecessor/successor/head/count bookkeeping required by the failing case.

- [ ] **Step 4: Add contact slot-reuse regression**

Create a contact, record its `contactId`, exercise collision update/step so `contactSim2` owns real manifold/cache state, destroy it, then create another contact that reuses the same slot. Assert:

```cpp
assert( reusedContact.index1 == oldContact.index1 );
assert( reusedContact.generation != oldContact.generation );
assert( !world.IsValid( oldContact ) );
```

Verify the new `contactData.manifold` describes the new pair only; if internal cache reset is suspect, add a narrowly scoped test through the existing solver/contact behavior rather than exposing `contactSim2` publicly.

- [ ] **Step 5: Fix contact/contactSim reset only if RED**

Keep generation-on-allocation semantics and fully reset only the slot fields proven stale by the test.

- [ ] **Step 6: Run Task 3 tests**

```bash
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/collision/narrowphase/contact2.h src/dynamics/contactSim2.h src/dynamics/world.cpp tests/dynamics/world_test.cpp
git commit -m "test(part5): cover contact lifecycle invariants"
```

---

### Task 4: Verify filter mutation and touching-query semantics

**Files:**
- Modify if needed: `src/dynamics/world.h`
- Modify if needed: `src/dynamics/world.cpp`
- Test: `tests/dynamics/world_test.cpp`

**Interfaces:**
- Consumes: `SetShapeFilter`, `SetBodyTransform`, `UpdateCollisions`, `GetBodyContactCapacity/Data`, `GetShapeContactCapacity/Data`.
- Produces: fixed distinction between persistent contacts and public touching-contact data across runtime mutations.

- [ ] **Step 1: Add persistent-but-non-touching query coverage**

Construct a pair that still has a persistent broad-phase contact while its narrow-phase manifold has zero points (disable recycling in this fixture). Assert body and shape capacity may remain non-zero while both data queries write zero entries.

- [ ] **Step 2: Add touching transition coverage**

Move the pair into actual contact, update collisions, and assert body/shape contact-data queries return the touching contact. Move it back to a zero-point position that keeps the persistent pair when possible; after update, data queries must return zero again.

- [ ] **Step 3: Verify speculative public queries against current Box2D**

Include speculative points in both queries: Box2D contact.c sets its touching flag from pointCount > 0. Keep the existing separation-based collision callback unchanged.

- [ ] **Step 4: Add SetShapeFilter invalidation coverage**

Create a touching pair, change one shape's filter so the pair is rejected, and assert the existing contact disappears immediately. Restore a compatible filter, update collisions, and assert a fresh valid contact is created without stale pair/contact state.

- [ ] **Step 5: Run and minimally fix mutation semantics if RED**

Only change filter/contact invalidation, proxy buffering, or touching-query gating identified by the tests. Do not alter Part 4 tree algorithms.

- [ ] **Step 6: Run Task 4 tests**

```bash
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add src/dynamics/world.h src/dynamics/world.cpp tests/dynamics/world_test.cpp
git commit -m "test(part5): lock contact query and filter semantics"
```

---

### Task 5: Final audit and verification

**Files:**
- Modify only if justified: files touched by Tasks 1-4
- Create: `docs/part5-body-shape-contact-audit.md`

**Interfaces:**
- Consumes: all Task 1-4 behavior and tests.
- Produces: merge-ready branch and a concise audit report.

- [ ] **Step 1: Compare against both old Part 5 branches**

Classify meaningful old changes as adopted, already present, rejected/outdated, or aesthetic-only. Record the reasoning in the final audit report; do not merge history wholesale.

- [ ] **Step 2: Check style and declaration/definition order**

For every touched file, ensure header/cpp order, lowerCamelCase, current comments, `#pragma region`, and formatting remain consistent. Make no unrelated cleanup.

- [ ] **Step 3: Run focused dynamics tests**

```bash
cmake --build --preset debug --target bodyTests bodyShapeTests worldTests
ctest --test-dir build -C Debug -R "^(bodyTests|bodyShapeTests|worldTests)$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 4: Run full Debug verification**

```bash
cmake --build --preset debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: all tests PASS.

- [ ] **Step 5: Run Release verification**

```bash
cmake --build --preset release
ctest --test-dir build -C Release --output-on-failure
```

Expected: build succeeds and all tests PASS.

- [ ] **Step 6: Verify Part 6 isolation**

Inspect the branch diff. There should be no intentional solver math/order change in `contactConstraint2.*`. Any unavoidable solver-facing interface change must be documented and covered by existing `contactConstraintTests`.

- [ ] **Step 7: Write `docs/part5-body-shape-contact-audit.md`**

Include audited invariants, correctness fixes, old-branch changes adopted/rejected, tests added, Debug/Release results, known issues, and merge recommendation.

- [ ] **Step 8: Commit final report**

```bash
git add docs/part5-body-shape-contact-audit.md
git commit -m "docs(part5): record body shape contact audit"
```
