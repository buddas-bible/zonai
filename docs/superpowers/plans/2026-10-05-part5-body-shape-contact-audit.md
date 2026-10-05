# Part 5 Body / Shape / Contact Audit Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Audit and minimally fix zonai's Body / Shape / Contact ownership and lifetime semantics against current Box2D intent without changing Part 6 solver behavior.

**Architecture:** Keep the current stable-slot storage and cold/hot split (`body`/`bodySim`/`bodyState`, `contact2`/`contactSim2`). Add world ownership to public handles, then verify lifecycle invariants with regression tests before making only the fixes those tests justify. Existing Part 5 branches are reference material only; current `master` is the source of truth.

**Tech Stack:** C++20, CMake 4.2, Visual Studio 2026/MSVC, CTest, zonai's existing assert-based test executables.

**Spec:** `docs/superpowers/specs/2026-10-05-part5-body-shape-contact-audit-design.md`

## Global Constraints

- Use lowerCamelCase for types/functions/variables according to the current zonai convention.
- Keep the current private member naming convention.
- Keep cpp definition order aligned with header declaration order.
- Preserve the current `#pragma region` organization where touched.
- Avoid unnecessary line breaks and aesthetic-only refactors.
- Comments should explain Box2D rationale/invariants briefly rather than narrate obvious code.
- Do not change contact constraint math, warm start math, bias/relaxation/restitution order, or other Part 6 solver behavior.
- Do not wholesale merge `refactor/body-shape-contact-audit` or `refactor/body-shape-contact-reaudit`; inspect and reapply only justified changes.
- Keep generation semantics as implemented today: deletion preserves the generation value; the next allocation of the same slot increments it.

## Review Focus

- Cross-world handles with the same `index1` and `generation` must be rejected by the wrong `world`.
- Lifetime-token generation must never produce the null token value and must remain stable for the entire `world` lifetime.
- Destroying a body or shape while iterating intrusive lists must not lose the next contact/shape link.
- Reused stable slots must not retain old velocity, transform, manifold, impulse, proxy, or list state.
- Persistent speculative contacts may remain allocated while not touching, but public contact-data queries must exclude them.

---

### Task 1: Make public handles world-aware

**Files:**
- Modify: `src/dynamics/id.h`
- Modify: `src/dynamics/world.h`
- Modify: `src/dynamics/world.cpp`
- Test: `tests/dynamics/world_test.cpp`

**Interfaces:**
- Consumes: current `bodyId`, `shapeId`, `contactId`, `world::IsValid`, `world::MakeBodyId`, `world::MakeShapeId`, `world::MakeContactId`.
- Produces: `bodyId/shapeId/contactId` with `std::uint64_t worldToken`; `world()` lifetime-token initialization; world-aware `IsValid` semantics.

- [ ] **Step 1: Add RED cross-world body handle coverage**

Add a test block to `tests/dynamics/world_test.cpp` that creates `worldA` and `worldB`, creates one Dynamic body in each, verifies their `index1` and generation may match, and asserts:

```cpp
assert( worldA.IsValid( bodyA ) );
assert( worldB.IsValid( bodyB ) );
assert( !worldA.IsValid( bodyB ) );
assert( !worldB.IsValid( bodyA ) );
```

Also create one shape in each world and assert the same cross-world invalidity for `shapeId`.

- [ ] **Step 2: Run `worldTests` and confirm the RED failure**

Run after configure/build:

```bash
cmake --preset vs2026
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: test fails because public handles currently encode only slot + generation.

- [ ] **Step 3: Add `worldToken` to public IDs**

In `src/dynamics/id.h`, add `std::uint64_t worldToken = 0;` to `bodyId`, `shapeId`, and `contactId`. Keep `IsNull(id)` based only on `index1 == 0`.

- [ ] **Step 4: Give every world a stable non-zero lifetime token**

In `src/dynamics/world.h`, declare:

```cpp
world();
world( const world& ) = delete;
world& operator=( const world& ) = delete;
world( world&& ) = delete;
world& operator=( world&& ) = delete;
```

Add private:

```cpp
std::uint64_t worldToken_ = 0;
```

In `src/dynamics/world.cpp`, implement a translation-unit-local `AllocateWorldToken() noexcept` using `std::atomic<std::uint64_t>` with `memory_order_relaxed`, skipping token `0`, and initialize `worldToken_` in `world::world()`.

- [ ] **Step 5: Make all public ID creation and validation world-aware**

Update:

```cpp
bool world::IsValid( bodyId bodyId ) const noexcept;
bool world::IsValid( shapeId shapeId ) const noexcept;
bool world::IsValid( contactId contactId ) const noexcept;
bodyId world::MakeBodyId( std::int32_t bodyIndex ) const;
shapeId world::MakeShapeId( std::int32_t shapeIndex ) const;
contactId world::MakeContactId( std::int32_t contactIndex ) const;
```

`IsValid` must reject mismatched `worldToken` before indexing storage. `Make*Id` must embed `worldToken_`.

- [ ] **Step 6: Add cross-world contact regression**

Create equivalent contacts in two worlds via overlapping Dynamic/Static shape pairs and `UpdateCollisions`, capture `contactId` through returned `contactData`, and assert the contact handle from one world is invalid in the other.

- [ ] **Step 7: Run Task 1 tests**

```bash
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 8: Commit Task 1**

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
- Consumes: `world::CreateBody`, `world::DestroyBody`, `world::CreateShape`, `world::DestroyShape`, `world::GetBody`, `world::GetShape`, `world::GetBodyCount`, `world::GetShapeCount`, `world::GetBroadPhase`.
- Produces: regression coverage proving stable-slot reset, intrusive shape-list integrity, and proxy cleanup.

- [ ] **Step 1: Add body slot-reuse reset coverage**

Extend the existing body reuse test to set transform, linear/angular velocity, damping/gravity-relevant state, destroy the body, recreate a Dynamic body in the same slot, and assert:

```cpp
assert( reusedBody.index1 == oldBody.index1 );
assert( reusedBody.generation != oldBody.generation );
assert( !world.IsValid( oldBody ) );
assert( world.GetBodyLinearVelocity( reusedBody ) == vec2{} );
assert( world.GetBodyAngularVelocity( reusedBody ) == 0.0f );
```

Also verify the recreated body has no shapes or contacts through public counts/query capacity.

- [ ] **Step 2: Run `worldTests`**

Expected: PASS if current reset logic is correct; if it fails, continue to Step 3 with the failing invariant only.

- [ ] **Step 3: Minimally fix `world::CreateBody` / `DestroyBodyByIndex` if RED**

Keep same-index `body`, `bodySim`, and `bodyState` storage. Reset all reused simulation/state fields before returning the new handle. Preserve generation on destroy and increment on allocation.

- [ ] **Step 4: Add shape list head/middle/tail deletion coverage**

Create one body with three shapes and capture internal indices through `world.GetShape(...)`. Delete in three independent scenarios:

- head deletion: new head has `prevShapeId == NULL_INDEX`
- middle deletion: previous/next neighbors point to each other
- final deletion: `body.headShapeId == NULL_INDEX` and `body.shapeCount == 0`

For every scenario assert `world.GetShapeCount()` and `world.IsValid(oldShape)` are correct.

- [ ] **Step 5: Run `worldTests` and fix only proven list/proxy defects**

If RED, adjust only `LinkShape`, `UnlinkShape`, `DestroyShapeByIndex`, or proxy cleanup involved in the failing invariant. Do not replace the intrusive list data structure.

- [ ] **Step 6: Add DestroyBody cascade coverage**

Create one Dynamic body with multiple shapes and at least one touching contact to another body. Record shape handles and broad-phase proxy/pair counts. Call `DestroyBody` and assert:

- body and all its shape handles are invalid
- body/shape/contact counts drop by the expected amount
- no proxy for destroyed shapes remains in broad phase
- the surviving body's contact capacity no longer includes the destroyed pair

- [ ] **Step 7: Run Task 2 tests**

```bash
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 8: Commit Task 2**

```bash
git add src/dynamics/bodyShape.h src/dynamics/world.cpp tests/dynamics/world_test.cpp
git commit -m "test(part5): cover body and shape lifecycle invariants"
```

---

### Task 3: Audit Contact intrusive lists and stable-slot reuse

**Files:**
- Modify if needed: `src/collision/narrowphase/contact2.h`
- Modify if needed: `src/dynamics/contactSim2.h`
- Modify if needed: `src/dynamics/world.cpp`
- Test: `tests/dynamics/world_test.cpp`

**Interfaces:**
- Consumes: `MakeContactKey`, `GetContactId`, `GetContactEdgeIndex`, `world::UpdateCollisions`, `world::GetBodyContactCapacity`, `world::GetBodyContactData`, `world::GetContactData`.
- Produces: regression coverage proving both body edges remain valid through head/middle/tail removal and that reused contact slots have fresh simulation caches.

- [ ] **Step 1: Add contact-key encoding regression**

Add constexpr/runtime assertions for representative contact IDs and both edge indices:

```cpp
const auto key = MakeContactKey( contactIndex, edgeIndex );
assert( GetContactId( key ) == contactIndex );
assert( GetContactEdgeIndex( key ) == edgeIndex );
```

- [ ] **Step 2: Add contact-list head/middle/tail removal coverage**

Construct one body touching three separate bodies so it owns three contact edges. Remove the corresponding shapes/bodies in orders that exercise head, middle, and tail deletion. After every deletion traverse the remaining chain through `world.GetBody(...).headContactKey` plus `world.GetContactData(...)`/public capacity and assert count/link consistency.

- [ ] **Step 3: Run `worldTests`**

Expected: PASS if `DestroyContact` already maintains both sides correctly; otherwise capture the first broken invariant.

- [ ] **Step 4: Minimally repair `world::DestroyContact` if RED**

Keep the existing `contactKey = (contactId << 1) | edgeIndex` representation. Update only predecessor/successor/head/count bookkeeping necessary to satisfy the failing test on both bodies.

- [ ] **Step 5: Add contact slot-reuse regression**

Create a touching contact, record its public `contactId`, run at least one collision/step path that populates manifold/impulse cache, destroy the contact by separating/destroying its shape, then create a new contact that reuses the same stable slot. Assert:

```cpp
assert( reusedContact.index1 == oldContact.index1 );
assert( reusedContact.generation != oldContact.generation );
assert( !world.IsValid( oldContact ) );
```

Verify the new contact's manifold and cached impulses reflect only the new pair.

- [ ] **Step 6: Fix contact/contactSim reset only if the reuse test fails**

If needed, make `CreateContact`/`DestroyContact` fully reinitialize `contact2` and `contactSim2` while preserving the current generation-on-allocation policy.

- [ ] **Step 7: Run Task 3 tests**

```bash
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 8: Commit Task 3**

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
- Consumes: `world::SetShapeFilter`, `world::SetBodyTransform`, `world::UpdateCollisions`, `world::GetBodyContactCapacity`, `world::GetBodyContactData`, `world::GetShapeContactCapacity`, `world::GetShapeContactData`.
- Produces: fixed semantics where persistent contacts and touching-contact data remain distinct across runtime changes.

- [ ] **Step 1: Add speculative persistent-contact query coverage**

Create shapes whose fat/speculative broad-phase bounds produce a persistent contact while the narrow-phase manifold has no touching point. Assert:

```cpp
assert( world.GetBodyContactCapacity( bodyId ) >= 1 );
assert( world.GetBodyContactData( bodyId, output ) == 0 );
```

Repeat with `GetShapeContactCapacity/Data`.

- [ ] **Step 2: Add touching transition coverage**

Move one body into actual contact, call collision update, assert body and shape contact-data queries return one touching contact. Move it back to a non-touching position that still preserves the persistent pair where possible, update again, and assert data queries return zero even if capacity remains non-zero.

- [ ] **Step 3: Reject old re-audit speculative-public-query behavior**

Do not port any old branch change that makes `GetBodyContactData`/`GetShapeContactData` expose non-touching speculative contacts. Keep public data queries gated by the current touching manifold semantics.

- [ ] **Step 4: Add runtime filter invalidation coverage**

Create a touching pair, verify one contact, then call `SetShapeFilter` with a mask that rejects the pair. Assert the existing contact is destroyed immediately. Restore a compatible filter, run collision update, and assert a new valid contact can be created without stale pair/contact state.

- [ ] **Step 5: Run `worldTests` and minimally fix mutation semantics if RED**

If failures appear, change only `SetShapeFilter`, contact destruction/pair invalidation, proxy buffering, or touching-query gating responsible for the test. Do not change broad-phase tree algorithms audited in Part 4.

- [ ] **Step 6: Run Task 4 tests**

```bash
cmake --build --preset debug --target worldTests
ctest --test-dir build -C Debug -R "^worldTests$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 7: Commit Task 4**

```bash
git add src/dynamics/world.h src/dynamics/world.cpp tests/dynamics/world_test.cpp
git commit -m "test(part5): lock contact query and filter semantics"
```

---

### Task 5: Final Part 5 audit, style pass, and verification

**Files:**
- Modify only if justified: files touched by Tasks 1-4
- Create: `docs/part5-body-shape-contact-audit.md`

**Interfaces:**
- Consumes: all Task 1-4 behavior and tests.
- Produces: merge-ready Part 5 branch plus audit report documenting adopted/rejected prior-branch changes and any known issues.

- [ ] **Step 1: Compare the new branch against both old Part 5 branches**

Use Git compare to classify old changes as:

- adopted because a regression test proves the need
- already present in current master
- rejected because semantics are wrong/outdated
- unnecessary aesthetic refactor

Record only meaningful findings in the audit report.

- [ ] **Step 2: Check declaration/definition order and touched comments**

For `world.h`/`world.cpp` and any other touched files, ensure cpp definitions follow header declaration order and comments describe lifetime/ownership rationale without excessive formatting changes.

- [ ] **Step 3: Run focused Part 5 tests**

```bash
cmake --build --preset debug --target bodyTests bodyShapeTests worldTests
ctest --test-dir build -C Debug -R "^(bodyTests|bodyShapeTests|worldTests)$" --output-on-failure
```

Expected: PASS.

- [ ] **Step 4: Run the entire Debug suite**

```bash
cmake --build --preset debug
ctest --test-dir build -C Debug --output-on-failure
```

Expected: all tests PASS.

- [ ] **Step 5: Verify Release build**

```bash
cmake --build --preset release
ctest --test-dir build -C Release --output-on-failure
```

Expected: build succeeds and all tests PASS.

- [ ] **Step 6: Verify Part 6 solver isolation**

Inspect branch diff and confirm no intentional solver math/order change in `contactConstraint2.*`; if a solver-facing interface changed, document exactly why it was necessary and prove behavior with existing `contactConstraintTests`.

- [ ] **Step 7: Write final audit report**

Create `docs/part5-body-shape-contact-audit.md` with:

- audited components/invariants
- correctness fixes made
- old Part 5 changes adopted/rejected
- tests added
- Debug/Release verification results
- remaining known issues, if any
- merge recommendation

- [ ] **Step 8: Commit final audit**

```bash
git add docs/part5-body-shape-contact-audit.md
git commit -m "docs(part5): record body shape contact audit"
```
