# Basic Prismatic Joint Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Implement Stage 1 of Prismatic Joint: allow translation only along Body A's local axis, block lateral translation and relative rotation, integrate it with World lifecycle/solver behavior, and expose a dedicated Sandbox rail demo.

**Architecture:** Follow the existing Joint stack without adding another ownership or graph layer. Add `prismaticJointDef/Data`, `prismaticJointSim2`, and a dedicated 2x2 block constraint, store the sim in the existing `jointSims_` variant, and reuse `joint2` slots, Body joint edges, island/wake/sleep, collision exclusion, and common destroy/validity paths. Stage 1 intentionally omits limit, motor, and spring fields.

**Tech Stack:** C++20, CMake/CTest, Dear ImGui Sandbox, current Zonai constraint softness/bias-relax flow, Box2D comparison baseline `ac7c751eaeddbabdc1c4d41ae4f3a25d78627790`.

**Spec:** `docs/superpowers/specs/2026-10-08-prismatic-joint-learning-design.md`

## Global Constraints

- Stage 1 exposes only basic rail behavior: no Prismatic limit, motor, or spring API/fields.
- Preserve existing `joint2` stable slot/generation/free-list and Body intrusive Joint edge ownership; do not introduce a parallel lifecycle system.
- Preserve existing World step ordering and Joint bias/relax policy: prepare -> warm start -> solve(useBias=true) -> integrate -> solve(useBias=false).
- Use a coupled 2x2 block solve for lateral translation plus angular lock; do not replace it with two independent scalar solves.
- Do not add a generic matrix subsystem for one consumer; keep the small 2x2 solve local to Prismatic constraint code.
- `localAnchorA/B` are Body-origin coordinates and must be converted to COM-relative lever arms in prepare.
- `localAxisA` is a finite unit vector in Body A local space and rotates with A.
- `referenceAngle` is finite and uses `(angleB - angleA) - referenceAngle`; default zero locks the two orientations together.
- Keep lowerCamelCase names, existing private-member conventions, header/definition ordering, concise formatting, and rationale-focused comments.
- `src/CMakeLists.txt` already discovers source/header files recursively; do not add source-list churn for the four new Joint files.

## Review Focus

- **Degenerate effective mass:** static/kinematic combinations or zero inertia must stay finite and must not divide by a near-zero 2x2 determinant. Task 1 adds an explicit finite-state regression.
- **Axis validity:** zero, non-finite, or non-unit axes must follow the same assertion/precondition policy as Wheel Joint; Task 2 pins valid unit-axis storage/query and keeps invalid input outside runtime behavior.
- **Reference-angle semantics:** non-zero initial relative orientation must remain locked to the requested reference rather than silently snapping to zero; Task 2 tests query/step behavior.
- **Off-center anchors/local center:** lateral correction must include angular coupling from COM-relative lever arms; Task 1 tests non-zero anchor and non-zero `localCenter`.
- **Lifecycle/filter restoration:** stale/foreign IDs, Body destruction, slot reuse, `collideConnected=false`, and contact re-evaluation after destroy must remain correct; Task 2 mirrors existing Joint lifecycle tests.

---

### Task 1: Add the basic Prismatic constraint core

**Files:**
- Create: `src/dynamics/joints/prismaticJoint2.h`
- Create: `src/dynamics/joints/prismaticJointSim2.h`
- Create: `src/dynamics/joints/prismaticJointConstraint2.h`
- Create: `src/dynamics/joints/prismaticJointConstraint2.cpp`
- Create: `tests/dynamics/prismatic_joint_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: `bodySim`, `bodyState`, `constraintSoftness2`, `makeConstraintSoftness(...)`, `vec2`, `rot2`, existing COM/local-center conventions.
- Produces:
  - `struct prismaticJointDef`
  - `struct prismaticJointData`
  - `struct prismaticJointSim2`
  - `struct prismaticJointConstraint2`
  - `prismaticJointConstraint2 preparePrismaticJointConstraint( const prismaticJointSim2&, const bodySim&, const bodySim&, float subStepTime )`
  - `void warmStartPrismaticJointConstraint( const prismaticJointConstraint2&, bodyState&, bodyState& )`
  - `void solvePrismaticJointConstraint( prismaticJointConstraint2&, bodyState&, bodyState&, bool useBias )`

- [ ] **Step 1: Write the failing constraint test and register `prismaticJointTests`**

Add `tests/dynamics/prismatic_joint_test.cpp` using the same NDEBUG-independent `check(...)` style as the other Joint tests. Pin these behaviors:

```text
1. axis = +X, centered anchors:
   input relative velocity = (3, 2), relative angular velocity = 4
   solve(useBias=false) keeps the +X component free while removing lateral Y and relative angular velocity within tolerance.

2. lateral position error with zero velocity:
   solve(useBias=true) produces correction toward the rail but does not create an axial correction component.

3. angular position error with zero velocity:
   solve(useBias=true) produces opposing angular correction; solve(useBias=false) with the same zero-velocity state adds no positional bias.

4. off-center anchor and non-zero localCenter:
   correction changes both linear and angular velocity and remains finite.

5. degenerate inverse-mass/inertia combination:
   prepare/solve returns finite values and no divide-by-zero/NaN.

6. warm start:
   cached lateral/angular impulses apply equal-and-opposite momentum/torque effects to movable endpoints.
```

Register:

```cmake
add_executable(prismaticJointTests dynamics/prismatic_joint_test.cpp)
target_link_libraries(prismaticJointTests PRIVATE zonai::zonai)
add_test(NAME prismaticJointTests COMMAND prismaticJointTests)
```

- [ ] **Step 2: Run the focused test and verify RED**

Run:

```bash
cmake --preset vs2026
cmake --build --preset debug --target prismaticJointTests
```

Expected: compile failure because the Prismatic headers/types/functions do not exist yet.

- [ ] **Step 3: Add `prismaticJointDef/Data` and persistent sim state**

`prismaticJoint2.h` exposes exactly:

```cpp
struct prismaticJointDef
{
    bodyId bodyA{};
    bodyId bodyB{};
    vec2 localAnchorA{};
    vec2 localAnchorB{};
    vec2 localAxisA{ 1.0f, 0.0f };
    float referenceAngle = 0.0f;
    bool collideConnected = false;
};
```

`prismaticJointData` contains body handles, world anchors, world axis, `currentTranslation`, `lateralError`, `currentAngle`, total reaction `force`, reaction `torque`, and `collideConnected`. Do not add Stage 2-4 fields.

`prismaticJointSim2` stores stable/body indices, local anchors, local axis, reference angle, and persistent `vec2 impulse` where `.x` is lateral impulse and `.y` is angular impulse.

- [ ] **Step 4: Implement prepare/warm-start and the 2x2 block solve**

Use the Box2D point-to-line + angular-lock Jacobian. With current world axis `a`, `p = LeftPerp(a)`, anchor separation `d`, and COM lever arms `rA/rB`:

```text
s1 = cross(rA + d, p)
s2 = cross(rB, p)

C1 = dot(p, d)
C2 = relativeAngle - referenceAngle

Cdot1 = dot(p, vB - vA) + s2*wB - s1*wA
Cdot2 = wB - wA

k11 = mA + mB + iA*s1*s1 + iB*s2*s2
k12 = iA*s1 + iB*s2
k22 = iA + iB
K = [ k11 k12 ; k12 k22 ]
```

Solve `K * lambda = -(Cdot + bias)` with the current Joint softness policy. In the bias pass, use the existing constraint softness coefficients for `C1` and `C2`; in relaxation, positional bias is zero and the standard mass/impulse scale values remove push velocity. If the determinant is not safely invertible, produce zero delta impulse for unsupported rows rather than NaN/Inf.

Apply combined impulse as:

```text
P = lambda.x * p
LA = lambda.x * s1 + lambda.y
LB = lambda.x * s2 + lambda.y

vA -= mA * P
wA -= iA * LA
vB += mB * P
wB += iB * LB
```

Warm start uses the cached two-component impulse with the same Jacobian/sign convention.

- [ ] **Step 5: Run the focused test to verify GREEN**

Run:

```bash
cmake --build --preset debug --target prismaticJointTests
ctest --test-dir build -C Debug -R "^prismaticJointTests$" --output-on-failure
```

Expected: PASS with finite-state, free-axis, lateral/angular lock, off-center, relaxation, and warm-start checks.

- [ ] **Step 6: Commit the constraint core**

```bash
git add src/dynamics/joints/prismaticJoint2.h src/dynamics/joints/prismaticJointSim2.h src/dynamics/joints/prismaticJointConstraint2.h src/dynamics/joints/prismaticJointConstraint2.cpp tests/dynamics/prismatic_joint_test.cpp tests/CMakeLists.txt
git commit -m "Add basic Prismatic constraint"
```

---

### Task 2: Integrate Prismatic ownership, public API, lifecycle, and query behavior

**Files:**
- Modify: `src/dynamics/world.h`
- Modify: `src/dynamics/world.cpp`
- Create: `tests/dynamics/prismatic_joint_world_test.cpp`
- Modify: `tests/CMakeLists.txt`

**Interfaces:**
- Consumes: Task 1 `prismaticJointDef`, `prismaticJointData`, `prismaticJointSim2`; existing `allocateJoint`, `destroyJoint`, Body joint-edge and collision-filter logic.
- Produces:
  - `jointId world::createPrismaticJoint( const prismaticJointDef& definition )`
  - `prismaticJointData world::getPrismaticJointData( jointId id ) const`
  - `jointSims_` variant support for `prismaticJointSim2`

- [ ] **Step 1: Write failing World lifecycle/API tests and register `prismaticJointWorldTests`**

Test these behaviors before adding the API:

```text
- create/query returns the two bodies, world anchors, normalized current world axis, initial translation/lateral error/current angle, collideConnected.
- explicit non-zero referenceAngle is reflected by currentAngle and remains the locked relative orientation through stepping.
- destroy invalidates the Joint ID and decrements count.
- Body destruction removes attached Prismatic Joint.
- freed Joint slot reuse increments generation so stale ID is invalid.
- foreign/null/stale handle validation follows existing World Joint rules.
- collideConnected=false removes/blocks contacts; destroying the last blocking Joint touches proxies so a stationary pair can collide again.
- multiple blocking Joints keep collision blocked until the final blocker is removed.
```

Register the target in `tests/CMakeLists.txt` next to the other Joint World tests.

- [ ] **Step 2: Run the focused World test and verify RED**

Run:

```bash
cmake --build --preset debug --target prismaticJointWorldTests
```

Expected: compile failure because `createPrismaticJoint/getPrismaticJointData` and `prismaticJointSim2` variant support do not exist.

- [ ] **Step 3: Add public includes/API declarations and `jointSims_` variant storage**

In `world.h`, include the new Prismatic public/sim/constraint headers in the same Joint order used by existing types. Add declarations beside Distance/Revolute/Wheel APIs. Extend:

```cpp
std::vector<std::variant<distanceJointSim2, mouseJointSim2, revoluteJointSim2, wheelJointSim2, prismaticJointSim2>> jointSims_;
```

Keep the common `joint2` ownership unchanged.

- [ ] **Step 4: Implement create/query using the existing Joint lifecycle helpers**

`createPrismaticJoint` must:

```text
- validate two distinct valid Body handles
- require at least one Dynamic endpoint, matching current Joint creation policy
- validate finite local anchors, finite unit localAxisA using Wheel's tolerance, and finite referenceAngle
- call existing allocateJoint(bodyIndexA, bodyIndexB, collideConnected)
- fill prismaticJointSim2 at the returned stable slot
- reuse existing collideConnected contact removal, wake, and graph behavior rather than duplicating it
```

`getPrismaticJointData` calculates current world anchors/axis from the latest transforms and returns:

```text
translation = dot(axis, anchorB - anchorA)
lateralError = dot(LeftPerp(axis), anchorB - anchorA)
currentAngle = wrapped/normalized relative angle according to the same convention already used by Revolute, minus referenceAngle
force/torque = cached impulse * invSubStepTime after solver integration exists; before the first solve they are zero
```

- [ ] **Step 5: Run lifecycle/API tests to verify GREEN**

Run:

```bash
cmake --build --preset debug --target prismaticJointWorldTests
ctest --test-dir build -C Debug -R "^prismaticJointWorldTests$" --output-on-failure
```

Expected: PASS for create/query/destroy, ID generation, Body cleanup, reference angle, and collision-filter restoration.

- [ ] **Step 6: Commit World ownership/API integration**

```bash
git add src/dynamics/world.h src/dynamics/world.cpp tests/dynamics/prismatic_joint_world_test.cpp tests/CMakeLists.txt
git commit -m "Integrate basic Prismatic lifecycle"
```

---

### Task 3: Connect Prismatic to World stepping, warm start, island solving, and cached reactions

**Files:**
- Modify: `src/dynamics/world.cpp`
- Modify: `tests/dynamics/prismatic_joint_world_test.cpp`
- Modify if required by existing generic dispatch shape only: `src/dynamics/world.h`

**Interfaces:**
- Consumes: Task 1 prepare/warm/solve functions; Task 2 stable `prismaticJointSim2` storage/API.
- Produces: Prismatic participation in the same island-local Joint prepare/warm-start/bias/relax/store path as existing Joint types.

- [ ] **Step 1: Extend the World test with failing dynamic/integration behaviors**

Add assertions for:

```text
- free-axis motion: an X-axis Prismatic does not remove X velocity over Step.
- lateral lock: Y velocity/offset is driven toward the rail.
- angular lock: relative angular velocity/angle is driven toward referenceAngle.
- off-center anchors produce coupled linear/angular reaction without NaN.
- Dynamic-Dynamic endpoints exchange reaction while preserving expected unconstrained axial motion.
- Kinematic-Dynamic uses kinematic velocity in Cdot but never modifies the kinematic endpoint through inverse mass/inertia.
- connected sleeping endpoint wakes with the active Joint component using existing graph behavior.
- pose/mass changes clear stale cached Prismatic impulse through existing Joint reset paths.
- changing step/substep timing follows existing Joint warm-start invalidation policy.
```

- [ ] **Step 2: Run the dynamic World test and verify RED**

Run:

```bash
cmake --build --preset debug --target prismaticJointWorldTests
ctest --test-dir build -C Debug -R "^prismaticJointWorldTests$" --output-on-failure
```

Expected: lifecycle tests may pass, but physics assertions fail because Prismatic is not yet dispatched in the solver.

- [ ] **Step 3: Add Prismatic to prepare/constraint variant dispatch**

Extend the existing island Joint constraint variant with `prismaticJointConstraint2`. In the `std::visit` preparation branch, map `prismaticJointSim2` to `preparePrismaticJointConstraint(...)`. Do not reorder Contact phases or existing Joint types beyond the minimal variant/dispatch extension.

- [ ] **Step 4: Add warm-start, solve, and impulse-store dispatch**

Extend existing `std::visit` branches so `prismaticJointConstraint2` calls:

```cpp
warmStartPrismaticJointConstraint( ... );
solvePrismaticJointConstraint( ..., useBias );
```

When storing the solved constraint back into `jointSims_`, map `prismaticJointConstraint2 -> prismaticJointSim2` and persist the two-component impulse. Preserve existing reset behavior for mass/pose/timestep changes; add only the type branch required for Prismatic to participate.

- [ ] **Step 5: Expose reaction force/torque from the stored impulse**

Track the latest inverse substep time the same way current Joint query data derives reaction force/torque. `force` is lateral only in Stage 1:

```text
perp = LeftPerp(currentWorldAxis)
force = (impulse.x * invH) * perp
torque = impulse.y * invH
```

No axial force exists yet because limit/motor/spring are Stage 2-4.

- [ ] **Step 6: Run focused and neighboring Joint regressions**

Run:

```bash
cmake --build --preset debug --target prismaticJointTests prismaticJointWorldTests distanceJointTests distanceJointWorldTests revoluteJointTests revoluteJointWorldTests wheelJointTests wheelJointWorldTests
ctest --test-dir build -C Debug -R "^(prismaticJointTests|prismaticJointWorldTests|distanceJointTests|distanceJointWorldTests|revoluteJointTests|revoluteJointWorldTests|wheelJointTests|wheelJointWorldTests)$" --output-on-failure
```

Expected: all PASS.

- [ ] **Step 7: Commit solver integration**

```bash
git add src/dynamics/world.cpp src/dynamics/world.h tests/dynamics/prismatic_joint_world_test.cpp
git commit -m "Solve basic Prismatic joints"
```

---

### Task 4: Add the Stage 1 Prismatic Sandbox rail demo

**Files:**
- Modify: `sandbox/demo.h`
- Modify: `sandbox/demo.cpp`
- Modify: `sandbox/rigidBodyDemo.h`
- Modify: `sandbox/rigidBodyDemo.cpp`
- Modify: `sandbox/rigidBodyJointDemo.cpp`
- Modify: `sandbox/jointDemoView.cpp`
- Modify: `sandbox/rigidBodyDemoUi.cpp` only if debug drawing remains centralized there
- Modify: `tests/sandbox/joint_demo_test.cpp`
- Modify: `tests/sandbox/demo_ui_test.cpp` only for existing ImGui headless coverage

**Interfaces:**
- Consumes: `world::createPrismaticJoint`, `world::getPrismaticJointData`.
- Produces: `demoKind::prismaticRail`, `rigidBodyDemo::getPrismaticJoint()`, a dedicated rail scene and observation UI.

- [ ] **Step 1: Write the failing Sandbox model test**

Extend `tests/sandbox/joint_demo_test.cpp` so the registry includes a new Joint-category Prismatic demo and assert:

```text
- scene creates one static rail/reference Body and one Dynamic slider with one Prismatic Joint.
- initial joint data is finite and near zero lateral/current-angle error.
- applying an axial impulse then stepping changes currentTranslation.
- applying lateral/angular impulse then stepping keeps lateral/current-angle error bounded and much smaller than unconstrained motion.
- mouse drag can act on the slider without destroying the Prismatic Joint.
- reset recreates a valid fresh Joint handle.
```

- [ ] **Step 2: Run Sandbox model tests and verify RED**

Run:

```bash
cmake --build --preset debug --target sandboxJointDemoTests
ctest --test-dir build -C Debug -R "^sandboxJointDemoTests$" --output-on-failure
```

Expected: compile/test failure because the new demo kind/scene/accessor do not exist.

- [ ] **Step 3: Add `demoKind::prismaticRail` and construct the rail scene**

Add one Joint category entry named consistently with the Korean Sandbox naming, e.g. `"프리즈매틱 레일"`. `createPrismaticRail()` should use a clearly visible horizontal or mildly angled static rail reference and one Dynamic slider. Create the Joint with Stage 1 fields only.

Add `jointId prismaticJoint_` and `getPrismaticJoint()` in `rigidBodyDemo`, following declaration/definition order conventions.

- [ ] **Step 4: Add observation/debug drawing and controls**

Draw:

```text
- current world axis and perpendicular direction
- anchor A/B
- current slider translation
- lateral error direction
- relative angle indication
```

The control panel should explain Stage 1 in plain language:

```text
축 방향 이동은 자유
축 수직 이동은 잠김
상대 회전은 잠김
```

Provide buttons for axial impulse, lateral impulse, and angular impulse so each allowed/blocked DOF can be observed independently. Do not add fake limit/motor/spring controls yet.

- [ ] **Step 5: Run Sandbox model/UI regressions**

Run:

```bash
cmake --build --preset debug --target sandboxJointDemoTests sandboxDemoTests
ctest --test-dir build -C Debug -R "^(sandboxJointDemoTests|sandboxDemoTests)$" --output-on-failure
```

Expected: PASS. On Windows, the existing headless ImGui path must also render/select the new demo without assertion or invalid handle use.

- [ ] **Step 6: Commit the demo**

```bash
git add sandbox tests/sandbox
git commit -m "Add Prismatic rail sandbox demo"
```

---

### Task 5: Document Stage 1 learning results and run release/full verification

**Files:**
- Create: `docs/basic-prismatic-joint.md`
- Modify: `tests/README.md`
- Modify: `.github/workflows/cmake.yml`
- Modify only if the docs index already lists Joint learning pages: `docs/README.md`

**Interfaces:**
- Consumes: completed Stage 1 implementation and observed tests.
- Produces: learning record for the user and Release CI coverage for `prismaticJointTests`/`prismaticJointWorldTests`.

- [ ] **Step 1: Write the Stage 1 learning note**

`docs/basic-prismatic-joint.md` must explain, using the actual final variable names:

```text
1. Why Prismatic is "lock two DOFs, leave one" rather than "apply force along an axis".
2. `axisA` vs `perpA` and why the axis itself is not constrained in Stage 1.
3. `rA/rB`, `d`, `s1/s2` and why off-center anchors couple linear/angular motion.
4. The 2x2 K matrix and why lateral + angular rows are solved together.
5. accumulated `impulse.x/.y`, warm start, bias pass, position integration, relaxation pass.
6. Wheel comparison: same rail-like lateral row, but Wheel deliberately leaves relative rotation free.
7. What the Sandbox axial/lateral/angular impulse buttons demonstrate.
```

Do not claim limit/motor/spring support; explicitly point to Stage 2 as next work.

- [ ] **Step 2: Add the two Prismatic runtime tests to Windows Release CI**

Append `prismaticJointTests` and `prismaticJointWorldTests` to both the Release build target list and the matching CTest regex in `.github/workflows/cmake.yml`. Update `tests/README.md` counts/list only from the actual final CTest output.

- [ ] **Step 3: Run the full local suite**

Run:

```bash
cmake --build --preset debug --parallel 4
ctest --test-dir build -C Debug --output-on-failure
cmake --build --preset release --target prismaticJointTests prismaticJointWorldTests sandboxJointDemoTests --parallel 4
ctest --test-dir build -C Release -R "^(prismaticJointTests|prismaticJointWorldTests|sandboxJointDemoTests)$" --output-on-failure
```

Expected: all existing Debug tests plus the new Prismatic/Sandbox tests PASS; selected Release runtime tests PASS.

- [ ] **Step 4: Review the branch diff for accidental physics/format churn**

Compare against `master` and verify:

```text
- no existing Contact/Distance/Revolute/Wheel equations changed except generic dispatch additions required for the new variant.
- no unrelated world.cpp split/refactor.
- no Stage 2-4 public fields leaked into Stage 1.
- no broad formatting/line-break churn.
- source definition order follows header declaration order.
```

- [ ] **Step 5: Commit documentation/CI coverage**

```bash
git add docs tests/README.md .github/workflows/cmake.yml
git commit -m "Document basic Prismatic Joint"
```

- [ ] **Step 6: Open PR, run CI, review patches, and merge when green**

Open a Stage 1 PR against `master`. Require Ubuntu Debug build/test and Windows Debug + Release lifecycle tests to pass. Review every changed-file patch for unintended math/format changes. When CI is green and the diff matches this plan, merge directly and verify the resulting `master` workflow run.
