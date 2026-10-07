# Joint Folder Structure Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move all Joint-specific runtime and solver files from the flat `src/dynamics` root into `src/dynamics/joints` without changing physics behavior, public type names, or solver order.

**Architecture:** `dynamics` keeps shared Body/Shape/Contact/Island/World and shared constraint utilities. `dynamics/joints` owns only Joint-specific persistent records, hot sim records, and Joint constraint implementations. Consumers include the responsibility-specific path directly; no compatibility or umbrella header is introduced.

**Tech Stack:** C++20, CMake 4.2, Visual Studio 18 2026/MSVC, Ubuntu CMake build, GitHub Actions/CTest.

**Spec:** `docs/superpowers/specs/2026-10-07-joints-folder-structure-design.md`

## Global Constraints

- Move only Joint-specific files listed in the spec into `src/dynamics/joints/`.
- Keep `constraintSoftness2.*`, `contactConstraint2.*`, `island2.*`, `world.*`, `id.h`, and Body state files in `src/dynamics/`.
- Do not change physics algorithms, solver equations, public type/function names, layouts, or execution order.
- Do not add an umbrella header or compatibility aliases for the old include paths.
- Preserve existing formatting and comments except for include-path edits required by the move.
- `src/CMakeLists.txt` already uses recursive source/header discovery, so do not change CMake unless verification proves the move is not discovered.

## Review Focus

- Old `dynamics/<joint-file>` includes must not survive in source, tests, or sandbox; verify by repository search after migration.
- `island2.h` must follow `joint2.h` to the new path without creating a dependency cycle.
- Joint constraint `.cpp/.h` files must include sibling Joint files through `dynamics/joints/...`, while shared Body/softness includes remain under `dynamics/...`.
- Recursive CMake discovery must include the new directory on both Windows and Ubuntu; verify full CI rather than editing CMake preemptively.
- File moves must be content-preserving except for include paths; compare moved files to their source versions and reject unrelated formatting/algorithm churn.

---

### Task 1: Move the Joint subsystem under `dynamics/joints`

**Files:**
- Move: `src/dynamics/joint2.h` → `src/dynamics/joints/joint2.h`
- Move: `src/dynamics/distanceJoint2.h` → `src/dynamics/joints/distanceJoint2.h`
- Move: `src/dynamics/distanceJointSim2.h` → `src/dynamics/joints/distanceJointSim2.h`
- Move: `src/dynamics/distanceJointConstraint2.h` → `src/dynamics/joints/distanceJointConstraint2.h`
- Move: `src/dynamics/distanceJointConstraint2.cpp` → `src/dynamics/joints/distanceJointConstraint2.cpp`
- Move: `src/dynamics/mouseJoint2.h` → `src/dynamics/joints/mouseJoint2.h`
- Move: `src/dynamics/mouseJointSim2.h` → `src/dynamics/joints/mouseJointSim2.h`
- Move: `src/dynamics/mouseJointConstraint2.h` → `src/dynamics/joints/mouseJointConstraint2.h`
- Move: `src/dynamics/mouseJointConstraint2.cpp` → `src/dynamics/joints/mouseJointConstraint2.cpp`
- Move: `src/dynamics/revoluteJoint2.h` → `src/dynamics/joints/revoluteJoint2.h`
- Move: `src/dynamics/revoluteJointSim2.h` → `src/dynamics/joints/revoluteJointSim2.h`
- Move: `src/dynamics/revoluteJointConstraint2.h` → `src/dynamics/joints/revoluteJointConstraint2.h`
- Move: `src/dynamics/revoluteJointConstraint2.cpp` → `src/dynamics/joints/revoluteJointConstraint2.cpp`
- Move: `src/dynamics/wheelJoint2.h` → `src/dynamics/joints/wheelJoint2.h`
- Move: `src/dynamics/wheelJointSim2.h` → `src/dynamics/joints/wheelJointSim2.h`
- Move: `src/dynamics/wheelJointConstraint2.h` → `src/dynamics/joints/wheelJointConstraint2.h`
- Move: `src/dynamics/wheelJointConstraint2.cpp` → `src/dynamics/joints/wheelJointConstraint2.cpp`
- Modify: all moved Joint `.h/.cpp` files whose sibling include still uses `dynamics/<joint-file>`.

**Interfaces:**
- Consumes: existing Joint type/function signatures unchanged.
- Produces: identical Joint interfaces under `dynamics/joints/...` include paths.

- [ ] **Step 1: Establish the structural RED condition**

Change one existing direct consumer include from `dynamics/distanceJointConstraint2.h` to `dynamics/joints/distanceJointConstraint2.h` before moving the file.

- [ ] **Step 2: Run the focused build and verify it fails because the new path does not yet exist**

Run:
```bash
cmake --preset vs2026
cmake --build --preset debug --target distanceJointTests
```
Expected: compile failure reporting missing `dynamics/joints/distanceJointConstraint2.h`.

- [ ] **Step 3: Move all Joint-specific files listed above without altering declarations or implementation bodies**

Only sibling include paths inside moved files may change, for example:
```cpp
#include "dynamics/joints/distanceJointSim2.h"
#include "dynamics/joints/distanceJointConstraint2.h"
```
Shared dependencies such as `dynamics/bodyState.h` and `dynamics/constraintSoftness2.h` remain unchanged.

- [ ] **Step 4: Update direct production consumers**

Modify:
- `src/dynamics/world.h`
- `src/dynamics/island2.h`

Replace old Joint include paths with `dynamics/joints/...`. Do not reorder unrelated includes or declarations.

- [ ] **Step 5: Run focused Joint builds**

Run:
```bash
cmake --build --preset debug --target distanceJointTests mouseJointTests revoluteJointTests wheelJointTests
```
Expected: all four targets build successfully.

- [ ] **Step 6: Commit**

```bash
git add src/dynamics src/dynamics/joints
git commit -m "refactor: group joint implementation files"
```

### Task 2: Update test consumers and prove the old paths are gone

**Files:**
- Modify: `tests/dynamics/distance_joint_test.cpp`
- Modify: `tests/dynamics/mouse_joint_test.cpp`
- Modify: `tests/dynamics/revolute_joint_test.cpp`
- Modify: `tests/dynamics/wheel_joint_test.cpp`
- Modify only if repository search finds a real direct include: any sandbox/source/test file still using one of the moved old paths.

**Interfaces:**
- Consumes: Joint headers now located under `dynamics/joints/` from Task 1.
- Produces: repository-wide direct consumers use only the new include paths.

- [ ] **Step 1: Update the four direct Joint unit-test includes**

Use:
```cpp
#include "dynamics/joints/distanceJointConstraint2.h"
#include "dynamics/joints/mouseJointConstraint2.h"
#include "dynamics/joints/revoluteJointConstraint2.h"
#include "dynamics/joints/wheelJointConstraint2.h"
```
respectively. Change no test expectations or formatting unrelated to includes.

- [ ] **Step 2: Search for stale include paths**

Run repository searches for:
```text
dynamics/joint2.h
dynamics/distanceJoint
dynamics/mouseJoint
dynamics/revoluteJoint
dynamics/wheelJoint
```
Expected: no source/test/sandbox include uses the old paths. Historical docs may retain old paths when they describe past repository state; do not rewrite historical audit/spec documents solely for this move.

- [ ] **Step 3: Run focused tests**

Run:
```bash
ctest --test-dir build -C Debug -R "^(distanceJointTests|distanceJointWorldTests|mouseJointTests|mouseJointWorldTests|revoluteJointTests|revoluteJointWorldTests|wheelJointTests|wheelJointWorldTests)$" --output-on-failure
```
Expected: all selected tests pass.

- [ ] **Step 4: Verify file-content preservation**

Compare every moved source/header with its pre-move version. Expected: differences are path/location plus required `#include` path replacements only; no solver math, constants, comments, declaration order, or formatting churn.

- [ ] **Step 5: Commit**

```bash
git add tests src sandbox
git commit -m "test: follow joint include paths"
```

### Task 3: Full verification and integration

**Files:**
- Verify: `src/CMakeLists.txt`
- Verify: `.github/workflows/cmake.yml`
- Modify: none unless verification exposes a real path-discovery failure.

**Interfaces:**
- Consumes: completed Joint path migration from Tasks 1–2.
- Produces: verified branch ready for merge with no behavior change.

- [ ] **Step 1: Verify recursive CMake discovery**

Confirm `src/CMakeLists.txt` still uses `file(GLOB_RECURSE ...)` for `*.h` and `*.cpp`. Do not add manual Joint source lists.

- [ ] **Step 2: Run full Debug verification**

Run:
```bash
cmake --build --preset debug
ctest --test-dir build -C Debug --output-on-failure
```
Expected: build succeeds and all Debug tests pass.

- [ ] **Step 3: Run Release verification**

Run:
```bash
cmake --build --preset release
ctest --test-dir build -C Release --output-on-failure
```
Expected: Release build succeeds; tests pass subject to the repository's existing Release/assert limitations.

- [ ] **Step 4: Review the branch diff**

Expected diff categories only:
- file renames into `src/dynamics/joints/`
- include-path replacements
- this design/plan documentation

No physics/body changes are acceptable.

- [ ] **Step 5: Open a PR and require GitHub Actions Windows/Ubuntu success**

PR title: `Group joint files under dynamics/joints`

Expected CI:
- Windows Debug build/CTest
- Ubuntu Debug build/CTest
- Windows Release lifecycle Joint targets

- [ ] **Step 6: Merge after green CI**

Use a normal merge commit after verifying the PR head SHA has not moved since review.
