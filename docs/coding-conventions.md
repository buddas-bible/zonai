# Zonai Coding Conventions

This document records the conventions already used by the project and makes the intended distinctions explicit.
The goal is consistency without hiding the data-oriented structure inherited from Box2D.

## 1. Scope

These rules apply to code owned by Zonai under `src/`, `tests/`, and `sandbox/`.
Third-party code is never reformatted to match Zonai.

## 2. Formatting

- UTF-8 without BOM, LF line endings, one final newline.
- Four spaces for indentation. Tabs are not used.
- Allman braces.
- Keep one space inside call/control parentheses: `if( condition )`, `Foo( value )`.
- Do not put a space between a function/control keyword and its opening parenthesis.
- Prefer one argument per line once a call or declaration is wrapped.
- Prefer direct braced initialization: `Type value{};`, not `Type value = {};`.
- Remove trailing whitespace.
- Close named namespaces with a comment: `} // namespace zonai`.

## 3. Includes

For a `.cpp` file:

1. Its matching project header.
2. A blank line.
3. Standard-library headers, alphabetically.
4. A blank line.
5. Other project headers, alphabetically.

For a header:

1. Standard-library headers, alphabetically.
2. A blank line.
3. Project headers, alphabetically.

Use project-root include paths such as `"math/vec2.h"` rather than sibling-relative spellings such as `"vec2.h"`.

## 4. Naming

The project intentionally has two type naming layers.

- Public/domain engine types use PascalCase: `World`, `Body`, `Shape`, `BodyId`, `ContactData`.
- Low-level mathematical, geometric, narrow-phase, and solver-layout value types keep the existing lower camel + `2` naming:
  `vec2`, `aabb2`, `polygon2`, `localManifold2`, `contactSim2`, `contactConstraint2`.

This distinction is intentional. Do not rename one layer merely to make every type visually identical.

Other naming rules:

- Functions: PascalCase.
- Local variables and struct fields: lowerCamelCase.
- Owning class data members: lowerCamelCase with a trailing underscore.
- File/class-scope configuration constants: UPPER_SNAKE_CASE.
- Local algorithmic constants that only explain a formula may stay lowerCamelCase.
- Namespaces: lower-case.

## 5. Types and const

- Persistent IDs and stored indices use exact-width integers such as `std::int32_t`.
- Container sizes and generic container indices use `std::size_t`.
- Small fixed solver/manifold counts may use `int` where that matches the algorithm and cannot grow with storage.
- Prefer `const` for locals that are not reassigned.
- Avoid adding abstraction solely for stylistic uniformity; hot-path layout and algorithmic intent take priority.

## 6. Comments

- Comments explain intent, invariants, ownership, or the reason behind an algorithm.
- Avoid comments that merely restate the next line.
- Short Korean comments use the concise declarative style already dominant in the engine, usually ending in `~함.`, `~임.`, or `~됨.`.
- Keep established physics terms in English when translating them would make the code less recognizable: broad-phase, narrow-phase, manifold, warm start, impulse, solver, contact.
- Use block comments for derivations and multi-step solver equations.

## 7. Structure

- Do not repeat an access specifier without a meaningful section boundary.
- Keep public API, private helpers, and storage visually separated.
- Do not qualify names with `zonai::` while already inside `namespace zonai` unless disambiguation is needed.
- Formatting-only cleanup must not change simulation behavior.
