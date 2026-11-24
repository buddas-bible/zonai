# R1b — Tick input log, World replay and canonical trace compare

> Depends on [R1a snapshot PR #329](https://github.com/buddas-bible/zonai/pull/329). Base implementation: R1a head `9b885e41`. This PR is **stacked** and must not be merged before R1a.  
> This adds **tests only**. No production physics, World step order, floating-point settings, Fluid code, or Sandbox behavior is changed.  
> The recorded snapshots are **public World/Body observations from R1a**, not yet full authoritative physics state or roll-back serialization.

## Executable

`worldReplayTests` registers as a CTest executable in the existing Windows/Ubuntu Debug pipeline. The CLI supports:

```sh
worldReplayTests
worldReplayTests --record-input recorded-input.zin
worldReplayTests --export baseline.ztr
worldReplayTests --replay recorded-input.zin replay.ztr
worldReplayTests --verify baseline.ztr
worldReplayTests --compare baseline.ztr replay.ztr
```

- **Default** runs file format, command and real World regression tests.
- **--record-input** exports one deterministic fixture's canonical input file.
- **--export** runs the same fixture and exports all snapshots including Tick 0.
- **--replay** reads a recorded input file and runs the World from a fresh initial state.
- **--verify** reruns the input embedded within a trace and compares every Tick and every recorded field.
- **--compare** compares two independently exported trace files (potentially from different OS/compiler), printing **first divergent Tick + named field + raw bit values**, and exits nonzero if they differ.

Use the same recorded command file on all target platforms. Unlike R1a, the binary trace can now be transported and compared between machines. Cross-platform CI artifact collection/comparison is still **R1c**, not part of R1b.

## Binary formats

All fields are fixed-width, little-endian, versioned. Raw IEEE-754 binary32 bits are recorded unchanged. Replay uses `std::bit_cast`; commands with NaN/Inf are rejected.

### Input log ZIN1 (version 1)

```text
u32 magic ("ZIN1")
u16 format version = 1
u32 Tick count
u32 dt float32 bits
u32 World substep count
u32 command count
repeat commands in original order:
    u32 Tick (1-based)
    u8 command type
    u8 Body slot (0 for gravity; 1/2 for tracked dynamic Bodies)
    u32 x float32 bits
    u32 y float32 bits
```

Supported fixture commands: apply force to center, apply impulse to center, set linear velocity, set gravity. Same-Tick command order is **semantically meaningful** and preserved. Validation rejects out-of-range Tick, unsorted input, invalid Body slot/command, invalid `dt`, unbounded counts and non-finite values. The scene itself is hard-coded as a **test fixture**, not generalized World save/restore.

### Snapshot trace ZTR1 (version 1)

```text
u32 magic ("ZTR1")
u16 format version = 1
u32 embedded input log size
bytes: full ZIN1 log
u32 snapshot count = Tick count + 1
repeat snapshot count:
    u32 snapshot size
    bytes: R1a versioned field-wise snapshot
```

Names, widths and full field bytes are compared; the optional FNV checksum in R1a is not used as the equality criterion. The file parser checks lengths, magic/version, malformed/truncated records and unexpected trailing bytes.

### First difference

Input file bytes are compared **before** physics snapshots; distinct input sets must never masquerade as deterministic simulation divergence. Then compare snapshot index `0..tickCount`. Report example:

```text
first divergence: tick=14 field=body[2].linearVelocity.x left=0x.../4 right=0x.../4
```

## R1b self-tests

1. Canonical input log encode/decode/encode yields exactly identical bytes.
2. Unsorted input sequence rejected; same-Tick order retained.
3. Two freshly constructed Worlds execute the same recorded commands at 1/60 s, 4 substeps over 60 Ticks and produce **identical full snapshot bytes at every Tick**.
4. Archived trace encode/decode/encode is bit-identical.
5. One bit tampered in Tick 14 Body 2 linear velocity must report exactly that Tick and field, including after a binary trace round-trip.
6. Truncated trace rejected.
7. Real filesystem input and trace binary writes and reads are lossless; loaded input reproduces the original trace.

## Known incompleteness

**Do not label R1b as cross-platform complete determinism.** It is a practical transport/comparison mechanism plus **same-binary repetition** tests. The R1a snapshot still omits persistent Contact and Joint solver caches, hidden body forces and ID pools, full collision/Shape/Tree/Island state, Fluid velocities, pressure, tracer and geometry history, RNG, and input sources beyond this dedicated fixture. A trace can compare equal while those unrecorded states differ.

Future:
- **R1c:** CI generates identical input and exports per-platform trace artifacts, downloads them into one comparison job and fails on the first differing Tick/field. Decide MSVC/GCC/Clang/Debug/Release environment contract and FP flags explicitly.
- **R2:** Investigate actual divergence from cross-platform comparisons; audit FP math, ordering, hidden authoritative cache/state, and expand fixture coverage.
- **Fluid / S2:** Orthogonal numerical stability and conservation tests remain required. Deterministic pressure spikes are still bugs.

## Release/merging

This PR branches from #329, which is still a Draft against `master`. A temporary PR against `master` can include R1a changes to activate existing GitHub Actions, but **merge order must remain #329 then R1b**, with a post-merge diff check and CI revalidation. Never merge the duplicated stacked diff in one step without review.
