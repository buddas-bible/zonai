# R3p — deterministic bounded wind source

Base: [R3o #356](https://github.com/buddas-bible/zonai/pull/356), `d2ea30759a911c965b30526003b3736ba8faa3aa`.

## Verified evidence

[R3o strict CI](https://github.com/buddas-bible/zonai/actions/runs/45406565579): all ten prior scenes (2,452,433 rows) exact; extended worldFluidDrag180 229,655/1,272,627 differences. Both producer jobs succeeded; engine math 14,608/14,608 exact. First physical trace mismatch tick 13, interval 2 (zero-based), projection.continuityRms: Ubuntu 3a0dcf4f, Windows 3a0dcf16. Full raw artifacts 14925467541 and 14926011592 compared independently. The source recurrence has identical input angles, four float sin mismatches and three Y increment mismatches. The first increment difference precedes the first recorded physical mismatch by one interval.

| Zero-based source index | Tick/interval (interval zero-based) | Angle | Ubuntu sin | Windows sin | Engine sin | Ubuntu deltaY | Windows deltaY | Engine deltaY |
|---|---|---|---|---|---|---|---|---|
| 49 | 13/1 | 3eb55554 | 3eb190ee | 3eb190ed | 3eb190ed | 3bbd6765 | 3bbd6764 | 3bbd6764 |
| 372 | 94/0 | 40291814 | 3ef53d84 | 3ef53d85 | 3ef53d85 | 3c02cb7a | 3c02cb7b | 3c02cb7b |
| 394 | 99/2 | 40331149 | 3eac831e | 3eac831d | 3eac831d | 3bb80354 | 3bb80353 | 3bb80353 |

Index 434 has a sine mismatch without a rounded increment mismatch. The source reconstruction is independent of the actual production locals. The source-region sample and projection summaries at tick 13 interval 1 are identical; they do not establish identical full arrays at that interval. Thus the time ordering supports a narrow migration candidate, not proven sole cause.

## Changes

- Replace only the wind source `std::sin(1.7f * windTime_)` in `sandbox/rigidBodyDemo.cpp` with `rot2::FromRadians(1.7f * windTime_).s`. Reuse validated bounded dispatch; preserve all time, amplitude, injection and solver equations. No change to src physics.
- In the 720-interval recurrence audit use the production rotation factory, verify it equals the bounded helper, assert exact angle/sine/increment bits for the three historical cases. Preserve libm outputs separately, 14,608 authoritative math rows and all Fluid trace coverage unchanged.
- Re-pin temporary Release A/B workflow to R3o; include Sandbox path. Existing benchmark is short, does not exercise the first failing interval, and only informs performance/early stability. Strict 180-tick replay is the actual acceptance gate.

## Acceptance / limitations

Both producer jobs, strict math/physics comparison, full CMake, and short Release A/B must be reviewed. If extended mismatch remains, capture actual full stage arrays near tick 13/intervals 1–2 to isolate it. The rotation factory retains its legacy libm fallback outside ±float(2*pi); the 180-tick source angle stays within that bound. Longer than approximately 3.7 simulated seconds of continuous source time can enter the fallback, so full-range rotation reduction must be addressed before claiming arbitrary-duration source determinism. Other direct libm callers, long-run stability, architectures and multithreading remain open. No merge to master or changes to experimental #326.
