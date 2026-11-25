# R3q — full-range binary32 sine/cosine candidate

Base: [R3p #357](https://github.com/buddas-bible/zonai/pull/357), `4124d5c2741b8fef5e42ce21fe902885fdc1c039`.

## Baseline

[R3p strict run](https://github.com/buddas-bible/zonai/actions/runs/45407924985): both trace producers succeeded; comparison pending when this candidate was prepared. [Release A/B](https://github.com/buddas-bible/zonai/actions/runs/45407924878) succeeded, no nonfinite faces/PCG failures in 192 measured rows, maximum face-speed paired delta 1e-7 m/s, field median 7.2361→7.2552 ms at h=0.25/8 Fluid substeps. The short benchmark is not a 180-tick stability proof.

## Design

- Add deterministicSinCos for any finite IEEE binary32 radians. Keep existing bounded contract and exact bounded arithmetic; share its fixed Horner evaluation kernel. Nonfinite input returns two canonical qNaNs.
- For larger angles, use a Payne–Hanek style reduction (see [fdlibm implementation](https://www.netlib.org/fdlibm/k_rem_pio2.c) for the algorithmic background). Independently implement fixed-size arithmetic using floor((2/pi)*2^256), 8 unsigned 32-bit limbs and a 24-bit significand. The constant is verified using 130-decimal-digit arithmetic. Product fits nine limbs; binary point 152..277, quotient low two bits plus rounding bit give nearest quadrant. Two's complement before fraction reconstruction avoids cancellation near integer boundaries. Reconstruct fractional distance in binary64 by powers of two and multiply by binary64 pi/2. No floating-point huge quotient or std::fmod/remainder reduction. Truncated constant contributes less than 2^-128 absolute error in x*2/pi for finite binary32. That bound alone is not a proof of correct rounding or quadrant selection for every input near a half-integer.
- Do not migrate rot2 in this PR. Preserve legacy large-angle behavior of public rotation until CI validates the candidate; no new production call sites. Shared bounded evaluation refactor must preserve existing raw bits.
- Add 19 exact cases generated from exact binary32 inputs with 130-digit reference arithmetic (including both FLT_MAX signs and the dispatch boundary), 3 nonfinite cases, and 4,032 exponent-stratified samples (32 mantissas for each biased exponent 129..254). Check opposite sign symmetry, finite range, rotation norm and numerical agreement with double libm, not platform libm bit equality. Verify bounded grid/seed results also match new entry point. Additional strict fingerprint 8,108 rows; total 22,716 expected.
- Independent Python reconstruction of the integer reduction and polynomial matched all 19 reference cases and the 4,032 stratified samples against rounded host double sin/cos, maximum relative error 5.82e-8, maximum norm error 8.22e-8. This is a prototype check, not C++/MSVC validation nor exhaustive oracle.

## Acceptance

Review R3p comparison first. Run both trace producers, strict math and existing Fluid/World/Joint rows, full CMake. If candidate passes, separately migrate rot2 large-angle fallback and extend source replay beyond 2*pi. Keep nonfinite API semantics under review. No merge to master, no change to experimental #326. No universal correct-rounding proof, architecture guarantee or long-run stability claim. Direct libm paths outside rot2 still require separate audits.
