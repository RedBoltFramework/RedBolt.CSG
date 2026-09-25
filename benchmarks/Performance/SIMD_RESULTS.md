# Managed SIMD versus TinyBVH builders, 2026-09-25

An experimental branch built on the optimized pipeline added a selectable face-tree builder: the managed
builder, TinyBVH's reference builder, or TinyBVH's AVX builder, through regenerated SharpCpp bindings.
Everything else, including queries, topology and splitting, was the same managed code. The managed builder
improvements below were adopted; the TinyBVH bridge was not, and the branch was removed. Measurements use
the same machine as [OPTIMIZED_RESULTS.md](OPTIMIZED_RESULTS.md): AMD Ryzen 9 3950X, one pinned core, .NET
10.0.12, tiered compilation disabled. Raw output is in [results/2026-09-25-simd](results/2026-09-25-simd).

## Instruction sets

The CPU supports AVX2, FMA and BMI2 and has no AVX-512. `DOTNET_JitDisasm` shows that RyuJIT emits
VEX-encoded AVX code for the managed kernels, with `ymm` registers for `Vector256<float>`. The native
bridge is compiled with `-mavx2 -mfma`, which enables TinyBVH's `BVH_USEAVX` path. Both sides therefore
target AVX2. Two differences remain: GCC may contract multiply-adds into FMA, and RyuJIT does not.

The first comparison was not equivalent. `Vector256.Max`, `Vector128.Min` and `Vector3.Min` implement
IEEE 754-2019 semantics for NaN and signed zero, which RyuJIT lowers to about nine compare and blend
instructions instead of one `vmaxps`. TinyBVH uses `_mm256_max_ps` directly. The managed builder and query
kernels now use `MinNative` and `MaxNative`, which compile to `vminps`, `vmaxps`, `vminss` and `vmaxss`.
The inputs cannot contain NaN in these kernels, so results are unchanged.

The original TinyBVH experiment (`codex/csg-tinybvh` at `c1c9b4e`) did not exercise any SIMD builder. It
passed AABBs through TinyBVH's callback overload, which always uses the scalar reference builder, and
compiled without AVX flags. TinyBVH's AVX builder accepts only triangles; the bridge passes each item box
as the degenerate triangle `(min, max, max)`, whose bounds equal the box.

## Managed builder changes

The managed builder now follows TinyBVH's AVX design:

- Each bin is one `Vector256<float>` holding (−min, max), so growing a bin is a single `vmaxps`.
- Primitives use the same 32-byte layout; one XOR produces the bin value.
- Bin indices for all three axes come from one vector conversion and are clamped as integers.
- Binning is software-pipelined and uses doubled centroids over the doubled node box, with eight bins,
  removing the separate centroid-bounds pass.

## Tree construction alone

**Median milliseconds** over 31 builds of the whole face tree for copies of the built-in sphere. TinyBVH
timings include copying boxes into and nodes out of the native library.

| Faces | Managed | TinyBVH reference | TinyBVH AVX |
| ---: | ---: | ---: | ---: |
| 1,280 | 0.35 | 0.65 | 0.35 |
| 20,480 | 6.18 | 11.16 | 5.19 |
| 81,920 | 25.87 | 47.93 | 24.36 |

Before the `MaxNative` change, the managed builder took 7.96 ms for 20,480 faces; before the TinyBVH-style
binning it took 14.03 ms.

## Worker time

**Median milliseconds**, full rebuild, harness defaults.

| Sphere case | Managed | TinyBVH reference | TinyBVH AVX |
| --- | ---: | ---: | ---: |
| 1 disjoint union | 5.45 | 5.44 | 5.46 |
| 16 disjoint union | 152.82 | 151.65 | 150.71 |
| Contained difference | 6.29 | 7.04 | 6.28 |
| Overlapping difference | 9.54 | 10.18 | 9.44 |
| Local cut in 16 spheres | 61.48 | 60.76 | 59.31 |

| Box case | Managed | TinyBVH reference | TinyBVH AVX |
| --- | ---: | ---: | ---: |
| 1 disjoint union | 0.081 | 0.082 | 0.081 |
| 16 disjoint union | 1.022 | 1.025 | 1.028 |
| Contained difference | 0.057 | 0.057 | 0.057 |
| Overlapping difference | 0.176 | 0.177 | 0.178 |
| Local cut in 16 boxes | 0.672 | 0.678 | 0.675 |

Differences are within about 4%. Face trees are built lazily per component, so construction is a small
part of each case, and disjoint operands build none. Overlapping and local-cut checksums differ slightly
between builders because tree order changes the order of intersection fragments; signed volumes agree.

TinyBVH's SIMD traversal layouts return the closest or any hit for rays against triangles. The CSG queries
need every ray hit for parity and box-pair overlap between two trees, so traversal remains managed.

## Validation

`--validate 200` passed for seeds 1 and 7 with both the managed and TinyBVH AVX builders, with one
exception common to both: seed 7 case 85, `(Sphere ∪ Cylinder) ∩ Box`, has 1.17% membership error where
BSP has none. The first boolean now succeeds on the solid path; the second produces a merged box face with
a spike whose two ends lie 1.6 × 10⁻⁴ apart, just outside the weld epsilon, so the result is open and
falls back to BSP on the solid output. Before the builder changes, the first boolean fell back to BSP
instead and the final result matched. This precision case is unrelated to the builder or SIMD changes.

## Conclusion

With matching instruction sets, the managed builder takes at most about 19% longer than TinyBVH's AVX
builder to construct a tree, and the two are indistinguishable in the CSG worker. The managed builder
avoids native builds for Android and WebAssembly and the managed/native copies, so it remains the
recommended implementation.
