# CSG comparison, 2026-09-25

Measured CSG `master` at `f56b784` (original BSP), `codex/csg-current` at
`5c3b3c4` (managed face BVH), and `codex/csg-tinybvh` at `3f00654` (TinyBVH
builder). All used the same Release build of RedBolt core `master` at
`80fc4999`. The TinyBVH branch
uses native TinyBVH for tree construction and managed code for traversal and
the boolean operation.

Machine: AMD Ryzen 7 5825U, one logical CPU pinned with `taskset -c 2`,
.NET 10.0.12, Linux x64. The harness runs four warmups and 15 timed iterations
per row. Values below are **median elapsed milliseconds**. `cold` includes
operand acceleration structure construction; `warm` reuses those structures.
Both include the boolean operation and final triangulation.

| Case | Phase | Master BSP | Managed BVH | TinyBVH |
| --- | --- | ---: | ---: | ---: |
| 1 disjoint box union | cold | 0.262 | 0.402 | 0.428 |
| 1 disjoint box union | warm | 0.216 | 0.320 | 0.325 |
| 16 disjoint boxes union | cold | 5.722 | 7.230 | 5.941 |
| 16 disjoint boxes union | warm | 4.044 | 5.626 | 4.751 |
| Contained box difference | cold | 0.344 | 0.320 | 0.342 |
| Contained box difference | warm | 0.293 | 0.216 | 0.231 |
| Overlapping box difference | cold | 0.596 | 1.913 | 1.920 |
| Overlapping box difference | warm | 0.533 | 1.860 | 1.824 |
| Local cut in 16 boxes | cold | 5.149 | 5.114 | 4.707 |
| Local cut in 16 boxes | warm | 4.390 | 4.236 | 4.018 |

Median allocated bytes for the cold phase:

| Case | Master BSP | Managed BVH | TinyBVH |
| --- | ---: | ---: | ---: |
| 1 disjoint box union | 126,216 | 193,464 | 200,216 |
| 16 disjoint boxes union | 1,764,472 | 2,801,080 | 2,926,664 |
| Contained box difference | 192,448 | 151,832 | 161,344 |
| Overlapping box difference | 226,840 | 792,912 | 799,224 |
| Local cut in 16 boxes | 2,209,544 | 2,176,296 | 2,243,976 |

TinyBVH was about 18% faster than the managed BVH on the cold 16-box union
and 8% faster on the cold local cut. It allocated more memory in every cold
case. The original BSP was substantially faster on the overlapping single-box
difference (0.596 ms versus about 1.9 ms) and faster on the disjoint unions.
The managed BVH was fastest for containment. These cases do not show a general
performance win for the rework or for TinyBVH.

All three variants produced the same signed volume within 0.000001 for each
case: 2, 32, 0.784, 0.725375, and 15.725375 respectively. The managed and
TinyBVH branches also produced identical face counts and triangulation
checksums in every case. The original BSP emitted more faces on the three
difference cases. Volume equality is a useful smoke check, not a complete
geometric equivalence proof.

The benchmark inputs are synthetic axis-aligned boxes, so these figures do
not predict editor workloads with complex brushes. Measured time includes
input packing and tree copying across the native boundary in the TinyBVH
variant. The TinyBVH bridge in this branch is a Linux x64 experiment and must
be built separately before runtime; it is not part of the normal module
package yet. See [README.md](README.md) to rerun the harness.
