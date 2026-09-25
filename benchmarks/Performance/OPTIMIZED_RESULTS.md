# Optimized CSG worker results, 2026-09-25

These changes optimize the managed solid pipeline from `codex/csg-current`. All variants below were
measured on the same machine with `run-worker.py` defaults: four warmups, 15 measured calls, tiered
compilation disabled, pinned with `taskset -c 2` on an AMD Ryzen 9 3950X (Linux x64, .NET 10.0.12,
RedBolt core `80fc4999`). Absolute timings are not comparable with [SPHERE_RESULTS.md](SPHERE_RESULTS.md),
which used a different machine. Raw output is in [results/2026-09-25-optimized](results/2026-09-25-optimized).

## Worker time, full rebuild

**Median milliseconds.** `Before` is the unmodified managed BVH at `6360376`.

| Sphere case | Master BSP | Managed BVH before | Optimized | vs master | vs before |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 disjoint union | 118.62 | 62.64 | 5.63 | 21× | 11× |
| 16 disjoint union | 10,118.30 | 8,664.70 | 159.17 | 64× | 54× |
| Contained difference | 206.34 | 65.33 | 7.07 | 29× | 9.2× |
| Overlapping difference | 131.79 | 198.76 | 10.69 | 12× | 19× |
| Local cut in 16 spheres | 4,207.49 | 4,870.19 | 65.85 | 64× | 74× |

| Box case | Master BSP | Managed BVH before | Optimized | vs master | vs before |
| --- | ---: | ---: | ---: | ---: | ---: |
| 1 disjoint union | 0.103 | 0.153 | 0.082 | 1.3× | 1.9× |
| 16 disjoint union | 2.112 | 2.996 | 1.025 | 2.1× | 2.9× |
| Contained difference | 0.157 | 0.133 | 0.058 | 2.7× | 2.3× |
| Overlapping difference | 0.198 | 0.677 | 0.185 | 1.1× | 3.7× |
| Local cut in 16 boxes | 1.932 | 2.009 | 0.681 | 2.8× | 3.0× |

With a cached first solid (`append`), the optimized 16-sphere union takes 85.9 ms and the local cut
34.1 ms. Allocation in the full 16-sphere union fell from 590 MB (master) and 387 MB (before) to 87 MB.

Checksums of the packed output are unchanged from the managed branch for every box case and for the
disjoint and contained sphere cases. The overlapping and local-cut sphere cases now take the solid path
instead of falling back to BSP, so they emit fewer faces (1,355 instead of 2,730 for the overlapping
spheres) with the same signed volume within `0.000001`.

## What changed

- **Plane grouping.** `SimplifyPolygons` compared each polygon with every earlier coplanar group,
  which is quadratic on curved meshes where every triangle has its own plane. A grid over unit normal
  and plane distance returns the same first-matching group. This alone made the 16-sphere union 10×
  faster and also applies to the BSP path.
- **Packed-key hashing.** `ulong.GetHashCode` XORs the two halves, so edge keys `(from << 32) | to`
  collided heavily. All packed-key dictionaries now use a mixing comparer.
- **Fallback removal.** Overlapping operands previously produced an open result and repeated the whole
  boolean with BSP. Vertex rounding split coincident intersection points across grid cells, near-surface
  fragments were misclassified because ray hits closer than epsilon were discarded, and collapsed needle
  fragments looked like non-manifold edges. Topology now welds vertices within epsilon, ignores
  collapsed loops, and point classification counts every forward hit.
- **Local splitting.** Frontier faces were cut by three infinite planes per intersection segment,
  producing a quadratic number of fragments. Only pieces a segment crosses are now cut, by the opposite
  face plane.
- **Result assembly.** Untouched operand components are copied into the result with their topology;
  only contact regions are rebuilt.
- **Two-level BVH.** A tree over component bounds sits above per-component binned-SAH face trees built
  on first use. Operands with disjoint bounds build no trees, and a local cut builds the tree of the one
  component it touches.
- **Data layout and allocation.** Queries use flat `System.Numerics.Vector3` triangle arrays and SIMD
  box tests without delegates or per-call allocation. Internal polygon copies share vertex attribute
  arrays; `ToPolygons`, `CsgPolygon.Clone` and triangulated output still return independent copies.

## TinyBVH

The TinyBVH branch replaces only BVH construction. After these changes, face-tree construction is about
2% of worker time in the local cut and 6% in the overlapping difference, and disjoint operands build no
tree. A native builder could therefore save at most a few percent while adding native builds for every
platform and a managed/native copy of each tree. The managed builder is retained.

## Robustness without BSP

The solid path no longer falls back to BSP. Operands must be closed meshes and every result is closed:

- Coplanar contact is handled directly: faces cut each other along their edges, and fragments in the
  shared region follow the orientation rule instead of a ray test.
- Intersection curve vertices from both operands are merged within 8 epsilons and cut vertices snap to
  them, so both sides of a seam share identical vertices.
- Any remaining seam gap is healed: short unmatched edges are collapsed, remaining loops are filled, and a
  gap that still remains is collapsed entirely. A hole would break later operations, so a small local
  loss of accuracy is preferred.
- `CsgRoot` runs booleans in the root frame with its world scale applied, so non-uniformly scaled roots do
  not distort operands or epsilon.

## Validation

`run-worker.py current --validate N [--seed S] [--only I]` builds random chained operations,
`(A op B) op C`, over the closed core Box, Sphere, Cylinder, Cone, Torus and Pyramid models. It compares
point membership of the solid result and of the BSP result against the exact boolean of the inputs, using
generalized winding numbers on 3,000 sample points, and fails a case whose result has any unmatched edge
or whose error exceeds both 0.5% and twice the BSP error. Six runs of 300 cases (seeds 1, 7, 23, 42, 99
and 1234) produced watertight results in all 1,800 cases. One case, seed 1234 #204, had 2.57% membership
error: a misclassified region was closed by healing. The solid path took about 3 s per 300 cases against
about 13 s for BSP.
