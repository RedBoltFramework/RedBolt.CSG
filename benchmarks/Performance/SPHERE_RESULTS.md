# CSG worker comparison with the built-in sphere, 2026-09-25

The input is RedBolt core's `CoreData/Models/Sphere.mdl` at SHA-256
`4ee44b63a0aed89bb78a3dd4bd6067d27104fafa42bd6c7596654bf2389e7620`.
It has 1,280 triangles and bounds approximately `[-0.5, 0.5]` on each axis.
The benchmark places scaled copies into the same unit bounding cubes and at
the same coordinates as the earlier box cases. A 16-sphere operand therefore
has 20,480 input triangles.

The comparison uses CSG `master` at `f56b784`, the managed BVH from
`codex/csg-current` at `eeedb0d`, and the TinyBVH branch at `1f9999c`, all
against RedBolt core `master` at `80fc4999`. The TinyBVH bridge uses its
separately built Linux x64 native library.

## Worker time

**Median milliseconds**, four warmups and 15 measured iterations per row,
on one pinned logical CPU of an AMD Ryzen 7 5825U. Runtime: .NET 10.0.12,
Release, tiered compilation disabled. All three variants ran the same harness.

| Sphere case | Phase | Master BSP | Managed BVH | TinyBVH |
| --- | --- | ---: | ---: | ---: |
| 1 disjoint union | full | 90.27 | 45.89 | 45.37 |
| 1 disjoint union | append | 90.21 | 41.50 | 40.60 |
| 16 disjoint union | full | 7,323.47 | 6,191.90 | 6,134.22 |
| 16 disjoint union | append | 7,321.77 | 6,044.40 | 5,944.00 |
| Contained difference | full | 152.99 | 46.38 | 47.21 |
| Contained difference | append | 154.93 | 43.25 | 42.70 |
| Overlapping difference | full | 97.92 | 146.53 | 143.23 |
| Overlapping difference | append | 97.78 | 136.69 | 135.08 |
| Local cut in 16 spheres | full | 3,046.15 | 3,481.94 | 3,453.26 |
| Local cut in 16 spheres | append | 3,040.05 | 3,333.60 | 3,339.40 |

`full` constructs both operands. `append` reuses the cached first solid for
the managed and TinyBVH implementations, as `CsgRoot` does when a brush is
appended to a flat operation tree. Master has no cached-solid append path and
performs the full rebuild in both phases. The benchmark runs the production
polygon boolean, triangulation, and `Csg.PrepareModel` packing functions on
a dedicated background thread. It also includes the source polygon cloning
performed by master's worker. Source model loading and polygon extraction
occur on the main thread before timing, as they do in `CsgRoot.Rebuild`.
Queue dispatch, logging, and final model application are outside the timing.

## Allocation and result checks

Median bytes allocated by the worker in the **full** phase:

| Sphere case | Master BSP | Managed BVH | TinyBVH |
| --- | ---: | ---: | ---: |
| 1 disjoint union | 53,554,768 | 23,491,096 | 25,501,472 |
| 16 disjoint union | 590,276,168 | 387,362,520 | 420,458,952 |
| Contained difference | 102,102,704 | 24,550,104 | 26,532,880 |
| Overlapping difference | 79,345,976 | 114,554,496 | 117,801,208 |
| Local cut in 16 spheres | 520,084,624 | 770,285,480 | 804,157,696 |

All variants produced matching signed volumes within `0.000012` for each
case. The managed and TinyBVH branches produced identical face counts and
packed-output checksums. Master emitted more faces for the disjoint unions
and contained difference. Signed volume agreement is a smoke check rather
than a complete geometric equivalence proof.

On these sphere cases, the managed BVH is substantially faster than master
for disjoint and contained operations, while master remains faster on the
overlapping cut and the local cut across 16 spheres. TinyBVH changes total
worker time by at most about 2.3% relative to the managed builder and
allocates more in every full case. This benchmark includes the full output
packing cost, so it does not isolate the BVH builder's own contribution.

For comparison, the same worker harness measured the original **box** inputs
under identical runtime settings. Full rebuild median milliseconds were:

| Box case | Master BSP | Managed BVH | TinyBVH |
| --- | ---: | ---: | ---: |
| 1 disjoint union | 0.068 | 0.104 | 0.113 |
| 16 disjoint union | 1.314 | 2.001 | 1.996 |
| Contained difference | 0.097 | 0.089 | 0.100 |
| Overlapping difference | 0.126 | 0.447 | 0.461 |
| Local cut in 16 boxes | 1.226 | 1.345 | 1.338 |

These box controls use the worker workload. The older [box benchmark](RESULTS.md)
timed a narrower boolean and triangulation path with different JIT settings;
its timings should be read separately. See [README.md](README.md) for the
command to reproduce the worker measurements. The recorded output, including
sample percentiles, face counts, volumes, and checksums, is in
[results/2026-09-25](results/2026-09-25).
