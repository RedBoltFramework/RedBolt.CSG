# CSG performance comparison

This harness compares the CSG `master` BSP revision `f56b784`, the managed BVH
on `codex/csg-current`, and the TinyBVH builder on `codex/csg-tinybvh`. All
variants use the same input polygons and the same RedBolt core build. The
baseline sources are generated from the pinned commit in a separate namespace.

Run `./prepare-baseline.sh` once. Build `RedBolt.slnx` in Release and build the
CSG project from the branch being measured. For the TinyBVH branch, also build
`benchmarks/TinyBvh` as documented there and add its output directory to
`LD_LIBRARY_PATH`. Then run, for example:

```sh
CSG_VARIANT=master dotnet run --project benchmarks/Performance/CsgPerformance.csproj \
    --configuration Release -p:CsgBenchmarkOriginal=true -p:BuildProjectReferences=false
CSG_VARIANT=current dotnet run --project benchmarks/Performance/CsgPerformance.csproj \
    --configuration Release -p:BuildProjectReferences=false
```

The program prints result face count and signed volume as a basic shape check,
then CSV rows: case, phase, median ms, lower sample ms, upper sample ms, median
allocated bytes, and triangulated output checksum. Each row uses four warmups
and 15 measured calls with a full GC before each call. `cold` constructs the
operand acceleration structures; `warm` reuses them. Both phases perform the
boolean operation and triangulate the final polygons. These synthetic box cases
measure CSG throughput, including managed/native transfer in the TinyBVH
variant. They do not isolate BVH construction itself.

## Worker workload with the built-in sphere

`run-worker.py` measures the CPU work performed by `CsgRoot`'s build job on a
dedicated background thread. It loads `CoreData/Models/Sphere.mdl` and extracts
source polygons on the main thread, as `CsgRoot.Rebuild` does. The timed work
constructs operands, performs the boolean operation, triangulates the result,
and packs vertex and index data with `Csg.PrepareModel`. It excludes model
loading, scene traversal, queue dispatch, logging, and application of the
renderable model on the main thread.

`full` reconstructs the whole result. `append` reuses the cached first solid
on the managed and TinyBVH branches, matching an appended brush in a flat
operation tree. Master has no cached-solid path and performs the full work in
both phases. Four warmups and 15 timed calls are the defaults; tiered JIT
compilation is disabled to keep the code version stable during measurement.

Build the branch under test first, then run each variant with the same CPU
pinning. The runner queries MSBuild for the active build paths and supplies
the TinyBVH library path when needed.

```sh
taskset -c 2 ./benchmarks/Performance/run-worker.py master --shape sphere
taskset -c 2 ./benchmarks/Performance/run-worker.py current --shape sphere
taskset -c 2 ./benchmarks/Performance/run-worker.py tinybvh --shape sphere
```

The runner accepts `--shape box` for the original synthetic inputs and
`--case`, `--samples`, and `--warmups` for focused checks. The full sphere
measurements are in [SPHERE_RESULTS.md](SPHERE_RESULTS.md).

The optimized measurements and a description of the changes are in
[OPTIMIZED_RESULTS.md](OPTIMIZED_RESULTS.md); the comparison of the managed SIMD builder with TinyBVH is in
[SIMD_RESULTS.md](SIMD_RESULTS.md). The `codex/csg-current` and `codex/csg-tinybvh` experiment branches
were removed after adoption; the runner measures the checked-out managed code as `current`, and
`--bvh-build` times face-tree construction alone. `--validate N` compares the solid boolean with the
BSP reference on random chained operations; `--seed` and `--only` select and repeat individual cases.
