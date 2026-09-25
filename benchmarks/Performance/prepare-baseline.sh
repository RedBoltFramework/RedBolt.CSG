#!/usr/bin/env bash
set -euo pipefail

benchmark_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
csg_root="$(cd -- "$benchmark_dir/../.." && pwd)"
baseline_dir="$benchmark_dir/Baseline"
mkdir -p "$baseline_dir"

# Pin the CSG master revision used for the published comparison.
baseline_commit=f56b784
for source_file in Csg.cs CsgBsp.cs CsgCommon.cs CsgOptimization.cs CsgTriangulation.cs; do
    git -C "$csg_root" show "$baseline_commit:src/RedBolt/CSG/$source_file" \
        | sed 's/^namespace RedBolt\.CSG;/namespace BaselineCSG;/' \
        > "$baseline_dir/$source_file"
done
