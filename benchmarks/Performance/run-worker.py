#!/usr/bin/env python3
"""Run the worker-path CSG benchmark with the active RedBolt build paths."""

import argparse
import json
import os
from pathlib import Path
import subprocess


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("variant", choices=("master", "current", "tinybvh"))
    parser.add_argument("--shape", choices=("box", "sphere"), default="sphere")
    parser.add_argument("--samples", type=int, default=15)
    parser.add_argument("--warmups", type=int, default=4)
    parser.add_argument("--case")
    parser.add_argument("--validate", type=int, help="compare against BSP on N random cases")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--only", type=int, help="validate only this case index")
    parser.add_argument("--bvh-build", action="store_true", help="time face tree construction only")
    args = parser.parse_args()

    here = Path(__file__).resolve().parent
    csg_root = here.parent.parent
    csg_project = csg_root / "src/RedBolt/CSG/RedBolt.CSG.csproj"
    if args.variant != "master":
        branch = subprocess.check_output(
            ["git", "-C", str(csg_root), "branch", "--show-current"], text=True
        ).strip()
        expected = f"codex/csg-{args.variant}"
        if branch != expected:
            print(f"note: measuring {args.variant} variant from branch {branch}", flush=True)
    else:
        subprocess.run([str(here / "prepare-baseline.sh")], check=True)

    result = subprocess.run(
        [
            "dotnet", "msbuild", str(csg_project),
            "-p:Configuration=Release",
            "-getProperty:RedBoltRoot",
            "-getProperty:RedBoltCoreOutputDir",
            "-getProperty:RedBoltExternOutputDir",
            "-getProperty:RedBoltHostExternOutputDir",
            "-getProperty:RedBoltToolsOutputDir",
        ],
        check=True, capture_output=True, text=True,
    )
    properties = json.loads(result.stdout)["Properties"]
    env = os.environ.copy()
    env["CSG_VARIANT"] = args.variant
    # Keep one optimized JIT version throughout the measurement sequence.
    env["DOTNET_TieredCompilation"] = "0"
    env["REDBOLT_BENCHMARK_ASSEMBLY_PATHS"] = os.pathsep.join(
        properties[name] for name in (
            "RedBoltCoreOutputDir", "RedBoltExternOutputDir",
            "RedBoltHostExternOutputDir", "RedBoltToolsOutputDir",
        )
    )
    env["REDBOLT_BENCHMARK_SPHERE_MODEL"] = str(
        Path(properties["RedBoltRoot"]) / "bin/CoreData/Models/Sphere.mdl"
    )
    if args.variant == "tinybvh":
        library_dir = str(csg_root / "build/tinybvh")
        env["LD_LIBRARY_PATH"] = os.pathsep.join(
            entry for entry in (library_dir, env.get("LD_LIBRARY_PATH", "")) if entry
        )

    command = [
        "dotnet", "run", "--project", str(here / "WorkerPerformance.csproj"),
        "--configuration", "Release", "-p:BuildProjectReferences=false",
    ]
    if args.variant == "master":
        command.append("-p:CsgBenchmarkOriginal=true")
    command.extend(["--", f"--shape={args.shape}", f"--samples={args.samples}",
                    f"--warmups={args.warmups}"])
    if args.case:
        command.append(f"--case={args.case}")
    if args.bvh_build:
        command.append("--bvh-build=1")
    if args.validate:
        command.extend([f"--validate={args.validate}", f"--seed={args.seed}"])
        if args.only is not None:
            command.append(f"--only={args.only}")
    subprocess.run(command, check=True, cwd=csg_root, env=env)


if __name__ == "__main__":
    main()
