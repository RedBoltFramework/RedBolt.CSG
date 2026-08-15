# RedBolt.CSG workspace instructions

This repository contains the optional constructive solid geometry module extracted from the sibling RedBolt repository.

- RedBolt is expected at `../RedBolt` unless `RedBoltRoot` is supplied.
- Tests remain in `../RedBolt.Tests`.
- Preserve the public `RedBolt.CSG` assembly and namespaces.
- Do not introduce dependencies on RedBolt.2D, RedBolt.PhysicsBox2D, or RedBolt.PhysicsJolt.
- Keep Linux x64, Windows x64, WebAssembly, Android x64, and Android arm64 working in Debug and Release.
- Run build invocations sequentially. Internal build parallelism may use at most four jobs, and only when no Android emulator or QEMU process is running.
- Build the smallest affected project and run the focused CSG tests after implementation changes.
- At completion, build all supported platform/configuration cells, create and validate packages, run focused tests, and review a captured sample screenshot.
