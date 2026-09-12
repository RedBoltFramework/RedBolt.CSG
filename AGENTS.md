# RedBolt.CSG workspace instructions

This repository contains the optional constructive solid geometry module extracted from the sibling RedBolt repository.

- RedBolt is expected at `../RedBolt` unless `RedBoltRoot` is supplied.
- Tests remain in `../RedBolt.Tests`.
- Preserve the public `RedBolt.CSG` assembly and namespaces.
- Do not introduce dependencies on RedBolt.2D, RedBolt.PhysicsBox2D, or RedBolt.PhysicsJolt.
- Keep Linux x64, Windows x64, WebAssembly, Android x64, and Android arm64 working in Debug and Release.
- Follow the monorepo `doc/VERIFICATION.md`: build affected projects and run focused tests; validate affected platform samples and packages when their behavior or integration changes. Run the full supported matrix for release validation or when requested.
