# Building RedBolt.CSG

Keep this repository beside RedBolt and RedBolt.Tests:

```text
RedBolt/
RedBolt.CSG/
RedBolt.Tests/
```

Build the Linux x64 rendering module and samples:

```sh
ps -eo pid=,comm=,args= | rg '[e]mulator|[q]emu-system' || true
dotnet build RedBolt.CSG.slnx --configuration Debug -m:4
```

The solution accepts the same platform, architecture, module, and sample selection properties as RedBolt. Its saved selection is independent and is written below `build/BuildConfig.props`.

Build the aggregate WebAssembly or Android sample hosts directly. Use one build invocation at a time and do not use four build jobs while an Android emulator is running:

```sh
dotnet build src/RedBolt/Sample/Web/RedBolt.Sample.Web.csproj --configuration Release -m:4 -p:RedBoltPlatform=browser-wasm
dotnet build src/RedBolt/Sample/Android/RedBolt.Sample.Android.csproj --configuration Debug -f net10.0-android -m:4 -p:RedBoltPlatform=android-x64 -p:RedBoltBuildAllAndroidPackages=false
```

For Android, use the workspace JDK and SDK described by the parent `AGENTS.md`. Managed assemblies, sample outputs, and packages are written below this repository's `build/` directory. Base RedBolt outputs remain below `../RedBolt/build/`.
