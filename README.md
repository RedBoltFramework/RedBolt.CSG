# RedBolt.CSG

Optional constructive solid geometry for [RedBolt](https://github.com/RedBoltFramework/RedBolt). The repository owns the managed CSG module and its desktop, WebAssembly, and Android showcase.

The source build expects RedBolt in `../RedBolt` by default. Override `RedBoltRoot` when the repositories use a different layout.

```sh
dotnet build RedBolt.CSG.slnx --configuration Debug
```

Desktop, WebAssembly, and Android samples are included under `src/RedBolt/Sample`.

Tests remain in the sibling `RedBolt.Tests` repository. See [building](doc/BUILDING.md), [package production](packaging/nuget/README.md), and the [CSG-specific deviations](doc/ALLOWED_DEVIATIONS.md).
