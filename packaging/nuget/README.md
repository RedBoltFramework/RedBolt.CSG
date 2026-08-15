# RedBolt.CSG NuGet packages

RedBolt.CSG versions independently from RedBolt, but every package records exact dependencies on the base RedBolt package version used for the build. Produce the matching base package set first in `../RedBolt/build/package/nuget/out`.

Create the managed CSG package from Release outputs, building the add-on first:

```sh
dotnet run --file packaging/nuget/package.cs -- create
```

Use `--no-build` when the managed output already exists. Override base version discovery only for an intentional local feed with `--redbolt-version VERSION`.

The command writes independently versioned packages below `build/package/nuget/out`:

- `RedBolt.CSG`

Validate package contents, exact dependency versions, and a package-only consumer with:

```sh
dotnet run --file packaging/nuget/package.cs -- validate
```
