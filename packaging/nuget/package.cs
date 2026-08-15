#:project ../../../RedBolt/src/RedBolt/Build/Packaging/RedBolt.ExternalModule.Packaging.csproj

using System.Runtime.CompilerServices;
using RedBolt.ExternalModule.Packaging;

return ExternalModulePackager.Run(args, new ExternalModulePackageManifest
{
    Name = "RedBolt.CSG",
    BuildSteps =
    [
        new("CSG/RedBolt.CSG.csproj", Arguments: "-p:BuildProjectReferences=false")
    ],
    ManagedPackages =
    [
        new(
            "RedBolt.CSG",
            "Constructive solid geometry components for RedBolt.",
            [new("build/bin/Release/linux-x64/core/RedBolt.CSG.dll", "lib/net10.0/RedBolt.CSG.dll")],
            [new("RedBolt.Core", PackageVersions.RedBolt)])
    ],
    Consumer = new(
        "RedBolt.CSG",
        """
        using System;
        using RedBolt.CSG;
        Console.WriteLine(typeof(CsgRoot).FullName);
        """)
}, SourcePath());

static string SourcePath([CallerFilePath] string path = "") => path;
