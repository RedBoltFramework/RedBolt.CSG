using System.Diagnostics;
using System.IO.Compression;
using System.Runtime.CompilerServices;
using System.Text.RegularExpressions;
using System.Xml.Linq;

try
{
    return new Packager(args).Run();
}
catch (Exception exception)
{
    Console.Error.WriteLine(exception.Message);
    return 1;
}

internal sealed class Packager
{
    private const string Configuration = "Release";
    private readonly string[] _arguments;
    private readonly string _scriptDirectory;
    private readonly string _repositoryRoot;
    private readonly string _redBoltRoot;
    private readonly string _packageProject;
    private readonly string _packageRoot;
    private readonly string _stageRoot;
    private readonly string _outputRoot;
    private readonly string _baseOutputRoot;
    private string _packageVersion = "";
    private string _dependencyVersion = "";
    private string _redBoltVersion = "";
    private string _repositoryCommit = "";
    private bool _skipBuild;

    public Packager(string[] arguments)
    {
        _arguments = arguments;
        _scriptDirectory = Path.GetDirectoryName(GetSourcePath())!;
        _repositoryRoot = Path.GetFullPath(Path.Combine(_scriptDirectory, "..", ".."));
        _redBoltRoot = Path.GetFullPath(Path.Combine(_repositoryRoot, "..", "RedBolt"));
        _packageProject = Path.Combine(_scriptDirectory, "RedBolt.Package.csproj");
        _packageRoot = Path.Combine(_repositoryRoot, "build", "package", "nuget");
        _stageRoot = Path.Combine(_packageRoot, "stage");
        _outputRoot = Path.Combine(_packageRoot, "out");
        _baseOutputRoot = Path.Combine(_redBoltRoot, "build", "package", "nuget", "out");
    }

    public int Run()
    {
        if (_arguments.Length == 0 || _arguments[0] is not ("create" or "validate" or "version"))
            return Usage();

        var command = _arguments[0];
        ParseArguments();
        _packageVersion = GitVersion();
        _dependencyVersion = NormalizeDependencyVersion(_packageVersion);
        if (command == "version")
        {
            Console.WriteLine(_packageVersion);
            return 0;
        }

        _repositoryCommit = GitOrEmpty("rev-parse", "HEAD").Trim();
        _redBoltVersion = ResolveRedBoltVersion();
        VerifyBasePackages();

        if (command == "validate")
        {
            ValidatePackages();
            return 0;
        }

        if (!_skipBuild)
            BuildSelectedOutputs();

        RecreateDirectory(_stageRoot);
        RecreateDirectory(_outputRoot);
        Directory.CreateDirectory(_packageRoot);
        File.WriteAllText(Path.Combine(_packageRoot, "version.txt"), _packageVersion + Environment.NewLine);
        File.WriteAllText(Path.Combine(_packageRoot, "redbolt-version.txt"), _redBoltVersion + Environment.NewLine);

        StageManagedPackages();
        ValidatePackages();
        return 0;
    }

    private void ParseArguments()
    {
        for (var index = 1; index < _arguments.Length; ++index)
        {
            switch (_arguments[index])
            {
                case "--no-build":
                    _skipBuild = true;
                    break;
                case "--redbolt-version" when index + 1 < _arguments.Length:
                    _redBoltVersion = _arguments[++index];
                    break;
                default:
                    throw new InvalidOperationException("Unknown or incomplete package option: " + _arguments[index]);
            }
        }

    }

    private string ResolveRedBoltVersion()
    {
        if (!string.IsNullOrWhiteSpace(_redBoltVersion))
            return NormalizeDependencyVersion(_redBoltVersion);

        var versionFile = Path.Combine(_redBoltRoot, "build", "package", "nuget", "version.txt");
        var version = File.Exists(versionFile)
            ? File.ReadAllText(versionFile).Trim()
            : RunCapture("dotnet", ["run", "--file", Path.Combine(_redBoltRoot, "packaging", "nuget", "package.cs"), "--", "version"]).Trim();
        return NormalizeDependencyVersion(version);
    }

    private void VerifyBasePackages()
    {
        if (!Directory.Exists(_baseOutputRoot))
            throw new InvalidOperationException($"Base RedBolt package feed is missing: {_baseOutputRoot}");

        RequirePackage(_baseOutputRoot, "RedBolt.Core", _redBoltVersion);
    }

    private void BuildSelectedOutputs()
    {
        var baseCore = Path.Combine(_redBoltRoot, "build", "bin", Configuration, "linux-x64", "core", "RedBolt.Core.dll");
        if (!File.Exists(baseCore))
            throw new InvalidOperationException($"Build the matching base RedBolt Release outputs first: {baseCore}");

        RunCommand("dotnet", ["build", Project("CSG", "RedBolt.CSG.csproj"), "--configuration", Configuration, "-m:4", "-p:BuildProjectReferences=false"]);
    }

    private void StageManagedPackages()
    {
        var managedRoot = Path.Combine(_repositoryRoot, "build", "bin", Configuration, "linux-x64");
        var csgStage = PrepareStage("RedBolt.CSG");
        CopyRequired(Path.Combine(managedRoot, "core", "RedBolt.CSG.dll"), Path.Combine(csgStage, "lib", "net10.0", "RedBolt.CSG.dll"));
        Pack("RedBolt.CSG", "Constructive solid geometry components for RedBolt.", csgStage,
        [
            ("RedBolt.Core", _redBoltVersion)
        ]);

    }

    private string PrepareStage(string packageId)
    {
        var stage = Path.Combine(_stageRoot, packageId);
        RecreateDirectory(stage);
        File.Copy(Path.Combine(_scriptDirectory, "README.md"), Path.Combine(stage, "README.md"), true);
        return stage;
    }

    private void Pack(string packageId, string description, string stage, IReadOnlyList<(string Id, string Version)> dependencies)
    {
        var spec = Path.Combine(stage, "package.props");
        var project = new XElement("Project");
        if (dependencies.Count != 0)
        {
            project.Add(new XElement("ItemGroup",
                dependencies.Select(dependency => new XElement("PackageReference",
                    new XAttribute("Include", dependency.Id),
                    new XAttribute("Version", $"[{dependency.Version}]")))));
        }
        new XDocument(project).Save(spec);

        var properties = new[]
        {
            $"-p:RedBoltPackageId={packageId}",
            $"-p:RedBoltPackageVersion={_packageVersion}",
            $"-p:RedBoltPackageDescription={description}",
            $"-p:RedBoltRepositoryCommit={_repositoryCommit}",
            $"-p:RedBoltPackageStage={stage}",
            $"-p:RedBoltPackageSpec={spec}",
            $"-p:RestoreSources={_outputRoot};{_baseOutputRoot};https://api.nuget.org/v3/index.json"
        };
        RunCommand("dotnet", ["restore", _packageProject, .. properties]);
        RunCommand("dotnet",
        [
            "pack", _packageProject, "--configuration", Configuration, "--no-build", "--no-restore",
            "--output", _outputRoot,
            .. properties
        ]);
    }

    private void ValidatePackages()
    {
        var expected = new List<string> { "RedBolt.CSG" };
        foreach (var id in expected)
            RequirePackage(_outputRoot, id, _dependencyVersion);

        foreach (var id in expected)
        {
            using var archive = ZipFile.OpenRead(RequirePackage(_outputRoot, id, _dependencyVersion));
            var nuspec = ReadNuspec(archive);
            foreach (var dependency in nuspec.Descendants().Where(element => element.Name.LocalName == "dependency"))
            {
                var dependencyId = dependency.Attribute("id")?.Value ?? "";
                if (!dependencyId.StartsWith("RedBolt", StringComparison.Ordinal)
                    && !dependencyId.StartsWith("SharpCpp", StringComparison.Ordinal))
                    continue;
                var expectedVersion = dependencyId.StartsWith("RedBolt.CSG", StringComparison.Ordinal)
                    ? _dependencyVersion
                    : _redBoltVersion;
                if (dependency.Attribute("version")?.Value != $"[{expectedVersion}]")
                    throw new InvalidOperationException($"{id} does not pin {dependencyId} to [{expectedVersion}].");
            }
        }

        ValidateConsumer();
        Console.WriteLine($"Validated {expected.Count} RedBolt.CSG packages at {_packageVersion} against RedBolt {_redBoltVersion}.");
    }

    private void ValidateConsumer()
    {
        var root = Path.Combine(_packageRoot, "consumer");
        RecreateDirectory(root);
        File.WriteAllText(Path.Combine(root, "Consumer.csproj"), $"""
            <Project Sdk="Microsoft.NET.Sdk">
              <PropertyGroup>
                <OutputType>Exe</OutputType>
                <TargetFramework>net10.0</TargetFramework>
                <RuntimeIdentifier>linux-x64</RuntimeIdentifier>
              </PropertyGroup>
              <ItemGroup>
                <PackageReference Include="RedBolt.CSG" Version="[{_dependencyVersion}]" />
                <PackageReference Include="RedBolt.Core.Runtime.linux-x64" Version="[{_redBoltVersion}]" />
              </ItemGroup>
            </Project>
            """);
        File.WriteAllText(Path.Combine(root, "Program.cs"), """
            using System;
            using RedBolt.CSG;
            Console.WriteLine(typeof(CsgRoot).FullName);
            """);
        RunCommand("dotnet",
        [
            "build", Path.Combine(root, "Consumer.csproj"),
            "--configuration", Configuration,
            $"-p:RestoreSources={_outputRoot};{_baseOutputRoot};https://api.nuget.org/v3/index.json"
        ]);
    }

    private string RequirePackage(string feed, string id, string version)
    {
        if (!Directory.Exists(feed))
            throw new InvalidOperationException($"Package feed is missing: {feed}");
        foreach (var path in Directory.GetFiles(feed, "*.nupkg", SearchOption.TopDirectoryOnly))
        {
            using var archive = ZipFile.OpenRead(path);
            var nuspec = ReadNuspec(archive);
            var candidateId = nuspec.Descendants().First(element => element.Name.LocalName == "id").Value;
            var candidateVersion = NormalizeDependencyVersion(
                nuspec.Descendants().First(element => element.Name.LocalName == "version").Value);
            if (candidateId == id && candidateVersion == version)
                return path;
        }
        throw new InvalidOperationException($"Required package {id} [{version}] is missing from {feed}.");
    }

    private static XDocument ReadNuspec(ZipArchive archive)
    {
        var entry = archive.Entries.Single(candidate => candidate.FullName.EndsWith(".nuspec", StringComparison.OrdinalIgnoreCase));
        using var stream = entry.Open();
        return XDocument.Load(stream);
    }

    private string Project(params string[] parts) =>
        Path.Combine([_repositoryRoot, "src", "RedBolt", .. parts]);

    private static void CopyRequired(string source, string destination)
    {
        if (!File.Exists(source))
            throw new InvalidOperationException("Required package input is missing: " + source);
        Directory.CreateDirectory(Path.GetDirectoryName(destination)!);
        File.Copy(source, destination, true);
    }

    private static void RecreateDirectory(string path)
    {
        if (Directory.Exists(path))
            Directory.Delete(path, true);
        Directory.CreateDirectory(path);
    }

    private string GitVersion()
    {
        var commit = GitOrEmpty("rev-parse", "--short=8", "HEAD").Trim();
        if (commit.Length == 0)
            return "0.1.0-dev.0+working";
        var exactTag = GitOrEmpty("describe", "--tags", "--exact-match", "--match", "v[0-9]*").Trim();
        if (Regex.IsMatch(exactTag, "^v[0-9]+\\.[0-9]+\\.[0-9]+$"))
            return exactTag[1..];
        var heightText = GitOrEmpty("rev-list", "--count", "HEAD").Trim();
        var height = int.TryParse(heightText, out var parsedHeight) ? parsedHeight : 0;
        var dirty = GitOrEmpty("status", "--porcelain").Length == 0 ? "" : ".dirty";
        return $"0.1.0-dev.{height}{dirty}+{commit}";
    }

    private string GitOrEmpty(params string[] arguments)
    {
        try { return RunCapture("git", ["-C", _repositoryRoot, .. arguments]); }
        catch { return ""; }
    }

    private static string NormalizeDependencyVersion(string version)
    {
        var normalized = version.Trim();
        var metadata = normalized.IndexOf('+');
        return metadata < 0 ? normalized : normalized[..metadata];
    }

    private static string RunCapture(string fileName, IReadOnlyList<string> arguments)
    {
        var startInfo = CreateStartInfo(fileName, arguments);
        startInfo.RedirectStandardOutput = true;
        startInfo.RedirectStandardError = true;
        using var process = Process.Start(startInfo)!;
        var output = process.StandardOutput.ReadToEnd();
        var error = process.StandardError.ReadToEnd();
        process.WaitForExit();
        if (process.ExitCode != 0)
            throw new InvalidOperationException($"{fileName} failed: {error.Trim()}");
        return output;
    }

    private static void RunCommand(string fileName, IReadOnlyList<string> arguments)
    {
        using var process = Process.Start(CreateStartInfo(fileName, arguments))!;
        process.WaitForExit();
        if (process.ExitCode != 0)
            throw new InvalidOperationException($"{fileName} exited with code {process.ExitCode}.");
    }

    private static ProcessStartInfo CreateStartInfo(string fileName, IReadOnlyList<string> arguments)
    {
        var startInfo = new ProcessStartInfo(fileName) { UseShellExecute = false };
        foreach (var argument in arguments)
            startInfo.ArgumentList.Add(argument);
        return startInfo;
    }

    private static string GetSourcePath([CallerFilePath] string sourcePath = "") => sourcePath;

    private static int Usage()
    {
        Console.Error.WriteLine("Usage: dotnet run --file packaging/nuget/package.cs -- <create|validate|version> [--no-build] [--redbolt-version VERSION]");
        return 2;
    }
}
