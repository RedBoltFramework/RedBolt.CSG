#if ORIGINAL
using CSG = BaselineCSG;
#else
using CSG = RedBolt.CSG;
#endif
using System.Diagnostics;
using System.Runtime.ExceptionServices;
using System.Runtime.Loader;
using RedBolt.Graphics;
using RedBolt.IO;
using RedBolt.Math;

record WorkerCase(string Name, List<CSG.CsgPolygon> A, List<CSG.CsgPolygon> B,
    CSG.CsgOperation Operation, IReadOnlyDictionary<uint, List<VertexElement>> SourceLayout);

static class WorkerProgram
{
    private static int _sink;

    static void Main(string[] args)
    {
        RegisterAssemblyProbe();
        var shape = Option(args, "shape") ?? "sphere";
        var samples = int.Parse(Option(args, "samples") ?? "15");
        var warmups = int.Parse(Option(args, "warmups") ?? "4");
        var caseFilter = Option(args, "case");
#if !ORIGINAL
        if (Option(args, "bvh-build") is not null)
        {
            MeasureBvhBuild(samples, warmups);
            return;
        }
        if (Option(args, "validate") is { } validateCount)
        {
            Validate(int.Parse(validateCount), int.Parse(Option(args, "seed") ?? "1"),
                Option(args, "only") is { } only ? int.Parse(only) : -1);
            return;
        }
#endif
        if (samples < 1 || warmups < 0)
            throw new ArgumentOutOfRangeException(nameof(args));
        var cases = Cases(shape).Where(c => caseFilter is null || c.Name == caseFilter).ToArray();
        if (cases.Length == 0)
            throw new ArgumentException("No matching benchmark case.");

        Exception? failure = null;
        var worker = new Thread(() =>
        {
            try { Measure(cases, shape, samples, warmups); }
            catch (Exception error) { failure = error; }
        }) { IsBackground = true, Name = "CSG benchmark worker" };
        worker.Start();
        worker.Join();
        if (failure is not null) ExceptionDispatchInfo.Capture(failure).Throw();
        GC.KeepAlive(_sink);
    }

    static string? Option(string[] args, string name)
    {
        var prefix = $"--{name}=";
        var match = args.FirstOrDefault(arg => arg.StartsWith(prefix, StringComparison.Ordinal));
        return match is null ? null : match[prefix.Length..];
    }

    static void RegisterAssemblyProbe()
    {
        var paths = (Environment.GetEnvironmentVariable("REDBOLT_BENCHMARK_ASSEMBLY_PATHS") ?? "")
            .Split(Path.PathSeparator, StringSplitOptions.RemoveEmptyEntries);
        AssemblyLoadContext.Default.Resolving += (context, assemblyName) =>
        {
            foreach (var path in paths)
            {
                var file = Path.Combine(path, assemblyName.Name + ".dll");
                if (System.IO.File.Exists(file)) return context.LoadFromAssemblyPath(file);
            }
            return null;
        };
    }

    static void Measure(WorkerCase[] cases, string shape, int samples, int warmups)
    {
        var variant = Environment.GetEnvironmentVariable("CSG_VARIANT") ?? "unknown";
        Console.WriteLine($"variant={variant};shape={shape};dotnet={Environment.Version};cpu={Environment.ProcessorCount};worker=true;tieredCompilation={Environment.GetEnvironmentVariable("DOTNET_TieredCompilation")};samples={samples}");
        foreach (var scenario in cases)
        {
            var result = BuildResult(scenario, null);
            Console.WriteLine($"case={scenario.Name};inputA={scenario.A.Count};inputB={scenario.B.Count};faces={result.Count};volume={SignedVolume(result):F6}");
        }
        foreach (var scenario in cases)
        foreach (var phase in new[] { "full", "append" })
        {
            object? cachedA = null;
#if !ORIGINAL
            if (phase == "append") cachedA = new CSG.CsgSolid(scenario.A, CSG.CsgCommon.DefaultEpsilon, false);
#endif
            Func<int> run = () => RunWorkerJob(scenario, cachedA);
            for (var i = 0; i < warmups; ++i) _sink ^= run();
            var times = new List<double>(samples);
            var allocations = new List<long>(samples);
            var checksum = 0;
            for (var i = 0; i < samples; ++i)
            {
                GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
                var beforeBytes = GC.GetAllocatedBytesForCurrentThread();
                var start = Stopwatch.GetTimestamp();
                checksum = run();
                times.Add(Stopwatch.GetElapsedTime(start).TotalMilliseconds);
                allocations.Add(GC.GetAllocatedBytesForCurrentThread() - beforeBytes);
                _sink ^= checksum;
            }
            times.Sort(); allocations.Sort();
            var middle = samples / 2;
            Console.WriteLine($"{scenario.Name},{phase},{times[middle]:F4},{times[samples / 10]:F4},{times[samples * 9 / 10]:F4},{allocations[middle]},{checksum}");
        }
    }

    static int RunWorkerJob(WorkerCase scenario, object? cachedA)
    {
        var polygons = BuildResult(scenario, cachedA);
        if (polygons.Count == 0) return 0;
#if ORIGINAL
        var triangulated = CSG.Csg.TriangulatePolygons((List<CSG.CsgPolygon>)polygons);
#else
        var triangulated = CSG.Csg.TriangulatePolygons(polygons);
#endif
        var prepared = CSG.Csg.PrepareModel(triangulated, scenario.SourceLayout);
        return polygons.Count + prepared.Geometries.Sum(geometry =>
            checked((int)geometry.IndexCount + geometry.VertexData.Length + geometry.IndexData.Length));
    }

    static IReadOnlyList<CSG.CsgPolygon> BuildResult(WorkerCase scenario, object? cachedA)
    {
#if ORIGINAL
        // CsgRoot clones both source polygon lists on every rebuild, including an append.
        var a = scenario.A.Select(polygon => polygon.Clone()).ToList();
        var b = scenario.B.Select(polygon => polygon.Clone()).ToList();
        var bspA = new CSG.CsgBsp(); bspA.Build(a);
        var bspB = new CSG.CsgBsp(); bspB.Build(b);
        return CSG.Csg.BooleanOperation(bspA, bspB, scenario.Operation);
#else
        // CsgRoot reuses the previous result solid only when a flat tree gains a brush. Solids adopt the
        // job's freshly extracted polygons without copying and the result faces are triangulated directly.
        var a = cachedA as CSG.CsgSolid ?? new CSG.CsgSolid(scenario.A, CSG.CsgCommon.DefaultEpsilon, false);
        var b = new CSG.CsgSolid(scenario.B, CSG.CsgCommon.DefaultEpsilon, false);
        return CSG.Csg.BooleanOperation(a, b, scenario.Operation).Faces;
#endif
    }

#if !ORIGINAL
    /// <summary>Time face-tree construction alone for sphere operands of increasing size.</summary>
    static void MeasureBvhBuild(int samples, int warmups)
    {
        var sphere = Cases("sphere").First(c => c.Name == "disjoint1_union").A;
        Console.WriteLine($"samples={samples}");
        foreach (var copies in new[] { 1, 16, 64 })
        {
            var polygons = new List<CSG.CsgPolygon>(sphere.Count * copies);
            for (var i = 0; i < copies; ++i)
            {
                foreach (var polygon in sphere)
                {
                    var copy = polygon.Clone();
                    for (var v = 0; v < copy.Vertices.Count; ++v)
                    {
                        var vertex = copy.Vertices[v];
                        vertex.SetPosition(vertex.GetPosition() + new Vector3f(3.0f * (i % 8), 3.0f * (i / 8), 0.0f));
                        copy.Vertices[v] = vertex;
                    }
                    polygons.Add(copy);
                }
            }
            var solid = new CSG.CsgSolid(polygons, CSG.CsgCommon.DefaultEpsilon, false);
            for (var i = 0; i < warmups; ++i) _sink ^= solid.BuildFaceTreeForBenchmark();
            var times = new List<double>(samples);
            for (var i = 0; i < samples; ++i)
            {
                var start = Stopwatch.GetTimestamp();
                _sink ^= solid.BuildFaceTreeForBenchmark();
                times.Add(Stopwatch.GetElapsedTime(start).TotalMilliseconds);
            }
            times.Sort();
            Console.WriteLine($"faces={solid.FaceCount},build,{times[samples / 2]:F4},{times[samples / 10]:F4},{times[samples * 9 / 10]:F4}");
        }
    }

    /// <summary>
    /// Compare the solid boolean against the BSP reference on random overlapping closed primitives, including
    /// chained operations where a boolean result becomes the next operand. Every result must be closed.
    /// </summary>
    static void Validate(int count, int seed, int only)
    {
        var modelDirectory = Path.GetDirectoryName(Environment.GetEnvironmentVariable("REDBOLT_BENCHMARK_SPHERE_MODEL")
            ?? throw new InvalidOperationException("Set REDBOLT_BENCHMARK_SPHERE_MODEL."))!;
        // Operands must be closed meshes, so open models such as TeaPot are not part of the set.
        var names = new[] { "Box", "Sphere", "Cylinder", "Cone", "Torus", "Pyramid" };
        var models = names.Select(name =>
        {
            using var source = new MemoryBuffer(System.IO.File.ReadAllBytes(Path.Combine(modelDirectory, name + ".mdl")));
            var model = new Model();
            if (!model.Load(source)) throw new InvalidOperationException(name);
            return model;
        }).ToArray();
        var random = new Random(seed);
        var operations = new[] { CSG.CsgOperation.Union, CSG.CsgOperation.DifferenceAB, CSG.CsgOperation.Intersection };
        var failures = 0;
        var worstRelative = 0.0;
        var fastTicks = 0L;
        var bspTicks = 0L;
        for (var i = 0; i < count; ++i)
        {
            List<CSG.CsgPolygon> Operand(out string label)
            {
                var index = random.Next(names.Length);
                var transform = new Matrix3x4(
                    new Vector3f(Next(-0.4f, 0.4f), Next(-0.4f, 0.4f), Next(-0.4f, 0.4f)),
                    new Quaternion(Next(0, 360), Next(0, 360), Next(0, 360)),
                    new Vector3f(Next(0.5f, 1.5f), Next(0.5f, 1.5f), Next(0.5f, 1.5f)));
                label = names[index];
                return CSG.Csg.BuildPolygonsFromModel(models[index], transform);
            }
            var first = Operand(out var firstLabel);
            var second = Operand(out var secondLabel);
            var third = Operand(out var thirdLabel);
            var op1 = operations[random.Next(3)];
            var op2 = operations[random.Next(3)];
            var label = $"#{i} ({firstLabel} {op1} {secondLabel}) {op2} {thirdLabel}";
            var caseSeed = random.Next();
            if (only >= 0 && i != only)
                continue;
            var sampler = new Random(caseSeed);

            var start = Stopwatch.GetTimestamp();
            var caseStart = start;
            var fast = CSG.Csg.BooleanOperation(
                CSG.Csg.BooleanOperation(new CSG.CsgSolid(first), new CSG.CsgSolid(second), op1),
                new CSG.CsgSolid(third), op2);
            var fastTicksCase = Stopwatch.GetTimestamp() - caseStart;
            fastTicks += fastTicksCase;

            start = Stopwatch.GetTimestamp();
            var reference = Bsp(Bsp(first, second, op1), third, op2);
            bspTicks += Stopwatch.GetTimestamp() - start;

            var caseMs = Stopwatch.GetElapsedTime(0, fastTicksCase).TotalMilliseconds;
            var bspStart = Stopwatch.GetTimestamp();
            var fastError = MembershipError(fast.Faces, first, second, third, op1, op2, sampler, out var bspError, reference);
            if (only >= 0)
            {
                var intermediate = CSG.Csg.BooleanOperation(new CSG.CsgSolid(first), new CSG.CsgSolid(second), op1);
                var intermediateError = MembershipError(intermediate.Faces, first, second, [], op1, CSG.CsgOperation.Union,
                    new Random(caseSeed), out var intermediateBspError, Bsp(first, second, op1));
                Console.WriteLine($"{label}: intermediate closed={intermediate.IsClosed} faces={intermediate.FaceCount} error={intermediateError:P2} bspError={intermediateBspError:P2}; final closed={fast.IsClosed} error={fastError:P2} bspError={bspError:P2}");
            }
            worstRelative = System.Math.Max(worstRelative, fastError);
            if (fastError > System.Math.Max(2.0 * bspError, 0.005))
            {
                ++failures;
                Console.WriteLine($"WORSE {label}: fastError={fastError:P2} bspError={bspError:P2} closed={fast.IsClosed}");
            }
            var bspCaseMs = Stopwatch.GetElapsedTime(caseStart + fastTicksCase, bspStart).TotalMilliseconds;
            if (!fast.IsClosed || fast.HasOpenEdges)
            {
                ++failures;
                Console.WriteLine($"OPEN {label}: result is not watertight (closed={fast.IsClosed})");
            }
            if (caseMs > 2000 || caseMs > 3 * bspCaseMs + 50)
                Console.WriteLine($"SLOW {label}: {caseMs:F0} ms, bsp {bspCaseMs:F0} ms");
        }
        Console.WriteLine($"validated={count};seed={seed};failures={failures};worstMembershipError={worstRelative:P2};fastMs={Stopwatch.GetElapsedTime(0, fastTicks).TotalMilliseconds:F1};bspMs={Stopwatch.GetElapsedTime(0, bspTicks).TotalMilliseconds:F1}");

        float Next(float min, float max) => min + (float)random.NextDouble() * (max - min);

        // Fraction of sample points in the joint bounds whose membership differs from the exact boolean of
        // the inputs. Membership uses generalized winding numbers, which tolerate open and cracked meshes.
        static double MembershipError(IReadOnlyList<CSG.CsgPolygon> result, List<CSG.CsgPolygon> a,
            List<CSG.CsgPolygon> b, List<CSG.CsgPolygon> c, CSG.CsgOperation op1, CSG.CsgOperation op2,
            Random random, out double referenceError, List<CSG.CsgPolygon> reference)
        {
            const int samples = 3000;
            var (min, max) = Bounds(a.Concat(b).Concat(c));
            var ta = Triangles(a); var tb = Triangles(b); var tc = Triangles(c);
            var tr = Triangles(result); var tref = Triangles(reference);
            var wrong = 0; var referenceWrong = 0;
            for (var i = 0; i < samples; ++i)
            {
                var p = new System.Numerics.Vector3(
                    min.X + (float)random.NextDouble() * (max.X - min.X),
                    min.Y + (float)random.NextDouble() * (max.Y - min.Y),
                    min.Z + (float)random.NextDouble() * (max.Z - min.Z));
                var expected = Apply(Apply(Winding(ta, p) > 0.5, Winding(tb, p) > 0.5, op1), Winding(tc, p) > 0.5, op2);
                if ((Winding(tr, p) > 0.5) != expected) ++wrong;
                if ((Winding(tref, p) > 0.5) != expected) ++referenceWrong;
            }
            referenceError = (double)referenceWrong / samples;
            return (double)wrong / samples;
        }

        static bool Apply(bool a, bool b, CSG.CsgOperation operation) => operation switch
        {
            CSG.CsgOperation.Union => a || b,
            CSG.CsgOperation.Intersection => a && b,
            _ => a && !b,
        };

        static (System.Numerics.Vector3 Min, System.Numerics.Vector3 Max) Bounds(IEnumerable<CSG.CsgPolygon> polygons)
        {
            var min = new System.Numerics.Vector3(float.MaxValue);
            var max = new System.Numerics.Vector3(float.MinValue);
            foreach (var polygon in polygons)
            foreach (var vertex in polygon.Vertices)
            {
                var p = new System.Numerics.Vector3(vertex.Position.X, vertex.Position.Y, vertex.Position.Z);
                min = System.Numerics.Vector3.Min(min, p);
                max = System.Numerics.Vector3.Max(max, p);
            }
            return (min, max);
        }

        static System.Numerics.Vector3[] Triangles(IReadOnlyList<CSG.CsgPolygon> polygons)
        {
            var result = new List<System.Numerics.Vector3>();
            foreach (var polygon in polygons)
            {
                var v = polygon.Vertices;
                for (var i = 1; i + 1 < v.Count; ++i)
                {
                    result.Add(new(v[0].Position.X, v[0].Position.Y, v[0].Position.Z));
                    result.Add(new(v[i].Position.X, v[i].Position.Y, v[i].Position.Z));
                    result.Add(new(v[i + 1].Position.X, v[i + 1].Position.Y, v[i + 1].Position.Z));
                }
            }
            return result.ToArray();
        }

        static double Winding(System.Numerics.Vector3[] triangles, System.Numerics.Vector3 p)
        {
            double total = 0;
            for (var t = 0; t + 2 < triangles.Length; t += 3)
            {
                var a = triangles[t] - p; var b = triangles[t + 1] - p; var c = triangles[t + 2] - p;
                double la = a.Length(), lb = b.Length(), lc = c.Length();
                var numerator = System.Numerics.Vector3.Dot(a, System.Numerics.Vector3.Cross(b, c));
                var denominator = la * lb * lc + System.Numerics.Vector3.Dot(a, b) * lc
                    + System.Numerics.Vector3.Dot(b, c) * la + System.Numerics.Vector3.Dot(c, a) * lb;
                total += 2.0 * System.Math.Atan2(numerator, denominator);
            }
            return total / (4.0 * System.Math.PI);
        }

        static List<CSG.CsgPolygon> Bsp(List<CSG.CsgPolygon> a, List<CSG.CsgPolygon> b, CSG.CsgOperation operation)
        {
            var bspA = new CSG.CsgBsp(); bspA.Build(a.Select(p => p.Clone()).ToList());
            var bspB = new CSG.CsgBsp(); bspB.Build(b.Select(p => p.Clone()).ToList());
            return CSG.Csg.BooleanOperation(bspA, bspB, operation);
        }
    }
#endif

    static double SignedVolume(IReadOnlyList<CSG.CsgPolygon> polygons)
    {
        double sixVolumes = 0;
        foreach (var polygon in polygons)
        {
            if (polygon.Vertices.Count < 3) continue;
            var first = polygon.Vertices[0].GetPosition();
            for (var i = 1; i + 1 < polygon.Vertices.Count; ++i)
            {
                var second = polygon.Vertices[i].GetPosition();
                var third = polygon.Vertices[i + 1].GetPosition();
                sixVolumes += first.DotProduct(second.CrossProduct(third));
            }
        }
        return sixVolumes / 6;
    }

    static IEnumerable<WorkerCase> Cases(string shape)
    {
        if (shape == "box")
        {
            var layout = new Dictionary<uint, List<VertexElement>>();
            yield return new("disjoint1_union", Boxes(1, 0, 0, 0, 3), Boxes(1, 10, 0, 0, 3), CSG.CsgOperation.Union, layout);
            yield return new("disjoint16_union", Boxes(16, 0, 0, 0, 3), Boxes(16, 100, 0, 0, 3), CSG.CsgOperation.Union, layout);
            yield return new("contained_difference", Boxes(1, 0, 0, 0, 3), Boxes(1, .2f, .2f, .2f, 3, .6f), CSG.CsgOperation.DifferenceAB, layout);
            yield return new("overlap_difference", Boxes(1, 0, 0, 0, 3), Boxes(1, .35f, .35f, .35f, 3), CSG.CsgOperation.DifferenceAB, layout);
            yield return new("local_cut16_difference", Boxes(16, 0, 0, 0, 3), Boxes(1, .35f, .35f, .35f, 3), CSG.CsgOperation.DifferenceAB, layout);
            yield break;
        }
        if (shape != "sphere") throw new ArgumentException($"Unknown shape: {shape}");
        var modelPath = Environment.GetEnvironmentVariable("REDBOLT_BENCHMARK_SPHERE_MODEL")
            ?? throw new InvalidOperationException("Set REDBOLT_BENCHMARK_SPHERE_MODEL to CoreData/Models/Sphere.mdl.");
        using var source = new MemoryBuffer(System.IO.File.ReadAllBytes(modelPath));
        using var model = new Model();
        if (!model.Load(source)) throw new InvalidOperationException($"Failed to load {modelPath}.");
        var layouts = CSG.Csg.GetSourceVertexElements(new Model?[] { model });
        var bounds = model.GetBoundingBox();
        var extent = bounds.Max - bounds.Min;
        if (MathF.Abs(extent.X - extent.Y) > .001f || MathF.Abs(extent.X - extent.Z) > .001f)
            throw new InvalidOperationException("The built-in sphere model has unexpected bounds.");
        Console.WriteLine($"model={modelPath};bounds={bounds}");
        yield return new("disjoint1_union", Spheres(model, 1, 0, 0, 0, 3), Spheres(model, 1, 10, 0, 0, 3), CSG.CsgOperation.Union, layouts);
        yield return new("disjoint16_union", Spheres(model, 16, 0, 0, 0, 3), Spheres(model, 16, 100, 0, 0, 3), CSG.CsgOperation.Union, layouts);
        yield return new("contained_difference", Spheres(model, 1, 0, 0, 0, 3), Spheres(model, 1, .2f, .2f, .2f, 3, .6f), CSG.CsgOperation.DifferenceAB, layouts);
        yield return new("overlap_difference", Spheres(model, 1, 0, 0, 0, 3), Spheres(model, 1, .35f, .35f, .35f, 3), CSG.CsgOperation.DifferenceAB, layouts);
        yield return new("local_cut16_difference", Spheres(model, 16, 0, 0, 0, 3), Spheres(model, 1, .35f, .35f, .35f, 3), CSG.CsgOperation.DifferenceAB, layouts);
    }

    static List<CSG.CsgPolygon> Spheres(Model model, int count, float x, float y, float z,
        float spacing, float size = 1)
    {
        var polygons = new List<CSG.CsgPolygon>();
        var bounds = model.GetBoundingBox();
        var scale = size / (bounds.Max.X - bounds.Min.X);
        for (var i = 0; i < count; ++i)
        {
            var transform = Matrix3x4.FromScale(scale);
            var targetCenter = new Vector3f(x + i * spacing + size * .5f, y + size * .5f, z + size * .5f);
            transform.SetTranslation(targetCenter - bounds.Center * scale);
            polygons.AddRange(CSG.Csg.BuildPolygonsFromModel(model, transform));
        }
        return polygons;
    }

    static List<CSG.CsgPolygon> Boxes(int count, float x, float y, float z, float spacing, float size = 1)
    {
        var polygons = new List<CSG.CsgPolygon>(count * 6);
        for (var i = 0; i < count; ++i)
        {
            var px = x + i * spacing;
            var points = new[] {
                new Vector3f(px,y,z), new Vector3f(px+size,y,z), new Vector3f(px+size,y+size,z), new Vector3f(px,y+size,z),
                new Vector3f(px,y,z+size), new Vector3f(px+size,y,z+size), new Vector3f(px+size,y+size,z+size), new Vector3f(px,y+size,z+size)
            };
            int[][] faces = [[0,3,2,1], [4,5,6,7], [0,1,5,4], [3,7,6,2], [0,4,7,3], [1,2,6,5]];
            foreach (var face in faces)
            {
                var normal = (points[face[1]] - points[face[0]]).CrossProduct(points[face[2]] - points[face[0]]).Normalized;
                var vertices = new List<ModelVertex>(4);
                foreach (var index in face)
                {
                    var vertex = new ModelVertex(); vertex.SetPosition(points[index]); vertex.SetNormal(normal);
                    vertices.Add(vertex);
                }
                polygons.Add(new CSG.CsgPolygon(vertices, 0));
            }
        }
        return polygons;
    }
}
