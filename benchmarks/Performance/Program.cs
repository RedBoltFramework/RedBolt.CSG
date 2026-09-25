#if ORIGINAL
using CSG = BaselineCSG;
#else
using CSG = RedBolt.CSG;
#endif
using System.Diagnostics;
using RedBolt.Graphics;
using RedBolt.Math;

record BenchCase(string Name, List<CSG.CsgPolygon> A, List<CSG.CsgPolygon> B, CSG.CsgOperation Operation);

static class Program
{
    private static int _sink;

    static void Main()
    {
        var variant = Environment.GetEnvironmentVariable("CSG_VARIANT") ?? "unknown";
        Console.WriteLine($"variant={variant};dotnet={Environment.Version};cpu={Environment.ProcessorCount}");
        var cases = Cases().ToArray();
        foreach (var scenario in cases)
        {
            var polygons = RawResult(scenario);
            Console.WriteLine($"shape={scenario.Name};faces={polygons.Count};volume={SignedVolume(polygons):F6}");
        }
        foreach (var scenario in cases)
        foreach (var phase in new[] { "cold", "warm" })
        {
            Func<int> run = MakeAction(scenario, phase);
            for (var i = 0; i < 4; i++) _sink ^= run();
            var times = new List<double>();
            var allocs = new List<long>();
            var count = -1;
            for (var i = 0; i < 15; i++)
            {
                GC.Collect(); GC.WaitForPendingFinalizers(); GC.Collect();
                var beforeAlloc = GC.GetAllocatedBytesForCurrentThread();
                var before = Stopwatch.GetTimestamp();
                count = run();
                var elapsed = Stopwatch.GetElapsedTime(before).TotalMilliseconds;
                var alloc = GC.GetAllocatedBytesForCurrentThread() - beforeAlloc;
                times.Add(elapsed);
                allocs.Add(alloc);
                _sink ^= count;
            }
            times.Sort(); allocs.Sort();
            Console.WriteLine($"{scenario.Name},{phase},{times[7]:F4},{times[1]:F4},{times[13]:F4},{allocs[7]},{count}");
        }
        GC.KeepAlive(_sink);
    }

    static List<CSG.CsgPolygon> RawResult(BenchCase scenario)
    {
#if ORIGINAL
        var a = scenario.A.Select(p => p.Clone()).ToList();
        var b = scenario.B.Select(p => p.Clone()).ToList();
        var bspA = new CSG.CsgBsp(); bspA.Build(a);
        var bspB = new CSG.CsgBsp(); bspB.Build(b);
        return CSG.Csg.BooleanOperation(bspA, bspB, scenario.Operation);
#else
        return CSG.Csg.BooleanOperation(new CSG.CsgSolid(scenario.A),
            new CSG.CsgSolid(scenario.B), scenario.Operation).ToPolygons();
#endif
    }

    static double SignedVolume(List<CSG.CsgPolygon> polygons)
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
        return sixVolumes / 6.0;
    }

    static Func<int> MakeAction(BenchCase scenario, string phase)
    {
#if ORIGINAL
        if (phase == "cold") return () =>
        {
            var a = scenario.A.Select(p => p.Clone()).ToList();
            var b = scenario.B.Select(p => p.Clone()).ToList();
            var bspA = new CSG.CsgBsp(); bspA.Build(a);
            var bspB = new CSG.CsgBsp(); bspB.Build(b);
            var polygons = CSG.Csg.BooleanOperation(bspA, bspB, scenario.Operation);
            var mesh = CSG.Csg.TriangulatePolygons(polygons);
            return polygons.Count + mesh.Geometries.Sum(g => g.IndexData.Count);
        };
        var warmA = new CSG.CsgBsp(); warmA.Build(scenario.A.Select(p => p.Clone()).ToList());
        var warmB = new CSG.CsgBsp(); warmB.Build(scenario.B.Select(p => p.Clone()).ToList());
        return () =>
        {
            var polygons = CSG.Csg.BooleanOperation(warmA, warmB, scenario.Operation);
            var mesh = CSG.Csg.TriangulatePolygons(polygons);
            return polygons.Count + mesh.Geometries.Sum(g => g.IndexData.Count);
        };
#else
        if (phase == "cold") return () =>
        {
            var a = new CSG.CsgSolid(scenario.A);
            var b = new CSG.CsgSolid(scenario.B);
            var result = CSG.Csg.BooleanOperation(a, b, scenario.Operation);
            var polygons = result.ToPolygons();
            var mesh = CSG.Csg.TriangulatePolygons(polygons);
            return polygons.Count + mesh.Geometries.Sum(g => g.IndexData.Count);
        };
        var warmA = new CSG.CsgSolid(scenario.A);
        var warmB = new CSG.CsgSolid(scenario.B);
        return () =>
        {
            var result = CSG.Csg.BooleanOperation(warmA, warmB, scenario.Operation);
            var polygons = result.ToPolygons();
            var mesh = CSG.Csg.TriangulatePolygons(polygons);
            return polygons.Count + mesh.Geometries.Sum(g => g.IndexData.Count);
        };
#endif
    }

    static IEnumerable<BenchCase> Cases()
    {
        yield return new("disjoint1_union", Boxes(1, 0, 0, 0, 3), Boxes(1, 10, 0, 0, 3), CSG.CsgOperation.Union);
        yield return new("disjoint16_union", Boxes(16, 0, 0, 0, 3), Boxes(16, 100, 0, 0, 3), CSG.CsgOperation.Union);
        yield return new("contained_difference", Boxes(1, 0, 0, 0, 3), Boxes(1, .2f, .2f, .2f, 3, .6f), CSG.CsgOperation.DifferenceAB);
        yield return new("overlap_difference", Boxes(1, 0, 0, 0, 3), Boxes(1, .35f, .35f, .35f, 3), CSG.CsgOperation.DifferenceAB);
        yield return new("local_cut16_difference", Boxes(16, 0, 0, 0, 3), Boxes(1, .35f, .35f, .35f, 3), CSG.CsgOperation.DifferenceAB);
    }

    static List<CSG.CsgPolygon> Boxes(int count, float x, float y, float z, float spacing, float scale = 1)
    {
        var polygons = new List<CSG.CsgPolygon>(count * 6);
        for (var i = 0; i < count; i++) Box(polygons, x + i * spacing, y, z, scale);
        return polygons;
    }

    static void Box(List<CSG.CsgPolygon> result, float x, float y, float z, float s)
    {
        var points = new[] {
            new Vector3f(x,y,z),new Vector3f(x+s,y,z),new Vector3f(x+s,y+s,z),new Vector3f(x,y+s,z),
            new Vector3f(x,y,z+s),new Vector3f(x+s,y,z+s),new Vector3f(x+s,y+s,z+s),new Vector3f(x,y+s,z+s)
        };
        int[][] faces = [
            [0,3,2,1], [4,5,6,7], [0,1,5,4], [3,7,6,2], [0,4,7,3], [1,2,6,5]
        ];
        foreach (var indices in faces)
        {
            var normal = (points[indices[1]] - points[indices[0]]).CrossProduct(points[indices[2]] - points[indices[0]]).Normalized;
            var verts = new List<ModelVertex>(4);
            foreach (var index in indices)
            {
                var v = new ModelVertex(); v.SetPosition(points[index]); v.SetNormal(normal); verts.Add(v);
            }
            result.Add(new CSG.CsgPolygon(verts, 0));
        }
    }
}
