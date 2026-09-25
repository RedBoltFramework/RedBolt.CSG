// Copyright (c) 2026 RedBolt and contributors.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

namespace RedBolt.CSG;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

using RedBolt.Graphics;
using RedBolt.Math;

using Vec3 = System.Numerics.Vector3;

/// <summary>
/// Reusable, polygonal CSG operand. Faces and their attributes are retained independently of render triangles.
/// A face BVH finds the intersection frontier; shared edges identify regions that can be classified together.
/// </summary>
/// <remarks>
/// Operands must be closed meshes without holes; boolean results are closed as well. Open input is not
/// supported, and <see cref="IsClosed"/> reports whether a solid satisfies that requirement.
/// Faces are immutable once owned by a solid. Internal copies share the per-vertex color and UV arrays
/// of <see cref="ModelVertex"/>, so CSG code must replace vertices rather than write into those arrays.
/// Geometry queries run on flat <see cref="Vec3"/> arrays so hot loops never touch the attribute structs.
/// </remarks>
public sealed class CsgSolid
{
    /// <summary>
    /// Length of unmatched edges that a solid may have and still count as closed: the larger of an absolute
    /// budget in epsilons and a fraction of the solid's total edge length. Plane intersections between nearly
    /// parallel faces leave hairline cracks; rejecting those would send every later boolean on the solid to the
    /// BSP fallback. Ray-parity classification is not affected by holes this small, while a misclassified
    /// region leaves a crack as long as its perimeter and is still rejected.
    /// </summary>
    private const float CrackBudgetEpsilons = 200.0f;
    private const float CrackBudgetFraction = 0.001f;

    /// <summary>
    /// Unmatched seam edges are collapsed while healing when shorter than CrackCollapseScale epsilons, or
    /// than CrackCollapseFraction of the unmatched edges beside them, but never beyond CrackCollapseMaxScale.
    /// </summary>
    private const float CrackCollapseScale = 64.0f;
    private const float CrackCollapseFraction = 0.05f;
    private const float CrackCollapseMaxScale = 256.0f;
    private const int MaxCollapseRounds = 4;

    /// <summary>Radius, in epsilons, within which intersection curve vertices from both operands are merged.</summary>
    private const float CurveSnapScale = 8.0f;

    /// <summary>A cut to apply to a frontier face: a segment on the face and the plane that divides it there.</summary>
    private readonly record struct IntersectionEdge(Vector3f Start, Vector3f End, Plane Cut);

    private readonly CsgPolygon[] _faces;
    // Face f owns triangles [_triangleStart[f], _triangleStart[f + 1]). Each triangle stores three positions
    // in _trianglePositions and the matching face-local vertex indices in _triangleVertices.
    private readonly int[] _triangleStart;
    private readonly Vec3[] _trianglePositions;
    private readonly int[] _triangleVertices;
    private readonly Vec3[] _faceMin;
    private readonly Vec3[] _faceMax;
    // Compressed rows: component c owns _componentFaces[_componentStart[c].._componentStart[c + 1]].
    private readonly int[] _componentStart;
    private readonly int[] _componentFaces;
    private readonly int[] _adjacencyStart;
    private readonly int[] _adjacentFaces;
    private readonly int[] _faceComponents;
    // Position of each face within its component row, so a component can be copied with local renumbering.
    private readonly int[] _faceRanks;
    // Directed edges that would close the surface: each cancels the winding surplus of an unmatched edge.
    // Null when every edge is matched.
    private List<(int From, int To, Vec3 FromPoint, Vec3 ToPoint, uint Shared)>? _openEdges;
    // Vertex ids of the open edges resolve through this welder; kept only while the solid has open edges.
    private PointWelder? _crackWelder;
    private readonly Vec3 _boundsMin;
    private readonly Vec3 _boundsMax;
    private readonly Vec3[] _componentMin;
    private readonly Vec3[] _componentMax;
    // Two-level hierarchy: a tree over component boxes and a face tree per component, both built on first
    // use. Queries only build the trees of components they reach.
    private FaceBvh? _componentTree;
    private readonly FaceBvh?[] _faceTrees;

    public int FaceCount => _faces.Length;
    public int ComponentCount => _componentStart.Length - 1;
    public bool IsClosed { get; }

    /// <summary>Whether any edge is unmatched, including hairline cracks tolerated by <see cref="IsClosed"/>.</summary>
    internal bool HasOpenEdges => _openEdges is not null;

    private FaceBvh ComponentTree => _componentTree ??= new FaceBvh(_componentMin, _componentMax);

    private FaceBvh FaceTree(int component) => _faceTrees[component] ??= new FaceBvh(_faceMin, _faceMax,
        _componentFaces.AsSpan(_componentStart[component], _componentStart[component + 1] - _componentStart[component]));

    private static void ComponentBounds(int[] componentStart, int[] componentFaces, Vec3[] faceMin, Vec3[] faceMax,
        out Vec3[] componentMin, out Vec3[] componentMax)
    {
        var count = componentStart.Length - 1;
        componentMin = new Vec3[count];
        componentMax = new Vec3[count];
        for (var component = 0; component < count; ++component)
        {
            var min = new Vec3(float.PositiveInfinity);
            var max = new Vec3(float.NegativeInfinity);
            for (var c = componentStart[component]; c < componentStart[component + 1]; ++c)
            {
                min = Vec3.MinNative(min, faceMin[componentFaces[c]]);
                max = Vec3.MaxNative(max, faceMax[componentFaces[c]]);
            }
            componentMin[component] = min;
            componentMax[component] = max;
        }
    }

    public CsgSolid(IReadOnlyList<CsgPolygon> polygons, float epsilon = CsgCommon.DefaultEpsilon)
        : this(polygons, epsilon, true)
    {
    }

    // Boolean results own newly split faces and may share untouched faces with immutable operands.
    internal CsgSolid(IReadOnlyList<CsgPolygon> polygons, float epsilon, bool copyFaces)
    {
        if (epsilon <= 0.0f)
            epsilon = CsgCommon.DefaultEpsilon;

        var faceCount = polygons.Count;
        _faces = new CsgPolygon[faceCount];
        _triangleStart = new int[faceCount + 1];
        _faceMin = new Vec3[faceCount];
        _faceMax = new Vec3[faceCount];

        var estimatedTriangles = 0;
        for (var i = 0; i < faceCount; ++i)
            estimatedTriangles += System.Math.Max(polygons[i].Vertices.Count - 2, 0);
        var positions = new Vec3[estimatedTriangles * 3];
        var locals = new int[estimatedTriangles * 3];

        var scratch = new List<uint>();
        var triangleCount = 0;
        for (var i = 0; i < faceCount; ++i)
        {
            var face = copyFaces ? polygons[i].Clone() : polygons[i];
            _faces[i] = face;
            _triangleStart[i] = triangleCount;
            var vertices = CollectionsMarshal.AsSpan(face.Vertices);

            var min = new Vec3(float.PositiveInfinity);
            var max = new Vec3(float.NegativeInfinity);
            foreach (ref readonly var vertex in vertices)
            {
                var point = Position(vertex);
                min = Vec3.MinNative(min, point);
                max = Vec3.MaxNative(max, point);
            }
            _faceMin[i] = min;
            _faceMax[i] = max;

            if (vertices.Length < 3)
                continue;
            if (vertices.Length == 3)
                AppendTriangle(vertices, 0, 1, 2);
            else if (Csg.TriangulatePolygon(face, scratch))
            {
                for (var t = 0; t + 2 < scratch.Count; t += 3)
                    AppendTriangle(vertices, (int)scratch[t], (int)scratch[t + 1], (int)scratch[t + 2]);
            }
        }
        _triangleStart[faceCount] = triangleCount;
        _trianglePositions = positions.Length == triangleCount * 3 ? positions : positions[..(triangleCount * 3)];
        _triangleVertices = locals.Length == triangleCount * 3 ? locals : locals[..(triangleCount * 3)];

        IsClosed = BuildTopology(epsilon, out _componentStart, out _componentFaces, out _faceComponents,
            out _faceRanks, out _adjacencyStart, out _adjacentFaces);
        (_boundsMin, _boundsMax) = MergeBounds(_faceMin, _faceMax);
        ComponentBounds(_componentStart, _componentFaces, _faceMin, _faceMax, out _componentMin, out _componentMax);
        _faceTrees = new FaceBvh?[ComponentCount];

        void AppendTriangle(ReadOnlySpan<ModelVertex> vertices, int a, int b, int c)
        {
            if (triangleCount * 3 + 3 > positions.Length)
            {
                var capacity = System.Math.Max(positions.Length * 2, 48);
                Array.Resize(ref positions, capacity);
                Array.Resize(ref locals, capacity);
            }
            var offset = triangleCount * 3;
            positions[offset] = Position(vertices[a]);
            positions[offset + 1] = Position(vertices[b]);
            positions[offset + 2] = Position(vertices[c]);
            locals[offset] = a;
            locals[offset + 1] = b;
            locals[offset + 2] = c;
            ++triangleCount;
        }
    }

    /// <summary>Copy editable polygon faces for a caller that needs to change them.</summary>
    public List<CsgPolygon> ToPolygons()
    {
        var result = new List<CsgPolygon>(_faces.Length);
        foreach (var face in _faces)
            result.Add(face.Clone());
        return result;
    }

    private CsgSolid(SolidBuilder builder)
    {
        _faces = builder.Faces.ToArray();
        _triangleStart = builder.TriangleStart.ToArray();
        _trianglePositions = builder.Positions.ToArray();
        _triangleVertices = builder.Locals.ToArray();
        _faceMin = builder.FaceMin.ToArray();
        _faceMax = builder.FaceMax.ToArray();
        _componentStart = builder.ComponentStart.ToArray();
        _componentFaces = builder.ComponentFaces.ToArray();
        _faceComponents = builder.FaceComponents.ToArray();
        _faceRanks = builder.FaceRanks.ToArray();
        _adjacencyStart = builder.AdjacencyStart.ToArray();
        _adjacentFaces = builder.AdjacentFaces.ToArray();
        IsClosed = builder.Closed;
        (_boundsMin, _boundsMax) = MergeBounds(_faceMin, _faceMax);
        ComponentBounds(_componentStart, _componentFaces, _faceMin, _faceMax, out _componentMin, out _componentMax);
        _faceTrees = new FaceBvh?[ComponentCount];
    }

    private static (Vec3 Min, Vec3 Max) MergeBounds(Vec3[] faceMin, Vec3[] faceMax)
    {
        var min = new Vec3(float.PositiveInfinity);
        var max = new Vec3(float.NegativeInfinity);
        for (var i = 0; i < faceMin.Length; ++i)
        {
            min = Vec3.MinNative(min, faceMin[i]);
            max = Vec3.MaxNative(max, faceMax[i]);
        }
        return (min, max);
    }

    private bool BoundsOverlap(Vec3 min, Vec3 max, float epsilon) =>
        _boundsMin.X <= max.X + epsilon && min.X <= _boundsMax.X + epsilon
        && _boundsMin.Y <= max.Y + epsilon && min.Y <= _boundsMax.Y + epsilon
        && _boundsMin.Z <= max.Z + epsilon && min.Z <= _boundsMax.Z + epsilon;

    /// <summary>
    /// Assembles a boolean result from whole operand components and freshly built contact regions.
    /// Components are closed and disjoint from everything else in the result, so their topology is
    /// copied with renumbering instead of being rebuilt.
    /// </summary>
    private sealed class SolidBuilder
    {
        public readonly List<CsgPolygon> Faces;
        public readonly List<int> TriangleStart;
        public readonly List<Vec3> Positions;
        public readonly List<int> Locals;
        public readonly List<Vec3> FaceMin;
        public readonly List<Vec3> FaceMax;
        public readonly List<int> ComponentStart = [0];
        public readonly List<int> ComponentFaces;
        public readonly List<int> FaceComponents;
        public readonly List<int> FaceRanks;
        public readonly List<int> AdjacencyStart;
        public readonly List<int> AdjacentFaces;
        public bool Closed = true;

        /// <summary>Capacities are the operand totals, an upper bound for results of whole components.</summary>
        public SolidBuilder(int faceCapacity, int cornerCapacity)
        {
            Faces = new List<CsgPolygon>(faceCapacity);
            TriangleStart = new List<int>(faceCapacity + 1) { 0 };
            Positions = new List<Vec3>(cornerCapacity);
            Locals = new List<int>(cornerCapacity);
            FaceMin = new List<Vec3>(faceCapacity);
            FaceMax = new List<Vec3>(faceCapacity);
            ComponentFaces = new List<int>(faceCapacity);
            FaceComponents = new List<int>(faceCapacity);
            FaceRanks = new List<int>(faceCapacity);
            AdjacencyStart = new List<int>(faceCapacity + 1) { 0 };
            AdjacentFaces = new List<int>(cornerCapacity);
        }

        public void AppendComponent(CsgSolid source, int component, bool flip)
        {
            var first = source._componentStart[component];
            var end = source._componentStart[component + 1];
            var baseIndex = Faces.Count - source._faceRanks[source._componentFaces[first]];
            var componentIndex = ComponentStart.Count - 1;
            for (var c = first; c < end; ++c)
            {
                var face = source._componentFaces[c];
                var newIndex = Faces.Count;
                ComponentFaces.Add(newIndex);
                FaceRanks.Add(c - first);
                AppendFace(source, face, flip, componentIndex, baseIndex, false);
            }
            ComponentStart.Add(ComponentFaces.Count);
        }

        /// <summary>Append every face of a solid in its own order, keeping its components.</summary>
        public void AppendSolid(CsgSolid source)
        {
            var faceBase = Faces.Count;
            var componentBase = ComponentStart.Count - 1;
            for (var face = 0; face < source._faces.Length; ++face)
                AppendFace(source, face, false, componentBase + source._faceComponents[face], faceBase, true);
            for (var component = 0; component < source.ComponentCount; ++component)
            {
                for (var c = source._componentStart[component]; c < source._componentStart[component + 1]; ++c)
                    ComponentFaces.Add(faceBase + source._componentFaces[c]);
                ComponentStart.Add(ComponentFaces.Count);
            }
            Closed &= source.IsClosed;
        }

        private void AppendFace(CsgSolid source, int face, bool flip, int component, int indexBase,
            bool faceIndexed)
        {
            Faces.Add(flip ? Flipped(source._faces[face]) : source._faces[face]);
            FaceComponents.Add(component);
            if (faceIndexed)
                FaceRanks.Add(source._faceRanks[face]);
            FaceMin.Add(source._faceMin[face]);
            FaceMax.Add(source._faceMax[face]);

            var vertexCount = source._faces[face].Vertices.Count;
            var firstTriangle = source._triangleStart[face];
            var triangleCount = source._triangleStart[face + 1] - firstTriangle;
            Positions.AddRange(source._trianglePositions.AsSpan(firstTriangle * 3, triangleCount * 3));
            var locals = source._triangleVertices.AsSpan(firstTriangle * 3, triangleCount * 3);
            if (flip)
            {
                // Flipping reverses the vertex list; keep the same corners under their new indices.
                foreach (var local in locals)
                    Locals.Add(vertexCount - 1 - local);
            }
            else
                Locals.AddRange(locals);
            TriangleStart.Add(TriangleStart[^1] + triangleCount);

            for (var n = source._adjacencyStart[face]; n < source._adjacencyStart[face + 1]; ++n)
            {
                var neighbor = source._adjacentFaces[n];
                AdjacentFaces.Add(indexBase + (faceIndexed ? neighbor : source._faceRanks[neighbor]));
            }
            AdjacencyStart.Add(AdjacentFaces.Count);
        }
    }

    /// <summary>Build a face tree over every face; for builder benchmarks.</summary>
    internal int BuildFaceTreeForBenchmark() => new FaceBvh(_faceMin, _faceMax).Nodes.Length;

    /// <summary>
    /// Close the seams left along intersection curves. Where the surface grazes an edge at a shallow angle,
    /// the two sides of a seam can place a curve vertex a few millimeters apart, leaving a sliver-shaped gap
    /// with a short unmatched edge at its wide end. Collapsing short unmatched edges makes the long sides of
    /// such a gap coincide; any gap that remains is filled.
    /// </summary>
    private CsgSolid Healed(float epsilon)
    {
        // Moving vertices can expose a neighboring sliver, so collapse repeats a few times before filling.
        var solid = this;
        for (var round = 0; round < MaxCollapseRounds && solid._openEdges is not null; ++round)
        {
            var collapsed = solid.CollapseShortOpenEdges(epsilon);
            if (ReferenceEquals(collapsed, solid))
                break;
            solid = collapsed;
        }
        if (solid._openEdges is not null)
            solid = solid.WithFilledCracks(epsilon);

        // A hole propagates into every later operation, while a slightly moved vertex does not. Any gap left
        // at this point is closed by collapsing its whole boundary, trading local accuracy for a closed mesh.
        // Each round merges at least one vertex pair, so the loop ends.
        while (solid._openEdges is not null)
        {
            var collapsed = solid.CollapseShortOpenEdges(epsilon, all: true);
            if (ReferenceEquals(collapsed, solid))
                break;
            solid = collapsed;
        }
        return solid;
    }

    /// <summary>
    /// Merge the endpoints of short unmatched edges, or of every unmatched edge when <paramref name="all"/> is
    /// set; returns this solid when none qualify.
    /// </summary>
    private CsgSolid CollapseShortOpenEdges(float epsilon, bool all = false)
    {
        var welder = _crackWelder!;
        var parents = new Dictionary<int, int>();
        // A seam sliver has two long unmatched sides and a short one at its wide end. The short edge qualifies
        // when it is small in absolute terms or small next to the unmatched edges beside it.
        var longest = new Dictionary<int, float>();
        foreach (var edge in _openEdges!)
        {
            var length = (edge.ToPoint - edge.FromPoint).Length();
            foreach (var id in (ReadOnlySpan<int>)[edge.From, edge.To])
            {
                ref var value = ref CollectionsMarshal.GetValueRefOrAddDefault(longest, id, out _);
                value = MathF.Max(value, length);
            }
        }
        foreach (var edge in _openEdges)
        {
            var length = (edge.ToPoint - edge.FromPoint).Length();
            var beside = MathF.Max(longest[edge.From], longest[edge.To]);
            var limit = MathF.Min(epsilon * CrackCollapseMaxScale,
                MathF.Max(epsilon * CrackCollapseScale, beside * CrackCollapseFraction));
            if (!all && length > limit)
                continue;
            var a = Find(edge.From);
            var b = Find(edge.To);
            if (a != b)
                parents[System.Math.Max(a, b)] = System.Math.Min(a, b);
        }
        if (parents.Count == 0)
            return this;

        // Every merged vertex moves to the average of its group.
        var members = new HashSet<int>(parents.Keys.Concat(parents.Values));
        var totals = new Dictionary<int, (Vec3 Sum, int Count)>();
        foreach (var id in members)
        {
            ref var total = ref CollectionsMarshal.GetValueRefOrAddDefault(totals, Find(id), out _);
            total = (total.Sum + welder.Positions[id], total.Count + 1);
        }

        var faces = new List<CsgPolygon>(_faces.Length);
        foreach (var face in _faces)
        {
            CsgPolygon? moved = null;
            var vertices = CollectionsMarshal.AsSpan(face.Vertices);
            for (var i = 0; i < vertices.Length; ++i)
            {
                var id = welder.Find(Position(vertices[i]));
                if (id < 0 || !members.Contains(id))
                    continue;
                moved ??= face.CloneShared();
                var (sum, count) = totals[Find(id)];
                var vertex = moved.Vertices[i];
                vertex.SetPosition(ToVector3f(sum / count));
                moved.Vertices[i] = vertex;
            }
            faces.Add(moved ?? face);
        }
        return new CsgSolid(faces, epsilon, false);

        int Find(int id)
        {
            while (parents.TryGetValue(id, out var parent))
                id = parent;
            return id;
        }
    }

    private CsgSolid WithFilledCracks(float epsilon)
    {
        var open = _openEdges!;
        var outgoing = new Dictionary<int, List<int>>();
        for (var i = 0; i < open.Count; ++i)
        {
            if (!outgoing.TryGetValue(open[i].From, out var list))
                outgoing.Add(open[i].From, list = []);
            list.Add(i);
        }

        var faces = new List<CsgPolygon>(_faces);
        var used = new bool[open.Count];
        var loop = new List<int>();
        for (var start = 0; start < open.Count; ++start)
        {
            if (used[start])
                continue;
            loop.Clear();
            loop.Add(start);
            used[start] = true;
            var closedLoop = false;
            while (loop.Count <= open.Count)
            {
                var head = open[loop[^1]].To;
                if (head == open[start].From)
                {
                    closedLoop = true;
                    break;
                }
                var next = -1;
                if (outgoing.TryGetValue(head, out var candidates))
                {
                    foreach (var candidate in candidates)
                    {
                        if (!used[candidate])
                        {
                            next = candidate;
                            break;
                        }
                    }
                }
                if (next < 0)
                    break;
                used[next] = true;
                loop.Add(next);
            }
            if (!closedLoop || loop.Count < 3)
                continue;

            // Fan triangles from the first loop vertex; the loop direction already opposes the gap's boundary.
            var normal = Vec3.Zero;
            var origin = open[loop[0]].FromPoint;
            for (var i = 1; i + 1 < loop.Count; ++i)
                normal += Vec3.Cross(open[loop[i]].FromPoint - origin, open[loop[i + 1]].FromPoint - origin);
            var unitNormal = normal.LengthSquared() > 1e-30f ? Vec3.Normalize(normal) : Vec3.UnitY;
            for (var i = 1; i + 1 < loop.Count; ++i)
            {
                faces.Add(new CsgPolygon([
                    FillVertex(origin, unitNormal),
                    FillVertex(open[loop[i]].FromPoint, unitNormal),
                    FillVertex(open[loop[i + 1]].FromPoint, unitNormal)
                ], open[loop[0]].Shared));
            }
        }
        return faces.Count == _faces.Length ? this : new CsgSolid(faces, epsilon, false);

        static ModelVertex FillVertex(Vec3 position, Vec3 normal)
        {
            var vertex = new ModelVertex();
            vertex.SetPosition(ToVector3f(position));
            vertex.SetNormal(ToVector3f(normal));
            return vertex;
        }
    }

    /// <summary>Faces owned by this solid. Callers must not modify them or their vertex attribute arrays.</summary>
    internal IReadOnlyList<CsgPolygon> Faces => _faces;

    internal CsgSolid Boolean(CsgSolid other, CsgOperation operation, float epsilon)
    {
        // Solids are immutable, so an unchanged operand is itself the result.
        if (_faces.Length == 0)
            return operation == CsgOperation.Union ? other : this;
        if (other._faces.Length == 0)
            return operation == CsgOperation.Intersection ? other : this;

        // Regions are classified by one sample each with ray parity, which requires closed operands.
        var contactsA = new List<int>[ComponentCount];
        var contactsB = new List<int>[other.ComponentCount];
        var componentLinks = new HashSet<(int A, int B)>();
        var frontierA = new Frontier[_faces.Length];
        var frontierB = new Frontier[other._faces.Length];

        // Operands with disjoint bounds need neither tree for contact.
        var pairs = new List<(int A, int B)>();
        if (BoundsOverlap(other._boundsMin, other._boundsMax, epsilon))
        {
            var componentPairs = new List<(int A, int B)>();
            ComponentTree.CollectOverlaps(other.ComponentTree, epsilon, componentPairs);
            foreach (var (componentA, componentB) in componentPairs)
                FaceTree(componentA).CollectOverlaps(other.FaceTree(componentB), epsilon, pairs);
        }
        foreach (var (faceA, faceB) in pairs)
        {
            if (!CollectIntersectionEdges(faceA, other, faceB, epsilon, frontierA, frontierB))
                continue;
            var componentA = _faceComponents[faceA];
            var componentB = other._faceComponents[faceB];
            if (componentLinks.Add((componentA, componentB)))
            {
                (contactsA[componentA] ??= []).Add(componentB);
                (contactsB[componentB] ??= []).Add(componentA);
            }
        }

        var curvePoints = SnapCurvePoints(frontierA, frontierB, epsilon);
        var result = new SolidBuilder(_faces.Length + other._faces.Length, _trianglePositions.Length + other._trianglePositions.Length);
        var visitedA = new bool[ComponentCount];
        var visitedB = new bool[other.ComponentCount];
        var queue = new Queue<(bool InA, int Index)>();

        for (var component = 0; component < ComponentCount; ++component)
        {
            if (visitedA[component])
                continue;
            if (contactsA[component] is null)
            {
                visitedA[component] = true;
                var inside = other.ContainsPoint(ComponentSample(component), epsilon);
                if (operation == CsgOperation.Intersection ? inside : !inside)
                    result.AppendComponent(this, component, false);
                continue;
            }

            var groupA = new List<int>();
            var groupB = new List<int>();
            visitedA[component] = true;
            queue.Enqueue((true, component));
            while (queue.TryDequeue(out var item))
            {
                if (item.InA)
                {
                    groupA.Add(item.Index);
                    foreach (var neighbor in contactsA[item.Index]!)
                    {
                        if (visitedB[neighbor])
                            continue;
                        visitedB[neighbor] = true;
                        queue.Enqueue((false, neighbor));
                    }
                }
                else
                {
                    groupB.Add(item.Index);
                    foreach (var neighbor in contactsB[item.Index]!)
                    {
                        if (visitedA[neighbor])
                            continue;
                        visitedA[neighbor] = true;
                        queue.Enqueue((true, neighbor));
                    }
                }
            }

            var local = new List<CsgPolygon>();
            AppendContactGroup(local, other, groupA, groupB, frontierA, frontierB, curvePoints, operation, epsilon);
            var region = new CsgSolid(local, epsilon, false);
            result.AppendSolid(region._openEdges is null ? region : region.Healed(epsilon));
        }

        for (var component = 0; component < other.ComponentCount; ++component)
        {
            if (visitedB[component])
                continue;
            var inside = ContainsPoint(other.ComponentSample(component), epsilon);
            if (operation == CsgOperation.Union && !inside
                || operation == CsgOperation.Intersection && inside
                || operation == CsgOperation.DifferenceAB && inside)
                result.AppendComponent(other, component, operation == CsgOperation.DifferenceAB);
        }
        return new CsgSolid(result);
    }

    private static CsgPolygon Flipped(CsgPolygon face)
    {
        var result = face.CloneShared();
        result.Flip();
        return result;
    }

    private void AppendContactGroup(List<CsgPolygon> result, CsgSolid other,
        IReadOnlyList<int> groupA, IReadOnlyList<int> groupB, Frontier[] frontierA, Frontier[] frontierB,
        PointWelder curvePoints, CsgOperation operation, float epsilon)
    {
        // The intersection segments form barriers in the shared-edge graph. Classify every intact
        // patch once, then split only faces at the frontier using local planes.
        AppendPatchesAndFragments(result, other, groupA, frontierA, curvePoints, operation, false, epsilon);
        other.AppendPatchesAndFragments(result, this, groupB, frontierB, curvePoints, operation, true, epsilon);
    }

    /// <summary>
    /// Merge cut endpoints of both operands that lie within a few epsilons into one representative, and move
    /// every cut onto its representatives. Each side splits its faces independently; without a shared set of
    /// curve vertices, the two sides compute copies of a curve vertex that can differ by more than epsilon at
    /// steep face angles, and the seam between them stays open.
    /// </summary>
    private static PointWelder SnapCurvePoints(Frontier[] frontierA, Frontier[] frontierB, float epsilon)
    {
        var points = new PointWelder(epsilon * CurveSnapScale, 64);
        Snap(frontierA);
        Snap(frontierB);
        return points;

        void Snap(Frontier[] frontiers)
        {
            foreach (var frontier in frontiers)
            {
                if (frontier is null)
                    continue;
                var cuts = CollectionsMarshal.AsSpan(frontier.Cuts);
                foreach (ref var cut in cuts)
                {
                    // Add may grow the position array, so index it only after both calls.
                    var startId = points.Add(ToVec3(cut.Start));
                    var endId = points.Add(ToVec3(cut.End));
                    var start = points.Positions[startId];
                    var end = points.Positions[endId];
                    cut = cut with { Start = ToVector3f(start), End = ToVector3f(end) };
                }
            }
        }
    }

    private static Vec3 ToVec3(Vector3f value) => new(value.X, value.Y, value.Z);

    private static Vector3f ToVector3f(Vec3 value) => new(value.X, value.Y, value.Z);

    /// <summary>
    /// Cuts collected for one frontier face, and the faces of the other solid that lie in its plane. A face
    /// with coplanar partners enters the frontier even without cuts, because a surface sample on it would lie
    /// on the other solid's surface.
    /// </summary>
    private sealed class Frontier
    {
        public readonly List<IntersectionEdge> Cuts = [];
        public List<int>? CoplanarFaces;
    }

    private void AppendPatchesAndFragments(List<CsgPolygon> output, CsgSolid other,
        IReadOnlyList<int> components, Frontier[] frontier, PointWelder curvePoints,
        CsgOperation operation, bool secondOperand, float epsilon)
    {
        var visited = new bool[_faces.Length];
        var patch = new List<int>();
        var flip = secondOperand && operation == CsgOperation.DifferenceAB;
        foreach (var component in components)
        {
            for (var c = _componentStart[component]; c < _componentStart[component + 1]; ++c)
            {
                var face = _componentFaces[c];
                if (visited[face])
                    continue;
                visited[face] = true;
                if (frontier[face] is { } faceFrontier)
                {
                    SplitAndClassifyFace(output, face, faceFrontier, curvePoints, other, operation, secondOperand,
                        epsilon);
                    continue;
                }

                // Breadth-first flood over shared edges; the patch list doubles as the queue.
                patch.Clear();
                patch.Add(face);
                for (var head = 0; head < patch.Count; ++head)
                {
                    var current = patch[head];
                    for (var n = _adjacencyStart[current]; n < _adjacencyStart[current + 1]; ++n)
                    {
                        var neighbor = _adjacentFaces[n];
                        if (visited[neighbor] || frontier[neighbor] is not null)
                            continue;
                        visited[neighbor] = true;
                        patch.Add(neighbor);
                    }
                }
                if (!Keep(other.ContainsPoint(FaceSample(face), epsilon), operation, secondOperand))
                    continue;
                output.EnsureCapacity(output.Count + patch.Count);
                foreach (var patchFace in patch)
                    output.Add(flip ? Flipped(_faces[patchFace]) : _faces[patchFace]);
            }
        }
    }

    private void SplitAndClassifyFace(List<CsgPolygon> output, int faceIndex, Frontier frontier,
        PointWelder curvePoints, CsgSolid other, CsgOperation operation, bool secondOperand, float epsilon)
    {
        var face = _faces[faceIndex];
        var pieces = new List<CsgPolygon>();
        if (face.IsConvex())
            pieces.Add(face.CloneShared());
        else
        {
            for (var t = _triangleStart[faceIndex]; t < _triangleStart[faceIndex + 1]; ++t)
            {
                pieces.Add(new CsgPolygon(
                [
                    face.Vertices[_triangleVertices[t * 3]],
                    face.Vertices[_triangleVertices[t * 3 + 1]],
                    face.Vertices[_triangleVertices[t * 3 + 2]]
                ], face.Shared));
            }
        }

        // Every intersection segment must become a fragment boundary; then no fragment interior crosses
        // the other surface and one sample classifies it. Cutting only the pieces a segment passes
        // through keeps the fragment count linear in the number of segments. Cut lines that run past a
        // segment's end separate fragments of equal class, and their T-junctions are resolved by topology.
        var normal = face.Plane.Normal;
        var split = new List<CsgPolygon>();
        foreach (var segment in frontier.Cuts)
        {
            var plane = segment.Cut;
            split.Clear();
            split.EnsureCapacity(pieces.Count + 1);
            foreach (var piece in pieces)
            {
                if (!Straddles(piece, plane, epsilon)
                    || !SegmentCrossesInterior(piece, normal, segment.Start, segment.End, epsilon))
                {
                    split.Add(piece);
                    continue;
                }
                BspNode.SplitPolygon(piece, plane, out var first, out var second, epsilon);
                if (first.Vertices.Count >= 3)
                    split.Add(first);
                if (second.Vertices.Count >= 3)
                    split.Add(second);
            }
            (pieces, split) = (split, pieces);
        }

        // Vertices created by the cuts snap to the shared curve vertices. Original corners stay in place, since
        // untouched neighbor faces share them.
        var corners = CollectionsMarshal.AsSpan(face.Vertices);
        foreach (var piece in pieces)
        {
            foreach (ref var vertex in CollectionsMarshal.AsSpan(piece.Vertices))
            {
                var isCorner = false;
                foreach (ref readonly var corner in corners)
                {
                    if (corner.Position == vertex.Position)
                    {
                        isCorner = true;
                        break;
                    }
                }
                if (isCorner)
                    continue;
                var snapped = curvePoints.Find(Position(vertex));
                if (snapped >= 0)
                    vertex.SetPosition(ToVector3f(curvePoints.Positions[snapped]));
            }
        }

        var selected = new List<CsgPolygon>(pieces.Count);
        foreach (var piece in pieces)
        {
            // A piece thinner than epsilon bounds no surface; kept, it duplicates edges of its neighbors.
            if (IsSliver(CollectionsMarshal.AsSpan(piece.Vertices), epsilon))
                continue;
            var sample = PolygonSample(piece);
            var keep = frontier.CoplanarFaces is { } coplanar
                && other.FindCoplanarFace(coplanar, sample, epsilon) is var partner && partner >= 0
                    ? KeepCoplanar(face.Plane.Normal.DotProduct(other._faces[partner].Plane.Normal) > 0.0f,
                        operation, secondOperand)
                    : Keep(other.ContainsPoint(sample, epsilon), operation, secondOperand);
            if (!keep)
                continue;
            // Every piece owns its vertex list, so flipping in place cannot touch the source face.
            if (secondOperand && operation == CsgOperation.DifferenceAB)
                piece.Flip();
            selected.Add(piece);
        }
        Csg.SimplifyPolygons(selected, epsilon);
        foreach (var polygon in selected)
        {
            if (!IsSliver(CollectionsMarshal.AsSpan(polygon.Vertices), epsilon))
                output.Add(polygon);
        }
    }

    /// <summary>
    /// Coplanar overlap: both solids share this surface region. The first operand's copy represents it when it
    /// survives, so the second operand's copy is always dropped. With equal orientation the region bounds both
    /// solids from the same side: union and intersection keep it, a difference removes it. With opposite
    /// orientation the solids only touch there: only a difference keeps the first operand's surface.
    /// </summary>
    private static bool KeepCoplanar(bool sameOrientation, CsgOperation operation, bool secondOperand) =>
        !secondOperand
        && (sameOrientation ? operation != CsgOperation.DifferenceAB : operation == CsgOperation.DifferenceAB);

    /// <summary>Index of a listed face whose triangles contain the point within epsilon of its plane, or -1.</summary>
    private int FindCoplanarFace(List<int> faces, Vector3f point, float epsilon)
    {
        var p = new Vec3(point.X, point.Y, point.Z);
        foreach (var face in faces)
        {
            if (MathF.Abs(_faces[face].Plane.Distance(point)) > epsilon * 2.0f)
                continue;
            for (var t = _triangleStart[face]; t < _triangleStart[face + 1]; ++t)
            {
                if (TriangleContains(_trianglePositions[t * 3], _trianglePositions[t * 3 + 1],
                        _trianglePositions[t * 3 + 2], p))
                    return face;
            }
        }
        return -1;
    }

    /// <summary>Whether the projection of a point onto a triangle's plane lies inside the triangle.</summary>
    private static bool TriangleContains(Vec3 a, Vec3 b, Vec3 c, Vec3 p)
    {
        var normal = Vec3.Cross(b - a, c - a);
        return Vec3.Dot(Vec3.Cross(b - a, p - a), normal) >= 0.0f
            && Vec3.Dot(Vec3.Cross(c - b, p - b), normal) >= 0.0f
            && Vec3.Dot(Vec3.Cross(a - c, p - c), normal) >= 0.0f;
    }

    /// <summary>
    /// Whether a polygon is thinner than epsilon: twice its area over its longest edge. Such needles arise
    /// when a cut runs within epsilon of an existing edge and carry no surface.
    /// </summary>
    private static bool IsSliver(ReadOnlySpan<ModelVertex> vertices, float epsilon)
    {
        if (vertices.Length < 3)
            return true;
        var normal = Vec3.Zero;
        var longest = 0.0f;
        var first = Position(vertices[0]);
        var previous = first;
        for (var i = 1; i <= vertices.Length; ++i)
        {
            var current = i == vertices.Length ? first : Position(vertices[i]);
            normal += Vec3.Cross(previous - first, current - first);
            longest = MathF.Max(longest, (current - previous).LengthSquared());
            previous = current;
        }
        // |normal| is twice the area; compare twice the area with epsilon times the longest edge.
        return normal.Length() <= epsilon * MathF.Sqrt(longest);
    }

    private static bool Straddles(CsgPolygon piece, Plane plane, float epsilon)
    {
        var front = false;
        var back = false;
        foreach (ref readonly var vertex in CollectionsMarshal.AsSpan(piece.Vertices))
        {
            var distance = plane.Distance(vertex.GetPosition());
            front |= distance > epsilon;
            back |= distance < -epsilon;
        }
        return front && back;
    }

    /// <summary>
    /// Whether a segment in the piece's plane comes within epsilon of the piece. The test errs toward cutting:
    /// an unneeded cut only separates fragments of equal class, while a missed one misclassifies a piece.
    /// Pieces are convex: frontier faces start convex or as triangles and plane splits preserve convexity.
    /// </summary>
    private static bool SegmentCrossesInterior(CsgPolygon piece, Vector3f normal, Vector3f start, Vector3f end,
        float epsilon)
    {
        var vertices = CollectionsMarshal.AsSpan(piece.Vertices);
        var n = new Vec3(normal.X, normal.Y, normal.Z);
        var s = new Vec3(start.X, start.Y, start.Z);
        var e = new Vec3(end.X, end.Y, end.Z);
        var low = 0.0f;
        var high = 1.0f;
        for (var i = 0; i < vertices.Length; ++i)
        {
            var from = Position(vertices[i]);
            var edge = Position(vertices[i + 1 == vertices.Length ? 0 : i + 1]) - from;
            var inward = Vec3.Cross(n, edge);
            var length = inward.Length();
            if (length <= 1e-20f)
                continue;
            // Signed distances of the segment ends inside this edge, with an epsilon margin outward.
            var ds = Vec3.Dot(inward, s - from) / length + epsilon;
            var de = Vec3.Dot(inward, e - from) / length + epsilon;
            if (ds < 0.0f && de < 0.0f)
                return false;
            if (ds < 0.0f)
                low = MathF.Max(low, ds / (ds - de));
            else if (de < 0.0f)
                high = MathF.Min(high, ds / (ds - de));
            if (low >= high)
                return false;
        }
        return true;
    }

    private Vector3f FaceSample(int faceIndex)
    {
        var offset = _triangleStart[faceIndex] * 3;
        var centroid = (_trianglePositions[offset] + _trianglePositions[offset + 1]
            + _trianglePositions[offset + 2]) / 3.0f;
        return new Vector3f(centroid.X, centroid.Y, centroid.Z);
    }

    private static Vector3f PolygonSample(CsgPolygon polygon)
    {
        var result = Vec3.Zero;
        foreach (ref readonly var vertex in CollectionsMarshal.AsSpan(polygon.Vertices))
            result += Position(vertex);
        result /= polygon.Vertices.Count;
        return new Vector3f(result.X, result.Y, result.Z);
    }

    private static bool Keep(bool inside, CsgOperation operation, bool secondOperand) =>
        operation == CsgOperation.Intersection ? inside
        : secondOperand && operation == CsgOperation.DifferenceAB ? inside : !inside;

    private Vector3f ComponentSample(int component)
    {
        var faceIndex = _componentFaces[_componentStart[component]];
        if (_triangleStart[faceIndex] == _triangleStart[faceIndex + 1])
            return _faces[faceIndex].Vertices[0].GetPosition();
        return FaceSample(faceIndex);
    }

    // Non-axis-aligned rays avoid most shared-edge and vertex hits.
    private static readonly Vec3[] RayDirections =
    [
        Vec3.Normalize(new Vec3(1.0f, 0.371f, 0.173f)),
        Vec3.Normalize(new Vec3(0.239f, 1.0f, 0.419f)),
        Vec3.Normalize(new Vec3(0.193f, 0.317f, 1.0f))
    ];

    [ThreadStatic]
    private static float[]? _rayHits;

    /// <summary>Classify a point in a closed solid using BVH-accelerated ray parity.</summary>
    public bool ContainsPoint(Vector3f point, float epsilon = CsgCommon.DefaultEpsilon)
    {
        if (epsilon <= 0.0f)
            epsilon = CsgCommon.DefaultEpsilon;
        if (_faces.Length == 0)
            return false;

        var origin = new Vec3(point.X, point.Y, point.Z);
        if (!BoundsOverlap(origin, origin, 0.0f))
            return false;
        var insideVotes = 0;
        for (var ray = 0; ray < RayDirections.Length; ++ray)
        {
            if ((CountCrossings(origin, RayDirections[ray], epsilon) & 1) != 0)
                ++insideVotes;
            // Majority of three: stop once the remaining ray cannot change the outcome.
            if (insideVotes == 2 || ray - insideVotes == 1)
                break;
        }
        return insideVotes >= 2;
    }

    private int CountCrossings(Vec3 origin, Vec3 direction, float epsilon)
    {
        var hits = _rayHits ??= new float[64];
        var hitCount = 0;
        var inverse = Vec3.One / direction;
        var components = ComponentTree;
        Span<int> stack = stackalloc int[64];
        var top = 0;
        stack[top++] = 0;
        while (top != 0)
        {
            ref readonly var node = ref components.Nodes[stack[--top]];
            if (!RayHitsBox(origin, inverse, node.Min, node.Max))
                continue;
            if (node.Count == 0)
            {
                Push(ref stack, top, node.First);
                top += 2;
                continue;
            }
            for (var i = node.First; i < node.First + node.Count; ++i)
            {
                var component = components.Items[i];
                if (RayHitsBox(origin, inverse, _componentMin[component], _componentMax[component]))
                    CollectRayHits(FaceTree(component), origin, direction, inverse, ref hits, ref hitCount);
            }
        }
        _rayHits = hits;

        // Counting unique distances prevents two triangles on the same surface from changing the parity twice.
        var sorted = hits.AsSpan(0, hitCount);
        sorted.Sort();
        var crossings = 0;
        var last = float.NegativeInfinity;
        foreach (var hit in sorted)
        {
            if (hit - last <= epsilon * 2.0f)
                continue;
            ++crossings;
            last = hit;
        }
        return crossings;
    }

    private void CollectRayHits(FaceBvh tree, Vec3 origin, Vec3 direction, Vec3 inverse, ref float[] hits,
        ref int hitCount)
    {
        Span<int> stack = stackalloc int[64];
        var top = 0;
        stack[top++] = 0;
        while (top != 0)
        {
            ref readonly var node = ref tree.Nodes[stack[--top]];
            if (!RayHitsBox(origin, inverse, node.Min, node.Max))
                continue;
            if (node.Count == 0)
            {
                Push(ref stack, top, node.First);
                top += 2;
                continue;
            }

            for (var i = node.First; i < node.First + node.Count; ++i)
            {
                var face = tree.Items[i];
                for (var t = _triangleStart[face]; t < _triangleStart[face + 1]; ++t)
                {
                    var distance = RayTriangle(origin, direction, _trianglePositions[t * 3],
                        _trianglePositions[t * 3 + 1], _trianglePositions[t * 3 + 2]);
                    if (!(distance > 0.0f) || !float.IsFinite(distance))
                        continue;
                    if (hitCount == hits.Length)
                        Array.Resize(ref hits, hits.Length * 2);
                    hits[hitCount++] = distance;
                }
            }
        }
    }

    /// <summary>Push the child pair of an inner node, growing the traversal stack when needed.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static void Push(ref Span<int> stack, int top, int firstChild)
    {
        if (top + 2 > stack.Length)
        {
            var larger = new int[stack.Length * 2];
            stack.CopyTo(larger);
            stack = larger;
        }
        stack[top] = firstChild;
        stack[top + 1] = firstChild + 1;
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static bool RayHitsBox(Vec3 origin, Vec3 inverse, Vec3 min, Vec3 max)
    {
        var t0 = (min - origin) * inverse;
        var t1 = (max - origin) * inverse;
        var near = Vec3.MinNative(t0, t1);
        var far = Vec3.MaxNative(t0, t1);
        var enter = float.MaxNative(float.MaxNative(near.X, near.Y), float.MaxNative(near.Z, 0.0f));
        var exit = float.MinNative(far.X, float.MinNative(far.Y, far.Z));
        return enter <= exit;
    }

    /// <summary>Two-sided Möller–Trumbore; infinity when the ray misses.</summary>
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static float RayTriangle(Vec3 origin, Vec3 direction, Vec3 v0, Vec3 v1, Vec3 v2)
    {
        var edge1 = v1 - v0;
        var edge2 = v2 - v0;
        var p = Vec3.Cross(direction, edge2);
        var det = Vec3.Dot(edge1, p);
        if (MathF.Abs(det) < MathDefs.Epsilon)
            return float.PositiveInfinity;
        // A negative determinant is the back face: mirror the barycentric tests instead of rewinding.
        var sign = det < 0.0f ? -1.0f : 1.0f;
        det *= sign;
        var offset = origin - v0;
        var u = Vec3.Dot(offset, p) * sign;
        if (u < 0.0f || u > det)
            return float.PositiveInfinity;
        var q = Vec3.Cross(offset, edge1);
        var v = Vec3.Dot(direction, q) * sign;
        if (v < 0.0f || u + v > det)
            return float.PositiveInfinity;
        var distance = Vec3.Dot(edge2, q) * sign / det;
        return distance >= 0.0f ? distance : float.PositiveInfinity;
    }

    /// <summary>
    /// Record the cuts between two faces that touch. Returns whether they touch at all, which links their
    /// components. Crossing triangles are cut along their intersection segment. Coplanar triangles cut each
    /// other along their edges and register as coplanar partners.
    /// </summary>
    private bool CollectIntersectionEdges(int faceIndex, CsgSolid other, int otherIndex, float epsilon,
        Frontier[] frontierA, Frontier[] frontierB)
    {
        var contact = false;
        var positionsA = _trianglePositions;
        var positionsB = other._trianglePositions;
        for (var a = _triangleStart[faceIndex]; a < _triangleStart[faceIndex + 1]; ++a)
        {
            var a0 = positionsA[a * 3];
            var a1 = positionsA[a * 3 + 1];
            var a2 = positionsA[a * 3 + 2];
            for (var b = other._triangleStart[otherIndex]; b < other._triangleStart[otherIndex + 1]; ++b)
            {
                var b0 = positionsB[b * 3];
                var b1 = positionsB[b * 3 + 1];
                var b2 = positionsB[b * 3 + 2];
                if (!TrianglesIntersect(a0, a1, a2, b0, b1, b2, epsilon))
                    continue;
                contact = true;
                if (!TryIntersectionSegment(a0, a1, a2, b0, b1, b2, epsilon,
                        out var start, out var end, out var coplanar))
                {
                    if (coplanar)
                        CollectCoplanarCuts(faceIndex, other, otherIndex, a0, a1, a2, b0, b1, b2, epsilon,
                            frontierA, frontierB);
                    continue; // A zero-length touch does not divide either surface.
                }
                (frontierA[faceIndex] ??= new Frontier()).Cuts.Add(
                    new IntersectionEdge(start, end, other._faces[otherIndex].Plane));
                (frontierB[otherIndex] ??= new Frontier()).Cuts.Add(
                    new IntersectionEdge(start, end, _faces[faceIndex].Plane));
            }
        }
        return contact;
    }

    private void CollectCoplanarCuts(int faceIndex, CsgSolid other, int otherIndex, Vec3 a0, Vec3 a1, Vec3 a2,
        Vec3 b0, Vec3 b1, Vec3 b2, float epsilon, Frontier[] frontierA, Frontier[] frontierB)
    {
        var frontA = frontierA[faceIndex] ??= new Frontier();
        var frontB = frontierB[otherIndex] ??= new Frontier();
        if (!(frontA.CoplanarFaces ??= []).Contains(otherIndex))
            frontA.CoplanarFaces.Add(otherIndex);
        if (!(frontB.CoplanarFaces ??= []).Contains(faceIndex))
            frontB.CoplanarFaces.Add(faceIndex);
        // Each triangle's edges bound the shared region on the other face.
        AddEdgeCuts(frontA, b0, b1, b2, a0, a1, a2, epsilon);
        AddEdgeCuts(frontB, a0, a1, a2, b0, b1, b2, epsilon);
    }

    /// <summary>Add the edges of one triangle, clipped to a coplanar target triangle, as cuts on the target.</summary>
    private static void AddEdgeCuts(Frontier target, Vec3 e0, Vec3 e1, Vec3 e2, Vec3 t0, Vec3 t1, Vec3 t2,
        float epsilon)
    {
        var normal = Vec3.Normalize(Vec3.Cross(t1 - t0, t2 - t0));
        AddEdge(e0, e1);
        AddEdge(e1, e2);
        AddEdge(e2, e0);

        void AddEdge(Vec3 from, Vec3 to)
        {
            if (!ClipToTriangle(ref from, ref to, t0, t1, t2, normal, epsilon))
                return;
            var cutNormal = Vec3.Cross(to - from, normal);
            if (cutNormal.LengthSquared() <= 1e-20f)
                return;
            var start = new Vector3f(from.X, from.Y, from.Z);
            var end = new Vector3f(to.X, to.Y, to.Z);
            var planeNormal = Vec3.Normalize(cutNormal);
            target.Cuts.Add(new IntersectionEdge(start, end,
                new Plane(new Vector3f(planeNormal.X, planeNormal.Y, planeNormal.Z), start)));
        }
    }

    /// <summary>
    /// Clip a segment to a triangle expanded by epsilon in its plane; false when less than epsilon remains.
    /// </summary>
    private static bool ClipToTriangle(ref Vec3 from, ref Vec3 to, Vec3 t0, Vec3 t1, Vec3 t2, Vec3 normal,
        float epsilon)
    {
        var low = 0.0f;
        var high = 1.0f;
        Span<Vec3> corners = [t0, t1, t2];
        for (var i = 0; i < 3; ++i)
        {
            var corner = corners[i];
            var inward = Vec3.Cross(normal, corners[(i + 1) % 3] - corner);
            var length = inward.Length();
            if (length <= 1e-20f)
                continue;
            var ds = Vec3.Dot(inward, from - corner) / length + epsilon;
            var de = Vec3.Dot(inward, to - corner) / length + epsilon;
            if (ds < 0.0f && de < 0.0f)
                return false;
            if (ds < 0.0f)
                low = MathF.Max(low, ds / (ds - de));
            else if (de < 0.0f)
                high = MathF.Min(high, ds / (ds - de));
        }
        if (low >= high)
            return false;
        var start = Vec3.Lerp(from, to, low);
        var end = Vec3.Lerp(from, to, high);
        if ((end - start).LengthSquared() <= epsilon * epsilon)
            return false;
        (from, to) = (start, end);
        return true;
    }

    private static bool TryIntersectionSegment(Vec3 a0, Vec3 a1, Vec3 a2, Vec3 b0, Vec3 b1, Vec3 b2,
        float epsilon, out Vector3f start, out Vector3f end, out bool coplanar)
    {
        start = end = Vector3f.Zero;
        coplanar = false;
        var normalA = Vec3.Normalize(Vec3.Cross(a1 - a0, a2 - a0));
        var normalB = Vec3.Normalize(Vec3.Cross(b1 - b0, b2 - b0));
        var distanceA = -Vec3.Dot(normalA, a0);
        var distanceB = -Vec3.Dot(normalB, b0);
        var line = Vec3.Cross(normalA, normalB);
        if (line.LengthSquared() < 1e-8f)
        {
            coplanar = true; // Parallel triangles that touch within epsilon share a plane.
            return false;
        }
        line = Vec3.Normalize(line);

        var cutsA = new CutList();
        var cutsB = new CutList();
        TrianglePlaneCuts(a0, a1, a2, normalB, distanceB, epsilon, ref cutsA);
        TrianglePlaneCuts(b0, b1, b2, normalA, distanceA, epsilon, ref cutsB);
        if (cutsA.Count < 2 || cutsB.Count < 2)
            return false;

        var origin = cutsA[0];
        var minA = float.PositiveInfinity;
        var maxA = float.NegativeInfinity;
        var minB = float.PositiveInfinity;
        var maxB = float.NegativeInfinity;
        for (var i = 0; i < cutsA.Count; ++i)
        {
            var value = Vec3.Dot(cutsA[i] - origin, line);
            minA = MathF.Min(minA, value);
            maxA = MathF.Max(maxA, value);
        }
        for (var i = 0; i < cutsB.Count; ++i)
        {
            var value = Vec3.Dot(cutsB[i] - origin, line);
            minB = MathF.Min(minB, value);
            maxB = MathF.Max(maxB, value);
        }
        var low = MathF.Max(minA, minB);
        var high = MathF.Min(maxA, maxB);
        if (high - low <= epsilon)
            return false;
        var first = origin + line * low;
        var second = origin + line * high;
        start = new Vector3f(first.X, first.Y, first.Z);
        end = new Vector3f(second.X, second.Y, second.Z);
        return true;
    }

    /// <summary>At most four distinct cut points: three vertices on the plane or three edge crossings.</summary>
    [InlineArray(4)]
    private struct CutBuffer
    {
        private Vec3 _element;
    }

    private struct CutList
    {
        private CutBuffer _points;
        public int Count;

        public readonly Vec3 this[int index] => _points[index];

        public void Add(Vec3 point, float epsilonSquared)
        {
            for (var i = 0; i < Count; ++i)
            {
                if ((_points[i] - point).LengthSquared() <= epsilonSquared)
                    return;
            }
            if (Count < 4)
                _points[Count++] = point;
        }
    }

    private static void TrianglePlaneCuts(Vec3 first, Vec3 second, Vec3 third, Vec3 normal, float distance,
        float epsilon, ref CutList cuts)
    {
        var epsilonSquared = epsilon * epsilon;
        var d0 = Vec3.Dot(normal, first) + distance;
        var d1 = Vec3.Dot(normal, second) + distance;
        var d2 = Vec3.Dot(normal, third) + distance;
        Edge(first, second, d0, d1, ref cuts);
        Edge(second, third, d1, d2, ref cuts);
        Edge(third, first, d2, d0, ref cuts);

        void Edge(Vec3 from, Vec3 to, float fromDistance, float toDistance, ref CutList output)
        {
            if (MathF.Abs(fromDistance) <= epsilon)
                output.Add(from, epsilonSquared);
            if (fromDistance > epsilon && toDistance < -epsilon || fromDistance < -epsilon && toDistance > epsilon)
                output.Add(Vec3.Lerp(from, to, fromDistance / (fromDistance - toDistance)), epsilonSquared);
        }
    }

    private static bool TrianglesIntersect(Vec3 a0, Vec3 a1, Vec3 a2, Vec3 b0, Vec3 b1, Vec3 b2, float epsilon)
    {
        var ea0 = a1 - a0;
        var ea1 = a2 - a1;
        var ea2 = a0 - a2;
        var eb0 = b1 - b0;
        var eb1 = b2 - b1;
        var eb2 = b0 - b2;
        var normalA = Vec3.Cross(ea0, ea1);
        var normalB = Vec3.Cross(eb0, eb1);
        var epsilonSquared = epsilon * epsilon;
        if (normalA.LengthSquared() <= epsilonSquared || normalB.LengthSquared() <= epsilonSquared)
            return false;

        // Plane tests reject most box-overlapping pairs before the edge-edge axes.
        return !Separated(normalA) && !Separated(normalB)
            && !Separated(Vec3.Cross(ea0, normalA)) && !Separated(Vec3.Cross(ea1, normalA))
            && !Separated(Vec3.Cross(ea2, normalA))
            && !Separated(Vec3.Cross(eb0, normalB)) && !Separated(Vec3.Cross(eb1, normalB))
            && !Separated(Vec3.Cross(eb2, normalB))
            && !Separated(Vec3.Cross(ea0, eb0)) && !Separated(Vec3.Cross(ea0, eb1))
            && !Separated(Vec3.Cross(ea0, eb2))
            && !Separated(Vec3.Cross(ea1, eb0)) && !Separated(Vec3.Cross(ea1, eb1))
            && !Separated(Vec3.Cross(ea1, eb2))
            && !Separated(Vec3.Cross(ea2, eb0)) && !Separated(Vec3.Cross(ea2, eb1))
            && !Separated(Vec3.Cross(ea2, eb2));

        bool Separated(Vec3 axis)
        {
            var lengthSquared = axis.LengthSquared();
            if (lengthSquared <= 1e-20f)
                return false;
            var pa0 = Vec3.Dot(a0, axis);
            var pa1 = Vec3.Dot(a1, axis);
            var pa2 = Vec3.Dot(a2, axis);
            var pb0 = Vec3.Dot(b0, axis);
            var pb1 = Vec3.Dot(b1, axis);
            var pb2 = Vec3.Dot(b2, axis);
            var aMin = float.MinNative(pa0, float.MinNative(pa1, pa2));
            var aMax = float.MaxNative(pa0, float.MaxNative(pa1, pa2));
            var bMin = float.MinNative(pb0, float.MinNative(pb1, pb2));
            var bMax = float.MaxNative(pb0, float.MaxNative(pb1, pb2));
            var tolerance = epsilon * MathF.Sqrt(lengthSquared);
            return aMax < bMin - tolerance || bMax < aMin - tolerance;
        }
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vec3 Position(in ModelVertex vertex) =>
        new(vertex.Position.X, vertex.Position.Y, vertex.Position.Z);

    private struct EdgeState
    {
        public int Face;
        public int Count;
        public int Winding;
    }

    private static ulong EdgeKey(int from, int to) =>
        from < to ? (ulong)(uint)from << 32 | (uint)to : (ulong)(uint)to << 32 | (uint)from;

    private bool BuildTopology(float epsilon, out int[] componentStart, out int[] componentFaces,
        out int[] faceComponents, out int[] faceRanks, out int[] adjacencyStart, out int[] adjacentFaces)
    {
        var faceCount = _faces.Length;
        // An empty solid bounds nothing and is trivially closed.
        var closed = true;

        // Weld positions within epsilon so float noise on either side of a rounding boundary cannot
        // split one shared vertex into two ids.
        var vertexStart = new int[faceCount + 1];
        for (var face = 0; face < faceCount; ++face)
            vertexStart[face + 1] = vertexStart[face] + _faces[face].Vertices.Count;
        var vertexIds = new int[vertexStart[faceCount]];
        // Welded loop length per face after dropping repeated ids. A loop that collapses below three
        // ids is a needle or point: it bounds no surface and takes no part in the edge graph.
        var loopLength = new int[faceCount];
        var welder = new PointWelder(epsilon, vertexIds.Length);
        for (var face = 0; face < faceCount; ++face)
        {
            var vertices = CollectionsMarshal.AsSpan(_faces[face].Vertices);
            var start = vertexStart[face];
            var length = 0;
            for (var i = 0; i < vertices.Length; ++i)
            {
                var id = welder.Add(Position(vertices[i]));
                if (length == 0 || vertexIds[start + length - 1] != id)
                    vertexIds[start + length++] = id;
            }
            while (length > 1 && vertexIds[start + length - 1] == vertexIds[start])
                --length;
            loopLength[face] = IsSliver(vertices, epsilon) ? 0 : length;
            if (vertices.Length < 3 || _triangleStart[face] == _triangleStart[face + 1])
                closed = false;
        }

        // Raw edges between welded ids.
        var rawEdges = new Dictionary<ulong, EdgeState>(vertexIds.Length, CsgCommon.PackedKeyComparer.Instance);
        for (var face = 0; face < faceCount; ++face)
        {
            var count = loopLength[face];
            if (count < 3 || _triangleStart[face] == _triangleStart[face + 1])
                continue;
            for (var i = 0; i < count; ++i)
            {
                var from = vertexIds[vertexStart[face] + i];
                var to = vertexIds[vertexStart[face] + (i + 1) % count];
                ref var state = ref CollectionsMarshal.GetValueRefOrAddDefault(rawEdges, EdgeKey(from, to), out _);
                ++state.Count;
                state.Winding += from < to ? 1 : -1;
            }
        }

        // Only edges without an exact opposite partner can end at a T-junction. Split those at welded
        // points on their interior so a long edge matches the shorter edges along it.
        var edges = new Dictionary<ulong, EdgeState>(rawEdges.Count, CsgCommon.PackedKeyComparer.Instance);
        var links = new List<(int Face, int Other)>(rawEdges.Count);
        var parents = new int[faceCount];
        var ranks = new byte[faceCount];
        for (var i = 0; i < faceCount; ++i)
            parents[i] = i;
        FaceBvh? pointBvh = null;
        var cuts = new List<(float T, int Id)>();
        var nearby = new List<int>();
        // T-junction vertices may sit up to the curve snap radius off a long edge; see SnapCurvePoints.
        var junctionTolerance = epsilon * CurveSnapScale;
        var junctionToleranceSquared = junctionTolerance * junctionTolerance;

        for (var face = 0; face < faceCount; ++face)
        {
            var count = loopLength[face];
            if (count < 3 || _triangleStart[face] == _triangleStart[face + 1])
                continue;
            for (var i = 0; i < count; ++i)
            {
                var from = vertexIds[vertexStart[face] + i];
                var to = vertexIds[vertexStart[face] + (i + 1) % count];
                var raw = rawEdges[EdgeKey(from, to)];
                if (raw.Count == 2 && raw.Winding == 0)
                {
                    Register(face, from, to);
                    continue;
                }

                var start = welder.Positions[from];
                var end = welder.Positions[to];
                var span = end - start;
                var spanLengthSquared = span.LengthSquared();
                if (pointBvh is null)
                {
                    var points = welder.Positions.AsSpan(0, welder.Count).ToArray();
                    pointBvh = new FaceBvh(points, points);
                }
                pointBvh.CollectBox(Vec3.MinNative(start, end), Vec3.MaxNative(start, end), junctionTolerance, nearby);
                cuts.Clear();
                foreach (var item in nearby)
                {
                    if (item == from || item == to)
                        continue;
                    var point = welder.Positions[item];
                    var t = Vec3.Dot(point - start, span) / spanLengthSquared;
                    if (t <= 0.0f || t >= 1.0f)
                        continue;
                    if ((point - (start + span * t)).LengthSquared() <= junctionToleranceSquared)
                        cuts.Add((t, item));
                }
                if (cuts.Count == 0)
                {
                    Register(face, from, to);
                    continue;
                }
                cuts.Sort(static (a, b) => a.T.CompareTo(b.T));
                var previous = from;
                foreach (var cut in cuts)
                {
                    if (cut.Id != previous)
                        Register(face, previous, cut.Id);
                    previous = cut.Id;
                }
                if (previous != to)
                    Register(face, previous, to);
            }
        }
        // Solids that touch along an edge pinch it: four or more faces with balanced winding still bound a
        // volume, so ray parity remains valid. Unbalanced or odd edges are cracks, tolerated up to a budget.
        var crackLength = 0.0;
        var totalLength = 0.0;
        foreach (var (key, edge) in edges)
        {
            var length = (welder.Positions[(int)(key >> 32)] - welder.Positions[(int)(uint)key]).Length();
            totalLength += length;
            if ((edge.Count & 1) != 0 || edge.Winding != 0)
                crackLength += length;
        }
        closed &= crackLength <= System.Math.Max(epsilon * CrackBudgetEpsilons, totalLength * CrackBudgetFraction);
        if (crackLength > 0.0)
        {
            _crackWelder = welder;
            _openEdges = [];
            foreach (var (key, edge) in edges)
            {
                if (edge.Winding == 0)
                    continue;
                // Winding counts edges running from the lower id to the higher one; fill in the other direction.
                var low = (int)(key >> 32);
                var high = (int)(uint)key;
                var (from, to) = edge.Winding > 0 ? (high, low) : (low, high);
                for (var i = 0; i < System.Math.Abs(edge.Winding); ++i)
                    _openEdges.Add((from, to, welder.Positions[from], welder.Positions[to], _faces[edge.Face].Shared));
            }
        }

        // Components in order of their first face.
        var componentIndices = new int[faceCount];
        Array.Fill(componentIndices, -1);
        faceComponents = new int[faceCount];
        var componentCount = 0;
        for (var face = 0; face < faceCount; ++face)
        {
            var root = Find(face);
            if (componentIndices[root] < 0)
                componentIndices[root] = componentCount++;
            faceComponents[face] = componentIndices[root];
        }
        componentStart = new int[componentCount + 1];
        foreach (var component in faceComponents)
            ++componentStart[component + 1];
        for (var i = 0; i < componentCount; ++i)
            componentStart[i + 1] += componentStart[i];
        componentFaces = new int[faceCount];
        faceRanks = new int[faceCount];
        var fill = componentStart[..^1];
        for (var face = 0; face < faceCount; ++face)
        {
            var slot = fill[faceComponents[face]]++;
            componentFaces[slot] = face;
            faceRanks[face] = slot - componentStart[faceComponents[face]];
        }

        // Adjacency rows keep the insertion order of shared-edge discovery.
        adjacencyStart = new int[faceCount + 1];
        foreach (var (face, other) in links)
        {
            ++adjacencyStart[face + 1];
            ++adjacencyStart[other + 1];
        }
        for (var i = 0; i < faceCount; ++i)
            adjacencyStart[i + 1] += adjacencyStart[i];
        adjacentFaces = new int[adjacencyStart[faceCount]];
        fill = adjacencyStart[..^1];
        foreach (var (face, other) in links)
        {
            adjacentFaces[fill[face]++] = other;
            adjacentFaces[fill[other]++] = face;
        }
        return closed;

        void Register(int face, int from, int to)
        {
            ref var state = ref CollectionsMarshal.GetValueRefOrAddDefault(edges, EdgeKey(from, to), out var exists);
            if (exists)
            {
                Union(face, state.Face);
                links.Add((face, state.Face));
            }
            else
                state.Face = face;
            ++state.Count;
            state.Winding += from < to ? 1 : -1;
        }

        int Find(int face)
        {
            while (parents[face] != face)
            {
                parents[face] = parents[parents[face]];
                face = parents[face];
            }
            return face;
        }

        void Union(int first, int second)
        {
            first = Find(first);
            second = Find(second);
            if (first == second)
                return;
            if (ranks[first] < ranks[second])
                (first, second) = (second, first);
            parents[second] = first;
            if (ranks[first] == ranks[second])
                ++ranks[first];
        }
    }

    /// <summary>
    /// Assigns one id to positions within epsilon of each other. The earliest representative wins, so the
    /// result depends only on input order. Cells are four epsilons wide, so neighbors are probed only near
    /// a cell boundary.
    /// </summary>
    private sealed class PointWelder
    {
        private readonly Dictionary<(long X, long Y, long Z), int> _cells;
        private readonly double _inverseCell;
        private readonly float _epsilon;
        private int[] _next;
        public Vec3[] Positions;
        public int Count;

        public PointWelder(float epsilon, int capacity)
        {
            _epsilon = epsilon;
            _inverseCell = 1.0 / (4.0 * epsilon);
            _cells = new Dictionary<(long, long, long), int>(capacity);
            Positions = new Vec3[System.Math.Max(capacity, 4)];
            _next = new int[Positions.Length];
        }

        public int Add(Vec3 point)
        {
            var found = Find(point);
            if (found >= 0)
                return found;
            if (Count == Positions.Length)
            {
                Array.Resize(ref Positions, Count * 2);
                Array.Resize(ref _next, Count * 2);
            }
            var index = Count++;
            Positions[index] = point;
            var key = ((long)global::System.Math.Floor(point.X * _inverseCell),
                (long)global::System.Math.Floor(point.Y * _inverseCell),
                (long)global::System.Math.Floor(point.Z * _inverseCell));
            ref var cell = ref CollectionsMarshal.GetValueRefOrAddDefault(_cells, key, out var exists);
            _next[index] = exists ? cell : -1;
            cell = index;
            return index;
        }

        /// <summary>Earliest representative within epsilon of the point, or -1.</summary>
        public int Find(Vec3 point)
        {
            var x = point.X * _inverseCell;
            var y = point.Y * _inverseCell;
            var z = point.Z * _inverseCell;
            var cx = (long)global::System.Math.Floor(x);
            var cy = (long)global::System.Math.Floor(y);
            var cz = (long)global::System.Math.Floor(z);
            var margin = _epsilon * _inverseCell;

            var best = int.MaxValue;
            var epsilonSquared = _epsilon * _epsilon;
            for (var ix = x - cx <= margin ? cx - 1 : cx; ix <= (cx + 1 - x <= margin ? cx + 1 : cx); ++ix)
            for (var iy = y - cy <= margin ? cy - 1 : cy; iy <= (cy + 1 - y <= margin ? cy + 1 : cy); ++iy)
            for (var iz = z - cz <= margin ? cz - 1 : cz; iz <= (cz + 1 - z <= margin ? cz + 1 : cz); ++iz)
            {
                if (!_cells.TryGetValue((ix, iy, iz), out var head))
                    continue;
                for (var id = head; id >= 0; id = _next[id])
                {
                    if (id < best && (Positions[id] - point).LengthSquared() <= epsilonSquared)
                        best = id;
                }
            }
            return best == int.MaxValue ? -1 : best;
        }
    }

    /// <summary>
    /// Binned-SAH bounding volume hierarchy over item boxes. Nodes live in one flat array; an inner node's
    /// children are stored as a pair at <see cref="Node.First"/> and <c>First + 1</c>.
    /// </summary>
    private sealed class FaceBvh
    {
        private const int MaxLeafSize = 4;
        private const int BinCount = 8;

        [StructLayout(LayoutKind.Sequential)]
        public struct Node
        {
            public Vec3 Min;
            public int First;
            public Vec3 Max;
            public int Count; // Zero for inner nodes.
        }

        public readonly Node[] Nodes;
        public readonly int[] Items;
        private readonly Vec3[] _itemMin;
        private readonly Vec3[] _itemMax;

        public FaceBvh(Vec3[] itemMin, Vec3[] itemMax) : this(itemMin, itemMax, null)
        {
        }

        /// <summary>Tree over a subset of items; <paramref name="subset"/> holds indices into the box arrays.</summary>
        public FaceBvh(Vec3[] itemMin, Vec3[] itemMax, ReadOnlySpan<int> subset) : this(itemMin, itemMax, subset.ToArray())
        {
        }

        private FaceBvh(Vec3[] itemMin, Vec3[] itemMax, int[]? subset)
        {
            _itemMin = itemMin;
            _itemMax = itemMax;
            var count = subset?.Length ?? itemMin.Length;
            Items = subset ?? new int[count];
            if (subset is null)
            {
                for (var i = 0; i < count; ++i)
                    Items[i] = i;
            }
            Nodes = new Node[System.Math.Max(count * 2, 1)];
            if (count == 0)
            {
                // An empty box that no query can hit.
                Nodes[0] = new Node { Min = new Vec3(float.PositiveInfinity), Max = new Vec3(float.NegativeInfinity) };
                return;
            }

            // Build over a contiguous copy of the item boxes that is partitioned in place, so every pass
            // reads memory sequentially instead of gathering through the index array.
            var primitives = new Primitive[count];
            for (var i = 0; i < count; ++i)
            {
                var item = Items[i];
                primitives[i] = new Primitive { Min = itemMin[item], Index = item, Max = itemMax[item] };
            }
            Build(primitives);
            for (var i = 0; i < count; ++i)
                Items[i] = primitives[i].Index;
        }

        [StructLayout(LayoutKind.Sequential)]
        private struct Primitive
        {
            public Vec3 Min;
            public int Index;
            public Vec3 Max;
            private int _padding;

            // Twice the centroid; binning is scale invariant.
            public readonly Vec3 Center => Min + Max;
        }

        private void Build(Primitive[] primitives)
        {
            var nodeCount = 1;
            Nodes[0].First = 0;
            Nodes[0].Count = primitives.Length;
            UpdateBounds(primitives, ref Nodes[0]);

            // Bins for all three axes, filled in one pass over the node's items.
            Span<Vector256<float>> bins = stackalloc Vector256<float>[3 * BinCount];
            Span<int> binCount = stackalloc int[3 * BinCount];
            Span<float> rightArea = stackalloc float[BinCount];
            Span<int> rightCount = stackalloc int[BinCount];
            Span<Vec3> rightMin = stackalloc Vec3[BinCount];
            Span<Vec3> rightMax = stackalloc Vec3[BinCount];
            var pending = new Stack<int>();
            pending.Push(0);
            while (pending.TryPop(out var nodeIndex))
            {
                ref var node = ref Nodes[nodeIndex];
                if (node.Count <= MaxLeafSize)
                    continue;
                var first = node.First;
                var end = first + node.Count;

                // Bin doubled centroids (min + max) over the doubled node box, as TinyBVH's AVX builder does;
                // this skips a pass that would compute tighter centroid bounds.
                var centroidMin = node.Min + node.Min;
                var extent = (node.Max - node.Min) * 2.0f;
                // A flat axis gets scale zero: every item falls in bin 0 and the axis offers no split.
                var scale = new Vec3(
                    extent.X > 0.0f ? BinCount / extent.X : 0.0f,
                    extent.Y > 0.0f ? BinCount / extent.Y : 0.0f,
                    extent.Z > 0.0f ? BinCount / extent.Z : 0.0f);

                FillBins(primitives.AsSpan(first, end - first), centroidMin, scale, bins, binCount);

                var bestAxis = -1;
                var bestSplit = 0;
                var bestCost = float.PositiveInfinity;
                Vec3 bestLeftMin = default, bestLeftMax = default, bestRightMin = default, bestRightMax = default;
                for (var axis = 0; axis < 3; ++axis)
                {
                    if (Component(scale, axis) == 0.0f)
                        continue;
                    var offset = axis * BinCount;
                    var sweepMin = new Vec3(float.PositiveInfinity);
                    var sweepMax = new Vec3(float.NegativeInfinity);
                    var sweepCount = 0;
                    for (var bin = BinCount - 1; bin > 0; --bin)
                    {
                        sweepMin = Vec3.MinNative(sweepMin, -bins[offset + bin].GetLower().AsVector3());
                        sweepMax = Vec3.MaxNative(sweepMax, bins[offset + bin].GetUpper().AsVector3());
                        sweepCount += binCount[offset + bin];
                        rightMin[bin] = sweepMin;
                        rightMax[bin] = sweepMax;
                        rightArea[bin] = sweepCount == 0 ? 0.0f : HalfArea(sweepMin, sweepMax);
                        rightCount[bin] = sweepCount;
                    }
                    sweepMin = new Vec3(float.PositiveInfinity);
                    sweepMax = new Vec3(float.NegativeInfinity);
                    sweepCount = 0;
                    for (var bin = 0; bin < BinCount - 1; ++bin)
                    {
                        sweepMin = Vec3.MinNative(sweepMin, -bins[offset + bin].GetLower().AsVector3());
                        sweepMax = Vec3.MaxNative(sweepMax, bins[offset + bin].GetUpper().AsVector3());
                        sweepCount += binCount[offset + bin];
                        if (sweepCount == 0 || rightCount[bin + 1] == 0)
                            continue;
                        var cost = sweepCount * HalfArea(sweepMin, sweepMax) + rightCount[bin + 1] * rightArea[bin + 1];
                        if (cost < bestCost)
                        {
                            bestCost = cost;
                            bestAxis = axis;
                            bestSplit = bin + 1;
                            bestLeftMin = sweepMin;
                            bestLeftMax = sweepMax;
                            bestRightMin = rightMin[bin + 1];
                            bestRightMax = rightMax[bin + 1];
                        }
                    }
                }

                int middle;
                if (bestAxis < 0)
                {
                    // Coincident centroids: split by position in the item range to bound leaf size.
                    middle = first + node.Count / 2;
                }
                else
                {
                    middle = first + Partition(primitives.AsSpan(first, end - first), bestAxis,
                        Component(centroidMin, bestAxis), Component(scale, bestAxis), bestSplit);
                }

                var left = nodeCount;
                nodeCount += 2;
                Nodes[left] = new Node { First = first, Count = middle - first };
                Nodes[left + 1] = new Node { First = middle, Count = end - middle };
                if (bestAxis < 0)
                {
                    UpdateBounds(primitives, ref Nodes[left]);
                    UpdateBounds(primitives, ref Nodes[left + 1]);
                }
                else
                {
                    Nodes[left].Min = bestLeftMin;
                    Nodes[left].Max = bestLeftMax;
                    Nodes[left + 1].Min = bestRightMin;
                    Nodes[left + 1].Max = bestRightMax;
                }
                node.First = left;
                node.Count = 0;
                pending.Push(left + 1);
                pending.Push(left);
            }
        }

        /// <remarks>
        /// Each bin is one 256-bit box holding (-min, max), as in TinyBVH's AVX builder, so growing a bin is a
        /// single <see cref="Vector256.MaxNative{T}"/>. Primitives share the layout: their lower half is min with the
        /// item index in lane 3, which a sign flip turns into -min; lanes 3 and 7 are ignored.
        /// </remarks>
        private static void FillBins(ReadOnlySpan<Primitive> primitives, Vec3 centroidMin, Vec3 scale,
            Span<Vector256<float>> bins, Span<int> binCount)
        {
            bins.Fill(Vector256.Create(float.NegativeInfinity));
            binCount.Clear();
            var negateMin = Vector256.Create(-0.0f, -0.0f, -0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            var origin = centroidMin.AsVector128();
            var scale4 = scale.AsVector128();
            var lastBin = Vector128.Create(BinCount - 1);
            var offsets = Vector128.Create(0, BinCount, 2 * BinCount, 0);
            ref var bin0 = ref MemoryMarshal.GetReference(bins);
            ref var count0 = ref MemoryMarshal.GetReference(binCount);
            ref var item = ref Unsafe.As<Primitive, Vector256<float>>(ref MemoryMarshal.GetReference(primitives));

            // Software pipelined like TinyBVH's AVX binning: the next box and its bins are computed while the
            // current box updates its three bins, hiding the load-max-store latency of repeated bins.
            var box = item;
            var bin = BinOf(box);
            for (var i = 1; i < primitives.Length; ++i)
            {
                var nextBox = Unsafe.Add(ref item, i);
                var nextBin = BinOf(nextBox);
                Grow(ref bin0, ref count0, bin.GetElement(0), box ^ negateMin);
                Grow(ref bin0, ref count0, bin.GetElement(1), box ^ negateMin);
                Grow(ref bin0, ref count0, bin.GetElement(2), box ^ negateMin);
                box = nextBox;
                bin = nextBin;
            }
            Grow(ref bin0, ref count0, bin.GetElement(0), box ^ negateMin);
            Grow(ref bin0, ref count0, bin.GetElement(1), box ^ negateMin);
            Grow(ref bin0, ref count0, bin.GetElement(2), box ^ negateMin);

            // Clamped as integers: rounding can leave a centroid just below the origin, and a NaN box converts
            // to int.MinValue; either would index outside the bins.
            Vector128<int> BinOf(Vector256<float> value) =>
                Vector128.Clamp(Vector128.ConvertToInt32((value.GetLower() + value.GetUpper() - origin) * scale4),
                    Vector128<int>.Zero, lastBin) + offsets;

            [MethodImpl(MethodImplOptions.AggressiveInlining)]
            static void Grow(ref Vector256<float> bins, ref int counts, int index, Vector256<float> value)
            {
                ++Unsafe.Add(ref counts, index);
                ref var target = ref Unsafe.Add(ref bins, index);
                target = Vector256.MaxNative(target, value);
            }
        }

        /// <summary>Move primitives in bins below <paramref name="split"/> to the front; return their count.</summary>
        private static int Partition(Span<Primitive> primitives, int axis, float low, float scale, int split)
        {
            var middle = 0;
            var last = primitives.Length - 1;
            while (middle <= last)
            {
                var bin = System.Math.Min((int)((Component(primitives[middle].Center, axis) - low) * scale), BinCount - 1);
                if (bin < split)
                    ++middle;
                else
                {
                    (primitives[middle], primitives[last]) = (primitives[last], primitives[middle]);
                    --last;
                }
            }
            return middle;
        }

        private static void UpdateBounds(Primitive[] primitives, ref Node node)
        {
            var min = new Vec3(float.PositiveInfinity);
            var max = new Vec3(float.NegativeInfinity);
            for (var i = node.First; i < node.First + node.Count; ++i)
            {
                min = Vec3.MinNative(min, primitives[i].Min);
                max = Vec3.MaxNative(max, primitives[i].Max);
            }
            node.Min = min;
            node.Max = max;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float Component(Vec3 value, int axis) => axis == 0 ? value.X : axis == 1 ? value.Y : value.Z;

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static float HalfArea(Vec3 min, Vec3 max)
        {
            var extent = max - min;
            return extent.X * extent.Y + extent.Y * extent.Z + extent.Z * extent.X;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        private static bool Overlaps(Vec3 minA, Vec3 maxA, Vec3 minB, Vec3 maxB, Vec3 epsilon) =>
            Vector128.LessThanOrEqualAll(minA.AsVector128(), (maxB + epsilon).AsVector128())
            && Vector128.GreaterThanOrEqualAll((maxA + epsilon).AsVector128(), minB.AsVector128());

        /// <summary>Collect every item pair whose boxes overlap within epsilon.</summary>
        public void CollectOverlaps(FaceBvh other, float epsilon, List<(int A, int B)> output)
        {
            if (Items.Length == 0 || other.Items.Length == 0)
                return;
            var tolerance = new Vec3(epsilon);
            var stack = new Stack<(int A, int B)>();
            stack.Push((0, 0));
            while (stack.TryPop(out var pair))
            {
                ref readonly var a = ref Nodes[pair.A];
                ref readonly var b = ref other.Nodes[pair.B];
                if (!Overlaps(a.Min, a.Max, b.Min, b.Max, tolerance))
                    continue;
                if (a.Count != 0 && b.Count != 0)
                {
                    for (var i = a.First; i < a.First + a.Count; ++i)
                    {
                        var itemA = Items[i];
                        var minA = _itemMin[itemA];
                        var maxA = _itemMax[itemA];
                        for (var j = b.First; j < b.First + b.Count; ++j)
                        {
                            var itemB = other.Items[j];
                            if (Overlaps(minA, maxA, other._itemMin[itemB], other._itemMax[itemB], tolerance))
                                output.Add((itemA, itemB));
                        }
                    }
                }
                else if (b.Count != 0 || a.Count == 0 && HalfArea(a.Min, a.Max) >= HalfArea(b.Min, b.Max))
                {
                    stack.Push((a.First + 1, pair.B));
                    stack.Push((a.First, pair.B));
                }
                else
                {
                    stack.Push((pair.A, b.First + 1));
                    stack.Push((pair.A, b.First));
                }
            }
        }

        /// <summary>Visit items whose boxes overlap a query box within epsilon.</summary>
        public void CollectBox(Vec3 min, Vec3 max, float epsilon, List<int> output)
        {
            output.Clear();
            if (Items.Length == 0)
                return;
            var tolerance = new Vec3(epsilon);
            Span<int> stack = stackalloc int[64];
            var top = 0;
            stack[top++] = 0;
            while (top != 0)
            {
                ref readonly var node = ref Nodes[stack[--top]];
                if (!Overlaps(node.Min, node.Max, min, max, tolerance))
                    continue;
                if (node.Count == 0)
                {
                    if (top + 2 > stack.Length)
                    {
                        var larger = new int[stack.Length * 2];
                        stack.CopyTo(larger);
                        stack = larger;
                    }
                    stack[top++] = node.First + 1;
                    stack[top++] = node.First;
                    continue;
                }
                for (var i = node.First; i < node.First + node.Count; ++i)
                {
                    var item = Items[i];
                    if (Overlaps(_itemMin[item], _itemMax[item], min, max, tolerance))
                        output.Add(item);
                }
            }
        }
    }
}
