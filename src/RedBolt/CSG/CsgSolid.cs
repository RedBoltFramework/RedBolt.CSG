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
public sealed partial class CsgSolid
{
    /// <summary>
    /// Length of unmatched edges that an operand may have and still count as closed: the larger of an absolute
    /// budget in epsilons and a fraction of the solid's total edge length. Modeled meshes can carry hairline cracks
    /// that ray-parity classification tolerates. Boolean results match every edge by construction.
    /// </summary>
    private const float CrackBudgetEpsilons = 200.0f;
    private const float CrackBudgetFraction = 0.001f;

    /// <summary>Distance, in epsilons, within which a vertex lying on another face's edge forms a T-junction.</summary>
    private const float JunctionScale = 8.0f;

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
    private readonly int[] _faceComponents;
    // Faces whose stored triangulation needed the ear clipper's fallback; they are triangulated again for output.
    private readonly HashSet<int>? _fallbackFaces;
    // Unmatched edges as welded position pairs; null when every edge is matched.
    private List<(Vec3 From, Vec3 To)>? _openEdges;
    // Whether a face edge ends at a T-junction, so the faces do not share vertices exactly.
    private bool _hasJunctions;
    // Face loops welded within epsilon while building the topology.
    private WeldedLoops? _weld;
    // Face loops welded by identical position.
    private WeldedLoops? _exactWeld;

    private WeldedLoops ExactWeld => _exactWeld ??= new WeldedLoops(this, 0.0f);
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

    /// <summary>Total length of unmatched edges.</summary>
    internal double OpenEdgeLength
    {
        get
        {
            var total = 0.0;
            foreach (var (from, to) in _openEdges ?? [])
                total += (to - from).Length();
            return total;
        }
    }

    private FaceBvh ComponentTree => _componentTree ??= new FaceBvh(_componentMin, _componentMax);

    private FaceBvh FaceTree(int component) => _faceTrees[component] ??= new FaceBvh(_faceMin, _faceMax,
        _componentFaces.AsSpan(_componentStart[component], _componentStart[component + 1] - _componentStart[component]));

    /// <summary>
    /// A face tree of a component that holds every face near a box. A boolean result is a fresh solid, and a small
    /// brush reaches few of its faces, so a tree over just those replaces building one over the whole component;
    /// the full tree is built and kept once most faces are near or it exists already.
    /// </summary>
    private FaceBvh NearFaces(int component, Vec3 min, Vec3 max, float epsilon)
    {
        if (_faceTrees[component] is { } cached)
            return cached;
        var faces = _componentFaces.AsSpan(_componentStart[component],
            _componentStart[component + 1] - _componentStart[component]);
        var low = (min - new Vec3(epsilon)).AsVector128();
        var high = (max + new Vec3(epsilon)).AsVector128();
        var near = new List<int>();
        foreach (var face in faces)
        {
            if (Vector128.LessThanOrEqualAll(_faceMin[face].AsVector128(), high)
                && Vector128.GreaterThanOrEqualAll(_faceMax[face].AsVector128(), low))
                near.Add(face);
        }
        return near.Count * 2 > faces.Length
            ? FaceTree(component)
            : new FaceBvh(_faceMin, _faceMax, CollectionsMarshal.AsSpan(near));
    }

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

    /// <summary>Build an operand from copies of the polygons.</summary>
    public CsgSolid(IReadOnlyList<CsgPolygon> polygons, float epsilon = CsgCommon.DefaultEpsilon)
        : this(FromOperand(polygons.Select(polygon => polygon.Clone()).ToList(), epsilon)._faces, epsilon, false)
    {
    }

    /// <summary>Build an operand over the polygons.</summary>
    /// <remarks>
    /// Vertices welded within epsilon take one position, and a face whose edge ends at another face's vertex gains
    /// that vertex, so neighboring faces share every edge vertex exactly. Boolean results already do, and are used
    /// as operands directly.
    /// </remarks>
    internal static CsgSolid FromOperand(List<CsgPolygon> polygons, float epsilon)
    {
        var solid = new CsgSolid(polygons, epsilon, false);
        // Corefinement identifies vertices by position and by the edges they lie on, so faces must share their
        // edge vertices at identical positions.
        return solid._hasJunctions || !solid._weld!.Canonical
            ? new CsgSolid(solid.WatertightFaces(epsilon), epsilon, false)
            : solid;
    }


    // Boolean results own newly split faces and may share untouched faces with immutable operands.
    /// <summary>Source of a face that a boolean keeps whole, so its triangles can be copied instead of rebuilt.</summary>
    internal readonly record struct FaceOrigin(CsgSolid Solid, int Face, bool Flip);

    /// <param name="exactIds">
    /// Optional id of every face vertex, in face order, as a boolean assigned them: vertices with one id share
    /// their position exactly, so the solid need not weld positions to find them.
    /// </param>
    internal CsgSolid(IReadOnlyList<CsgPolygon> polygons, float epsilon, bool copyFaces,
        IReadOnlyList<FaceOrigin?>? origins = null, int[]? exactIds = null, int exactIdSpace = 0)
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

        // Two planar faces can share a run of several edges only along the line where their planes meet, so a
        // vertex whose edges both lead to the same neighbor is straight up to rounding. If both faces clipped it as
        // an ear, both would claim the sliver under the run and share a diagonal. Both faces of such a pair have
        // more than three vertices, so only those enter the map.
        Dictionary<(Vec3, Vec3), int>? edgeOwner = null;
        for (var i = 0; i < faceCount; ++i)
        {
            var vertices = polygons[i].Vertices;
            if (vertices.Count <= 3 || origins?[i] is not null)
                continue;
            edgeOwner ??= [];
            for (var k = 0; k < vertices.Count; ++k)
                edgeOwner[(Position(vertices[k]), Position(vertices[(k + 1) % vertices.Count]))] = i;
        }
        var straight = new List<bool>();

        var scratch = new List<uint>();
        var triangleCount = 0;
        HashSet<int>? fallbackFaces = null;
        for (var i = 0; i < faceCount; ++i)
        {
            var face = copyFaces ? polygons[i].Clone() : polygons[i];
            _faces[i] = face;
            _triangleStart[i] = triangleCount;
            var vertices = CollectionsMarshal.AsSpan(face.Vertices);

            if (origins?[i] is { } origin)
            {
                CopyOrigin(i, origin);
                continue;
            }

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
            else if (Csg.TriangulatePolygon(face, scratch, SharedRuns(i, vertices), out var clean))
            {
                if (!clean)
                    (fallbackFaces ??= []).Add(i);
                for (var t = 0; t + 2 < scratch.Count; t += 3)
                    AppendTriangle(vertices, (int)scratch[t], (int)scratch[t + 1], (int)scratch[t + 2]);
            }
        }
        _triangleStart[faceCount] = triangleCount;
        _fallbackFaces = fallbackFaces;
        _trianglePositions = positions.Length == triangleCount * 3 ? positions : positions[..(triangleCount * 3)];
        _triangleVertices = locals.Length == triangleCount * 3 ? locals : locals[..(triangleCount * 3)];

        IsClosed = BuildTopology(epsilon, exactIds, exactIdSpace, out _componentStart, out _componentFaces,
            out _faceComponents);
        (_boundsMin, _boundsMax) = MergeBounds(_faceMin, _faceMax);
        ComponentBounds(_componentStart, _componentFaces, _faceMin, _faceMax, out _componentMin, out _componentMax);
        _faceTrees = new FaceBvh?[ComponentCount];

        List<bool>? SharedRuns(int face, ReadOnlySpan<ModelVertex> vertices)
        {
            straight.Clear();
            var any = false;
            for (var k = 0; k < vertices.Length; ++k)
            {
                var previous = Position(vertices[(k + vertices.Length - 1) % vertices.Length]);
                var current = Position(vertices[k]);
                var next = Position(vertices[(k + 1) % vertices.Length]);
                // Only the later face of the pair refrains, which is enough to keep the sliver from being claimed
                // twice and leaves the earlier face free to triangulate as it must.
                var interior = edgeOwner is not null && edgeOwner.TryGetValue((current, previous), out var before)
                    && edgeOwner.TryGetValue((next, current), out var after) && before == after && before < face;
                straight.Add(interior);
                any |= interior;
            }
            return any ? straight : null;
        }

        // The face is the source face or its flipped copy; bounds and triangles carry over, reversed for a flip.
        void CopyOrigin(int index, FaceOrigin origin)
        {
            var source = origin.Solid;
            _faceMin[index] = source._faceMin[origin.Face];
            _faceMax[index] = source._faceMax[origin.Face];
            if (source._fallbackFaces?.Contains(origin.Face) == true)
                (fallbackFaces ??= []).Add(index);
            var first = source._triangleStart[origin.Face];
            var count = source._triangleStart[origin.Face + 1] - first;
            if (triangleCount * 3 + count * 3 > positions.Length)
            {
                var capacity = System.Math.Max(positions.Length * 2, (triangleCount + count) * 3);
                Array.Resize(ref positions, capacity);
                Array.Resize(ref locals, capacity);
            }
            var sourcePositions = source._trianglePositions.AsSpan(first * 3, count * 3);
            var sourceLocals = source._triangleVertices.AsSpan(first * 3, count * 3);
            var offset = triangleCount * 3;
            if (origin.Flip)
            {
                var vertexCount = source._faces[origin.Face].Vertices.Count;
                for (var t = 0; t < count; ++t)
                {
                    for (var corner = 0; corner < 3; ++corner)
                    {
                        positions[offset + t * 3 + corner] = sourcePositions[t * 3 + 2 - corner];
                        locals[offset + t * 3 + corner] = vertexCount - 1 - sourceLocals[t * 3 + 2 - corner];
                    }
                }
            }
            else
            {
                sourcePositions.CopyTo(positions.AsSpan(offset));
                sourceLocals.CopyTo(locals.AsSpan(offset));
            }
            triangleCount += count;
        }

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
        _fallbackFaces = builder.FallbackFaces.Count != 0 ? builder.FallbackFaces : null;
        _exactWeld = new WeldedLoops(this, builder.ExactIds.ToArray(), builder.ExactIdSpace);
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
        public readonly HashSet<int> FallbackFaces = [];
        // Exact vertex ids per face vertex; each source's ids are offset so that sources never share an id.
        public readonly List<int> ExactIds;
        public int ExactIdSpace;
        private readonly Dictionary<CsgSolid, int> _idOffsets = [];
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
            ExactIds = new List<int>(cornerCapacity);
        }

        private int IdOffset(CsgSolid source)
        {
            ref var offset = ref CollectionsMarshal.GetValueRefOrAddDefault(_idOffsets, source, out var known);
            if (!known)
            {
                offset = ExactIdSpace;
                ExactIdSpace += source.ExactWeld.Welder.Count;
            }
            return offset;
        }

        public void AppendComponent(CsgSolid source, int component, bool flip)
        {
            var componentIndex = ComponentStart.Count - 1;
            for (var c = source._componentStart[component]; c < source._componentStart[component + 1]; ++c)
            {
                ComponentFaces.Add(Faces.Count);
                AppendFace(source, source._componentFaces[c], flip, componentIndex);
            }
            ComponentStart.Add(ComponentFaces.Count);
        }

        /// <summary>Append every face of a solid in its own order, keeping its components.</summary>
        public void AppendSolid(CsgSolid source)
        {
            var faceBase = Faces.Count;
            var componentBase = ComponentStart.Count - 1;
            for (var face = 0; face < source._faces.Length; ++face)
                AppendFace(source, face, false, componentBase + source._faceComponents[face]);
            for (var component = 0; component < source.ComponentCount; ++component)
            {
                for (var c = source._componentStart[component]; c < source._componentStart[component + 1]; ++c)
                    ComponentFaces.Add(faceBase + source._componentFaces[c]);
                ComponentStart.Add(ComponentFaces.Count);
            }
            Closed &= source.IsClosed;
        }

        private void AppendFace(CsgSolid source, int face, bool flip, int component)
        {
            if (source._fallbackFaces?.Contains(face) == true)
                FallbackFaces.Add(Faces.Count);
            Faces.Add(flip ? Flipped(source._faces[face]) : source._faces[face]);
            FaceComponents.Add(component);
            var offset = IdOffset(source);
            var weld = source.ExactWeld;
            var idStart = weld.VertexStart[face];
            var idCount = weld.VertexStart[face + 1] - idStart;
            if (flip)
            {
                for (var i = idCount - 1; i >= 0; --i)
                    ExactIds.Add(offset + weld.Ids[idStart + i]);
            }
            else
            {
                for (var i = 0; i < idCount; ++i)
                    ExactIds.Add(offset + weld.Ids[idStart + i]);
            }
            FaceMin.Add(source._faceMin[face]);
            FaceMax.Add(source._faceMax[face]);

            var vertexCount = source._faces[face].Vertices.Count;
            var firstTriangle = source._triangleStart[face];
            var triangleCount = source._triangleStart[face + 1] - firstTriangle;
            var positions = source._trianglePositions.AsSpan(firstTriangle * 3, triangleCount * 3);
            var locals = source._triangleVertices.AsSpan(firstTriangle * 3, triangleCount * 3);
            if (flip)
            {
                // Flipping reverses the vertex list: keep the same corners under their new indices, and reverse
                // each triangle so it winds with the flipped face.
                for (var t = 0; t < triangleCount; ++t)
                {
                    for (var corner = 2; corner >= 0; --corner)
                    {
                        Positions.Add(positions[t * 3 + corner]);
                        Locals.Add(vertexCount - 1 - locals[t * 3 + corner]);
                    }
                }
            }
            else
            {
                Positions.AddRange(positions);
                Locals.AddRange(locals);
            }
            TriangleStart.Add(TriangleStart[^1] + triangleCount);
        }
    }

    /// <summary>Id of a face vertex's position; vertices with one id coincide exactly.</summary>
    internal int VertexKey(int face, int vertex) => ExactWeld.Ids[ExactWeld.VertexStart[face] + vertex];

    /// <summary>Number of distinct vertex positions, the bound of <see cref="VertexKey"/>.</summary>
    internal int VertexKeySpace => ExactWeld.Welder.Count;

    /// <summary>Triangles of a face as face-local vertex indices.</summary>
    /// <remarks>
    /// A face whose stored triangles needed the fallback is triangulated afresh without the shared-run constraint,
    /// which exists for the boolean's benefit and can leave a flat or inverted triangle for rendering.
    /// </remarks>
    internal bool CopyTriangles(int face, List<uint> output)
    {
        if (_fallbackFaces?.Contains(face) == true)
            return Csg.TriangulatePolygon(_faces[face], output);
        output.Clear();
        for (var i = _triangleStart[face] * 3; i < _triangleStart[face + 1] * 3; ++i)
            output.Add((uint)_triangleVertices[i]);
        return output.Count != 0;
    }

    /// <summary>Faces owned by this solid. Callers must not modify them or their vertex attribute arrays.</summary>
    internal IReadOnlyList<CsgPolygon> Faces => _faces;

    private static CsgPolygon Flipped(CsgPolygon face)
    {
        var result = face.CloneShared();
        result.Flip();
        return result;
    }

    private static Vec3 ToVec3(Vector3f value) => new(value.X, value.Y, value.Z);

    private static Vector3f ToVector3f(Vec3 value) => new(value.X, value.Y, value.Z);

    private Vector3f FaceSample(int faceIndex)
    {
        var offset = _triangleStart[faceIndex] * 3;
        var centroid = (_trianglePositions[offset] + _trianglePositions[offset + 1]
            + _trianglePositions[offset + 2]) / 3.0f;
        return new Vector3f(centroid.X, centroid.Y, centroid.Z);
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

    /// <summary>
    /// Classify a point with three rays. The answer is reliable when the rays agree and none passes within a few
    /// epsilons of the point, which would put the point on the surface.
    /// </summary>
    internal bool ClassifyPoint(Vector3f point, float epsilon, out bool reliable)
    {
        reliable = true;
        if (_faces.Length == 0)
            return false;
        var origin = new Vec3(point.X, point.Y, point.Z);
        if (!BoundsOverlap(origin, origin, epsilon * 4.0f))
            return false;
        var insideVotes = 0;
        foreach (var direction in RayDirections)
        {
            if ((CountCrossings(origin, direction, epsilon, out var nearest) & 1) != 0)
                ++insideVotes;
            reliable &= nearest > epsilon * 4.0f;
        }
        reliable &= insideVotes is 0 or 3;
        return insideVotes >= 2;
    }

    private int CountCrossings(Vec3 origin, Vec3 direction, float epsilon) =>
        CountCrossings(origin, direction, epsilon, out _);

    private int CountCrossings(Vec3 origin, Vec3 direction, float epsilon, out float nearest)
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
        nearest = hitCount != 0 ? sorted[0] : float.PositiveInfinity;
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

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static Vec3 Position(in ModelVertex vertex) =>
        new(vertex.Position.X, vertex.Position.Y, vertex.Position.Z);

    private struct EdgeState
    {
        public int Face;
        public int Count;
        public int Winding;
    }

    /// <summary>Whether a face with triangles welds below three ids.</summary>
    private bool HasCollapsedFace(WeldedLoops loops)
    {
        for (var face = 0; face < _faces.Length; ++face)
        {
            if (loops.LoopLength[face] < 3 && _triangleStart[face] != _triangleStart[face + 1])
                return true;
        }
        return false;
    }

    private static ulong EdgeKey(int from, int to) =>
        from < to ? (ulong)(uint)from << 32 | (uint)to : (ulong)(uint)to << 32 | (uint)from;

    /// <summary>Number components in order of their first face and list each component's faces.</summary>
    private static void GroupComponents(int faceCount, Func<int, int> root, out int[] componentStart,
        out int[] componentFaces, out int[] faceComponents)
    {
        var componentIndices = new int[faceCount];
        Array.Fill(componentIndices, -1);
        faceComponents = new int[faceCount];
        var componentCount = 0;
        for (var face = 0; face < faceCount; ++face)
        {
            var r = root(face);
            if (componentIndices[r] < 0)
                componentIndices[r] = componentCount++;
            faceComponents[face] = componentIndices[r];
        }
        componentStart = new int[componentCount + 1];
        foreach (var component in faceComponents)
            ++componentStart[component + 1];
        for (var i = 0; i < componentCount; ++i)
            componentStart[i + 1] += componentStart[i];
        componentFaces = new int[faceCount];
        var fill = componentStart[..^1];
        for (var face = 0; face < faceCount; ++face)
            componentFaces[fill[faceComponents[face]]++] = face;
    }

    private bool BuildTopology(float epsilon, int[]? exactIds, int exactIdSpace, out int[] componentStart,
        out int[] componentFaces, out int[] faceComponents)
    {
        var faceCount = _faces.Length;
        // An empty solid bounds nothing and is trivially closed.
        var closed = true;

        // Vertices are welded exactly first; that welding also serves booleans, which identify shared vertices
        // by position. The loops within epsilon follow from the distinct positions.
        var exact = _exactWeld ??= exactIds is null
            ? new WeldedLoops(this, 0.0f)
            : new WeldedLoops(this, exactIds, exactIdSpace);
        // Loops that close up exactly, as boolean results and most modeled meshes do, are closed within epsilon
        // as well and need no second welding; only float noise between neighboring faces calls for it.
        var loops = exact.IsBalanced() ? exact : new WeldedLoops(this, exact, epsilon);
        _weld = loops;
        for (var face = 0; face < faceCount; ++face)
        {
            if (_faces[face].Vertices.Count < 3 || _triangleStart[face] == _triangleStart[face + 1])
                closed = false;
        }
        var parents = new int[faceCount];
        var ranks = new byte[faceCount];
        for (var i = 0; i < faceCount; ++i)
            parents[i] = i;

        // Balanced loops match every edge exactly: nothing can end at a T-junction or leave a crack, and the
        // faces each edge joins are known already.
        if (loops == exact && !HasCollapsedFace(loops))
        {
            foreach (var (face, first) in loops.Links)
                Union(face, first);
            GroupComponents(faceCount, Find, out componentStart, out componentFaces, out faceComponents);
            return closed;
        }

        var welder = loops.Welder;
        var vertexStart = loops.VertexStart;
        var vertexIds = loops.VertexIds;
        var loopLength = loops.LoopLength;

        // Only edges without an exact opposite partner can end at a T-junction. Split those at welded
        // points on their interior so a long edge matches the shorter edges along it.
        var edges = new Dictionary<ulong, EdgeState>(loops.RawEdges.Count, CsgCommon.PackedKeyComparer.Instance);
        var cuts = new List<(float T, int Id)>();
        for (var face = 0; face < faceCount; ++face)
        {
            var count = loopLength[face];
            if (count < 3)
                continue;
            for (var i = 0; i < count; ++i)
            {
                var from = vertexIds[vertexStart[face] + i];
                var to = vertexIds[vertexStart[face] + (i + 1) % count];
                if (loops.IsMatched(from, to))
                {
                    Register(face, from, to);
                    continue;
                }
                loops.CollectJunctions(from, to, cuts);
                _hasJunctions |= cuts.Count != 0;
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
        // A face smaller than epsilon welds below three ids and takes no part in the edge graph; it belongs to the
        // component of the faces that share its vertices.
        if (HasCollapsedFace(loops))
        {
            var vertexOwner = new Dictionary<int, int>();
            for (var face = 0; face < faceCount; ++face)
            {
                if (loopLength[face] < 3)
                    continue;
                for (var i = 0; i < loopLength[face]; ++i)
                    vertexOwner.TryAdd(vertexIds[vertexStart[face] + i], face);
            }
            for (var face = 0; face < faceCount; ++face)
            {
                if (loopLength[face] >= 3)
                    continue;
                for (var i = vertexStart[face]; i < vertexStart[face + 1]; ++i)
                {
                    if (vertexOwner.TryGetValue(loops.Ids[i], out var owner))
                    {
                        Union(face, owner);
                        break;
                    }
                }
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
            _openEdges = [];
            foreach (var (key, edge) in edges)
            {
                if ((edge.Count & 1) != 0 || edge.Winding != 0)
                    _openEdges.Add((welder.Positions[(int)(key >> 32)], welder.Positions[(int)(uint)key]));
            }
        }

        GroupComponents(faceCount, Find, out componentStart, out componentFaces, out faceComponents);
        return closed;

        void Register(int face, int from, int to)
        {
            ref var state = ref CollectionsMarshal.GetValueRefOrAddDefault(edges, EdgeKey(from, to), out var exists);
            if (exists)
                Union(face, state.Face);
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
    /// Faces prepared for rendering without cracks. Corners move to the welded position they share with their
    /// neighbors, and an edge that ends at a T-junction gains the vertices of the shorter edges along it, so
    /// adjacent triangles meet at identical vertices. Needles are dropped; unchanged faces are returned as they are.
    /// </summary>
    internal List<CsgPolygon> WatertightFaces(float epsilon)
    {
        if (epsilon <= 0.0f)
            epsilon = CsgCommon.DefaultEpsilon;
        var loops = new WeldedLoops(this, epsilon);
        var result = new List<CsgPolygon>(_faces.Length);
        var cuts = new List<(float T, int Id)>();
        var buffer = new List<ModelVertex>();
        for (var face = 0; face < _faces.Length; ++face)
        {
            var count = loops.LoopLength[face];
            if (count < 3)
                continue;
            var source = _faces[face];
            var start = loops.VertexStart[face];
            var changed = count != source.Vertices.Count;
            buffer.Clear();
            for (var i = 0; i < count; ++i)
            {
                var from = loops.VertexIds[start + i];
                var corner = source.Vertices[loops.Corners[start + i]];
                var position = ToVector3f(loops.Welder.Positions[from]);
                if (corner.Position.ToVector3() != position)
                {
                    corner.SetPosition(position);
                    changed = true;
                }
                buffer.Add(corner);

                var to = loops.VertexIds[start + (i + 1) % count];
                if (loops.IsMatched(from, to))
                    continue;
                loops.CollectJunctions(from, to, cuts);
                if (cuts.Count == 0)
                    continue;
                var next = source.Vertices[loops.Corners[start + (i + 1) % count]];
                foreach (var (t, id) in cuts)
                {
                    var vertex = CsgCommon.LerpModelVertex(corner, next, t);
                    vertex.SetPosition(ToVector3f(loops.Welder.Positions[id]));
                    buffer.Add(vertex);
                }
                changed = true;
            }
            result.Add(changed ? new CsgPolygon([.. buffer], source.Shared) { Plane = source.Plane } : source);
        }
        return result;
    }

    /// <summary>
    /// Face corners welded within epsilon into loops of vertex ids, with raw edge counts between the ids. Welding
    /// keeps float noise on either side of a rounding boundary from splitting one shared vertex into two ids.
    /// Repeated ids are dropped; a loop that collapses below three ids, or a face without triangles, is a
    /// needle that bounds no surface and gets a loop length of zero.
    /// </summary>
    private sealed class WeldedLoops
    {
        public readonly float Epsilon;
        public readonly PointWelder Welder;
        public readonly int[] VertexStart;
        /// <summary>Welded id of every face vertex, in the layout of <see cref="VertexStart"/>.</summary>
        public readonly int[] Ids;
        /// <summary>Loop ids of every face, in the same layout, with repeated ids dropped.</summary>
        public readonly int[] VertexIds;
        /// <summary>Index in its face of the vertex each loop id was taken from.</summary>
        public readonly int[] Corners;
        public readonly int[] LoopLength;
        private Dictionary<ulong, EdgeState>? _rawEdges;
        private readonly CsgSolid _solid;

        /// <summary>Directed edge counts between loop ids, built on first use.</summary>
        public Dictionary<ulong, EdgeState> RawEdges => _rawEdges ??= BuildRawEdges();
        /// <summary>
        /// Each face edge after the first on its welded edge, paired with that edge's first face: the faces an
        /// edge joins, once <see cref="RawEdges"/> is built.
        /// </summary>
        public readonly List<(int Face, int First)> Links = [];
        /// <summary>Whether every vertex already sits exactly at the position of its welded id.</summary>
        public readonly bool Canonical = true;
        private readonly float _junctionTolerance;
        private readonly List<int> _nearby = [];
        private FaceBvh? _points;

        /// <summary>Loops from ids a boolean assigned: vertices with one id share their position exactly.</summary>
        public WeldedLoops(CsgSolid solid, int[] ids, int idSpace)
        {
            _solid = solid;
            var faces = solid._faces;
            VertexStart = new int[faces.Length + 1];
            for (var face = 0; face < faces.Length; ++face)
                VertexStart[face + 1] = VertexStart[face] + faces[face].Vertices.Count;
            Epsilon = 0.0f;
            VertexIds = new int[VertexStart[faces.Length]];
            Ids = new int[VertexIds.Length];
            Corners = new int[VertexIds.Length];
            LoopLength = new int[faces.Length];
            _junctionTolerance = 0.0f;
            // Ids are renumbered densely in order of first use, which fixes each id's position.
            var dense = new int[idSpace];
            Array.Fill(dense, -1);
            var positions = new Vec3[System.Math.Max(Ids.Length, 4)];
            var count = 0;
            for (var face = 0; face < faces.Length; ++face)
            {
                var vertices = CollectionsMarshal.AsSpan(faces[face].Vertices);
                var start = VertexStart[face];
                for (var i = 0; i < vertices.Length; ++i)
                {
                    ref var id = ref dense[ids[start + i]];
                    if (id < 0)
                    {
                        id = count;
                        positions[count++] = Position(vertices[i]);
                    }
                    Ids[start + i] = id;
                }
            }
            Welder = new PointWelder(positions, count);
            BuildLoops(solid);
        }

        public WeldedLoops(CsgSolid solid, float epsilon)
        {
            _solid = solid;
            var faces = solid._faces;
            VertexStart = new int[faces.Length + 1];
            for (var face = 0; face < faces.Length; ++face)
                VertexStart[face + 1] = VertexStart[face] + faces[face].Vertices.Count;
            Epsilon = epsilon;
            VertexIds = new int[VertexStart[faces.Length]];
            Ids = new int[VertexIds.Length];
            Corners = new int[VertexIds.Length];
            LoopLength = new int[faces.Length];
            Welder = new PointWelder(epsilon, VertexIds.Length);
            _junctionTolerance = epsilon * JunctionScale;
            for (var face = 0; face < faces.Length; ++face)
            {
                var vertices = CollectionsMarshal.AsSpan(faces[face].Vertices);
                var start = VertexStart[face];
                for (var i = 0; i < vertices.Length; ++i)
                {
                    var point = Position(vertices[i]);
                    var id = Welder.Add(point);
                    Ids[start + i] = id;
                    Canonical &= Welder.Positions[id] == point;
                }
            }
            BuildLoops(solid);
        }

        /// <summary>
        /// Weld within epsilon starting from loops welded exactly: only the distinct positions are welded again,
        /// which is far fewer than the face vertices when faces share their vertices.
        /// </summary>
        public WeldedLoops(CsgSolid solid, WeldedLoops exact, float epsilon)
        {
            _solid = solid;
            var faces = solid._faces;
            VertexStart = exact.VertexStart;
            Epsilon = epsilon;
            VertexIds = new int[exact.VertexIds.Length];
            Ids = new int[VertexIds.Length];
            Corners = new int[VertexIds.Length];
            LoopLength = new int[faces.Length];
            Welder = new PointWelder(epsilon, exact.Welder.Count);
            _junctionTolerance = epsilon * JunctionScale;
            var map = new int[exact.Welder.Count];
            for (var i = 0; i < map.Length; ++i)
            {
                map[i] = Welder.Add(exact.Welder.Positions[i]);
                Canonical &= map[i] == i;
            }
            for (var i = 0; i < Ids.Length; ++i)
                Ids[i] = map[exact.Ids[i]];
            BuildLoops(solid);
        }

        private void BuildLoops(CsgSolid solid)
        {
            var faces = solid._faces;
            for (var face = 0; face < faces.Length; ++face)
            {
                var start = VertexStart[face];
                var count = VertexStart[face + 1] - start;
                var length = 0;
                for (var i = 0; i < count; ++i)
                {
                    var id = Ids[start + i];
                    if (length != 0 && VertexIds[start + length - 1] == id)
                        continue;
                    Corners[start + length] = i;
                    VertexIds[start + length++] = id;
                }
                while (length > 1 && VertexIds[start + length - 1] == VertexIds[start])
                    --length;
                var hasSurface = solid._triangleStart[face] != solid._triangleStart[face + 1];
                LoopLength[face] = hasSurface && length >= 3 ? length : 0;
            }
        }

        private Dictionary<ulong, EdgeState> BuildRawEdges()
        {
            var edges = new Dictionary<ulong, EdgeState>(VertexIds.Length, CsgCommon.PackedKeyComparer.Instance);
            for (var face = 0; face < _solid._faces.Length; ++face)
            {
                var count = LoopLength[face];
                for (var i = 0; i < count; ++i)
                {
                    var from = VertexIds[VertexStart[face] + i];
                    var to = VertexIds[VertexStart[face] + (i + 1) % count];
                    ref var state = ref CollectionsMarshal.GetValueRefOrAddDefault(edges, EdgeKey(from, to),
                        out var exists);
                    if (exists)
                        Links.Add((face, state.Face));
                    else
                        state.Face = face;
                    ++state.Count;
                    state.Winding += from < to ? 1 : -1;
                }
            }
            return edges;
        }

        /// <summary>Whether every directed edge has an opposite partner, so the loops close up exactly.</summary>
        public bool IsBalanced()
        {
            foreach (var edge in RawEdges.Values)
            {
                if (edge.Winding != 0)
                    return false;
            }
            return true;
        }

        /// <summary>
        /// Whether every face edge between the ids has an opposite partner, so none can end at a T-junction. Solids
        /// that touch along an edge pinch it with two balanced pairs.
        /// </summary>
        public bool IsMatched(int from, int to) => RawEdges[EdgeKey(from, to)].Winding == 0;

        /// <summary>Welded points within the junction tolerance of the edge interior, in order along it.</summary>
        public void CollectJunctions(int from, int to, List<(float T, int Id)> cuts)
        {
            cuts.Clear();
            var start = Welder.Positions[from];
            var end = Welder.Positions[to];
            var span = end - start;
            var spanLengthSquared = span.LengthSquared();
            if (_points is null)
            {
                var points = Welder.Positions.AsSpan(0, Welder.Count).ToArray();
                _points = new FaceBvh(points, points);
            }
            _points.CollectBox(Vec3.MinNative(start, end), Vec3.MaxNative(start, end), _junctionTolerance, _nearby);
            var toleranceSquared = _junctionTolerance * _junctionTolerance;
            foreach (var item in _nearby)
            {
                if (item == from || item == to)
                    continue;
                var point = Welder.Positions[item];
                var t = Vec3.Dot(point - start, span) / spanLengthSquared;
                if (t <= 0.0f || t >= 1.0f)
                    continue;
                if ((point - (start + span * t)).LengthSquared() <= toleranceSquared)
                    cuts.Add((t, item));
            }
            cuts.Sort(static (a, b) => a.T.CompareTo(b.T));
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

        // With a zero epsilon only identical positions share an id.
        private Dictionary<Vec3, int>? _exact;

        /// <summary>A welder over positions that are already distinct; ids are their indices.</summary>
        public PointWelder(Vec3[] positions, int count)
        {
            _epsilon = 0.0f;
            _inverseCell = 0.0;
            _cells = new Dictionary<(long, long, long), int>(0);
            Positions = positions;
            Count = count;
            _next = [];
        }

        public PointWelder(float epsilon, int capacity)
        {
            _epsilon = epsilon;
            if (epsilon <= 0.0f)
                _exact = new Dictionary<Vec3, int>(capacity);
            _inverseCell = epsilon > 0.0f ? 1.0 / (4.0 * epsilon) : 0.0;
            _cells = new Dictionary<(long, long, long), int>(_exact is null ? capacity : 0);
            Positions = new Vec3[System.Math.Max(capacity, 4)];
            _next = new int[Positions.Length];
        }

        public int Add(Vec3 point)
        {
            if (_epsilon <= 0.0f && _exact is null)
            {
                // Distinct positions were given; index them on first use.
                _exact = new Dictionary<Vec3, int>(Count);
                for (var i = 0; i < Count; ++i)
                    _exact.TryAdd(Positions[i], i);
            }
            if (_exact is not null)
            {
                ref var id = ref CollectionsMarshal.GetValueRefOrAddDefault(_exact, point, out var known);
                if (!known)
                {
                    if (Count == Positions.Length)
                        Array.Resize(ref Positions, Count * 2);
                    id = Count++;
                    Positions[id] = point;
                }
                return id;
            }
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
            if (_epsilon <= 0.0f)
            {
                if (_exact is null)
                {
                    _exact = new Dictionary<Vec3, int>(Count);
                    for (var i = 0; i < Count; ++i)
                        _exact.TryAdd(Positions[i], i);
                }
                return _exact.TryGetValue(point, out var id) ? id : -1;
            }
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
