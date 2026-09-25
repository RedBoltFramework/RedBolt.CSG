// Copyright (c) 2026 RedBolt and contributors.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

namespace RedBolt.CSG;

using System;
using System.Collections.Generic;

using RedBolt.Math;

/// <summary>
/// Reusable, polygonal CSG operand. Faces and their attributes are retained independently of render triangles.
/// A face BVH finds the intersection frontier; shared edges identify regions that can be classified together.
/// </summary>
public sealed class CsgSolid
{
    private readonly record struct IntersectionEdge(int FaceA, int FaceB, Vector3f Start, Vector3f End);

    private readonly CsgPolygon[] _faces;
    private readonly int[][] _triangles;
    private readonly int[][] _components;
    private readonly int[][] _adjacentFaces;
    private readonly int[] _faceComponents;
    private readonly BoundingBox[] _faceBounds;
    private readonly FaceBvh _bvh;

    public int FaceCount => _faces.Length;
    public int ComponentCount => _components.Length;
    public bool IsClosed { get; }

    public CsgSolid(IReadOnlyList<CsgPolygon> polygons, float epsilon = CsgCommon.DefaultEpsilon)
        : this(polygons, epsilon, true)
    {
    }

    // Boolean results own newly split faces and may share untouched faces with immutable operands.
    internal CsgSolid(IReadOnlyList<CsgPolygon> polygons, float epsilon, bool copyFaces)
    {
        if (epsilon <= 0.0f)
            epsilon = CsgCommon.DefaultEpsilon;

        _faces = new CsgPolygon[polygons.Count];
        _triangles = new int[polygons.Count][];
        _faceBounds = new BoundingBox[polygons.Count];
        for (var i = 0; i < polygons.Count; ++i)
        {
            var face = copyFaces ? polygons[i].Clone() : polygons[i];
            _faces[i] = face;
            var indices = new List<uint>();
            Csg.TriangulatePolygon(face, indices);
            _triangles[i] = indices.ConvertAll(static index => checked((int)index)).ToArray();
            var bounds = new BoundingBox();
            foreach (var vertex in face.Vertices)
                bounds.Merge(vertex.GetPosition());
            _faceBounds[i] = bounds;
        }

        (_components, _faceComponents, _adjacentFaces, IsClosed) = BuildTopology(epsilon);
        _bvh = new FaceBvh(_faceBounds);
    }

    /// <summary>Copy editable polygon faces for a caller that needs to change them.</summary>
    public List<CsgPolygon> ToPolygons()
    {
        var result = new List<CsgPolygon>(_faces.Length);
        foreach (var face in _faces)
            result.Add(face.Clone());
        return result;
    }

    internal CsgSolid BooleanWithBsp(CsgSolid other, CsgOperation operation, float epsilon) =>
        new(BooleanWithBsp(ToPolygons(), other.ToPolygons(), operation, epsilon), epsilon, false);

    internal List<CsgPolygon> Boolean(CsgSolid other, CsgOperation operation, float epsilon)
    {
        if (_faces.Length == 0)
            return operation == CsgOperation.Union ? other.ToPolygons() : [];
        if (other._faces.Length == 0)
            return operation == CsgOperation.Intersection ? [] : ToPolygons();

        // Classification by a single sample is valid only for closed connected surfaces. Retain the
        // established BSP behavior for open or non-manifold inputs.
        if (!IsClosed || !other.IsClosed)
            return BooleanWithBsp(ToPolygons(), other.ToPolygons(), operation, epsilon);

        var contactsA = new List<int>[_components.Length];
        var contactsB = new List<int>[other._components.Length];
        var componentLinks = new HashSet<(int A, int B)>();
        var frontierA = new List<IntersectionEdge>[_faces.Length];
        var frontierB = new List<IntersectionEdge>[other._faces.Length];
        var uncertainContact = false;
        _bvh.VisitOverlaps(other._bvh, (faceA, faceB) =>
        {
            if (!FacesIntersect(faceA, other, faceB, epsilon))
                return;
            if (!CollectIntersectionEdges(faceA, other, faceB, epsilon, frontierA, frontierB))
            {
                uncertainContact = true;
                return;
            }
            var componentA = _faceComponents[faceA];
            var componentB = other._faceComponents[faceB];
            if (componentLinks.Add((componentA, componentB)))
            {
                (contactsA[componentA] ??= []).Add(componentB);
                (contactsB[componentB] ??= []).Add(componentA);
            }
        }, epsilon);
        if (uncertainContact)
            return BooleanWithBsp(ToPolygons(), other.ToPolygons(), operation, epsilon);

        var result = new List<CsgPolygon>();
        var visitedA = new bool[_components.Length];
        var visitedB = new bool[other._components.Length];
        var queue = new Queue<(bool InA, int Index)>();

        for (var component = 0; component < _components.Length; ++component)
        {
            if (visitedA[component])
                continue;
            if (contactsA[component] is null)
            {
                visitedA[component] = true;
                var inside = other.ContainsPoint(ComponentSample(component), epsilon);
                if (operation == CsgOperation.Intersection ? inside : !inside)
                    AppendComponent(result, component, false);
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

            if (!AppendContactGroup(result, other, groupA, groupB, frontierA, frontierB, operation, epsilon))
                return BooleanWithBsp(ToPolygons(), other.ToPolygons(), operation, epsilon);
        }

        for (var component = 0; component < other._components.Length; ++component)
        {
            if (visitedB[component])
                continue;
            var inside = ContainsPoint(other.ComponentSample(component), epsilon);
            if (operation == CsgOperation.Union && !inside
                || operation == CsgOperation.Intersection && inside
                || operation == CsgOperation.DifferenceAB && inside)
                other.AppendComponent(result, component, operation == CsgOperation.DifferenceAB);
        }
        return result;
    }

    private void AppendComponent(List<CsgPolygon> output, int component, bool flip)
    {
        foreach (var faceIndex in _components[component])
        {
            var face = flip ? _faces[faceIndex].Clone() : _faces[faceIndex];
            if (flip)
                face.Flip();
            output.Add(face);
        }
    }

    private bool AppendContactGroup(List<CsgPolygon> result, CsgSolid other,
        IReadOnlyList<int> groupA, IReadOnlyList<int> groupB,
        List<IntersectionEdge>[] frontierA, List<IntersectionEdge>[] frontierB,
        CsgOperation operation, float epsilon)
    {
        // The intersection segments form barriers in the shared-edge graph. Classify every intact
        // patch once, then split only faces at the frontier using local planes.
        var local = new List<CsgPolygon>();
        if (!AppendPatchesAndFragments(local, other, groupA, frontierA, operation, false, epsilon)
            || !other.AppendPatchesAndFragments(local, this, groupB, frontierB, operation, true, epsilon))
            return false;
        result.AddRange(local);
        return true;
    }

    private bool AppendPatchesAndFragments(List<CsgPolygon> output, CsgSolid other,
        IReadOnlyList<int> components, List<IntersectionEdge>[] frontier,
        CsgOperation operation, bool secondOperand, float epsilon)
    {
        var visited = new bool[_faces.Length];
        var queue = new Queue<int>();
        foreach (var component in components)
        foreach (var face in _components[component])
        {
            if (visited[face])
                continue;
            visited[face] = true;
            if (frontier[face] is { } segments)
            {
                if (!SplitAndClassifyFace(output, face, segments, other, operation, secondOperand, epsilon))
                    return false;
                continue;
            }

            var patch = new List<int> { face };
            queue.Enqueue(face);
            while (queue.TryDequeue(out var current))
            {
                foreach (var neighbor in _adjacentFaces[current])
                {
                    if (visited[neighbor] || frontier[neighbor] is not null)
                        continue;
                    visited[neighbor] = true;
                    patch.Add(neighbor);
                    queue.Enqueue(neighbor);
                }
            }
            if (!Keep(other.ContainsPoint(FaceSample(face), epsilon), operation, secondOperand))
                continue;
            foreach (var patchFace in patch)
            {
                var flip = secondOperand && operation == CsgOperation.DifferenceAB;
                var polygon = flip ? _faces[patchFace].Clone() : _faces[patchFace];
                if (flip)
                    polygon.Flip();
                output.Add(polygon);
            }
        }
        return true;
    }

    private bool SplitAndClassifyFace(List<CsgPolygon> output, int faceIndex,
        IReadOnlyList<IntersectionEdge> segments, CsgSolid other,
        CsgOperation operation, bool secondOperand, float epsilon)
    {
        const int maxFragmentsPerFace = 4096;
        var face = _faces[faceIndex];
        var pieces = new List<CsgPolygon>();
        if (face.IsConvex())
            pieces.Add(face.Clone());
        else
        {
            var triangles = _triangles[faceIndex];
            for (var i = 0; i + 2 < triangles.Length; i += 3)
            {
                pieces.Add(new CsgPolygon(
                [
                    face.Vertices[triangles[i]].Clone(),
                    face.Vertices[triangles[i + 1]].Clone(),
                    face.Vertices[triangles[i + 2]].Clone()
                ], face.Shared));
            }
        }

        var planes = new List<Plane>(segments.Count * 3);
        foreach (var segment in segments)
        {
            var oppositeFace = other._faces[secondOperand ? segment.FaceA : segment.FaceB];
            AddPlane(oppositeFace.Plane);
            var direction = (segment.End - segment.Start).Normalized;
            AddPlane(new Plane(direction, segment.Start));
            AddPlane(new Plane(direction, segment.End));
        }

        foreach (var plane in planes)
        {
            var split = new List<CsgPolygon>(pieces.Count * 2);
            foreach (var piece in pieces)
            {
                var front = false;
                var back = false;
                foreach (var vertex in piece.Vertices)
                {
                    var distance = plane.Distance(vertex.GetPosition());
                    front |= distance > epsilon;
                    back |= distance < -epsilon;
                }
                if (!front || !back)
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
            if (split.Count > maxFragmentsPerFace)
                return false;
            pieces = split;
        }

        var selected = new List<CsgPolygon>();
        foreach (var piece in pieces)
        {
            if (!Keep(other.ContainsPoint(PolygonSample(piece), epsilon), operation, secondOperand))
                continue;
            if (secondOperand && operation == CsgOperation.DifferenceAB)
                piece.Flip();
            selected.Add(piece);
        }
        Csg.SimplifyPolygons(selected, epsilon);
        output.AddRange(selected);
        return true;

        void AddPlane(Plane plane)
        {
            foreach (var existing in planes)
            {
                if (MathF.Abs(existing.Normal.DotProduct(plane.Normal)) > 0.99999f
                    && MathF.Abs(existing.D - plane.D) <= epsilon)
                    return;
            }
            planes.Add(plane);
        }
    }

    private Vector3f FaceSample(int faceIndex)
    {
        var face = _faces[faceIndex];
        var triangle = _triangles[faceIndex];
        return (face.Vertices[triangle[0]].GetPosition()
            + face.Vertices[triangle[1]].GetPosition()
            + face.Vertices[triangle[2]].GetPosition()) / 3.0f;
    }

    private static Vector3f PolygonSample(CsgPolygon polygon)
    {
        var result = Vector3f.Zero;
        foreach (var vertex in polygon.Vertices)
            result += vertex.GetPosition();
        return result / polygon.Vertices.Count;
    }

    private static bool Keep(bool inside, CsgOperation operation, bool secondOperand) =>
        operation == CsgOperation.Intersection ? inside
        : secondOperand && operation == CsgOperation.DifferenceAB ? inside : !inside;

    private Vector3f ComponentSample(int component)
    {
        var face = _faces[_components[component][0]];
        var triangle = _triangles[_components[component][0]];
        if (triangle.Length < 3)
            return face.Vertices[0].GetPosition();
        return (face.Vertices[triangle[0]].GetPosition()
            + face.Vertices[triangle[1]].GetPosition()
            + face.Vertices[triangle[2]].GetPosition()) / 3.0f;
    }

    /// <summary>Classify a point in a closed solid using BVH-accelerated ray parity.</summary>
    public bool ContainsPoint(Vector3f point, float epsilon = CsgCommon.DefaultEpsilon)
    {
        if (epsilon <= 0.0f)
            epsilon = CsgCommon.DefaultEpsilon;
        // Non-axis-aligned rays avoid most shared-edge and vertex hits. Counting unique distances
        // prevents two triangles on the same surface from changing the parity twice.
        var directions = new[]
        {
            new Vector3f(1.0f, 0.371f, 0.173f).Normalized,
            new Vector3f(0.239f, 1.0f, 0.419f).Normalized,
            new Vector3f(0.193f, 0.317f, 1.0f).Normalized
        };
        var insideVotes = 0;
        foreach (var direction in directions)
        {
            var ray = new Ray(point, direction);
            var hits = new List<float>();
            _bvh.VisitRay(ray, faceIndex =>
            {
                var face = _faces[faceIndex];
                var triangles = _triangles[faceIndex];
                for (var i = 0; i + 2 < triangles.Length; i += 3)
                {
                    var a = face.Vertices[triangles[i]].GetPosition();
                    var b = face.Vertices[triangles[i + 1]].GetPosition();
                    var c = face.Vertices[triangles[i + 2]].GetPosition();
                    var distance = ray.HitDistance(a, b, c, out _, out _);
                    if (float.IsPositiveInfinity(distance))
                        distance = ray.HitDistance(c, b, a, out _, out _);
                    if (distance > epsilon && float.IsFinite(distance))
                        hits.Add(distance);
                }
            });
            hits.Sort();
            var crossings = 0;
            var last = float.NegativeInfinity;
            foreach (var hit in hits)
            {
                if (hit - last <= epsilon * 2.0f)
                    continue;
                ++crossings;
                last = hit;
            }
            if ((crossings & 1) != 0)
                ++insideVotes;
        }
        return insideVotes >= 2;
    }

    private bool FacesIntersect(int faceIndex, CsgSolid other, int otherIndex, float epsilon)
    {
        var faceA = _faces[faceIndex];
        var faceB = other._faces[otherIndex];
        var trisA = _triangles[faceIndex];
        var trisB = other._triangles[otherIndex];
        for (var a = 0; a + 2 < trisA.Length; a += 3)
        {
            var a0 = faceA.Vertices[trisA[a]].GetPosition();
            var a1 = faceA.Vertices[trisA[a + 1]].GetPosition();
            var a2 = faceA.Vertices[trisA[a + 2]].GetPosition();
            for (var b = 0; b + 2 < trisB.Length; b += 3)
            {
                var b0 = faceB.Vertices[trisB[b]].GetPosition();
                var b1 = faceB.Vertices[trisB[b + 1]].GetPosition();
                var b2 = faceB.Vertices[trisB[b + 2]].GetPosition();
                if (TrianglesIntersect(a0, a1, a2, b0, b1, b2, epsilon))
                    return true;
            }
        }
        return false;
    }

    private bool CollectIntersectionEdges(int faceIndex, CsgSolid other, int otherIndex, float epsilon,
        List<IntersectionEdge>[] frontierA, List<IntersectionEdge>[] frontierB)
    {
        var faceA = _faces[faceIndex];
        var faceB = other._faces[otherIndex];
        var trisA = _triangles[faceIndex];
        var trisB = other._triangles[otherIndex];
        for (var a = 0; a + 2 < trisA.Length; a += 3)
        {
            var a0 = faceA.Vertices[trisA[a]].GetPosition();
            var a1 = faceA.Vertices[trisA[a + 1]].GetPosition();
            var a2 = faceA.Vertices[trisA[a + 2]].GetPosition();
            for (var b = 0; b + 2 < trisB.Length; b += 3)
            {
                var b0 = faceB.Vertices[trisB[b]].GetPosition();
                var b1 = faceB.Vertices[trisB[b + 1]].GetPosition();
                var b2 = faceB.Vertices[trisB[b + 2]].GetPosition();
                if (!TrianglesIntersect(a0, a1, a2, b0, b1, b2, epsilon))
                    continue;
                if (!TryIntersectionSegment(a0, a1, a2, b0, b1, b2, epsilon,
                        out var start, out var end, out var uncertain))
                {
                    if (uncertain)
                        return false;
                    continue; // A zero-length touch does not divide either surface.
                }
                var edge = new IntersectionEdge(faceIndex, otherIndex, start, end);
                (frontierA[faceIndex] ??= []).Add(edge);
                (frontierB[otherIndex] ??= []).Add(edge);
            }
        }
        return true;
    }

    private static bool TryIntersectionSegment(Vector3f a0, Vector3f a1, Vector3f a2,
        Vector3f b0, Vector3f b1, Vector3f b2, float epsilon,
        out Vector3f start, out Vector3f end, out bool uncertain)
    {
        start = end = Vector3f.Zero;
        uncertain = false;
        var planeA = new Plane(a0, a1, a2);
        var planeB = new Plane(b0, b1, b2);
        var line = planeA.Normal.CrossProduct(planeB.Normal);
        if (line.LengthSquared < 1e-8f)
        {
            uncertain = true; // Coplanar or nearly parallel contact uses the BSP fallback.
            return false;
        }
        line = line.Normalized;

        var cutsA = TrianglePlaneCuts(a0, a1, a2, planeB, epsilon);
        var cutsB = TrianglePlaneCuts(b0, b1, b2, planeA, epsilon);
        if (cutsA.Count < 2 || cutsB.Count < 2)
            return false;

        var origin = cutsA[0];
        var minA = float.PositiveInfinity;
        var maxA = float.NegativeInfinity;
        var minB = float.PositiveInfinity;
        var maxB = float.NegativeInfinity;
        foreach (var point in cutsA)
        {
            var value = (point - origin).DotProduct(line);
            minA = MathF.Min(minA, value);
            maxA = MathF.Max(maxA, value);
        }
        foreach (var point in cutsB)
        {
            var value = (point - origin).DotProduct(line);
            minB = MathF.Min(minB, value);
            maxB = MathF.Max(maxB, value);
        }
        var low = MathF.Max(minA, minB);
        var high = MathF.Min(maxA, maxB);
        if (high - low <= epsilon)
            return false;
        start = origin + line * low;
        end = origin + line * high;
        return true;
    }

    private static List<Vector3f> TrianglePlaneCuts(Vector3f first, Vector3f second, Vector3f third,
        Plane plane, float epsilon)
    {
        var vertices = new[] { first, second, third };
        var distances = new[] { plane.Distance(first), plane.Distance(second), plane.Distance(third) };
        var cuts = new List<Vector3f>(2);
        for (var i = 0; i < 3; ++i)
        {
            var next = (i + 1) % 3;
            if (MathF.Abs(distances[i]) <= epsilon)
                Add(vertices[i]);
            if (distances[i] > epsilon && distances[next] < -epsilon
                || distances[i] < -epsilon && distances[next] > epsilon)
            {
                var fraction = distances[i] / (distances[i] - distances[next]);
                Add(vertices[i].Lerp(vertices[next], fraction));
            }
        }
        return cuts;

        void Add(Vector3f point)
        {
            foreach (var existing in cuts)
            {
                if ((existing - point).LengthSquared <= epsilon * epsilon)
                    return;
            }
            cuts.Add(point);
        }
    }

    private static bool TrianglesIntersect(Vector3f a0, Vector3f a1, Vector3f a2,
        Vector3f b0, Vector3f b1, Vector3f b2, float epsilon)
    {
        var aEdges = new[] { a1 - a0, a2 - a1, a0 - a2 };
        var bEdges = new[] { b1 - b0, b2 - b1, b0 - b2 };
        var normalA = aEdges[0].CrossProduct(aEdges[1]);
        var normalB = bEdges[0].CrossProduct(bEdges[1]);
        if (normalA.LengthSquared <= epsilon * epsilon || normalB.LengthSquared <= epsilon * epsilon)
            return false;
        if (Separated(normalA) || Separated(normalB))
            return false;
        foreach (var edgeA in aEdges)
        {
            if (Separated(edgeA.CrossProduct(normalA)))
                return false;
            foreach (var edgeB in bEdges)
            {
                if (Separated(edgeA.CrossProduct(edgeB)))
                    return false;
            }
        }
        foreach (var edgeB in bEdges)
        {
            if (Separated(edgeB.CrossProduct(normalB)))
                return false;
        }
        return true;

        bool Separated(Vector3f axis)
        {
            var lengthSquared = axis.LengthSquared;
            if (lengthSquared <= 1e-20f)
                return false;
            var aMin = MathF.Min(a0.DotProduct(axis), MathF.Min(a1.DotProduct(axis), a2.DotProduct(axis)));
            var aMax = MathF.Max(a0.DotProduct(axis), MathF.Max(a1.DotProduct(axis), a2.DotProduct(axis)));
            var bMin = MathF.Min(b0.DotProduct(axis), MathF.Min(b1.DotProduct(axis), b2.DotProduct(axis)));
            var bMax = MathF.Max(b0.DotProduct(axis), MathF.Max(b1.DotProduct(axis), b2.DotProduct(axis)));
            var tolerance = epsilon * MathF.Sqrt(lengthSquared);
            return aMax < bMin - tolerance || bMax < aMin - tolerance;
        }
    }

    private static List<CsgPolygon> BooleanWithBsp(List<CsgPolygon> a, List<CsgPolygon> b,
        CsgOperation operation, float epsilon)
    {
        var bspA = new CsgBsp();
        bspA.Build(a, epsilon);
        var bspB = new CsgBsp();
        bspB.Build(b, epsilon);
        return Csg.BooleanOperation(bspA, bspB, operation, epsilon);
    }

    private (int[][] Components, int[] FaceComponents, int[][] AdjacentFaces, bool Closed) BuildTopology(float epsilon)
    {
        var parents = new int[_faces.Length];
        var ranks = new byte[_faces.Length];
        for (var i = 0; i < parents.Length; ++i)
            parents[i] = i;
        var edges = new Dictionary<EdgeKey, (int Face, int Count, int Winding)>();
        var adjacency = new List<int>[_faces.Length];
        for (var i = 0; i < adjacency.Length; ++i)
            adjacency[i] = [];
        var closed = _faces.Length != 0;

        // Register vertices first so a long edge can be matched to several short edges at a
        // T-junction. Such junctions are normal after local cuts and across material boundaries.
        var positionIds = new Dictionary<PositionKey, int>();
        var positions = new List<Vector3f>();
        foreach (var face in _faces)
        foreach (var vertex in face.Vertices)
        {
            var point = vertex.GetPosition();
            var key = PositionKey.From(point, epsilon);
            if (positionIds.TryAdd(key, positions.Count))
                positions.Add(point);
        }
        var pointBounds = new BoundingBox[positions.Count];
        for (var i = 0; i < positions.Count; ++i)
            pointBounds[i] = new BoundingBox(positions[i], positions[i]);
        var pointBvh = new FaceBvh(pointBounds);

        for (var faceIndex = 0; faceIndex < _faces.Length; ++faceIndex)
        {
            var vertices = _faces[faceIndex].Vertices;
            if (vertices.Count < 3 || _triangles[faceIndex].Length == 0)
            {
                closed = false;
                continue;
            }
            for (var vertex = 0; vertex < vertices.Count; ++vertex)
            {
                var start = vertices[vertex].GetPosition();
                var end = vertices[(vertex + 1) % vertices.Count].GetPosition();
                var first = PositionKey.From(start, epsilon);
                var second = PositionKey.From(end, epsilon);
                var span = end - start;
                var spanLengthSquared = span.LengthSquared;
                if (first == second || spanLengthSquared <= epsilon * epsilon)
                {
                    closed = false;
                    continue;
                }

                var cuts = new List<(float T, PositionKey Key)> { (0.0f, first), (1.0f, second) };
                var min = new Vector3f(MathF.Min(start.X, end.X), MathF.Min(start.Y, end.Y),
                    MathF.Min(start.Z, end.Z));
                var max = new Vector3f(MathF.Max(start.X, end.X), MathF.Max(start.Y, end.Y),
                    MathF.Max(start.Z, end.Z));
                pointBvh.VisitBox(new BoundingBox(min, max), pointIndex =>
                {
                    var point = positions[pointIndex];
                    var t = (point - start).DotProduct(span) / spanLengthSquared;
                    if (t <= 0.0f || t >= 1.0f)
                        return;
                    if ((point - (start + span * t)).LengthSquared > epsilon * epsilon)
                        return;
                    cuts.Add((t, PositionKey.From(point, epsilon)));
                }, epsilon);
                cuts.Sort(static (a, b) => a.T.CompareTo(b.T));
                for (var cut = 0; cut + 1 < cuts.Count; ++cut)
                {
                    var from = cuts[cut].Key;
                    var to = cuts[cut + 1].Key;
                    if (from == to)
                        continue;
                    var key = new EdgeKey(from, to);
                    var winding = from.CompareTo(to) < 0 ? 1 : -1;
                    if (edges.TryGetValue(key, out var previous))
                    {
                        Union(faceIndex, previous.Face);
                        adjacency[faceIndex].Add(previous.Face);
                        adjacency[previous.Face].Add(faceIndex);
                        edges[key] = (previous.Face, previous.Count + 1, previous.Winding + winding);
                    }
                    else
                        edges.Add(key, (faceIndex, 1, winding));
                }
            }
        }
        foreach (var edge in edges.Values)
            closed &= edge.Count == 2 && edge.Winding == 0;

        var componentIndices = new Dictionary<int, int>();
        var components = new List<List<int>>();
        var faceComponents = new int[_faces.Length];
        for (var face = 0; face < _faces.Length; ++face)
        {
            var root = Find(face);
            if (!componentIndices.TryGetValue(root, out var index))
            {
                index = components.Count;
                componentIndices.Add(root, index);
                components.Add([]);
            }
            faceComponents[face] = index;
            components[index].Add(face);
        }
        return (components.ConvertAll(static component => component.ToArray()).ToArray(), faceComponents,
            Array.ConvertAll(adjacency, static neighbors => neighbors.ToArray()), closed);

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

    private readonly record struct PositionKey(long X, long Y, long Z) : IComparable<PositionKey>
    {
        public static PositionKey From(Vector3f position, float epsilon) => new(
            checked((long)System.Math.Round((double)position.X / epsilon)),
            checked((long)System.Math.Round((double)position.Y / epsilon)),
            checked((long)System.Math.Round((double)position.Z / epsilon)));

        public int CompareTo(PositionKey other)
        {
            var result = X.CompareTo(other.X);
            if (result != 0) return result;
            result = Y.CompareTo(other.Y);
            return result != 0 ? result : Z.CompareTo(other.Z);
        }
    }

    private readonly record struct EdgeKey
    {
        public readonly PositionKey First;
        public readonly PositionKey Second;

        public EdgeKey(PositionKey first, PositionKey second)
        {
            if (first.CompareTo(second) <= 0)
                (First, Second) = (first, second);
            else
                (First, Second) = (second, first);
        }
    }

    private sealed class FaceBvh
    {
        private const int LeafSize = 8;
        private readonly BoundingBox[] _bounds;
        private readonly int[] _indices;
        private readonly List<Node> _nodes = [];

        private readonly record struct Node(BoundingBox Bounds, int Start, int Count, int Left, int Right)
        {
            public bool IsLeaf => Left < 0;
        }

        public FaceBvh(BoundingBox[] bounds)
        {
            _bounds = bounds;
            _indices = new int[bounds.Length];
            for (var i = 0; i < bounds.Length; ++i)
                _indices[i] = i;
            if (bounds.Length != 0)
                Build(0, bounds.Length);
        }

        private int Build(int start, int count)
        {
            var bounds = new BoundingBox();
            for (var i = start; i < start + count; ++i)
                bounds.Merge(_bounds[_indices[i]]);
            var index = _nodes.Count;
            _nodes.Add(default);
            if (count <= LeafSize)
            {
                _nodes[index] = new Node(bounds, start, count, -1, -1);
                return index;
            }
            var size = bounds.Size;
            var axis = size.X >= size.Y && size.X >= size.Z ? 0 : size.Y >= size.Z ? 1 : 2;
            Array.Sort(_indices, start, count, Comparer<int>.Create((a, b) =>
                Axis(_bounds[a].Center, axis).CompareTo(Axis(_bounds[b].Center, axis))));
            var leftCount = count / 2;
            var left = Build(start, leftCount);
            var right = Build(start + leftCount, count - leftCount);
            _nodes[index] = new Node(bounds, start, count, left, right);
            return index;
        }

        public void VisitOverlaps(FaceBvh other, Action<int, int> visitor, float epsilon)
        {
            if (_nodes.Count == 0 || other._nodes.Count == 0)
                return;
            var stack = new Stack<(int A, int B)>();
            stack.Push((0, 0));
            while (stack.TryPop(out var pair))
            {
                var a = _nodes[pair.A];
                var b = other._nodes[pair.B];
                if (!Overlaps(a.Bounds, b.Bounds, epsilon))
                    continue;
                if (a.IsLeaf && b.IsLeaf)
                {
                    for (var i = a.Start; i < a.Start + a.Count; ++i)
                    for (var j = b.Start; j < b.Start + b.Count; ++j)
                    {
                        var faceA = _indices[i];
                        var faceB = other._indices[j];
                        if (Overlaps(_bounds[faceA], other._bounds[faceB], epsilon))
                            visitor(faceA, faceB);
                    }
                }
                else if (b.IsLeaf || !a.IsLeaf && a.Count >= b.Count)
                {
                    stack.Push((a.Left, pair.B));
                    stack.Push((a.Right, pair.B));
                }
                else
                {
                    stack.Push((pair.A, b.Left));
                    stack.Push((pair.A, b.Right));
                }
            }
        }

        public void VisitRay(Ray ray, Action<int> visitor)
        {
            if (_nodes.Count == 0)
                return;
            var stack = new Stack<int>();
            stack.Push(0);
            while (stack.TryPop(out var index))
            {
                var node = _nodes[index];
                if (!float.IsFinite(ray.HitDistance(node.Bounds)))
                    continue;
                if (node.IsLeaf)
                {
                    for (var i = node.Start; i < node.Start + node.Count; ++i)
                    {
                        var face = _indices[i];
                        if (float.IsFinite(ray.HitDistance(_bounds[face])))
                            visitor(face);
                    }
                }
                else
                {
                    stack.Push(node.Left);
                    stack.Push(node.Right);
                }
            }
        }

        public void VisitBox(BoundingBox box, Action<int> visitor, float epsilon)
        {
            if (_nodes.Count == 0)
                return;
            var stack = new Stack<int>();
            stack.Push(0);
            while (stack.TryPop(out var index))
            {
                var node = _nodes[index];
                if (!Overlaps(node.Bounds, box, epsilon))
                    continue;
                if (node.IsLeaf)
                {
                    for (var i = node.Start; i < node.Start + node.Count; ++i)
                    {
                        var item = _indices[i];
                        if (Overlaps(_bounds[item], box, epsilon))
                            visitor(item);
                    }
                }
                else
                {
                    stack.Push(node.Left);
                    stack.Push(node.Right);
                }
            }
        }

        private static bool Overlaps(BoundingBox a, BoundingBox b, float epsilon) =>
            a.Min.X <= b.Max.X + epsilon && a.Max.X + epsilon >= b.Min.X
            && a.Min.Y <= b.Max.Y + epsilon && a.Max.Y + epsilon >= b.Min.Y
            && a.Min.Z <= b.Max.Z + epsilon && a.Max.Z + epsilon >= b.Min.Z;

        private static float Axis(Vector3f vector, int axis) => axis == 0 ? vector.X : axis == 1 ? vector.Y : vector.Z;
    }
}
