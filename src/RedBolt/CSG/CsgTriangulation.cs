// Copyright (c) 2026 RedBolt and contributors.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

namespace RedBolt.CSG;

using System;
using System.Collections.Generic;

using RedBolt.Graphics;
using RedBolt.Math;

/// <summary>Triangulated CPU representation of CSG output.</summary>
public sealed class CsgTriangulatedModel
{
    public sealed class Geometry
    {
        /// <summary>
        /// <c>(sourceMeshId &lt;&lt; 16) | sourceGeometryIndex</c> copied from <see cref="CsgPolygon.Shared"/>.
        /// Used on the main thread to resolve source vertex layout and output material.
        /// </summary>
        public uint Shared;

        /// <summary>Triangulated vertex stream in unpacked <see cref="ModelVertex"/> format.</summary>
        public List<ModelVertex> Vertices = [];
        public List<uint> IndexData = [];
    }

    public List<Geometry> Geometries = [];
    public BoundingBox BoundingBox;
    public bool HasAnyVertex;
}

public static partial class Csg
{
    /// <summary>
    /// Distance from the line through its neighbors, relative to the largest coordinate of the polygon, within
    /// which a vertex counts as part of a straight run: a few float roundings. Faces that share a run then agree
    /// that it is straight, so neither claims the sliver between the run and its chord.
    /// </summary>
    private const double StraightDistance = 1e-6;


    /// <summary>Triangulate polygon into triangle list indices (triplets).</summary>
    /// <remarks>
    /// Ear clipping in the plane of the polygon with double-precision orientation tests. An ear tip must turn
    /// strictly toward the inside, so a vertex on a straight run of the boundary never becomes a zero-area
    /// triangle whose diagonal lies along the boundary, and no other vertex may lie in the ear or on its
    /// diagonal. Vertices at the same position as an ear corner, as in a polygon that visits a bridge twice, do
    /// not block it. Only when no such ear exists is the most convex vertex clipped.
    /// </remarks>
    public static bool TriangulatePolygon(CsgPolygon poly, List<uint> outIndices) =>
        TriangulatePolygon(poly, outIndices, null);

    /// <summary>
    /// Triangulate a polygon; a vertex marked in <paramref name="straight"/> lies on a run shared with one
    /// neighbor and is not an ear tip while it lies within rounding of its neighbors' chord.
    /// </summary>
    internal static bool TriangulatePolygon(CsgPolygon poly, List<uint> outIndices, IReadOnlyList<bool>? straight) =>
        TriangulatePolygon(poly, outIndices, straight, out _);

    /// <summary>
    /// Triangulate a polygon, reporting whether every triangle was a proper ear. When no ear passes the checks the
    /// most convex vertex is clipped anyway, which can leave a flat or inverted triangle.
    /// </summary>
    internal static bool TriangulatePolygon(CsgPolygon poly, List<uint> outIndices, IReadOnlyList<bool>? straight,
        out bool clean)
    {
        clean = true;
        var numVertices = poly.Vertices.Count;
        if (numVertices < 3)
            return false;

        outIndices.Clear();
        if (numVertices == 3)
        {
            outIndices.AddRange([0, 1, 2]);
            return true;
        }

        CsgCommon.MakeAxesFromNormal(poly.Plane.Normal, out var axisU, out var axisV);
        var xs = new double[numVertices];
        var ys = new double[numVertices];
        var extent = 0.0;
        for (var i = 0; i < numVertices; ++i)
        {
            var p = poly.Vertices[i].GetPosition();
            extent = System.Math.Max(extent, System.Math.Max(System.Math.Abs(p.X),
                System.Math.Max(System.Math.Abs(p.Y), System.Math.Abs(p.Z))));
            xs[i] = (double)p.X * axisU.X + (double)p.Y * axisU.Y + (double)p.Z * axisU.Z;
            ys[i] = (double)p.X * axisV.X + (double)p.Y * axisV.Y + (double)p.Z * axisV.Z;
        }

        // Orient tests by the winding of the projected polygon, whatever the handedness of the projection.
        var area = 0.0;
        for (var i = 0; i < numVertices; ++i)
        {
            var j = (i + 1) % numVertices;
            area += xs[i] * ys[j] - xs[j] * ys[i];
        }
        var winding = area < 0.0 ? -1.0 : 1.0;

        var ring = new List<int>(numVertices);
        for (var i = 0; i < numVertices; ++i)
            ring.Add(i);

        while (ring.Count > 3)
        {
            var count = ring.Count;
            var clip = -1;
            var fallback = -1;
            var fallbackTurn = double.NegativeInfinity;
            for (var i = 0; i < count; ++i)
            {
                var a = ring[(i + count - 1) % count];
                var b = ring[i];
                var c = ring[(i + 1) % count];
                var turn = Orient(a, b, c);
                var scale = Length(a, b) * Length(b, c);
                var normalized = scale > 0.0 ? turn / scale : 0.0;
                if (normalized > fallbackTurn)
                {
                    fallbackTurn = normalized;
                    fallback = i;
                }
                // A tip within rounding of a neighbor closes a zero-length edge: the needle has no area to invert, and
                // leaving it would block every ear around it.
                var needle = extent * StraightDistance;
                if (straight?[b] != true && turn >= 0.0 && (Length(a, b) <= needle || Length(b, c) <= needle))
                {
                    clip = i;
                    break;
                }
                // Distance of b from the chord a-c is twice the ear's area over the chord length.
                var chord = Length(a, c);
                if (turn <= 0.0 || turn <= chord * extent * StraightDistance || IsStraight(b)
                    || count == 4 && IsStraight(ring[(i + 2) % count]) || ContainsOther(a, b, c))
                    continue;
                clip = i;
                break;
            }
            if (clip < 0)
            {
                clip = fallback;
                clean = false;
            }
            outIndices.Add((uint)ring[(clip + count - 1) % count]);
            outIndices.Add((uint)ring[clip]);
            outIndices.Add((uint)ring[(clip + 1) % count]);
            ring.RemoveAt(clip);
        }
        outIndices.Add((uint)ring[0]);
        outIndices.Add((uint)ring[1]);
        outIndices.Add((uint)ring[2]);
        return true;

        // A vertex on a shared run within rounding. With four vertices left, clipping the vertex opposite one
        // would leave the run itself as the last triangle.
        bool IsStraight(int vertex)
        {
            if (straight?[vertex] != true)
                return false;
            var at = ring.IndexOf(vertex);
            var previous = ring[(at + ring.Count - 1) % ring.Count];
            var next = ring[(at + 1) % ring.Count];
            var chord = Length(previous, next);
            return chord <= 0.0 || System.Math.Abs(Orient(previous, vertex, next)) / chord < extent * StraightDistance;
        }

        double Orient(int a, int b, int c) =>
            winding * ((xs[b] - xs[a]) * (ys[c] - ys[a]) - (ys[b] - ys[a]) * (xs[c] - xs[a]));

        double Length(int a, int b)
        {
            var dx = xs[b] - xs[a];
            var dy = ys[b] - ys[a];
            return System.Math.Sqrt(dx * dx + dy * dy);
        }

        bool Same(int a, int b) => xs[a] == xs[b] && ys[a] == ys[b];

        // A vertex inside the ear blocks it, and so does one within rounding of the new diagonal: clipping past a
        // vertex on the diagonal would strand it on a straight run with nothing left to connect to. Vertices on
        // the ear's own boundary edges, or their extensions, do not block it.
        bool ContainsOther(int a, int b, int c)
        {
            var diagonal = Length(c, a) * extent * StraightDistance;
            foreach (var j in ring)
            {
                if (j == a || j == b || j == c || Same(j, a) || Same(j, b) || Same(j, c))
                    continue;
                if (Orient(a, b, j) <= 0.0 || Orient(b, c, j) <= 0.0)
                    continue;
                var side = Orient(c, a, j);
                if (side > 0.0)
                    return true;
                // Within rounding of the diagonal counts only alongside its interior: a vertex next to an end of it
                // closes a tiny edge rather than lying on a straight run.
                if (side >= -diagonal)
                {
                    var along = (xs[j] - xs[c]) * (xs[a] - xs[c]) + (ys[j] - ys[c]) * (ys[a] - ys[c]);
                    var length = (xs[a] - xs[c]) * (xs[a] - xs[c]) + (ys[a] - ys[c]) * (ys[a] - ys[c]);
                    if (along > length * 1e-3 && along < length * (1.0 - 1e-3))
                        return true;
                }
            }
            return false;
        }
    }

    /// <summary>Triangulate a collection of polygons into one indexed triangle list.</summary>
    /// <remarks>Output vertices are returned as full <see cref="ModelVertex"/> (unpacked).</remarks>
    public static bool TriangulateGeometry(IReadOnlyList<CsgPolygon> polys, float epsilon,
        List<ModelVertex> outVertices, List<uint> outIndexData) =>
        TriangulateGeometry(polys, (poly, _, indices) => TriangulatePolygon(poly, indices), null, 0, epsilon,
            outVertices, outIndexData);

    /// <summary>
    /// Triangulate polygons with triangles from <paramref name="triangulate"/>, given each polygon and its index.
    /// Vertices are shared when they coincide and their attributes agree; <paramref name="vertexKey"/>, when given,
    /// names each polygon vertex's position by an id below <paramref name="keySpace"/> so no hashing is needed.
    /// </summary>
    private static bool TriangulateGeometry(IReadOnlyList<CsgPolygon> polys,
        Func<CsgPolygon, int, List<uint>, bool> triangulate, Func<int, int, int>? vertexKey, int keySpace,
        float epsilon, List<ModelVertex> outVertices, List<uint> outIndexData)
    {
        outVertices.Clear();
        outIndexData.Clear();

        if (polys.Count == 0)
            return false;

        var eps = MathF.Max(epsilon, 1e-6f);
        var invEps = 1.0f / eps;

        var estimatedVertexCount = 0;
        var estimatedIndexCount = 0;
        foreach (var poly in polys)
        {
            estimatedVertexCount += poly.Vertices.Count;
            if (poly.Vertices.Count >= 3)
                estimatedIndexCount += (poly.Vertices.Count - 2) * 3;
        }

        outVertices.EnsureCapacity(estimatedVertexCount);
        outIndexData.EnsureCapacity(estimatedIndexCount);

        // Vertices with one key form a singly linked chain through `next`. Keys are the ids given, else cells of
        // quantized positions.
        var next = new List<int>(estimatedVertexCount);
        var headsById = vertexKey is not null ? new int[keySpace] : null;
        if (headsById is not null)
            Array.Fill(headsById, -1);
        var headsByCell = vertexKey is null ? new Dictionary<Vector3i, int>(estimatedVertexCount) : null;
        var polyIndex = 0;

        uint GetOrCreateVertexIndex(in ModelVertex vertex, int localIndex)
        {
            var key = 0;
            var cell = default(Vector3i);
            int head;
            if (headsById is not null)
            {
                key = vertexKey!(polyIndex, localIndex);
                head = headsById[key];
            }
            else
            {
                cell = CsgCommon.QuantizePosition(vertex.GetPosition(), invEps);
                head = headsByCell!.TryGetValue(cell, out var found) ? found : -1;
            }

            var span = System.Runtime.InteropServices.CollectionsMarshal.AsSpan(outVertices);
            for (var existingIndex = head; existingIndex >= 0; existingIndex = next[existingIndex])
            {
                ref readonly var existing = ref span[existingIndex];

                // Faces that share a vertex hold the same position exactly; merging a vertex into one merely
                // nearby would move it for some faces and not for others, opening the mesh along their edges.
                if (existing.Position != vertex.Position)
                    continue;

                // Other attributes are expected to match very closely if the vertex is truly shared.
                if (SameAttributes(existing, vertex))
                    return (uint)existingIndex;
            }

            var newIndex = outVertices.Count;
            outVertices.Add(CsgCommon.CloneVertex(vertex));
            next.Add(head);
            if (headsById is not null)
                headsById[key] = newIndex;
            else
                headsByCell![cell] = newIndex;
            return (uint)newIndex;
        }

        var triIndices = new List<uint>();
        for (polyIndex = 0; polyIndex < polys.Count; ++polyIndex)
        {
            var poly = polys[polyIndex];
            if (poly.Vertices.Count < 3)
                continue;

            if (!triangulate(poly, polyIndex, triIndices))
                continue;

            var vertices = System.Runtime.InteropServices.CollectionsMarshal.AsSpan(poly.Vertices);
            foreach (var polyVertexIndex in triIndices)
                outIndexData.Add(GetOrCreateVertexIndex(vertices[(int)polyVertexIndex], (int)polyVertexIndex));
        }

        return outIndexData.Count != 0;
    }

    /// <summary>
    /// <see cref="ModelVertex.Equals(ModelVertex)"/> for vertices at one position, by reference and with the
    /// attribute most likely to differ, the normal, first. Attribute arrays shared by both compare at once.
    /// </summary>
    private static bool SameAttributes(in ModelVertex a, in ModelVertex b)
    {
        if (!a.Normal.ApproximatelyEquivalent(b.Normal) || !a.Tangent.ApproximatelyEquivalent(b.Tangent)
            || !a.Binormal.ApproximatelyEquivalent(b.Binormal))
            return false;
        if (!ReferenceEquals(a.Uv, b.Uv))
        {
            for (var i = 0; i < ModelVertex.MaxUVs; ++i)
            {
                if (!a.Uv[i].ApproximatelyEquivalent(b.Uv[i]))
                    return false;
            }
        }
        if (!ReferenceEquals(a.Color, b.Color))
        {
            for (var i = 0; i < ModelVertex.MaxColors; ++i)
            {
                if (!a.Color[i].ApproximatelyEquivalent(b.Color[i]))
                    return false;
            }
        }
        return a.BlendIndices.ApproximatelyEquivalent(b.BlendIndices)
            && a.BlendWeights.ApproximatelyEquivalent(b.BlendWeights);
    }

    /// <summary>Triangulate a solid into a render mesh without cracks.</summary>
    /// <remarks>
    /// Solids share every edge vertex between neighboring faces at identical positions, so the triangles the solid
    /// already holds meet exactly and are emitted as they are.
    /// </remarks>
    public static CsgTriangulatedModel TriangulateSolid(CsgSolid solid, float epsilon = CsgCommon.DefaultEpsilon)
    {
        ArgumentNullException.ThrowIfNull(solid);
        return Triangulate(solid.Faces, (_, face, indices) => solid.CopyTriangles(face, indices), epsilon,
            solid.VertexKey, solid.VertexKeySpace);
    }

    /// <summary>Triangulate polygons into CPU representation of CSG output.</summary>
    public static CsgTriangulatedModel TriangulatePolygons(List<CsgPolygon> polygons,
        float epsilon = CsgCommon.DefaultEpsilon) => TriangulatePolygons((IReadOnlyList<CsgPolygon>)polygons, epsilon);

    /// <summary>Triangulate polygons into CPU representation of CSG output.</summary>
    /// <remarks>
    /// Input polygons are not modified; output vertices never share attribute arrays with them. Polygons are
    /// triangulated as given.
    /// </remarks>
    public static CsgTriangulatedModel TriangulatePolygons(IReadOnlyList<CsgPolygon> polygons,
        float epsilon = CsgCommon.DefaultEpsilon) =>
        Triangulate(polygons, (polygon, _, indices) => TriangulatePolygon(polygon, indices), epsilon);

    /// <summary>Group polygons by source geometry and emit one indexed triangle list per group.</summary>
    private static CsgTriangulatedModel Triangulate(IReadOnlyList<CsgPolygon> polygons,
        Func<CsgPolygon, int, List<uint>, bool> triangulate, float epsilon, Func<int, int, int>? vertexKey = null,
        int keySpace = 0)
    {
        var result = new CsgTriangulatedModel();

        if (polygons.Count == 0)
            return result;

        var groups = new Dictionary<uint, List<int>>();
        for (var i = 0; i < polygons.Count; ++i)
        {
            var p = polygons[i];
            if (!groups.TryGetValue(p.Shared, out var group))
            {
                group = [];
                groups.Add(p.Shared, group);
            }
            group.Add(i);
        }

        var groupKeys = new List<uint>(groups.Keys);
        groupKeys.Sort();

        result.Geometries.EnsureCapacity(groupKeys.Count);

        foreach (var sharedKey in groupKeys)
        {
            var members = groups[sharedKey];
            var polys = new CsgPolygon[members.Count];
            for (var i = 0; i < members.Count; ++i)
                polys[i] = polygons[members[i]];
            var geometry = new CsgTriangulatedModel.Geometry
            {
                Shared = sharedKey
            };

            Func<int, int, int>? groupKey = vertexKey is null ? null : (index, local) => vertexKey(members[index], local);
            if (!TriangulateGeometry(polys, (polygon, index, indices) => triangulate(polygon, members[index], indices),
                    groupKey, keySpace, epsilon, geometry.Vertices, geometry.IndexData))
                continue;

            if (geometry.Vertices.Count == 0 || geometry.IndexData.Count == 0)
                continue;

            foreach (var v in geometry.Vertices)
            {
                var pos = v.GetPosition();
                if (!result.HasAnyVertex)
                {
                    result.BoundingBox = new BoundingBox(pos, pos);
                    result.HasAnyVertex = true;
                }
                else
                    result.BoundingBox.Merge(pos);
            }

            result.Geometries.Add(geometry);
        }

        return result;
    }
}
