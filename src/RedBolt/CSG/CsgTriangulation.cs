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
    /// <summary>Triangulate polygon into triangle list indices (triplets).</summary>
    public static bool TriangulatePolygon(CsgPolygon poly, List<uint> outIndices)
    {
        var numVertices = poly.Vertices.Count;
        if (numVertices < 3)
            return false;

        outIndices.Clear();
        if (numVertices == 3)
        {
            outIndices.AddRange([0, 1, 2]);
            return true;
        }

        // Project to 2D for robustness
        CsgCommon.MakeAxesFromNormal(poly.Plane.Normal, out var axisU, out var axisV);

        var verts2D = new Vector2f[numVertices];
        for (var i = 0; i < numVertices; ++i)
        {
            var p = poly.Vertices[i].GetPosition();
            verts2D[i] = new Vector2f(p.DotProduct(axisU), p.DotProduct(axisV));
        }

        var indices = new List<uint>(numVertices);
        for (uint i = 0; i < numVertices; ++i)
            indices.Add(i);

        var count = numVertices;
        var maxIterations = count * 2;

        while (count > 2)
        {
            if (maxIterations-- <= 0)
                break;

            var earFound = false;

            // Pass 1: Look for a valid ear
            for (var i = 0; i < count; ++i)
            {
                var prev = (i + count - 1) % count;
                var next = (i + 1) % count;

                var u = indices[prev];
                var v = indices[i];
                var w = indices[next];

                // Check convexity using 3D normal
                var p0_3d = poly.Vertices[(int)u].GetPosition();
                var p1_3d = poly.Vertices[(int)v].GetPosition();
                var p2_3d = poly.Vertices[(int)w].GetPosition();
                var e1_3d = p1_3d - p0_3d;
                var e2_3d = p2_3d - p1_3d;

                if (e1_3d.CrossProduct(e2_3d).DotProduct(poly.Plane.Normal) < -MathDefs.Epsilon)
                    continue; // Reflex

                var p0 = verts2D[u];
                var p1 = verts2D[v];
                var p2 = verts2D[w];

                // Check if any other vertex is inside the triangle
                var containsVertex = false;
                for (var j = 0; j < count; ++j)
                {
                    if (j == prev || j == i || j == next)
                        continue;

                    var p = verts2D[indices[j]];

                    var v0 = p2 - p0;
                    var v1 = p1 - p0;
                    var v2 = p - p0;

                    var dot00 = v0.DotProduct(v0);
                    var dot01 = v0.DotProduct(v1);
                    var dot02 = v0.DotProduct(v2);
                    var dot11 = v1.DotProduct(v1);
                    var dot12 = v1.DotProduct(v2);

                    var denom = dot00 * dot11 - dot01 * dot01;
                    if (MathF.Abs(denom) < MathDefs.Epsilon)
                        continue;

                    var invDenom = 1.0f / denom;
                    var uBary = (dot11 * dot02 - dot01 * dot12) * invDenom;
                    var vBary = (dot00 * dot12 - dot01 * dot02) * invDenom;

                    if (uBary >= -1e-5f && vBary >= -1e-5f && uBary + vBary <= 1.0f + 1e-5f)
                    {
                        containsVertex = true;
                        break;
                    }
                }

                if (!containsVertex)
                {
                    outIndices.Add(u);
                    outIndices.Add(v);
                    outIndices.Add(w);

                    indices.RemoveAt(i);
                    --count;
                    earFound = true;
                    break;
                }
            }

            // Pass 2: Force clip first convex ear
            if (!earFound)
            {
                for (var i = 0; i < count; ++i)
                {
                    var prev = (i + count - 1) % count;
                    var next = (i + 1) % count;
                    var u = indices[prev];
                    var v = indices[i];
                    var w = indices[next];

                    var p0_3d = poly.Vertices[(int)u].GetPosition();
                    var p1_3d = poly.Vertices[(int)v].GetPosition();
                    var p2_3d = poly.Vertices[(int)w].GetPosition();
                    var e1_3d = p1_3d - p0_3d;
                    var e2_3d = p2_3d - p1_3d;

                    if (e1_3d.CrossProduct(e2_3d).DotProduct(poly.Plane.Normal) >= -MathDefs.Epsilon)
                    {
                        outIndices.Add(u);
                        outIndices.Add(v);
                        outIndices.Add(w);
                        indices.RemoveAt(i);
                        --count;
                        earFound = true;
                        break;
                    }
                }
            }

            // Pass 3: Force clip any ear
            if (!earFound)
            {
                const int i = 0;
                var prev = (i + count - 1) % count;
                var next = (i + 1) % count;
                outIndices.Add(indices[prev]);
                outIndices.Add(indices[i]);
                outIndices.Add(indices[next]);
                indices.RemoveAt(i);
                --count;
            }
        }

        return true;
    }

    /// <summary>Triangulate a collection of polygons into one indexed triangle list.</summary>
    /// <remarks>Output vertices are returned as full <see cref="ModelVertex"/> (unpacked).</remarks>
    public static bool TriangulateGeometry(IReadOnlyList<CsgPolygon> polys, float epsilon,
        List<ModelVertex> outVertices, List<uint> outIndexData)
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

        // Each bucket is a singly linked chain through `next`, in insertion order, so the earliest
        // matching vertex wins as it would in a per-bucket list.
        var buckets = new Dictionary<Vector3i, (int Head, int Tail)>(estimatedVertexCount);
        var next = new List<int>(estimatedVertexCount);

        uint GetOrCreateVertexIndex(in ModelVertex vertex)
        {
            var key = CsgCommon.QuantizePosition(vertex.GetPosition(), invEps);
            ref var bucket = ref System.Runtime.InteropServices.CollectionsMarshal.GetValueRefOrAddDefault(
                buckets, key, out var exists);
            if (exists)
            {
                var span = System.Runtime.InteropServices.CollectionsMarshal.AsSpan(outVertices);
                for (var existingIndex = bucket.Head; existingIndex >= 0; existingIndex = next[existingIndex])
                {
                    ref readonly var existing = ref span[existingIndex];

                    // Fast reject by position with CSG epsilon.
                    if (!existing.Position.ApproximatelyEquivalent(vertex.Position, eps))
                        continue;

                    // Other attributes are expected to match very closely if the vertex is truly shared.
                    if (existing.Equals(vertex))
                        return (uint)existingIndex;
                }
            }

            var newIndex = outVertices.Count;
            outVertices.Add(CsgCommon.CloneVertex(vertex));
            next.Add(-1);
            if (exists)
                next[bucket.Tail] = newIndex;
            else
                bucket.Head = newIndex;
            bucket.Tail = newIndex;
            return (uint)newIndex;
        }

        var triIndices = new List<uint>();
        foreach (var poly in polys)
        {
            if (poly.Vertices.Count < 3)
                continue;

            if (!TriangulatePolygon(poly, triIndices))
                continue;

            var vertices = System.Runtime.InteropServices.CollectionsMarshal.AsSpan(poly.Vertices);
            foreach (var polyVertexIndex in triIndices)
                outIndexData.Add(GetOrCreateVertexIndex(vertices[(int)polyVertexIndex]));
        }

        return outIndexData.Count != 0;
    }

    /// <summary>Triangulate polygons into CPU representation of CSG output.</summary>
    public static CsgTriangulatedModel TriangulatePolygons(List<CsgPolygon> polygons,
        float epsilon = CsgCommon.DefaultEpsilon) => TriangulatePolygons((IReadOnlyList<CsgPolygon>)polygons, epsilon);

    /// <summary>Triangulate polygons into CPU representation of CSG output.</summary>
    /// <remarks>Input polygons are not modified; output vertices never share attribute arrays with them.</remarks>
    public static CsgTriangulatedModel TriangulatePolygons(IReadOnlyList<CsgPolygon> polygons,
        float epsilon = CsgCommon.DefaultEpsilon)
    {
        var result = new CsgTriangulatedModel();

        if (polygons.Count == 0)
            return result;

        var groups = new Dictionary<uint, List<CsgPolygon>>();
        foreach (var sourcePolygon in polygons)
        {
            // Simplification copies polygons before editing them, so inputs are grouped as they are.
            var p = sourcePolygon;
            if (!groups.TryGetValue(p.Shared, out var group))
            {
                group = [];
                groups.Add(p.Shared, group);
            }
            group.Add(p);
        }

        var groupKeys = new List<uint>(groups.Keys);
        groupKeys.Sort();

        result.Geometries.EnsureCapacity(groupKeys.Count);

        foreach (var sharedKey in groupKeys)
        {
            var polys = groups[sharedKey];

            SimplifyPolygons(polys, epsilon, true);

            var geometry = new CsgTriangulatedModel.Geometry
            {
                Shared = sharedKey
            };

            if (!TriangulateGeometry(polys, epsilon, geometry.Vertices, geometry.IndexData))
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
