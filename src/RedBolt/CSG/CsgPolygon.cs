// Copyright (c) 2026 RedBolt and contributors.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

namespace RedBolt.CSG;

using System;
using System.Collections.Generic;

using SystemMath = global::System.Math;

using RedBolt.Graphics;
using RedBolt.Math;

/// <summary>Convex or concave planar polygon used by the CSG pipeline.</summary>
/// <remarks>
/// Inside the CSG pipeline, the <see cref="ModelVertex.Color"/> and <see cref="ModelVertex.Uv"/> arrays of
/// vertices are shared between polygon copies and must be treated as immutable: replace a vertex instead of
/// writing into those arrays. <see cref="Clone"/>, <see cref="CsgSolid.ToPolygons"/> and triangulated output
/// return independent copies for callers that edit attributes.
/// </remarks>
public sealed class CsgPolygon : ICloneable
{
    public List<ModelVertex> Vertices = [];
    public uint Shared; // (sourceMeshId << 16) | sourceGeometryIndex
    public Plane Plane;

    public CsgPolygon()
    {
    }

    public CsgPolygon(List<ModelVertex> vertices, uint sharedKey)
    {
        Vertices = vertices;
        Shared = sharedKey;
        if (Vertices.Count >= 3)
            Plane = FitPlane(Vertices);
    }

    /// <summary>
    /// Plane of a polygon by Newell's method: the normal sums the edge cross products about the centroid, so it
    /// does not depend on the first vertices being a proper corner.
    /// </summary>
    private static Plane FitPlane(List<ModelVertex> vertices)
    {
        var normal = Vector3f.Zero;
        var centroid = Vector3f.Zero;
        for (var i = 0; i < vertices.Count; ++i)
        {
            var current = vertices[i].GetPosition();
            var next = vertices[(i + 1) % vertices.Count].GetPosition();
            normal += new Vector3f((current.Y - next.Y) * (current.Z + next.Z), (current.Z - next.Z) * (current.X + next.X),
                (current.X - next.X) * (current.Y + next.Y));
            centroid += current;
        }
        centroid /= vertices.Count;
        return new Plane(normal.NormalizedOrDefault(Vector3f.Up), centroid);
    }

    public CsgPolygon Clone()
    {
        var result = new CsgPolygon
        {
            Shared = Shared,
            Plane = Plane
        };
        result.Vertices.EnsureCapacity(Vertices.Count);
        foreach (ref readonly var vertex in System.Runtime.InteropServices.CollectionsMarshal.AsSpan(Vertices))
            result.Vertices.Add(CsgCommon.CloneVertex(vertex));
        return result;
    }

    object ICloneable.Clone() => Clone();

    /// <summary>
    /// Copy the polygon with its own vertex list while sharing each vertex's color and UV arrays.
    /// CSG treats those arrays as immutable, so the copy can be flipped, split or extended safely.
    /// </summary>
    internal CsgPolygon CloneShared() => new()
    {
        Vertices = new List<ModelVertex>(Vertices),
        Shared = Shared,
        Plane = Plane
    };

    public void Flip()
    {
        Vertices.Reverse();
        for (var i = 0; i < Vertices.Count; ++i)
        {
            var vertex = Vertices[i];
            FlipModelVertex(ref vertex);
            Vertices[i] = vertex;
        }
        Plane.Normal = -Plane.Normal;
        Plane.D = -Plane.D;
    }

    public bool IsConvex()
    {
        if (Vertices.Count < 3)
            return false;

        var count = Vertices.Count;
        for (var i = 0; i < count; ++i)
        {
            var p0 = Vertices[i].GetPosition();
            var p1 = Vertices[(i + 1) % count].GetPosition();
            var p2 = Vertices[(i + 2) % count].GetPosition();

            var edge1 = p1 - p0;
            var edge2 = p2 - p1;
            var cross = edge1.CrossProduct(edge2);

            if (cross.DotProduct(Plane.Normal) < -MathDefs.Epsilon)
                return false;
        }
        return true;
    }

    /// <summary>Split the polygon by a plane; vertices within epsilon of the plane go to both sides.</summary>
    /// <remarks>A polygon entirely on one side is copied to that side and the other output stays empty.</remarks>
    internal void Split(Plane plane, out CsgPolygon frontPoly, out CsgPolygon backPoly, float epsilon)
    {
        frontPoly = new CsgPolygon();
        backPoly = new CsgPolygon();
        var vertices = Vertices;
        var count = vertices.Count;

        if (count < 3)
            return;

        var distances = new List<float>(count);
        var frontCount = 0;
        var backCount = 0;

        foreach (var vertex in vertices)
        {
            var distance = plane.Distance(vertex.GetPosition());
            distances.Add(distance);
            if (distance > epsilon)
                ++frontCount;
            else if (distance < -epsilon)
                ++backCount;
        }

        if (backCount == 0)
        {
            frontPoly = CloneShared();
            return;
        }
        if (frontCount == 0)
        {
            backPoly = CloneShared();
            return;
        }

        frontPoly.Shared = Shared;
        frontPoly.Plane = Plane;
        backPoly.Shared = Shared;
        backPoly.Plane = Plane;
        frontPoly.Vertices.EnsureCapacity(count + 2);
        backPoly.Vertices.EnsureCapacity(count + 2);

        for (var i = 0; i < count; ++i)
        {
            var j = (i + 1) % count;
            var v1 = vertices[i];
            var v2 = vertices[j];
            var d1 = distances[i];
            var d2 = distances[j];

            var v1Front = d1 >= -epsilon;
            var v1Back = d1 <= epsilon;

            if (v1Front)
                frontPoly.Vertices.Add(v1);
            if (v1Back)
                backPoly.Vertices.Add(v1);

            var crossing = d1 > epsilon && d2 < -epsilon || d1 < -epsilon && d2 > epsilon;
            if (crossing)
            {
                var t = SystemMath.Clamp(d1 / (d1 - d2), 0.0f, 1.0f);
                var mid = CsgCommon.LerpModelVertex(v1, v2, t);
                frontPoly.Vertices.Add(mid);
                backPoly.Vertices.Add(mid);
            }
        }
    }

    private static void FlipModelVertex(ref ModelVertex vertex)
    {
        vertex.Normal = -vertex.Normal;
        vertex.Tangent = new Vector4f(-vertex.Tangent.X, -vertex.Tangent.Y, -vertex.Tangent.Z, vertex.Tangent.W);
        if (vertex.HasBinormal())
            vertex.Binormal = -vertex.Binormal;
    }
}
