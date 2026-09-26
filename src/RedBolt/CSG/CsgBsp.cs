// Copyright (c) 2026 RedBolt and contributors.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

namespace RedBolt.CSG;

using System;
using System.Collections.Generic;

using SystemMath = global::System.Math;

using RedBolt.Graphics;
using RedBolt.Math;

public enum CsgClipMode
{
    ClipToInside,
    ClipToOutside
}

public sealed class BspNode
{
    public Plane Plane;
    public List<CsgPolygon> Polygons = [];
    public BspNode? Front;
    public BspNode? Back;

    public void Build(List<CsgPolygon> polygons, float epsilon)
    {
        if (polygons.Count == 0)
            return;

        var stack = new Stack<BuildFrame>();
        stack.Push(new BuildFrame(this, polygons));

        while (stack.Count != 0)
        {
            var frame = stack.Pop();
            var current = frame.Node;
            var currentPolys = frame.Polygons;

            if (currentPolys.Count == 0)
                continue;

            // Termination heuristic: if there is only one polygon, further splitting cannot improve
            // the BSP and may lead to pathological recursion on degenerate inputs.
            if (currentPolys.Count == 1)
            {
                // `ClipImpl` always classifies polygons against `Plane`, even for leaf nodes,
                // so ensure it's initialized to a meaningful plane.
                var poly = currentPolys[0];
                if (poly.Vertices.Count >= 3 && poly.Plane.Normal.LengthSquared <= MathDefs.Epsilon)
                {
                    poly.Plane = new Plane(poly.Vertices[0].GetPosition(), poly.Vertices[1].GetPosition(),
                        poly.Vertices[2].GetPosition());
                }

                current.Plane = poly.Plane;
                current.Polygons.Add(poly);
                continue;
            }

            current.Plane = PickSplittingPlane(currentPolys, epsilon);

            var frontList = new List<CsgPolygon>(currentPolys.Count + 2);
            var backList = new List<CsgPolygon>(currentPolys.Count + 2);
            foreach (var poly in currentPolys)
            {
                var frontCount = 0;
                var backCount = 0;
                var coplanarCount = 0;

                foreach (var vertex in poly.Vertices)
                {
                    var distance = current.Plane.Normal.DotProduct(vertex.GetPosition()) + current.Plane.D;
                    if (distance > epsilon)
                        ++frontCount;
                    else if (distance < -epsilon)
                        ++backCount;
                    else
                        ++coplanarCount;
                }

                if (coplanarCount == poly.Vertices.Count)
                    current.Polygons.Add(poly);
                else if (frontCount > 0 && backCount == 0)
                    frontList.Add(poly);
                else if (backCount > 0 && frontCount == 0)
                    backList.Add(poly);
                else
                {
                    poly.Split(current.Plane, out var front, out var back, epsilon);
                    if (front.Vertices.Count != 0)
                        frontList.Add(front);
                    if (back.Vertices.Count != 0)
                        backList.Add(back);
                }
            }

            // Preserve recursive build order: build front subtree first, then back subtree.
            // With LIFO stack, push back work first so front is processed next.
            if (backList.Count != 0)
            {
                current.Back = new BspNode();
                stack.Push(new BuildFrame(current.Back, backList));
            }

            if (frontList.Count != 0)
            {
                current.Front = new BspNode();
                stack.Push(new BuildFrame(current.Front, frontList));
            }
        }
    }

    public List<CsgPolygon> Clip(IReadOnlyList<CsgPolygon> polygons, CsgClipMode mode,
        float epsilon = CsgCommon.DefaultEpsilon)
    {
        return ClipImpl(polygons, mode, epsilon, false);
    }

    public List<CsgPolygon> Clip(List<CsgPolygon> polygons, CsgClipMode mode,
        float epsilon = CsgCommon.DefaultEpsilon)
    {
        return ClipImpl(polygons, mode, epsilon, true);
    }

    public void Invert()
    {
        Plane.Normal = -Plane.Normal;
        Plane.D = -Plane.D;

        foreach (var poly in Polygons)
            poly.Flip();

        Front?.Invert();
        Back?.Invert();

        (Front, Back) = (Back, Front);
    }

    private readonly record struct BuildFrame(BspNode Node, List<CsgPolygon> Polygons);

    private readonly record struct ClipFrame(BspNode Node, List<CsgPolygon> Polygons);

    private List<CsgPolygon> ClipImpl(IReadOnlyList<CsgPolygon> polygons, CsgClipMode mode, float epsilon, bool move)
    {
        if (polygons.Count == 0)
            return [];

        // Iterative DFS that preserves the exact output concatenation order of the previous
        // recursive implementation: front subtree results first, then back subtree results.
        var rootPolygons = move ? (List<CsgPolygon>)polygons : ClonePolygons(polygons);
        var stack = new Stack<ClipFrame>();
        stack.Push(new ClipFrame(this, rootPolygons));

        var result = new List<CsgPolygon>(rootPolygons.Count);
        while (stack.Count != 0)
        {
            var frame = stack.Pop();
            if (frame.Polygons.Count == 0)
                continue;

            var current = frame.Node;
            var frontList = new List<CsgPolygon>(frame.Polygons.Count + 2);
            var backList = new List<CsgPolygon>(frame.Polygons.Count + 2);
            foreach (var poly in frame.Polygons)
            {
                var frontCount = 0;
                var backCount = 0;

                foreach (var vertex in poly.Vertices)
                {
                    var distance = current.Plane.Normal.DotProduct(vertex.GetPosition()) + current.Plane.D;
                    if (distance > epsilon)
                        ++frontCount;
                    else if (distance < -epsilon)
                        ++backCount;
                }

                if (frontCount == 0 && backCount == 0)
                {
                    var list = current.Plane.Normal.DotProduct(poly.Plane.Normal) > 0.0f ? frontList : backList;
                    list.Add(poly);
                }
                else if (frontCount > 0 && backCount == 0)
                    frontList.Add(poly);
                else if (backCount > 0 && frontCount == 0)
                    backList.Add(poly);
                else
                {
                    poly.Split(current.Plane, out var front, out var back, epsilon);
                    if (front.Vertices.Count != 0)
                        frontList.Add(front);
                    if (back.Vertices.Count != 0)
                        backList.Add(back);
                }
            }

            // To preserve recursive order (front results then back results) using LIFO stack,
            // we push the back work first so the front is processed next.
            if (current.Back is not null)
                stack.Push(new ClipFrame(current.Back, backList));
            else if (mode == CsgClipMode.ClipToInside)
                result.AddRange(backList);

            if (current.Front is not null)
                stack.Push(new ClipFrame(current.Front, frontList));
            else if (mode == CsgClipMode.ClipToOutside)
                result.AddRange(frontList);
        }

        return result;
    }

    private static float CalculatePlaneScore(Plane plane, IReadOnlyList<CsgPolygon> polygons, float epsilon)
    {
        const float splitWeight = 1.0f;
        const float balanceWeight = 0.5f;
        const float coplanarWeight = 5.0f;

        var frontCount = 0;
        var backCount = 0;
        var splitCount = 0;
        var nearCoplanarSplitCount = 0;

        const int maxPolysToCheck = 100;
        var stride = polygons.Count > maxPolysToCheck ? polygons.Count / maxPolysToCheck : 1;
        for (var i = 0; i < polygons.Count; i += stride)
        {
            var poly = polygons[i];
            var vertexFront = 0;
            var vertexBack = 0;

            foreach (var vertex in poly.Vertices)
            {
                var distance = plane.Normal.DotProduct(vertex.GetPosition()) + plane.D;
                if (distance > epsilon)
                    ++vertexFront;
                else if (distance < -epsilon)
                    ++vertexBack;
            }

            if (vertexFront > 0 && vertexBack > 0)
            {
                ++splitCount;
                if (MathF.Abs(plane.Normal.DotProduct(poly.Plane.Normal)) > 0.99f)
                    ++nearCoplanarSplitCount;
            }
            else if (vertexFront > 0)
                ++frontCount;
            else if (vertexBack > 0)
                ++backCount;
        }

        return splitWeight * splitCount + balanceWeight * MathF.Abs(frontCount - backCount)
            + coplanarWeight * nearCoplanarSplitCount;
    }

    private static Plane PickSplittingPlane(IReadOnlyList<CsgPolygon> polygons, float epsilon)
    {
        if (polygons.Count == 0)
            return Plane.Up;

        var bestPlane = polygons[polygons.Count / 2].Plane;
        var bestScore = 1e30f;
        var numCandidates = SystemMath.Clamp(polygons.Count / 3, 1, 5);
        var step = polygons.Count / numCandidates;
        if (step == 0)
            step = 1;

        for (var i = 0; i < polygons.Count; i += step)
        {
            var candidate = polygons[i].Plane;
            var score = CalculatePlaneScore(candidate, polygons, epsilon);
            if (score < bestScore)
            {
                bestScore = score;
                bestPlane = candidate;
            }
        }
        return bestPlane;
    }

    private static List<CsgPolygon> ClonePolygons(IReadOnlyList<CsgPolygon> polygons)
    {
        var result = new List<CsgPolygon>(polygons.Count);
        foreach (var polygon in polygons)
            result.Add(polygon.CloneShared());
        return result;
    }
}

public sealed class CsgBsp
{
    private BspNode? _root;

    /// <summary>Return true if BSP tree is built (non-empty).</summary>
    public bool IsBuilt => _root is not null;

    public void Build(List<CsgPolygon> polygons, float epsilon = CsgCommon.DefaultEpsilon)
    {
        if (polygons.Count == 0)
            return;

        var boundingBox = new BoundingBox();
        foreach (var poly in polygons)
        {
            foreach (var vertex in poly.Vertices)
                boundingBox.Merge(vertex.GetPosition());
        }

        var size = boundingBox.Size;
        var min = boundingBox.Min;
        var scale = new Vector3f(
            size.X > 0.0f ? 1.0f / size.X : 0.0f,
            size.Y > 0.0f ? 1.0f / size.Y : 0.0f,
            size.Z > 0.0f ? 1.0f / size.Z : 0.0f);

        var keyed = new List<KeyedPolygon>(polygons.Count);
        foreach (var poly in polygons)
        {
            var center = GetPolygonCentroid(poly);
            keyed.Add(new KeyedPolygon(Morton3D((center - min) * scale), poly));
        }
        keyed.Sort(static (left, right) => left.Key.CompareTo(right.Key));

        polygons.Clear();
        polygons.EnsureCapacity(keyed.Count);
        foreach (var item in keyed)
            polygons.Add(item.Polygon);

        _root = new BspNode();
        _root.Build(polygons, epsilon);
    }

    public List<CsgPolygon> Clip(IReadOnlyList<CsgPolygon> polygons, CsgClipMode mode,
        float epsilon = CsgCommon.DefaultEpsilon)
    {
        if (_root is null)
        {
            if (mode == CsgClipMode.ClipToInside)
                return [];
            return ClonePolygons(polygons);
        }

        return _root.Clip(polygons, mode, epsilon);
    }

    public List<CsgPolygon> Clip(List<CsgPolygon> polygons, CsgClipMode mode,
        float epsilon = CsgCommon.DefaultEpsilon)
    {
        if (_root is null)
        {
            if (mode == CsgClipMode.ClipToInside)
                return [];
            return polygons;
        }

        return _root.Clip(polygons, mode, epsilon);
    }

    public void Invert()
    {
        _root?.Invert();
    }

    public List<CsgPolygon> AllPolygons()
    {
        var result = new List<CsgPolygon>();
        CollectPolygons(_root, result);
        return result;
    }

    public List<CsgPolygon> AllPolygonsInverted()
    {
        var result = AllPolygons();
        foreach (var polygon in result)
            polygon.Flip();
        return result;
    }

    private readonly record struct KeyedPolygon(uint Key, CsgPolygon Polygon);

    private static Vector3f GetPolygonCentroid(CsgPolygon poly)
    {
        var center = Vector3f.Zero;
        foreach (var vertex in poly.Vertices)
            center += vertex.GetPosition();
        if (poly.Vertices.Count != 0)
            center /= poly.Vertices.Count;
        return center;
    }

    private static uint ExpandBits(uint value)
    {
        value = unchecked(value * 0x00010001u) & 0xFF0000FFu;
        value = unchecked(value * 0x00000101u) & 0x0F00F00Fu;
        value = unchecked(value * 0x00000011u) & 0xC30C30C3u;
        value = unchecked(value * 0x00000005u) & 0x49249249u;
        return value;
    }

    private static uint Morton3D(Vector3f position)
    {
        var x = SystemMath.Clamp(position.X, 0.0f, 1.0f);
        var y = SystemMath.Clamp(position.Y, 0.0f, 1.0f);
        var z = SystemMath.Clamp(position.Z, 0.0f, 1.0f);

        var xx = ExpandBits((uint)(x * 1023.0f));
        var yy = ExpandBits((uint)(y * 1023.0f));
        var zz = ExpandBits((uint)(z * 1023.0f));
        return unchecked(xx * 4 + yy * 2 + zz);
    }

    private static void CollectPolygons(BspNode? node, List<CsgPolygon> result)
    {
        if (node is null)
            return;
        foreach (var polygon in node.Polygons)
            result.Add(polygon.CloneShared());
        CollectPolygons(node.Front, result);
        CollectPolygons(node.Back, result);
    }

    private static List<CsgPolygon> ClonePolygons(IReadOnlyList<CsgPolygon> polygons)
    {
        var result = new List<CsgPolygon>(polygons.Count);
        foreach (var polygon in polygons)
            result.Add(polygon.CloneShared());
        return result;
    }
}
