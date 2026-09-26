// Copyright (c) 2026 RedBolt and contributors.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

namespace RedBolt.CSG;

using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.InteropServices;
using System.Runtime.Intrinsics;

using RedBolt.Graphics;
using RedBolt.Math;

using Vec3 = System.Numerics.Vector3;

/// <remarks>
/// Booleans corefine the operands: the intersection curve is computed once and both operands are split along it
/// at the same vertices, so the result is closed by construction and needs no repair.
/// <list type="bullet">
/// <item>Every decision whether an edge of one operand crosses a triangle of the other is an exact orientation
/// test on vertices snapped to a fine grid. Ties, such as coplanar faces or a vertex on a plane, are broken by a
/// symbolic perturbation that displaces the second operand by (ε, ε², ε³), so every decision is consistent with
/// every other.</item>
/// <item>A crossing is identified by its edge and triangle, and its vertex is shared by every face that touches
/// either. Faces on both sides of an edge therefore insert the same vertices in the same order.</item>
/// <item>Each face is divided along the curve by splicing vertex lists; geometry is only consulted to place
/// closed curves that lie inside a face.</item>
/// <item>Patches alternate between inside and outside across the curve, so one ray cast classifies a whole
/// connected surface.</item>
/// </list>
/// </remarks>
public sealed partial class CsgSolid
{
    /// <summary>
    /// Grid for exact orientation tests, 2^-24 units: as fine as float spacing for coordinates of magnitude 0.5
    /// and above, so vertices move at most a few hundredths of a micrometer. Coordinates must stay below 2^16.
    /// </summary>
    private const double GridScale = 1 << 24;

    private readonly record struct GridPoint(long X, long Y, long Z)
    {
        public static GridPoint From(Vec3 point) => new(
            (long)System.Math.Round(point.X * GridScale),
            (long)System.Math.Round(point.Y * GridScale),
            (long)System.Math.Round(point.Z * GridScale));
    }

    /// <summary>A crossing of an edge of one operand through a triangle of the other.</summary>
    private readonly record struct CrossingKey(bool EdgeInSecond, int Low, int High, int Triangle);

    private struct Crossing
    {
        public Vec3 Position;
        // Parameter along the edge from its lower to its higher welded id.
        public double T;
        public CrossingKey Key;
        // Exact plane volumes of the edge ends against the crossed triangle, and that triangle's normal, which
        // order crossings on one edge exactly.
        public Int128 LowVolume;
        public Int128 HighVolume;
        public (Int128 X, Int128 Y, Int128 Z) Normal;
    }

    /// <summary>Welded view of an operand: ids per face vertex and per triangle corner, and grid positions.</summary>
    private sealed class Mesh
    {
        public readonly CsgSolid Solid;
        public readonly WeldedLoops Loops;
        public readonly bool IsSecond;
        public readonly int Offset;
        public readonly int[] TriangleFace;
        public readonly int[] TriangleIds;
        public readonly GridPoint[] Grid;

        public Mesh(CsgSolid solid, float epsilon, bool isSecond, int offset)
        {
            Solid = solid;
            IsSecond = isSecond;
            Offset = offset;
            // Operands share vertices at identical positions, so only identical positions are welded; nearby but
            // distinct vertices of an earlier result keep their own ids and positions.
            Loops = solid.ExactWeld;
            var triangleCount = solid._triangleStart[^1];
            TriangleFace = new int[triangleCount];
            TriangleIds = new int[triangleCount * 3];
            for (var face = 0; face < solid._faces.Length; ++face)
            {
                var start = Loops.VertexStart[face];
                for (var t = solid._triangleStart[face]; t < solid._triangleStart[face + 1]; ++t)
                {
                    TriangleFace[t] = face;
                    for (var corner = 0; corner < 3; ++corner)
                        TriangleIds[t * 3 + corner] = Loops.Ids[start + solid._triangleVertices[t * 3 + corner]];
                }
            }
            Grid = new GridPoint[Loops.Welder.Count];
            for (var id = 0; id < Grid.Length; ++id)
                Grid[id] = GridPoint.From(Loops.Welder.Positions[id]);
        }

        public int VertexCount => Grid.Length;

        public bool IsCollapsed(int triangle)
        {
            var a = TriangleIds[triangle * 3];
            var b = TriangleIds[triangle * 3 + 1];
            var c = TriangleIds[triangle * 3 + 2];
            return a == b || b == c || c == a;
        }
    }

    /// <summary>Exact orientation with the symbolic displacement of the second operand.</summary>
    private static class Exact
    {
        /// <summary>
        /// Sign of (d - a)·((b - a) × (c - a)): positive when d lies on the side of plane (a, b, c) that its normal
        /// (b - a) × (c - a) points to. Bit i of <paramref name="second"/> marks argument i as a vertex of the
        /// second operand. Zero only for a degenerate configuration that the displacement cannot resolve.
        /// </summary>
        public static int Orient(in GridPoint a, in GridPoint b, in GridPoint c, in GridPoint d, int second)
        {
            long ux = b.X - a.X, uy = b.Y - a.Y, uz = b.Z - a.Z;
            long vx = c.X - a.X, vy = c.Y - a.Y, vz = c.Z - a.Z;
            long wx = d.X - a.X, wy = d.Y - a.Y, wz = d.Z - a.Z;

            // Coordinates below 2^40 make every difference exact in double, so only the products round.
            double m1 = (double)vy * wz - (double)vz * wy;
            double m2 = (double)vz * wx - (double)vx * wz;
            double m3 = (double)vx * wy - (double)vy * wx;
            var det = ux * m1 + uy * m2 + uz * m3;
            var permanent = System.Math.Abs((double)ux) * (System.Math.Abs((double)vy * wz) + System.Math.Abs((double)vz * wy))
                + System.Math.Abs((double)uy) * (System.Math.Abs((double)vz * wx) + System.Math.Abs((double)vx * wz))
                + System.Math.Abs((double)uz) * (System.Math.Abs((double)vx * wy) + System.Math.Abs((double)vy * wx));
            if (System.Math.Abs(det) > permanent * 1e-12)
                return det > 0.0 ? 1 : -1;

            var exact = (Int128)ux * ((Int128)vy * wz - (Int128)vz * wy)
                + (Int128)uy * ((Int128)vz * wx - (Int128)vx * wz)
                + (Int128)uz * ((Int128)vx * wy - (Int128)vy * wx);
            if (exact != 0)
                return exact > 0 ? 1 : -1;
            if (second == 0 || second == 0b1111)
                return 0;

            // Displacing the marked points by δ adds δ·G to first order, where G sums their gradients. With
            // δ = (ε, ε², ε³) the first nonzero component of G decides; higher orders vanish for a translation.
            Int128 gx = 0, gy = 0, gz = 0;
            // Gradients: ∂/∂b = v × w, ∂/∂c = w × u, ∂/∂d = u × v, and ∂/∂a is minus their sum.
            Span<(long X, long Y, long Z)> pairs = [(vx, vy, vz), (wx, wy, wz), (ux, uy, uz)];
            for (var i = 1; i <= 3; ++i)
            {
                var (px, py, pz) = pairs[i - 1];
                var (qx, qy, qz) = pairs[i % 3];
                var cx = (Int128)py * qz - (Int128)pz * qy;
                var cy = (Int128)pz * qx - (Int128)px * qz;
                var cz = (Int128)px * qy - (Int128)py * qx;
                var weight = ((second >> i) & 1) - (second & 1);
                gx += cx * weight;
                gy += cy * weight;
                gz += cz * weight;
            }
            if (gx != 0)
                return gx > 0 ? 1 : -1;
            if (gy != 0)
                return gy > 0 ? 1 : -1;
            return gz > 0 ? 1 : gz < 0 ? -1 : 0;
        }

        /// <summary>
        /// Unperturbed (d - a)·((b - a) × (c - a)), computed exactly and rounded once, for placing and ordering
        /// crossing points. Crossings a hair apart on one edge keep their true order.
        /// </summary>
        public static Int128 Volume(in GridPoint a, in GridPoint b, in GridPoint c, in GridPoint d)
        {
            var n = Normal(a, b, c);
            return (Int128)(d.X - a.X) * n.X + (Int128)(d.Y - a.Y) * n.Y + (Int128)(d.Z - a.Z) * n.Z;
        }

        /// <summary>(b - a) × (c - a), exactly.</summary>
        public static (Int128 X, Int128 Y, Int128 Z) Normal(in GridPoint a, in GridPoint b, in GridPoint c)
        {
            long ux = b.X - a.X, uy = b.Y - a.Y, uz = b.Z - a.Z;
            long vx = c.X - a.X, vy = c.Y - a.Y, vz = c.Z - a.Z;
            return ((Int128)uy * vz - (Int128)uz * vy, (Int128)uz * vx - (Int128)ux * vz,
                (Int128)ux * vy - (Int128)uy * vx);
        }

        /// <summary>
        /// Order two crossings of one edge by their parameter from its lower to its higher end, exactly and with
        /// the displacement of the second operand breaking ties, as every other decision does. The parameter of
        /// a crossing is L / (L - H) for the plane volumes L and H of the edge ends; displacing the edge by δ, or
        /// the crossed triangle by -δ, moves it by ±δ·N / (L - H) for the triangle normal N.
        /// </summary>
        public static int CompareOnEdge(in Crossing x, in Crossing y)
        {
            var dx = x.LowVolume - x.HighVolume;
            var dy = y.LowVolume - y.HighVolume;
            // t_x - t_y = (Lx·Dy - Ly·Dx) / (Dx·Dy)
            var numerator = (System.Numerics.BigInteger)x.LowVolume * (System.Numerics.BigInteger)dy
                - (System.Numerics.BigInteger)y.LowVolume * (System.Numerics.BigInteger)dx;
            var denominatorSign = Int128.Sign(dx) * Int128.Sign(dy);
            if (!numerator.IsZero)
                return numerator.Sign * denominatorSign;
            // Tie: compare δ·(Nx / Dx - Ny / Dy), component by component.
            var shift = x.Key.EdgeInSecond ? 1 : -1;
            Span<(Int128 Nx, Int128 Ny)> components = [(x.Normal.X, y.Normal.X), (x.Normal.Y, y.Normal.Y),
                (x.Normal.Z, y.Normal.Z)];
            foreach (var (nx, ny) in components)
            {
                var difference = (System.Numerics.BigInteger)nx * (System.Numerics.BigInteger)dy
                    - (System.Numerics.BigInteger)ny * (System.Numerics.BigInteger)dx;
                if (!difference.IsZero)
                    return difference.Sign * denominatorSign * shift;
            }
            return 0;
        }

        /// <summary>
        /// Whether edge (e0, e1) crosses the interior of triangle (t0, t1, t2) of the other operand, and whether e1
        /// lies on the side its normal points to. A degenerate triangle is never crossed; the displacement lets a
        /// neighboring triangle claim the crossing instead.
        /// </summary>
        public static bool Crosses(in GridPoint e0, in GridPoint e1, in GridPoint t0, in GridPoint t1,
            in GridPoint t2, bool edgeInSecond, out bool e1Above)
        {
            var sidesMask = edgeInSecond ? 0b1000 : 0b0111;
            var s0 = Orient(t0, t1, t2, e0, sidesMask);
            var s1 = Orient(t0, t1, t2, e1, sidesMask);
            e1Above = s1 > 0;
            if (s0 == 0 || s1 == 0 || s0 == s1)
                return false;
            return PassesInside(e0, e1, t0, t1, t2, edgeInSecond);
        }

        /// <summary>
        /// Whether the line through an edge that crosses the plane of a triangle of the other operand passes inside
        /// that triangle.
        /// </summary>
        public static bool PassesInside(in GridPoint e0, in GridPoint e1, in GridPoint t0, in GridPoint t1,
            in GridPoint t2, bool edgeInSecond)
        {
            var linesMask = edgeInSecond ? 0b0011 : 0b1100;
            if (Vector256.IsHardwareAccelerated)
            {
                // Lane k holds Orient(e0, e1, t_k, t_k+1); undecided lanes take the exact path when needed.
                var vx = Vector256.Create((double)(t0.X - e0.X), t1.X - e0.X, t2.X - e0.X, 0.0);
                var vy = Vector256.Create((double)(t0.Y - e0.Y), t1.Y - e0.Y, t2.Y - e0.Y, 0.0);
                var vz = Vector256.Create((double)(t0.Z - e0.Z), t1.Z - e0.Z, t2.Z - e0.Z, 0.0);
                var decided = Filter(Vector256.Create((double)(e1.X - e0.X)), Vector256.Create((double)(e1.Y - e0.Y)),
                    Vector256.Create((double)(e1.Z - e0.Z)), vx, vy, vz, Rotate(vx), Rotate(vy), Rotate(vz),
                    out var det);
                var s0 = (decided & 1) != 0 ? Sign(det[0]) : Orient(e0, e1, t0, t1, linesMask);
                if (s0 == 0)
                    return false;
                var s1 = (decided & 2) != 0 ? Sign(det[1]) : Orient(e0, e1, t1, t2, linesMask);
                if (s1 != s0)
                    return false;
                return ((decided & 4) != 0 ? Sign(det[2]) : Orient(e0, e1, t2, t0, linesMask)) == s0;
            }
            var o0 = Orient(e0, e1, t0, t1, linesMask);
            var o1 = Orient(e0, e1, t1, t2, linesMask);
            if (o0 == 0 || o0 != o1)
                return false;
            return Orient(e0, e1, t2, t0, linesMask) == o0;
        }

        /// <summary>
        /// The floating-point filter of <see cref="Orient"/> for up to four determinants u·(v × w) at once, lane by
        /// lane with the same operations, so a lane decides exactly when the scalar filter would. Returns a bit per
        /// decided lane; <paramref name="det"/> then carries its sign.
        /// </summary>
        public static int Filter(Vector256<double> ux, Vector256<double> uy, Vector256<double> uz,
            Vector256<double> vx, Vector256<double> vy, Vector256<double> vz, Vector256<double> wx,
            Vector256<double> wy, Vector256<double> wz, out Vector256<double> det)
        {
            var m1 = vy * wz - vz * wy;
            var m2 = vz * wx - vx * wz;
            var m3 = vx * wy - vy * wx;
            det = ux * m1 + uy * m2 + uz * m3;
            var permanent = Vector256.Abs(ux) * (Vector256.Abs(vy * wz) + Vector256.Abs(vz * wy))
                + Vector256.Abs(uy) * (Vector256.Abs(vz * wx) + Vector256.Abs(vx * wz))
                + Vector256.Abs(uz) * (Vector256.Abs(vx * wy) + Vector256.Abs(vy * wx));
            return (int)Vector256.GreaterThan(Vector256.Abs(det), permanent * 1e-12).ExtractMostSignificantBits()
                & 0b111;
        }

        /// <summary>Lanes (x0, x1, x2, _) as (x1, x2, x0, _).</summary>
        private static Vector256<double> Rotate(Vector256<double> value) =>
            Vector256.Shuffle(value, Vector256.Create(1L, 2L, 0L, 3L));

        public static int Sign(double value) => value > 0.0 ? 1 : -1;
    }

    /// <summary>The intersection of two operands: crossing vertices and the curve segments on each face.</summary>
    private sealed class Corefinement
    {
        public readonly Mesh A;
        public readonly Mesh B;
        public readonly List<Crossing> Crossings = [];
        /// <summary>Curve segments on each triangle of the first and the second operand.</summary>
        public readonly Dictionary<int, List<(int P, int Q)>> SegmentsA = [];
        public readonly Dictionary<int, List<(int P, int Q)>> SegmentsB = [];
        /// <summary>Curve segments by their undirected ends, oriented from entering to exiting along n_A × n_B.</summary>
        public readonly Dictionary<ulong, (int Start, int End)> CurveEdges = new(CsgCommon.PackedKeyComparer.Instance);
        private readonly Dictionary<CrossingKey, int> _crossingIds = [];
        private readonly Dictionary<(bool, int, int), List<int>> _edgeCrossings = [];
        private readonly float _epsilon;

        public float Epsilon => _epsilon;

        public Corefinement(CsgSolid a, CsgSolid b, float epsilon)
        {
            _epsilon = epsilon;
            A = new Mesh(a, epsilon, false, 0);
            B = new Mesh(b, epsilon, true, A.VertexCount);
        }

        /// <summary>Global id of the first crossing; mesh vertices take the ids below it.</summary>
        public int CrossingBase => A.VertexCount + B.VertexCount;

        public int Global(Mesh mesh, int id) => mesh.Offset + id;

        public ref Crossing CrossingAt(int globalId) =>
            ref CollectionsMarshal.AsSpan(Crossings)[globalId - CrossingBase];

        public void IntersectFaces(int faceA, int faceB)
        {
            var solidA = A.Solid;
            var solidB = B.Solid;
            Span<int> found = stackalloc int[6];
            Span<bool> entering = stackalloc bool[6];
            Span<int> sidesA = stackalloc int[3];
            Span<int> sidesB = stackalloc int[3];
            for (var ta = solidA._triangleStart[faceA]; ta < solidA._triangleStart[faceA + 1]; ++ta)
            {
                // A triangle whose corners weld together has no area; its neighbors claim every crossing.
                if (A.IsCollapsed(ta))
                    continue;
                var (minA, maxA) = TriangleBounds(solidA, ta);
                for (var tb = solidB._triangleStart[faceB]; tb < solidB._triangleStart[faceB + 1]; ++tb)
                {
                    if (B.IsCollapsed(tb))
                        continue;
                    var (minB, maxB) = TriangleBounds(solidB, tb);
                    if (Vec3.Max(minA, minB).X > Vec3.Min(maxA, maxB).X + _epsilon
                        || Vec3.Max(minA, minB).Y > Vec3.Min(maxA, maxB).Y + _epsilon
                        || Vec3.Max(minA, minB).Z > Vec3.Min(maxA, maxB).Z + _epsilon)
                        continue;
                    // Sides of each triangle's corners against the other's plane. When all corners of either
                    // triangle lie on one side, the triangles cannot meet.
                    Sides(A, ta, B, tb, sidesA);
                    if (sidesA[0] != 0 && sidesA[0] == sidesA[1] && sidesA[1] == sidesA[2])
                        continue;
                    Sides(B, tb, A, ta, sidesB);
                    if (sidesB[0] != 0 && sidesB[0] == sidesB[1] && sidesB[1] == sidesB[2])
                        continue;
                    var count = 0;
                    count = EdgesThrough(A, ta, B, tb, sidesA, found, entering, count);
                    count = EdgesThrough(B, tb, A, ta, sidesB, found, entering, count);
                    if (count == 0)
                        continue;
                    // Exact decisions give two ends; anything else comes from a degenerate triangle.
                    if (count != 2 || found[0] == found[1])
                        continue;
                    var segment = entering[1] && !entering[0] ? (found[1], found[0]) : (found[0], found[1]);
                    Add(SegmentsA, ta, segment);
                    Add(SegmentsB, tb, segment);
                    CurveEdges[UndirectedKey(found[0], found[1])] = segment;
                }
            }
        }

        private static void Add(Dictionary<int, List<(int P, int Q)>> map, int face, (int, int) segment)
        {
            if (!map.TryGetValue(face, out var list))
                map.Add(face, list = []);
            list.Add(segment);
        }

        private static (Vec3 Min, Vec3 Max) TriangleBounds(CsgSolid solid, int triangle)
        {
            var p0 = solid._trianglePositions[triangle * 3];
            var p1 = solid._trianglePositions[triangle * 3 + 1];
            var p2 = solid._trianglePositions[triangle * 3 + 2];
            return (Vec3.Min(Vec3.Min(p0, p1), p2), Vec3.Max(Vec3.Max(p0, p1), p2));
        }

        /// <summary>Record crossings of the edges of <paramref name="triangle"/> through the other triangle.</summary>
        /// <remarks>
        /// Along d = n_A × n_B, the line shared by the two triangle planes enters triangle A across an edge a → b
        /// of A (in winding order) exactly when b lies above B, and enters B across an edge of B exactly when b
        /// lies below A. The intersection segment runs from its entering end to its exiting end.
        /// </remarks>
        private int EdgesThrough(Mesh edges, int triangle, Mesh other, int otherTriangle, ReadOnlySpan<int> sides,
            Span<int> found, Span<bool> entering, int count)
        {
            ref readonly var t0 = ref other.Grid[other.TriangleIds[otherTriangle * 3]];
            ref readonly var t1 = ref other.Grid[other.TriangleIds[otherTriangle * 3 + 1]];
            ref readonly var t2 = ref other.Grid[other.TriangleIds[otherTriangle * 3 + 2]];
            for (var corner = 0; corner < 3; ++corner)
            {
                var next = (corner + 1) % 3;
                // An edge crosses the plane only between corners on opposite sides of it.
                if (sides[corner] == 0 || sides[next] == 0 || sides[corner] == sides[next])
                    continue;
                var u = edges.TriangleIds[triangle * 3 + corner];
                var v = edges.TriangleIds[triangle * 3 + next];
                var low = System.Math.Min(u, v);
                var high = System.Math.Max(u, v);
                if (!Exact.PassesInside(edges.Grid[low], edges.Grid[high], t0, t1, t2, edges.IsSecond))
                    continue;
                var id = CrossingId(new CrossingKey(edges.IsSecond, low, high, otherTriangle), edges, t0, t1, t2);
                if (count < found.Length)
                {
                    var headAbove = sides[next] > 0;
                    found[count] = id;
                    entering[count] = edges.IsSecond ? !headAbove : headAbove;
                }
                ++count;
            }
            return count;
        }

        /// <summary>Side of each corner of a triangle against the plane of a triangle of the other operand.</summary>
        private static void Sides(Mesh corners, int triangle, Mesh other, int otherTriangle, Span<int> sides)
        {
            ref readonly var t0 = ref other.Grid[other.TriangleIds[otherTriangle * 3]];
            ref readonly var t1 = ref other.Grid[other.TriangleIds[otherTriangle * 3 + 1]];
            ref readonly var t2 = ref other.Grid[other.TriangleIds[otherTriangle * 3 + 2]];
            var mask = corners.IsSecond ? 0b1000 : 0b0111;
            if (Vector256.IsHardwareAccelerated)
            {
                // The three corners share the plane, one lane each; undecided lanes take the exact path.
                var ids = corners.TriangleIds.AsSpan(triangle * 3, 3);
                ref readonly var d0 = ref corners.Grid[ids[0]];
                ref readonly var d1 = ref corners.Grid[ids[1]];
                ref readonly var d2 = ref corners.Grid[ids[2]];
                var decided = Exact.Filter(Vector256.Create((double)(t1.X - t0.X)),
                    Vector256.Create((double)(t1.Y - t0.Y)), Vector256.Create((double)(t1.Z - t0.Z)),
                    Vector256.Create((double)(t2.X - t0.X)), Vector256.Create((double)(t2.Y - t0.Y)),
                    Vector256.Create((double)(t2.Z - t0.Z)),
                    Vector256.Create((double)(d0.X - t0.X), d1.X - t0.X, d2.X - t0.X, 0.0),
                    Vector256.Create((double)(d0.Y - t0.Y), d1.Y - t0.Y, d2.Y - t0.Y, 0.0),
                    Vector256.Create((double)(d0.Z - t0.Z), d1.Z - t0.Z, d2.Z - t0.Z, 0.0), out var det);
                sides[0] = (decided & 1) != 0 ? Exact.Sign(det[0]) : Exact.Orient(t0, t1, t2, d0, mask);
                sides[1] = (decided & 2) != 0 ? Exact.Sign(det[1]) : Exact.Orient(t0, t1, t2, d1, mask);
                sides[2] = (decided & 4) != 0 ? Exact.Sign(det[2]) : Exact.Orient(t0, t1, t2, d2, mask);
                return;
            }
            for (var k = 0; k < 3; ++k)
                sides[k] = Exact.Orient(t0, t1, t2, corners.Grid[corners.TriangleIds[triangle * 3 + k]], mask);
        }

        private int CrossingId(CrossingKey key, Mesh edges, in GridPoint t0, in GridPoint t1, in GridPoint t2)
        {
            ref var id = ref CollectionsMarshal.GetValueRefOrAddDefault(_crossingIds, key, out var exists);
            if (exists)
                return id;
            var lowVolume = Exact.Volume(t0, t1, t2, edges.Grid[key.Low]);
            var highVolume = Exact.Volume(t0, t1, t2, edges.Grid[key.High]);
            var t = lowVolume == highVolume
                ? 0.5
                : System.Math.Clamp((double)lowVolume / ((double)lowVolume - (double)highVolume), 0.0, 1.0);
            var low = edges.Loops.Welder.Positions[key.Low];
            var high = edges.Loops.Welder.Positions[key.High];
            id = CrossingBase + Crossings.Count;
            Crossings.Add(new Crossing
            {
                Position = low + (high - low) * (float)t, T = t, Key = key, LowVolume = lowVolume,
                HighVolume = highVolume, Normal = Exact.Normal(t0, t1, t2)
            });
            var edgeKey = (key.EdgeInSecond, key.Low, key.High);
            if (!_edgeCrossings.TryGetValue(edgeKey, out var list))
                _edgeCrossings.Add(edgeKey, list = []);
            list.Add(id);
            return id;
        }

        /// <summary>Crossings on an edge of the mesh, ordered from <paramref name="from"/> to <paramref name="to"/>.</summary>
        public void EdgeCrossings(Mesh mesh, int from, int to, List<int> output)
        {
            output.Clear();
            if (!_edgeCrossings.TryGetValue((mesh.IsSecond, System.Math.Min(from, to), System.Math.Max(from, to)),
                    out var list))
                return;
            output.AddRange(list);
            var crossings = Crossings;
            var baseId = CrossingBase;
            output.Sort((x, y) =>
            {
                var cx = crossings[x - baseId];
                var cy = crossings[y - baseId];
                // Doubles separate all but nearly coincident crossings; those are ordered exactly.
                if (System.Math.Abs(cx.T - cy.T) > 1e-9)
                    return cx.T.CompareTo(cy.T);
                var order = Exact.CompareOnEdge(cx, cy);
                return order != 0 ? order : x.CompareTo(y);
            });
            if (from > to)
                output.Reverse();
        }
    }

    private static ulong UndirectedKey(int a, int b) => a < b ? DirectedKey(a, b) : DirectedKey(b, a);

    private static ulong DirectedKey(int from, int to) => (ulong)(uint)from << 32 | (uint)to;

    /// <summary>A face of the contact region after corefinement, as a loop of global vertex ids.</summary>
    private struct Element
    {
        public Element()
        {
        }

        // Null for an unsplit face, whose loop is read from its mesh.
        public int[]? Loop;
        public required CsgPolygon Polygon;
        // Global id of every polygon vertex for a region; the loop drops repeated ids, this does not. Null for an
        // unsplit face, whose ids are read from its solid.
        public int[]? Ids;
        // The face an unsplit element is, so the result copies its triangles; -1 for a region of a split triangle.
        public int SourceFace = -1;
        // A region of a split triangle; only these border the curve, which runs inside triangles.
        public bool Split;
    }

    /// <summary>A face the boolean keeps: its polygon, its loop and vertex ids, and where it came from.</summary>
    /// <remarks>Only regions carry their loop and ids; an unsplit face reads them from its origin.</remarks>
    private readonly record struct KeptFace(CsgPolygon Polygon, int[]? Loop, int[]? Ids, FaceOrigin? Origin);

    /// <summary>
    /// Divide a triangle of a face along its curve segments into regions of global vertex ids that keep the face
    /// orientation. Working per triangle keeps every region inside one triangle, so regions stay small and simple
    /// however often a face is cut; the crossings on a diagonal are shared by the triangles on both sides of it.
    /// </summary>
    /// <remarks>
    /// Regions are traced over directed edges: boundary pieces in the face direction and curve segments in both
    /// directions, each used by exactly one region, so the regions of a face always close up against each other
    /// and against neighboring faces. Where a boundary edge reaches the curve the trace turns into the face, and
    /// where the curve reaches the boundary it continues along it; only a vertex joining more than two curve
    /// segments needs angles. Closed curves inside the face yield an island and a hole, and each hole is joined
    /// to the region around it by two bridges.
    /// </remarks>
    private static bool SplitTriangle(Corefinement core, Mesh mesh, int face, int triangle,
        List<(int P, int Q)> rawSegments, List<List<int>> regions)
    {
        var boundary = new List<int>(12);
        var edgePoints = new List<int>();
        for (var i = 0; i < 3; ++i)
        {
            var from = mesh.TriangleIds[triangle * 3 + i];
            var to = mesh.TriangleIds[triangle * 3 + (i + 1) % 3];
            boundary.Add(core.Global(mesh, from));
            core.EdgeCrossings(mesh, from, to, edgePoints);
            boundary.AddRange(edgePoints);
        }
        var segments = rawSegments;

        // A face may visit a vertex twice, so a vertex can have more than one boundary edge leaving it.
        var boundaryOut = new Dictionary<int, List<int>>(boundary.Count);
        var boundaryEdges = new HashSet<(int, int)>(boundary.Count);
        for (var i = 0; i < boundary.Count; ++i)
        {
            var (from, to) = (boundary[i], boundary[(i + 1) % boundary.Count]);
            if (!boundaryOut.TryGetValue(from, out var outs))
                boundaryOut.Add(from, outs = []);
            outs.Add(to);
            boundaryEdges.Add((from, to));
        }
        var curve = new Dictionary<int, List<int>>();
        foreach (var (p, q) in segments)
        {
            Link(p, q);
            Link(q, p);
        }

        CsgCommon.MakeAxesFromNormal(mesh.Solid._faces[face].Plane.Normal, out var axisU, out var axisV);
        var visited = new HashSet<(int, int)>();
        var cycles = new List<List<int>>();
        var closedCycles = new List<List<int>>();
        var holes = new List<List<int>>();
        var edgeCount = boundaryEdges.Count + segments.Count * 2;
        for (var i = 0; i < boundary.Count; ++i)
            Trace(boundary[i], boundary[(i + 1) % boundary.Count], true);
        foreach (var (p, q) in segments)
        {
            Trace(p, q, false);
            Trace(q, p, false);
        }
        if (visited.Count != edgeCount)
        {
            return false;
        }

        // A closed curve inside the triangle is traced once in each direction: the turn that encloses more area is
        // the island and the other is the hole around it, even when rounding gives both the same sign.
        foreach (var group in closedCycles.GroupBy(cycle => string.Join(",", cycle.Order())))
        {
            var members = group.ToList();
            if (members.Count == 2)
            {
                var (island, hole) = SignedArea(members[0]) >= SignedArea(members[1])
                    ? (members[0], members[1])
                    : (members[1], members[0]);
                cycles.Add(island);
                holes.Add(hole);
                continue;
            }
            foreach (var cycle in members)
                (SignedArea(cycle) > 0.0 ? cycles : holes).Add(cycle);
        }

        regions.Clear();
        regions.AddRange(cycles);
        if (holes.Count != 0)
            PlaceHoles(core, face, axisU, axisV, holes, regions);
        return true;

        void Link(int from, int to)
        {
            if (!curve.TryGetValue(from, out var list))
                curve.Add(from, list = []);
            list.Add(to);
        }

        void Trace(int from, int to, bool onBoundary)
        {
            if (visited.Contains((from, to)))
                return;
            var cycle = new List<int>();
            var touchesBoundary = false;
            var (u, v) = (from, to);
            while (visited.Add((u, v)))
            {
                cycle.Add(u);
                touchesBoundary |= boundaryEdges.Contains((u, v));
                (u, v) = (v, Next(u, v));
                if (cycle.Count > edgeCount)
                    break;
            }
            if (cycle.Count < 3)
                return;
            (touchesBoundary ? cycles : closedCycles).Add(cycle);
        }

        // The edge that follows u -> v around the region on its left.
        int Next(int u, int v)
        {
            curve.TryGetValue(v, out var branches);
            if (boundaryOut.TryGetValue(v, out var outs))
            {
                if (outs.Count == 1 && branches is null)
                    return outs[0];
                if (outs.Count == 1 && branches!.Count == 1)
                    return boundaryEdges.Contains((u, v)) ? branches[0] : outs[0];
                return Turn(u, v, branches is null ? outs : [.. outs, .. branches]);
            }
            if (branches!.Count == 2)
                return branches[0] == u ? branches[1] : branches[0];
            return Turn(u, v, branches);
        }

        // Among the outgoing directions at v, the first one clockwise from the way back to u.
        int Turn(int u, int v, List<int> outgoing)
        {
            var origin = Project(v);
            var back = Angle(Project(u) - origin);
            var best = -1;
            var bestSweep = double.PositiveInfinity;
            foreach (var w in outgoing)
            {
                if (w == u && outgoing.Count > 1)
                    continue;
                var sweep = back - Angle(Project(w) - origin);
                while (sweep <= 0.0)
                    sweep += System.Math.Tau;
                if (sweep < bestSweep)
                {
                    bestSweep = sweep;
                    best = w;
                }
            }
            return best >= 0 ? best : outgoing[0];
        }

        Vector2f Project(int id)
        {
            var p = ToVector3f(PointPosition(core, id));
            return new Vector2f(p.DotProduct(axisU), p.DotProduct(axisV));
        }

        static double Angle(Vector2f d) => System.Math.Atan2(d.Y, d.X);

        // Twice the signed area in the face plane, in double and relative to the first vertex, so even a curve a
        // few micrometers across has a reliable sign.
        double SignedArea(List<int> cycle)
        {
            var origin = PointPosition(core, cycle[0]);
            double Along(Vec3 p, Vector3f axis) =>
                ((double)p.X - origin.X) * axis.X + ((double)p.Y - origin.Y) * axis.Y + ((double)p.Z - origin.Z) * axis.Z;
            var area = 0.0;
            for (var i = 0; i < cycle.Count; ++i)
            {
                var p = PointPosition(core, cycle[i]);
                var q = PointPosition(core, cycle[(i + 1) % cycle.Count]);
                area += Along(p, axisU) * Along(q, axisV) - Along(q, axisU) * Along(p, axisV);
            }
            return area;
        }
    }

    /// <summary>
    /// Join each hole left by a closed curve to the region around it with two bridges, which divides the region
    /// into two simple polygons. Holes run clockwise and regions counterclockwise, so both pieces keep the face
    /// orientation. When no pair of bridges is clear, one bridge makes a region that visits it twice. Geometry is
    /// evaluated in double relative to the hole, since a curve can be a few micrometers across.
    /// </summary>
    private static void PlaceHoles(Corefinement core, int face, Vector3f axisU, Vector3f axisV,
        List<List<int>> holes, List<List<int>> regions)
    {
        var origin = PointPosition(core, holes[0][0]);
        (double X, double Y) Project(int id)
        {
            var p = PointPosition(core, id);
            double dx = (double)p.X - origin.X, dy = (double)p.Y - origin.Y, dz = (double)p.Z - origin.Z;
            return (dx * axisU.X + dy * axisU.Y + dz * axisU.Z, dx * axisV.X + dy * axisV.Y + dz * axisV.Z);
        }

        // Outer holes first, so a hole inside an island is placed after the island exists.
        holes.Sort((x, y) => System.Math.Abs(Area(y)).CompareTo(System.Math.Abs(Area(x))));
        var pending = new List<List<int>>(holes);
        foreach (var hole in holes)
        {
            pending.Remove(hole);
            // The island traced from the same curve has the same vertices; the hole never belongs inside it.
            var twin = new HashSet<int>(hole);
            // Probe just to the left of the first hole edge, which is the side of the region around it.
            var a = Project(hole[0]);
            var b = Project(hole[1]);
            var probe = ((a.X + b.X) * 0.5 - (b.Y - a.Y) * 1e-3, (a.Y + b.Y) * 0.5 + (b.X - a.X) * 1e-3);
            var index = -1;
            var fallback = -1;
            var smallest = double.PositiveInfinity;
            for (var r = 0; r < regions.Count; ++r)
            {
                if (regions[r].Count == hole.Count && twin.SetEquals(regions[r]))
                    continue;
                if (fallback < 0)
                    fallback = r;
                if (!Contains(regions[r], probe))
                    continue;
                var area = System.Math.Abs(Area(regions[r]));
                if (area < smallest)
                {
                    smallest = area;
                    index = r;
                }
            }
            if (index < 0)
                index = System.Math.Max(fallback, 0);
            var outer = regions[index];
            if (Bridge(outer, hole, out var first, out var second))
            {
                regions[index] = first;
                regions.Add(second);
            }
            else
            {
                var (o, h) = Nearest(outer, hole);
                var keyhole = new List<int>();
                for (var k = 0; k <= outer.Count; ++k)
                    keyhole.Add(outer[(o + k) % outer.Count]);
                for (var k = 0; k <= hole.Count; ++k)
                    keyhole.Add(hole[(h + k) % hole.Count]);
                regions[index] = keyhole;
            }
        }

        double Area(List<int> loop)
        {
            var area = 0.0;
            for (var i = 0; i < loop.Count; ++i)
            {
                var p = Project(loop[i]);
                var q = Project(loop[(i + 1) % loop.Count]);
                area += p.X * q.Y - q.X * p.Y;
            }
            return area * 0.5;
        }

        bool Contains(List<int> region, (double X, double Y) point)
        {
            var inside = false;
            for (int i = 0, j = region.Count - 1; i < region.Count; j = i++)
            {
                var p = Project(region[i]);
                var q = Project(region[j]);
                if ((p.Y > point.Y) != (q.Y > point.Y)
                    && point.X < (q.X - p.X) * (point.Y - p.Y) / (q.Y - p.Y) + p.X)
                    inside = !inside;
            }
            return inside;
        }

        (int Outer, int Hole) Nearest(List<int> outer, List<int> hole)
        {
            var best = (0, 0);
            var bestDistance = double.PositiveInfinity;
            for (var i = 0; i < outer.Count; ++i)
            for (var j = 0; j < hole.Count; ++j)
            {
                var distance = DistanceSquared(Project(outer[i]), Project(hole[j]));
                if (distance < bestDistance)
                {
                    bestDistance = distance;
                    best = (i, j);
                }
            }
            return best;
        }

        // Two bridges from the extreme hole vertices along the first axis to visible region vertices.
        bool Bridge(List<int> outer, List<int> hole, out List<int> first, out List<int> second)
        {
            first = second = null!;
            int low = 0, high = 0;
            for (var i = 1; i < hole.Count; ++i)
            {
                if (Project(hole[i]).X < Project(hole[low]).X)
                    low = i;
                if (Project(hole[i]).X > Project(hole[high]).X)
                    high = i;
            }
            if (low == high)
                return false;
            var lowTarget = Visible(outer, hole, low, -1, -1);
            if (lowTarget < 0)
                return false;
            var highTarget = Visible(outer, hole, high, lowTarget, low);
            if (highTarget < 0)
                return false;

            first = [];
            for (var k = lowTarget; ; k = (k + 1) % outer.Count)
            {
                first.Add(outer[k]);
                if (k == highTarget)
                    break;
            }
            for (var k = high; ; k = (k + 1) % hole.Count)
            {
                first.Add(hole[k]);
                if (k == low)
                    break;
            }
            second = [];
            for (var k = highTarget; ; k = (k + 1) % outer.Count)
            {
                second.Add(outer[k]);
                if (k == lowTarget)
                    break;
            }
            for (var k = low; ; k = (k + 1) % hole.Count)
            {
                second.Add(hole[k]);
                if (k == high)
                    break;
            }
            return true;
        }

        int Visible(List<int> outer, List<int> hole, int from, int avoidOuter, int avoidHole)
        {
            var start = Project(hole[from]);
            var order = new List<int>(outer.Count);
            for (var i = 0; i < outer.Count; ++i)
                order.Add(i);
            order.Sort((x, y) => DistanceSquared(Project(outer[x]), start).CompareTo(
                DistanceSquared(Project(outer[y]), start)));
            foreach (var candidate in order)
            {
                if (candidate == avoidOuter)
                    continue;
                var target = Project(outer[candidate]);
                // Holes not yet joined also lie in the region, so a bridge must not cut through them either.
                if (Blocked(outer, start, target, outer[candidate], hole[from])
                    || Blocked(hole, start, target, outer[candidate], hole[from])
                    || pending.Any(other => Blocked(other, start, target, outer[candidate], hole[from])))
                    continue;
                if (avoidOuter >= 0 && SegmentsCross(start, target, Project(hole[avoidHole]),
                        Project(outer[avoidOuter])))
                    continue;
                var midpoint = ((start.X + target.X) * 0.5, (start.Y + target.Y) * 0.5);
                if (!Contains(outer, midpoint) || Contains(hole, midpoint))
                    continue;
                return candidate;
            }
            return -1;
        }

        bool Blocked(List<int> loop, (double X, double Y) from, (double X, double Y) to, int endA, int endB)
        {
            for (var i = 0; i < loop.Count; ++i)
            {
                var p = loop[i];
                var q = loop[(i + 1) % loop.Count];
                if (p == endA || p == endB || q == endA || q == endB)
                    continue;
                if (SegmentsCross(from, to, Project(p), Project(q)))
                    return true;
            }
            return false;
        }

        static double DistanceSquared((double X, double Y) a, (double X, double Y) b) =>
            (a.X - b.X) * (a.X - b.X) + (a.Y - b.Y) * (a.Y - b.Y);

        static double Cross((double X, double Y) o, (double X, double Y) a, (double X, double Y) b) =>
            (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);

        static bool SegmentsCross((double X, double Y) a0, (double X, double Y) a1, (double X, double Y) b0,
            (double X, double Y) b1)
        {
            var d1 = Cross(a0, a1, b0);
            var d2 = Cross(a0, a1, b1);
            var d3 = Cross(b0, b1, a0);
            var d4 = Cross(b0, b1, a1);
            return (d1 > 0.0) != (d2 > 0.0) && (d3 > 0.0) != (d4 > 0.0);
        }
    }

    private static Vec3 PointPosition(Corefinement core, int id)
    {
        if (id >= core.CrossingBase)
            return core.CrossingAt(id).Position;
        return id >= core.B.Offset
            ? core.B.Loops.Welder.Positions[id - core.B.Offset]
            : core.A.Loops.Welder.Positions[id];
    }

    /// <summary>Build the polygon of a region with vertex attributes interpolated within its source face.</summary>
    private static CsgPolygon RegionPolygon(Corefinement core, Mesh mesh, int face, List<int> region)
    {
        var source = mesh.Solid._faces[face];
        var start = mesh.Loops.VertexStart[face];
        var vertices = new List<ModelVertex>(region.Count);
        foreach (var id in region)
        {
            if (id < core.CrossingBase)
            {
                var local = LocalVertex(mesh, face, id - mesh.Offset);
                vertices.Add(source.Vertices[local]);
                continue;
            }
            ref readonly var crossing = ref core.CrossingAt(id);
            ModelVertex vertex;
            if (crossing.Key.EdgeInSecond == mesh.IsSecond)
            {
                // On an edge of this face, or on a diagonal of its triangulation.
                var low = source.Vertices[LocalVertex(mesh, face, crossing.Key.Low)];
                var high = source.Vertices[LocalVertex(mesh, face, crossing.Key.High)];
                vertex = CsgCommon.LerpModelVertex(low, high, (float)crossing.T);
            }
            else
            {
                var triangle = crossing.Key.Triangle;
                var a = source.Vertices[mesh.Solid._triangleVertices[triangle * 3]];
                var b = source.Vertices[mesh.Solid._triangleVertices[triangle * 3 + 1]];
                var c = source.Vertices[mesh.Solid._triangleVertices[triangle * 3 + 2]];
                vertex = Barycentric(a, b, c, crossing.Position);
            }
            vertex.SetPosition(ToVector3f(crossing.Position));
            vertices.Add(vertex);
        }
        return new CsgPolygon(vertices, source.Shared) { Plane = source.Plane };

        int LocalVertex(Mesh m, int f, int id)
        {
            var ids = m.Loops.Ids;
            for (var i = start; i < m.Loops.VertexStart[f + 1]; ++i)
            {
                if (ids[i] == id)
                    return i - start;
            }
            throw new InvalidOperationException("Crossing edge is not part of its face.");
        }
    }

    private static ModelVertex Barycentric(in ModelVertex a, in ModelVertex b, in ModelVertex c, Vec3 point)
    {
        var pa = Position(a);
        var pb = Position(b);
        var pc = Position(c);
        var normal = Vec3.Cross(pb - pa, pc - pa);
        var area = normal.LengthSquared();
        if (area <= 0.0f)
            return a;
        var wb = System.Math.Clamp(Vec3.Dot(Vec3.Cross(point - pa, pc - pa), normal) / area, 0.0f, 1.0f);
        var wc = System.Math.Clamp(Vec3.Dot(Vec3.Cross(pb - pa, point - pa), normal) / area, 0.0f, 1.0f - wb);
        var wa = 1.0f - wb - wc;
        // Blend b and c first, then blend the pair with a.
        var pair = wb + wc > 0.0f ? CsgCommon.LerpModelVertex(b, c, wc / (wb + wc)) : b;
        return CsgCommon.LerpModelVertex(a, pair, 1.0f - wa);
    }

    internal CsgSolid Boolean(CsgSolid other, CsgOperation operation, float epsilon)
    {
        // Solids are immutable, so an unchanged operand is itself the result.
        if (_faces.Length == 0)
            return operation == CsgOperation.Union ? other : this;
        if (other._faces.Length == 0)
            return operation == CsgOperation.Intersection ? other : this;

        var core = new Corefinement(this, other, epsilon);
        if (BoundsOverlap(other._boundsMin, other._boundsMax, epsilon))
        {
            var componentPairs = new List<(int A, int B)>();
            var facePairs = new List<(int A, int B)>();
            ComponentTree.CollectOverlaps(other.ComponentTree, epsilon, componentPairs);
            foreach (var (componentA, componentB) in componentPairs)
            {
                var treeA = NearFaces(componentA, other._componentMin[componentB], other._componentMax[componentB],
                    epsilon);
                var treeB = other.NearFaces(componentB, _componentMin[componentA], _componentMax[componentA], epsilon);
                treeA.CollectOverlaps(treeB, epsilon, facePairs);
            }
            foreach (var (faceA, faceB) in facePairs)
            {
                if (core.A.Loops.LoopLength[faceA] >= 3 && core.B.Loops.LoopLength[faceB] >= 3)
                    core.IntersectFaces(faceA, faceB);
            }
        }
        var result = new SolidBuilder(_faces.Length + other._faces.Length,
            _trianglePositions.Length + other._trianglePositions.Length);
        var region = new List<KeptFace>();
        AppendOperand(core, core.A, core.SegmentsA, other, operation, false, result, region);
        AppendOperand(core, core.B, core.SegmentsB, this, operation, true, result, region);
        if (region.Count != 0)
        {
            var kept = CancelOpposites(region);
            var faces = new CsgPolygon[kept.Count];
            var origins = new FaceOrigin?[kept.Count];
            var idCount = 0;
            for (var i = 0; i < kept.Count; ++i)
            {
                faces[i] = kept[i].Polygon;
                origins[i] = kept[i].Origin;
                idCount += kept[i].Polygon.Vertices.Count;
            }
            // Region ids are recorded; an unsplit face reads its ids from its solid, offset like the mesh's.
            var exactIds = new int[idCount];
            var next = 0;
            foreach (var face in kept)
            {
                if (face.Ids is { } ids)
                    ids.CopyTo(exactIds, next);
                else
                {
                    var origin = face.Origin!.Value;
                    var weld = origin.Solid.ExactWeld;
                    var offset = ReferenceEquals(origin.Solid, this) ? core.A.Offset : core.B.Offset;
                    var start = weld.VertexStart[origin.Face];
                    var count = weld.VertexStart[origin.Face + 1] - start;
                    for (var k = 0; k < count; ++k)
                        exactIds[next + k] = offset + weld.Ids[start + (origin.Flip ? count - 1 - k : k)];
                }
                next += face.Polygon.Vertices.Count;
            }
            result.AppendSolid(new CsgSolid(faces, epsilon, false, origins, exactIds,
                core.CrossingBase + core.Crossings.Count));
        }
        return new CsgSolid(result);
    }

    /// <summary>
    /// Remove pairs of kept faces that visit the same vertices in opposite directions. Where a closed curve
    /// passes through collinear crossings, both operands keep a zero-area island with the same vertices; the two
    /// cancel, and the faces around them then meet directly along the same edges.
    /// </summary>
    private static List<KeptFace> CancelOpposites(List<KeptFace> faces)
    {
        // Faces are bucketed by an order-independent hash of their vertex ids, then compared exactly.
        var buckets = new Dictionary<(int Count, ulong Hash), List<int>>();
        for (var i = 0; i < faces.Count; ++i)
        {
            // Unsplit faces of the two operands never share vertex ids, so only regions can cancel.
            if (faces[i].Origin is not null)
                continue;
            var loop = faces[i].Loop!;
            var hash = 0UL;
            foreach (var id in loop)
                hash += ((ulong)(uint)id + 0x9E3779B97F4A7C15UL) * 0xBF58476D1CE4E5B9UL;
            var key = (loop.Length, hash);
            if (!buckets.TryGetValue(key, out var bucket))
                buckets.Add(key, bucket = []);
            bucket.Add(i);
        }
        var removed = new bool[faces.Count];
        foreach (var bucket in buckets.Values)
        {
            if (bucket.Count < 2)
                continue;
            for (var a = 0; a < bucket.Count; ++a)
            {
                if (removed[bucket[a]])
                    continue;
                for (var b = a + 1; b < bucket.Count; ++b)
                {
                    if (removed[bucket[b]] || !IsReverse(faces[bucket[a]].Loop!, faces[bucket[b]].Loop!))
                        continue;
                    removed[bucket[a]] = removed[bucket[b]] = true;
                    break;
                }
            }
        }
        var result = new List<KeptFace>(faces.Count);
        for (var i = 0; i < faces.Count; ++i)
        {
            if (!removed[i])
                result.Add(faces[i]);
        }
        return result;

        static bool IsReverse(int[] x, int[] y)
        {
            var n = x.Length;
            var start = Array.IndexOf(y, x[0]);
            if (start < 0)
                return false;
            for (var k = 0; k < n; ++k)
            {
                if (y[(start - k + n) % n] != x[k])
                    return false;
            }
            return true;
        }
    }

    /// <summary>
    /// Append the kept surface of one operand. Components that the curve does not reach are classified whole; the
    /// others are split along the curve, grouped into patches between curve edges and classified patch by patch.
    /// </summary>
    private static void AppendOperand(Corefinement core, Mesh mesh, Dictionary<int, List<(int P, int Q)>> segments,
        CsgSolid other, CsgOperation operation, bool second, SolidBuilder result,
        List<KeptFace> region)
    {
        var solid = mesh.Solid;
        var flip = second && operation == CsgOperation.DifferenceAB;
        var touched = new bool[solid.ComponentCount];
        var frontier = new HashSet<int>();
        foreach (var triangle in segments.Keys)
        {
            var face = mesh.TriangleFace[triangle];
            frontier.Add(face);
            touched[solid._faceComponents[face]] = true;
        }

        var epsilon = core.Epsilon;
        for (var component = 0; component < solid.ComponentCount; ++component)
        {
            if (touched[component])
                continue;
            var inside = other.ContainsPoint(solid.ComponentSample(component), epsilon);
            if (Keep(inside, operation, second))
                result.AppendComponent(solid, component, flip);
        }

        var elements = new List<Element>();
        var regions = new List<List<int>>();
        for (var component = 0; component < solid.ComponentCount; ++component)
        {
            if (!touched[component])
                continue;
            for (var c = solid._componentStart[component]; c < solid._componentStart[component + 1]; ++c)
            {
                var face = solid._componentFaces[c];
                var count = mesh.Loops.LoopLength[face];
                if (count < 3)
                    continue;
                if (frontier.Contains(face))
                {
                    // A face the curve reaches is output triangle by triangle: split where the curve runs, whole
                    // elsewhere.
                    for (var t = solid._triangleStart[face]; t < solid._triangleStart[face + 1]; ++t)
                    {
                        if (mesh.IsCollapsed(t))
                            continue;
                        if (segments.TryGetValue(t, out var triangleSegments)
                            && SplitTriangle(core, mesh, face, t, triangleSegments, regions))
                        {
                            foreach (var loop in regions)
                            {
                                if (loop.Count >= 3)
                                    elements.Add(new Element
                                    {
                                        Loop = [.. loop], Ids = [.. loop], Polygon = RegionPolygon(core, mesh, face, loop),
                                        Split = true
                                    });
                            }
                            continue;
                        }
                        var source = solid._faces[face];
                        var corners = new List<ModelVertex>(3);
                        var cornerIds = new int[3];
                        for (var k = 0; k < 3; ++k)
                        {
                            corners.Add(source.Vertices[solid._triangleVertices[t * 3 + k]]);
                            cornerIds[k] = core.Global(mesh, mesh.TriangleIds[t * 3 + k]);
                        }
                        elements.Add(new Element
                        {
                            Loop = cornerIds, Ids = cornerIds,
                            Polygon = new CsgPolygon(corners, source.Shared) { Plane = source.Plane }
                        });
                    }
                    continue;
                }
                elements.Add(new Element { Polygon = solid._faces[face], SourceFace = face });
            }
        }
        if (elements.Count == 0)
            return;

        var inside2 = ClassifyElements(core, mesh, elements, other, second, epsilon);
        for (var i = 0; i < elements.Count; ++i)
        {
            if (Keep(inside2[i], operation, second))
            {
                var loop = elements[i].Loop;
                var ids = elements[i].Ids;
                if (flip && loop is not null)
                {
                    loop = (int[])loop.Clone();
                    Array.Reverse(loop);
                    if (ids is not null)
                    {
                        ids = (int[])ids.Clone();
                        Array.Reverse(ids);
                    }
                }
                var sourceFace = elements[i].SourceFace;
                region.Add(new KeptFace(flip ? Flipped(elements[i].Polygon) : elements[i].Polygon, loop, ids,
                    sourceFace >= 0 ? new FaceOrigin(solid, sourceFace, flip) : null));
            }
        }
    }

    /// <summary>
    /// Classify elements against the other operand. Each curve edge of an element decides exactly which side of
    /// the curve is inside the other operand: the element on the left of an oriented segment is inside for the
    /// first operand, and on its right for the second. Elements joined by an edge off the curve form a patch;
    /// decisions spread across such edges from the nearest decided element, and a patch without curve edges
    /// falls back to a ray cast.
    /// </summary>
    /// <remarks>
    /// The symbolic perturbation keeps the curve off the edges of both operands, so only regions of split
    /// triangles have curve edges.
    /// </remarks>
    private static bool[] ClassifyElements(Corefinement core, Mesh mesh, List<Element> elements, CsgSolid other,
        bool second, float epsilon)
    {
        // Every element loop in one buffer; unsplit faces read theirs from the mesh.
        var edgeStart = new int[elements.Count + 1];
        for (var e = 0; e < elements.Count; ++e)
        {
            edgeStart[e + 1] = edgeStart[e] + (elements[e].Loop?.Length
                ?? mesh.Loops.LoopLength[elements[e].SourceFace]);
        }
        var loops = new int[edgeStart[^1]];
        for (var e = 0; e < elements.Count; ++e)
        {
            if (elements[e].Loop is { } own)
            {
                own.CopyTo(loops, edgeStart[e]);
                continue;
            }
            var start = mesh.Loops.VertexStart[elements[e].SourceFace];
            for (var i = edgeStart[e]; i < edgeStart[e + 1]; ++i)
                loops[i] = core.Global(mesh, mesh.Loops.VertexIds[start + i - edgeStart[e]]);
        }
        var edgeOwner = new Dictionary<ulong, int>(edgeStart[^1], CsgCommon.PackedKeyComparer.Instance);
        for (var e = 0; e < elements.Count; ++e)
        {
            var loop = loops.AsSpan(edgeStart[e], edgeStart[e + 1] - edgeStart[e]);
            for (var i = 0; i < loop.Length; ++i)
                edgeOwner[DirectedKey(loop[i], loop[(i + 1) % loop.Length])] = e;
        }

        // The element across each edge off the curve, or -1 for a curve edge or an unmatched one.
        var twins = new int[edgeStart[^1]];
        var result = new bool[elements.Count];
        var decided = new bool[elements.Count];
        var frontier = new Queue<int>();
        for (var e = 0; e < elements.Count; ++e)
        {
            var loop = loops.AsSpan(edgeStart[e], edgeStart[e + 1] - edgeStart[e]);
            var split = elements[e].Split;
            var votes = 0;
            for (var i = 0; i < loop.Length; ++i)
            {
                var from = loop[i];
                var to = loop[(i + 1) % loop.Length];
                twins[edgeStart[e] + i] = -1;
                if (split && core.CurveEdges.TryGetValue(UndirectedKey(from, to), out var segment))
                {
                    var forward = segment.Start == from && segment.End == to;
                    votes += forward != second ? 1 : -1;
                    continue;
                }
                if (edgeOwner.TryGetValue(DirectedKey(to, from), out var twin) && twin != e)
                    twins[edgeStart[e] + i] = twin;
            }
            // Each element next to the curve keeps its own exact decision, so the elements on either side of a
            // curve edge always differ; the rest inherit from the nearest decided element across edges off the
            // curve. A local inconsistency then stays local instead of flipping a whole patch.
            if (votes == 0)
                continue;
            result[e] = votes > 0;
            decided[e] = true;
            frontier.Enqueue(e);
        }
        Spread();

        // What remains are patches the curve does not reach: sample each one's largest element.
        var patch = new List<int>();
        for (var e = 0; e < elements.Count; ++e)
        {
            if (decided[e])
                continue;
            patch.Clear();
            decided[e] = true;
            patch.Add(e);
            for (var p = 0; p < patch.Count; ++p)
            {
                for (var k = edgeStart[patch[p]]; k < edgeStart[patch[p] + 1]; ++k)
                {
                    var twin = twins[k];
                    if (twin >= 0 && !decided[twin])
                    {
                        decided[twin] = true;
                        patch.Add(twin);
                    }
                }
            }
            var best = e;
            var bestArea = -1.0;
            foreach (var member in patch)
            {
                var area = PolygonArea(elements[member].Polygon);
                if (area > bestArea)
                {
                    bestArea = area;
                    best = member;
                }
            }
            var inside = other.ClassifyPoint(PolygonInteriorPoint(elements[best].Polygon), epsilon, out _);
            foreach (var member in patch)
                result[member] = inside;
        }
        return result;

        void Spread()
        {
            while (frontier.TryDequeue(out var e))
            {
                for (var k = edgeStart[e]; k < edgeStart[e + 1]; ++k)
                {
                    var twin = twins[k];
                    if (twin < 0 || decided[twin])
                        continue;
                    result[twin] = result[e];
                    decided[twin] = true;
                    frontier.Enqueue(twin);
                }
            }
        }
    }

    private static double PolygonArea(CsgPolygon polygon)
    {
        var vertices = CollectionsMarshal.AsSpan(polygon.Vertices);
        var normal = Vec3.Zero;
        for (var i = 0; i < vertices.Length; ++i)
            normal += Vec3.Cross(Position(vertices[i]), Position(vertices[(i + 1) % vertices.Length]));
        return normal.Length() * 0.5;
    }

    /// <summary>A point inside a polygon: the centroid of its largest triangle.</summary>
    private static Vector3f PolygonInteriorPoint(CsgPolygon polygon)
    {
        var vertices = polygon.Vertices;
        var indices = new List<uint>();
        if (vertices.Count > 3 && Csg.TriangulatePolygon(polygon, indices) && indices.Count >= 3)
        {
            var best = 0;
            var bestArea = -1.0f;
            for (var t = 0; t + 2 < indices.Count; t += 3)
            {
                var a = Position(vertices[(int)indices[t]]);
                var area = Vec3.Cross(Position(vertices[(int)indices[t + 1]]) - a,
                    Position(vertices[(int)indices[t + 2]]) - a).LengthSquared();
                if (area > bestArea)
                {
                    bestArea = area;
                    best = t;
                }
            }
            var centroid = (Position(vertices[(int)indices[best]]) + Position(vertices[(int)indices[best + 1]])
                + Position(vertices[(int)indices[best + 2]])) / 3.0f;
            return ToVector3f(centroid);
        }
        var sum = Vec3.Zero;
        foreach (var vertex in vertices)
            sum += Position(vertex);
        return ToVector3f(sum / vertices.Count);
    }
}
