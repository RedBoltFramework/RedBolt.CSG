// Copyright (c) 2026 RedBolt and contributors.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

namespace RedBolt.Sample.Android;

internal static class SampleCatalog
{
    internal static readonly SampleEntry[] Entries =
    [
        new("csg", "Constructive Solid Geometry", static () => new global::RedBolt.Sample.CSG.CsgShowcase()),
    ];
}
