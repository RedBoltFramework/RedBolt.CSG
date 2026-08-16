// Copyright (c) 2026 RedBolt and contributors.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

namespace RedBolt.Sample.Web;

using RedBolt.Engine;

internal static class SampleCatalog
{
    internal static Application Create(string applicationTypeName) => applicationTypeName switch
    {
        "RedBolt.Sample.CSG.CsgShowcase" => new global::RedBolt.Sample.CSG.CsgShowcase(),
        _ => throw new System.InvalidOperationException($"Unknown RedBolt sample application: {applicationTypeName}"),
    };
}
