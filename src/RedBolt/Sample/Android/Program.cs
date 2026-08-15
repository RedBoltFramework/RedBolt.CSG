namespace RedBolt.Sample.Android;

internal static class SampleCatalog
{
    internal static readonly SampleEntry[] Entries =
    [
        new("csg", "Constructive Solid Geometry", static () => new global::RedBolt.Sample.CSG.CsgShowcase()),
    ];
}
