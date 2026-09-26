# RedBolt.CSG

Optional constructive solid geometry for [RedBolt](https://github.com/RedBoltFramework/RedBolt). The repository owns the managed CSG module and its desktop, WebAssembly, and Android showcase.

The source build expects RedBolt in `../RedBolt` by default. Override `RedBoltRoot` when the repositories use a different layout.

```sh
dotnet build RedBolt.CSG.slnx --configuration Debug
```

Desktop, WebAssembly, and Android samples are included under `src/RedBolt/Sample`.

Tests remain in the sibling `RedBolt.Tests` repository. See [building](doc/BUILDING.md), [package production](packaging/nuget/README.md), and the [CSG-specific deviations](doc/ALLOWED_DEVIATIONS.md).

## Boolean operands

`CsgSolid` retains polygon faces, source attributes, shared-edge connectivity, and a face BVH independently
of the render mesh. Build one with `Csg.BuildSolidFromModel` or from editable polygons and reuse it across
boolean calls. `Csg.BooleanOperation(CsgSolid, CsgSolid, ...)`
uses BVH traversal to find contacting faces and corefines them: every crossing of an edge of one operand
through a triangle of the other is decided by an exact orientation test on a fine grid, with a symbolic
perturbation breaking ties such as coplanar contact, and becomes one vertex shared by every face that touches
it. Triangles along the intersection curve are split at those vertices, so both operands meet exactly and the
result is closed by construction, with no repair step; it can be used as an operand again. Each piece next to
the curve is classified exactly from the orientation of the curve, and the rest of its surface follows it;
components that the curve does not reach are classified with ray parity, including the containment case.
Untouched faces are shared between immutable solids; callers receive editable copies through `ToPolygons()`.
Operands must be closed.
`CsgRoot` uses this solid API for its asynchronous operation tree and triangulates only the final result.
When brushes are appended to a flat operation tree, it reuses the last accepted solid and evaluates only
the new brushes. Edits to existing brushes rebuild from the base operand.

The BVH is implemented in managed code, so the module remains usable on desktop, Android, and WebAssembly
without a new native binding or platform package.
