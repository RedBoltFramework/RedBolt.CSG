# CSG porting checklist

This inventory moved from the base RedBolt repository with the optional CSG module.

## B12a. CSG `OPTIONAL` — depends on Graphics and Scene

- [x] `CsgCommon.h` geometry interpolation and quantization helpers
- [x] `CsgBsp.h` clip modes, polygons, BSP nodes, and boolean clipping
- [x] `CsgOptimization.h` polygon simplification, coplanar merging, self-intersection repair, and T-junction resolution
- [x] `CsgTriangulation.h` triangulated models and triangulation helpers
- [x] `Csg.h` operations, model extraction/build, boolean operations, and assembly registration
- [x] `CsgComponent.h` base scene component
- [x] `CsgBrush.h` source brush component
- [x] `CsgRoot.h` asynchronous result-root component
- [x] Desktop showcase with union, difference, intersection, source-material preservation, and Dear ImGui instructions
- [x] Aggregate WebAssembly and Android sample hosts
- [x] Managed NuGet package definition
- [x] Tests remain in `RedBolt.Tests`
