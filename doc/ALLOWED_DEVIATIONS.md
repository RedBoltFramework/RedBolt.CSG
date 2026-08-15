# Allowed deviations

These deviations apply to the CSG module extracted from RedBolt:

- The module is managed-only and reuses RedBolt.Core scene, graphics, model, work-queue, and reflection APIs; it owns no native wrapper.
- Boolean operations preserve source materials and rebuild `CsgRoot` geometry asynchronously through RedBolt's work queue.
- The showcase uses Dear ImGui for instructions because the built-in rbfx UI subsystem is intentionally not ported.
