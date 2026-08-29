// Copyright (c) 2026 RedBolt and contributors.
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT> or the accompanying LICENSE file.

namespace RedBolt.Sample.CSG;

using System;
using System.Collections.Generic;

using RedBolt.Bindings.DearImGui;
using RedBolt.Core;
using RedBolt.CSG;
using RedBolt.DearImGui;
using RedBolt.Engine;
using RedBolt.Graphics;
using RedBolt.Input;
using RedBolt.Math;
using RedBolt.Resource;
using RedBolt.Scene;
using Widgets = RedBolt.SystemUI.Widgets;

internal static class CsgProgram
{
    private static int Main()
    {
        try
        {
            using var application = new CsgShowcase();
            var exitCode = application.Run();
            if (exitCode == 0)
                Console.WriteLine("RedBolt CSG sample finished.");
            return exitCode;
        }
        catch (Exception ex)
        {
            Console.Error.WriteLine(ex);
            return 1;
        }
    }
}

/// <summary>Interactive constructive solid geometry demolition example.</summary>
public class CsgShowcase : Application
{
    private sealed class Projectile
    {
        public required Node Node;
        public Vector3f Direction;
        public Vector3f Destination;
        public CsgRoot? Root;
        public uint RebuildRevision;
        public bool IsMud;
        public bool HasSurfaceHit;
        public bool Arrived;
    }

    private sealed class ImpactEffect
    {
        public required Node Node;
        public float TimeToLive;
    }

    private sealed class MudPlacement
    {
        public uint TargetNodeId;
        public Vector3f Center;
    }

    protected override void Start()
    {
        base.Start();
        Csg.RegisterCsgLibrary(Context);
        Context.RegisterSubsystem(new DearImGui((ImGuiConfigFlags)Engine.Parameters.DearImGuiFlags));

        PreloadWeaponResources();
        CreateScene();
        SetupViewport();
        _movePrompts =
        [
            Resources.Get<Texture2D>("ThirdParty/KenneyInputPrompts/keyboard_w.png"),
            Resources.Get<Texture2D>("ThirdParty/KenneyInputPrompts/keyboard_a.png"),
            Resources.Get<Texture2D>("ThirdParty/KenneyInputPrompts/keyboard_s.png"),
            Resources.Get<Texture2D>("ThirdParty/KenneyInputPrompts/keyboard_d.png")
        ];
        _lookPrompt = Resources.Get<Texture2D>("ThirdParty/KenneyInputPrompts/mouse_move.png");
        _sprintPrompt = Resources.Get<Texture2D>("ThirdParty/KenneyInputPrompts/keyboard_shift.png");
        _explosivePrompt = Resources.Get<Texture2D>("ThirdParty/KenneyInputPrompts/mouse_left.png");
        _mudPrompt = Resources.Get<Texture2D>("ThirdParty/KenneyInputPrompts/mouse_right.png");
        _exitPrompt = Resources.Get<Texture2D>("ThirdParty/KenneyInputPrompts/keyboard_escape.png");
        SubscribeToEvent<DearImGuiEvents.EndRenderingDearImGuiEvent>(
            (ref DearImGuiEvents.EndRenderingDearImGuiEvent _) => DrawInstructions());

        Input.SetMouseMode(MouseMode.Relative);
        Input.SetMouseVisible(false);
    }

    private void PreloadWeaponResources()
    {
        // Keep immutable weapon resources ready before gameplay and reuse them for projectiles and effects.
        _sphereModel = Resources.Get<Model>("Models/Sphere.mdl");
        _explosiveOperandModel = Resources.Get<Model>("Models/Cylinder.mdl");
        _mudOperandModel = CreateLowPolyMudBall();
        if (!Resources.AddManualResource(_mudOperandModel))
            throw new InvalidOperationException("Could not register the low-poly CSG mud model.");
        _explosiveMaterial = Resources.Get<Material>("Materials/TechnicalDemo/Yellow.xml");
        _explosionMaterial = Resources.Get<Material>("Materials/Constant/GlowingRed.xml");
        _mudMaterial = Resources.Get<Material>("Materials/TechnicalDemo/Dark.xml").Clone(MudMaterialName);
        _mudMaterial.SetShaderParameter("MatDiffColor", new Vector4f(0.52f, 0.22f, 0.045f, 1.0f));
        _mudMaterial.SetShaderParameter("MatEmissiveColor", new Vector3f(0.08f, 0.018f, 0.002f));
        _mudMaterial.SetShaderParameter("Roughness", 0.96f);
        _mudMaterial.SetShaderParameter("Metallic", 0.0f);
        if (!Resources.AddManualResource(_mudMaterial))
            throw new InvalidOperationException("Could not register the CSG mud material.");
    }

    private Model CreateLowPolyMudBall()
    {
        // An icosahedron reads as a ball but contributes only 20 triangles to each additive BSP operation.
        var t = (1.0f + MathF.Sqrt(5.0f)) * 0.5f;
        var positions = new Vector3f[]
        {
            new(-1, t, 0), new(1, t, 0), new(-1, -t, 0), new(1, -t, 0),
            new(0, -1, t), new(0, 1, t), new(0, -1, -t), new(0, 1, -t),
            new(t, 0, -1), new(t, 0, 1), new(-t, 0, -1), new(-t, 0, 1)
        };
        for (var i = 0; i < positions.Length; ++i)
            positions[i] = positions[i].Normalized * 0.5f;

        var faces = new int[]
        {
            0, 11, 5, 0, 5, 1, 0, 1, 7, 0, 7, 10, 0, 10, 11,
            1, 5, 9, 5, 11, 4, 11, 10, 2, 10, 7, 6, 7, 1, 8,
            3, 9, 4, 3, 4, 2, 3, 2, 6, 3, 6, 8, 3, 8, 9,
            4, 9, 5, 2, 4, 11, 6, 2, 10, 8, 6, 7, 9, 8, 1
        };

        var geometry = new CsgTriangulatedModel.Geometry();
        for (var i = 0; i < faces.Length; i += 3)
        {
            var a = positions[faces[i]];
            var b = positions[faces[i + 1]];
            var c = positions[faces[i + 2]];
            var normal = (b - a).CrossProduct(c - a).Normalized;
            foreach (var position in new[] { a, b, c })
            {
                var vertex = new ModelVertex();
                vertex.SetPosition(position);
                vertex.SetNormal(normal);
                geometry.IndexData.Add(checked((uint)geometry.Vertices.Count));
                geometry.Vertices.Add(vertex);
            }
        }

        var triangulated = new CsgTriangulatedModel
        {
            BoundingBox = new BoundingBox(-0.5f, 0.5f),
            HasAnyVertex = true,
            Geometries = { geometry }
        };
        var model = Csg.BuildModel(triangulated, [_sphereModel], [null]);
        model.SetName(MudOperandModelName);
        return model;
    }

    private void CreateScene()
    {
        _scene = new Scene();
        _octree = _scene.CreateComponent<Octree>();

        var zoneNode = _scene.CreateChild("Zone");
        var zone = zoneNode.CreateComponent<Zone>();
        zone.SetBoundingBox(new BoundingBox(-1000.0f, 1000.0f));
        zone.SetAmbientColor(new Color(0.08f, 0.11f, 0.16f));
        zone.SetFogColor(new Color(0.025f, 0.045f, 0.075f));
        zone.SetFogStart(24.0f);
        zone.SetFogEnd(42.0f);

        var lightNode = _scene.CreateChild("DirectionalLight");
        lightNode.SetDirection(new Vector3f(0.35f, -1.0f, 0.25f));
        var light = lightNode.CreateComponent<Light>();
        light.SetLightType(LightType.Directional);
        light.SetColor(new Color(0.52f, 0.64f, 0.82f));
        light.SetCastShadows(true);
        light.SetSpecularIntensity(0.7f);

        CreateStructure("Floor", new Vector3f(0.0f, -0.25f, 0.0f), new Vector3f(20.0f, 0.5f, 28.0f),
            "Materials/TechnicalDemo/FloorGrid.xml", false);
        CreateStructure("Ceiling", new Vector3f(0.0f, 6.25f, 0.0f), new Vector3f(20.0f, 0.5f, 28.0f),
            "Materials/TechnicalDemo/Dark.xml", false);
        CreateStructure("OuterLeft", new Vector3f(-10.0f, 3.0f, 0.0f), new Vector3f(0.5f, 6.0f, 28.0f),
            "Materials/TechnicalDemo/SideWallGrid.xml", false);
        CreateStructure("OuterRight", new Vector3f(10.0f, 3.0f, 0.0f), new Vector3f(0.5f, 6.0f, 28.0f),
            "Materials/TechnicalDemo/SideWallGrid.xml", false);
        CreateStructure("OuterFront", new Vector3f(0.0f, 3.0f, -14.0f), new Vector3f(20.0f, 6.0f, 0.5f),
            "Materials/TechnicalDemo/BackWallGrid.xml", false);
        CreateStructure("OuterBack", new Vector3f(0.0f, 3.0f, 14.0f), new Vector3f(20.0f, 6.0f, 0.5f),
            "Materials/TechnicalDemo/BackWallGrid.xml", false);

        CreatePartition(-4.5f, "A");
        CreatePartition(5.0f, "B");

        CreateTrim(new Vector3f(0.0f, 0.08f, -4.5f), new Vector3f(5.0f, 0.12f, 0.16f));
        CreateTrim(new Vector3f(0.0f, 4.17f, -4.5f), new Vector3f(5.0f, 0.12f, 0.16f));
        CreateTrim(new Vector3f(0.0f, 0.08f, 5.0f), new Vector3f(5.0f, 0.12f, 0.16f));
        CreateTrim(new Vector3f(0.0f, 4.17f, 5.0f), new Vector3f(5.0f, 0.12f, 0.16f));

        CreateRoomLight(new Vector3f(0.0f, 5.6f, -9.0f), new Color(0.18f, 0.58f, 1.0f));
        CreateRoomLight(new Vector3f(0.0f, 5.6f, 0.0f), new Color(0.12f, 0.8f, 0.72f));
        CreateRoomLight(new Vector3f(0.0f, 5.6f, 9.5f), new Color(1.0f, 0.46f, 0.12f));

        _cameraNode = _scene.CreateChild("FPS Camera");
        // Start opposite a destructible panel, with the doorway still visible to the player's left.
        _cameraNode.SetPosition(new Vector3f(4.0f, EyeHeight, -10.5f));
        var camera = _cameraNode.CreateComponent<Camera>();
        camera.SetFarClip(60.0f);
        camera.SetViewMask(WorldViewMask | ForegroundViewMask);
        CreateGun();
    }

    private void CreatePartition(float z, string suffix)
    {
        CreateStructure($"InnerLeft{suffix}", new Vector3f(-6.25f, 3.0f, z), new Vector3f(7.5f, 6.0f, 0.5f),
            "Materials/TechnicalDemo/Grid.xml", true);
        CreateStructure($"InnerRight{suffix}", new Vector3f(6.25f, 3.0f, z), new Vector3f(7.5f, 6.0f, 0.5f),
            "Materials/TechnicalDemo/Grid.xml", true);
        CreateStructure($"InnerLintel{suffix}", new Vector3f(0.0f, 5.1f, z), new Vector3f(5.0f, 1.8f, 0.5f),
            "Materials/TechnicalDemo/Grid.xml", true);
    }

    private void CreateStructure(string name, Vector3f position, Vector3f scale, string material, bool destructible)
    {
        var node = _scene!.CreateChild(name);
        node.SetPosition(position);
        node.SetScale(scale);
        var root = node.CreateComponent<CsgRoot>();
        root.SetModel(new ResourceRef(nameof(Model), "Models/Box.mdl"));
        root.SetMaterials(new ResourceRefList(nameof(Material), [material]));
        root.Rebuild();
        var output = node.GetComponent<StaticModel>()!;
        output.SetViewMask(WorldViewMask);
        output.SetCastShadows(true);
        if (destructible)
            _destructibleStructures.Add(node.Id);
    }

    private void CreateTrim(Vector3f position, Vector3f scale)
    {
        var node = _scene!.CreateChild("Safety trim");
        node.SetPosition(position);
        node.SetScale(scale);
        var model = node.CreateComponent<StaticModel>();
        model.SetModel(Resources.Get<Model>("Models/Box.mdl"));
        model.SetMaterial(Resources.Get<Material>("Materials/TechnicalDemo/Accent.xml"));
        model.SetViewMask(ForegroundViewMask);
    }

    private void CreateRoomLight(Vector3f position, Color color)
    {
        var fixture = _scene!.CreateChild("Room light");
        fixture.SetPosition(position);
        var light = fixture.CreateComponent<Light>();
        light.SetLightType(LightType.Point);
        light.SetRange(12.0f);
        light.SetColor(color);
        light.SetSpecularIntensity(1.2f);

        var panel = fixture.CreateChild("Panel");
        panel.SetScale(new Vector3f(2.4f, 0.08f, 0.4f));
        var model = panel.CreateComponent<StaticModel>();
        model.SetModel(Resources.Get<Model>("Models/Box.mdl"));
        model.SetMaterial(Resources.Get<Material>("Materials/Constant/GlowingBlue.xml"));
        model.SetViewMask(ForegroundViewMask);
    }

    private void CreateGun()
    {
        _gunNode = _cameraNode!.CreateChild("CSG launcher");
        _gunNode.SetPosition(new Vector3f(0.38f, -0.34f, 0.82f));

        CreateGunPart("Receiver", Vector3f.Zero, new Vector3f(0.26f, 0.22f, 0.72f),
            "Materials/TechnicalDemo/Dark.xml");
        CreateGunPart("Barrel", new Vector3f(0.0f, 0.035f, 0.53f), new Vector3f(0.13f, 0.13f, 0.48f),
            "Materials/TechnicalDemo/Accent.xml");
        CreateGunPart("Sight", new Vector3f(0.0f, 0.16f, 0.05f), new Vector3f(0.07f, 0.08f, 0.18f),
            "Materials/Constant/GlowingBlue.xml");
        CreateGunPart("Grip", new Vector3f(0.0f, -0.24f, -0.15f), new Vector3f(0.17f, 0.38f, 0.2f),
            "Materials/TechnicalDemo/Dark.xml", new Quaternion(-12.0f, 0.0f, 0.0f));
    }

    private void CreateGunPart(string name, Vector3f position, Vector3f scale, string material,
        Quaternion? rotation = null)
    {
        var node = _gunNode!.CreateChild(name);
        node.SetPosition(position);
        node.SetScale(scale);
        if (rotation.HasValue)
            node.SetRotation(rotation.Value);
        var model = node.CreateComponent<StaticModel>();
        model.SetModel(Resources.Get<Model>("Models/Box.mdl"));
        model.SetMaterial(Resources.Get<Material>(material));
        model.SetViewMask(ForegroundViewMask);
        model.SetCastShadows(false);
    }

    private void SetupViewport()
    {
        _viewport = new Viewport(_scene!, _cameraNode!.GetComponent<Camera>()!);
        Renderer.SetViewport(0, _viewport);
    }

    protected override void Update(float timeStep)
    {
        if (Input.ExitRequested || Input.GetKeyPress(Key.Escape))
        {
            Exit();
            return;
        }

        UpdateCamera(timeStep);
        UpdateProjectiles(timeStep);
        ApplyArrivedProjectileResults();
        UpdateImpactEffects(timeStep);

        if (Input.GetMouseButtonPress(MouseButton.Left))
            FireProjectile(false);
        if (Input.GetMouseButtonPress(MouseButton.Right))
            FireProjectile(true);

        _recoil = MathF.Max(0.0f, _recoil - timeStep * 7.0f);
        _gunNode!.SetPosition(new Vector3f(0.38f, -0.34f, 0.82f - _recoil * 0.16f));
    }

    private void UpdateCamera(float timeStep)
    {
        var mouseMove = Input.GetMouseMove();
        _yaw += mouseMove.X * MouseSensitivity;
        _pitch = Math.Clamp(_pitch + mouseMove.Y * MouseSensitivity, -82.0f, 82.0f);
        _cameraNode!.SetRotation(new Quaternion(_pitch, _yaw, 0.0f));

        var movement = Vector3f.Zero;
        if (Input.GetScancodeDown(Scancode.W)) movement += Vector3f.Forward;
        if (Input.GetScancodeDown(Scancode.S)) movement += Vector3f.Back;
        if (Input.GetScancodeDown(Scancode.A)) movement += Vector3f.Left;
        if (Input.GetScancodeDown(Scancode.D)) movement += Vector3f.Right;
        if (movement.LengthSquared <= 0.0f)
            return;

        movement.Normalize();
        var heading = new Quaternion(0.0f, _yaw, 0.0f);
        var speed = Input.GetKeyDown(Key.Shift) ? SprintSpeed : WalkSpeed;
        var position = _cameraNode.Position + heading * movement * (speed * timeStep);
        position.X = Math.Clamp(position.X, -9.45f, 9.45f);
        position.Y = EyeHeight;
        position.Z = Math.Clamp(position.Z, -13.45f, 13.45f);
        _cameraNode.SetPosition(position);
    }

    private void FireProjectile(bool isMud)
    {
        var aimRay = _cameraNode!.GetComponent<Camera>()!.GetScreenRay(0.5f, 0.5f);
        using var query = new RayOctreeQuery(aimRay, RayQueryLevel.RayTriangle, MaxProjectileDistance,
            DrawableFlags.Geometry, WorldViewMask);
        _octree!.RaycastSingle(query);

        var hasSurfaceHit = query.Result.Count != 0;
        var hit = hasSurfaceHit ? query.Result[0] : default;
        var destination = hasSurfaceHit
            ? hit.Position
            : aimRay.Origin + aimRay.Direction * MaxProjectileDistance;

        CsgRoot? root = null;
        var rebuildRevision = 0u;
        if (hasSurfaceHit)
            root = BeginCsgOperation(hit, isMud, ref destination, out rebuildRevision);

        var node = _scene!.CreateChild(isMud ? "Mud round" : "Explosive round");
        node.SetWorldPosition(_gunNode!.LocalToWorld(new Vector3f(0.0f, 0.04f, 0.9f)));
        node.SetScale(isMud ? 0.2f : 0.16f);
        var model = node.CreateComponent<StaticModel>();
        model.SetModel(_sphereModel);
        model.SetMaterial(isMud ? _mudMaterial : _explosiveMaterial);
        model.SetViewMask(ForegroundViewMask);
        model.SetCastShadows(false);

        _projectiles.Add(new Projectile
        {
            Node = node,
            Direction = (destination - node.WorldPosition).Normalized,
            Destination = destination,
            Root = root,
            RebuildRevision = rebuildRevision,
            IsMud = isMud,
            HasSurfaceHit = hasSurfaceHit
        });
        _recoil = 1.0f;
    }

    private void UpdateProjectiles(float timeStep)
    {
        var distance = ProjectileSpeed * timeStep;
        for (var i = _projectiles.Count - 1; i >= 0; --i)
        {
            var projectile = _projectiles[i];
            if (projectile.Arrived)
                continue;

            var toDestination = projectile.Destination - projectile.Node.WorldPosition;
            if (toDestination.LengthSquared > distance * distance)
            {
                projectile.Node.SetWorldPosition(projectile.Node.WorldPosition + projectile.Direction * distance);
                continue;
            }

            projectile.Node.SetWorldPosition(projectile.Destination);
            projectile.Arrived = true;
            if (projectile.Root is not null)
                continue;

            FinishProjectile(i);
        }
    }

    private CsgRoot? BeginCsgOperation(RayQueryResult hit, bool isMud, ref Vector3f projectileDestination,
        out uint rebuildRevision)
    {
        rebuildRevision = 0;
        var targetNode = hit.Node;
        var root = targetNode?.GetComponent<CsgRoot>();
        if (targetNode is null || root is null)
            return null;
        if (!isMud && !_destructibleStructures.Contains(targetNode.Id))
            return null;

        _operationCounts.TryGetValue(targetNode.Id, out var operationCount);
        if (operationCount >= MaxOperationsPerStructure)
            return null;
        _operationCounts[targetNode.Id] = operationCount + 1;

        root.AutoApplyResults = false;
        _stagedRoots.Add(root);

        var brushNode = targetNode.CreateChild(isMud ? "Mud addition" : "Explosion subtraction");
        var brushPosition = hit.Position + (isMud ? hit.Normal * 0.12f : -hit.Normal * 0.08f);
        if (isMud)
        {
            // Several rounds may be in flight before the first result is visible. Do not create coincident union
            // operands in that case: they are pathological for BSP and make a rapid mud pile exponentially costly.
            // Advance the predicted contact point along the surface normal so pending rounds build the future pile.
            var displacement = Vector3f.Zero;
            while (HasNearbyMudPlacement(targetNode.Id, brushPosition + displacement))
                displacement += hit.Normal * MudStackStep;
            brushPosition += displacement;
            projectileDestination += displacement;
            _mudPlacements.Add(new MudPlacement { TargetNodeId = targetNode.Id, Center = brushPosition });
        }
        brushNode.SetWorldPosition(brushPosition);
        brushNode.SetWorldRotation(new Quaternion(Vector3f.Up, hit.Normal));
        SetOrientedWorldScale(brushNode, isMud
            ? new Vector3f(0.95f, 0.62f, 0.95f)
            : new Vector3f(2.0f, 1.5f, 2.0f));
        var brush = brushNode.CreateComponent<CsgBrush>();
        brush.Operation = isMud ? CsgOperation.Union : CsgOperation.DifferenceAB;
        // Keep both interactive operations low-poly. Explosions use a 124-triangle cylinder for round openings;
        // additive mud uses a generated 20-triangle ball because unions retain operand surfaces and grow faster
        // than subtraction. The flying round remains smoothly spherical in both cases.
        brush.SetModel(new ResourceRef(nameof(Model), isMud ? MudOperandModelName : "Models/Cylinder.mdl"));
        brush.SetMaterials(new ResourceRefList(nameof(Material),
            [isMud ? MudMaterialName : "Materials/TechnicalDemo/Yellow.xml"]));
        rebuildRevision = root.RebuildRevision;
        root.Rebuild();
        return root;
    }

    private bool HasNearbyMudPlacement(uint targetNodeId, Vector3f position)
    {
        foreach (var placement in _mudPlacements)
        {
            if (placement.TargetNodeId == targetNodeId
                && (placement.Center - position).LengthSquared < MudMinimumSeparation * MudMinimumSeparation)
                return true;
        }
        return false;
    }

    private void ApplyArrivedProjectileResults()
    {
        foreach (var root in _stagedRoots)
        {
            while (root.HasPendingResult)
            {
                var revision = root.PendingResultRevision;
                var waitingForProjectile = false;
                foreach (var projectile in _projectiles)
                {
                    if (ReferenceEquals(projectile.Root, root) && projectile.RebuildRevision <= revision
                        && !projectile.Arrived)
                    {
                        waitingForProjectile = true;
                        break;
                    }
                }
                if (waitingForProjectile)
                    break;

                root.ApplyPendingResult();
                for (var i = _projectiles.Count - 1; i >= 0; --i)
                {
                    var projectile = _projectiles[i];
                    if (ReferenceEquals(projectile.Root, root) && projectile.RebuildRevision <= revision
                        && projectile.Arrived)
                    {
                        FinishProjectile(i);
                    }
                }
            }
        }
    }

    private void FinishProjectile(int index)
    {
        var projectile = _projectiles[index];
        if (projectile.HasSurfaceHit)
            CreateImpactEffect(projectile.Destination, projectile.IsMud);
        projectile.Node.Remove();
        _projectiles.RemoveAt(index);
    }

    private static void SetOrientedWorldScale(Node node, Vector3f desiredScale)
    {
        // Node.SetWorldScale assumes aligned parent axes. CSG roots are non-uniformly scaled boxes, so account for
        // the brush rotation when converting the desired world-space cylinder dimensions into local scale.
        var parentScale = node.Parent!.WorldScale;
        var rotation = node.Rotation;
        var xAxis = parentScale * (rotation * Vector3f.Right);
        var yAxis = parentScale * (rotation * Vector3f.Up);
        var zAxis = parentScale * (rotation * Vector3f.Forward);
        node.SetScale(new Vector3f(
            desiredScale.X / xAxis.Length,
            desiredScale.Y / yAxis.Length,
            desiredScale.Z / zAxis.Length));
    }

    private void CreateImpactEffect(Vector3f position, bool isMud)
    {
        var node = _scene!.CreateChild("Impact flash");
        node.SetWorldPosition(position);
        node.SetScale(isMud ? 0.5f : 0.75f);
        var model = node.CreateComponent<StaticModel>();
        model.SetModel(_sphereModel);
        model.SetMaterial(isMud ? _mudMaterial : _explosionMaterial);
        model.SetViewMask(ForegroundViewMask);
        model.SetCastShadows(false);
        _impactEffects.Add(new ImpactEffect { Node = node, TimeToLive = 0.14f });
    }

    private void UpdateImpactEffects(float timeStep)
    {
        for (var i = _impactEffects.Count - 1; i >= 0; --i)
        {
            var effect = _impactEffects[i];
            effect.TimeToLive -= timeStep;
            effect.Node.SetScale(effect.Node.Scale * (1.0f + timeStep * 8.0f));
            if (effect.TimeToLive > 0.0f)
                continue;
            effect.Node.Remove();
            _impactEffects.RemoveAt(i);
        }
    }

    protected override void Setup(EngineParameters parameters)
    {
        parameters.ApplicationName = "RedBolt.Sample.CSG";
        parameters.Borderless = false;
        parameters.FullScreen = false;
        parameters.ResourcePaths = "SampleData.CSG;CoreData;SampleData";
        parameters.ResourcePrefixPaths = ".;..";
        parameters.WindowMaximize = false;
#if !REDBOLT_PLATFORM_BROWSER
        parameters.WindowHeight = 720;
        parameters.WindowWidth = 1280;
#endif
        parameters.WindowTitle = "RedBolt CSG Demolition Lab";
    }

    protected override void Stop()
    {
        Renderer.SetNumViewports(0);
        base.Stop();
    }

    private void DrawInstructions()
    {
        var windowOpen = true;
        ImGui.SetNextWindowPos(new Vector2f(10.0f, 10.0f), ImGuiCond.ImGuiCondAlways);
        if (ImGui.Begin("CSG Demolition Lab"u8, ref windowOpen,
                ImGuiWindowFlags.ImGuiWindowFlagsNoResize | ImGuiWindowFlags.ImGuiWindowFlagsNoSavedSettings))
        {
            ImGui.TextUnformatted("INNER WALL TEST CHAMBERS"u8);
            ImGui.Separator();
            for (var i = 0; i < _movePrompts!.Length - 1; ++i)
            {
                ImGui.Image(_movePrompts[i], PromptIconSize);
                ImGui.SameLine();
            }
            DrawPrompt(_movePrompts[^1], "Move"u8);
            DrawPrompt(_lookPrompt!, "Look (always active)"u8);
            DrawPrompt(_sprintPrompt!, "Sprint"u8);
            DrawPrompt(_explosivePrompt!, "Explosive: subtract from inner walls"u8);
            DrawPrompt(_mudPrompt!, "Mud: add to any structural surface"u8);
            DrawPrompt(_exitPrompt!, "Exit"u8);
            ImGui.TextUnformatted("Orange trim marks destructible partitions"u8);
        }
        ImGui.End();

        using var viewport = ImGui.GetMainViewport();
        using var drawList = ImGui.GetForegroundDrawList(viewport);
        var center = new Vector2f(Graphics.Width * 0.5f, Graphics.Height * 0.5f);
        const float gap = 4.0f;
        const float length = 11.0f;
        var color = new Color(0.25f, 0.9f, 1.0f).ToUInt();
        drawList.AddLine(center - new Vector2f(length, 0.0f), center - new Vector2f(gap, 0.0f), color, 2.0f);
        drawList.AddLine(center + new Vector2f(gap, 0.0f), center + new Vector2f(length, 0.0f), color, 2.0f);
        drawList.AddLine(center - new Vector2f(0.0f, length), center - new Vector2f(0.0f, gap), color, 2.0f);
        drawList.AddLine(center + new Vector2f(0.0f, gap), center + new Vector2f(0.0f, length), color, 2.0f);
    }

    private static void DrawPrompt(Texture2D texture, ReadOnlySpan<byte> label)
    {
        ImGui.Image(texture, PromptIconSize);
        ImGui.SameLine();
        Widgets.AlignTextToLastItem();
        ImGui.TextUnformatted(label);
    }

    public override void Dispose()
    {
        if (_viewport is not null)
            Renderer.SetNumViewports(0);
        _cameraNode = null;
        _gunNode = null;
        _octree = null;
        _projectiles.Clear();
        _impactEffects.Clear();
        _mudPlacements.Clear();
        _scene?.Dispose();
        _scene = null;
        _viewport?.Dispose();
        _viewport = null;
        base.Dispose();
    }

    private const uint WorldViewMask = 1;
    private const uint ForegroundViewMask = 2;
    private const string MudMaterialName = "Materials/CSG/Mud.xml";
    private const string MudOperandModelName = "Models/CSG/LowPolyMudBall.mdl";
    private const float EyeHeight = 1.7f;
    private const float MouseSensitivity = 0.1f;
    private const float WalkSpeed = 4.5f;
    private const float SprintSpeed = 8.0f;
    private const float ProjectileSpeed = 26.0f;
    private const float MaxProjectileDistance = 50.0f;
    private const int MaxOperationsPerStructure = 18;
    private const float MudStackStep = 0.5f;
    private const float MudMinimumSeparation = 0.42f;

    private readonly HashSet<uint> _destructibleStructures = [];
    private readonly Dictionary<uint, int> _operationCounts = [];
    private readonly List<Projectile> _projectiles = [];
    private readonly List<ImpactEffect> _impactEffects = [];
    private readonly List<MudPlacement> _mudPlacements = [];
    private readonly HashSet<CsgRoot> _stagedRoots = [];
    private Viewport? _viewport;
    private Scene? _scene;
    private Octree? _octree;
    private Node? _cameraNode;
    private Node? _gunNode;
    private Model? _sphereModel;
    private Model? _explosiveOperandModel;
    private Model? _mudOperandModel;
    private Material? _explosiveMaterial;
    private Material? _explosionMaterial;
    private Material? _mudMaterial;
    private Texture2D[]? _movePrompts;
    private Texture2D? _lookPrompt;
    private Texture2D? _sprintPrompt;
    private Texture2D? _explosivePrompt;
    private Texture2D? _mudPrompt;
    private Texture2D? _exitPrompt;
    private float _yaw;
    private float _pitch;
    private float _recoil;
    private static readonly Vector2f PromptIconSize = new(24.0f, 24.0f);
}
