using System.Numerics;
using Flanker.Flight;
using Flanker.Simulation;
using Raylib_cs;
using static Raylib_cs.Raylib;

namespace Flanker.Rendering;

public sealed class WorldRenderer
{
    private readonly AircraftMesh aircraft = new();
    private readonly List<(Vector3 A, Vector3 B, Vector3 C, Color Color)> terrain = [];
    private int tileX = int.MinValue, tileZ = int.MinValue;
    private static readonly Color Fog = new(183, 193, 196, 255);

    public void Draw(FlightModel f, Camera3D camera, bool cockpit, bool vectors)
    {
        BeginMode3D(camera);
        DrawTerrain(f.Position);
        DrawAirfield();
        if (!cockpit) aircraft.Draw(f);
        if (vectors)
        {
            foreach (SurfaceLoad load in f.SurfaceLoads)
            {
                Vector3 root = f.BodyPointToWorld(load.Position);
                DrawLine3D(root, root + f.ToWorld(load.Force) / 9000,
                    load.Separation > .2f ? new(201, 174, 136, 255) : new(191, 214, 219, 255));
            }
            DrawLine3D(f.Position, f.Position + f.Up * f.Aero.Lift / 18000, new(221, 226, 219, 255));
            DrawLine3D(f.Position, f.Position + f.Forward * f.Thrust / 10000, new(128, 156, 169, 255));
            DrawLine3D(f.Position, f.Position + f.Velocity * .15f, new(188, 179, 148, 255));
        }
        EndMode3D();
    }

    private void DrawTerrain(Vector3 position)
    {
        const float size = 650;
        int x = (int)MathF.Floor(position.X / size), z = (int)MathF.Floor(position.Z / size);
        if (x != tileX || z != tileZ)
        {
            tileX = x; tileZ = z; terrain.Clear();
            for (int i = x - 36; i < x + 36; i++)
                for (int j = z - 36; j < z + 36; j++)
                {
                    Vector3 Point(int px, int pz) => new(px * size,
                        Terrain.Height(px * size, pz * size) - .08f, pz * size);
                    Vector3 a = Point(i, j), b = Point(i + 1, j), c = Point(i + 1, j + 1), d = Point(i, j + 1);
                    uint hash = unchecked((uint)(i * 73856093 ^ j * 19349663));
                    float tone = .92f + (hash % 31) / 220f;
                    Color color = AircraftMesh.Shade(new(106, 116, 115, 255), tone);
                    terrain.Add((a, b, c, color)); terrain.Add((a, c, d, AircraftMesh.Shade(color, .97f)));
                }
        }
        foreach (var tri in terrain)
        {
            Vector3 center = (tri.A + tri.B + tri.C) / 3;
            float distance = Vector2.Distance(new(position.X, position.Z), new(center.X, center.Z));
            float fog = Math.Clamp((distance - 1800) / 18000, 0, 1);
            fog = 1 - MathF.Pow(1 - fog, 2);
            AircraftMesh.DrawFace(tri.A, tri.B, tri.C, Blend(tri.Color, Fog, fog));
        }
    }

    private static void DrawAirfield()
    {
        Color asphalt = new(66, 76, 80, 255), paint = new(179, 186, 184, 255);
        DrawCube(new(0, -.05f, 0), Terrain.RunwayWidth, .1f, Terrain.RunwayLength, asphalt);
        DrawCube(new(120, -.03f, 0), 16, .1f, 2450, new(85, 94, 95, 255));
        foreach (int sign in new[] { -1, 1 })
        {
            DrawCube(new(sign * 25, .025f, 0), .6f, .035f, 2780, paint);
            for (int z = -1320; z < 1400; z += 80)
                DrawCube(new(sign * 29, .08f, z), .7f, .25f, .7f, paint);
            for (int i = 0; i < 5; i++)
                DrawCube(new(sign * (4 + i * 3.8f), .025f, -1310), 2.4f, .04f, 40, paint);
            for (int i = 0; i < 5; i++)
                DrawCube(new(sign * (4 + i * 3.8f), .025f, 1310), 2.4f, .04f, 40, paint);
        }
        for (int z = -1200; z < 1250; z += 100)
            DrawCube(new(0, .025f, z), 1.3f, .04f, 38, paint);
        for (int z = -950; z < 1100; z += 450)
            DrawCube(new(60, -.02f, z), 90, .1f, 18, new(85, 94, 95, 255));
        DrawCube(new(230, -.01f, -600), 190, .08f, 750, new(111, 120, 120, 255));
        for (int i = 0; i < 5; i++)
        {
            Vector3 hangar = new(310, 8, -920 + i * 145);
            DrawCube(hangar, 60, 16, 75, new(143, 151, 153, 255));
            DrawCube(hangar + new Vector3(-30.1f, -2, 0), .2f, 11, 55, new(69, 80, 86, 255));
            DrawCube(hangar + new Vector3(0, 8.5f, 0), 62, 1, 77, new(108, 119, 124, 255));
        }
        DrawCube(new(210, 14, 70), 12, 28, 12, new(139, 149, 152, 255));
        DrawCube(new(210, 31, 70), 21, 7, 18, new(55, 71, 80, 255));
        DrawCube(new(210, 35, 70), 23, 1, 20, new(150, 158, 158, 255));
        DrawCylinderEx(new(210, 35, 70), new(210, 46, 70), .2f, .2f, 6, paint);
        // A long service road gives a clear visual speed/height reference.
        DrawCube(new(-600, -.025f, 0), 10, .08f, 7000, new(89, 100, 101, 255));
    }

    private static Color Blend(Color a, Color b, float t) => new(
        (byte)(a.R + (b.R - a.R) * t), (byte)(a.G + (b.G - a.G) * t),
        (byte)(a.B + (b.B - a.B) * t), (byte)255);
}
