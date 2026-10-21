using System.Numerics;
using Flanker.Flight;
using Raylib_cs;
using static Raylib_cs.Raylib;

namespace Flanker.Rendering;

public sealed class AircraftMesh
{
    private readonly record struct Face(Vector3 A, Vector3 B, Vector3 C, Color Color);
    private readonly List<Face> faces = [];
    private static readonly Color Skin = new(139, 148, 153, 255);
    private static readonly Color Wing = new(151, 159, 163, 255);
    private static readonly Color Dark = new(46, 55, 61, 255);
    private static readonly Vector3 Sun = Vector3.Normalize(new(-.4f, .85f, .3f));

    public AircraftMesh()
    {
        // Su-27 planform: long radome, blended LERX, swept wings, widely spaced
        // nacelles, twin outward-canted fins and separate horizontal stabilizers.
        Loft(0, [(-9.5f, .05f, .45f, .40f), (-7f, .05f, .9f, .55f),
            (-3f, .05f, 1.55f, .65f), (1.5f, .12f, 1.45f, .65f),
            (4.7f, .25f, .95f, .8f), (7f, .15f, .68f, .65f),
            (9f, .05f, .38f, .42f), (11.1f, 0, .015f, .015f)], Skin);
        Loft(0, [(7.1f, .14f, .67f, .64f), (9f, .05f, .39f, .43f),
            (11.12f, 0, .02f, .02f)], new(91, 103, 111, 255));

        foreach (int sign in new[] { -1, 1 })
        {
            Vector3 P(float x, float y, float z) => new(x * sign, y, z);
            // Leading-edge root extensions flow from the fuselage into the wing.
            Slab([P(.65f, .3f, 6), P(1.65f, .15f, 1.3f), P(3.1f, .08f, -1.8f),
                P(1.4f, .1f, -4.8f)], .13f, Skin);
            Slab([P(1.3f, .08f, 1.4f), P(7.0f, -.05f, -3.4f), P(7.2f, -.05f, -5.1f),
                P(2.0f, .04f, -5.0f), P(1.5f, .1f, -3.4f)], .12f, Wing);
            // Wingtip launch rails, no weapons.
            Loft(sign * 7.12f, [(-5.7f, -.04f, .07f, .07f),
                (-2.7f, -.04f, .07f, .07f)], Dark);
            Loft(sign * 1.23f, [(-9.7f, -.43f, .56f, .55f),
                (-8.4f, -.43f, .67f, .62f), (-3.8f, -.40f, .73f, .68f),
                (.9f, -.45f, .64f, .55f)], new(117, 127, 132, 255));
            Quad(P(.60f, -.91f, 1.02f), P(1.85f, -.91f, 1.02f),
                P(1.85f, -.04f, 1.02f), P(.60f, -.04f, 1.02f), Dark);
            // Vertical tails, canted outboard.
            Slab([P(1.9f, .2f, -4.5f), P(2.36f, 3.1f, -6.65f),
                P(2.5f, 3.05f, -8.15f), P(2.0f, .2f, -8.9f)], .075f,
                new(132, 143, 149, 255), horizontal: false);
            // Low-contrast fin identifiers.
            Quad(P(2.32f, 2.65f, -6.5f), P(2.47f, 2.65f, -7.98f),
                P(2.43f, 2.4f, -8.05f), P(2.28f, 2.4f, -6.65f), Dark);
            Slab([P(1.3f, -.8f, -5.7f), P(1.5f, -1.6f, -7.5f),
                P(1.5f, -1.5f, -8.4f), P(1.3f, -.7f, -8.8f)], .035f,
                new(98, 110, 116, 255), horizontal: false);
        }

        // Smoked canopy is faceted and opaque; no external assets are required.
        Loft(0, [(2.9f, .75f, .23f, .03f), (3.8f, .92f, .57f, .55f),
            (5.4f, .95f, .52f, .70f), (6.5f, .8f, .29f, .30f),
            (6.9f, .7f, .03f, .01f)], new(47, 62, 70, 255));
        // Dorsal spine and tail sting.
        Loft(0, [(-10.7f, .2f, .06f, .07f), (-7f, .32f, .34f, .32f),
            (-1.5f, .63f, .38f, .25f), (2.9f, .75f, .24f, .12f)], Skin);
    }

    private void Triangle(Vector3 a, Vector3 b, Vector3 c, Color color) => faces.Add(new(a, b, c, color));
    private void Quad(Vector3 a, Vector3 b, Vector3 c, Vector3 d, Color color)
    { Triangle(a, b, c, color); Triangle(a, c, d, color); }

    private void Slab(Vector3[] outline, float thickness, Color color, bool horizontal = true)
    {
        Vector3 offset = (horizontal ? Vector3.UnitY : Vector3.UnitX) * thickness / 2;
        for (int i = 1; i < outline.Length - 1; i++)
        {
            Triangle(outline[0] + offset, outline[i] + offset, outline[i + 1] + offset, color);
            Triangle(outline[0] - offset, outline[i + 1] - offset, outline[i] - offset, color);
        }
        for (int i = 0; i < outline.Length; i++)
        {
            int j = (i + 1) % outline.Length;
            Quad(outline[i] - offset, outline[j] - offset, outline[j] + offset, outline[i] + offset, color);
        }
    }

    private void Loft(float x, (float Z, float Y, float W, float H)[] rings, Color color)
    {
        const int count = 12;
        Vector3 Vertex(int ring, int k)
        {
            var r = rings[ring]; float a = k * MathF.Tau / count;
            return new(x + MathF.Cos(a) * r.W, r.Y + MathF.Sin(a) * r.H, r.Z);
        }
        for (int j = 0; j < rings.Length - 1; j++)
            for (int k = 0; k < count; k++)
                Quad(Vertex(j, k), Vertex(j, (k + 1) % count),
                    Vertex(j + 1, (k + 1) % count), Vertex(j + 1, k), color);
        for (int k = 0; k < count; k++)
        {
            Triangle(new(x, rings[0].Y, rings[0].Z), Vertex(0, k), Vertex(0, (k + 1) % count), color);
            int n = rings.Length - 1;
            Triangle(new(x, rings[n].Y, rings[n].Z), Vertex(n, k), Vertex(n, (k + 1) % count), color);
        }
    }

    public void Draw(FlightModel f)
    {
        Vector3 World(Vector3 v) => f.BodyPointToWorld(v);
        foreach (var face in faces)
        {
            Vector3 a = World(face.A), b = World(face.B), c = World(face.C);
            Vector3 normal = Vector3.Cross(b - a, c - a);
            float light = normal.LengthSquared() < 1e-8f ? .7f :
                .55f + .45f * MathF.Abs(Vector3.Dot(Vector3.Normalize(normal), Sun));
            DrawFace(a, b, c, Shade(face.Color, light));
        }
        foreach (int side in new[] { -1, 1 })
        {
            Vector3[] stabilizer = [new(side * 1.3f, .05f, -5.8f), new(side * 4.5f, .05f, -8),
                new(side * 4.7f, .05f, -9.3f), new(side * 1.5f, .05f, -8.65f)];
            Vector3 pivot = new(side * 2.2f, .05f, -7.7f);
            for (int i = 0; i < stabilizer.Length; i++)
                stabilizer[i] = World(pivot + Vector3.Transform(stabilizer[i] - pivot,
                    Quaternion.CreateFromAxisAngle(Vector3.UnitX, f.Elevator * .36f)));
            DrawFace(stabilizer[0], stabilizer[1], stabilizer[2], Wing);
            DrawFace(stabilizer[0], stabilizer[2], stabilizer[3], Wing);
            DrawLine3D(stabilizer[0], stabilizer[1], Dark);
            DrawLine3D(World(new(side * 3.1f, .11f, -4.65f)), World(new(side * 6.8f, .03f, -4.8f)), Dark);
            // Nozzles are hollow dark discs surrounded by metallic petals.
            Vector3 center = new(side * 1.23f, -.43f, -9.73f);
            Color exhaust = f.Afterburner && f.EngineSpool > .95f ? new(208, 203, 184, 255) : new(26, 31, 35, 255);
            for (int k = 0; k < 16; k++)
            {
                float a = k * MathF.Tau / 16, b = (k + 1) * MathF.Tau / 16;
                DrawFace(World(center), World(center + new Vector3(MathF.Cos(a), MathF.Sin(a), 0) * .48f),
                    World(center + new Vector3(MathF.Cos(b), MathF.Sin(b), 0) * .48f), exhaust);
            }
            if (f.GearExtension > .05f)
            {
                Vector3 top = new(side * 2.4f, -.5f, -1.7f);
                Vector3 wheel = new(side * 2.4f, -.5f - 1.85f * f.GearExtension
                    + f.GearCompression[side < 0 ? 1 : 2], -1.7f);
                DrawCylinderEx(World(top), World(wheel), .08f, .08f, 7, Wing);
                DrawCylinderEx(World(wheel - Vector3.UnitX * .16f), World(wheel + Vector3.UnitX * .16f),
                    .4f, .4f, 12, Dark);
            }
        }
        DrawLine3D(World(new(0, 0, 11.1f)), World(new(0, 0, 12.1f)), Skin);
        if (f.GearExtension > .05f)
        {
            Vector3 wheel = new(0, -.5f - 1.95f * f.GearExtension + f.GearCompression[0], 5.2f);
            DrawCylinderEx(World(new(0, -.4f, 5.2f)), World(wheel), .065f, .065f, 8, Wing);
            DrawCylinderEx(World(wheel - Vector3.UnitX * .12f), World(wheel + Vector3.UnitX * .12f), .30f, .30f, 12, Dark);
        }
    }

    public static Color Shade(Color c, float light) => new((byte)Math.Clamp(c.R * light, 0, 255),
        (byte)Math.Clamp(c.G * light, 0, 255), (byte)Math.Clamp(c.B * light, 0, 255), c.A);

    public static void DrawFace(Vector3 a, Vector3 b, Vector3 c, Color color)
    {
        DrawTriangle3D(a, b, c, color);
        DrawTriangle3D(c, b, a, color);
    }
}
