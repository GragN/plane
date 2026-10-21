using System.Numerics;
using Flanker.Flight;
using Raylib_cs;
using static Raylib_cs.Raylib;

namespace Flanker.Rendering;

public sealed class FlightCamera
{
    public int Mode { get; private set; }
    public string Name => Mode switch { 1 => "COCKPIT", 2 => "ORBIT", _ => "CHASE" };
    public Camera3D Camera { get; private set; }
    private float orbit = .40f, elevation = .21f, distance = 34;
    private bool initialized;

    public void Reset() => initialized = false;

    public void SetMode(int mode) { Mode = mode % 3; initialized = false; }

    public void Update(FlightModel f, float dt)
    {
        if (IsKeyPressed(KeyboardKey.V)) { Mode = (Mode + 1) % 3; initialized = false; }
        distance = Math.Clamp(distance - GetMouseWheelMove() * 3, 22, 100);
        if (Mode == 2 && IsMouseButtonDown(MouseButton.Right))
        {
            Vector2 delta = GetMouseDelta();
            orbit += delta.X * .006f;
            elevation = Math.Clamp(elevation + delta.Y * .005f, -.15f, 1.25f);
        }
        Vector3 pos, target, up;
        if (Mode == 1)
        {
            pos = f.Position + f.ToWorld(new(0, 1.28f, 5.2f));
            target = pos + f.Forward * 400;
            up = f.Up;
        }
        else
        {
            float heading = MathF.Atan2(f.Forward.X, f.Forward.Z);
            float angle = heading + orbit;
            pos = f.Position + new Vector3(-MathF.Sin(angle) * MathF.Cos(elevation),
                MathF.Sin(elevation), -MathF.Cos(angle) * MathF.Cos(elevation)) * distance;
            pos.Y = Math.Max(pos.Y, Simulation.Terrain.Height(pos.X, pos.Z) + 2);
            target = f.Position + f.Forward * 1.5f;
            up = Vector3.UnitY;
        }
        float blend = initialized && Mode != 1 ? 1 - MathF.Exp(-dt * 9) : 1;
        Camera = new Camera3D
        {
            Position = Vector3.Lerp(Camera.Position, pos, blend),
            Target = Vector3.Lerp(Camera.Target, target, blend),
            Up = up,
            FovY = Mode == 1 ? 72 : 52,
            Projection = CameraProjection.Perspective
        };
        initialized = true;
    }
}
