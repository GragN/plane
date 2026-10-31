using System.Globalization;
using System.Numerics;
using Flanker.Flight;
using Flanker.Rendering;
using Raylib_cs;
using static Raylib_cs.Raylib;

namespace Flanker;

public sealed class SimulatorApp
{
    public int Run(bool smokeTest, float opacity = WindowAppearance.DefaultOpacity)
    {
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
        if (OperatingSystem.IsLinux() && string.IsNullOrEmpty(Environment.GetEnvironmentVariable("DISPLAY"))
            && string.IsNullOrEmpty(Environment.GetEnvironmentVariable("WAYLAND_DISPLAY")))
        {
            Console.Error.WriteLine("A desktop display is required. Run locally in VS Code, or use Xvfb for --smoke-test.");
            return 1;
        }
        SetConfigFlags(ConfigFlags.Msaa4xHint | ConfigFlags.ResizableWindow | ConfigFlags.VSyncHint);
        InitWindow(1440, 900, "FLANKER | Su-27 Flight Laboratory");
        if (!IsWindowReady()) return 1;
        var appearance = new WindowAppearance(opacity);
        SetWindowMinSize(960, 640);
        SetTargetFPS(smokeTest ? 0 : 120);
        Rlgl.SetClipPlanes(.15, 60000);
        var flight = new FlightModel(ground: true);
        var view = new FlightCamera();
        var world = new WorldRenderer();
        var hud = new FlightHud();
        float accumulator = 0;
        bool paused = false, help = false, vectors = false;
        int frames = 0;

        try
        {
            while (!WindowShouldClose())
            {
                float elapsed = smokeTest ? 1f / 60 : Math.Clamp(GetFrameTime(), 0, .1f);
                appearance.Update();
                if (IsKeyPressed(KeyboardKey.P)) { paused = !paused; accumulator = 0; }
                if (IsKeyPressed(KeyboardKey.H)) help = !help;
                if (IsKeyPressed(KeyboardKey.Tab)) vectors = !vectors;
                if (IsKeyPressed(KeyboardKey.N)) flight.WindEnabled = !flight.WindEnabled;
                if (IsKeyPressed(KeyboardKey.F4))
                {
                    flight.Controls.Law = flight.Controls.Law == ControlLaw.Normal ? ControlLaw.Direct : ControlLaw.Normal;
                    Console.WriteLine($"CONTROL LAW: {flight.Controls.Law}");
                }
                if (IsKeyPressed(KeyboardKey.R)) { flight.Reset(true); view.Reset(); accumulator = 0; }
                if (IsKeyPressed(KeyboardKey.T)) { flight.Reset(true); view.Reset(); accumulator = 0; }
                if (!paused && !flight.Crashed)
                {
                    HandleControls(flight, elapsed);
                    accumulator += elapsed;
                    while (accumulator >= FlightModel.FixedStep)
                    {
                        flight.Step(); accumulator -= FlightModel.FixedStep;
                    }
                }
                view.Update(flight, elapsed);
                BeginDrawing();
                ClearBackground(new(183, 193, 196, 255));
                DrawRectangleGradientV(0, 0, GetScreenWidth(), GetScreenHeight(),
                    new(116, 135, 146, 255), new(213, 218, 217, 255));
                world.Draw(flight, view.Camera, view.Mode == 1, vectors);
                hud.Draw(flight, view, paused, help, vectors, appearance.Opacity);
                EndDrawing();
                frames++;
                if (smokeTest && frames is 180 or 300 or 420)
                {
                    Directory.CreateDirectory("artifacts");
                    string name = frames switch { 180 => "runway", 300 => "chase", _ => "cockpit" };
                    bool valid = !flight.Crashed && float.IsFinite(flight.Position.Y)
                        && (frames != 180 || (flight.OnGround && flight.GearExtension == 1
                            && flight.Throttle == 0 && flight.Velocity.Length() < .1f))
                        && SaveCapture($"artifacts/{name}.png");
                    Console.WriteLine($"RENDER SMOKE [{name}]: frame={frames}; t={flight.Time:0.00}s; altitude={flight.Position.Y:0.0}m; " +
                        $"speed={flight.Aero.Speed:0.0}m/s; {(valid ? "PASS" : "FAIL")}");
                    if (!valid) return 1;
                    if (frames == 420) return 0;
                    flight.Reset(false);
                    view.SetMode(frames == 300 ? 1 : 0);
                    accumulator = 0;
                }
            }
        }
        finally { CloseWindow(); }
        return smokeTest ? 1 : 0;
    }

    private static bool SaveCapture(string path)
    {
        Image capture = LoadImageFromScreen();
        try
        {
            var colors = new HashSet<(byte, byte, byte)>();
            for (int y = 10; y < capture.Height; y += 30)
                for (int x = 10; x < capture.Width; x += 30)
                {
                    Color c = GetImageColor(capture, x, y);
                    colors.Add((c.R, c.G, c.B));
                }
            return colors.Count > 30 && ExportImage(capture, path) && File.Exists(path);
        }
        finally { UnloadImage(capture); }
    }

    private static void HandleControls(FlightModel f, float dt)
    {
        static float Axis(KeyboardKey positive, KeyboardKey negative) =>
            (IsKeyDown(positive) ? 1 : 0) - (IsKeyDown(negative) ? 1 : 0);
        float precision = IsKeyDown(KeyboardKey.LeftControl) || IsKeyDown(KeyboardKey.RightControl) ? .25f : 1;
        f.ElevatorInput = Axis(KeyboardKey.Down, KeyboardKey.Up) * precision;
        f.RollInput = Axis(KeyboardKey.Left, KeyboardKey.Right) * precision;
        f.RudderInput = Axis(KeyboardKey.D, KeyboardKey.A) * precision;
        f.Throttle += Axis(KeyboardKey.W, KeyboardKey.S) * dt * .22f;
        f.PitchTrim += Axis(KeyboardKey.E, KeyboardKey.Q) * dt * .035f;
        f.WheelBrake = IsKeyDown(KeyboardKey.Space);
        if (IsKeyPressed(KeyboardKey.G)) f.GearDown = !f.GearDown;
        if (IsKeyPressed(KeyboardKey.B)) f.Airbrake = !f.Airbrake;
        if (IsKeyPressed(KeyboardKey.K)) f.ParkingBrake = !f.ParkingBrake;
        if (IsKeyPressed(KeyboardKey.F1)) f.LeftEngineEnabled = !f.LeftEngineEnabled;
        if (IsKeyPressed(KeyboardKey.F2)) f.RightEngineEnabled = !f.RightEngineEnabled;
        if (IsKeyPressed(KeyboardKey.LeftShift)) f.Afterburner = !f.Afterburner;

        if (IsGamepadAvailable(0))
        {
            static float Deadzone(float v) => MathF.Abs(v) < .02f ? 0 : MathF.CopySign((MathF.Abs(v) - .02f) / .98f, v);
            f.RollInput = Math.Clamp(f.RollInput + Deadzone(GetGamepadAxisMovement(0, GamepadAxis.LeftX)), -1, 1);
            f.ElevatorInput = Math.Clamp(f.ElevatorInput + Deadzone(GetGamepadAxisMovement(0, GamepadAxis.LeftY)), -1, 1);
            f.RudderInput = Math.Clamp(f.RudderInput + Deadzone(GetGamepadAxisMovement(0, GamepadAxis.RightX)), -1, 1);
        }
    }
}
