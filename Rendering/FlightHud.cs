using System.Globalization;
using System.Numerics;
using Flanker.Flight;
using Raylib_cs;
using static Raylib_cs.Raylib;

namespace Flanker.Rendering;

public sealed class FlightHud
{
    private static readonly Color Ink = new(224, 232, 233, 255);
    private static readonly Color Secondary = new(158, 176, 183, 255);
    private static readonly Color Panel = new(22, 32, 39, 229);
    private static readonly Color Line = new(119, 141, 153, 100);
    private static readonly Color Caution = new(224, 197, 147, 255);
    private readonly Font font = GetFontDefault();
    private float scale = 1;

    public void Draw(FlightModel f, FlightCamera view, bool paused, bool help, bool vectors)
    {
        int width = GetScreenWidth(), height = GetScreenHeight();
        scale = Math.Clamp(Math.Min(width / 1440f, height / 900f), .65f, 1.75f);
        float w = width / scale, h = height / scale;
        // All UI coordinates are in a 1440x900 design space and scale with the window.
        Box(24, 24, 340, 86);
        Text("F L A N K E R", 42, 36, 29, Ink);
        Text("SU-27  /  FLIGHT LABORATORY", 43, 76, 12, Secondary);
        Rect(24, 24, 3, 86, Secondary);
        Text("01", 382, 38, 25, Ink);
        Text(f.OnGround ? "AIRFIELD / RUNWAY 36" : "FREE FLIGHT / MOUNTAIN BASIN", 382, 72, 11, Secondary);

        Box(w - 259, 24, 235, 86);
        Text(paused ? "PAUSED" : f.Crashed ? "FLIGHT ENDED" : "SIMULATION LIVE", w - 240, 39, 14,
            paused || f.Crashed ? Caution : Ink);
        Text($"{view.Name}  /  {(f.WindEnabled ? "WIND 5 M/S" : "ISA + CALM")}", w - 240, 67, 11, Secondary);
        Text($"120 HZ PHYSICS   {GetFPS(),3} FPS", w - 240, 87, 10, Secondary);

        DrawHeading(f, w);
        if (view.Mode == 1) DrawAttitude(f, w, h);
        else
        {
            Text("EXTERNAL VIEW", w / 2 - 51, 157, 10, new(223, 231, 232, 150));
            // A small corner frame leaves the airframe silhouette unobstructed.
            float x = w / 2, y = h * .48f;
            Stroke(x - 205, y - 125, x - 185, y - 125, Line);
            Stroke(x - 205, y - 125, x - 205, y - 105, Line);
            Stroke(x + 205, y + 125, x + 185, y + 125, Line);
            Stroke(x + 205, y + 125, x + 205, y + 105, Line);
        }

        float ias = f.Aero.Speed * MathF.Sqrt(Atmosphere.At(f.Position.Y).Density / 1.225f) * 3.6f;
        Tape(50, h * .45f, ias, 50, "IAS", "KM/H", false);
        Tape(w - 128, h * .45f, f.Position.Y, 200, "ALT", "M MSL", true);

        float panelY = h - 234;
        Box(24, panelY, 284, 160);
        Text("FLIGHT DATA", 42, panelY + 15, 11, Secondary);
        Data("MACH", f.Aero.Mach.ToString("0.00", CultureInfo.InvariantCulture), 42, panelY + 44);
        Data("AOA", $"{f.Aero.Alpha * 180 / MathF.PI,5:0.0} DEG", 42, panelY + 71);
        Data("LOAD", $"{f.LoadFactor,5:0.00} G", 42, panelY + 98);
        Data("V/S", $"{f.Velocity.Y:+0.0;-0.0;0.0} M/S", 42, panelY + 125);

        Box(w - 308, panelY, 284, 160);
        Text("POWERPLANT / AIRFRAME", w - 290, panelY + 15, 11, Secondary);
        Text($"{f.EngineSpool * 100:000}%", w - 290, panelY + 41, 30, Ink);
        Text(f.Afterburner ? "A/B ARMED" : "DRY / NO A/B", w - 193, panelY + 52, 10,
            f.Afterburner ? Caution : Secondary);
        Rect(w - 290, panelY + 80, 246, 3, Line);
        Rect(w - 290, panelY + 80, 246 * f.EngineSpool, 3, Ink);
        Text($"FUEL {f.Fuel,5:0} KG", w - 290, panelY + 96, 12, Ink);
        Text($"GEAR {(f.GearExtension > .99f ? "DOWN" : f.GearExtension < .01f ? "UP" : "TRANSIT")}", w - 160, panelY + 96, 11, Secondary);
        Text($"TRIM {f.PitchTrim * 100:+0.0;-0.0;0.0}%", w - 290, panelY + 126, 11, Secondary);
        Text(f.Airbrake ? "AIRBRAKE OUT" : $"RADALT {f.AltitudeAgl:0} M", w - 160, panelY + 126, 11,
            f.Airbrake ? Caution : Secondary);

        // Control position: the crosshair shows actual actuators, not just key presses.
        float cx = w / 2, cy = h - 132;
        Stroke(cx - 28, cy, cx + 28, cy, Line); Stroke(cx, cy - 28, cx, cy + 28, Line);
        Circle(cx + f.Aileron * 25, cy - f.Elevator * 25, 3, Ink);
        Text("CONTROL INPUT", cx - 45, cy + 39, 9, Secondary);

        Box(24, h - 56, w - 48, 32);
        Text("ARROWS  PITCH / ROLL     A D  RUDDER     W S  THROTTLE     V  CAMERA     H  CONTROLS", 41, h - 44, 11, Ink);
        Text($"{TimeSpan.FromSeconds(f.Time):mm\\:ss}  /  {(vectors ? "FORCES ON" : "SI UNITS")}", w - 196, h - 44, 11, Secondary);

        string? warning = f.Crashed ? f.Failure : f.Aero.Separation > .15f ? "STALL / REDUCE ANGLE OF ATTACK" :
            f.LoadFactor > 8.5f ? "HIGH G / UNLOAD" :
            f.AltitudeAgl < 140 && f.Velocity.Y < -8 ? "TERRAIN / PULL UP" : f.Fuel < 500 ? "LOW FUEL" : null;
        if (warning is not null)
        {
            float tw = Measure(warning, 14);
            Box(w / 2 - tw / 2 - 20, h * .24f, tw + 40, 38);
            Text(warning, w / 2 - tw / 2, h * .24f + 12, 14, Caution);
        }
        if (paused || f.Crashed)
        {
            string label = f.Crashed ? "FLIGHT ENDED" : "SIMULATION PAUSED";
            Box(w / 2 - 208, h / 2 - 45, 416, 102);
            Text(label, w / 2 - Measure(label, 23) / 2, h / 2 - 23, 23, Ink);
            string action = f.Crashed ? "R  AIR START     T  RUNWAY START" : "P  RESUME FLIGHT";
            Text(action, w / 2 - Measure(action, 12) / 2, h / 2 + 20, 12, Secondary);
        }
        if (help) DrawHelp(w, h);
    }

    private void DrawHeading(FlightModel f, float width)
    {
        float center = width / 2;
        float heading = f.Heading;
        for (int offset = -45; offset <= 45; offset += 5)
        {
            float degrees = MathF.Floor(heading / 5) * 5 + offset;
            float x = center + (degrees - heading) * 4;
            Stroke(x, 125, x, 125 + (degrees % 10 == 0 ? 9 : 4), Secondary);
            if (degrees % 10 == 0)
            {
                string label = (((int)degrees + 720) % 360).ToString("000");
                Text(label, x - Measure(label, 10) / 2, 108, 10, Secondary);
            }
        }
        Box(center - 29, 128, 58, 23);
        Text($"{heading:000}", center - 17, 132, 14, Ink);
    }

    private void DrawAttitude(FlightModel f, float w, float h)
    {
        Vector2 center = new(w / 2, h * .46f);
        float bank = -f.Bank;
        Vector2 Rotate(Vector2 p) => center + new Vector2(p.X * MathF.Cos(bank) - p.Y * MathF.Sin(bank),
            p.X * MathF.Sin(bank) + p.Y * MathF.Cos(bank));
        for (int a = -30; a <= 30; a += 5)
        {
            float y = (f.Pitch * 180 / MathF.PI - a) * 5;
            if (MathF.Abs(y) > 170) continue;
            float length = a == 0 ? 145 : 60;
            Vector2 p1 = Rotate(new(-length, y)), p2 = Rotate(new(-18, y));
            Vector2 p3 = Rotate(new(18, y)), p4 = Rotate(new(length, y));
            Stroke(p1.X, p1.Y, p2.X, p2.Y, Ink); Stroke(p3.X, p3.Y, p4.X, p4.Y, Ink);
            if (a != 0)
            {
                Vector2 label = Rotate(new(length + 8, y - 4));
                Text(Math.Abs(a).ToString(), label.X, label.Y, 11, Secondary);
            }
        }
        Stroke(center.X - 42, center.Y, center.X - 12, center.Y, Ink);
        Stroke(center.X + 12, center.Y, center.X + 42, center.Y, Ink);
        Stroke(center.X - 12, center.Y, center.X, center.Y + 7, Ink);
        Stroke(center.X, center.Y + 7, center.X + 12, center.Y, Ink);
        // Velocity-vector marker relative to the actual cockpit camera.
        if (f.Velocity.LengthSquared() > 10)
        {
            Vector3 local = f.ToLocal(Vector3.Normalize(f.Velocity));
            if (local.Z > .2f)
            {
                float x = center.X + local.X / local.Z * h * .68f;
                float y = center.Y - local.Y / local.Z * h * .68f;
                if (MathF.Abs(x - center.X) < 230 && MathF.Abs(y - center.Y) < 190)
                {
                    DrawCircleLines((int)(x * scale), (int)(y * scale), 8 * scale, Ink);
                    Stroke(x - 18, y, x - 8, y, Ink); Stroke(x + 8, y, x + 18, y, Ink);
                    Stroke(x, y - 8, x, y - 17, Ink);
                }
            }
        }
        // A restrained cockpit sill; the world remains visible above it.
        Rect(w / 2 - 208, h - 237, 416, 20, new(27, 37, 43, 255));
        Text("SU-27   /   EXPERIMENTAL FLIGHT MODEL", w / 2 - 172, h - 232, 10, Secondary);
    }

    private void DrawHelp(float w, float h)
    {
        float x = w / 2 - 272, y = h / 2 - 241;
        Box(x, y, 544, 482);
        Text("FLIGHT CONTROLS", x + 26, y + 23, 23, Ink);
        Text("DIRECT AXIS CONTROL / NO AUTOPILOT", x + 26, y + 60, 11, Secondary);
        string[] controls = [
            "UP / DOWN          Push nose down / pull nose up",
            "LEFT / RIGHT       Roll left / right",
            "A / D              Rudder + nosewheel steering",
            "W / S              Increase / decrease throttle",
            "LEFT SHIFT         Toggle afterburner (full throttle)",
            "Q / E              Nose-down / nose-up pitch trim",
            "G / B              Landing gear / airbrake",
            "SPACE              Hold wheel brakes",
            "V                  Chase / cockpit / orbit camera",
            "RMB + MOUSE        Rotate orbit camera",
            "MOUSE WHEEL        Camera distance",
            "N / TAB            Wind + gusts / force vectors",
            "P / H              Pause / close this panel",
            "R / T              Reset airborne / on runway"
        ];
        for (int i = 0; i < controls.Length; i++) Text(controls[i], x + 26, y + 96 + i * 23, 11, Ink);
        Text("TAKEOFF: full power + A/B, rotate gently near 260 km/h.", x + 26, y + 433, 11, Secondary);
        Text("Gear up after lift-off. Approach: 280-310 km/h, gear down.", x + 26, y + 455, 11, Secondary);
    }

    private void Tape(float x, float y, float value, int step, string label, string unit, bool right)
    {
        Text(label, x + 8, y - 103, 11, Ink);
        for (int offset = -2; offset <= 2; offset++)
        {
            float tick = MathF.Floor(value / step) * step + offset * step;
            float ty = y - (tick - value) / step * 36;
            Stroke(x + (right ? 0 : 63), ty, x + (right ? 9 : 72), ty, Secondary);
            Text(tick.ToString("0"), x + 12, ty - 6, 12, Secondary);
        }
        Box(x - 3, y - 17, 82, 34);
        Text(value.ToString("0"), x + 8, y - 10, 21, Ink);
        Text(unit, x + 8, y + 103, 10, Secondary);
    }

    private void Data(string label, string value, float x, float y)
    { Text(label, x, y, 11, Secondary); Text(value, x + 100, y - 2, 15, Ink); }
    private void Box(float x, float y, float w, float h)
    {
        Rect(x, y, w, h, Panel);
        DrawRectangleLinesEx(new(x * scale, y * scale, w * scale, h * scale), scale, Line);
    }
    private void Rect(float x, float y, float w, float h, Color c) =>
        DrawRectangleRec(new(x * scale, y * scale, w * scale, h * scale), c);
    private void Text(string text, float x, float y, float size, Color c) =>
        DrawTextEx(font, text, new(x * scale, y * scale), size * scale, scale, c);
    private float Measure(string text, float size) => MeasureTextEx(font, text, size * scale, scale).X / scale;
    private void Stroke(float x1, float y1, float x2, float y2, Color c) =>
        DrawLineEx(new(x1 * scale, y1 * scale), new(x2 * scale, y2 * scale), scale, c);
    private void Circle(float x, float y, float r, Color c) => DrawCircleV(new(x * scale, y * scale), r * scale, c);
}
