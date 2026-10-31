using System.Globalization;
using Raylib_cs;
using static Raylib_cs.Raylib;

namespace Flanker.Rendering;

public sealed class WindowAppearance
{
    public const float DefaultOpacity = .05f;
    public const float MinimumOpacity = .05f;
    public float Opacity { get; private set; }
    public WindowAppearance(float opacity)
    {
        Apply(opacity);
        Console.WriteLine($"WINDOW OPACITY REQUEST: {Opacity:0.00} ({(1 - Opacity) * 100:0}% transparent); desktop compositor required.");
    }
    private void Apply(float opacity)
    {
        Opacity = Math.Clamp(opacity, MinimumOpacity, 1);
        SetWindowOpacity(Opacity); // Whole native window, including the 3D scene and instruments.
    }
    public void Update()
    {
        if (IsKeyPressed(KeyboardKey.F8)) Apply(Opacity - .1f);
        if (IsKeyPressed(KeyboardKey.F9)) Apply(Opacity + .1f);
    }
    public static float FromArguments(string[] args)
    {
        int index = Array.IndexOf(args, "--opacity");
        if (index < 0) return DefaultOpacity; // 95% transparency = 5% opacity.
        if (index + 1 >= args.Length || !float.TryParse(args[index + 1], NumberStyles.Float,
            CultureInfo.InvariantCulture, out float value) || !float.IsFinite(value) || value < MinimumOpacity || value > 1)
            throw new ArgumentException("--opacity requires a number from 0.05 to 1.0 (e.g. --opacity 0.05)");
        return value;
    }
}
