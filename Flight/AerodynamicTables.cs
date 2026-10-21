namespace Flanker.Flight;

// These are explicit engineering estimates, NOT measured Su-27 aerodynamic data.
// Linear interpolation preserves bounded values; separation smoothsteps the stall transition.
public static class AerodynamicTables
{
    private static readonly (float, float)[] LiftMach =
        [(0, 1), (.6f, 1.03f), (.85f, 1.12f), (1, 1.08f), (1.2f, .88f), (1.6f, .73f), (2.35f, .62f), (3, .5f)];
    private static readonly (float, float)[] WaveMach =
        [(0, 0), (.72f, 0), (.85f, .003f), (1, .025f), (1.15f, .023f), (1.5f, .015f), (2.35f, .009f), (3, .007f)];

    public static float Interpolate(ReadOnlySpan<(float X, float Y)> table, float x)
    {
        if (x <= table[0].X) return table[0].Y;
        for (int i = 1; i < table.Length; i++)
            if (x < table[i].X)
                return table[i - 1].Y + (table[i].Y - table[i - 1].Y)
                    * (x - table[i - 1].X) / (table[i].X - table[i - 1].X);
        return table[^1].Y;
    }

    public static float LiftFactor(float mach) => Interpolate(LiftMach, mach);
    public static float WaveDrag(float mach) => Interpolate(WaveMach, mach);
    public static float Smooth(float a, float b, float x)
    {
        float t = Math.Clamp((x - a) / (b - a), 0, 1);
        return t * t * (3 - 2 * t);
    }
    public static float SkinFriction(float reynolds, float mach)
    {
        float re = Math.Max(10000, reynolds);
        float turbulent = .455f / (MathF.Pow(MathF.Log10(re), 2.58f) * MathF.Pow(1 + .144f * mach * mach, .65f));
        float laminar = 1.328f / MathF.Sqrt(re);
        return laminar + (turbulent - laminar) * Smooth(100000, 900000, re);
    }
}
