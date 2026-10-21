namespace Flanker.Simulation;

public static class Terrain
{
    public const float RunwayLength = 2800;
    public const float RunwayWidth = 55;

    public static float Height(float x, float z)
    {
        // A flat airfield fades continuously into a procedural mountain basin.
        float radius = MathF.Sqrt(x * x + z * z);
        float blend = Math.Clamp((radius - 2500) / 5500, 0, 1);
        blend *= blend * (3 - 2 * blend);
        float ridges = 310 + 240 * MathF.Sin(x * .00043f + 1.2f)
            * MathF.Cos(z * .00037f) + 140 * MathF.Sin(x * .00091f + z * .00067f)
            + 65 * MathF.Cos(z * .0021f - x * .0013f);
        return Math.Max(0, ridges * blend);
    }

    public static bool OnRunway(float x, float z) => MathF.Abs(x) < RunwayWidth / 2
        && MathF.Abs(z) < RunwayLength / 2;
}
