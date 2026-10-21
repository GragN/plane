using System.Numerics;

namespace Flanker.Flight;

public readonly record struct Air(float Density, float SoundSpeed, float Temperature)
{
    public float DynamicViscosity => 1.716e-5f * MathF.Pow(Temperature / 273.15f, 1.5f)
        * (273.15f + 110.4f) / (Temperature + 110.4f);
}

public static class Atmosphere
{
    public const float Gravity = 9.80665f;

    // ISA troposphere + isothermal lower stratosphere; SI throughout the physics.
    public static Air At(float altitude)
    {
        float h = Math.Clamp(altitude, -500, 32000);
        float t = 288.15f - .0065f * Math.Min(h, 11000);
        float p = 101325 * MathF.Pow(t / 288.15f, 5.25588f);
        if (h > 11000) p *= MathF.Exp(-Gravity * (h - 11000) / (287.05287f * t));
        return new(p / (287.05287f * t), MathF.Sqrt(1.4f * 287.05287f * t), t);
    }

    public static Vector3 Wind(double seconds, float altitude, bool enabled)
        => WindAt(seconds, new(0, altitude, 0), enabled);

    public static Vector3 WindAt(double seconds, Vector3 position, bool enabled)
    {
        if (!enabled) return Vector3.Zero;
        float t = (float)seconds;
        float strength = Math.Clamp(position.Y / 200, 0, 1);
        float phase = position.X * .012f + position.Z * .004f;
        return new Vector3(5 + 1.8f * MathF.Sin(t * .19f + phase),
            .8f * MathF.Sin(t * .7f + phase * 1.7f) + .3f * MathF.Sin(t * 2.3f - position.X * .09f),
            2 + 1.2f * MathF.Sin(t * .31f - phase)) * strength;
    }
}
