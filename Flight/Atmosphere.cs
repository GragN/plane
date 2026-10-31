using System.Numerics;
using Flanker.Simulation;

namespace Flanker.Flight;

public readonly record struct Air(float Density, float SoundSpeed, float Temperature)
{
    public float DynamicViscosity => 1.716e-5f * MathF.Pow(Temperature / 273.15f, 1.5f)
        * (273.15f + 110.4f) / (Temperature + 110.4f);
}

public readonly record struct WindSettings(Vector3 WindAt10Meters, float Turbulence = 1)
{
    public static WindSettings Default => new(new(5, 0, 2));
}

public static class Atmosphere
{
    public const float Gravity = 9.80665f;
    private readonly record struct GustMode(Vector3 WaveNumber, Vector3 Direction, float Phase, float Weight);
    private static readonly GustMode[] Gusts = CreateGusts();

    private static GustMode[] CreateGusts()
    {
        // A fixed spatial spectrum: repeatable, continuous and independent of frame rate.
        // Each polarization is perpendicular to its wave vector (solenoidal field).
        var random = new Random(2701);
        Vector3 Direction() => Vector3.Normalize(new Vector3(
            (float)random.NextDouble() * 2 - 1, (float)random.NextDouble() * 2 - 1,
            (float)random.NextDouble() * 2 - 1));
        return Enumerable.Range(0, 12).Select(i =>
        {
            float length = 25 * MathF.Pow(40, i / 11f);
            Vector3 k = Direction();
            Vector3 polarization = Vector3.Normalize(Vector3.Cross(k, Direction()));
            return new GustMode(k * (MathF.Tau / length), polarization,
                (float)random.NextDouble() * MathF.Tau, MathF.Pow(length / 1000, 1f / 3));
        }).ToArray();
    }

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

    public static Vector3 WindAt(double seconds, Vector3 position, bool enabled, WindSettings? settings = null)
    {
        if (!enabled) return Vector3.Zero;
        WindSettings weather = settings ?? WindSettings.Default;
        float height = Math.Max(0, position.Y - Terrain.Height(position.X, position.Z));
        const float roughness = .03f; // short grass / open airfield; metres
        float profile = Math.Min(1.8f, MathF.Log(1 + height / roughness) / MathF.Log(1 + 10 / roughness));
        Vector3 mean = weather.WindAt10Meters * profile;
        if (weather.Turbulence <= 0) return mean;
        Vector3 advected = position - weather.WindAt10Meters * (float)seconds;
        Vector3 gust = Vector3.Zero;
        float energy = 0;
        foreach (GustMode mode in Gusts)
        {
            gust += mode.Direction * (mode.Weight * MathF.Sin(Vector3.Dot(mode.WaveNumber, advected) + mode.Phase));
            energy += mode.Weight * mode.Weight;
        }
        gust *= weather.Turbulence * 1.6f / MathF.Sqrt(energy);
        // The surface constrains vertical eddies, while horizontal crosswind persists
        // at wheel/wing height. Height is AGL, including above mountain terrain.
        gust.X *= Math.Min(1, profile); gust.Z *= Math.Min(1, profile);
        gust.Y *= 1 - MathF.Exp(-height / 25);
        return mean + gust;
    }
}
