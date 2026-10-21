using System.Numerics;

namespace Flanker.Flight;

public readonly record struct Matrix3(Vector3 X, Vector3 Y, Vector3 Z)
{
    public Vector3 Multiply(Vector3 v) => X * v.X + Y * v.Y + Z * v.Z;
    public Vector3 Solve(Vector3 v)
    {
        float determinant = Vector3.Dot(X, Vector3.Cross(Y, Z));
        if (MathF.Abs(determinant) < 1e-8f) throw new InvalidOperationException("Singular inertia/trim matrix");
        return new Vector3(Vector3.Dot(v, Vector3.Cross(Y, Z)), Vector3.Dot(X, Vector3.Cross(v, Z)),
            Vector3.Dot(X, Vector3.Cross(Y, v))) / determinant;
    }

    public Matrix3 AddPointMass(float mass, Vector3 r) => new(
        X + mass * new Vector3(r.Y * r.Y + r.Z * r.Z, -r.X * r.Y, -r.X * r.Z),
        Y + mass * new Vector3(-r.X * r.Y, r.X * r.X + r.Z * r.Z, -r.Y * r.Z),
        Z + mass * new Vector3(-r.X * r.Z, -r.Y * r.Z, r.X * r.X + r.Y * r.Y));
}

public readonly record struct MassProperties(float Mass, Vector3 Center, Matrix3 Inertia)
{
    public readonly record struct Component(string Name, float Mass, Vector3 Center, Vector3 Size);
    // Coarse structural mass model in the same datum/axes as the visual airframe.
    // These component weights are estimates; their sum is the published empty mass.
    // Inertias follow from dimensions and the parallel-axis theorem, not handling gains.
    public static IReadOnlyList<Component> Components { get; } = new Component[]
    {
        new("Forward fuselage", 3200, new(0, .1f, 5.2f), new(1.8f, 1.6f, 10)),
        new("Center fuselage", 4800, new(0, -.1f, 0), new(3.8f, 1.7f, 9)),
        new("Rear structure", 1100, new(0, .1f, -5.8f), new(3.2f, 1.4f, 7)),
        new("Left wing", 1000, new(-3.8f, 0, -2), new(6, .15f, 4.6f)),
        new("Right wing", 1000, new(3.8f, 0, -2), new(6, .15f, 4.6f)),
        new("Left engine", 1550, new(-1.23f, -.43f, -4.8f), new(1.25f, 1.25f, 4.9f)),
        new("Right engine", 1550, new(1.23f, -.43f, -4.8f), new(1.25f, 1.25f, 4.9f)),
        new("Left empennage", 500, new(-2.5f, 1.1f, -7.6f), new(3.8f, 2.5f, 3)),
        new("Right empennage", 500, new(2.5f, 1.1f, -7.6f), new(3.8f, 2.5f, 3)),
        new("Nose gear", 250, new(0, -1.4f, 5.2f), new(.6f, 2, .8f)),
        new("Left main gear", 425, new(-2.4f, -1.2f, -1.7f), new(.8f, 2, 1.2f)),
        new("Right main gear", 425, new(2.4f, -1.2f, -1.7f), new(.8f, 2, 1.2f))
    };
    private static readonly (Vector3 Position, float Fraction)[] Tanks =
    [ (new(0, -.25f, 4.1f), .30f), (new(0, -.4f, 1.1f), .40f), (new(0, -.15f, -5), .30f) ];

    public static MassProperties FromFuel(float fuel)
    {
        fuel = Math.Clamp(fuel, 0, AircraftParameters.InitialFuel);
        float mass = AircraftParameters.EmptyMass + fuel;
        Vector3 center = Vector3.Zero;
        foreach (var part in Components) center += part.Center * part.Mass;
        foreach (var tank in Tanks) center += tank.Position * (tank.Fraction * fuel);
        center /= mass;
        Matrix3 inertia = new(Vector3.Zero, Vector3.Zero, Vector3.Zero);
        foreach (var part in Components)
        {
            Vector3 s = part.Size;
            inertia = inertia.AddPointMass(part.Mass, part.Center - center);
            inertia = new(inertia.X + Vector3.UnitX * (part.Mass * (s.Y * s.Y + s.Z * s.Z) / 12),
                inertia.Y + Vector3.UnitY * (part.Mass * (s.X * s.X + s.Z * s.Z) / 12),
                inertia.Z + Vector3.UnitZ * (part.Mass * (s.X * s.X + s.Y * s.Y) / 12));
        }
        foreach (var tank in Tanks) inertia = inertia.AddPointMass(tank.Fraction * fuel, tank.Position - center);
        return new(mass, center, inertia);
    }
}
