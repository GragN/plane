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
    private static readonly Vector3 EmptyCenter = new(0, -.12f, -.12f);
    private static readonly (Vector3 Position, float Fraction)[] Tanks =
    [ (new(0, -.25f, 4.1f), .30f), (new(0, -.4f, 1.1f), .40f), (new(0, -.15f, -5), .30f) ];

    public static MassProperties FromFuel(float fuel)
    {
        fuel = Math.Clamp(fuel, 0, AircraftParameters.InitialFuel);
        float mass = AircraftParameters.EmptyMass + fuel;
        Vector3 center = EmptyCenter * AircraftParameters.EmptyMass;
        foreach (var tank in Tanks) center += tank.Position * (tank.Fraction * fuel);
        center /= mass;
        // Estimated empty-airframe tensor, including a yaw/roll product of inertia.
        Matrix3 inertia = new(new(112000, 0, 0), new(0, 135000, 2500), new(0, 2500, 25000));
        inertia = inertia.AddPointMass(AircraftParameters.EmptyMass, EmptyCenter - center);
        foreach (var tank in Tanks) inertia = inertia.AddPointMass(tank.Fraction * fuel, tank.Position - center);
        return new(mass, center, inertia);
    }
}
