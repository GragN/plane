using System.Numerics;

namespace Flanker.Flight;

public readonly record struct AeroResult(Vector3 Force, Vector3 Moment, float Alpha,
    float Beta, float Speed, float Mach, float Lift, float Drag, float Separation);

public static class Aerodynamics
{
    public static float Separation(float alpha)
    {
        float t = Math.Clamp((MathF.Abs(alpha) - AircraftParameters.StallAlpha) / .24f, 0, 1);
        return t * t * (3 - 2 * t);
    }

    public static float LiftCoefficient(float alpha)
    {
        float attached = AircraftParameters.ZeroAlphaLift + AircraftParameters.LiftSlope * alpha;
        float separated = 1.1f * MathF.Sin(2 * alpha);
        float blend = Separation(alpha);
        return attached * (1 - blend) + separated * blend;
    }

    public static AeroResult Evaluate(Vector3 localVelocity, Vector3 omega, Air air,
        float elevator, float roll, float rudder, float gear, float airbrake)
    {
        float speed = localVelocity.Length();
        if (speed < .1f) return default;
        float alpha = MathF.Atan2(-localVelocity.Y, localVelocity.Z);
        float beta = MathF.Asin(Math.Clamp(localVelocity.X / speed, -1, 1));
        float mach = speed / air.SoundSpeed;
        float qS = .5f * air.Density * speed * speed * AircraftParameters.WingArea;
        float separation = Separation(alpha);
        float cl = LiftCoefficient(alpha);
        float aspect = AircraftParameters.Span * AircraftParameters.Span / AircraftParameters.WingArea;
        float transonic = Math.Max(0, (mach - .82f) / .14f);
        float waveDrag = .025f * (1 - MathF.Exp(-transonic * transonic)) / Math.Max(1, mach * mach);
        float cd = .022f + cl * cl / (MathF.PI * aspect * .78f)
            + .8f * separation * MathF.Pow(MathF.Sin(alpha), 2)
            + .25f * Math.Max(0, -MathF.Cos(alpha)) + waveDrag
            + .055f * gear + .095f * airbrake + .22f * beta * beta;
        Vector3 direction = localVelocity / speed;
        Vector3 liftAxis = Vector3.Cross(direction, Vector3.UnitX);
        if (liftAxis.LengthSquared() > .0001f) liftAxis = Vector3.Normalize(liftAxis);
        float lift = qS * cl;
        float drag = qS * cd;
        Vector3 force = liftAxis * lift - direction * drag;
        force.X += qS * (-.85f * beta - .12f * rudder);

        float effectiveness = (1 - .65f * separation) / (1 + .25f * Math.Max(0, mach - .8f));
        float pitchRate = omega.X * AircraftParameters.Chord / (2 * speed);
        float rollRate = omega.Z * AircraftParameters.Span / (2 * speed);
        float yawRate = omega.Y * AircraftParameters.Span / (2 * speed);
        float cm = .025f - .5f * MathF.Sin(alpha) + .9f * elevator * effectiveness + 12 * pitchRate;
        float cr = -.075f * roll * effectiveness - .30f * rollRate + .025f * beta;
        float cy = .075f * rudder * effectiveness + .16f * beta - .34f * yawRate;
        Vector3 moment = qS * new Vector3(-AircraftParameters.Chord * cm,
            AircraftParameters.Span * cy, AircraftParameters.Span * cr);
        return new(force, moment, alpha, beta, speed, mach, lift, drag, separation);
    }
}
