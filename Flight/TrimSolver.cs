using System.Numerics;

namespace Flanker.Flight;

public readonly record struct TrimSolution(float Alpha, float Elevator, float Throttle);

public static class TrimSolver
{
    public static TrimSolution Solve(float speed, float altitude, MassProperties mass)
    {
        var aero = new Aerodynamics();
        Air air = Atmosphere.At(altitude);
        float weight = mass.Mass * Atmosphere.Gravity;
        Vector3 Residual(Vector3 x)
        {
            Vector3 v = new(0, -MathF.Sin(x.X) * speed, MathF.Cos(x.X) * speed);
            var a = aero.EvaluateSteady(new(v, Vector3.Zero, air, Elevator: x.Y, CenterOfMass: mass.Center));
            float thrust = 2 * JetEngine.DryThrust(air, speed / air.SoundSpeed, x.Z);
            Vector3 force = a.Force + new Vector3(0, 0, thrust);
            float pitchMoment = a.Moment.X + (-.43f - mass.Center.Y) * thrust;
            return new((force.Y * MathF.Cos(x.X) + force.Z * MathF.Sin(x.X)) / weight - 1,
                pitchMoment / (weight * AircraftParameters.Chord),
                (-force.Y * MathF.Sin(x.X) + force.Z * MathF.Cos(x.X)) / weight);
        }
        Vector3 state = new(.04f, 0, .4f);
        for (int iteration = 0; iteration < 25; iteration++)
        {
            Vector3 residual = Residual(state);
            if (residual.Length() < 1e-5f) return new(state.X, state.Y, state.Z);
            const float delta = .0002f;
            Matrix3 jacobian = new((Residual(state + Vector3.UnitX * delta) - residual) / delta,
                (Residual(state + Vector3.UnitY * delta) - residual) / delta,
                (Residual(state + Vector3.UnitZ * delta) - residual) / delta);
            state -= jacobian.Solve(residual) * .8f;
            state.X = Math.Clamp(state.X, -.15f, .3f);
            state.Y = Math.Clamp(state.Y, -.6f, .6f);
            state.Z = Math.Clamp(state.Z, .01f, 1);
        }
        throw new InvalidOperationException($"No level-flight trim at V={speed}, h={altitude}, mass={mass.Mass}");
    }
}
