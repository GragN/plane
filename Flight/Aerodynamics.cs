using System.Numerics;

namespace Flanker.Flight;

public readonly record struct AeroResult(Vector3 Force, Vector3 Moment, float Alpha,
    float Beta, float Speed, float Mach, float Lift, float Drag, float Separation,
    float LeftSeparation = 0, float RightSeparation = 0);
public readonly record struct SurfaceLoad(Vector3 Position, Vector3 Force, float Alpha, float Separation);
public readonly record struct AeroContext(Vector3 Velocity, Vector3 Omega, Air Air,
    float Elevator = 0, float Roll = 0, float Rudder = 0, float Gear = 0, float Airbrake = 0,
    float GroundClearance = 1000, Vector3 CenterOfMass = default,
    Quaternion Orientation = default, Vector3 Position = default, bool WindEnabled = false, double Time = 0,
    float DifferentialTail = 0);

public sealed class Aerodynamics
{
    private enum Kind { Wing, Tail, Fin }
    private readonly record struct Panel(Vector3 Position, Vector3 Span, float Area, float Chord,
        Kind Kind, int Side, float StallAngle, float Control);
    private struct FlowState
    {
        public float Separation, Cl, Alpha, AlphaRate;
    }
    private readonly Panel[] panels;
    private readonly FlowState[] states;
    private readonly SurfaceLoad[] loads;
    public IReadOnlyList<SurfaceLoad> Loads => loads;
    private float wingCl;
    private const float WingAspect = AircraftParameters.Span * AircraftParameters.Span / AircraftParameters.WingArea;
    private static readonly (float Z, float SideWeight, float NormalWeight)[] BodySections =
    [(-7.5f, 4.9f, 9.8f), (-3.8f, 6.4f, 16), (0, 6.8f, 12.8f), (4, 5.2f, 6.4f), (8.5f, 3, 3.2f)];

    public Aerodynamics()
    {
        var parts = new List<Panel>();
        foreach (int side in new[] { -1, 1 })
        {
            const int strips = 7;
            for (int i = 0; i < strips; i++)
            {
                float u = (i + .5f) / strips;
                float x = .65f + (AircraftParameters.Span / 2 - .65f) * u;
                float weight = (1 - .6f * u) / (strips * .7f);
                Vector3 span = Vector3.Normalize(new Vector3(1, -side * .035f, 0));
                // Swept outer-wing quarter chords stay inside the visual planform.
                parts.Add(new(new(side * x, 0, .6f - .63f * x), span,
                    (AircraftParameters.WingArea - 14) * .5f * weight, 6.2f - 4 * u,
                    Kind.Wing, side, (AircraftParameters.StallAlpha + .036f - .075f * u), u > .55f ? -.22f * side : 0));
            }
            // Account for the forward root extensions at their own locations rather
            // than shifting every outer-wing force ahead of its leading edge.
            parts.Add(new(new(side * .95f, .2f, 3.2f), Vector3.UnitX,
                7, 4.5f, Kind.Wing, side, .49f, 0));
            for (int i = 0; i < 3; i++)
                parts.Add(new(new(side * (1.9f + .8f * i), .08f, -6.8f - i * .35f), Vector3.UnitX,
                    12.4f / 6, 2.6f - .3f * i, Kind.Tail, side, .31f, -.36f));
            for (int i = 0; i < 2; i++)
                parts.Add(new(new(side * 2.15f, 1.1f + .9f * i, -6.5f - .45f * i), Vector3.UnitY,
                    8.6f / 4, 2.8f - .6f * i, Kind.Fin, side, .34f, .38f));
        }
        // Wings first, so the tail receives their downwash in the same evaluation.
        panels = parts.OrderBy(p => p.Kind).ToArray();
        states = new FlowState[panels.Length];
        loads = new SurfaceLoad[panels.Length];
    }

    // A stateless equilibrium evaluator for calibration/tests. Runtime owns an instance
    // so circulation, separation and stall hysteresis persist independently on every strip.
    public static AeroResult Evaluate(Vector3 velocity, Vector3 omega, Air air,
        float elevator, float roll, float rudder, float gear, float airbrake)
    {
        var model = new Aerodynamics();
        return model.EvaluateSteady(new(velocity, omega, air, elevator, roll, rudder, gear, airbrake));
    }

    public AeroResult EvaluateSteady(in AeroContext c)
    {
        ResetFlow(c);
        return Evaluate(c);
    }

    public void ResetFlow(in AeroContext c) => UpdateFlow(c, 0, true);
    public void AdvanceFlow(in AeroContext c, float dt) => UpdateFlow(c, dt, false);

    private void UpdateFlow(in AeroContext c, float dt, bool reset)
    {
        float weightedCl = 0;
        for (int i = 0; i < panels.Length; i++)
        {
            Panel p = panels[i];
            var (velocity, alpha, _) = Flow(c, p, wingCl);
            float speed = velocity.Length();
            float reynolds = c.Air.Density * speed * p.Chord / c.Air.DynamicViscosity;
            float reCorrection = Math.Clamp(MathF.Log10(Math.Max(1, reynolds) / 1e7f) * .012f, -.04f, .015f);
            float critical = p.StallAngle + reCorrection;
            float target = AerodynamicTables.Smooth(critical, critical + .22f, MathF.Abs(alpha));
            ref FlowState s = ref states[i];
            float tau = Math.Clamp(4 * p.Chord / Math.Max(15, speed), .025f, .6f);
            if (target < s.Separation) tau *= 2.5f; // slower reattachment gives hysteresis
            float previousAlpha = s.Alpha;
            s.Alpha = alpha;
            s.AlphaRate = reset ? 0 : Math.Clamp(MathF.IEEERemainder(alpha - previousAlpha, MathF.Tau) / dt, -12, 12);
            s.Separation = reset ? target : s.Separation + (target - s.Separation) * (1 - MathF.Exp(-dt / tau));
            float slope = p.Kind == Kind.Wing ? AircraftParameters.LiftSlope : p.Kind == Kind.Tail ? 3.35f : 2.7f;
            float attached = Math.Clamp((p.Kind == Kind.Wing ? AircraftParameters.ZeroAlphaLift : 0) + slope * alpha, -1.9f, 1.9f)
                * AerodynamicTables.LiftFactor(speed / c.Air.SoundSpeed);
            if (p.Kind == Kind.Wing) attached += .9f * MathF.Sin(alpha) * MathF.Abs(MathF.Sin(alpha)) * MathF.Cos(alpha);
            float cl = attached * (1 - s.Separation) + 1.05f * MathF.Sin(2 * alpha) * s.Separation;
            float circulationTau = Math.Clamp(p.Chord / (2 * Math.Max(15, speed)), .006f, .15f);
            s.Cl = reset ? cl : s.Cl + (cl - s.Cl) * (1 - MathF.Exp(-dt / circulationTau));
            if (p.Kind == Kind.Wing)
            {
                weightedCl += s.Cl * p.Area / AircraftParameters.WingArea;
                wingCl = weightedCl; // complete before any tail panel is processed
            }
        }
    }

    private static (Vector3 Velocity, float Alpha, float Beta) Flow(in AeroContext c, Panel p, float wingLift)
    {
        Vector3 arm = p.Position - c.CenterOfMass;
        Vector3 v = c.Velocity + Vector3.Cross(c.Omega, arm);
        if (c.WindEnabled)
        {
            Vector3 deltaWind = Atmosphere.WindAt(c.Time, c.Position, true) -
                Atmosphere.WindAt(c.Time, c.Position + Vector3.Transform(arm, c.Orientation), true);
            v += Vector3.Transform(deltaWind, Quaternion.Conjugate(c.Orientation));
        }
        if (p.Kind == Kind.Tail)
        {
            float ground = GroundFactor(c.GroundClearance);
            float downwash = 1.1f * wingLift / (MathF.PI * WingAspect) * ground;
            v.Y += MathF.Abs(v.Z) * downwash;
        }
        float speed = v.Length();
        float beta = speed > .01f ? MathF.Asin(Math.Clamp(v.X / speed, -1, 1)) : 0;
        float alpha = p.Kind == Kind.Fin ? MathF.Atan2(v.X, v.Z) + p.Control * c.Rudder :
            MathF.Atan2(-Vector3.Dot(v, Vector3.Cross(Vector3.UnitZ, p.Span)), v.Z)
                + p.Control * (p.Kind == Kind.Tail
                    ? Math.Clamp(c.Elevator + p.Side * c.DifferentialTail, -1, 1) : c.Roll);
        return (v, MathF.IEEERemainder(alpha, MathF.Tau), beta);
    }

    public static float GroundFactor(float height)
    {
        float ratio = 16 * Math.Max(.02f, height) / AircraftParameters.Span;
        return ratio * ratio / (1 + ratio * ratio);
    }

    public AeroResult Evaluate(in AeroContext c)
    {
        Vector3 total = Vector3.Zero, moment = Vector3.Zero;
        float leftSep = 0, rightSep = 0;
        for (int i = 0; i < panels.Length; i++)
        {
            Panel p = panels[i]; FlowState s = states[i];
            var (v, alpha, beta) = Flow(c, p, wingCl);
            float speed = v.Length();
            if (speed < .05f) { loads[i] = new(p.Position, Vector3.Zero, alpha, s.Separation); continue; }
            float mach = speed / c.Air.SoundSpeed;
            float qS = .5f * c.Air.Density * speed * speed * p.Area;
            if (p.Kind == Kind.Wing)
            {
                // Swept panels see different normal velocities during sideslip.
                float sweep = Math.Clamp(1 + p.Side * .65f * MathF.Sin(beta), .3f, 1.7f);
                qS *= sweep * sweep;
                if (p.Side < 0) leftSep += s.Separation * p.Area / (AircraftParameters.WingArea / 2);
                else rightSep += s.Separation * p.Area / (AircraftParameters.WingArea / 2);
            }
            Vector3 direction = v / speed;
            Vector3 liftAxis = Vector3.Cross(direction, p.Span);
            if (liftAxis.LengthSquared() > 1e-8f) liftAxis = Vector3.Normalize(liftAxis);
            float addedMassCl = Math.Clamp(.35f * MathF.PI * p.Chord / (2 * speed) * s.AlphaRate, -.25f, .25f);
            float cl = s.Cl + addedMassCl * (1 - s.Separation);
            float re = c.Air.Density * speed * p.Chord / c.Air.DynamicViscosity;
            float height = c.GroundClearance + Vector3.Transform(p.Position - c.CenterOfMass, c.Orientation).Y;
            float inducedFactor = p.Kind == Kind.Fin ? 1 : GroundFactor(height);
            float aspect = p.Kind == Kind.Wing ? WingAspect : p.Kind == Kind.Tail ? 4.8f : 1.7f;
            float cd = .005f + 2 * AerodynamicTables.SkinFriction(re, mach)
                + cl * cl * inducedFactor / (MathF.PI * aspect * .8f)
                + 1.2f * s.Separation * MathF.Pow(MathF.Sin(alpha), 2)
                + .3f * Math.Max(0, -MathF.Cos(alpha))
                + (p.Kind == Kind.Wing ? .4f * AerodynamicTables.WaveDrag(mach) : 0);
            Vector3 force = qS * (liftAxis * cl - direction * cd);
            Vector3 arm = p.Position - c.CenterOfMass;
            if (p.Kind == Kind.Wing)
            {
                // Aft pressure-center travel through the transonic region (Mach tuck).
                arm.Z -= .55f * AerodynamicTables.Smooth(.78f, 1.15f, mach);
                moment += p.Span * (qS * p.Chord * .008f * (1 - s.Separation));
            }
            total += force;
            moment += Vector3.Cross(arm, force);
            loads[i] = new(arm + c.CenterOfMass, force, alpha, s.Separation);
        }
        float airspeed = c.Velocity.Length();
        Vector3 flowDirection = airspeed > .01f ? c.Velocity / airspeed : Vector3.UnitZ;
        float bodyMach = airspeed / c.Air.SoundSpeed;
        float qArea = .5f * c.Air.Density * airspeed * airspeed * AircraftParameters.WingArea;
        Vector3 bodyForce = -flowDirection * qArea *
            (.008f + .6f * AerodynamicTables.WaveDrag(bodyMach) + .055f * c.Gear + .095f * c.Airbrake);
        total += bodyForce;
        moment += Vector3.Cross(new Vector3(0, 0, -1.5f) - c.CenterOfMass, bodyForce);
        // Crossflow pressure is distributed along the hull. A single force at the nose
        // caused excessive pitch-up after stall and missed rotational damping.
        foreach (var section in BodySections)
        {
            Vector3 arm = new Vector3(0, -.05f, section.Z) - c.CenterOfMass;
            Vector3 v = c.Velocity + Vector3.Cross(c.Omega, arm);
            if (c.WindEnabled)
                v += Vector3.Transform(Atmosphere.WindAt(c.Time, c.Position, true)
                    - Atmosphere.WindAt(c.Time, c.Position + Vector3.Transform(arm, c.Orientation), true),
                    Quaternion.Conjugate(c.Orientation));
            Vector3 force = -.5f * c.Air.Density * new Vector3(
                32 * section.SideWeight / 26.3f * v.X * MathF.Abs(v.X),
                24 * section.NormalWeight / 48.2f * v.Y * MathF.Abs(v.Y), 0);
            total += force;
            moment += Vector3.Cross(arm, force);
        }
        Vector3 liftDirection = Vector3.Cross(flowDirection, Vector3.UnitX);
        if (liftDirection.LengthSquared() > 1e-8f) liftDirection = Vector3.Normalize(liftDirection);
        // Direction-of-flow instruments are not meaningful while standing still.
        float alphaBody = airspeed > 5 ? MathF.Atan2(-c.Velocity.Y, c.Velocity.Z) : 0;
        float betaBody = airspeed > 5 ? MathF.Asin(Math.Clamp(c.Velocity.X / airspeed, -1, 1)) : 0;
        return new(total, moment, alphaBody, betaBody, airspeed, bodyMach,
            Vector3.Dot(total, liftDirection), -Vector3.Dot(total, flowDirection), Math.Max(leftSep, rightSep), leftSep, rightSep);
    }
}
