using System.Numerics;
using Flanker.Simulation;

namespace Flanker.Flight;

public sealed class FlightModel
{
    public const float FixedStep = 1f / 120;
    public Vector3 Position { get; set; }
    public Vector3 Velocity { get; set; }
    public Quaternion Orientation { get; set; } = Quaternion.Identity;
    public Vector3 AngularVelocity { get; set; }
    public float Fuel { get; set; }
    public float Mass => AircraftParameters.EmptyMass + Fuel;
    public float Throttle { get; set; }
    public float EngineSpool { get; private set; }
    public bool Afterburner { get; set; }
    public bool GearDown { get; set; }
    public float GearExtension { get; private set; }
    public bool Airbrake { get; set; }
    public bool WheelBrake { get; set; }
    public bool WindEnabled { get; set; }
    public float PitchTrim { get; set; }
    public float ElevatorInput { get; set; }
    public float RollInput { get; set; }
    public float RudderInput { get; set; }
    public float Elevator { get; private set; }
    public float Aileron { get; private set; }
    public float Rudder { get; private set; }
    public float Thrust { get; private set; }
    public float LoadFactor { get; private set; } = 1;
    public float AltitudeAgl => Position.Y - Terrain.Height(Position.X, Position.Z);
    public double Time { get; private set; }
    public bool Crashed { get; private set; }
    public string Failure { get; private set; } = "";
    public bool OnGround { get; private set; }
    public AeroResult Aero { get; private set; }
    public Vector3 Forward => Vector3.Transform(Vector3.UnitZ, Orientation);
    public Vector3 Up => Vector3.Transform(Vector3.UnitY, Orientation);
    public Vector3 Right => Vector3.Transform(Vector3.UnitX, Orientation);
    public float Heading => (MathF.Atan2(Forward.X, Forward.Z) * 180 / MathF.PI + 360) % 360;
    public float Pitch => MathF.Asin(Math.Clamp(Forward.Y, -1, 1));
    public float Bank => MathF.Atan2(-Right.Y, Up.Y);

    public FlightModel(bool ground = false) => Reset(ground);

    public void Reset(bool ground)
    {
        Fuel = AircraftParameters.InitialFuel;
        AngularVelocity = Vector3.Zero;
        Time = 0;
        Crashed = false;
        Failure = "";
        OnGround = ground;
        ElevatorInput = RollInput = RudderInput = Elevator = Aileron = Rudder = 0;
        Airbrake = WheelBrake = Afterburner = false;
        GearDown = ground;
        GearExtension = ground ? 1 : 0;
        Position = ground ? new(0, 2.15f, -1050) : new(0, 1100, -1200);
        Velocity = ground ? Vector3.Zero : new(0, 0, 175);
        Air air = Atmosphere.At(Position.Y);
        float alpha = ground ? 0 : (Mass * Atmosphere.Gravity /
            (.5f * air.Density * Velocity.LengthSquared() * AircraftParameters.WingArea)
            - AircraftParameters.ZeroAlphaLift) / AircraftParameters.LiftSlope;
        Orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, -alpha);
        PitchTrim = ground ? 0 : (.5f * MathF.Sin(alpha) - .025f) / .9f;
        Aero = ground ? default : Aerodynamics.Evaluate(ToLocal(Velocity), Vector3.Zero,
            air, PitchTrim, 0, 0, 0, 0);
        Throttle = ground ? 0 : Aero.Drag / AvailableDryThrust(air, Aero.Mach);
        EngineSpool = Throttle;
        Thrust = ground ? 0 : Aero.Drag;
        LoadFactor = 1;
        overstressTime = 0;
    }

    private float overstressTime;
    public Vector3 ToWorld(Vector3 v) => Vector3.Transform(v, Orientation);
    public Vector3 ToLocal(Vector3 v) => Vector3.Transform(v, Quaternion.Conjugate(Orientation));

    private static float AvailableDryThrust(Air air, float mach) =>
        AircraftParameters.DryThrust * MathF.Pow(air.Density / 1.225f, .70f)
        * (1 + .12f * mach * mach) * Math.Max(.4f, 1 - .08f * mach);

    public void Step(float dt = FixedStep)
    {
        if (Crashed) return;
        Time += dt;
        Throttle = Math.Clamp(Throttle, 0, 1);
        PitchTrim = Math.Clamp(PitchTrim, -.25f, .25f);
        Elevator = Move(Elevator, Math.Clamp(ElevatorInput, -1, 1), dt * 2.4f);
        Aileron = Move(Aileron, Math.Clamp(RollInput, -1, 1), dt * 3.6f);
        Rudder = Move(Rudder, Math.Clamp(RudderInput, -1, 1), dt * 2.8f);
        EngineSpool = Move(EngineSpool, Throttle, dt / (Throttle > EngineSpool ? 3 : 1.8f));
        GearExtension = Move(GearExtension, GearDown ? 1 : 0, dt / 3.5f);
        Air air = Atmosphere.At(Position.Y);
        Vector3 relative = Velocity - Atmosphere.Wind(Time, Position.Y, WindEnabled);
        Aero = Aerodynamics.Evaluate(ToLocal(relative), AngularVelocity, air,
            Elevator + PitchTrim, Aileron, Rudder, GearExtension, Airbrake ? 1 : 0);
        float boost = Afterburner && EngineSpool > .95f ?
            (AircraftParameters.AfterburnerThrust / AircraftParameters.DryThrust - 1)
            * Math.Clamp((EngineSpool - .95f) / .05f, 0, 1) : 0;
        Thrust = Fuel > 0 ? AvailableDryThrust(air, Aero.Mach) * (EngineSpool + boost) : 0;
        Fuel = Math.Max(0, Fuel - Thrust * (boost > 0 ? .000045f : .000022f) * dt);
        Vector3 bodyForce = Aero.Force + new Vector3(0, 0, Thrust);
        Vector3 force = ToWorld(bodyForce) - Vector3.UnitY * (Mass * Atmosphere.Gravity);
        Vector3 moment = Aero.Moment;
        OnGround = false;

        if (GearExtension > .95f)
        {
            Contact(new(0, -2.15f, 5.2f), true, dt, ref force, ref moment);
            Contact(new(-2.4f, -2.15f, -1.7f), false, dt, ref force, ref moment);
            Contact(new(2.4f, -2.15f, -1.7f), false, dt, ref force, ref moment);
        }
        if (AltitudeAgl < .65f) Fail("AIRFRAME / GROUND COLLISION");
        overstressTime = MathF.Abs(LoadFactor) > 11 || Aero.Mach > 2.35f ? overstressTime + dt : 0;
        if (overstressTime > .8f) Fail("STRUCTURAL LIMIT EXCEEDED");
        if (Crashed) return;

        LoadFactor = bodyForce.Y / (Mass * Atmosphere.Gravity);
        Velocity += force / Mass * dt;
        Position += Velocity * dt;
        Vector3 inertia = AircraftParameters.Inertia * (Mass / 21300f);
        Vector3 gyro = Vector3.Cross(AngularVelocity, AngularVelocity * inertia);
        AngularVelocity += (moment - gyro) / inertia * dt;
        float rotationSpeed = AngularVelocity.Length();
        if (rotationSpeed > 1e-7f)
            Orientation = Quaternion.Normalize(Orientation * Quaternion.CreateFromAxisAngle(
                AngularVelocity / rotationSpeed, rotationSpeed * dt));
    }

    private void Contact(Vector3 point, bool nose, float dt, ref Vector3 force, ref Vector3 moment)
    {
        Vector3 offset = ToWorld(point);
        Vector3 world = Position + offset;
        float penetration = Terrain.Height(world.X, world.Z) - world.Y;
        if (penetration <= 0) return;
        Vector3 v = Velocity + ToWorld(Vector3.Cross(AngularVelocity, point));
        if (!OnGround && (v.Y < -8 || MathF.Abs(Bank) > .45f || MathF.Abs(Pitch) > .35f))
            Fail("HARD LANDING / GEAR FAILURE");
        OnGround = true;
        Vector3 normal = Vector3.Normalize(new Vector3(
            Terrain.Height(world.X - 1, world.Z) - Terrain.Height(world.X + 1, world.Z), 2,
            Terrain.Height(world.X, world.Z - 1) - Terrain.Height(world.X, world.Z + 1)));
        float vn = Vector3.Dot(v, normal);
        float normalForce = Math.Clamp(340000 * penetration - 35000 * vn, 0, 1800000);
        Vector3 wheelForward = ToWorld(Vector3.Transform(Vector3.UnitZ,
            Quaternion.CreateFromAxisAngle(Vector3.UnitY, nose ? Rudder * .35f : 0)));
        wheelForward = Vector3.Normalize(wheelForward - normal * Vector3.Dot(wheelForward, normal));
        Vector3 side = Vector3.Cross(normal, wheelForward);
        float lateral = Vector3.Dot(v, side);
        float longitudinal = Vector3.Dot(v, wheelForward);
        float grip = Terrain.OnRunway(world.X, world.Z) ? .8f : .45f;
        Vector3 friction = -side * (lateral * Mass / (3 * Math.Max(.06f, dt)))
            - wheelForward * MathF.Tanh(longitudinal * 2) * normalForce *
            (WheelBrake ? .70f : .015f);
        if (friction.Length() > grip * normalForce)
            friction = Vector3.Normalize(friction) * grip * normalForce;
        Vector3 f = normal * normalForce + friction;
        force += f;
        moment += Vector3.Cross(point, ToLocal(f));
    }

    private void Fail(string reason) { Crashed = true; Failure = reason; }

    private static float Move(float from, float to, float amount) => from +
        Math.Clamp(to - from, -amount, amount);
}
