using System.Numerics;
using Flanker.Simulation;

namespace Flanker.Flight;

public sealed class FlightModel
{
    public const float FixedStep = 1f / 240;
    private readonly Aerodynamics airframe = new();
    private readonly LandingGear gear = new();
    private readonly JetEngine leftEngine = new(), rightEngine = new();
    public FlightControls Controls { get; } = new();
    private float cachedFuel = float.NaN;
    private MassProperties cachedMass;
    public Vector3 Position { get; set; }
    public Vector3 Velocity { get; set; }
    public Quaternion Orientation { get; set; } = Quaternion.Identity;
    public Vector3 AngularVelocity { get; set; }
    public float Fuel { get; set; }
    public MassProperties MassDistribution
    {
        get
        {
            if (cachedFuel != Fuel) { cachedMass = MassProperties.FromFuel(Fuel); cachedFuel = Fuel; }
            return cachedMass;
        }
    }
    public float Mass => MassDistribution.Mass;
    public float Throttle { get; set; }
    public float EngineSpool => (leftEngine.Rpm + rightEngine.Rpm) / 2;
    public float LeftThrust => leftEngine.Thrust;
    public float RightThrust => rightEngine.Thrust;
    public bool LeftEngineEnabled { get => leftEngine.Enabled; set => leftEngine.Enabled = value; }
    public bool RightEngineEnabled { get => rightEngine.Enabled; set => rightEngine.Enabled = value; }
    public bool Afterburner { get; set; }
    public bool GearDown { get; set; }
    public float GearExtension { get; private set; }
    public bool Airbrake { get; set; }
    public bool WheelBrake { get; set; }
    public bool ParkingBrake { get; set; }
    public bool WindEnabled { get; set; }
    public float PitchTrim { get; set; }
    public float ElevatorInput { get; set; }
    public float RollInput { get; set; }
    public float RudderInput { get; set; }
    public float Elevator { get; private set; }
    public float DifferentialTail { get; private set; }
    public float Aileron { get; private set; }
    public float Rudder { get; private set; }
    public float Thrust => LeftThrust + RightThrust;
    public float LoadFactor { get; private set; } = 1;
    public float StructuralDamage { get; private set; }
    public float AltitudeAgl => Position.Y - Terrain.Height(Position.X, Position.Z);
    public double Time { get; private set; }
    public bool Crashed { get; private set; }
    public string Failure { get; private set; } = "";
    public bool OnGround { get; private set; }
    public AeroResult Aero { get; private set; }
    public IReadOnlyList<SurfaceLoad> SurfaceLoads => airframe.Loads;
    public IReadOnlyList<float> WheelSpeeds => gear.WheelSpeeds;
    public IReadOnlyList<float> GearCompression => gear.Compression;
    public Vector3 Forward => ToWorld(Vector3.UnitZ);
    public Vector3 Up => ToWorld(Vector3.UnitY);
    public Vector3 Right => ToWorld(Vector3.UnitX);
    public float Heading => (MathF.Atan2(Forward.X, Forward.Z) * 180 / MathF.PI + 360) % 360;
    public float Pitch => MathF.Asin(Math.Clamp(Forward.Y, -1, 1));
    public float Bank => MathF.Atan2(-Right.Y, Up.Y);

    public FlightModel(bool ground = false, ControlLaw controlLaw = ControlLaw.Normal)
    {
        Controls.Law = controlLaw;
        Reset(ground);
    }
    public Vector3 ToWorld(Vector3 v) => Vector3.Transform(v, Orientation);
    public Vector3 ToLocal(Vector3 v) => Vector3.Transform(v, Quaternion.Conjugate(Orientation));
    public Vector3 BodyPointToWorld(Vector3 point) => Position + ToWorld(point - MassDistribution.Center);

    public void Reset(bool ground) => Initialize(ground, 175, 1100, AircraftParameters.InitialFuel);

    public void ResetAirborne(float speed, float altitude, float fuel = AircraftParameters.InitialFuel)
    {
        if (speed <= 0 || altitude < 50 || fuel < 0 || fuel > AircraftParameters.InitialFuel)
            throw new ArgumentOutOfRangeException(nameof(speed), "Invalid airborne trim conditions");
        Initialize(false, speed, altitude, fuel);
    }

    private void Initialize(bool ground, float speed, float altitude, float fuel)
    {
        Fuel = fuel;
        AngularVelocity = Vector3.Zero;
        Time = 0;
        Crashed = false; Failure = ""; StructuralDamage = 0;
        OnGround = ground;
        ElevatorInput = RollInput = RudderInput = Elevator = Aileron = Rudder = DifferentialTail = 0;
        Airbrake = WheelBrake = Afterburner = false;
        ParkingBrake = ground;
        GearDown = ground;
        GearExtension = ground ? 1 : 0;
        Position = ground ? new(0, 2.75f, -1050) : new(0, altitude, -1200);
        Velocity = ground ? Vector3.Zero : new(0, 0, speed);
        Orientation = Quaternion.Identity;
        PitchTrim = Throttle = 0;
        if (!ground)
        {
            TrimSolution trim = TrimSolver.Solve(speed, Position.Y, MassDistribution);
            Orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, -trim.Alpha);
            Elevator = PitchTrim = trim.Elevator;
            Throttle = trim.Throttle;
        }
        Air initialAir = Atmosphere.At(Position.Y);
        leftEngine.Reset(Throttle, initialAir, Velocity.Length() / initialAir.SoundSpeed);
        rightEngine.Reset(Throttle, initialAir, Velocity.Length() / initialAir.SoundSpeed);
        gear.Reset();
        if (ground)
        {
            var pose = LandingGear.ParkedPose(MassDistribution, Thrust);
            Position = new(0, pose.Height, -1050);
            Orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitX, -pose.Pitch);
        }
        Controls.Reset(ground, PitchTrim);
        var context = Context(Position, Velocity, Orientation, AngularVelocity, MassDistribution);
        airframe.ResetFlow(context);
        Aero = airframe.Evaluate(context);
        LoadFactor = 1;
    }

    private AeroContext Context(Vector3 position, Vector3 velocity, Quaternion orientation, Vector3 omega, MassProperties mass)
    {
        Air air = Atmosphere.At(position.Y);
        Vector3 relative = velocity - Atmosphere.WindAt(Time, position, WindEnabled);
        return new(Vector3.Transform(relative, Quaternion.Conjugate(orientation)), omega, air,
            Elevator, Aileron, Rudder, GearExtension, Airbrake ? 1 : 0,
            position.Y - Terrain.Height(position.X, position.Z), mass.Center, orientation, position, WindEnabled, Time,
            DifferentialTail);
    }

    private readonly record struct Dynamics(Vector3 Force, Vector3 Moment, AeroResult Aero, ContactResult Contact);
    private Dynamics Evaluate(Vector3 position, Vector3 velocity, Quaternion orientation, Vector3 omega,
        MassProperties mass, float dt)
    {
        AeroContext context = Context(position, velocity, orientation, omega, mass);
        AeroResult aero = airframe.Evaluate(context);
        Vector3 bodyForce = aero.Force, moment = aero.Moment;
        for (int i = 0; i < 2; i++)
        {
            JetEngine engine = i == 0 ? leftEngine : rightEngine;
            float drag = engine.Enabled && Fuel > 0 ? 0 : .5f * context.Air.Density * context.Velocity.Z * MathF.Abs(context.Velocity.Z) * .09f;
            Vector3 f = new(0, 0, engine.Thrust - drag);
            bodyForce += f;
            moment += Vector3.Cross(new Vector3(i == 0 ? -1.23f : 1.23f, -.43f, -5) - mass.Center, f);
        }
        ContactResult contact = gear.Evaluate(position, velocity, orientation, omega, mass,
            GearExtension, Rudder, WheelBrake || ParkingBrake, dt);
        Vector3 force = Vector3.Transform(bodyForce, orientation) + contact.Force
            - Vector3.UnitY * (mass.Mass * Atmosphere.Gravity);
        return new(force, moment + contact.Moment, aero, contact);
    }

    public void Step(float dt = FixedStep)
    {
        if (Crashed) return;
        if (!(dt > 0 && dt <= .02f)) throw new ArgumentOutOfRangeException(nameof(dt), "Physics step must be in (0, .02] seconds");
        Time += dt;
        Throttle = Math.Clamp(Throttle, 0, 1);
        PitchTrim = Math.Clamp(PitchTrim, -.25f, .25f);
        Air air = Atmosphere.At(Position.Y);
        Vector3 measuredMoment = Aero.Moment;
        measuredMoment += Vector3.Cross(new Vector3(-1.23f, -.43f, -5) - MassDistribution.Center, new Vector3(0, 0, LeftThrust));
        measuredMoment += Vector3.Cross(new Vector3(1.23f, -.43f, -5) - MassDistribution.Center, new Vector3(0, 0, RightThrust));
        SurfaceCommands command = Controls.Step(dt, this, air, MassDistribution, measuredMoment, Elevator, Aileron, Rudder, DifferentialTail);
        float leftTail = Move(Elevator - DifferentialTail, Math.Clamp(command.Elevator - command.DifferentialTail, -1, 1), dt * 1.6f);
        float rightTail = Move(Elevator + DifferentialTail, Math.Clamp(command.Elevator + command.DifferentialTail, -1, 1), dt * 1.6f);
        Elevator = (leftTail + rightTail) / 2;
        DifferentialTail = (rightTail - leftTail) / 2;
        Aileron = Move(Aileron, command.Aileron, dt * 2.5f);
        Rudder = Move(Rudder, command.Rudder, dt * 1.8f);
        GearExtension = Move(GearExtension, GearDown ? 1 : 0, dt / 3.5f);
        MassProperties mass = MassDistribution;
        AeroContext c = Context(Position, Velocity, Orientation, AngularVelocity, mass);
        float speed = c.Velocity.Length();
        float alpha = speed > .01f ? MathF.Atan2(-c.Velocity.Y, c.Velocity.Z) : 0;
        float beta = speed > .01f ? MathF.Asin(Math.Clamp(c.Velocity.X / speed, -1, 1)) : 0;
        leftEngine.Step(dt, Throttle, Afterburner, air, speed / air.SoundSpeed, alpha, beta, Fuel > 0);
        rightEngine.Step(dt, Throttle, Afterburner, air, speed / air.SoundSpeed, alpha, beta, Fuel > 0);
        Fuel = Math.Max(0, Fuel - (leftEngine.FuelFlow + rightEngine.FuelFlow) * dt);
        mass = MassDistribution;
        airframe.AdvanceFlow(c, dt);

        // Explicit midpoint for translation and Euler rigid-body dynamics. Forces are
        // reevaluated at the predicted midpoint; wheel slip is solved implicitly.
        Dynamics first = Evaluate(Position, Velocity, Orientation, AngularVelocity, mass, dt / 2);
        Vector3 acceleration = first.Force / mass.Mass;
        Vector3 angularAcceleration = AngularAcceleration(first.Moment, AngularVelocity, mass.Inertia);
        Vector3 midVelocity = Velocity + acceleration * (dt / 2);
        Vector3 midOmega = AngularVelocity + angularAcceleration * (dt / 2);
        Vector3 midPosition = Position + Velocity * (dt / 2) + acceleration * (dt * dt / 8);
        Quaternion midOrientation = Rotate(Orientation, AngularVelocity + angularAcceleration * (dt / 4), dt / 2);
        Dynamics middle = Evaluate(midPosition, midVelocity, midOrientation, midOmega, mass, dt);
        Velocity += middle.Force / mass.Mass * dt;
        Position += midVelocity * dt;
        AngularVelocity += AngularAcceleration(middle.Moment, midOmega, mass.Inertia) * dt;
        Orientation = Rotate(Orientation, midOmega, dt);
        gear.Commit();
        Aero = middle.Aero;
        OnGround = middle.Contact.OnGround;
        LoadFactor = Vector3.Transform(middle.Force + Vector3.UnitY * mass.Mass * Atmosphere.Gravity,
            Quaternion.Conjugate(midOrientation)).Y / (mass.Mass * Atmosphere.Gravity);
        float excess = Math.Max(0, LoadFactor - 9) + Math.Max(0, -3 - LoadFactor);
        StructuralDamage = Math.Clamp(StructuralDamage + .12f * excess * excess * dt
            + Math.Max(0, Aero.Mach - 2.35f) * dt, 0, 1);
        if (middle.Contact.ImpactSpeed > 8 || middle.Contact.Compression > .65f) Fail("LANDING GEAR OVERLOAD");
        if (StructuralDamage >= 1) Fail("STRUCTURAL LIMIT EXCEEDED");
        foreach (Vector3 point in CollisionPoints)
        {
            Vector3 world = BodyPointToWorld(point);
            if (world.Y < Terrain.Height(world.X, world.Z) - .02f) { Fail("AIRFRAME / GROUND COLLISION"); break; }
        }
        if (!float.IsFinite(Position.LengthSquared() + Velocity.LengthSquared() + AngularVelocity.LengthSquared()))
            Fail("NON-FINITE PHYSICS STATE");
    }

    private static readonly Vector3[] CollisionPoints = [new(0, -.5f, 10), new(0, -.6f, -9.6f),
        new(-7.1f, -.1f, -4.5f), new(7.1f, -.1f, -4.5f), new(-1.23f, -1, -7), new(1.23f, -1, -7)];
    private static Vector3 AngularAcceleration(Vector3 moment, Vector3 omega, Matrix3 inertia) =>
        inertia.Solve(moment - Vector3.Cross(omega, inertia.Multiply(omega)));
    private static Quaternion Rotate(Quaternion orientation, Vector3 omega, float dt)
    {
        float speed = omega.Length();
        return speed < 1e-8f ? orientation : Quaternion.Normalize(orientation *
            Quaternion.CreateFromAxisAngle(omega / speed, speed * dt));
    }
    private void Fail(string reason) { if (!Crashed) Failure = reason; Crashed = true; }
    private static float Move(float from, float to, float amount) => from + Math.Clamp(to - from, -amount, amount);
}
