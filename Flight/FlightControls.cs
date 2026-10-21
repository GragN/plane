using System.Numerics;

namespace Flanker.Flight;

public enum ControlLaw { Normal, Direct }
public readonly record struct SurfaceCommands(float Elevator, float Aileron, float Rudder, float DifferentialTail = 0);

// An approximate control-augmentation system, NOT the proprietary Su-27 SDU.
// It commands physical surface actuators only. It never changes motion/attitude.
public sealed class FlightControls
{
    public ControlLaw Law { get; set; } = ControlLaw.Normal;
    public float StickPitch { get; private set; }
    public float StickRoll { get; private set; }
    public float Pedal { get; private set; }
    public float CommandedG { get; private set; } = 1;
    public float CommandedRollRate { get; private set; }
    public bool AlphaLimited { get; private set; }
    public bool GSelectedLimit { get; private set; }
    private float airborneBlend, trimBaseline;
    private float elevatorCommand, rudderCommand, rollControl;

    public void Reset(bool ground, float trim)
    {
        StickPitch = StickRoll = Pedal = 0;
        airborneBlend = ground ? 0 : 1;
        trimBaseline = trim;
        elevatorCommand = trim; rudderCommand = rollControl = 0;
        CommandedG = 1; CommandedRollRate = 0;
        AlphaLimited = GSelectedLimit = false;
    }

    public SurfaceCommands Step(float dt, FlightModel f, Air air, MassProperties mass,
        Vector3 appliedMoment, float elevator, float aileron, float rudder, float differentialTail)
    {
        StickPitch = Move(StickPitch, Math.Clamp(f.ElevatorInput, -1, 1), dt * 1.8f);
        StickRoll = Move(StickRoll, Math.Clamp(f.RollInput, -1, 1), dt * 2.2f);
        Pedal = Move(Pedal, Math.Clamp(f.RudderInput, -1, 1), dt * 2);
        float speed = f.Aero.Speed;
        float pressure = .5f * air.Density * speed * speed;
        float authority = 1 / MathF.Sqrt(1 + pressure / 120000);
        SurfaceCommands direct = new(Math.Clamp(StickPitch * authority + f.PitchTrim, -1, 1),
            StickRoll * authority, Pedal * authority, StickRoll * authority * .3f);
        AlphaLimited = GSelectedLimit = false;
        if (Law == ControlLaw.Direct)
        {
            elevatorCommand = elevator; rudderCommand = rudder;
            rollControl = aileron + .323f * differentialTail;
            CommandedG = f.LoadFactor;
            return direct;
        }
        // Unload gradually into the air law; ground operation keeps direct wheel steering.
        // Air-data loss at low speed must not switch protections to full direct input.
        // Weight-on-wheels is the ground/air discriminator; authority still falls with q.
        float targetBlend = f.OnGround ? 0 : 1;
        airborneBlend = Move(airborneBlend, targetBlend, dt / 1.2f);
        if (airborneBlend < .001f)
        {
            elevatorCommand = elevator; rudderCommand = rudder;
            rollControl = aileron + .323f * differentialTail;
            CommandedG = 1; CommandedRollRate = 0;
            return direct;
        }
        float shapedPitch = Shape(StickPitch), shapedRoll = Shape(StickRoll);
        CommandedG = Math.Clamp(1 + (shapedPitch >= 0 ? 8 : 4) * shapedPitch
            + (f.PitchTrim - trimBaseline) * 4, -3, 9);
        GSelectedLimit = MathF.Abs(StickPitch) > .98f;
        float gravityNormal = f.Up.Y;
        float safeSpeed = Math.Max(50, speed);
        float pitchRateCommand = (CommandedG - gravityNormal) * Atmosphere.Gravity / safeSpeed;
        float flightPathRate = (f.LoadFactor - gravityNormal) * Atmosphere.Gravity / safeSpeed;
        float alphaRate = -f.AngularVelocity.X - flightPathRate;
        // Anticipate inertia and actuator travel; waiting for alpha=26 deg is too late.
        float upperPrediction = f.Aero.Alpha + Math.Max(0, alphaRate) * .55f;
        float lowerPrediction = f.Aero.Alpha + Math.Min(0, alphaRate) * .55f;
        float maximumRate = flightPathRate + 2.4f * (26 * MathF.PI / 180 - upperPrediction);
        float minimumRate = flightPathRate + 2.4f * (-12 * MathF.PI / 180 - lowerPrediction);
        if (minimumRate > maximumRate)
        {
            // Predictions may overlap when NORMAL is engaged during a fast upset.
            if (alphaRate >= 0) minimumRate = maximumRate;
            else maximumRate = minimumRate;
        }
        float limitedPitch = Math.Clamp(pitchRateCommand, minimumRate, maximumRate);
        AlphaLimited = MathF.Abs(limitedPitch - pitchRateCommand) > .001f;
        float maxRoll = AerodynamicTables.Interpolate(RollSchedule, speed * MathF.Sqrt(air.Density / 1.225f));
        maxRoll *= 1 - .72f * AerodynamicTables.Smooth(.24f, .48f, MathF.Abs(f.Aero.Alpha));
        CommandedRollRate = maxRoll * shapedRoll;
        float betaTarget = -Pedal * .14f;
        // Body-vector form fades coordination in vertical flight and avoids Euler-bank
        // singularities injecting yaw/roll into a symmetric vertical maneuver.
        float turnCoordination = -Math.Clamp(f.LoadFactor, -3, 9) * Atmosphere.Gravity / safeSpeed
            * f.Right.Y * f.Up.Y;
        float yawRateCommand = turnCoordination + 1.2f * (f.Aero.Beta - betaTarget);
        Vector3 omegaTarget = new(-limitedPitch, yawRateCommand, -CommandedRollRate);
        Vector3 acceleration = new(3 * (omegaTarget.X - f.AngularVelocity.X),
            2.4f * (omegaTarget.Y - f.AngularVelocity.Y), 4 * (omegaTarget.Z - f.AngularVelocity.Z));
        acceleration.X = Math.Clamp(acceleration.X, -1.4f, 1.4f);
        acceleration.Y = Math.Clamp(acceleration.Y, -.8f, .8f);
        acceleration.Z = Math.Clamp(acceleration.Z, -2.2f, 2.2f);
        Vector3 desiredMoment = mass.Inertia.Multiply(acceleration)
            + Vector3.Cross(f.AngularVelocity, mass.Inertia.Multiply(f.AngularVelocity));
        float tailArm = Math.Max(3, 7.15f + mass.Center.Z);
        float finArm = Math.Max(3, 6.7f + mass.Center.Z);
        float pitchEffect = -pressure * 12.4f * 3.35f * .36f * tailArm;
        float rollEffect = -pressure * 125;
        float tailRollEffect = -pressure * 12.4f * 3.35f * .36f * 2.7f;
        float yawEffect = pressure * 8.6f * 2.7f * .38f * finArm;
        // Continuous-time moment tracking with bounded integrators. The old per-tick
        // increment implicitly changed gain with the integration frequency.
        const float bandwidth = 10;
        elevatorCommand = IntegrateCommand(elevatorCommand, desiredMoment.X - appliedMoment.X, pitchEffect, dt * bandwidth);
        // Ailerons can reverse their effect on a stalled outboard wing. Transfer roll
        // authority to differential stabilators before reaching that region.
        float aileronShare = 1 - AerodynamicTables.Smooth(.21f, .35f, MathF.Abs(f.Aero.Alpha));
        rollControl = IntegrateCommand(rollControl, desiredMoment.Z - appliedMoment.Z, rollEffect, dt * bandwidth);
        float normalAileron = Math.Clamp(rollControl * aileronShare, -1, 1);
        float normalDifferential = MathF.Abs(tailRollEffect) < 100 ? 0 :
            Math.Clamp(rollControl * rollEffect / tailRollEffect * (1 - aileronShare), -.45f, .45f);
        float achievedControl = normalAileron + .323f * normalDifferential;
        rollControl += (achievedControl - rollControl) * (1 - MathF.Exp(-dt * bandwidth));
        rudderCommand = IntegrateCommand(rudderCommand, desiredMoment.Y - appliedMoment.Y, yawEffect, dt * bandwidth);
        return new(Lerp(direct.Elevator, elevatorCommand, airborneBlend),
            Lerp(direct.Aileron, normalAileron, airborneBlend),
            Lerp(direct.Rudder, rudderCommand, airborneBlend),
            Lerp(direct.DifferentialTail, normalDifferential, airborneBlend));
    }

    private static readonly (float, float)[] RollSchedule =
        [(0, .7f), (80, 1.6f), (130, 2.8f), (180, 3.5f), (260, 3.2f), (400, 2.2f)];
    private static float Shape(float input) => .35f * input + .65f * input * input * input;
    private static float IntegrateCommand(float current, float difference, float effectiveness, float gainDt) =>
        MathF.Abs(effectiveness) < 100 ? current : Math.Clamp(current + gainDt * difference / effectiveness, -1, 1);
    private static float Lerp(float a, float b, float t) => a + (b - a) * t;
    private static float Move(float from, float to, float amount) => from + Math.Clamp(to - from, -amount, amount);
}
