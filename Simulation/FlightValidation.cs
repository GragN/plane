using System.Globalization;
using System.Numerics;
using System.Text;
using System.Text.Json;
using System.Text.Json.Serialization;
using Flanker.Flight;

namespace Flanker.Simulation;

public sealed record FlightMetrics(string Name, string Mode, float Duration, float PeakG, float MinimumG,
    float PeakPitchRateDegPerSecond, float PeakRollRateDegPerSecond, float PeakAlphaDegrees,
    float PeakSeparation, float BankAtOneSecondDegrees, float EndBankDegrees,
    float EndRollRateDegPerSecond, float HeightChangeMeters, float MaximumHeightExcursionMeters,
    float StartSpeedMetersPerSecond, float EndSpeedMetersPerSecond, float MinimumSpeedMetersPerSecond,
    float SpecificEnergyChangeJoulesPerKilogram, float EndSeparation, bool AlphaProtectionUsed,
    float? TakeoffDistanceTo4mMeters, bool Crashed, string Failure,
    float MaximumCrossTrackMeters, float? TouchdownSinkMetersPerSecond,
    float? LandingRollMeters, bool StoppedOnRunway);

public static class FlightValidation
{
    private const float Degrees = 180 / MathF.PI;
    private sealed record Trial(string Name, float Seconds, float Speed, ControlLaw Law,
        Action<FlightModel, float> Input, Action<FlightModel>? Prepare, Func<FlightMetrics, bool> Accept,
        float Altitude = 1100);

    public static int Run()
    {
        CultureInfo.CurrentCulture = CultureInfo.InvariantCulture;
        Directory.CreateDirectory("artifacts");
        var csv = new StringBuilder("scenario,mode,time_s,speed_m_s,altitude_m,alpha_deg,beta_deg,load_g,pitch_rate_deg_s,roll_rate_deg_s,bank_deg,elevator,aileron,commanded_g,separation,stabilator_diff,cross_track_m,vertical_speed_m_s,ground_speed_m_s,on_ground\n");
        var results = new List<FlightMetrics>();
        int failed = 0;
        bool recovering = false;
        static float StickForG(float demandedG)
        {
            float normalized = (demandedG - 1) / (demandedG >= 1 ? 8 : 4);
            float low = -1, high = 1;
            for (int i = 0; i < 16; i++)
            {
                float input = (low + high) / 2;
                if (.35f * input + .65f * input * input * input > normalized) high = input; else low = input;
            }
            return (low + high) / 2;
        }
        static void TrackRunway(FlightModel f)
        {
            float desiredBank = 0;
            if (f.OnGround)
            {
                float heading = MathF.Atan2(f.Forward.X, f.Forward.Z);
                float target = -.007f * f.Position.X - .08f * f.Velocity.X;
                f.RudderInput = Math.Clamp(3 * (target - heading) - 1.4f * f.AngularVelocity.Y, -.8f, .8f);
            }
            else
            {
                // Correct airborne ground track with the horizontal lift component,
                // rather than trying to turn a wings-level aircraft with the rudder.
                desiredBank = Math.Clamp(MathF.Atan2(-.045f * f.Position.X - .8f * f.Velocity.X,
                    Atmosphere.Gravity), -.15f, .15f);
                f.RudderInput = 0;
            }
            f.RollInput = Math.Clamp(1.5f * (desiredBank - f.Bank) + .35f * f.AngularVelocity.Z, -.6f, .6f);
        }
        static void PrepareLanding(FlightModel f)
        {
            f.Position = new(0, 60, -2300);
            Vector3 wind = Atmosphere.WindAt(0, f.Position, f.WindEnabled, f.Weather);
            float crab = MathF.Atan2(-wind.X, 90 - wind.Z);
            f.Orientation = Quaternion.CreateFromAxisAngle(Vector3.UnitY, crab)
                * Quaternion.CreateFromAxisAngle(Vector3.UnitX, -(f.Pitch - MathF.Atan2(4, 90)));
            f.Velocity = new(0, -4, 90);
            f.GearDown = true;
            f.Throttle = Math.Clamp(f.Throttle + .15f, 0, 1);
        }
        static void LandingPilot(FlightModel f, float _)
        {
            TrackRunway(f);
            if (f.OnGround)
            {
                f.Throttle = 0; f.WheelBrake = true;
                f.ElevatorInput = Math.Clamp(-.5f * f.Pitch + .5f * f.AngularVelocity.X, -.3f, .15f);
                return;
            }
            float descent = -Math.Clamp((f.AltitudeAgl - 2.2f) * .22f, .6f, 4);
            float demandedG = 1 + .55f * (descent - f.Velocity.Y) / Atmosphere.Gravity;
            f.ElevatorInput = StickForG(Math.Clamp(demandedG, .5f, 1.6f));
            f.Throttle += (90 - f.Aero.Speed) * .018f * FlightModel.FixedStep;
            if (f.AltitudeAgl < 5) f.Throttle = 0;
        }
        void RecoveryPilot(FlightModel f, float _)
        {
            recovering |= f.Aero.Separation > .2f;
            if (!recovering) { f.ElevatorInput = 1; f.RudderInput = .2f; return; }
            float pathRate = (f.LoadFactor - f.Up.Y)
                * Atmosphere.Gravity / Math.Max(30, f.Aero.Speed);
            float desiredPitchRate = pathRate + 2 * (.09f - f.Aero.Alpha);
            f.ElevatorInput = Math.Clamp(.65f * (desiredPitchRate + f.AngularVelocity.X), -.75f, .2f);
            f.RudderInput = Math.Clamp(-1.5f * f.AngularVelocity.Y, -.7f, .7f);
        }
        void TakeoffPilot(FlightModel f, float _)
        {
            if (f.Aero.Speed < 82) { f.ElevatorInput = 0; return; }
            float pitchError = .1745f - f.Pitch;
            if (f.OnGround)
                f.ElevatorInput = Math.Clamp(.15f + 1.8f * pitchError + .65f * f.AngularVelocity.X, -.35f, .6f);
            else
            {
                float demandedG = Math.Clamp(MathF.Cos(f.Pitch) + f.Aero.Speed / Atmosphere.Gravity * pitchError, -.5f, 3.5f);
                f.ElevatorInput = StickForG(demandedG);
            }
            if (f.AltitudeAgl > 8) f.GearDown = false;
        }
        Trial[] trials =
        [
            new("half_pitch", 2, 175, ControlLaw.Normal, (f, _) => f.ElevatorInput = .5f, null,
                m => !m.Crashed && m.PeakG is > 1.5f and < 5.5f && m.PeakPitchRateDegPerSecond < 22),
            new("full_pitch", 6, 175, ControlLaw.Normal, (f, _) => f.ElevatorInput = 1, null,
                m => !m.Crashed && m.PeakG is > 4 and < 10.5f && m.PeakAlphaDegrees < 29
                    && m.PeakPitchRateDegPerSecond < 45 && m.PeakRollRateDegPerSecond < 5
                    && m.SpecificEnergyChangeJoulesPerKilogram < -500),
            new("half_roll_release", 4, 175, ControlLaw.Normal, (f, t) => f.RollInput = t < 1 ? .5f : 0, null,
                m => !m.Crashed && m.BankAtOneSecondDegrees is > 15 and < 65 && m.PeakRollRateDegPerSecond < 80
                    && MathF.Abs(m.EndRollRateDegPerSecond) < 3 && m.EndBankDegrees > 15),
            new("fast_full_pitch", 6, 250, ControlLaw.Normal, (f, _) => f.ElevatorInput = 1, null,
                m => !m.Crashed && m.PeakG is > 5 and < 11 && m.PeakAlphaDegrees < 29),
            new("slow_alpha_protection", 8, 90, ControlLaw.Normal, (f, _) => f.ElevatorInput = 1, null,
                m => !m.Crashed && m.AlphaProtectionUsed && m.PeakAlphaDegrees < 29.5f
                    && m.EndSpeedMetersPerSecond < m.StartSpeedMetersPerSecond + 5),
            new("bank60_no_height_hold", 12, 175, ControlLaw.Normal, (_, _) => { },
                f => f.Orientation *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -MathF.PI / 3),
                m => !m.Crashed && m.HeightChangeMeters < -40),
            new("unpowered_glide", 20, 175, ControlLaw.Direct, (_, _) => { },
                f => { f.LeftEngineEnabled = f.RightEngineEnabled = false; f.Throttle = 0; },
                m => !m.Crashed && m.SpecificEnergyChangeJoulesPerKilogram < -1000),
            new("parked_equilibrium", 2, 0, ControlLaw.Normal, (_, _) => { }, null,
                m => !m.Crashed && m.MaximumHeightExcursionMeters < .05f && m.EndSpeedMetersPerSecond < .1f),
            new("normal_takeoff", 30, 0, ControlLaw.Normal, TakeoffPilot,
                f => { f.ParkingBrake = false; f.Throttle = 1; f.Afterburner = true; },
                m => !m.Crashed && m.HeightChangeMeters > 50 && m.TakeoffDistanceTo4mMeters is > 200 and < 2000),
            new("direct_stall_recovery", 14, 100, ControlLaw.Direct,
                RecoveryPilot,
                f => { f.LeftEngineEnabled = f.RightEngineEnabled = false; f.Throttle = 0; },
                m => !m.Crashed && m.PeakSeparation > .15f && m.EndSeparation < m.PeakSeparation - .1f
                    && m.EndSpeedMetersPerSecond > m.MinimumSpeedMetersPerSecond + 10),
            new("gusty_cruise", 60, 175, ControlLaw.Normal, (_, _) => { },
                f => f.WindEnabled = true,
                m => !m.Crashed && m.PeakG is > 1.02f and < 1.8f && m.MinimumG > .3f
                    && MathF.Abs(m.HeightChangeMeters) < 300 && m.PeakRollRateDegPerSecond < 15),
            new("crosswind_takeoff", 30, 0, ControlLaw.Normal,
                (f, t) => { TakeoffPilot(f, t); TrackRunway(f); },
                f =>
                {
                    f.WindEnabled = true; f.Weather = new(new(4, 0, 0), .5f);
                    f.ParkingBrake = false; f.Throttle = 1; f.Afterburner = true;
                },
                m => !m.Crashed && m.HeightChangeMeters > 50 && m.MaximumCrossTrackMeters < 25),
            new("landing_calm", 65, 90, ControlLaw.Normal, LandingPilot, PrepareLanding,
                m => !m.Crashed && m.StoppedOnRunway && m.TouchdownSinkMetersPerSecond is > .1f and < 3
                    && m.LandingRollMeters is > 100 and < 1300 && m.MaximumCrossTrackMeters < 10, Altitude: 60),
            new("landing_crosswind", 65, 90, ControlLaw.Normal, LandingPilot,
                f =>
                {
                    f.WindEnabled = true; f.Weather = new(new(4, 0, 0), .5f);
                    PrepareLanding(f);
                },
                m => !m.Crashed && m.StoppedOnRunway && m.TouchdownSinkMetersPerSecond is > .1f and < 3
                    && m.LandingRollMeters is > 100 and < 1300 && m.MaximumCrossTrackMeters < 25, Altitude: 60)
        ];
        foreach (Trial trial in trials)
        {
            FlightMetrics metrics = Execute(trial, csv);
            results.Add(metrics);
            bool passed = trial.Accept(metrics);
            if (!passed) failed++;
            Console.WriteLine($"{(passed ? "PASS" : "FAIL")} FLIGHT {trial.Name}: peak {metrics.PeakG:0.00} g, " +
                $"q={metrics.PeakPitchRateDegPerSecond:0.0} deg/s, p={metrics.PeakRollRateDegPerSecond:0.0} deg/s, " +
                $"alpha={metrics.PeakAlphaDegrees:0.0} deg, dE={metrics.SpecificEnergyChangeJoulesPerKilogram:0} J/kg" +
                (metrics.TouchdownSinkMetersPerSecond is float sink
                    ? $", touchdown={sink:0.00} m/s, roll={metrics.LandingRollMeters:0} m, cross-track={metrics.MaximumCrossTrackMeters:0.0} m" : "")
                + (metrics.Crashed ? $", {metrics.Failure}" : ""));
        }
        var mass = MassProperties.FromFuel(AircraftParameters.InitialFuel);
        TrimSolution trim = TrimSolver.Solve(175, 1100, mass);
        var staticModel = new Aerodynamics();
        AeroResult Perturbed(float alpha) => staticModel.EvaluateSteady(new(
            new(0, -MathF.Sin(alpha) * 175, MathF.Cos(alpha) * 175), Vector3.Zero,
            Atmosphere.At(1100), Elevator: trim.Elevator, CenterOfMass: mass.Center));
        AeroResult below = Perturbed(trim.Alpha - .001f), above = Perturbed(trim.Alpha + .001f);
        float staticMargin = (above.Moment.X - below.Moment.X) /
            (AircraftParameters.Chord * (above.Lift - below.Lift));
        var options = new JsonSerializerOptions { WriteIndented = true, Converters = { new JsonStringEnumConverter() } };
        File.WriteAllText("artifacts/flight-validation.json", JsonSerializer.Serialize(new
        {
            Scope = "Engineering regression envelopes, not measurements of a real Su-27",
            InitialMassKg = mass.Mass, CenterOfMassMeters = new[] { mass.Center.X, mass.Center.Y, mass.Center.Z },
            InertiaKgM2 = new[] { mass.Inertia.X.X, mass.Inertia.Y.Y, mass.Inertia.Z.Z },
            EstimatedStaticMarginFractionMac = staticMargin,
            Passed = trials.Length - failed, Failed = failed, Results = results
        }, options));
        File.WriteAllText("artifacts/flight-telemetry.csv", csv.ToString());
        Console.WriteLine($"\nFlight scenarios: {trials.Length - failed} passed, {failed} failed, {trials.Length} executed.");
        Console.WriteLine("Current results: artifacts/flight-validation.json and artifacts/flight-telemetry.csv");
        return failed == 0 ? 0 : 1;
    }

    private static FlightMetrics Execute(Trial trial, StringBuilder csv)
    {
        var f = new FlightModel(ground: trial.Speed == 0, controlLaw: trial.Law);
        if (trial.Speed > 0) f.ResetAirborne(trial.Speed, trial.Altitude);
        trial.Prepare?.Invoke(f);
        float initialHeight = f.Position.Y;
        float initialSpeed = (f.Velocity - Atmosphere.WindAt(f.Time, f.Position, f.WindEnabled, f.Weather)).Length();
        float initialEnergy = .5f * f.Velocity.LengthSquared() + Atmosphere.Gravity * initialHeight;
        float peakG = 1, minimumG = 1, peakPitchRate = 0, peakRollRate = 0, peakAlpha = 0, peakSeparation = 0;
        float minimumSpeed = initialSpeed, bankAtOne = f.Bank * Degrees, heightExcursion = 0;
        bool alphaProtectionUsed = false;
        float? distanceAt4m = null;
        float initialZ = f.Position.Z;
        float initialX = f.Position.X, maximumCrossTrack = 0;
        float? touchdownSink = null, touchdownZ = null, landingRoll = null;
        int steps = (int)MathF.Round(trial.Seconds / FlightModel.FixedStep);
        for (int i = 0; i < steps; i++)
        {
            trial.Input(f, (float)f.Time);
            bool wasOnGround = f.OnGround;
            float incomingVerticalSpeed = f.Velocity.Y;
            f.Step();
            maximumCrossTrack = Math.Max(maximumCrossTrack, MathF.Abs(f.Position.X - initialX));
            if (!wasOnGround && f.OnGround && touchdownSink is null)
            {
                touchdownSink = Math.Max(0, -incomingVerticalSpeed);
                touchdownZ = f.Position.Z;
            }
            if (touchdownZ is float z && landingRoll is null && f.OnGround && f.Velocity.Length() < 1)
                landingRoll = MathF.Abs(f.Position.Z - z);
            peakG = Math.Max(peakG, f.LoadFactor); minimumG = Math.Min(minimumG, f.LoadFactor);
            peakPitchRate = Math.Max(peakPitchRate, MathF.Abs(f.AngularVelocity.X) * Degrees);
            peakRollRate = Math.Max(peakRollRate, MathF.Abs(f.AngularVelocity.Z) * Degrees);
            peakAlpha = Math.Max(peakAlpha, f.Aero.Alpha * Degrees);
            peakSeparation = Math.Max(peakSeparation, f.Aero.Separation);
            alphaProtectionUsed |= f.Controls.AlphaLimited;
            minimumSpeed = Math.Min(minimumSpeed, f.Aero.Speed);
            heightExcursion = Math.Max(heightExcursion, MathF.Abs(f.Position.Y - initialHeight));
            if (trial.Speed == 0 && distanceAt4m is null && !f.OnGround && f.AltitudeAgl > 4)
                distanceAt4m = f.Position.Z - initialZ;
            if (i + 1 == (int)MathF.Round(1 / FlightModel.FixedStep)) bankAtOne = f.Bank * Degrees;
            float groundSpeed = MathF.Sqrt(f.Velocity.X * f.Velocity.X + f.Velocity.Z * f.Velocity.Z);
            if (i % 12 == 0 || i + 1 == steps || f.Crashed)
                csv.AppendLine(FormattableString.Invariant($"{trial.Name},{trial.Law},{f.Time:F4},{f.Aero.Speed:F3},{f.Position.Y:F3},{f.Aero.Alpha * Degrees:F3},{f.Aero.Beta * Degrees:F3},{f.LoadFactor:F4},{-f.AngularVelocity.X * Degrees:F3},{-f.AngularVelocity.Z * Degrees:F3},{f.Bank * Degrees:F3},{f.Elevator:F4},{f.Aileron:F4},{f.Controls.CommandedG:F3},{f.Aero.Separation:F4},{f.DifferentialTail:F4},{f.Position.X - initialX:F3},{f.Velocity.Y:F3},{groundSpeed:F3},{f.OnGround}"));
            if (f.Crashed) break;
        }
        float finalEnergy = .5f * f.Velocity.LengthSquared() + Atmosphere.Gravity * f.Position.Y;
        return new(trial.Name, trial.Law.ToString(), (float)f.Time, peakG, minimumG, peakPitchRate, peakRollRate,
            peakAlpha, peakSeparation, bankAtOne, f.Bank * Degrees, f.AngularVelocity.Z * Degrees,
            f.Position.Y - initialHeight, heightExcursion, initialSpeed, f.Aero.Speed, minimumSpeed,
            finalEnergy - initialEnergy, f.Aero.Separation, alphaProtectionUsed, distanceAt4m, f.Crashed, f.Failure,
            maximumCrossTrack, touchdownSink, landingRoll,
            f.OnGround && f.Velocity.Length() < 1 && Terrain.OnRunway(f.Position.X, f.Position.Z));
    }
}
