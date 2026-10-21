using System.Numerics;
using Flanker.Flight;

namespace Flanker.Simulation;

public static class PhysicsTests
{
    public static int Run()
    {
        int passed = 0, failed = 0;
        void Test(string name, Action action)
        {
            try { action(); Console.WriteLine($"PASS  {name}"); passed++; }
            catch (Exception ex) { Console.WriteLine($"FAIL  {name}: {ex.Message}"); failed++; }
        }
        static void Check(bool condition, string message)
        { if (!condition) throw new InvalidOperationException(message); }
        static void Advance(FlightModel f, float seconds, float dt = FlightModel.FixedStep)
        { for (int i = 0; i < (int)MathF.Round(seconds / dt); i++) f.Step(dt); }
        static AeroResult Aero(float speed = 100, float alpha = .1f, float gear = 0, float brake = 0) =>
            Aerodynamics.Evaluate(new(0, -MathF.Sin(alpha) * speed, MathF.Cos(alpha) * speed),
                Vector3.Zero, Atmosphere.At(0), 0, 0, 0, gear, brake);

        Test("ISA reference atmosphere", () =>
        {
            Check(MathF.Abs(Atmosphere.At(0).Density - 1.225f) < .002f, "sea-level density");
            Check(MathF.Abs(Atmosphere.At(11000).Density - .364f) < .003f, "11 km density");
            Check(Atmosphere.At(20000).Density < Atmosphere.At(11000).Density, "stratosphere density");
        });
        Test("Lift and drag scale with dynamic pressure", () =>
        {
            Check(MathF.Abs(Aero(200).Lift / Aero(100).Lift - 4) < .001f, "lift is not proportional to V squared");
            Check(MathF.Abs(Aero(200).Drag / Aero(100).Drag - 4) < .001f, "drag is not proportional to V squared");
        });
        Test("Lift perpendicular to flow; drag dissipates energy", () =>
        {
            Vector3 velocity = new(0, -10, 150);
            AeroResult r = Aerodynamics.Evaluate(velocity, Vector3.Zero, Atmosphere.At(0), 0, 0, 0, 0, 0);
            float work = Vector3.Dot(r.Force, Vector3.Normalize(velocity));
            Check(MathF.Abs(work + r.Drag) < 1, $"unexpected aerodynamic work: {work}");
        });
        Test("Post-stall lift loss and drag rise", () =>
        {
            Check(Aero(alpha: .6f).Lift < Aero(alpha: .30f).Lift, "no post-stall lift loss");
            Check(Aero(alpha: .6f).Drag > Aero(alpha: .15f).Drag * 2, "no separated-flow drag rise");
            float a = AircraftParameters.StallAlpha;
            Check(MathF.Abs(Aerodynamics.LiftCoefficient(a - .0001f) - Aerodynamics.LiftCoefficient(a + .0001f)) < .002f,
                "discontinuous stall onset");
        });
        Test("Gear and airbrake increase drag", () =>
        {
            Check(Aero(gear: 1).Drag > Aero().Drag * 1.5f, "gear drag");
            Check(Aero(brake: 1).Drag > Aero().Drag * 2, "airbrake drag");
        });
        Test("Transonic drag rise and finite supersonic coefficients", () =>
        {
            float Cd(float mach)
            {
                var a = Aero(speed: mach * Atmosphere.At(0).SoundSpeed, alpha: 0);
                return a.Drag / (.5f * 1.225f * a.Speed * a.Speed * AircraftParameters.WingArea);
            }
            Check(Cd(1.05f) > Cd(.7f) * 1.5f, "missing transonic drag rise");
            Check(Cd(2) is > .02f and < .1f, "unphysical supersonic wave-drag growth");
        });
        Test("Trimmed level flight remains near equilibrium for 60 s", () =>
        {
            var f = new FlightModel(); Advance(f, 60);
            Check(!f.Crashed, f.Failure);
            Check(MathF.Abs(f.Position.Y - 1100) < 100, $"altitude {f.Position.Y:0.0} m");
            Check(MathF.Abs(f.Aero.Speed - 175) < 15, $"speed {f.Aero.Speed:0.0} m/s");
        });
        Test("Elevator produces nose-up angular acceleration", () =>
        {
            var f = new FlightModel { ElevatorInput = .5f }; Advance(f, .5f);
            Check(f.AngularVelocity.X < -.05f && f.Pitch > .04f, $"pitch rate {f.AngularVelocity.X}");
        });
        Test("Ailerons roll right and rudder yaws right", () =>
        {
            var roll = new FlightModel { RollInput = .5f }; Advance(roll, .5f);
            var yaw = new FlightModel { RudderInput = .5f }; Advance(yaw, .5f);
            Check(roll.AngularVelocity.Z < -.1f, "roll direction");
            Check(yaw.AngularVelocity.Y > .02f, "yaw direction");
        });
        Test("No fuel means no thrust", () =>
        {
            var f = new FlightModel { Fuel = 0, Throttle = 1, Afterburner = true }; Advance(f, 4);
            Check(f.Thrust == 0 && f.Fuel == 0, "fuel starvation");
        });
        Test("Engine spool-up, afterburner and fuel consumption", () =>
        {
            var dry = new FlightModel { Throttle = 1 };
            var ab = new FlightModel { Throttle = 1, Afterburner = true };
            Advance(dry, 4); Advance(ab, 4);
            Check(ab.Thrust > dry.Thrust * 1.4f, "afterburner thrust");
            Check(ab.Fuel < dry.Fuel && dry.Fuel < AircraftParameters.InitialFuel, "fuel consumption");
        });
        Test("Landing gear supports a stationary aircraft", () =>
        {
            var f = new FlightModel(); f.Reset(true); Advance(f, 20);
            Check(!f.Crashed && f.OnGround, f.Failure);
            Check(f.Position.Y is > 1.5f and < 2.2f && MathF.Abs(f.Velocity.Y) < .1f,
                $"ground equilibrium: y={f.Position.Y}, vy={f.Velocity.Y}");
        });
        Test("Full-power runway rotation produces takeoff", () =>
        {
            var f = new FlightModel(); f.Reset(true); f.Throttle = 1; f.Afterburner = true;
            for (int i = 0; i < 120 * 35; i++)
            {
                f.ElevatorInput = f.Aero.Speed > 72 ? .16f : 0;
                if (f.AltitudeAgl > 8) { f.GearDown = false; f.ElevatorInput = .07f; }
                f.Step();
                if (f.AltitudeAgl > 50) break;
            }
            Check(!f.Crashed && f.AltitudeAgl > 30, $"takeoff: {f.Failure}, AGL={f.AltitudeAgl:0.0}, speed={f.Aero.Speed:0.0}");
        });
        Test("Hard terrain impact ends flight", () =>
        {
            var f = new FlightModel { Position = new(0, 8, 0), Velocity = new(0, -25, 0) };
            Advance(f, 1); Check(f.Crashed, "ground collision ignored");
        });
        Test("Unpowered glide loses mechanical energy", () =>
        {
            var f = new FlightModel { Fuel = 0, Throttle = 0 };
            float initial = .5f * f.Velocity.LengthSquared() + Atmosphere.Gravity * f.Position.Y;
            Advance(f, 20);
            float final = .5f * f.Velocity.LengthSquared() + Atmosphere.Gravity * f.Position.Y;
            Check(!f.Crashed && final < initial - 500, $"specific energy: {initial:0} -> {final:0}");
        });
        Test("A banked aircraft turns under the lateral lift component", () =>
        {
            var f = new FlightModel();
            f.Orientation *= Quaternion.CreateFromAxisAngle(Vector3.UnitZ, -.35f);
            Advance(f, 6);
            Check(!f.Crashed && f.Position.X > 20 && f.Heading > 1,
                $"banked turn: x={f.Position.X:0.0}, heading={f.Heading:0.0}");
        });
        Test("Quaternion stays normalized through a maneuver", () =>
        {
            var f = new FlightModel { Position = new(0, 7000, 0), RollInput = .3f };
            Advance(f, 12);
            Check(!f.Crashed, f.Failure);
            Check(MathF.Abs(f.Orientation.LengthSquared() - 1) < 1e-5f, "orientation normalization");
            Check(float.IsFinite(f.Velocity.LengthSquared()), "non-finite velocity");
        });
        Test("Fixed-step numerical convergence", () =>
        {
            var a = new FlightModel { ElevatorInput = .08f, RollInput = .04f };
            var b = new FlightModel { ElevatorInput = .08f, RollInput = .04f };
            Advance(a, 5); Advance(b, 5, 1f / 240);
            Check(Vector3.Distance(a.Position, b.Position) < 3, $"position error {Vector3.Distance(a.Position, b.Position):0.00} m");
        });

        Console.WriteLine($"\n{passed} passed, {failed} failed, {passed + failed} executed.");
        return failed == 0 ? 0 : 1;
    }
}
