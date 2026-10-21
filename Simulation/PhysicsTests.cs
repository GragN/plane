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
        Test("Dynamic-pressure scaling at fixed Mach and Reynolds number", () =>
        {
            // Hold non-dimensional flow parameters fixed while quadrupling q.
            AeroResult At(float density, float speed)
            {
                Air original = Atmosphere.At(0);
                var air = original with { Density = density, SoundSpeed = speed / .3f };
                return Aerodynamics.Evaluate(new(0, -MathF.Sin(.1f) * speed, MathF.Cos(.1f) * speed),
                    Vector3.Zero, air, 0, 0, 0, 0, 0);
            }
            AeroResult slow = At(1.225f, 100), fast = At(1.225f / 4, 400);
            Check(MathF.Abs(fast.Lift / slow.Lift - 4) < .002f, "lift is not proportional to dynamic pressure");
            Check(MathF.Abs(fast.Drag / slow.Drag - 4) < .002f, "drag is not proportional to dynamic pressure");
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
            float deltaCl = MathF.Abs(Aero(alpha: a - .0001f).Lift - Aero(alpha: a + .0001f).Lift)
                / (.5f * Atmosphere.At(0).Density * 100 * 100 * AircraftParameters.WingArea);
            Check(deltaCl < .002f,
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
            var f = new FlightModel(ground: true); Advance(f, 20);
            Check(!f.Crashed && f.OnGround, f.Failure);
            Check(f.Position.Y is > 2.2f and < 2.8f && f.Velocity.Length() < .1f && f.ParkingBrake,
                $"ground equilibrium: y={f.Position.Y}, vy={f.Velocity.Y}");
        });
        Test("Full-power runway rotation produces takeoff", () =>
        {
            var f = new FlightModel(ground: true) { Throttle = 1, Afterburner = true, ParkingBrake = false };
            for (int i = 0; i < (int)(40 / FlightModel.FixedStep); i++)
            {
                // A test pilot rotates to ten degrees, rather than holding an arbitrary surface deflection forever.
                f.ElevatorInput = f.Aero.Speed < 82 ? 0 : Math.Clamp(.18f + 2.5f * (.1745f - f.Pitch)
                    + .6f * f.AngularVelocity.X, -.3f, .65f);
                if (f.AltitudeAgl > 8) f.GearDown = false;
                f.Step();
                if (f.AltitudeAgl > 50) break;
            }
            Check(!f.Crashed && f.AltitudeAgl > 30, $"takeoff: {f.Failure}, AGL={f.AltitudeAgl:0.0}, speed={f.Aero.Speed:0.0}, pitch={f.Pitch * 180 / MathF.PI:0.0}");
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
            Advance(a, 5); Advance(b, 5, 1f / 480);
            Check(Vector3.Distance(a.Position, b.Position) < 3, $"position error {Vector3.Distance(a.Position, b.Position):0.00} m");
        });

        Test("Control torque comes from forces at physical tail locations", () =>
        {
            AeroContext context = new(new(0, -8, 160), Vector3.Zero, Atmosphere.At(1000));
            var model = new Aerodynamics();
            AeroResult neutral = model.EvaluateSteady(context);
            var baseline = model.Loads.ToArray();
            AeroResult controlled = model.EvaluateSteady(context with { Elevator = .2f });
            Vector3 momentChange = Vector3.Zero;
            for (int i = 0; i < baseline.Length; i++)
                momentChange += Vector3.Cross(baseline[i].Position, model.Loads[i].Force - baseline[i].Force);
            Check(MathF.Abs(controlled.Moment.X - neutral.Moment.X - momentChange.X) < 10,
                "elevator torque bypasses the surface forces");
        });
        Test("Rotation creates aerodynamic roll damping without commanded controls", () =>
        {
            var r = Aerodynamics.Evaluate(new(0, -10, 150), new(0, 0, .4f), Atmosphere.At(0), 0, 0, 0, 0, 0);
            Check(r.Moment.Z < -10000, $"roll damping torque {r.Moment.Z:0}");
            float power = Vector3.Dot(r.Force, new(0, -10, 150)) + Vector3.Dot(r.Moment, new(0, 0, .4f));
            Check(power < 0, "unpowered aerodynamic force/torque creates mechanical energy");
        });
        Test("Rotating wings can stall asymmetrically", () =>
        {
            float alpha = .38f, speed = 85;
            var r = Aerodynamics.Evaluate(new(0, -MathF.Sin(alpha) * speed, MathF.Cos(alpha) * speed),
                new(0, 0, .9f), Atmosphere.At(0), 0, 0, 0, 0, 0);
            Check(r.LeftSeparation > r.RightSeparation + .015f,
                $"left separation {r.LeftSeparation:0.000}, right {r.RightSeparation:0.000}");
        });
        Test("Separated flow retains history and reattaches gradually", () =>
        {
            AeroContext Context(float alpha) => new(new(0, -MathF.Sin(alpha) * 100, MathF.Cos(alpha) * 100),
                Vector3.Zero, Atmosphere.At(0));
            var model = new Aerodynamics();
            AeroContext attached = Context(.1f), stalled = Context(.75f);
            model.ResetFlow(attached);
            for (int i = 0; i < 120; i++) model.AdvanceFlow(stalled, FlightModel.FixedStep);
            float before = model.Evaluate(stalled).Separation;
            model.AdvanceFlow(attached, FlightModel.FixedStep);
            float recovering = model.Evaluate(attached).Separation;
            Check(before > .5f && recovering > .4f, $"stall memory {before:0.00} -> {recovering:0.00}");
            for (int i = 0; i < 720; i++) model.AdvanceFlow(attached, FlightModel.FixedStep);
            Check(model.Evaluate(attached).Separation < .02f, "flow never reattached");
        });
        Test("Aileron input produces adverse yaw through unequal drag", () =>
        {
            var r = Aerodynamics.Evaluate(new(0, -20, 150), Vector3.Zero, Atmosphere.At(0), 0, .4f, 0, 0, 0);
            Check(r.Moment.Z < 0 && r.Moment.Y < 0, $"aileron moments roll={r.Moment.Z:0}, yaw={r.Moment.Y:0}");
        });
        Test("Ground effect reduces induced drag", () =>
        {
            var model = new Aerodynamics();
            AeroContext c = new(new(0, -20, 90), Vector3.Zero, Atmosphere.At(0));
            var far = model.EvaluateSteady(c);
            var near = model.EvaluateSteady(c with { GroundClearance = .4f });
            Check(near.Drag < far.Drag * .9f, $"induced drag far={far.Drag:0}, near={near.Drag:0}");
        });
        Test("Wing downwash changes the tail's local angle of attack", () =>
        {
            var model = new Aerodynamics();
            model.EvaluateSteady(new(new(0, -20, 150), Vector3.Zero, Atmosphere.At(0)));
            var tail = model.Loads.Where(p => p.Position.Z < -6 && p.Position.Y < .5f).ToArray();
            Check(tail.Length == 6 && tail.Average(p => p.Alpha) < MathF.Atan2(20, 150) - .02f, "missing tail downwash");
        });
        Test("Fuel changes CG and the full inertia tensor", () =>
        {
            var full = MassProperties.FromFuel(5000); var empty = MassProperties.FromFuel(0);
            Check(Vector3.Distance(full.Center, empty.Center) > .04f, "fuel has no CG effect");
            Check(full.Inertia.Solve(Vector3.UnitX * 100000).X < empty.Inertia.Solve(Vector3.UnitX * 100000).X,
                "fuel does not increase pitch inertia");
            Check(MathF.Abs(full.Inertia.Y.Z) > 1000, "product of inertia missing");
        });
        Test("One engine out produces asymmetric-thrust yaw", () =>
        {
            var f = new FlightModel { LeftEngineEnabled = false }; Advance(f, .15f);
            Check(!f.Crashed && f.LeftThrust == 0 && f.RightThrust > 1000 && f.AngularVelocity.Y < -.005f,
                $"engine-out yaw {f.AngularVelocity.Y:0.000}, L={f.LeftThrust:0}, R={f.RightThrust:0}");
        });
        Test("Inlet distortion at high alpha reduces thrust", () =>
        {
            var clean = new JetEngine(); var distorted = new JetEngine(); clean.Reset(1); distorted.Reset(1);
            clean.Step(.1f, 1, false, Atmosphere.At(0), .5f, .05f, 0, true);
            distorted.Step(.1f, 1, false, Atmosphere.At(0), .5f, 1, 0, true);
            Check(distorted.Thrust < clean.Thrust * .6f, "no high-alpha inlet loss");
        });
        Test("Wheel brakes stop the aircraft; free wheels spin up", () =>
        {
            var rolling = new FlightModel(ground: true) { ParkingBrake = false, LeftEngineEnabled = false, RightEngineEnabled = false };
            var braking = new FlightModel(ground: true) { ParkingBrake = true, LeftEngineEnabled = false, RightEngineEnabled = false };
            rolling.Velocity = braking.Velocity = new(0, 0, 30);
            Advance(rolling, 9); Advance(braking, 9);
            Check(!rolling.Crashed && !braking.Crashed, $"ground failures: {rolling.Failure} / {braking.Failure}");
            Check(rolling.Velocity.Z > 20 && MathF.Abs(braking.Velocity.Z) < 2,
                $"brakes: free={rolling.Velocity.Z:0.00}, braked={braking.Velocity.Z:0.00}");
            Check(MathF.Abs(rolling.WheelSpeeds[1] * .4f - rolling.Velocity.Z) < 3, "free-wheel rolling speed mismatch");
        });

        Console.WriteLine($"\n{passed} passed, {failed} failed, {passed + failed} executed.");
        return failed == 0 ? 0 : 1;
    }
}
