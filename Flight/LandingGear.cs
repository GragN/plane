using System.Numerics;
using Flanker.Simulation;

namespace Flanker.Flight;

public readonly record struct ContactResult(Vector3 Force, Vector3 Moment, bool OnGround,
    float ImpactSpeed, float Compression);

public sealed class LandingGear
{
    public static readonly Vector3[] Mounts = [new(0, -2.75f, 5.2f), new(-2.4f, -2.75f, -1.7f), new(2.4f, -2.75f, -1.7f)];
    private readonly float[] wheelSpeeds = new float[3], nextSpeeds = new float[3], compression = new float[3];
    public IReadOnlyList<float> WheelSpeeds => wheelSpeeds;
    public IReadOnlyList<float> Compression => compression;
    public void Reset() { Array.Clear(wheelSpeeds); Array.Clear(nextSpeeds); Array.Clear(compression); }
    public void Commit() => nextSpeeds.CopyTo(wheelSpeeds, 0);

    public ContactResult Evaluate(Vector3 position, Vector3 velocity, Quaternion orientation, Vector3 omega,
        MassProperties mass, float extension, float steering, bool brakes, float dt)
    {
        Vector3 force = Vector3.Zero, moment = Vector3.Zero;
        bool onGround = false;
        float impact = 0, maxCompression = 0;
        for (int i = 0; i < Mounts.Length; i++)
        {
            bool nose = i == 0;
            float radius = nose ? .30f : .40f;
            float wheelInertia = nose ? 1.8f : 4.2f;
            Vector3 arm = Mounts[i] - mass.Center;
            Vector3 world = position + Vector3.Transform(arm, orientation);
            float penetration = Terrain.Height(world.X, world.Z) - world.Y;
            compression[i] = extension > .95f ? Math.Max(0, penetration) : 0;
            nextSpeeds[i] = wheelSpeeds[i] / (1 + dt * (brakes ? 20 : .04f));
            if (extension <= .95f || penetration <= 0) continue;
            Vector3 v = velocity + Vector3.Transform(Vector3.Cross(omega, arm), orientation);
            Vector3 normal = Vector3.Normalize(new Vector3(
                Terrain.Height(world.X - 1, world.Z) - Terrain.Height(world.X + 1, world.Z), 2,
                Terrain.Height(world.X, world.Z - 1) - Terrain.Height(world.X, world.Z + 1)));
            float vn = Vector3.Dot(v, normal);
            float spring = nose ? 260000 : 430000;
            float damping = nose ? 30000 : 47000;
            float normalForce = Math.Clamp(spring * penetration * (1 + .7f * penetration) - damping * vn, 0, 1800000);
            if (normalForce <= 0) continue;
            onGround = true;
            impact = Math.Max(impact, -vn);
            maxCompression = Math.Max(maxCompression, penetration);
            float steerAngle = nose ? steering * .55f / (1 + velocity.LengthSquared() / 130) : 0;
            Vector3 forward = Vector3.Transform(Vector3.Transform(Vector3.UnitZ,
                Quaternion.CreateFromAxisAngle(Vector3.UnitY, steerAngle)), orientation);
            forward = Vector3.Normalize(forward - normal * Vector3.Dot(forward, normal));
            Vector3 side = Vector3.Cross(normal, forward);
            float longitudinal = Vector3.Dot(v, forward), lateral = Vector3.Dot(v, side);
            float grip = Terrain.OnRunway(world.X, world.Z) ? .82f : .43f;
            float limit = grip * normalForce;
            float lateralForce = -limit * MathF.Tanh(7 * MathF.Atan2(lateral, Math.Max(2, MathF.Abs(longitudinal))));
            float longitudinalLimit = MathF.Sqrt(Math.Max(0, limit * limit - lateralForce * lateralForce));
            float speedReference = Math.Max(3, MathF.Abs(longitudinal));
            float brakeTorque = brakes ? (nose ? 4500 : 14500) : 0;
            float rollingTorque = .013f * normalForce * radius;
            float Traction(float w) => longitudinalLimit * MathF.Tanh(9 * (w * radius - longitudinal) / speedReference);
            // Implicit wheel/tire solve avoids the low-speed instability of explicit slip integration.
            float Residual(float w) => w - wheelSpeeds[i] + dt / wheelInertia *
                (Traction(w) * radius + (brakeTorque + rollingTorque) * MathF.Tanh(w * 2));
            float bound = dt / wheelInertia * (longitudinalLimit * radius + brakeTorque + rollingTorque) + .01f;
            float low = wheelSpeeds[i] - bound, high = wheelSpeeds[i] + bound;
            for (int iteration = 0; iteration < 24; iteration++)
            {
                float mid = (low + high) / 2;
                if (Residual(mid) > 0) high = mid; else low = mid;
            }
            nextSpeeds[i] = (low + high) / 2;
            Vector3 f = normal * normalForce + side * lateralForce + forward * Traction(nextSpeeds[i]);
            force += f;
            moment += Vector3.Cross(arm, Vector3.Transform(f, Quaternion.Conjugate(orientation)));
        }
        return new(force, moment, onGround, impact, maxCompression);
    }
}
