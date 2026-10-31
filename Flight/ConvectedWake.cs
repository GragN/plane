using System.Numerics;

namespace Flanker.Flight;

// History travels from wing to tail at the local forward speed. Evaluating a force
// never advances this state: midpoint evaluations must read the same wake history.
internal sealed class ConvectedWake
{
    internal readonly record struct Sample(double Time, Vector2 Downwash, Vector2 Separation);
    private readonly Sample[] history = new Sample[512];
    private int head, count;
    private double time;

    public void Reset(Vector2 downwash, Vector2 separation)
    {
        head = 0; count = 1; time = 0;
        history[0] = new(0, downwash, separation);
    }

    public void Advance(float dt, Vector2 downwash, Vector2 separation)
    {
        time += dt;
        head = (head + 1) % history.Length;
        count = Math.Min(count + 1, history.Length);
        history[head] = new(time, downwash, separation);
    }

    public Sample At(float delay)
    {
        double target = time - Math.Max(0, delay);
        Sample newer = history[head];
        for (int i = 1; i < count; i++)
        {
            Sample older = history[(head - i + history.Length) % history.Length];
            if (older.Time <= target)
            {
                float blend = (float)Math.Clamp((target - older.Time) / (newer.Time - older.Time), 0, 1);
                return new(target, Vector2.Lerp(older.Downwash, newer.Downwash, blend),
                    Vector2.Lerp(older.Separation, newer.Separation, blend));
            }
            newer = older;
        }
        return newer;
    }
}
