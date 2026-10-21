namespace Flanker.Flight;

public sealed class JetEngine
{
    public bool Enabled { get; set; } = true;
    public float Rpm { get; private set; }
    public float Reheat { get; private set; }
    public float Thrust { get; private set; }
    public float FuelFlow { get; private set; }

    public void Reset(float throttle)
    {
        Enabled = true;
        Rpm = .55f + .45f * throttle;
        Reheat = Thrust = FuelFlow = 0;
    }

    public static float DryThrust(Air air, float mach, float throttle)
    {
        float altitude = MathF.Pow(air.Density / 1.225f, .70f);
        float ram = (1 + .12f * mach * mach) * Math.Max(.4f, 1 - .08f * mach);
        float power = .025f + .975f * throttle * throttle;
        return AircraftParameters.DryThrust * .5f * altitude * ram * power;
    }

    public void Step(float dt, float throttle, bool reheat, Air air, float mach,
        float alpha, float beta, bool fuelAvailable)
    {
        bool running = Enabled && fuelAvailable;
        float target = running ? .55f + .45f * throttle : .12f * Math.Clamp(mach, 0, 1);
        float tau = target > Rpm ? 1.6f - .8f * Rpm : 1.1f;
        Rpm += (target - Rpm) * (1 - MathF.Exp(-dt / tau));
        float abTarget = running && reheat && throttle > .98f && Rpm > .97f ? 1 : 0;
        Reheat += (abTarget - Reheat) * (1 - MathF.Exp(-dt / .45f));
        float normalizedPower = Math.Clamp((Rpm - .55f) / .45f, 0, 1);
        float distortion = .55f * AerodynamicTables.Smooth(.35f, .95f, MathF.Abs(alpha))
            + .25f * AerodynamicTables.Smooth(.2f, .65f, MathF.Abs(beta));
        float inlet = 1 - distortion * AerodynamicTables.Smooth(.03f, .15f, mach);
        Thrust = running ? DryThrust(air, mach, normalizedPower) * inlet *
            (1 + Reheat * (AircraftParameters.AfterburnerThrust / AircraftParameters.DryThrust - 1)) : 0;
        FuelFlow = running ? .08f + Thrust * (.000022f + .000025f * Reheat) : 0;
    }
}
