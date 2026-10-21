namespace Flanker.Flight;

// Public Su-27 dimensions/performance anchors. Coefficients and inertias are estimates,
// NOT flight-manual tables. Keep these explicit so the model can be calibrated.
public static class AircraftParameters
{
    public const float EmptyMass = 16300;
    public const float InitialFuel = 5000;
    public const float WingArea = 62;
    public const float Span = 14.7f;
    public const float Chord = 4.8f;
    public const float DryThrust = 149000; // Both AL-31F engines together, sea level.
    public const float AfterburnerThrust = 245200;
    public const float LiftSlope = 4.4f; // per radian
    public const float ZeroAlphaLift = .10f;
    public const float StallAlpha = 22 * MathF.PI / 180; // nominal; individual strips differ
}
