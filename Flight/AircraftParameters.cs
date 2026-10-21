using System.Numerics;

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
    public const float ZeroAlphaLift = .12f;
    public const float StallAlpha = 18 * MathF.PI / 180;
    // Body axes: X right (pitch inertia), Y up (yaw), Z nose (roll).
    public static readonly Vector3 Inertia = new(128000, 158000, 31000);
}
