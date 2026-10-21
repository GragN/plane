using Flanker;
using Flanker.Simulation;
using Flanker.Rendering;

if (args.Contains("--self-test"))
    return PhysicsTests.Run();
if (args.Contains("--flight-report"))
    return FlightValidation.Run();

if (args.Contains("--help"))
{
    Console.WriteLine("Flanker | Su-27 research flight simulator");
    Console.WriteLine("dotnet run                  Start the simulator");
    Console.WriteLine("dotnet run -- --self-test    Run deterministic physics tests (no display)");
    Console.WriteLine("dotnet run -- --flight-report Run handling/energy/recovery trials; export CSV and JSON");
    Console.WriteLine("dotnet run -- --smoke-test   Validate 420 frames: runway start, airborne, cockpit; save artifacts/*.png");
    Console.WriteLine("dotnet run -- --opacity 0.1  Window opacity (0.1..1.0); default: 90% transparent");
    return 0;
}

try
{
    return new SimulatorApp().Run(args.Contains("--smoke-test"), WindowAppearance.FromArguments(args));
}
catch (ArgumentException ex) { Console.Error.WriteLine(ex.Message); return 2; }
catch (DllNotFoundException ex)
{
    Console.Error.WriteLine($"Native graphics dependency missing: {ex.Message}");
    Console.Error.WriteLine("Linux: install the desktop libraries listed in README.md. Physics tests need no display.");
    return 1;
}
