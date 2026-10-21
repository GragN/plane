using Flanker;
using Flanker.Simulation;

if (args.Contains("--self-test"))
    return PhysicsTests.Run();

if (args.Contains("--help"))
{
    Console.WriteLine("Flanker | Su-27 research flight simulator");
    Console.WriteLine("dotnet run                  Start the simulator");
    Console.WriteLine("dotnet run -- --self-test    Run deterministic physics tests (no display)");
    Console.WriteLine("dotnet run -- --smoke-test   Validate 420 frames: runway start, airborne, cockpit; save artifacts/*.png");
    return 0;
}

try
{
    return new SimulatorApp().Run(args.Contains("--smoke-test"));
}
catch (DllNotFoundException ex)
{
    Console.Error.WriteLine($"Native graphics dependency missing: {ex.Message}");
    Console.Error.WriteLine("Linux: install the desktop libraries listed in README.md. Physics tests need no display.");
    return 1;
}
