namespace DipLabSim.Core;

public readonly record struct PendulumState(
    double Time,
    double X,
    double Theta1,
    double Theta2,
    double Dx,
    double DTheta1,
    double DTheta2)
{
    public static PendulumState Zero => new(0, 0, 0, 0, 0, 0, 0);

    public static PendulumState AddScaled(in PendulumState s, in PendulumStateDerivative d, double scale) =>
        new(
            s.Time + scale,
            s.X + d.Dx * scale,
            s.Theta1 + d.DTheta1 * scale,
            s.Theta2 + d.DTheta2 * scale,
            s.Dx + d.Ddx * scale,
            s.DTheta1 + d.DdTheta1 * scale,
            s.DTheta2 + d.DdTheta2 * scale);
}

public readonly record struct PendulumStateDerivative(
    double Dx,
    double DTheta1,
    double DTheta2,
    double Ddx,
    double DdTheta1,
    double DdTheta2);

/// <summary>
/// Self-describing parameter metadata used by the WPF Configuration Tab to auto-generate sliders/inputs.
/// </summary>
public sealed record ParameterDescriptor(
    string Key,
    string DisplayName,
    string Symbol,
    string Unit,
    string Category,
    double DefaultValue,
    double MinValue,
    double MaxValue,
    double Step = 0.01);

public interface IConfigurableComponent
{
    string Name { get; }
    string Description { get; }
    IReadOnlyList<ParameterDescriptor> GetParameterDescriptors();
    double GetParameter(string key);
    void SetParameter(string key, double value);
}

public interface IPlant : IConfigurableComponent
{
    void ComputeDerivatives(
        in PendulumState state,
        double controlForceU,
        double disturbanceForce,
        out PendulumStateDerivative derivative);
}

public interface IOdeSolver
{
    string Name { get; }
    PendulumState Step(
        IPlant plant,
        in PendulumState current,
        double controlForceU,
        double disturbanceForce,
        double dt);
}

public interface IController : IConfigurableComponent, IDisposable
{
    void Initialize(IReadOnlyDictionary<string, double> plantParameters);
    double ComputeControlInput(in PendulumState state, in PendulumState reference, double dt);
    void Reset();
}

public readonly record struct SimulationFrame(
    PendulumState State,
    PendulumState Reference,
    double ControlForceU,
    double DisturbanceForce);
