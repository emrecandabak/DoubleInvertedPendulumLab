using DipLabSim.Core;

namespace DipLabSim.Physics;

/// <summary>Rigid planar links; both angles are absolute, clockwise from upright.
/// Inertias are about each link's COM. Damping c2 acts on relative joint velocity.</summary>
public abstract class DoublePendulumPlant : ITargetedDisturbancePlant
{
    private static readonly ParameterDescriptor[] Descriptors =
    [
        new("M", "Cart Mass", "M", "kg", "Masses", 1.5, 0.1, 10, 0.05),
        new("m1", "Lower Link Mass", "m₁", "kg", "Masses", 0.5, 0.01, 2, 0.01),
        new("m2", "Upper Link Mass", "m₂", "kg", "Masses", 0.75, 0.01, 2, 0.01),
        new("L1", "Lower Link Length", "L₁", "m", "Geometry", 0.5, 0.05, 1.5, 0.01),
        new("L2", "Upper Link Length", "L₂", "m", "Geometry", 0.75, 0.05, 1.5, 0.01),
        new("l1", "Lower Link COM Distance", "l₁", "m", "Geometry", 0.25, 0.001, 1.5, 0.005),
        new("l2", "Upper Link COM Distance", "l₂", "m", "Geometry", 0.375, 0.001, 1.5, 0.005),
        new("J1", "Lower Link COM Inertia", "J₁", "kg·m²", "Inertias", 0.010417, 0.000001, 1, 0.0001),
        new("J2", "Upper Link COM Inertia", "J₂", "kg·m²", "Inertias", 0.035156, 0.000001, 1, 0.0001),
        new("g", "Gravity", "g", "m/s²", "Environment", 9.81, 0, 20, 0.01),
        new("b", "Cart Viscous Damping", "b", "N·s/m", "Damping & Friction", 0, 0, 5, 0.01),
        new("c1", "Cart / Lower Joint Damping", "c₁", "N·m·s/rad", "Damping & Friction", 0, 0, 0.5, 0.001),
        new("c2", "Lower / Upper Joint Damping", "c₂", "N·m·s/rad", "Damping & Friction", 0, 0, 0.5, 0.001)
    ];
    private static readonly IReadOnlyList<ParameterDescriptor> ReadOnlyDescriptors = Array.AsReadOnly(Descriptors);
    private readonly Dictionary<string, double> _values = new(StringComparer.Ordinal);
    public event Action? ParametersChanged;
    public abstract string Name { get; }
    public abstract string Description { get; }
    public DisturbanceTarget DisturbanceTarget { get; set; } = DisturbanceTarget.UpperLinkTip;

    protected double M, M1, M2, L1, L2, R1, R2, J1, J2, Gravity, B, C1, C2;
    protected double D1, D2, D3, D4, D5, D6;

    protected DoublePendulumPlant() => ResetToDefaults();
    public IReadOnlyList<ParameterDescriptor> GetParameterDescriptors() => ReadOnlyDescriptors;
    public double GetParameter(string key) => _values[key];

    // Parameter edits must be made while the engine is paused.
    public void SetParameter(string key, double value)
    {
        ParameterDescriptor? descriptor = null;
        foreach (var item in Descriptors) if (item.Key == key) { descriptor = item; break; }
        if (descriptor is null) throw new ArgumentException("Unknown parameter", nameof(key));
        if (!double.IsFinite(value) || value < descriptor.MinValue || value > descriptor.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(value));
        // Keep COM within its link, including when shortening a link.
        if (key == "l1" && value > L1 || key == "l2" && value > L2)
            throw new ArgumentOutOfRangeException(nameof(value), "COM distance cannot exceed link length.");
        _values[key] = value;
        if (key == "L1") _values["l1"] = Math.Min(_values["l1"], value);
        if (key == "L2") _values["l2"] = Math.Min(_values["l2"], value);
        Cache();
        ParametersChanged?.Invoke();
    }

    public void ResetToDefaults()
    {
        foreach (var item in Descriptors) _values[item.Key] = item.DefaultValue;
        Cache();
        ParametersChanged?.Invoke();
    }

    private void Cache()
    {
        M = _values["M"]; M1 = _values["m1"]; M2 = _values["m2"];
        L1 = _values["L1"]; L2 = _values["L2"]; R1 = _values["l1"]; R2 = _values["l2"];
        J1 = _values["J1"]; J2 = _values["J2"]; Gravity = _values["g"];
        B = _values["b"]; C1 = _values["c1"]; C2 = _values["c2"];
        D1 = M + M1 + M2; D2 = M1 * R1 + M2 * L1; D3 = M2 * R2;
        D4 = J1 + M1 * R1 * R1 + M2 * L1 * L1;
        D5 = M2 * L1 * R2; D6 = J2 + M2 * R2 * R2;
    }

    public double Energy(in PendulumState s) =>
        0.5 * (D1 * s.Dx * s.Dx + D4 * s.DTheta1 * s.DTheta1 + D6 * s.DTheta2 * s.DTheta2)
        + D2 * Math.Cos(s.Theta1) * s.Dx * s.DTheta1
        + D3 * Math.Cos(s.Theta2) * s.Dx * s.DTheta2
        + D5 * Math.Cos(s.Theta1 - s.Theta2) * s.DTheta1 * s.DTheta2
        + Gravity * (D2 * Math.Cos(s.Theta1) + D3 * Math.Cos(s.Theta2));

    public abstract void ComputeDerivatives(in PendulumState state, double controlForceU,
        double disturbanceForce, out PendulumStateDerivative derivative);
}
