using DipLabSim.Core;

namespace DipLabSim.Physics;

/// <summary>Rigid planar links; both angles are absolute, clockwise from upright.
/// Inertias are about each link's COM. Damping c2 acts on relative joint velocity.</summary>
public abstract class DoublePendulumPlant : ITargetedDisturbancePlant
{
    private const string RedesignNote =
        " Changing it while paused re-designs the selected controller for the new plant.";

    private static readonly ParameterDescriptor[] Descriptors =
    [
        new("M", "Cart Mass", "M", "kg", "Masses", 1.5, 0.1, 10, 0.05)
        {
            Help = "Mass of the cart that the actuator pushes. A heavier cart accelerates less for the same force, "
                + "so the controller needs larger forces and may reach its force limit sooner." + RedesignNote
        },
        new("m1", "Lower Link Mass", "m₁", "kg", "Masses", 0.5, 0.01, 2, 0.01)
        {
            Help = "Mass of the lower link. It adds to the total mass the cart must move and to the gravity "
                + "(toppling) torque on the lower link." + RedesignNote
        },
        new("m2", "Upper Link Mass", "m₂", "kg", "Masses", 0.75, 0.01, 2, 0.01)
        {
            Help = "Mass of the upper link. It sits at the end of the lower link, so it loads the lower link as well "
                + "as adding its own gravity torque. Observe how the required force u changes." + RedesignNote
        },
        new("L1", "Lower Link Length", "L₁", "m", "Geometry", 0.5, 0.05, 1.5, 0.01)
        {
            Help = "Length of the lower link: the distance from the cart pivot to the joint between the links. "
                + "The drawing updates immediately. Shortening it also shortens l₁ if needed." + RedesignNote
        },
        new("L2", "Upper Link Length", "L₂", "m", "Geometry", 0.75, 0.05, 1.5, 0.01)
        {
            Help = "Length of the upper link: the distance from the joint to the upper tip, where an upper-tip push "
                + "is applied. The drawing updates immediately. Shortening it also shortens l₂ if needed." + RedesignNote
        },
        new("l1", "Lower Link COM Distance", "l₁", "m", "Geometry", 0.25, 0.001, 1.5, 0.005)
        {
            Help = "Distance from the cart pivot to the lower link's centre of mass. Moving the mass higher increases "
                + "the gravity torque and the moment of inertia about the pivot. Cannot exceed L₁." + RedesignNote
        },
        new("l2", "Upper Link COM Distance", "l₂", "m", "Geometry", 0.375, 0.001, 1.5, 0.005)
        {
            Help = "Distance from the joint to the upper link's centre of mass. Moving the mass higher increases "
                + "the gravity torque and the moment of inertia about the joint. Cannot exceed L₂." + RedesignNote
        },
        new("J1", "Lower Link COM Inertia", "J₁", "kg·m²", "Inertias", 0.010417, 0.000001, 1, 0.0001)
        {
            Help = "Moment of inertia of the lower link about its own centre of mass (a uniform rod has m₁L₁²/12). "
                + "A larger value makes the link resist angular acceleration: it falls more slowly but also "
                + "responds more slowly to the cart." + RedesignNote
        },
        new("J2", "Upper Link COM Inertia", "J₂", "kg·m²", "Inertias", 0.035156, 0.000001, 1, 0.0001)
        {
            Help = "Moment of inertia of the upper link about its own centre of mass (a uniform rod has m₂L₂²/12). "
                + "A larger value makes the link resist angular acceleration." + RedesignNote
        },
        new("g", "Gravity", "g", "m/s²", "Environment", 9.81, 0, 20, 0.01)
        {
            Help = "Gravitational acceleration. Gravity is what makes the upright position unstable: a larger g makes "
                + "the links fall faster and harder to balance; with g = 0 they do not fall at all." + RedesignNote
        },
        new("b", "Cart Viscous Damping", "b", "N·s/m", "Damping & Friction", 0, 0, 5, 0.01)
        {
            Help = "Viscous friction on the cart: a force −b·ẋ opposing cart velocity. It slows the cart and removes "
                + "energy. 0 = frictionless track." + RedesignNote
        },
        new("c1", "Cart / Lower Joint Damping", "c₁", "N·m·s/rad", "Damping & Friction", 0, 0, 0.5, 0.001)
        {
            Help = "Rotational friction at the pivot between the cart and the lower link: a torque −c₁·θ̇₁. "
                + "In open loop, swinging motion dies out faster with a larger value." + RedesignNote
        },
        new("c2", "Lower / Upper Joint Damping", "c₂", "N·m·s/rad", "Damping & Friction", 0, 0, 0.5, 0.001)
        {
            Help = "Rotational friction in the joint between the two links, acting on their relative angular "
                + "velocity θ̇₂ − θ̇₁. A larger value resists the links bending relative to each other." + RedesignNote
        }
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
