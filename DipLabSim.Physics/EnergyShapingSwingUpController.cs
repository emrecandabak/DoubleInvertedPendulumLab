using DipLabSim.Core;

namespace DipLabSim.Physics;

/// <summary>
/// Baseline swing-up law: energy shaping through collocated partial feedback linearization (PFL).
/// A desired cart acceleration pumps the pendulum energy E_p toward E_up = g(d2 + d3) and a weak
/// PD term keeps the cart centred; the acceleration is then mapped to cart force u through the
/// mass matrix of the Euler–Lagrange model. This law alone does not guarantee entry into the
/// LQR basin (see the switching report, Section 4); it is the swing-up stage of
/// <see cref="SwingUpSupervisorController"/>.
/// </summary>
public sealed class EnergyShapingSwingUpController : IController
{
    private static readonly ParameterDescriptor[] Descriptors =
    [
        new("KEnergy", "Energy Pumping Gain", "k_E", "1/(J·s)", "Swing-Up Law", 0.3, 0.0, 5.0, 0.01),
        new("KCartX", "Cart Centring Stiffness", "k_x", "1/s²", "Swing-Up Law", 2.0, 0.0, 50.0, 0.1),
        new("KCartV", "Cart Centring Damping", "k_v", "1/s", "Swing-Up Law", 3.0, 0.0, 50.0, 0.1),
        new("MaxAccel", "Cart Acceleration Limit", "a_max", "m/s²", "Swing-Up Law", 15.0, 0.5, 50.0, 0.5),
        new("MaxForce", "Saturation Limit", "u_max", "N", "Swing-Up Law", 80.0, 5.0, 100.0, 1.0)
    ];

    private static readonly IReadOnlyList<ParameterDescriptor> ReadOnlyDescriptors = Array.AsReadOnly(Descriptors);

    // Below this |w| the pumping direction is undefined (e.g. hanging at rest); a fixed
    // direction is used so the law still injects energy instead of sitting still.
    private const double MinPumpingRate = 0.05;

    private readonly Dictionary<string, double> _parameters = new(StringComparer.Ordinal);
    private double _kEnergy, _kCartX, _kCartV, _maxAccel, _maxForce;
    private double _d1, _d2, _d3, _d4, _d5, _d6, _g, _b, _c1, _c2;

    public string Name => "Energy-Shaping Swing-Up (PFL)";
    public string Description => "Collocated PFL: cart acceleration pumps pendulum energy toward the upright level.";

    public EnergyShapingSwingUpController()
    {
        foreach (var item in Descriptors)
            _parameters[item.Key] = item.DefaultValue;
        CacheParameters();
        Initialize(DpcLinearizer.DefaultPlantParameters);
    }

    /// <summary>Upright potential energy E_up = g(d2 + d3) for the current plant.</summary>
    public double UprightEnergy => _g * (_d2 + _d3);

    public IReadOnlyList<ParameterDescriptor> GetParameterDescriptors() => ReadOnlyDescriptors;

    public double GetParameter(string key) => _parameters[key];

    public void SetParameter(string key, double value)
    {
        ParameterDescriptor? descriptor = null;
        foreach (var item in Descriptors)
            if (item.Key == key) { descriptor = item; break; }
        if (descriptor is null) throw new ArgumentException("Unknown swing-up parameter.", nameof(key));
        if (!double.IsFinite(value) || value < descriptor.MinValue || value > descriptor.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(value));
        _parameters[key] = value;
        CacheParameters();
    }

    public void ResetParametersToDefaults()
    {
        foreach (var item in Descriptors)
            _parameters[item.Key] = item.DefaultValue;
        CacheParameters();
    }

    public void Initialize(IReadOnlyDictionary<string, double> plantParameters)
    {
        ArgumentNullException.ThrowIfNull(plantParameters);
        double mCart = DpcLinearizer.GetOrDefault(plantParameters, "M", 1.5);
        double m1 = DpcLinearizer.GetOrDefault(plantParameters, "m1", 0.5);
        double m2 = DpcLinearizer.GetOrDefault(plantParameters, "m2", 0.75);
        double l1 = DpcLinearizer.GetOrDefault(plantParameters, "L1", 0.5);
        double l2 = DpcLinearizer.GetOrDefault(plantParameters, "L2", 0.75);
        double r1 = Math.Min(l1, DpcLinearizer.GetOrDefault(plantParameters, "l1", 0.25));
        double r2 = Math.Min(l2, DpcLinearizer.GetOrDefault(plantParameters, "l2", 0.375));
        double j1 = DpcLinearizer.GetOrDefault(plantParameters, "J1", 0.010417);
        double j2 = DpcLinearizer.GetOrDefault(plantParameters, "J2", 0.035156);
        _g = DpcLinearizer.GetOrDefault(plantParameters, "g", 9.81);
        _b = DpcLinearizer.GetOrDefault(plantParameters, "b", 0.0);
        _c1 = DpcLinearizer.GetOrDefault(plantParameters, "c1", 0.0);
        _c2 = DpcLinearizer.GetOrDefault(plantParameters, "c2", 0.0);

        _d1 = mCart + m1 + m2;
        _d2 = m1 * r1 + m2 * l1;
        _d3 = m2 * r2;
        _d4 = j1 + m1 * r1 * r1 + m2 * l1 * l1;
        _d5 = m2 * l1 * r2;
        _d6 = j2 + m2 * r2 * r2;
    }

    /// <summary>Pendulum energy in the cart frame: link kinetic energy plus potential energy.</summary>
    public double PendulumEnergy(in PendulumState s) =>
        0.5 * (_d4 * s.DTheta1 * s.DTheta1 + _d6 * s.DTheta2 * s.DTheta2)
        + _d5 * Math.Cos(s.Theta1 - s.Theta2) * s.DTheta1 * s.DTheta2
        + _g * (_d2 * Math.Cos(s.Theta1) + _d3 * Math.Cos(s.Theta2));

    public double ComputeControlInput(in PendulumState state, in PendulumState reference, double dt)
    {
        double s1 = Math.Sin(state.Theta1), c1 = Math.Cos(state.Theta1);
        double s2 = Math.Sin(state.Theta2), c2 = Math.Cos(state.Theta2);
        double delta = state.Theta1 - state.Theta2;
        double sinDelta = Math.Sin(delta), cosDelta = Math.Cos(delta);

        // dE_p/dt = -xdd * w (undamped), so xdd = kE (E_p - E_up) w drives E_p to E_up.
        double energyError = PendulumEnergy(state) - UprightEnergy;
        double w = _d2 * c1 * state.DTheta1 + _d3 * c2 * state.DTheta2;
        if (Math.Abs(w) < MinPumpingRate) w = Math.CopySign(MinPumpingRate, w);

        double accel = _kEnergy * energyError * w
            - _kCartX * (state.X - reference.X)
            - _kCartV * (state.Dx - reference.Dx);
        accel = Math.Clamp(accel, -_maxAccel, _maxAccel);

        // Collocated PFL: with xdd fixed, solve the 2x2 link block for the link accelerations,
        // then read u from the cart row of M(q) qdd = rhs + H u.
        double mx1 = _d2 * c1, mx2 = _d3 * c2;
        double m11 = _d4, m12 = _d5 * cosDelta, m22 = _d6;
        double relativeDamping = _c2 * (state.DTheta2 - state.DTheta1);
        double r1 = -_c1 * state.DTheta1 + relativeDamping
            - _d5 * sinDelta * state.DTheta2 * state.DTheta2 + _d2 * _g * s1 - mx1 * accel;
        double r2 = -relativeDamping
            + _d5 * sinDelta * state.DTheta1 * state.DTheta1 + _d3 * _g * s2 - mx2 * accel;
        double det = m11 * m22 - m12 * m12;
        double dd1 = (m22 * r1 - m12 * r2) / det;
        double dd2 = (m11 * r2 - m12 * r1) / det;

        double u = _d1 * accel + mx1 * dd1 + mx2 * dd2
            + _b * state.Dx
            - _d2 * s1 * state.DTheta1 * state.DTheta1
            - _d3 * s2 * state.DTheta2 * state.DTheta2;

        return double.IsFinite(u) ? Math.Clamp(u, -_maxForce, _maxForce) : 0.0;
    }

    public void Reset() { }

    public void Dispose() { }

    private void CacheParameters()
    {
        _kEnergy = _parameters["KEnergy"];
        _kCartX = _parameters["KCartX"];
        _kCartV = _parameters["KCartV"];
        _maxAccel = _parameters["MaxAccel"];
        _maxForce = _parameters["MaxForce"];
    }
}
