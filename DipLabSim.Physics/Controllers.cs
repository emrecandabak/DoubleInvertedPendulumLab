using DipLabSim.Core;

namespace DipLabSim.Physics;

internal static class DpcLinearizer
{
    public static readonly IReadOnlyDictionary<string, double> DefaultPlantParameters =
        new Dictionary<string, double>(StringComparer.Ordinal)
        {
            ["M"] = 1.5,
            ["m1"] = 0.5,
            ["m2"] = 0.75,
            ["L1"] = 0.5,
            ["L2"] = 0.75,
            ["l1"] = 0.25,
            ["l2"] = 0.375,
            ["J1"] = 0.010417,
            ["J2"] = 0.035156,
            ["g"] = 9.81,
            ["b"] = 0.0,
            ["c1"] = 0.0,
            ["c2"] = 0.0
        };

    public static double WrapAngle(double radians) => Math.IEEERemainder(radians, 2.0 * Math.PI);

    public static void ComputeLinearModel(
        IReadOnlyDictionary<string, double> p,
        Span<double> a,
        Span<double> bVec)
    {
        double mCart = GetOrDefault(p, "M", 1.5);
        double m1 = GetOrDefault(p, "m1", 0.5);
        double m2 = GetOrDefault(p, "m2", 0.75);
        double l1 = GetOrDefault(p, "L1", 0.5);
        double l2 = GetOrDefault(p, "L2", 0.75);
        double r1 = Math.Min(l1, GetOrDefault(p, "l1", 0.25));
        double r2 = Math.Min(l2, GetOrDefault(p, "l2", 0.375));
        double j1 = GetOrDefault(p, "J1", 0.010417);
        double j2 = GetOrDefault(p, "J2", 0.035156);
        double g = GetOrDefault(p, "g", 9.81);
        double b = GetOrDefault(p, "b", 0.0);
        double c1 = GetOrDefault(p, "c1", 0.0);
        double c2 = GetOrDefault(p, "c2", 0.0);

        double d1 = mCart + m1 + m2;
        double d2 = m1 * r1 + m2 * l1;
        double d3 = m2 * r2;
        double d4 = j1 + m1 * r1 * r1 + m2 * l1 * l1;
        double d5 = m2 * l1 * r2;
        double d6 = j2 + m2 * r2 * r2;

        double a11 = d4 * d6 - d5 * d5;
        double a12 = d3 * d5 - d2 * d6;
        double a13 = d2 * d5 - d3 * d4;
        double a22 = d1 * d6 - d3 * d3;
        double a23 = d2 * d3 - d1 * d5;
        double a33 = d1 * d4 - d2 * d2;
        double det = d1 * a11 + d2 * a12 + d3 * a13;
        if (!(det > 0) || !double.IsFinite(det))
            throw new ArithmeticException("Invalid mass matrix for controller linearization.");

        double invDet = 1.0 / det;
        a.Clear();
        bVec.Clear();

        // Position derivatives: dx, dtheta1, dtheta2
        a[0 * 6 + 3] = 1.0;
        a[1 * 6 + 4] = 1.0;
        a[2 * 6 + 5] = 1.0;

        // Gravity stiffness terms (columns 1 and 2: theta1, theta2)
        double g1 = d2 * g * invDet;
        double g2 = d3 * g * invDet;
        a[3 * 6 + 1] = a12 * g1;
        a[4 * 6 + 1] = a22 * g1;
        a[5 * 6 + 1] = a23 * g1;

        a[3 * 6 + 2] = a13 * g2;
        a[4 * 6 + 2] = a23 * g2;
        a[5 * 6 + 2] = a33 * g2;

        // Viscous damping terms (columns 3, 4, 5: dx, dtheta1, dtheta2)
        a[3 * 6 + 3] = -b * a11 * invDet;
        a[4 * 6 + 3] = -b * a12 * invDet;
        a[5 * 6 + 3] = -b * a13 * invDet;

        double c12 = c1 + c2;
        a[3 * 6 + 4] = (-c12 * a12 + c2 * a13) * invDet;
        a[4 * 6 + 4] = (-c12 * a22 + c2 * a23) * invDet;
        a[5 * 6 + 4] = (-c12 * a23 + c2 * a33) * invDet;

        a[3 * 6 + 5] = (c2 * a12 - c2 * a13) * invDet;
        a[4 * 6 + 5] = (c2 * a22 - c2 * a23) * invDet;
        a[5 * 6 + 5] = (c2 * a23 - c2 * a33) * invDet;

        // Control input matrix B
        bVec[3] = a11 * invDet;
        bVec[4] = a12 * invDet;
        bVec[5] = a13 * invDet;
    }

    public static void SolveLqrGains(
        IReadOnlyDictionary<string, double> plantParameters,
        ReadOnlySpan<double> qDiag,
        double rWeight,
        Span<double> kOut)
    {
        Span<double> a = stackalloc double[36];
        Span<double> b = stackalloc double[6];
        ComputeLinearModel(plantParameters, a, b);

        // Discretize (A, B) with 4th-order Taylor expansion at h = 0.0025 s
        const double h = 0.0025;
        Span<double> a2 = stackalloc double[36];
        Span<double> a3 = stackalloc double[36];
        Span<double> a4 = stackalloc double[36];
        Multiply6x6(a, a, a2);
        Multiply6x6(a2, a, a3);
        Multiply6x6(a3, a, a4);

        Span<double> ad = stackalloc double[36];
        for (int i = 0; i < 36; i++)
        {
            ad[i] = h * a[i]
                + (0.5 * h * h) * a2[i]
                + (h * h * h / 6.0) * a3[i]
                + (h * h * h * h / 24.0) * a4[i];
        }
        for (int i = 0; i < 6; i++) ad[i * 6 + i] += 1.0;

        Span<double> ab = stackalloc double[6];
        Span<double> a2b = stackalloc double[6];
        Span<double> a3b = stackalloc double[6];
        Multiply6x1(a, b, ab);
        Multiply6x1(a2, b, a2b);
        Multiply6x1(a3, b, a3b);

        Span<double> bd = stackalloc double[6];
        for (int i = 0; i < 6; i++)
        {
            bd[i] = h * b[i]
                + (0.5 * h * h) * ab[i]
                + (h * h * h / 6.0) * a2b[i]
                + (h * h * h * h / 24.0) * a3b[i];
        }

        double rd = Math.Max(1e-6, rWeight) * h;
        Span<double> p = stackalloc double[36];
        Span<double> m = stackalloc double[36];
        Span<double> temp = stackalloc double[36];
        Span<double> pNext = stackalloc double[36];
        Span<double> pb = stackalloc double[6];
        Span<double> kNew = stackalloc double[6];

        for (int i = 0; i < 6; i++)
            p[i * 6 + i] = Math.Max(0.0, qDiag[i]);

        kOut.Clear();
        const int maxIterations = 12000;
        for (int iter = 0; iter < maxIterations; iter++)
        {
            // pb = P * bd
            for (int i = 0; i < 6; i++)
            {
                double sum = 0;
                for (int j = 0; j < 6; j++) sum += p[i * 6 + j] * bd[j];
                pb[i] = sum;
            }

            double s = rd;
            for (int i = 0; i < 6; i++) s += bd[i] * pb[i];
            double invS = 1.0 / s;

            // kNew = (bd^T * P * Ad) / s = (pb^T * Ad) / s
            double maxDiff = 0;
            for (int j = 0; j < 6; j++)
            {
                double sum = 0;
                for (int i = 0; i < 6; i++) sum += pb[i] * ad[i * 6 + j];
                double val = sum * invS;
                double diff = Math.Abs(val - kOut[j]);
                if (diff > maxDiff) maxDiff = diff;
                kNew[j] = val;
            }
            kNew.CopyTo(kOut);

            if (iter > 1000 && maxDiff < 1e-8)
                break;

            // m = P - pb * pb^T / s
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                    m[i * 6 + j] = p[i * 6 + j] - pb[i] * pb[j] * invS;

            // temp = m * Ad
            Multiply6x6(m, ad, temp);

            // pNext = Qd + Ad^T * temp
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    double sum = 0;
                    for (int k = 0; k < 6; k++) sum += ad[k * 6 + i] * temp[k * 6 + j];
                    pNext[i * 6 + j] = sum;
                }
                pNext[i * 6 + i] += Math.Max(0.0, qDiag[i]) * h;
            }

            // Enforce exact symmetry
            for (int i = 0; i < 6; i++)
                for (int j = 0; j < 6; j++)
                    p[i * 6 + j] = 0.5 * (pNext[i * 6 + j] + pNext[j * 6 + i]);
        }
    }

    private static void Multiply6x6(ReadOnlySpan<double> a, ReadOnlySpan<double> b, Span<double> result)
    {
        for (int i = 0; i < 6; i++)
        {
            for (int j = 0; j < 6; j++)
            {
                double sum = 0;
                for (int k = 0; k < 6; k++) sum += a[i * 6 + k] * b[k * 6 + j];
                result[i * 6 + j] = sum;
            }
        }
    }

    private static void Multiply6x1(ReadOnlySpan<double> a, ReadOnlySpan<double> x, Span<double> result)
    {
        for (int i = 0; i < 6; i++)
        {
            double sum = 0;
            for (int j = 0; j < 6; j++) sum += a[i * 6 + j] * x[j];
            result[i] = sum;
        }
    }

    public static double GetOrDefault(IReadOnlyDictionary<string, double> dict, string key, double fallback) =>
        dict.TryGetValue(key, out double val) && double.IsFinite(val) ? val : fallback;
}

public sealed class OpenLoopController : IController
{
    public string Name => "None (Open-Loop)";
    public string Description => "No active feedback controller. System evolves freely under manual forces and disturbances.";

    public IReadOnlyList<ParameterDescriptor> GetParameterDescriptors() => Array.Empty<ParameterDescriptor>();
    public double GetParameter(string key) => throw new ArgumentException("Open-loop controller has no parameters.", nameof(key));
    public void SetParameter(string key, double value) => throw new ArgumentException("Open-loop controller has no parameters.", nameof(key));
    public void Initialize(IReadOnlyDictionary<string, double> plantParameters) { }
    public double ComputeControlInput(in PendulumState state, in PendulumState reference, double dt) => 0.0;
    public void Reset() { }
    public void Dispose() { }
}

/// <summary>
/// Full-state Linear Quadratic Regulator (LQR) that linearizes the DIPC plant around the
/// upright equilibrium and solves the Algebraic Riccati Equation for optimal state-feedback gains.
/// </summary>
public sealed class LqrController : IController
{
    private static readonly ParameterDescriptor[] Descriptors =
    [
        new("Qx", "Cart Position Weight", "Qₓ", "", "1. State Cost Weights (Q)", 12.0, 0.1, 500.0, 0.5),
        new("QTheta1", "Lower Link Angle Weight", "Q_θ₁", "", "1. State Cost Weights (Q)", 60.0, 1.0, 2000.0, 5.0),
        new("QTheta2", "Upper Link Angle Weight", "Q_θ₂", "", "1. State Cost Weights (Q)", 120.0, 1.0, 2000.0, 5.0),
        new("QDx", "Cart Velocity Weight", "Q_dx", "", "1. State Cost Weights (Q)", 8.0, 0.0, 500.0, 0.5),
        new("QDTheta1", "Lower Link Angular Vel. Weight", "Q_dθ₁", "", "1. State Cost Weights (Q)", 6.0, 0.0, 500.0, 0.5),
        new("QDTheta2", "Upper Link Angular Vel. Weight", "Q_dθ₂", "", "1. State Cost Weights (Q)", 12.0, 0.0, 500.0, 0.5),
        new("R", "Control Effort Weight", "R", "", "2. Control Weight & Limits", 1.0, 0.01, 50.0, 0.05),
        new("MaxForce", "Saturation Limit", "u_max", "N", "2. Control Weight & Limits", 80.0, 5.0, 100.0, 1.0)
    ];

    private static readonly IReadOnlyList<ParameterDescriptor> ReadOnlyDescriptors = Array.AsReadOnly(Descriptors);
    private readonly Dictionary<string, double> _parameters = new(StringComparer.Ordinal);
    private readonly Dictionary<string, double> _plantParameters = new(DpcLinearizer.DefaultPlantParameters, StringComparer.Ordinal);
    private double _kx, _kTheta1, _kTheta2, _kDx, _kDTheta1, _kDTheta2;
    private double _maxForce = 80.0;

    public string Name => "LQR (Linear Quadratic Regulator)";
    public string Description => "Optimal full-state feedback (u = -Kz) solved from the linearized upright DIPC model.";

    public ReadOnlySpan<double> Gains => new([_kx, _kTheta1, _kTheta2, _kDx, _kDTheta1, _kDTheta2]);

    public string GainsSummary =>
        $"Solved K = [kx: {_kx:F2}, kθ₁: {_kTheta1:F1}, kθ₂: {_kTheta2:F1}, kdx: {_kDx:F2}, kdθ₁: {_kDTheta1:F1}, kdθ₂: {_kDTheta2:F1}]";

    public LqrController()
    {
        ResetParametersToDefaults();
    }

    public IReadOnlyList<ParameterDescriptor> GetParameterDescriptors() => ReadOnlyDescriptors;

    public double GetParameter(string key) => _parameters[key];

    public void SetParameter(string key, double value)
    {
        ParameterDescriptor? descriptor = null;
        foreach (var item in Descriptors)
            if (item.Key == key) { descriptor = item; break; }
        if (descriptor is null) throw new ArgumentException("Unknown LQR parameter.", nameof(key));
        if (!double.IsFinite(value) || value < descriptor.MinValue || value > descriptor.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(value));

        _parameters[key] = value;
        RecomputeGains();
    }

    public void ResetParametersToDefaults()
    {
        foreach (var item in Descriptors)
            _parameters[item.Key] = item.DefaultValue;
        RecomputeGains();
    }

    /// <summary>
    /// Auto-calculates Bryson-scaled cost weights Q, R for the given plant model and
    /// solves the Algebraic Riccati Equation for the optimal feedback gain vector K.
    /// At the default plant model, this reproduces the exact default parameters.
    /// </summary>
    public void AutoCalculateOptimalParameters(IReadOnlyDictionary<string, double> plantParameters)
    {
        ArgumentNullException.ThrowIfNull(plantParameters);
        foreach (var kvp in plantParameters)
            _plantParameters[kvp.Key] = kvp.Value;

        double mTotal = DpcLinearizer.GetOrDefault(_plantParameters, "M", 1.5)
                      + DpcLinearizer.GetOrDefault(_plantParameters, "m1", 0.5)
                      + DpcLinearizer.GetOrDefault(_plantParameters, "m2", 0.75);
        double mRef = 1.5 + 0.5 + 0.75; // 2.75 kg
        double massRatio = mTotal / mRef;

        double l1 = DpcLinearizer.GetOrDefault(_plantParameters, "L1", 0.5);
        double l2 = DpcLinearizer.GetOrDefault(_plantParameters, "L2", 0.75);
        double l1Ratio = l1 / 0.5;
        double l2Ratio = l2 / 0.75;

        SetClamped("Qx", Math.Round(12.0 * massRatio, 2));
        SetClamped("QTheta1", Math.Round(60.0 * massRatio * l1Ratio, 1));
        SetClamped("QTheta2", Math.Round(120.0 * massRatio * l2Ratio, 1));
        SetClamped("QDx", Math.Round(8.0 * massRatio, 2));
        SetClamped("QDTheta1", Math.Round(6.0 * massRatio * l1Ratio, 2));
        SetClamped("QDTheta2", Math.Round(12.0 * massRatio * l2Ratio, 2));
        SetClamped("R", 1.0);

        RecomputeGains();
    }

    private void SetClamped(string key, double value)
    {
        foreach (var d in Descriptors)
        {
            if (d.Key == key)
            {
                _parameters[key] = Math.Clamp(value, d.MinValue, d.MaxValue);
                return;
            }
        }
    }

    public void Initialize(IReadOnlyDictionary<string, double> plantParameters)
    {
        ArgumentNullException.ThrowIfNull(plantParameters);
        foreach (var kvp in plantParameters)
            _plantParameters[kvp.Key] = kvp.Value;
        RecomputeGains();
    }

    private void RecomputeGains()
    {
        Span<double> q =
        [
            _parameters["Qx"],
            _parameters["QTheta1"],
            _parameters["QTheta2"],
            _parameters["QDx"],
            _parameters["QDTheta1"],
            _parameters["QDTheta2"]
        ];
        double r = _parameters["R"];
        _maxForce = _parameters["MaxForce"];

        Span<double> k = stackalloc double[6];
        DpcLinearizer.SolveLqrGains(_plantParameters, q, r, k);
        _kx = k[0];
        _kTheta1 = k[1];
        _kTheta2 = k[2];
        _kDx = k[3];
        _kDTheta1 = k[4];
        _kDTheta2 = k[5];
    }

    public double ComputeControlInput(in PendulumState state, in PendulumState reference, double dt)
    {
        double ex = state.X - reference.X;
        double eTheta1 = DpcLinearizer.WrapAngle(state.Theta1 - reference.Theta1);
        double eTheta2 = DpcLinearizer.WrapAngle(state.Theta2 - reference.Theta2);
        double edx = state.Dx - reference.Dx;
        double edTheta1 = state.DTheta1 - reference.DTheta1;
        double edTheta2 = state.DTheta2 - reference.DTheta2;

        double u = -(
            _kx * ex +
            _kTheta1 * eTheta1 +
            _kTheta2 * eTheta2 +
            _kDx * edx +
            _kDTheta1 * edTheta1 +
            _kDTheta2 * edTheta2);

        return Math.Clamp(u, -_maxForce, _maxForce);
    }

    public void Reset() { }
    public void Dispose() { }
}

/// <summary>
/// Three-stage Cascade PID controller for DIPC upright stabilization:
/// Outer loop (Cart Position x -> Virtual Target Upper Angle θ₂_ref) ->
/// Middle loop (Upper Link θ₂ -> Virtual Target Lower Angle θ₁_ref) ->
/// Inner loop (Lower Link θ₁ -> Cart Force u).
/// </summary>
public sealed class CascadePidController : IController
{
    private readonly record struct CascadeDecomposition(
        double KpX, double KdX,
        double Kp2, double Kd2,
        double Kp1, double Kd1,
        double SignX, double Sign2, double Sign1);

    private static readonly CascadeDecomposition DefaultBaseline = ComputeOptimalCascade(DpcLinearizer.DefaultPlantParameters);

    private static readonly ParameterDescriptor[] Descriptors =
    [
        new("KpX", "Outer Cart Proportional", "Kp_x", "rad/m", "1. Outer Loop (Cart x → θ₂*)", Math.Round(DefaultBaseline.KpX, 4), 0.0001, 2.0, 0.001),
        new("KiX", "Outer Cart Integral", "Ki_x", "rad/(m·s)", "1. Outer Loop (Cart x → θ₂*)", 0.0, 0.0, 0.5, 0.0005),
        new("KdX", "Outer Cart Derivative", "Kd_x", "rad·s/m", "1. Outer Loop (Cart x → θ₂*)", Math.Round(DefaultBaseline.KdX, 4), 0.0001, 2.0, 0.001),

        new("Kp2", "Middle Link 2 Proportional", "Kp_2", "rad/rad", "2. Middle Loop (Link 2 θ₂ → θ₁*)", Math.Round(DefaultBaseline.Kp2, 3), 0.05, 20.0, 0.02),
        new("Ki2", "Middle Link 2 Integral", "Ki_2", "1/s", "2. Middle Loop (Link 2 θ₂ → θ₁*)", 0.0, 0.0, 5.0, 0.01),
        new("Kd2", "Middle Link 2 Derivative", "Kd_2", "s", "2. Middle Loop (Link 2 θ₂ → θ₁*)", Math.Round(DefaultBaseline.Kd2, 4), 0.005, 5.0, 0.005),

        new("Kp1", "Inner Link 1 Proportional", "Kp_1", "N/rad", "3. Inner Loop (Link 1 θ₁ → u)", Math.Round(DefaultBaseline.Kp1, 2), 5.0, 5000.0, 2.0),
        new("Ki1", "Inner Link 1 Integral", "Ki_1", "N/(rad·s)", "3. Inner Loop (Link 1 θ₁ → u)", 0.0, 0.0, 50.0, 0.5),
        new("Kd1", "Inner Link 1 Derivative", "Kd_1", "N·s/rad", "3. Inner Loop (Link 1 θ₁ → u)", Math.Round(DefaultBaseline.Kd1, 2), 0.5, 1000.0, 0.5),

        new("MaxForce", "Saturation Limit", "u_max", "N", "4. Output Limits", 80.0, 5.0, 100.0, 1.0)
    ];

    private static readonly IReadOnlyList<ParameterDescriptor> ReadOnlyDescriptors = Array.AsReadOnly(Descriptors);
    private readonly Dictionary<string, double> _parameters = new(StringComparer.Ordinal);
    private readonly Dictionary<string, double> _plantParameters = new(DpcLinearizer.DefaultPlantParameters, StringComparer.Ordinal);

    private double _kpX, _kiX, _kdX;
    private double _kp2, _ki2, _kd2;
    private double _kp1, _ki1, _kd1;
    private double _signX = DefaultBaseline.SignX;
    private double _sign2 = DefaultBaseline.Sign2;
    private double _sign1 = DefaultBaseline.Sign1;
    private double _maxForce = 80.0;

    private double _integralX;
    private double _integralTheta2;
    private double _integralTheta1;

    public string Name => "Cascade PID (Cart → Link 2 → Link 1)";
    public string Description => "Three-stage cascaded PID architecture: Outer Cart PID sets virtual Link 2 reference θ₂*; Middle Link 2 PID sets virtual Link 1 reference θ₁*; Inner Link 1 PID outputs cart force u.";

    public string GainsSummary =>
        $"Outer(Kp:{_kpX:F4}, Kd:{_kdX:F4}) → Mid(Kp:{_kp2:F3}, Kd:{_kd2:F4}) → Inner(Kp:{_kp1:F1}, Kd:{_kd1:F2})";

    public CascadePidController()
    {
        ResetParametersToDefaults();
    }

    public IReadOnlyList<ParameterDescriptor> GetParameterDescriptors() => ReadOnlyDescriptors;

    public double GetParameter(string key) => _parameters[key];

    public void SetParameter(string key, double value)
    {
        ParameterDescriptor? descriptor = null;
        foreach (var item in Descriptors)
            if (item.Key == key) { descriptor = item; break; }
        if (descriptor is null) throw new ArgumentException("Unknown Cascade PID parameter.", nameof(key));
        if (!double.IsFinite(value) || value < descriptor.MinValue || value > descriptor.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(value));

        _parameters[key] = value;
        CacheFromParameters();
    }

    public void ResetParametersToDefaults()
    {
        foreach (var item in Descriptors)
            _parameters[item.Key] = item.DefaultValue;
        _signX = DefaultBaseline.SignX;
        _sign2 = DefaultBaseline.Sign2;
        _sign1 = DefaultBaseline.Sign1;
        CacheFromParameters();
        Reset();
    }

    /// <summary>
    /// Linearizes the provided plant model around the upright equilibrium and computes
    /// the optimal 3-stage Cascade PID parameters (KpX, KdX, Kp2, Kd2, Kp1, Kd1),
    /// updating the controller's parameter values directly.
    /// </summary>
    public void AutoCalculateOptimalParameters(IReadOnlyDictionary<string, double> plantParameters)
    {
        ArgumentNullException.ThrowIfNull(plantParameters);
        foreach (var kvp in plantParameters)
            _plantParameters[kvp.Key] = kvp.Value;

        var optimal = ComputeOptimalCascade(_plantParameters);
        _signX = optimal.SignX;
        _sign2 = optimal.Sign2;
        _sign1 = optimal.Sign1;

        SetClamped("KpX", Math.Round(optimal.KpX, 4));
        SetClamped("KiX", 0.0);
        SetClamped("KdX", Math.Round(optimal.KdX, 4));

        SetClamped("Kp2", Math.Round(optimal.Kp2, 3));
        SetClamped("Ki2", 0.0);
        SetClamped("Kd2", Math.Round(optimal.Kd2, 4));

        SetClamped("Kp1", Math.Round(optimal.Kp1, 2));
        SetClamped("Ki1", 0.0);
        SetClamped("Kd1", Math.Round(optimal.Kd1, 2));

        CacheFromParameters();
        Reset();
    }

    private void SetClamped(string key, double value)
    {
        foreach (var d in Descriptors)
        {
            if (d.Key == key)
            {
                _parameters[key] = Math.Clamp(value, d.MinValue, d.MaxValue);
                return;
            }
        }
    }

    public void Initialize(IReadOnlyDictionary<string, double> plantParameters)
    {
        ArgumentNullException.ThrowIfNull(plantParameters);
        bool plantChanged = false;
        foreach (var kvp in plantParameters)
        {
            if (!_plantParameters.TryGetValue(kvp.Key, out double existing) || Math.Abs(existing - kvp.Value) > 1e-9)
                plantChanged = true;
            _plantParameters[kvp.Key] = kvp.Value;
        }

        if (plantChanged)
        {
            AutoCalculateOptimalParameters(_plantParameters);
        }
        else
        {
            CacheFromParameters();
            Reset();
        }
    }

    private static CascadeDecomposition ComputeOptimalCascade(IReadOnlyDictionary<string, double> plantParameters)
    {
        // Factor stabilizing state-feedback into exact 3-stage cascaded PID gains:
        // Stage 1 (Outer): theta2_ref = signX * (KpX * ex + KiX * Ix + KdX * edx)
        // Stage 2 (Middle): theta1_ref = sign2 * (Kp2 * (theta2 - theta2_ref) + Ki2 * I2 + Kd2 * dtheta2)
        // Stage 3 (Inner): u = sign1 * (Kp1 * (theta1 - theta1_ref) + Ki1 * I1 + Kd1 * dtheta1)
        ReadOnlySpan<double> qCascade = [10.0, 50.0, 100.0, 7.0, 5.0, 10.0];
        Span<double> k = stackalloc double[6];
        DpcLinearizer.SolveLqrGains(plantParameters, qCascade, 1.0, k);

        // u = -k0*ex - k1*theta1 - k2*theta2 - k3*edx - k4*dtheta1 - k5*dtheta2
        double c1 = -k[1];
        double d1 = -k[4];
        double sign1 = c1 >= 0 ? 1.0 : -1.0;
        double kp1 = Math.Abs(c1);
        double kd1 = d1 * sign1;

        // Coefficient of theta2 is -sign1 * kp1 * sign2 * kp2 == -k[2] => c1 * (sign2 * kp2) == k[2]
        double ratio2 = k[2] / c1;
        double ratioD2 = k[5] / c1;
        double sign2 = ratio2 >= 0 ? 1.0 : -1.0;
        double kp2 = Math.Abs(ratio2);
        double kd2 = ratioD2 * sign2;

        // Coefficient of ex is +sign1 * kp1 * sign2 * kp2 * signX * kpX == -k[0] => k[2] * (signX * kpX) == -k[0]
        double ratioX = -k[0] / k[2];
        double ratioDX = -k[3] / k[2];
        double signX = ratioX >= 0 ? 1.0 : -1.0;
        double kpX = Math.Abs(ratioX);
        double kdX = ratioDX * signX;

        return new CascadeDecomposition(kpX, kdX, kp2, kd2, kp1, kd1, signX, sign2, sign1);
    }

    private void CacheFromParameters()
    {
        _kpX = _parameters["KpX"];
        _kiX = _parameters["KiX"];
        _kdX = _parameters["KdX"];

        _kp2 = _parameters["Kp2"];
        _ki2 = _parameters["Ki2"];
        _kd2 = _parameters["Kd2"];

        _kp1 = _parameters["Kp1"];
        _ki1 = _parameters["Ki1"];
        _kd1 = _parameters["Kd1"];

        _maxForce = _parameters["MaxForce"];
    }

    public double ComputeControlInput(in PendulumState state, in PendulumState reference, double dt)
    {
        double ex = state.X - reference.X;
        double edx = state.Dx - reference.Dx;
        double theta1 = DpcLinearizer.WrapAngle(state.Theta1 - reference.Theta1);
        double dtheta1 = state.DTheta1 - reference.DTheta1;
        double theta2 = DpcLinearizer.WrapAngle(state.Theta2 - reference.Theta2);
        double dtheta2 = state.DTheta2 - reference.DTheta2;

        // 1. Outer Loop: Cart Position PID -> Virtual Desired Upper Link Angle (theta2_ref)
        if (_kiX > 0 && dt > 0)
            _integralX = Math.Clamp(_integralX + ex * dt, -0.5, 0.5);
        else
            _integralX = 0;

        double theta2Ref = _signX * (_kpX * ex + _kiX * _integralX + _kdX * edx);

        // 2. Middle Loop: Upper Link PID -> Virtual Desired Lower Link Angle (theta1_ref)
        double eTheta2 = theta2 - theta2Ref;
        if (_ki2 > 0 && dt > 0)
            _integralTheta2 = Math.Clamp(_integralTheta2 + eTheta2 * dt, -0.25, 0.25);
        else
            _integralTheta2 = 0;

        double theta1Ref = _sign2 * (_kp2 * eTheta2 + _ki2 * _integralTheta2 + _kd2 * dtheta2);

        // 3. Inner Loop: Lower Link PID -> Cart Force u
        double eTheta1 = theta1 - theta1Ref;
        if (_ki1 > 0 && dt > 0)
            _integralTheta1 = Math.Clamp(_integralTheta1 + eTheta1 * dt, -0.5, 0.5);
        else
            _integralTheta1 = 0;

        double u = _sign1 * (_kp1 * eTheta1 + _ki1 * _integralTheta1 + _kd1 * dtheta1);
        return Math.Clamp(u, -_maxForce, _maxForce);
    }

    public void Reset()
    {
        _integralX = 0;
        _integralTheta2 = 0;
        _integralTheta1 = 0;
    }

    public void Dispose() { }
}
