using DipLabSim.Core;

namespace DipLabSim.Physics;

/// <summary>
/// Hybrid swing-up / balance controller (Swing-Up Switching Design Report). Each tick it is in one
/// <see cref="SupervisorMode"/>:
/// <list type="number">
/// <item>SwingUp: the swing-up law acts. When V(z) = eᵀPe &lt; α·c_sat (Lyapunov pre-gate, P from the
/// LQR Riccati solve) and the last capture test is at least 10 ms old, the mode becomes Capture.</item>
/// <item>Capture: the swing-up law still acts while a predictive capture test simulates the saturated
/// LQR on a private Euler–Lagrange plant with RK4 (h = 5 ms, horizon Th). Pass → Balance, fail → SwingUp.</item>
/// <item>Balance: the inner LQR acts. If V &gt; β·c_sat or |θᵢ| &gt; π/2 for the dwell time, the mode
/// returns to SwingUp.</item>
/// </list>
/// All buffers are allocated in <see cref="Initialize"/>; <see cref="ComputeControlInput"/> does not
/// allocate. The swing-up stage is any <see cref="IController"/>; the supervisor owns and disposes it.
/// </summary>
public sealed class SwingUpSupervisorController : IController
{
    private const string Category = "Swing-Up Supervisor";

    // β's range starts above α's, so the exit level is always above the entry level (hysteresis).
    private static readonly ParameterDescriptor[] Descriptors =
    [
        new("Alpha", "Pre-Gate Factor (V < α·c_sat)", "α", "", Category, 5.0, 1.0, 9.0, 0.1),
        new("Beta", "Fallback Factor (V > β·c_sat)", "β", "", Category, 10.0, 10.0, 50.0, 0.5),
        new("HorizonTh", "Capture Test Horizon", "T_h", "s", Category, 1.5, 0.25, 5.0, 0.05),
        new("Rho", "Required V Decay over T_h", "ρ", "", Category, 0.3, 0.01, 0.99, 0.01),
        new("DwellTime", "Fallback Dwell Time", "t_dwell", "s", Category, 0.05, 0.0, 0.5, 0.005),
        new("MaxForce", "Saturation Limit", "u_max", "N", Category, 80.0, 5.0, 100.0, 1.0)
    ];

    private static readonly IReadOnlyList<ParameterDescriptor> ReadOnlyDescriptors = Array.AsReadOnly(Descriptors);

    /// <summary>Shadow-simulation RK4 step (s).</summary>
    public const double ShadowStep = 0.005;
    /// <summary>Minimum time between two capture tests (s).</summary>
    public const double CaptureTestInterval = 0.01;
    /// <summary>Rail end used to clamp the cart in the shadow simulation (m). Matches the engine rail.</summary>
    public const double ShadowRailLimit = 1.8;
    /// <summary>The predicted cart position must stay below this (m), leaving a margin to the rail end.</summary>
    public const double CartMarginLimit = 1.4;
    /// <summary>Angle bound for the capture test and the fallback check (rad).</summary>
    public const double AngleLimit = Math.PI / 2;
    // Absolute convergence floor so that a start at exactly upright (V(0) = 0) still passes.
    private const double ConvergedFloor = 1e-6;

    private readonly Dictionary<string, double> _parameters = new(StringComparer.Ordinal);
    private readonly Dictionary<string, double> _plantParameters = new(DpcLinearizer.DefaultPlantParameters, StringComparer.Ordinal);
    private readonly LqrController _lqr = new();
    private readonly IController _swingUp;
    private readonly NonlinearEulerLagrangePlant _shadowPlant = new();
    private readonly Rk4Solver _shadowSolver = new();

    // Allocated in Initialize.
    private double[] _p = null!;
    private double[] _k = null!;
    private double[] _work36 = null!;
    private double[] _work6 = null!;

    private double _alpha, _beta, _horizon, _rho, _dwell, _maxForce;
    private int _horizonSteps;
    private double _cSat;

    private SupervisorMode _mode;
    private double _lastV;
    private double _timeSinceTest;
    private double _violationTime;
    private int _captureTestCount;

    public SwingUpSupervisorController() : this(new EnergyShapingSwingUpController()) { }

    public SwingUpSupervisorController(IController swingUpController)
    {
        ArgumentNullException.ThrowIfNull(swingUpController);
        _swingUp = swingUpController;
        foreach (var item in Descriptors)
            _parameters[item.Key] = item.DefaultValue;
        CacheParameters();
        ForwardMaxForce();
        Initialize(DpcLinearizer.DefaultPlantParameters);
        Reset();
    }

    public string Name => "Swing-Up Supervisor (Energy PFL → LQR)";
    public string Description =>
        "Hybrid controller: swing-up law until a Lyapunov pre-gate and a 1.5 s predictive capture test accept the hand-over to LQR; hysteretic fallback to swing-up.";

    public SupervisorMode Mode => _mode;
    /// <summary>Largest level c_sat = u_max² / (K P⁻¹ Kᵀ) inside which the LQR command stays unsaturated.</summary>
    public double SaturationLevel => _cSat;
    /// <summary>V(z) evaluated on the most recent tick.</summary>
    public double LastLyapunovValue => _lastV;
    public double LyapunovRatio => _cSat > 0 ? _lastV / _cSat : double.PositiveInfinity;
    /// <summary>Number of predictive capture tests run since the last <see cref="Reset"/>.</summary>
    public int CaptureTestCount => _captureTestCount;
    public LqrController BalanceController => _lqr;
    public IController SwingUpController => _swingUp;

    public string GainsSummary =>
        $"Mode: {_mode} | V/c_sat = {LyapunovRatio:F2} | c_sat = {_cSat:F1} | K = [{_k[0]:F2}, {_k[1]:F1}, {_k[2]:F1}, {_k[3]:F2}, {_k[4]:F1}, {_k[5]:F1}]";

    public IReadOnlyList<ParameterDescriptor> GetParameterDescriptors() => ReadOnlyDescriptors;

    public double GetParameter(string key) => _parameters[key];

    public void SetParameter(string key, double value)
    {
        ParameterDescriptor? descriptor = null;
        foreach (var item in Descriptors)
            if (item.Key == key) { descriptor = item; break; }
        if (descriptor is null) throw new ArgumentException("Unknown supervisor parameter.", nameof(key));
        if (!double.IsFinite(value) || value < descriptor.MinValue || value > descriptor.MaxValue)
            throw new ArgumentOutOfRangeException(nameof(value));

        _parameters[key] = value;
        CacheParameters();
        if (key == "MaxForce")
        {
            ForwardMaxForce();
            RecomputeSaturationLevel();
        }
    }

    public void ResetParametersToDefaults()
    {
        foreach (var item in Descriptors)
            _parameters[item.Key] = item.DefaultValue;
        CacheParameters();
        _lqr.ResetParametersToDefaults();
        ForwardMaxForce();
        RecomputeLyapunovFunction();
    }

    /// <summary>Re-tunes the inner LQR weights for the plant, then rebuilds P and c_sat.</summary>
    public void AutoCalculateOptimalParameters(IReadOnlyDictionary<string, double> plantParameters)
    {
        ArgumentNullException.ThrowIfNull(plantParameters);
        _lqr.AutoCalculateOptimalParameters(plantParameters);
        Initialize(plantParameters);
    }

    public void Initialize(IReadOnlyDictionary<string, double> plantParameters)
    {
        ArgumentNullException.ThrowIfNull(plantParameters);
        foreach (var kvp in plantParameters)
            _plantParameters[kvp.Key] = kvp.Value;

        if (_p is null)
        {
            _p = new double[36];
            _k = new double[6];
            _work36 = new double[36];
            _work6 = new double[6];
        }

        ConfigureShadowPlant();
        _lqr.Initialize(_plantParameters);
        _swingUp.Initialize(_plantParameters);
        RecomputeLyapunovFunction();
    }

    public void Reset()
    {
        _mode = SupervisorMode.SwingUp;
        _lastV = 0;
        _timeSinceTest = double.PositiveInfinity;
        _violationTime = 0;
        _captureTestCount = 0;
        _swingUp.Reset();
        _lqr.Reset();
    }

    public double ComputeControlInput(in PendulumState state, in PendulumState reference, double dt)
    {
        double v = Lyapunov(state, reference);
        _lastV = v;
        _timeSinceTest += dt;

        switch (_mode)
        {
            case SupervisorMode.SwingUp:
                // Layer 1: cheap pre-gate, rate-limited so a rejected state is not re-tested every tick.
                if (v < _alpha * _cSat && _timeSinceTest >= CaptureTestInterval)
                    _mode = SupervisorMode.Capture;
                return _swingUp.ComputeControlInput(state, reference, dt);

            case SupervisorMode.Capture:
                // Layer 2: predictive capture test on the private nonlinear model.
                _timeSinceTest = 0;
                if (RunCaptureTest(state, reference))
                {
                    _mode = SupervisorMode.Balance;
                    _violationTime = 0;
                    _lqr.Reset();
                    return _lqr.ComputeControlInput(state, reference, dt);
                }
                _mode = SupervisorMode.SwingUp;
                return _swingUp.ComputeControlInput(state, reference, dt);

            case SupervisorMode.Balance:
                // Layer 3: hysteretic fallback with dwell time.
                bool violated = v > _beta * _cSat
                    || Math.Abs(DpcLinearizer.WrapAngle(state.Theta1 - reference.Theta1)) > AngleLimit
                    || Math.Abs(DpcLinearizer.WrapAngle(state.Theta2 - reference.Theta2)) > AngleLimit;
                _violationTime = violated ? _violationTime + dt : 0;
                if (violated && _violationTime >= _dwell - 1e-12)
                {
                    _mode = SupervisorMode.SwingUp;
                    _violationTime = 0;
                    _timeSinceTest = 0;
                    _swingUp.Reset();
                    return _swingUp.ComputeControlInput(state, reference, dt);
                }
                return _lqr.ComputeControlInput(state, reference, dt);

            default:
                return 0.0;
        }
    }

    /// <summary>V(z) = eᵀPe with e = z − z_ref and angle errors wrapped to [−π, π].</summary>
    public double Lyapunov(in PendulumState state, in PendulumState reference)
    {
        Span<double> e = stackalloc double[6];
        e[0] = state.X - reference.X;
        e[1] = DpcLinearizer.WrapAngle(state.Theta1 - reference.Theta1);
        e[2] = DpcLinearizer.WrapAngle(state.Theta2 - reference.Theta2);
        e[3] = state.Dx - reference.Dx;
        e[4] = state.DTheta1 - reference.DTheta1;
        e[5] = state.DTheta2 - reference.DTheta2;

        double v = 0;
        for (int i = 0; i < 6; i++)
        {
            double row = 0;
            for (int j = 0; j < 6; j++) row += _p[i * 6 + j] * e[j];
            v += e[i] * row;
        }
        return v;
    }

    /// <summary>
    /// Layer 2: simulates the saturated LQR from <paramref name="state"/> for T_h on the private plant
    /// (RK4, h = 5 ms, cart clamped at ±1.8 m). Accepts only if |θᵢ| stays below π/2, |x| below 1.4 m,
    /// and V(z(T_h)) &lt; ρ·V(z(0)). Stops at the first violation.
    /// </summary>
    public bool RunCaptureTest(in PendulumState state, in PendulumState reference)
    {
        _captureTestCount++;
        double v0 = Lyapunov(state, reference);
        var s = state with { Time = 0 };
        if (!WithinCaptureBounds(s, reference)) return false;

        for (int i = 0; i < _horizonSteps; i++)
        {
            double u = _lqr.ComputeControlInput(s, reference, ShadowStep);
            s = _shadowSolver.Step(_shadowPlant, s, u, 0.0, ShadowStep);
            if (s.X >= ShadowRailLimit) s = s with { X = ShadowRailLimit, Dx = Math.Min(0.0, s.Dx) };
            else if (s.X <= -ShadowRailLimit) s = s with { X = -ShadowRailLimit, Dx = Math.Max(0.0, s.Dx) };
            if (!WithinCaptureBounds(s, reference)) return false;
        }

        return Lyapunov(s, reference) < _rho * v0 + ConvergedFloor;
    }

    private static bool WithinCaptureBounds(in PendulumState s, in PendulumState reference) =>
        double.IsFinite(s.X) && double.IsFinite(s.Dx)
        && double.IsFinite(s.DTheta1) && double.IsFinite(s.DTheta2)
        && Math.Abs(s.X) < CartMarginLimit
        && Math.Abs(DpcLinearizer.WrapAngle(s.Theta1 - reference.Theta1)) < AngleLimit
        && Math.Abs(DpcLinearizer.WrapAngle(s.Theta2 - reference.Theta2)) < AngleLimit;

    public void Dispose()
    {
        _swingUp.Dispose();
        _lqr.Dispose();
    }

    private void CacheParameters()
    {
        _alpha = _parameters["Alpha"];
        _beta = _parameters["Beta"];
        _horizon = _parameters["HorizonTh"];
        _rho = _parameters["Rho"];
        _dwell = _parameters["DwellTime"];
        _maxForce = _parameters["MaxForce"];
        _horizonSteps = Math.Max(1, (int)Math.Round(_horizon / ShadowStep));
    }

    private void ForwardMaxForce()
    {
        _lqr.SetParameter("MaxForce", _maxForce);
        foreach (var d in _swingUp.GetParameterDescriptors())
        {
            if (d.Key == "MaxForce")
            {
                _swingUp.SetParameter("MaxForce", Math.Clamp(_maxForce, d.MinValue, d.MaxValue));
                break;
            }
        }
    }

    // Copies the plant parameters into the shadow plant; L1/L2 before l1/l2 so COM <= length holds.
    private void ConfigureShadowPlant()
    {
        foreach (var d in _shadowPlant.GetParameterDescriptors())
            if (d.Key != "l1" && d.Key != "l2" && _plantParameters.TryGetValue(d.Key, out double value))
                _shadowPlant.SetParameter(d.Key, value);
        if (_plantParameters.TryGetValue("l1", out double l1)) _shadowPlant.SetParameter("l1", l1);
        if (_plantParameters.TryGetValue("l2", out double l2)) _shadowPlant.SetParameter("l2", l2);
    }

    // Same Q, R and plant as the inner LQR, so K here equals the LQR's gains and P is its Riccati matrix.
    private void RecomputeLyapunovFunction()
    {
        Span<double> q =
        [
            _lqr.GetParameter("Qx"),
            _lqr.GetParameter("QTheta1"),
            _lqr.GetParameter("QTheta2"),
            _lqr.GetParameter("QDx"),
            _lqr.GetParameter("QDTheta1"),
            _lqr.GetParameter("QDTheta2")
        ];
        DpcLinearizer.SolveLqrGains(_plantParameters, q, _lqr.GetParameter("R"), _k, _p);
        RecomputeSaturationLevel();
    }

    // c_sat = u_max² / (K P⁻¹ Kᵀ).
    private void RecomputeSaturationLevel()
    {
        _p.CopyTo(_work36, 0);
        _k.CopyTo(_work6, 0);
        SmallLinearSystem.Solve(_work36, _work6, 6);
        double kPinvK = 0;
        for (int i = 0; i < 6; i++) kPinvK += _k[i] * _work6[i];
        _cSat = kPinvK > 0 && double.IsFinite(kPinvK) ? _maxForce * _maxForce / kPinvK : 0.0;
    }
}
