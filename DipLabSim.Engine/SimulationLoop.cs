using System.Diagnostics;
using System.Threading.Channels;
using DipLabSim.Core;

namespace DipLabSim.Engine;

/// <summary>Background physics with fixed simulation steps. UI consumes a bounded
/// latest-frame channel; rendering cadence never determines numerical step size.
/// Configure/parameter/state edits require Pause(), which joins the worker first.</summary>
public sealed class SimulationLoop : IDisposable
{
    private readonly Channel<SimulationFrame> _frames = Channel.CreateBounded<SimulationFrame>(
        new BoundedChannelOptions(256) { SingleReader = true, SingleWriter = false, FullMode = BoundedChannelFullMode.DropOldest });
    private readonly object _inputGate = new();
    private Thread? _worker;
    private volatile bool _running;
    private double _speed = 1, _manualForce;
    private double _pendingPulseForce, _pendingPulseDuration;
    private DisturbanceTarget _pendingPulseTarget;
    private bool _pulsePending;
    private double _pulseForce, _pulseUntil;
    private DisturbanceTarget _pulseTarget;
    private IPlant _plant;
    private IOdeSolver _solver;
    private PendulumState _state;
    private double _dt = 0.001;
    private string? _fault;
    private bool _disposed;

    public SimulationLoop(IPlant plant, IOdeSolver solver) { _plant = plant; _solver = solver; }
    public ChannelReader<SimulationFrame> Frames => _frames.Reader;
    public bool IsRunning => _running;
    public PendulumState PausedState { get { EnsurePaused(); return _state; } }
    public string? Fault => Volatile.Read(ref _fault);
    public double TimeStep => _dt;
    public double SpeedMultiplier
    {
        get => Volatile.Read(ref _speed);
        set
        {
            if (!double.IsFinite(value) || value < 0.1 || value > 10) throw new ArgumentOutOfRangeException(nameof(value));
            Volatile.Write(ref _speed, value);
        }
    }
    public double ManualForce
    {
        get => Volatile.Read(ref _manualForce);
        set
        {
            if (!double.IsFinite(value) || Math.Abs(value) > 100) throw new ArgumentOutOfRangeException(nameof(value));
            Volatile.Write(ref _manualForce, value);
        }
    }

    public void Configure(IPlant plant, IOdeSolver solver, double dt)
    {
        EnsurePaused();
        if (!double.IsFinite(dt) || dt < 0.0001 || dt > 0.01) throw new ArgumentOutOfRangeException(nameof(dt));
        _plant = plant; _solver = solver; _dt = dt;
        ClearInputs();
        Volatile.Write(ref _fault, null);
        Publish(0, 0);
    }

    // A finite rectangular force pulse (N, seconds), not an instantaneous angle edit.
    // A new pulse replaces the active pulse at the next physics step.
    public void Push(DisturbanceTarget target, double force, double duration)
    {
        if (!double.IsFinite(force) || Math.Abs(force) > 100 || !double.IsFinite(duration) || duration <= 0 || duration > 5)
            throw new ArgumentOutOfRangeException(nameof(force));
        if (!Enum.IsDefined(target)) throw new ArgumentOutOfRangeException(nameof(target));
        lock (_inputGate)
        {
            _pendingPulseTarget = target; _pendingPulseForce = force;
            _pendingPulseDuration = duration; _pulsePending = true;
        }
    }

    public void SetState(in PendulumState state)
    {
        EnsurePaused();
        if (!IsFinite(state)) throw new ArgumentOutOfRangeException(nameof(state));
        _state = state;
        ClearInputs();
        Volatile.Write(ref _fault, null);
        Publish(0, 0);
    }

    public void StepOnce()
    {
        EnsurePaused();
        AdvanceOneStep();
    }

    public void Start()
    {
        ObjectDisposedException.ThrowIf(_disposed, this);
        if (_running) return;
        _worker?.Join();
        if (Fault != null) return; // reset or reconfigure after a numerical fault
        _running = true;
        _worker = new Thread(Run) { IsBackground = true, Name = "DipLabSim physics" };
        _worker.Start();
    }

    public void Pause()
    {
        _running = false;
        _worker?.Join();
        _worker = null;
    }

    private void Run()
    {
        long previous = Stopwatch.GetTimestamp();
        double accumulator = 0;
        try
        {
            while (_running)
            {
                long now = Stopwatch.GetTimestamp();
                // Cap wall-clock catch-up after OS suspension; simulation time stays fixed-step.
                accumulator += Math.Min(0.05, (double)(now - previous) / Stopwatch.Frequency) * SpeedMultiplier;
                previous = now;
                int batch = 0;
                while (_running && accumulator >= _dt && batch++ < 200)
                {
                    AdvanceOneStep();
                    accumulator -= _dt;
                }
                // This is a desktop simulator, not a hard real-time deadline guarantee.
                Thread.Sleep(1);
            }
        }
        catch (Exception ex)
        {
            Volatile.Write(ref _fault, ex.Message);
        }
        finally { _running = false; }
    }

    private void AdvanceOneStep()
    {
        lock (_inputGate)
        {
            if (_pulsePending)
            {
                _pulseTarget = _pendingPulseTarget; _pulseForce = _pendingPulseForce;
                // Quantize duration upward to whole fixed steps, avoiding fractional-step ambiguity.
                _pulseUntil = _state.Time + Math.Ceiling(_pendingPulseDuration / _dt) * _dt;
                _pulsePending = false;
            }
        }
        double force = _state.Time < _pulseUntil - _dt * 1e-6 ? _pulseForce : 0;
        if (_plant is ITargetedDisturbancePlant targeted) targeted.DisturbanceTarget = _pulseTarget;
        double u = ManualForce;
        var next = _solver.Step(_plant, _state, u, force, _dt);
        if (!IsFinite(next) || Math.Abs(next.X) > 100 || Math.Abs(next.Dx) > 1000
            || Math.Abs(next.DTheta1) > 10000 || Math.Abs(next.DTheta2) > 10000)
            throw new ArithmeticException("Simulation exceeded numerical/display limits. Reduce step or force, then reset.");
        _state = next;
        Publish(u, force);
    }

    private void Publish(double u, double disturbance) => _frames.Writer.TryWrite(
        new SimulationFrame(_state, PendulumState.Zero, u, disturbance) { DisturbanceTarget = _pulseTarget });

    private void ClearInputs()
    {
        ManualForce = 0;
        lock (_inputGate) { _pulsePending = false; _pulseForce = 0; _pulseUntil = 0; }
    }
    private void EnsurePaused()
    {
        if (_running || _worker is { IsAlive: true }) throw new InvalidOperationException("Pause before editing simulation configuration.");
    }
    private static bool IsFinite(in PendulumState s) => double.IsFinite(s.Time) && double.IsFinite(s.X)
        && double.IsFinite(s.Theta1) && double.IsFinite(s.Theta2) && double.IsFinite(s.Dx)
        && double.IsFinite(s.DTheta1) && double.IsFinite(s.DTheta2);
    public void Dispose() { if (_disposed) return; Pause(); _disposed = true; _frames.Writer.TryComplete(); }
}
