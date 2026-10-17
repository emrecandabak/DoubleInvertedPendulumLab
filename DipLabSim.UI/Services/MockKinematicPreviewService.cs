using System;
using System.Diagnostics;
using System.Windows.Threading;
using DipLabSim.Core;

namespace DipLabSim.UI.Services;

public sealed class MockKinematicPreviewService : IDisposable
{
    private readonly DispatcherTimer _timer;
    private readonly Stopwatch _stopwatch = new();
    private double _simulatedTime = 0.0;
    private double _speedMultiplier = 1.0;
    private bool _isRunning = false;

    private PendulumState _currentState = PendulumState.Zero;
    private double _controlForceU = 0.0;

    public event Action<SimulationFrame>? FrameAvailable;
    public event Action? StateChanged;

    public bool IsRunning => _isRunning;

    public double SpeedMultiplier
    {
        get => _speedMultiplier;
        set => _speedMultiplier = Math.Clamp(value, 0.1, 10.0);
    }

    public PendulumState CurrentState => _currentState;
    public double ControlForceU => _controlForceU;
    public double SimulatedTime => _simulatedTime;

    public MockKinematicPreviewService()
    {
        _timer = new DispatcherTimer(DispatcherPriority.Render)
        {
            Interval = TimeSpan.FromMilliseconds(16) // ~60 Hz UI preview tick
        };
        _timer.Tick += OnTimerTick;
    }

    public void Start()
    {
        if (!_isRunning)
        {
            _isRunning = true;
            _stopwatch.Restart();
            _timer.Start();
            StateChanged?.Invoke();
        }
    }

    public void Pause()
    {
        if (_isRunning)
        {
            _isRunning = false;
            _timer.Stop();
            _stopwatch.Stop();
            StateChanged?.Invoke();
        }
    }

    public void TogglePlayPause()
    {
        if (_isRunning) Pause();
        else Start();
    }

    public void Reset()
    {
        Pause();
        _simulatedTime = 0.0;
        _currentState = PendulumState.Zero;
        _controlForceU = 0.0;
        StateChanged?.Invoke();
        PublishCurrentFrame();
    }

    public void SetManualState(double x, double theta1, double theta2)
    {
        // When user scrubs manual state sliders
        _currentState = new PendulumState(
            Time: _simulatedTime,
            X: x,
            Theta1: theta1,
            Theta2: theta2,
            Dx: 0,
            DTheta1: 0,
            DTheta2: 0);

        _controlForceU = 0.0;
        StateChanged?.Invoke();
        PublishCurrentFrame();
    }

    private void OnTimerTick(object? sender, EventArgs e)
    {
        if (!_isRunning) return;

        double dt = 0.016 * _speedMultiplier;
        _simulatedTime += dt;
        double t = _simulatedTime;

        // Kinematic multi-harmonic trajectory for realistic double pendulum preview
        // Cart position
        double x = 0.45 * Math.Sin(1.1 * t) + 0.15 * Math.Cos(2.4 * t);
        double dx = 0.45 * 1.1 * Math.Cos(1.1 * t) - 0.15 * 2.4 * Math.Sin(2.4 * t);

        // Link 1 angle (around upright 0)
        double theta1 = 0.28 * Math.Sin(1.7 * t) + 0.09 * Math.Sin(3.5 * t);
        double dtheta1 = 0.28 * 1.7 * Math.Cos(1.7 * t) + 0.09 * 3.5 * Math.Cos(3.5 * t);

        // Link 2 angle (around upright 0, coupled motion)
        double theta2 = -0.38 * Math.Sin(2.1 * t + 0.6) + 0.14 * Math.Cos(4.2 * t);
        double dtheta2 = -0.38 * 2.1 * Math.Cos(2.1 * t + 0.6) - 0.14 * 4.2 * Math.Sin(4.2 * t);

        // Control force representation
        _controlForceU = 18.0 * Math.Sin(1.7 * t) - 7.5 * Math.Cos(2.1 * t + 0.6);

        _currentState = new PendulumState(
            Time: _simulatedTime,
            X: x,
            Theta1: theta1,
            Theta2: theta2,
            Dx: dx,
            DTheta1: dtheta1,
            DTheta2: dtheta2);

        PublishCurrentFrame();
    }

    private void PublishCurrentFrame()
    {
        var frame = new SimulationFrame(
            State: _currentState,
            Reference: PendulumState.Zero,
            ControlForceU: _controlForceU,
            DisturbanceForce: 0.0);

        FrameAvailable?.Invoke(frame);
    }

    public void Dispose()
    {
        _timer.Stop();
        _timer.Tick -= OnTimerTick;
    }
}
