using System;
using System.Windows.Media;
using DipLabSim.Core;
using DipLabSim.Engine;

namespace DipLabSim.UI.Services;

/// <summary>WPF adapter only. Physics and engine have no dependency on WPF.</summary>
public sealed class SimulationService : IDisposable
{
    private readonly SimulationLoop _engine;
    private bool _lastRunning;
    public event Action<SimulationFrame>? FrameAvailable;
    public event Action? StateChanged;
    public PendulumState CurrentState { get; private set; }
    public bool IsRunning => _engine.IsRunning;
    public string? Fault => _engine.Fault;
    public double SpeedMultiplier { get => _engine.SpeedMultiplier; set => _engine.SpeedMultiplier = value; }
    public double ManualForce { get => _engine.ManualForce; set => _engine.ManualForce = value; }

    public SimulationService(IPlant plant, IOdeSolver solver)
    {
        _engine = new SimulationLoop(plant, solver);
        CompositionTarget.Rendering += OnRendering;
    }
    public void Configure(IPlant plant, IOdeSolver solver, double step)
    {
        Pause();
        _engine.Configure(plant, solver, step);
        Drain();
    }
    public void Start() { _engine.Start(); Notify(); }
    public void Pause() { _engine.Pause(); Drain(); Notify(); }
    public void SetManualState(double x, double theta1, double theta2)
    {
        if (IsRunning) return;
        _engine.SetState(new(CurrentState.Time, x, theta1, theta2, 0, 0, 0));
        Drain();
    }
    public void Reset(PendulumState initial)
    {
        Pause();
        _engine.SetState(initial with { Time = 0 });
        Drain();
    }
    public void Push(DisturbanceTarget target, double force, double seconds) => _engine.Push(target, force, seconds);
    private void OnRendering(object? sender, EventArgs e)
    {
        Drain();
        if (_lastRunning != IsRunning) Notify();
    }
    private void Drain()
    {
        bool found = false;
        SimulationFrame latest = default;
        while (_engine.Frames.TryRead(out var frame)) { latest = frame; found = true; }
        if (!found) return;
        CurrentState = latest.State;
        FrameAvailable?.Invoke(latest);
    }
    private void Notify() { _lastRunning = IsRunning; StateChanged?.Invoke(); }
    public void Dispose()
    {
        CompositionTarget.Rendering -= OnRendering;
        _engine.Dispose();
    }
}
