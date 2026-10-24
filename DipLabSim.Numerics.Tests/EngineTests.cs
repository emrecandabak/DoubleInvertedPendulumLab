using DipLabSim.Core;
using DipLabSim.Engine;
using DipLabSim.Physics;
using Xunit;

namespace DipLabSim.Numerics.Tests;

public class EngineTests
{
    [Theory]
    [InlineData(DisturbanceTarget.LowerLinkTip)]
    [InlineData(DisturbanceTarget.UpperLinkTip)]
    public void PulseUsesSelectedTargetAndExpiresAtFixedStepBoundary(DisturbanceTarget target)
    {
        var p = new NonlinearEulerLagrangePlant();
        using var loop = new SimulationLoop(p, new Rk4Solver());
        loop.Push(target, 2, 0.0025); // rounds to 3 x 1ms
        for (int i = 0; i < 5; i++)
        {
            loop.StepOnce();
            Assert.True(loop.Frames.TryRead(out var frame));
            Assert.Equal(i < 3 ? 2 : 0, frame.DisturbanceForce);
            Assert.Equal(target, frame.DisturbanceTarget);
        }
        Assert.Equal(target, p.DisturbanceTarget);
        Assert.InRange(Math.Abs(loop.PausedState.Time - 0.005), 0, 1e-12);
        Assert.True(Math.Abs(loop.PausedState.Theta1) > 0);
    }
    [Fact]
    public void ResetClearsQueuedPulseAndCartForce()
    {
        using var loop = new SimulationLoop(new NonlinearEulerLagrangePlant(), new Rk4Solver());
        loop.ManualForce = 3;
        loop.Push(DisturbanceTarget.UpperLinkTip, 5, 1);
        loop.SetState(PendulumState.Zero);
        while (loop.Frames.TryRead(out _)) { }
        loop.StepOnce();
        Assert.True(loop.Frames.TryRead(out var frame));
        Assert.Equal(0, frame.ControlForceU);
        Assert.Equal(0, frame.DisturbanceForce);
        Assert.Equal(PendulumState.Zero with { Time = 0.001 }, frame.State);
    }
    [Fact]
    public void ChangingSpeedDoesNotChangeNumericalStep()
    {
        using var loop = new SimulationLoop(new NonlinearEulerLagrangePlant(), new Rk4Solver());
        loop.SpeedMultiplier = 2;
        loop.StepOnce();
        Assert.Equal(0.001, loop.PausedState.Time);
    }
    [Fact]
    public void WorkerPausesWithoutFurtherWritesAndCanResume()
    {
        using var loop = new SimulationLoop(new NonlinearEulerLagrangePlant(), new Rk4Solver());
        loop.SetState(PendulumState.Zero with { Theta1 = 0.05 });
        while (loop.Frames.TryRead(out _)) { }
        loop.Start();
        Assert.True(SpinWait.SpinUntil(() => loop.Frames.TryPeek(out _), 3000));
        Assert.Throws<InvalidOperationException>(() => loop.SetState(PendulumState.Zero));
        loop.Pause();
        var paused = loop.PausedState;
        Assert.True(paused.Time > 0);
        Assert.Null(loop.Fault);
        while (loop.Frames.TryRead(out _)) { }
        loop.Start();
        Assert.True(SpinWait.SpinUntil(() => loop.Frames.TryPeek(out _), 3000));
        loop.Pause();
        Assert.True(loop.PausedState.Time > paused.Time);
    }
}
