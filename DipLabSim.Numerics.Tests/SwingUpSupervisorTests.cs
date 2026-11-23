using DipLabSim.Core;
using DipLabSim.Engine;
using DipLabSim.Physics;
using Xunit;

namespace DipLabSim.Numerics.Tests;

/// <summary>The four supervisor tests from the Swing-Up Switching Design Report (Table 2).</summary>
public class SwingUpSupervisorTests
{
    private const double FixedAngleThreshold = 0.3;
    private static readonly PendulumState Reference = PendulumState.Zero;

    // Near upright and inside the fixed-angle threshold, but moving fast. Inside the α pre-gate,
    // so the predictive layer is what has to reject it.
    private static readonly PendulumState FastUncatchableState = new(0, 0.0, 0.1, 0.15, 1.0, 2.5, -0.5);

    // At rest, both angles above the fixed threshold, and V > c_sat (the Lyapunov gate alone
    // would reject it), yet the saturated LQR catches it.
    private static readonly PendulumState SlowCatchableState = new(0, -0.3, 0.32, 0.36, 0, 0, 0);

    // Report ground truth: 10 s closed-loop LQR with RK4 at 2 ms, success if V < 0.5 at the end
    // and the rail end was never reached.
    private static bool LqrCatches(in PendulumState start, SwingUpSupervisorController lyapunov)
    {
        var plant = new NonlinearEulerLagrangePlant();
        var lqr = new LqrController();
        var solver = new Rk4Solver();
        var s = start;
        for (int i = 0; i < 5000; i++)
        {
            s = solver.Step(plant, s, lqr.ComputeControlInput(s, Reference, 0.002), 0, 0.002);
            if (!double.IsFinite(s.X) || Math.Abs(s.X) >= SwingUpSupervisorController.ShadowRailLimit)
                return false;
        }
        return lyapunov.Lyapunov(s, Reference) < 0.5;
    }

    [Fact]
    public void DoesNotSwitchFromFastNearUprightStateThatLqrCannotCatch()
    {
        var supervisor = new SwingUpSupervisorController();
        var state = FastUncatchableState;
        double ratio = supervisor.Lyapunov(state, Reference) / supervisor.SaturationLevel;

        // A fixed angle rule would switch here, and the pre-gate lets it through to Layer 2.
        Assert.True(Math.Abs(state.Theta1) < FixedAngleThreshold && Math.Abs(state.Theta2) < FixedAngleThreshold);
        Assert.InRange(ratio, 0, supervisor.GetParameter("Alpha"));
        Assert.False(LqrCatches(state, supervisor));

        // Hold the state for 30 ms of 1 ms ticks: the capture test runs and keeps rejecting it.
        for (int i = 0; i < 30; i++)
        {
            supervisor.ComputeControlInput(state, Reference, 0.001);
            Assert.NotEqual(SupervisorMode.Balance, supervisor.Mode);
        }
        // Rate-limited to one test per 10 ms.
        Assert.InRange(supervisor.CaptureTestCount, 2, 4);
    }

    [Fact]
    public void SwitchesFromSlowStateOutsideAngleThresholdAndStabilizes()
    {
        var plant = new NonlinearEulerLagrangePlant();
        var supervisor = new SwingUpSupervisorController();
        var state = SlowCatchableState;
        double ratio = supervisor.Lyapunov(state, Reference) / supervisor.SaturationLevel;

        Assert.True(Math.Abs(state.Theta1) > FixedAngleThreshold && Math.Abs(state.Theta2) > FixedAngleThreshold);
        Assert.InRange(ratio, 1.0, supervisor.GetParameter("Alpha"));
        Assert.True(LqrCatches(state, supervisor));

        using var loop = new SimulationLoop(plant, new Rk4Solver(), supervisor);
        loop.SetState(state);
        for (int i = 0; i < 5; i++) loop.StepOnce();
        Assert.Equal(SupervisorMode.Balance, supervisor.Mode);

        for (int i = 0; i < 10000; i++)
        {
            loop.StepOnce();
            Assert.Equal(SupervisorMode.Balance, supervisor.Mode);
        }
        var final = loop.PausedState;
        Assert.InRange(Math.Abs(final.X), 0, 0.02);
        Assert.InRange(Math.Abs(final.Theta1), 0, 0.01);
        Assert.InRange(Math.Abs(final.Theta2), 0, 0.01);
    }

    [Fact]
    public void FallsBackToSwingUpAfterLargeTipPushButNotAfterSmallOne()
    {
        var plant = new NonlinearEulerLagrangePlant();
        var supervisor = new SwingUpSupervisorController();
        using var loop = new SimulationLoop(plant, new Rk4Solver(), supervisor);
        loop.SetState(PendulumState.Zero);
        for (int i = 0; i < 500; i++) loop.StepOnce();
        Assert.Equal(SupervisorMode.Balance, supervisor.Mode);

        // Small push: rejected by the LQR, no mode change.
        loop.Push(DisturbanceTarget.UpperLinkTip, 2.0, 0.08);
        for (int i = 0; i < 3000; i++)
        {
            loop.StepOnce();
            Assert.Equal(SupervisorMode.Balance, supervisor.Mode);
        }

        // Large push: falls back to SwingUp, but only after the dwell time.
        loop.Push(DisturbanceTarget.UpperLinkTip, 40.0, 0.2);
        double beta = supervisor.GetParameter("Beta");
        int firstViolation = -1, fallback = -1;
        for (int i = 0; i < 3000 && fallback < 0; i++)
        {
            var s = loop.PausedState;
            bool violated = supervisor.Lyapunov(s, Reference) > beta * supervisor.SaturationLevel
                || Math.Abs(DpcAngle(s.Theta1)) > Math.PI / 2 || Math.Abs(DpcAngle(s.Theta2)) > Math.PI / 2;
            if (violated && firstViolation < 0) firstViolation = i;
            loop.StepOnce();
            if (supervisor.Mode == SupervisorMode.SwingUp) fallback = i;
        }

        Assert.True(firstViolation >= 0, "The large push never left the Balance region.");
        Assert.True(fallback >= 0, "Supervisor did not fall back to SwingUp.");
        int dwellTicks = (int)Math.Round(supervisor.GetParameter("DwellTime") / loop.TimeStep);
        Assert.InRange(fallback - firstViolation, dwellTicks - 2, dwellTicks + 50);
    }

    [Fact]
    public void ComputeControlInputDoesNotAllocateOver1000TicksAcrossAllModes()
    {
        var plant = new NonlinearEulerLagrangePlant();
        var parameters = new Dictionary<string, double>(StringComparer.Ordinal);
        foreach (var d in plant.GetParameterDescriptors()) parameters[d.Key] = plant.GetParameter(d.Key);

        var supervisor = new SwingUpSupervisorController();
        supervisor.Initialize(parameters);

        var hanging = new PendulumState(0, 0.1, Math.PI - 0.2, Math.PI + 0.1, 0.2, 1.0, -0.5);

        // Warm up the same sequence so JIT tiering does not count.
        RunSequence(supervisor, hanging, out _, out _, out _);
        supervisor.Reset();

        long before = GC.GetAllocatedBytesForCurrentThread();
        RunSequence(supervisor, hanging, out bool sawSwingUp, out bool sawCapture, out bool sawBalance);
        long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

        Assert.Equal(0, allocated);
        Assert.True(sawSwingUp && sawCapture && sawBalance);
        // Repeated shadow simulations ran inside the measured window: the rejected ones stop early
        // at the cart limit, and the accepted slow-state test runs the full horizon.
        Assert.True(supervisor.CaptureTestCount >= 30, $"Only {supervisor.CaptureTestCount} capture tests ran.");
    }

    // 1,000 ticks: 300 swing-up (hanging), 400 repeated rejected capture tests, 300 balance.
    private static void RunSequence(SwingUpSupervisorController supervisor, in PendulumState hanging,
        out bool sawSwingUp, out bool sawCapture, out bool sawBalance)
    {
        sawSwingUp = sawCapture = sawBalance = false;
        var reference = Reference;
        for (int i = 0; i < 1000; i++)
        {
            PendulumState s = i < 300 ? hanging : i < 700 ? FastUncatchableState : SlowCatchableState;
            supervisor.ComputeControlInput(in s, in reference, 0.001);
            switch (supervisor.Mode)
            {
                case SupervisorMode.SwingUp: sawSwingUp = true; break;
                case SupervisorMode.Capture: sawCapture = true; break;
                case SupervisorMode.Balance: sawBalance = true; break;
            }
        }
    }

    private static double DpcAngle(double radians) => Math.IEEERemainder(radians, 2.0 * Math.PI);
}
