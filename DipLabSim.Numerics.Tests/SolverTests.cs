using DipLabSim.Core;
using DipLabSim.Physics;
using Xunit;

namespace DipLabSim.Numerics.Tests;

public class SolverTests
{
    public static IEnumerable<object[]> Solvers()
    {
        yield return [new ForwardEulerSolver(), 1];
        yield return [new HeunSolver(), 2];
        yield return [new Rk4Solver(), 4];
        yield return [new BackwardEulerSolver(), 1];
    }
    private static double Error(IOdeSolver solver, double h)
    {
        var plant = new Oscillator();
        var s = PendulumState.Zero with { X = 1 };
        for (int i = 0; i < (int)Math.Round(1 / h); i++) s = solver.Step(plant, s, 0, 0, h);
        Assert.InRange(Math.Abs(s.Time - 1), 0, 1e-12);
        return Math.Sqrt(Math.Pow(s.X - Math.Cos(1), 2) + Math.Pow(s.Dx + Math.Sin(1), 2));
    }
    [Theory, MemberData(nameof(Solvers))]
    public void HalvingStepShowsExpectedConvergenceOrder(IOdeSolver solver, int order)
    {
        double ratio = Error(solver, 0.1) / Error(solver, 0.05);
        Assert.InRange(ratio, 0.85 * Math.Pow(2, order), 1.15 * Math.Pow(2, order));
    }
    [Theory, MemberData(nameof(Solvers))]
    public void InvalidStepIsRejected(IOdeSolver solver, int _)
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => solver.Step(new Oscillator(), PendulumState.Zero, 0, 0, 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => solver.Step(new Oscillator(), PendulumState.Zero, 0, 0, double.NaN));
    }
    [Theory, MemberData(nameof(Solvers))]
    public void BothPendulumModelsGiveSameShortTrajectory(IOdeSolver solver, int _)
    {
        var a = new NonlinearEulerLagrangePlant(); var b = new NewtonEulerPlant();
        var sa = new PendulumState(0, 0, 0.05, -0.03, 0, 0, 0); var sb = sa;
        for (int i = 0; i < 1000; i++)
        {
            sa = solver.Step(a, sa, 0.2, 0, 0.001); sb = solver.Step(b, sb, 0.2, 0, 0.001);
        }
        Assert.InRange(Math.Abs(sa.X - sb.X) + Math.Abs(sa.Theta1 - sb.Theta1) + Math.Abs(sa.Theta2 - sb.Theta2), 0, 1e-7);
    }
    [Fact]
    public void BackwardEulerReportsNonConvergence()
    {
        // x' = x^2 + 1, h = 1, x0 = 0 -> y = y^2+1 has no real root.
        Assert.Throws<ArithmeticException>(() => new BackwardEulerSolver().Step(new NoRootPlant(), PendulumState.Zero, 0, 0, 1));
    }
    [Theory, MemberData(nameof(Solvers))]
    public void SolverAndPhysicsHotPathsDoNotAllocate(IOdeSolver solver, int _)
    {
        foreach (DoublePendulumPlant p in new DoublePendulumPlant[] { new NonlinearEulerLagrangePlant(), new NewtonEulerPlant() })
        {
            var s = new PendulumState(0, 0, 0.05, -0.03, 0, 0, 0);
            for (int i = 0; i < 100; i++) solver.Step(p, s, 0, 0, 0.001);
            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 100; i++) solver.Step(p, s, 0, 0, 0.001);
            Assert.Equal(0, GC.GetAllocatedBytesForCurrentThread() - before);
        }
    }
    private class Oscillator : IPlant
    {
        public string Name => "Analytic oscillator";
        public string Description => "x'' = -x";
        public IReadOnlyList<ParameterDescriptor> GetParameterDescriptors() => Array.Empty<ParameterDescriptor>();
        public double GetParameter(string key) => throw new NotSupportedException();
        public void SetParameter(string key, double value) => throw new NotSupportedException();
        public virtual void ComputeDerivatives(in PendulumState s, double u, double f, out PendulumStateDerivative d) => d = new(s.Dx, 0, 0, -s.X, 0, 0);
    }
    private sealed class NoRootPlant : Oscillator
    {
        public override void ComputeDerivatives(in PendulumState s, double u, double f, out PendulumStateDerivative d) => d = new(s.X * s.X + 1, 0, 0, 0, 0, 0);
    }
}
