using DipLabSim.Core;
using DipLabSim.Physics;
using Xunit;

namespace DipLabSim.Numerics.Tests;

public class PhysicsTests
{
    [Theory]
    [InlineData(DisturbanceTarget.LowerLinkTip)]
    [InlineData(DisturbanceTarget.UpperLinkTip)]
    public void IndependentModelsAgreeAcrossRandomStatesParametersAndForces(DisturbanceTarget target)
    {
        var a = new NonlinearEulerLagrangePlant { DisturbanceTarget = target };
        var b = new NewtonEulerPlant { DisturbanceTarget = target };
        var random = new Random(445);
        for (int trial = 0; trial < 500; trial++)
        {
            foreach (var key in new[] { "M", "m1", "m2", "L1", "L2", "J1", "J2", "b", "c1", "c2" })
            {
                double value = key switch
                {
                    "L1" or "L2" => 0.3 + random.NextDouble(),
                    "J1" or "J2" => 0.002 + 0.1 * random.NextDouble(),
                    "c1" or "c2" => 0.03 * random.NextDouble(),
                    _ => 0.1 + random.NextDouble()
                };
                a.SetParameter(key, value); b.SetParameter(key, value);
            }
            a.SetParameter("l1", a.GetParameter("L1") * 0.5); b.SetParameter("l1", a.GetParameter("l1"));
            a.SetParameter("l2", a.GetParameter("L2") * 0.5); b.SetParameter("l2", a.GetParameter("l2"));
            double R() => random.NextDouble() * 2 - 1;
            var state = new PendulumState(0, R(), 3 * R(), 3 * R(), R(), 5 * R(), 5 * R());
            double u = 10 * R(), f = 5 * R();
            a.ComputeDerivatives(state, u, f, out var da);
            b.ComputeDerivatives(state, u, f, out var db);
            Assert.InRange(Math.Abs(da.Ddx - db.Ddx), 0, 1e-8);
            Assert.InRange(Math.Abs(da.DdTheta1 - db.DdTheta1), 0, 1e-8);
            Assert.InRange(Math.Abs(da.DdTheta2 - db.DdTheta2), 0, 1e-8);
        }
    }

    [Fact]
    public void UnforcedEquilibriaAndSmallPerturbationHaveCorrectBehavior()
    {
        foreach (DoublePendulumPlant plant in new DoublePendulumPlant[] { new NonlinearEulerLagrangePlant(), new NewtonEulerPlant() })
        {
            plant.ComputeDerivatives(PendulumState.Zero, 0, 0, out var upright);
            Assert.Equal(default, upright);
            var hanging = PendulumState.Zero with { Theta1 = Math.PI, Theta2 = Math.PI };
            plant.ComputeDerivatives(hanging, 0, 0, out var d);
            Assert.InRange(Math.Abs(d.DdTheta1) + Math.Abs(d.DdTheta2), 0, 1e-10);
            var s = PendulumState.Zero with { Theta1 = 0.01 };
            for (int i = 0; i < 300; i++) s = new Rk4Solver().Step(plant, s, 0, 0, 0.001);
            Assert.True(Math.Abs(s.Theta1) > 0.02);
        }
    }

    [Theory]
    [InlineData(false, DisturbanceTarget.LowerLinkTip)]
    [InlineData(false, DisturbanceTarget.UpperLinkTip)]
    [InlineData(true, DisturbanceTarget.LowerLinkTip)]
    [InlineData(true, DisturbanceTarget.UpperLinkTip)]
    public void EnergyDerivativeMatchesExternalPowerAndJointDissipation(bool newton, DisturbanceTarget target)
    {
        DoublePendulumPlant p = newton ? new NewtonEulerPlant() : new NonlinearEulerLagrangePlant();
        p.DisturbanceTarget = target;
        p.SetParameter("b", 0.3); p.SetParameter("c1", 0.04); p.SetParameter("c2", 0.03);
        var s = new PendulumState(0, 0.1, 0.7, -0.4, 0.5, -1.2, 2.1);
        p.ComputeDerivatives(s, 3, -2, out var d);
        const double epsilon = 1e-7;
        double measured = (p.Energy(PendulumState.AddScaled(s, d, epsilon))
            - p.Energy(PendulumState.AddScaled(s, d, -epsilon))) / (2 * epsilon);
        double tipVelocity = s.Dx + p.GetParameter("L1") * Math.Cos(s.Theta1) * s.DTheta1;
        if (target == DisturbanceTarget.UpperLinkTip) tipVelocity += p.GetParameter("L2") * Math.Cos(s.Theta2) * s.DTheta2;
        double expected = 3 * s.Dx - 2 * tipVelocity - 0.3 * s.Dx * s.Dx
            - 0.04 * s.DTheta1 * s.DTheta1 - 0.03 * Math.Pow(s.DTheta2 - s.DTheta1, 2);
        Assert.InRange(Math.Abs(measured - expected), 0, 1e-6);
    }

    [Theory]
    [InlineData(false)]
    [InlineData(true)]
    public void Rk4ConservesUnforcedUndampedEnergyOverFiveSeconds(bool newton)
    {
        DoublePendulumPlant p = newton ? new NewtonEulerPlant() : new NonlinearEulerLagrangePlant();
        var solver = new Rk4Solver();
        var s = new PendulumState(0, 0, 0.1, -0.08, 0, 0, 0);
        double e0 = p.Energy(s), maxError = 0;
        for (int i = 0; i < 5000; i++)
        {
            s = solver.Step(p, s, 0, 0, 0.001);
            maxError = Math.Max(maxError, Math.Abs(p.Energy(s) - e0));
        }
        Assert.InRange(maxError, 0, 1e-5);
    }

    [Fact]
    public void ParametersRejectNonphysicalValuesAndClampComWhenShortening()
    {
        var p = new NonlinearEulerLagrangePlant();
        Assert.Throws<ArgumentOutOfRangeException>(() => p.SetParameter("M", double.NaN));
        Assert.Throws<ArgumentOutOfRangeException>(() => p.SetParameter("J1", 0));
        Assert.Throws<ArgumentOutOfRangeException>(() => p.SetParameter("l1", 0.8));
        p.SetParameter("L1", 0.1);
        Assert.Equal(0.1, p.GetParameter("l1"));
    }
}
