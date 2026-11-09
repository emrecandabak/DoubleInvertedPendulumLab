using DipLabSim.Core;
using DipLabSim.Engine;
using DipLabSim.Physics;
using Xunit;

namespace DipLabSim.Numerics.Tests;

public class ControllerTests
{
    public static IEnumerable<object[]> BuiltInControllersAndPlants()
    {
        yield return [new LqrController(), new NonlinearEulerLagrangePlant()];
        yield return [new LqrController(), new NewtonEulerPlant()];
        yield return [new CascadePidController(), new NonlinearEulerLagrangePlant()];
        yield return [new CascadePidController(), new NewtonEulerPlant()];
    }

    [Theory]
    [MemberData(nameof(BuiltInControllersAndPlants))]
    public void ControllerStabilizesPerturbedInitialState(IController controller, DoublePendulumPlant plant)
    {
        using var loop = new SimulationLoop(plant, new Rk4Solver(), controller);
        loop.SetState(new PendulumState(0, 0.25, 0.06, -0.05, 0, 0, 0));

        // Simulate 8 seconds at dt = 1 ms
        for (int i = 0; i < 8000; i++)
            loop.StepOnce();

        var finalState = loop.PausedState;
        Assert.InRange(Math.Abs(finalState.X), 0, 0.02);
        Assert.InRange(Math.Abs(finalState.Theta1), 0, 0.01);
        Assert.InRange(Math.Abs(finalState.Theta2), 0, 0.01);
        Assert.InRange(Math.Abs(finalState.Dx), 0, 0.03);
        Assert.InRange(Math.Abs(finalState.DTheta1), 0, 0.03);
        Assert.InRange(Math.Abs(finalState.DTheta2), 0, 0.03);
    }

    [Theory]
    [InlineData(typeof(LqrController), DisturbanceTarget.UpperLinkTip)]
    [InlineData(typeof(LqrController), DisturbanceTarget.LowerLinkTip)]
    [InlineData(typeof(CascadePidController), DisturbanceTarget.UpperLinkTip)]
    [InlineData(typeof(CascadePidController), DisturbanceTarget.LowerLinkTip)]
    public void ControllerRejectsLinkTipDisturbanceAndCartPulse(Type controllerType, DisturbanceTarget target)
    {
        var plant = new NonlinearEulerLagrangePlant();
        var controller = (IController)Activator.CreateInstance(controllerType)!;
        using var loop = new SimulationLoop(plant, new Rk4Solver(), controller);
        loop.SetState(PendulumState.Zero);

        // Apply a link tip push disturbance
        loop.Push(target, 2.0, 0.08);
        for (int i = 0; i < 3000; i++)
            loop.StepOnce();

        // Apply a momentary cart disturbance pulse in the opposite direction
        loop.PulseCart(-6.0, 0.08);
        for (int i = 0; i < 6000; i++)
            loop.StepOnce();

        var finalState = loop.PausedState;
        Assert.InRange(Math.Abs(finalState.X), 0, 0.02);
        Assert.InRange(Math.Abs(finalState.Theta1), 0, 0.01);
        Assert.InRange(Math.Abs(finalState.Theta2), 0, 0.01);
    }

    [Theory]
    [InlineData(typeof(LqrController))]
    [InlineData(typeof(CascadePidController))]
    public void ControllerAdaptsToModifiedPlantGeometryAndMasses(Type controllerType)
    {
        var plant = new NonlinearEulerLagrangePlant();
        plant.SetParameter("M", 2.2);
        plant.SetParameter("m1", 0.65);
        plant.SetParameter("m2", 0.55);
        plant.SetParameter("L1", 0.6);
        plant.SetParameter("l1", 0.3);
        plant.SetParameter("L2", 0.65);
        plant.SetParameter("l2", 0.325);

        var controller = (IController)Activator.CreateInstance(controllerType)!;
        using var loop = new SimulationLoop(plant, new Rk4Solver(), controller);
        loop.SetState(new PendulumState(0, -0.15, 0.04, -0.03, 0, 0, 0));

        for (int i = 0; i < 8000; i++)
            loop.StepOnce();

        var finalState = loop.PausedState;
        Assert.InRange(Math.Abs(finalState.X), 0, 0.03);
        Assert.InRange(Math.Abs(finalState.Theta1), 0, 0.01);
        Assert.InRange(Math.Abs(finalState.Theta2), 0, 0.01);
    }

    [Fact]
    public void ControllersDoNotAllocateInHotLoop()
    {
        IController[] controllers = [new OpenLoopController(), new LqrController(), new CascadePidController()];
        var state = new PendulumState(0, 0.1, 0.03, -0.02, -0.1, 0.2, -0.15);
        var reference = PendulumState.Zero;

        foreach (var controller in controllers)
        {
            for (int i = 0; i < 100; i++)
                _ = controller.ComputeControlInput(in state, in reference, 0.001);

            long before = GC.GetAllocatedBytesForCurrentThread();
            for (int i = 0; i < 1000; i++)
                _ = controller.ComputeControlInput(in state, in reference, 0.001);
            long allocated = GC.GetAllocatedBytesForCurrentThread() - before;

            Assert.Equal(0, allocated);
        }
    }
}
