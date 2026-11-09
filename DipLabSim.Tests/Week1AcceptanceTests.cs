using System;
using System.Linq;
using System.Runtime.ExceptionServices;
using System.Threading;
using DipLabSim.Core;
using DipLabSim.Physics;
using DipLabSim.UI.ViewModels;
using Xunit;

namespace DipLabSim.Tests;

public class UiAcceptanceTests
{
    // FactAttribute alone does not select STA. Every WPF integration assertion runs
    // on a real STA thread and rethrows failures on the xUnit test thread.
    private static void Sta(Action action)
    {
        Exception? failure = null;
        var thread = new Thread(() => { try { action(); } catch (Exception ex) { failure = ex; } });
        thread.SetApartmentState(ApartmentState.STA);
        thread.Start(); thread.Join();
        if (failure != null) ExceptionDispatchInfo.Capture(failure).Throw();
    }
    [Fact]
    public void ModelsShareParametersWhenSwitching() => Sta(() =>
    {
        using var vm = new MainViewModel();
        Assert.IsType<NonlinearEulerLagrangePlant>(vm.CurrentPlant);
        vm.CurrentPlant.SetParameter("L1", 0.7);
        vm.CurrentPlant.SetParameter("l1", 0.3);
        vm.CurrentPlant.SetParameter("b", 0.2);
        vm.CurrentPlant = vm.AvailablePlants[1];
        Assert.IsType<NewtonEulerPlant>(vm.CurrentPlant);
        Assert.Equal(0.7, vm.L1);
        Assert.Equal(0.3, vm.CurrentPlant.GetParameter("l1"));
        Assert.Equal(0.2, vm.CurrentPlant.GetParameter("b"));
        Assert.Equal(vm.CurrentPlant.GetParameterDescriptors().Count, vm.ParameterItems.Count);
    });
    [Fact]
    public void FourSolversAndBothDisturbanceTargetsAreAvailable() => Sta(() =>
    {
        using var vm = new MainViewModel();
        Assert.Equal(4, vm.AvailableSolvers.Count);
        Assert.IsType<Rk4Solver>(vm.SelectedSolver);
        Assert.Contains(DisturbanceTarget.LowerLinkTip, vm.DisturbanceTargets);
        Assert.Contains(DisturbanceTarget.UpperLinkTip, vm.DisturbanceTargets);
        vm.SelectedSolver = vm.AvailableSolvers[3];
        vm.SelectedStep = 0.002;
        Assert.True(vm.IsPaused);
    });
    [Fact]
    public void PresetsResetStateAndTime() => Sta(() =>
    {
        using var vm = new MainViewModel();
        vm.SetHangingPresetCommand.Execute(null);
        Assert.Equal(Math.PI, vm.CurrentState.Theta1);
        Assert.Equal(Math.PI, vm.CurrentState.Theta2);
        vm.SetUprightPresetCommand.Execute(null);
        Assert.Equal(PendulumState.Zero, vm.CurrentState);
        vm.SetPerturbedPresetCommand.Execute(null);
        Assert.Equal(0.05, vm.CurrentState.Theta1);
        vm.ManualCartForce = 4;
        vm.ResetSimulationCommand.Execute(null);
        Assert.Equal(0, vm.ManualCartForce);
        Assert.Equal(0, vm.CurrentState.Time);
    });
    [Fact]
    public void ParameterEditsRespectGeometryAndRejectNonFiniteInput() => Sta(() =>
    {
        using var vm = new MainViewModel();
        var length = vm.ParameterItems.First(p => p.Key == "L1");
        var com = vm.ParameterItems.First(p => p.Key == "l1");
        length.ValueString = "0.1";
        Assert.Equal(0.1, vm.L1);
        Assert.Equal(0.1, com.Value);
        com.Value = 999;
        Assert.Equal(0.1, com.Value);
        length.ValueString = "NaN";
        Assert.Equal(0.1, length.Value);
    });
    [Fact]
    public void CartForceSliderOnlySetsForceForButtonsAndDoesNotApplyDirectly() => Sta(() =>
    {
        using var vm = new MainViewModel();
        Assert.Equal(5, vm.CartForce);
        Assert.Equal(0, vm.ManualCartForce);
        vm.CartForce = 12;
        Assert.Equal(0, vm.ManualCartForce);

        vm.CartLeftCommand.Execute(null);
        Assert.Equal(-12, vm.ManualCartForce);

        vm.CartRightCommand.Execute(null);
        Assert.Equal(12, vm.ManualCartForce);

        vm.StopCartForceCommand.Execute(null);
        Assert.Equal(0, vm.ManualCartForce);
    });
    [Fact]
    public void MomentaryCartForceCommandsRequireRunningAndApplyPulse() => Sta(() =>
    {
        using var vm = new MainViewModel();
        vm.PulseCartLeftCommand.Execute(null);
        Assert.Contains("Press Play", vm.StatusMessage);
        vm.TogglePlayPauseCommand.Execute(null);
        Assert.True(vm.IsRunning);
        vm.PulseCartLeftCommand.Execute(null);
        Assert.Contains("Momentary cart force", vm.StatusMessage);
        vm.PulseCartLeftCommand.Execute(null);
        Assert.Contains("Momentary cart force", vm.StatusMessage);
        vm.PulseCartRightCommand.Execute(null);
        Assert.Contains("Momentary cart force", vm.StatusMessage);
    });
    [Fact]
    public void IntegratedLqrAndCascadePidControllersAreSelectableAndConfigurable() => Sta(() =>
    {
        using var vm = new MainViewModel();
        Assert.Equal(3, vm.AvailableControllers.Count);
        Assert.Contains(vm.AvailableControllers, c => c is LqrController);
        Assert.Contains(vm.AvailableControllers, c => c is CascadePidController);
        Assert.Contains(vm.AvailableControllers, c => c is OpenLoopController);

        Assert.IsType<LqrController>(vm.SelectedController);
        Assert.True(vm.HasControllerParameters);
        var qx = vm.ControllerParameterItems.First(p => p.Key == "Qx");
        double defaultQx = qx.Value;
        qx.Value = 25.0;
        Assert.Equal(25.0, vm.SelectedController.GetParameter("Qx"));
        vm.ResetControllerParametersToDefaultCommand.Execute(null);
        Assert.Equal(defaultQx, qx.Value);

        vm.SelectedController = vm.AvailableControllers.First(c => c is CascadePidController);
        Assert.True(vm.HasControllerParameters);
        Assert.Contains(vm.ControllerParameterItems, p => p.Key == "KpX");
        Assert.Contains(vm.ControllerParameterItems, p => p.Key == "Kp2");
        Assert.Contains(vm.ControllerParameterItems, p => p.Key == "Kp1");

        double defaultKp1 = vm.ControllerParameterItems.First(p => p.Key == "Kp1").Value;
        vm.CurrentPlant.SetParameter("M", 3.0);
        vm.AutoCalculateControllerParametersCommand.Execute(null);
        double updatedKp1 = vm.ControllerParameterItems.First(p => p.Key == "Kp1").Value;
        Assert.NotEqual(defaultKp1, updatedKp1);

        vm.SelectedController = vm.AvailableControllers.First(c => c is OpenLoopController);
        Assert.False(vm.HasControllerParameters);
    });
}
