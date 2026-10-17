using System;
using System.IO;
using System.Linq;
using System.Threading;
using DipLabSim.Core;
using DipLabSim.UI.Models;
using DipLabSim.UI.Services;
using DipLabSim.UI.ViewModels;
using Xunit;

namespace DipLabSim.Tests;

public class Week1AcceptanceTests
{
    [Fact]
    public void PendulumState_AddScaled_ComputesCorrectly()
    {
        var s = new PendulumState(Time: 1.0, X: 0.5, Theta1: 0.1, Theta2: -0.2, Dx: 1.0, DTheta1: 0.5, DTheta2: -0.5);
        var d = new PendulumStateDerivative(Dx: 1.0, DTheta1: 0.5, DTheta2: -0.5, Ddx: 2.0, DdTheta1: 1.0, DdTheta2: -1.0);

        double dt = 0.01;
        var next = PendulumState.AddScaled(in s, in d, dt);

        Assert.Equal(1.01, next.Time, 6);
        Assert.Equal(0.5 + 1.0 * dt, next.X, 6);
        Assert.Equal(0.1 + 0.5 * dt, next.Theta1, 6);
        Assert.Equal(-0.2 - 0.5 * dt, next.Theta2, 6);
        Assert.Equal(1.0 + 2.0 * dt, next.Dx, 6);
        Assert.Equal(0.5 + 1.0 * dt, next.DTheta1, 6);
        Assert.Equal(-0.5 - 1.0 * dt, next.DTheta2, 6);
    }

    [Fact]
    public void MockPlantModel_Model1_HasStandardDescriptors()
    {
        var plant = new MockPlantModel(PlantModelType.Model1_IdealEulerLagrange);
        var descriptors = plant.GetParameterDescriptors();

        Assert.NotEmpty(descriptors);
        Assert.Contains(descriptors, p => p.Key == "M" && p.Category == "Masses");
        Assert.Contains(descriptors, p => p.Key == "m1" && p.Category == "Masses");
        Assert.Contains(descriptors, p => p.Key == "m2" && p.Category == "Masses");
        Assert.Contains(descriptors, p => p.Key == "L1" && p.Category == "Geometry");
        Assert.Contains(descriptors, p => p.Key == "L2" && p.Category == "Geometry");
        Assert.Contains(descriptors, p => p.Key == "b" && p.Category == "Damping & Friction");

        // Model 1 should NOT have actuator parameters
        Assert.DoesNotContain(descriptors, p => p.Key == "Km");
        Assert.DoesNotContain(descriptors, p => p.Key == "Rm");
    }

    [Fact]
    public void MockPlantModel_Model2_HasActuatorDescriptors()
    {
        var plant = new MockPlantModel(PlantModelType.Model2_ExtendedPlant);
        var descriptors = plant.GetParameterDescriptors();

        Assert.Contains(descriptors, p => p.Key == "Km" && p.Category == "Actuator & Drive");
        Assert.Contains(descriptors, p => p.Key == "Rm" && p.Category == "Actuator & Drive");
        Assert.Contains(descriptors, p => p.Key == "r" && p.Category == "Actuator & Drive");
        Assert.Contains(descriptors, p => p.Key == "fc" && p.Category == "Actuator & Drive");
    }

    [Fact]
    public void MockPlantModel_SetParameter_FiresEventAndUpdates()
    {
        var plant = new MockPlantModel(PlantModelType.Model1_IdealEulerLagrange);
        bool eventFired = false;
        plant.ParametersChanged += () => eventFired = true;

        plant.SetParameter("L1", 0.55);

        Assert.True(eventFired);
        Assert.Equal(0.55, plant.GetParameter("L1"), 4);

        // Reset
        plant.ResetToDefaults();
        Assert.Equal(0.35, plant.GetParameter("L1"), 4);
    }

    [Fact]
    public void ControllerDirectoryScanner_DiscoversBuiltInAndDllFiles()
    {
        string tempDir = Path.Combine(Path.GetTempPath(), "DipLabSimTests_" + Guid.NewGuid().ToString("N"));
        try
        {
            using var scanner = new ControllerDirectoryScanner(tempDir);
            var initialList = scanner.Scan();

            // Must include None and External Python
            Assert.Contains(initialList, c => c.IsBuiltIn && c.DisplayName.Contains("None"));
            Assert.Contains(initialList, c => c.IsBuiltIn && c.DisplayName.Contains("External Python"));

            // Add a mock controller DLL
            string mockDllPath = Path.Combine(tempDir, "LqrController.dll");
            File.WriteAllText(mockDllPath, "mock dll content");

            var updatedList = scanner.Scan();
            Assert.Contains(updatedList, c => !c.IsBuiltIn && c.DisplayName.Contains("LqrController.dll"));
        }
        finally
        {
            if (Directory.Exists(tempDir))
            {
                Directory.Delete(tempDir, true);
            }
        }
    }

    [Fact]
    public void MockKinematicPreviewService_ManualStateScrubbing_UpdatesState()
    {
        using var service = new MockKinematicPreviewService();

        service.SetManualState(1.25, 0.45, -0.35);

        Assert.Equal(1.25, service.CurrentState.X, 4);
        Assert.Equal(0.45, service.CurrentState.Theta1, 4);
        Assert.Equal(-0.35, service.CurrentState.Theta2, 4);

        service.Reset();
        Assert.Equal(0.0, service.CurrentState.X, 4);
        Assert.Equal(0.0, service.CurrentState.Theta1, 4);
        Assert.Equal(0.0, service.CurrentState.Theta2, 4);
    }

    [UIFact]
    public void MainViewModel_PlantSwitching_RebuildsParameterCategories()
    {
        var vm = new MainViewModel();

        Assert.Equal("Model 1: Ideal Euler–Lagrange", vm.CurrentPlant.Name);
        int model1Count = vm.ParameterItems.Count;

        // Switch to Model 2
        var model2 = vm.AvailablePlants.First(p => p.ModelType == PlantModelType.Model2_ExtendedPlant);
        vm.CurrentPlant = model2;

        Assert.True(vm.ParameterItems.Count > model1Count);
        Assert.Contains(vm.ParameterItems, p => p.Category == "Actuator & Drive");

        // Verify L1 and L2 geometry reflection
        Assert.Equal(vm.CurrentPlant.GetParameter("L1"), vm.L1, 4);
        Assert.Equal(vm.CurrentPlant.GetParameter("L2"), vm.L2, 4);

        vm.Dispose();
    }

    [UIFact]
    public void MainViewModel_Presets_SetsExpectedAngles()
    {
        var vm = new MainViewModel();

        // Upright preset
        vm.SetUprightPresetCommand.Execute(null);
        Assert.Equal(0.0, vm.ManualX, 4);
        Assert.Equal(0.0, vm.ManualTheta1, 4);
        Assert.Equal(0.0, vm.ManualTheta2, 4);

        // Hanging preset
        vm.SetHangingPresetCommand.Execute(null);
        Assert.Equal(Math.PI, vm.ManualTheta1, 4);
        Assert.Equal(Math.PI, vm.ManualTheta2, 4);

        vm.Dispose();
    }

    [Fact]
    public void ParameterItemViewModel_TwoWayBindingAndClamping_Works()
    {
        var plant = new MockPlantModel(PlantModelType.Model1_IdealEulerLagrange);
        var desc = plant.GetParameterDescriptors().First(d => d.Key == "L1");
        var item = new ParameterItemViewModel(plant, desc);

        // Default
        Assert.Equal(0.35, item.Value, 4);
        Assert.Equal("0.35", item.ValueString);

        // Setting Value updates plant and ValueString
        item.Value = 0.52;
        Assert.Equal(0.52, plant.GetParameter("L1"), 4);
        Assert.Equal("0.52", item.ValueString);

        // Setting ValueString parses and updates Value and plant
        item.ValueString = "0.68";
        Assert.Equal(0.68, item.Value, 4);
        Assert.Equal(0.68, plant.GetParameter("L1"), 4);

        // Clamping to MaxValue
        item.Value = 999.0;
        Assert.Equal(desc.MaxValue, item.Value, 4);

        // Reset to default
        item.ResetToDefaultCommand.Execute(null);
        Assert.Equal(desc.DefaultValue, item.Value, 4);
    }

    [Theory]
    [InlineData(0.0, 0.0, 0.0, 0.35, 0.0, 0.35 + 0.28)] // Upright: x=0, y1=L1, y2=L1+L2
    [InlineData(Math.PI, Math.PI, 0.0, -0.35, 0.0, -0.35 - 0.28)] // Hanging down: y1=-L1, y2=-L1-L2
    [InlineData(Math.PI / 2, 0.0, 0.35, 0.0, 0.35, 0.28)] // Link 1 horizontal (+X), Link 2 vertical upright (+Y)
    public void KinematicsConvention_MatchesKon445E(
        double theta1, double theta2,
        double expectedX1, double expectedY1,
        double expectedX2, double expectedY2)
    {
        double x = 0.0;
        double L1 = 0.35;
        double L2 = 0.28;

        // Joint 1: (x1, y1) = (x + L1*sin(theta1), L1*cos(theta1))
        double x1 = x + L1 * Math.Sin(theta1);
        double y1 = L1 * Math.Cos(theta1);

        // Tip 2: (x2, y2) = (x1 + L2*sin(theta2), y1 + L2*cos(theta2))
        double x2 = x1 + L2 * Math.Sin(theta2);
        double y2 = y1 + L2 * Math.Cos(theta2);

        Assert.Equal(expectedX1, x1, 4);
        Assert.Equal(expectedY1, y1, 4);
        Assert.Equal(expectedX2, x2, 4);
        Assert.Equal(expectedY2, y2, 4);
    }
}

/// <summary>
/// Custom attribute to execute tests on STA thread for WPF components if needed.
/// </summary>
public class UIFactAttribute : FactAttribute { }
