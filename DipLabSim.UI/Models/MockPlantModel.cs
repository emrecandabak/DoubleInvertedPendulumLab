using System;
using System.Collections.Generic;
using System.Linq;
using DipLabSim.Core;

namespace DipLabSim.UI.Models;

public enum PlantModelType
{
    Model1_IdealEulerLagrange,
    Model2_ExtendedPlant
}

/// <summary>
/// Week 1 Mock Plant Model implementing IPlant. Exposes both Model 1 (KON-445E baseline)
/// and Model 2 (extended actuator/friction parameters) parameter sets.
/// </summary>
public sealed class MockPlantModel : IPlant
{
    private readonly Dictionary<string, double> _parameters = new(StringComparer.Ordinal);
    private readonly List<ParameterDescriptor> _descriptors = new();
    private readonly PlantModelType _modelType;

    public event Action? ParametersChanged;

    public string Name => _modelType switch
    {
        PlantModelType.Model1_IdealEulerLagrange => "Model 1: Ideal Euler–Lagrange",
        PlantModelType.Model2_ExtendedPlant => "Model 2: Extended Plant (Actuator & Coulomb Friction)",
        _ => "Unknown Model"
    };

    public string Description => _modelType switch
    {
        PlantModelType.Model1_IdealEulerLagrange => "Nonlinear double inverted pendulum on a cart based on KON-445E Section 3.6.",
        PlantModelType.Model2_ExtendedPlant => "Extended double inverted pendulum model including DC motor dynamics, pulley ratio, and Coulomb friction.",
        _ => string.Empty
    };

    public PlantModelType ModelType => _modelType;

    public MockPlantModel(PlantModelType modelType)
    {
        _modelType = modelType;
        InitializeParameters();
    }

    private void InitializeParameters()
    {
        _descriptors.Clear();
        _parameters.Clear();

        // --- Category: Masses ---
        AddDescriptor("M", "Cart Mass", "M", "kg", "Masses", 1.20, 0.1, 10.0, 0.05);
        AddDescriptor("m1", "Link 1 Mass", "m₁", "kg", "Masses", 0.20, 0.01, 2.0, 0.01);
        AddDescriptor("m2", "Link 2 Mass", "m₂", "kg", "Masses", 0.15, 0.01, 2.0, 0.01);

        // --- Category: Geometry ---
        AddDescriptor("L1", "Link 1 Length", "L₁", "m", "Geometry", 0.35, 0.05, 1.5, 0.01);
        AddDescriptor("L2", "Link 2 Length", "L₂", "m", "Geometry", 0.28, 0.05, 1.5, 0.01);
        AddDescriptor("l1", "Link 1 COM Distance", "l₁", "m", "Geometry", 0.175, 0.01, 1.0, 0.005);
        AddDescriptor("l2", "Link 2 COM Distance", "l₂", "m", "Geometry", 0.140, 0.01, 1.0, 0.005);

        // --- Category: Inertias ---
        AddDescriptor("J1", "Link 1 Inertia", "J₁", "kg·m²", "Inertias", 0.002, 0.0001, 0.1, 0.0001);
        AddDescriptor("J2", "Link 2 Inertia", "J₂", "kg·m²", "Inertias", 0.001, 0.0001, 0.1, 0.0001);

        // --- Category: Damping & Friction ---
        AddDescriptor("b", "Cart Viscous Damping", "b", "N·s/m", "Damping & Friction", 0.10, 0.0, 5.0, 0.01);
        AddDescriptor("c1", "Joint 1 Viscous Damping", "c₁", "N·m·s/rad", "Damping & Friction", 0.005, 0.0, 0.5, 0.001);
        AddDescriptor("c2", "Joint 2 Viscous Damping", "c₂", "N·m·s/rad", "Damping & Friction", 0.005, 0.0, 0.5, 0.001);

        // Extended parameters for Model 2
        if (_modelType == PlantModelType.Model2_ExtendedPlant)
        {
            AddDescriptor("Km", "Motor Torque Constant", "Kₘ", "N·m/A", "Actuator & Drive", 0.050, 0.001, 1.0, 0.001);
            AddDescriptor("Rm", "Armature Resistance", "Rₘ", "Ω", "Actuator & Drive", 2.50, 0.1, 20.0, 0.1);
            AddDescriptor("r", "Drive Pulley Radius", "r", "m", "Actuator & Drive", 0.020, 0.005, 0.1, 0.001);
            AddDescriptor("fc", "Coulomb Friction Force", "f_c", "N", "Actuator & Drive", 0.08, 0.0, 2.0, 0.01);
        }
    }

    private void AddDescriptor(
        string key,
        string displayName,
        string symbol,
        string unit,
        string category,
        double defaultValue,
        double minValue,
        double maxValue,
        double step)
    {
        var desc = new ParameterDescriptor(key, displayName, symbol, unit, category, defaultValue, minValue, maxValue, step);
        _descriptors.Add(desc);
        _parameters[key] = defaultValue;
    }

    public IReadOnlyList<ParameterDescriptor> GetParameterDescriptors() => _descriptors;

    public double GetParameter(string key) =>
        _parameters.TryGetValue(key, out var val) ? val : 0.0;

    public void SetParameter(string key, double value)
    {
        if (_parameters.ContainsKey(key))
        {
            _parameters[key] = value;
            ParametersChanged?.Invoke();
        }
    }

    public void ResetToDefaults()
    {
        foreach (var desc in _descriptors)
        {
            _parameters[desc.Key] = desc.DefaultValue;
        }
        ParametersChanged?.Invoke();
    }

    public void ComputeDerivatives(
        in PendulumState state,
        double controlForceU,
        double disturbanceForce,
        out PendulumStateDerivative derivative)
    {
        // Week 1 mock derivative computation
        derivative = new PendulumStateDerivative(
            Dx: state.Dx,
            DTheta1: state.DTheta1,
            DTheta2: state.DTheta2,
            Ddx: 0,
            DdTheta1: 0,
            DdTheta2: 0);
    }
}
