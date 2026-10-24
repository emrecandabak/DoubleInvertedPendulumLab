using System;
using System.Globalization;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DipLabSim.Core;
using DipLabSim.UI.Models;

namespace DipLabSim.UI.ViewModels;

public sealed partial class ParameterItemViewModel : ObservableObject
{
    private readonly IConfigurableComponent _plant;
    private readonly ParameterDescriptor _descriptor;
    private double _currentValue;

    public string Key => _descriptor.Key;
    public string DisplayName => _descriptor.DisplayName;
    public string Symbol => _descriptor.Symbol;
    public string Unit => _descriptor.Unit;
    public string Category => _descriptor.Category;
    public double MinValue => _descriptor.MinValue;
    public double MaxValue => Key switch
    {
        "l1" => Math.Min(_descriptor.MaxValue, _plant.GetParameter("L1")),
        "l2" => Math.Min(_descriptor.MaxValue, _plant.GetParameter("L2")),
        _ => _descriptor.MaxValue
    };
    public double Step => _descriptor.Step;
    public double DefaultValue => _descriptor.DefaultValue;

    public string HeaderLabel => string.IsNullOrWhiteSpace(_descriptor.Unit)
        ? $"{_descriptor.DisplayName} ({_descriptor.Symbol})"
        : $"{_descriptor.DisplayName} ({_descriptor.Symbol}) [{_descriptor.Unit}]";

    public double Value
    {
        get => _currentValue;
        set
        {
            if (!double.IsFinite(value)) return;
            var clamped = Math.Clamp(value, MinValue, MaxValue);
            if (Math.Abs(_currentValue - clamped) > 1e-9)
            {
                _currentValue = clamped;
                _plant.SetParameter(Key, clamped);
                OnPropertyChanged();
                OnPropertyChanged(nameof(ValueString));
            }
        }
    }

    public string ValueString
    {
        get => _currentValue.ToString("0.####", CultureInfo.InvariantCulture);
        set
        {
            if (double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out var parsed))
            {
                Value = parsed;
            }
            else if (double.TryParse(value, NumberStyles.Float, CultureInfo.CurrentCulture, out var parsedLocal))
            {
                Value = parsedLocal;
            }
        }
    }

    public ParameterItemViewModel(IConfigurableComponent plant, ParameterDescriptor descriptor)
    {
        _plant = plant;
        _descriptor = descriptor;
        _currentValue = plant.GetParameter(descriptor.Key);
    }

    public void RefreshFromPlant()
    {
        OnPropertyChanged(nameof(MaxValue));
        var plantVal = _plant.GetParameter(Key);
        if (Math.Abs(_currentValue - plantVal) > 1e-9)
        {
            _currentValue = plantVal;
            OnPropertyChanged(nameof(Value));
            OnPropertyChanged(nameof(ValueString));
        }
    }

    [RelayCommand]
    private void ResetToDefault()
    {
        Value = DefaultValue;
    }
}
