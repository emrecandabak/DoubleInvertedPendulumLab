using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.IO;
using System.Linq;
using System.Windows;
using System.Windows.Data;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DipLabSim.Core;
using DipLabSim.UI.Models;
using DipLabSim.UI.Services;

namespace DipLabSim.UI.ViewModels;

public sealed partial class MainViewModel : ObservableObject, IDisposable
{
    private readonly MockKinematicPreviewService _previewService;
    private readonly ControllerDirectoryScanner _directoryScanner;

    private readonly MockPlantModel _model1;
    private readonly MockPlantModel _model2;

    [ObservableProperty]
    private MockPlantModel _currentPlant;

    [ObservableProperty]
    private ControllerOption? _selectedController;

    [ObservableProperty]
    private double _selectedSpeed = 1.0;

    [ObservableProperty]
    private bool _isRunning;

    [ObservableProperty]
    private double _simulatedTime;

    [ObservableProperty]
    private double _currentX;

    [ObservableProperty]
    private double _currentTheta1;

    [ObservableProperty]
    private double _currentTheta2;

    [ObservableProperty]
    private double _currentDx;

    [ObservableProperty]
    private double _currentDTheta1;

    [ObservableProperty]
    private double _currentDTheta2;

    [ObservableProperty]
    private double _controlForceU;

    // Manual slider values
    [ObservableProperty]
    private double _manualX = 0.0;

    [ObservableProperty]
    private double _manualTheta1 = 0.0;

    [ObservableProperty]
    private double _manualTheta2 = 0.0;

    [ObservableProperty]
    private string _statusMessage = "Ready";

    public ObservableCollection<MockPlantModel> AvailablePlants { get; } = new();
    public ObservableCollection<ControllerOption> AvailableControllers { get; } = new();
    public ObservableCollection<double> AvailableSpeeds { get; } = new() { 0.25, 0.5, 1.0, 2.0 };
    public ObservableCollection<ParameterItemViewModel> ParameterItems { get; } = new();

    public ICollectionView ParameterItemsView { get; }

    public MockKinematicPreviewService PreviewService => _previewService;
    public ControllerDirectoryScanner DirectoryScanner => _directoryScanner;

    public PendulumState CurrentState => _previewService.CurrentState;
    public double L1 => CurrentPlant.GetParameter("L1");
    public double L2 => CurrentPlant.GetParameter("L2");

    public MainViewModel()
    {
        _previewService = new MockKinematicPreviewService();
        _directoryScanner = new ControllerDirectoryScanner();

        _model1 = new MockPlantModel(PlantModelType.Model1_IdealEulerLagrange);
        _model2 = new MockPlantModel(PlantModelType.Model2_ExtendedPlant);

        _model1.ParametersChanged += OnPlantParametersChanged;
        _model2.ParametersChanged += OnPlantParametersChanged;

        AvailablePlants.Add(_model1);
        AvailablePlants.Add(_model2);
        CurrentPlant = _model1;

        // Setup Parameter Items view with category grouping
        ParameterItemsView = CollectionViewSource.GetDefaultView(ParameterItems);
        ParameterItemsView.GroupDescriptions.Add(new PropertyGroupDescription(nameof(ParameterItemViewModel.Category)));

        RebuildParameterItems();

        // Load controllers
        RefreshControllers();

        // Wire scanner events
        _directoryScanner.ControllersChanged += () =>
        {
            Application.Current.Dispatcher.Invoke(RefreshControllers);
        };

        // Wire preview service events
        _previewService.FrameAvailable += OnSimulationFrameAvailable;
        _previewService.StateChanged += () =>
        {
            IsRunning = _previewService.IsRunning;
            StatusMessage = IsRunning ? "Simulating dynamic kinematics..." : "Paused (Manual preview active)";
        };

        StatusMessage = "Ready (Week 1 Kinematic Preview)";
    }

    private void RebuildParameterItems()
    {
        ParameterItems.Clear();
        foreach (var desc in CurrentPlant.GetParameterDescriptors())
        {
            ParameterItems.Add(new ParameterItemViewModel(CurrentPlant, desc));
        }
    }

    partial void OnCurrentPlantChanged(MockPlantModel value)
    {
        RebuildParameterItems();
        OnPropertyChanged(nameof(L1));
        OnPropertyChanged(nameof(L2));
        StatusMessage = $"Switched to {value.Name}";
    }

    private void OnPlantParametersChanged()
    {
        OnPropertyChanged(nameof(L1));
        OnPropertyChanged(nameof(L2));
    }

    partial void OnSelectedSpeedChanged(double value)
    {
        _previewService.SpeedMultiplier = value;
    }

    partial void OnSelectedControllerChanged(ControllerOption? value)
    {
        if (value != null)
        {
            StatusMessage = $"Selected controller: {value.DisplayName}";
        }
    }

    partial void OnManualXChanged(double value)
    {
        if (!IsRunning)
        {
            _previewService.SetManualState(ManualX, ManualTheta1, ManualTheta2);
        }
    }

    public double ManualTheta1Deg => ManualTheta1 * 180.0 / Math.PI;
    public double ManualTheta2Deg => ManualTheta2 * 180.0 / Math.PI;

    partial void OnManualTheta1Changed(double value)
    {
        OnPropertyChanged(nameof(ManualTheta1Deg));
        if (!IsRunning)
        {
            _previewService.SetManualState(ManualX, ManualTheta1, ManualTheta2);
        }
    }

    partial void OnManualTheta2Changed(double value)
    {
        OnPropertyChanged(nameof(ManualTheta2Deg));
        if (!IsRunning)
        {
            _previewService.SetManualState(ManualX, ManualTheta1, ManualTheta2);
        }
    }

    [RelayCommand]
    private void SetUprightPreset()
    {
        ManualX = 0;
        ManualTheta1 = 0;
        ManualTheta2 = 0;
        if (!IsRunning) _previewService.SetManualState(0, 0, 0);
    }

    [RelayCommand]
    private void SetHangingPreset()
    {
        ManualX = 0;
        ManualTheta1 = Math.PI;
        ManualTheta2 = Math.PI;
        if (!IsRunning) _previewService.SetManualState(0, Math.PI, Math.PI);
    }

    [RelayCommand]
    private void SetPerturbedPreset()
    {
        ManualX = 0.2;
        ManualTheta1 = 0.25;
        ManualTheta2 = -0.3;
        if (!IsRunning) _previewService.SetManualState(0.2, 0.25, -0.3);
    }

    private void OnSimulationFrameAvailable(SimulationFrame frame)
    {
        SimulatedTime = frame.State.Time;
        CurrentX = frame.State.X;
        CurrentTheta1 = frame.State.Theta1;
        CurrentTheta2 = frame.State.Theta2;
        CurrentDx = frame.State.Dx;
        CurrentDTheta1 = frame.State.DTheta1;
        CurrentDTheta2 = frame.State.DTheta2;
        ControlForceU = frame.ControlForceU;

        if (IsRunning)
        {
            ManualX = frame.State.X;
            ManualTheta1 = frame.State.Theta1;
            ManualTheta2 = frame.State.Theta2;
        }

        OnPropertyChanged(nameof(CurrentState));
    }

    [RelayCommand]
    private void TogglePlayPause()
    {
        _previewService.TogglePlayPause();
    }

    [RelayCommand]
    private void ResetSimulation()
    {
        _previewService.Reset();
        ManualX = 0;
        ManualTheta1 = 0;
        ManualTheta2 = 0;
        OnPropertyChanged(nameof(CurrentState));
        StatusMessage = "Simulation reset to zero initial state";
    }

    [RelayCommand]
    private void RefreshControllers()
    {
        var list = _directoryScanner.Scan();
        var previousSelection = SelectedController;

        AvailableControllers.Clear();
        foreach (var item in list)
        {
            AvailableControllers.Add(item);
        }

        if (previousSelection != null)
        {
            SelectedController = AvailableControllers.FirstOrDefault(c => c.DisplayName == previousSelection.DisplayName)
                                 ?? AvailableControllers.FirstOrDefault();
        }
        else
        {
            SelectedController = AvailableControllers.FirstOrDefault();
        }
    }

    [RelayCommand]
    private void OpenControllersFolder()
    {
        try
        {
            if (!Directory.Exists(_directoryScanner.ControllersDirectory))
            {
                Directory.CreateDirectory(_directoryScanner.ControllersDirectory);
            }
            System.Diagnostics.Process.Start(new System.Diagnostics.ProcessStartInfo
            {
                FileName = _directoryScanner.ControllersDirectory,
                UseShellExecute = true
            });
        }
        catch (Exception ex)
        {
            StatusMessage = $"Cannot open directory: {ex.Message}";
        }
    }

    [RelayCommand]
    private void ResetParametersToDefault()
    {
        CurrentPlant.ResetToDefaults();
        foreach (var item in ParameterItems)
        {
            item.RefreshFromPlant();
        }
        StatusMessage = "Parameters reset to default values";
    }

    [RelayCommand]
    private void ZeroManualState()
    {
        ManualX = 0.0;
        ManualTheta1 = 0.0;
        ManualTheta2 = 0.0;
        _previewService.SetManualState(0, 0, 0);
    }

    public void Dispose()
    {
        _previewService.Dispose();
        _directoryScanner.Dispose();
    }
}
