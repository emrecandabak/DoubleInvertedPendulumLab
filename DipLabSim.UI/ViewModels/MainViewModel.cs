using System;
using System.Collections.ObjectModel;
using System.ComponentModel;
using System.Windows.Data;
using CommunityToolkit.Mvvm.ComponentModel;
using CommunityToolkit.Mvvm.Input;
using DipLabSim.Core;
using DipLabSim.Physics;
using DipLabSim.UI.Services;

namespace DipLabSim.UI.ViewModels;

public sealed partial class MainViewModel : ObservableObject, IDisposable
{
    private readonly SimulationService _simulation;
    private bool _synchronizing;
    private readonly DoublePendulumPlant _model1 = new NonlinearEulerLagrangePlant();
    private readonly DoublePendulumPlant _model2 = new NewtonEulerPlant();

    [ObservableProperty] private DoublePendulumPlant _currentPlant;
    [ObservableProperty] private IOdeSolver _selectedSolver;
    [ObservableProperty] private double _selectedStep = 0.001;
    [ObservableProperty] private double _selectedSpeed = 1;
    [ObservableProperty] private bool _isRunning;
    [ObservableProperty] private double _simulatedTime;
    [ObservableProperty] private double _currentX;
    [ObservableProperty] private double _currentTheta1;
    [ObservableProperty] private double _currentTheta2;
    [ObservableProperty] private double _currentDx;
    [ObservableProperty] private double _currentDTheta1;
    [ObservableProperty] private double _currentDTheta2;
    [ObservableProperty] private double _controlForceU;
    [ObservableProperty] private double _disturbanceForce;
    [ObservableProperty] private DisturbanceTarget _activeDisturbanceTarget;
    [ObservableProperty] private double _manualX;
    [ObservableProperty] private double _manualTheta1 = 0.05;
    [ObservableProperty] private double _manualTheta2 = -0.03;
    [ObservableProperty] private double _manualCartForce;
    [ObservableProperty] private double _pushMagnitude = 2;
    [ObservableProperty] private double _pushDuration = 0.1;
    [ObservableProperty] private DisturbanceTarget _selectedDisturbanceTarget = DisturbanceTarget.UpperLinkTip;
    [ObservableProperty] private string _statusMessage = "Ready — open loop";

    public ObservableCollection<DoublePendulumPlant> AvailablePlants { get; } = new();
    public ObservableCollection<IOdeSolver> AvailableSolvers { get; } = new()
        { new ForwardEulerSolver(), new HeunSolver(), new Rk4Solver(), new BackwardEulerSolver() };
    public double[] AvailableSteps { get; } = [0.0001, 0.0005, 0.001, 0.002, 0.005, 0.01];
    public double[] AvailableSpeeds { get; } = [0.25, 0.5, 1, 2];
    public DisturbanceTarget[] DisturbanceTargets { get; } = Enum.GetValues<DisturbanceTarget>();
    public ObservableCollection<ParameterItemViewModel> ParameterItems { get; } = new();
    public ICollectionView ParameterItemsView { get; }
    public SimulationService PreviewService => _simulation;
    public PendulumState CurrentState => _simulation.CurrentState;
    public bool IsPaused => !IsRunning;
    public double L1 => CurrentPlant.GetParameter("L1");
    public double L2 => CurrentPlant.GetParameter("L2");
    public double ManualTheta1Deg => ManualTheta1 * 180 / Math.PI;
    public double ManualTheta2Deg => ManualTheta2 * 180 / Math.PI;

    public MainViewModel()
    {
        // Set backing fields before hooks; UI is the composition root.
        _currentPlant = _model1;
        _selectedSolver = AvailableSolvers[2];
        AvailablePlants.Add(_model1); AvailablePlants.Add(_model2);
        ParameterItemsView = CollectionViewSource.GetDefaultView(ParameterItems);
        ParameterItemsView.GroupDescriptions.Add(new PropertyGroupDescription(nameof(ParameterItemViewModel.Category)));
        _simulation = new SimulationService(_currentPlant, _selectedSolver);
        _simulation.FrameAvailable += OnFrame;
        _simulation.StateChanged += OnEngineStateChanged;
        _model1.ParametersChanged += OnPlantParametersChanged;
        _model2.ParametersChanged += OnPlantParametersChanged;
        RebuildParameterItems();
        ResetSimulation();
    }

    partial void OnIsRunningChanged(bool value) => OnPropertyChanged(nameof(IsPaused));
    partial void OnCurrentPlantChanged(DoublePendulumPlant value)
    {
        _simulation.Pause();
        // Both modelling routes always share one physical parameter set.
        var previous = ReferenceEquals(value, _model1) ? _model2 : _model1;
        foreach (var p in previous.GetParameterDescriptors())
            if (p.Key != "l1" && p.Key != "l2") value.SetParameter(p.Key, previous.GetParameter(p.Key));
        value.SetParameter("l1", previous.GetParameter("l1"));
        value.SetParameter("l2", previous.GetParameter("l2"));
        RebuildParameterItems();
        Configure();
        OnPropertyChanged(nameof(L1)); OnPropertyChanged(nameof(L2));
    }
    partial void OnSelectedSolverChanged(IOdeSolver value) => Configure();
    partial void OnSelectedStepChanged(double value) => Configure();
    partial void OnSelectedSpeedChanged(double value) => _simulation.SpeedMultiplier = value;
    partial void OnManualCartForceChanged(double value)
    {
        if (!double.IsFinite(value)) { ManualCartForce = 0; return; }
        double bounded = Math.Clamp(value, -30, 30);
        if (bounded != value) { ManualCartForce = bounded; return; }
        _simulation.ManualForce = bounded;
    }
    partial void OnManualXChanged(double value) => UpdateManualState();
    partial void OnManualTheta1Changed(double value) { OnPropertyChanged(nameof(ManualTheta1Deg)); UpdateManualState(); }
    partial void OnManualTheta2Changed(double value) { OnPropertyChanged(nameof(ManualTheta2Deg)); UpdateManualState(); }

    private void Configure()
    {
        _simulation.Configure(CurrentPlant, SelectedSolver, SelectedStep);
        ManualCartForce = 0;
        StatusMessage = $"Open loop | {CurrentPlant.Name} | {SelectedSolver.Name}";
    }
    private void UpdateManualState()
    {
        if (!_synchronizing && !IsRunning)
            _simulation.SetManualState(ManualX, ManualTheta1, ManualTheta2);
    }
    private void RebuildParameterItems()
    {
        ParameterItems.Clear();
        foreach (var p in CurrentPlant.GetParameterDescriptors()) ParameterItems.Add(new(CurrentPlant, p));
    }
    private void OnPlantParametersChanged()
    {
        foreach (var item in ParameterItems) item.RefreshFromPlant();
        OnPropertyChanged(nameof(L1)); OnPropertyChanged(nameof(L2));
    }
    private void OnFrame(SimulationFrame frame)
    {
        SimulatedTime = frame.State.Time; CurrentX = frame.State.X;
        CurrentTheta1 = frame.State.Theta1; CurrentTheta2 = frame.State.Theta2;
        CurrentDx = frame.State.Dx; CurrentDTheta1 = frame.State.DTheta1; CurrentDTheta2 = frame.State.DTheta2;
        ControlForceU = frame.ControlForceU; DisturbanceForce = frame.DisturbanceForce;
        ActiveDisturbanceTarget = frame.DisturbanceTarget;
        OnPropertyChanged(nameof(CurrentState));
    }
    private void OnEngineStateChanged()
    {
        IsRunning = _simulation.IsRunning;
        StatusMessage = _simulation.Fault ?? (IsRunning ? "Running — open-loop physics" : "Paused — edit initial state, model or solver");
    }
    [RelayCommand] private void TogglePlayPause()
    {
        if (IsRunning) _simulation.Pause();
        else { _simulation.ManualForce = ManualCartForce; _simulation.Start(); }
    }
    [RelayCommand] private void ResetSimulation()
    {
        _simulation.Reset(new(0, ManualX, ManualTheta1, ManualTheta2, 0, 0, 0));
        ManualCartForce = 0;
        StatusMessage = "Reset to initial state (zero velocities); Play to simulate";
    }
    [RelayCommand] private void StopCartForce() => ManualCartForce = 0;
    [RelayCommand] private void CartLeft() => ManualCartForce = -Math.Abs(ValidMagnitude());
    [RelayCommand] private void CartRight() => ManualCartForce = Math.Abs(ValidMagnitude());
    [RelayCommand] private void PushLeft() => ApplyPush(-1);
    [RelayCommand] private void PushRight() => ApplyPush(1);
    private double ValidMagnitude() => double.IsFinite(PushMagnitude) ? Math.Clamp(PushMagnitude, 0, 30) : 2;
    private void ApplyPush(int sign)
    {
        if (!IsRunning) { StatusMessage = "Press Play before applying a link disturbance."; return; }
        double duration = double.IsFinite(PushDuration) ? Math.Clamp(PushDuration, 0.001, 5) : 0.1;
        _simulation.Push(SelectedDisturbanceTarget, sign * ValidMagnitude(), duration);
        StatusMessage = $"Push: {SelectedDisturbanceTarget}, {sign * ValidMagnitude():F2} N, {duration:F3} s";
    }
    private void Preset(double x, double t1, double t2)
    {
        if (IsRunning) return;
        _synchronizing = true;
        ManualX = x; ManualTheta1 = t1; ManualTheta2 = t2;
        _synchronizing = false;
        ResetSimulation();
    }
    [RelayCommand] private void SetUprightPreset() => Preset(0, 0, 0);
    [RelayCommand] private void SetHangingPreset() => Preset(0, Math.PI, Math.PI);
    [RelayCommand] private void SetPerturbedPreset() => Preset(0, 0.05, -0.03);
    [RelayCommand] private void ZeroManualState() => SetUprightPreset();
    [RelayCommand] private void ResetParametersToDefault()
    {
        if (IsRunning) return;
        CurrentPlant.ResetToDefaults();
        Configure();
    }
    public void Dispose()
    {
        _model1.ParametersChanged -= OnPlantParametersChanged;
        _model2.ParametersChanged -= OnPlantParametersChanged;
        _simulation.FrameAvailable -= OnFrame;
        _simulation.StateChanged -= OnEngineStateChanged;
        _simulation.Dispose();
    }
}
