using System;
using System.Windows;
using DipLabSim.Core;
using DipLabSim.UI.ViewModels;

namespace DipLabSim.UI;

/// <summary>
/// Interaction logic for MainWindow.xaml
/// </summary>
public partial class MainWindow : Window
{
    private MainViewModel? _viewModel;

    public MainWindow()
    {
        InitializeComponent();

        Loaded += OnMainWindowLoaded;
        Closed += OnMainWindowClosed;
    }

    private void OnMainWindowLoaded(object sender, RoutedEventArgs e)
    {
        if (DataContext is MainViewModel vm)
        {
            _viewModel = vm;
            _viewModel.PreviewService.FrameAvailable += OnSimulationFrameAvailable;
        }
    }

    private void OnMainWindowClosed(object? sender, EventArgs e)
    {
        if (_viewModel != null)
        {
            _viewModel.PreviewService.FrameAvailable -= OnSimulationFrameAvailable;
            _viewModel.Dispose();
        }
    }

    private void OnSimulationFrameAvailable(SimulationFrame frame)
    {
        // Deliver frame to ScottPlot scope
        //Hello
        TelemetryScope.AddFrame(in frame);
    }

    private void PendulumCanvas_Loaded(object sender, RoutedEventArgs e)
    {

    }
}