using System;
using System.Windows;
using System.Windows.Controls;
using DipLabSim.Core;
using ScottPlot;
using ScottPlot.Plottables;

namespace DipLabSim.UI.Views;

public partial class TelemetryScopeControl : UserControl
{
    private DataLogger? _loggerCart;
    private DataLogger? _loggerTheta1;
    private DataLogger? _loggerTheta2;
    private DataLogger? _loggerControl;
    private DataLogger? _loggerDisturbance;
    private double _lastSampleTime = -1;
    private int _sampleCount;

    private readonly ScottPlot.Color _bgDark = ScottPlot.Color.FromHex("#12131C");
    private readonly ScottPlot.Color _dataBgDark = ScottPlot.Color.FromHex("#0E0F17");
    private readonly ScottPlot.Color _axisColor = ScottPlot.Color.FromHex("#7080A0");
    private readonly ScottPlot.Color _gridColor = ScottPlot.Color.FromHex("#1C2030");

    private readonly ScottPlot.Color _cyanColor = ScottPlot.Color.FromHex("#00D2FF");
    private readonly ScottPlot.Color _amberColor = ScottPlot.Color.FromHex("#FF9E00");
    private readonly ScottPlot.Color _magentaColor = ScottPlot.Color.FromHex("#FF2A85");
    private readonly ScottPlot.Color _blueColor = ScottPlot.Color.FromHex("#38BDF8");

    private double _lastRenderTime = 0;

    public TelemetryScopeControl()
    {
        InitializeComponent();
        InitializePlots();
    }

    private void InitializePlots()
    {
        // 1. Cart Plot: x(t)
        ConfigurePlotBase(CartPlot.Plot, "CART POSITION: x(t) [m]");
        _loggerCart = CartPlot.Plot.Add.DataLogger();
        _loggerCart.Color = _blueColor;
        _loggerCart.LineWidth = 2f;
        _loggerCart.LegendText = "Cart x";
        _loggerCart.ViewSlide(10.0);
        CartPlot.Plot.Axes.SetLimitsY(-2.5, 2.5);

        // 2. Angles Plot: theta1(t) & theta2(t)
        ConfigurePlotBase(AnglesPlot.Plot, "PENDULUM ANGLES: θ₁(t), θ₂(t) [rad]");
        _loggerTheta1 = AnglesPlot.Plot.Add.DataLogger();
        _loggerTheta1.Color = _cyanColor;
        _loggerTheta1.LineWidth = 2f;
        _loggerTheta1.LegendText = "Link 1 θ₁";
        _loggerTheta1.ViewSlide(10.0);

        _loggerTheta2 = AnglesPlot.Plot.Add.DataLogger();
        _loggerTheta2.Color = _amberColor;
        _loggerTheta2.LineWidth = 2f;
        _loggerTheta2.LegendText = "Link 2 θ₂";
        _loggerTheta2.ViewSlide(10.0);
        AnglesPlot.Plot.Axes.SetLimitsY(-Math.PI, Math.PI);

        // 3. Control Force Plot: u(t)
        ConfigurePlotBase(ControlPlot.Plot, "CART INPUT / LINK DISTURBANCE [N]");
        _loggerControl = ControlPlot.Plot.Add.DataLogger();
        _loggerControl.Color = _magentaColor;
        _loggerControl.LineWidth = 2f;
        _loggerControl.LegendText = "Control u";
        _loggerControl.ViewSlide(10.0);
        ControlPlot.Plot.Axes.SetLimitsY(-30.0, 30.0);
        _loggerDisturbance = ControlPlot.Plot.Add.DataLogger();
        _loggerDisturbance.Color = _amberColor;
        _loggerDisturbance.LegendText = "Link disturbance";
        _loggerDisturbance.ViewSlide(10.0);

        // Refresh initially
        CartPlot.Refresh();
        AnglesPlot.Refresh();
        ControlPlot.Refresh();
    }

    private void ConfigurePlotBase(Plot plot, string title)
    {
        plot.FigureBackground.Color = _bgDark;
        plot.DataBackground.Color = _dataBgDark;

        plot.Axes.Color(_axisColor);
        plot.Grid.MajorLineColor = _gridColor;

        plot.Title(title);
        plot.Axes.Title.Label.FontSize = 12f;
        plot.Axes.Title.Label.Bold = true;
        plot.Axes.Title.Label.ForeColor = ScottPlot.Color.FromHex("#C0D0E8");

        plot.ShowLegend(Alignment.UpperRight);
        plot.Legend.BackgroundColor = ScottPlot.Color.FromHex("#9012131C");
        plot.Legend.FontColor = ScottPlot.Color.FromHex("#C0D0E8");
        plot.Legend.FontSize = 10f;
    }

    public void AddFrame(in SimulationFrame frame)
    {
        double t = frame.State.Time;
        // DataLogger requires monotone X. Reset/configuration/manual edits may repeat t.
        if (t < _lastSampleTime || _sampleCount >= 36000) ClearData();
        if (t <= _lastSampleTime) return;
        _lastSampleTime = t;
        _sampleCount++;
        _loggerCart?.Add(t, frame.State.X);
        _loggerTheta1?.Add(t, frame.State.Theta1);
        _loggerTheta2?.Add(t, frame.State.Theta2);
        _loggerControl?.Add(t, frame.ControlForceU);
        _loggerDisturbance?.Add(t, frame.DisturbanceForce);
        // Angles remain unwrapped in the physical state; auto-scale to see full rotations.
        AnglesPlot.Plot.Axes.AutoScaleY();
        CartPlot.Plot.Axes.AutoScaleY();

        // Throttle UI refreshes to ~30-60 Hz max
        if (t - _lastRenderTime >= 0.025 || t < _lastRenderTime)
        {
            _lastRenderTime = t;
            CartPlot.Refresh();
            AnglesPlot.Refresh();
            ControlPlot.Refresh();
        }
    }

    public void ClearData()
    {
        _loggerCart?.Clear();
        _loggerTheta1?.Clear();
        _loggerTheta2?.Clear();
        _loggerControl?.Clear();
        _lastRenderTime = 0;
        _lastSampleTime = -1;
        _sampleCount = 0;
        _loggerDisturbance?.Clear();

        CartPlot.Refresh();
        AnglesPlot.Refresh();
        ControlPlot.Refresh();
    }
}
