using System;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media;
using DipLabSim.Core;
using SkiaSharp;
using SkiaSharp.Views.Desktop;
using SkiaSharp.Views.WPF;

namespace DipLabSim.UI.Views;

public partial class PendulumCanvasControl : UserControl
{
    public static readonly DependencyProperty StateProperty =
        DependencyProperty.Register(
            nameof(State),
            typeof(PendulumState),
            typeof(PendulumCanvasControl),
            new FrameworkPropertyMetadata(PendulumState.Zero, FrameworkPropertyMetadataOptions.AffectsRender));

    public static readonly DependencyProperty ControlForceUProperty =
        DependencyProperty.Register(
            nameof(ControlForceU),
            typeof(double),
            typeof(PendulumCanvasControl),
            new FrameworkPropertyMetadata(0.0, FrameworkPropertyMetadataOptions.AffectsRender));

    public static readonly DependencyProperty DisturbanceForceProperty = DependencyProperty.Register(
        nameof(DisturbanceForce), typeof(double), typeof(PendulumCanvasControl), new FrameworkPropertyMetadata(0.0));
    public static readonly DependencyProperty DisturbanceTargetProperty = DependencyProperty.Register(
        nameof(DisturbanceTarget), typeof(DisturbanceTarget), typeof(PendulumCanvasControl),
        new FrameworkPropertyMetadata(DisturbanceTarget.UpperLinkTip));
    public double DisturbanceForce { get => (double)GetValue(DisturbanceForceProperty); set => SetValue(DisturbanceForceProperty, value); }
    public DisturbanceTarget DisturbanceTarget { get => (DisturbanceTarget)GetValue(DisturbanceTargetProperty); set => SetValue(DisturbanceTargetProperty, value); }

    public static readonly DependencyProperty L1Property =
        DependencyProperty.Register(
            nameof(L1),
            typeof(double),
            typeof(PendulumCanvasControl),
            new FrameworkPropertyMetadata(0.35, FrameworkPropertyMetadataOptions.AffectsRender));

    public static readonly DependencyProperty L2Property =
        DependencyProperty.Register(
            nameof(L2),
            typeof(double),
            typeof(PendulumCanvasControl),
            new FrameworkPropertyMetadata(0.28, FrameworkPropertyMetadataOptions.AffectsRender));

    public PendulumState State
    {
        get => (PendulumState)GetValue(StateProperty);
        set => SetValue(StateProperty, value);
    }

    public double ControlForceU
    {
        get => (double)GetValue(ControlForceUProperty);
        set => SetValue(ControlForceUProperty, value);
    }

    public double L1
    {
        get => (double)GetValue(L1Property);
        set => SetValue(L1Property, value);
    }

    public double L2
    {
        get => (double)GetValue(L2Property);
        set => SetValue(L2Property, value);
    }

    // --- Cached Paint & Font Resources (Zero-Heap Allocation in Rendering Loop) ---
    private readonly SKPaint _bgPaint = new() { Color = new SKColor(14, 15, 23), Style = SKPaintStyle.Fill };
    private readonly SKPaint _gridPaint = new() { Color = new SKColor(28, 30, 46), StrokeWidth = 1f, Style = SKPaintStyle.Stroke };
    private readonly SKPaint _gridSubPaint = new() { Color = new SKColor(20, 22, 34), StrokeWidth = 1f, Style = SKPaintStyle.Stroke };
    
    private readonly SKPaint _trackRailPaint = new() { Color = new SKColor(45, 50, 70), StrokeWidth = 10f, Style = SKPaintStyle.Stroke, StrokeCap = SKStrokeCap.Round };
    private readonly SKPaint _trackSlotPaint = new() { Color = new SKColor(20, 24, 38), StrokeWidth = 4f, Style = SKPaintStyle.Stroke, StrokeCap = SKStrokeCap.Round };
    private readonly SKPaint _tickPaint = new() { Color = new SKColor(80, 90, 120), StrokeWidth = 2f, Style = SKPaintStyle.Stroke };
    private readonly SKPaint _tickTextPaint = new() { Color = new SKColor(110, 125, 160), IsAntialias = true };

    private readonly SKPaint _cartBodyPaint = new() { Color = new SKColor(32, 38, 56), Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _cartBorderPaint = new() { Color = new SKColor(65, 80, 115), StrokeWidth = 2.5f, Style = SKPaintStyle.Stroke, IsAntialias = true };
    private readonly SKPaint _cartAccentPaint = new() { Color = new SKColor(0, 229, 255), StrokeWidth = 3f, Style = SKPaintStyle.Stroke, IsAntialias = true };
    private readonly SKPaint _cartWheelPaint = new() { Color = new SKColor(18, 22, 32), Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _cartWheelRimPaint = new() { Color = new SKColor(90, 110, 150), StrokeWidth = 2f, Style = SKPaintStyle.Stroke, IsAntialias = true };

    private readonly SKPaint _link1Paint = new() { Color = new SKColor(0, 210, 255), StrokeWidth = 8f, Style = SKPaintStyle.Stroke, StrokeCap = SKStrokeCap.Round, IsAntialias = true };
    private readonly SKPaint _link1CorePaint = new() { Color = new SKColor(200, 245, 255), StrokeWidth = 3f, Style = SKPaintStyle.Stroke, StrokeCap = SKStrokeCap.Round, IsAntialias = true };
    
    private readonly SKPaint _link2Paint = new() { Color = new SKColor(255, 158, 0), StrokeWidth = 6.5f, Style = SKPaintStyle.Stroke, StrokeCap = SKStrokeCap.Round, IsAntialias = true };
    private readonly SKPaint _link2CorePaint = new() { Color = new SKColor(255, 235, 190), StrokeWidth = 2.5f, Style = SKPaintStyle.Stroke, StrokeCap = SKStrokeCap.Round, IsAntialias = true };

    private readonly SKPaint _jointOuterPaint = new() { Color = new SKColor(240, 245, 255), Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _jointBorderPaint = new() { Color = new SKColor(20, 24, 38), StrokeWidth = 2f, Style = SKPaintStyle.Stroke, IsAntialias = true };
    private readonly SKPaint _jointCenterPaint = new() { Color = new SKColor(0, 160, 230), Style = SKPaintStyle.Fill, IsAntialias = true };

    private readonly SKPaint _tipPaint = new() { Color = new SKColor(255, 60, 100), Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _tipGlowPaint = new() { Color = new SKColor(255, 60, 100, 80), Style = SKPaintStyle.Fill, IsAntialias = true };

    private readonly SKPaint _forceArrowPaint = new() { Color = new SKColor(230, 40, 90), StrokeWidth = 4f, Style = SKPaintStyle.Stroke, StrokeCap = SKStrokeCap.Round, IsAntialias = true };

    // HUD paints
    private readonly SKPaint _hudBgPaint = new() { Color = new SKColor(16, 20, 30, 225), Style = SKPaintStyle.Fill, IsAntialias = true };
    private readonly SKPaint _hudBorderPaint = new() { Color = new SKColor(45, 60, 90), StrokeWidth = 1.5f, Style = SKPaintStyle.Stroke, IsAntialias = true };
    private readonly SKPaint _hudLabelPaint = new() { Color = new SKColor(140, 160, 195), IsAntialias = true };
    private readonly SKPaint _hudValuePaint = new() { Color = new SKColor(240, 245, 255), IsAntialias = true };
    private readonly SKPaint _hudAccentPaint = new() { Color = new SKColor(0, 229, 255), IsAntialias = true };

    private readonly SKTypeface _typeface;
    private readonly SKFont _fontTicks;
    private readonly SKFont _fontHudRegular;
    private readonly SKFont _fontHudBold;

    // Preallocated buffer for number formatting to avoid string garbage in hot loop
    private readonly char[] _textBuffer = new char[128];

    private bool _isHookedToRendering = false;

    public PendulumCanvasControl()
    {
        InitializeComponent();

        _typeface = SKTypeface.FromFamilyName("Segoe UI", SKFontStyleWeight.Normal, SKFontStyleWidth.Normal, SKFontStyleSlant.Upright) 
                    ?? SKTypeface.Default;

        _fontTicks = new SKFont(_typeface, 11f);
        _fontHudRegular = new SKFont(_typeface, 12f);
        _fontHudBold = new SKFont(_typeface, 13f);

        Loaded += OnLoaded;
        Unloaded += OnUnloaded;
    }

    private void OnLoaded(object sender, RoutedEventArgs e)
    {
        if (!_isHookedToRendering)
        {
            CompositionTarget.Rendering += OnCompositionTargetRendering;
            _isHookedToRendering = true;
        }
    }

    private void OnUnloaded(object sender, RoutedEventArgs e)
    {
        if (_isHookedToRendering)
        {
            CompositionTarget.Rendering -= OnCompositionTargetRendering;
            _isHookedToRendering = false;
        }
    }

    private void OnCompositionTargetRendering(object? sender, EventArgs e)
    {
        SkiaCanvas.InvalidateVisual();
    }

    private void OnPaintSurface(object sender, SKPaintSurfaceEventArgs e)
    {
        var canvas = e.Surface.Canvas;
        var info = e.Info;

        int width = info.Width;
        int height = info.Height;

        // 1. Clear background
        canvas.Clear(_bgPaint.Color);

        // 2. Compute coordinate mapping
        float centerX = width * 0.5f;
        float trackY = height * 0.52f;

        // Scale: fit 3.5m track width and max rod reach
        double l1 = Math.Max(0.05, L1);
        double l2 = Math.Max(0.05, L2);
        double totalReach = l1 + l2;

        float horizScale = width / 3.8f;
        float vertScale = (height * 0.43f - 30f) / (float)totalReach;
        float scale = Math.Min(horizScale, vertScale);
        scale = Math.Max(10f, scale);
        // Follow the cart once it leaves the central viewport; no physical rail stops are modelled.
        if (Math.Abs(State.X) > 1.0) centerX -= (float)(State.X * scale);

        // 3. Draw grid lines
        DrawGrid(canvas, width, height, centerX, trackY, scale);

        // 4. Draw track rail and distance scale
        DrawTrack(canvas, width, centerX, trackY, scale);

        // 5. Draw cart and pendulums
        var st = State;
        DrawDIPC(canvas, in st, ControlForceU, (float)l1, (float)l2, centerX, trackY, scale);

        // 6. Draw HUD Telemetry Box
        DrawHud(canvas, in st, ControlForceU, (float)l1, (float)l2);
    }

    private void DrawGrid(SKCanvas canvas, int width, int height, float centerX, float trackY, float scale)
    {
        // Vertical grid lines every 0.5 meters
        float halfMeterPx = 0.5f * scale;
        int maxGridLines = (int)(width / halfMeterPx) + 1;

        for (int i = -maxGridLines; i <= maxGridLines; i++)
        {
            float gx = centerX + i * halfMeterPx;
            if (gx >= 0 && gx <= width)
            {
                var paint = (i % 2 == 0) ? _gridPaint : _gridSubPaint;
                canvas.DrawLine(gx, 0, gx, height, paint);
            }
        }

        // Horizontal grid lines
        for (float gy = trackY; gy >= 0; gy -= halfMeterPx)
        {
            canvas.DrawLine(0, gy, width, gy, _gridSubPaint);
        }
        for (float gy = trackY + halfMeterPx; gy <= height; gy += halfMeterPx)
        {
            canvas.DrawLine(0, gy, width, gy, _gridSubPaint);
        }
    }

    private void DrawTrack(SKCanvas canvas, int width, float centerX, float trackY, float scale)
    {
        float trackLeft = centerX - 1.8f * scale;
        float trackRight = centerX + 1.8f * scale;

        // Limit track within screen bounds with slight margin
        if (trackLeft < 30f) trackLeft = 30f;
        if (trackRight > width - 30f) trackRight = width - 30f;

        // Outer Rail
        canvas.DrawLine(trackLeft, trackY, trackRight, trackY, _trackRailPaint);
        // Inner Slot
        canvas.DrawLine(trackLeft, trackY, trackRight, trackY, _trackSlotPaint);

        // Distance scale ticks: -1.5m to +1.5m every 0.25m
        for (double m = -1.5; m <= 1.501; m += 0.25)
        {
            float tx = centerX + (float)(m * scale);
            if (tx >= trackLeft && tx <= trackRight)
            {
                bool isMajor = Math.Abs(m % 0.5) < 0.01;
                float tickHeight = isMajor ? 12f : 6f;
                canvas.DrawLine(tx, trackY + 8f, tx, trackY + 8f + tickHeight, _tickPaint);

                if (isMajor)
                {
                    string label = m.ToString("0.0") + "m";
                    float labelWidth = _fontTicks.MeasureText(label);
                    canvas.DrawText(label, tx - labelWidth * 0.5f, trackY + 34f, _fontTicks, _tickTextPaint);
                }
            }
        }
    }

    private void DrawDIPC(
        SKCanvas canvas,
        in PendulumState state,
        double controlForceU,
        float l1,
        float l2,
        float centerX,
        float trackY,
        float scale)
    {
        // Cart Center in screen coords
        float cartScreenX = centerX + (float)(state.X * scale);
        float cartScreenY = trackY;

        // Cart Dimensions
        float cartWidth = 84f;
        float cartHeight = 36f;
        float halfW = cartWidth * 0.5f;
        float halfH = cartHeight * 0.5f;

        // Wheels
        float wheelRadius = 8f;
        float wheelOffset = halfW - 14f;
        float wheelY = cartScreenY + halfH + wheelRadius - 2f;

        // Left wheel
        canvas.DrawCircle(cartScreenX - wheelOffset, wheelY, wheelRadius, _cartWheelPaint);
        canvas.DrawCircle(cartScreenX - wheelOffset, wheelY, wheelRadius, _cartWheelRimPaint);
        // Right wheel
        canvas.DrawCircle(cartScreenX + wheelOffset, wheelY, wheelRadius, _cartWheelPaint);
        canvas.DrawCircle(cartScreenX + wheelOffset, wheelY, wheelRadius, _cartWheelRimPaint);

        // Cart Body
        var cartRect = new SKRect(
            cartScreenX - halfW,
            cartScreenY - halfH,
            cartScreenX + halfW,
            cartScreenY + halfH);

        canvas.DrawRoundRect(cartRect, 6f, 6f, _cartBodyPaint);
        canvas.DrawRoundRect(cartRect, 6f, 6f, _cartBorderPaint);

        // Cart Accent Stripe
        canvas.DrawLine(
            cartScreenX - halfW + 10f,
            cartScreenY - halfH + 6f,
            cartScreenX + halfW - 10f,
            cartScreenY - halfH + 6f,
            _cartAccentPaint);

        // Control Force Vector arrow (if active)
        if (Math.Abs(controlForceU) > 0.1)
        {
            float arrowLen = Math.Clamp((float)(controlForceU * 2.5), -70f, 70f);
            float startArrowX = cartScreenX;
            float endArrowX = cartScreenX + arrowLen;
            float arrowY = cartScreenY;
            canvas.DrawLine(startArrowX, arrowY, endArrowX, arrowY, _forceArrowPaint);
        }

        // --- Kinematics in World Coordinates (Meters) ---
        // Cart Pivot: (x, 0)
        // Joint 1: (x1, y1) = (x + L1*sin(theta1), L1*cos(theta1))
        // Tip 2: (x2, y2) = (x1 + L2*sin(theta2), y1 + L2*cos(theta2))
        // Angles are measured from upright vertical (0 = upright).
        // Screen Y goes downward, so:
        // Screen Y = trackY - y_world * scale

        float pivotScreenX = cartScreenX;
        float pivotScreenY = cartScreenY - halfH; // Pivot mounted on cart top

        double s1 = Math.Sin(state.Theta1);
        double c1 = Math.Cos(state.Theta1);
        float j1ScreenX = pivotScreenX + (float)(l1 * s1 * scale);
        float j1ScreenY = pivotScreenY - (float)(l1 * c1 * scale);

        double s2 = Math.Sin(state.Theta2);
        double c2 = Math.Cos(state.Theta2);
        float tip2ScreenX = j1ScreenX + (float)(l2 * s2 * scale);
        float tip2ScreenY = j1ScreenY - (float)(l2 * c2 * scale);

        // Link 1 (Cyan)
        canvas.DrawLine(pivotScreenX, pivotScreenY, j1ScreenX, j1ScreenY, _link1Paint);
        canvas.DrawLine(pivotScreenX, pivotScreenY, j1ScreenX, j1ScreenY, _link1CorePaint);

        // Cart Pivot Pin
        canvas.DrawCircle(pivotScreenX, pivotScreenY, 7f, _jointOuterPaint);
        canvas.DrawCircle(pivotScreenX, pivotScreenY, 7f, _jointBorderPaint);
        canvas.DrawCircle(pivotScreenX, pivotScreenY, 3.5f, _jointCenterPaint);

        // Link 2 (Amber / Orange)
        canvas.DrawLine(j1ScreenX, j1ScreenY, tip2ScreenX, tip2ScreenY, _link2Paint);
        canvas.DrawLine(j1ScreenX, j1ScreenY, tip2ScreenX, tip2ScreenY, _link2CorePaint);

        // Joint 1 Pin
        canvas.DrawCircle(j1ScreenX, j1ScreenY, 6.5f, _jointOuterPaint);
        canvas.DrawCircle(j1ScreenX, j1ScreenY, 6.5f, _jointBorderPaint);
        canvas.DrawCircle(j1ScreenX, j1ScreenY, 3f, _jointCenterPaint);

        // Tip 2 Bob / Indicator
        canvas.DrawCircle(tip2ScreenX, tip2ScreenY, 9f, _tipGlowPaint);
        canvas.DrawCircle(tip2ScreenX, tip2ScreenY, 5f, _tipPaint);
        if (Math.Abs(DisturbanceForce) > 0.001)
        {
            float px = DisturbanceTarget == DisturbanceTarget.LowerLinkTip ? j1ScreenX : tip2ScreenX;
            float py = DisturbanceTarget == DisturbanceTarget.LowerLinkTip ? j1ScreenY : tip2ScreenY;
            float length = (float)Math.CopySign(Math.Clamp(Math.Abs(DisturbanceForce) * 12, 25, 100), DisturbanceForce);
            float end = px + length, direction = Math.Sign(length);
            canvas.DrawLine(px, py, end, py, _forceArrowPaint);
            canvas.DrawLine(end, py, end - direction * 9, py - 6, _forceArrowPaint);
            canvas.DrawLine(end, py, end - direction * 9, py + 6, _forceArrowPaint);
        }
    }

    private void DrawHud(
        SKCanvas canvas,
        in PendulumState state,
        double controlForceU,
        float l1,
        float l2)
    {
        float hudX = 18f;
        float hudY = 18f;
        float hudW = 260f;
        float hudH = 160f;

        // HUD Background panel
        var hudRect = new SKRect(hudX, hudY, hudX + hudW, hudY + hudH);
        canvas.DrawRoundRect(hudRect, 8f, 8f, _hudBgPaint);
        canvas.DrawRoundRect(hudRect, 8f, 8f, _hudBorderPaint);

        // Title
        canvas.DrawText("OPEN-LOOP PHYSICS", hudX + 14f, hudY + 22f, _fontHudBold, _hudAccentPaint);
        canvas.DrawLine(hudX + 14f, hudY + 28f, hudX + hudW - 14f, hudY + 28f, _hudBorderPaint);

        float rowY = hudY + 46f;
        float rowSpacing = 20f;

        // Row 1: Time & Cart X
        DrawHudRow(canvas, "Time (t):", $"{state.Time:F2} s", hudX + 14f, rowY);
        rowY += rowSpacing;

        DrawHudRow(canvas, "Cart Pos (x):", $"{state.X:+0.000;-0.000; 0.000} m", hudX + 14f, rowY);
        rowY += rowSpacing;

        // Row 2: Theta 1
        double deg1 = state.Theta1 * 180.0 / Math.PI;
        DrawHudRow(canvas, "Angle Link 1 (θ₁):", $"{state.Theta1:+0.00;-0.00; 0.00} rad ({deg1:+0.0;-0.0; 0.0}°)", hudX + 14f, rowY);
        rowY += rowSpacing;

        // Row 3: Theta 2
        double deg2 = state.Theta2 * 180.0 / Math.PI;
        DrawHudRow(canvas, "Angle Link 2 (θ₂):", $"{state.Theta2:+0.00;-0.00; 0.00} rad ({deg2:+0.0;-0.0; 0.0}°)", hudX + 14f, rowY);
        rowY += rowSpacing;

        // Row 4: Control Input
        DrawHudRow(canvas, "Control Input (u):", $"{controlForceU:+0.00;-0.00; 0.00} N", hudX + 14f, rowY);
        rowY += rowSpacing;

        // Row 5: Active Rod Geometry
        DrawHudRow(canvas, "Geometry (L₁, L₂):", $"{l1:F2} m , {l2:F2} m", hudX + 14f, rowY);
    }

    private void DrawHudRow(SKCanvas canvas, string label, string value, float x, float y)
    {
        canvas.DrawText(label, x, y, _fontHudRegular, _hudLabelPaint);
        canvas.DrawText(value, x + 115f, y, _fontHudRegular, _hudValuePaint);
    }
}
