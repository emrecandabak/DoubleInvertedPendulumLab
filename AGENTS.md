# DipLabSim — Unified AI Agent Instructions (`AGENTS.md`)
---

## 1. Project Overview & Phased Roadmap

**DipLabSim** is a modular, object-oriented Double Inverted Pendulum on a Cart (DIPC) laboratory simulator built on **.NET 10.0 LTS** (Visual Studio 2026 Community) using **WPF**, **CommunityToolkit.Mvvm**, **SkiaSharp**, and **ScottPlot 5**.

### Current Focus: Week 1 — Visual Design, Dynamic Parameter Input & 2D Visualizer Only
- **Goal for Week 1:** Deliver a minimalist, ultra-responsive WPF UI shell with dynamic model parameter inputs (`ItemsControl`), a hardware-accelerated `SkiaSharp` 2D pendulum visualizer that resizes/updates live when parameters or angles change, a `ScottPlot 5` telemetry scope layout, and a `/Controllers` directory scanner ComboBox.
- **What is Mocked in Week 1:** Real Euler–Lagrange physics, RK4 integration, and active controller execution are **not required this week**. Bind the UI to `MockPlantModel` presets (exposing Model 1 & Model 2 parameters) and a lightweight `MockKinematicPreviewService` (allowing manual slider scrubbing of $x, \theta_1, \theta_2$ or a simple sine-wave demo animation to test the visualizer and charts).

### Upcoming Weeks: Physics, Solver, Engine & Controller Integration
- **Phase 2 (Physics & Solver — 2 Teammates):** Replace `MockPlantModel` with `NonlinearEulerLagrangePlant` (KON-445E Report Section 3.6) and `SecondPlantModel`, integrated via `Rk4Solver` at $dt = 1\text{ ms}$ ($1000\text{ Hz}$).
- **Phase 3 (Sim Engine — 1 Teammate):** Replace the mock preview timer with the dedicated background `SimulationLoop` thread ($dt = 1\text{ ms}$ physics, $T_s = 5\text{ ms} / 200\text{ Hz}$ controller sampling) streaming through a lock-free `Channel<SimulationFrame>`.
- **Phase 4 (DLL Plugins & Python gRPC — 1 Teammate):** Wire the UI's `/Controllers` directory dropdown to load compiled `.dll` controllers via `AssemblyLoadContext` (`isCollectible: true`) and external Python MPC/RL servers via gRPC.

---

## 2. Global Coding & Architectural Rules

1. **Strict Dependency Direction (Ports & Adapters):**
   - `DipLabSim.Core` has **zero external NuGet or project dependencies**.
   - `DipLabSim.Physics`, `DipLabSim.Extensibility`, and external `.dll` controller plugins reference **only** `DipLabSim.Core`.
   - `DipLabSim.Engine` references `DipLabSim.Core`.
   - `DipLabSim.UI` (WPF app) is the Composition Root and ViewModel layer (`CommunityToolkit.Mvvm`).
2. **Coordinate & Angle Conventions (KON-445E Standard):**
   - Generalized coordinates: $q = [x, \theta_1, \theta_2]^T$, where $x$ is horizontal cart position ($\text{m}$), and $\theta_1, \theta_2$ are pendulum angles measured from the **upright vertical axis** ($\text{rad}$, $0 = \text{upright}$).
   - Full 6th-order state vector: $z = [x, \theta_1, \theta_2, \dot{x}, \dot{\theta}_1, \dot{\theta}_2]^T$.
3. **Zero Heap Allocations in Rendering & Simulation Loops:**
   - **Never** instantiate `new SKPaint()`, `new SKPath()`, `new SKFont()`, `List<T>`, or LINQ queries inside `SKElement.PaintSurface`, `IPlant.ComputeDerivatives`, `IOdeSolver.Step`, or `IController.ComputeControlInput`.
   - Pass `PendulumState` and `PendulumStateDerivative` by `in` and `out` reference (`readonly record struct`).

---

## 3. Shared Domain Contracts (`DipLabSim.Core`)

Lock these types in `DipLabSim.Core` so the UI team can build the full Week 1 interface immediately while the Physics, Engine, and Extensibility teams build against the exact same interfaces in upcoming weeks:

```csharp
namespace DipLabSim.Core;

public readonly record struct PendulumState(
    double Time,
    double X,
    double Theta1,
    double Theta2,
    double Dx,
    double DTheta1,
    double DTheta2)
{
    public static PendulumState Zero => new(0, 0, 0, 0, 0, 0, 0);

    public static PendulumState AddScaled(in PendulumState s, in PendulumStateDerivative d, double scale) =>
        new(
            s.Time + scale,
            s.X + d.Dx * scale,
            s.Theta1 + d.DTheta1 * scale,
            s.Theta2 + d.DTheta2 * scale,
            s.Dx + d.Ddx * scale,
            s.DTheta1 + d.DdTheta1 * scale,
            s.DTheta2 + d.DdTheta2 * scale);
}

public readonly record struct PendulumStateDerivative(
    double Dx,
    double DTheta1,
    double DTheta2,
    double Ddx,
    double DdTheta1,
    double DdTheta2);

/// <summary>
/// Self-describing parameter metadata used by the WPF Configuration Tab to auto-generate sliders/inputs.
/// </summary>
public sealed record ParameterDescriptor(
    string Key,
    string DisplayName,
    string Symbol,
    string Unit,
    string Category,
    double DefaultValue,
    double MinValue,
    double MaxValue,
    double Step = 0.01);

public interface IConfigurableComponent
{
    string Name { get; }
    string Description { get; }
    IReadOnlyList<ParameterDescriptor> GetParameterDescriptors();
    double GetParameter(string key);
    void SetParameter(string key, double value);
}

public interface IPlant : IConfigurableComponent
{
    void ComputeDerivatives(
        in PendulumState state,
        double controlForceU,
        double disturbanceForce,
        out PendulumStateDerivative derivative);
}

public interface IOdeSolver
{
    string Name { get; }
    PendulumState Step(
        IPlant plant,
        in PendulumState current,
        double controlForceU,
        double disturbanceForce,
        double dt);
}

public interface IController : IConfigurableComponent, IDisposable
{
    void Initialize(IReadOnlyDictionary<string, double> plantParameters);
    double ComputeControlInput(in PendulumState state, in PendulumState reference, double dt);
    void Reset();
}

public readonly record struct SimulationFrame(
    PendulumState State,
    PendulumState Reference,
    double ControlForceU,
    double DisturbanceForce);
```

---

## 4. Week 1 Implementation Spec: WPF UI, Dynamic Config, Directory Scanner & SkiaSharp (`DipLabSim.UI`)

### 4.1 Minimalist Layout Structure (`MainWindow.xaml`)
Use a clean, modern dark/light laboratory aesthetic with a 3-zone grid:
1. **Top Transport & Selection Bar:**
   - **Plant Model Selector (`ComboBox`):** Switches between *Model 1 (Ideal Euler–Lagrange)* and *Model 2 (Extended Plant Model)*.
   - **Controller Selector (`ComboBox` + Refresh Button):** Lists available `.dll` files discovered in the `./Controllers` folder (plus a `"None / Open-Loop"` and `"External Python (gRPC)"` entry).
   - **Simulation Controls:** `Start`, `Pause`, `Reset`, and a `Simulation Speed` selector (`0.25x`, `0.5x`, `1.0x`, `2.0x`).
2. **Left Collapsible Sidebar (Dynamic Configuration & State Preview Tab):**
   - **Model Parameters Tab:** Data-driven `ItemsControl` bound to `SelectedPlantParameters` (grouped by `Category`: *Masses*, *Geometry*, *Inertias*, *Damping & Friction*). Each item displays `DisplayName (Symbol) [Unit]`, a `Slider`, and a synchronized numeric `TextBox`.
   - **Initial / Manual State Tab (Week 1 Visualizer Test):** Sliders for $x \in [-2, 2]\text{ m}$, $\theta_1 \in [-\pi, \pi]\text{ rad}$, and $\theta_2 \in [-\pi, \pi]\text{ rad}$. Moving these sliders when paused immediately updates `CurrentState` and redraws the SkiaSharp pendulum canvas.
3. **Main Workspace Split (Right Area):**
   - **Top 65% — `SkiaSharp` 2D Visualizer (`SKElement`):** Hardware-accelerated viewport showing the track, cart, Link 1, Link 2, joint pivots, and a live HUD overlay ($t, x, \theta_1, \theta_2, u$).
   - **Bottom 35% — `ScottPlot 5` Real-Time Scope (`WpfPlot`):** Live scrolling time-series plots for Cart Position $x(t)$, Angles $\theta_1(t), \theta_2(t)$, and Control Input $u(t)$.

### 4.2 SkiaSharp Immediate-Mode Visualizer Rules (`PendulumCanvasControl`)
- **Live Geometry Reflection:** Whenever the user edits $L_1$ or $L_2$ (or $l_2$) in the Configuration Tab, the SkiaSharp canvas must immediately scale the rendered rod lengths proportionally on screen.
- **Kinematics in World Coordinates (Meters):**
  - Cart center: $(x, 0)$
  - Joint 1 (top of Link 1): $(x_1, y_1) = (x + L_1 \sin\theta_1,\; L_1 \cos\theta_1)$
  - Tip 2 (top of Link 2): $(x_2, y_2) = (x_1 + L_2 \sin\theta_2,\; y_1 + L_2 \cos\theta_2)$
- **Cached Paint Resources:** Declare all `SKPaint` and `SKFont` objects once as `readonly` fields (e.g., `_trackPaint`, `_cartPaint`, `_link1Paint`, `_link2Paint`, `_jointPaint`, `_trailPaint`).
- **VSync Rendering Hook:** Drive redraws from `CompositionTarget.Rendering` ($60\text{–}144\text{ Hz}$).

### 4.3 Controller `.DLL` Directory Discovery (`ControllerDirectoryScanner`)
- Target directory: `Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Controllers")` (create the directory on startup if it does not exist).
- Use `Directory.EnumerateFiles(controllersDir, "*.dll")` plus a `FileSystemWatcher` (or a UI "Refresh" button) so dropping a new controller `.dll` into the `./Controllers` folder immediately populates the UI `ComboBox` without restarting the app.
- In Week 1, selecting a `.dll` in the `ComboBox` simply stores the selected `ControllerDllEntry(string FileName, string FullPath)` in `MainViewModel`. In upcoming weeks, `DllPluginLoader` will instantiate the `IController` from `FullPath`.

---

## 5. Upcoming Weeks Implementation Spec (Physics, Engine & Extensibility)

### 5.1 Physics & Solver (`DipLabSim.Physics`)
- **Model 1 (`NonlinearEulerLagrangePlant`):** Solves $M(q)\ddot{q} + C(q,\dot{q})\dot{q} + G(q) = Hu$ from KON-445E Report Section 3.6. Precompute $d_1 \dots d_6, f_1, f_2$ inside `SetParameter()` and solve the $3 \times 3$ symmetric system analytically on the stack (no per-step heap allocations).
- **Model 2 (`SecondPlantModel`):** Implements `IPlant` and adds model-specific `ParameterDescriptor` items (e.g., viscous damping $b_0, b_1, b_2$ or actuator constants), which automatically appear in the Week 1 UI Configuration Tab.
- **Integrator (`Rk4Solver`):** Fixed-step 4th-order Runge-Kutta solver at $dt = 1\text{ ms}$, validated against MATLAB reference trajectories ($< 10^{-6}$ error).

### 5.2 Multi-Rate Simulation Engine (`DipLabSim.Engine`)
- Dedicated background thread using `Stopwatch.GetTimestamp()` spin-wait for $dt = 1\text{ ms}$ ($1000\text{ Hz}$) physics ticks.
- Every 5th tick ($T_s = 5\text{ ms} / 200\text{ Hz}$), samples `IController.ComputeControlInput(...)`.
- Supports **Mode A (Deterministic Lockstep)** on the physics thread and **Mode B (Async Real-Time ZOH)** via a background worker decorator.
- Publishes `SimulationFrame` structs to a bounded `Channel<SimulationFrame>` drained by the WPF UI on `CompositionTarget.Rendering`.

### 5.3 DLL Plugin Loader & Python gRPC Bridge (`DipLabSim.Extensibility`)
- **Collectible `.DLL` Loader (`PluginLoadContext : AssemblyLoadContext`):** Loads `.dll` files from `./Controllers` with `isCollectible: true`. Must return `null` when resolving `DipLabSim.Core` so `IController` type identity matches the host application.
- **Python gRPC Adapter (`GrpcControllerAdapter : IController`):** Connects to external Python MPC/RL policies over Named Pipes or `http://127.0.0.1:50051` (`TCP_NODELAY`) using a flat `double`-only `controller.proto` contract (`Initialize`, `ComputeControl`, `Reset`).
