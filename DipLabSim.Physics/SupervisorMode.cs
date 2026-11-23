namespace DipLabSim.Physics;

/// <summary>Operating mode of <see cref="SwingUpSupervisorController"/>.</summary>
public enum SupervisorMode
{
    /// <summary>The nonlinear swing-up law acts.</summary>
    SwingUp,
    /// <summary>The Lyapunov pre-gate passed; the predictive capture test runs this tick
    /// while the swing-up law still acts.</summary>
    Capture,
    /// <summary>The inner LQR regulates around the upright reference.</summary>
    Balance
}
