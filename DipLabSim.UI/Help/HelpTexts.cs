namespace DipLabSim.UI.Help;

/// <summary>
/// Plain-language explanations for the fixed settings of the main window, shown by the "?" badges.
/// Parameter-specific help comes from <c>ParameterDescriptor.Help</c> of each plant or controller.
/// </summary>
public static class HelpTexts
{
    public const string Plant =
        "Which set of equations of motion is simulated. Model 1 (Euler–Lagrange, energy method) and Model 2 "
        + "(Newton–Euler, force and moment balances) are two independent derivations of the same cart with two "
        + "links and use the same parameters, so they should produce the same motion. Comparing them checks the "
        + "equations. Switching keeps all parameter values.";

    public const string ControllerIntro =
        "The feedback law that computes the cart force u at every physics step. "
        + "Only editable while paused. The Controller tab shows the parameters of the selected one.";

    public const string Solver =
        "Numerical method that advances the equations by one step dt.\n"
        + "• Forward Euler (1st order): simplest; an undamped swing slowly gains energy, and it can blow up with a large dt.\n"
        + "• Heun (2nd order): a predictor-corrector, much more accurate than Euler.\n"
        + "• Runge–Kutta 4 (4th order): recommended default; very accurate at dt = 1 ms.\n"
        + "• Backward Euler (implicit, 1st order): very stable, but adds artificial damping, so an undamped swing "
        + "slowly loses energy.\n"
        + "Try it: choose None (Open-Loop), set θ₁ = θ₂ = 2.5 rad, and compare how the swing amplitude changes "
        + "with each solver, especially at dt = 5 ms.";

    public const string TimeStep =
        "Physics step size dt. Smaller: more accurate but more computation. Larger: less accurate and, with "
        + "explicit solvers, possibly unstable. The controller is also evaluated every step, so dt is its sample "
        + "time as well.";

    public const string Speed =
        "Playback speed relative to real time. It changes only how fast the motion is shown; the physics uses "
        + "the same fixed step dt, so the results are identical.";

    public const string PlayReset =
        "Play / Pause starts and stops the simulation. Plant, controller, solver, dt and initial state can only be "
        + "changed while paused; speed and the Forces tab also work while running. "
        + "Reset returns to the positions in the Initial State tab with all velocities zero.";

    public const string ResetPlantParameters =
        "Restores all plant parameters to their default values (and re-designs the selected controller for them).";

    public const string PlantParameters =
        "Physical parameters of the cart and the two links. Hover over a \"?\" for what each one changes. "
        + "Editing a value while paused re-designs the selected controller for the new plant.";

    public const string AutoCalculate =
        "Re-tunes the selected controller for the current plant parameters. LQR: scales the Q weights with the "
        + "masses and link lengths (R = 1) and re-solves the Riccati equation. Cascade PID: re-derives all loop gains "
        + "from an LQR design. Swing-Up Supervisor: re-tunes its inner LQR and recomputes V and c_sat.";

    public const string ResetControllerDefaults =
        "Restores the selected controller's default parameter values. These defaults were designed for the default "
        + "plant. LQR and the Swing-Up Supervisor re-solve their gains for the current plant, but Cascade PID's "
        + "default gains may not balance a modified plant: press Auto-Calculate after resetting.";

    public const string GainsSummary =
        "Live summary of the selected controller. LQR: the gain vector K in u = −K·z, in the order "
        + "[x, θ₁, θ₂, ẋ, θ̇₁, θ̇₂]. Cascade PID: the P and D gains of the three loops. Swing-Up Supervisor: the "
        + "current mode (SwingUp / Capture / Balance), V/c_sat (distance from upright; below 1 the LQR force "
        + "stays within its limit), c_sat and K.";

    public const string InitialState =
        "Starting position used by Reset and by the presets; velocities always start at zero. Angles are measured "
        + "from upright, positive clockwise (the tip moves toward +x). Both angles are absolute: θ₂ is the upper "
        + "link's angle from vertical, not relative to the lower link.";

    public const string Presets =
        "Upright (0, 0): the unstable equilibrium. In open loop it stays only while nothing disturbs it; "
        + "the smallest push makes it fall. Balancing controllers hold it.\n"
        + "Hanging (π, π): the stable equilibrium at the bottom. LQR and Cascade PID cannot recover from here; "
        + "use the Swing-Up Supervisor.\n"
        + "Perturbed (0.05, −0.03): a small tilt for testing balancing controllers.\n"
        + "Zero All: identical to Upright.";

    public const string CartPosition =
        "Initial cart position. The track ends at ±1.8 m; the cart stops there and its velocity is set to zero.";

    public const string Theta1 =
        "Initial angle of the lower link from upright. 0 = upright, ±π = hanging down.";

    public const string Theta2 =
        "Initial angle of the upper link from upright (absolute, not relative to the lower link). "
        + "0 = upright, ±π = hanging down.";

    public const string CartForce =
        "Size of the force used by the Cart ← / → and Pulse Cart buttons. It is added to the controller's force "
        + "and the total is clipped to ±100 N.";

    public const string ContinuousForce =
        "Applies the cart force continuously: a constant disturbance. It stays on until Stop force or Reset, and is "
        + "also cleared when you change the plant, controller, solver or dt. Watch where the cart settles: a controller "
        + "without integral action keeps the links upright with the cart offset from x = 0, and the offset grows with "
        + "the force. With the default plant and LQR, from about 6 N the cart ends pinned at the rail (1.8 m) with the "
        + "links leaning.";

    public const string CartPulse =
        "A short force pulse on the cart (only while running). Its effect depends on the impulse = force × "
        + "duration.";

    public const string PulseDuration =
        "How long a cart pulse lasts (0.001–5 s). Rounded up to whole physics steps. A longer pulse with the same "
        + "force gives a larger impulse.";

    public const string DisturbanceTarget =
        "Where the push acts: a horizontal force at the tip of the lower link or of the upper link. An upper-tip "
        + "push acts through both links with a longer lever arm and is usually harder to reject.";

    public const string PushMagnitude =
        "Horizontal force of the link push (0–30 N). Try to find the largest push each controller can reject.";

    public const string PushDuration =
        "How long the link push lasts. A longer push with the same force gives a larger impulse.";
}
