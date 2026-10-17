namespace DipLabSim.UI.Models;

public sealed record ControllerOption(
    string DisplayName,
    string? FullPath,
    bool IsBuiltIn,
    string? Description = null)
{
    public static ControllerOption None { get; } = new(
        DisplayName: "None / Open-Loop (Manual / Uncontrolled)",
        FullPath: null,
        IsBuiltIn: true,
        Description: "No active feedback controller. System evolves freely under manual input or test preview.");

    public static ControllerOption ExternalPythonGrpc { get; } = new(
        DisplayName: "External Python (gRPC) [127.0.0.1:50051]",
        FullPath: null,
        IsBuiltIn: true,
        Description: "Streams state frames and receives control inputs from an external Python RL/MPC server.");

    public override string ToString() => DisplayName;
}
