namespace DipLabSim.Core;

// Horizontal force at the distal end of the selected link; positive = world right.
public enum DisturbanceTarget { LowerLinkTip, UpperLinkTip }

// Additive contract: existing IPlant / IOdeSolver implementations remain compatible.
public interface ITargetedDisturbancePlant : IPlant
{
    DisturbanceTarget DisturbanceTarget { get; set; }
}
