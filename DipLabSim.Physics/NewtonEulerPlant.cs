using DipLabSim.Core;

namespace DipLabSim.Physics;

/// <summary>Independent force / COM moment balances. Joint reactions are recovered
/// from COM accelerations, then eliminated by solving three affine residuals.</summary>
public sealed class NewtonEulerPlant : DoublePendulumPlant
{
    public override string Name => "Model 2: Newton–Euler";
    public override string Description => "Independent Newton force and COM moment balances; same plant and parameters as Model 1.";

    private void Residual(in PendulumState s, double u, double force,
        double ax, double alpha1, double alpha2, Span<double> residual)
    {
        double s1 = Math.Sin(s.Theta1), c1 = Math.Cos(s.Theta1);
        double s2 = Math.Sin(s.Theta2), c2 = Math.Cos(s.Theta2);
        double w1sq = s.DTheta1 * s.DTheta1, w2sq = s.DTheta2 * s.DTheta2;
        double f1 = DisturbanceTarget == DisturbanceTarget.LowerLinkTip ? force : 0;
        double f2 = DisturbanceTarget == DisturbanceTarget.UpperLinkTip ? force : 0;
        double a1x = ax + R1 * (c1 * alpha1 - s1 * w1sq);
        double a1y = -R1 * (s1 * alpha1 + c1 * w1sq);
        double a2x = ax + L1 * (c1 * alpha1 - s1 * w1sq) + R2 * (c2 * alpha2 - s2 * w2sq);
        double a2y = -L1 * (s1 * alpha1 + c1 * w1sq) - R2 * (s2 * alpha2 + c2 * w2sq);
        // R: lower link on upper link; P: cart on lower link.
        double rx = M2 * a2x - f2, ry = M2 * (a2y + Gravity);
        double px = M1 * a1x + rx - f1, py = M1 * (a1y + Gravity) + ry;
        double jointDamping = C2 * (s.DTheta2 - s.DTheta1);
        residual[0] = M * ax + px - u + B * s.Dx;
        residual[1] = J1 * alpha1 + R1 * (c1 * px - s1 * py)
            + (L1 - R1) * (c1 * rx - s1 * ry - c1 * f1)
            + C1 * s.DTheta1 - jointDamping;
        residual[2] = J2 * alpha2 + R2 * (c2 * rx - s2 * ry)
            - (L2 - R2) * c2 * f2 + jointDamping;
    }

    public override void ComputeDerivatives(in PendulumState s, double u, double force,
        out PendulumStateDerivative derivative)
    {
        Span<double> baseline = stackalloc double[3];
        Span<double> column = stackalloc double[3];
        Span<double> matrix = stackalloc double[9];
        Span<double> rhs = stackalloc double[3];
        Residual(s, u, force, 0, 0, 0, baseline);
        for (int j = 0; j < 3; j++)
        {
            Residual(s, u, force, j == 0 ? 1 : 0, j == 1 ? 1 : 0, j == 2 ? 1 : 0, column);
            for (int i = 0; i < 3; i++) matrix[i * 3 + j] = column[i] - baseline[i];
        }
        for (int i = 0; i < 3; i++) rhs[i] = -baseline[i];
        SmallLinearSystem.Solve(matrix, rhs, 3);
        derivative = new(s.Dx, s.DTheta1, s.DTheta2, rhs[0], rhs[1], rhs[2]);
    }
}
