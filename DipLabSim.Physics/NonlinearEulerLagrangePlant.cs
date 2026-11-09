using DipLabSim.Core;

namespace DipLabSim.Physics;

public sealed class NonlinearEulerLagrangePlant : DoublePendulumPlant
{
    public override string Name => "Model 1: Euler–Lagrange";
    public override string Description => "Nonlinear rigid-body energy formulation; absolute angles from upright.";

    public override void ComputeDerivatives(in PendulumState s, double u, double force,
        out PendulumStateDerivative derivative)
    {
        double c1 = Math.Cos(s.Theta1), c2 = Math.Cos(s.Theta2);
        double delta = s.Theta1 - s.Theta2, sinDelta = Math.Sin(delta);
        double a = D1, b = D2 * c1, c = D3 * c2;
        double d = D4, e = D5 * Math.Cos(delta), f = D6;
        // J_tip(q)^T F: an upper-tip push also acts through the lower link.
        double tau1 = force * L1 * c1;
        double tau2 = DisturbanceTarget == DisturbanceTarget.UpperLinkTip ? force * L2 * c2 : 0;
        double relativeDamping = C2 * (s.DTheta2 - s.DTheta1);
        double r0 = u + force - B * s.Dx
            + D2 * Math.Sin(s.Theta1) * s.DTheta1 * s.DTheta1
            + D3 * Math.Sin(s.Theta2) * s.DTheta2 * s.DTheta2;
        double r1 = tau1 - C1 * s.DTheta1 + relativeDamping
            - D5 * sinDelta * s.DTheta2 * s.DTheta2 + D2 * Gravity * Math.Sin(s.Theta1);
        double r2 = tau2 - relativeDamping
            + D5 * sinDelta * s.DTheta1 * s.DTheta1 + D3 * Gravity * Math.Sin(s.Theta2);
        // Symmetric 3x3 adjugate, evaluated without allocating a matrix.
        double a11 = d * f - e * e, a12 = c * e - b * f, a13 = b * e - c * d;
        double a22 = a * f - c * c, a23 = b * c - a * e, a33 = a * d - b * b;
        double determinant = a * a11 + b * a12 + c * a13;
        if (!(determinant > 0) || !double.IsFinite(determinant))
            throw new ArithmeticException("Invalid mass matrix. Check physical parameters.");
        derivative = new(s.Dx, s.DTheta1, s.DTheta2,
            (a11 * r0 + a12 * r1 + a13 * r2) / determinant,
            (a12 * r0 + a22 * r1 + a23 * r2) / determinant,
            (a13 * r0 + a23 * r1 + a33 * r2) / determinant);
    }
}
