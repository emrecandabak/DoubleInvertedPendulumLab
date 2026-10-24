using DipLabSim.Core;

namespace DipLabSim.Physics;

internal static class OdeMath
{
    public static void Validate(double dt)
    {
        if (!double.IsFinite(dt) || dt <= 0) throw new ArgumentOutOfRangeException(nameof(dt));
    }
    public static void Write(in PendulumState s, Span<double> a)
    { a[0] = s.X; a[1] = s.Theta1; a[2] = s.Theta2; a[3] = s.Dx; a[4] = s.DTheta1; a[5] = s.DTheta2; }
    public static void Write(in PendulumStateDerivative s, Span<double> a)
    { a[0] = s.Dx; a[1] = s.DTheta1; a[2] = s.DTheta2; a[3] = s.Ddx; a[4] = s.DdTheta1; a[5] = s.DdTheta2; }
    public static PendulumState State(ReadOnlySpan<double> a, double time) => new(time, a[0], a[1], a[2], a[3], a[4], a[5]);
    public static PendulumStateDerivative Average(in PendulumStateDerivative a, in PendulumStateDerivative b) =>
        new((a.Dx + b.Dx) / 2, (a.DTheta1 + b.DTheta1) / 2, (a.DTheta2 + b.DTheta2) / 2,
            (a.Ddx + b.Ddx) / 2, (a.DdTheta1 + b.DdTheta1) / 2, (a.DdTheta2 + b.DdTheta2) / 2);
}

public sealed class ForwardEulerSolver : IOdeSolver
{
    public string Name => "Forward Euler (1st order)";
    public PendulumState Step(IPlant plant, in PendulumState current, double u, double force, double dt)
    {
        OdeMath.Validate(dt);
        plant.ComputeDerivatives(current, u, force, out var k);
        return PendulumState.AddScaled(current, k, dt);
    }
}

public sealed class HeunSolver : IOdeSolver
{
    public string Name => "Heun (2nd order)";
    public PendulumState Step(IPlant plant, in PendulumState current, double u, double force, double dt)
    {
        OdeMath.Validate(dt);
        plant.ComputeDerivatives(current, u, force, out var k1);
        var predictor = PendulumState.AddScaled(current, k1, dt);
        plant.ComputeDerivatives(predictor, u, force, out var k2);
        var average = OdeMath.Average(k1, k2);
        return PendulumState.AddScaled(current, average, dt);
    }
}

public sealed class Rk4Solver : IOdeSolver
{
    public string Name => "Runge–Kutta 4 (4th order)";
    public PendulumState Step(IPlant plant, in PendulumState current, double u, double force, double dt)
    {
        OdeMath.Validate(dt);
        plant.ComputeDerivatives(current, u, force, out var k1);
        var s2 = PendulumState.AddScaled(current, k1, dt / 2);
        plant.ComputeDerivatives(s2, u, force, out var k2);
        var s3 = PendulumState.AddScaled(current, k2, dt / 2);
        plant.ComputeDerivatives(s3, u, force, out var k3);
        var s4 = PendulumState.AddScaled(current, k3, dt);
        plant.ComputeDerivatives(s4, u, force, out var k4);
        var k = new PendulumStateDerivative(
            (k1.Dx + 2 * k2.Dx + 2 * k3.Dx + k4.Dx) / 6,
            (k1.DTheta1 + 2 * k2.DTheta1 + 2 * k3.DTheta1 + k4.DTheta1) / 6,
            (k1.DTheta2 + 2 * k2.DTheta2 + 2 * k3.DTheta2 + k4.DTheta2) / 6,
            (k1.Ddx + 2 * k2.Ddx + 2 * k3.Ddx + k4.Ddx) / 6,
            (k1.DdTheta1 + 2 * k2.DdTheta1 + 2 * k3.DdTheta1 + k4.DdTheta1) / 6,
            (k1.DdTheta2 + 2 * k2.DdTheta2 + 2 * k3.DdTheta2 + k4.DdTheta2) / 6);
        return PendulumState.AddScaled(current, k, dt);
    }
}

/// <summary>Solves y - x - h*f(t+h,y)=0 using damped Newton iteration.
/// Failure is explicit; it never silently substitutes an explicit Euler step.</summary>
public sealed class BackwardEulerSolver : IOdeSolver
{
    public string Name => "Backward Euler (implicit, 1st order)";
    public PendulumState Step(IPlant plant, in PendulumState current, double u, double force, double dt)
    {
        OdeMath.Validate(dt);
        Span<double> x = stackalloc double[6], y = stackalloc double[6], r = stackalloc double[6];
        Span<double> work = stackalloc double[6], trial = stackalloc double[6], rhs = stackalloc double[6];
        Span<double> jacobian = stackalloc double[36];
        OdeMath.Write(current, x);
        plant.ComputeDerivatives(current, u, force, out var initial);
        OdeMath.Write(PendulumState.AddScaled(current, initial, dt), y);
        double time = current.Time + dt;
        for (int iteration = 0; iteration < 16; iteration++)
        {
            double norm = Residual(plant, y, x, time, u, force, dt, r);
            if (norm < 1e-10) return OdeMath.State(y, time);
            if (!double.IsFinite(norm)) break;
            for (int j = 0; j < 6; j++)
            {
                double saved = y[j], epsilon = 1e-7 * (1 + Math.Abs(saved));
                y[j] += epsilon;
                Residual(plant, y, x, time, u, force, dt, work);
                y[j] = saved;
                for (int i = 0; i < 6; i++) jacobian[i * 6 + j] = (work[i] - r[i]) / epsilon;
            }
            for (int i = 0; i < 6; i++) rhs[i] = -r[i];
            SmallLinearSystem.Solve(jacobian, rhs, 6);
            bool accepted = false;
            for (double factor = 1; factor >= 1.0 / 128; factor /= 2)
            {
                for (int i = 0; i < 6; i++) trial[i] = y[i] + factor * rhs[i];
                if (Residual(plant, trial, x, time, u, force, dt, work) < norm)
                { trial.CopyTo(y); accepted = true; break; }
            }
            if (!accepted) break;
        }
        throw new ArithmeticException("Backward Euler did not converge. Reduce the time step and reset.");
    }

    private static double Residual(IPlant plant, ReadOnlySpan<double> y, ReadOnlySpan<double> x,
        double time, double u, double force, double h, Span<double> residual)
    {
        var state = OdeMath.State(y, time);
        plant.ComputeDerivatives(state, u, force, out var derivative);
        OdeMath.Write(derivative, residual);
        double norm = 0;
        for (int i = 0; i < 6; i++)
        {
            residual[i] = y[i] - x[i] - h * residual[i];
            norm = Math.Max(norm, Math.Abs(residual[i]) / (1 + Math.Abs(y[i])));
        }
        return norm;
    }
}
