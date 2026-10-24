namespace DipLabSim.Physics;

internal static class SmallLinearSystem
{
    // Destructive Gaussian elimination with partial pivoting; caller owns stack buffers.
    public static void Solve(Span<double> a, Span<double> b, int n)
    {
        for (int k = 0; k < n; k++)
        {
            int pivot = k;
            for (int i = k + 1; i < n; i++)
                if (Math.Abs(a[i * n + k]) > Math.Abs(a[pivot * n + k])) pivot = i;
            if (!double.IsFinite(a[pivot * n + k]) || Math.Abs(a[pivot * n + k]) < 1e-14)
                throw new ArithmeticException("Singular numerical system.");
            if (pivot != k)
            {
                for (int j = k; j < n; j++)
                    (a[k * n + j], a[pivot * n + j]) = (a[pivot * n + j], a[k * n + j]);
                (b[k], b[pivot]) = (b[pivot], b[k]);
            }
            for (int i = k + 1; i < n; i++)
            {
                double ratio = a[i * n + k] / a[k * n + k];
                for (int j = k + 1; j < n; j++) a[i * n + j] -= ratio * a[k * n + j];
                b[i] -= ratio * b[k];
            }
        }
        for (int i = n - 1; i >= 0; i--)
        {
            for (int j = i + 1; j < n; j++) b[i] -= a[i * n + j] * b[j];
            b[i] /= a[i * n + i];
        }
    }
}
