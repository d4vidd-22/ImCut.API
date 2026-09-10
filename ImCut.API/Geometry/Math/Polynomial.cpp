#include "Polynomial.hpp"

#include <algorithm>
#include <cmath>

namespace ImCut::Geometry::Polynomial
{
    namespace
    {
        // One Newton step against the original coefficients, used to recover the
        // digits lost by the closed-form solution. Guarded so a flat derivative can
        // never throw the root somewhere worse.
        void PolishCubic(double a, double b, double c, double d, double& t) noexcept
        {
            for (int iteration = 0; iteration < 2; ++iteration)
            {
                const double value = ((a * t + b) * t + c) * t + d;
                const double slope = (3.0 * a * t + 2.0 * b) * t + c;
                if (slope == 0.0 || !std::isfinite(slope))
                    return;

                const double step = value / slope;
                if (!std::isfinite(step))
                    return;

                const double candidate = t - step;
                const double candidateValue = ((a * candidate + b) * candidate + c) * candidate + d;
                if (std::fabs(candidateValue) >= std::fabs(value))
                    return;

                t = candidate;
            }
        }

        [[nodiscard]] int Deduplicate(double* roots, int count, double epsilon) noexcept
        {
            if (count < 2)
                return count;

            std::sort(roots, roots + count);

            int unique = 1;
            for (int i = 1; i < count; ++i)
            {
                if (std::fabs(roots[i] - roots[unique - 1]) > epsilon)
                    roots[unique++] = roots[i];
            }
            return unique;
        }
    }

    int SolveQuadratic(double a, double b, double c, double roots[2], double epsilon) noexcept
    {
        // Degenerate leading coefficient: fall back to the linear equation instead of
        // dividing by something indistinguishable from zero.
        //
        // The test is RELATIVE to the size of the coefficients. It used to compare
        // |a| against an absolute 1e-12, which makes the answer depend on the units the
        // caller happened to use: the same curve expressed in metres rather than
        // millimetres scales every coefficient by 1e-3 or 1e-9 and can cross the
        // threshold without any change of shape. A quadratic is degenerate when its
        // leading term is negligible NEXT TO ITS OTHER TERMS, which is a ratio and has
        // no units.
        const double magnitude = std::max({ std::fabs(a), std::fabs(b), std::fabs(c) });
        const double degenerate = magnitude > 0.0 ? epsilon * magnitude : epsilon;

        if (std::fabs(a) <= degenerate)
        {
            if (std::fabs(b) <= degenerate)
                return 0;
            roots[0] = -c / b;
            return std::isfinite(roots[0]) ? 1 : 0;
        }

        const double discriminant = b * b - 4.0 * a * c;
        if (discriminant < 0.0)
            return 0;

        if (discriminant == 0.0)
        {
            roots[0] = -b / (2.0 * a);
            return std::isfinite(roots[0]) ? 1 : 0;
        }

        // Citardauq form: computing the larger-magnitude root first and deriving the
        // other from the product avoids cancellation when b*b dominates 4ac.
        const double root = std::sqrt(discriminant);
        const double q = -0.5 * (b + (b >= 0.0 ? root : -root));

        int count = 0;
        double candidates[2];
        if (q != 0.0)
        {
            candidates[count++] = q / a;
            candidates[count++] = c / q;
        }
        else
        {
            candidates[count++] = 0.0;
            candidates[count++] = -b / a;
        }

        int written = 0;
        for (int i = 0; i < count; ++i)
        {
            if (std::isfinite(candidates[i]))
                roots[written++] = candidates[i];
        }

        return Deduplicate(roots, written, epsilon);
    }

    int SolveCubic(double a, double b, double c, double d, double roots[3], double epsilon) noexcept
    {
        if (std::fabs(a) <= epsilon)
            return SolveQuadratic(b, c, d, roots, epsilon);

        // Depress to t^3 + p*t + q by substituting t = x - shift.
        const double inverseA = 1.0 / a;
        const double b2 = b * inverseA;
        const double c2 = c * inverseA;
        const double d2 = d * inverseA;

        const double shift = b2 / 3.0;
        const double p = c2 - b2 * shift;
        const double q = d2 - shift * (c2 - 2.0 * b2 * b2 / 9.0);

        const double halfQ = 0.5 * q;
        const double thirdP = p / 3.0;
        const double discriminant = halfQ * halfQ + thirdP * thirdP * thirdP;

        int count = 0;

        if (discriminant > 0.0)
        {
            // One real root: plain Cardano.
            const double root = std::sqrt(discriminant);
            const double u = std::cbrt(-halfQ + root);
            const double v = std::cbrt(-halfQ - root);
            roots[count++] = u + v - shift;
        }
        else if (discriminant == 0.0)
        {
            const double u = std::cbrt(-halfQ);
            roots[count++] = 2.0 * u - shift;
            roots[count++] = -u - shift;
        }
        else
        {
            // Three distinct real roots. Cardano would need complex arithmetic here,
            // so use the trigonometric form, which is also better conditioned.
            const double magnitude = std::sqrt(-thirdP);
            double cosine = -halfQ / (magnitude * magnitude * magnitude);
            cosine = std::clamp(cosine, -1.0, 1.0);

            const double angle = std::acos(cosine) / 3.0;
            constexpr double kTwoThirdsPi = 2.0943951023931954923;

            roots[count++] = 2.0 * magnitude * std::cos(angle) - shift;
            roots[count++] = 2.0 * magnitude * std::cos(angle - kTwoThirdsPi) - shift;
            roots[count++] = 2.0 * magnitude * std::cos(angle + kTwoThirdsPi) - shift;
        }

        int written = 0;
        for (int i = 0; i < count; ++i)
        {
            if (!std::isfinite(roots[i]))
                continue;
            PolishCubic(a, b, c, d, roots[i]);
            if (std::isfinite(roots[i]))
                roots[written++] = roots[i];
        }

        return Deduplicate(roots, written, epsilon);
    }

    int ClampToInterval(double* roots, int count, double low, double high, double epsilon) noexcept
    {
        int written = 0;
        for (int i = 0; i < count; ++i)
        {
            double value = roots[i];
            if (value < low)
            {
                if (value < low - epsilon) continue;
                value = low;
            }
            if (value > high)
            {
                if (value > high + epsilon) continue;
                value = high;
            }
            roots[written++] = value;
        }
        return Deduplicate(roots, written, epsilon);
    }
}
