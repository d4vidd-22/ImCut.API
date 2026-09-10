#pragma once

#include <cmath>
#include <limits>

namespace ImCut::Geometry::Certified
{
    // OUTWARD-ROUNDED ARITHMETIC, in the smallest form that can be audited by reading it.
    //
    // WHY THIS EXISTS. DISTANCE_INTERVAL_PROOF.md proves, over the reals, that a curve lies
    // inside its exact bounds and therefore that the gap between two such boxes is a true
    // lower bound. The implementation computed those boxes in `double`, and F31 measured
    // what that costs: 4675 escapes in 391 374 samples, worst 2.00 ULP - points OF the
    // curve falling OUTSIDE the box that claimed to contain it (evidence 1156). A lower
    // bound derived from a box that is too tight is a lower bound that is too high, and
    // that is the direction that produces a DistanceLessThan false negative.
    //
    // Section 11 of the continuation prompt is explicit about the shape of the fix: one
    // small primitive that rounds outward, not `nextafter` sprinkled through the solver.
    //
    // THE GUARANTEE. For every operation here, the exact real result lies inside the
    // returned interval. Nothing else is claimed - these are not tight intervals, they are
    // honest ones.

    // 2^-51. Half of one ULP is the error of a single correctly-rounded operation, so 2^-52
    // would already suffice; this is deliberately one bit looser.
    //
    // THE MARGIN, WRITTEN OUT, because "roughly one ULP" is how F31 happened. Let x be the
    // computed result of an exact real r, so |x - r| <= 0.5 * ulp(x). For a normal x,
    // ulp(x) <= |x| * 2^-52, hence d = |x| * 2^-51 >= 2 * ulp(x). The widening subtraction
    // x - d is itself rounded and may land up to 0.5 * ulp above the exact x - d, giving at
    // worst x - 2*ulp + 0.5*ulp = x - 1.5*ulp, which is still strictly below
    // r >= x - 0.5*ulp. The same argument mirrors for Up().
    inline constexpr double kRelativeWiden = 4.440892098500626e-16;

    // Smallest positive subnormal. Carries the argument through zero and the subnormal
    // range, where the relative term vanishes and would otherwise widen by nothing.
    inline constexpr double kAbsoluteWiden = 4.9406564584124654e-324;

    [[nodiscard]] inline double Down(double x) noexcept
    {
        if (!(x > -std::numeric_limits<double>::infinity()))
            return x;                                    // -inf and NaN pass through
        return x - (std::fabs(x) * kRelativeWiden + kAbsoluteWiden);
    }

    [[nodiscard]] inline double Up(double x) noexcept
    {
        if (!(x < std::numeric_limits<double>::infinity()))
            return x;                                    // +inf and NaN pass through
        return x + (std::fabs(x) * kRelativeWiden + kAbsoluteWiden);
    }

    // WIDENING BY A COUNTED CHAIN OF OPERATIONS.
    //
    // Down()/Up() cover ONE rounded operation. A short fixed chain of `operations`
    // elementary operations on values of magnitude |x| accumulates at most
    // operations * 0.5 * ulp, so widening by operations * 2^-51 * |x| carries a factor of
    // four in hand.
    //
    // This exists so that a quantity produced by an existing double routine can be made
    // sound by COUNTING that routine's operations, rather than by fitting a constant to
    // whatever error a test happened to observe - which is the thing F31 forbids. Every
    // call site must say which routine it counted and how many.
    [[nodiscard]] inline double WidenDown(double x, int operations) noexcept
    {
        if (!(x > -std::numeric_limits<double>::infinity()))
            return x;
        return x - (std::fabs(x) * (kRelativeWiden * static_cast<double>(operations))
                    + kAbsoluteWiden);
    }

    [[nodiscard]] inline double WidenUp(double x, int operations) noexcept
    {
        if (!(x < std::numeric_limits<double>::infinity()))
            return x;
        return x + (std::fabs(x) * (kRelativeWiden * static_cast<double>(operations))
                    + kAbsoluteWiden);
    }

    // A closed interval that is known to contain the real quantity it stands for.
    struct Interval
    {
        double lo = 0.0;
        double hi = 0.0;

        [[nodiscard]] constexpr double Midpoint() const noexcept { return (lo + hi) * 0.5; }
        [[nodiscard]] constexpr double HalfWidth() const noexcept { return (hi - lo) * 0.5; }
    };

    // A double IS its exact value. Nothing is widened here: the stored coordinate is the
    // geometry, and asking whether the user meant decimal 0.1 belongs to import and
    // provenance, not to this file.
    [[nodiscard]] inline constexpr Interval Exactly(double v) noexcept { return { v, v }; }

    [[nodiscard]] inline Interval Add(Interval a, Interval b) noexcept
    {
        return { Down(a.lo + b.lo), Up(a.hi + b.hi) };
    }

    [[nodiscard]] inline Interval Sub(Interval a, Interval b) noexcept
    {
        return { Down(a.lo - b.hi), Up(a.hi - b.lo) };
    }

    // Multiplication by an EXACT scalar. Restricted to that case on purpose: de Casteljau
    // needs nothing else, and a general interval product needs a four-way sign analysis
    // that would be four more chances to be subtly wrong.
    [[nodiscard]] inline Interval Scale(Interval a, double t) noexcept
    {
        const double p = a.lo * t;
        const double q = a.hi * t;
        return { Down(p < q ? p : q), Up(p > q ? p : q) };
    }

    // The single operation the blossom is built from.
    [[nodiscard]] inline Interval Lerp(Interval a, Interval b, double t) noexcept
    {
        return Add(a, Scale(Sub(b, a), t));
    }

    [[nodiscard]] inline Interval Hull(Interval a, Interval b) noexcept
    {
        return { a.lo < b.lo ? a.lo : b.lo, a.hi > b.hi ? a.hi : b.hi };
    }

    // sqrt of an interval of NON-NEGATIVE reals. A negative lower end is clamped to zero:
    // it can only come from rounding, and a squared distance is never really negative.
    [[nodiscard]] inline Interval Sqrt(Interval a) noexcept
    {
        const double lo = a.lo > 0.0 ? Down(std::sqrt(a.lo)) : 0.0;
        const double hi = a.hi > 0.0 ? Up(std::sqrt(a.hi)) : 0.0;
        return { lo < 0.0 ? 0.0 : lo, hi };
    }
}
