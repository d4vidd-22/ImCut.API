#include "CubicBezier.hpp"

#include "../Math/CertifiedInterval.hpp"
#include "../Math/Polynomial.hpp"

#include <algorithm>
#include <cmath>

namespace ImCut::Geometry::CubicBezier
{
    namespace
    {
        // Coefficients of dB/dt written in the power basis, divided by 3:
        //   B'(t)/3 = q2*t^2 + q1*t + q0
        struct DerivativeCoefficients
        {
            Vec2 q2;
            Vec2 q1;
            Vec2 q0;
        };

        [[nodiscard]] DerivativeCoefficients Coefficients(const Segment& segment) noexcept
        {
            const Vec2 a = segment.c1 - segment.p0;
            const Vec2 b = segment.c2 - segment.c1;
            const Vec2 c = segment.p1 - segment.c2;

            return { a - b * 2.0 + c, (b - a) * 2.0, a };
        }
    }

    Vec2 UnitTangent(const Segment& segment, double t) noexcept
    {
        const Vec2 derivative = Derivative(segment, t);
        const double lengthSquared = LengthSquared(derivative);
        if (lengthSquared > 0.0)
            return derivative * (1.0 / std::sqrt(lengthSquared));

        // Parametric speed vanished (cusp or coincident handles). The chord still
        // carries the direction the caller actually wants.
        const Vec2 chord = segment.p1 - segment.p0;
        return Normalized(chord);
    }

    void Split(const Segment& segment, double t, Segment& left, Segment& right) noexcept
    {
        const Vec2 p01 = Lerp(segment.p0, segment.c1, t);
        const Vec2 p12 = Lerp(segment.c1, segment.c2, t);
        const Vec2 p23 = Lerp(segment.c2, segment.p1, t);

        const Vec2 p012 = Lerp(p01, p12, t);
        const Vec2 p123 = Lerp(p12, p23, t);

        const Vec2 mid = Lerp(p012, p123, t);

        left = { segment.p0, p01, p012, mid, segment.kind };
        right = { mid, p123, p23, segment.p1, segment.kind };
    }

    Segment SubCurve(const Segment& segment, double t0, double t1) noexcept
    {
        if (t1 < t0)
            std::swap(t0, t1);

        t0 = std::clamp(t0, 0.0, 1.0);
        t1 = std::clamp(t1, 0.0, 1.0);

        Segment head, tail;

        if (t0 <= 0.0)
        {
            Split(segment, t1, head, tail);
            return head;
        }

        Split(segment, t0, head, tail);
        if (t0 >= 1.0)
            return tail;

        // Re-map t1 into the trailing piece's own parameter space.
        const double remapped = (t1 - t0) / (1.0 - t0);
        Segment sub, rest;
        Split(tail, std::clamp(remapped, 0.0, 1.0), sub, rest);
        return sub;
    }

    int Extrema(const Segment& segment, double out[4]) noexcept
    {
        const DerivativeCoefficients coefficients = Coefficients(segment);

        double roots[4];
        int count = 0;

        double axis[2];
        int found = Polynomial::SolveQuadratic(coefficients.q2.x, coefficients.q1.x, coefficients.q0.x, axis);
        for (int i = 0; i < found; ++i)
            roots[count++] = axis[i];

        found = Polynomial::SolveQuadratic(coefficients.q2.y, coefficients.q1.y, coefficients.q0.y, axis);
        for (int i = 0; i < found; ++i)
            roots[count++] = axis[i];

        // Endpoints are handled separately by every caller, so only interior extrema
        // are reported. The strict bounds keep t=0 and t=1 out.
        int written = 0;
        for (int i = 0; i < count; ++i)
        {
            if (roots[i] > 0.0 && roots[i] < 1.0 && std::isfinite(roots[i]))
                out[written++] = roots[i];
        }

        std::sort(out, out + written);

        int unique = 0;
        for (int i = 0; i < written; ++i)
        {
            if (unique == 0 || std::fabs(out[i] - out[unique - 1]) > 1e-12)
                out[unique++] = out[i];
        }
        return unique;
    }

    int Inflections(const Segment& segment, double out[2]) noexcept
    {
        // Curvature vanishes where cross(B', B'') = 0. Substituting the power-basis
        // derivative coefficients reduces that to a quadratic in t:
        //   cross(q1,q2) t^2 + 2 cross(q0,q2) t + cross(q0,q1) = 0
        const DerivativeCoefficients coefficients = Coefficients(segment);

        const double a = Cross(coefficients.q1, coefficients.q2);
        const double b = 2.0 * Cross(coefficients.q0, coefficients.q2);
        const double c = Cross(coefficients.q0, coefficients.q1);

        double roots[2];
        const int count = Polynomial::SolveQuadratic(a, b, c, roots);

        int written = 0;
        for (int i = 0; i < count; ++i)
        {
            if (roots[i] > 0.0 && roots[i] < 1.0 && std::isfinite(roots[i]))
                out[written++] = roots[i];
        }

        std::sort(out, out + written);
        return written;
    }

    Bounds2 ExactBounds(const Segment& segment) noexcept
    {
        Bounds2 bounds;
        bounds.Add(segment.p0);
        bounds.Add(segment.p1);

        // A straight segment stored in cubic form has its handles on the chord, so
        // the endpoints already bound it and the root solve is pure overhead.
        if (segment.kind == SegmentKind::Line)
            return bounds;

        double roots[4];
        const int count = Extrema(segment, roots);
        for (int i = 0; i < count; ++i)
            bounds.Add(Evaluate(segment, roots[i]));

        return bounds;
    }

    Bounds2 CertifiedExactBounds(const Segment& segment) noexcept
    {
        // A straight segment stored in cubic form has its handles on the chord, so the
        // endpoints bound it - but the endpoints still have to be certified boxes, because
        // the transform that produced them rounded.
        if (segment.kind == SegmentKind::Line)
        {
            Bounds2 bounds;
            bounds.Add(CertifiedPoint(segment, 0.0));
            bounds.Add(CertifiedPoint(segment, 1.0));
            return bounds;
        }

        // The roots are CUT POINTS here, not answers. Any partition of [0,1] gives a
        // sound union; these particular cuts are the ones that make each piece monotone,
        // which is what keeps the result as tight as the nominal box.
        double cuts[6];
        int count = 0;
        cuts[count++] = 0.0;

        double roots[4];
        const int found = Extrema(segment, roots);
        for (int i = 0; i < found; ++i)
        {
            if (roots[i] > 0.0 && roots[i] < 1.0)
                cuts[count++] = roots[i];
        }
        cuts[count++] = 1.0;
        std::sort(cuts, cuts + count);

        Bounds2 bounds;
        for (int i = 0; i + 1 < count; ++i)
        {
            // Equal cuts happen when a root lands on an endpoint or two roots coincide.
            // An empty parameter interval contributes nothing and asking for it would
            // only produce a degenerate piece.
            if (!(cuts[i] < cuts[i + 1]))
                continue;
            bounds.Add(CertifiedSubCurve(segment, cuts[i], cuts[i + 1]).box);
        }
        return bounds;
    }

    namespace
    {
        // THE COST OF DOING THIS THE OBVIOUS WAY, MEASURED.
        //
        // The first implementation carried a Certified::Interval through every step of the
        // blossom, so every lerp paid six outward roundings. It is correct and it is what
        // the primitive is for - and it cost 16.3 ms against 3.4 ms on the prepared
        // MinimumDistance benchmark. Removing the roundings (unsound, as an experiment)
        // brought it to 4.6 ms, which says the structure costs 36% and the per-operation
        // rounding costs the other 280%.
        //
        // So the rounding happens ONCE per output instead of six times per step, and the
        // bound it uses is counted rather than accumulated. Two facts make that sound:
        //
        //   1. EVERY intermediate of the blossom is a CONVEX COMBINATION of the original
        //      control points, because every level is a lerp with a parameter in [0,1].
        //      So no intermediate leaves [min P, max P] on its axis, and a single number -
        //      the largest |control point| on that axis - bounds them all. This is what
        //      makes an ABSOLUTE error bound possible; a relative one would be wrong
        //      exactly where it matters, on a piece whose coordinates cancel to near zero.
        //
        //   2. A lerp does not AMPLIFY the error of its inputs: it is a convex combination,
        //      so the output error is at most the larger input error plus its own rounding.
        //      Three levels of three rounded operations therefore give at most
        //      9 * 0.5 * ulp(2*scale) <= 9 * 2^-52 * scale.
        //
        // kBlossomOps * kRelativeWiden = 32 * 2^-51 leaves a factor of seven in hand over
        // that. The number is derived from the operation count, not fitted to an observed
        // escape - which is the distinction F31 turns on.
        //
        // The per-operation interval form is not gone: TestHardeningV812NumericCertification
        // rebuilds it from Certified::Interval and asserts that the box below CONTAINS the
        // one that form produces. The slow, obviously-correct version is the oracle for the
        // fast one.
        inline constexpr int kBlossomOps = 32;

        [[nodiscard]] inline double AxisScale(double a, double b, double c, double d) noexcept
        {
            double s = std::fabs(a);
            const double sb = std::fabs(b);
            const double sc = std::fabs(c);
            const double sd = std::fabs(d);
            if (sb > s) s = sb;
            if (sc > s) s = sc;
            if (sd > s) s = sd;
            return s;
        }

        // The uncertainty of any blossom or de Casteljau output on an axis of this scale.
        [[nodiscard]] inline double AxisUncertainty(double scale) noexcept
        {
            return scale * (Certified::kRelativeWiden * static_cast<double>(kBlossomOps))
                   + Certified::kAbsoluteWiden;
        }

        // The four control values of the restriction to [t0, t1], on one axis.
        //
        //   Q0 = f(t0,t0,t0)   Q1 = f(t0,t0,t1)   Q2 = f(t0,t1,t1)   Q3 = f(t1,t1,t1)
        //
        // Three levels, always, from the ORIGINAL control values - never from a subcurve of
        // a subcurve. That is the entire content of the F31 fix.
        struct AxisBlossom { double q[4]; };

        [[nodiscard]] AxisBlossom Blossom(double p0, double p1, double p2, double p3,
                                          double t0, double t1) noexcept
        {
            const double a00 = p0 + (p1 - p0) * t0;
            const double a01 = p1 + (p2 - p1) * t0;
            const double a02 = p2 + (p3 - p2) * t0;

            const double a10 = p0 + (p1 - p0) * t1;
            const double a11 = p1 + (p2 - p1) * t1;
            const double a12 = p2 + (p3 - p2) * t1;

            const double b000 = a00 + (a01 - a00) * t0;
            const double b001 = a01 + (a02 - a01) * t0;

            const double b010 = a00 + (a01 - a00) * t1;
            const double b011 = a01 + (a02 - a01) * t1;

            const double b110 = a10 + (a11 - a10) * t1;
            const double b111 = a11 + (a12 - a11) * t1;

            AxisBlossom out;
            out.q[0] = b000 + (b001 - b000) * t0;
            out.q[1] = b000 + (b001 - b000) * t1;
            out.q[2] = b010 + (b011 - b010) * t1;
            out.q[3] = b110 + (b111 - b110) * t1;
            return out;
        }

        // THE EXTENT OF ONE AXIS, given its four control values and their shared
        // uncertainty.
        //
        // The convex hull of the control points always contains the curve, so the hull of
        // the four values, widened by the uncertainty, is always a valid answer. It is also
        // loose: an arch whose handles reach to 24 has a hull that reaches to 24 and a
        // curve that stops at 18.
        //
        // LEMMA 6 recovers the tightness. The derivative of a cubic Bezier is the quadratic
        // Bezier whose control values are the three differences below; Bernstein weights are
        // non-negative, so three non-negative control values force a non-negative derivative
        // on the whole interval, and a monotone axis attains its extremes at the endpoints.
        //
        // The test is against 4*e rather than 0 because each difference is uncertain by 2*e
        // plus its own rounding: a difference that cannot be PROVED to have a sign falls
        // through to the hull instead of claiming monotonicity.
        //
        // A cubic has at most two extrema per axis, so beyond a couple of subdivisions
        // essentially every piece takes the monotone branch.
        [[nodiscard]] Certified::Interval AxisExtent(const AxisBlossom& blossom,
                                                     double e) noexcept
        {
            const double* const q = blossom.q;
            const double d0 = q[1] - q[0];
            const double d1 = q[2] - q[1];
            const double d2 = q[3] - q[2];
            const double guard = 4.0 * e;

            if (d0 >= guard && d1 >= guard && d2 >= guard)
                return { q[0] - e, q[3] + e };
            if (d0 <= -guard && d1 <= -guard && d2 <= -guard)
                return { q[3] - e, q[0] + e };

            double lo = q[0];
            double hi = q[0];
            for (int i = 1; i < 4; ++i)
            {
                if (q[i] < lo) lo = q[i];
                if (q[i] > hi) hi = q[i];
            }
            return { lo - e, hi + e };
        }

        [[nodiscard]] double DeCasteljau(double p0, double p1, double p2, double p3,
                                         double t) noexcept
        {
            const double a0 = p0 + (p1 - p0) * t;
            const double a1 = p1 + (p2 - p1) * t;
            const double a2 = p2 + (p3 - p2) * t;
            const double b0 = a0 + (a1 - a0) * t;
            const double b1 = a1 + (a2 - a1) * t;
            return b0 + (b1 - b0) * t;
        }
    }

    Bounds2 CertifiedPoint(const Segment& segment, double t) noexcept
    {
        const double ex = AxisUncertainty(
            AxisScale(segment.p0.x, segment.c1.x, segment.c2.x, segment.p1.x));
        const double ey = AxisUncertainty(
            AxisScale(segment.p0.y, segment.c1.y, segment.c2.y, segment.p1.y));

        const double x = DeCasteljau(segment.p0.x, segment.c1.x, segment.c2.x, segment.p1.x, t);
        const double y = DeCasteljau(segment.p0.y, segment.c1.y, segment.c2.y, segment.p1.y, t);

        Bounds2 box;
        box.min = { x - ex, y - ey };
        box.max = { x + ex, y + ey };
        return box;
    }

    CertifiedPiece CertifiedSubCurve(const Segment& segment, double t0, double t1) noexcept
    {
        const double ex = AxisUncertainty(
            AxisScale(segment.p0.x, segment.c1.x, segment.c2.x, segment.p1.x));
        const double ey = AxisUncertainty(
            AxisScale(segment.p0.y, segment.c1.y, segment.c2.y, segment.p1.y));

        const AxisBlossom bx =
            Blossom(segment.p0.x, segment.c1.x, segment.c2.x, segment.p1.x, t0, t1);
        const AxisBlossom by =
            Blossom(segment.p0.y, segment.c1.y, segment.c2.y, segment.p1.y, t0, t1);

        CertifiedPiece out;

        const Certified::Interval extentX = AxisExtent(bx, ex);
        const Certified::Interval extentY = AxisExtent(by, ey);
        out.box.min = { extentX.lo, extentY.lo };
        out.box.max = { extentX.hi, extentY.hi };

        out.startBox.min = { bx.q[0] - ex, by.q[0] - ey };
        out.startBox.max = { bx.q[0] + ex, by.q[0] + ey };
        out.endBox.min = { bx.q[3] - ex, by.q[3] - ey };
        out.endBox.max = { bx.q[3] + ex, by.q[3] + ey };

        // The midpoint of the PIECE, from the piece's own control values. Going back to the
        // original curve would be equally valid and twice the work: the piece's control
        // values already carry the uncertainty, and de Casteljau on them does not amplify
        // it - three more convex combinations, whose own rounding the same budget covers,
        // because kBlossomOps was counted against nine and this chain is nine as well.
        const double mx = DeCasteljau(bx.q[0], bx.q[1], bx.q[2], bx.q[3], 0.5);
        const double my = DeCasteljau(by.q[0], by.q[1], by.q[2], by.q[3], 0.5);
        out.midBox.min = { mx - 2.0 * ex, my - 2.0 * ey };
        out.midBox.max = { mx + 2.0 * ex, my + 2.0 * ey };

        out.nominal = { { bx.q[0], by.q[0] }, { bx.q[1], by.q[1] },
                        { bx.q[2], by.q[2] }, { bx.q[3], by.q[3] }, segment.kind };

        // Bounds |true(s) - nominal(s)| per axis: the nominal control values ARE the
        // computed blossom, and each is within `e` of the true one. Doubled for margin.
        out.spread = 2.0 * (ex > ey ? ex : ey);
        return out;
    }

    Bounds2 ControlBounds(const Segment& segment) noexcept
    {
        Bounds2 bounds;
        bounds.Add(segment.p0);
        bounds.Add(segment.c1);
        bounds.Add(segment.c2);
        bounds.Add(segment.p1);
        return bounds;
    }

    double FlatnessMetric(const Segment& segment) noexcept
    {
        // Standard conservative bound: the curve's maximum deviation d from its chord
        // satisfies 16*d^2 <= max(ux2,vx2) + max(uy2,vy2) for the control-point
        // residuals below. Comparing this metric against 16*tol^2 therefore never
        // accepts a segment that is actually further than tol from its chord.
        double ux = 3.0 * segment.c1.x - 2.0 * segment.p0.x - segment.p1.x;
        double uy = 3.0 * segment.c1.y - 2.0 * segment.p0.y - segment.p1.y;
        double vx = 3.0 * segment.c2.x - 2.0 * segment.p1.x - segment.p0.x;
        double vy = 3.0 * segment.c2.y - 2.0 * segment.p1.y - segment.p0.y;

        ux *= ux;
        uy *= uy;
        vx *= vx;
        vy *= vy;

        return (ux > vx ? ux : vx) + (uy > vy ? uy : vy);
    }

    bool IsGeometricallyFlat(const Segment& segment, double tolerance) noexcept
    {
        const Vec2 chord = segment.p1 - segment.p0;
        const double chordLengthSquared = LengthSquared(chord);

        const double toleranceSquared = tolerance * tolerance;

        if (chordLengthSquared <= 0.0)
        {
            // Endpoints coincide, so there is no chord direction to project onto. The
            // curve is flat only if the handles collapse onto the endpoints too;
            // otherwise it is a loop and must be subdivided.
            return DistanceSquared(segment.c1, segment.p0) <= toleranceSquared &&
                   DistanceSquared(segment.c2, segment.p0) <= toleranceSquared;
        }

        const Vec2 d1 = segment.c1 - segment.p0;
        const Vec2 d2 = segment.c2 - segment.p0;

        // Perpendicular offsets, kept squared to avoid two sqrt calls per test.
        const double cross1 = Cross(chord, d1);
        const double cross2 = Cross(chord, d2);
        if (cross1 * cross1 > toleranceSquared * chordLengthSquared) return false;
        if (cross2 * cross2 > toleranceSquared * chordLengthSquared) return false;

        // Both handles must project inside the chord span, otherwise the curve can
        // overshoot past an endpoint while still hugging the infinite line.
        const double along1 = Dot(d1, chord);
        const double along2 = Dot(d2, chord);
        if (along1 < 0.0 || along1 > chordLengthSquared) return false;
        if (along2 < 0.0 || along2 > chordLengthSquared) return false;

        return true;
    }

    double LengthApprox(const Segment& segment, double tolerance, int maxDepth) noexcept
    {
        // The arc length is always bracketed by the chord below and the control
        // polygon above. Subdividing tightens both until they agree, so the returned
        // value is accurate to `tolerance` rather than being an unqualified estimate.
        struct Item
        {
            Segment curve;
            int depth;
        };

        Item stack[64];
        int top = 0;
        stack[top++] = { segment, 0 };

        double total = 0.0;

        while (top > 0)
        {
            const Item item = stack[--top];
            const Segment& curve = item.curve;

            const double chord = Distance(curve.p0, curve.p1);
            const double polygon = Distance(curve.p0, curve.c1) +
                                   Distance(curve.c1, curve.c2) +
                                   Distance(curve.c2, curve.p1);

            if (polygon - chord <= tolerance || item.depth >= maxDepth ||
                top + 2 > static_cast<int>(sizeof(stack) / sizeof(stack[0])))
            {
                // Gravesen's degree-weighted blend of the two bounds.
                total += (2.0 * chord + 2.0 * polygon) * 0.25;
                continue;
            }

            Segment left, right;
            Split(curve, 0.5, left, right);
            stack[top++] = { left, item.depth + 1 };
            stack[top++] = { right, item.depth + 1 };
        }

        return total;
    }

    ClosestPointResult ClosestPoint(const Segment& segment, Vec2 point,
                                    int seedCount, int refineIterations) noexcept
    {
        seedCount = std::clamp(seedCount, 4, 64);

        auto distanceSquaredAt = [&](double t) noexcept
        {
            return DistanceSquared(Evaluate(segment, t), point);
        };

        // f(t) = (B(t) - P) . B'(t); its roots are the stationary points of the
        // squared distance.
        auto stationary = [&](double t) noexcept
        {
            return Dot(Evaluate(segment, t) - point, Derivative(segment, t));
        };

        ClosestPointResult best;
        best.parameter = 0.0;
        best.point = segment.p0;
        best.distanceSquared = distanceSquaredAt(0.0);

        auto consider = [&](double t) noexcept
        {
            const double distance = distanceSquaredAt(t);
            if (distance < best.distanceSquared)
            {
                best.distanceSquared = distance;
                best.parameter = t;
                best.point = Evaluate(segment, t);
            }
        };

        consider(1.0);

        double previousT = 0.0;
        double previousF = stationary(0.0);

        for (int i = 1; i <= seedCount; ++i)
        {
            const double t = static_cast<double>(i) / static_cast<double>(seedCount);
            const double f = stationary(t);

            if (previousF == 0.0)
                consider(previousT);

            if ((previousF < 0.0) != (f < 0.0) && f != 0.0)
            {
                // Sign change brackets a stationary point. Refine with Newton, but
                // keep the bracket and fall back to bisection whenever a Newton step
                // would leave it or fails to reduce |f|. This is what stops an
                // ill-conditioned derivative from throwing the iterate away.
                double low = previousT;
                double high = t;
                double lowF = previousF;
                double root = 0.5 * (low + high);

                for (int iteration = 0; iteration < refineIterations; ++iteration)
                {
                    const Vec2 delta = Evaluate(segment, root) - point;
                    const Vec2 first = Derivative(segment, root);
                    const double value = Dot(delta, first);

                    if (value == 0.0)
                        break;

                    if ((value < 0.0) == (lowF < 0.0))
                    {
                        low = root;
                        lowF = value;
                    }
                    else
                    {
                        high = root;
                    }

                    const double slope = LengthSquared(first) + Dot(delta, SecondDerivative(segment, root));

                    double next = root;
                    if (slope != 0.0 && std::isfinite(slope))
                        next = root - value / slope;

                    if (!(next > low && next < high) || !std::isfinite(next))
                        next = 0.5 * (low + high);

                    if (std::fabs(next - root) <= 1e-15)
                    {
                        root = next;
                        break;
                    }
                    root = next;
                }

                consider(root);
            }

            previousT = t;
            previousF = f;
        }

        return best;
    }

    bool IsDegenerate(const Segment& segment, double tolerance) noexcept
    {
        const double toleranceSquared = tolerance * tolerance;
        return DistanceSquared(segment.p0, segment.p1) <= toleranceSquared &&
               DistanceSquared(segment.p0, segment.c1) <= toleranceSquared &&
               DistanceSquared(segment.p0, segment.c2) <= toleranceSquared;
    }

    double SignedAreaContribution(const Segment& segment, Vec2 origin) noexcept
    {
        // Closed form of (1/2) * integral over t of (x*y' - y*x') dt for a cubic.
        // Exact within floating point, so contour area needs no flattening and
        // carries no flattening error.
        //
        // The control points are taken relative to `origin` first. That is exact for
        // the closed-loop sum - see the header for why - and it is what keeps the
        // cross products from being differences of huge nearly equal numbers.
        const Vec2 p0 = segment.p0 - origin;
        const Vec2 p1 = segment.c1 - origin;
        const Vec2 p2 = segment.c2 - origin;
        const Vec2 p3 = segment.p1 - origin;

        return (3.0 / 10.0) * (p0.x * p1.y - p1.x * p0.y) +
               (3.0 / 20.0) * (p0.x * p2.y - p2.x * p0.y) +
               (1.0 / 20.0) * (p0.x * p3.y - p3.x * p0.y) +
               (3.0 / 20.0) * (p1.x * p2.y - p2.x * p1.y) +
               (3.0 / 20.0) * (p1.x * p3.y - p3.x * p1.y) +
               (3.0 / 10.0) * (p2.x * p3.y - p3.x * p2.y);
    }
}
