#pragma once

#include "../GeometryTypes.hpp"
#include "../Math/Bounds2.hpp"
#include "../Math/Vec2.hpp"

namespace ImCut::Geometry::CubicBezier
{
    struct ClosestPointResult
    {
        double parameter = 0.0;
        Vec2 point;
        double distanceSquared = 0.0;
    };

    // Bernstein basis. For t in [0,1] every weight is non-negative and they sum to
    // one, so evaluation is a convex combination of the control points and cannot
    // amplify rounding the way the expanded power basis can.
    [[nodiscard]] inline Vec2 Evaluate(const Segment& segment, double t) noexcept
    {
        const double mt = 1.0 - t;
        const double mt2 = mt * mt;
        const double t2 = t * t;

        const double w0 = mt2 * mt;
        const double w1 = 3.0 * mt2 * t;
        const double w2 = 3.0 * mt * t2;
        const double w3 = t2 * t;

        return { segment.p0.x * w0 + segment.c1.x * w1 + segment.c2.x * w2 + segment.p1.x * w3,
                 segment.p0.y * w0 + segment.c1.y * w1 + segment.c2.y * w2 + segment.p1.y * w3 };
    }

    // dB/dt. Note this is the true derivative, so its magnitude is the parametric
    // speed, not a unit tangent.
    [[nodiscard]] inline Vec2 Derivative(const Segment& segment, double t) noexcept
    {
        const double mt = 1.0 - t;
        const Vec2 a = segment.c1 - segment.p0;
        const Vec2 b = segment.c2 - segment.c1;
        const Vec2 c = segment.p1 - segment.c2;

        const double w0 = 3.0 * mt * mt;
        const double w1 = 6.0 * mt * t;
        const double w2 = 3.0 * t * t;

        return { a.x * w0 + b.x * w1 + c.x * w2,
                 a.y * w0 + b.y * w1 + c.y * w2 };
    }

    [[nodiscard]] inline Vec2 SecondDerivative(const Segment& segment, double t) noexcept
    {
        const Vec2 a = segment.c1 - segment.p0;
        const Vec2 b = segment.c2 - segment.c1;
        const Vec2 c = segment.p1 - segment.c2;

        const Vec2 d0 = b - a;
        const Vec2 d1 = c - b;

        const double mt = 1.0 - t;
        return { 6.0 * (d0.x * mt + d1.x * t), 6.0 * (d0.y * mt + d1.y * t) };
    }

    // Unit tangent, falling back to the chord when the parametric speed vanishes
    // (which happens at a cusp or with coincident handles).
    [[nodiscard]] Vec2 UnitTangent(const Segment& segment, double t) noexcept;

    // de Casteljau split. Exact: both halves reproduce the original curve.
    void Split(const Segment& segment, double t, Segment& left, Segment& right) noexcept;

    // The piece of `segment` spanning the parameter range [t0, t1].
    [[nodiscard]] Segment SubCurve(const Segment& segment, double t0, double t1) noexcept;

    // Interior parameters where dx/dt or dy/dt vanishes, ascending, endpoints
    // excluded. At most four. These are what make the bounding box tight.
    [[nodiscard]] int Extrema(const Segment& segment, double out[4]) noexcept;

    // Interior parameters where the curvature changes sign. At most two.
    [[nodiscard]] int Inflections(const Segment& segment, double out[2]) noexcept;

    // Tight bounds: the hull of the endpoints plus every axis extremum.
    //
    // NOT AN ENCLOSURE, AND NO LONGER USED BY THE KERNEL. Read the block on
    // CertifiedExactBounds below before reaching for this: the extremum parameters are
    // solved in double, so the point this evaluates is ON the curve but is not the
    // extremum, and 9807 of 59 359 measured true extrema fall OUTSIDE the box it
    // returns, by up to 4 ULP (evidence 1354).
    //
    // Every production use is gone. It survived in five pruning sites across
    // Intersection.cpp and Distance.cpp and in Metrics::ComputeBounds, each of which
    // DISCARDED a candidate or ASSERTED an enclosure on the strength of it, and the
    // soundness of all six rested on an unstated assumption that the working tolerance
    // dominates a few ULP of the coordinate - true for artwork at metre scale and
    // untrue for the coordinate ceiling the kernel documents. They now take
    // CertifiedExactBounds or CertifiedSubCurve, and the change made the narrow phase
    // 2.5x FASTER rather than slower, because carrying the box with the piece removed
    // more work than certifying it added (PERFORMANCE_BEFORE_AFTER.tsv).
    //
    // It remains public because it is public ABI, and it remains useful as the TIGHT
    // reference the certified box is measured against - which is what the tests use it
    // for. It must not be used to prune, to reject, or to build a BoundsEnclosure.
    [[nodiscard]] Bounds2 ExactBounds(const Segment& segment) noexcept;

    // Convex hull of the control points. Always contains the curve, never tight.
    // Cheap enough for broad-phase rejection.
    [[nodiscard]] Bounds2 ControlBounds(const Segment& segment) noexcept;

    // ------------------------------------------------------------------------------
    // CERTIFIED ENCLOSURE OF A PIECE OF THIS CURVE.
    //
    // ExactBounds is exact over the REALS and was measured not to be over IEEE-754 once
    // it is composed: Split -> subcurve -> Split -> subcurve -> ExactBounds excluded
    // points of the original curve in 4675 of 391 374 samples, worst 2.00 ULP, on all
    // nine adversarial fixtures including the plain arch (F31, evidence 1156). A lower
    // bound taken from a box that is too tight is a lower bound that is too high, which
    // is the direction that produces a DistanceLessThan false negative.
    //
    // THE FIX IS THE REFERENCE, NOT THE ARITHMETIC. A piece is named by the ORIGINAL
    // curve and a parameter interval, and the enclosure is computed from the original
    // four control points every time. The blossom below is always exactly three levels
    // deep, whatever the subdivision depth, so the error stops composing - which is the
    // composition the red actually measured. Inflating the old box by the worst observed
    // escape would have fitted a constant to nine fixtures instead.
    //
    // See DISTANCE_INTERVAL_PROOF.md, lemmas 5 to 7.
    struct CertifiedPiece
    {
        // PROVED to contain segment([t0, t1]) in IEEE-754 arithmetic.
        Bounds2 box;

        // PROVED to contain the single points segment(t0), segment(mid), segment(t1).
        // Points of the ORIGINAL curve, so a distance measured between two of these
        // boxes is witnessed by geometry that really is on both curves.
        Bounds2 startBox;
        Bounds2 midBox;
        Bounds2 endBox;

        // The piece as ordinary doubles: the midpoints of the certified control
        // intervals. HEURISTICS ONLY - traversal order, which side to split, the
        // flatness gate. It must never be the source of a published bound; `spread`
        // below is what converts a quantity measured on it into one that is.
        Segment nominal;

        // Widest certified control interval, over both axes. Bounds |true(s) -
        // nominal(s)| per axis for every s, because the Bernstein weights are
        // non-negative and sum to one.
        double spread = 0.0;
    };

    [[nodiscard]] CertifiedPiece CertifiedSubCurve(const Segment& segment,
                                                   double t0, double t1) noexcept;

    // A box PROVED to contain the single point segment(t).
    [[nodiscard]] Bounds2 CertifiedPoint(const Segment& segment, double t) noexcept;

    // The exact box, and an ENCLOSURE - which `ExactBounds` is not.
    //
    // `ExactBounds` evaluates the curve at the extremum parameters it solved for in
    // double. The point it evaluates is on the curve, but it is not the extremum, so the
    // box falls short. Measured over 28 000 random cubics across seven decades: 9807 of
    // 59 359 true extrema fall OUTSIDE the box, by up to 4 ULP (1354). Nine hand-picked
    // adversarial fixtures found zero, because they were chosen for pathological SHAPE
    // and the defect lives in root CONDITIONING.
    //
    // This one takes the same roots and uses them as CUT POINTS instead of as answers.
    // The union of certified pieces over a partition of [0,1] covers the curve exactly
    // (lemma 7: children share the same double, so [t0,tm] u [tm,t1] = [t0,t1]), and that
    // holds for ANY cuts - a wrong root costs tightness and cannot cost soundness.
    //
    // And it stays tight, which the etapa B's first attempt did not. Certifying the whole
    // curve at once fell back to the control hull, because a whole curve is rarely
    // monotone on either axis, and 112 spatial assertions failed. Cut AT the extrema, each
    // piece is monotone by construction, so the certified enclosure is the endpoint hull
    // plus a rounding term rather than the control hull.
    [[nodiscard]] Bounds2 CertifiedExactBounds(const Segment& segment) noexcept;
    // ------------------------------------------------------------------------------

    // Parametric flatness bound: bounds max|B(t) - Lerp(p0,p1,t)|. Compare against
    // 16*tolerance^2. Sufficient but not necessary - it charges for non-uniform
    // parameterisation even when the curve sits exactly on its chord.
    [[nodiscard]] double FlatnessMetric(const Segment& segment) noexcept;

    // Geometric flatness: true when every point of the curve lies within `tolerance`
    // of the chord *segment* [p0,p1].
    //
    // Sound by the convex hull property. Distance to the chord line is a convex
    // function, and p0/p1 lie on it, so if both handles are within tolerance of the
    // line then the whole hull - and therefore the whole curve - is too. Requiring
    // both handles to project inside [0,1] along the chord upgrades that from "near
    // the infinite line" to "near the finite segment", since projection is linear and
    // the hull's projection is just the span of the four projected control points.
    [[nodiscard]] bool IsGeometricallyFlat(const Segment& segment, double tolerance) noexcept;

    // Either criterion alone is sufficient, so accepting their disjunction is still
    // conservative while subdividing markedly less on real artwork, where handles are
    // rarely at exact thirds.
    [[nodiscard]] inline bool IsFlatEnough(const Segment& segment, double tolerance) noexcept
    {
        return IsGeometricallyFlat(segment, tolerance) ||
               FlatnessMetric(segment) <= 16.0 * tolerance * tolerance;
    }

    // Arc length by adaptive subdivision, refined until the control-polygon and
    // chord bounds agree to within `tolerance`. The true length always lies between
    // those two, so the result is accurate to `tolerance` and never a guess.
    [[nodiscard]] double LengthApprox(const Segment& segment, double tolerance,
                                      int maxDepth = 24) noexcept;

    // Closest point on the curve to `point`.
    //
    // The exact condition (B(t) - P) . B'(t) = 0 is a quintic, so this seeds from a
    // uniform sampling, then refines each promising bracket with Newton guarded by
    // bisection. The bisection bracket is what makes it safe: an unstable Newton step
    // is rejected rather than followed, so the result cannot run away from the
    // bracketed root. Endpoints are always considered.
    [[nodiscard]] ClosestPointResult ClosestPoint(const Segment& segment, Vec2 point,
                                                  int seedCount = 16,
                                                  int refineIterations = 24) noexcept;

    // True when the curve collapses to a point within tolerance.
    [[nodiscard]] bool IsDegenerate(const Segment& segment, double tolerance) noexcept;

    // Signed area enclosed between the curve and `origin`, integrated exactly in closed
    // form. Summing this over a closed contour yields the contour's signed area with no
    // flattening at all, which is both faster and exact.
    //
    // `origin` is mandatory, and every segment of one contour must be given the SAME
    // one, because that is the only thing that makes the sum well conditioned.
    //
    // The integral (1/2)*loop(x dy - y dx) is translation invariant, so a common origin
    // changes no exact value; what it changes is the size of the intermediate terms. A
    // feature of side s at distance d from the reference point produces terms of order
    // d^2 for a result of order s^2, so the sum loses log10(d^2/s^2) digits to
    // cancellation. With the absolute origin as reference, a CCW 0.1 mm square at 1e8 mm
    // reported area -2 and orientation Clockwise; failures start at d^2/s^2 ~ 1e13,
    // which is 1 micron at 10 metres. Referenced to the contour's own first node, the
    // envelope is set only by the double spacing at d.
    //
    // Per-segment origins are NOT equivalent: translating one segment by -o shifts its
    // contribution by -(o.x*dy - o.y*dx)/2, and those terms only telescope to zero when
    // o is the same for the whole loop.
    [[nodiscard]] double SignedAreaContribution(const Segment& segment, Vec2 origin) noexcept;
}
