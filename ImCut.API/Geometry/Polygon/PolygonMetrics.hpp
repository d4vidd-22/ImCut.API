#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryTypes.hpp"
#include "../Math/Bounds2.hpp"
#include "../Math/Transform2.hpp"

#include <cstddef>
#include <vector>

namespace ImCut::Geometry::Metrics
{
    // Signed area of a contour, computed analytically from the cubic segments.
    //
    // No flattening is involved, so this is exact within floating point and carries
    // no chord-tolerance error at all - a flattened shoelace would only ever be as
    // accurate as the polyline that produced it, and would need thousands of points
    // on a curved contour to match what the closed form gets for free.
    //
    // Positive means counter-clockwise, which is the kernel's outer-ring convention.
    //
    // SUPPORTED ENVELOPE (measured, V5).
    //
    // The sum is referenced to the contour's own first node, so it is not a difference
    // of large absolute cross products and does not degrade with distance from the
    // origin. Measured over coordinates from 0 to 1e9 mm - the documented coordinate
    // ceiling - a square of side s reports its area to about ten significant digits and
    // its orientation correctly for every s at or above 0.01 mm:
    //
    //     s = 0.1 mm   correct at every offset up to 1e9
    //     s = 0.01 mm  correct at every offset up to 1e9
    //     s = 0.001 mm area 1e-6 mm^2, which is EXACTLY GeometryTolerance::absoluteArea
    //
    // The last row is not a numerical failure: the computed area is accurate to ten
    // digits there too. It is the degeneracy floor doing what it says. OrientationOf
    // reports Degenerate when |area| does not exceed AreaThreshold, and a 1 um square
    // sits on that threshold, so which side of it the final bit lands on is arbitrary.
    // A caller that needs orientation for features at or below 1e-6 mm^2 has to lower
    // absoluteArea deliberately and accept what else that reclassifies; it is a
    // contract decision, not a precision one.
    //
    // Before V5 this sum was referenced to the absolute origin and lost
    // log10(d^2/s^2) digits, inverting the SIGN past d^2/s^2 ~ 1e13 - a 1 mm feature
    // failed at 1e8 mm and a 1 um feature at 1e4 mm. See Curves/CubicBezier.hpp.
    [[nodiscard]] double SignedArea(const Contour& contour) noexcept;

    [[nodiscard]] inline double AbsoluteArea(const Contour& contour) noexcept
    {
        const double area = SignedArea(contour);
        return area < 0.0 ? -area : area;
    }

    // Shoelace over an explicit ring. Used for already-flattened geometry and as the
    // reference the analytic form is checked against.
    [[nodiscard]] double SignedArea(const Vec2* ring, std::size_t count) noexcept;

    [[nodiscard]] inline double SignedArea(const std::vector<Vec2>& ring) noexcept
    {
        return SignedArea(ring.data(), ring.size());
    }

    // Arc length. Curved segments are measured by adaptive subdivision, so the result
    // is accurate to `tolerance` rather than to whatever a flattening step chose.
    [[nodiscard]] double Perimeter(const Contour& contour, double tolerance) noexcept;

    [[nodiscard]] double Perimeter(const Vec2* ring, std::size_t count, bool closed) noexcept;

    // Tight bounds over the true curve extents, not the control hull.
    // TIGHT extents. A measurement, not a proof: `ExactBounds` is exact over the reals
    // and short over IEEE-754 by up to 4 ULP of the coordinate, so this box does NOT
    // provably contain the geometry. Use it to report, to size, to compare; never to
    // prove a shape is inside something. ComputeEnclosure below is the box for that.
    [[nodiscard]] Bounds2 ComputeBounds(const Contour& contour) noexcept;
    [[nodiscard]] Bounds2 ComputeBounds(const Path& path) noexcept;

    // A box PROVED to contain the geometry, in the type that says so.
    //
    // Slightly larger than ComputeBounds - the difference is the certified rounding term,
    // about 1.4e-12 mm on a 100 mm shape - and that difference is the whole point: it is
    // what makes `BoundsEnclosure::Enclosing` a true assertion rather than one that holds
    // for polylines and fails by a few ULP for curves.
    [[nodiscard]] BoundsEnclosure ComputeEnclosure(const Contour& contour) noexcept;
    [[nodiscard]] BoundsEnclosure ComputeEnclosure(const Path& path) noexcept;

    // A WITNESS box for the transformed shape: the shape reaches at least this far in
    // each of the four axis directions. BOUNDS_EVIDENCE_PROOF.md section 8.2.
    //
    // This exists because `transform.ApplyEnclosure(ComputeBounds(path))` is NOT a
    // witness - lemma B1b gives the counterexample, and F26 and F38 are what happens
    // when it is used as one. The construction here is by POINTS of the shape, so its
    // validity does not depend on any extremum search being accurate: `Evaluate(seg, t)`
    // lies on the curve for every `t`, whatever `t` is. Only the tightness depends on it.
    //
    // Costs one pass over the segments. A caller that needs it per query and not per
    // shape should hold on to the result; nothing here is cached.
    [[nodiscard]] BoundsWitness ComputeWitness(const Contour& contour,
                                               const Transform2& toWorld) noexcept;
    [[nodiscard]] BoundsWitness ComputeWitness(const Path& path,
                                               const Transform2& toWorld) noexcept;

    // The same witness, from points already known to lie on the shape. Every point in
    // `points` must BE a point of the shape - that is the whole precondition, and it is
    // what `CollectWitnessPoints` produces.
    [[nodiscard]] BoundsWitness ComputeWitness(const Vec2* points, std::size_t count,
                                               const Transform2& toWorld) noexcept;

    // Points of the shape, in its own frame: contour nodes plus one evaluated point per
    // cubic extremum parameter. Appends; does not clear.
    //
    // Callers that will ask for a witness under many transforms should keep the CONVEX
    // HULL of these - a hull has the same support function as the set it spans, so the
    // witness is identical and the per-query cost drops to the hull size.
    void CollectWitnessPoints(const Path& path, std::vector<Vec2>& out);

    // Area centroid. Returns the bounds centre for a zero-area contour, where the
    // centroid is undefined, rather than dividing by zero.
    [[nodiscard]] Vec2 Centroid(const Contour& contour) noexcept;

    [[nodiscard]] Orientation OrientationOf(const Contour& contour, const GeometryTolerance& tolerance) noexcept;

    [[nodiscard]] Orientation OrientationOf(const Vec2* ring, std::size_t count,
                                            const GeometryTolerance& tolerance) noexcept;

    // True when the ring turns the same way at every vertex. Collinear vertices are
    // tolerated; a reversal is not.
    [[nodiscard]] bool IsConvex(const Vec2* ring, std::size_t count,
                                const GeometryTolerance& tolerance) noexcept;

    [[nodiscard]] inline bool IsConvex(const std::vector<Vec2>& ring,
                                       const GeometryTolerance& tolerance) noexcept
    {
        return IsConvex(ring.data(), ring.size(), tolerance);
    }

    // Conservative exact-route predicate. Unlike IsConvex, no geometric tolerance is
    // allowed to erase a reflex turn: every turn must have the same robust sign and a
    // collinear triple is rejected. Use this when the answer will be labelled exact or
    // when an angular edge merge requires a genuinely convex input.
    [[nodiscard]] bool IsStrictlyConvex(const Vec2* ring, std::size_t count) noexcept;

    [[nodiscard]] inline bool IsStrictlyConvex(const std::vector<Vec2>& ring) noexcept
    {
        return IsStrictlyConvex(ring.data(), ring.size());
    }

    // Ratio of the ring's area to its convex hull's area, in (0,1]. One means convex.
    // The reference corpus sits around 0.4 to 0.83, which is why concavity is the
    // normal case for this kernel rather than an edge case.
    [[nodiscard]] double Convexity(const std::vector<Vec2>& ring);
}
