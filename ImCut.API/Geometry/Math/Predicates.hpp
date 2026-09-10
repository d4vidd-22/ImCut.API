#pragma once

#include "../GeometryTolerance.hpp"
#include "../GeometryTypes.hpp"
#include "Bounds2.hpp"
#include "Vec2.hpp"

#include <cmath>

namespace ImCut::Geometry::Predicates
{
    // Filtered orientation test.
    //
    // Level 1 is the plain double determinant guarded by Shewchuk's forward error
    // bound: if the magnitude clears the bound, the sign is provably correct and we
    // return immediately, which is what happens for the overwhelming majority of
    // real inputs. Level 2 only runs inside the uncertain band and re-evaluates the
    // determinant in exact expansion arithmetic, so the returned sign is always the
    // true sign.
    //
    // This requires IEEE-754 double semantics with no reassociation: build with
    // /fp:precise (the MSVC default). /fp:fast would invalidate the error-free
    // transformations used by the exact fallback.
    [[nodiscard]] Orientation Orientation2D(Vec2 a, Vec2 b, Vec2 c) noexcept;

    // Signed area of the parallelogram (b-a) x (c-a), exact sign, raw magnitude.
    // Positive means c lies left of a->b, i.e. the turn a->b->c is counter-clockwise.
    [[nodiscard]] double Orient2D(Vec2 a, Vec2 b, Vec2 c) noexcept;

    // Unfiltered determinant. Fast, but the sign is unreliable near degeneracy;
    // use only where a wrong answer in the uncertain band is harmless.
    [[nodiscard]] inline double Orient2DFast(Vec2 a, Vec2 b, Vec2 c) noexcept
    {
        return (a.x - c.x) * (b.y - c.y) - (a.y - c.y) * (b.x - c.x);
    }

    [[nodiscard]] inline bool NearlyZero(double value, double tolerance) noexcept
    {
        return std::fabs(value) <= tolerance;
    }

    // Mixed absolute/relative comparison: the absolute term keeps it meaningful near
    // zero, the relative term keeps it meaningful for large coordinates.
    [[nodiscard]] inline bool NearlyEqual(double a, double b, double tolerance) noexcept
    {
        const double difference = std::fabs(a - b);
        if (difference <= tolerance)
            return true;
        const double magnitude = std::fabs(a) > std::fabs(b) ? std::fabs(a) : std::fabs(b);
        return difference <= tolerance * magnitude;
    }

    [[nodiscard]] inline bool NearlyEqual(Vec2 a, Vec2 b, double tolerance) noexcept
    {
        return DistanceSquared(a, b) <= tolerance * tolerance;
    }

    [[nodiscard]] bool Collinear(Vec2 a, Vec2 b, Vec2 c, const GeometryTolerance& tolerance) noexcept;

    // True when p lies on segment [a,b] within `tolerance` (perpendicular distance
    // and inside the segment span, not just on the infinite line).
    [[nodiscard]] bool PointOnSegment(Vec2 p, Vec2 a, Vec2 b, double tolerance) noexcept;

    // Parametric position of the projection of p onto [a,b], clamped to [0,1].
    // Returns 0 for a degenerate segment.
    [[nodiscard]] double ProjectionParameter(Vec2 p, Vec2 a, Vec2 b) noexcept;

    [[nodiscard]] inline bool BoundsOverlap(const Bounds2& a, const Bounds2& b) noexcept
    {
        return a.Overlaps(b);
    }

    [[nodiscard]] inline bool BoundsOverlap(const Bounds2& a, const Bounds2& b, double tolerance) noexcept
    {
        return a.Overlaps(b, tolerance);
    }

    // Tri-state point-in-ring over an explicit polyline ring (closed implicitly:
    // the last point connects back to the first).
    //
    // Boundary is detected first and exactly, because the crossing count alone is
    // meaningless for a point sitting on an edge. The crossing test itself uses the
    // robust orientation predicate rather than a divided x-intersection, so a vertex
    // grazing the ray can never flip the parity by rounding.
    [[nodiscard]] PointClassification PointInRing(Vec2 point, const Vec2* ring, std::size_t count,
                                                  double boundaryTolerance) noexcept;

    // Winding number of a closed polyline ring around `point`.
    // Returns 0 when the point lies on the boundary within tolerance.
    [[nodiscard]] int WindingNumber(Vec2 point, const Vec2* ring, std::size_t count,
                                    double boundaryTolerance, bool* onBoundary = nullptr) noexcept;
}
