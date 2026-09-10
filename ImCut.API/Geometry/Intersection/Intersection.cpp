#include "Intersection.hpp"

#include "../Curves/CubicBezier.hpp"
#include "../Math/Bounds2.hpp"
#include "../Math/Polynomial.hpp"
#include "../Math/Predicates.hpp"

#include <algorithm>
#include <cmath>

namespace ImCut::Geometry::Intersect
{
    namespace
    {
        // Classifies contact as a tangency when the two directions are parallel to
        // within the tolerance. Both directions are normalised first so the test means
        // "the angle between them is negligible" and does not drift with segment
        // length.
        [[nodiscard]] IntersectionKind ClassifyByTangents(Vec2 directionA, Vec2 directionB,
                                                          IntersectionKind fallback,
                                                          const GeometryTolerance& tolerance) noexcept
        {
            const Vec2 unitA = Normalized(directionA);
            const Vec2 unitB = Normalized(directionB);
            if (unitA == Vec2{} || unitB == Vec2{})
                return fallback;

            if (std::fabs(Cross(unitA, unitB)) <= tolerance.tangent)
                return IntersectionKind::Tangent;

            return fallback;
        }

        [[nodiscard]] bool IsEndpointParameter(double t, double epsilon) noexcept
        {
            return t <= epsilon || t >= 1.0 - epsilon;
        }
    }

    LineIntersection Lines(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1,
                           const GeometryTolerance& tolerance) noexcept
    {
        LineIntersection result;

        // Exact orientation signs drive every branch below. Deciding the topology from
        // these rather than from a computed point is what makes the classification
        // stable near degeneracy.
        const double d1 = Predicates::Orient2D(b0, b1, a0);
        const double d2 = Predicates::Orient2D(b0, b1, a1);
        const double d3 = Predicates::Orient2D(a0, a1, b0);
        const double d4 = Predicates::Orient2D(a0, a1, b1);

        const bool aStraddles = (d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0);
        const bool bStraddles = (d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0);

        if (aStraddles && bStraddles)
        {
            // Each segment strictly separates the other's endpoints, so they cross.
            const double denominatorA = d1 - d2;
            const double denominatorB = d3 - d4;

            result.parameterA = denominatorA != 0.0 ? d1 / denominatorA : 0.0;
            result.parameterB = denominatorB != 0.0 ? d3 / denominatorB : 0.0;
            result.point = Lerp(a0, a1, result.parameterA);
            result.kind = ClassifyByTangents(a1 - a0, b1 - b0, IntersectionKind::ProperCross, tolerance);
            return result;
        }

        // All four orientations vanish: the segments are on one line.
        if (d1 == 0.0 && d2 == 0.0 && d3 == 0.0 && d4 == 0.0)
        {
            const Vec2 direction = a1 - a0;
            const double lengthSquared = LengthSquared(direction);

            if (lengthSquared <= 0.0)
            {
                // A degenerate A collapses to a point test against B.
                if (Predicates::PointOnSegment(a0, b0, b1, tolerance.duplicate))
                {
                    result.kind = IntersectionKind::EndpointTouch;
                    result.point = a0;
                    result.parameterB = Predicates::ProjectionParameter(a0, b0, b1);
                }
                return result;
            }

            // Project B onto A's parameter axis and intersect the two spans.
            double tb0 = Dot(b0 - a0, direction) / lengthSquared;
            double tb1 = Dot(b1 - a0, direction) / lengthSquared;
            if (tb0 > tb1) std::swap(tb0, tb1);

            const double low = std::max(0.0, tb0);
            const double high = std::min(1.0, tb1);

            const double spanEpsilon = tolerance.duplicate / std::sqrt(lengthSquared);

            if (high < low - spanEpsilon)
            {
                result.kind = IntersectionKind::Collinear;
                return result;
            }

            if (high - low <= spanEpsilon)
            {
                // The spans meet at a single point: touching end to end.
                const double at = 0.5 * (low + high);
                result.kind = IntersectionKind::EndpointTouch;
                result.parameterA = std::clamp(at, 0.0, 1.0);
                result.point = Lerp(a0, a1, result.parameterA);
                result.parameterB = Predicates::ProjectionParameter(result.point, b0, b1);
                return result;
            }

            result.kind = IntersectionKind::Overlap;
            result.overlapMinA = low;
            result.overlapMaxA = high;
            result.overlapStart = Lerp(a0, a1, low);
            result.overlapEnd = Lerp(a0, a1, high);
            result.parameterA = low;
            result.point = result.overlapStart;
            result.parameterB = Predicates::ProjectionParameter(result.point, b0, b1);
            return result;
        }

        // Exactly one or two orientations vanish: an endpoint lies on the other
        // segment's line. It is a real contact only if it also lies within its span.
        auto touch = [&](Vec2 point, double parameterA, double parameterB) noexcept
        {
            result.kind = ClassifyByTangents(a1 - a0, b1 - b0, IntersectionKind::EndpointTouch, tolerance);
            if (result.kind == IntersectionKind::Tangent)
                result.kind = IntersectionKind::EndpointTouch;
            result.point = point;
            result.parameterA = parameterA;
            result.parameterB = parameterB;
        };

        if (d1 == 0.0 && Predicates::PointOnSegment(a0, b0, b1, tolerance.duplicate))
        {
            touch(a0, 0.0, Predicates::ProjectionParameter(a0, b0, b1));
            return result;
        }
        if (d2 == 0.0 && Predicates::PointOnSegment(a1, b0, b1, tolerance.duplicate))
        {
            touch(a1, 1.0, Predicates::ProjectionParameter(a1, b0, b1));
            return result;
        }
        if (d3 == 0.0 && Predicates::PointOnSegment(b0, a0, a1, tolerance.duplicate))
        {
            touch(b0, Predicates::ProjectionParameter(b0, a0, a1), 0.0);
            return result;
        }
        if (d4 == 0.0 && Predicates::PointOnSegment(b1, a0, a1, tolerance.duplicate))
        {
            touch(b1, Predicates::ProjectionParameter(b1, a0, a1), 1.0);
            return result;
        }

        return result;
    }

    bool LinesIntersect(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1,
                        const GeometryTolerance& tolerance) noexcept
    {
        const double d1 = Predicates::Orient2D(b0, b1, a0);
        const double d2 = Predicates::Orient2D(b0, b1, a1);
        const double d3 = Predicates::Orient2D(a0, a1, b0);
        const double d4 = Predicates::Orient2D(a0, a1, b1);

        if (((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0)) &&
            ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0)))
        {
            return true;
        }

        if (d1 == 0.0 && Predicates::PointOnSegment(a0, b0, b1, tolerance.duplicate)) return true;
        if (d2 == 0.0 && Predicates::PointOnSegment(a1, b0, b1, tolerance.duplicate)) return true;
        if (d3 == 0.0 && Predicates::PointOnSegment(b0, a0, a1, tolerance.duplicate)) return true;
        if (d4 == 0.0 && Predicates::PointOnSegment(b1, a0, a1, tolerance.duplicate)) return true;

        return false;
    }

    int LineCubic(Vec2 a0, Vec2 a1, const Segment& curve,
                  const GeometryTolerance& tolerance, IntersectionPoint out[3]) noexcept
    {
        const Vec2 lineDirection = a1 - a0;
        const double lineLengthSquared = LengthSquared(lineDirection);
        if (lineLengthSquared <= 0.0)
            return 0;

        // Signed distance to the line is linear, so composing it with the cubic gives a
        // cubic polynomial whose roots are exactly the parameters where the curve meets
        // the line. Using the un-normalised normal keeps the coefficients exact.
        const Vec2 normal = Perpendicular(lineDirection);

        // Power-basis coefficients of the curve.
        const Vec2 k0 = curve.p0;
        const Vec2 k1 = (curve.c1 - curve.p0) * 3.0;
        const Vec2 k2 = (curve.c2 - curve.c1 * 2.0 + curve.p0) * 3.0;
        const Vec2 k3 = curve.p1 - curve.c2 * 3.0 + curve.c1 * 3.0 - curve.p0;

        const double c3 = Dot(k3, normal);
        const double c2 = Dot(k2, normal);
        const double c1 = Dot(k1, normal);
        const double c0 = Dot(k0 - a0, normal);

        double roots[3];
        int count = Polynomial::SolveCubic(c3, c2, c1, c0, roots);

        // SolveCubic writes at most three roots by construction - one, two or three
        // depending on the discriminant, and it delegates the degenerate leading
        // coefficient to SolveQuadratic, which writes at most two. So this clamp changes
        // nothing today.
        //
        // It is here because that argument lives in ANOTHER translation unit. The static
        // analyser reported "Out of bound access to memory after the end of 'roots'" for
        // the read below, precisely because it cannot see across the boundary and has to
        // assume the returned count is unbounded. Bounding it locally turns a cross-TU
        // assumption into an invariant a reader - and a checker - can verify here, in the
        // function that owns the array. That is cheap insurance against exactly the class
        // of defect the V8.1 WP13 P0 belonged to.
        count = (std::min)(count, 3);

        // The polynomial is defined on the whole real line; only [0,1] is on the curve.
        const double parameterEpsilon = 1e-9;
        count = Polynomial::ClampToInterval(roots, count, 0.0, 1.0, parameterEpsilon);

        int written = 0;
        for (int i = 0; i < count && written < 3; ++i)
        {
            const double t = roots[i];
            const Vec2 point = CubicBezier::Evaluate(curve, t);

            // The root places the point on the infinite line; it counts only if it also
            // falls inside the finite segment.
            const double along = Dot(point - a0, lineDirection) / lineLengthSquared;
            if (along < -parameterEpsilon || along > 1.0 + parameterEpsilon)
                continue;

            const double clampedAlong = std::clamp(along, 0.0, 1.0);
            if (DistanceSquared(point, Lerp(a0, a1, clampedAlong)) >
                tolerance.intersection * tolerance.intersection + lineLengthSquared * 1e-24)
            {
                continue;
            }

            IntersectionPoint& hit = out[written++];
            hit.point = point;
            hit.parameterA = clampedAlong;
            hit.parameterB = t;

            const Vec2 curveTangent = CubicBezier::Derivative(curve, t);
            IntersectionKind kind = IntersectionKind::ProperCross;
            if (IsEndpointParameter(t, parameterEpsilon) ||
                IsEndpointParameter(clampedAlong, parameterEpsilon))
            {
                kind = IntersectionKind::EndpointTouch;
            }
            hit.kind = ClassifyByTangents(lineDirection, curveTangent, kind, tolerance);
        }

        return written;
    }

    void MergeNearbyPoints(std::vector<IntersectionPoint>& points, double tolerance)
    {
        if (points.size() < 2)
            return;

        // Deterministic ordering first, so the survivor of a merge does not depend on
        // the order the subdivision happened to discover candidates in.
        std::sort(points.begin(), points.end(),
                  [](const IntersectionPoint& lhs, const IntersectionPoint& rhs) noexcept
                  {
                      if (lhs.parameterA != rhs.parameterA) return lhs.parameterA < rhs.parameterA;
                      if (lhs.parameterB != rhs.parameterB) return lhs.parameterB < rhs.parameterB;
                      if (lhs.point.x != rhs.point.x) return lhs.point.x < rhs.point.x;
                      return lhs.point.y < rhs.point.y;
                  });

        const double toleranceSquared = tolerance * tolerance;
        std::size_t unique = 1;

        for (std::size_t i = 1; i < points.size(); ++i)
        {
            const IntersectionPoint& previous = points[unique - 1];
            if (DistanceSquared(points[i].point, previous.point) <= toleranceSquared)
            {
                // A genuine crossing outranks a merely-touching duplicate report.
                if (points[unique - 1].kind == IntersectionKind::EndpointTouch &&
                    points[i].kind == IntersectionKind::ProperCross)
                {
                    points[unique - 1].kind = IntersectionKind::ProperCross;
                }
                continue;
            }
            points[unique++] = points[i];
        }

        points.resize(unique);
    }

    void Cubics(const Segment& a, const Segment& b, const GeometryContext& context,
                std::vector<IntersectionPoint>& out)
    {
        const GeometryTolerance& tolerance = context.Tolerance();

        // F35a. The box travels WITH the piece instead of being rebuilt on every pop, and
        // it is an ENCLOSURE rather than the tight-but-short box `ExactBounds` returns.
        //
        // Two separate defects were being relied on not to matter here. `ExactBounds`
        // evaluates the curve at the extremum parameters it solved for in double, so the
        // point it adds is ON the curve but is not the extremum: measured over 28 000
        // random cubics, 9 807 of 59 359 true extrema fall OUTSIDE the box it returns, by
        // up to 4 ULP (evidence 1354). And the piece it was being asked about is not the
        // true sub-curve either - it is a chain of Split results, each rounded, so its
        // control points have drifted from the original curve by more again.
        //
        // A box that is short in a prune that DISCARDS is a false negative waiting for
        // coordinates large enough that 4 ULP outgrows `tolerance.intersection`. Nothing
        // in the kernel enforced or even stated the relation the soundness depended on.
        //
        // CertifiedSubCurve(original, t0, t1) closes both at once: it is proved to contain
        // the true restriction of the ORIGINAL curve to [t0,t1], and its blossom is always
        // exactly three levels deep whatever the subdivision depth, so the error does not
        // compose. The parameter spans this loop already tracked for reporting are what
        // make it addressable that way.
        //
        // AND IT IS NOT A NEW COST. Each pop used to build two boxes; now each SPLIT
        // builds two, and the untouched side's box is copied. Pops outnumber splits, so
        // the number of boxes built falls even though each one is certified. Measured A/B
        // in PERFORMANCE_BEFORE_AFTER.tsv.
        struct Pair
        {
            Segment curveA;
            double a0, a1;
            Segment curveB;
            double b0, b1;
            int depth;
            Bounds2 boundsA;
            Bounds2 boundsB;
        };

        const auto boxA = [&a](double t0, double t1) noexcept
        {
            return CubicBezier::CertifiedSubCurve(a, t0, t1).box;
        };
        const auto boxB = [&b](double t0, double t1) noexcept
        {
            return CubicBezier::CertifiedSubCurve(b, t0, t1).box;
        };

        // Depth is capped, and each pop pushes at most two, so the stack is bounded.
        //
        // MEASURED, and kept as a std::vector on purpose. AnyCubicIntersection below uses
        // a fixed automatic array for the same walk, and the obvious symmetry argument
        // says this should too: Pair holds two Segments, so reserve(64) is an ~11.8 KB
        // heap allocation on every call, and Cubics runs once per BVH candidate pair -
        // 1 023 959 allocations and 12.25 GB for one 50k self-intersection.
        //
        // The A/B rejected it. Allocations fell to 42 and bytes to 155 MB, findings and
        // candidate pairs were identical, and the narrow phase got CONSISTENTLY SLOWER:
        // +6.8/+4.5/+3.9/+3.5/+3.8/+3.6% across the six sizes, against a +-0.6% noise
        // floor established by the untouched build and broad stages in the same runs.
        // 64 x 184 bytes is ~11.8 KB of automatic storage, which exceeds a page and puts
        // an MSVC stack probe on every call, while the vector's block came back hot from
        // the allocator's free list. Trading one for the other loses.
        // Evidence: 544 (before), 547 (after), 548 (verdict).
        std::vector<Pair> stack;
        stack.reserve(64);
        stack.push_back({ a, 0.0, 1.0, b, 0.0, 1.0, 0, boxA(0.0, 1.0), boxB(0.0, 1.0) });

        const int maxDepth = std::min(context.Limits().maxSubdivisionDepth, 30);
        const std::size_t startSize = out.size();

        while (!stack.empty())
        {
            const Pair pair = stack.back();
            stack.pop_back();

            // Broad phase: certified curve enclosures, carried with the pieces. Proved to
            // contain the true sub-curves, so it can over-accept but never wrongly
            // rejects a real intersection - which is what a prune that DISCARDS needs and
            // what the tight box it replaced could not provide.
            const Bounds2& boundsA = pair.boundsA;
            const Bounds2& boundsB = pair.boundsB;
            if (!boundsA.Overlaps(boundsB, tolerance.intersection))
                continue;

            if (out.size() - startSize > context.Limits().maxIntersections)
                break;

            const bool flatA = CubicBezier::IsGeometricallyFlat(pair.curveA, tolerance.intersection);
            const bool flatB = CubicBezier::IsGeometricallyFlat(pair.curveB, tolerance.intersection);

            if ((flatA && flatB) || pair.depth >= maxDepth)
            {
                // Both pieces are now indistinguishable from segments at the working
                // tolerance, so the robust segment predicate decides the contact.
                const LineIntersection hit = Lines(pair.curveA.p0, pair.curveA.p1,
                                                   pair.curveB.p0, pair.curveB.p1, tolerance);

                if (!IsContact(hit.kind))
                    continue;

                IntersectionPoint point;
                point.point = hit.point;
                point.parameterA = pair.a0 + (pair.a1 - pair.a0) * hit.parameterA;
                point.parameterB = pair.b0 + (pair.b1 - pair.b0) * hit.parameterB;

                const Vec2 tangentA = CubicBezier::Derivative(a, point.parameterA);
                const Vec2 tangentB = CubicBezier::Derivative(b, point.parameterB);
                point.kind = ClassifyByTangents(tangentA, tangentB, IntersectionKind::ProperCross, tolerance);

                out.push_back(point);
                continue;
            }

            // Split whichever piece is still the least line-like, so subdivision is
            // spent where the geometry is actually curved.
            const bool splitA = !flatA && (flatB || boundsA.Perimeter() >= boundsB.Perimeter());

            if (splitA)
            {
                Segment left, right;
                CubicBezier::Split(pair.curveA, 0.5, left, right);
                const double middle = 0.5 * (pair.a0 + pair.a1);
                stack.push_back({ left, pair.a0, middle, pair.curveB, pair.b0, pair.b1,
                                  pair.depth + 1, boxA(pair.a0, middle), boundsB });
                stack.push_back({ right, middle, pair.a1, pair.curveB, pair.b0, pair.b1,
                                  pair.depth + 1, boxA(middle, pair.a1), boundsB });
            }
            else
            {
                Segment left, right;
                CubicBezier::Split(pair.curveB, 0.5, left, right);
                const double middle = 0.5 * (pair.b0 + pair.b1);
                stack.push_back({ pair.curveA, pair.a0, pair.a1, left, pair.b0, middle,
                                  pair.depth + 1, boundsA, boxB(pair.b0, middle) });
                stack.push_back({ pair.curveA, pair.a0, pair.a1, right, middle, pair.b1,
                                  pair.depth + 1, boundsA, boxB(middle, pair.b1) });
            }
        }

        MergeNearbyPoints(out, tolerance.duplicate);
        CountStat(context, &GeometryStatistics::narrowPhaseTests);
        CountStat(context, &GeometryStatistics::intersectionsFound,
                  static_cast<std::uint64_t>(out.size() - startSize));
    }

    void Segments(const Segment& a, const Segment& b, const GeometryContext& context,
                  std::vector<IntersectionPoint>& out)
    {
        const GeometryTolerance& tolerance = context.Tolerance();

        if (a.IsLine() && b.IsLine())
        {
            const LineIntersection hit = Lines(a.p0, a.p1, b.p0, b.p1, tolerance);
            if (!IsContact(hit.kind))
                return;

            IntersectionPoint point;
            point.point = hit.point;
            point.parameterA = hit.parameterA;
            point.parameterB = hit.parameterB;
            point.kind = hit.kind;
            out.push_back(point);

            // An overlapping pair shares a span, not a point; report both ends so the
            // caller sees the extent rather than an arbitrary sample of it.
            if (hit.kind == IntersectionKind::Overlap)
            {
                IntersectionPoint end;
                end.point = hit.overlapEnd;
                end.parameterA = hit.overlapMaxA;
                end.parameterB = Predicates::ProjectionParameter(hit.overlapEnd, b.p0, b.p1);
                end.kind = IntersectionKind::Overlap;
                out.push_back(end);
            }
            return;
        }

        if (a.IsLine() != b.IsLine())
        {
            const Segment& line = a.IsLine() ? a : b;
            const Segment& curve = a.IsLine() ? b : a;

            IntersectionPoint hits[3];
            const int count = LineCubic(line.p0, line.p1, curve, tolerance, hits);

            for (int i = 0; i < count; ++i)
            {
                IntersectionPoint point = hits[i];
                if (!a.IsLine())
                    std::swap(point.parameterA, point.parameterB);
                out.push_back(point);
            }
            return;
        }

        Cubics(a, b, context, out);
    }

    bool AnyCubicIntersection(const Segment& a, const Segment& b,
                              const GeometryTolerance& tolerance, int maxDepth)
    {
        // F35a, as in Cubics: the box is a certified enclosure of the true restriction of
        // the ORIGINAL curve, carried with the piece rather than rebuilt on every pop.
        // The parameter spans exist here purely to address CertifiedSubCurve - this
        // any-hit form reports no parameters - and they cost four doubles per stack entry
        // in a 64-entry automatic array.
        struct Pair
        {
            Segment first;
            Segment second;
            int depth;
            double a0, a1;
            double b0, b1;
            Bounds2 boundsA;
            Bounds2 boundsB;
        };

        const auto boxA = [&a](double t0, double t1) noexcept
        {
            return CubicBezier::CertifiedSubCurve(a, t0, t1).box;
        };
        const auto boxB = [&b](double t0, double t1) noexcept
        {
            return CubicBezier::CertifiedSubCurve(b, t0, t1).box;
        };

        // Fixed stack in automatic storage. Each pop pushes at most two, and depth is
        // capped, so 64 entries cannot be exceeded.
        constexpr int kCapacity = 64;
        Pair stack[kCapacity];

        maxDepth = std::clamp(maxDepth, 1, 30);

        int top = 0;
        stack[top++] = { a, b, 0, 0.0, 1.0, 0.0, 1.0, boxA(0.0, 1.0), boxB(0.0, 1.0) };

        while (top > 0)
        {
            const Pair pair = stack[--top];

            const Bounds2& boundsA = pair.boundsA;
            const Bounds2& boundsB = pair.boundsB;
            if (!boundsA.Overlaps(boundsB, tolerance.intersection))
                continue;

            const bool flatA = CubicBezier::IsGeometricallyFlat(pair.first, tolerance.intersection);
            const bool flatB = CubicBezier::IsGeometricallyFlat(pair.second, tolerance.intersection);

            if ((flatA && flatB) || pair.depth >= maxDepth || top + 2 > kCapacity)
            {
                if (LinesIntersect(pair.first.p0, pair.first.p1,
                                   pair.second.p0, pair.second.p1, tolerance))
                {
                    return true;
                }
                continue;
            }

            Segment left, right;
            if (!flatA && (flatB || boundsA.Perimeter() >= boundsB.Perimeter()))
            {
                CubicBezier::Split(pair.first, 0.5, left, right);
                const double middle = 0.5 * (pair.a0 + pair.a1);
                stack[top++] = { left, pair.second, pair.depth + 1, pair.a0, middle,
                                 pair.b0, pair.b1, boxA(pair.a0, middle), boundsB };
                stack[top++] = { right, pair.second, pair.depth + 1, middle, pair.a1,
                                 pair.b0, pair.b1, boxA(middle, pair.a1), boundsB };
            }
            else
            {
                CubicBezier::Split(pair.second, 0.5, left, right);
                const double middle = 0.5 * (pair.b0 + pair.b1);
                stack[top++] = { pair.first, left, pair.depth + 1, pair.a0, pair.a1,
                                 pair.b0, middle, boundsA, boxB(pair.b0, middle) };
                stack[top++] = { pair.first, right, pair.depth + 1, pair.a0, pair.a1,
                                 middle, pair.b1, boundsA, boxB(middle, pair.b1) };
            }
        }

        return false;
    }

    bool FindFirstCubicIntersection(const Segment& a, const Segment& b,
                                    const GeometryTolerance& tolerance,
                                    IntersectionPoint& out, int maxDepth)
    {
        struct Pair
        {
            Segment first;
            Segment second;
            int depth;

            // Parameter span each piece occupies on its original curve. Without this the
            // reported parameters would be local to whatever fragment the subdivision
            // happened to stop on - and, since F35a, it is also what lets the certified
            // enclosure be taken against the ORIGINAL curve rather than against a chain
            // of rounded Split results.
            double a0, a1;
            double b0, b1;

            Bounds2 boundsA;
            Bounds2 boundsB;
        };

        const auto boxA = [&a](double t0, double t1) noexcept
        {
            return CubicBezier::CertifiedSubCurve(a, t0, t1).box;
        };
        const auto boxB = [&b](double t0, double t1) noexcept
        {
            return CubicBezier::CertifiedSubCurve(b, t0, t1).box;
        };

        constexpr int kCapacity = 64;
        Pair stack[kCapacity];

        maxDepth = std::clamp(maxDepth, 1, 30);

        int top = 0;
        stack[top++] = { a, b, 0, 0.0, 1.0, 0.0, 1.0, boxA(0.0, 1.0), boxB(0.0, 1.0) };

        while (top > 0)
        {
            const Pair pair = stack[--top];

            const Bounds2& boundsA = pair.boundsA;
            const Bounds2& boundsB = pair.boundsB;
            if (!boundsA.Overlaps(boundsB, tolerance.intersection))
                continue;

            const bool flatA = CubicBezier::IsGeometricallyFlat(pair.first, tolerance.intersection);
            const bool flatB = CubicBezier::IsGeometricallyFlat(pair.second, tolerance.intersection);

            if ((flatA && flatB) || pair.depth >= maxDepth || top + 2 > kCapacity)
            {
                const LineIntersection hit = Lines(pair.first.p0, pair.first.p1,
                                                   pair.second.p0, pair.second.p1, tolerance);
                if (!IsContact(hit.kind))
                    continue;

                out.point = hit.point;
                out.kind = hit.kind;
                out.parameterA = pair.a0 + (pair.a1 - pair.a0) * hit.parameterA;
                out.parameterB = pair.b0 + (pair.b1 - pair.b0) * hit.parameterB;
                return true;
            }

            Segment left, right;
            if (!flatA && (flatB || boundsA.Perimeter() >= boundsB.Perimeter()))
            {
                const double mid = 0.5 * (pair.a0 + pair.a1);
                CubicBezier::Split(pair.first, 0.5, left, right);
                stack[top++] = { left, pair.second, pair.depth + 1, pair.a0, mid,
                                 pair.b0, pair.b1, boxA(pair.a0, mid), boundsB };
                stack[top++] = { right, pair.second, pair.depth + 1, mid, pair.a1,
                                 pair.b0, pair.b1, boxA(mid, pair.a1), boundsB };
            }
            else
            {
                const double mid = 0.5 * (pair.b0 + pair.b1);
                CubicBezier::Split(pair.second, 0.5, left, right);
                stack[top++] = { pair.first, left, pair.depth + 1, pair.a0, pair.a1,
                                 pair.b0, mid, boundsA, boxB(pair.b0, mid) };
                stack[top++] = { pair.first, right, pair.depth + 1, pair.a0, pair.a1,
                                 mid, pair.b1, boundsA, boxB(mid, pair.b1) };
            }
        }

        return false;
    }

    bool FindFirstSegmentIntersection(const Segment& a, const Segment& b,
                                      const GeometryTolerance& tolerance, IntersectionPoint& out)
    {
        // Line/line first, for the reason given in AnySegmentIntersection: the exact
        // predicate is cheaper than the certified box that would screen for it.
        if (a.IsLine() && b.IsLine())
        {
            const LineIntersection hit = Lines(a.p0, a.p1, b.p0, b.p1, tolerance);
            if (!IsContact(hit.kind))
                return false;

            out.point = hit.point;
            out.parameterA = hit.parameterA;
            out.parameterB = hit.parameterB;
            out.kind = hit.kind;
            return true;
        }

        // F35a. The curved pairs keep the reject, and it DISCARDS, so the box has to be
        // an enclosure - the tight box is short of the true extrema by up to 4 ULP of
        // the coordinate, which exceeds tolerance.intersection inside the documented
        // envelope. Here the box is worth its cost: what it screens for is a root solve,
        // not four orientation predicates.
        if (!CubicBezier::CertifiedExactBounds(a).Overlaps(
                CubicBezier::CertifiedExactBounds(b), tolerance.intersection))
            return false;

        if (a.IsLine() != b.IsLine())
        {
            const bool lineIsA = a.IsLine();
            const Segment& line = lineIsA ? a : b;
            const Segment& curve = lineIsA ? b : a;

            IntersectionPoint hits[3];
            const int count = LineCubic(line.p0, line.p1, curve, tolerance, hits);
            if (count <= 0)
                return false;

            // LineCubic reports parameterA on the line and parameterB on the curve;
            // swap them back when the caller's A was the curve.
            out = hits[0];
            if (!lineIsA)
                std::swap(out.parameterA, out.parameterB);
            return true;
        }

        return FindFirstCubicIntersection(a, b, tolerance, out);
    }

    bool AnySegmentIntersection(const Segment& a, const Segment& b, const GeometryTolerance& tolerance)
    {
        // LINE/LINE FIRST, AND WITHOUT A BOX AT ALL.
        //
        // LinesIntersect is exact - four Orient2D predicates - and cheaper than either
        // box. Rejecting first was worth it when the box was two Add()s; once the box
        // has to be a certified enclosure it is strictly more expensive than the answer
        // it was screening for, and this is the hottest narrow-phase pair in the kernel:
        // flattened artwork is line segments, and Collision::Intersects calls this once
        // per BVH candidate pair. Measured on `regionqueryv811`, screening first cost 6
        // to 9 per cent of every prepared region query.
        //
        // Removing a prune can only ADD work, never drop a candidate, so this is sound by
        // construction rather than by a bound.
        if (a.IsLine() && b.IsLine())
            return LinesIntersect(a.p0, a.p1, b.p0, b.p1, tolerance);

        // F35a. For the curved pairs the reject stays, and it DISCARDS, so the box has to
        // be an enclosure. The tight box is short of the true extrema by up to 4 ULP,
        // and nothing here enforced the relation to tolerance.intersection that its
        // soundness rested on.
        if (!CubicBezier::CertifiedExactBounds(a).Overlaps(
                CubicBezier::CertifiedExactBounds(b), tolerance.intersection))
            return false;

        if (a.IsLine() != b.IsLine())
        {
            const Segment& line = a.IsLine() ? a : b;
            const Segment& curve = a.IsLine() ? b : a;
            IntersectionPoint hits[3];
            return LineCubic(line.p0, line.p1, curve, tolerance, hits) > 0;
        }

        return AnyCubicIntersection(a, b, tolerance);
    }

    bool SegmentsIntersect(const Segment& a, const Segment& b, const GeometryContext& context)
    {
        // Allocation-free for every segment-kind pair, cubic-cubic included.
        //
        // This used to answer the curve-curve case by calling Cubics() into a local
        // std::vector and testing whether it was empty: an allocation per candidate pair
        // on a pure yes/no query, and the function was declared noexcept, so a throwing
        // allocation would have called std::terminate rather than propagating. V3 had
        // already built the allocation-free any-hit for PreparedCollision; the generic
        // API simply was not using it.
        //
        // `noexcept` is deliberately gone rather than kept now that nothing allocates.
        // The predicates below make no such declaration themselves, and claiming a
        // guarantee the call chain does not provide is how the previous version turned a
        // recoverable failure into process termination.
        return AnySegmentIntersection(a, b, context.Tolerance());
    }
}
