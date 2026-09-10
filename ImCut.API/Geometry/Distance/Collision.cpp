#include "Collision.hpp"

#include "../Curves/Flatten.hpp"
#include "../Intersection/SelfIntersection.hpp"
#include "../Math/Predicates.hpp"
#include "../Polygon/Boolean.hpp"
#include "../Polygon/PolygonMetrics.hpp"
#include "../Topology/ContainmentTree.hpp"
#include "Distance.hpp"
#include "../Spatial/SegmentBVH.hpp"

#include <algorithm>
#include <limits>

namespace ImCut::Geometry::Collision
{
    namespace
    {
        [[nodiscard]] GeometryResult<std::vector<Vec2>> FlattenRing(const Contour& contour,
                                                                     const GeometryContext& context)
        {
            FlattenOptions options;
            options.tolerance = context.Tolerance().flatten;
            options.maxDepth = context.Limits().maxSubdivisionDepth;

            GeometryResult<FlattenedContour> flattened = Flatten::Contour(contour, options, context);
            if (!flattened.Ok())
                return GeometryResult<std::vector<Vec2>>::Failure(flattened.Status());

            return GeometryResult<std::vector<Vec2>>::Success(std::move(flattened.Value().points));
        }

    }

    // F35a, class sweep. This REJECTS - a false here ends the query with "disjoint" -
    // so both boxes have to contain their shapes. `Metrics::ComputeBounds` is the tight
    // measurement and falls short of the true extrema by up to 4 ULP of the coordinate,
    // which at the documented 1e9 mm ceiling is 4.8e-7 mm against a `tolerance` of 1e-7
    // by default and 1e-9 under PrecisionMode::Maximum. The margin does not dominate the
    // defect inside the supported envelope, so the enclosure is taken as an enclosure.
    bool BoundsCouldTouch(const Path& a, const Path& b, double tolerance) noexcept
    {
        return Metrics::ComputeEnclosure(a).Box().Overlaps(
            Metrics::ComputeEnclosure(b).Box(), tolerance);
    }

    GeometryResult<bool> Intersects(const Contour& a, const Contour& b, const GeometryContext& context)
    {
        return Intersect::ContoursIntersect(a, b, context);
    }

    GeometryResult<bool> Intersects(const Path& a, const Path& b, const GeometryContext& context)
    {
        if (!a.IsStructurallyValid() || !b.IsStructurallyValid())
            return GeometryResult<bool>::Failure(GeometryStatus::InvalidInput);

        if (!BoundsCouldTouch(a, b, context.Tolerance().intersection))
            return GeometryResult<bool>::Success(false);

        // One hierarchy per PATH, for the same reason as MinimumDistance below: this
        // looped over contour pairs and each iteration built two trees from scratch.
        std::vector<Segment> segmentsA;
        std::vector<Segment> segmentsB;
        SegmentBVH hierarchyA;
        SegmentBVH hierarchyB;

        if (!Proximity::BuildPathHierarchy(a, context, segmentsA, hierarchyA) ||
            !Proximity::BuildPathHierarchy(b, context, segmentsB, hierarchyB))
        {
            return GeometryResult<bool>::Success(false);
        }

        return Intersect::SegmentsIntersectAny(segmentsA.data(), segmentsA.size(), hierarchyA,
                                               segmentsB.data(), segmentsB.size(), hierarchyB,
                                               context);
    }

    GeometryResult<bool> FindFirstIntersection(const Contour& a, const Contour& b,
                                               const GeometryContext& context,
                                               IntersectionPoint& out)
    {
        const auto result = Intersect::ContourIntersections(a, b, context);
        if (!result.Ok())
            return GeometryResult<bool>::Failure(result.Status());

        if (result.Value().findings.empty())
            return GeometryResult<bool>::Success(false);

        // ContourIntersections sorts its findings, so "first" is well defined.
        out = result.Value().findings.front().point;
        return GeometryResult<bool>::Success(true);
    }

    GeometryResult<std::vector<IntersectionPoint>> FindAllIntersections(const Contour& a, const Contour& b,
                                                                        const GeometryContext& context)
    {
        std::vector<IntersectionPoint> points;

        const auto result = Intersect::ContourIntersections(a, b, context);
        if (!result.Ok())
            return GeometryResult<std::vector<IntersectionPoint>>::Failure(result.Status());

        points.reserve(result.Value().findings.size());
        for (const SelfIntersectionFinding& finding : result.Value().findings)
            points.push_back(finding.point);

        Intersect::MergeNearbyPoints(points, context.Tolerance().duplicate);

        if (points.empty())
            return GeometryResult<std::vector<IntersectionPoint>>::Empty(std::move(points));

        return GeometryResult<std::vector<IntersectionPoint>>::Success(std::move(points));
    }

    GeometryResult<bool> Overlaps(const Path& a, const Path& b, const GeometryContext& context)
    {
        if (!BoundsCouldTouch(a, b, context.Tolerance().intersection))
            return GeometryResult<bool>::Success(false);

        // Crossing boundaries is sufficient but not necessary: one region nested inside
        // the other overlaps without any boundary contact at all.
        const auto crossing = Intersects(a, b, context);
        if (!crossing.Ok())
            return crossing;
        if (crossing.Value())
            return GeometryResult<bool>::Success(true);

        const auto intersection = Boolean::Intersection(a, b, context);
        if (!intersection.Ok())
            return GeometryResult<bool>::Failure(intersection.Status());

        return GeometryResult<bool>::Success(!intersection.Value().contours.empty());
    }

    GeometryResult<bool> Contains(const Path& outer, const Path& inner, const GeometryContext& context)
    {
        if (inner.contours.empty())
            return GeometryResult<bool>::Success(true);
        if (outer.contours.empty())
            return GeometryResult<bool>::Success(false);

        // Lemma B4, in the raw route. This PUBLISHES `false`, so the asymmetry is not
        // optional: an ENCLOSURE on the container side, a WITNESS on the contained side.
        // Two tight boxes had it backwards on the container side - `outer`'s box is short
        // of `outer`, so a part genuinely inside it but reaching its extreme could be
        // reported as escaping. The prepared route has spelled this out since F38
        // (RefutesContainment takes an ExtentWitness and a BoundsEnclosure by type); the
        // raw route it is required to agree with was still comparing two measurements.
        const BoundsEnclosure outerBox = Metrics::ComputeEnclosure(outer);
        const BoundsWitness innerReach = Metrics::ComputeWitness(inner, Transform2::Identity());
        if (ProvesNotContained(innerReach, outerBox, 0.0))
            return GeometryResult<bool>::Success(false);

        // Any boundary crossing means part of `inner` escapes. Kept in front because it
        // is the cheap rejection and it decides most real calls.
        const auto crossing = Intersects(outer, inner, context);
        if (!crossing.Ok())
            return crossing;
        if (crossing.Value())
            return GeometryResult<bool>::Success(false);

        // Contains(A, B) is true if and only if B minus A is empty, so ask exactly that.
        //
        // This used to be one interior probe per ring of `inner` against `outer`'s
        // containment tree. A probe cannot see a hole of `outer` that lies strictly
        // inside `inner`: no boundary crosses, because the hole is entirely enclosed,
        // and the probe at the centre of `inner` lands on material. A 15x15 part laid
        // over a 2x2 hole was therefore reported contained, with 4 mm^2 of it sitting
        // over void - the IFP predicate approving a placement on a cut-out.
        //
        // Difference costs about 33 us against the probe's 1.3 us. Contains is not an
        // ultra-hot path - the bounds test and the crossing test above answer the
        // common cases first - and the alternative is a wrong answer.
        const auto residue = Boolean::Difference(inner, outer, context);
        if (!IsSuccess(residue.Status()))
            return GeometryResult<bool>::Failure(residue.Status());

        // Empty is a success status carrying no geometry: nothing of `inner` fell
        // outside `outer`, which is precisely containment.
        const bool nothingEscaped = residue.Status() == GeometryStatus::Empty ||
                                    residue.Value().contours.empty();
        return GeometryResult<bool>::Success(nothingEscaped);
    }

    double MinimumDistance(const Contour& a, const Contour& b, const GeometryContext& context)
    {
        return Proximity::ContourContour(a, b, context);
    }

    double MinimumDistance(const Path& a, const Path& b, const GeometryContext& context)
    {
        // One hierarchy per PATH, not one per contour pair.
        //
        // This iterated contour pairs and let ContourContour build two hierarchies for
        // each, so two 4-contour paths built 32 trees for 16 pairs. Measured, that was
        // 138.7 us against 14.7 us for a single pair - the setup, not the geometry.
        // Building one tree over every segment of each path also gives the traversal a
        // better bound to prune with, because it can reject a whole contour at a node
        // instead of re-descending into it once per partner.
        std::vector<Segment> segmentsA;
        std::vector<Segment> segmentsB;
        SegmentBVH hierarchyA;
        SegmentBVH hierarchyB;

        if (!Proximity::BuildPathHierarchy(a, context, segmentsA, hierarchyA) ||
            !Proximity::BuildPathHierarchy(b, context, segmentsB, hierarchyB))
        {
            return std::numeric_limits<double>::infinity();
        }

        return Proximity::SegmentsSegments(segmentsA.data(), segmentsA.size(), hierarchyA,
                                           segmentsB.data(), segmentsB.size(), hierarchyB,
                                           context);
    }
}
