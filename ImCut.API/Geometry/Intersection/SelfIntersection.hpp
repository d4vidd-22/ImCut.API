#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Spatial/SegmentBVH.hpp"
#include "Intersection.hpp"

#include <cstdint>
#include <vector>

namespace ImCut::Geometry
{
    struct SelfIntersectionFinding
    {
        std::uint32_t segmentA = 0;
        std::uint32_t segmentB = 0;
        IntersectionPoint point;
    };

    struct SelfIntersectionResult
    {
        std::vector<SelfIntersectionFinding> findings;

        // True when the complexity guard stopped the search, so `findings` is a
        // partial answer and "no more intersections" must not be inferred from it.
        bool truncated = false;

        [[nodiscard]] bool Empty() const noexcept { return findings.empty(); }
    };

    namespace Intersect
    {
        // Finds every place a contour crosses itself.
        //
        // Candidate pairs come from the BVH, so cost tracks the number of pairs that
        // actually share space rather than the square of the segment count. On the
        // reference corpus a contour of ~1.6k segments would be 1.3M naive pair tests.
        //
        // Segments that merely share a node with their neighbour are not a defect, so
        // contacts located at that shared node are filtered out. Adjacent segments are
        // still tested against each other, because a contour that doubles back can
        // genuinely cross its own neighbour away from the shared node.
        [[nodiscard]] GeometryResult<SelfIntersectionResult> SelfIntersections(
            const Contour& contour, const GeometryContext& context);

        // Same, reusing a hierarchy the caller already built over the contour's
        // segments. This is the form PreparedGeometry uses so the BVH is paid for once.
        [[nodiscard]] GeometryResult<SelfIntersectionResult> SelfIntersections(
            const Contour& contour, const SegmentBVH& bvh, const GeometryContext& context);

        // Stops at the first genuine self-intersection.
        //
        // Returns a result, not a bare bool. A cancelled or budget-limited search knows
        // nothing about the geometry, and reporting "false" there reads as "verified
        // safe" - the most dangerous possible answer for a validity query.
        [[nodiscard]] GeometryResult<bool> HasSelfIntersection(const Contour& contour,
                                                               const GeometryContext& context);

        // Intersections between two contours whose hierarchies the caller already has.
        //
        // Validation compares every pair of contours, and building two hierarchies per
        // pair made that C*(C-1) builds for C contours - 1,000,000 builds at C = 1000,
        // against the 1000 the geometry actually needs. The arrays must be the ones the
        // hierarchies were built from; the traversal indexes them directly.
        [[nodiscard]] GeometryResult<SelfIntersectionResult> SegmentsIntersections(
            const Segment* segmentsA, std::size_t countA, const SegmentBVH& hierarchyA,
            const Segment* segmentsB, std::size_t countB, const SegmentBVH& hierarchyB,
            const GeometryContext& context);

        // Intersections between two different contours. Shared endpoints are real
        // findings here, since separate contours are not expected to touch.
        [[nodiscard]] GeometryResult<SelfIntersectionResult> ContourIntersections(
            const Contour& a, const Contour& b, const GeometryContext& context);

        // Do any two segments of the two arrays cross?
        //
        // The hierarchies come from the caller, so a Path-level query can build ONE tree
        // over all its contours instead of one per contour pair. Measured, the build was
        // 64% to 94% of an unprepared query, and the pairwise loop multiplied it.
        //
        // `segmentsA` and `segmentsB` must be the arrays the hierarchies were built
        // from; the traversal indexes them directly.
        [[nodiscard]] GeometryResult<bool> SegmentsIntersectAny(
            const Segment* segmentsA, std::size_t countA, const SegmentBVH& hierarchyA,
            const Segment* segmentsB, std::size_t countB, const SegmentBVH& hierarchyB,
            const GeometryContext& context);

        [[nodiscard]] GeometryResult<bool> ContoursIntersect(const Contour& a, const Contour& b,
                                                             const GeometryContext& context);
    }
}
