#include "SelfIntersection.hpp"

#include "../Curves/CubicBezier.hpp"

#include <algorithm>
#include <cmath>

namespace ImCut::Geometry::Intersect
{
    namespace
    {
        // Collects a contour's segments once so both the hierarchy and the narrow
        // phase read from contiguous memory instead of rebuilding each segment.
        //
        // THE STRUCTURAL GATE LIVES HERE, AND IT RETURNS A VALUE ON PURPOSE.
        //
        // SegmentAt() indexes three parallel arrays with no bounds check. That is sound
        // only behind Contour::IsStructurallyValid, and this file used to apply that gate
        // at the two entry points that return a bool - HasSelfIntersection and
        // ContoursIntersect - and forget it at the three that return a
        // SelfIntersectionResult. `SelfIntersections(contour, context)` on a contour with
        // four nodes, `closed = true` and EMPTY handles and kinds read handles[0] of a
        // zero-length vector and took the process down with 0xC0000005 - no status, no
        // exception, on a public entry point reachable from anything that builds a
        // Contour by hand: an importer, a file, a fuzzer.
        //
        // A guard at three more call sites would leave the class alive. The gather is the
        // one thing every route here funnels through, so the gate is here, it is
        // [[nodiscard]], and a caller that ignores it does not compile clean.
        [[nodiscard]] bool GatherSegments(const Contour& contour, std::vector<Segment>& out)
        {
            out.clear();
            if (!contour.IsStructurallyValid())
                return false;

            const std::size_t count = contour.SegmentCount();
            out.reserve(count);
            for (std::size_t i = 0; i < count; ++i)
                out.push_back(contour.SegmentAt(i));
            return true;
        }

        [[nodiscard]] bool AreNeighbours(std::uint32_t a, std::uint32_t b,
                                         std::size_t segmentCount, bool closed) noexcept
        {
            if (a > b) std::swap(a, b);
            if (b == a + 1)
                return true;

            // On a closed contour the last segment is also the first one's neighbour.
            return closed && a == 0 && b + 1 == segmentCount;
        }

        // Every node the two neighbouring segments legitimately share.
        //
        // Usually there is exactly one, but a closed contour built from only two
        // segments - a lens or a teardrop, both common in real artwork - joins at both
        // ends. Filtering a single shared node there would report the second junction
        // as a self-intersection.
        struct SharedNodes
        {
            Vec2 points[2];
            int count = 0;

            void Add(Vec2 point, double toleranceSquared) noexcept
            {
                for (int i = 0; i < count; ++i)
                {
                    if (DistanceSquared(points[i], point) <= toleranceSquared)
                        return;
                }
                if (count < 2)
                    points[count++] = point;
            }

            [[nodiscard]] bool Contains(Vec2 point, double toleranceSquared) const noexcept
            {
                for (int i = 0; i < count; ++i)
                {
                    if (DistanceSquared(points[i], point) <= toleranceSquared)
                        return true;
                }
                return false;
            }
        };

        [[nodiscard]] SharedNodes CollectSharedNodes(const Segment& a, const Segment& b,
                                                     double tolerance) noexcept
        {
            const double toleranceSquared = tolerance * tolerance;
            SharedNodes shared;

            if (DistanceSquared(a.p1, b.p0) <= toleranceSquared) shared.Add(a.p1, toleranceSquared);
            if (DistanceSquared(a.p0, b.p1) <= toleranceSquared) shared.Add(a.p0, toleranceSquared);
            if (DistanceSquared(a.p0, b.p0) <= toleranceSquared) shared.Add(a.p0, toleranceSquared);
            if (DistanceSquared(a.p1, b.p1) <= toleranceSquared) shared.Add(a.p1, toleranceSquared);

            return shared;
        }

        [[nodiscard]] GeometryResult<SelfIntersectionResult> RunSelfIntersections(
            const Contour& contour, const SegmentBVH& bvh, const GeometryContext& context,
            bool stopAtFirst)
        {
            SelfIntersectionResult result;

            const std::size_t segmentCount = contour.SegmentCount();
            if (segmentCount < 2)
                return GeometryResult<SelfIntersectionResult>::Empty(std::move(result));

            std::vector<Segment> segments;
            if (!GatherSegments(contour, segments))
            {
                return GeometryResult<SelfIntersectionResult>::Failure(
                    GeometryStatus::InvalidInput);
            }

            const GeometryTolerance& tolerance = context.Tolerance();
            const std::size_t maxFindings = context.Limits().maxIntersections;

            std::vector<IntersectionPoint> scratch;
            bool cancelled = false;
            std::uint64_t tested = 0;

            bvh.QuerySelfPairs([&](std::uint32_t indexA, std::uint32_t indexB)
            {
                if (context.ShouldCheckCancellation(static_cast<std::size_t>(tested)) && context.IsCancelled())
                {
                    cancelled = true;
                    return false;
                }
                ++tested;

                const Segment& a = segments[indexA];
                const Segment& b = segments[indexB];

                const bool neighbours = AreNeighbours(indexA, indexB, segmentCount, contour.closed);

                scratch.clear();
                Segments(a, b, context, scratch);
                if (scratch.empty())
                    return true;

                SharedNodes shared;
                if (neighbours)
                    shared = CollectSharedNodes(a, b, tolerance.duplicate);

                // A shared node is expected geometry, not a defect. Anything else two
                // neighbours do to each other still counts.
                const double sharedToleranceSquared = tolerance.duplicate * tolerance.duplicate;

                for (const IntersectionPoint& point : scratch)
                {
                    if (neighbours && shared.Contains(point.point, sharedToleranceSquared))
                        continue;

                    SelfIntersectionFinding finding;
                    finding.segmentA = indexA;
                    finding.segmentB = indexB;
                    finding.point = point;
                    result.findings.push_back(finding);

                    if (stopAtFirst)
                        return false;

                    if (result.findings.size() >= maxFindings)
                    {
                        result.truncated = true;
                        return false;
                    }
                }

                return true;
            });

            CountStat(context, &GeometryStatistics::bvhCandidatePairs, tested);

            if (cancelled)
                return GeometryResult<SelfIntersectionResult>::Failure(GeometryStatus::Cancelled);

            // Stable ordering so repeated runs, and runs across different thread
            // counts, produce byte-identical findings.
            std::sort(result.findings.begin(), result.findings.end(),
                      [](const SelfIntersectionFinding& lhs, const SelfIntersectionFinding& rhs) noexcept
                      {
                          if (lhs.segmentA != rhs.segmentA) return lhs.segmentA < rhs.segmentA;
                          if (lhs.segmentB != rhs.segmentB) return lhs.segmentB < rhs.segmentB;
                          if (lhs.point.parameterA != rhs.point.parameterA)
                              return lhs.point.parameterA < rhs.point.parameterA;
                          return lhs.point.parameterB < rhs.point.parameterB;
                      });

            if (result.findings.empty())
                return GeometryResult<SelfIntersectionResult>::Empty(std::move(result));

            return GeometryResult<SelfIntersectionResult>::Success(std::move(result));
        }
    }

    GeometryResult<SelfIntersectionResult> SelfIntersections(const Contour& contour,
                                                             const SegmentBVH& bvh,
                                                             const GeometryContext& context)
    {
        return RunSelfIntersections(contour, bvh, context, false);
    }

    GeometryResult<SelfIntersectionResult> SelfIntersections(const Contour& contour,
                                                             const GeometryContext& context)
    {
        std::vector<Segment> segments;
        if (!GatherSegments(contour, segments))
        {
            return GeometryResult<SelfIntersectionResult>::Failure(
                GeometryStatus::InvalidInput);
        }

        SegmentBVH bvh;
        bvh.BuildFromSegments(segments.data(), segments.size(), context);

        return RunSelfIntersections(contour, bvh, context, false);
    }

    GeometryResult<bool> HasSelfIntersection(const Contour& contour, const GeometryContext& context)
    {
        if (!contour.IsStructurallyValid())
            return GeometryResult<bool>::Failure(GeometryStatus::InvalidInput);

        std::vector<Segment> segments;
        if (!GatherSegments(contour, segments))
            return GeometryResult<bool>::Failure(GeometryStatus::InvalidInput);

        SegmentBVH bvh;
        bvh.BuildFromSegments(segments.data(), segments.size(), context);

        const auto result = RunSelfIntersections(contour, bvh, context, true);
        if (!result.Ok())
            return GeometryResult<bool>::Failure(result.Status());

        return GeometryResult<bool>::Success(!result.Value().findings.empty());
    }

    GeometryResult<SelfIntersectionResult> SegmentsIntersections(
        const Segment* segmentsA, std::size_t countA, const SegmentBVH& bvhA,
        const Segment* segmentsB, std::size_t countB, const SegmentBVH& bvhB,
        const GeometryContext& context)
    {
        SelfIntersectionResult result;

        if (segmentsA == nullptr || segmentsB == nullptr || countA == 0 || countB == 0)
            return GeometryResult<SelfIntersectionResult>::Empty(std::move(result));

        const std::size_t maxFindings = context.Limits().maxIntersections;

        std::vector<IntersectionPoint> scratch;
        bool cancelled = false;
        std::uint64_t tested = 0;

        bvhA.QueryPairs(bvhB, [&](std::uint32_t indexA, std::uint32_t indexB)
        {
            if (context.ShouldCheckCancellation(static_cast<std::size_t>(tested)) && context.IsCancelled())
            {
                cancelled = true;
                return false;
            }
            ++tested;

            scratch.clear();
            Segments(segmentsA[indexA], segmentsB[indexB], context, scratch);

            for (const IntersectionPoint& point : scratch)
            {
                SelfIntersectionFinding finding;
                finding.segmentA = indexA;
                finding.segmentB = indexB;
                finding.point = point;
                result.findings.push_back(finding);

                if (result.findings.size() >= maxFindings)
                {
                    result.truncated = true;
                    return false;
                }
            }

            return true;
        });

        CountStat(context, &GeometryStatistics::bvhCandidatePairs, tested);

        if (cancelled)
            return GeometryResult<SelfIntersectionResult>::Failure(GeometryStatus::Cancelled);

        std::sort(result.findings.begin(), result.findings.end(),
                  [](const SelfIntersectionFinding& lhs, const SelfIntersectionFinding& rhs) noexcept
                  {
                      if (lhs.segmentA != rhs.segmentA) return lhs.segmentA < rhs.segmentA;
                      if (lhs.segmentB != rhs.segmentB) return lhs.segmentB < rhs.segmentB;
                      if (lhs.point.parameterA != rhs.point.parameterA)
                          return lhs.point.parameterA < rhs.point.parameterA;
                      return lhs.point.parameterB < rhs.point.parameterB;
                  });

        if (result.findings.empty())
            return GeometryResult<SelfIntersectionResult>::Empty(std::move(result));

        return GeometryResult<SelfIntersectionResult>::Success(std::move(result));
    }

    GeometryResult<SelfIntersectionResult> ContourIntersections(const Contour& a, const Contour& b,
                                                                const GeometryContext& context)
    {
        std::vector<Segment> segmentsA;
        std::vector<Segment> segmentsB;
        if (!GatherSegments(a, segmentsA) || !GatherSegments(b, segmentsB))
        {
            return GeometryResult<SelfIntersectionResult>::Failure(
                GeometryStatus::InvalidInput);
        }

        if (segmentsA.empty() || segmentsB.empty())
            return GeometryResult<SelfIntersectionResult>::Empty(SelfIntersectionResult{});

        SegmentBVH bvhA;
        SegmentBVH bvhB;
        bvhA.BuildFromSegments(segmentsA.data(), segmentsA.size(), context);
        bvhB.BuildFromSegments(segmentsB.data(), segmentsB.size(), context);

        return SegmentsIntersections(segmentsA.data(), segmentsA.size(), bvhA,
                                     segmentsB.data(), segmentsB.size(), bvhB, context);
    }

    GeometryResult<bool> SegmentsIntersectAny(const Segment* segmentsA, std::size_t countA,
                                              const SegmentBVH& bvhA,
                                              const Segment* segmentsB, std::size_t countB,
                                              const SegmentBVH& bvhB,
                                              const GeometryContext& context)
    {
        if (segmentsA == nullptr || segmentsB == nullptr || countA == 0 || countB == 0)
            return GeometryResult<bool>::Success(false);

        // UP FRONT, BECAUSE THE CHECK BELOW LIVES INSIDE THE CALLBACK.
        //
        // QueryPairs only invokes its callback for CANDIDATE pairs. Two boundaries that
        // never come close - one region strictly inside another, which is the ordinary
        // containment case - produce no candidates at all, so the cancellation check in
        // the lambda never executes and the query answers Success after the caller has
        // already cancelled. Measured on V8.1: Collision::Intersects built two hierarchies
        // and returned Success(false) with an already-set token (evidence 892), while
        // Overlaps and Contains on the same fixture reported Cancelled.
        //
        // One relaxed atomic load, once per query, on a path that has just built two
        // bounding-volume hierarchies.
        if (context.IsCancelled())
            return GeometryResult<bool>::Failure(GeometryStatus::Cancelled);

        bool found = false;
        bool cancelled = false;
        std::uint64_t tested = 0;

        bvhA.QueryPairs(bvhB, [&](std::uint32_t indexA, std::uint32_t indexB)
        {
            if (context.ShouldCheckCancellation(static_cast<std::size_t>(tested)) && context.IsCancelled())
            {
                cancelled = true;
                return false;
            }
            ++tested;

            if (SegmentsIntersect(segmentsA[indexA], segmentsB[indexB], context))
            {
                found = true;
                return false;
            }
            return true;
        });

        if (cancelled)
            return GeometryResult<bool>::Failure(GeometryStatus::Cancelled);

        return GeometryResult<bool>::Success(found);
    }

    GeometryResult<bool> ContoursIntersect(const Contour& a, const Contour& b,
                                           const GeometryContext& context)
    {
        if (!a.IsStructurallyValid() || !b.IsStructurallyValid())
            return GeometryResult<bool>::Failure(GeometryStatus::InvalidInput);

        std::vector<Segment> segmentsA;
        std::vector<Segment> segmentsB;
        if (!GatherSegments(a, segmentsA) || !GatherSegments(b, segmentsB))
            return GeometryResult<bool>::Failure(GeometryStatus::InvalidInput);

        if (segmentsA.empty() || segmentsB.empty())
            return GeometryResult<bool>::Success(false);

        SegmentBVH bvhA;
        SegmentBVH bvhB;
        bvhA.BuildFromSegments(segmentsA.data(), segmentsA.size(), context);
        bvhB.BuildFromSegments(segmentsB.data(), segmentsB.size(), context);

        return SegmentsIntersectAny(segmentsA.data(), segmentsA.size(), bvhA,
                                    segmentsB.data(), segmentsB.size(), bvhB, context);
    }
}
