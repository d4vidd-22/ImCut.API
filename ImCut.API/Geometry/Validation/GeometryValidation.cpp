#include "GeometryValidation.hpp"

#include "../Curves/CubicBezier.hpp"
#include "../Intersection/SelfIntersection.hpp"
#include "../Polygon/PolygonMetrics.hpp"
#include "../Spatial/SegmentBVH.hpp"
#include "../Topology/ContainmentTree.hpp"

#include <algorithm>
#include <cmath>

namespace ImCut::Geometry
{
    bool ValidationReport::Has(ValidationFinding finding) const noexcept
    {
        return std::any_of(issues.begin(), issues.end(),
                           [finding](const ValidationIssue& issue) noexcept
                           { return issue.finding == finding; });
    }

    std::size_t ValidationReport::Count(ValidationFinding finding) const noexcept
    {
        return static_cast<std::size_t>(
            std::count_if(issues.begin(), issues.end(),
                          [finding](const ValidationIssue& issue) noexcept
                          { return issue.finding == finding; }));
    }

    namespace Validation
    {
        namespace
        {
            class Collector
            {
            public:
                Collector(ValidationReport& report, std::size_t maxIssues)
                    : report_(report), maxIssues_(maxIssues) {}

                void Add(ValidationFinding finding, std::uint32_t contour,
                         std::uint32_t segment, Vec2 location,
                         IntersectionKind kind = IntersectionKind::None)
                {
                    if (CapacityReached())
                    {
                        MarkIncomplete();
                        return;
                    }
                    report_.issues.push_back({ finding, contour, segment, location, kind });
                }

                // Capacity and completeness are different facts. Merely storing exactly
                // maxIssues findings does not make the analysis partial: it is partial
                // only when another finding cannot be stored or requested work is
                // actually skipped. Every early-stop site goes through this method so a
                // caller cannot forget to update the report state.
                [[nodiscard]] bool ShouldStopRequestedWork() noexcept
                {
                    if (!CapacityReached())
                        return false;

                    MarkIncomplete();
                    return true;
                }

            private:
                [[nodiscard]] bool CapacityReached() const noexcept
                {
                    return report_.issues.size() >= maxIssues_;
                }

                void MarkIncomplete() noexcept
                {
                    report_.truncated = true;
                    if (report_.completeness == ValidationCompleteness::Complete)
                        report_.completeness = ValidationCompleteness::Partial;
                }

                ValidationReport& report_;
                std::size_t maxIssues_;
            };

            void ValidateContour(const Contour& contour, std::uint32_t index,
                                 const GeometryContext& context, const ValidationOptions& options,
                                 ValidationReport& report, Collector& collector)
            {
                const GeometryTolerance& tolerance = context.Tolerance();

                if (contour.nodes.empty())
                {
                    collector.Add(ValidationFinding::EmptyGeometry, index, 0, {});
                    return;
                }

                // Structure first. Everything below indexes the parallel arrays, and a
                // contour whose arrays disagree cannot be walked at all - reporting the
                // fact and stopping is the only safe response.
                bool finite = true;
                for (std::size_t i = 0; i < contour.nodes.size(); ++i)
                {
                    if (!IsFinite(contour.nodes[i]))
                    {
                        finite = false;
                        collector.Add(ValidationFinding::NonFiniteCoordinate, index,
                                      static_cast<std::uint32_t>(i), {});
                    }
                }
                for (const Vec2& handle : contour.handles)
                {
                    if (!IsFinite(handle))
                    {
                        finite = false;
                        collector.Add(ValidationFinding::NonFiniteCoordinate, index, 0, {});
                    }
                }

                if (!contour.HasConsistentArrays())
                {
                    collector.Add(ValidationFinding::InconsistentStorage, index, 0, {});
                    return;
                }

                // Non-finite coordinates poison area, bounds and every hierarchy built
                // from them, so the heavy checks are skipped rather than fed garbage.
                if (!finite)
                    return;

                const std::size_t segmentCount = contour.SegmentCount();
                report.segmentCount += segmentCount;

                if (!contour.closed)
                {
                    ++report.openContours;
                    collector.Add(ValidationFinding::OpenContour, index, 0, contour.nodes.front());
                }

                if (contour.nodes.size() < 3 && contour.closed)
                    collector.Add(ValidationFinding::InsufficientNodes, index, 0, contour.nodes.front());

                const double zeroLengthSquared = tolerance.zeroLength * tolerance.zeroLength;
                const double tinySquared = tolerance.tinySegment * tolerance.tinySegment;
                const double duplicateSquared = tolerance.duplicate * tolerance.duplicate;

                for (std::size_t i = 0;
                     i < segmentCount && !collector.ShouldStopRequestedWork(); ++i)
                {
                    const Segment segment = contour.SegmentAt(i);
                    const auto segmentIndex = static_cast<std::uint32_t>(i);

                    const double chordSquared = DistanceSquared(segment.p0, segment.p1);

                    if (segment.IsLine())
                    {
                        if (chordSquared <= zeroLengthSquared)
                            collector.Add(ValidationFinding::ZeroLengthSegment, index, segmentIndex, segment.p0);
                        else if (chordSquared <= tinySquared)
                            collector.Add(ValidationFinding::TinySegment, index, segmentIndex, segment.p0);
                    }
                    else if (CubicBezier::IsDegenerate(segment, tolerance.zeroLength))
                    {
                        // Endpoints and both handles collapsed: the curve has no extent.
                        collector.Add(ValidationFinding::DegenerateCurve, index, segmentIndex, segment.p0);
                    }
                    else if (chordSquared <= zeroLengthSquared &&
                             DistanceSquared(segment.c1, segment.p0) <= zeroLengthSquared &&
                             DistanceSquared(segment.c2, segment.p1) <= zeroLengthSquared)
                    {
                        collector.Add(ValidationFinding::ZeroLengthSegment, index, segmentIndex, segment.p0);
                    }

                    if (chordSquared <= duplicateSquared && segment.IsLine())
                        collector.Add(ValidationFinding::DuplicateNode, index, segmentIndex, segment.p0);
                }

                if (contour.closed && segmentCount >= 3)
                {
                    const double area = std::fabs(Metrics::SignedArea(contour));
                    const double threshold = tolerance.AreaThreshold(Metrics::ComputeBounds(contour).Area());
                    if (area <= threshold)
                        collector.Add(ValidationFinding::ZeroArea, index, 0, contour.nodes.front());
                }

                if (options.checkSelfIntersections && segmentCount >= 3)
                {
                    const auto self = Intersect::SelfIntersections(contour, context);
                    if (self.Ok())
                    {
                        for (const SelfIntersectionFinding& finding : self.Value().findings)
                        {
                            if (collector.ShouldStopRequestedWork())
                                break;

                            ++report.selfIntersections;
                            // The kind was already computed; keeping it is free and it
                            // is what lets a repair policy tell a proper crossing from a
                            // shared endpoint.
                            collector.Add(ValidationFinding::SelfIntersection, index,
                                          finding.segmentA, finding.point.point,
                                          finding.point.kind);
                        }
                        if (self.Value().truncated)
                            report.completeness = ValidationCompleteness::Partial;
                    }
                    else
                    {
                        // The search did not finish, so "no self-intersection found"
                        // here means nothing. Say so instead of implying a clean result.
                        report.completeness = self.Status() == GeometryStatus::Cancelled
                            ? ValidationCompleteness::Cancelled
                            : ValidationCompleteness::Partial;
                    }
                }
            }
        }

        bool IsStructurallySound(const Path& path) noexcept
        {
            return path.IsStructurallyValid();
        }

        GeometryResult<ValidationReport> Validate(const Path& path, const GeometryContext& context,
                                                  const ValidationOptions& options)
        {
            ValidationReport report;
            report.contourCount = path.contours.size();

            Collector collector(report, options.maxIssues);

            if (path.contours.empty())
            {
                collector.Add(ValidationFinding::EmptyGeometry, 0, 0, {});
                return GeometryResult<ValidationReport>::Empty(std::move(report));
            }

            for (std::size_t i = 0; i < path.contours.size(); ++i)
            {
                if (collector.ShouldStopRequestedWork())
                    break;

                if (context.ShouldCheckCancellation(i) && context.IsCancelled())
                    return GeometryResult<ValidationReport>::Failure(GeometryStatus::Cancelled);

                ValidateContour(path.contours[i], static_cast<std::uint32_t>(i), context, options,
                                report, collector);
            }

            if (options.checkContourIntersections && path.contours.size() > 1 &&
                !collector.ShouldStopRequestedWork())
            {
                // One hierarchy per contour, built once, plus a bounds prefilter.
                //
                // This used to call ContourIntersections for every pair, and that
                // function builds two hierarchies of its own - so C contours cost
                // C*(C-1) builds where C is enough. At 1000 contours that is a million
                // builds for geometry that needs a thousand, and the pairs that overlap
                // at all are a small fraction of the 500,000 considered.
                const std::size_t count = path.contours.size();

                std::vector<std::vector<Segment>> segments(count);
                std::vector<SegmentBVH> hierarchies(count);
                std::vector<Bounds2> bounds(count);

                for (std::size_t i = 0; i < count; ++i)
                {
                    const Contour& contour = path.contours[i];

                    // Section 15A. ValidateContour above has ALREADY reported
                    // InconsistentStorage for this contour and returned early rather than
                    // index it - and then this second pass indexed it anyway. Validation
                    // is the one operation whose whole job is to survive malformed input
                    // and describe it, so reading out of bounds here is the worst place
                    // in the kernel for it to happen. Leaving `segments[i]` empty makes
                    // the pair loop below skip this contour, which is the honest answer:
                    // a contour that cannot be indexed has no segments to intersect.
                    if (!contour.IsStructurallyValid())
                        continue;

                    const std::size_t segmentCount = contour.SegmentCount();
                    segments[i].reserve(segmentCount);
                    for (std::size_t k = 0; k < segmentCount; ++k)
                        segments[i].push_back(contour.SegmentAt(k));

                    if (!segments[i].empty())
                    {
                        hierarchies[i].BuildFromSegments(segments[i].data(), segments[i].size(),
                                                         context);
                    }
                    // F35a, class sweep. This box PRUNES the pair loop below, and a pair
                    // it drops is a ContourIntersection finding that never gets reported.
                    // For the one operation whose output a Doctor will treat as the list
                    // of what is wrong with the artwork, a missed defect is the worst
                    // direction to be short in - so the box has to contain the contour.
                    bounds[i] = Metrics::ComputeEnclosure(contour).Box();
                }

                const double tolerance = context.Tolerance().intersection;

                for (std::size_t i = 0;
                     i < count && !collector.ShouldStopRequestedWork(); ++i)
                {
                    if (segments[i].empty())
                        continue;

                    for (std::size_t j = i + 1;
                         j < count && !collector.ShouldStopRequestedWork(); ++j)
                    {
                        if (segments[j].empty())
                            continue;

                        // Two contours whose boxes do not meet cannot intersect, and the
                        // box test is two comparisons against a traversal.
                        if (!bounds[i].Overlaps(bounds[j], tolerance))
                            continue;

                        const auto crossings = Intersect::SegmentsIntersections(
                            segments[i].data(), segments[i].size(), hierarchies[i],
                            segments[j].data(), segments[j].size(), hierarchies[j], context);

                        if (!crossings.Ok() && crossings.Status() != GeometryStatus::Empty)
                        {
                            report.completeness = crossings.Status() == GeometryStatus::Cancelled
                                ? ValidationCompleteness::Cancelled
                                : ValidationCompleteness::Partial;
                            continue;
                        }
                        if (crossings.Status() == GeometryStatus::Empty)
                            continue;

                        for (const SelfIntersectionFinding& finding : crossings.Value().findings)
                        {
                            if (collector.ShouldStopRequestedWork())
                                break;

                            collector.Add(ValidationFinding::ContourIntersection,
                                          static_cast<std::uint32_t>(i),
                                          finding.segmentA, finding.point.point,
                                          finding.point.kind);
                        }
                    }
                }
            }

            if (options.checkTopology && !collector.ShouldStopRequestedWork())
            {
                const auto tree = Topology::BuildContainmentTree(path, context);
                if (!tree.Ok())
                {
                    report.completeness = tree.Status() == GeometryStatus::Cancelled
                        ? ValidationCompleteness::Cancelled
                        : ValidationCompleteness::Partial;
                }

                if (tree.Ok() && tree.Value().hasCoincidentContours)
                    collector.Add(ValidationFinding::CoincidentContours, 0, 0, {});

                // EvenOdd only, and the restriction is not a convenience.
                //
                // Under EvenOdd the role comes from containment alone, so the authored
                // direction is independent information and disagreeing with the role is
                // a genuine finding. Under NonZero the role is DERIVED from that same
                // direction, so the comparison degenerates: at every ring below the top
                // it agrees by construction, and at the top it reduces to "was the
                // outermost ring drawn counter-clockwise?", which is a drawing
                // convention and not a defect. Left ungated, a wholly clockwise NonZero
                // region - a perfectly good solid - was reported broken at every ring,
                // and a repair pass acting on that would repair it into other geometry.
                if (tree.Ok() && tree.Value().fillRule == FillRule::EvenOdd)
                {
                    // Winding is only "inconsistent" relative to the role containment
                    // assigned; it is a fact to report, not an error to act on here.
                    for (const ContainmentNode& node : tree.Value().nodes)
                    {
                        if (collector.ShouldStopRequestedWork())
                            break;

                        if (node.contour >= path.contours.size())
                            continue;

                        const Orientation actual =
                            Metrics::OrientationOf(path.contours[node.contour], context.Tolerance());
                        if (actual == Orientation::Degenerate)
                            continue;

                        const Orientation expected = node.role == ContourRole::Outer
                            ? Orientation::CounterClockwise
                            : Orientation::Clockwise;

                        if (actual != expected)
                        {
                            collector.Add(ValidationFinding::WindingInconsistency, node.contour, 0, {});
                        }
                    }
                }
            }

            if (report.truncated && report.completeness == ValidationCompleteness::Complete)
                report.completeness = ValidationCompleteness::Partial;

            return GeometryResult<ValidationReport>::Success(std::move(report));
        }

        GeometryResult<ValidationReport> Validate(const Contour& contour, const GeometryContext& context,
                                                  const ValidationOptions& options)
        {
            Path path;
            path.contours.push_back(contour);
            return Validate(path, context, options);
        }
    }
}
