#include "NfpConvolution.hpp"

#include "../Intersection/Intersection.hpp"
#include "../Intersection/SelfIntersection.hpp"
#include "../Math/Predicates.hpp"
#include "PolygonMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ImCut::Geometry::Nfp
{
    namespace
    {
        [[nodiscard]] bool IsSimplePolygonalOperand(
            const PreparedNfpOperand& operand) noexcept
        {
            const Path& path = operand.OrientedPath();
            if (path.contours.size() != 1)
                return false;
            const Contour& contour = path.contours.front();
            if (!contour.closed || contour.nodes.size() < 3)
                return false;
            for (const SegmentKind kind : contour.kinds)
                if (kind != SegmentKind::Line) return false;
            return true;
        }

        [[nodiscard]] std::vector<Vec2> CounterClockwiseNodes(
            const PreparedNfpOperand& operand)
        {
            std::vector<Vec2> nodes = operand.OrientedPath().contours.front().nodes;
            if (Metrics::SignedArea(nodes.data(), nodes.size()) < 0.0)
                std::reverse(nodes.begin(), nodes.end());
            return nodes;
        }

        [[nodiscard]] double PositiveAngle(double value) noexcept
        {
            constexpr double twoPi = 2.0 * 3.14159265358979323846;
            while (value < 0.0) value += twoPi;
            while (value >= twoPi) value -= twoPi;
            return value;
        }

        [[nodiscard]] bool DirectionInTangentWedge(
            Vec2 direction, Vec2 incoming, Vec2 outgoing,
            double angularTolerance) noexcept
        {
            if (LengthSquared(direction) == 0.0 || LengthSquared(incoming) == 0.0 ||
                LengthSquared(outgoing) == 0.0)
            {
                return false;
            }
            const double start = std::atan2(incoming.y, incoming.x);
            const double end = std::atan2(outgoing.y, outgoing.x);
            const double angle = std::atan2(direction.y, direction.x);
            const double span = PositiveAngle(end - start);
            const double offset = PositiveAngle(angle - start);
            return offset <= span + angularTolerance;
        }

        [[nodiscard]] bool ConvexVertex(const std::vector<Vec2>& ring,
                                        std::size_t index) noexcept
        {
            const Vec2 previous = ring[(index + ring.size() - 1) % ring.size()];
            const Vec2 current = ring[index];
            const Vec2 next = ring[(index + 1) % ring.size()];
            return Predicates::Orient2D(previous, current, next) > 0.0;
        }

        [[nodiscard]] bool Compatible(const PreparedNfpOperand& stationary,
                                      const PreparedNfpOperand& reflectedMoving,
                                      const NfpOptions& options) noexcept
        {
            return stationary.OwnerSessionIdentity() != 0 &&
                   stationary.OwnerSessionIdentity() ==
                       reflectedMoving.OwnerSessionIdentity() &&
                   !stationary.Reflected() && reflectedMoving.Reflected() &&
                   stationary.Profile() == reflectedMoving.Profile() &&
                   stationary.Profile().decomposition.simplifyTolerance ==
                       options.simplifyTolerance &&
                   stationary.Profile().decomposition.maxPieces == options.maxPieces &&
                   stationary.Profile().decomposition.mergeStrategy ==
                       options.decompositionMergeStrategy &&
                   options.clearance == 0.0;
        }

        void AddMatches(const std::vector<Vec2>& edgeRing,
                        const std::vector<Vec2>& vertexRing,
                        ConvolutionFeatureSource source,
                        const GeometryContext& context,
                        std::vector<ConvolutionSegment>& output)
        {
            for (std::size_t edgeIndex = 0; edgeIndex < edgeRing.size(); ++edgeIndex)
            {
                const Vec2 edgeStart = edgeRing[edgeIndex];
                const Vec2 edgeEnd = edgeRing[(edgeIndex + 1) % edgeRing.size()];
                const Vec2 direction = edgeEnd - edgeStart;
                for (std::size_t vertexIndex = 0;
                     vertexIndex < vertexRing.size(); ++vertexIndex)
                {
                    if (!ConvexVertex(vertexRing, vertexIndex))
                        continue;
                    const Vec2 previous = vertexRing[
                        (vertexIndex + vertexRing.size() - 1) % vertexRing.size()];
                    const Vec2 vertex = vertexRing[vertexIndex];
                    const Vec2 next = vertexRing[(vertexIndex + 1) % vertexRing.size()];
                    if (!DirectionInTangentWedge(
                            direction, vertex - previous, next - vertex,
                            context.Tolerance().collinearity))
                    {
                        continue;
                    }

                    output.push_back({
                        edgeStart + vertex, edgeEnd + vertex,
                        static_cast<std::uint32_t>(edgeIndex),
                        static_cast<std::uint32_t>(vertexIndex), source });
                }
            }
        }

        [[nodiscard]] Bounds2 SegmentBounds(const ConvolutionSegment& segment) noexcept
        {
            return Bounds2::FromPoints(segment.start, segment.end);
        }

        [[nodiscard]] bool OracleEdgeCovered(
            Vec2 start, Vec2 end,
            const std::vector<ConvolutionSegment>& candidates,
            double tolerance) noexcept
        {
            const Vec2 samples[] = {
                Lerp(start, end, 0.25), Lerp(start, end, 0.5),
                Lerp(start, end, 0.75)
            };
            for (const Vec2 sample : samples)
            {
                bool covered = false;
                for (const ConvolutionSegment& candidate : candidates)
                {
                    if (Predicates::PointOnSegment(
                            sample, candidate.start, candidate.end, tolerance))
                    {
                        covered = true;
                        break;
                    }
                }
                if (!covered) return false;
            }
            return true;
        }
    }

    GeometryResult<ReducedConvolutionAudit> AuditReducedConvolution(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options)
    {
        ScopedGeometryTotal totalTimer(context.Diagnostics());
        CountStat(context, &GeometryStatistics::nfpConvolution);
        if (!Compatible(stationary, reflectedMoving, options))
            return GeometryResult<ReducedConvolutionAudit>::Failure(
                options.clearance == 0.0
                    ? GeometryStatus::InvalidInput : GeometryStatus::Unsupported);
        if (!IsSimplePolygonalOperand(stationary) ||
            !IsSimplePolygonalOperand(reflectedMoving))
        {
            return GeometryResult<ReducedConvolutionAudit>::Failure(
                GeometryStatus::Unsupported);
        }

        const auto selfA = Intersect::HasSelfIntersection(
            stationary.OrientedPath().contours.front(), context);
        const auto selfB = Intersect::HasSelfIntersection(
            reflectedMoving.OrientedPath().contours.front(), context);
        if (!selfA.Ok() || !selfB.Ok())
            return GeometryResult<ReducedConvolutionAudit>::Failure(
                !selfA.Ok() ? selfA.Status() : selfB.Status());
        if (selfA.Value() || selfB.Value())
            return GeometryResult<ReducedConvolutionAudit>::Failure(
                GeometryStatus::InvalidTopology);

        const std::vector<Vec2> a = CounterClockwiseNodes(stationary);
        const std::vector<Vec2> b = CounterClockwiseNodes(reflectedMoving);
        ReducedConvolutionAudit audit;
        audit.verticesA = a.size();
        audit.verticesB = b.size();
        for (std::size_t i = 0; i < a.size(); ++i)
            if (ConvexVertex(a, i)) ++audit.convexVerticesA;
        for (std::size_t i = 0; i < b.size(); ++i)
            if (ConvexVertex(b, i)) ++audit.convexVerticesB;
        audit.reflexVerticesA = a.size() - audit.convexVerticesA;
        audit.reflexVerticesB = b.size() - audit.convexVerticesB;

        if (b.size() != 0 && a.size() >
            (std::numeric_limits<std::size_t>::max)() / b.size())
        {
            return GeometryResult<ReducedConvolutionAudit>::Failure(
                GeometryStatus::ComplexityLimit);
        }
        const std::size_t product = a.size() * b.size();
        if (product > (std::numeric_limits<std::size_t>::max)() / 2)
            return GeometryResult<ReducedConvolutionAudit>::Failure(
                GeometryStatus::ComplexityLimit);
        audit.fullVertexEdgeSegments = product * 2;

        audit.segments.reserve((a.size() * audit.convexVerticesB) +
                               (b.size() * audit.convexVerticesA));
        AddMatches(a, b, ConvolutionFeatureSource::StationaryEdgeMovingVertex,
                   context, audit.segments);
        AddMatches(b, a, ConvolutionFeatureSource::MovingEdgeStationaryVertex,
                   context, audit.segments);
        audit.reducedSegmentCount = audit.segments.size();
        if (audit.reducedSegmentCount > context.Limits().maxConvolutionSegments)
            return GeometryResult<ReducedConvolutionAudit>::Failure(
                GeometryStatus::ComplexityLimit);

        std::vector<std::vector<double>> splitParameters(audit.segments.size());
        for (std::vector<double>& parameters : splitParameters)
            parameters = { 0.0, 1.0 };
        for (std::size_t i = 0; i < audit.segments.size(); ++i)
        {
            const Bounds2 boundsI = SegmentBounds(audit.segments[i]);
            for (std::size_t j = i + 1; j < audit.segments.size(); ++j)
            {
                if (++audit.arrangementPairTests >
                    context.Limits().maxConvolutionPairTests)
                {
                    return GeometryResult<ReducedConvolutionAudit>::Failure(
                        GeometryStatus::ComplexityLimit);
                }
                if (context.ShouldCheckCancellation(audit.arrangementPairTests) &&
                    context.IsCancelled())
                {
                    return GeometryResult<ReducedConvolutionAudit>::Failure(
                        GeometryStatus::Cancelled);
                }
                if (!boundsI.Overlaps(SegmentBounds(audit.segments[j])))
                    continue;
                const LineIntersection hit = Intersect::Lines(
                    audit.segments[i].start, audit.segments[i].end,
                    audit.segments[j].start, audit.segments[j].end,
                    context.Tolerance());
                if (!IsContact(hit.kind))
                    continue;
                ++audit.arrangementIntersections;
                if (audit.arrangementIntersections > context.Limits().maxIntersections)
                    return GeometryResult<ReducedConvolutionAudit>::Failure(
                        GeometryStatus::ComplexityLimit);
                if (hit.kind == IntersectionKind::Overlap)
                {
                    splitParameters[i].push_back(hit.overlapMinA);
                    splitParameters[i].push_back(hit.overlapMaxA);
                    splitParameters[j].push_back(Predicates::ProjectionParameter(
                        hit.overlapStart, audit.segments[j].start,
                        audit.segments[j].end));
                    splitParameters[j].push_back(Predicates::ProjectionParameter(
                        hit.overlapEnd, audit.segments[j].start,
                        audit.segments[j].end));
                }
                else
                {
                    splitParameters[i].push_back(hit.parameterA);
                    splitParameters[j].push_back(hit.parameterB);
                }
            }
        }
        for (std::vector<double>& parameters : splitParameters)
        {
            std::sort(parameters.begin(), parameters.end());
            const double epsilon = context.Tolerance().coordinateEpsilon;
            parameters.erase(std::unique(parameters.begin(), parameters.end(),
                [epsilon](double lhs, double rhs)
                {
                    return std::fabs(lhs - rhs) <= epsilon;
                }), parameters.end());
            if (parameters.size() > 1)
                audit.arrangementFragmentCount += parameters.size() - 1;
        }

        const auto cover = ComputeCoverForMaterialization(
            stationary, reflectedMoving, context, options);
        if (!cover.Ok())
            return GeometryResult<ReducedConvolutionAudit>::Failure(cover.Status());
        const auto oracle = MaterializePersistentLattice(
            cover.Value(), context, NfpLatticeGrouping::PerStationaryPiece);
        if (!oracle.Ok())
            return GeometryResult<ReducedConvolutionAudit>::Failure(oracle.Status());
        audit.budget = oracle.Value().budget;

        const double coverageTolerance = (std::max)(
            context.Tolerance().duplicate,
            audit.budget.Total() * 2.0 + context.Tolerance().coordinateEpsilon);
        for (const Contour& contour : oracle.Value().region.contours)
        {
            for (std::size_t i = 0; i < contour.nodes.size(); ++i)
            {
                const Vec2 start = contour.nodes[i];
                const Vec2 end = contour.nodes[(i + 1) % contour.nodes.size()];
                ++audit.oracleBoundaryEdges;
                if (OracleEdgeCovered(start, end, audit.segments, coverageTolerance))
                    ++audit.coveredOracleBoundaryEdges;
            }
        }

        auto success = GeometryResult<ReducedConvolutionAudit>::Success(
            std::move(audit));
        success.Budget().Merge(success.Value().budget);
        return success;
    }
}
