#include "NfpHoleFilter.hpp"

#include "PolygonMetrics.hpp"
#include "ConvexHull.hpp"
#include "../Topology/ContainmentTree.hpp"
#include "../Validation/GeometryValidation.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace ImCut::Geometry::Nfp
{
    namespace
    {
        [[nodiscard]] bool FatalForProof(ValidationFinding finding) noexcept
        {
            // WindingInconsistency under EvenOdd is an authoring convention; roles are
            // derived from containment and remain unambiguous. Small/duplicate segments
            // are likewise conservative for the bounds-only proof. Intersections,
            // coincident contours and invalid region structure are not.
            switch (finding)
            {
                case ValidationFinding::NonFiniteCoordinate:
                case ValidationFinding::InconsistentStorage:
                case ValidationFinding::EmptyGeometry:
                case ValidationFinding::OpenContour:
                case ValidationFinding::InsufficientNodes:
                case ValidationFinding::ZeroArea:
                case ValidationFinding::SelfIntersection:
                case ValidationFinding::ContourIntersection:
                case ValidationFinding::CoincidentContours:
                case ValidationFinding::HoleOutsideOuter:
                case ValidationFinding::HoleTouchingOuter:
                case ValidationFinding::QuantizationCollapse:
                case ValidationFinding::CoincidentEdge:
                case ValidationFinding::UnsupportedTopology:
                    return true;
                case ValidationFinding::ZeroLengthSegment:
                case ValidationFinding::TinySegment:
                case ValidationFinding::DuplicateNode:
                case ValidationFinding::DegenerateCurve:
                case ValidationFinding::WindingInconsistency:
                case ValidationFinding::DuplicateEdge:
                case ValidationFinding::TinyFeature:
                    return false;
            }
            return true;
        }

        [[nodiscard]] OperandHoleFilterAudit FilterOne(
            const Path& source,
            const PreparedHoleFilterAnalysis& sourceAnalysis,
            const Transform2& sourceTransform,
            const PreparedHoleFilterAnalysis& otherAnalysis,
            const Transform2& otherTransform,
            const GeometryContext& context)
        {
            OperandHoleFilterAudit audit;
            audit.filtered = source;
            audit.sourceTopologyAccepted = sourceAnalysis.topologyAccepted;
            audit.otherOperandTopologyAccepted = otherAnalysis.topologyAccepted;
            audit.otherOperandConnected = otherAnalysis.connected;
            audit.otherOperandBounds = Metrics::ComputeWitness(
                otherAnalysis.witnessPoints.data(), otherAnalysis.witnessPoints.size(),
                otherTransform);

            const std::size_t count = sourceAnalysis.tree.nodes.size();
            if (!sourceAnalysis.topologyAccepted || !otherAnalysis.connected || count == 0)
            {
                for (const ContainmentNode& node : sourceAnalysis.tree.nodes)
                {
                    if (node.role == ContourRole::Hole)
                    {
                        HoleFilterDecision decision;
                        decision.contourIndex = node.contour;
                        decision.holeBounds = sourceTransform.ApplyEnclosure(node.bounds);
                        audit.holes.push_back(decision);
                        ++audit.preservedHoles;
                    }
                }
                return audit;
            }

            std::vector<std::uint32_t> order(count);
            std::iota(order.begin(), order.end(), 0u);
            std::sort(order.begin(), order.end(), [&](std::uint32_t a, std::uint32_t b)
            {
                const ContainmentNode& left = sourceAnalysis.tree.nodes[a];
                const ContainmentNode& right = sourceAnalysis.tree.nodes[b];
                return left.depth != right.depth ? left.depth < right.depth
                                                 : left.contour < right.contour;
            });

            std::vector<bool> removedNodes(count, false);
            std::vector<bool> removedContours(source.contours.size(), false);
            const BoundsWitness& otherWitness = audit.otherOperandBounds;
            const double otherWidth = otherWitness.Width();
            const double otherHeight = otherWitness.Height();

            for (const std::uint32_t index : order)
            {
                const ContainmentNode& node = sourceAnalysis.tree.nodes[index];
                const bool ancestorRemoved =
                    node.parent != ContainmentNode::kNoParent &&
                    removedNodes[node.parent];
                if (ancestorRemoved)
                {
                    removedNodes[index] = true;
                    if (node.contour < removedContours.size())
                        removedContours[node.contour] = true;
                }

                if (node.role != ContourRole::Hole)
                    continue;

                HoleFilterDecision decision;
                decision.contourIndex = node.contour;
                decision.holeBounds = sourceTransform.ApplyEnclosure(node.bounds);

                if (ancestorRemoved)
                {
                    decision.proof = HoleFilterProof::RemovedWithProvenAncestor;
                    decision.removed = true;
                    ++audit.removedHoles;
                    audit.holes.push_back(decision);
                    continue;
                }

                const double scale = std::max({
                    std::fabs(otherWidth), std::fabs(otherHeight),
                    std::fabs(decision.holeBounds.Box().Width()),
                    std::fabs(decision.holeBounds.Box().Height()), 1.0
                });
                const double margin = context.Tolerance().ScaledEpsilon(scale);

                // The two flags are the audit's decomposition of the single predicate
                // below - they exist so a report can say WHICH axis proved it. That they
                // agree with the predicate is checked by the suite, not assumed here.
                decision.widthProvesNonFit =
                    otherWidth > decision.holeBounds.Box().Width() + margin;
                decision.heightProvesNonFit =
                    otherHeight > decision.holeBounds.Box().Height() + margin;

                // Lemma B4.2: witness on the contained side, enclosure on the container
                // side. Both are needed, and F26 was reading an enclosure as a witness.
                if (ProvesCannotFitByTranslation(otherWitness, decision.holeBounds, margin))
                {
                    decision.proof = HoleFilterProof::BoundingBoxNonFit;
                    decision.removed = true;
                    removedNodes[index] = true;
                    if (node.contour < removedContours.size())
                        removedContours[node.contour] = true;
                    ++audit.removedHoles;
                }
                else
                {
                    ++audit.preservedHoles;
                }
                audit.holes.push_back(decision);
            }

            std::vector<Contour> kept;
            kept.reserve(source.contours.size());
            for (std::size_t i = 0; i < source.contours.size(); ++i)
            {
                if (removedContours[i])
                {
                    ++audit.removedContours;
                    continue;
                }
                kept.push_back(source.contours[i]);
            }
            audit.filtered.contours = std::move(kept);
            return audit;
        }

        [[nodiscard]] OperandHoleFilterPlan PlanOne(
            const PreparedHoleFilterAnalysis& source,
            const Transform2& sourceTransform,
            const PreparedHoleFilterAnalysis& other,
            const Transform2& otherTransform,
            const GeometryContext& context) noexcept
        {
            OperandHoleFilterPlan plan;
            if (!source.topologyAccepted || !other.connected ||
                source.tree.nodes.empty())
            {
                return plan;
            }

            const BoundsWitness otherWitness = Metrics::ComputeWitness(
                other.witnessPoints.data(), other.witnessPoints.size(), otherTransform);
            const double otherWidth = otherWitness.Width();
            const double otherHeight = otherWitness.Height();
            std::size_t remainingContours = 0;
            std::uint32_t remainingContour = 0;
            for (std::size_t index = 0; index < source.tree.nodes.size(); ++index)
            {
                const ContainmentNode& sourceNode = source.tree.nodes[index];
                bool removed = false;
                std::uint32_t current = static_cast<std::uint32_t>(index);
                while (current != ContainmentNode::kNoParent)
                {
                    const ContainmentNode& node = source.tree.nodes[current];
                    if (node.role == ContourRole::Hole)
                    {
                        const BoundsEnclosure holeBounds =
                            sourceTransform.ApplyEnclosure(node.bounds);
                        const double scale = std::max({
                            std::fabs(otherWidth), std::fabs(otherHeight),
                            std::fabs(holeBounds.Box().Width()),
                            std::fabs(holeBounds.Box().Height()), 1.0
                        });
                        const double margin =
                            context.Tolerance().ScaledEpsilon(scale);
                        // The same lemma B4.2 as FilterOne. PLAN and FILTER answering the
                        // same question two different ways was how F26 lived in two
                        // functions at once; now there is one predicate.
                        if (ProvesCannotFitByTranslation(otherWitness, holeBounds, margin))
                        {
                            removed = true;
                            break;
                        }
                    }
                    current = node.parent;
                }

                if (!removed)
                {
                    ++remainingContours;
                    remainingContour = sourceNode.contour;
                    continue;
                }
                ++plan.removedContours;
                if (sourceNode.role == ContourRole::Hole)
                    ++plan.removedHoles;
            }
            plan.strictlyConvexAfterFilter = remainingContours == 1 &&
                remainingContour < source.exactStrictlyConvexContours.size() &&
                source.exactStrictlyConvexContours[remainingContour] != 0;
            return plan;
        }
    }

    GeometryResult<PreparedHoleFilterAnalysis> PrepareHoleFilterAnalysis(
        const Path& path, const GeometryContext& context)
    {
        if (!path.IsStructurallyValid())
        {
            return GeometryResult<PreparedHoleFilterAnalysis>::Failure(
                GeometryStatus::InvalidInput);
        }
        if (path.HasOpenContours())
        {
            return GeometryResult<PreparedHoleFilterAnalysis>::Failure(
                GeometryStatus::InvalidTopology);
        }
        if (path.contours.empty())
        {
            return GeometryResult<PreparedHoleFilterAnalysis>::Empty(
                PreparedHoleFilterAnalysis{});
        }

        auto validation = Validation::Validate(path, context);
        if (!validation.Ok())
        {
            return GeometryResult<PreparedHoleFilterAnalysis>::Failure(
                validation.Status());
        }
        auto tree = Topology::BuildContainmentTree(path, context);
        if (!tree.Ok())
        {
            return GeometryResult<PreparedHoleFilterAnalysis>::Failure(tree.Status());
        }

        PreparedHoleFilterAnalysis analysis;
        analysis.tree = std::move(tree).Value();
        analysis.bounds = Metrics::ComputeBounds(path);

        // Points ON the shape, hulled once. The hull is what makes the per-pair witness
        // cost the hull size instead of the segment count, and it loses nothing: a hull
        // has the same support function as the points it spans.
        {
            std::vector<Vec2> witnessSamples;
            Metrics::CollectWitnessPoints(path, witnessSamples);
            Hull::ComputeInto(witnessSamples.data(), witnessSamples.size(),
                              analysis.witnessPoints);
            // A hull needs three points; below that (a segment, a point) the samples ARE
            // the support set and there is nothing to reduce.
            if (analysis.witnessPoints.empty())
                analysis.witnessPoints = std::move(witnessSamples);
        }
        analysis.exactStrictlyConvexContours.resize(path.contours.size(), 0);
        for (std::size_t contourIndex = 0;
             contourIndex < path.contours.size(); ++contourIndex)
        {
            const Contour& contour = path.contours[contourIndex];
            bool polygonal = contour.closed && contour.nodes.size() >= 3;
            for (const SegmentKind kind : contour.kinds)
                polygonal = polygonal && kind == SegmentKind::Line;
            analysis.exactStrictlyConvexContours[contourIndex] =
                polygonal && Metrics::IsStrictlyConvex(contour.nodes) ? 1u : 0u;
        }
        analysis.topologyAccepted =
            validation.Value().completeness == ValidationCompleteness::Complete &&
            !analysis.tree.hasCoincidentContours;
        for (const ValidationIssue& issue : validation.Value().issues)
        {
            if (FatalForProof(issue.finding))
            {
                analysis.topologyAccepted = false;
                break;
            }
        }

        std::size_t filledComponents = 0;
        for (const ContainmentNode& node : analysis.tree.nodes)
        {
            if (node.role != ContourRole::Outer)
                continue;
            if (node.parent == ContainmentNode::kNoParent ||
                analysis.tree.nodes[node.parent].role == ContourRole::Hole)
            {
                ++filledComponents;
            }
        }
        analysis.connected = analysis.topologyAccepted && filledComponents == 1;
        return GeometryResult<PreparedHoleFilterAnalysis>::Success(
            std::move(analysis));
    }

    ProvenHoleFilterPlan PlanProvenIrrelevantHoles(
        const PreparedHoleFilterAnalysis& stationary,
        const Transform2& stationaryTransform,
        const PreparedHoleFilterAnalysis& moving,
        const Transform2& movingTransform,
        const GeometryContext& context) noexcept
    {
        ProvenHoleFilterPlan plan;
        plan.stationary = PlanOne(
            stationary, stationaryTransform, moving, movingTransform, context);
        plan.moving = PlanOne(
            moving, movingTransform, stationary, stationaryTransform, context);
        return plan;
    }

    GeometryResult<ProvenHoleFilterAudit> FilterProvenIrrelevantHoles(
        const Path& stationary,
        const Path& moving,
        const GeometryContext& context)
    {
        auto stationaryAnalysis = PrepareHoleFilterAnalysis(stationary, context);
        if (!stationaryAnalysis.Ok())
            return GeometryResult<ProvenHoleFilterAudit>::Failure(
                stationaryAnalysis.Status());
        auto movingAnalysis = PrepareHoleFilterAnalysis(moving, context);
        if (!movingAnalysis.Ok())
            return GeometryResult<ProvenHoleFilterAudit>::Failure(movingAnalysis.Status());

        return FilterProvenIrrelevantHoles(
            stationary, stationaryAnalysis.Value(), moving,
            movingAnalysis.Value(), context);
    }

    GeometryResult<ProvenHoleFilterAudit> FilterProvenIrrelevantHoles(
        const Path& stationary,
        const PreparedHoleFilterAnalysis& stationaryAnalysis,
        const Path& moving,
        const PreparedHoleFilterAnalysis& movingAnalysis,
        const GeometryContext& context)
    {
        return FilterProvenIrrelevantHoles(
            stationary, stationaryAnalysis, Transform2::Identity(),
            moving, movingAnalysis, Transform2::Identity(), context);
    }

    GeometryResult<ProvenHoleFilterAudit> FilterProvenIrrelevantHoles(
        const Path& stationary,
        const PreparedHoleFilterAnalysis& stationaryAnalysis,
        const Transform2& stationaryTransform,
        const Path& moving,
        const PreparedHoleFilterAnalysis& movingAnalysis,
        const Transform2& movingTransform,
        const GeometryContext& context)
    {
        ProvenHoleFilterAudit audit;
        audit.stationary = FilterOne(
            stationary, stationaryAnalysis, stationaryTransform,
            movingAnalysis, movingTransform, context);
        audit.moving = FilterOne(
            moving, movingAnalysis, movingTransform,
            stationaryAnalysis, stationaryTransform, context);
        CountStat(context, &GeometryStatistics::nfpHoleFilterAudits);
        CountStat(context, &GeometryStatistics::nfpHolesFiltered,
                  audit.RemovedHoles());
        return GeometryResult<ProvenHoleFilterAudit>::Success(std::move(audit));
    }
}
