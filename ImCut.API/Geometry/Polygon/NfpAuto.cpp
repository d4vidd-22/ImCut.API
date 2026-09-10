#include "NfpAuto.hpp"

#include "NfpHoleFilter.hpp"
#include "NfpPrepared.hpp"
#include "PolygonMetrics.hpp"
#include "../Curves/Flatten.hpp"
#include "../Math/Predicates.hpp"
#include "../Topology/ContainmentTree.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace ImCut::Geometry::Nfp
{
    namespace
    {
        [[nodiscard]] GeometryResult<NfpOperandFeatures> AnalyzeFeatures(
            const Path& path, const GeometryContext& context,
            const PreparedHoleFilterAnalysis* preparedHoleAnalysis = nullptr)
        {
            if (!path.IsStructurallyValid())
                return GeometryResult<NfpOperandFeatures>::Failure(
                    GeometryStatus::InvalidInput);
            if (path.HasOpenContours())
                return GeometryResult<NfpOperandFeatures>::Failure(
                    GeometryStatus::InvalidTopology);
            if (path.contours.empty())
                return GeometryResult<NfpOperandFeatures>::Empty(NfpOperandFeatures{});
            if (context.IsCancelled())
                return GeometryResult<NfpOperandFeatures>::Failure(
                    GeometryStatus::Cancelled);

            // Common NFP operand: one closed polygonal ring. Reading it directly
            // avoids building a containment tree and flatten buffers only to discover
            // that there are no holes. This deliberately mirrors RegionIsConvex's
            // polygonal test and leaves curves/compound regions on the general path.
            if (path.contours.size() == 1)
            {
                const Contour& contour = path.contours.front();
                bool polygonal = contour.closed && contour.nodes.size() >= 3;
                for (const SegmentKind kind : contour.kinds)
                    polygonal = polygonal && kind == SegmentKind::Line;
                if (polygonal)
                {
                    NfpOperandFeatures features;
                    const std::vector<Vec2>& ring = contour.nodes;
                    features.vertices = ring.size();
                    features.bounds = Metrics::ComputeBounds(contour);
                    const double orientation =
                        Metrics::SignedArea(ring) < 0.0 ? -1.0 : 1.0;
                    for (std::size_t i = 0; i < ring.size(); ++i)
                    {
                        const double turn = Predicates::Orient2D(
                            ring[(i + ring.size() - 1) % ring.size()], ring[i],
                            ring[(i + 1) % ring.size()]);
                        if (turn * orientation < 0.0)
                            ++features.reflex;
                    }
                    features.strictlyConvex = Metrics::IsStrictlyConvex(ring);
                    return GeometryResult<NfpOperandFeatures>::Success(
                        std::move(features));
                }
            }

            GeometryResult<ContainmentTree> builtTree;
            const ContainmentTree* tree = nullptr;
            if (preparedHoleAnalysis != nullptr)
            {
                tree = &preparedHoleAnalysis->tree;
            }
            else
            {
                builtTree = Topology::BuildContainmentTree(path, context);
                if (!builtTree.Ok())
                {
                    return GeometryResult<NfpOperandFeatures>::Failure(
                        builtTree.Status());
                }
                tree = &builtTree.Value();
            }

            NfpOperandFeatures features;
            features.bounds = Metrics::ComputeBounds(path);
            for (std::size_t ringIndex = 0;
                 ringIndex < tree->rings.size(); ++ringIndex)
            {
                const std::vector<Vec2>& ring = tree->rings[ringIndex].points;
                features.vertices += ring.size();
                if (ring.size() < 3)
                    continue;

                const double orientation = Metrics::SignedArea(ring) < 0.0 ? -1.0 : 1.0;
                for (std::size_t i = 0; i < ring.size(); ++i)
                {
                    const double turn = Predicates::Orient2D(
                        ring[(i + ring.size() - 1) % ring.size()], ring[i],
                        ring[(i + 1) % ring.size()]);
                    if (turn * orientation < 0.0)
                        ++features.reflex;
                }
            }
            for (const ContainmentNode& node : tree->nodes)
                if (node.role == ContourRole::Hole) ++features.holes;

            // Curves may flatten to a convex ring, but that is a bounded
            // approximation, not the exact polygonal edge-merge precondition. The
            // exact single-ring case returned above; every general-path case remains
            // on certified decomposition.
            features.strictlyConvex = false;
            return GeometryResult<NfpOperandFeatures>::Success(std::move(features));
        }

        void PublishFeatures(const NfpOperandFeatures& a, const NfpOperandFeatures& b,
                             NfpCostFeatures& out) noexcept
        {
            out.verticesA = a.vertices;
            out.verticesB = b.vertices;
            out.reflexA = a.reflex;
            out.reflexB = b.reflex;
            out.reflexRatioA = a.vertices == 0 ? 0.0
                : static_cast<double>(a.reflex) / static_cast<double>(a.vertices);
            out.reflexRatioB = b.vertices == 0 ? 0.0
                : static_cast<double>(b.reflex) / static_cast<double>(b.vertices);
            out.holesA = a.holes;
            out.holesB = b.holes;
            out.boundaryEdgeCount = a.vertices + b.vertices;
            out.boundsA = a.bounds;
            out.boundsB = b.bounds;
            out.strictlyConvexA = a.strictlyConvex;
            out.strictlyConvexB = b.strictlyConvex;
        }

        [[nodiscard]] bool CertifiedDecompositionStrategy(
            DecompositionMergeStrategy strategy) noexcept
        {
            return strategy == DecompositionMergeStrategy::HertelMehlhorn ||
                   strategy == DecompositionMergeStrategy::LongestSharedEdgeFirst;
        }

        void ComposeInputProvenance(
            GeometryResult<NfpResult>& result,
            const ErrorBudget& stationary,
            const ErrorBudget& moving)
        {
            if (!result.Ok())
                return;
            result.Value().budget.MergeSequential(stationary);
            result.Value().budget.MergeSequential(moving);
            result.Value().region.budget = result.Value().budget;
            result.Budget() = result.Value().budget;
        }
    }

    GeometryResult<NfpOperandFeatures> AnalyzeOperandFeatures(
        const Path& path,
        const GeometryContext& context)
    {
        return AnalyzeFeatures(path, context);
    }

    GeometryResult<NfpExecutionPlan> ResolveExecutionPlan(
        const NfpOperandFeatures& stationary,
        const NfpOperandFeatures& moving,
        NfpAlgorithm requested,
        DecompositionMergeStrategy forcedStrategy)
    {
        if (requested > NfpAlgorithm::ConvexDecomposition ||
            !CertifiedDecompositionStrategy(forcedStrategy))
        {
            return GeometryResult<NfpExecutionPlan>::Failure(
                GeometryStatus::InvalidInput);
        }
        if (requested == NfpAlgorithm::ReducedConvolution)
        {
            return GeometryResult<NfpExecutionPlan>::Failure(
                GeometryStatus::Unsupported);
        }

        NfpExecutionPlan plan;
        if (requested == NfpAlgorithm::ConvexEdgeMerge)
        {
            if (!stationary.strictlyConvex || !moving.strictlyConvex)
            {
                return GeometryResult<NfpExecutionPlan>::Failure(
                    GeometryStatus::Unsupported);
            }
            plan.algorithm = NfpAlgorithm::ConvexEdgeMerge;
            plan.reason = NfpSelectionReason::Forced;
            plan.decompositionStrategy = forcedStrategy;
        }
        else if (requested == NfpAlgorithm::ConvexDecomposition)
        {
            plan.algorithm = NfpAlgorithm::ConvexDecomposition;
            plan.reason = NfpSelectionReason::Forced;
            plan.decompositionStrategy = forcedStrategy;
        }
        else if (stationary.strictlyConvex && moving.strictlyConvex)
        {
            plan.algorithm = NfpAlgorithm::ConvexEdgeMerge;
            plan.reason = NfpSelectionReason::StrictlyConvexPair;
            plan.decompositionStrategy = DecompositionMergeStrategy::HertelMehlhorn;
        }
        else
        {
            plan = ResolveCertifiedAutoExecutionPlan(stationary, moving);
        }
        return GeometryResult<NfpExecutionPlan>::Success(std::move(plan));
    }

    GeometryResult<NfpResult> ExecutePreparedPlan(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& resolvedOptions,
        const NfpExecutionPlan& plan)
    {
        if (plan.policyVersion != NfpExecutionPlan::kCurrentPolicyVersion ||
            plan.algorithm == NfpAlgorithm::Auto ||
            plan.algorithm == NfpAlgorithm::ReducedConvolution ||
            plan.decompositionStrategy != resolvedOptions.decompositionMergeStrategy)
        {
            return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
        }
        if (plan.algorithm == NfpAlgorithm::ConvexEdgeMerge &&
            (!stationary.StrictlyConvex() || !reflectedMoving.StrictlyConvex()))
        {
            return GeometryResult<NfpResult>::Failure(GeometryStatus::Unsupported);
        }
        return ComputePrepared(
            stationary, reflectedMoving, context, resolvedOptions);
    }

    GeometryResult<AdaptiveNfpResult> ComputeAdaptive(
        const Path& stationary,
        const Path& moving,
        double rotationRadians,
        const GeometryContext& context,
        const AdaptiveNfpOptions& options)
    {
        if (!std::isfinite(rotationRadians) ||
            options.algorithm > NfpAlgorithm::ConvexDecomposition ||
            !CertifiedDecompositionStrategy(options.forcedDecompositionStrategy))
        {
            return GeometryResult<AdaptiveNfpResult>::Failure(
                GeometryStatus::InvalidInput);
        }
        const Transform2 movingTransform = rotationRadians == 0.0
            ? Transform2::Identity() : Transform2::Rotation(rotationRadians);

        std::optional<PreparedHoleFilterAnalysis> holeAnalysisA;
        std::optional<PreparedHoleFilterAnalysis> holeAnalysisB;
        if (options.allowProvenHoleFilter &&
            (stationary.contours.size() > 1 || moving.contours.size() > 1))
        {
            auto preparedA = PrepareHoleFilterAnalysis(stationary, context);
            if (!preparedA.Ok())
            {
                return GeometryResult<AdaptiveNfpResult>::Failure(
                    preparedA.Status());
            }
            auto preparedB = PrepareHoleFilterAnalysis(moving, context);
            if (!preparedB.Ok())
            {
                return GeometryResult<AdaptiveNfpResult>::Failure(
                    preparedB.Status());
            }
            holeAnalysisA.emplace(std::move(preparedA).Value());
            holeAnalysisB.emplace(std::move(preparedB).Value());
        }

        auto featuresA = AnalyzeFeatures(
            stationary, context,
            holeAnalysisA.has_value() ? &*holeAnalysisA : nullptr);
        if (!featuresA.Ok())
            return GeometryResult<AdaptiveNfpResult>::Failure(featuresA.Status());
        auto featuresB = AnalyzeFeatures(
            moving, context,
            holeAnalysisB.has_value() ? &*holeAnalysisB : nullptr);
        if (!featuresB.Ok())
            return GeometryResult<AdaptiveNfpResult>::Failure(featuresB.Status());
        // Feature descriptor, used to choose an execution plan - never as proof of a
        // set relation. An enclosure is the right evidence and `.Box()` says that this
        // caller wants the rectangle. The hole filter does NOT read this: it gets the
        // untransformed analysis plus `movingTransform`, and builds its own witness.
        featuresB.Value().bounds =
            movingTransform.ApplyEnclosure(featuresB.Value().bounds).Box();

        AdaptiveNfpResult output;
        output.selection.requested = options.algorithm;
        PublishFeatures(featuresA.Value(), featuresB.Value(), output.features);
        auto selected = ResolveExecutionPlan(
            featuresA.Value(), featuresB.Value(), options.algorithm,
            options.forcedDecompositionStrategy);
        if (!selected.Ok())
            return GeometryResult<AdaptiveNfpResult>::Failure(selected.Status());
        NfpExecutionPlan plan = selected.Value();
        output.selection.selected = plan.algorithm;
        output.selection.reason = plan.reason;
        output.selection.decompositionStrategy = plan.decompositionStrategy;

        std::optional<Path> filteredA;
        std::optional<Path> filteredB;
        std::optional<NfpOperandFeatures> filteredFeaturesA;
        std::optional<NfpOperandFeatures> filteredFeaturesB;

        // Forced ConvexEdgeMerge is a strict contract and never changes its operands to
        // make them satisfy the precondition. Auto/decomposition may apply the frozen
        // pair-specific proof first.
        if (options.allowProvenHoleFilter &&
            output.selection.selected != NfpAlgorithm::ConvexEdgeMerge &&
            holeAnalysisA.has_value() && holeAnalysisB.has_value())
        {
            const ProvenHoleFilterPlan holePlan = PlanProvenIrrelevantHoles(
                *holeAnalysisA, Transform2::Identity(),
                *holeAnalysisB, movingTransform, context);
            if (holePlan.Applied())
            {
                output.selection.holeFilterAttempted = true;
                Path orientedMoving = rotationRadians == 0.0
                    ? moving : TransformPath(moving, movingTransform);
                auto filtered = FilterProvenIrrelevantHoles(
                    stationary, *holeAnalysisA, Transform2::Identity(),
                    orientedMoving, *holeAnalysisB, movingTransform, context);
                if (!filtered.Ok())
                {
                    return GeometryResult<AdaptiveNfpResult>::Failure(
                        filtered.Status());
                }
                output.selection.holesRemoved = filtered.Value().RemovedHoles();
                output.selection.holeFilterApplied =
                    output.selection.holesRemoved != 0;
                filteredA.emplace(
                    std::move(filtered.Value().stationary.filtered));
                filteredB.emplace(
                    std::move(filtered.Value().moving.filtered));
            }

            if (options.algorithm == NfpAlgorithm::Auto &&
                output.selection.holeFilterApplied)
            {
                auto analyzedA = AnalyzeFeatures(*filteredA, context);
                auto analyzedB = AnalyzeFeatures(*filteredB, context);
                if (!analyzedA.Ok() || !analyzedB.Ok())
                {
                    return GeometryResult<AdaptiveNfpResult>::Failure(
                        !analyzedA.Ok() ? analyzedA.Status() : analyzedB.Status());
                }
                filteredFeaturesA.emplace(std::move(analyzedA).Value());
                filteredFeaturesB.emplace(std::move(analyzedB).Value());
                if (filteredFeaturesA->strictlyConvex &&
                    filteredFeaturesB->strictlyConvex)
                {
                    output.selection.selected = NfpAlgorithm::ConvexEdgeMerge;
                    output.selection.reason =
                        NfpSelectionReason::ProvenHoleFilterMadePairConvex;
                    plan.algorithm = NfpAlgorithm::ConvexEdgeMerge;
                    plan.reason = NfpSelectionReason::ProvenHoleFilterMadePairConvex;
                }
            }
        }

        NfpOptions geometry = options.geometry;
        geometry.routingPolicy = NfpRoutingPolicy::Forced;
        geometry.decompositionMergeStrategy =
            output.selection.decompositionStrategy;
        geometry.allowProvenHoleFilter = false;

        constexpr std::uint64_t kTransientOwner =
            (std::numeric_limits<std::uint64_t>::max)();
        const NfpLatticeIdentity lattice{};
        const Path& preparedPathA = filteredA.has_value() ? *filteredA : stationary;
        const Path& preparedPathB = filteredB.has_value() ? *filteredB : moving;
        const double preparedRotationB = filteredB.has_value()
            ? 0.0 : rotationRadians;
        auto preparedA = PreparedNfpOperand::BuildTransient(
            preparedPathA, 1, kTransientOwner, 1, 0.0, false,
            lattice, context, geometry,
            filteredFeaturesA.has_value() ? &*filteredFeaturesA : &featuresA.Value());
        if (!preparedA.Ok())
            return GeometryResult<AdaptiveNfpResult>::Failure(preparedA.Status());
        auto preparedB = PreparedNfpOperand::BuildTransient(
            preparedPathB, 2, kTransientOwner, 2, preparedRotationB, true,
            lattice, context, geometry,
            filteredFeaturesB.has_value() ? &*filteredFeaturesB : &featuresB.Value());
        if (!preparedB.Ok())
            return GeometryResult<AdaptiveNfpResult>::Failure(preparedB.Status());

        auto computed = ExecutePreparedPlan(
            preparedA.Value(), preparedB.Value(), context, geometry, plan);
        if (!computed.Ok())
            return GeometryResult<AdaptiveNfpResult>::Failure(computed.Status());
        ComposeInputProvenance(computed, stationary.budget, moving.budget);

        const GeometryStatus computedStatus = computed.Status();
        output.result = std::move(computed).Value();
        output.features.convexPiecesA = output.result.piecesA;
        output.features.convexPiecesB = output.result.piecesB;
        if (output.result.piecesB != 0 &&
            output.result.piecesA <=
                static_cast<std::size_t>(-1) / output.result.piecesB)
        {
            output.features.piecePairCount =
                output.result.piecesA * output.result.piecesB;
        }

        auto result = computedStatus == GeometryStatus::Empty
            ? GeometryResult<AdaptiveNfpResult>::Empty(std::move(output))
            : GeometryResult<AdaptiveNfpResult>::Success(std::move(output));
        result.Budget().Merge(result.Value().result.budget);
        return result;
    }
}
