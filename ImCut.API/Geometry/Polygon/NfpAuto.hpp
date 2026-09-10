#pragma once

#include "Nfp.hpp"

#include "../Math/Bounds2.hpp"

#include <cstddef>
#include <cstdint>

namespace ImCut::Geometry::Nfp
{
    enum class NfpAlgorithm : std::uint8_t
    {
        Auto,
        ConvexEdgeMerge,
        ReducedConvolution,
        ConvexDecomposition
    };

    enum class NfpSelectionReason : std::uint8_t
    {
        Forced,
        StrictlyConvexPair,
        LowReflexPressure,
        ReflexPairPressure,
        ProvenHoleFilterMadePairConvex
    };

    struct NfpOperandFeatures
    {
        std::size_t vertices = 0;
        std::size_t reflex = 0;
        std::size_t holes = 0;
        Bounds2 bounds{};
        bool strictlyConvex = false;
    };

    struct NfpExecutionPlan
    {
        static constexpr std::uint32_t kCurrentPolicyVersion = 1;

        NfpAlgorithm algorithm = NfpAlgorithm::ConvexDecomposition;
        NfpSelectionReason reason = NfpSelectionReason::LowReflexPressure;
        DecompositionMergeStrategy decompositionStrategy =
            DecompositionMergeStrategy::HertelMehlhorn;
        std::uint32_t policyVersion = kCurrentPolicyVersion;
    };

    // Hot, infallible lane for callers that already fixed the request to the
    // certified Auto portfolio. Keeping this combine in the header lets a warm
    // session hit resolve its semantic cache identity without constructing a
    // GeometryResult or paying an out-of-line selector call.
    [[nodiscard]] inline NfpExecutionPlan ResolveCertifiedAutoExecutionPlan(
        const NfpOperandFeatures& stationary,
        const NfpOperandFeatures& moving) noexcept
    {
        NfpExecutionPlan plan;
        if (stationary.strictlyConvex && moving.strictlyConvex)
        {
            plan.algorithm = NfpAlgorithm::ConvexEdgeMerge;
            plan.reason = NfpSelectionReason::StrictlyConvexPair;
            plan.decompositionStrategy =
                DecompositionMergeStrategy::HertelMehlhorn;
            return plan;
        }

        plan.algorithm = NfpAlgorithm::ConvexDecomposition;
        const bool pairPressure = stationary.reflex >= 8 ||
            moving.reflex >= 8 - stationary.reflex;
        plan.decompositionStrategy = pairPressure
            ? DecompositionMergeStrategy::LongestSharedEdgeFirst
            : DecompositionMergeStrategy::HertelMehlhorn;
        plan.reason = pairPressure
            ? NfpSelectionReason::ReflexPairPressure
            : NfpSelectionReason::LowReflexPressure;
        return plan;
    }

    struct NfpCostFeatures
    {
        std::size_t verticesA = 0;
        std::size_t verticesB = 0;
        std::size_t reflexA = 0;
        std::size_t reflexB = 0;
        double reflexRatioA = 0.0;
        double reflexRatioB = 0.0;
        std::size_t holesA = 0;
        std::size_t holesB = 0;
        std::size_t boundaryEdgeCount = 0;
        Bounds2 boundsA{};
        Bounds2 boundsB{};
        bool strictlyConvexA = false;
        bool strictlyConvexB = false;

        // Filled after execution; the selector does not decompose merely to obtain
        // them. They remain operational diagnostics, not universal result semantics.
        std::size_t convexPiecesA = 0;
        std::size_t convexPiecesB = 0;
        std::size_t piecePairCount = 0;
    };

    struct AdaptiveNfpOptions
    {
        NfpOptions geometry{};
        NfpAlgorithm algorithm = NfpAlgorithm::Auto;

        // Used only when ConvexDecomposition is explicitly forced. Auto chooses
        // between the two frozen decomposition routes from measured features.
        DecompositionMergeStrategy forcedDecompositionStrategy =
            DecompositionMergeStrategy::HertelMehlhorn;

        bool allowProvenHoleFilter = true;
    };

    struct NfpSelectionDiagnostics
    {
        NfpAlgorithm requested = NfpAlgorithm::Auto;
        NfpAlgorithm selected = NfpAlgorithm::ConvexDecomposition;
        NfpSelectionReason reason = NfpSelectionReason::LowReflexPressure;
        DecompositionMergeStrategy decompositionStrategy =
            DecompositionMergeStrategy::HertelMehlhorn;
        bool holeFilterAttempted = false;
        bool holeFilterApplied = false;
        std::size_t holesRemoved = 0;
    };

    struct AdaptiveNfpResult
    {
        NfpResult result{};
        NfpCostFeatures features{};
        NfpSelectionDiagnostics selection{};
    };

    // Deterministic rule-based selector over the frozen V8 portfolio. Forced
    // ReducedConvolution returns Unsupported because that prototype has no certified
    // region extraction; Auto can therefore never select it.
    [[nodiscard]] GeometryResult<AdaptiveNfpResult> ComputeAdaptive(
        const Path& stationary,
        const Path& moving,
        double rotationRadians,
        const GeometryContext& context,
        const AdaptiveNfpOptions& options = {});

    [[nodiscard]] GeometryResult<NfpOperandFeatures> AnalyzeOperandFeatures(
        const Path& path,
        const GeometryContext& context);

    [[nodiscard]] GeometryResult<NfpExecutionPlan> ResolveExecutionPlan(
        const NfpOperandFeatures& stationary,
        const NfpOperandFeatures& moving,
        NfpAlgorithm requested,
        DecompositionMergeStrategy forcedStrategy =
            DecompositionMergeStrategy::HertelMehlhorn);

    class PreparedNfpOperand;
    [[nodiscard]] GeometryResult<NfpResult> ExecutePreparedPlan(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& resolvedOptions,
        const NfpExecutionPlan& plan);
}
