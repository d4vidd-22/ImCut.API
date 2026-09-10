#pragma once

#include "ConvexDecomposition.hpp"

#include "../GeometryCertification.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ImCut::Geometry::Decompose
{
    struct DecompositionQuality
    {
        DecompositionBackend backend = DecompositionBackend::None;
        DecompositionMergeStrategy appliedMergeStrategy =
            DecompositionMergeStrategy::None;
        std::size_t triangleCount = 0;
        std::size_t pieceCount = 0;
        std::size_t totalPieceVertices = 0;
        std::size_t maximumPieceVertices = 0;
        double meanPieceVertices = 0.0;
        double totalPiecePerimeter = 0.0;
        ErrorBudget budget{};
    };

    struct MinkowskiDecompositionCandidateAudit
    {
        DecompositionMergeStrategy requestedStrategy =
            DecompositionMergeStrategy::HertelMehlhorn;
        GeometryStatus statusA = GeometryStatus::Unsupported;
        GeometryStatus statusB = GeometryStatus::Unsupported;
        DecompositionQuality operandA{};
        DecompositionQuality operandB{};

        // Cheap downstream cost features. convexSumVertexWork is the exact sum of
        // |pieceA| + |pieceB| over the Cartesian product of pieces, matching the
        // linear edge merge performed by Minkowski::ConvexSum.
        std::size_t pairProduct = 0;
        std::size_t convexSumVertexWork = 0;
        bool costOverflow = false;

        [[nodiscard]] bool Complete() const noexcept
        {
            return IsSuccess(statusA) && IsSuccess(statusB);
        }
    };

    // Candidate-only audit artifact. It records every strategy instead of choosing
    // one, so WP8A cannot accidentally become the WP8C Auto selector before the
    // decomposition and hole-filter portfolios have been frozen.
    struct MinkowskiDecompositionPortfolioAudit
    {
        std::vector<MinkowskiDecompositionCandidateAudit> candidates{};

        [[nodiscard]] bool AutoSelectionPerformed() const noexcept { return false; }
        [[nodiscard]] bool ProductionEligible() const noexcept { return false; }
        [[nodiscard]] GeometryCertification Certification() const noexcept
        {
            return GeometryCertification::CandidateOnly;
        }
    };

    // Compare exact decomposition policies for two filled regions. The operands are
    // not modified, no candidate is selected, and wall-clock timing is deliberately
    // excluded from the deterministic artifact; end-to-end timing belongs to the
    // benchmark harness.
    [[nodiscard]] GeometryResult<MinkowskiDecompositionPortfolioAudit>
    AuditMinkowskiPortfolio(
        const Path& a,
        const Path& b,
        const GeometryContext& context,
        const DecompositionOptions& baseOptions = {});
}
