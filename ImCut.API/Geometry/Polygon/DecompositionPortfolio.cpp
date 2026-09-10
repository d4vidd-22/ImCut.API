#include "DecompositionPortfolio.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace ImCut::Geometry::Decompose
{
    namespace
    {
        [[nodiscard]] DecompositionQuality QualityOf(
            const ConvexDecompositionResult& decomposition) noexcept
        {
            DecompositionQuality quality;
            quality.backend = decomposition.backend;
            quality.appliedMergeStrategy = decomposition.appliedMergeStrategy;
            quality.triangleCount = decomposition.triangles.size();
            quality.pieceCount = decomposition.pieces.size();
            quality.budget = decomposition.budget;

            for (const std::vector<Vec2>& piece : decomposition.pieces)
            {
                quality.totalPieceVertices += piece.size();
                quality.maximumPieceVertices =
                    std::max(quality.maximumPieceVertices, piece.size());
                for (std::size_t i = 0; i < piece.size(); ++i)
                {
                    quality.totalPiecePerimeter += std::sqrt(DistanceSquared(
                        piece[i], piece[(i + 1) % piece.size()]));
                }
            }
            if (quality.pieceCount != 0)
            {
                quality.meanPieceVertices =
                    static_cast<double>(quality.totalPieceVertices) /
                    static_cast<double>(quality.pieceCount);
            }
            return quality;
        }

        [[nodiscard]] std::size_t SaturatingMultiply(std::size_t a, std::size_t b,
                                                     bool& overflow) noexcept
        {
            if (a != 0 && b > std::numeric_limits<std::size_t>::max() / a)
            {
                overflow = true;
                return std::numeric_limits<std::size_t>::max();
            }
            return a * b;
        }

        [[nodiscard]] std::size_t SaturatingAdd(std::size_t a, std::size_t b,
                                                bool& overflow) noexcept
        {
            if (b > std::numeric_limits<std::size_t>::max() - a)
            {
                overflow = true;
                return std::numeric_limits<std::size_t>::max();
            }
            return a + b;
        }

        void CompleteCosts(MinkowskiDecompositionCandidateAudit& audit) noexcept
        {
            if (!audit.Complete())
                return;

            audit.pairProduct = SaturatingMultiply(
                audit.operandA.pieceCount, audit.operandB.pieceCount,
                audit.costOverflow);
            const std::size_t aWork = SaturatingMultiply(
                audit.operandA.totalPieceVertices, audit.operandB.pieceCount,
                audit.costOverflow);
            const std::size_t bWork = SaturatingMultiply(
                audit.operandB.totalPieceVertices, audit.operandA.pieceCount,
                audit.costOverflow);
            audit.convexSumVertexWork = SaturatingAdd(
                aWork, bWork, audit.costOverflow);
        }
    }

    GeometryResult<MinkowskiDecompositionPortfolioAudit> AuditMinkowskiPortfolio(
        const Path& a,
        const Path& b,
        const GeometryContext& context,
        const DecompositionOptions& baseOptions)
    {
        constexpr DecompositionMergeStrategy strategies[] = {
            DecompositionMergeStrategy::HertelMehlhorn,
            DecompositionMergeStrategy::LongestSharedEdgeFirst,
            DecompositionMergeStrategy::ShortestSharedEdgeFirst,
            DecompositionMergeStrategy::SmallNBestOfThree
        };

        MinkowskiDecompositionPortfolioAudit portfolio;
        portfolio.candidates.reserve(std::size(strategies));

        for (const DecompositionMergeStrategy strategy : strategies)
        {
            if (context.IsCancelled())
                return GeometryResult<MinkowskiDecompositionPortfolioAudit>::Failure(
                    GeometryStatus::Cancelled);

            DecompositionOptions options = baseOptions;
            options.mergeToConvex = true;
            options.mergeStrategy = strategy;

            MinkowskiDecompositionCandidateAudit candidate;
            candidate.requestedStrategy = strategy;

            auto decompositionA = ConvexRegion(a, context, options);
            candidate.statusA = decompositionA.Status();
            if (decompositionA.Ok())
                candidate.operandA = QualityOf(decompositionA.Value());

            auto decompositionB = ConvexRegion(b, context, options);
            candidate.statusB = decompositionB.Status();
            if (decompositionB.Ok())
                candidate.operandB = QualityOf(decompositionB.Value());

            CompleteCosts(candidate);
            portfolio.candidates.push_back(std::move(candidate));
        }

        // The current route is the control. If it cannot describe the pair, there is
        // no meaningful portfolio baseline to return; preserve its status rather than
        // presenting a vector of failed candidates as a successful audit.
        if (portfolio.candidates.empty() || !portfolio.candidates.front().Complete())
        {
            const GeometryStatus status = portfolio.candidates.empty()
                ? GeometryStatus::NumericalFailure
                : (!IsSuccess(portfolio.candidates.front().statusA)
                    ? portfolio.candidates.front().statusA
                    : portfolio.candidates.front().statusB);
            return GeometryResult<MinkowskiDecompositionPortfolioAudit>::Failure(status);
        }

        return GeometryResult<MinkowskiDecompositionPortfolioAudit>::Success(
            std::move(portfolio));
    }
}
