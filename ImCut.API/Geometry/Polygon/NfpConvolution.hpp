#pragma once

#include "NfpLattice.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ImCut::Geometry::Nfp
{
    enum class ConvolutionFeatureSource : std::uint8_t
    {
        StationaryEdgeMovingVertex,
        MovingEdgeStationaryVertex
    };

    struct ConvolutionSegment
    {
        Vec2 start{};
        Vec2 end{};
        std::uint32_t edgeIndex = 0;
        std::uint32_t vertexIndex = 0;
        ConvolutionFeatureSource source =
            ConvolutionFeatureSource::StationaryEdgeMovingVertex;
    };

    // Experimental audit artifact, not an NFP region. The reduced convolution is a
    // boundary superset; without arrangement loop extraction plus the independent
    // boundary predicate it cannot prove membership or feasibility.
    struct ReducedConvolutionAudit
    {
        std::vector<ConvolutionSegment> segments{};
        std::size_t verticesA = 0;
        std::size_t verticesB = 0;
        std::size_t convexVerticesA = 0;
        std::size_t convexVerticesB = 0;
        std::size_t reflexVerticesA = 0;
        std::size_t reflexVerticesB = 0;
        std::size_t fullVertexEdgeSegments = 0;
        std::size_t reducedSegmentCount = 0;
        std::size_t arrangementPairTests = 0;
        std::size_t arrangementIntersections = 0;
        std::size_t arrangementFragmentCount = 0;
        std::size_t oracleBoundaryEdges = 0;
        std::size_t coveredOracleBoundaryEdges = 0;
        ErrorBudget budget{};

        [[nodiscard]] bool CompleteBoundaryCoverage() const noexcept
        {
            return oracleBoundaryEdges != 0 &&
                   coveredOracleBoundaryEdges == oracleBoundaryEdges;
        }
        [[nodiscard]] bool OracleDependent() const noexcept { return true; }
        [[nodiscard]] bool ProductionEligible() const noexcept { return false; }
        [[nodiscard]] GeometryCertification Certification() const noexcept
        {
            return GeometryCertification::CandidateOnly;
        }
    };

    // Implements the reduced convolution generation rule for two simple polygonal
    // operands, audits its arrangement size, and verifies boundary-superset coverage
    // against an independently materialized exact Cover. It intentionally does not
    // expose a Path: loop/nesting extraction is not yet production-certified.
    [[nodiscard]] GeometryResult<ReducedConvolutionAudit> AuditReducedConvolution(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options = {});
}
