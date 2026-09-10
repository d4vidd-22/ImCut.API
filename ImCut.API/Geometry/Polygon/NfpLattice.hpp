#pragma once

#include "NfpCover.hpp"

#include <cstddef>
#include <cstdint>

namespace ImCut::Geometry::Nfp
{
    enum class NfpLatticeGrouping : std::uint8_t
    {
        PerStationaryPiece,
        CostBased,
        OneShot
    };

    enum class NfpLatticeResolvedGrouping : std::uint8_t
    {
        PerStationaryPiece,
        StationaryFanIn2,
        StationaryFanIn4,
        StationaryFanIn8,
        OneShot,
        Streaming
    };

    struct NfpLatticeReport
    {
        double resolution = 0.0;
        Vec2 origin{};
        std::int64_t maximumIntegerMagnitude = 0;
        std::size_t inputRings = 0;
        std::size_t inputVertices = 0;
        std::size_t groupUnions = 0;
        std::size_t unionPasses = 0;
        std::size_t intermediateRings = 0;
        std::size_t conversionsToInteger = 0;
        std::size_t conversionsToDouble = 0;
        std::size_t streamBatches = 0;
        std::size_t workingPeakBytes = 0;
        std::size_t resultPeakBytes = 0;
        NfpLatticeGrouping grouping = NfpLatticeGrouping::PerStationaryPiece;
        NfpLatticeResolvedGrouping resolvedGrouping =
            NfpLatticeResolvedGrouping::PerStationaryPiece;
    };

    struct NfpLatticeMaterialized
    {
        Path region{};
        ErrorBudget budget{};
        GeometryCertification certification =
            GeometryCertification::BoundedApproximation;
        NfpLatticeReport report{};
    };

    // All intermediate unions stay on one session/profile-bound int64 lattice. Only
    // the cover input and final materialized Path cross numeric domains.
    [[nodiscard]] GeometryResult<NfpLatticeMaterialized> MaterializePersistentLattice(
        const NfpCover& cover,
        const GeometryContext& context,
        NfpLatticeGrouping grouping = NfpLatticeGrouping::CostBased);
}
