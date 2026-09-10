#pragma once

#include "../GeometryCertification.hpp"
#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../Math/Bounds2.hpp"
#include "../PreparedGeometry.hpp"
#include "../Quantization.hpp"
#include "ConvexDecomposition.hpp"
#include "Nfp.hpp"
#include "NfpAuto.hpp"

#include <cstdint>
#include <memory>
#include <span>
#include <vector>

namespace ImCut::Geometry::Nfp
{
    struct PreparedConvexPiece
    {
        std::uint32_t begin = 0;
        std::uint32_t count = 0;
        Bounds2 bounds{};
    };

    // Compact immutable derivative: two allocations regardless of piece count.
    // Public decomposition keeps its historical vector<vector<Vec2>> payload, while
    // hot prepared NFP queries get contiguous traversal and stable span views.
    class PreparedConvexPieces
    {
    public:
        [[nodiscard]] bool Assign(std::vector<std::vector<Vec2>> pieces);
        [[nodiscard]] bool AssignSingle(std::span<const Vec2> piece);
        [[nodiscard]] bool empty() const noexcept { return pieces_.empty(); }
        [[nodiscard]] std::size_t size() const noexcept { return pieces_.size(); }
        [[nodiscard]] std::span<const Vec2> Piece(std::size_t index) const noexcept;
        [[nodiscard]] const Bounds2& PieceBounds(std::size_t index) const noexcept
        {
            return pieces_[index].bounds;
        }
        [[nodiscard]] const std::vector<Vec2>& Points() const noexcept { return points_; }
        [[nodiscard]] const std::vector<PreparedConvexPiece>& Descriptors() const noexcept
        {
            return pieces_;
        }
        [[nodiscard]] std::size_t ApproximateBytes() const noexcept
        {
            return points_.capacity() * sizeof(Vec2) +
                   pieces_.capacity() * sizeof(PreparedConvexPiece);
        }

    private:
        std::vector<Vec2> points_;
        std::vector<PreparedConvexPiece> pieces_;
    };

    struct NfpLatticeIdentity
    {
        double requestedResolution = Quantization::kDefaultResolution;
        double maximumSafeCoordinate = Quantization::kMaxSafeCoordinate;

        [[nodiscard]] bool operator==(const NfpLatticeIdentity& other) const noexcept
        {
            return requestedResolution == other.requestedResolution &&
                   maximumSafeCoordinate == other.maximumSafeCoordinate;
        }
    };

    struct NfpPreparationProfile
    {
        GeometryTolerance tolerance{};
        PrecisionMode precision = PrecisionMode::Production;
        NfpLatticeIdentity lattice{};
        DecompositionOptions decomposition{};
        // Ceilings in force while the reusable derivative was built. A query may
        // loosen them, but must not silently bypass a tighter computational ceiling.
        ComplexityLimits limits{};

        [[nodiscard]] bool operator==(const NfpPreparationProfile& other) const noexcept;
        [[nodiscard]] bool CompatibleWith(const GeometryContext& context) const noexcept;
        [[nodiscard]] GeometryStatus ContextStatus(
            const GeometryContext& context) const noexcept;
    };

    // Context-dependent, exact-orientation derivative owned by one GeometrySession.
    // PreparedShapeDefinition remains the intrinsic source; this object holds the
    // rotated/reflected path and decomposition produced under one explicit profile.
    class PreparedNfpOperand
    {
    public:
        [[nodiscard]] static GeometryResult<PreparedNfpOperand> Build(
            const PreparedShapeDefinition& definition,
            std::uint64_t ownerSessionIdentity,
            std::uint64_t geometryIdentity,
            double rotationRadians,
            bool reflected,
            const NfpLatticeIdentity& lattice,
            const GeometryContext& context,
            const NfpOptions& options,
            const NfpOperandFeatures* knownFeatures = nullptr);

        // Call-local adapter for the non-session API. It builds the same immutable
        // derivative directly from a Path, without creating a cache identity or a
        // temporary PreparedShapeDefinition. Input provenance is deliberately excluded
        // here and is composed once when the query result crosses the public boundary.
        [[nodiscard]] static GeometryResult<PreparedNfpOperand> BuildTransient(
            const Path& path,
            std::uint64_t definitionIdentity,
            std::uint64_t ownerSessionIdentity,
            std::uint64_t geometryIdentity,
            double rotationRadians,
            bool reflected,
            const NfpLatticeIdentity& lattice,
            const GeometryContext& context,
            const NfpOptions& options,
            const NfpOperandFeatures* knownFeatures = nullptr);

        [[nodiscard]] std::uint64_t OwnerSessionIdentity() const noexcept
        {
            return ownerSessionIdentity_;
        }
        [[nodiscard]] std::uint64_t GeometryIdentity() const noexcept
        {
            return geometryIdentity_;
        }
        [[nodiscard]] std::uint64_t DefinitionIdentity() const noexcept
        {
            return definitionIdentity_;
        }
        [[nodiscard]] double RotationRadians() const noexcept { return rotationRadians_; }
        [[nodiscard]] bool Reflected() const noexcept { return reflected_; }
        [[nodiscard]] bool StrictlyConvex() const noexcept { return strictlyConvex_; }
        [[nodiscard]] const NfpOperandFeatures& Features() const noexcept
        {
            return features_;
        }
        [[nodiscard]] const NfpPreparationProfile& Profile() const noexcept { return profile_; }
        [[nodiscard]] const Path& OrientedPath() const noexcept { return orientedPath_; }
        [[nodiscard]] const PreparedConvexPieces& ConvexPieces() const noexcept
        {
            return convexPieces_;
        }
        [[nodiscard]] const ErrorBudget& IntrinsicBudget() const noexcept
        {
            return intrinsicBudget_;
        }
        [[nodiscard]] GeometryCertification Certification() const noexcept
        {
            return certification_;
        }
        [[nodiscard]] std::size_t ApproximateBytes() const noexcept;

    private:
        std::uint64_t ownerSessionIdentity_ = 0;
        std::uint64_t geometryIdentity_ = 0;
        std::uint64_t definitionIdentity_ = 0;
        double rotationRadians_ = 0.0;
        bool reflected_ = false;
        bool strictlyConvex_ = false;
        NfpOperandFeatures features_{};
        NfpPreparationProfile profile_{};
        Path orientedPath_{};
        PreparedConvexPieces convexPieces_{};
        ErrorBudget intrinsicBudget_{};
        GeometryCertification certification_ = GeometryCertification::BoundedApproximation;
    };

    using PreparedNfpOperandPtr = std::shared_ptr<const PreparedNfpOperand>;

    [[nodiscard]] GeometryResult<NfpResult> ComputePrepared(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options = {});
}
