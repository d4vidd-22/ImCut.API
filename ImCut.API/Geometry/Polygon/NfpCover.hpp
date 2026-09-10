#pragma once

#include "NfpPrepared.hpp"

#include "../Math/Bounds2.hpp"
#include "../Spatial/SegmentBVH.hpp"

#include <cstddef>
#include <memory>
#include <span>
#include <vector>

namespace ImCut::Geometry
{
    class GeometrySession;
}

namespace ImCut::Geometry::Nfp
{
    class PersistentLatticeBuilder;

    enum class NfpRegionBuildLevel : std::uint8_t
    {
        Cover,
        Materialized
    };

    enum class NfpCoverRepresentation : std::uint8_t
    {
        Explicit,
        Product
    };

    struct NfpCoverCost
    {
        std::size_t piecesA = 0;
        std::size_t piecesB = 0;
        std::size_t pairProduct = 0;
        std::size_t nodesA = 0;
        std::size_t nodesB = 0;
        std::size_t estimatedExplicitBytes = 0;
        std::size_t estimatedProductBytes = 0;
    };

    struct NfpProductQueryReport
    {
        std::size_t nodePairsVisited = 0;
        std::size_t leafPairsVisited = 0;
        std::size_t convexPredicatesEvaluated = 0;
        std::size_t workingBytes = 0;
    };

    // Exact union-of-convex-sums representation of the supported forbidden set.
    // It deliberately exposes no edge/boundary iterator: individual region edges may
    // be internal to the union and are not an NFP boundary.
    class NfpCover IMCUT_GEOMETRY_COUNTED(NfpCoverTag)
    {
    public:
        [[nodiscard]] bool Contains(Vec2 point) const noexcept;

        // Context-aware form exposes cancellation and hard resource status. The bool
        // compatibility overload above remains allocation-free and returns false for an
        // invalid point; callers that need to distinguish cancellation use this form.
        [[nodiscard]] GeometryResult<bool> Contains(
            Vec2 point, const GeometryContext& context,
            NfpProductQueryReport* report = nullptr) const;

        // Explicit promotion. If this is never called, no Boolean union is executed.
        [[nodiscard]] GeometryResult<Path> Materialize(
            const GeometryContext& context) const;

        [[nodiscard]] NfpRegionBuildLevel BuildLevel() const noexcept
        {
            return NfpRegionBuildLevel::Cover;
        }
        [[nodiscard]] NfpCoverRepresentation Representation() const noexcept
        {
            return representation_;
        }
        [[nodiscard]] std::size_t RegionCount() const noexcept
        {
            return representation_ == NfpCoverRepresentation::Product
                ? convexPairCount_ : regions_.size();
        }
        [[nodiscard]] std::size_t PiecesA() const noexcept { return piecesA_; }
        [[nodiscard]] std::size_t PiecesB() const noexcept { return piecesB_; }
        [[nodiscard]] std::size_t ConvexPairCount() const noexcept
        {
            return convexPairCount_;
        }
        [[nodiscard]] const Bounds2& Bounds() const noexcept { return bounds_; }
        [[nodiscard]] const ErrorBudget& Budget() const noexcept { return budget_; }
        [[nodiscard]] GeometryCertification Certification() const noexcept
        {
            return certification_;
        }
        [[nodiscard]] const NfpPreparationProfile& Profile() const noexcept
        {
            return profile_;
        }
        [[nodiscard]] const NfpCoverCost& Cost() const noexcept { return cost_; }
        [[nodiscard]] std::size_t ApproximateBytes() const noexcept;

#if defined(IMCUT_GEOMETRY_INTERNAL_TESTS)
        // Test/benchmark oracle compiled only by the standalone harness. Production
        // consumers cannot select the reference scan or inspect internal regions.
        [[nodiscard]] bool ContainsLinearReference(Vec2 point) const noexcept;
        [[nodiscard]] bool ContainsHalfPlaneReference(Vec2 point) const noexcept;
        [[nodiscard]] bool ContainsBinaryFanReference(Vec2 point) const noexcept;
        [[nodiscard]] std::span<const Vec2> RegionPointsForTesting(
            std::size_t index) const noexcept;
        [[nodiscard]] const SegmentBVH* SpatialIndexForTesting() const noexcept
        {
            return regionIndex_.get();
        }
        [[nodiscard]] static GeometryResult<NfpCover> BuildProductForTesting(
            PreparedConvexPieces piecesA, PreparedConvexPieces piecesB,
            const GeometryContext& context, double clearance = 0.0);
#endif

    private:
        struct ConvexRegion
        {
            std::size_t begin = 0;
            std::size_t count = 0;
        };

        std::vector<ConvexRegion> regions_{};
        // Query-ready linear covers own these bounds directly; indexed covers transfer
        // the same allocation into SegmentBVH, so bounds are never retained twice.
        // The internal materialization-only policy retains neither representation.
        std::vector<Bounds2> regionBounds_{};
        // One allocation for all exact convex sums. Regions reference immutable
        // subranges by offsets, so moving the cover cannot invalidate its metadata.
        std::vector<Vec2> points_{};
        // End offsets after each piece of A, retained solely to reproduce the proven
        // per-A Boolean batching policy during explicit materialization.
        std::vector<std::size_t> groupEnds_{};
        // Built eagerly before publication only for covers above the measured
        // small-R threshold. Shared immutable ownership keeps copied cache payloads
        // cheap and makes concurrent Contains calls lock-free.
        std::shared_ptr<const SegmentBVH> regionIndex_{};
        // Product representation: compact operand copies plus immutable BVHs. No
        // pair-indexed object is retained.
        PreparedConvexPieces productPiecesA_{};
        PreparedConvexPieces productPiecesB_{};
        std::shared_ptr<const SegmentBVH> productIndexA_{};
        std::shared_ptr<const SegmentBVH> productIndexB_{};
        std::size_t regionIndexOffset_ = 0;
        Bounds2 bounds_{};
        ErrorBudget budget_{};
        NfpPreparationProfile profile_{};
        GeometryCertification certification_ =
            GeometryCertification::BoundedApproximation;
        std::size_t piecesA_ = 0;
        std::size_t piecesB_ = 0;
        std::size_t convexPairCount_ = 0;
        double clearance_ = 0.0;
        NfpCoverRepresentation representation_ = NfpCoverRepresentation::Explicit;
        NfpCoverCost cost_{};

        [[nodiscard]] bool ContainsLinear(
            Vec2 point, double tolerance) const noexcept;
        [[nodiscard]] bool ContainsLinearRange(
            Vec2 point, double tolerance, std::size_t begin,
            std::size_t end) const noexcept;
        [[nodiscard]] bool ContainsConvexRange(
            Vec2 point, double tolerance, std::size_t begin,
            std::size_t end) const noexcept;
        [[nodiscard]] GeometryStatus ContainsProduct(
            Vec2 point, double tolerance, const GeometryContext* context,
            NfpProductQueryReport* report, bool& found) const noexcept;
        [[nodiscard]] Bounds2 RegionBounds(
            std::size_t index) const noexcept
        {
            if (regionIndex_)
                return regionIndex_->PrimitiveBounds(
                    static_cast<std::uint32_t>(index));
            if (index < regionBounds_.size())
                return regionBounds_[index];

            Bounds2 bounds;
            const ConvexRegion& region = regions_[index];
            for (std::size_t point = 0; point < region.count; ++point)
                bounds.Add(points_[region.begin + point]);
            return bounds;
        }
        [[nodiscard]] static GeometryResult<NfpCover> ComputeInternal(
            const PreparedNfpOperand& stationary,
            const PreparedNfpOperand& reflectedMoving,
            const GeometryContext& context,
            const NfpOptions& options,
            bool buildSpatialIndex,
            struct NfpCoverScratch* scratch);

        friend GeometryResult<NfpCover> ComputeCover(
            const PreparedNfpOperand&, const PreparedNfpOperand&,
            const GeometryContext&, const NfpOptions&, struct NfpCoverScratch*);
        friend GeometryResult<NfpCover> ComputeCoverForMaterialization(
            const PreparedNfpOperand&, const PreparedNfpOperand&,
            const GeometryContext&, const NfpOptions&);
        friend GeometryResult<NfpCover> ComposeQueryProvenance(
            GeometryResult<NfpCover>, const ErrorBudget&, const ErrorBudget&);
        friend class ::ImCut::Geometry::GeometrySession;
        friend class PersistentLatticeBuilder;
    };

    // Candidate information is intentionally a different type. It has no Contains,
    // boundary or materialization method and therefore cannot be passed accidentally
    // as proof that a placement is feasible.
    class NfpContactCandidates
    {
    public:
        [[nodiscard]] const std::vector<Vec2>& Points() const noexcept { return points_; }
        [[nodiscard]] const ErrorBudget& Budget() const noexcept { return budget_; }
        [[nodiscard]] GeometryCertification Certification() const noexcept
        {
            return GeometryCertification::CandidateOnly;
        }

    private:
        std::vector<Vec2> points_{};
        ErrorBudget budget_{};

        friend GeometryResult<NfpContactCandidates> ComputeContactCandidates(
            const PreparedNfpOperand&, const PreparedNfpOperand&,
            const GeometryContext&, const NfpOptions&);
        friend GeometryResult<NfpContactCandidates> ComposeQueryProvenance(
            GeometryResult<NfpContactCandidates>, const ErrorBudget&,
            const ErrorBudget&);
        friend class ::ImCut::Geometry::GeometrySession;
    };

    // Reusable per-worker buffers for the convex-sum inner loop.
    //
    // BuildCover allocates three point buffers per PAIR. In a batch that is three
    // allocations multiplied by the pair count, on every worker, in every generation.
    // Handing the same scratch back for the next pair keeps the buffers at high-water
    // mark and drops those allocations to three per worker per generation.
    //
    // Owned by the caller, never global and never thread_local: one instance belongs
    // to exactly one worker for the duration of a batch. Sharing one across threads is
    // a data race and is not supported.
    // Cache-line aligned and padded. The buffers' DATA is on the heap, but a
    // push_back writes the vector's own size/end pointer, which lives here. Packed
    // contiguously, three 24-byte vectors put several workers' headers on one cache
    // line and every push_back invalidates a neighbour's copy. Measured: unaligned
    // scratch cut allocations 63% and still lost 10-29% of wall time at 2-12 workers.
    // C4324 says "structure was padded due to alignment specifier". That is precisely
    // what the alignas is for and what the measurement above bought, so the warning is
    // suppressed HERE, narrowly, with the reason attached - rather than left to fire in
    // all 66 translation units that include this header. A permanent warning under a
    // /W4 contract is worse than no contract: it trains everyone to scroll past the
    // place a NEW warning would appear.
#if defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4324)
#endif
    struct alignas(64) NfpCoverScratch
    {
        std::vector<Vec2> ring;
        std::vector<Vec2> sum;
        std::vector<Vec2> clearance;
    };
#if defined(_MSC_VER)
#pragma warning(pop)
#endif

    // `scratch` is optional; nullptr keeps the call-local behaviour exactly.
    [[nodiscard]] GeometryResult<NfpCover> ComputeCover(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options = {},
        NfpCoverScratch* scratch = nullptr);

    // Internal materialization route: identical Cover semantics, but it deliberately
    // omits the repeated-query index that the immediate consumer will never query.
    [[nodiscard]] GeometryResult<NfpCover> ComputeCoverForMaterialization(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options = {});

    [[nodiscard]] GeometryResult<NfpContactCandidates> ComputeContactCandidates(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options = {});

    // Query-boundary composition shared by session and batch front-ends. Intrinsic
    // cached geometry never stores caller provenance; these helpers apply it once to
    // both wrapper and payload while preserving Success versus Empty.
    [[nodiscard]] GeometryResult<NfpCover> ComposeQueryProvenance(
        GeometryResult<NfpCover> intrinsic,
        const ErrorBudget& provenanceA,
        const ErrorBudget& provenanceB);

    [[nodiscard]] GeometryResult<NfpContactCandidates> ComposeQueryProvenance(
        GeometryResult<NfpContactCandidates> intrinsic,
        const ErrorBudget& provenanceA,
        const ErrorBudget& provenanceB);
}
