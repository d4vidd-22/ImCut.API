#include "NfpCover.hpp"

#include "../Math/Predicates.hpp"
#include "../GeometryTestHooks.hpp"
#include "Minkowski.hpp"
#include "NfpLattice.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <new>
#include <span>

namespace ImCut::Geometry::Nfp
{
    namespace
    {
        // V8.1's 1M-query sweep put 132 regions on the losing side of the BVH
        // crossover for every distribution, while 784 and 1,764 regions won strongly
        // on uniform/boundary traffic. Keep the measured gap rather than building a
        // tree for medium covers on theory alone.
        constexpr std::size_t kCoverSpatialIndexThreshold = 256;

        // Highly overlapping sums often hit early in generation order. Scanning a
        // bounded hot prefix before the tree retained the BVH's large-cover win while
        // removing its clustered/interior regression.
        constexpr std::size_t kCoverLinearPrefix = 128;

        [[nodiscard]] constexpr std::size_t SaturatingAdd(
            std::size_t a, std::size_t b) noexcept
        {
            return b > (std::numeric_limits<std::size_t>::max)() - a
                ? (std::numeric_limits<std::size_t>::max)() : a + b;
        }

        [[nodiscard]] constexpr std::size_t SaturatingMultiply(
            std::size_t a, std::size_t b) noexcept
        {
            return a != 0 && b > (std::numeric_limits<std::size_t>::max)() / a
                ? (std::numeric_limits<std::size_t>::max)() : a * b;
        }

        [[nodiscard]] std::size_t EstimatedBvhBytes(std::size_t pieces) noexcept
        {
            const std::size_t twice = SaturatingMultiply(pieces, 2);
            const std::size_t nodeCount = pieces == 0 ? 0 :
                (twice == (std::numeric_limits<std::size_t>::max)()
                    ? twice : twice - 1);
            std::size_t bytes = sizeof(SegmentBVH);
            bytes = SaturatingAdd(bytes,
                SaturatingMultiply(nodeCount, sizeof(SegmentBVH::Node)));
            bytes = SaturatingAdd(bytes,
                SaturatingMultiply(pieces, sizeof(std::uint32_t) +
                    sizeof(Bounds2) + sizeof(Vec2)));
            return bytes;
        }

        [[nodiscard]] GeometryStatus CompatibilityStatus(
                                      const PreparedNfpOperand& stationary,
                                      const PreparedNfpOperand& reflectedMoving,
                                      const GeometryContext& context,
                                      const NfpOptions& options) noexcept
        {
            if (stationary.OwnerSessionIdentity() == 0 ||
                stationary.OwnerSessionIdentity() != reflectedMoving.OwnerSessionIdentity() ||
                stationary.Reflected() || !reflectedMoving.Reflected() ||
                !(stationary.Profile() == reflectedMoving.Profile()) ||
                stationary.Profile().decomposition.simplifyTolerance !=
                    options.simplifyTolerance ||
                stationary.Profile().decomposition.maxPieces != options.maxPieces ||
                stationary.Profile().decomposition.mergeStrategy !=
                options.decompositionMergeStrategy ||
                !(options.clearance >= 0.0) || !std::isfinite(options.clearance) ||
                options.coverMode > NfpCoverMode::Product)
            {
                return GeometryStatus::InvalidInput;
            }
            return stationary.Profile().ContextStatus(context);
        }

        [[nodiscard]] Contour ContourOf(const Vec2* points, std::size_t count)
        {
            Contour contour;
            if (count == 0) return contour;
            contour.Reserve(count);
            contour.MoveTo(points[0]);
            for (std::size_t i = 1; i < count; ++i)
                contour.LineTo(points[i]);
            contour.Close(0.0);
            return contour;
        }

        [[nodiscard]] GeometryCertification CombinedCertification(
            const PreparedNfpOperand& a, const PreparedNfpOperand& b,
            const ErrorBudget& budget) noexcept
        {
            if (a.Certification() == GeometryCertification::Certified &&
                b.Certification() == GeometryCertification::Certified && budget.exact)
            {
                return GeometryCertification::Certified;
            }
            if ((a.Certification() == GeometryCertification::ConservativeSuperset ||
                 a.Certification() == GeometryCertification::Certified) &&
                (b.Certification() == GeometryCertification::ConservativeSuperset ||
                 b.Certification() == GeometryCertification::Certified))
            {
                return GeometryCertification::ConservativeSuperset;
            }
            return GeometryCertification::BoundedApproximation;
        }

        [[nodiscard]] std::vector<Vec2> ClearanceSquare(double clearance)
        {
            return {
                { -clearance, -clearance }, { clearance, -clearance },
                { clearance, clearance }, { -clearance, clearance }
            };
        }

#if defined(IMCUT_GEOMETRY_INTERNAL_TESTS)
        [[nodiscard]] bool ConvexContainsHalfPlane(
            Vec2 point, const Vec2* ring, std::size_t count,
            double tolerance) noexcept
        {
            if (ring == nullptr || count < 3)
                return false;

            // Preserve PointInRing's inclusive Euclidean boundary contract before
            // using the CCW half planes as the interior proof.
            for (std::size_t i = 0, previous = count - 1;
                 i < count; previous = i++)
            {
                if (Predicates::PointOnSegment(
                        point, ring[previous], ring[i], tolerance))
                {
                    return true;
                }
            }
            for (std::size_t i = 0, previous = count - 1;
                 i < count; previous = i++)
            {
                if (Predicates::Orient2D(
                        ring[previous], ring[i], point) < 0.0)
                {
                    return false;
                }
            }
            return true;
        }
#endif

        [[nodiscard]] bool ConvexContainsBinaryFan(
            Vec2 point, const Vec2* ring, std::size_t count,
            double tolerance) noexcept
        {
            if (ring == nullptr || count < 3)
                return false;

            const Vec2 origin = ring[0];
            const double first = Predicates::Orient2D(origin, ring[1], point);
            if (first <= 0.0)
            {
                return Predicates::PointOnSegment(
                    point, origin, ring[1], tolerance);
            }

            const double last = Predicates::Orient2D(
                origin, ring[count - 1], point);
            if (last >= 0.0)
            {
                return Predicates::PointOnSegment(
                    point, origin, ring[count - 1], tolerance);
            }

            std::size_t low = 1;
            std::size_t high = count - 1;
            while (high - low > 1)
            {
                const std::size_t middle = low + (high - low) / 2;
                if (Predicates::Orient2D(origin, ring[middle], point) >= 0.0)
                    low = middle;
                else
                    high = middle;
            }

            const double side = Predicates::Orient2D(
                ring[low], ring[high], point);
            if (side > 0.0)
                return true;
            return Predicates::PointOnSegment(
                point, ring[low], ring[high], tolerance);
        }
    }

    bool NfpCover::ContainsLinear(Vec2 point, double tolerance) const noexcept
    {
        return ContainsLinearRange(point, tolerance, 0, regions_.size());
    }

    bool NfpCover::ContainsLinearRange(
        Vec2 point, double tolerance, std::size_t begin,
        std::size_t end) const noexcept
    {
        end = (std::min)(end, regions_.size());
        for (std::size_t index = begin; index < end; ++index)
        {
            const ConvexRegion& region = regions_[index];
            if (!RegionBounds(index).Contains(point, tolerance))
                continue;
            const Vec2* ring = points_.data() + region.begin;
            const PointClassification classification = Predicates::PointInRing(
                point, ring, region.count, tolerance);
            // Membership is inclusive. We intentionally do not report Boundary:
            // an edge of this region may be internal to the full union.
            if (classification != PointClassification::Outside)
                return true;
        }
        return false;
    }

    bool NfpCover::ContainsConvexRange(
        Vec2 point, double tolerance, std::size_t begin,
        std::size_t end) const noexcept
    {
        end = (std::min)(end, regions_.size());
        for (std::size_t index = begin; index < end; ++index)
        {
            const ConvexRegion& region = regions_[index];
            if (!RegionBounds(index).Contains(point, tolerance))
                continue;
            if (ConvexContainsBinaryFan(
                    point, points_.data() + region.begin,
                    region.count, tolerance))
            {
                return true;
            }
        }
        return false;
    }

    bool NfpCover::Contains(Vec2 point) const noexcept
    {
        if (!IsFinite(point))
            return false;

        // A BROAD PHASE MAY FILTER, NEVER DECIDE MORE THAN THE NARROW PREDICATE PROVES.
        //
        // Every test below accepts a point up to `tolerance` OUTSIDE a region boundary -
        // RegionBounds(...).Contains(point, tolerance) and
        // ConvexContainsBinaryFan(..., tolerance). V8.1 filtered here with NO tolerance,
        // so a point the decision predicate would have called Inside was discarded before
        // the predicate ever saw it: at all 31 probed positions along the max-x edge, a
        // point half a tolerance INSIDE was accepted while its mirror half a tolerance
        // OUTSIDE was rejected, at both small and large magnitudes (evidence 817).
        //
        // TWO STAGES, because the obvious repair costs 8x on the hottest path in the
        // kernel. Computing the magnitude and ScaledEpsilon before filtering charges every
        // query - including the far-away ones this filter exists to kill cheaply - about
        // 20 ns on top of a 2.6 ns rejection. Measured: `outside` went 2.6 -> 23 ns per
        // query across every cover size (evidence 826).
        //
        // The escape is that ScaledEpsilon is CLAMPED: it can never exceed
        // scaledEpsilonMax, whatever the magnitude. So a point outside the bounds grown by
        // that ceiling cannot be accepted by any tolerance this cover could compute, and
        // rejecting it needs no magnitude and no clamp. That is a proof, not a heuristic -
        // which is the only kind of fast path allowed back in here.
        if (!bounds_.Contains(point, profile_.tolerance.scaledEpsilonMax))
            return false;

        const double magnitude = (std::max)({
            std::fabs(point.x), std::fabs(point.y),
            std::fabs(bounds_.min.x), std::fabs(bounds_.min.y),
            std::fabs(bounds_.max.x), std::fabs(bounds_.max.y) });
        const double tolerance = profile_.tolerance.ScaledEpsilon(magnitude);

        // The exact test is only needed for points OUTSIDE the strict box. A point
        // strictly inside it is inside for every tolerance >= 0, so asking again would
        // charge boundary traffic - the very traffic this whole fix exists for - a second
        // bounds test for an answer already known.
        if (!bounds_.Contains(point) && !bounds_.Contains(point, tolerance))
            return false;

        if (representation_ == NfpCoverRepresentation::Product)
        {
            bool found = false;
            static_cast<void>(ContainsProduct(
                point, tolerance, nullptr, nullptr, found));
            return found;
        }

        if (!regionIndex_)
            return ContainsConvexRange(point, tolerance, 0, regions_.size());

        if (ContainsConvexRange(point, tolerance, 0, regionIndexOffset_))
            return true;

        bool found = false;
        const Bounds2 pointBounds{
            { point.x - tolerance, point.y - tolerance },
            { point.x + tolerance, point.y + tolerance }
        };
        regionIndex_->QueryBounds(pointBounds, [&](std::uint32_t index)
        {
            if (index < regionIndexOffset_)
                return true;
            const ConvexRegion& region = regions_[index];
            if (!RegionBounds(index).Contains(point, tolerance))
                return true;
            if (!ConvexContainsBinaryFan(
                    point, points_.data() + region.begin,
                    region.count, tolerance))
            {
                return true;
            }
            found = true;
            return false;
        });
        return found;
    }

    GeometryStatus NfpCover::ContainsProduct(
        Vec2 point, double tolerance, const GeometryContext* context,
        NfpProductQueryReport* report, bool& found) const noexcept
    {
        found = false;
        if (!productIndexA_ || !productIndexB_)
            return GeometryStatus::InvalidInput;

        // SegmentBVH proves a fixed upper bound for this stack from its build-depth
        // limit. No pair-product storage and no dynamic allocation occur in a query.
        constexpr std::size_t workingBytes =
            sizeof(std::pair<std::uint32_t, std::uint32_t>) *
            (SegmentBVH::kMaxBuildDepth * 6);
        if (report != nullptr)
        {
            *report = {};
            report->workingBytes = workingBytes;
        }
        if (context != nullptr &&
            context->Limits().maxNfpWorkingBytes < workingBytes)
        {
            return GeometryStatus::ComplexityLimit;
        }
        if (context != nullptr && context->IsCancelled())
            return GeometryStatus::Cancelled;

        std::size_t cancellationCounter = 0;
        std::size_t nodePairsVisited = 0;
        std::size_t leafPairsVisited = 0;
        std::size_t predicatesEvaluated = 0;
        const bool completed = productIndexA_->QueryMinkowskiPairs(
            *productIndexB_, point, clearance_ + tolerance,
            [&]() noexcept
            {
                const bool shouldStop = context != nullptr &&
                    context->ShouldCheckCancellation(cancellationCounter++) &&
                    context->IsCancelled();
                return !shouldStop;
            },
            [&](std::uint32_t indexA, std::uint32_t indexB) noexcept
            {
                ++predicatesEvaluated;
                if (!Minkowski::ConvexSumContains(
                        productPiecesA_.Piece(indexA),
                        productPiecesB_.Piece(indexB), point,
                        tolerance, clearance_))
                {
                    return true;
                }
                found = true;
                return false;
            },
            &nodePairsVisited, &leafPairsVisited);

        if (report != nullptr)
        {
            report->nodePairsVisited = nodePairsVisited;
            report->leafPairsVisited = leafPairsVisited;
            report->convexPredicatesEvaluated = predicatesEvaluated;
        }

        if (context != nullptr)
        {
            CountStat(*context, &GeometryStatistics::bvhQueries);
            CountStat(*context, &GeometryStatistics::bvhCandidatePairs,
                static_cast<std::uint64_t>(leafPairsVisited));
            CountStat(*context, &GeometryStatistics::convexPairs,
                static_cast<std::uint64_t>(predicatesEvaluated));
            if (!found && !completed && context->IsCancelled())
                return GeometryStatus::Cancelled;
        }
        return GeometryStatus::Success;
    }

    GeometryResult<bool> NfpCover::Contains(
        Vec2 point, const GeometryContext& context,
        NfpProductQueryReport* report) const
    {
        if (report != nullptr) *report = {};
        const GeometryStatus profileStatus = profile_.ContextStatus(context);
        if (profileStatus != GeometryStatus::Success)
            return GeometryResult<bool>::Failure(profileStatus);
        if (!IsFinite(point))
            return GeometryResult<bool>::Failure(GeometryStatus::InvalidInput);
        if (context.IsCancelled())
            return GeometryResult<bool>::Failure(GeometryStatus::Cancelled);
        if (representation_ != NfpCoverRepresentation::Product)
            return GeometryResult<bool>::Success(Contains(point));

        if (!bounds_.Contains(point, profile_.tolerance.scaledEpsilonMax))
            return GeometryResult<bool>::Success(false);
        const double magnitude = (std::max)({
            std::fabs(point.x), std::fabs(point.y),
            std::fabs(bounds_.min.x), std::fabs(bounds_.min.y),
            std::fabs(bounds_.max.x), std::fabs(bounds_.max.y) });
        const double tolerance = profile_.tolerance.ScaledEpsilon(magnitude);
        if (!bounds_.Contains(point) && !bounds_.Contains(point, tolerance))
            return GeometryResult<bool>::Success(false);

        bool found = false;
        const GeometryStatus status = ContainsProduct(
            point, tolerance, &context, report, found);
        if (status != GeometryStatus::Success)
            return GeometryResult<bool>::Failure(status);
        return GeometryResult<bool>::Success(found);
    }

#if defined(IMCUT_GEOMETRY_INTERNAL_TESTS)
    bool NfpCover::ContainsLinearReference(Vec2 point) const noexcept
    {
        // The same two-stage filter production uses, for the same reason.
        //
        // These references exist to be compared against the production path, and all
        // three carried the very defect that path had: an identical tolerance-free
        // bounds test, so any differential between them agreed - on the wrong answer.
        // Fixing production alone would make the differential fail on the CORRECTED
        // behaviour, so they move together.
        //
        // DROPPING THE FILTER INSTEAD OF FIXING IT was the first attempt here, and it
        // cost up to 640x on `outside` traffic: with no cheap reject these scan all
        // 1764 regions for a point nowhere near the cover (evidence 826). A reference
        // implementation is allowed to be simple; it is not allowed to be useless as a
        // benchmark baseline.
        if (!IsFinite(point) ||
            !bounds_.Contains(point, profile_.tolerance.scaledEpsilonMax))
        {
            return false;
        }

        const double magnitude = (std::max)({
            std::fabs(point.x), std::fabs(point.y),
            std::fabs(bounds_.min.x), std::fabs(bounds_.min.y),
            std::fabs(bounds_.max.x), std::fabs(bounds_.max.y) });
        return ContainsLinear(
            point, profile_.tolerance.ScaledEpsilon(magnitude));
    }

    std::span<const Vec2> NfpCover::RegionPointsForTesting(
        std::size_t index) const noexcept
    {
        if (index >= regions_.size())
            return {};
        const ConvexRegion& region = regions_[index];
        return { points_.data() + region.begin, region.count };
    }

    bool NfpCover::ContainsHalfPlaneReference(Vec2 point) const noexcept
    {
        // The same two-stage filter production uses, for the same reason.
        //
        // These references exist to be compared against the production path, and all
        // three carried the very defect that path had: an identical tolerance-free
        // bounds test, so any differential between them agreed - on the wrong answer.
        // Fixing production alone would make the differential fail on the CORRECTED
        // behaviour, so they move together.
        //
        // DROPPING THE FILTER INSTEAD OF FIXING IT was the first attempt here, and it
        // cost up to 640x on `outside` traffic: with no cheap reject these scan all
        // 1764 regions for a point nowhere near the cover (evidence 826). A reference
        // implementation is allowed to be simple; it is not allowed to be useless as a
        // benchmark baseline.
        if (!IsFinite(point) ||
            !bounds_.Contains(point, profile_.tolerance.scaledEpsilonMax))
        {
            return false;
        }
        const double magnitude = (std::max)({
            std::fabs(point.x), std::fabs(point.y),
            std::fabs(bounds_.min.x), std::fabs(bounds_.min.y),
            std::fabs(bounds_.max.x), std::fabs(bounds_.max.y) });
        const double tolerance = profile_.tolerance.ScaledEpsilon(magnitude);
        for (std::size_t index = 0; index < regions_.size(); ++index)
        {
            const ConvexRegion& region = regions_[index];
            if (!RegionBounds(index).Contains(point, tolerance))
                continue;
            if (ConvexContainsHalfPlane(
                    point, points_.data() + region.begin,
                    region.count, tolerance))
            {
                return true;
            }
        }
        return false;
    }

    bool NfpCover::ContainsBinaryFanReference(Vec2 point) const noexcept
    {
        // The same two-stage filter production uses, for the same reason.
        //
        // These references exist to be compared against the production path, and all
        // three carried the very defect that path had: an identical tolerance-free
        // bounds test, so any differential between them agreed - on the wrong answer.
        // Fixing production alone would make the differential fail on the CORRECTED
        // behaviour, so they move together.
        //
        // DROPPING THE FILTER INSTEAD OF FIXING IT was the first attempt here, and it
        // cost up to 640x on `outside` traffic: with no cheap reject these scan all
        // 1764 regions for a point nowhere near the cover (evidence 826). A reference
        // implementation is allowed to be simple; it is not allowed to be useless as a
        // benchmark baseline.
        if (!IsFinite(point) ||
            !bounds_.Contains(point, profile_.tolerance.scaledEpsilonMax))
        {
            return false;
        }
        const double magnitude = (std::max)({
            std::fabs(point.x), std::fabs(point.y),
            std::fabs(bounds_.min.x), std::fabs(bounds_.min.y),
            std::fabs(bounds_.max.x), std::fabs(bounds_.max.y) });
        const double tolerance = profile_.tolerance.ScaledEpsilon(magnitude);
        for (std::size_t index = 0; index < regions_.size(); ++index)
        {
            const ConvexRegion& region = regions_[index];
            if (!RegionBounds(index).Contains(point, tolerance))
                continue;
            if (ConvexContainsBinaryFan(
                    point, points_.data() + region.begin,
                    region.count, tolerance))
            {
                return true;
            }
        }
        return false;
    }

    GeometryResult<NfpCover> NfpCover::BuildProductForTesting(
        PreparedConvexPieces piecesA, PreparedConvexPieces piecesB,
        const GeometryContext& context, double clearance)
    {
        if (piecesA.empty() || piecesB.empty() || !(clearance >= 0.0) ||
            !std::isfinite(clearance) ||
            piecesA.size() > (std::numeric_limits<std::size_t>::max)() /
                piecesB.size())
        {
            return GeometryResult<NfpCover>::Failure(
                GeometryStatus::InvalidInput);
        }
        NfpCover cover;
        cover.representation_ = NfpCoverRepresentation::Product;
        cover.profile_.tolerance = context.Tolerance();
        cover.profile_.precision = context.Precision();
        cover.profile_.limits = context.Limits();
        cover.piecesA_ = piecesA.size();
        cover.piecesB_ = piecesB.size();
        cover.convexPairCount_ = piecesA.size() * piecesB.size();
        cover.clearance_ = clearance;
        cover.certification_ = GeometryCertification::Certified;
        try
        {
            cover.productPiecesA_ = std::move(piecesA);
            cover.productPiecesB_ = std::move(piecesB);
            std::vector<Bounds2> boundsA;
            std::vector<Bounds2> boundsB;
            boundsA.reserve(cover.piecesA_);
            boundsB.reserve(cover.piecesB_);
            for (std::size_t index = 0; index < cover.piecesA_; ++index)
                boundsA.push_back(cover.productPiecesA_.PieceBounds(index));
            for (std::size_t index = 0; index < cover.piecesB_; ++index)
                boundsB.push_back(cover.productPiecesB_.PieceBounds(index));
            auto indexA = std::make_shared<SegmentBVH>();
            auto indexB = std::make_shared<SegmentBVH>();
            indexA->Build(std::move(boundsA));
            indexB->Build(std::move(boundsB));
            cover.cost_.piecesA = cover.piecesA_;
            cover.cost_.piecesB = cover.piecesB_;
            cover.cost_.pairProduct = cover.convexPairCount_;
            cover.cost_.nodesA = indexA->NodeCount();
            cover.cost_.nodesB = indexB->NodeCount();
            const Bounds2 rootA = indexA->RootBounds();
            const Bounds2 rootB = indexB->RootBounds();
            cover.bounds_ = Bounds2{
                rootA.min + rootB.min, rootA.max + rootB.max };
            if (clearance > 0.0)
                cover.bounds_ = cover.bounds_.Expanded(clearance);
            cover.productIndexA_ = std::move(indexA);
            cover.productIndexB_ = std::move(indexB);
            cover.cost_.estimatedProductBytes = cover.ApproximateBytes();
            const std::size_t pairMetadata = SaturatingMultiply(
                cover.convexPairCount_, sizeof(ConvexRegion) + sizeof(Bounds2));
            cover.cost_.estimatedExplicitBytes = SaturatingAdd(
                sizeof(NfpCover), pairMetadata);
        }
        catch (const std::bad_alloc&)
        {
            return GeometryResult<NfpCover>::Failure(
                GeometryStatus::OutOfMemory);
        }
        if (cover.ApproximateBytes() > context.Limits().maxNfpResultBytes)
            return GeometryResult<NfpCover>::Failure(
                GeometryStatus::ComplexityLimit);
        return GeometryResult<NfpCover>::Success(std::move(cover));
    }
#endif

    GeometryResult<Path> NfpCover::Materialize(const GeometryContext& context) const
    {
        if (regions_.size() == 1)
        {
            ScopedGeometryTotal totalTimer(context.Diagnostics());
            CountStat(context, &GeometryStatistics::nfpMaterializations);
            const GeometryStatus profileStatus = profile_.ContextStatus(context);
            if (profileStatus != GeometryStatus::Success)
                return GeometryResult<Path>::Failure(profileStatus);

            const ConvexRegion& convex = regions_.front();
            Path region;
            region.fillRule = FillRule::NonZero;
            region.budget = budget_;
            region.contours.push_back(ContourOf(
                points_.data() + convex.begin, convex.count));
            auto result = GeometryResult<Path>::Success(std::move(region));
            result.Budget().Merge(budget_);
            return result;
        }

        auto materialized = MaterializePersistentLattice(
            *this, context, NfpLatticeGrouping::CostBased);
        if (!materialized.Ok() &&
            materialized.Status() != GeometryStatus::Empty)
        {
            return GeometryResult<Path>::Failure(materialized.Status());
        }

        NfpLatticeMaterialized payload = std::move(materialized).Value();
        Path region = std::move(payload.region);
        auto result = region.contours.empty()
            ? GeometryResult<Path>::Empty(std::move(region))
            : GeometryResult<Path>::Success(std::move(region));
        result.Budget().Merge(payload.budget);
        return result;
    }

    std::size_t NfpCover::ApproximateBytes() const noexcept
    {
        std::size_t bytes = sizeof(NfpCover) +
                            regions_.capacity() * sizeof(ConvexRegion) +
                            regionBounds_.capacity() * sizeof(Bounds2) +
                            points_.capacity() * sizeof(Vec2) +
                            groupEnds_.capacity() * sizeof(std::size_t);
        if (regionIndex_)
            bytes += sizeof(SegmentBVH) + regionIndex_->DynamicBytes();
        bytes += productPiecesA_.ApproximateBytes();
        bytes += productPiecesB_.ApproximateBytes();
        if (productIndexA_)
            bytes += sizeof(SegmentBVH) + productIndexA_->DynamicBytes();
        if (productIndexB_)
            bytes += sizeof(SegmentBVH) + productIndexB_->DynamicBytes();
        return bytes;
    }

    GeometryResult<NfpCover> NfpCover::ComputeInternal(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options,
        bool buildSpatialIndex,
        NfpCoverScratch* scratch)
    {
        ScopedGeometryTotal totalTimer(context.Diagnostics());
        CountStat(context, &GeometryStatistics::nfpCalls);
        CountStat(context, &GeometryStatistics::nfpCoverBuilds);

        {
            ScopedGeometryStage stage(
                context.Diagnostics(), GeometryDiagnosticStage::ValidationExclusive);
            const GeometryStatus status = CompatibilityStatus(
                stationary, reflectedMoving, context, options);
            if (status != GeometryStatus::Success)
                return GeometryResult<NfpCover>::Failure(status);
        }

        const auto& piecesA = stationary.ConvexPieces();
        const auto& piecesB = reflectedMoving.ConvexPieces();
        if (piecesA.empty() || piecesB.empty())
            return GeometryResult<NfpCover>::Empty(NfpCover{});

        if (piecesB.size() != 0 &&
            piecesA.size() > (std::numeric_limits<std::size_t>::max)() / piecesB.size())
        {
            return GeometryResult<NfpCover>::Failure(GeometryStatus::ComplexityLimit);
        }
        const std::size_t pairCount = piecesA.size() * piecesB.size();

        NfpCover cover;
        cover.profile_ = stationary.Profile();
        cover.piecesA_ = piecesA.size();
        cover.piecesB_ = piecesB.size();
        cover.convexPairCount_ = pairCount;
        cover.clearance_ = options.clearance;
        cover.budget_.MergeSequential(stationary.IntrinsicBudget());
        cover.budget_.MergeSequential(reflectedMoving.IntrinsicBudget());
        cover.certification_ = CombinedCertification(
            stationary, reflectedMoving, cover.budget_);

        std::size_t maximumPointCount = 0;
        auto addProduct = [&](std::size_t multiplicand,
                              std::size_t multiplier)
        {
            if (multiplicand != 0 &&
                multiplier > ((std::numeric_limits<std::size_t>::max)() -
                    maximumPointCount) / multiplicand)
            {
                return false;
            }
            maximumPointCount += multiplicand * multiplier;
            return true;
        };
        const bool pointCountSafe =
            addProduct(piecesA.Points().size(), piecesB.size()) &&
            addProduct(piecesB.Points().size(), piecesA.size()) &&
            (options.clearance <= 0.0 || addProduct(pairCount, 4));
        if (!pointCountSafe)
        {
            return GeometryResult<NfpCover>::Failure(
                GeometryStatus::ComplexityLimit);
        }

        std::size_t largestPieceA = 0;
        std::size_t largestPieceB = 0;
        for (std::size_t index = 0; index < piecesA.size(); ++index)
            largestPieceA = (std::max)(largestPieceA, piecesA.Piece(index).size());
        for (std::size_t index = 0; index < piecesB.size(); ++index)
            largestPieceB = (std::max)(largestPieceB, piecesB.Piece(index).size());
        std::size_t maximumRegionPointCount = SaturatingAdd(
            largestPieceA, largestPieceB);
        if (options.clearance > 0.0)
            maximumRegionPointCount = SaturatingAdd(maximumRegionPointCount, 4);

        std::size_t explicitBytes = sizeof(NfpCover);
        explicitBytes = SaturatingAdd(explicitBytes,
            SaturatingMultiply(pairCount, sizeof(NfpCover::ConvexRegion)));
        if (buildSpatialIndex)
            explicitBytes = SaturatingAdd(explicitBytes,
                SaturatingMultiply(pairCount, sizeof(Bounds2)));
        explicitBytes = SaturatingAdd(explicitBytes,
            SaturatingMultiply(maximumPointCount, sizeof(Vec2)));
        explicitBytes = SaturatingAdd(explicitBytes,
            SaturatingMultiply(piecesA.size(), sizeof(std::size_t)));
        if (buildSpatialIndex && pairCount > kCoverSpatialIndexThreshold)
            explicitBytes = SaturatingAdd(explicitBytes, EstimatedBvhBytes(pairCount));

        std::size_t productBytes = sizeof(NfpCover);
        productBytes = SaturatingAdd(productBytes, piecesA.ApproximateBytes());
        productBytes = SaturatingAdd(productBytes, piecesB.ApproximateBytes());
        productBytes = SaturatingAdd(productBytes, EstimatedBvhBytes(piecesA.size()));
        productBytes = SaturatingAdd(productBytes, EstimatedBvhBytes(piecesB.size()));
        const std::size_t productWorkingBytes = SaturatingMultiply(
            SaturatingAdd(piecesA.size(), piecesB.size()), sizeof(Bounds2));

        cover.cost_.piecesA = piecesA.size();
        cover.cost_.piecesB = piecesB.size();
        cover.cost_.pairProduct = pairCount;
        cover.cost_.estimatedExplicitBytes = explicitBytes;
        cover.cost_.estimatedProductBytes = productBytes;

        const ComplexityLimits& limits = context.Limits();
        const bool explicitFits = pairCount <= limits.maxConvexPairs &&
            explicitBytes <= limits.maxNfpResultBytes &&
            maximumRegionPointCount <= limits.maxPolygonVertices;
        const bool productFits = productBytes <= limits.maxNfpResultBytes &&
            productWorkingBytes <= limits.maxNfpWorkingBytes;
        bool useProduct = false;
        switch (options.coverMode)
        {
            case NfpCoverMode::Auto:
                useProduct = !explicitFits;
                break;
            case NfpCoverMode::Explicit:
                if (!explicitFits)
                    return GeometryResult<NfpCover>::Failure(
                        GeometryStatus::ComplexityLimit);
                break;
            case NfpCoverMode::Product:
                useProduct = true;
                break;
            default:
                return GeometryResult<NfpCover>::Failure(
                    GeometryStatus::InvalidInput);
        }

        if (useProduct)
        {
            if (!productFits)
                return GeometryResult<NfpCover>::Failure(
                    GeometryStatus::ComplexityLimit);
            try
            {
                GeometryStatus injected = GeometryStatus::Success;
                if (Testing::ShouldInjectFailure(
                        Testing::FailurePoint::NfpProductAllocation, injected))
                {
                    if (injected == GeometryStatus::OutOfMemory)
                        throw std::bad_alloc();
                    return GeometryResult<NfpCover>::Failure(injected);
                }
                cover.representation_ = NfpCoverRepresentation::Product;
                cover.productPiecesA_ = piecesA;
                cover.productPiecesB_ = piecesB;
                std::vector<Bounds2> boundsA;
                std::vector<Bounds2> boundsB;
                boundsA.reserve(piecesA.size());
                boundsB.reserve(piecesB.size());
                for (std::size_t index = 0; index < piecesA.size(); ++index)
                    boundsA.push_back(piecesA.PieceBounds(index));
                for (std::size_t index = 0; index < piecesB.size(); ++index)
                    boundsB.push_back(piecesB.PieceBounds(index));
                auto indexA = std::make_shared<SegmentBVH>();
                auto indexB = std::make_shared<SegmentBVH>();
                indexA->Build(std::move(boundsA));
                indexB->Build(std::move(boundsB));
                const Bounds2 rootA = indexA->RootBounds();
                const Bounds2 rootB = indexB->RootBounds();
                cover.bounds_ = Bounds2{
                    rootA.min + rootB.min, rootA.max + rootB.max };
                if (options.clearance > 0.0)
                    cover.bounds_ = cover.bounds_.Expanded(options.clearance);
                cover.cost_.nodesA = indexA->NodeCount();
                cover.cost_.nodesB = indexB->NodeCount();
                cover.productIndexA_ = std::move(indexA);
                cover.productIndexB_ = std::move(indexB);
            }
            catch (const std::bad_alloc&)
            {
                return GeometryResult<NfpCover>::Failure(
                    GeometryStatus::OutOfMemory);
            }

            CountStat(context, &GeometryStatistics::bvhBuilds, 2);
            CountStat(context, &GeometryStatistics::bvhNodes,
                static_cast<std::uint64_t>(
                    cover.cost_.nodesA + cover.cost_.nodesB));
            if (stationary.StrictlyConvex() && reflectedMoving.StrictlyConvex())
                CountStat(context, &GeometryStatistics::nfpConvexFastPath);
            else
                CountStat(context, &GeometryStatistics::nfpDecomposition);
            auto result = GeometryResult<NfpCover>::Success(std::move(cover));
            result.Budget().Merge(result.Value().budget_);
            return result;
        }

        cover.regions_.reserve(pairCount);
        if (buildSpatialIndex)
            cover.regionBounds_.reserve(pairCount);
        cover.groupEnds_.reserve(piecesA.size());
        cover.points_.reserve(maximumPointCount);

        const std::vector<Vec2> clearance = options.clearance > 0.0
            ? ClearanceSquare(options.clearance) : std::vector<Vec2>{};
        // Reuse the caller's buffers when it supplied them, so a worker running many
        // pairs allocates these three once instead of once per pair. Contents are
        // overwritten by ConvexSumInto before being read, so a dirty buffer from the
        // previous pair cannot leak into this one.
        NfpCoverScratch localScratch;
        NfpCoverScratch& buffers = scratch != nullptr ? *scratch : localScratch;
        std::vector<Vec2>& ring = buffers.ring;
        std::vector<Vec2>& sumScratch = buffers.sum;
        std::vector<Vec2>& clearanceScratch = buffers.clearance;
        std::size_t emitted = 0;
        for (std::size_t indexA = 0; indexA < piecesA.size(); ++indexA)
        {
            const std::span<const Vec2> pieceA = piecesA.Piece(indexA);
            const Bounds2& boundsA = piecesA.PieceBounds(indexA);
            for (std::size_t indexB = 0; indexB < piecesB.size(); ++indexB)
            {
                const std::span<const Vec2> pieceB = piecesB.Piece(indexB);
                const Bounds2& boundsB = piecesB.PieceBounds(indexB);
                if (context.ShouldCheckCancellation(emitted) && context.IsCancelled())
                    return GeometryResult<NfpCover>::Failure(GeometryStatus::Cancelled);
                ++emitted;

                {
                    ScopedGeometryStage stage(
                        context.Diagnostics(), GeometryDiagnosticStage::ConvexSumExclusive);
                    Minkowski::ConvexSumInto(pieceA, pieceB, ring, sumScratch);
                    if (!clearance.empty())
                    {
                        Minkowski::ConvexSumInto(
                            ring, clearance, clearanceScratch, sumScratch);
                        ring.swap(clearanceScratch);
                    }
                }
                if (ring.size() < 3)
                    continue;

                NfpCover::ConvexRegion region;
                region.begin = cover.points_.size();
                region.count = ring.size();
                Bounds2 regionBounds{
                    boundsA.min + boundsB.min,
                    boundsA.max + boundsB.max
                };
                if (options.clearance > 0.0)
                    regionBounds = regionBounds.Expanded(options.clearance);
                cover.points_.insert(cover.points_.end(), ring.begin(), ring.end());
                cover.bounds_.Add(regionBounds);
                cover.regions_.push_back(region);
                if (buildSpatialIndex)
                    cover.regionBounds_.push_back(regionBounds);
            }
            cover.groupEnds_.push_back(cover.regions_.size());
        }
        CountStat(context, &GeometryStatistics::convexPairs, emitted);
        if (stationary.StrictlyConvex() && reflectedMoving.StrictlyConvex())
            CountStat(context, &GeometryStatistics::nfpConvexFastPath);
        else
            CountStat(context, &GeometryStatistics::nfpDecomposition);

        if (buildSpatialIndex &&
            cover.regions_.size() > kCoverSpatialIndexThreshold)
        {
            cover.regionIndexOffset_ = (std::min)(
                kCoverLinearPrefix, cover.regions_.size());
            auto index = std::make_shared<SegmentBVH>();
            index->Build(std::move(cover.regionBounds_));
            CountStat(context, &GeometryStatistics::bvhBuilds);
            CountStat(context, &GeometryStatistics::bvhNodes,
                      static_cast<std::uint64_t>(index->NodeCount()));
            cover.regionIndex_ = std::move(index);
        }

        auto result = cover.regions_.empty()
            ? GeometryResult<NfpCover>::Empty(std::move(cover))
            : GeometryResult<NfpCover>::Success(std::move(cover));
        result.Budget().Merge(result.Value().budget_);
        return result;
    }

    GeometryResult<NfpCover> ComputeCover(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options,
        NfpCoverScratch* scratch)
    {
        return NfpCover::ComputeInternal(
            stationary, reflectedMoving, context, options, true, scratch);
    }

    GeometryResult<NfpCover> ComputeCoverForMaterialization(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options)
    {
        return NfpCover::ComputeInternal(
            stationary, reflectedMoving, context, options, false, nullptr);
    }

    GeometryResult<NfpContactCandidates> ComputeContactCandidates(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options)
    {
        ScopedGeometryTotal totalTimer(context.Diagnostics());
        CountStat(context, &GeometryStatistics::nfpCalls);
        CountStat(context, &GeometryStatistics::nfpContactBuilds);
        const GeometryStatus compatibility = CompatibilityStatus(
            stationary, reflectedMoving, context, options);
        if (compatibility != GeometryStatus::Success)
        {
            return GeometryResult<NfpContactCandidates>::Failure(
                compatibility);
        }

        NfpContactCandidates candidates;
        candidates.budget_.MergeSequential(stationary.IntrinsicBudget());
        candidates.budget_.MergeSequential(reflectedMoving.IntrinsicBudget());

        constexpr std::array<Vec2, 8> directions = {
            Vec2{ 1, 0 }, Vec2{ -1, 0 }, Vec2{ 0, 1 }, Vec2{ 0, -1 },
            Vec2{ 1, 1 }, Vec2{ 1, -1 }, Vec2{ -1, 1 }, Vec2{ -1, -1 }
        };
        auto support = [](const PreparedConvexPieces& pieces, Vec2 direction)
        {
            Vec2 best{};
            double bestProjection = -(std::numeric_limits<double>::infinity)();
            for (std::size_t index = 0; index < pieces.size(); ++index)
            {
                const std::span<const Vec2> piece = pieces.Piece(index);
                for (const Vec2 point : piece)
                {
                    const double projection = Dot(point, direction);
                    if (projection > bestProjection)
                    {
                        bestProjection = projection;
                        best = point;
                    }
                }
            }
            return best;
        };

        candidates.points_.reserve(directions.size());
        for (const Vec2 direction : directions)
        {
            Vec2 point = support(stationary.ConvexPieces(), direction) +
                         support(reflectedMoving.ConvexPieces(), direction);
            if (options.clearance > 0.0)
            {
                point.x += direction.x >= 0.0 ? options.clearance : -options.clearance;
                point.y += direction.y >= 0.0 ? options.clearance : -options.clearance;
            }
            bool duplicate = false;
            for (const Vec2 existing : candidates.points_)
                duplicate = duplicate || existing == point;
            if (!duplicate) candidates.points_.push_back(point);
        }

        auto result = candidates.points_.empty()
            ? GeometryResult<NfpContactCandidates>::Empty(std::move(candidates))
            : GeometryResult<NfpContactCandidates>::Success(std::move(candidates));
        result.Budget().Merge(result.Value().budget_);
        return result;
    }

    GeometryResult<NfpCover> ComposeQueryProvenance(
        GeometryResult<NfpCover> intrinsic,
        const ErrorBudget& provenanceA,
        const ErrorBudget& provenanceB)
    {
        if (!intrinsic.Ok())
            return intrinsic;
        intrinsic.Value().budget_.MergeSequential(provenanceA);
        intrinsic.Value().budget_.MergeSequential(provenanceB);
        if (!intrinsic.Value().budget_.exact &&
            intrinsic.Value().certification_ == GeometryCertification::Certified)
        {
            intrinsic.Value().certification_ =
                GeometryCertification::BoundedApproximation;
        }
        intrinsic.Budget() = intrinsic.Value().budget_;
        return intrinsic;
    }

    GeometryResult<NfpContactCandidates> ComposeQueryProvenance(
        GeometryResult<NfpContactCandidates> intrinsic,
        const ErrorBudget& provenanceA,
        const ErrorBudget& provenanceB)
    {
        if (!intrinsic.Ok())
            return intrinsic;
        intrinsic.Value().budget_.MergeSequential(provenanceA);
        intrinsic.Value().budget_.MergeSequential(provenanceB);
        intrinsic.Budget() = intrinsic.Value().budget_;
        return intrinsic;
    }
}
