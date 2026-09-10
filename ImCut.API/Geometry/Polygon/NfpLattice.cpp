#include "NfpLattice.hpp"

#include "PolygonBackend.hpp"
#include "PolygonConversion.hpp"
#include "Minkowski.hpp"
#include "../GeometryTestHooks.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <new>

namespace ImCut::Geometry::Nfp
{
    namespace
    {
        [[nodiscard]] GeometryStatus ToGeometryStatus(
            Backend::BackendStatus status) noexcept
        {
            switch (status)
            {
                case Backend::BackendStatus::Success: return GeometryStatus::Success;
                case Backend::BackendStatus::DepthLimit:
                    return GeometryStatus::ComplexityLimit;
                case Backend::BackendStatus::OutOfMemory:
                    return GeometryStatus::OutOfMemory;
                case Backend::BackendStatus::Failed:
                default: return GeometryStatus::NumericalFailure;
            }
        }

        [[nodiscard]] QuantizationScale ChooseProfileScale(
            const Bounds2& localBounds, const NfpLatticeIdentity& identity) noexcept
        {
            QuantizationScale result;
            if (!(identity.requestedResolution > 0.0) ||
                !std::isfinite(identity.requestedResolution) ||
                !(identity.maximumSafeCoordinate > 0.0) ||
                !std::isfinite(identity.maximumSafeCoordinate) ||
                identity.maximumSafeCoordinate > Quantization::kMaxSafeCoordinate ||
                !IsFinite(localBounds))
            {
                return result;
            }

            double scale = 1.0 / identity.requestedResolution;
            if (!localBounds.IsEmpty())
            {
                const double extreme = (std::max)({
                    std::fabs(localBounds.min.x), std::fabs(localBounds.max.x),
                    std::fabs(localBounds.min.y), std::fabs(localBounds.max.y) });
                if (extreme > 0.0)
                    scale = (std::min)(scale,
                        identity.maximumSafeCoordinate / extreme);
            }
            if (!(scale > 0.0) || !std::isfinite(scale))
                return result;

            result.scale = scale;
            result.resolution = 1.0 / scale;
            result.valid = true;
            return result;
        }

        [[nodiscard]] bool IsSupportedGrouping(NfpLatticeGrouping grouping) noexcept
        {
            switch (grouping)
            {
                case NfpLatticeGrouping::PerStationaryPiece:
                case NfpLatticeGrouping::CostBased:
                case NfpLatticeGrouping::OneShot:
                    return true;
                default:
                    return false;
            }
        }

        [[nodiscard]] std::size_t StationaryFanIn(
            NfpLatticeResolvedGrouping grouping) noexcept
        {
            switch (grouping)
            {
                case NfpLatticeResolvedGrouping::StationaryFanIn2: return 2;
                case NfpLatticeResolvedGrouping::StationaryFanIn4: return 4;
                case NfpLatticeResolvedGrouping::StationaryFanIn8: return 8;
                default: return 1;
            }
        }

        [[nodiscard]] std::size_t IntRingsBytes(
            const Backend::IntRings& rings) noexcept
        {
            std::size_t bytes = rings.capacity() * sizeof(Backend::IntRing);
            for (const Backend::IntRing& ring : rings)
            {
                const std::size_t payload =
                    ring.capacity() * sizeof(Backend::IntPoint);
                if (payload > (std::numeric_limits<std::size_t>::max)() - bytes)
                    return (std::numeric_limits<std::size_t>::max)();
                bytes += payload;
            }
            return bytes;
        }

        [[nodiscard]] std::size_t PathBytes(const Path& path) noexcept
        {
            std::size_t bytes = path.contours.capacity() * sizeof(Contour);
            for (const Contour& contour : path.contours)
            {
                const std::size_t payload =
                    contour.nodes.capacity() * sizeof(Vec2) +
                    contour.handles.capacity() * sizeof(Vec2) +
                    contour.kinds.capacity() * sizeof(SegmentKind);
                if (payload > (std::numeric_limits<std::size_t>::max)() - bytes)
                    return (std::numeric_limits<std::size_t>::max)();
                bytes += payload;
            }
            return bytes;
        }




    }

    class PersistentLatticeBuilder
    {
    public:
        [[nodiscard]] static GeometryResult<NfpLatticeMaterialized> BuildProduct(
            const NfpCover& cover, const GeometryContext& context,
            NfpLatticeGrouping grouping, NfpLatticeMaterialized materialized)
        {
            try
            {
                GeometryStatus injected = GeometryStatus::Success;
                if (Testing::ShouldInjectFailure(
                        Testing::FailurePoint::NfpProductMaterializationAllocation,
                        injected))
                {
                    if (injected == GeometryStatus::OutOfMemory)
                        throw std::bad_alloc();
                    return GeometryResult<NfpLatticeMaterialized>::Failure(injected);
                }
                if (!cover.productIndexA_ || !cover.productIndexB_ ||
                    cover.productPiecesA_.empty() || cover.productPiecesB_.empty() ||
                    !IsFinite(cover.bounds_))
                {
                    return GeometryResult<NfpLatticeMaterialized>::Failure(
                        GeometryStatus::InvalidInput);
                }

                Convert::LatticeFrame frame;
                frame.origin = cover.bounds_.Center();
                Bounds2 localBounds;
                localBounds.Add(cover.bounds_.min - frame.origin);
                localBounds.Add(cover.bounds_.max - frame.origin);
                frame.scale = ChooseProfileScale(localBounds, cover.profile_.lattice);
                if (!frame.scale.valid)
                    return GeometryResult<NfpLatticeMaterialized>::Failure(
                        GeometryStatus::Degenerate);

                materialized.report.resolution = frame.scale.resolution;
                materialized.report.origin = frame.origin;
                materialized.report.inputRings = cover.convexPairCount_;
                materialized.report.conversionsToInteger = 1;
                materialized.report.resolvedGrouping =
                    NfpLatticeResolvedGrouping::Streaming;

                const std::size_t workingLimit =
                    context.Limits().maxNfpWorkingBytes;
                const std::size_t resultLimit =
                    context.Limits().maxNfpResultBytes;
                const Backend::TreeLimits treeLimits{
                    context.Limits().maxNestingDepth };
                Backend::IntRings batch;
                Backend::IntRings accumulated;
                Backend::IntRings partial;
                NfpCoverScratch scratch;
                const std::array<Vec2, 4> clearance{
                    Vec2{ -cover.clearance_, -cover.clearance_ },
                    Vec2{ cover.clearance_, -cover.clearance_ },
                    Vec2{ cover.clearance_, cover.clearance_ },
                    Vec2{ -cover.clearance_, cover.clearance_ }
                };

                auto updatePeak = [&]() noexcept
                {
                    const std::size_t scratchBytes =
                        scratch.ring.capacity() * sizeof(Vec2) +
                        scratch.sum.capacity() * sizeof(Vec2) +
                        scratch.clearance.capacity() * sizeof(Vec2);
                    const std::size_t current = IntRingsBytes(batch) +
                        IntRingsBytes(partial) + scratchBytes;
                    materialized.report.workingPeakBytes = (std::max)(
                        materialized.report.workingPeakBytes, current);
                    materialized.report.resultPeakBytes = (std::max)(
                        materialized.report.resultPeakBytes,
                        IntRingsBytes(accumulated));
                };

                auto executeFlat = [&](Backend::IntRingsView input,
                                       Backend::IntRings& output) -> GeometryStatus
                {
                    if (context.IsCancelled()) return GeometryStatus::Cancelled;
                    Backend::BackendStatus backendStatus;
                    {
                        ScopedGeometryStage stage(context.Diagnostics(),
                            GeometryDiagnosticStage::BooleanBackendExclusive);
                        backendStatus = Backend::Default().ExecuteFlat(
                            Backend::Operation::Union, Backend::Fill::NonZero,
                            input, Backend::IntRingsView{}, output);
                    }
                    ++materialized.report.unionPasses;
                    CountStat(context, &GeometryStatistics::booleanOperations);
                    CountStat(context, &GeometryStatistics::booleanBatches);
                    return ToGeometryStatus(backendStatus);
                };

                auto flush = [&]() -> GeometryStatus
                {
                    if (batch.empty()) return GeometryStatus::Success;
                    updatePeak();
                    if (materialized.report.workingPeakBytes > workingLimit)
                        return GeometryStatus::ComplexityLimit;
                    partial.clear();
                    GeometryStatus status = executeFlat(batch, partial);
                    if (!IsSuccess(status)) return status;
                    ++materialized.report.streamBatches;
                    ++materialized.report.groupUnions;
                    batch.clear();

                    if (accumulated.empty())
                    {
                        accumulated = std::move(partial);
                        partial.clear();
                    }
                    else
                    {
                        Backend::IntRings merge;
                        merge.reserve(accumulated.size() + partial.size());
                        for (Backend::IntRing& ring : accumulated)
                            merge.push_back(std::move(ring));
                        for (Backend::IntRing& ring : partial)
                            merge.push_back(std::move(ring));
                        accumulated.clear();
                        partial.clear();
                        if (IntRingsBytes(merge) > workingLimit)
                            return GeometryStatus::ComplexityLimit;
                        status = executeFlat(merge, accumulated);
                        if (!IsSuccess(status)) return status;
                    }
                    materialized.report.intermediateRings = accumulated.size();
                    updatePeak();
                    return IntRingsBytes(accumulated) <= resultLimit
                        ? GeometryStatus::Success
                        : GeometryStatus::ComplexityLimit;
                };

                std::size_t pairCounter = 0;
                std::size_t vertexCount = 0;
                for (std::size_t indexA = 0;
                     indexA < cover.productPiecesA_.size(); ++indexA)
                {
                    for (std::size_t indexB = 0;
                         indexB < cover.productPiecesB_.size(); ++indexB)
                    {
                        if (context.ShouldCheckCancellation(pairCounter) &&
                            context.IsCancelled())
                        {
                            return GeometryResult<NfpLatticeMaterialized>::Failure(
                                GeometryStatus::Cancelled);
                        }
                        ++pairCounter;
                        {
                            ScopedGeometryStage stage(context.Diagnostics(),
                                GeometryDiagnosticStage::ConvexSumExclusive);
                            Minkowski::ConvexSumInto(
                                cover.productPiecesA_.Piece(indexA),
                                cover.productPiecesB_.Piece(indexB),
                                scratch.ring, scratch.sum);
                            if (cover.clearance_ > 0.0)
                            {
                                Minkowski::ConvexSumInto(
                                    std::span<const Vec2>{ scratch.ring },
                                    std::span<const Vec2>{ clearance },
                                    scratch.clearance, scratch.sum);
                                scratch.ring.swap(scratch.clearance);
                            }
                        }
                        if (scratch.ring.size() < 3) continue;
                        if (scratch.ring.size() >
                            context.Limits().maxPolygonVertices)
                        {
                            return GeometryResult<NfpLatticeMaterialized>::Failure(
                                GeometryStatus::ComplexityLimit);
                        }

                        Backend::IntRing integerRing;
                        integerRing.reserve(scratch.ring.size());
                        for (Vec2 point : scratch.ring)
                        {
                            const Backend::IntPoint integer = frame.ToInteger(point);
                            const std::int64_t magnitude = (std::max)(
                                integer.x >= 0 ? integer.x : -integer.x,
                                integer.y >= 0 ? integer.y : -integer.y);
                            materialized.report.maximumIntegerMagnitude = (std::max)(
                                materialized.report.maximumIntegerMagnitude, magnitude);
                            integerRing.push_back(integer);
                        }
                        vertexCount = integerRing.size() >
                            (std::numeric_limits<std::size_t>::max)() - vertexCount
                            ? (std::numeric_limits<std::size_t>::max)()
                            : vertexCount + integerRing.size();

                        const std::size_t prospective = IntRingsBytes(batch) +
                            sizeof(Backend::IntRing) +
                            integerRing.capacity() * sizeof(Backend::IntPoint) +
                            scratch.ring.capacity() * sizeof(Vec2) +
                            scratch.sum.capacity() * sizeof(Vec2) +
                            scratch.clearance.capacity() * sizeof(Vec2);
                        if (!batch.empty() && prospective > workingLimit)
                        {
                            const GeometryStatus status = flush();
                            if (!IsSuccess(status))
                                return GeometryResult<NfpLatticeMaterialized>::Failure(status);
                        }
                        batch.push_back(std::move(integerRing));
                        updatePeak();
                        if (materialized.report.workingPeakBytes > workingLimit)
                            return GeometryResult<NfpLatticeMaterialized>::Failure(
                                GeometryStatus::ComplexityLimit);
                    }
                }
                CountStat(context, &GeometryStatistics::convexPairs,
                    static_cast<std::uint64_t>(pairCounter));
                materialized.report.inputVertices = vertexCount;
                if (static_cast<double>(
                        materialized.report.maximumIntegerMagnitude) >
                    cover.profile_.lattice.maximumSafeCoordinate)
                {
                    return GeometryResult<NfpLatticeMaterialized>::Failure(
                        GeometryStatus::Degenerate);
                }
                const GeometryStatus flushed = flush();
                if (!IsSuccess(flushed))
                    return GeometryResult<NfpLatticeMaterialized>::Failure(flushed);

                if (accumulated.empty())
                {
                    materialized.region.budget = materialized.budget;
                    auto empty = GeometryResult<NfpLatticeMaterialized>::Empty(
                        std::move(materialized));
                    empty.Budget().Merge(empty.Value().budget);
                    return empty;
                }

                Backend::PolyTree finalTree;
                Backend::BackendStatus backendStatus;
                {
                    ScopedGeometryStage stage(context.Diagnostics(),
                        GeometryDiagnosticStage::BooleanBackendExclusive);
                    backendStatus = Backend::Default().Execute(
                        Backend::Operation::Union, Backend::Fill::NonZero,
                        accumulated, Backend::IntRingsView{}, treeLimits, finalTree);
                }
                ++materialized.report.unionPasses;
                CountStat(context, &GeometryStatistics::booleanOperations);
                CountStat(context, &GeometryStatistics::booleanBatches);
                const GeometryStatus finalStatus = ToGeometryStatus(backendStatus);
                if (!IsSuccess(finalStatus))
                    return GeometryResult<NfpLatticeMaterialized>::Failure(finalStatus);

                {
                    ScopedGeometryStage stage(context.Diagnostics(),
                        GeometryDiagnosticStage::ResultConversionExclusive);
                    materialized.region = Convert::PolyTreeToPath(
                        finalTree, frame, FillRule::NonZero, context);
                }
                materialized.report.conversionsToDouble = 1;
                materialized.report.resultPeakBytes = (std::max)(
                    materialized.report.resultPeakBytes,
                    PathBytes(materialized.region));
                if (materialized.report.resultPeakBytes > resultLimit)
                    return GeometryResult<NfpLatticeMaterialized>::Failure(
                        GeometryStatus::ComplexityLimit);

                materialized.budget.AddSequentialQuantization(
                    frame.scale.MaxDisplacement());
                materialized.certification =
                    GeometryCertification::BoundedApproximation;
                materialized.region.budget = materialized.budget;
                const Convert::ResultValidation validation =
                    Convert::ValidateAndPrune(materialized.region, context);
                if (validation.nonFinite)
                    return GeometryResult<NfpLatticeMaterialized>::Failure(
                        GeometryStatus::NumericalFailure);
                auto result = materialized.region.contours.empty()
                    ? GeometryResult<NfpLatticeMaterialized>::Empty(
                        std::move(materialized))
                    : GeometryResult<NfpLatticeMaterialized>::Success(
                        std::move(materialized));
                result.Budget().Merge(result.Value().budget);
                return result;
            }
            catch (const std::bad_alloc&)
            {
                return GeometryResult<NfpLatticeMaterialized>::Failure(
                    GeometryStatus::OutOfMemory);
            }
        }

        [[nodiscard]] static GeometryResult<NfpLatticeMaterialized> Build(
            const NfpCover& cover, const GeometryContext& context,
            NfpLatticeGrouping grouping)
        {
            ScopedGeometryTotal totalTimer(context.Diagnostics());
            CountStat(context, &GeometryStatistics::nfpMaterializations);

            const GeometryStatus profileStatus = cover.profile_.ContextStatus(context);
            if (profileStatus != GeometryStatus::Success)
            {
                return GeometryResult<NfpLatticeMaterialized>::Failure(
                    profileStatus);
            }

            if (!IsSupportedGrouping(grouping))
            {
                return GeometryResult<NfpLatticeMaterialized>::Failure(
                    GeometryStatus::InvalidInput);
            }
            NfpLatticeMaterialized materialized;
            materialized.report.grouping = grouping;
            materialized.report.resolvedGrouping =
                grouping == NfpLatticeGrouping::OneShot
                    ? NfpLatticeResolvedGrouping::OneShot
                    : NfpLatticeResolvedGrouping::PerStationaryPiece;
            materialized.budget = cover.budget_;
            materialized.certification = cover.certification_;
            materialized.region.fillRule = FillRule::NonZero;

            if (cover.representation_ == NfpCoverRepresentation::Product)
                return BuildProduct(
                    cover, context, grouping, std::move(materialized));

            if (cover.regions_.empty())
            {
                materialized.region.budget = materialized.budget;
                auto empty = GeometryResult<NfpLatticeMaterialized>::Empty(
                    std::move(materialized));
                empty.Budget().Merge(empty.Value().budget);
                return empty;
            }
            if (!IsFinite(cover.bounds_))
            {
                return GeometryResult<NfpLatticeMaterialized>::Failure(
                    GeometryStatus::InvalidInput);
            }

            Convert::LatticeFrame frame;
            frame.origin = cover.bounds_.Center();
            Bounds2 localBounds;
            localBounds.Add(cover.bounds_.min - frame.origin);
            localBounds.Add(cover.bounds_.max - frame.origin);
            frame.scale = ChooseProfileScale(localBounds, cover.profile_.lattice);
            if (!frame.scale.valid)
            {
                return GeometryResult<NfpLatticeMaterialized>::Failure(
                    GeometryStatus::Degenerate);
            }

            materialized.report.resolution = frame.scale.resolution;
            materialized.report.origin = frame.origin;
            materialized.report.inputRings = cover.regions_.size();
            materialized.report.conversionsToInteger = 1;

            Backend::IntRings rings;
            rings.reserve(cover.regions_.size());
            std::size_t vertexCount = 0;
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::BooleanConversionExclusive);
                for (const NfpCover::ConvexRegion& region : cover.regions_)
                {
                    if (region.count >
                        context.Limits().maxPolygonVertices - vertexCount)
                    {
                        return GeometryResult<NfpLatticeMaterialized>::Failure(
                            GeometryStatus::ComplexityLimit);
                    }
                    vertexCount += region.count;

                    Backend::IntRing ring;
                    ring.reserve(region.count);
                    const Vec2* points = cover.points_.data() + region.begin;
                    for (std::size_t pointIndex = 0;
                         pointIndex < region.count; ++pointIndex)
                    {
                        const Vec2 point = points[pointIndex];
                        const Backend::IntPoint integer = frame.ToInteger(point);
                        const std::int64_t magnitude = (std::max)(
                            integer.x >= 0 ? integer.x : -integer.x,
                            integer.y >= 0 ? integer.y : -integer.y);
                        materialized.report.maximumIntegerMagnitude = (std::max)(
                            materialized.report.maximumIntegerMagnitude, magnitude);
                        ring.push_back(integer);
                    }
                    rings.push_back(std::move(ring));
                }
            }
            materialized.report.inputVertices = vertexCount;

            if (static_cast<double>(materialized.report.maximumIntegerMagnitude) >
                cover.profile_.lattice.maximumSafeCoordinate)
            {
                return GeometryResult<NfpLatticeMaterialized>::Failure(
                    GeometryStatus::Degenerate);
            }

            const Backend::TreeLimits limits{ context.Limits().maxNestingDepth };
            auto executeFinalUnion = [&](Backend::IntRingsView subject,
                                    Backend::PolyTree& output) -> GeometryStatus
            {
                if (context.IsCancelled())
                    return GeometryStatus::Cancelled;
                Backend::BackendStatus backendStatus;
                {
                    ScopedGeometryStage stage(
                        context.Diagnostics(), GeometryDiagnosticStage::BooleanBackendExclusive);
                    backendStatus = Backend::Default().Execute(
                        Backend::Operation::Union, Backend::Fill::NonZero,
                        subject, Backend::IntRingsView{}, limits, output);
                }
                ++materialized.report.unionPasses;
                CountStat(context, &GeometryStatistics::booleanOperations);
                CountStat(context, &GeometryStatistics::booleanBatches);
                return ToGeometryStatus(backendStatus);
            };

            auto executeFlatUnion = [&](Backend::IntRingsView subject,
                                        Backend::IntRings& output) -> GeometryStatus
            {
                if (context.IsCancelled())
                    return GeometryStatus::Cancelled;
                Backend::BackendStatus backendStatus;
                {
                    ScopedGeometryStage stage(
                        context.Diagnostics(), GeometryDiagnosticStage::BooleanBackendExclusive);
                    backendStatus = Backend::Default().ExecuteFlat(
                        Backend::Operation::Union, Backend::Fill::NonZero,
                        subject, Backend::IntRingsView{}, output);
                }
                ++materialized.report.unionPasses;
                CountStat(context, &GeometryStatistics::booleanOperations);
                CountStat(context, &GeometryStatistics::booleanBatches);
                return ToGeometryStatus(backendStatus);
            };

            Backend::PolyTree finalTree;
            NfpLatticeResolvedGrouping resolvedGrouping =
                grouping == NfpLatticeGrouping::OneShot
                    ? NfpLatticeResolvedGrouping::OneShot
                    : NfpLatticeResolvedGrouping::PerStationaryPiece;
            if (grouping == NfpLatticeGrouping::CostBased)
            {
                // Keep raw Boolean batches near or below 24 rings. Below that point,
                // launch overhead dominates on the frozen corpus; above it, Clipper's
                // solve growth dominates. Larger groups retain the proven per-A path;
                // a balanced tree was faster in one corpus case but increased bytes
                // by roughly 44%, so it did not pass the Pareto gate.
                if (cover.piecesB_ <= 3)
                    resolvedGrouping = NfpLatticeResolvedGrouping::StationaryFanIn8;
                else if (cover.piecesB_ <= 6)
                    resolvedGrouping = NfpLatticeResolvedGrouping::StationaryFanIn4;
                else if (cover.piecesB_ <= 12)
                    resolvedGrouping = NfpLatticeResolvedGrouping::StationaryFanIn2;
                else
                    resolvedGrouping =
                        NfpLatticeResolvedGrouping::PerStationaryPiece;
            }
            materialized.report.resolvedGrouping = resolvedGrouping;

            if (resolvedGrouping == NfpLatticeResolvedGrouping::OneShot)
            {
                const GeometryStatus status = executeFinalUnion(rings, finalTree);
                if (!IsSuccess(status))
                    return GeometryResult<NfpLatticeMaterialized>::Failure(status);
            }
            else
            {
                Backend::IntRings accumulated;
                // The union normally reduces ring count. Reserving the input count is
                // a conservative lower-overhead starting point and avoids growing the
                // outer ring vector once per group.
                accumulated.reserve(rings.size());
                Backend::IntRings partial;
                std::size_t begin = 0;
                std::size_t stationaryGroup = 0;
                const std::size_t fanIn = StationaryFanIn(resolvedGrouping);
                while (begin < rings.size())
                {
                    if (stationaryGroup >= cover.groupEnds_.size())
                        break;
                    const std::size_t groupLimit = (std::min)(
                        stationaryGroup + fanIn, cover.groupEnds_.size());
                    const std::size_t end = cover.groupEnds_[groupLimit - 1];
                    stationaryGroup = groupLimit;
                    if (end <= begin)
                        continue;
                    const Backend::IntRingsView groupRings{
                        rings.data() + begin, end - begin };
                    begin = end;

                    const GeometryStatus status = executeFlatUnion(groupRings, partial);
                    if (!IsSuccess(status))
                        return GeometryResult<NfpLatticeMaterialized>::Failure(status);
                    ++materialized.report.groupUnions;
                    accumulated.reserve(accumulated.size() + partial.size());
                    for (Backend::IntRing& ring : partial)
                        accumulated.push_back(std::move(ring));
                }

                if (accumulated.empty())
                {
                    materialized.budget.AddSequentialQuantization(
                        frame.scale.MaxDisplacement());
                    materialized.certification =
                        GeometryCertification::BoundedApproximation;
                    materialized.region.budget = materialized.budget;
                    auto empty = GeometryResult<NfpLatticeMaterialized>::Empty(
                        std::move(materialized));
                    empty.Budget().Merge(empty.Value().budget);
                    return empty;
                }

                materialized.report.intermediateRings = accumulated.size();

                const GeometryStatus status = executeFinalUnion(accumulated, finalTree);
                if (!IsSuccess(status))
                    return GeometryResult<NfpLatticeMaterialized>::Failure(status);
            }

            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::ResultConversionExclusive);
                materialized.region = Convert::PolyTreeToPath(
                    finalTree, frame, FillRule::NonZero, context);
            }
            materialized.report.conversionsToDouble = 1;
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::BudgetExclusive);
                materialized.budget.AddSequentialQuantization(
                    frame.scale.MaxDisplacement());
                materialized.certification =
                    GeometryCertification::BoundedApproximation;
                materialized.region.budget = materialized.budget;
            }

            const Convert::ResultValidation validation =
                Convert::ValidateAndPrune(materialized.region, context);
            if (validation.nonFinite)
                return GeometryResult<NfpLatticeMaterialized>::Failure(
                    GeometryStatus::NumericalFailure);
            if (materialized.region.contours.empty())
            {
                auto empty = GeometryResult<NfpLatticeMaterialized>::Empty(
                    std::move(materialized));
                empty.Budget().Merge(empty.Value().budget);
                return empty;
            }

            auto success = GeometryResult<NfpLatticeMaterialized>::Success(
                std::move(materialized));
            success.Budget().Merge(success.Value().budget);
            return success;
        }
    };

    GeometryResult<NfpLatticeMaterialized> MaterializePersistentLattice(
        const NfpCover& cover, const GeometryContext& context,
        NfpLatticeGrouping grouping)
    {
        return PersistentLatticeBuilder::Build(
            cover, context, grouping);
    }
}
