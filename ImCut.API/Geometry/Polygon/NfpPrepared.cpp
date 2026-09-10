#include "NfpPrepared.hpp"

#include "NfpCover.hpp"
#include "PolygonMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ImCut::Geometry::Nfp
{
    bool PreparedConvexPieces::Assign(std::vector<std::vector<Vec2>> pieces)
    {
        points_.clear();
        pieces_.clear();

        std::size_t pointCount = 0;
        for (const std::vector<Vec2>& piece : pieces)
        {
            if (piece.size() > (std::numeric_limits<std::uint32_t>::max)() - pointCount)
                return false;
            pointCount += piece.size();
        }
        if (pieces.size() > (std::numeric_limits<std::uint32_t>::max)())
            return false;

        points_.reserve(pointCount);
        pieces_.reserve(pieces.size());
        for (std::vector<Vec2>& piece : pieces)
        {
            if (piece.size() < 3)
                continue;
            if (Metrics::SignedArea(piece.data(), piece.size()) < 0.0)
                std::reverse(piece.begin(), piece.end());

            PreparedConvexPiece descriptor;
            descriptor.begin = static_cast<std::uint32_t>(points_.size());
            descriptor.count = static_cast<std::uint32_t>(piece.size());
            for (const Vec2 point : piece)
                descriptor.bounds.Add(point);
            points_.insert(points_.end(), piece.begin(), piece.end());
            pieces_.push_back(descriptor);
        }
        return true;
    }

    bool PreparedConvexPieces::AssignSingle(std::span<const Vec2> piece)
    {
        points_.clear();
        pieces_.clear();
        if (piece.size() < 3)
            return true;
        if (piece.size() > (std::numeric_limits<std::uint32_t>::max)())
            return false;

        points_.assign(piece.begin(), piece.end());
        if (Metrics::SignedArea(points_.data(), points_.size()) < 0.0)
            std::reverse(points_.begin(), points_.end());
        PreparedConvexPiece descriptor;
        descriptor.count = static_cast<std::uint32_t>(points_.size());
        for (const Vec2 point : points_)
            descriptor.bounds.Add(point);
        pieces_.push_back(descriptor);
        return true;
    }

    std::span<const Vec2> PreparedConvexPieces::Piece(std::size_t index) const noexcept
    {
        if (index >= pieces_.size())
            return {};
        const PreparedConvexPiece& piece = pieces_[index];
        return std::span<const Vec2>{ points_.data() + piece.begin, piece.count };
    }

    namespace
    {
        [[nodiscard]] bool SameTolerance(const GeometryTolerance& a,
                                         const GeometryTolerance& b) noexcept
        {
            return a.coordinateEpsilon == b.coordinateEpsilon &&
                   a.closure == b.closure && a.nodeMerge == b.nodeMerge &&
                   a.zeroLength == b.zeroLength && a.tinySegment == b.tinySegment &&
                   a.intersection == b.intersection &&
                   a.collinearity == b.collinearity && a.tangent == b.tangent &&
                   a.duplicate == b.duplicate && a.flatten == b.flatten &&
                   a.absoluteArea == b.absoluteArea &&
                   a.relativeArea == b.relativeArea &&
                   a.scaledEpsilonMin == b.scaledEpsilonMin &&
                   a.scaledEpsilonMax == b.scaledEpsilonMax;
        }

        [[nodiscard]] bool IsClosedPolygon(const Contour& contour) noexcept
        {
            if (!contour.closed || contour.SegmentCount() < 3)
                return false;
            for (const SegmentKind kind : contour.kinds)
                if (kind != SegmentKind::Line) return false;
            return true;
        }

        void TransformInPlace(Path& path, const Transform2& transform) noexcept
        {
            for (Contour& contour : path.contours)
            {
                transform.ApplyBatch(
                    contour.nodes.data(), contour.nodes.data(), contour.nodes.size());
                transform.ApplyBatch(
                    contour.handles.data(), contour.handles.data(), contour.handles.size());
            }
        }
    }

    bool NfpPreparationProfile::operator==(const NfpPreparationProfile& other) const noexcept
    {
        return SameTolerance(tolerance, other.tolerance) && precision == other.precision &&
               lattice == other.lattice &&
               decomposition.earClippingLimit == other.decomposition.earClippingLimit &&
               decomposition.mergeToConvex == other.decomposition.mergeToConvex &&
               decomposition.mergeStrategy == other.decomposition.mergeStrategy &&
               decomposition.smallNPortfolioTriangleLimit ==
                   other.decomposition.smallNPortfolioTriangleLimit &&
               decomposition.simplifyTolerance == other.decomposition.simplifyTolerance &&
               decomposition.maxPieces == other.decomposition.maxPieces &&
               decomposition.maxSimplifyAttempts == other.decomposition.maxSimplifyAttempts &&
               limits.maxSegments == other.limits.maxSegments &&
               limits.maxFlattenPoints == other.limits.maxFlattenPoints &&
               limits.maxIntersections == other.limits.maxIntersections &&
               limits.maxPolygonVertices == other.limits.maxPolygonVertices &&
               limits.maxContours == other.limits.maxContours &&
               limits.maxSubdivisionDepth == other.limits.maxSubdivisionDepth &&
               limits.maxConvexPairs == other.limits.maxConvexPairs &&
               limits.maxNfpWorkingBytes == other.limits.maxNfpWorkingBytes &&
               limits.maxNfpResultBytes == other.limits.maxNfpResultBytes &&
               limits.maxConvolutionSegments == other.limits.maxConvolutionSegments &&
               limits.maxConvolutionPairTests == other.limits.maxConvolutionPairTests &&
               limits.maxNestingDepth == other.limits.maxNestingDepth;
    }

    bool NfpPreparationProfile::CompatibleWith(
        const GeometryContext& context) const noexcept
    {
        return ContextStatus(context) == GeometryStatus::Success;
    }

    GeometryStatus NfpPreparationProfile::ContextStatus(
        const GeometryContext& context) const noexcept
    {
        if (!SameTolerance(tolerance, context.Tolerance()) ||
            precision != context.Precision())
        {
            return GeometryStatus::InvalidInput;
        }

        // The comparison itself now lives in GeometryContext.hpp as LimitsAreTighter, so
        // the prepared collision routes enforce the identical rule rather than a second
        // copy of it that could drift. This was the only place in V8.1 that had it.
        return LimitsAreTighter(context.Limits(), limits)
                   ? GeometryStatus::ComplexityLimit
                   : GeometryStatus::Success;
    }

    GeometryResult<PreparedNfpOperand> PreparedNfpOperand::Build(
        const PreparedShapeDefinition& definition,
        std::uint64_t ownerSessionIdentity,
        std::uint64_t geometryIdentity,
        double rotationRadians,
        bool reflected,
        const NfpLatticeIdentity& lattice,
        const GeometryContext& context,
        const NfpOptions& options,
        const NfpOperandFeatures* knownFeatures)
    {
        return BuildTransient(
            definition.LocalPath(), definition.Identity(), ownerSessionIdentity,
            geometryIdentity, rotationRadians, reflected, lattice, context, options,
            knownFeatures);
    }

    GeometryResult<PreparedNfpOperand> PreparedNfpOperand::BuildTransient(
        const Path& path,
        std::uint64_t definitionIdentity,
        std::uint64_t ownerSessionIdentity,
        std::uint64_t geometryIdentity,
        double rotationRadians,
        bool reflected,
        const NfpLatticeIdentity& lattice,
        const GeometryContext& context,
        const NfpOptions& options,
        const NfpOperandFeatures* knownFeatures)
    {
        if (!std::isfinite(rotationRadians) || !(options.simplifyTolerance >= 0.0) ||
            !std::isfinite(options.simplifyTolerance) ||
            options.simplifyTolerance > kMaxSupportedSimplifyTolerance ||
            options.routingPolicy > NfpRoutingPolicy::Forced ||
            options.coverMode > NfpCoverMode::Product ||
            options.decompositionMergeStrategy >
                DecompositionMergeStrategy::SmallNBestOfThree ||
            !(lattice.requestedResolution > 0.0) ||
            !std::isfinite(lattice.requestedResolution) ||
            !(lattice.maximumSafeCoordinate > 0.0) ||
            !std::isfinite(lattice.maximumSafeCoordinate) ||
            lattice.maximumSafeCoordinate > Quantization::kMaxSafeCoordinate)
        {
            return GeometryResult<PreparedNfpOperand>::Failure(GeometryStatus::InvalidInput);
        }

        PreparedNfpOperand result;
        result.ownerSessionIdentity_ = ownerSessionIdentity;
        result.geometryIdentity_ = geometryIdentity;
        result.definitionIdentity_ = definitionIdentity;
        result.rotationRadians_ = rotationRadians;
        result.reflected_ = reflected;
        result.profile_.tolerance = context.Tolerance();
        result.profile_.precision = context.Precision();
        result.profile_.lattice = lattice;
        result.profile_.limits = context.Limits();
        result.profile_.decomposition.simplifyTolerance = options.simplifyTolerance;
        result.profile_.decomposition.maxPieces = options.maxPieces;
        result.profile_.decomposition.mergeStrategy =
            options.decompositionMergeStrategy;

        Path intrinsic = path;
        intrinsic.budget = ErrorBudget{};
        {
            ScopedGeometryStage stage(
                context.Diagnostics(), GeometryDiagnosticStage::TransformExclusive);
            if (rotationRadians != 0.0)
                TransformInPlace(intrinsic, Transform2::Rotation(rotationRadians));
            if (reflected)
                TransformInPlace(
                    intrinsic, Transform2{ -1.0, 0.0, 0.0, -1.0, 0.0, 0.0 });
        }
        result.orientedPath_ = std::move(intrinsic);

        if (knownFeatures != nullptr)
        {
            result.features_ = *knownFeatures;
            result.features_.bounds = Metrics::ComputeBounds(result.orientedPath_);
        }
        else
        {
            auto analyzed = AnalyzeOperandFeatures(result.orientedPath_, context);
            if (!analyzed.Ok())
            {
                return GeometryResult<PreparedNfpOperand>::Failure(
                    analyzed.Status());
            }
            result.features_ = std::move(analyzed).Value();
        }

        if (result.orientedPath_.contours.size() == 1 &&
            IsClosedPolygon(result.orientedPath_.contours.front()) &&
            result.features_.strictlyConvex)
        {
            result.strictlyConvex_ = true;
            if (!result.convexPieces_.AssignSingle(
                    result.orientedPath_.contours.front().nodes))
                return GeometryResult<PreparedNfpOperand>::Failure(
                    GeometryStatus::ComplexityLimit);
            result.certification_ = GeometryCertification::Certified;
        }
        else
        {
            GeometryResult<ConvexDecompositionResult> decomposition;
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::DecompositionExclusive);
                decomposition = Decompose::ConvexRegion(
                    result.orientedPath_, context, result.profile_.decomposition);
            }
            if (!decomposition.Ok())
                return GeometryResult<PreparedNfpOperand>::Failure(decomposition.Status());
            ConvexDecompositionResult prepared = std::move(decomposition).Value();
            result.intrinsicBudget_ = prepared.budget;
            if (!result.convexPieces_.Assign(std::move(prepared.pieces)))
                return GeometryResult<PreparedNfpOperand>::Failure(
                    GeometryStatus::ComplexityLimit);
            result.certification_ = result.intrinsicBudget_.exact
                ? GeometryCertification::Certified
                : GeometryCertification::BoundedApproximation;
        }

        if (result.convexPieces_.empty())
            return GeometryResult<PreparedNfpOperand>::Empty(std::move(result));

        auto prepared = GeometryResult<PreparedNfpOperand>::Success(std::move(result));
        prepared.Budget().Merge(prepared.Value().intrinsicBudget_);
        return prepared;
    }

    std::size_t PreparedNfpOperand::ApproximateBytes() const noexcept
    {
        std::size_t bytes = sizeof(PreparedNfpOperand) +
                            orientedPath_.contours.capacity() * sizeof(Contour) +
                            convexPieces_.ApproximateBytes();
        for (const Contour& contour : orientedPath_.contours)
        {
            bytes += contour.nodes.capacity() * sizeof(Vec2);
            bytes += contour.handles.capacity() * sizeof(Vec2);
            bytes += contour.kinds.capacity() * sizeof(SegmentKind);
        }
        return bytes;
    }

    GeometryResult<NfpResult> ComputePrepared(
        const PreparedNfpOperand& stationary,
        const PreparedNfpOperand& reflectedMoving,
        const GeometryContext& context,
        const NfpOptions& options)
    {
        ScopedGeometryTotal totalTimer(context.Diagnostics());
        if (stationary.Reflected() || !reflectedMoving.Reflected() ||
            !(stationary.Profile() == reflectedMoving.Profile()) ||
            stationary.Profile().decomposition.simplifyTolerance !=
                options.simplifyTolerance ||
            stationary.Profile().decomposition.maxPieces != options.maxPieces ||
            stationary.Profile().decomposition.mergeStrategy !=
                options.decompositionMergeStrategy ||
            !(options.clearance >= 0.0) || !std::isfinite(options.clearance) ||
            options.coverMode > NfpCoverMode::Product)
        {
            return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
        }

        auto cover = ComputeCoverForMaterialization(
            stationary, reflectedMoving, context, options);
        if (!cover.Ok())
            return GeometryResult<NfpResult>::Failure(cover.Status());
        auto region = cover.Value().Materialize(context);
        if (!region.Ok() && region.Status() != GeometryStatus::Empty)
            return GeometryResult<NfpResult>::Failure(region.Status());

        NfpResult result;
        result.piecesA = cover.Value().PiecesA();
        result.piecesB = cover.Value().PiecesB();
        result.budget = region.Budget();
        result.exact = stationary.StrictlyConvex() && reflectedMoving.StrictlyConvex() &&
                       options.simplifyTolerance == 0.0 && options.clearance == 0.0 &&
                       result.budget.exact;
        result.certification = cover.Value().Certification() ==
                                   GeometryCertification::Certified &&
                               result.budget.exact
            ? GeometryCertification::Certified
            : GeometryCertification::BoundedApproximation;

        if (region.Status() == GeometryStatus::Empty || region.Value().contours.empty())
        {
            auto empty = GeometryResult<NfpResult>::Empty(std::move(result));
            empty.Budget().Merge(empty.Value().budget);
            return empty;
        }

        result.region = std::move(region).Value();
        result.region.budget = result.budget;
        auto success = GeometryResult<NfpResult>::Success(std::move(result));
        success.Budget().Merge(success.Value().budget);
        return success;
    }
}
