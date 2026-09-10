#include "Nfp.hpp"

#include "Minkowski.hpp"
#include "NfpAuto.hpp"
#include "NfpPrepared.hpp"

#include <cmath>
#include <limits>

namespace ImCut::Geometry::Nfp
{
    namespace
    {
        [[nodiscard]] Path Rotated(const Path& path, double radians)
        {
            if (radians == 0.0)
                return path;
            return TransformPath(path, Transform2::Rotation(radians));
        }

        // An axis-aligned square of side 2*clearance centred on the origin.
        //
        // Summing with this grows the region by at least `clearance` in every direction.
        // A disc would be a tighter grow, and it would have to be approximated by arcs -
        // which puts an approximation error inside the one number whose job is to be a
        // safety margin. Erring outward is the safe direction for a clearance.
        [[nodiscard]] Path ClearanceSquare(double clearance)
        {
            Contour ring;
            ring.MoveTo({ -clearance, -clearance });
            ring.LineTo({ clearance, -clearance });
            ring.LineTo({ clearance, clearance });
            ring.LineTo({ -clearance, clearance });
            ring.Close(0.0);

            Path path;
            path.fillRule = FillRule::NonZero;
            path.contours.push_back(std::move(ring));
            return path;
        }

        [[nodiscard]] DecompositionOptions ToDecomposition(const NfpOptions& options)
        {
            DecompositionOptions decomposition;
            decomposition.simplifyTolerance = options.simplifyTolerance;
            decomposition.maxPieces = options.maxPieces;
            decomposition.mergeStrategy = options.decompositionMergeStrategy;
            return decomposition;
        }

        // The option checks shared by every inner-fit entry point.
        //
        // Preparation and query are separate calls now, and both have to refuse exactly
        // the same options, or a value rejected by one could be accepted by the other.
        [[nodiscard]] bool ValidInnerFitOptions(const NfpOptions& options) noexcept
        {
            if (!(options.clearance >= 0.0) || !std::isfinite(options.clearance))
                return false;
            if (!(options.simplifyTolerance >= 0.0) ||
                !std::isfinite(options.simplifyTolerance) ||
                options.simplifyTolerance > kMaxSupportedSimplifyTolerance ||
                options.routingPolicy > NfpRoutingPolicy::Forced ||
                options.coverMode > NfpCoverMode::Product ||
                options.decompositionMergeStrategy >
                    DecompositionMergeStrategy::SmallNBestOfThree)
            {
                return false;
            }
            return true;
        }

        // The polygon an inner fit actually erodes by: the part, rotated, then grown by
        // the clearance. Depends only on the part, never on the container.
        //
        // `stripProvenance` is the difference between the two callers. A prepared
        // derivative must be provenance-free so it can be reused by anyone, and hands the
        // stripped stages back through PreparedInnerFitPart::carriedBudget_. The one-shot
        // route keeps the caller's budget on the Path, which is how Minkowski::Erode has
        // always picked it up. Both end at the same total - MergeSequential is addition,
        // so the order the two contributions arrive in does not matter - and the
        // V8_1Ifp differential tests hold that equality down.
        [[nodiscard]] GeometryResult<Path> BuildStructuringElement(
            const Path& part, double rotationRadians, const GeometryContext& context,
            const NfpOptions& options, bool stripProvenance)
        {
            Path structuring;
            {
                ScopedGeometryStage transformStage(
                    context.Diagnostics(), GeometryDiagnosticStage::TransformExclusive);
                structuring = Rotated(part, rotationRadians);
            }
            if (stripProvenance)
                structuring.budget = ErrorBudget{};

            // Clearance shrinks the feasible region, so it is applied by growing the PART
            // before eroding: a part that has to keep `clearance` away from the container
            // wall behaves exactly like a part that is that much bigger.
            if (options.clearance > 0.0)
            {
                auto grown = Minkowski::SumRegions(
                    structuring, ClearanceSquare(options.clearance), context,
                    ToDecomposition(options));
                if (!grown.Ok() && grown.Status() != GeometryStatus::Empty)
                    return GeometryResult<Path>::Failure(grown.Status());
                if (grown.Status() == GeometryStatus::Empty)
                    return GeometryResult<Path>::Empty(Path{});

                structuring = std::move(grown).Value();
            }

            return GeometryResult<Path>::Success(std::move(structuring));
        }

        [[nodiscard]] GeometryResult<NfpResult> Finish(GeometryResult<Path> region,
                                                       std::size_t piecesA, std::size_t piecesB,
                                                       bool exact,
                                                       const GeometryContext& context)
        {
            ScopedGeometryStage conversionStage(
                context.Diagnostics(), GeometryDiagnosticStage::ResultConversionExclusive);
            if (!region.Ok() && region.Status() != GeometryStatus::Empty)
                return GeometryResult<NfpResult>::Failure(region.Status());

            NfpResult result;
            // The child result owns the complete chain: input provenance plus every
            // Minkowski/Boolean stage. Re-merging an ancestor accumulator here counted
            // caller provenance twice (and three times on clearance routes).
            result.budget = region.Budget();
            result.piecesA = piecesA;
            result.piecesB = piecesB;
            result.exact = exact && result.budget.exact;
            result.certification = result.exact
                ? GeometryCertification::Certified
                : GeometryCertification::BoundedApproximation;

            if (region.Status() == GeometryStatus::Empty || region.Value().contours.empty())
            {
                auto empty = GeometryResult<NfpResult>::Empty(std::move(result));
                empty.Budget().Merge(empty.Value().budget);
                return empty;
            }

            result.region = std::move(region).Value();
            auto success = GeometryResult<NfpResult>::Success(std::move(result));
            success.Budget().Merge(success.Value().budget);
            return success;
        }

        [[nodiscard]] GeometryResult<NfpResult> ComposeInputProvenance(
            GeometryResult<NfpResult> intrinsic,
            const ErrorBudget& stationary,
            const ErrorBudget& moving)
        {
            if (!intrinsic.Ok())
                return intrinsic;
            intrinsic.Value().budget.MergeSequential(stationary);
            intrinsic.Value().budget.MergeSequential(moving);
            intrinsic.Value().region.budget = intrinsic.Value().budget;
            intrinsic.Budget() = intrinsic.Value().budget;
            return intrinsic;
        }
    }

    GeometryResult<NfpResult> Compute(const Path& stationary, const Path& moving,
                                      double rotationRadians, const GeometryContext& context,
                                      const NfpOptions& options)
    {
        // A non-HM assignment remains an explicit legacy force. The default HM marker
        // resolves through the certified pair selector.
        if (options.routingPolicy == NfpRoutingPolicy::Auto &&
            options.decompositionMergeStrategy ==
                DecompositionMergeStrategy::HertelMehlhorn)
        {
            AdaptiveNfpOptions adaptive;
            adaptive.geometry = options;
            adaptive.algorithm = NfpAlgorithm::Auto;
            adaptive.allowProvenHoleFilter = options.allowProvenHoleFilter;
            auto selected = ComputeAdaptive(
                stationary, moving, rotationRadians, context, adaptive);
            if (!selected.Ok())
                return GeometryResult<NfpResult>::Failure(selected.Status());

            const GeometryStatus status = selected.Status();
            NfpResult payload = std::move(selected).Value().result;
            auto result = status == GeometryStatus::Empty
                ? GeometryResult<NfpResult>::Empty(std::move(payload))
                : GeometryResult<NfpResult>::Success(std::move(payload));
            result.Budget() = result.Value().budget;
            return result;
        }

        ScopedGeometryTotal totalTimer(context.Diagnostics());
        {
            ScopedGeometryStage validationStage(
                context.Diagnostics(), GeometryDiagnosticStage::ValidationExclusive);
            if (!stationary.IsStructurallyValid() || !moving.IsStructurallyValid())
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
            if (stationary.HasOpenContours() || moving.HasOpenContours())
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidTopology);
            if (!std::isfinite(rotationRadians))
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
            if (!(options.clearance >= 0.0) || !std::isfinite(options.clearance))
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
            if (!(options.simplifyTolerance >= 0.0) ||
                !std::isfinite(options.simplifyTolerance) ||
                options.simplifyTolerance > kMaxSupportedSimplifyTolerance ||
                options.routingPolicy > NfpRoutingPolicy::Forced ||
                options.coverMode > NfpCoverMode::Product ||
                options.decompositionMergeStrategy >
                    DecompositionMergeStrategy::SmallNBestOfThree)
            {
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
            }
        }

        // Raw and session/prepared entry points converge here. The transient operands
        // carry no cache identity and exclude caller provenance, but use the exact same
        // convex analysis, decomposition, clearance and materialization engine.
        // Zero is deliberately rejected by prepared compatibility checks. This owner
        // is call-local metadata only (no session/cache observes it), but must still be
        // a valid non-zero identity so both transient operands pass the same contract.
        constexpr std::uint64_t kTransientOwner =
            (std::numeric_limits<std::uint64_t>::max)();
        constexpr std::uint64_t kStationaryGeometry = 1;
        constexpr std::uint64_t kMovingGeometry = 2;
        const NfpLatticeIdentity lattice{};
        auto preparedA = PreparedNfpOperand::BuildTransient(
            stationary, 1, kTransientOwner, kStationaryGeometry, 0.0, false,
            lattice, context, options);
        if (!preparedA.Ok())
            return GeometryResult<NfpResult>::Failure(preparedA.Status());
        auto preparedB = PreparedNfpOperand::BuildTransient(
            moving, 2, kTransientOwner, kMovingGeometry, rotationRadians, true,
            lattice, context, options);
        if (!preparedB.Ok())
            return GeometryResult<NfpResult>::Failure(preparedB.Status());

        auto intrinsic = ComputePrepared(
            preparedA.Value(), preparedB.Value(), context, options);
        return ComposeInputProvenance(
            std::move(intrinsic), stationary.budget, moving.budget);
    }

    // NOTE ON PROVENANCE, so nobody "fixes" this into a double count.
    //
    // Compute() ends in ComposeInputProvenance and InnerFit() does not. That asymmetry is
    // correct and deliberate.
    //
    // Compute() builds its operands with PreparedNfpOperand::BuildTransient, which
    // EXCLUDES the input Paths' budgets by design, so the caller's error would be missing
    // unless it is composed at the end.
    //
    // InnerFit() runs through Minkowski::ErodePrepared, whose chain already carries it:
    // Boolean::Difference merges subject.budget and clip.budget, so the CONTAINER's
    // provenance arrives with the complement, and the PART's provenance is handed in
    // explicitly as PreparedInnerFitOperand::provenance and composed exactly once at the
    // erosion boundary. Calling ComposeInputProvenance here as well would count both
    // operands twice.
    //
    // V8_1ErrorBudget.InnerFitAndErodeComposeCallerProvenanceExactlyOnce and the prepared
    // route's sibling test hold this down: attaching provenance to the inputs must move
    // the reported total by exactly A + B, no more.
    GeometryResult<NfpResult> InnerFit(const Path& container, const Path& part,
                                       double rotationRadians, const GeometryContext& context,
                                       const NfpOptions& options)
    {
        ScopedGeometryTotal totalTimer(context.Diagnostics());
        {
            ScopedGeometryStage validationStage(
                context.Diagnostics(), GeometryDiagnosticStage::ValidationExclusive);
            if (!container.IsStructurallyValid() || !part.IsStructurallyValid())
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
            if (container.HasOpenContours() || part.HasOpenContours())
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidTopology);
            if (!std::isfinite(rotationRadians))
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
            if (!ValidInnerFitOptions(options))
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
        }

        // Build the element, then hand it to Erode - deliberately NOT via
        // PrepareInnerFitPart.
        //
        // Preparing here would decompose the element into convex pieces before Erode ever
        // got to notice that the container is a rectangle and answer in closed form
        // without reading a single one of them. Measured on the corpus, that is 1.0-2.7 ms
        // of work the answer never looks at: routing the one-shot call through the
        // prepared object made `raw` and `prep` the same number.
        //
        // The two routes still share every implementation that matters - this element
        // builder, Minkowski::Erode/ErodePrepared, and the exact rectangle form - and the
        // V8_1Ifp differential tests assert they agree on geometry, status and budget
        // rather than leaving it to construction.
        auto structuring = BuildStructuringElement(part, rotationRadians, context, options,
                                                   /*stripProvenance=*/false);
        if (!structuring.Ok() && structuring.Status() != GeometryStatus::Empty)
            return GeometryResult<NfpResult>::Failure(structuring.Status());
        if (structuring.Status() == GeometryStatus::Empty)
            return GeometryResult<NfpResult>::Empty(NfpResult{});

        auto region = Minkowski::Erode(container, structuring.Value(), context,
                                       ToDecomposition(options));
        return Finish(std::move(region), 0, 0, true, context);
    }

    std::size_t PreparedInnerFitPart::ApproximateBytes() const noexcept
    {
        return sizeof(PreparedInnerFitPart) - sizeof(Minkowski::PreparedErosionElement) +
               element_.ApproximateBytes();
    }

    bool PreparedInnerFitPart::CompatibleWith(double rotationRadians,
                                              const NfpOptions& options) const noexcept
    {
        // Bitwise equality on the doubles is deliberate. These are not measurements to be
        // compared with a tolerance; they are the exact parameters that produced this
        // element, and "close enough" would silently answer with the wrong polygon.
        return valid_ &&
               rotationRadians == rotationRadians_ &&
               options.clearance == clearance_ &&
               options.simplifyTolerance == simplifyTolerance_ &&
               options.maxPieces == maxPieces_ &&
               options.decompositionMergeStrategy == mergeStrategy_;
    }

    GeometryResult<PreparedInnerFitPart> PrepareInnerFitPart(
        const Path& part, double rotationRadians, const GeometryContext& context,
        const NfpOptions& options)
    {
        {
            ScopedGeometryStage validationStage(
                context.Diagnostics(), GeometryDiagnosticStage::ValidationExclusive);
            if (!part.IsStructurallyValid())
                return GeometryResult<PreparedInnerFitPart>::Failure(
                    GeometryStatus::InvalidInput);
            if (part.HasOpenContours())
                return GeometryResult<PreparedInnerFitPart>::Failure(
                    GeometryStatus::InvalidTopology);
            if (!std::isfinite(rotationRadians))
                return GeometryResult<PreparedInnerFitPart>::Failure(
                    GeometryStatus::InvalidInput);
            if (!ValidInnerFitOptions(options))
                return GeometryResult<PreparedInnerFitPart>::Failure(
                    GeometryStatus::InvalidInput);
            if (part.contours.empty())
                return GeometryResult<PreparedInnerFitPart>::Empty(PreparedInnerFitPart{});
        }

        const DecompositionOptions decomposition = ToDecomposition(options);

        // Provenance is stripped before any intrinsic stage runs: this derivative is
        // meant to be reusable by any caller, so it must not bake in the error the
        // geometry happened to arrive with. What accumulates below is this preparation's
        // own cost, and the caller's error is re-composed per query in InnerFitPrepared.
        auto built = BuildStructuringElement(part, rotationRadians, context, options,
                                             /*stripProvenance=*/true);
        if (!built.Ok() && built.Status() != GeometryStatus::Empty)
            return GeometryResult<PreparedInnerFitPart>::Failure(built.Status());
        if (built.Status() == GeometryStatus::Empty)
            return GeometryResult<PreparedInnerFitPart>::Empty(PreparedInnerFitPart{});

        const Path structuring = std::move(built).Value();

        auto element = Minkowski::PrepareErosionElement(structuring, context, decomposition);
        if (!element.Ok() && element.Status() != GeometryStatus::Empty)
            return GeometryResult<PreparedInnerFitPart>::Failure(element.Status());
        if (element.Status() == GeometryStatus::Empty || !element.Value().Valid())
            return GeometryResult<PreparedInnerFitPart>::Empty(PreparedInnerFitPart{});

        PreparedInnerFitPart prepared;
        prepared.element_ = std::move(element).Value();
        // Zero unless clearance ran; either way this is intrinsic, never provenance.
        prepared.carriedBudget_ = structuring.budget;
        prepared.intrinsicBudget_ = prepared.carriedBudget_;
        prepared.intrinsicBudget_.MergeSequential(prepared.element_.Budget());
        prepared.rotationRadians_ = rotationRadians;
        prepared.clearance_ = options.clearance;
        prepared.simplifyTolerance_ = options.simplifyTolerance;
        prepared.maxPieces_ = options.maxPieces;
        prepared.mergeStrategy_ = options.decompositionMergeStrategy;
        prepared.valid_ = true;

        auto success = GeometryResult<PreparedInnerFitPart>::Success(std::move(prepared));
        success.Budget() = success.Value().Budget();
        return success;
    }

    GeometryResult<NfpResult> InnerFitPrepared(
        const Path& container, const PreparedInnerFitOperand& part,
        const GeometryContext& context, const NfpOptions& options)
    {
        ScopedGeometryTotal totalTimer(context.Diagnostics());
        {
            ScopedGeometryStage validationStage(
                context.Diagnostics(), GeometryDiagnosticStage::ValidationExclusive);
            if (part.part == nullptr || !part.part->Valid())
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
            if (!container.IsStructurallyValid())
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
            if (container.HasOpenContours())
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidTopology);
            if (!ValidInnerFitOptions(options))
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
            if (!part.part->CompatibleWith(part.part->RotationRadians(), options))
                return GeometryResult<NfpResult>::Failure(GeometryStatus::InvalidInput);
        }

        const DecompositionOptions decomposition = ToDecomposition(options);

        auto region = Minkowski::ErodePrepared(
            container,
            Minkowski::PreparedErosionOperand(part.part->element_, part.provenance),
            context, decomposition);

        // The clearance grow ran at prepare time, so ErodePrepared never saw its stages.
        // Declare them here, once, so the prepared route reports the same total the
        // one-shot route did when the grow happened inline.
        // Unconditional: merging a default ErrorBudget is an exact no-op, and guarding on
        // Total() > 0 would drop an `exact == false` that carried no displacement.
        if (region.HasValue())
        {
            region.Value().budget.MergeSequential(part.part->carriedBudget_);
            region.Budget() = region.Value().budget;
        }

        // IFP erosion may take several internal Minkowski routes. Piece counts are not
        // universal semantic metadata and are deliberately not recomputed afterwards.
        //
        // `exact` is true because an erosion has no candidate-only stage: unlike reduced
        // convolution, the region it produces IS the feasible set, never a superset to be
        // filtered. Whether the ANSWER is exact is therefore entirely a question of what
        // the pipeline quantised, and Finish already resolves that as
        // `exact && budget.exact`. Passing false here instead hard-coded every inner fit
        // to BoundedApproximation, which understated the rectangular-container closed
        // form - that route runs no lossy stage at all and is genuinely Certified.
        return Finish(std::move(region), 0, 0, true, context);
    }
}
