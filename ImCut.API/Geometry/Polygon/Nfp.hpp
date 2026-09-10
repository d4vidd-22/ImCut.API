#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryCertification.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../PreparedGeometry.hpp"
#include "ConvexDecomposition.hpp"
#include "Minkowski.hpp"

#include <memory>

namespace ImCut::Geometry
{
    // No-fit and inner-fit polygons.
    //
    // These are the two questions a true-shape nesting engine asks about a pair of
    // parts, expressed once, here, so that every consumer gets the same answer and the
    // same declared error:
    //
    //   Nfp::Compute(stationary, moving, ...)
    //       Where may `moving`'s reference point NOT go, if the two are not to overlap?
    //       That region is stationary (+) (-moving). A reference point strictly OUTSIDE
    //       it is a legal placement; strictly inside it overlaps.
    //
    //   Nfp::InnerFit(container, part, ...)
    //       Where MAY `part`'s reference point go, if the part is to stay wholly inside
    //       the container? That region is the EROSION of the container by the part, not
    //       a dilation and not a negative offset. It can be empty - the part does not
    //       fit - and it can have several components and holes.
    //
    // Both are built from Minkowski::SumRegions and Minkowski::Erode, which decompose
    // into convex pieces and merge edges exactly. Neither goes near the polygon
    // backend's Minkowski, which carried the star-shaped precondition that made a
    // concave pattern punch a hole that does not exist.
    namespace Nfp
    {
        enum class NfpRoutingPolicy : std::uint8_t
        {
            Auto,
            Forced
        };

        // Cover representation is an execution policy. Both modes represent the same
        // forbidden set and therefore this value must never enter a semantic cache key.
        enum class NfpCoverMode : std::uint8_t
        {
            Auto,
            Explicit,
            Product
        };

        struct NfpOptions
        {
            // Douglas-Peucker tolerance applied to both operands before decomposition,
            // in millimetres. Zero keeps every vertex.
            //
            // This is the knob that makes the cost predictable. Real parts decompose
            // into thousands of convex pieces and the sum evaluates one exact convex sum
            // per PAIR, so two 4600-piece parts would ask for 21 million of them.
            // Whatever this costs is added to the result's budget.
            double simplifyTolerance = 0.0;

            // TARGET convex-piece count per operand for the simplification escalation.
            // Not a resource ceiling: see DecompositionOptions::maxPieces, whose
            // classification this inherits, and ComplexityLimits::maxConvexPairs, which
            // is the hard contract on the pair cost this number was standing in for.
            //
            // With simplifyTolerance set as well, the tolerance is escalated until the
            // target is met and the tolerance that actually did it is what the budget
            // declares. With simplifyTolerance left at zero - the default - there is
            // nothing to escalate, so an operand that decomposes into more pieces than
            // this is computed exactly rather than refused. Under V8.1.2 it was refused,
            // which made `Nfp::NfpOptions{}` - the options a caller gets by writing
            // nothing at all - return ComplexityLimit for ordinary artwork (F24b).
            std::size_t maxPieces = 64;

            // Auto is the production default. For source compatibility, assigning a
            // non-HM merge strategy also counts as an explicit forced request; forcing
            // HM itself requires routingPolicy=Forced.
            NfpRoutingPolicy routingPolicy = NfpRoutingPolicy::Auto;

            // Part of the resolved preparation/cache identity.
            DecompositionMergeStrategy decompositionMergeStrategy =
                DecompositionMergeStrategy::HertelMehlhorn;

            // Pair-specific exact proof. A hole is filled only when the other connected
            // operand provably cannot fit inside it.
            bool allowProvenHoleFilter = true;

            // Auto retains the existing explicit fast lane while using the implicit
            // Cartesian-product representation when pair count or retained bytes make
            // explicit storage unsuitable. Forced modes exist for differential tests,
            // reproducible diagnostics and callers with a measured workload policy.
            NfpCoverMode coverMode = NfpCoverMode::Auto;

            // Exact axis-aligned L-infinity clearance grown around the no-fit region, in
            // millimetres. The boundary is inclusive: distance_infinity == clearance is
            // forbidden, preserving the kernel's no-touch convention.
            //
            // Applied by summing with a square of side 2*clearance centred on the
            // origin. This is deliberately not Euclidean distance; see
            // NFP_CLEARANCE_CONTRACT.md.
            double clearance = 0.0;
        };

        struct NfpResult IMCUT_GEOMETRY_COUNTED(NfpResultTag)
        {
            // May have several components and legitimate holes.
            Path region;

            // Simplification, flattening and quantisation, SUMMED. A nesting clearance
            // composed from this must not be smaller than it.
            ErrorBudget budget;

            GeometryCertification certification =
                GeometryCertification::BoundedApproximation;

            std::size_t piecesA = 0;
            std::size_t piecesB = 0;

            // True when both operands were convex and no simplification was applied, so
            // the region is the exact Minkowski sum rather than an approximation of it.
            bool exact = false;
        };

        // Shared NFP geometry paired with the CALLER's metadata, without either being
        // copied into the other.
        //
        // Deliberately NOT convertible to Path: an implicit conversion would put a
        // `const Path&` that looks provenance-complete back within reach, which is the
        // whole of F22. Geometry() is spelled out at the call site, and so is Budget().
        class NfpRegionView
        {
        public:
            NfpRegionView(const Path& geometry, const ErrorBudget& budget, bool exact,
                          GeometryCertification certification) noexcept
                : geometry_(&geometry), budget_(&budget), exact_(exact),
                  certification_(certification) {}

            // The shared payload. Zero-copy; the same object IntrinsicRegion() returns.
            [[nodiscard]] const Path& Geometry() const noexcept { return *geometry_; }

            // The CALLER's: intrinsic composed with the provenance of both operands, the
            // same number the handle reports.
            [[nodiscard]] const ErrorBudget& Budget() const noexcept { return *budget_; }
            [[nodiscard]] bool Exact() const noexcept { return exact_; }
            [[nodiscard]] GeometryCertification Certification() const noexcept
            {
                return certification_;
            }

            // A Path that IS provenance-complete, at the cost of a deep copy the caller
            // asked for by name.
            [[nodiscard]] Path CopyGeometry() const
            {
                Path copy = *geometry_;
                copy.budget = *budget_;
                return copy;
            }

        private:
            const Path* geometry_;
            const ErrorBudget* budget_;
            bool exact_;
            GeometryCertification certification_;
        };

        // Immutable, owning view of a cached intrinsic NFP payload. Geometry storage
        // is shared with the cache while caller provenance stays on the handle, so a
        // warm read neither mutates shared state nor deep-copies a Path. CopyValue()
        // is the explicit bridge to the compatibility value API.
        class NfpResultHandle
        {
        public:
            NfpResultHandle() = default;

            // `queryExact` and `queryCertification` are the CALLER-VISIBLE metadata,
            // composed from the intrinsic payload and the caller's provenance. They are
            // passed in rather than read from the payload because the payload is shared
            // and immutable: two callers with different source quality hold handles to the
            // same NfpResult, and neither may see the other's answer.
            //
            // V8.1 read them straight from the payload, so a caller with lossy input got
            // Budget().exact == false and Exact() == true from the same object.
            NfpResultHandle(
                std::shared_ptr<const NfpResult> payload,
                const ErrorBudget& queryBudget,
                bool queryExact,
                GeometryCertification queryCertification) noexcept
                : payload_(std::move(payload)), budget_(queryBudget),
                  exact_(queryExact), certification_(queryCertification) {}

            [[nodiscard]] bool Valid() const noexcept { return payload_ != nullptr; }

            // The shared payload, NAMED. Its `budget` is the INTRINSIC error of
            // representing this NFP and carries nothing of the caller: the payload is
            // shared and immutable, and two callers with different source quality hold
            // handles to the same object, so writing either one's provenance into it
            // would let each see the other's error (F06).
            //
            // Use for pure geometry - contour count, bounds, topology, area. Never for a
            // decision about the caller's own geometry; RegionView() carries that.
            [[nodiscard]] const Path& IntrinsicRegion() const { return Payload().region; }

            // DEPRECATED: returns the INTRINSIC payload, whose budget is not the caller's.
            //
            // V8.1.1 shipped this as the zero-copy accessor, and a handle reporting
            // Budget().exact == false handed out a Path reporting budget.exact == true
            // from the same object (evidence 1016). The object was right; the name was
            // silent about which of the two budgets it belonged to.
            //
            // Kept because it is public ABI. Deprecated so the compiler names every
            // remaining use, which is the only way to know the migration finished rather
            // than to assume it. See NFP_REGION_VIEW_CONTRACT.md.
            [[deprecated("Region() returns the INTRINSIC payload: its budget is not the "
                         "caller's. Use RegionView() for geometry plus caller metadata, "
                         "IntrinsicRegion() if the intrinsic payload is what you want, or "
                         "CopyValue() for a provenance-complete Path.")]]
            [[nodiscard]] const Path& Region() const { return Payload().region; }

            [[nodiscard]] const ErrorBudget& Budget() const noexcept { return budget_; }
            [[nodiscard]] GeometryCertification Certification() const
            {
                // Caller-visible, not the payload's. See the constructor. Payload() is
                // called for its VALIDITY CHECK - it throws on a dead handle - and the
                // reference it returns is deliberately dropped. Cast to void, because
                // silently discarding a [[nodiscard]] emitted C4834 into every one of
                // the 136 translation units that include this header, under a /W4
                // contract whose whole value is that a NEW warning is visible.
                static_cast<void>(Payload());
                return certification_;
            }
            [[nodiscard]] std::size_t PiecesA() const { return Payload().piecesA; }
            [[nodiscard]] std::size_t PiecesB() const { return Payload().piecesB; }
            [[nodiscard]] bool Exact() const { static_cast<void>(Payload()); return exact_; }

            // Shared geometry, caller metadata, no copy and no confusion.
            [[nodiscard]] NfpRegionView RegionView() const
            {
                return NfpRegionView(Payload().region, budget_, exact_, certification_);
            }

            [[nodiscard]] NfpResult CopyValue() const
            {
                NfpResult copy = Payload();
                copy.budget = budget_;
                copy.region.budget = budget_;
                // The copy has to agree with the handle it came from, metadata included.
                copy.exact = exact_;
                copy.certification = certification_;
                return copy;
            }

        private:
            [[nodiscard]] const NfpResult& Payload() const
            {
                if (!payload_)
                {
                    OnResultInvariantViolation(
                        GeometryStatus::InvalidInput,
                        "NfpResultHandle payload is null");
                }
                return *payload_;
            }

            std::shared_ptr<const NfpResult> payload_{};
            ErrorBudget budget_{};
            bool exact_ = false;
            GeometryCertification certification_ =
                GeometryCertification::BoundedApproximation;
        };

        // NFP(A, B) = A (+) (-B), with B rotated by `rotationRadians` first.
        //
        // On reflection: the definition uses -B, and reflecting a ring reverses its
        // traversal direction. Callers do not have to care - both operands are
        // direction-normalised on the way in, and the result is emitted with the
        // kernel's outer-counter-clockwise convention.
        //
        // Both definitions must be prepared at least to PreparationLevel::BooleanReady;
        // the local path is what is operated on.
        // Both questions against raw geometry.
        //
        // There is deliberately NO overload taking PreparedShapeDefinition. Two used to
        // exist and both simply forwarded `definition.LocalPath()` into these functions,
        // so they looked like prepared routes and re-did every scrap of the work. A
        // caller that wants preparation to actually pay for itself uses
        // PrepareInnerFitPart below, or GeometrySession for the cached NFP. A caller that
        // wants the raw route now has to write `.LocalPath()` and can see that it is raw.
        [[nodiscard]] GeometryResult<NfpResult> Compute(const Path& stationary, const Path& moving,
                                                        double rotationRadians,
                                                        const GeometryContext& context,
                                                        const NfpOptions& options = {});

        // IFP: the region a part's reference point may occupy inside a container.
        //
        // Erosion, not dilation. An empty result means the part does not fit, which is
        // an answer and not a failure - GeometryStatus::Empty, and IsSuccess is true
        // for it.
        [[nodiscard]] GeometryResult<NfpResult> InnerFit(const Path& container, const Path& part,
                                                         double rotationRadians,
                                                         const GeometryContext& context,
                                                         const NfpOptions& options = {});

        struct PreparedInnerFitOperand;

        // A part, prepared once for inner fit against many containers.
        //
        // Everything an inner fit does that depends only on the PART lives here: the
        // rotation, the clearance grow, the reflection through the origin, the
        // normalisation to the part's anchor, and the convex decomposition of the result.
        // Only the container's complement is decomposed per query, because only that
        // depends on the container.
        //
        // This is why it is worth having, measured rather than assumed: on the real
        // corpus (BETA, CRF 250R, CRF 300F, KTM) the element decomposition is 22-35% of a
        // whole inner fit. On synthetic shapes it is 3-10%, so the win is an artwork win.
        //
        // PreparedShapeDefinition::Decomposition() is NOT a substitute. That is the
        // decomposition of the definition's LOCAL path - not of the part rotated by this
        // query's angle, grown by this query's clearance, reflected, and normalised to
        // its own anchor. Those are different polygons with different convex pieces.
        //
        // Provenance-free, like every other shared derivative in the kernel: Budget() is
        // intrinsic preparation cost only, and the caller's own error rides in
        // PreparedInnerFitOperand.
        class PreparedInnerFitPart
        {
        public:
            [[nodiscard]] bool Valid() const noexcept { return valid_; }
            [[nodiscard]] double RotationRadians() const noexcept { return rotationRadians_; }

            // Everything this preparation cost, and nothing the caller brought with them.
            [[nodiscard]] const ErrorBudget& Budget() const noexcept { return intrinsicBudget_; }
            [[nodiscard]] std::size_t PieceCount() const noexcept
            {
                return element_.PieceCount();
            }
            [[nodiscard]] std::size_t ApproximateBytes() const noexcept;

            // A prepared part answers only for the query parameters it was built from.
            // Anything else is a different polygon, so it is refused rather than silently
            // answered with the wrong element.
            [[nodiscard]] bool CompatibleWith(double rotationRadians,
                                              const NfpOptions& options) const noexcept;

        private:
            friend GeometryResult<PreparedInnerFitPart> PrepareInnerFitPart(
                const Path&, double, const GeometryContext&, const NfpOptions&);
            friend GeometryResult<NfpResult> InnerFitPrepared(
                const Path&, const PreparedInnerFitOperand&, const GeometryContext&,
                const NfpOptions&);

            Minkowski::PreparedErosionElement element_{};

            // Total intrinsic cost of this preparation: the clearance grow's stages plus
            // the element's decomposition.
            ErrorBudget intrinsicBudget_{};

            // The subset of intrinsicBudget_ that ErodePrepared cannot observe.
            //
            // The element contributes its own decomposition to the erosion from the
            // inside, but the clearance grow ran here, at prepare time, against a
            // provenance-stripped copy of the part. Those stages are real and have to be
            // declared, so InnerFitPrepared re-composes them at the boundary. Zero
            // whenever clearance is zero, which is the common case.
            ErrorBudget carriedBudget_{};

            double rotationRadians_ = 0.0;
            double clearance_ = 0.0;
            double simplifyTolerance_ = 0.0;
            std::size_t maxPieces_ = 0;
            DecompositionMergeStrategy mergeStrategy_ =
                DecompositionMergeStrategy::HertelMehlhorn;
            bool valid_ = false;
        };

        // Query-bound view of a prepared part. Non-owning: the part must outlive it.
        struct PreparedInnerFitOperand
        {
            const PreparedInnerFitPart* part = nullptr;
            ErrorBudget provenance{};

            PreparedInnerFitOperand() = default;
            explicit PreparedInnerFitOperand(const PreparedInnerFitPart& value)
                : part(&value) {}
            PreparedInnerFitOperand(const PreparedInnerFitPart& value,
                                    ErrorBudget sourceProvenance)
                : part(&value), provenance(sourceProvenance) {}
        };

        [[nodiscard]] GeometryResult<PreparedInnerFitPart> PrepareInnerFitPart(
            const Path& part, double rotationRadians, const GeometryContext& context,
            const NfpOptions& options = {});

        // Same inner fit, same result, with the part's share of the work already done.
        //
        // `options` must match what the part was prepared with (they also drive the
        // container's own decomposition); a mismatch is InvalidInput, never a quiet
        // answer computed from the wrong element.
        [[nodiscard]] GeometryResult<NfpResult> InnerFitPrepared(
            const Path& container, const PreparedInnerFitOperand& part,
            const GeometryContext& context, const NfpOptions& options = {});
    }
}
