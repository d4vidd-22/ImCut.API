#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Math/Vec2.hpp"
#include "ConvexDecomposition.hpp"

#include <span>
#include <vector>

namespace ImCut::Geometry
{
    // Minkowski sum and difference as generic primitives.
    //
    // These are the mathematical building blocks a no-fit polygon needs, provided here
    // so ImCut::Nesting can be built on top of them later. No nesting logic, no NFP
    // cache and no placement heuristics live in this kernel.
    namespace Minkowski
    {
        struct SumRegionsDiagnostics
        {
            std::size_t piecesA = 0;
            std::size_t piecesB = 0;
            std::size_t convexPairCount = 0;
        };

        // SINGLE-RING PRIMITIVES.
        //
        // Both operands must be a Path holding exactly one CLOSED contour. Anything else
        // returns InvalidTopology.
        //
        // This is a deliberate contract, not a limitation left unimplemented. The
        // previous behaviour took contours.front() and dropped the rest in silence, so a
        // compound pattern produced a result for a shape the caller never asked about.
        // Multi-region Minkowski is a genuinely different operation - holes do not
        // distribute over the sum - and the nesting work these primitives exist for
        // decomposes concave parts into convex rings before calling them anyway. An
        // explicit refusal is honest; a silent partial answer is not.
        //
        // Use SumRings/DifferenceRings for ring-level work, or decompose the region and
        // combine the results yourself.

        // A (+) B, the set of all a + b. Growing a container by a tool shape.
        [[nodiscard]] GeometryResult<Path> Sum(const Path& pattern, const Path& subject,
                                               const GeometryContext& context);

        // A (-) B, computed as A (+) (-B). For two convex parts this is the boundary a
        // reference point may not cross, which is what makes it the NFP primitive.
        [[nodiscard]] GeometryResult<Path> Difference(const Path& pattern, const Path& subject,
                                                      const GeometryContext& context);

        // Ring-level forms. `closed` distinguishes sweeping a shape along a closed
        // outline from sweeping it along an open polyline.
        [[nodiscard]] GeometryResult<Path> SumRings(const std::vector<Vec2>& pattern,
                                                    const std::vector<Vec2>& subject,
                                                    bool closed, const GeometryContext& context);

        [[nodiscard]] GeometryResult<Path> DifferenceRings(const std::vector<Vec2>& pattern,
                                                           const std::vector<Vec2>& subject,
                                                           bool closed, const GeometryContext& context);

        // REGION PRIMITIVES.
        //
        // The single-ring forms above take one closed contour each. These take whole
        // filled REGIONS - outer rings, holes, several components - which is what a
        // no-fit polygon actually operates on.
        //
        // Both work by decomposing each region into convex pieces and merging edges
        // exactly, because Minkowski distributes over union:
        //
        //     (union_i A_i) (+) (union_j B_j) == union_ij (A_i (+) B_j)
        //
        // `options` controls that decomposition. Its simplifyTolerance and maxPieces are
        // the knobs that make the cost predictable, and whatever they cost is added to
        // the result's ErrorBudget rather than absorbed silently.

        [[nodiscard]] GeometryResult<Path> SumRegions(const Path& a, const Path& b,
                                                      const GeometryContext& context,
                                                      const DecompositionOptions& options = {},
                                                      SumRegionsDiagnostics* diagnostics = nullptr);

        // Same exact pairwise union once context-compatible convex pieces are already
        // prepared. This is the WP4 reuse boundary: no Path flatten/decomposition is
        // repeated per pair.
        [[nodiscard]] GeometryResult<Path> SumPreparedPieces(
            const std::vector<std::vector<Vec2>>& piecesA,
            const std::vector<std::vector<Vec2>>& piecesB,
            const ErrorBudget& budgetA,
            const ErrorBudget& budgetB,
            const GeometryContext& context,
            SumRegionsDiagnostics* diagnostics = nullptr);

        // A (-) B in the EROSION sense: { x : x + B is entirely inside A }.
        //
        // Not the same operation as Difference above, which is a dilation by -B and is
        // what "Minkowski difference" means in the Clipper sense. The audit measured the
        // gap: a 60x60 square differenced by a 10x10 gives area 4900, while its erosion
        // is 50x50 = 2500. Offsetting by a negative distance is not a substitute either -
        // that erodes by a DISC, and for a 10x30 rectangular part in a 100x100 container
        // it reported 4675.4 against the true 6300, an error of -1625 mm2.
        //
        // This is the primitive an inner-fit polygon is built from, so it is worth being
        // exact about: the feasible region for a part's reference point inside a
        // container IS the erosion of the container by the part.
        //
        // Computed as the complement of the dilation of the complement, evaluated inside
        // a window large enough that the window's own boundary cannot intrude:
        //
        //     A (-) B  ==  W \ ((W \ A) (+) (-B))     for W big enough
        //
        // The result may be empty - the part does not fit - and may have several
        // components and holes. Both are legitimate answers, not failures.
        [[nodiscard]] GeometryResult<Path> Erode(const Path& region, const Path& structuringElement,
                                                 const GeometryContext& context,
                                                 const DecompositionOptions& options = {});

        struct PreparedErosionOperand;

        // The structuring element of an erosion, prepared once.
        //
        // Erosion normalises B to its own anchor, reflects it through the origin and
        // decomposes it into convex pieces. All three depend ONLY on B and the
        // decomposition options - never on the region being eroded - so a caller that
        // erodes many regions by the same element (a nesting run asking where one part
        // fits as a sheet fills up) should pay for them once, not once per query.
        //
        // Measured on the real corpus: the element decomposition is 22-35% of a whole
        // inner fit. On synthetic parts it is 3-10%, which is why this is worth doing
        // for artwork and not worth pretending about for toy shapes.
        //
        // PROVENANCE-FREE BY CONTRACT.
        //
        // Budget() is the INTRINSIC preparation cost only - the flatten and quantization
        // this decomposition performed. The source Path's own budget is deliberately NOT
        // stored here, for the same reason PreparedNfpOperand::BuildTransient excludes it:
        // a derivative that embeds one caller's provenance cannot be shared with another
        // caller without lying about that caller's error. Keeping it out is what makes
        // this object safe to cache or share later.
        //
        // Caller provenance travels beside the element in PreparedErosionOperand and is
        // composed exactly once, on the way out of ErodePrepared.
        class PreparedErosionElement
        {
        public:
            [[nodiscard]] bool Valid() const noexcept { return valid_; }
            [[nodiscard]] const Bounds2& ElementBounds() const noexcept
            {
                return elementBounds_;
            }

            // Intrinsic preparation cost. Never the source geometry's provenance.
            [[nodiscard]] const ErrorBudget& Budget() const noexcept { return budget_; }
            [[nodiscard]] std::size_t PieceCount() const noexcept
            {
                return reflectedPieces_.size();
            }
            [[nodiscard]] std::size_t ApproximateBytes() const noexcept;

        private:
            friend GeometryResult<PreparedErosionElement> PrepareErosionElement(
                const Path&, const GeometryContext&, const DecompositionOptions&);
            friend GeometryResult<Path> ErodePrepared(
                const Path&, const PreparedErosionOperand&, const GeometryContext&,
                const DecompositionOptions&);

            Bounds2 elementBounds_{};
            // Already normalised to the anchor AND reflected through the origin, which
            // is the exact form the dilation consumes.
            std::vector<std::vector<Vec2>> reflectedPieces_{};
            ErrorBudget budget_{};
            bool valid_ = false;
        };

        // Query-bound view of an intrinsic prepared element.
        //
        // Same shape and same reason as NfpPreparedQueryOperand: the derivative stays
        // provenance-free and reusable, while the error the CALLER's geometry arrived
        // with rides along per query and is composed exactly once at the boundary.
        //
        // The element must outlive the operand; this is a non-owning view.
        struct PreparedErosionOperand
        {
            const PreparedErosionElement* element = nullptr;
            ErrorBudget provenance{};

            PreparedErosionOperand() = default;
            explicit PreparedErosionOperand(const PreparedErosionElement& value)
                : element(&value) {}
            PreparedErosionOperand(const PreparedErosionElement& value,
                                   ErrorBudget sourceProvenance)
                : element(&value), provenance(sourceProvenance) {}
        };

        [[nodiscard]] GeometryResult<PreparedErosionElement> PrepareErosionElement(
            const Path& structuringElement,
            const GeometryContext& context,
            const DecompositionOptions& options = {});

        // Same erosion, same result, with the element's share of the work already done.
        //
        // The declared total is identical to the one-shot Erode: the intrinsic stages are
        // the same stages, and `element.provenance` restores exactly what Erode would have
        // picked up from the structuring element's own Path::budget.
        [[nodiscard]] GeometryResult<Path> ErodePrepared(
            const Path& region,
            const PreparedErosionOperand& element,
            const GeometryContext& context,
            const DecompositionOptions& options = {});

        // Exact convex-convex Minkowski sum by edge merging.
        //
        // Both hulls' edges are already angularly sorted, so merging them in angle order
        // produces the sum directly in O(n + m) with no clipping and no lattice. This is
        // the path the future nesting engine will lean on, since it decomposes concave
        // parts into convex pieces first.
        [[nodiscard]] std::vector<Vec2> ConvexSum(const std::vector<Vec2>& a,
                                                  const std::vector<Vec2>& b);

        // Scratch-aware form used by materialized NFP pipelines. Both vectors retain
        // their capacity between calls; output receives the cleaned sum and scratch is
        // an implementation workspace. They must be distinct objects.
        void ConvexSumInto(const std::vector<Vec2>& a,
                           const std::vector<Vec2>& b,
                           std::vector<Vec2>& output,
                           std::vector<Vec2>& scratch);

        // Prepared derivatives store piece vertices contiguously. This overload keeps
        // the same exact angular merge while consuming non-owning piece slices.
        void ConvexSumInto(std::span<const Vec2> a,
                           std::span<const Vec2> b,
                           std::vector<Vec2>& output,
                           std::vector<Vec2>& scratch);

        // Inclusive membership in (a (+) b), optionally grown by the axis-aligned
        // clearance square. The merged boundary is streamed directly: no result vector
        // or hidden scratch allocation is created. This is the Product-cover leaf
        // predicate used by concurrent Contains calls.
        [[nodiscard]] bool ConvexSumContains(
            std::span<const Vec2> a,
            std::span<const Vec2> b,
            Vec2 point,
            double boundaryTolerance,
            double clearance = 0.0) noexcept;
    }
}
