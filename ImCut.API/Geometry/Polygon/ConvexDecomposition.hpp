#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Math/Vec2.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace ImCut::Geometry
{
    // Geometry is documented only inside the +/-1e9 mm production envelope. A larger
    // Douglas-Peucker distance has no additional meaning there and risks overflow when
    // squared or repeatedly doubled by the piece-budget escalation.
    inline constexpr double kMaxSupportedSimplifyTolerance = 1.0e9;

    using Triangle = std::array<std::uint32_t, 3>;

    enum class DecompositionBackend : std::uint8_t
    {
        None,
        // O(n^2) ear clipping. Exact input vertices, no holes.
        EarClipping,
        // Sweep-line Delaunay triangulation of a whole filled region. O(n log n),
        // handles holes, works on the quantised lattice.
        SweepDelaunay
    };

    // Order used when deleting triangulation diagonals. Every strategy preserves the
    // same exact partition; only the grouping of triangles into convex faces changes.
    // HertelMehlhorn is the historical/default route. The other routes exist for the
    // explicitly-audited Minkowski portfolio and never participate in Auto implicitly.
    enum class DecompositionMergeStrategy : std::uint8_t
    {
        None,
        HertelMehlhorn,
        LongestSharedEdgeFirst,
        ShortestSharedEdgeFirst,

        // Small-input portfolio: run all three deterministic orders above and retain
        // the partition with the fewest pieces, then the least downstream vertex work.
        // This is deliberately not named "optimal"; it is best-of-three, not an exact
        // minimum-convex-partition solver.
        SmallNBestOfThree
    };

    struct DecompositionOptions
    {
        // Rings at or below this vertex count go through ear clipping, which keeps the
        // input coordinates exactly and beats the sweep backend on small inputs. Above
        // it the sweep backend takes over. Threshold measured - see Performance.md.
        std::size_t earClippingLimit = 256;

        // Off yields the raw triangulation.
        bool mergeToConvex = true;

        // Explicit merge policy. The default preserves the pre-V8 behaviour.
        DecompositionMergeStrategy mergeStrategy =
            DecompositionMergeStrategy::HertelMehlhorn;

        // Strict cutoff for SmallNBestOfThree, expressed in triangles after
        // triangulation. Exceeding it returns ComplexityLimit instead of silently
        // running three portfolios on an unbounded region.
        std::size_t smallNPortfolioTriangleLimit = 80;

        // Douglas-Peucker tolerance applied to each ring BEFORE triangulation, in
        // millimetres. Zero means no simplification, which is the default and keeps
        // every input vertex.
        //
        // Why this exists: the decomposition is correct but essentially triangular -
        // real corpus parts of 700 to 1800 curve segments come out as 3212 to 4660
        // convex pieces, 3.8 to 5.4 vertices each. That is fine for collision, and
        // fatal for Minkowski, which evaluates one exact sum per PAIR of pieces: two
        // 4600-piece parts would ask for 21 million sums.
        //
        // The error is bounded by this value and is added to the result's ErrorBudget
        // as a flatten term, so a caller composing a nesting clearance can see it. It
        // is a declared trade, never a silent one.
        double simplifyTolerance = 0.0;

        // TARGET piece count for the simplification escalation. NOT a resource ceiling,
        // and never a reason to discard a partition. Zero means no target.
        //
        // CLASSIFICATION (section 29): SOFT_POLICY / escalation target. The HARD resource
        // contract on this cost is ComplexityLimits::maxConvexPairs - Minkowski evaluates
        // one exact sum per PAIR of pieces, that is the unit that actually limits, and it
        // is checked with overflow-free arithmetic before any pair is enumerated.
        //
        // WITH `simplifyTolerance` > 0 this is a GUARANTEE: the tolerance is doubled and
        // the decomposition retried until the piece count fits the target or
        // `maxSimplifyAttempts` is spent. Whatever tolerance actually produced the answer
        // is the one declared in the ErrorBudget, so the caller learns the price rather
        // than the request. Only a caller who set BOTH knobs - a target and permission to
        // approximate - can be told ComplexityLimit here, and only when the escalation
        // could not reach the target it asked for.
        //
        // WITH `simplifyTolerance` == 0 there is nothing to escalate, so the target has
        // no effect and the exact partition is returned whatever its cardinality. It used
        // to be refused instead, AFTER the triangulation and merge had already been paid
        // for: a 96-arm star decomposes into 98 exact convex pieces and the shipping
        // default of 64 turned that into ComplexityLimit while saving no work at all.
        // That was F24 - a cost proxy wearing the type of a hard contract - and the
        // judgement now lives in exactly one place, Escalate() in the .cpp.
        std::size_t maxPieces = 0;

        // How many doublings the escalation above may spend. Eight is a 256x range,
        // which covers everything measured on the reference corpus with room over.
        int maxSimplifyAttempts = 8;
    };

    struct ConvexDecompositionResult
    {
        // Vertices the triangles index into. For the ear-clipping backend these are the
        // caller's ring vertices unchanged; for the sweep backend they are lattice
        // points, and `budget` carries the displacement.
        std::vector<Vec2> vertices;

        // Convex pieces, each a counter-clockwise ring.
        std::vector<std::vector<Vec2>> pieces;

        // Triangulation the decomposition was merged from.
        std::vector<Triangle> triangles;

        ErrorBudget budget;
        DecompositionBackend backend = DecompositionBackend::None;
        DecompositionMergeStrategy appliedMergeStrategy =
            DecompositionMergeStrategy::None;

        [[nodiscard]] std::size_t PieceCount() const noexcept { return pieces.size(); }
    };

    namespace Decompose
    {
        // Ear clipping. Input must be a simple ring (no self-intersections, no holes);
        // orientation is handled internally. Returns n-2 triangles as indices into the
        // input ring.
        //
        // O(n^2): each ear search may scan the remaining vertices. Guarded by
        // ComplexityLimits::maxPolygonVertices. Prefer Convex()/ConvexRegion(), which
        // switch to the scalable backend once the ring is large.
        [[nodiscard]] GeometryResult<std::vector<Triangle>> Triangulate(
            const std::vector<Vec2>& ring, const GeometryContext& context);

        // Triangulation followed by Hertel-Mehlhorn diagonal removal.
        //
        // Triangulation alone yields n-2 pieces, which is far more fragmentation than
        // downstream Minkowski and collision work wants. Hertel-Mehlhorn deletes every
        // diagonal whose removal leaves both sides convex, and is guaranteed to stay
        // within four times the optimal piece count.
        [[nodiscard]] GeometryResult<ConvexDecompositionResult> Convex(
            const std::vector<Vec2>& ring, const GeometryContext& context,
            const DecompositionOptions& options = DecompositionOptions{});

        // Whole filled region: outer contours and holes together, curves flattened at
        // the context tolerance. The region is normalised through the Boolean backend
        // first, so overlapping or badly wound input is resolved rather than rejected.
        //
        // Open contours are rejected (InvalidTopology) - a decomposition of a region
        // needs the region to be a region.
        [[nodiscard]] GeometryResult<ConvexDecompositionResult> ConvexRegion(
            const Path& path, const GeometryContext& context,
            const DecompositionOptions& options = DecompositionOptions{});
    }
}
