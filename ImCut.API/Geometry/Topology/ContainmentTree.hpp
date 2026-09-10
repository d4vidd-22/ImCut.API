#pragma once

#include "../Curves/Flatten.hpp"
#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Math/Bounds2.hpp"

#include <cstdint>
#include <vector>

namespace ImCut::Geometry
{
    // One node of the nesting hierarchy. Depth is unbounded: outer, hole, island
    // inside that hole, hole inside that island, and so on.
    struct ContainmentNode
    {
        // Index of the contour in the source path.
        std::uint32_t contour = 0;

        // Index of the enclosing node, or kNoParent for a top-level ring.
        std::uint32_t parent = kNoParent;

        std::vector<std::uint32_t> children;

        // Nesting depth: 0 for a top-level ring, +1 per enclosing ring.
        int depth = 0;

        // Whether the region immediately inside this ring - and outside its children -
        // is filled. How it is derived depends on the path's fill rule:
        //
        //   EvenOdd  parity of `depth`. Winding direction is irrelevant.
        //   NonZero  sign of `winding`, which is the turn count accumulated down the
        //            parent chain. A ring wound the same way as its parent ADDS to the
        //            count and stays filled; only an opposing winding can cancel it.
        //
        // The rule is not a detail of one path: reading every path as EvenOdd reported
        // 6000 mm^2 for five concentric CCW rings that are a solid 10000 mm^2 region.
        ContourRole role = ContourRole::Outer;

        // Turn count at a point just inside this ring, summed over the parent chain
        // including this ring. Only meaningful under NonZero; zero-filled otherwise.
        int winding = 0;

        Bounds2 bounds;
        double signedArea = 0.0;

        static constexpr std::uint32_t kNoParent = 0xFFFFFFFFu;
    };

    struct ContainmentTree
    {
        std::vector<ContainmentNode> nodes;
        std::vector<std::uint32_t> roots;

        // The rule the roles above were derived under, recorded so that a tree built
        // for one rule cannot silently answer a question asked under the other.
        FillRule fillRule = FillRule::EvenOdd;

        // Flattened rings, index-aligned with `nodes`. Kept because containment needs
        // them and every downstream consumer would otherwise flatten again.
        std::vector<FlattenedContour> rings;

        // A point strictly inside each ring, index-aligned with `nodes`. Empty entries
        // are marked by the parallel flag below - a degenerate ring has no interior.
        //
        // Computed once here because the prepared collision queries need it on a path
        // that must not allocate or search: deciding whether one placed part sits inside
        // another comes down to classifying one point, and finding that point by walking
        // the ring on every query would put an O(n) search in the hot path.
        std::vector<Vec2> interiorPoints;
        std::vector<bool> hasInteriorPoint;

        // A point inside the FILLED region - inside an outer ring and not inside any of
        // its holes. Distinct from interiorPoints[root]: for a donut, the outer ring's
        // interior point can land in the hole.
        Vec2 representativePoint{};
        bool hasRepresentativePoint = false;

        // Set when two contours are geometrically indistinguishable, so their nesting
        // order is arbitrary. The tree is still usable; the caller decides whether
        // duplicate rings matter for its purpose.
        bool hasCoincidentContours = false;

        [[nodiscard]] std::size_t Size() const noexcept { return nodes.size(); }
        [[nodiscard]] int MaxDepth() const noexcept;
    };

    namespace Topology
    {
        // Builds the nesting hierarchy of a path's contours.
        //
        // Order of reasoning is deliberate and matters:
        //     geometry -> containment -> fill rule -> role -> winding normalisation
        // Containment comes from what actually encloses what, never from the direction
        // the rings were drawn in - that would break on any artwork whose rings were not
        // authored consistently, which is most of it. The ROLE then follows from the
        // path's declared fill rule applied to that containment: parity under EvenOdd,
        // accumulated winding under NonZero. `path.fillRule` is recorded in the result.
        //
        // Parent search is bounds-filtered through a hierarchy over contour bounds, so
        // the cost tracks genuinely overlapping pairs instead of the square of the
        // contour count.
        [[nodiscard]] GeometryResult<ContainmentTree> BuildContainmentTree(
            const Path& path, const GeometryContext& context);

        // Rewrites each contour's direction to match the role the tree assigned.
        // Returns how many contours were reversed.
        [[nodiscard]] std::size_t NormalizeWinding(Path& path, const ContainmentTree& tree,
                                                   const GeometryContext& context);

        // Classifies a point against the whole hierarchy: inside the filled region,
        // outside it, or on a boundary. Nesting parity decides filled versus hole.
        [[nodiscard]] PointClassification ClassifyPoint(const ContainmentTree& tree, Vec2 point,
                                                        const GeometryContext& context);
    }
}
