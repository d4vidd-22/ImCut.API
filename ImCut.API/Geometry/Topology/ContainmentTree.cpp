#include "ContainmentTree.hpp"

#include "../Math/Predicates.hpp"
#include "../Polygon/PolygonMetrics.hpp"
#include "../Spatial/SegmentBVH.hpp"
#include "Winding.hpp"

#include <algorithm>
#include <cmath>

namespace ImCut::Geometry
{
    int ContainmentTree::MaxDepth() const noexcept
    {
        int deepest = 0;
        for (const ContainmentNode& node : nodes)
            deepest = std::max(deepest, node.depth);
        return deepest;
    }

    namespace Topology
    {
        namespace
        {
            // Decides whether `inner` lies inside `outer`.
            //
            // For valid input the two rings do not cross, so every vertex of `inner`
            // gives the same answer and one test would do. Vertices can still land
            // exactly on `outer`'s boundary though - shared edges and touching rings are
            // common in real artwork - so vertices are tried until one returns a
            // decisive answer. Rings that are boundary-only against each other are
            // coincident, and that is reported rather than guessed.
            [[nodiscard]] bool RingInsideRing(const std::vector<Vec2>& inner,
                                              const std::vector<Vec2>& outer,
                                              double boundaryTolerance,
                                              bool& coincident) noexcept
            {
                coincident = false;

                if (inner.size() < 3 || outer.size() < 3)
                    return false;

                auto probe = [&](Vec2 point, bool& decided) noexcept
                {
                    const PointClassification classification =
                        Predicates::PointInRing(point, outer.data(), outer.size(), boundaryTolerance);
                    decided = classification != PointClassification::Boundary;
                    return classification == PointClassification::Inside;
                };

                // Fast path: the first handful of vertices settles it for almost every
                // real input.
                const std::size_t fastLimit = std::min<std::size_t>(inner.size(), 32);
                for (std::size_t i = 0; i < fastLimit; ++i)
                {
                    bool decided = false;
                    const bool inside = probe(inner[i], decided);
                    if (decided)
                        return inside;
                }

                // Ambiguous so far. Two rings can legitimately share a long stretch of
                // boundary before diverging, and concluding "coincident" from a
                // truncated prefix loses the nesting entirely - so spread the remaining
                // probes across the whole ring instead of walking the next 32 vertices,
                // which would still be the same shared stretch.
                if (inner.size() > fastLimit)
                {
                    constexpr std::size_t kDistributedProbes = 64;
                    const std::size_t stride = std::max<std::size_t>(1, inner.size() / kDistributedProbes);

                    for (std::size_t i = 0; i < inner.size(); i += stride)
                    {
                        bool decided = false;
                        const bool inside = probe(inner[i], decided);
                        if (decided)
                            return inside;
                    }
                }

                // Vertices are all on the boundary. Edge midpoints are not vertices, so
                // they can still separate a genuinely nested ring from a coincident one:
                // where the rings truly coincide every midpoint is on the boundary too,
                // and where they only share a stretch the midpoints off that stretch are
                // decisive.
                for (std::size_t i = 0; i < inner.size(); ++i)
                {
                    const Vec2 midpoint = Lerp(inner[i], inner[(i + 1) % inner.size()], 0.5);
                    bool decided = false;
                    const bool inside = probe(midpoint, decided);
                    if (decided)
                        return inside;
                }

                // Every vertex and every midpoint lies on the other ring: the two really
                // are the same ring.
                coincident = true;
                return false;
            }
        }

        namespace
        {
            // A point strictly inside a ring.
            //
            // The bounds centre settles it for a convex ring; a concave one need not
            // contain its own centre, so the fallback walks the edges and steps inward
            // from each midpoint.
            [[nodiscard]] bool ProbeInterior(const std::vector<Vec2>& ring, double tolerance,
                                             Vec2& out)
            {
                if (ring.size() < 3)
                    return false;

                Bounds2 bounds;
                for (const Vec2& point : ring) bounds.Add(point);

                const Vec2 centre = bounds.Center();
                if (Predicates::PointInRing(centre, ring.data(), ring.size(), tolerance) ==
                    PointClassification::Inside)
                {
                    out = centre;
                    return true;
                }

                // A fraction of the ring's own size, floored by the caller's tolerance.
                constexpr double kInwardStepFraction = 1e-4;
                const double step =
                    std::max(bounds.LongestAxisLength() * kInwardStepFraction, tolerance * 4.0);
                for (std::size_t i = 0; i < ring.size(); ++i)
                {
                    const Vec2 a = ring[i];
                    const Vec2 b = ring[(i + 1) % ring.size()];
                    const Vec2 midpoint = Lerp(a, b, 0.5);
                    const Vec2 inward = Normalized(Perpendicular(b - a));
                    if (inward == Vec2{})
                        continue;

                    for (const double sign : { 1.0, -1.0 })
                    {
                        const Vec2 probe = midpoint + inward * (step * sign);
                        if (Predicates::PointInRing(probe, ring.data(), ring.size(), tolerance) ==
                            PointClassification::Inside)
                        {
                            out = probe;
                            return true;
                        }
                    }
                }

                return false;
            }
        }

        GeometryResult<ContainmentTree> BuildContainmentTree(const Path& path,
                                                             const GeometryContext& context)
        {
            ContainmentTree tree;

            // The fill rule DECIDES the role of every ring in this tree, and an enum class
            // does not constrain the VALUE: a std::uint8_t of 77 stored into a FillRule is
            // well-formed C++, and `path.fillRule == FillRule::NonZero` is simply false
            // for it, so the whole path was silently read as EvenOdd. That is the same
            // defect Contour::HasKnownSegmentKinds was added for, one level up, and
            // Path::HasKnownFillRule already existed to answer it - this entry point just
            // never asked. Boolean and Offset did, through IsStructurallyValid; the
            // topology entry a Doctor will lean on hardest did not.
            if (!path.HasKnownFillRule())
                return GeometryResult<ContainmentTree>::Failure(GeometryStatus::InvalidInput);

            if (path.contours.empty())
                return GeometryResult<ContainmentTree>::Empty(std::move(tree));

            if (path.contours.size() > context.Limits().maxContours)
                return GeometryResult<ContainmentTree>::Failure(GeometryStatus::ComplexityLimit);

            FlattenOptions options;
            options.tolerance = context.Tolerance().flatten;
            options.maxDepth = context.Limits().maxSubdivisionDepth;

            const std::size_t count = path.contours.size();
            tree.nodes.resize(count);
            tree.rings.resize(count);

            std::vector<Bounds2> bounds(count);
            std::size_t remainingPoints = context.Limits().maxFlattenPoints;

            for (std::size_t i = 0; i < count; ++i)
            {
                if (context.ShouldCheckCancellation(i) && context.IsCancelled())
                    return GeometryResult<ContainmentTree>::Failure(GeometryStatus::Cancelled);

                // Each ring gets what is left of the point budget, so a pathological
                // curve is stopped during emission instead of after the whole tree has
                // been materialised.
                GeometryContext scoped = context;
                ComplexityLimits scopedLimits = context.Limits();
                scopedLimits.maxFlattenPoints = remainingPoints;
                scoped.SetLimits(scopedLimits);

                const GeometryStatus status =
                    Flatten::ContourInto(path.contours[i], options, scoped, tree.rings[i]);
                if (!IsSuccess(status))
                    return GeometryResult<ContainmentTree>::Failure(status);

                remainingPoints -= (std::min)(remainingPoints, tree.rings[i].points.size());

                ContainmentNode& node = tree.nodes[i];
                node.contour = static_cast<std::uint32_t>(i);
                node.bounds = Metrics::ComputeBounds(path.contours[i]);
                node.signedArea = Metrics::SignedArea(path.contours[i]);
                bounds[i] = node.bounds;
            }

            // A ring can only be contained by one with strictly larger area, so sorting
            // by descending area means a contour's potential parents are exactly those
            // already visited. Ties break on index to keep the result deterministic.
            std::vector<std::uint32_t> byArea(count);
            for (std::size_t i = 0; i < count; ++i)
                byArea[i] = static_cast<std::uint32_t>(i);

            std::sort(byArea.begin(), byArea.end(), [&](std::uint32_t lhs, std::uint32_t rhs) noexcept
            {
                const double a = std::fabs(tree.nodes[lhs].signedArea);
                const double b = std::fabs(tree.nodes[rhs].signedArea);
                if (a != b) return a > b;
                return lhs < rhs;
            });

            // Bounds hierarchy over the contours themselves, so the parent search is
            // driven by actual overlap rather than by testing every earlier contour.
            SegmentBVH boundsIndex;
            boundsIndex.Build(bounds.data(), bounds.size());

            const double boundaryTolerance = context.Tolerance().duplicate;

            std::vector<std::uint32_t> candidates;

            for (std::size_t rank = 0; rank < byArea.size(); ++rank)
            {
                const std::uint32_t index = byArea[rank];
                ContainmentNode& node = tree.nodes[index];

                if (context.ShouldCheckCancellation(rank) && context.IsCancelled())
                    return GeometryResult<ContainmentTree>::Failure(GeometryStatus::Cancelled);

                // Only rings whose bounds enclose this one can be its parent.
                boundsIndex.QueryBounds(node.bounds, candidates);

                std::uint32_t bestParent = ContainmentNode::kNoParent;
                double bestArea = 0.0;

                for (const std::uint32_t candidate : candidates)
                {
                    if (candidate == index)
                        continue;

                    const ContainmentNode& outer = tree.nodes[candidate];

                    const double outerArea = std::fabs(outer.signedArea);
                    const double innerArea = std::fabs(node.signedArea);

                    // Strictly larger, with the index tie-break mirroring the sort so
                    // equal-area rings cannot both claim to contain each other.
                    if (outerArea < innerArea || (outerArea == innerArea && candidate >= index))
                        continue;

                    // F35a, class sweep: this one is SOUND ON ITS MARGIN and stays as it
                    // is, with the derivation written down rather than assumed.
                    //
                    // `node.bounds` is Metrics::ComputeBounds, which is short of the true
                    // extrema by at most 4 ULP of the coordinate - 4.8e-7 mm at the
                    // documented 1e9 mm ceiling. `boundaryTolerance` is
                    // GeometryTolerance::duplicate, 1e-3 mm by default and 1e-4 at
                    // PrecisionMode::Maximum, so the margin exceeds the worst-case defect
                    // by at least two orders of magnitude everywhere in the supported
                    // envelope. Unlike the intersection rejects, which had a 1e-7 margin
                    // against the same defect, this inequality actually holds.
                    //
                    // And it is a PREFILTER: every candidate it admits still goes through
                    // RingInsideRing below, which decides on the geometry.
                    if (!outer.bounds.Contains(node.bounds.Expanded(-boundaryTolerance)))
                        continue;

                    bool coincident = false;
                    if (!RingInsideRing(tree.rings[index].points, tree.rings[candidate].points,
                                        boundaryTolerance, coincident))
                    {
                        if (coincident)
                            tree.hasCoincidentContours = true;
                        continue;
                    }

                    // The immediate parent is the smallest ring that contains it.
                    if (bestParent == ContainmentNode::kNoParent || outerArea < bestArea)
                    {
                        bestParent = candidate;
                        bestArea = outerArea;
                    }
                }

                node.parent = bestParent;
            }

            // Depth, winding and role follow from the parent chain. Because the sort
            // visited larger rings first, a parent's depth and winding are always
            // already final here, so one pass in `byArea` order suffices and no chain
            // is walked twice.
            const bool nonZero = path.fillRule == FillRule::NonZero;
            tree.fillRule = path.fillRule;

            for (const std::uint32_t index : byArea)
            {
                ContainmentNode& node = tree.nodes[index];

                // A ring of zero signed area contributes no turn. Treating it as +1
                // would make a degenerate sliver flip its parent's fill.
                const int turn = node.signedArea > 0.0 ? 1 : (node.signedArea < 0.0 ? -1 : 0);

                if (node.parent == ContainmentNode::kNoParent)
                {
                    node.depth = 0;
                    node.winding = turn;
                    tree.roots.push_back(index);
                }
                else
                {
                    const ContainmentNode& parent = tree.nodes[node.parent];
                    node.depth = parent.depth + 1;
                    node.winding = parent.winding + turn;
                    tree.nodes[node.parent].children.push_back(index);
                }

                // EvenOdd ignores winding entirely, which is correct for EvenOdd and was
                // wrongly generalised to every path. Under NonZero the region just
                // inside a ring is filled exactly when the accumulated turn count is not
                // zero, so an inner ring wound the same way as its parent stays solid.
                node.role = nonZero
                    ? (node.winding != 0 ? ContourRole::Outer : ContourRole::Hole)
                    : ((node.depth % 2 == 0) ? ContourRole::Outer : ContourRole::Hole);
            }

            // Deterministic ordering of roots and children, independent of the
            // area-sorted visit order.
            std::sort(tree.roots.begin(), tree.roots.end());
            for (ContainmentNode& node : tree.nodes)
                std::sort(node.children.begin(), node.children.end());

            // One interior point per ring, computed once.
            //
            // The prepared collision queries decide "is this part inside that one?" by
            // classifying a single point, and searching for that point per query would
            // put an O(n) walk on a path that runs millions of times.
            tree.interiorPoints.assign(count, Vec2{});
            tree.hasInteriorPoint.assign(count, false);

            for (std::size_t i = 0; i < count; ++i)
            {
                Vec2 probe;
                if (ProbeInterior(tree.rings[i].points, boundaryTolerance, probe))
                {
                    tree.interiorPoints[i] = probe;
                    tree.hasInteriorPoint[i] = true;
                }
            }

            // A point in the FILLED region. An outer ring's own interior point can land
            // in one of its holes - a donut's bounds centre is the classic case - so the
            // candidate is confirmed against the finished tree before it is kept.
            for (const std::uint32_t index : tree.roots)
            {
                if (tree.nodes[index].role != ContourRole::Outer || !tree.hasInteriorPoint[index])
                    continue;

                const Vec2 candidate = tree.interiorPoints[index];
                if (ClassifyPoint(tree, candidate, context) == PointClassification::Inside)
                {
                    tree.representativePoint = candidate;
                    tree.hasRepresentativePoint = true;
                    break;
                }
            }

            // Every root's centre fell in a hole. Any ring's interior point that the
            // tree agrees is filled will do.
            if (!tree.hasRepresentativePoint)
            {
                for (std::size_t i = 0; i < count; ++i)
                {
                    if (!tree.hasInteriorPoint[i])
                        continue;
                    if (ClassifyPoint(tree, tree.interiorPoints[i], context) ==
                        PointClassification::Inside)
                    {
                        tree.representativePoint = tree.interiorPoints[i];
                        tree.hasRepresentativePoint = true;
                        break;
                    }
                }
            }

            return GeometryResult<ContainmentTree>::Success(std::move(tree));
        }

        std::size_t NormalizeWinding(Path& path, const ContainmentTree& tree,
                                     const GeometryContext& context)
        {
            std::size_t reversed = 0;
            for (const ContainmentNode& node : tree.nodes)
            {
                if (node.contour >= path.contours.size())
                    continue;
                if (Winding::Normalize(path.contours[node.contour], node.role, context.Tolerance()))
                    ++reversed;
            }
            return reversed;
        }

        PointClassification ClassifyPoint(const ContainmentTree& tree, Vec2 point,
                                          const GeometryContext& context)
        {
            const double boundaryTolerance = context.Tolerance().duplicate;

            // The innermost ring containing the point decides the answer: its depth
            // parity says whether the point is in filled area or in a hole.
            int deepest = -1;
            bool inside = false;

            for (std::size_t i = 0; i < tree.nodes.size(); ++i)
            {
                const ContainmentNode& node = tree.nodes[i];
                if (!node.bounds.Contains(point, boundaryTolerance))
                    continue;

                const std::vector<Vec2>& ring = tree.rings[i].points;
                const PointClassification classification =
                    Predicates::PointInRing(point, ring.data(), ring.size(), boundaryTolerance);

                if (classification == PointClassification::Boundary)
                    return PointClassification::Boundary;

                if (classification == PointClassification::Inside && node.depth > deepest)
                {
                    deepest = node.depth;
                    inside = node.role == ContourRole::Outer;
                }
            }

            return inside ? PointClassification::Inside : PointClassification::Outside;
        }
    }
}
