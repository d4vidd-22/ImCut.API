#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryTypes.hpp"
#include "../Math/Bounds2.hpp"

#include <cstdint>
#include <utility>
#include <vector>

namespace ImCut::Geometry
{
    enum class BvhSplitStrategy : std::uint8_t
    {
        // Split at the median of primitive centroids along the longest axis.
        // O(n log n) and very cheap to build. This is the default, on measurement.
        LongestAxisMedian,

        // Binned surface-area heuristic: more expensive to build, in exchange for
        // cheaper traversal.
        //
        // Measured on contour segments (Bench "bvh") it does not pay for itself. At
        // 100k segments and 200k bounds queries, median totalled 139 ms against SAH's
        // 167 ms, and both produced the identical 9,467,577 candidates - so SAH's extra
        // build time bought no traversal advantage at all. That is what a contour is:
        // consecutive segments are already spatially coherent and similarly sized, so a
        // median split lands where SAH would anyway. SAH is kept because scattered,
        // wildly uneven primitive sets are a different distribution where it can win,
        // but it is not the default here.
        BinnedSAH
    };

    // Static bounding volume hierarchy over segment bounds.
    //
    // Nodes live in one contiguous vector and children are referenced by index, so a
    // build costs a handful of allocations rather than one per node and traversal
    // walks memory forwards. Queries keep their stack in automatic storage; the
    // callback forms never touch the heap.
    class SegmentBVH
    {
    public:
        struct Node
        {
            Bounds2 bounds;

            // Leaf: index of the first primitive in Order().
            // Internal: index of the left child. Children are always allocated as an
            // adjacent pair, so the right child is start + 1. Storing one index
            // instead of two keeps the node at 40 bytes and works regardless of the
            // order the build happens to visit subtrees in.
            std::uint32_t start = 0;

            // Zero marks an internal node.
            std::uint32_t count = 0;

            [[nodiscard]] bool IsLeaf() const noexcept { return count != 0; }
            [[nodiscard]] std::uint32_t LeftChild() const noexcept { return start; }
            [[nodiscard]] std::uint32_t RightChild() const noexcept { return start + 1; }
        };

        static constexpr std::uint32_t kDefaultLeafSize = 8;

        // Inputs at or below this size skip the build and become one linear-scan leaf.
        //
        // Measured, and the measurement contradicted the obvious guess. Sweeping 4 to
        // 128 primitives over 10k bounds queries (Bench "smalln"), building the tree won
        // at every size that produces more than one node - 4.2x at 128 primitives, and
        // never slower below that. A larger threshold only helps a build-once,
        // query-once workload, which is the opposite of this kernel's contract, so the
        // threshold is pinned to the leaf size: below that a tree would be a single leaf
        // anyway, and above it the tree always pays for itself.
        static constexpr std::uint32_t kDefaultSmallInputThreshold = kDefaultLeafSize;

        // Hard cap on tree depth, enforced during the build by forcing a leaf. This is
        // what makes every traversal stack below provably large enough, so no query
        // ever has to drop a node and risk a broad-phase false negative.
        static constexpr int kMaxBuildDepth = 60;

        struct BuildOptions
        {
            BvhSplitStrategy strategy = BvhSplitStrategy::LongestAxisMedian;
            std::uint32_t leafSize = kDefaultLeafSize;

            // Inputs at or below this size become a single leaf answered by linear scan.
            std::uint32_t smallInputThreshold = kDefaultSmallInputThreshold;
        };

        void Clear() noexcept;

        // Builds over externally-owned bounds. The hierarchy keeps indices, never the
        // geometry, so the caller stays the single owner of the segment data.
        // Explicit overloads rather than `= {}` default arguments.
        //
        // A default argument of `{}` for a nested class with default member initialisers
        // needs that class to be complete at the point of declaration, which it is not
        // inside the enclosing class on Clang. Overloads express the same API and compile
        // on both toolchains; the bodies below are parsed after the class is complete, so
        // naming BuildOptions there is fine.
        void Build(const Bounds2* bounds, std::size_t count, const BuildOptions& options);
        void Build(const Bounds2* bounds, std::size_t count)
        {
            Build(bounds, count, BuildOptions{});
        }
        void Build(std::vector<Bounds2>&& bounds, const BuildOptions& options);
        void Build(std::vector<Bounds2>&& bounds)
        {
            Build(std::move(bounds), BuildOptions{});
        }

        void BuildFromSegments(const Segment* segments, std::size_t count,
                               const BuildOptions& options);
        void BuildFromSegments(const Segment* segments, std::size_t count)
        {
            BuildFromSegments(segments, count, BuildOptions{});
        }

        void BuildFromSegments(const Segment* segments, std::size_t count,
                               const GeometryContext& context, const BuildOptions& options);
        void BuildFromSegments(const Segment* segments, std::size_t count,
                               const GeometryContext& context)
        {
            BuildFromSegments(segments, count, context, BuildOptions{});
        }

        [[nodiscard]] bool Empty() const noexcept { return primitiveCount_ == 0; }
        [[nodiscard]] std::size_t PrimitiveCount() const noexcept { return primitiveCount_; }
        [[nodiscard]] std::size_t NodeCount() const noexcept { return nodes_.size(); }
        [[nodiscard]] std::size_t DynamicBytes() const noexcept
        {
            return nodes_.capacity() * sizeof(Node) +
                   order_.capacity() * sizeof(std::uint32_t) +
                   primitiveBounds_.capacity() * sizeof(Bounds2) +
                   centroids_.capacity() * sizeof(Vec2);
        }
        [[nodiscard]] const std::vector<Node>& Nodes() const noexcept { return nodes_; }
        [[nodiscard]] int MaxDepth() const noexcept { return maxDepth_; }

        // Primitive indices in leaf order. Query callbacks receive original indices.
        [[nodiscard]] const std::vector<std::uint32_t>& Order() const noexcept { return order_; }

        [[nodiscard]] Bounds2 RootBounds() const noexcept
        {
            return nodes_.empty() ? Bounds2{} : nodes_.front().bounds;
        }

        [[nodiscard]] const Bounds2& PrimitiveBounds(std::uint32_t index) const noexcept
        {
            return primitiveBounds_[index];
        }

        // Visits every primitive whose bounds overlap `box`.
        // `visit` returns false to stop early, which is what makes boolean
        // "does anything hit this" queries cheap.
        // Returns false when the traversal was stopped early.
        template <typename Visitor>
        bool QueryBounds(const Bounds2& box, Visitor&& visit) const
        {
            if (nodes_.empty() || box.IsEmpty())
                return true;

            std::uint32_t stack[kMaxStack];
            int top = 0;
            stack[top++] = 0;

            while (top > 0)
            {
                const Node& node = nodes_[stack[--top]];
                if (!node.bounds.Overlaps(box))
                    continue;

                if (node.IsLeaf())
                {
                    const std::uint32_t end = node.start + node.count;
                    for (std::uint32_t i = node.start; i < end; ++i)
                    {
                        const std::uint32_t primitive = order_[i];
                        if (!primitiveBounds_[primitive].Overlaps(box))
                            continue;
                        if (!visit(primitive))
                            return false;
                    }
                    continue;
                }

                // Cannot overflow: the build caps depth at kMaxBuildDepth and each
                // internal node grows the stack by at most one.
                stack[top++] = node.LeftChild();
                stack[top++] = node.RightChild();
            }

            return true;
        }

        // Appends overlapping primitive indices to `out`, which is cleared first.
        // Reuse the same vector across queries to keep the hot path allocation-free.
        void QueryBounds(const Bounds2& box, std::vector<std::uint32_t>& out) const;

        [[nodiscard]] bool AnyOverlap(const Bounds2& box) const;

        // Visits every pair of primitives whose bounds overlap, each unordered pair
        // exactly once with first < second. This is what keeps self-intersection off
        // the naive O(n^2) pairwise path.
        // `visit` returns false to stop early; returns false when stopped.
        template <typename Visitor>
        bool QuerySelfPairs(Visitor&& visit) const
        {
            if (nodes_.empty())
                return true;

            std::pair<std::uint32_t, std::uint32_t> stack[kMaxPairStack];
            int top = 0;
            stack[top++] = { 0u, 0u };

            while (top > 0)
            {
                const auto [indexA, indexB] = stack[--top];
                const Node& a = nodes_[indexA];
                const Node& b = nodes_[indexB];

                if (indexA != indexB && !a.bounds.Overlaps(b.bounds))
                    continue;

                if (a.IsLeaf() && b.IsLeaf())
                {
                    const std::uint32_t endA = a.start + a.count;
                    const std::uint32_t endB = b.start + b.count;

                    for (std::uint32_t i = a.start; i < endA; ++i)
                    {
                        // Within a single leaf, walk the upper triangle only.
                        const std::uint32_t firstJ = (indexA == indexB) ? i + 1 : b.start;
                        for (std::uint32_t j = firstJ; j < endB; ++j)
                        {
                            std::uint32_t first = order_[i];
                            std::uint32_t second = order_[j];
                            if (first == second)
                                continue;
                            if (first > second)
                                std::swap(first, second);

                            if (!primitiveBounds_[first].Overlaps(primitiveBounds_[second]))
                                continue;
                            if (!visit(first, second))
                                return false;
                        }
                    }
                    continue;
                }

                if (indexA == indexB)
                {
                    // Each subtree against itself, plus the cross pair exactly once.
                    const std::uint32_t left = a.LeftChild();
                    const std::uint32_t right = a.RightChild();
                    stack[top++] = { left, left };
                    stack[top++] = { right, right };
                    stack[top++] = { left, right };
                    continue;
                }

                // Descend the larger node so the recursion always makes progress.
                if (!a.IsLeaf() && (b.IsLeaf() || a.bounds.Area() >= b.bounds.Area()))
                {
                    stack[top++] = { a.LeftChild(), indexB };
                    stack[top++] = { a.RightChild(), indexB };
                }
                else
                {
                    stack[top++] = { indexA, b.LeftChild() };
                    stack[top++] = { indexA, b.RightChild() };
                }
            }

            return true;
        }

        // Visits candidate pairs across two hierarchies as visit(indexInThis,
        // indexInOther). Returns false when stopped early.
        template <typename Visitor>
        bool QueryPairs(const SegmentBVH& other, Visitor&& visit) const
        {
            if (nodes_.empty() || other.nodes_.empty())
                return true;

            std::pair<std::uint32_t, std::uint32_t> stack[kMaxPairStack];
            int top = 0;
            stack[top++] = { 0u, 0u };

            while (top > 0)
            {
                const auto [indexA, indexB] = stack[--top];
                const Node& a = nodes_[indexA];
                const Node& b = other.nodes_[indexB];

                if (!a.bounds.Overlaps(b.bounds))
                    continue;

                if (a.IsLeaf() && b.IsLeaf())
                {
                    const std::uint32_t endA = a.start + a.count;
                    const std::uint32_t endB = b.start + b.count;
                    for (std::uint32_t i = a.start; i < endA; ++i)
                    {
                        const std::uint32_t first = order_[i];
                        for (std::uint32_t j = b.start; j < endB; ++j)
                        {
                            const std::uint32_t second = other.order_[j];
                            if (!primitiveBounds_[first].Overlaps(other.primitiveBounds_[second]))
                                continue;
                            if (!visit(first, second))
                                return false;
                        }
                    }
                    continue;
                }

                if (!a.IsLeaf() && (b.IsLeaf() || a.bounds.Area() >= b.bounds.Area()))
                {
                    stack[top++] = { a.LeftChild(), indexB };
                    stack[top++] = { a.RightChild(), indexB };
                }
                else
                {
                    stack[top++] = { indexA, b.LeftChild() };
                    stack[top++] = { indexA, b.RightChild() };
                }
            }

            return true;
        }

        // Visits pairs whose Minkowski-sum enclosure can contain `point`. For bounds
        // A and B, every a+b is enclosed by [A.min+B.min, A.max+B.max]; expanding that
        // box by `margin` therefore makes rejection one-sided and proof-safe. The
        // traversal uses a fixed stack and retains no Cartesian-product state.
        //
        // `keepGoing` is called once per node pair for cooperative cancellation.
        // `visit` returns false after it has found an answer or otherwise wants to stop.
        template <typename Continue, typename Visitor>
        bool QueryMinkowskiPairs(const SegmentBVH& other, Vec2 point, double margin,
                                 Continue&& keepGoing, Visitor&& visit,
                                 std::size_t* nodePairsVisited = nullptr,
                                 std::size_t* leafPairsVisited = nullptr) const
        {
            if (nodes_.empty() || other.nodes_.empty())
                return true;

            auto couldContain = [&](const Bounds2& a, const Bounds2& b) noexcept
            {
                const Bounds2 sum{ a.min + b.min, a.max + b.max };
                return sum.Contains(point, margin);
            };

            std::pair<std::uint32_t, std::uint32_t> stack[kMaxPairStack];
            int top = 0;
            stack[top++] = { 0u, 0u };
            while (top > 0)
            {
                if (!keepGoing()) return false;
                const auto [indexA, indexB] = stack[--top];
                if (nodePairsVisited != nullptr) ++*nodePairsVisited;
                const Node& a = nodes_[indexA];
                const Node& b = other.nodes_[indexB];
                if (!couldContain(a.bounds, b.bounds))
                    continue;

                if (a.IsLeaf() && b.IsLeaf())
                {
                    const std::uint32_t endA = a.start + a.count;
                    const std::uint32_t endB = b.start + b.count;
                    for (std::uint32_t i = a.start; i < endA; ++i)
                    {
                        const std::uint32_t first = order_[i];
                        for (std::uint32_t j = b.start; j < endB; ++j)
                        {
                            const std::uint32_t second = other.order_[j];
                            if (!couldContain(primitiveBounds_[first],
                                              other.primitiveBounds_[second]))
                            {
                                continue;
                            }
                            if (leafPairsVisited != nullptr) ++*leafPairsVisited;
                            if (!visit(first, second)) return false;
                        }
                    }
                    continue;
                }

                if (!a.IsLeaf() &&
                    (b.IsLeaf() || a.bounds.Area() >= b.bounds.Area()))
                {
                    stack[top++] = { a.LeftChild(), indexB };
                    stack[top++] = { a.RightChild(), indexB };
                }
                else
                {
                    stack[top++] = { indexA, b.LeftChild() };
                    stack[top++] = { indexA, b.RightChild() };
                }
            }
            return true;
        }

        // Visits primitives in increasing order of box distance from `point`, pruning
        // any branch that cannot beat the caller's current best.
        //
        // `visit(index, boxDistanceSquared)` returns the caller's updated best squared
        // distance, which immediately tightens the pruning radius.
        template <typename Visitor>
        void QueryNearest(Vec2 point, double maxDistanceSquared, Visitor&& visit) const
        {
            if (nodes_.empty())
                return;

            struct Entry
            {
                double distanceSquared;
                std::uint32_t node;
            };

            Entry stack[kMaxStack];
            int top = 0;
            stack[top++] = { nodes_.front().bounds.DistanceSquared(point), 0u };

            double best = maxDistanceSquared;

            while (top > 0)
            {
                const Entry entry = stack[--top];
                if (entry.distanceSquared > best)
                    continue;

                const Node& node = nodes_[entry.node];
                if (node.IsLeaf())
                {
                    const std::uint32_t end = node.start + node.count;
                    for (std::uint32_t i = node.start; i < end; ++i)
                    {
                        const std::uint32_t primitive = order_[i];
                        const double boxDistance = primitiveBounds_[primitive].DistanceSquared(point);
                        if (boxDistance > best)
                            continue;
                        best = visit(primitive, boxDistance);
                    }
                    continue;
                }

                const std::uint32_t left = node.LeftChild();
                const std::uint32_t right = node.RightChild();

                const double leftDistance = nodes_[left].bounds.DistanceSquared(point);
                const double rightDistance = nodes_[right].bounds.DistanceSquared(point);

                // Push the farther child first so the nearer one is popped next and
                // tightens `best` before the far branch is even considered.
                if (leftDistance <= rightDistance)
                {
                    if (rightDistance <= best) stack[top++] = { rightDistance, right };
                    if (leftDistance <= best) stack[top++] = { leftDistance, left };
                }
                else
                {
                    if (leftDistance <= best) stack[top++] = { leftDistance, left };
                    if (rightDistance <= best) stack[top++] = { rightDistance, right };
                }
            }
        }

    private:
        // Single-tree traversal grows by at most one entry per internal node, so
        // kMaxBuildDepth plus slack is sufficient. Pair traversals descend two trees
        // and the self-pair case pushes three, hence the larger stack.
        static constexpr int kMaxStack = kMaxBuildDepth + 8;
        static constexpr int kMaxPairStack = kMaxBuildDepth * 6;

        struct BuildEntry
        {
            std::uint32_t begin;
            std::uint32_t end;
            std::uint32_t nodeIndex;
            int depth;
        };

        void BuildInternal(const BuildOptions& options);

        [[nodiscard]] std::uint32_t PartitionMedian(std::uint32_t begin, std::uint32_t end, int axis);
        [[nodiscard]] std::uint32_t PartitionSAH(std::uint32_t begin, std::uint32_t end,
                                                 const Bounds2& centroidBounds, int axis);

        [[nodiscard]] Bounds2 ComputeBounds(std::uint32_t begin, std::uint32_t end) const noexcept;
        [[nodiscard]] Bounds2 ComputeCentroidBounds(std::uint32_t begin, std::uint32_t end) const noexcept;

        std::vector<Node> nodes_;
        std::vector<std::uint32_t> order_;
        std::vector<Bounds2> primitiveBounds_;
        std::vector<Vec2> centroids_;
        std::size_t primitiveCount_ = 0;
        int maxDepth_ = 0;
    };
}
