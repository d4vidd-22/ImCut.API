#include "SegmentBVH.hpp"

#include "../Curves/CubicBezier.hpp"

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <limits>

namespace ImCut::Geometry
{
    namespace
    {
        // Twelve bins is the usual sweet spot: enough resolution that the chosen plane
        // is close to a full sweep, cheap enough that binning stays a single linear
        // pass over the range.
        constexpr int kSahBins = 12;

        // 2D analogue of surface area. Perimeter is the correct measure for the
        // probability that a random line hits a box in the plane.
        [[nodiscard]] double SahCost(const Bounds2& bounds, std::uint32_t count) noexcept
        {
            if (count == 0 || bounds.IsEmpty())
                return 0.0;
            return bounds.Perimeter() * static_cast<double>(count);
        }
    }

    void SegmentBVH::Clear() noexcept
    {
        nodes_.clear();
        order_.clear();
        primitiveBounds_.clear();
        centroids_.clear();
        primitiveCount_ = 0;
        maxDepth_ = 0;
    }

    Bounds2 SegmentBVH::ComputeBounds(std::uint32_t begin, std::uint32_t end) const noexcept
    {
        Bounds2 bounds;
        for (std::uint32_t i = begin; i < end; ++i)
            bounds.Add(primitiveBounds_[order_[i]]);
        return bounds;
    }

    Bounds2 SegmentBVH::ComputeCentroidBounds(std::uint32_t begin, std::uint32_t end) const noexcept
    {
        Bounds2 bounds;
        for (std::uint32_t i = begin; i < end; ++i)
            bounds.Add(centroids_[order_[i]]);
        return bounds;
    }

    std::uint32_t SegmentBVH::PartitionMedian(std::uint32_t begin, std::uint32_t end, int axis)
    {
        const std::uint32_t middle = begin + (end - begin) / 2;

        // nth_element is O(n) and deterministic for a given input, which is all the
        // determinism guarantee requires. The index tie-break keeps equal centroids in
        // a stable, reproducible order.
        std::nth_element(order_.begin() + begin, order_.begin() + middle, order_.begin() + end,
                         [&](std::uint32_t lhs, std::uint32_t rhs) noexcept
                         {
                             const double a = axis == 0 ? centroids_[lhs].x : centroids_[lhs].y;
                             const double b = axis == 0 ? centroids_[rhs].x : centroids_[rhs].y;
                             if (a != b) return a < b;
                             return lhs < rhs;
                         });

        return middle;
    }

    std::uint32_t SegmentBVH::PartitionSAH(std::uint32_t begin, std::uint32_t end,
                                           const Bounds2& centroidBounds, int axis)
    {
        const double low = axis == 0 ? centroidBounds.min.x : centroidBounds.min.y;
        const double high = axis == 0 ? centroidBounds.max.x : centroidBounds.max.y;
        const double extent = high - low;

        if (!(extent > 0.0))
            return PartitionMedian(begin, end, axis);

        const double scale = static_cast<double>(kSahBins) / extent;

        Bounds2 binBounds[kSahBins];
        std::uint32_t binCounts[kSahBins] = {};

        auto binOf = [&](std::uint32_t primitive) noexcept
        {
            const double centroid = axis == 0 ? centroids_[primitive].x : centroids_[primitive].y;
            int bin = static_cast<int>((centroid - low) * scale);
            return std::clamp(bin, 0, kSahBins - 1);
        };

        for (std::uint32_t i = begin; i < end; ++i)
        {
            const std::uint32_t primitive = order_[i];
            const int bin = binOf(primitive);
            binBounds[bin].Add(primitiveBounds_[primitive]);
            ++binCounts[bin];
        }

        // Prefix sweep from the left, suffix sweep from the right, so every candidate
        // plane is evaluated in two linear passes rather than a quadratic scan.
        Bounds2 leftBounds[kSahBins];
        std::uint32_t leftCounts[kSahBins] = {};
        Bounds2 running;
        std::uint32_t runningCount = 0;
        for (int i = 0; i < kSahBins; ++i)
        {
            running.Add(binBounds[i]);
            runningCount += binCounts[i];
            leftBounds[i] = running;
            leftCounts[i] = runningCount;
        }

        double bestCost = std::numeric_limits<double>::infinity();
        int bestSplit = -1;

        Bounds2 rightRunning;
        std::uint32_t rightCount = 0;
        for (int i = kSahBins - 1; i > 0; --i)
        {
            rightRunning.Add(binBounds[i]);
            rightCount += binCounts[i];

            const std::uint32_t leftCount = leftCounts[i - 1];
            if (leftCount == 0 || rightCount == 0)
                continue;

            const double cost = SahCost(leftBounds[i - 1], leftCount) + SahCost(rightRunning, rightCount);
            if (cost < bestCost)
            {
                bestCost = cost;
                bestSplit = i;
            }
        }

        if (bestSplit < 0)
            return PartitionMedian(begin, end, axis);

        const auto middle = std::partition(order_.begin() + begin, order_.begin() + end,
                                           [&](std::uint32_t primitive) noexcept
                                           {
                                               return binOf(primitive) < bestSplit;
                                           });

        const auto index = static_cast<std::uint32_t>(middle - order_.begin());

        // A plane that leaves one side empty would loop forever; fall back.
        if (index == begin || index == end)
            return PartitionMedian(begin, end, axis);

        return index;
    }

    void SegmentBVH::BuildInternal(const BuildOptions& options)
    {
        const auto count = static_cast<std::uint32_t>(primitiveCount_);

        nodes_.clear();
        maxDepth_ = 0;

        if (count == 0)
            return;

        // Small-N fast path: a single leaf answered by linear scan. Building a tree
        // over a handful of segments costs more than the scans it would save.
        if (count <= options.smallInputThreshold)
        {
            Node leaf;
            leaf.bounds = ComputeBounds(0, count);
            leaf.start = 0;
            leaf.count = count;
            nodes_.push_back(leaf);
            maxDepth_ = 1;
            return;
        }

        const std::uint32_t leafSize = options.leafSize == 0 ? 1u : options.leafSize;

        // Median splitting produces a complete balanced leaf level. Reserve that
        // exact upper bound instead of 2*n; with the default leaf size this avoids
        // retaining roughly 7/8 of an unused node buffer. SAH may split unevenly,
        // so it keeps the general 2*n upper bound.
        std::size_t nodeCapacity = static_cast<std::size_t>(count) * 2;
        if (options.strategy == BvhSplitStrategy::LongestAxisMedian)
        {
            const std::size_t minimumLeaves =
                (static_cast<std::size_t>(count) + leafSize - 1) / leafSize;
            const std::size_t balancedLeaves = std::bit_ceil(minimumLeaves);
            nodeCapacity = balancedLeaves * 2 - 1;
        }
        nodes_.reserve(nodeCapacity);
        nodes_.push_back(Node{});

        std::array<BuildEntry, kMaxBuildDepth + 8> stack{};
        std::size_t top = 0;
        stack[top++] = { 0u, count, 0u, 1 };

        while (top > 0)
        {
            const BuildEntry entry = stack[--top];

            maxDepth_ = std::max(maxDepth_, entry.depth);

            Bounds2 bounds = ComputeBounds(entry.begin, entry.end);
            const std::uint32_t rangeCount = entry.end - entry.begin;

            const bool forceLeaf = rangeCount <= leafSize || entry.depth >= kMaxBuildDepth;
            if (forceLeaf)
            {
                Node& node = nodes_[entry.nodeIndex];
                node.bounds = bounds;
                node.start = entry.begin;
                node.count = rangeCount;
                continue;
            }

            const Bounds2 centroidBounds = ComputeCentroidBounds(entry.begin, entry.end);
            const int axis = centroidBounds.LongestAxis();

            std::uint32_t middle;
            if (centroidBounds.LongestAxisLength() <= 0.0)
            {
                // Every centroid coincides, so no split plane separates anything.
                // Halving the range keeps the tree balanced and terminating.
                middle = entry.begin + rangeCount / 2;
            }
            else if (options.strategy == BvhSplitStrategy::BinnedSAH)
            {
                middle = PartitionSAH(entry.begin, entry.end, centroidBounds, axis);
            }
            else
            {
                middle = PartitionMedian(entry.begin, entry.end, axis);
            }

            if (middle <= entry.begin || middle >= entry.end)
                middle = entry.begin + rangeCount / 2;

            // Children are allocated adjacently so the right child is always left + 1.
            const auto leftIndex = static_cast<std::uint32_t>(nodes_.size());
            nodes_.push_back(Node{});
            nodes_.push_back(Node{});

            Node& node = nodes_[entry.nodeIndex];
            node.bounds = bounds;
            node.start = leftIndex;
            node.count = 0;

            stack[top++] = {
                middle, entry.end, leftIndex + 1, entry.depth + 1 };
            stack[top++] = {
                entry.begin, middle, leftIndex, entry.depth + 1 };
        }
    }

    void SegmentBVH::Build(const Bounds2* bounds, std::size_t count, const BuildOptions& options)
    {
        Clear();

        if (bounds == nullptr || count == 0)
            return;

        primitiveCount_ = count;
        primitiveBounds_.assign(bounds, bounds + count);

        centroids_.resize(count);
        order_.resize(count);
        for (std::size_t i = 0; i < count; ++i)
        {
            centroids_[i] = primitiveBounds_[i].IsEmpty() ? Vec2{} : primitiveBounds_[i].Center();
            order_[i] = static_cast<std::uint32_t>(i);
        }

        BuildInternal(options);
        std::vector<Vec2>{}.swap(centroids_);
    }

    void SegmentBVH::Build(
        std::vector<Bounds2>&& bounds, const BuildOptions& options)
    {
        Clear();
        if (bounds.empty())
            return;

        primitiveBounds_ = std::move(bounds);
        primitiveCount_ = primitiveBounds_.size();
        centroids_.resize(primitiveCount_);
        order_.resize(primitiveCount_);
        for (std::size_t i = 0; i < primitiveCount_; ++i)
        {
            centroids_[i] = primitiveBounds_[i].IsEmpty()
                ? Vec2{} : primitiveBounds_[i].Center();
            order_[i] = static_cast<std::uint32_t>(i);
        }

        BuildInternal(options);
        std::vector<Vec2>{}.swap(centroids_);
    }

    void SegmentBVH::BuildFromSegments(const Segment* segments, std::size_t count,
                                       const BuildOptions& options)
    {
        Clear();

        if (segments == nullptr || count == 0)
            return;

        primitiveCount_ = count;
        primitiveBounds_.resize(count);
        centroids_.resize(count);
        order_.resize(count);

        for (std::size_t i = 0; i < count; ++i)
        {
            // Tight curve bounds, not the control hull: a loose box here multiplies
            // through every traversal as extra candidate pairs. And an ENCLOSURE, because
            // this box PRUNES a certified query - TraverseNearestFirst drops a candidate
            // pair once its box gap reaches the running upper bound, so a box a few ULP
            // too tight can drop the pair that held the true minimum. That was F35b.
            //
            // `ExactBounds` is not an enclosure: measured over 28 000 random cubics,
            // 9807 of 59 359 true extrema fall outside it, by up to 4 ULP (1354).
            //
            // The etapa B tried `CertifiedSubCurve(segment, 0, 1)` and measured why that
            // is not the answer either: a whole curve is rarely monotone on either axis,
            // so the certified enclosure falls back to the control hull, and the hull of a
            // curve with reaching handles is far larger than its exact box - 112 spatial
            // assertions failed, one nearest query returning 0 against an expected 38.7.
            //
            // `CertifiedExactBounds` cuts at the extrema and certifies each piece, so each
            // piece is monotone by construction and the hull fallback is avoided on 97% of
            // random curves. Where it still falls back the box is merely loose, which
            // costs candidate pairs and never an answer.
            primitiveBounds_[i] = CubicBezier::CertifiedExactBounds(segments[i]);
            centroids_[i] = primitiveBounds_[i].IsEmpty() ? Vec2{} : primitiveBounds_[i].Center();
            order_[i] = static_cast<std::uint32_t>(i);
        }

        BuildInternal(options);
        std::vector<Vec2>{}.swap(centroids_);
    }

    void SegmentBVH::BuildFromSegments(const Segment* segments, std::size_t count,
                                       const GeometryContext& context, const BuildOptions& options)
    {
        BuildFromSegments(segments, count, options);
        CountStat(context, &GeometryStatistics::bvhBuilds);
        CountStat(context, &GeometryStatistics::bvhNodes, static_cast<std::uint64_t>(nodes_.size()));
    }

    void SegmentBVH::QueryBounds(const Bounds2& box, std::vector<std::uint32_t>& out) const
    {
        out.clear();
        QueryBounds(box, [&](std::uint32_t primitive)
        {
            out.push_back(primitive);
            return true;
        });
    }

    bool SegmentBVH::AnyOverlap(const Bounds2& box) const
    {
        // Stops at the first hit rather than gathering every candidate.
        return !QueryBounds(box, [](std::uint32_t) { return false; });
    }
}
