#include "Flatten.hpp"

#include "CubicBezier.hpp"

#include <algorithm>
#include <cmath>
#include <new>
#include <limits>

namespace ImCut::Geometry
{
    Bounds2 FlattenedContour::ComputeBounds() const noexcept
    {
        Bounds2 bounds;
        for (const Vec2& point : points)
            bounds.Add(point);
        return bounds;
    }

    std::size_t FlattenedPath::TotalPoints() const noexcept
    {
        std::size_t total = 0;
        for (const FlattenedContour& contour : contours)
            total += contour.points.size();
        return total;
    }

    namespace Flatten
    {
        std::size_t EstimatePointCount(const Segment& segment, double tolerance) noexcept
        {
            if (segment.kind == SegmentKind::Line || tolerance <= 0.0)
                return 1;

            // FlatnessMetric is 16*d^2 for chord deviation d, and each bisection cuts
            // the deviation by about four, so the level count is log4(d / tolerance)
            // and the point count is 2^level.
            const double metric = CubicBezier::FlatnessMetric(segment);
            if (metric <= 0.0)
                return 1;

            const double deviation = std::sqrt(metric) * 0.25;
            if (deviation <= tolerance)
                return 1;

            const double levels = std::log(deviation / tolerance) / std::log(4.0);
            const double estimate = std::pow(2.0, std::ceil(levels));

            if (!std::isfinite(estimate) || estimate <= 1.0)
                return 1;

            return static_cast<std::size_t>(std::min(estimate, 4096.0));
        }

        namespace
        {
            // Rigorous upper bound on how far a piece strays from its own chord.
            //
            // FlatnessMetric bounds 16*d^2 for chord deviation d, so d <= sqrt(metric)/4.
            // Deriving the bound from the metric rather than sampling the curve means it
            // is guaranteed, not merely observed - a sampled maximum can always miss the
            // true extremum between two samples and understate the error.
            [[nodiscard]] double DeviationBound(const Segment& piece) noexcept
            {
                const double metric = CubicBezier::FlatnessMetric(piece);
                return metric > 0.0 ? std::sqrt(metric) * 0.25 : 0.0;
            }
        }

        SegmentOutcome SegmentInto(const Segment& segment, double tolerance, int maxDepth,
                                   const GeometryContext& context,
                                   std::vector<Vec2>& out, bool& hitDepthLimit,
                                   std::size_t& budget, double& achievedDeviation)
        {
            // Cancellation is checked here, on the emit path, because this is the only
            // place that runs long. The check is block-granular - once per
            // GeometryContext::kCancellationBlock points - so an uncancellable run pays
            // one predictable branch and never touches an atomic.
            SegmentOutcome outcome = SegmentOutcome::Complete;

            // NOT noexcept, deliberately.
            //
            // This used to be marked noexcept while calling out.push_back, and the
            // reserve upstream is min(estimate, budget + 1) where `estimate` is a
            // heuristic from EstimatePointCount - so the vector CAN reallocate here. A
            // throwing reallocation inside a noexcept frame is std::terminate, which is
            // the one outcome GeometryStatus::OutOfMemory exists to prevent: that status
            // was added precisely because "std::bad_alloc escaped Boolean::Execute,
            // Offset and Minkowski uncaught". Flatten had the same hole with the
            // terminate already baked in.
            //
            // Found by the WP13 hostile audit (allocation inside noexcept). The entry
            // points below now map bad_alloc to OutOfMemory, the same contract the
            // polygon backend already honours.
            auto emit = [&](Vec2 point)
            {
                if (budget == 0)
                {
                    outcome = SegmentOutcome::BudgetExhausted;
                    return false;
                }
                if (context.ShouldCheckCancellation(out.size()) && context.IsCancelled())
                {
                    outcome = SegmentOutcome::Cancelled;
                    return false;
                }
                --budget;
                out.push_back(point);
                return true;
            };

            // A straight segment is already its own polyline.
            if (segment.kind == SegmentKind::Line || CubicBezier::IsFlatEnough(segment, tolerance))
                return emit(segment.p1) ? SegmentOutcome::Complete : outcome;

            struct Item
            {
                Segment curve;
                int depth;
            };

            // Processing the left half first means the stack only ever holds the
            // pending right halves, so its depth is bounded by maxDepth + 1 and it can
            // live on the stack. No heap traffic per segment.
            constexpr int kCapacity = 64;
            Item stack[kCapacity];

            maxDepth = std::clamp(maxDepth, 1, kCapacity - 2);

            int top = 0;
            stack[top++] = { segment, 0 };

            while (top > 0)
            {
                const Item item = stack[--top];

                if (CubicBezier::IsFlatEnough(item.curve, tolerance))
                {
                    if (!emit(item.curve.p1)) return outcome;
                    continue;
                }

                if (item.depth >= maxDepth || top + 2 > kCapacity)
                {
                    // Accepted without meeting the tolerance, so the honoured deviation
                    // is this piece's own bound rather than what was asked for.
                    hitDepthLimit = true;
                    achievedDeviation = std::max(achievedDeviation, DeviationBound(item.curve));
                    if (!emit(item.curve.p1)) return outcome;
                    continue;
                }

                Segment left, right;
                CubicBezier::Split(item.curve, 0.5, left, right);

                // Right first so the left half pops next and points stay in order.
                stack[top++] = { right, item.depth + 1 };
                stack[top++] = { left, item.depth + 1 };
            }

            return SegmentOutcome::Complete;
        }

        GeometryStatus ContourInto(const ImCut::Geometry::Contour& contour, const FlattenOptions& options,
                                   const GeometryContext& context, FlattenedContour& out)
        {
            ScopedGeometryStage diagnosticStage(
                context.Diagnostics(), GeometryDiagnosticStage::FlattenExclusive);
            out.points.clear();
            out.closed = contour.closed;
            out.toleranceUsed = options.tolerance;
            out.hitDepthLimit = false;

            // Structure first: everything below indexes the parallel arrays.
            if (!contour.IsStructurallyValid())
                return GeometryStatus::InvalidInput;

            if (!(options.tolerance > 0.0) || !std::isfinite(options.tolerance))
                return GeometryStatus::InvalidInput;

            const std::size_t segmentCount = contour.SegmentCount();
            if (contour.nodes.empty())
                return GeometryStatus::Empty;

            // A token that is already signalled means the answer is Cancelled, not a
            // 600 ms run that reports a complexity limit it never actually hit.
            if (context.IsCancelled())
                return GeometryStatus::Cancelled;

            std::size_t budget = context.Limits().maxFlattenPoints;
            if (budget == 0)
                return GeometryStatus::ComplexityLimit;

            if (segmentCount == 0)
            {
                out.points.push_back(contour.nodes.front());
                return GeometryStatus::Success;
            }

            std::size_t estimate = 1;
            for (std::size_t i = 0; i < segmentCount; ++i)
                estimate += EstimatePointCount(contour.SegmentAt(i), options.tolerance);
            out.points.reserve(std::min(estimate, budget + 1));

            out.points.push_back(contour.nodes.front());
            --budget;

            double achievedDeviation = 0.0;

            for (std::size_t i = 0; i < segmentCount; ++i)
            {
                switch (SegmentInto(contour.SegmentAt(i), options.tolerance, options.maxDepth,
                                    context, out.points, out.hitDepthLimit, budget,
                                    achievedDeviation))
                {
                    case SegmentOutcome::Complete:
                        break;
                    case SegmentOutcome::Cancelled:
                        return GeometryStatus::Cancelled;
                    case SegmentOutcome::BudgetExhausted:
                    default:
                        return GeometryStatus::ComplexityLimit;
                }
            }

            // Where the depth cap fired the requested tolerance was not honoured, so the
            // reported figure is the bound that actually holds.
            out.toleranceUsed = std::max(out.toleranceUsed, achievedDeviation);

            // A closed contour is stored without repeating the start point, and the
            // final segment already returns to it. Drop the duplicate the last segment
            // just emitted so the ring has exactly one vertex per corner.
            if (contour.closed && out.points.size() > 1)
                out.points.pop_back();

            return GeometryStatus::Success;
        }

        GeometryResult<FlattenedContour> Contour(const ImCut::Geometry::Contour& contour,
                                                 const FlattenOptions& options,
                                                 const GeometryContext& context)
        {
            FlattenedContour result;
            GeometryStatus status = GeometryStatus::Success;
            try
            {
                status = ContourInto(contour, options, context, result);
            }
            catch (const std::bad_alloc&)
            {
                // Exhaustion is a status, not a crash. The limits were satisfied and the
                // machine still could not provide the memory - a different problem with a
                // different response, which is why OutOfMemory is distinct from
                // ComplexityLimit.
                return GeometryResult<FlattenedContour>::Failure(GeometryStatus::OutOfMemory);
            }
            if (status == GeometryStatus::Empty)
                return GeometryResult<FlattenedContour>::Empty(std::move(result));
            if (status != GeometryStatus::Success)
                return GeometryResult<FlattenedContour>::Failure(status);
            return GeometryResult<FlattenedContour>::Success(std::move(result));
        }

        GeometryResult<FlattenedPath> PathWith(const ImCut::Geometry::Path& path,
                                               const GeometryContext& context,
                                               const FlattenOptions& options)
        {
            if (options.tolerance <= 0.0 || !std::isfinite(options.tolerance))
                return GeometryResult<FlattenedPath>::Failure(GeometryStatus::InvalidInput);

            // Refuse structurally broken storage before anything indexes it.
            if (!path.IsStructurallyValid())
                return GeometryResult<FlattenedPath>::Failure(GeometryStatus::InvalidInput);

            // Input size limits, checked before the first allocation. maxFlattenPoints
            // bounds what comes out; these bound what goes in.
            if (path.contours.size() > context.Limits().maxContours)
                return GeometryResult<FlattenedPath>::Failure(GeometryStatus::ComplexityLimit);
            if (path.SegmentCount() > context.Limits().maxSegments)
                return GeometryResult<FlattenedPath>::Failure(GeometryStatus::ComplexityLimit);

            FlattenedPath result;
            result.fillRule = path.fillRule;
            result.contours.reserve(path.contours.size());

            // Whatever the input already carried is a stage that already happened.
            //
            // The result used to start from a zeroed budget, so a Path arriving with
            // 0.375 mm of accumulated error from earlier stages came out declaring only
            // this flatten's 0.01 mm. Flattening runs ON TOP of that error, not instead
            // of it, so the two compose sequentially.
            result.budget.MergeSequential(path.budget);

            std::size_t emitted = 0;
            std::size_t remaining = context.Limits().maxFlattenPoints;
            double worstTolerance = options.tolerance;

            for (const ImCut::Geometry::Contour& contour : path.contours)
            {
                if (context.ShouldCheckCancellation(emitted) && context.IsCancelled())
                    return GeometryResult<FlattenedPath>::Failure(GeometryStatus::Cancelled);

                // Each contour gets whatever is left, so the budget bites during
                // emission rather than after a contour has already been materialised.
                GeometryContext scoped = context;
                ComplexityLimits limits = context.Limits();
                limits.maxFlattenPoints = remaining;
                scoped.SetLimits(limits);

                FlattenedContour flattened;
                GeometryStatus status = GeometryStatus::Success;
                try
                {
                    status = ContourInto(contour, options, scoped, flattened);
                }
                catch (const std::bad_alloc&)
                {
                    // Same contract as the single-contour entry point above: exhaustion
                    // is reported, never terminated on.
                    return GeometryResult<FlattenedPath>::Failure(GeometryStatus::OutOfMemory);
                }
                if (status != GeometryStatus::Success && status != GeometryStatus::Empty)
                    return GeometryResult<FlattenedPath>::Failure(status);

                // An empty contour contributes no geometry. Preserve Empty for a path
                // whose every contour is empty, but do not store phantom contours in a
                // partially non-empty flattened path.
                if (status == GeometryStatus::Empty)
                    continue;

                emitted += flattened.points.size();
                remaining -= std::min(remaining, flattened.points.size());
                worstTolerance = std::max(worstTolerance, flattened.toleranceUsed);

                result.contours.push_back(std::move(flattened));
            }

            CountStat(context, &GeometryStatistics::flattenCalls);
            CountStat(context, &GeometryStatistics::flattenPoints, static_cast<std::uint64_t>(emitted));

            // Report the deviation actually achieved. Where the depth cap fired this is
            // larger than the tolerance that was asked for.
            result.budget.AddFlatten(worstTolerance);

            if (result.contours.empty() || emitted == 0)
            {
                const ErrorBudget carriedEmpty = result.budget;
                auto empty = GeometryResult<FlattenedPath>::Empty(std::move(result));
                empty.Budget().Merge(carriedEmpty);
                return empty;
            }

            const ErrorBudget carried = result.budget;
            auto success = GeometryResult<FlattenedPath>::Success(std::move(result));
            success.Budget().Merge(carried);
            return success;
        }

        GeometryResult<FlattenedPath> PathWith(const ImCut::Geometry::Path& path,
                                               const GeometryContext& context)
        {
            FlattenOptions options;
            options.tolerance = context.Tolerance().flatten;
            options.maxDepth = context.Limits().maxSubdivisionDepth;
            return PathWith(path, context, options);
        }
    }
}
