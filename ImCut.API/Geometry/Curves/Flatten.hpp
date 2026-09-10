#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Math/Bounds2.hpp"

#include <cstdint>
#include <vector>

namespace ImCut::Geometry
{
    struct FlattenOptions
    {
        // Maximum allowed deviation between the curve and the emitted polyline (mm).
        double tolerance = 1e-2;

        int maxDepth = 40;

        [[nodiscard]] bool operator==(const FlattenOptions& other) const noexcept
        {
            return tolerance == other.tolerance && maxDepth == other.maxDepth;
        }
    };

    // A flattened contour always travels with its own approximation metadata, so a
    // consumer can never mistake a 0.05 mm polyline for exact geometry, and a cache
    // can never hand a coarse result to a caller that asked for a finer one.
    struct FlattenedContour
    {
        std::vector<Vec2> points;
        bool closed = false;

        // The tolerance actually honoured. Equal to the requested one unless
        // `hitDepthLimit` is set.
        double toleranceUsed = 0.0;

        // True when subdivision stopped on the depth cap rather than on the flatness
        // test, meaning `toleranceUsed` is a floor and not a guarantee.
        bool hitDepthLimit = false;

        [[nodiscard]] Bounds2 ComputeBounds() const noexcept;
    };

    struct FlattenedPath
    {
        std::vector<FlattenedContour> contours;
        FillRule fillRule = FillRule::EvenOdd;
        ErrorBudget budget;

        [[nodiscard]] std::size_t TotalPoints() const noexcept;
    };

    namespace Flatten
    {
        // Conservative estimate of how many points a segment will produce, used only
        // to size reservations. Never affects the emitted geometry.
        [[nodiscard]] std::size_t EstimatePointCount(const Segment& segment, double tolerance) noexcept;

        // Appends the polyline approximation of `segment` to `out`, excluding p0 so
        // that consecutive segments chain without duplicating shared endpoints.
        //
        // Subdivision is adaptive and iterative: a nearly-straight span emits a single
        // point while a tight corner keeps splitting locally, and the explicit stack
        // (sized for the depth cap) means no recursion and no allocation per segment.
        //
        // `budget` is decremented as points are emitted and the walk stops the moment it
        // runs out. Enforcing the limit only after a contour finished meant an
        // adversarial curve allocated everything first and complained afterwards.
        // `achievedDeviation` is raised to a rigorous upper bound on the deviation of
        // any piece that stopped on the depth cap rather than on the flatness test. It
        // is derived from that piece's own flatness metric, not sampled, so the reported
        // error can never be smaller than the real one.
        //
        // Outcome of subdividing one segment. Three distinct answers, because
        // "exhausted the point budget" and "the caller asked us to stop" are different
        // facts, and code composing them has to be able to tell which happened.
        enum class SegmentOutcome : std::uint8_t { Complete, BudgetExhausted, Cancelled };

        // `context` is consulted while points are being EMITTED, not once per contour.
        //
        // Cancellation used to be checked one level up, per contour, so a Path with a
        // single contour was never checked at all. With the token already signalled
        // before the call, the audit measured this running for 614 ms, allocating 16
        // million points, and then returning ComplexityLimit - a limit that was not the
        // reason it stopped.
        [[nodiscard]] SegmentOutcome SegmentInto(const Segment& segment, double tolerance,
                                                 int maxDepth, const GeometryContext& context,
                                                 std::vector<Vec2>& out, bool& hitDepthLimit,
                                                 std::size_t& budget, double& achievedDeviation);

        // Every flatten entry point takes a context.
        //
        // There used to be context-free overloads that built an internal context with
        // maxFlattenPoints = SIZE_MAX. They were convenient, and they were the reason a
        // configured complexity limit protected Flatten::PathWith and nothing else:
        // GeometrySession::Flatten, PolygonConversion and the containment tree all went
        // through them and could allocate without bound. Removing them makes the bypass
        // impossible to reach rather than merely fixed at three call sites.
        [[nodiscard]] GeometryResult<FlattenedContour> Contour(const ImCut::Geometry::Contour& contour,
                                                               const FlattenOptions& options,
                                                               const GeometryContext& context);

        // Budget-aware form. Returns ComplexityLimit when the context's point budget is
        // exhausted mid-emission, InvalidInput for structurally broken storage.
        [[nodiscard]] GeometryStatus ContourInto(const ImCut::Geometry::Contour& contour,
                                                 const FlattenOptions& options,
                                                 const GeometryContext& context,
                                                 FlattenedContour& out);

        // Flattens every contour of a path, enforcing the context's point budget.
        // Returns ComplexityLimit rather than exhausting memory on runaway input, and
        // Cancelled when the caller's token trips.
        [[nodiscard]] GeometryResult<FlattenedPath> PathWith(const ImCut::Geometry::Path& path,
                                                             const GeometryContext& context,
                                                             const FlattenOptions& options);

        // Uses the context's own flatten tolerance.
        [[nodiscard]] GeometryResult<FlattenedPath> PathWith(const ImCut::Geometry::Path& path,
                                                             const GeometryContext& context);
    }
}
