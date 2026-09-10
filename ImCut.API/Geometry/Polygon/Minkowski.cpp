#include "Minkowski.hpp"

#include "../Curves/Flatten.hpp"
#include "../Math/Predicates.hpp"
#include "Boolean.hpp"
#include "ConvexDecomposition.hpp"
#include "ConvexHull.hpp"
#include "PolygonMetrics.hpp"
#include "../PreparedGeometry.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace ImCut::Geometry::Minkowski
{
    namespace
    {
        // PUBLISH ONE TOTAL TO BOTH PLACES.
        //
        // GEOMETRY_ERROR_BUDGET_CONTRACT.md: the payload owns the complete total and the
        // wrapper mirrors it. Writing only the wrapper leaves a Path whose declared error
        // is zero the moment a caller extracts it with .Value() and feeds it into the next
        // operation - measured on Minkowski::Sum as wrapper 1.0 mm against payload 0.0,
        // and the same on Minkowski::Difference (evidence 857). The wrapper is a courtesy
        // to the immediate caller; the payload is what survives the call.
        //
        // Empty is published exactly like Success, because Empty is an answer: "no point
        // qualifies", reached from geometry known only to some tolerance, and a caller
        // cannot tell whether a finer tolerance would change it without that number.
        [[nodiscard]] GeometryResult<Path> PublishPath(Path payload, const ErrorBudget& budget)
        {
            payload.budget = budget;
            if (payload.contours.empty())
            {
                auto empty = GeometryResult<Path>::Empty(std::move(payload));
                empty.Budget() = budget;
                return empty;
            }
            auto success = GeometryResult<Path>::Success(std::move(payload));
            success.Budget() = budget;
            return success;
        }

        // The same, for the routes that decide Empty before there is any payload at all.
        [[nodiscard]] GeometryResult<Path> PublishEmptyPath(const ErrorBudget& budget)
        {
            return PublishPath(Path{}, budget);
        }

        // For the entry points that amend an already-built wrapper with the operands' own
        // budgets: mirror the amended total back onto the payload, for the same reason.
        void MirrorBudgetToPayload(GeometryResult<Path>& result)
        {
            if (result.HasValue())
                result.Value().budget = result.Budget();
        }

        // Flattens the single ring a Path operand is allowed to carry.
        //
        // This replaced FlattenFirstContour, which took contours.front() and discarded
        // everything else without a word: a pattern whose second contour changed from an
        // 8x8 square to an 80x80 square 200 mm away produced byte-identical output. The
        // count is now the caller's contract, checked before any work happens.
        // `achievedTolerance` receives the deviation the flattener actually honoured,
        // which is the tolerance that was asked for unless the depth cap fired. It is
        // an output because the caller has to DECLARE it: flattening a curve is a lossy
        // stage, and a Minkowski sum built on flattened operands is no more accurate
        // than the polylines it was built from.
        [[nodiscard]] GeometryResult<std::vector<Vec2>> FlattenSingleRing(const Path& path,
                                                                          const GeometryContext& context,
                                                                          double& achievedTolerance)
        {
            achievedTolerance = 0.0;
            if (path.contours.empty())
                return GeometryResult<std::vector<Vec2>>::Empty(std::vector<Vec2>{});

            if (path.contours.size() != 1)
                return GeometryResult<std::vector<Vec2>>::Failure(GeometryStatus::InvalidTopology);

            if (!path.contours.front().closed)
                return GeometryResult<std::vector<Vec2>>::Failure(GeometryStatus::InvalidTopology);

            FlattenOptions options;
            options.tolerance = context.Tolerance().flatten;
            options.maxDepth = context.Limits().maxSubdivisionDepth;

            GeometryResult<FlattenedContour> flattened =
                Flatten::Contour(path.contours.front(), options, context);
            if (!flattened.Ok())
                return GeometryResult<std::vector<Vec2>>::Failure(flattened.Status());

            // Only a curved contour is approximated by flattening. A polygon's nodes are
            // already its exact outline, and declaring a tolerance for it would inflate
            // every downstream clearance for an error that did not happen.
            bool curved = false;
            for (const SegmentKind kind : path.contours.front().kinds)
            {
                if (kind != SegmentKind::Line) { curved = true; break; }
            }
            if (curved)
                achievedTolerance = flattened.Value().toleranceUsed;

            return GeometryResult<std::vector<Vec2>>::Success(std::move(flattened.Value().points));
        }

        // Counter-clockwise copy of a ring.
        //
        // The traversal direction a ring happened to be written in is not part of the
        // set it describes, and Minkowski is an operation on sets. It used to matter:
        // the sweep quads and the subject ring were unioned under NonZero, so a subject
        // wound the other way SUBTRACTED and 2916 came back as 1944.
        [[nodiscard]] std::vector<Vec2> AsCounterClockwise(const std::vector<Vec2>& ring)
        {
            std::vector<Vec2> result = ring;
            if (Metrics::SignedArea(result.data(), result.size()) < 0.0)
                std::reverse(result.begin(), result.end());
            return result;
        }

        [[nodiscard]] Contour ContourOf(const std::vector<Vec2>& ring)
        {
            Contour contour;
            contour.Reserve(ring.size());
            contour.MoveTo(ring.front());
            for (std::size_t i = 1; i < ring.size(); ++i)
                contour.LineTo(ring[i]);
            contour.Close(0.0);
            return contour;
        }

        // Convex pieces of a ring, as one piece when the ring is already convex.
        //
        // Decomposition is not free, and the overwhelmingly common operands - a tool
        // shape, a clearance square, a hull - are convex already.
        [[nodiscard]] GeometryResult<std::vector<std::vector<Vec2>>> ConvexPieces(
            const std::vector<Vec2>& ring, const GeometryContext& context,
            ErrorBudget& budget)
        {
            using Pieces = std::vector<std::vector<Vec2>>;

            if (ring.size() < 3)
                return GeometryResult<Pieces>::Empty(Pieces{});

            if (Metrics::IsStrictlyConvex(ring.data(), ring.size()))
                return GeometryResult<Pieces>::Success(Pieces{ ring });

            GeometryResult<ConvexDecompositionResult> decomposed;
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::DecompositionExclusive);
                decomposed = Decompose::Convex(ring, context);
            }
            if (!decomposed.Ok())
                return GeometryResult<Pieces>::Failure(decomposed.Status());

            budget.MergeSequential(decomposed.Value().budget);
            return GeometryResult<Pieces>::Success(std::move(decomposed).Value().pieces);
        }

        // Unions the pairwise convex sums of two piece lists.
        //
        // Shared by the ring route and the region route, because the expensive part is
        // the same for both: ConvexSum is a microsecond-scale edge merge, and handing
        // the polygon backend a thousand heavily overlapping rings in one call is what
        // costs tens of milliseconds. Unioning per piece of A first lets the sweep see
        // tens of rings at a time instead of a thousand.
        [[nodiscard]] GeometryResult<Path> UnionPairwiseSums(
            const std::vector<std::vector<Vec2>>& piecesA,
            const std::vector<std::vector<Vec2>>& piecesB,
            const GeometryContext& context, ErrorBudget& budget)
        {
            if (piecesA.empty() || piecesB.empty())
                return GeometryResult<Path>::Empty(Path{});

            const std::size_t pairLimit = context.Limits().maxConvexPairs;
            if (piecesB.size() != 0 && piecesA.size() > pairLimit / piecesB.size())
                return GeometryResult<Path>::Failure(GeometryStatus::ComplexityLimit);
            const std::size_t pairs = piecesA.size() * piecesB.size();

            BooleanOptions unionOptions;
            unionOptions.fillRule = BooleanFillRule::NonZero;

            Path summed;
            summed.fillRule = FillRule::NonZero;

            const bool batched = piecesA.size() > 1 && pairs > 64;

            ErrorBudget batchBudget;
            std::size_t emitted = 0;

            for (const std::vector<Vec2>& pieceA : piecesA)
            {
                Path batch;
                batch.fillRule = FillRule::NonZero;
                batch.contours.reserve(piecesB.size());

                for (const std::vector<Vec2>& pieceB : piecesB)
                {
                    if (context.ShouldCheckCancellation(emitted) && context.IsCancelled())
                        return GeometryResult<Path>::Failure(GeometryStatus::Cancelled);
                    ++emitted;

                    std::vector<Vec2> piece;
                    {
                        ScopedGeometryStage stage(
                            context.Diagnostics(), GeometryDiagnosticStage::ConvexSumExclusive);
                        piece = ConvexSum(pieceA, pieceB);
                    }
                    if (piece.size() >= 3)
                    {
                        ScopedGeometryStage stage(
                            context.Diagnostics(), GeometryDiagnosticStage::ResultConversionExclusive);
                        batch.contours.push_back(ContourOf(piece));
                    }
                }

                if (batch.contours.empty())
                    continue;

                if (!batched)
                {
                    for (Contour& contour : batch.contours)
                        summed.contours.push_back(std::move(contour));
                    continue;
                }

                auto partial = Boolean::Simplify(batch, context, unionOptions);
                if (!partial.Ok() && partial.Status() != GeometryStatus::Empty)
                    return GeometryResult<Path>::Failure(partial.Status());
                if (partial.Status() == GeometryStatus::Empty)
                    continue;

                // Merge, not MergeSequential: the batches are alternatives. Any point of
                // the result passes through one batch union and then the final union,
                // never through all of them.
                batchBudget.Merge(partial.Budget());

                for (Contour& contour : partial.Value().contours)
                    summed.contours.push_back(std::move(contour));
            }

            budget.MergeSequential(batchBudget);

            if (summed.contours.empty())
                return GeometryResult<Path>::Empty(Path{});

            // One ring needs no union, and running it through the backend would quantise
            // what ConvexSum produced exactly.
            if (summed.contours.size() == 1)
                return GeometryResult<Path>::Success(std::move(summed));

            auto unioned = Boolean::Simplify(summed, context, unionOptions);
            if (!unioned.Ok() && unioned.Status() != GeometryStatus::Empty)
                return GeometryResult<Path>::Failure(unioned.Status());

            budget.MergeSequential(unioned.Budget());

            if (unioned.Status() == GeometryStatus::Empty || unioned.Value().contours.empty())
                return GeometryResult<Path>::Empty(Path{});

            return GeometryResult<Path>::Success(std::move(unioned).Value());
        }

        // A (+) B for two closed rings, by convex decomposition and exact edge merge.
        //
        // `sum == false` asks for A (-) B, which is A (+) (-B): the pattern is reflected
        // through the origin once, here, and everything below is the same code.
        //
        // The band-plus-region identity this replaced - A (+) B == A union (dA (+) B) -
        // needs B to be STAR-SHAPED about the origin, and the code only ensured the
        // origin was a VERTEX of B. For a concave pattern that punched a hole which does
        // not exist: measured against the set definition, 388 of 2401 sampled points
        // inside the true sum were reported outside.
        [[nodiscard]] GeometryResult<Path> SumByConvexPieces(const std::vector<Vec2>& pattern,
                                                             const std::vector<Vec2>& subject,
                                                             bool sum,
                                                             const GeometryContext& context)
        {
            ErrorBudget budget;

            std::vector<Vec2> patternRing = pattern;
            if (!sum)
            {
                for (Vec2& point : patternRing)
                    point = { -point.x, -point.y };
            }

            // Direction is normalised on BOTH operands, so the contract is "traversal
            // direction does not matter" rather than "undocumented".
            patternRing = AsCounterClockwise(patternRing);
            const std::vector<Vec2> subjectRing = AsCounterClockwise(subject);

            auto patternPieces = ConvexPieces(patternRing, context, budget);
            if (!patternPieces.Ok() && patternPieces.Status() != GeometryStatus::Empty)
                return GeometryResult<Path>::Failure(patternPieces.Status());

            auto subjectPieces = ConvexPieces(subjectRing, context, budget);
            if (!subjectPieces.Ok() && subjectPieces.Status() != GeometryStatus::Empty)
                return GeometryResult<Path>::Failure(subjectPieces.Status());

            auto region = UnionPairwiseSums(patternPieces.Value(), subjectPieces.Value(),
                                            context, budget);
            if (!region.Ok() && region.Status() != GeometryStatus::Empty)
                return GeometryResult<Path>::Failure(region.Status());

            CountStat(context, &GeometryStatistics::minkowskiOperations);

            if (region.Status() == GeometryStatus::Empty || region.Value().contours.empty())
                return PublishEmptyPath(budget);

            return PublishPath(std::move(region).Value(), budget);
        }

        [[nodiscard]] GeometryResult<Path> RunRings(const std::vector<Vec2>& pattern,
                                                    const std::vector<Vec2>& subject,
                                                    bool closed, bool sum,
                                                    const GeometryContext& context)
        {
            if (pattern.size() < 2 || subject.size() < 2)
                return GeometryResult<Path>::Empty(Path{});

            // CLOSED OPERANDS GO THROUGH CONVEX DECOMPOSITION, NOT THROUGH THE SWEEP.
            //
            // The sweep route computes A (+) B as A union (dA (+) B). That identity
            // needs B to be star-shaped about the origin; the code only ensured the
            // origin was a VERTEX of B, which is a strictly weaker condition, and for a
            // concave B the result grew a hole that does not exist. Measured against the
            // set definition on a 49x49 grid, 388 points inside the true sum were
            // reported outside - the direction that makes a no-fit polygon accept
            // overlapping placements.
            //
            // Minkowski distributes over union in both operands, so decomposing each
            // into convex pieces and summing every pair gives the same set:
            //
            //     (union_i A_i) (+) (union_j B_j) == union_ij (A_i (+) B_j)
            //
            // and every convex piece satisfies the precondition by construction. Each
            // pair goes through ConvexSum, an exact O(n+m) angular edge merge with no
            // lattice at all, so the only approximation left is the union at the end.
            // It is also far cheaper than the sweep it replaces - the audit measured
            // ConvexSum at 9-13 us against 64-516 ms for a backend sweep.
            if (closed)
                return SumByConvexPieces(pattern, subject, sum, context);

            // OPEN SUBJECT: sweep the pattern along a polyline.
            //
            // Same distributive argument as above, one dimension down. For a CONVEX
            // piece P and a single segment [a,b], P (+) [a,b] is convex and is exactly
            // the convex hull of (P + a) union (P + b) - the Minkowski sum of two convex
            // sets is the hull of the sum of their vertex sets. So the whole sweep is
            //
            //     union over pieces P_i, segments s_k of  hull(P_i + s_k.start,
            //                                                  P_i + s_k.end)
            //
            // which is exact in double, needs no lattice, and does not depend on the
            // backend's Minkowski at all. The old route handed the raw ring to the
            // backend sweep, which carries the same star-shaped precondition that P0-5
            // is about.
            ErrorBudget budget;

            std::vector<Vec2> patternRing = pattern;
            if (!sum)
            {
                for (Vec2& point : patternRing)
                    point = { -point.x, -point.y };
            }
            patternRing = AsCounterClockwise(patternRing);

            auto pieces = ConvexPieces(patternRing, context, budget);
            if (!pieces.Ok())
                return GeometryResult<Path>::Failure(pieces.Status());
            if (pieces.Value().empty())
                return PublishEmptyPath(budget);

            const std::size_t segments = subject.size() - 1;
            const std::size_t pairLimit = context.Limits().maxConvexPairs;
            if (segments != 0 && pieces.Value().size() > pairLimit / segments)
                return GeometryResult<Path>::Failure(GeometryStatus::ComplexityLimit);
            const std::size_t pairs = pieces.Value().size() * segments;

            Path swept;
            swept.fillRule = FillRule::NonZero;
            swept.contours.reserve(pairs);

            std::vector<Vec2> corners;
            std::size_t emitted = 0;

            for (const std::vector<Vec2>& piece : pieces.Value())
            {
                for (std::size_t k = 0; k < segments; ++k)
                {
                    if (context.ShouldCheckCancellation(emitted) && context.IsCancelled())
                        return GeometryResult<Path>::Failure(GeometryStatus::Cancelled);
                    ++emitted;

                    corners.clear();
                    corners.reserve(piece.size() * 2);
                    for (const Vec2& point : piece) corners.push_back(point + subject[k]);
                    for (const Vec2& point : piece) corners.push_back(point + subject[k + 1]);

                    const std::vector<Vec2> hull = Hull::Compute(corners);
                    if (hull.size() < 3)
                        continue;

                    swept.contours.push_back(ContourOf(hull));
                }
            }

            if (swept.contours.empty())
                return PublishEmptyPath(budget);

            BooleanOptions options;
            options.fillRule = BooleanFillRule::NonZero;

            auto unioned = Boolean::Simplify(swept, context, options);
            if (!unioned.Ok() && unioned.Status() != GeometryStatus::Empty)
                return GeometryResult<Path>::Failure(unioned.Status());

            CountStat(context, &GeometryStatistics::minkowskiOperations);

            budget.MergeSequential(unioned.Budget());

            return PublishPath(std::move(unioned).Value(), budget);
        }
    }

    GeometryResult<Path> SumRings(const std::vector<Vec2>& pattern, const std::vector<Vec2>& subject,
                                  bool closed, const GeometryContext& context)
    {
        return RunRings(pattern, subject, closed, true, context);
    }

    GeometryResult<Path> DifferenceRings(const std::vector<Vec2>& pattern, const std::vector<Vec2>& subject,
                                         bool closed, const GeometryContext& context)
    {
        return RunRings(pattern, subject, closed, false, context);
    }

    GeometryResult<Path> Sum(const Path& pattern, const Path& subject, const GeometryContext& context)
    {
        double patternTolerance = 0.0;
        double subjectTolerance = 0.0;

        const auto patternRing = FlattenSingleRing(pattern, context, patternTolerance);
        if (!patternRing.Ok())
            return GeometryResult<Path>::Failure(patternRing.Status());

        const auto subjectRing = FlattenSingleRing(subject, context, subjectTolerance);
        if (!subjectRing.Ok())
            return GeometryResult<Path>::Failure(subjectRing.Status());

        auto result = SumRings(patternRing.Value(), subjectRing.Value(), true, context);

        // Both operands were flattened before the sum, so both deviations are stages the
        // result sits on top of. They compose sequentially: a point of the sum can be
        // displaced by the pattern's flattening AND the subject's.
        result.Budget().AddSequentialFlatten(patternTolerance);
        result.Budget().AddSequentialFlatten(subjectTolerance);
        result.Budget().MergeSequential(pattern.budget);
        result.Budget().MergeSequential(subject.budget);
        MirrorBudgetToPayload(result);
        return result;
    }

    GeometryResult<Path> Difference(const Path& pattern, const Path& subject, const GeometryContext& context)
    {
        double patternTolerance = 0.0;
        double subjectTolerance = 0.0;

        const auto patternRing = FlattenSingleRing(pattern, context, patternTolerance);
        if (!patternRing.Ok())
            return GeometryResult<Path>::Failure(patternRing.Status());

        const auto subjectRing = FlattenSingleRing(subject, context, subjectTolerance);
        if (!subjectRing.Ok())
            return GeometryResult<Path>::Failure(subjectRing.Status());

        auto result = DifferenceRings(patternRing.Value(), subjectRing.Value(), true, context);
        result.Budget().AddSequentialFlatten(patternTolerance);
        result.Budget().AddSequentialFlatten(subjectTolerance);
        result.Budget().MergeSequential(pattern.budget);
        result.Budget().MergeSequential(subject.budget);
        MirrorBudgetToPayload(result);
        return result;
    }

    namespace
    {
        // True when the contour is a closed polygon - no curve segments at all - so its
        // nodes ARE its exact outline and flattening it would add a declared error to
        // geometry that has none.
        [[nodiscard]] bool IsClosedPolygon(const Contour& contour) noexcept
        {
            if (!contour.closed || contour.SegmentCount() < 3)
                return false;
            for (const SegmentKind kind : contour.kinds)
            {
                if (kind != SegmentKind::Line)
                    return false;
            }
            return true;
        }

        // Convex pieces of a whole filled region, holes and components included.
        [[nodiscard]] GeometryResult<std::vector<std::vector<Vec2>>> RegionPieces(
            const Path& path, const GeometryContext& context,
            const DecompositionOptions& options, ErrorBudget& budget)
        {
            using Pieces = std::vector<std::vector<Vec2>>;

            if (path.contours.empty())
                return GeometryResult<Pieces>::Empty(Pieces{});

            // A single convex polygonal ring is already its own convex decomposition.
            //
            // Taking it straight through keeps the whole operation exact: no flatten
            // tolerance, no lattice, nothing for the ErrorBudget to declare. Sending it
            // to Decompose::ConvexRegion would quantise it through the polygon backend
            // and the result would report an error it does not have - which matters,
            // because NfpResult::exact is what tells a caller whether the region is the
            // true Minkowski sum or an approximation of it.
            if (path.contours.size() == 1 && IsClosedPolygon(path.contours.front()))
            {
                const std::vector<Vec2>& nodes = path.contours.front().nodes;
                if (Metrics::IsStrictlyConvex(nodes.data(), nodes.size()))
                    return GeometryResult<Pieces>::Success(Pieces{ AsCounterClockwise(nodes) });
            }

            GeometryResult<ConvexDecompositionResult> decomposed;
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::DecompositionExclusive);
                decomposed = Decompose::ConvexRegion(path, context, options);
            }
            if (decomposed.Status() == GeometryStatus::Empty)
                return GeometryResult<Pieces>::Empty(Pieces{});
            if (!decomposed.Ok())
                return GeometryResult<Pieces>::Failure(decomposed.Status());

            budget.MergeSequential(decomposed.Value().budget);
            return GeometryResult<Pieces>::Success(std::move(decomposed).Value().pieces);
        }

        [[nodiscard]] Path ReflectedThroughOrigin(const Path& path)
        {
            return TransformPath(path, Transform2{ -1.0, 0.0, 0.0, -1.0, 0.0, 0.0 });
        }

        [[nodiscard]] Path RectanglePath(const Bounds2& box);

        // Exactly one closed ring of line segments whose four corners ARE the bounding
        // box corners.
        //
        // Deliberately strict. A ring with a redundant collinear vertex describes the
        // same set but is not accepted, because the whole value of saying "this is a
        // rectangle" is that the closed form below is then exact with no case analysis.
        // Refusing a shape that would have qualified costs a fast path; accepting one
        // that would not costs correctness.
        [[nodiscard]] bool IsAxisAlignedRectangle(const Path& path,
                                                  const Bounds2& bounds) noexcept
        {
            if (path.contours.size() != 1 || bounds.IsEmpty())
                return false;
            if (!IsClosedPolygon(path.contours.front()))
                return false;
            if (!(bounds.Width() > 0.0) || !(bounds.Height() > 0.0))
                return false;

            const std::vector<Vec2>& nodes = path.contours.front().nodes;
            if (nodes.size() != 4)
                return false;

            // Every vertex sits on a box corner, and all four corners are used once.
            int cornerMask = 0;
            for (const Vec2& node : nodes)
            {
                const bool left = node.x == bounds.min.x;
                const bool right = node.x == bounds.max.x;
                const bool bottom = node.y == bounds.min.y;
                const bool top = node.y == bounds.max.y;
                if ((!left && !right) || (!bottom && !top))
                    return false;
                cornerMask |= 1 << ((right ? 1 : 0) | (top ? 2 : 0));
            }
            return cornerMask == 0b1111;
        }

        // A (-) B where A is an axis-aligned box, in closed form.
        //
        //     A (-) B = { x : x + B subset of A }
        //
        // and for a box A = [Ax0,Ax1]x[Ay0,Ay1] the condition x + b in A for EVERY b in B
        // binds only on B's extremes, so
        //
        //     A (-) B = [Ax0 - Bx0, Ax1 - Bx1] x [Ay0 - By0, Ay1 - By1].
        //
        // B's interior, its holes and its components never enter, which is why this needs
        // no decomposition, no window, no Boolean and no lattice - and why the answer is
        // EXACT rather than quantised. The two worked examples in Minkowski.hpp are the
        // check: 60x60 eroded by 10x10 gives 50x50 = 2500, and the 100x100 container with
        // the 10x30 part gives 90x70 = 6300, against the 4675.4 a disc offset reported.
        //
        // Being exact means this route declares LESS error than the general one. That is
        // the honest report, not a discrepancy: no lossy stage ran.
        // The closed form applied to the two bounding boxes.
        //
        // Because A is a subset of bbox(A) and B is a subset of bbox(B), this box always
        // CONTAINS A (-) B, whatever shape either operand really is. That makes it both
        // the exact answer when A is genuinely a rectangle and a sound clamp for the
        // general route, which is how the same three subtractions serve both.
        [[nodiscard]] Bounds2 ErodedBoundsBox(const Bounds2& regionBounds,
                                              const Bounds2& elementBounds) noexcept
        {
            const double minX = regionBounds.min.x - elementBounds.min.x;
            const double maxX = regionBounds.max.x - elementBounds.max.x;
            const double minY = regionBounds.min.y - elementBounds.min.y;
            const double maxY = regionBounds.max.y - elementBounds.max.y;
            if (!(maxX > minX) || !(maxY > minY))
                return {};
            return Bounds2{ { minX, minY }, { maxX, maxY } };
        }

        [[nodiscard]] GeometryResult<Path> ErodeRectangleExact(
            const Bounds2& regionBounds, const Bounds2& elementBounds,
            const ErrorBudget& regionBudget, const ErrorBudget& provenance)
        {
            ErrorBudget budget = regionBudget;
            budget.MergeSequential(provenance);

            const Bounds2 feasible = ErodedBoundsBox(regionBounds, elementBounds);

            // The part does not fit, or fits so exactly that the feasible set has no
            // area. Both are Empty: a correct answer, not a failure.
            if (feasible.IsEmpty())
            {
                Path payload;
                payload.budget = budget;
                auto empty = GeometryResult<Path>::Empty(std::move(payload));
                empty.Budget() = budget;
                return empty;
            }

            Path payload = RectanglePath(feasible);
            payload.budget = budget;
            auto success = GeometryResult<Path>::Success(std::move(payload));
            success.Budget() = budget;
            return success;
        }

        [[nodiscard]] Path RectanglePath(const Bounds2& box)
        {
            Contour ring;
            ring.MoveTo({ box.min.x, box.min.y });
            ring.LineTo({ box.max.x, box.min.y });
            ring.LineTo({ box.max.x, box.max.y });
            ring.LineTo({ box.min.x, box.max.y });
            ring.Close(0.0);

            Path path;
            path.fillRule = FillRule::NonZero;
            path.contours.push_back(std::move(ring));
            return path;
        }
    }

    GeometryResult<Path> SumRegions(const Path& a, const Path& b, const GeometryContext& context,
                                    const DecompositionOptions& options,
                                    SumRegionsDiagnostics* diagnostics)
    {
        if (!a.IsStructurallyValid() || !b.IsStructurallyValid())
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidInput);
        if (a.HasOpenContours() || b.HasOpenContours())
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidTopology);

        ErrorBudget budget;
        budget.MergeSequential(a.budget);
        budget.MergeSequential(b.budget);

        auto piecesA = RegionPieces(a, context, options, budget);
        if (!piecesA.Ok() && piecesA.Status() != GeometryStatus::Empty)
            return GeometryResult<Path>::Failure(piecesA.Status());

        auto piecesB = RegionPieces(b, context, options, budget);
        if (!piecesB.Ok() && piecesB.Status() != GeometryStatus::Empty)
            return GeometryResult<Path>::Failure(piecesB.Status());

        if (diagnostics != nullptr)
        {
            diagnostics->piecesA = piecesA.Value().size();
            diagnostics->piecesB = piecesB.Value().size();
            if (piecesB.Value().size() != 0 &&
                piecesA.Value().size() <=
                    (std::numeric_limits<std::size_t>::max)() / piecesB.Value().size())
            {
                diagnostics->convexPairCount =
                    piecesA.Value().size() * piecesB.Value().size();
            }
        }

        auto region = UnionPairwiseSums(piecesA.Value(), piecesB.Value(), context, budget);
        if (!region.Ok() && region.Status() != GeometryStatus::Empty)
            return GeometryResult<Path>::Failure(region.Status());

        CountStat(context, &GeometryStatistics::minkowskiOperations);

        if (region.Status() == GeometryStatus::Empty || region.Value().contours.empty())
        {
            Path payload;
            payload.budget = budget;
            auto empty = GeometryResult<Path>::Empty(std::move(payload));
            empty.Budget().Merge(budget);
            return empty;
        }

        Path payload = std::move(region).Value();
        payload.budget = budget;
        auto success = GeometryResult<Path>::Success(std::move(payload));
        success.Budget().Merge(budget);
        return success;
    }

    GeometryResult<Path> SumPreparedPieces(
        const std::vector<std::vector<Vec2>>& piecesA,
        const std::vector<std::vector<Vec2>>& piecesB,
        const ErrorBudget& budgetA,
        const ErrorBudget& budgetB,
        const GeometryContext& context,
        SumRegionsDiagnostics* diagnostics)
    {
        ErrorBudget budget;
        budget.MergeSequential(budgetA);
        budget.MergeSequential(budgetB);

        if (diagnostics != nullptr)
        {
            diagnostics->piecesA = piecesA.size();
            diagnostics->piecesB = piecesB.size();
            if (piecesB.size() != 0 &&
                piecesA.size() <= (std::numeric_limits<std::size_t>::max)() / piecesB.size())
            {
                diagnostics->convexPairCount = piecesA.size() * piecesB.size();
            }
        }

        auto region = UnionPairwiseSums(piecesA, piecesB, context, budget);
        if (!region.Ok() && region.Status() != GeometryStatus::Empty)
            return GeometryResult<Path>::Failure(region.Status());

        Path payload = region.Status() == GeometryStatus::Empty
            ? Path{} : std::move(region).Value();
        payload.budget = budget;
        if (payload.contours.empty())
        {
            auto empty = GeometryResult<Path>::Empty(std::move(payload));
            empty.Budget().Merge(budget);
            return empty;
        }

        auto success = GeometryResult<Path>::Success(std::move(payload));
        success.Budget().Merge(budget);
        return success;
    }

    std::size_t PreparedErosionElement::ApproximateBytes() const noexcept
    {
        std::size_t bytes = sizeof(PreparedErosionElement) +
                            reflectedPieces_.capacity() *
                                sizeof(std::vector<Vec2>);
        for (const std::vector<Vec2>& piece : reflectedPieces_)
            bytes += piece.capacity() * sizeof(Vec2);
        return bytes;
    }

    GeometryResult<PreparedErosionElement> PrepareErosionElement(
        const Path& structuringElement, const GeometryContext& context,
        const DecompositionOptions& options)
    {
        if (!structuringElement.IsStructurallyValid())
            return GeometryResult<PreparedErosionElement>::Failure(
                GeometryStatus::InvalidInput);
        if (structuringElement.HasOpenContours())
            return GeometryResult<PreparedErosionElement>::Failure(
                GeometryStatus::InvalidTopology);
        if (structuringElement.contours.empty())
            return GeometryResult<PreparedErosionElement>::Empty(
                PreparedErosionElement{});

        const Bounds2 elementBounds = Metrics::ComputeBounds(structuringElement);
        if (elementBounds.IsEmpty())
            return GeometryResult<PreparedErosionElement>::Empty(
                PreparedErosionElement{});

        // Normalise to the anchor, then reflect through the origin: exactly the form
        // the dilation consumes, so the per-query path does neither again.
        const Vec2 elementAnchor = elementBounds.min;
        const Path normalizedElement = TransformPath(
            structuringElement,
            Transform2::Translation({ -elementAnchor.x, -elementAnchor.y }));
        const Path reflected = ReflectedThroughOrigin(normalizedElement);

        PreparedErosionElement prepared;
        prepared.elementBounds_ = elementBounds;

        // No MergeSequential of structuringElement.budget here, on purpose.
        //
        // The derivative owns its own preparation cost and nothing else. RegionPieces
        // adds only intrinsic stages: Convert::PathToIntegers builds its budget from a
        // fresh ErrorBudget (flatten + quantization) rather than from path.budget, so
        // the caller's error cannot leak in through the decomposition either. The
        // element therefore describes B's convex pieces for ANY caller, and the one who
        // asked carries their own provenance in PreparedErosionOperand.
        auto pieces = RegionPieces(reflected, context, options, prepared.budget_);
        if (!pieces.Ok() && pieces.Status() != GeometryStatus::Empty)
            return GeometryResult<PreparedErosionElement>::Failure(pieces.Status());
        if (pieces.Status() == GeometryStatus::Empty || pieces.Value().empty())
            return GeometryResult<PreparedErosionElement>::Empty(
                PreparedErosionElement{});

        prepared.reflectedPieces_ = std::move(pieces).Value();
        prepared.valid_ = true;

        auto success = GeometryResult<PreparedErosionElement>::Success(
            std::move(prepared));
        success.Budget().Merge(success.Value().Budget());
        return success;
    }

    GeometryResult<Path> ErodePrepared(
        const Path& region, const PreparedErosionOperand& operand,
        const GeometryContext& context, const DecompositionOptions& options)
    {
        if (operand.element == nullptr)
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidInput);
        const PreparedErosionElement& element = *operand.element;
        if (!element.Valid())
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidInput);
        if (!region.IsStructurallyValid())
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidInput);
        if (region.HasOpenContours())
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidTopology);
        // Even the earliest Empty carries what the caller declared. "Nothing fits" derived
        // from a region known to 0.5 mm is a different statement from "nothing fits"
        // derived from an exact one, and only the budget tells the two apart.
        ErrorBudget earlyBudget = region.budget;
        earlyBudget.MergeSequential(operand.provenance);

        if (region.contours.empty())
            return PublishEmptyPath(earlyBudget);

        const Bounds2 regionBounds = Metrics::ComputeBounds(region);
        const Bounds2& elementBounds = element.ElementBounds();
        if (regionBounds.IsEmpty() || elementBounds.IsEmpty())
            return PublishEmptyPath(earlyBudget);

        // A rectangular sheet is the common case a nesting run asks about, and for it the
        // answer is a closed form that needs none of the machinery below. The element's
        // prepared decomposition is simply not consulted - its bounding box is the whole
        // input - so this reports no intrinsic error, because none was incurred.
        if (IsAxisAlignedRectangle(region, regionBounds))
        {
            return ErodeRectangleExact(regionBounds, elementBounds, region.budget,
                                       operand.provenance);
        }

        // Same reference frame as the one-shot route: solve at the element's anchor and
        // translate feasible reference points back by -anchor.
        //   A erode (B0 + anchor) == (A erode B0) - anchor
        const Vec2 elementAnchor = elementBounds.min;

        // One full diameter of B on each side, so the window boundary cannot reach back
        // into the answer.
        const double margin = 2.0 * (elementBounds.Width() + elementBounds.Height()) + 1.0;
        const Bounds2 window = regionBounds.Expanded(margin);
        const Path windowPath = RectanglePath(window);

        auto complement = Boolean::Difference(windowPath, region, context);
        if (!complement.Ok() && complement.Status() != GeometryStatus::Empty)
            return GeometryResult<Path>::Failure(complement.Status());

        if (complement.Status() == GeometryStatus::Empty ||
            complement.Value().contours.empty())
        {
            // A bounded region cannot fill a window strictly larger than itself.
            // Refuse rather than answer from a contradiction.
            return GeometryResult<Path>::Failure(GeometryStatus::NumericalFailure);
        }

        ErrorBudget complementBudget;
        complementBudget.MergeSequential(complement.Value().budget);
        auto complementPieces = RegionPieces(
            complement.Value(), context, options, complementBudget);
        if (!complementPieces.Ok() &&
            complementPieces.Status() != GeometryStatus::Empty)
        {
            return GeometryResult<Path>::Failure(complementPieces.Status());
        }

        // The element's pieces are already prepared; only the complement is decomposed
        // per query, and it depends on the region so it cannot be hoisted.
        auto dilated = SumPreparedPieces(
            complementPieces.Value(), element.reflectedPieces_,
            complementBudget, element.Budget(), context, nullptr);
        if (!dilated.Ok() && dilated.Status() != GeometryStatus::Empty)
            return GeometryResult<Path>::Failure(dilated.Status());

        if (dilated.Status() == GeometryStatus::Empty ||
            dilated.Value().contours.empty())
        {
            // Nothing dilated means nothing excluded, so the erosion would be the whole
            // window, which is not a meaningful answer. Empty, not Success.
            //
            // Empty is a correct value and carries the same budget ownership rules as
            // Success, so caller provenance is composed on this route too.
            ErrorBudget emptyBudget = dilated.Budget();
            emptyBudget.MergeSequential(operand.provenance);
            Path emptyPayload;
            emptyPayload.budget = emptyBudget;
            auto empty = GeometryResult<Path>::Empty(std::move(emptyPayload));
            empty.Budget() = emptyBudget;
            return empty;
        }

        auto eroded = Boolean::Difference(windowPath, dilated.Value(), context);
        if (!eroded.Ok() && eroded.Status() != GeometryStatus::Empty)
            return GeometryResult<Path>::Failure(eroded.Status());

        // Intrinsic ancestry only, at this point. The window is a synthetic exact
        // rectangle, the region's own budget arrived through Boolean::Difference, and the
        // element contributes only its preparation stages.
        //
        // The caller provenance is composed once, at the bottom, AFTER the clamp. V8.1
        // added it here instead, which left the clamp branch below merging a wrapper into
        // a budget that was already downstream of it. Composing last makes the single
        // composition point structural rather than a promise in a comment.
        ErrorBudget budget = eroded.Budget();

        CountStat(context, &GeometryStatistics::minkowskiOperations);

        Path output = eroded.Status() == GeometryStatus::Empty
            ? Path{} : std::move(eroded).Value();
        if (!output.contours.empty())
        {
            output = TransformPath(
                output, Transform2::Translation({ -elementAnchor.x, -elementAnchor.y }));

            // CLAMP TO THE FEASIBLE BOX. This is a correctness step, not a tidy-up.
            //
            // The window identity  A (-) B == W \ ((W \ A) (+) (-B))  only holds where
            // x + B stays inside W. Points close to the window's own boundary can survive
            // the subtraction while being nowhere near feasible, because the dilation
            // never reached out far enough to exclude them - it grows the frame by -B0,
            // which contains the origin only when B happens to touch the minimum corner
            // of its own bounding box. Real artwork does not.
            //
            // Measured on BETA.svg against a 3000x2000 sheet: the erosion came back as
            // the correct 3.5819e6 mm2 region PLUS a spurious 4.2598e4 mm2 ring hugging
            // the window, and the reported bounds spanned [-3825, 3736] x [-2433, 4129]
            // instead of the sheet - a feasible region wider than its own container.
            // All four corpus parts were affected. No test covered it because the only
            // corpus IFP coverage was a benchmark, which checked status and not geometry.
            //
            // A (-) B is always a subset of bbox(A) (-) bbox(B), and every point of that
            // box satisfies x + B subset of W, which is exactly the domain where the
            // identity is valid. So intersecting with it cannot remove a feasible point
            // and cannot keep an infeasible one.
            const Bounds2 feasible = ErodedBoundsBox(regionBounds, elementBounds);
            if (feasible.IsEmpty())
            {
                output.contours.clear();
            }
            else if (feasible.Contains(Metrics::ComputeBounds(output)))
            {
                // Already inside the feasible box, so intersecting with it is the
                // identity. Skipping the Boolean here is not a shortcut past the check -
                // the check has been made and passed. It matters because the clamp is on
                // every non-rectangular erosion and costs 15-97% of one when it runs:
                // 'holed x convex' went 0.0324 -> 0.0639 ms before this short-circuit.
            }
            else
            {
                auto clamped = Boolean::Intersection(
                    output, RectanglePath(feasible), context);
                if (!clamped.Ok() && clamped.Status() != GeometryStatus::Empty)
                    return GeometryResult<Path>::Failure(clamped.Status());

                // TAKE the clamp budget, do not ADD it.
                //
                // `output` is the payload of `eroded`, so output.budget already carries
                // the whole ancestry `budget` holds, and Boolean composes the subject
                // budget into its result (Boolean.cpp:239). clamped.Budget() is therefore
                // ancestry + the clamp stage - a complete total, not an increment.
                // MergeSequential is additive, so merging it in counted the ancestry
                // twice: measured slope 2.0 against the region's declared error on four
                // fixtures, against 1.0 on the same route with the clamp skipped
                // (evidence 843). That is the contract rule quoted in the budget document:
                // a parent does not re-add a wrapper whose payload already carries
                // ancestry.
                //
                // Overcount is a defect in its own right, not a safe conservatism: in
                // nesting an inflated budget becomes clearance that is not needed and
                // sheet that is not used.
                budget = clamped.Budget();
                output = clamped.Status() == GeometryStatus::Empty
                    ? Path{} : std::move(clamped).Value();
            }
        }
        // THE single composition point for caller provenance, below every intrinsic stage
        // including the clamp. Nothing after this line touches the budget, so the caller
        // error enters exactly once on every exit - Success and Empty alike, which carry
        // the same ownership rules.
        budget.MergeSequential(operand.provenance);

        output.budget = budget;
        if (output.contours.empty())
        {
            auto empty = GeometryResult<Path>::Empty(std::move(output));
            empty.Budget().Merge(budget);
            return empty;
        }

        auto success = GeometryResult<Path>::Success(std::move(output));
        success.Budget().Merge(budget);
        return success;
    }

    GeometryResult<Path> Erode(const Path& region, const Path& structuringElement,
                               const GeometryContext& context, const DecompositionOptions& options)
    {
        // One-shot erosion is now literally "prepare the element, then use it". The two
        // routes cannot drift apart on reference frame, window margin, budget or empty
        // semantics, because after this there is only one implementation of any of them.
        if (!region.IsStructurallyValid() || !structuringElement.IsStructurallyValid())
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidInput);
        if (region.HasOpenContours() || structuringElement.HasOpenContours())
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidTopology);
        // Same rule as the prepared route: an early Empty still declares what the caller
        // declared about both operands.
        ErrorBudget oneShotEarlyBudget = region.budget;
        oneShotEarlyBudget.MergeSequential(structuringElement.budget);

        if (region.contours.empty() || structuringElement.contours.empty())
            return PublishEmptyPath(oneShotEarlyBudget);

        // Same closed form, checked before preparing, so the one-shot route does not pay
        // for a decomposition the answer never reads. Both routes call the SAME exact
        // helper, so there is still only one implementation of the mathematics.
        const Bounds2 oneShotRegionBounds = Metrics::ComputeBounds(region);
        if (IsAxisAlignedRectangle(region, oneShotRegionBounds))
        {
            const Bounds2 oneShotElementBounds = Metrics::ComputeBounds(structuringElement);
            if (!oneShotElementBounds.IsEmpty())
            {
                return ErodeRectangleExact(oneShotRegionBounds, oneShotElementBounds,
                                           region.budget, structuringElement.budget);
            }
            return PublishEmptyPath(oneShotEarlyBudget);
        }

        auto element = PrepareErosionElement(structuringElement, context, options);
        if (!element.Ok() && element.Status() != GeometryStatus::Empty)
            return GeometryResult<Path>::Failure(element.Status());
        if (element.Status() == GeometryStatus::Empty || !element.Value().Valid())
        {
            // The preparation ran, so its own stages count too.
            ErrorBudget prepared = oneShotEarlyBudget;
            prepared.MergeSequential(element.Budget());
            return PublishEmptyPath(prepared);
        }

        // The structuring element's own provenance is the caller's error for THIS query.
        // It rides in the operand rather than inside the derivative, and ErodePrepared
        // composes it exactly once, so this route's declared total is unchanged.
        return ErodePrepared(
            region,
            PreparedErosionOperand(element.Value(), structuringElement.budget),
            context, options);
    }

    void ConvexSumInto(std::span<const Vec2> a, std::span<const Vec2> b,
                       std::vector<Vec2>& output, std::vector<Vec2>& scratch)
    {
        output.clear();
        scratch.clear();
        if (a.size() < 3 || b.size() < 3)
            return;

        // Both inputs must run counter-clockwise for the angular merge below to be
        // monotone in edge direction. Traverse clockwise callers in reverse logical
        // order instead of allocating reversed copies; prepared operands are CCW.
        const bool forwardA = Metrics::SignedArea(a.data(), a.size()) >= 0.0;
        const bool forwardB = Metrics::SignedArea(b.data(), b.size()) >= 0.0;

        // Start each ring at its bottom-most, then left-most vertex, which is the
        // vertex whose outgoing edge has the smallest polar angle.
        auto lowestIndex = [](std::span<const Vec2> ring) noexcept
        {
            std::size_t best = 0;
            for (std::size_t i = 1; i < ring.size(); ++i)
            {
                if (ring[i].y < ring[best].y || (ring[i].y == ring[best].y && ring[i].x < ring[best].x))
                    best = i;
            }
            return best;
        };

        const std::size_t startA = lowestIndex(a);
        const std::size_t startB = lowestIndex(b);

        scratch.reserve(a.size() + b.size());
        output.reserve(a.size() + b.size());

        auto pointAt = [](std::span<const Vec2> ring, std::size_t start,
                          std::size_t offset, bool forward) noexcept
        {
            const std::size_t normalized = offset % ring.size();
            const std::size_t index = forward
                ? (start + normalized) % ring.size()
                : (start + ring.size() - normalized) % ring.size();
            return ring[index];
        };

        std::size_t i = 0;
        std::size_t j = 0;

        // Merge the two edge sequences in angle order. Every edge of the sum is an edge
        // of one input translated, so no clipping is needed and the result is exact.
        while (i < a.size() || j < b.size())
        {
            const Vec2 pointA = pointAt(a, startA, i, forwardA);
            const Vec2 pointB = pointAt(b, startB, j, forwardB);
            scratch.push_back(pointA + pointB);

            if (i >= a.size())      { ++j; continue; }
            if (j >= b.size())      { ++i; continue; }

            const Vec2 edgeA = pointAt(a, startA, i + 1, forwardA) - pointA;
            const Vec2 edgeB = pointAt(b, startB, j + 1, forwardB) - pointB;

            const double cross = Cross(edgeA, edgeB);
            if (cross > 0.0)      ++i;
            else if (cross < 0.0) ++j;
            else                  { ++i; ++j; }
        }

        // Collinear vertices can appear where the two edge directions coincided.
        for (std::size_t k = 0; k < scratch.size(); ++k)
        {
            const Vec2 previous = scratch[(k + scratch.size() - 1) % scratch.size()];
            const Vec2 current = scratch[k];
            const Vec2 next = scratch[(k + 1) % scratch.size()];

            if (DistanceSquared(previous, current) == 0.0)
                continue;
            if (Predicates::Orient2D(previous, current, next) == 0.0)
                continue;

            output.push_back(current);
        }

        if (output.size() < 3)
            output.assign(scratch.begin(), scratch.end());
    }

    void ConvexSumInto(const std::vector<Vec2>& a, const std::vector<Vec2>& b,
                       std::vector<Vec2>& output, std::vector<Vec2>& scratch)
    {
        ConvexSumInto(std::span<const Vec2>{ a }, std::span<const Vec2>{ b },
                      output, scratch);
    }

    bool ConvexSumContains(std::span<const Vec2> a, std::span<const Vec2> b,
                           Vec2 point, double boundaryTolerance,
                           double clearance) noexcept
    {
        if (a.size() < 3 || b.size() < 3 || !IsFinite(point) ||
            !(boundaryTolerance >= 0.0) || !std::isfinite(boundaryTolerance) ||
            !(clearance >= 0.0) || !std::isfinite(clearance))
        {
            return false;
        }

        const bool forwardA = Metrics::SignedArea(a.data(), a.size()) >= 0.0;
        const bool forwardB = Metrics::SignedArea(b.data(), b.size()) >= 0.0;
        auto lowestIndex = [](std::span<const Vec2> ring) noexcept
        {
            std::size_t best = 0;
            for (std::size_t index = 1; index < ring.size(); ++index)
            {
                if (ring[index].y < ring[best].y ||
                    (ring[index].y == ring[best].y && ring[index].x < ring[best].x))
                {
                    best = index;
                }
            }
            return best;
        };
        const std::size_t startA = lowestIndex(a);
        const std::size_t startB = lowestIndex(b);
        auto pointAt = [](std::span<const Vec2> ring, std::size_t start,
                          std::size_t offset, bool forward) noexcept
        {
            const std::size_t normalized = offset % ring.size();
            const std::size_t index = forward
                ? (start + normalized) % ring.size()
                : (start + ring.size() - normalized) % ring.size();
            return ring[index];
        };

        // Replays the same angular merge as ConvexSumInto, but emits edges directly.
        // Collinear and duplicate vertices do not change the represented convex set.
        auto forEachEdge = [&](auto&& visit) noexcept
        {
            std::size_t i = 0;
            std::size_t j = 0;
            Vec2 first{};
            Vec2 previous{};
            bool havePrevious = false;
            while (i < a.size() || j < b.size())
            {
                const Vec2 pointA = pointAt(a, startA, i, forwardA);
                const Vec2 pointB = pointAt(b, startB, j, forwardB);
                const Vec2 current = pointA + pointB;
                if (!havePrevious)
                {
                    first = current;
                    previous = current;
                    havePrevious = true;
                }
                else if (current != previous)
                {
                    if (!visit(previous, current)) return false;
                    previous = current;
                }

                if (i >= a.size()) { ++j; continue; }
                if (j >= b.size()) { ++i; continue; }
                const Vec2 edgeA = pointAt(a, startA, i + 1, forwardA) - pointA;
                const Vec2 edgeB = pointAt(b, startB, j + 1, forwardB) - pointB;
                const double cross = Cross(edgeA, edgeB);
                if (cross > 0.0) ++i;
                else if (cross < 0.0) ++j;
                else { ++i; ++j; }
            }
            return !havePrevious || previous == first || visit(previous, first);
        };

        if (clearance == 0.0)
        {
            bool boundary = false;
            bool outside = false;
            forEachEdge([&](Vec2 from, Vec2 to) noexcept
            {
                if (Predicates::PointOnSegment(
                        point, from, to, boundaryTolerance))
                {
                    boundary = true;
                }
                if (Predicates::Orient2D(from, to, point) < 0.0)
                    outside = true;
                return true;
            });
            return boundary || !outside;
        }

        // p belongs to C (+) [-c,c]^2 exactly when C intersects the square p-[-c,c]^2.
        // SAT gives the exact interior decision. If the two closed convex sets are
        // disjoint, their Euclidean distance is attained by a vertex/edge pair; that
        // distance reproduces PointInRing's boundary-tolerance contract without ever
        // materializing C or C(+square).
        const std::array<Vec2, 4> square{
            Vec2{ point.x - clearance, point.y - clearance },
            Vec2{ point.x + clearance, point.y - clearance },
            Vec2{ point.x + clearance, point.y + clearance },
            Vec2{ point.x - clearance, point.y + clearance }
        };
        Bounds2 convexBounds;
        bool separatedByConvexEdge = false;
        forEachEdge([&](Vec2 from, Vec2 to) noexcept
        {
            convexBounds.Add(from);
            convexBounds.Add(to);
            bool allOutside = true;
            for (Vec2 corner : square)
            {
                if (Predicates::Orient2D(from, to, corner) >= 0.0)
                {
                    allOutside = false;
                    break;
                }
            }
            separatedByConvexEdge = separatedByConvexEdge || allOutside;
            return true;
        });
        const Bounds2 squareBounds{ square[0], square[2] };
        if (!separatedByConvexEdge && convexBounds.Overlaps(squareBounds))
            return true;
        if (boundaryTolerance == 0.0)
            return false;

        auto pointSegmentDistanceSquared = [](Vec2 p, Vec2 from, Vec2 to) noexcept
        {
            const Vec2 edge = to - from;
            const double lengthSquared = Dot(edge, edge);
            if (!(lengthSquared > 0.0)) return DistanceSquared(p, from);
            double t = Dot(p - from, edge) / lengthSquared;
            t = (std::max)(0.0, (std::min)(1.0, t));
            return DistanceSquared(p, from + edge * t);
        };
        const double toleranceSquared = boundaryTolerance * boundaryTolerance;
        bool closeEnough = false;
        forEachEdge([&](Vec2 from, Vec2 to) noexcept
        {
            for (std::size_t edge = 0; edge < square.size(); ++edge)
            {
                const Vec2 q0 = square[edge];
                const Vec2 q1 = square[(edge + 1) % square.size()];
                const double candidate = (std::min)({
                    pointSegmentDistanceSquared(from, q0, q1),
                    pointSegmentDistanceSquared(to, q0, q1),
                    pointSegmentDistanceSquared(q0, from, to),
                    pointSegmentDistanceSquared(q1, from, to) });
                if (candidate <= toleranceSquared)
                {
                    closeEnough = true;
                    return false;
                }
            }
            return true;
        });
        return closeEnough;
    }

    std::vector<Vec2> ConvexSum(const std::vector<Vec2>& a, const std::vector<Vec2>& b)
    {
        std::vector<Vec2> result;
        std::vector<Vec2> scratch;
        ConvexSumInto(a, b, result, scratch);
        return result;
    }
}
