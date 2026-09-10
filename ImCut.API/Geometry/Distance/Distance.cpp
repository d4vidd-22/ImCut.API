#include "Distance.hpp"

#include "../Curves/CubicBezier.hpp"
#include "../Math/CertifiedInterval.hpp"
#include "../Spatial/SegmentBVH.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <queue>
#include <vector>

namespace ImCut::Geometry::Proximity
{
    namespace
    {
        // THE STRUCTURAL GATE, at the one place every Contour-taking route here funnels
        // through. Section 15A.
        //
        // SegmentAt() indexes three parallel arrays unchecked, which is sound only behind
        // Contour::IsStructurallyValid. The sibling file Intersection/SelfIntersection.cpp
        // had exactly this shape and applied the gate at two of its six entry points; the
        // three that returned a result rather than a bool read out of bounds and killed
        // the process. Nothing here applied it at all - PointWithin, ContoursWithin,
        // ContourContour and BuildPathHierarchy all gathered straight from the caller's
        // contour.
        //
        // [[nodiscard]] so the next route added cannot quietly skip it.
        [[nodiscard]] bool GatherSegments(const Contour& contour, std::vector<Segment>& out)
        {
            out.clear();
            if (!contour.IsStructurallyValid())
                return false;

            const std::size_t count = contour.SegmentCount();
            out.reserve(count);
            for (std::size_t i = 0; i < count; ++i)
                out.push_back(contour.SegmentAt(i));
            return true;
        }

        // Descends two hierarchies together, pruning any node pair already farther
        // apart than `limitSquared`. The caller tightens `limitSquared` as it finds
        // closer pairs, which shrinks the surviving tree rapidly - this is what keeps
        // contour-to-contour distance off the O(n*m) all-pairs path.
        // `visit(i, j)` returns false to stop the traversal.
        template <typename Visitor>
        void PrunedPairs(const SegmentBVH& a, const SegmentBVH& b,
                         const double& limitSquared, Visitor&& visit)
        {
            if (a.Empty() || b.Empty())
                return;

            struct Entry { std::uint32_t a; std::uint32_t b; };
            Entry stack[SegmentBVH::kMaxBuildDepth * 6];
            int top = 0;
            stack[top++] = { 0u, 0u };

            while (top > 0)
            {
                const Entry entry = stack[--top];
                const SegmentBVH::Node& nodeA = a.Nodes()[entry.a];
                const SegmentBVH::Node& nodeB = b.Nodes()[entry.b];

                if (nodeA.bounds.DistanceSquared(nodeB.bounds) >= limitSquared)
                    continue;

                if (nodeA.IsLeaf() && nodeB.IsLeaf())
                {
                    const std::uint32_t endA = nodeA.start + nodeA.count;
                    const std::uint32_t endB = nodeB.start + nodeB.count;
                    for (std::uint32_t i = nodeA.start; i < endA; ++i)
                    {
                        const std::uint32_t first = a.Order()[i];
                        for (std::uint32_t j = nodeB.start; j < endB; ++j)
                        {
                            const std::uint32_t second = b.Order()[j];
                            if (a.PrimitiveBounds(first).DistanceSquared(b.PrimitiveBounds(second)) >= limitSquared)
                                continue;
                            if (!visit(first, second))
                                return;
                        }
                    }
                    continue;
                }

                if (!nodeA.IsLeaf() && (nodeB.IsLeaf() || nodeA.bounds.Area() >= nodeB.bounds.Area()))
                {
                    stack[top++] = { nodeA.LeftChild(), entry.b };
                    stack[top++] = { nodeA.RightChild(), entry.b };
                }
                else
                {
                    stack[top++] = { entry.a, nodeB.LeftChild() };
                    stack[top++] = { entry.a, nodeB.RightChild() };
                }
            }
        }
    }

    double PointSegmentSquared(Vec2 point, Vec2 a, Vec2 b) noexcept
    {
        const Vec2 ab = b - a;
        const double lengthSquared = LengthSquared(ab);
        if (lengthSquared <= 0.0)
            return DistanceSquared(point, a);

        const double t = std::clamp(Dot(point - a, ab) / lengthSquared, 0.0, 1.0);
        return DistanceSquared(point, a + ab * t);
    }

    ClosestResult ClosestOnSegment(Vec2 point, Vec2 a, Vec2 b) noexcept
    {
        ClosestResult result;

        const Vec2 ab = b - a;
        const double lengthSquared = LengthSquared(ab);
        if (lengthSquared <= 0.0)
        {
            result.point = a;
            result.parameter = 0.0;
            result.distanceSquared = DistanceSquared(point, a);
            return result;
        }

        result.parameter = std::clamp(Dot(point - a, ab) / lengthSquared, 0.0, 1.0);
        result.point = a + ab * result.parameter;
        result.distanceSquared = DistanceSquared(point, result.point);
        return result;
    }

    double SegmentSegmentSquared(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1) noexcept
    {
        const Vec2 d1 = a1 - a0;
        const Vec2 d2 = b1 - b0;
        const Vec2 r = a0 - b0;

        const double a = LengthSquared(d1);
        const double e = LengthSquared(d2);
        const double f = Dot(d2, r);

        // Either or both degenerate to points.
        if (a <= 0.0 && e <= 0.0) return DistanceSquared(a0, b0);
        if (a <= 0.0)             return PointSegmentSquared(a0, b0, b1);
        if (e <= 0.0)             return PointSegmentSquared(b0, a0, a1);

        const double c = Dot(d1, r);
        const double b = Dot(d1, d2);
        const double denominator = a * e - b * b;

        // Parallel segments have no unique closest pair, so clamp to an endpoint and
        // let the endpoint refinement below settle it.
        double s = denominator != 0.0 ? std::clamp((b * f - c * e) / denominator, 0.0, 1.0) : 0.0;
        double t = (b * s + f) / e;

        if (t < 0.0)
        {
            t = 0.0;
            s = std::clamp(-c / a, 0.0, 1.0);
        }
        else if (t > 1.0)
        {
            t = 1.0;
            s = std::clamp((b - c) / a, 0.0, 1.0);
        }

        return DistanceSquared(a0 + d1 * s, b0 + d2 * t);
    }

    double PointCurveSquared(Vec2 point, const Segment& curve) noexcept
    {
        if (curve.IsLine())
            return PointSegmentSquared(point, curve.p0, curve.p1);
        return CubicBezier::ClosestPoint(curve, point).distanceSquared;
    }

    namespace
    {
        // The guaranteed chord deviation of one piece, derived from the flatness metric
        // rather than sampled: FlatnessMetric bounds 16 d^2, so d <= sqrt(metric)/4. Same
        // expression Flatten.cpp uses when its own depth cap fires.
        [[nodiscard]] double PieceDeviationBound(const Segment& piece) noexcept
        {
            const double metric = CubicBezier::FlatnessMetric(piece);
            return metric > 0.0 ? std::sqrt(metric) * 0.25 : 0.0;
        }
    }

    double CurveCurveSquared(const Segment& a, const Segment& b, double tolerance,
                             double upperBoundSquared, std::uint64_t* subdivisionsOut,
                             double* capDeviationOut) noexcept
    {
        if (a.IsLine() && b.IsLine())
            return SegmentSegmentSquared(a.p0, a.p1, b.p0, b.p1);

        // Counted locally and published once at the end, so the hot loop touches a
        // register rather than the caller's memory.
        std::uint64_t subdivisions = 0;

        // Chord deviation attached to the pair that currently OWNS `best`.
        //
        // Not "the worst over every capped pair the search touched": the error in the
        // answer is governed by the pair that produced the answer, and a capped pair whose
        // chord distance was rejected contributes nothing to it. Recording every capped
        // pair overstated massively - two curves that intersect drive the fine-tolerance
        // search into the depth cap while the true distance is 0, so the blanket version
        // declared 2.6e-6 of error on an answer that was exactly right.
        double bestCapDeviation = 0.0;

        // F35a, the nominal sibling. The box here PRUNES - `lowerBoundSquared >= best`
        // discards the pair outright - so a box that is short of the true extrema makes
        // the lower bound too high, and the pair holding the true minimum can be dropped.
        // That is a false negative in `ContoursWithin`, which turns this number into a
        // proximity BOOLEAN, not merely a slightly large distance.
        //
        // Same fix as Intersection.cpp: the piece is named by the ORIGINAL curve and a
        // parameter interval, so the enclosure comes from a blossom of fixed depth three
        // off the original control points and stops composing with subdivision depth. The
        // boxes were already carried in the node here, so this costs the difference
        // between one blossom and one quadratic solve per child, and nothing else.
        struct Pair
        {
            Segment first;
            Segment second;
            Bounds2 boxFirst;
            Bounds2 boxSecond;
            double lowerBoundSquared;
            int depth;
            double a0, a1;
            double b0, b1;
        };

        const auto boxA = [&a](double t0, double t1) noexcept
        {
            return CubicBezier::CertifiedSubCurve(a, t0, t1).box;
        };
        const auto boxB = [&b](double t0, double t1) noexcept
        {
            return CubicBezier::CertifiedSubCurve(b, t0, t1).box;
        };

        // Depth is capped at 24 and a pop pushes two, so the stack holds at most one
        // unvisited sibling per level: 25 entries is the true ceiling and this is more
        // than twice that.
        Pair stack[64];
        int top = 0;
        stack[top++] = { a, b, boxA(0.0, 1.0), boxB(0.0, 1.0), 0.0, 0, 0.0, 1.0, 0.0, 1.0 };

        // Seeded with what the caller already knows. A pair it cannot use is pruned on
        // the first test rather than resolved to the tolerance and then discarded.
        double best = upperBoundSquared;
        const double toleranceSquared = tolerance * tolerance;

        while (top > 0)
        {
            const Pair pair = stack[--top];

            // Re-tested on pop, not only on push: best has usually shrunk since, and
            // that is what makes the ordering below pay.
            if (pair.lowerBoundSquared >= best)
                continue;

            const bool flatA = CubicBezier::IsGeometricallyFlat(pair.first, tolerance);
            const bool flatB = CubicBezier::IsGeometricallyFlat(pair.second, tolerance);

            if ((flatA && flatB) || pair.depth >= 24 || top + 2 > 62)
            {
                const double candidate = SegmentSegmentSquared(
                    pair.first.p0, pair.first.p1, pair.second.p0, pair.second.p1);

                if (candidate < best)
                {
                    best = candidate;

                    // This pair now owns the answer, so the answer inherits ITS error.
                    // A pair accepted because both halves were flat contributes nothing
                    // beyond the 2*tolerance the caller already assumes; a pair accepted
                    // at the depth or stack cap was never tested for flatness, so its
                    // real deviation has to travel with the result.
                    bestCapDeviation = (flatA && flatB)
                        ? 0.0
                        : PieceDeviationBound(pair.first) + PieceDeviationBound(pair.second);
                }

                if (best <= toleranceSquared)
                {
                    if (subdivisionsOut != nullptr) *subdivisionsOut += subdivisions;
                    if (capDeviationOut != nullptr) *capDeviationOut =
                        std::max(*capDeviationOut, bestCapDeviation);
                    return best;
                }
                continue;
            }

            Segment left, right;
            Pair children[2];
            ++subdivisions;

            if (!flatA && (flatB || pair.boxFirst.Perimeter() >= pair.boxSecond.Perimeter()))
            {
                // Splitting A: B is unchanged, so its box is carried over rather than
                // recomputed.
                CubicBezier::Split(pair.first, 0.5, left, right);
                const double mid = 0.5 * (pair.a0 + pair.a1);
                children[0] = { left,  pair.second, boxA(pair.a0, mid),
                                pair.boxSecond, 0.0, pair.depth + 1,
                                pair.a0, mid, pair.b0, pair.b1 };
                children[1] = { right, pair.second, boxA(mid, pair.a1),
                                pair.boxSecond, 0.0, pair.depth + 1,
                                mid, pair.a1, pair.b0, pair.b1 };
            }
            else
            {
                CubicBezier::Split(pair.second, 0.5, left, right);
                const double mid = 0.5 * (pair.b0 + pair.b1);
                children[0] = { pair.first, left,  pair.boxFirst,
                                boxB(pair.b0, mid),  0.0, pair.depth + 1,
                                pair.a0, pair.a1, pair.b0, mid };
                children[1] = { pair.first, right, pair.boxFirst,
                                boxB(mid, pair.b1), 0.0, pair.depth + 1,
                                pair.a0, pair.a1, mid, pair.b1 };
            }

            for (Pair& child : children)
                child.lowerBoundSquared = child.boxFirst.DistanceSquared(child.boxSecond);

            // Nearest first. A LIFO that descends in arbitrary order reaches a good
            // best late, and everything before that point is explored unpruned - which
            // is precisely why one curve-curve pair cost 762 us. Pushing the farther
            // half first makes the nearer half pop first and tighten the bound for it.
            const int nearer = children[0].lowerBoundSquared <= children[1].lowerBoundSquared ? 0 : 1;
            const int farther = 1 - nearer;

            if (children[farther].lowerBoundSquared < best)
                stack[top++] = children[farther];
            if (children[nearer].lowerBoundSquared < best)
                stack[top++] = children[nearer];
        }

        // Both exits publish, not just the early one: a search that drains the stack can
        // still have accepted a capped pair on the way.
        if (subdivisionsOut != nullptr) *subdivisionsOut += subdivisions;
        if (capDeviationOut != nullptr)
            *capDeviationOut = std::max(*capDeviationOut, bestCapDeviation);
        return best;
    }

    namespace
    {
        // ONE PAIR ON THE FRONTIER, named by PARAMETER INTERVALS of the two ORIGINAL
        // curves rather than by two subcurves.
        //
        // This is the whole F31 fix, and it is a change of REFERENCE, not of arithmetic.
        // The old node held a Segment produced by Split(Split(Split(...))), and each
        // rounding composed with the next; measured, the resulting ExactBounds excluded
        // points of the original curve in 4675 of 391 374 samples (evidence 1156). A node
        // that carries [t0, t1] instead is enclosed by a blossom of fixed depth three from
        // the original control points, so nothing composes with depth.
        //
        // `pa`/`pb` carry the certified boxes, the certified endpoint and midpoint boxes
        // that witnesses are taken from, and a nominal Segment for heuristics only.
        struct BoundedPair
        {
            double a0 = 0.0;
            double a1 = 1.0;
            double b0 = 0.0;
            double b1 = 1.0;
            CubicBezier::CertifiedPiece pa;
            CubicBezier::CertifiedPiece pb;
            double lowerSquared = 0.0;
        };

        // A LOWER bound on the squared distance between two certified boxes, rounded DOWN
        // at every step. Lemma 2 turns this into a lower bound on the distance between the
        // curves; rounding it down is what makes Lemma 2 survive IEEE-754.
        [[nodiscard]] double CertifiedGapSquared(const Bounds2& x, const Bounds2& y) noexcept
        {
            const double dx = y.min.x > x.max.x ? Certified::Down(y.min.x - x.max.x)
                            : (x.min.x > y.max.x ? Certified::Down(x.min.x - y.max.x) : 0.0);
            const double dy = y.min.y > x.max.y ? Certified::Down(y.min.y - x.max.y)
                            : (x.min.y > y.max.y ? Certified::Down(x.min.y - y.max.y) : 0.0);

            const double sx = dx > 0.0 ? Certified::Down(dx * dx) : 0.0;
            const double sy = dy > 0.0 ? Certified::Down(dy * dy) : 0.0;
            const double sum = Certified::Down(sx + sy);
            return sum > 0.0 ? sum : 0.0;
        }

        // An UPPER bound on the squared distance between ANY point of one box and ANY
        // point of the other, rounded UP. Used on the certified box of a single curve
        // POINT, where it says: wherever that point really is, it is no further than this.
        [[nodiscard]] double CertifiedFarSquared(const Bounds2& x, const Bounds2& y) noexcept
        {
            double dx = Certified::Up(x.max.x - y.min.x);
            const double dxOther = Certified::Up(y.max.x - x.min.x);
            if (dxOther > dx) dx = dxOther;
            if (dx < 0.0) dx = 0.0;

            double dy = Certified::Up(x.max.y - y.min.y);
            const double dyOther = Certified::Up(y.max.y - x.min.y);
            if (dyOther > dy) dy = dyOther;
            if (dy < 0.0) dy = 0.0;

            return Certified::Up(Certified::Up(dx * dx) + Certified::Up(dy * dy));
        }

        // AN UPPER BOUND WITNESSED BY POINTS THAT REALLY ARE ON BOTH CURVES.
        //
        // The old form evaluated the SUBCURVE at three parameters, and a subcurve computed
        // through repeated splitting does not pass exactly through the points of the curve
        // it stands for - so those witnesses were very slightly off the curve, in an
        // unexamined direction. These boxes enclose points of the ORIGINAL curve, and the
        // far corner of two boxes bounds the distance between whatever points they really
        // hold. Lemma 1 then applies without a tolerance term: an infimum is at most any
        // element of the set it is taken over.
        [[nodiscard]] double CertifiedWitness(const CubicBezier::CertifiedPiece& pa,
                                              const CubicBezier::CertifiedPiece& pb) noexcept
        {
            const Bounds2* const sa[3] = { &pa.startBox, &pa.midBox, &pa.endBox };
            const Bounds2* const sb[3] = { &pb.startBox, &pb.midBox, &pb.endBox };

            double best = std::numeric_limits<double>::infinity();
            for (int i = 0; i < 3; ++i)
            {
                for (int j = 0; j < 3; ++j)
                    best = (std::min)(best, CertifiedFarSquared(*sa[i], *sb[j]));
            }
            return Certified::Up(std::sqrt(best));
        }

        // Splits the parameter interval of the piece whose box is larger.
        //
        // RETURNS FALSE FOR AN ATOMIC NODE. When no double lies strictly between t0 and
        // t1 the interval cannot be halved, and a solver that kept trying would not
        // terminate. The caller retires such a node with the bound it already has.
        //
        // The two children share the SINGLE double `mid`, so [t0, mid] and [mid, t1] cover
        // [t0, t1] exactly - there is no rounding gap between siblings to widen away
        // (lemma 7).
        [[nodiscard]] bool SplitLarger(const Segment& a, const Segment& b,
                                       const BoundedPair& pair, BoundedPair out[2]) noexcept
        {
            const bool firstIsLarger = pair.pa.box.Perimeter() >= pair.pb.box.Perimeter();

            for (int attempt = 0; attempt < 2; ++attempt)
            {
                const bool splitFirst = ((attempt == 0) == firstIsLarger);
                const double t0 = splitFirst ? pair.a0 : pair.b0;
                const double t1 = splitFirst ? pair.a1 : pair.b1;
                const double mid = 0.5 * (t0 + t1);
                if (!(mid > t0 && mid < t1))
                    continue;                       // atomic on this side; try the other

                for (int k = 0; k < 2; ++k)
                {
                    out[k] = pair;
                    const double lo = (k == 0) ? t0 : mid;
                    const double hi = (k == 0) ? mid : t1;
                    if (splitFirst)
                    {
                        out[k].a0 = lo;
                        out[k].a1 = hi;
                        out[k].pa = CubicBezier::CertifiedSubCurve(a, lo, hi);
                    }
                    else
                    {
                        out[k].b0 = lo;
                        out[k].b1 = hi;
                        out[k].pb = CubicBezier::CertifiedSubCurve(b, lo, hi);
                    }
                    out[k].lowerSquared = CertifiedGapSquared(out[k].pa.box, out[k].pb.box);
                }
                return true;
            }
            return false;
        }
        // ONE PLACE WHERE A PUBLISHED INTERVAL IS ASSEMBLED.
        //
        // CurveCurveBounds has seven exits. Before this type existed each of them built
        // `lower`, `upper` and `converged` on its own, and five of the seven got it wrong
        // in a different way:
        //
        //   A  an exit published `abandoned.empty() ? upper : frontierLower`, claiming a
        //      zero-width interval the search had never proved;
        //   B  the caller pruning seed was folded into `upper`, so a bound that witnessed
        //      nothing became the witnessed upper bound;
        //   C  two exits consulted `width` while DECIDING, putting the refinement target -
        //      and therefore PrecisionMode - inside a threshold answer;
        //   D  the flatness terminal deposited a deviation larger than the one the
        //      flatness test had proved;
        //   E  a piece handed to the certified frontier deposited its PROVISIONAL parent
        //      bound into a running minimum, which never rises again.
        //
        // Every one of those is the same missing invariant, not five separate slips:
        //
        //     `upper` is the minimum over WITNESSED point-pair distances, and nothing else.
        //     `lower` is the minimum over bounds of pieces that have LEFT the search,
        //             plus whatever the live frontier still guarantees - and never a
        //             provisional bound, never a borrowed one.
        //     `converged` describes the PUBLISHED interval, and no other quantity.
        //     `width` is invisible to a decision.
        //
        // Enforcing that per call site is how a sixth exit gets it wrong. Here the rules
        // have exactly one implementation: contributions go in through named methods that
        // say what was proved, and Publish() is the only thing that can produce a
        // DistanceBounds.
        class IntervalLedger
        {
        public:
            IntervalLedger(double width, bool deciding, double decideAt) noexcept
                : width_(width), decideAt_(decideAt), deciding_(deciding) {}

            // THE ONLY WAY `upper` MOVES. Takes a distance between two points that lie
            // exactly ON the curves, so it carries no tolerance term (Lemma 1). A pruning
            // bound is not a witness and has no path to this.
            void Witness(double distance) noexcept
            {
                if (distance < upper_)
                    upper_ = distance;
            }

            // A piece that has LEFT the search, with the bound it proved on the way out.
            // A piece still on the frontier, or one queued for refinement, does not
            // belong here - its children will carry strictly better bounds.
            void Retired(double lower, DistanceBounds::LowerSource why) noexcept
            {
                if (lower < retired_)
                {
                    retired_ = lower;
                    why_ = why;
                }
            }

            // A piece discarded against the caller bound. Its true distance is at least
            // that bound, so the bound is a valid lower bound for everything discarded
            // this way - and it is tracked apart from `retired_` so the two can be told
            // apart in a diagnostic.
            void PrunedBySeed(double seed) noexcept
            {
                if (seed < seedFloor_)
                    seedFloor_ = seed;
            }

            [[nodiscard]] double Upper() const noexcept { return upper_; }

            [[nodiscard]] bool Decided(double liveLower) const noexcept
            {
                return deciding_ && (upper_ <= decideAt_ || liveLower > decideAt_);
            }

            // A piece leaves the frontier when nothing the caller asked for can still be
            // learned from it: narrow enough against the running upper bound when
            // measuring, provably beyond the threshold when deciding. `width` never
            // participates in the deciding branch.
            [[nodiscard]] bool LeavesFrontier(double lower) const noexcept
            {
                return deciding_ ? (lower > decideAt_) : (upper_ - lower <= width_);
            }

            [[nodiscard]] bool Deciding() const noexcept { return deciding_; }
            [[nodiscard]] double Width() const noexcept { return width_; }

            // THE ONLY WAY A DistanceBounds IS PRODUCED.
            //
            // `liveLower` is what the not-yet-finished part of the search still
            // guarantees: the smallest lower bound among pieces that are queued but
            // unrefined. Infinity means nothing is live. Passing it explicitly is what
            // makes every exit state its own remaining uncertainty instead of silently
            // omitting it.
            // `status` has no default, for the same reason `liveLower` is a parameter:
            // every exit has to say how it ended. A defaulted Success would let a
            // cancelled exit publish silently and look finished.
            [[nodiscard]] DistanceBounds Publish(double liveLower, std::uint64_t subdivisions,
                                                 bool usedFallback,
                                                 GeometryStatus status) const noexcept
            {
                DistanceBounds out;
                out.status = status;
                out.subdivisions = subdivisions;
                out.usedFallback = usedFallback;
                out.upper = upper_;
                out.frontierLower = retired_;
                out.seedFloor = seedFloor_;

                double lower = retired_;
                DistanceBounds::LowerSource source = why_;
                if (seedFloor_ < lower)
                {
                    lower = seedFloor_;
                    source = DistanceBounds::LowerSource::SeedFloor;
                }
                if (liveLower < lower)
                {
                    lower = liveLower;
                    source = DistanceBounds::LowerSource::FallbackDecided;
                }

                if (!(lower <= upper_))
                {
                    // Nothing live and nothing retired below the witness: every piece was
                    // pruned because it could not hold anything smaller, so the interval
                    // has collapsed onto the witness.
                    lower = upper_;
                    source = DistanceBounds::LowerSource::CollapsedToUpper;
                }
                if (lower < 0.0)
                    lower = 0.0;

                out.lower = lower;
                out.lowerSource = source;
                out.converged = deciding_ ? (upper_ <= decideAt_ || lower > decideAt_)
                                          : (upper_ - lower <= width_);
                return out;
            }

        private:
            double upper_ = std::numeric_limits<double>::infinity();
            double retired_ = std::numeric_limits<double>::infinity();
            double seedFloor_ = std::numeric_limits<double>::infinity();
            double width_;
            double decideAt_;
            bool deciding_;
            DistanceBounds::LowerSource why_ = DistanceBounds::LowerSource::FrontierRetired;
        };

    }

    // NOT noexcept, deliberately - Flatten.cpp:92 says the same thing for the same
    // reason. `abandoned` and `frontier` below are heap containers, so an allocation here
    // can throw, and a throwing allocation inside a noexcept frame is std::terminate.
    // That is not a hypothesis: F27 armed the allocator to fail inside this function and
    // the child process died with 0xC0000409 without ever reaching its own print
    // statement (evidence 1144), while CERTIFIED_CURVE_DISTANCE_SOLVER.md section 6
    // promised OutOfMemory.
    //
    // The public entry point below owns the translation. Static, because nothing outside
    // this file may call the unguarded form.
    static DistanceBounds SolveCurveCurve(const Segment& a, const Segment& b,
                                          const DistanceSolveControl& control,
                                          const GeometryContext& context)
    {
        // Named locally so the body reads as the mathematics it is. The GROUPING in
        // DistanceSolveControl is what carries the hard/soft distinction; once past this
        // point they are numbers again, and only `maxWorkingBytes` is allowed to fail.
        const double targetWidth = control.targetWidth;
        const double upperSeed = control.upperSeed;
        const std::uint64_t workLimit = control.refinementBudget;
        const double decideAt = control.decideAt;

        // COOPERATIVE CANCELLATION, at the kernel's own granularity.
        //
        // GeometryContext::kCancellationBlock is 4096 items, and the outer traversal
        // already honours it BETWEEN candidate pairs. What was missing was any check
        // WITHIN one call - F28. Measured, one call costs at most 504 subdivisions on the
        // adversarial fixtures (evidence 1152), and the check at step zero means the
        // caller's wait is bounded by one call's work: a number that has been measured,
        // rather than a granularity that was promised.
        std::size_t steps = 0;
        const auto stopRequested = [&]() noexcept
        {
            return context.ShouldCheckCancellation(steps++) && context.IsCancelled();
        };

        // The only HARD ceiling in this function. Compare with `workLimit` above, which
        // is soft and answers with a wider interval; this one answers with
        // ComplexityLimit because the caller set a contract that cannot be honoured.
        const auto exceedsWorkingSet = [&](std::size_t pairs) noexcept
        {
            return control.maxWorkingBytes != 0 &&
                   pairs * sizeof(BoundedPair) > control.maxWorkingBytes;
        };

        // A THRESHOLD QUERY IS A DECISION, NOT A MEASUREMENT. Once the whole interval sits
        // on one side of decideAt the answer cannot change, and narrowing further is work
        // nobody asked for. This is what keeps DistanceLessThan affordable when the caller
        // asks for the analytic tolerance: converging to 1e-9 mm on a near-tangential pair
        // can need 2^30 pieces, while deciding it against a 1.2 mm threshold takes a
        // handful.
        const bool deciding = std::isfinite(decideAt);
        const double width = targetWidth > 0.0 ? targetWidth : 0.0;

        // Two lines are solved in closed form. The interval is as narrow as the closed
        // form can honestly make it, which is NOT a point.
        //
        // SegmentSegmentSquared returns the squared distance between two points it
        // computed on the two segments: a0 + d1*s and b0 + d2*t. Any s and t give points
        // ON the segments, so the parameters' own error cannot invalidate the value - but
        // the coordinate differences, products and square root that form it are rounded,
        // and this exit published the result as an exact lower AND upper bound at once.
        // That is the F31 class in the shortest routine in the file, so it gets the same
        // treatment: sixteen operations counted against a chain of nine, and an interval
        // a hair wide instead of one that is falsely exact.
        if (a.IsLine() && b.IsLine())
        {
            const double exact = std::sqrt(SegmentSegmentSquared(a.p0, a.p1, b.p0, b.p1));
            DistanceBounds out;
            out.lower = Certified::WidenDown(exact, 16);
            out.upper = Certified::WidenUp(exact, 16);
            if (out.lower < 0.0)
                out.lower = 0.0;
            out.converged = (out.upper - out.lower) <= width;
            out.lowerSource = DistanceBounds::LowerSource::LineLineClosedForm;
            return out;
        }

        IntervalLedger ledger(width, deciding, decideAt);

        BoundedPair root;
        root.a0 = 0.0;
        root.a1 = 1.0;
        root.b0 = 0.0;
        root.b1 = 1.0;
        root.pa = CubicBezier::CertifiedSubCurve(a, 0.0, 1.0);
        root.pb = CubicBezier::CertifiedSubCurve(b, 0.0, 1.0);
        root.lowerSquared = CertifiedGapSquared(root.pa.box, root.pb.box);

        ledger.Witness(CertifiedWitness(root.pa, root.pb));

        // The caller bound PRUNES; it is never a witness. A piece discarded because its
        // lower bound reached the seed has true distance at least the seed, which is what
        // the ledger records when that happens.
        const double pruneSeed = (upperSeed >= 0.0) ? upperSeed
                                                    : std::numeric_limits<double>::infinity();

        // Half the target for each piece, because a pair carries the SUM of the two
        // deviations and the interval it produces has to meet `width`.
        const double flatTolerance = width * 0.5;

        std::uint64_t subdivisions = 0;

        // THE FLAT TERMINAL, WITH BOTH HALVES CERTIFIED.
        //
        // WHY IT CANNOT BE DROPPED, measured rather than assumed. The first version of
        // this stage kept only the witness and left the lower bound to the certified box,
        // on the reasoning that a flat piece has a box that is a thin sliver around its
        // chord. That reasoning is wrong: the box is an AABB, and the AABB of a thin
        // DIAGONAL sliver is a large square. Two such boxes have a gap far below the
        // distance between the chords inside them, so the interval stopped narrowing -
        // measured, a pair whose answer is 1.1589 stalled at a width of 2.7e-06 against a
        // 1e-08 target, and the fuzz converged on 1670 of 3000 pairs instead of 2700.
        // Escaping the AABB's weakness on diagonal pieces is exactly what this terminal is
        // for.
        //
        // BOTH HALVES OF THE SAME PROVEN INEQUALITY. |chord - true| <= deviation gives
        // true >= chord - deviation AND true <= chord + deviation. The upper half matters
        // most where the answer is smallest: two curves that CROSS have true distance 0
        // and their chords cross too, so it collapses the interval at once while sampling
        // would have to localise the crossing by subdivision.
        //
        // WHAT F31 CHANGES HERE. Every term is now widened in the direction that keeps it
        // a bound:
        //
        //   chord       SegmentSegmentSquared returns the distance between two points it
        //               COMPUTED on the two chords. Any s and t give points ON the chords,
        //               so its own parameter error cannot invalidate it; what is rounded
        //               is the nine operations that form the two points and their
        //               distance. Sixteen counted against nine, both directions.
        //
        //   deviation   FlatnessMetric and the flatness test are a chain of about a dozen
        //               operations. Thirty-two counted against twelve, upward.
        //
        //   slack       the nominal piece is the midpoints of the certified control
        //               intervals, so it is within `spread` per axis of the true piece -
        //               hence within 2*spread in length, since the Bernstein weights are
        //               non-negative and sum to one. The chord's endpoints are control
        //               points, so they move by at most `spread` too. Three times the
        //               spread covers curve, chord and their sum.
        const auto tryFlatTerminal = [&](const BoundedPair& pair, double pairLower) noexcept
        {
            if (deciding)
                return false;
            if (!CubicBezier::IsGeometricallyFlat(pair.pa.nominal, flatTolerance) ||
                !CubicBezier::IsGeometricallyFlat(pair.pb.nominal, flatTolerance))
            {
                return false;
            }

            const Segment& na = pair.pa.nominal;
            const Segment& nb = pair.pb.nominal;
            const double chord = std::sqrt(
                SegmentSegmentSquared(na.p0, na.p1, nb.p0, nb.p1));

            // The tighter of two valid deviation bounds. IsGeometricallyFlat proves
            // deviation <= its tolerance; PieceDeviationBound is a second, deliberately
            // conservative bound from FlatnessMetric and can be far larger - pairing the
            // terminal with it alone published 3.0e-05 of deviation on a piece that had
            // passed the test at 1e-07.
            const double deviation = Certified::WidenUp(
                (std::min)(PieceDeviationBound(na), flatTolerance) +
                (std::min)(PieceDeviationBound(nb), flatTolerance), 32);

            const double slack =
                Certified::Up(3.0 * (pair.pa.spread + pair.pb.spread));

            ledger.Witness(
                Certified::WidenUp(Certified::Up(chord + deviation + slack), 16));

            double resolved = Certified::Down(
                Certified::WidenDown(chord, 16) - deviation - slack);
            if (resolved < pairLower)
                resolved = pairLower;
            if (resolved < 0.0)
                resolved = 0.0;
            ledger.Retired(resolved, DistanceBounds::LowerSource::FrontierFlat);
            return true;
        };

        // FAST PATH: fixed stack, no allocation. It differs from the scalar solver in one
        // respect - a piece it cannot resolve is not converted into a number, it is handed
        // to the certified frontier with its own rigorous lower bound.
        BoundedPair stack[64];
        int depth[64] = { 0 };
        int top = 0;
        depth[top] = 0;
        stack[top++] = root;

        std::vector<BoundedPair> abandoned;

        // Bounds of pieces handed to the certified frontier. PROVISIONAL: the fallback
        // replaces each of them with its children, so they are not given to the ledger as
        // retired. They are published only through the live-frontier argument of the exit
        // that returns without running the fallback.
        double abandonedLower = std::numeric_limits<double>::infinity();

        // THE LIVE FLOOR OF THE FAST PATH, and the reason it is a function rather than
        // the popped pair's own bound. F33.
        //
        // `Publish` takes what everything NOT YET FINISHED still guarantees. On the
        // certified frontier that is free: the priority queue is ordered by lower bound,
        // so the pair just popped IS the minimum, and the comment at that call site says
        // so. The fixed stack is NOT that. It is a LIFO visited nearest-child-first, and
        // splitting only ever RAISES a bound, so when a deep piece is popped the shallow
        // "farther" siblings left behind carry SMALLER bounds.
        //
        // Passing the popped piece's own bound instead claimed it covered them. Measured
        // over a threshold sweep: 666 of 1440 decided intervals published a lower bound
        // ABOVE the true distance, by as much as 1.52 mm - not ULPs (evidence 1167).
        //
        // Squared throughout, one sqrt at the end, over `top` entries. A depth-24 DFS that
        // pushes two and pops one keeps `top` at about 25, and the scan runs only where a
        // decision is actually plausible.
        const auto liveFloor = [&](double currentLower) noexcept
        {
            double floorSquared = std::numeric_limits<double>::infinity();
            for (int i = 0; i < top; ++i)
            {
                if (stack[i].lowerSquared < floorSquared)
                    floorSquared = stack[i].lowerSquared;
            }

            double floor = currentLower;
            if (floorSquared < std::numeric_limits<double>::infinity())
            {
                // ROUNDED DOWN, like every other square root of a certified box gap in
                // this function: the root is not exact and the quantity has to stay a
                // lower bound.
                const double fromStack = Certified::Down(std::sqrt(floorSquared));
                if (fromStack < floor)
                    floor = fromStack;
            }
            if (abandonedLower < floor)
                floor = abandonedLower;
            return floor;
        };

        while (top > 0)
        {
            --top;
            const int currentDepth = depth[top];
            const BoundedPair pair = stack[top];

            // ROUNDED DOWN. The box gap is a lower bound and its square root is not
            // exact, so the certified quantity is the rounded-down root, not the root.
            const double pairLower = Certified::Down(std::sqrt(pair.lowerSquared));

            // Answering `Cancelled` with the interval proved so far, not with a failure
            // that throws the work away. The interval is still true; it is just wider
            // than the caller asked for, which is exactly what a soft budget overrun
            // produces too.
            if (stopRequested())
                return ledger.Publish(liveFloor(pairLower), subdivisions, false,
                                      GeometryStatus::Cancelled);

            if (pairLower >= ledger.Upper())
                continue;                        // Lemma 4: cannot hold the minimum
            if (pairLower >= pruneSeed)
            {
                ledger.PrunedBySeed(pruneSeed);
                continue;
            }

            ledger.Witness(CertifiedWitness(pair.pa, pair.pb));

            // THE FLOOR, NOT THIS PIECE'S OWN BOUND - see liveFloor above.
            //
            // The guard is exact rather than an optimisation: floor <= pairLower always,
            // so Decided(floor) can only be true when the witness has already crossed the
            // threshold or when this piece itself sits beyond it. Testing that first keeps
            // the scan off every pop of a search that is nowhere near deciding.
            if (ledger.Deciding() && (ledger.Upper() <= decideAt || pairLower > decideAt))
            {
                const double floor = liveFloor(pairLower);
                if (ledger.Decided(floor))
                    return ledger.Publish(floor, subdivisions, false, GeometryStatus::Success);
            }

            if (tryFlatTerminal(pair, pairLower))
                continue;

            if (ledger.LeavesFrontier(pairLower))
            {
                ledger.Retired(pairLower, DistanceBounds::LowerSource::FrontierRetired);
                continue;
            }

            if (currentDepth >= 24 || top + 2 > 62)
            {
                // ABANDONED, NOT ACCEPTED, and NOT RETIRED. V8.1.1 turned this into a
                // chord distance plus a deviation attached to whichever pair happened to
                // own the running best. Depositing the parent bound in the ledger instead
                // made that weaker number permanent, because a running minimum never rises
                // - measured at 2.606e-04 published against a 1e-07 target, unchanged by a
                // hundredfold work budget, because the number was fixed BEFORE the
                // fallback ever ran (evidence 1093).
                if (exceedsWorkingSet(abandoned.size() + 1))
                {
                    return ledger.Publish(liveFloor(pairLower), subdivisions, false,
                                          GeometryStatus::ComplexityLimit);
                }
                abandoned.push_back(pair);
                abandonedLower = (std::min)(abandonedLower, pairLower);
                continue;
            }

            BoundedPair children[2];
            if (!SplitLarger(a, b, pair, children))
            {
                // ATOMIC. Neither parameter interval has a double strictly inside it any
                // more, so there is nothing left to halve. The pair leaves the search with
                // the certified bound it already has; pretending it could be refined would
                // not terminate.
                ledger.Retired(pairLower, DistanceBounds::LowerSource::FrontierRetired);
                continue;
            }
            ++subdivisions;

            const int nearer = children[0].lowerSquared <= children[1].lowerSquared ? 0 : 1;
            const int farther = 1 - nearer;

            for (const int which : { farther, nearer })
            {
                const double childLower =
                    Certified::Down(std::sqrt(children[which].lowerSquared));
                if (childLower >= ledger.Upper())
                    continue;
                if (childLower >= pruneSeed)
                {
                    ledger.PrunedBySeed(pruneSeed);
                    continue;
                }
                depth[top] = currentDepth + 1;
                stack[top++] = children[which];
            }
        }

        if (abandoned.empty())
            return ledger.Publish(std::numeric_limits<double>::infinity(), subdivisions, false,
                                  GeometryStatus::Success);

        {
            // This exit returns WITHOUT refining the abandoned pieces, so their
            // provisional bounds are the only thing known about them and are declared as
            // the live frontier. `width` is deliberately absent when deciding: a threshold
            // answer that depended on the refinement target would depend on PrecisionMode,
            // and Production - whose width is the coarse operand deviation - exited here
            // with [1e-06, 0.0064] and answered "not within 1e-06" on a pair 2.5e-08 apart,
            // a FALSE NEGATIVE (evidence 1105).
            const DistanceBounds early =
                ledger.Publish(abandonedLower, subdivisions, false, GeometryStatus::Success);
            if (!deciding && early.converged)
                return early;
        }

        // SLOW CERTIFIED FALLBACK. Dynamic frontier, no depth cap, smallest lower bound
        // first - that is the piece which DECIDES the global lower bound, so refining
        // anything else spends work where the interval is already settled.
        //
        // A HEAP, NOT A SCAN. Picking the minimum by walking the frontier is quadratic in
        // the number of live pieces, and on a near-tangential pair the frontier grows into
        // the thousands.
        struct ByLower
        {
            [[nodiscard]] bool operator()(const BoundedPair& x, const BoundedPair& y) const noexcept
            {
                return x.lowerSquared > y.lowerSquared;   // priority_queue is a max heap
            }
        };

        std::priority_queue<BoundedPair, std::vector<BoundedPair>, ByLower> frontier(
            ByLower{}, std::move(abandoned));

        while (!frontier.empty())
        {
            const BoundedPair pair = frontier.top();
            frontier.pop();

            const double pairLower = Certified::Down(std::sqrt(pair.lowerSquared));

            if (stopRequested())
                return ledger.Publish(pairLower, subdivisions, true, GeometryStatus::Cancelled);

            if (pairLower >= ledger.Upper())
                continue;
            if (pairLower >= pruneSeed)
            {
                ledger.PrunedBySeed(pruneSeed);
                continue;
            }

            ledger.Witness(CertifiedWitness(pair.pa, pair.pb));

            // This pair holds the smallest lower bound on the frontier, so it IS the live
            // frontier bound and everything still queued sits at or above it. That is a
            // property of the HEAP, not of the search, which is why the fixed stack above
            // needs liveFloor() and this does not.
            {
                const DistanceBounds candidate =
                    ledger.Publish(pairLower, subdivisions, true, GeometryStatus::Success);
                if (candidate.converged)
                    return candidate;
            }

            if (tryFlatTerminal(pair, pairLower))
                continue;

            if (ledger.LeavesFrontier(pairLower))
            {
                ledger.Retired(pairLower, DistanceBounds::LowerSource::FrontierRetired);
                continue;
            }

            if (workLimit != 0 && subdivisions >= workLimit)
            {
                // Reported as a WIDER INTERVAL, never as a finished number. A resource
                // contract may bound the work; it may not bound the honesty.
                DistanceBounds out =
                    ledger.Publish(pairLower, subdivisions, true, GeometryStatus::Success);
                out.converged = false;
                out.lowerSource = DistanceBounds::LowerSource::FallbackWorkLimit;
                return out;
            }

            BoundedPair children[2];
            if (!SplitLarger(a, b, pair, children))
            {
                ledger.Retired(pairLower, DistanceBounds::LowerSource::FrontierRetired);
                continue;
            }
            ++subdivisions;

            for (const BoundedPair& child : children)
            {
                const double childLower = Certified::Down(std::sqrt(child.lowerSquared));
                if (childLower >= ledger.Upper())
                    continue;
                if (childLower >= pruneSeed)
                {
                    ledger.PrunedBySeed(pruneSeed);
                    continue;
                }
                if (exceedsWorkingSet(frontier.size() + 1))
                {
                    return ledger.Publish(pairLower, subdivisions, true,
                                          GeometryStatus::ComplexityLimit);
                }
                frontier.push(child);
            }
        }

        // The frontier drained: every remaining piece was pruned because it could not hold
        // anything smaller than a bound already held, so nothing is live.
        return ledger.Publish(std::numeric_limits<double>::infinity(), subdivisions, true,
                              GeometryStatus::Success);
    }

    DistanceBounds CurveCurveBounds(const Segment& a, const Segment& b,
                                    const DistanceSolveControl& control,
                                    const GeometryContext& context)
    {
        try
        {
            return SolveCurveCurve(a, b, control, context);
        }
        catch (const std::bad_alloc&)
        {
            // Exhaustion is a status, not a crash - the same sentence Flatten.cpp:249
            // already carries, and the same reason OutOfMemory is distinct from
            // ComplexityLimit: the limits were satisfied and the machine still could not
            // provide the memory.
            //
            // THE INTERVAL RETURNED CLAIMS NOTHING. [0, infinity) is true for every pair
            // of curves, so a caller that ignores the status is made conservative rather
            // than wrong: DistanceLessThan falls into its ambiguous band and answers the
            // safe `true`, and MinimumDistance widens. Publishing the partial interval
            // instead would be tempting and unsound - the frame that held it is gone.
            DistanceBounds out;
            out.status = GeometryStatus::OutOfMemory;
            out.lower = 0.0;
            out.upper = std::numeric_limits<double>::infinity();
            out.converged = false;
            out.lowerSource = DistanceBounds::LowerSource::None;
            return out;
        }
    }

    ClosestResult ClosestOnPolyline(Vec2 point, const Vec2* ring, std::size_t count, bool closed) noexcept
    {
        ClosestResult best;
        best.distanceSquared = std::numeric_limits<double>::infinity();

        if (ring == nullptr || count == 0)
            return best;

        if (count == 1)
        {
            best.point = ring[0];
            best.distanceSquared = DistanceSquared(point, ring[0]);
            return best;
        }

        const std::size_t limit = closed ? count : count - 1;
        for (std::size_t i = 0; i < limit; ++i)
        {
            const ClosestResult candidate = ClosestOnSegment(point, ring[i], ring[(i + 1) % count]);
            if (candidate.distanceSquared < best.distanceSquared)
            {
                best = candidate;
                best.index = i;
            }
        }

        return best;
    }

    ClosestResult ClosestOnContour(Vec2 point, const Contour& contour, const GeometryContext& context)
    {
        ClosestResult best;
        best.distanceSquared = std::numeric_limits<double>::infinity();

        std::vector<Segment> segments;
        if (!GatherSegments(contour, segments) || segments.empty())
            return best;

        SegmentBVH bvh;
        bvh.BuildFromSegments(segments.data(), segments.size(), context);

        // The hierarchy hands back candidates nearest-box-first and prunes on the best
        // distance found so far, so most segments are never touched.
        bvh.QueryNearest(point, std::numeric_limits<double>::infinity(),
                         [&](std::uint32_t index, double) noexcept
                         {
                             const Segment& segment = segments[index];
                             double candidate;
                             double parameter = 0.0;
                             Vec2 location;

                             if (segment.IsLine())
                             {
                                 const ClosestResult onSegment =
                                     ClosestOnSegment(point, segment.p0, segment.p1);
                                 candidate = onSegment.distanceSquared;
                                 parameter = onSegment.parameter;
                                 location = onSegment.point;
                             }
                             else
                             {
                                 const auto onCurve = CubicBezier::ClosestPoint(segment, point);
                                 candidate = onCurve.distanceSquared;
                                 parameter = onCurve.parameter;
                                 location = onCurve.point;
                             }

                             if (candidate < best.distanceSquared)
                             {
                                 best.distanceSquared = candidate;
                                 best.parameter = parameter;
                                 best.point = location;
                                 best.index = index;
                             }
                             return best.distanceSquared;
                         });

        return best;
    }

    bool PointWithin(Vec2 point, const Contour& contour, double threshold, const GeometryContext& context)
    {
        std::vector<Segment> segments;
        if (!GatherSegments(contour, segments) || segments.empty())
            return false;

        const double thresholdSquared = threshold * threshold;

        SegmentBVH bvh;
        bvh.BuildFromSegments(segments.data(), segments.size(), context);

        // Only segments whose bounds already fall inside the threshold can qualify, and
        // the first qualifying one ends the search.
        bool found = false;
        bvh.QueryBounds(Bounds2::FromPoints(point, point).Expanded(threshold),
                        [&](std::uint32_t index) noexcept
                        {
                            if (PointCurveSquared(point, segments[index]) <= thresholdSquared)
                            {
                                found = true;
                                return false;
                            }
                            return true;
                        });

        return found;
    }

    bool ContoursWithin(const Contour& a, const Contour& b, double threshold,
                        const GeometryContext& context)
    {
        std::vector<Segment> segmentsA;
        std::vector<Segment> segmentsB;
        if (!GatherSegments(a, segmentsA) || !GatherSegments(b, segmentsB))
            return false;

        if (segmentsA.empty() || segmentsB.empty())
            return false;

        SegmentBVH bvhA;
        SegmentBVH bvhB;
        bvhA.BuildFromSegments(segmentsA.data(), segmentsA.size(), context);
        bvhB.BuildFromSegments(segmentsB.data(), segmentsB.size(), context);

        // Fixed pruning radius: any pair whose boxes are already farther apart than the
        // threshold cannot satisfy it, and the first pair that does ends the query.
        const double thresholdSquared = threshold * threshold;
        const double limit = thresholdSquared > 0.0 ? std::nextafter(thresholdSquared, 1e308)
                                                    : std::numeric_limits<double>::min();
        const double tolerance = context.Tolerance().intersection;

        bool found = false;
        PrunedPairs(bvhA, bvhB, limit, [&](std::uint32_t i, std::uint32_t j) noexcept
        {
            // `limit` is one ulp above the threshold, so a pair exactly AT the threshold
            // is still resolved rather than pruned, and the <= below keeps its meaning.
            if (CurveCurveSquared(segmentsA[i], segmentsB[j], tolerance, limit) <= thresholdSquared)
            {
                found = true;
                return false;
            }
            return true;
        });

        return found;
    }

    double SegmentsSegments(const Segment* segmentsA, std::size_t countA,
                            const SegmentBVH& hierarchyA,
                            const Segment* segmentsB, std::size_t countB,
                            const SegmentBVH& hierarchyB,
                            const GeometryContext& context)
    {
        if (segmentsA == nullptr || segmentsB == nullptr || countA == 0 || countB == 0)
            return std::numeric_limits<double>::infinity();

        const double tolerance = context.Tolerance().intersection;
        double best = std::numeric_limits<double>::infinity();

        // `best` is read by the traversal on every node pair, so each improvement
        // immediately prunes the remaining search.
        PrunedPairs(hierarchyA, hierarchyB, best, [&](std::uint32_t i, std::uint32_t j) noexcept
        {
            // The running minimum goes in as the bound: a pair that cannot beat it is
            // abandoned as soon as its boxes prove it, instead of being resolved to
            // `tolerance` and then thrown away by the min.
            best = std::min(best, CurveCurveSquared(segmentsA[i], segmentsB[j], tolerance, best));
            return best > 0.0;
        });

        return std::sqrt(best);
    }

    bool BuildPathHierarchy(const Path& path, const GeometryContext& context,
                            std::vector<Segment>& segments, SegmentBVH& hierarchy)
    {
        segments.clear();

        // Section 15A: the same unchecked indexing, one level up. A Path is only as
        // traversable as its worst contour.
        if (!path.IsStructurallyValid())
            return false;

        segments.reserve(path.SegmentCount());

        for (const Contour& contour : path.contours)
        {
            const std::size_t count = contour.SegmentCount();
            for (std::size_t i = 0; i < count; ++i)
                segments.push_back(contour.SegmentAt(i));
        }

        if (segments.empty())
            return false;

        hierarchy.BuildFromSegments(segments.data(), segments.size(), context);
        return true;
    }

    double ContourContour(const Contour& a, const Contour& b, const GeometryContext& context)
    {
        std::vector<Segment> segmentsA;
        std::vector<Segment> segmentsB;
        if (!GatherSegments(a, segmentsA) || !GatherSegments(b, segmentsB))
            return std::numeric_limits<double>::infinity();

        if (segmentsA.empty() || segmentsB.empty())
            return std::numeric_limits<double>::infinity();

        SegmentBVH bvhA;
        SegmentBVH bvhB;
        bvhA.BuildFromSegments(segmentsA.data(), segmentsA.size(), context);
        bvhB.BuildFromSegments(segmentsB.data(), segmentsB.size(), context);

        return SegmentsSegments(segmentsA.data(), segmentsA.size(), bvhA,
                                segmentsB.data(), segmentsB.size(), bvhB, context);
    }
}
