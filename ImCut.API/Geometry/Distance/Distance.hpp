#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Spatial/SegmentBVH.hpp"
#include "../Math/Vec2.hpp"

#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

namespace ImCut::Geometry::Proximity
{
    struct ClosestResult
    {
        Vec2 point;
        double parameter = 0.0;
        double distanceSquared = 0.0;
        std::size_t index = 0;
    };

    // Squared distances throughout: the hot paths compare distances, and comparing
    // squares avoids a sqrt per test.
    [[nodiscard]] double PointSegmentSquared(Vec2 point, Vec2 a, Vec2 b) noexcept;

    [[nodiscard]] ClosestResult ClosestOnSegment(Vec2 point, Vec2 a, Vec2 b) noexcept;

    [[nodiscard]] double SegmentSegmentSquared(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1) noexcept;

    [[nodiscard]] double PointCurveSquared(Vec2 point, const Segment& curve) noexcept;

    // Adaptive subdivision under bounds pruning; accurate to `tolerance`.
    //
    // `upperBoundSquared` is a bound the caller ALREADY holds - a running minimum, or a
    // threshold it is testing against. Any answer at or above it cannot change what the
    // caller does, so the search is allowed to stop as soon as it can prove the pair is
    // that far apart, instead of resolving it to `tolerance` for nothing. Precisely:
    //
    //   - if the true squared distance is BELOW the bound, that value is returned,
    //     accurate to `tolerance` exactly as the unbounded form;
    //   - otherwise SOME value at or above the bound is returned, and its magnitude
    //     carries no information.
    //
    // So it is safe for `best = min(best, ...)` and for `... < threshold`, and unsafe
    // for anything that reports the number itself. The default is infinity, which is
    // the unbounded search.
    //
    // This is not a micro-optimisation. Two curved outlines in the near-touch regime -
    // the one a nesting engine meets constantly - cost 17,171 us per MinimumDistance
    // query without it, of which 99.9% was 23 curve-curve pairs each resolved in full
    // isolation at 762 us. Measured in audit-v5/evidence-v5/78_neartouch.txt.
    // `subdivisionsOut`, when given, receives the number of subdivision steps this call
    // performed, ADDED to whatever it already holds. It exists so a caller that owns a
    // GeometryContext can publish GeometryStatistics::curveSubdivisions without this
    // function having to know about contexts or stop being noexcept. Null costs one
    // predictable branch at the end of the call, which is the production path.
    // `capDeviationOut`, when given, receives the worst dA + dB over any pair the search
    // accepted WITHOUT both pieces being flat - that is, where the depth or stack cap
    // fired. For those pairs the usual "each chord is within `tolerance` of its curve"
    // guarantee does not hold, so the caller cannot bound its answer by 2*tolerance and
    // has to publish this instead. Zero means every accepted pair was genuinely flat.
    //
    // Flatten.cpp does the same thing at its own depth cap (DeviationBound at :63, folded
    // into toleranceUsed at :228); this is that mechanism, for the distance solver.
    [[nodiscard]] double CurveCurveSquared(const Segment& a, const Segment& b, double tolerance,
                                           double upperBoundSquared =
                                               std::numeric_limits<double>::infinity(),
                                           std::uint64_t* subdivisionsOut = nullptr,
                                           double* capDeviationOut = nullptr) noexcept;

    // A CERTIFIED INTERVAL for the distance between two curve segments.
    //
    // This is the form CurveCurveSquared should always have had. It returns bounds rather
    // than a scalar with an error attached afterwards, and the difference is not stylistic:
    //
    //   upper   the smallest distance between two points that lie exactly ON the curves.
    //           An upper bound with NO tolerance term, because the infimum is at most any
    //           element of the set it is taken over (DISTANCE_INTERVAL_PROOF.md, Lemma 1).
    //
    //   lower   the smallest CubicBezier::ExactBounds gap over the live frontier. The
    //           curve is inside its exact AABB, so the gap between two such boxes is a
    //           true lower bound (Lemma 2).
    //
    // WHAT THIS FIXES. The scalar form accepts a leaf pair when `depth >= 24` or the stack
    // fills, and reports the CHORD distance for it. The deviation of such a pair was
    // tracked only for whichever pair owned the running minimum, and REASSIGNED on every
    // change of owner - so a flat pair winning last erased the deviation of a capped pair
    // accepted earlier, and a capped leaf whose chord sat above the running best was
    // discarded even though its true distance could sit below it.
    //
    // Here there is no owner. `lower` is the minimum over the whole live frontier, and a
    // leaf accepted at the cap STAYS on that frontier with its own bound until refinement
    // resolves it or pruning eliminates it. Pruned pairs need not be tracked: they were
    // pruned because their lower bound already exceeded the upper bound (Lemma 4).
    //
    // `targetWidth` is what the caller needs, in millimetres: refinement continues until
    // `upper - lower <= targetWidth`. There is NO depth cap. `converged` says whether the
    // target was met; when it is false the interval is still valid, just wider than asked,
    // and the caller decides what that means rather than being handed a number that looks
    // finished.
    //
    // `workLimit` bounds the number of subdivisions. Zero means unlimited. It exists so a
    // caller can impose a resource contract; it is NOT a correctness limit and exceeding
    // it produces a wider interval, never a wrong one.
    struct DistanceBounds
    {
        double lower = 0.0;
        double upper = 0.0;
        std::uint64_t subdivisions = 0;

        // True when the fixed-stack fast path could not reach `targetWidth` and the
        // dynamic certified frontier was used. Instrumentation only; the answer means the
        // same either way.
        bool usedFallback = false;

        // upper - lower <= targetWidth.
        bool converged = false;

        // HOW THE SEARCH ENDED - and the reason this type is not three doubles.
        //
        //   Success          the interval is the answer, converged or not
        //   Cancelled        the caller asked to stop; the interval is valid but partial
        //   ComplexityLimit  a HARD ceiling the caller set was reached. Never "the
        //                    algorithm gave up": that is what the SOFT refinement budget
        //                    does, and it stays Success with a wider interval
        //   OutOfMemory      the certified frontier could not allocate
        //
        // F27 measured what the absence of this field cost. With the allocator armed to
        // fail inside the fallback, the process died with 0xC0000409 - the MSVC fastfail
        // for a noexcept violation - while CERTIFIED_CURVE_DISTANCE_SOLVER.md section 6
        // promised OutOfMemory (evidence 1144). A contract the return type cannot express
        // is not a contract.
        //
        // A caller that ignores this is not led into a WRONG answer, only a wide one:
        // every non-Success exit publishes an interval that still contains the truth.
        GeometryStatus status = GeometryStatus::Success;

        // DIAGNOSTIC ONLY - never read by production logic.
        //
        // Which code path fixed the published `lower`. The question that matters when an
        // interval refuses to narrow is not "which pair produced the best upper bound" but
        // "what stops the lower bound from rising", and those are routinely different
        // places. Written unconditionally because a diagnostic that only exists in the
        // test build measures a binary nobody ships.
        enum class LowerSource : std::uint8_t
        {
            None,
            LineLineClosedForm,   // both operands straight: the interval is a point
            FrontierRetired,      // a piece left on `upper - lower <= width`
            FrontierFlat,         // a piece left on the flatness terminal
            SeedFloor,            // pruned against the caller bound
            CollapsedToUpper,     // nothing live: interval collapsed onto the witness
            FallbackDecided,      // the certified frontier decided or converged
            FallbackWorkLimit     // the work budget stopped the certified frontier
        };

        LowerSource lowerSource = LowerSource::None;

        // The two candidates the published lower is chosen from, kept apart so the probe
        // can see which one won and by how much.
        double frontierLower = 0.0;
        double seedFloor = 0.0;

        [[nodiscard]] constexpr double Midpoint() const noexcept { return (lower + upper) * 0.5; }
        [[nodiscard]] constexpr double HalfWidth() const noexcept { return (upper - lower) * 0.5; }
    };

    // WHAT THE CALLER WANTS, AND WHAT THE CALLER WILL NOT PAY - kept apart on purpose.
    //
    // ComplexityLimits opens by saying "hard ceilings: exceeded => ComplexityLimit" and
    // then holds maxDistanceRefinements, whose excess returns a WIDER INTERVAL and no
    // failure at all. The type lies about half its contents, and maxPieces and
    // maxConvexPairs are about to need the same separation. So the split is made here
    // first, in the smallest place it can be made, and the two groups do not share a
    // field or a sentence.
    //
    // SOFT is a preference. Exceeding it changes the ANSWER - a wider interval, converged
    // false - and never the status.
    //
    // HARD is a contract. Exceeding it is an honest failure with ComplexityLimit, and it
    // is the only thing in this solver allowed to produce that status. "The algorithm
    // stopped early" is not a complexity limit.
    struct DistanceSolveControl
    {
        // ---- SOFT ----------------------------------------------------------------

        // Refinement continues until `upper - lower <= targetWidth`. There is no depth
        // cap; falling short sets `converged` false and leaves the interval valid.
        double targetWidth = 0.0;

        // A bound the caller ALREADY holds - a running minimum, or a threshold under
        // test. It PRUNES and is never a witness: a piece discarded against it has true
        // distance at least the seed, which is what the ledger records. Seeding cannot
        // move `upper`, because `upper` only moves on a point that lies ON both curves.
        double upperSeed = std::numeric_limits<double>::infinity();

        // When finite, the search becomes a DECISION rather than a measurement:
        // refinement stops as soon as the interval lies entirely on one side of this.
        // Converging to 1e-9 mm on a near-tangential pair can need 2^30 pieces; deciding
        // that pair against 1.2 mm usually takes a handful. `targetWidth` never
        // participates in the deciding path - F19 was exactly that dependency, and it
        // produced a false negative in Production at 1e-06 on a pair 2.5e-08 apart
        // (evidence 1105).
        double decideAt = std::numeric_limits<double>::quiet_NaN();

        // Bounds the number of subdivisions; 0 means unlimited. Exceeding it returns a
        // WIDER interval with `converged` false - never a narrower one, and never a
        // failure. A resource contract may bound the work; it may not bound the honesty.
        std::uint64_t refinementBudget = 0;

        // ---- HARD ----------------------------------------------------------------

        // Ceiling on the certified frontier's working set, in bytes. 0 means unlimited.
        // Exceeding it returns ComplexityLimit with the interval proved so far.
        //
        // Nothing in the kernel sets this today, and that is stated rather than hidden:
        // it exists so that the hard/soft separation has a place to live before WP4 has
        // to make the same split for maxPieces, and so that a caller who genuinely needs
        // a memory contract does not have to invent one at a call site.
        std::size_t maxWorkingBytes = 0;
    };

    // NOT noexcept, and that is the point of this declaration.
    //
    // The certified frontier owns a std::vector and a std::priority_queue, so it can
    // throw std::bad_alloc; F27 proved that the old noexcept turned that into process
    // death (evidence 1144). This entry point translates it into
    // DistanceBounds::status == OutOfMemory, which is the same shape Flatten.cpp already
    // uses at :249 and :316 for the identical problem.
    //
    // `context` is REQUIRED, not defaulted. Cancellation cannot be forgotten by a caller
    // that has no way to omit it, and a default-constructed GeometryContext has no
    // cancellation source, so the check costs one predictable branch. F13 and F18 are
    // both "an API forgot to ask"; this is the cheapest place to stop asking politely.
    [[nodiscard]] DistanceBounds CurveCurveBounds(const Segment& a, const Segment& b,
                                                  const DistanceSolveControl& control,
                                                  const GeometryContext& context);

    // ------------------------------------------------------------------------------
    // THE QUERY-LEVEL LEDGER: which pair contributes what, NOW.
    //
    // IntervalLedger, inside CurveCurveBounds, already draws the line this type is about.
    // It accumulates `retired_` as a running minimum - and that is correct there, because a
    // retired piece is FINAL and will never be refined - while everything still refinable
    // is passed to Publish() as `liveLower` and recomputed at each exit. Putting a
    // provisional bound into `retired_` was defect E of WP3, and separating the two is what
    // fixed it.
    //
    // MinimumDistance did the opposite one level up. It wrote
    //
    //     globalLower = min(globalLower, bounds.lower);        // pair solved with a seed
    //     ...
    //     globalLower = min(globalLower, refreshed.lower);     // SAME pair, refined
    //
    // and a running minimum never rises, so the second line cannot replace what the first
    // contributed. Measured: 120 pairs whose lower a refresh RAISES, worst raise 75.36
    // (evidence 1152). The published width did not inflate - TraverseNearestFirst happens
    // to resolve the pair that dominates the minimum first, unseeded, so it is already
    // final - which makes this a LATENT defect whose consequence is masked by traversal
    // order, not one that traversal order makes impossible. Change the order, the BVH, the
    // tie-breaking or the scheduling and it wakes up.
    //
    // TWO CHANNELS, AND THE SPLIT IS NOT DECORATION.
    //
    //   Final()     a contribution that can never be refined. A running minimum is exactly
    //               right for these, for the same reason it is right for `retired_`.
    //   Certify()   a contribution that CAN be refined. Stored per owner and REPLACED, so
    //               the published minimum is over current values and never over history.
    //
    // The split is also what keeps a line-line query at zero allocations: a pair of
    // straight segments is answered in closed form and is final immediately, so it never
    // touches the certificate vector.
    struct DistancePairKey
    {
        std::uint32_t a = 0;
        std::uint32_t b = 0;

        [[nodiscard]] constexpr bool operator==(const DistancePairKey& other) const noexcept
        {
            return a == other.a && b == other.b;
        }
    };

    struct PairCertificate
    {
        DistancePairKey owner;

        // The best lower bound currently proved for this pair. Replaced, never merged.
        double lower = 0.0;
    };

    // What the query publishes. One place produces it, the way one place produces a
    // DistanceBounds.
    struct GlobalDistanceInterval
    {
        double lower = 0.0;
        double upper = 0.0;
    };

    // INLINE STORAGE FOR THE COMMON CASE, AND A SPILL FOR EVERYTHING ELSE.
    //
    // MEASURED FIRST, SIZED SECOND (evidence 1254). Over 1011 queries - 1005 of them
    // pairwise across the four real corpus drawings - the number of certificates ONE query
    // holds is
    //
    //     P50 39    P90 133    P95 200    P99 337    MAX 645
    //
    // which is nothing like the 10 to 19 the synthetic fixtures suggested. Coverage against
    // stack cost:
    //
    //     N=64   68.6%   1 KB        N=192  94.7%   3 KB
    //     N=128  89.0%   2 KB        N=256  97.3%   4 KB
    //
    // 256 is the choice. Four kilobytes is small beside the 29 KB that SolveCurveCurve's
    // fixed pair stack already occupies in the same call chain, and 2.7% of corpus queries
    // still spill - enough that the branch stays exercised on every corpus run instead of
    // being dead code that only a fixture visits.
    //
    // CAPACITY IS A PERFORMANCE POLICY, NEVER A LIMIT. Overflow migrates to a vector and
    // the ledger answers exactly the same; nothing above N is refused, truncated, or
    // reported as ComplexityLimit. The template parameter exists so the metamorphic test
    // can drive capacities of 1, 2, 4, 8, 16, 32 and 64 through the same operation
    // sequences and require identical answers - which is what turns "capacity does not
    // change semantics" from a claim into a test.
    template <std::size_t Capacity>
    class BasicGlobalDistanceLedger
    {
    public:
        // THE ONLY WAY `upper` MOVES. A witness is an upper bound proved by geometry that
        // exists, so a running minimum over witnesses is sound: refining never makes a
        // witness worse, and the smallest one seen is the best one proved.
        void Witness(double upper) noexcept
        {
            if (upper < upper_)
                upper_ = upper;
        }

        // A lower bound from a contribution that CANNOT be refined - a closed form, or a
        // pair that has left the search resolved. Running minimum, deliberately.
        void Final(double lower) noexcept
        {
            if (lower < finalFloor_)
                finalFloor_ = lower;
        }

        // INSERT OR REPLACE the current lower bound of a refinable owner.
        //
        // This is the whole point of the type. Calling it twice for the same owner leaves
        // ONE contribution, the second - not the minimum of the two.
        //
        // May allocate, but only on the transition out of inline storage and afterwards.
        // The caller is a status-bearing entry point and translates.
        void Certify(DistancePairKey owner, double lower)
        {
            const std::size_t count = Count();
            for (std::size_t i = 0; i < count; ++i)
            {
                if (KeyAt(i) == owner)
                {
                    SetLowerAt(i, lower);
                    return;
                }
            }

            if (!spilled_ && inlineCount_ < Capacity)
            {
                inline_[inlineCount_].a = owner.a;
                inline_[inlineCount_].b = owner.b;
                inline_[inlineCount_].lower = lower;
                ++inlineCount_;
                return;
            }

            Spill();
            spill_.push_back({ owner, lower });
        }

        // Removes an owner's contribution entirely. A retired or pruned pair that proved
        // nothing must not keep holding the global minimum down.
        void Retire(DistancePairKey owner) noexcept
        {
            const std::size_t count = Count();
            for (std::size_t i = 0; i < count; ++i)
            {
                if (!(KeyAt(i) == owner))
                    continue;

                if (spilled_)
                {
                    spill_[i] = spill_.back();
                    spill_.pop_back();
                }
                else
                {
                    inline_[i] = inline_[inlineCount_ - 1];
                    --inlineCount_;
                }
                return;
            }
        }

        [[nodiscard]] double Upper() const noexcept { return upper_; }

        // The minimum over CURRENT contributions. Never over history.
        [[nodiscard]] double Lower() const noexcept
        {
            double lower = finalFloor_;
            if (spilled_)
            {
                for (const PairCertificate& certificate : spill_)
                {
                    if (certificate.lower < lower)
                        lower = certificate.lower;
                }
            }
            else
            {
                for (std::size_t i = 0; i < inlineCount_; ++i)
                {
                    if (inline_[i].lower < lower)
                        lower = inline_[i].lower;
                }
            }
            return lower;
        }

        [[nodiscard]] std::size_t Count() const noexcept
        {
            return spilled_ ? spill_.size() : inlineCount_;
        }

        [[nodiscard]] PairCertificate At(std::size_t index) const noexcept
        {
            if (spilled_)
                return spill_[index];
            return PairCertificate{ DistancePairKey{ inline_[index].a, inline_[index].b },
                                    inline_[index].lower };
        }

        // DIAGNOSTIC ONLY - never read by production logic. A test that cannot see whether
        // the spill branch ran cannot claim to have exercised it.
        [[nodiscard]] bool Spilled() const noexcept { return spilled_; }
        [[nodiscard]] static constexpr std::size_t InlineCapacity() noexcept { return Capacity; }

        // THE ONLY WAY THE QUERY-LEVEL INTERVAL IS PRODUCED, for the same reason
        // IntervalLedger::Publish is the only way a DistanceBounds is.
        [[nodiscard]] GlobalDistanceInterval Publish() const noexcept
        {
            GlobalDistanceInterval out;
            out.upper = upper_;
            out.lower = Lower();

            // Nothing proved a floor below the witness: every contribution was either
            // removed or sits above it, so the interval has collapsed onto the witness.
            // Same rule, and the same sentence, as IntervalLedger::Publish.
            if (!(out.lower <= out.upper))
                out.lower = out.upper;
            if (out.lower < 0.0)
                out.lower = 0.0;
            return out;
        }

    private:
        // A POD WITHOUT default member initialisers, deliberately. An array of
        // PairCertificate would run Capacity initialisers on every query for values about
        // to be overwritten - O(N) per query bought for nothing, which is exactly what a
        // fixed buffer must not cost.
        struct Slot
        {
            std::uint32_t a;
            std::uint32_t b;
            double lower;
        };

        [[nodiscard]] DistancePairKey KeyAt(std::size_t index) const noexcept
        {
            if (spilled_)
                return spill_[index].owner;
            return DistancePairKey{ inline_[index].a, inline_[index].b };
        }

        void SetLowerAt(std::size_t index, double lower) noexcept
        {
            if (spilled_)
                spill_[index].lower = lower;
            else
                inline_[index].lower = lower;
        }

        // ONE MIGRATION, not a growing pair of storages. Everything inline moves across in
        // order and the inline half is never read again, so there is a single place that
        // knows where the certificates live rather than two that have to agree.
        void Spill()
        {
            if (spilled_)
                return;
            spill_.reserve(Capacity * 2);
            for (std::size_t i = 0; i < inlineCount_; ++i)
            {
                spill_.push_back(PairCertificate{ DistancePairKey{ inline_[i].a, inline_[i].b },
                                                  inline_[i].lower });
            }
            inlineCount_ = 0;
            spilled_ = true;
        }

        double upper_ = std::numeric_limits<double>::infinity();
        double finalFloor_ = std::numeric_limits<double>::infinity();

        Slot inline_[Capacity];
        std::size_t inlineCount_ = 0;
        bool spilled_ = false;
        std::vector<PairCertificate> spill_;
    };

    using GlobalDistanceLedger = BasicGlobalDistanceLedger<256>;
    // ------------------------------------------------------------------------------

    [[nodiscard]] ClosestResult ClosestOnPolyline(Vec2 point, const Vec2* ring, std::size_t count,
                                                  bool closed) noexcept;

    [[nodiscard]] ClosestResult ClosestOnContour(Vec2 point, const Contour& contour,
                                                 const GeometryContext& context);

    // Threshold-aware forms.
    //
    // "Is anything within d?" is a different question from "how far is the nearest
    // thing?", and answering the first by computing the second wastes most of the work.
    // These stop the moment the answer is decided.
    [[nodiscard]] bool PointWithin(Vec2 point, const Contour& contour, double threshold,
                                   const GeometryContext& context);

    [[nodiscard]] bool ContoursWithin(const Contour& a, const Contour& b, double threshold,
                                      const GeometryContext& context);

    // Minimum distance between two contours. Returns infinity when either is empty.
    [[nodiscard]] double ContourContour(const Contour& a, const Contour& b,
                                        const GeometryContext& context);

    // Same query against hierarchies the caller already has.
    //
    // The form above gathers segments and builds two SegmentBVHs on every call, and for
    // an unprepared query that setup was measured at 64% to 94% of the total. Iterating
    // contour pairs multiplied it: MinimumDistance over two 4-contour paths built 32
    // hierarchies for 16 pairs and cost 138 us against 14 us for a single pair.
    //
    // A caller with a whole Path builds ONE hierarchy over all of its segments and calls
    // this. `segments` must be the array the hierarchy was built from - the traversal
    // indexes it directly - and both must outlive the call.
    [[nodiscard]] double SegmentsSegments(const Segment* segmentsA, std::size_t countA,
                                          const SegmentBVH& hierarchyA,
                                          const Segment* segmentsB, std::size_t countB,
                                          const SegmentBVH& hierarchyB,
                                          const GeometryContext& context);

    // Builds the hierarchy for a whole Path: every segment of every contour in one
    // array and one tree. Returns false when the path has no segments.
    [[nodiscard]] bool BuildPathHierarchy(const Path& path, const GeometryContext& context,
                                          std::vector<Segment>& segments, SegmentBVH& hierarchy);
}
