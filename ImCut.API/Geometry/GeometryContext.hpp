#pragma once

#include "GeometryResult.hpp"
#include "GeometryDiagnostics.hpp"
#include "GeometryTolerance.hpp"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <iterator>

namespace ImCut::Geometry
{
    // Cooperative cancellation. The kernel never owns the flag; the caller does, and
    // the caller decides its lifetime. A null token never cancels, so the common
    // uncancellable path costs one predictable branch.
    class CancellationToken
    {
    public:
        CancellationToken() noexcept = default;
        explicit CancellationToken(const std::atomic<bool>* flag) noexcept : flag_(flag) {}

        [[nodiscard]] bool IsCancelled() const noexcept
        {
            return flag_ != nullptr && flag_->load(std::memory_order_relaxed);
        }

        [[nodiscard]] bool CanCancel() const noexcept { return flag_ != nullptr; }

    private:
        const std::atomic<bool>* flag_ = nullptr;
    };

    // Hard ceilings that turn a runaway input into an explicit ComplexityLimit status
    // instead of an out-of-memory crash. Defaults are generous enough that no sane
    // production artwork reaches them: the largest file in the reference corpus
    // produces roughly 1.6k curve segments.
    struct ComplexityLimits
    {
        std::size_t maxSegments = 4'000'000;
        std::size_t maxFlattenPoints = 16'000'000;
        std::size_t maxIntersections = 4'000'000;
        std::size_t maxPolygonVertices = 16'000'000;
        std::size_t maxContours = 1'000'000;

        // Depth cap for curve subdivision. Each level halves the parameter interval,
        // so 40 levels is already far below any representable double spacing.
        int maxSubdivisionDepth = 40;

        // Memory ceilings, in bytes.
        //
        // The count-based limits above were each chosen in isolation and none of them
        // was ever converted into memory: maxFlattenPoints of 16 million is 244 MB of
        // Vec2, maxSegments of 4 million is 275 MB of Segment, and a BVH at the segment
        // limit measured 504 MB. One operation could ask for most of a gigabyte while
        // every individual limit looked reasonable, and nothing accumulated across the
        // operations of a session at all.
        //
        // These two are derived from what a worker may actually hold, and both are
        // enforced rather than documented. Defaults are deliberately generous next to
        // real artwork - the largest corpus file produces 1763 segments - and small next
        // to what the count limits allowed.
        //
        //   maxCacheBytes    bounds the flatten cache alone, evicted least-recently-used
        //                    by GeometrySession::EvictFlattenTo.
        //   maxSessionBytes  bounds what the SESSION RETAINS - the flatten, definition
        //                    and NFP payloads, plus the lookup maps and spare container
        //                    capacity the session owns to manage them - via
        //                    GeometrySession::CacheBytes() and EvictDefinitionsTo.
        //                    Definitions are dropped before flatten entries: re-preparing
        //                    one costs microseconds and it is cached again immediately,
        //                    whereas thrashing the flatten cache repeats that work on
        //                    every contour of every later operation.
        //
        // WHAT THEY DO NOT BOUND. V8.1 said maxSessionBytes bounded "EVERYTHING the
        // session holds" while CacheBytes() summed three payload totals and nothing else.
        // Measured against process private bytes under a 400-definition load, the
        // accounted figure sat 1.31x-1.41x low (evidence 884). The index structures are
        // now counted; allocator overhead is not, because no portable C++ can measure it,
        // and neither is memory a caller keeps alive through its own shared_ptr after the
        // session evicted it - that memory belongs to the caller.
        //
        // ZERO IS A CEILING OF ZERO, exactly like every other field in this struct:
        // maxContours == 0 refuses a path with one contour. A session configured with 0
        // computes normally and retains nothing. V8.1 returned early from eviction on
        // zero, which inverted the meaning - the tightest ceiling a caller could express
        // retained the most of any value (evidence 883).
        //
        // The one unavoidable exception, declared rather than hidden: the flatten cache
        // returns its result by reference into itself, so under maxCacheBytes == 0 it
        // still holds the single entry it is handing back, until the next call.
        //
        // See V8_1_1_SESSION_MEMORY_CONTRACT.md.
        std::size_t maxCacheBytes = std::size_t{ 256 } * 1024 * 1024;
        std::size_t maxSessionBytes = std::size_t{ 1024 } * 1024 * 1024;

        // Cap on the number of convex piece PAIRS a Minkowski sum may evaluate.
        //
        // A (+) B is computed as the union over every pair of convex pieces, which is
        // quadratic in the two piece counts. Left unbounded, two 4600-piece real parts
        // would ask for 21 million exact sums. Exceeding it is ComplexityLimit, so a
        // caller finds out instead of waiting.
        std::size_t maxConvexPairs = 100'000;

        // NFP operation memory ceilings. `maxConvexPairs` controls whether the explicit
        // A*B representation is admissible; it is not a validity ceiling because the
        // Product representation can answer from O(A+B) retained state. These limits
        // name the actual resources independently.
        std::size_t maxNfpWorkingBytes = std::size_t{ 256 } * 1024 * 1024;
        std::size_t maxNfpResultBytes = std::size_t{ 512 } * 1024 * 1024;

        // Subdivision budget for ONE certified curve-curve distance interval.
        //
        // This is a RESOURCE contract, not a correctness limit, and the difference is the
        // whole point. Exceeding it does not fail and does not approximate: the solver
        // returns a WIDER interval and says so, and the published value plus budget still
        // contains the true distance. A caller that needs a narrower answer raises this;
        // a caller that needs a bounded one lowers it.
        //
        // Why it has to exist at all: the certified frontier has no depth cap, and for two
        // curves in near-tangential contact the set of piece pairs whose exact bounding
        // boxes still overlap grows without limit while the interval narrows only slowly.
        // Left unbounded, a single leaf pair can consume the machine. Measured usage on
        // the corpus and the full suite is reported in V8_1_2_PERFORMANCE_CLOSURE.md; if
        // the observed maximum ever approaches this number, the number is wrong, not the
        // geometry - and section 134 forbids deriving a new ceiling from it.
        std::size_t maxDistanceRefinements = 20'000;

        // Experimental reduced-convolution ceilings. Segment generation is linear in
        // the vertex/edge product, while the prototype arrangement audit can still be
        // quadratic before spatial pruning. Both dimensions are bounded explicitly.
        std::size_t maxConvolutionSegments = 250'000;
        std::size_t maxConvolutionPairTests = 25'000'000;

        // Explicit NFP batch ceilings. Batch preparation is O(shape count * rotation
        // count), while pair execution is O(request count); neither dimension is
        // allowed to turn an otherwise bounded primitive into an unbounded scheduler.
        std::size_t maxNfpBatchVariants = 100'000;
        std::size_t maxNfpBatchPairs = 1'000'000;
        std::size_t maxNfpBatchWorkers = 256;

        // Cap on the nesting depth of a polygon result: outer, hole, island in that
        // hole, and so on. Nesting depth is input-controlled, and two separate
        // recursions used to walk it - Backend::CollectTree and the recursive
        // Clipper2Lib::PolyPath64 destructor - so past roughly 4950 levels a Boolean
        // took the process down with 0xC00000FD instead of returning a status.
        //
        // 256 is generous for artwork: real nesting is a handful of levels, and a
        // concentric test pattern deep enough to matter is already pathological.
        // Exceeding it is ComplexityLimit, the same contract as every other limit here.
        int maxNestingDepth = 256;
    };

    // Is `query` stricter than `prepared` on a limit that shaped a prepared NFP
    // operand's CONTENT?
    //
    // A prepared derivative was built under one set of limits. If a later query runs under
    // TIGHTER ones, answering from that derivative would let the caller's own ceiling be
    // bypassed simply because the work happened earlier - which is what section 49 of the
    // hardening brief forbids in as many words:
    //
    //     "Nenhuma rota prepared pode contornar limit porque algo foi preparado antes."
    //
    // Measured on V8.1: with a definition prepared under the defaults, a query under
    // maxSegments=1, maxContours=0, maxFlattenPoints=1 or maxPolygonVertices=1 got an
    // answer from the prepared route while the raw route on the same geometry returned
    // ComplexityLimit - four of six limits diverging (evidence 892).
    //
    // ONE implementation, shared. NfpPreparationProfile::ContextStatus had this comparison
    // and was the only thing in the kernel that did; a second copy for the collision
    // routes is how the two would drift apart, so both now call this.
    //
    // Query-only limits are deliberately absent. They are enforced where the query runs;
    // rejecting the prepared operand first would make maxConvexPairs block the Product
    // fallback even though no A*B payload was prepared or retained.
    [[nodiscard]] constexpr bool LimitsAreTighter(const ComplexityLimits& query,
                                                  const ComplexityLimits& prepared) noexcept
    {
        return query.maxSegments < prepared.maxSegments ||
               query.maxFlattenPoints < prepared.maxFlattenPoints ||
               query.maxIntersections < prepared.maxIntersections ||
               query.maxPolygonVertices < prepared.maxPolygonVertices ||
               query.maxContours < prepared.maxContours ||
               query.maxSubdivisionDepth < prepared.maxSubdivisionDepth ||
               query.maxNestingDepth < prepared.maxNestingDepth;
    }

    // The same question, restricted to the limits that govern what a
    // PreparedShapeDefinition CONTAINS rather than what a query does with it.
    //
    // WHY TWO FUNCTIONS AND NOT ONE. A prepared NFP operand includes a convex
    // decomposition, whose backend can consume maxNestingDepth. A
    // PreparedShapeDefinition does not contain that decomposition.
    // A PreparedShapeDefinition is a flatten, a hierarchy and a topology; maxConvexPairs
    // and maxIntersections bound work a QUERY does, and the query enforces them as it runs
    // on the raw and prepared routes alike.
    //
    // Gating a collision query on those would not be "stricter", it would be WRONG: a
    // caller who tightens maxConvexPairs for safety would find every prepared collision
    // query refused, including the ones that evaluate no convex pairs at all. Measured
    // exactly that way - the first version of this gate used the full comparison and made
    // maxConvexPairs=1 and maxIntersections=1 refuse where the raw route answered, turning
    // four divergences into two in the opposite direction (evidence 894).
    //
    // The set below is not a judgement call. It is the set of limits actually consulted
    // while a definition is built, read off the source: Flatten.cpp (maxContours,
    // maxFlattenPoints, maxSegments, maxSubdivisionDepth), PolygonConversion.cpp
    // (maxPolygonVertices and the same three), ContainmentTree.cpp and
    // PreparedGeometry.cpp (subsets of those). Nothing else is consulted at preparation
    // time, so nothing else can invalidate a derivative after the fact.
    [[nodiscard]] constexpr bool PreparationLimitsAreTighter(
        const ComplexityLimits& query, const ComplexityLimits& prepared) noexcept
    {
        return query.maxSegments < prepared.maxSegments ||
               query.maxFlattenPoints < prepared.maxFlattenPoints ||
               query.maxPolygonVertices < prepared.maxPolygonVertices ||
               query.maxContours < prepared.maxContours ||
               query.maxSubdivisionDepth < prepared.maxSubdivisionDepth;
    }

    // Relaxed atomic counters.
    //
    // These were plain integers, justified by "a context belongs to one operation on one
    // thread". The justification does not hold up: copying a GeometryContext copies the
    // SINK POINTER, and ContainmentTree and Flatten::PathWith both copy the context
    // inside a loop. Nothing prevented two workers from sharing one sink, nothing
    // signalled it, and README.md claimed the kernel was thread-safe by construction
    // while a data race sat one context copy away.
    //
    // Relaxed ordering is the right strength: these are counters read after the fact,
    // never used to order anything. The cost is a lock-free add on a path that only runs
    // when instrumentation is attached at all - CountStat returns immediately when the
    // sink is null, which is the production configuration.
    struct GeometryStatistics
    {
        using Counter = std::atomic<std::uint64_t>;

        Counter flattenCalls{ 0 };
        Counter flattenPoints{ 0 };
        Counter curveSubdivisions{ 0 };
        Counter bvhBuilds{ 0 };
        Counter bvhNodes{ 0 };
        Counter bvhQueries{ 0 };
        Counter bvhCandidatePairs{ 0 };
        Counter narrowPhaseTests{ 0 };
        Counter intersectionsFound{ 0 };
        Counter pointInPolygonTests{ 0 };
        Counter booleanOperations{ 0 };
        Counter offsetOperations{ 0 };
        Counter minkowskiOperations{ 0 };
        Counter cacheHits{ 0 };
        Counter cacheMisses{ 0 };
        Counter nfpCalls{ 0 };
        Counter nfpConvexFastPath{ 0 };
        Counter nfpDecomposition{ 0 };
        Counter nfpCoverBuilds{ 0 };
        Counter nfpMaterializations{ 0 };
        Counter nfpContactBuilds{ 0 };
        Counter nfpConvolution{ 0 };
        Counter nfpHoleFilterAudits{ 0 };
        Counter nfpHolesFiltered{ 0 };
        Counter nfpBatchRuns{ 0 };
        Counter nfpBatchPairs{ 0 };
        Counter booleanBatches{ 0 };
        Counter convexPairs{ 0 };
        Counter operandPrepareHits{ 0 };
        Counter operandPrepareMisses{ 0 };

        // Prepared distance route (WP12). These answer one question: how many queries
        // actually had to pay for exact curve-curve proximity?
        //
        // distanceCandidatePairs   leaf pairs the BVH traversal delivered
        // distanceSeedProbes       cheap on-curve samples taken to bound the answer
        // distanceBoundedAccepts   threshold decided TRUE by the upper bound alone
        // distanceBoundedRejects   threshold decided FALSE by the lower bound alone
        // distanceExactRefinements leaf pairs that reached Proximity::CurveCurveSquared
        Counter distanceCandidatePairs{ 0 };
        Counter distanceSeedProbes{ 0 };
        Counter distanceBoundedAccepts{ 0 };
        Counter distanceBoundedRejects{ 0 };
        Counter distanceExactRefinements{ 0 };

        // distanceLedgerSpills  queries whose certificate ledger outgrew its inline buffer
        //
        // Diagnostic, and the reason it exists is a test: an inline buffer with a spill
        // branch nobody can observe is a branch nobody can claim to have exercised. The
        // corpus crosses it on a few per cent of queries, and this is how a test says so
        // instead of deriving it from a percentile.
        Counter distanceLedgerSpills{ 0 };

        GeometryStatistics() = default;

        // Atomics are neither copyable nor movable, and a snapshot of counters is a
        // useful thing to have, so the copy is spelled out and takes each value once.
        GeometryStatistics(const GeometryStatistics& other) noexcept { Assign(other); }

        GeometryStatistics& operator=(const GeometryStatistics& other) noexcept
        {
            if (this != &other) Assign(other);
            return *this;
        }

        void Merge(const GeometryStatistics& other) noexcept
        {
            Combine(other, [](Counter& into, std::uint64_t value) noexcept
            {
                into.fetch_add(value, std::memory_order_relaxed);
            });
        }

        void Reset() noexcept
        {
            Apply([](Counter& counter) noexcept
            {
                counter.store(0, std::memory_order_relaxed);
            });
        }

    private:
        // One list of the counters, walked by copy, merge and reset alike. Fifteen
        // hand-written repetitions of the same field list is how a counter gets
        // forgotten in one of them.
        template <typename Fn>
        void Apply(Fn&& fn) noexcept
        {
            Counter* const counters[] = {
                &flattenCalls, &flattenPoints, &curveSubdivisions, &bvhBuilds, &bvhNodes,
                &bvhQueries, &bvhCandidatePairs, &narrowPhaseTests, &intersectionsFound,
                &pointInPolygonTests, &booleanOperations, &offsetOperations,
                &minkowskiOperations, &cacheHits, &cacheMisses, &nfpCalls,
                &nfpConvexFastPath, &nfpDecomposition, &nfpCoverBuilds,
                &nfpMaterializations, &nfpContactBuilds, &convexPairs,
                &nfpConvolution, &nfpHoleFilterAudits, &nfpHolesFiltered,
                &nfpBatchRuns, &nfpBatchPairs, &booleanBatches,
                &operandPrepareHits, &operandPrepareMisses,
                &distanceCandidatePairs, &distanceSeedProbes, &distanceBoundedAccepts,
                &distanceBoundedRejects, &distanceExactRefinements, &distanceLedgerSpills
            };
            for (Counter* counter : counters) fn(*counter);
        }

        template <typename Fn>
        void Combine(const GeometryStatistics& other, Fn&& fn) noexcept
        {
            const Counter* const source[] = {
                &other.flattenCalls, &other.flattenPoints, &other.curveSubdivisions,
                &other.bvhBuilds, &other.bvhNodes, &other.bvhQueries,
                &other.bvhCandidatePairs, &other.narrowPhaseTests, &other.intersectionsFound,
                &other.pointInPolygonTests, &other.booleanOperations, &other.offsetOperations,
                &other.minkowskiOperations, &other.cacheHits, &other.cacheMisses,
                &other.nfpCalls, &other.nfpConvexFastPath, &other.nfpDecomposition,
                &other.nfpCoverBuilds, &other.nfpMaterializations,
                &other.nfpContactBuilds, &other.convexPairs, &other.nfpConvolution,
                &other.nfpHoleFilterAudits, &other.nfpHolesFiltered,
                &other.nfpBatchRuns, &other.nfpBatchPairs, &other.booleanBatches,
                &other.operandPrepareHits, &other.operandPrepareMisses,
                &other.distanceCandidatePairs, &other.distanceSeedProbes,
                &other.distanceBoundedAccepts, &other.distanceBoundedRejects,
                &other.distanceExactRefinements, &other.distanceLedgerSpills
            };
            Counter* const target[] = {
                &flattenCalls, &flattenPoints, &curveSubdivisions, &bvhBuilds, &bvhNodes,
                &bvhQueries, &bvhCandidatePairs, &narrowPhaseTests, &intersectionsFound,
                &pointInPolygonTests, &booleanOperations, &offsetOperations,
                &minkowskiOperations, &cacheHits, &cacheMisses, &nfpCalls,
                &nfpConvexFastPath, &nfpDecomposition, &nfpCoverBuilds,
                &nfpMaterializations, &nfpContactBuilds, &convexPairs,
                &nfpConvolution, &nfpHoleFilterAudits, &nfpHolesFiltered,
                &nfpBatchRuns, &nfpBatchPairs, &booleanBatches,
                &operandPrepareHits, &operandPrepareMisses,
                &distanceCandidatePairs, &distanceSeedProbes, &distanceBoundedAccepts,
                &distanceBoundedRejects, &distanceExactRefinements, &distanceLedgerSpills
            };
            for (std::size_t i = 0; i < std::size(target); ++i)
                fn(*target[i], source[i]->load(std::memory_order_relaxed));
        }

        void Assign(const GeometryStatistics& other) noexcept
        {
            Combine(other, [](Counter& into, std::uint64_t value) noexcept
            {
                into.store(value, std::memory_order_relaxed);
            });
        }
    };

    // Everything an operation needs to know about how to behave. Created per
    // operation, passed by const reference, never a global singleton, so two
    // concurrent operations can run under different tolerances without interfering.
    class GeometryContext
    {
    public:
        GeometryContext() = default;

        explicit GeometryContext(PrecisionMode mode)
            : tolerance_(GeometryTolerance::ForMode(mode)), precision_(mode) {}

        GeometryContext(GeometryTolerance tolerance, PrecisionMode mode)
            : tolerance_(tolerance), precision_(mode) {}

        [[nodiscard]] const GeometryTolerance& Tolerance() const noexcept { return tolerance_; }
        [[nodiscard]] PrecisionMode Precision() const noexcept { return precision_; }
        [[nodiscard]] const ComplexityLimits& Limits() const noexcept { return limits_; }
        [[nodiscard]] const CancellationToken& Cancellation() const noexcept { return cancellation_; }
        [[nodiscard]] GeometryDiagnostics* Diagnostics() const noexcept { return diagnostics_; }

        void SetTolerance(const GeometryTolerance& value) noexcept { tolerance_ = value; }
        void SetPrecision(PrecisionMode value) noexcept { precision_ = value; }
        void SetLimits(const ComplexityLimits& value) noexcept { limits_ = value; }
        void SetCancellation(CancellationToken token) noexcept { cancellation_ = token; }
        void SetDiagnostics(GeometryDiagnostics* value) noexcept { diagnostics_ = value; }

        // Statistics are opt-in and owned by the caller, so an instrumented run and a
        // production run execute the same code path.
        void SetStatistics(GeometryStatistics* sink) noexcept { statistics_ = sink; }
        [[nodiscard]] GeometryStatistics* Statistics() const noexcept { return statistics_; }

        [[nodiscard]] bool IsCancelled() const noexcept { return cancellation_.IsCancelled(); }

        // Block-granular cancellation: callers accumulate work into `counter` and only
        // touch the atomic once per `kCancellationBlock` items. Checking per point
        // would cost more than most of the geometry it guards.
        static constexpr std::size_t kCancellationBlock = 4096;

        [[nodiscard]] bool ShouldCheckCancellation(std::size_t counter) const noexcept
        {
            return cancellation_.CanCancel() && (counter % kCancellationBlock) == 0;
        }

    private:
        GeometryTolerance tolerance_{};
        PrecisionMode precision_ = PrecisionMode::Production;
        ComplexityLimits limits_{};
        CancellationToken cancellation_{};
        GeometryStatistics* statistics_ = nullptr;
        GeometryDiagnostics* diagnostics_ = nullptr;
    };

    // Increments a counter only when instrumentation is attached.
    inline void CountStat(const GeometryContext& context,
                          GeometryStatistics::Counter GeometryStatistics::* field,
                          std::uint64_t amount = 1) noexcept
    {
        if (GeometryStatistics* sink = context.Statistics())
            (sink->*field).fetch_add(amount, std::memory_order_relaxed);
    }
}
