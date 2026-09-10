#include "PreparedCollision.hpp"

#include "../Curves/CubicBezier.hpp"
#include "../Math/CertifiedInterval.hpp"
#include "../Polygon/Boolean.hpp"
#include "../Spatial/SegmentBVH.hpp"
#include "Distance.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <optional>
#include <vector>

namespace ImCut::Geometry::Collision
{
    namespace
    {
        // THE ONE PLACE THIS FAMILY TURNS EXHAUSTION INTO A STATUS.
        //
        // F37. Every function in this header hands the caller a GeometryResult, which is
        // a promise that failure arrives as a status. Two of the seven kept that promise
        // through a hand-written try/catch of their own; the other five did not, and
        // Overlaps and Contains allocate heavily - GeometryInstance::WorldPath()
        // materialises the whole transformed path, and Boolean::Intersection takes two of
        // them. A std::bad_alloc from either walked straight out through a signature that
        // said it could not.
        //
        // Writing a third and a fourth copy of the same four lines is how the two that
        // existed came to be the only two. The rule lives here once, and every public
        // entry point below is spelled `return TranslateExhaustion<T>([&] { ... })`, so
        // the census that proves the family is covered is a census of a single name.
        //
        // ONLY std::bad_alloc is caught. A catch(...) here would convert a broken
        // internal invariant into OutOfMemory and lose it - README's "no silent
        // catch (...) anywhere in the kernel" is a claim this function has to keep, not
        // one it may spend.
        template <typename T, typename Body>
        [[nodiscard]] GeometryResult<T> TranslateExhaustion(Body&& body)
        {
            try
            {
                return body();
            }
            catch (const std::bad_alloc&)
            {
                return GeometryResult<T>::Failure(GeometryStatus::OutOfMemory);
            }
        }

        struct PairContext
        {
            const SegmentBVH* hierarchyA = nullptr;
            const SegmentBVH* hierarchyB = nullptr;
            const std::vector<Segment>* segmentsA = nullptr;
            const std::vector<Segment>* segmentsB = nullptr;

            // B's local frame mapped into A's local frame.
            Transform2 bToA = Transform2::Identity();
        };

        // A rigorous UPPER bound on the true distance between two curve segments, taken
        // from points that lie exactly ON both curves.
        //
        // WHY THIS NEEDS NO ERROR TERM. For any p in A and q in B, d(A,B) <= |p-q| by
        // definition of the infimum. CubicBezier::Evaluate returns a point of the curve,
        // not an approximation of one, so every distance below is an admissible upper
        // bound with no tolerance attached. That is what separates this from a polyline
        // bound, which would need the flatten tolerance of BOTH operands added back.
        //
        // Three parameters per curve. This is not trying to BE the answer - it exists so
        // the exact solver starts from a real number instead of infinity. Measured in
        // WP12: MinimumDistance seeded `best` with infinity, so the first leaf pair the
        // traversal delivered was subdivided to 1e-7 mm with nothing to prune against,
        // which is the 762 us per pair the V5 audit recorded and the reason a Doctor-like
        // pass spent 98.8% of its time here.
        [[nodiscard]] double OnCurveUpperBoundSquared(const Segment& a, const Segment& b) noexcept
        {
            // Two lines have a closed-form answer, so sampling them would be both slower
            // and weaker than simply solving them. Returning the exact distance here is
            // still a valid upper bound - it is the exact one - and it matters because
            // already-flattened operands reach this path as nothing but lines: the probe
            // cost 4.4% on `early hit` and 4.9% on `nearby miss` before this branch.
            if (a.IsLine() && b.IsLine())
                return Proximity::SegmentSegmentSquared(a.p0, a.p1, b.p0, b.p1);

            Vec2 pa[3];
            Vec2 pb[3];
            for (int i = 0; i < 3; ++i)
            {
                const double t = 0.5 * static_cast<double>(i);
                pa[i] = CubicBezier::Evaluate(a, t);
                pb[i] = CubicBezier::Evaluate(b, t);
            }

            double best = std::numeric_limits<double>::infinity();
            for (const Vec2 p : pa)
                for (const Vec2 q : pb)
                    best = (std::min)(best, DistanceSquared(p, q));
            return best;
        }

        // THE ABSOLUTE SLACK that covers a distance computed in ordinary doubles between
        // two segments, and the reason it is absolute rather than relative.
        //
        // A relative widening of the RESULT would be wrong exactly where it matters: the
        // rounding of SegmentSegmentSquared is an ULP of the COORDINATES, not of the
        // distance, and two nearly touching segments at 1e6 mm have a distance whose own
        // ULP is meaningless next to that. The certified blossom of stage B keys its
        // widening to the coordinate magnitude for the same reason.
        //
        // Thirty-two operations counted against a chain of about ten: the two points the
        // routine forms as a0 + d1*s and b0 + d2*t, one squared distance, one square root.
        // The clamped parameters s and t carry more error than that and it does not matter
        // for the UPPER direction, because any s and t give points ON the segments - it is
        // the LOWER direction that needs this, and F36 is what happens without it.
        [[nodiscard]] double ClosedFormSlack(const Segment& a, const Segment& b) noexcept
        {
            const Vec2 points[8] = { a.p0, a.c1, a.c2, a.p1, b.p0, b.c1, b.c2, b.p1 };
            double scale = 0.0;
            for (const Vec2 p : points)
            {
                scale = (std::max)(scale, std::fabs(p.x));
                scale = (std::max)(scale, std::fabs(p.y));
            }
            return scale * (Certified::kRelativeWiden * 32.0) + Certified::kAbsoluteWiden;
        }

        // The deviation the prepared representation of one instance actually declares.
        //
        // Flatten::PathWith publishes toleranceUsed as the deviation that ACTUALLY holds:
        // CubicBezier::FlatnessMetric bounds 16*d^2 for chord deviation d, IsFlatEnough
        // accepts only when that is within the request, and where the depth cap fires
        // Flatten.cpp raises toleranceUsed to the bound of the piece that fell short.
        // So this is a proven upper bound on how far the prepared geometry can be from
        // the true curve - not a heuristic, and not a number invented here.
        [[nodiscard]] double PreparedDeviation(const GeometryInstance& instance) noexcept
        {
            const PreparedShapeDefinitionPtr& definition = instance.Definition();
            if (definition == nullptr)
                return 0.0;
            const GeometryResult<FlattenedPath>& flattened = definition->Flattened();
            if (!flattened.Ok())
                return 0.0;

            // Worst contour wins: the bound has to hold for the whole path.
            //
            // Deliberately NOT FlattenedPath::budget. That budget merges the source
            // Path's own budget (Flatten.cpp:265), so it carries CALLER PROVENANCE, and
            // provenance is not a statement about how far this polyline sits from its
            // curve. Using it here would both overstate the geometric deviation and
            // double-count provenance the result composes elsewhere.
            //
            // FlattenedContour::toleranceUsed is the geometric quantity: max(requested,
            // worst depth-capped DeviationBound). Its own doc-comment calls it "a floor
            // and not a guarantee" when hitDepthLimit fires, which understates it -
            // Flatten.cpp:133 raises it to DeviationBound(piece) = sqrt(FlatnessMetric)/4,
            // and that expression IS the conservative upper bound on chord deviation.
            double worst = 0.0;
            for (const FlattenedContour& contour : flattened.Value().contours)
                worst = (std::max)(worst, contour.toleranceUsed);
            return worst;
        }

        // How finely a distance query between these two instances is worth refining.
        //
        // WHY THIS IS NOT "LOOSENING THE TOLERANCE". The curve solver used
        // tolerance.intersection - 1e-7 mm in Production - as its flatness target, so a
        // segment of a 700 mm artwork part was subdivided until every piece sat within
        // 1e-7 mm of its chord. WP12 measured the consequence: 758 subdivisions per leaf
        // pair, 19.6 million for one corpus file, 98.8% of a whole inspection pass.
        //
        // But the OPERANDS are only known to their own declared deviation. Refining the
        // distance between two curves five orders of magnitude finer than the geometry
        // those curves were reconstructed from is precision that does not exist. The
        // floor below is therefore the honest target, and because MinimumDistance already
        // publishes Budget().AddFlatten(curveTolerance), raising it makes the DECLARED
        // uncertainty correct automatically - the caller is told exactly what it got.
        //
        // Maximum is exempt: it exists precisely for callers who want the analytic answer
        // regardless of what the input declared, and it stays the oracle the differential
        // tests measure the other modes against.
        [[nodiscard]] double RefinementTolerance(const GeometryInstance& a,
                                                 const GeometryInstance& b,
                                                 const GeometryContext& context) noexcept
        {
            const double analytic = context.Tolerance().intersection;
            if (context.Precision() == PrecisionMode::Maximum)
                return analytic;

            // Both operands contribute: a point of A may be off by up to its own
            // deviation and a point of B by its own, so the distance between them
            // carries the sum. This is the same eA + eB that bounds the answer.
            const double declared = PreparedDeviation(a) + PreparedDeviation(b);
            return declared > analytic ? declared : analytic;
        }

        // A PAIR THAT HAS BEEN SCREENED - and the only way to say so.
        //
        // I07: every prepared pair query validates placement, checks cancellation BEFORE
        // working, and refuses when the query's limits are tighter than the ones its
        // derivative was prepared under. Until V8.1.2 that was a helper each API had to
        // remember to call. F13 forgot one. F18 forgot another in the same family, after
        // F13 had been closed - which is the definition of a repeatable class.
        //
        // The constructor is private, so a `ScreenedPair` cannot be forged; `Screen` is
        // the only thing that returns one, and screening is its body. `Bind` and
        // `BindTrees` - the two doors into a prepared derivative - take this instead of
        // two instances, so a query that skips the gate does not compile.
        //
        // It deliberately does NOT screen inside `Bind`. Two queries do a bounds reject
        // between screening and binding, and folding the gate into `Bind` would let a
        // cancelled context answer `false` instead of `Cancelled` - which is exactly the
        // divergence F18 measured, arriving from the other direction.
        class ScreenedPair
        {
        public:
            [[nodiscard]] static GeometryStatus Screen(const GeometryInstance& a,
                                                       const GeometryInstance& b,
                                                       const GeometryContext& context,
                                                       std::optional<ScreenedPair>& out) noexcept;

            [[nodiscard]] const GeometryInstance& A() const noexcept { return *a_; }
            [[nodiscard]] const GeometryInstance& B() const noexcept { return *b_; }

            // THE FINALIZER. I01: every answer this family publishes carries the error the
            // caller declared about its own sources, composed exactly once.
            //
            // It lives on the token because the token is what every prepared pair query
            // already holds, and because the two operands it composes are the two the
            // screening vouched for. Before V8.1.2 there were three implementations of
            // this - `Answer`, the `publish` lambda of the certified decision, and two
            // hand-written lines at the end of the distance solver - and the third had an
            // exit that returned before reaching them (F40, 1324): a pair of zero-segment
            // outlines got `Total()=0` and `exact=true` while its siblings published the
            // 1.00 mm the same operands declared, in the same call.
            //
            // MergeSequential and not Merge: a point of A may sit up to A's declared error
            // from where its caller believes it is, and a point of B likewise, so a
            // relation between the two carries the SUM. They do not compete.
            void Compose(ErrorBudget& budget) const noexcept
            {
                budget.MergeSequential(a_->Provenance());
                budget.MergeSequential(b_->Provenance());
            }

            // The answer, finished. `intrinsic` is what the query itself proved about its
            // own precision - the solver's interval half-width, say - and is empty for a
            // query whose answer is exact for the geometry the kernel represents.
            template <typename T>
            [[nodiscard]] GeometryResult<T> Publish(T value) const
            {
                GeometryResult<T> result = GeometryResult<T>::Success(std::move(value));
                Compose(result.Budget());
                return result;
            }

            template <typename T>
            [[nodiscard]] GeometryResult<T> Publish(T value, const ErrorBudget& intrinsic) const
            {
                GeometryResult<T> result = GeometryResult<T>::Success(std::move(value));
                result.Budget() = intrinsic;
                Compose(result.Budget());
                return result;
            }

        private:
            ScreenedPair(const GeometryInstance& a, const GeometryInstance& b) noexcept
                : a_(&a), b_(&b) {}

            const GeometryInstance* a_;
            const GeometryInstance* b_;
        };

        // Gathers everything both queries need and reports why it could not.
        [[nodiscard]] GeometryStatus Bind(const ScreenedPair& screened, PairContext& out)
        {
            const GeometryInstance& a = screened.A();
            const GeometryInstance& b = screened.B();

            if (!a.Valid() || !b.Valid())
                return GeometryStatus::InvalidInput;

            const PreparedShapeDefinition& definitionA = *a.Definition();
            const PreparedShapeDefinition& definitionB = *b.Definition();

            if (!definitionA.Hierarchy().Ok() || !definitionB.Hierarchy().Ok() ||
                !definitionA.Segments().Ok() || !definitionB.Segments().Ok())
            {
                // Not prepared, or preparation failed. Either way the caller must be
                // told rather than handed a "no collision" answer.
                return GeometryStatus::Unsupported;
            }

            // Every prepared query is metric: distances are returned in millimetres and
            // the narrow phase compares against millimetre tolerances. The traversal
            // works in A's local frame, and a local length only equals a world length
            // when the placement preserves distance. Under a 2x scale a 10 mm gap
            // measures 5 in A's frame, and returning that as millimetres - or comparing
            // a millimetre threshold against it - is a wrong answer that looks valid.
            //
            // Scale, non-uniform scale and skew are therefore refused rather than
            // mismeasured. Translation, rotation and reflection, the transforms nesting
            // actually places pieces with, are the supported set.
            if (!a.ToWorld().IsIsometry() || !b.ToWorld().IsIsometry())
                return GeometryStatus::Unsupported;

            Transform2 worldToA;
            if (!a.ToWorld().TryInvert(worldToA))
                return GeometryStatus::Degenerate;

            out.hierarchyA = &definitionA.Hierarchy().Value();
            out.hierarchyB = &definitionB.Hierarchy().Value();
            out.segmentsA = &definitionA.Segments().Value();
            out.segmentsB = &definitionB.Segments().Value();
            out.bToA = Transform2::Compose(worldToA, b.ToWorld());
            return GeometryStatus::Success;
        }

        [[nodiscard]] Segment TransformSegment(const Segment& segment, const Transform2& transform) noexcept
        {
            return { transform.Apply(segment.p0), transform.Apply(segment.c1),
                     transform.Apply(segment.c2), transform.Apply(segment.p1), segment.kind };
        }

        // Dual-hierarchy descent in A's frame.
        //
        // `separation` is the gap that still counts as a candidate: 0 for intersection
        // queries, the threshold for proximity queries, and the running best for the
        // minimum-distance query, which tightens as it goes.
        //
        // `visit(indexA, indexB)` returns false to stop. Returns false when stopped.
        template <typename Visitor>
        bool TraversePairs(const PairContext& pair, const double& separation, Visitor&& visit)
        {
            const SegmentBVH& treeA = *pair.hierarchyA;
            const SegmentBVH& treeB = *pair.hierarchyB;

            if (treeA.Empty() || treeB.Empty())
                return true;

            struct Entry { std::uint32_t a; std::uint32_t b; };
            Entry stack[SegmentBVH::kMaxBuildDepth * 6];
            int top = 0;
            stack[top++] = { 0u, 0u };

            while (top > 0)
            {
                const Entry entry = stack[--top];
                const SegmentBVH::Node& nodeA = treeA.Nodes()[entry.a];
                const SegmentBVH::Node& nodeB = treeB.Nodes()[entry.b];

                // B's box is mapped into A's frame. The AABB of a transformed AABB is
                // conservative, which is exactly what a broad phase needs - and the
                // return type of ApplyEnclosure now says which of the two it is.
                const Bounds2 boxB = pair.bToA.ApplyEnclosure(nodeB.bounds).Box();
                if (!nodeA.bounds.Overlaps(boxB, separation))
                    continue;

                if (nodeA.IsLeaf() && nodeB.IsLeaf())
                {
                    const std::uint32_t endA = nodeA.start + nodeA.count;
                    const std::uint32_t endB = nodeB.start + nodeB.count;

                    // B's primitive box depends only on j, and the old inner loop mapped
                    // it once per (i, j). Measured: 133 leaf combinations per overlapping
                    // query against 16 node visits, so this was most of the traversal.
                    // Visit order is unchanged.
                    Bounds2 mappedB[SegmentBVH::kDefaultLeafSize];
                    const bool cacheable = nodeB.count <= SegmentBVH::kDefaultLeafSize;
                    if (cacheable)
                        for (std::uint32_t j = nodeB.start; j < endB; ++j)
                            mappedB[j - nodeB.start] =
                                pair.bToA.ApplyEnclosure(
                                    treeB.PrimitiveBounds(treeB.Order()[j])).Box();

                    for (std::uint32_t i = nodeA.start; i < endA; ++i)
                    {
                        const std::uint32_t first = treeA.Order()[i];
                        const Bounds2& boundsA = treeA.PrimitiveBounds(first);
                        for (std::uint32_t j = nodeB.start; j < endB; ++j)
                        {
                            const std::uint32_t second = treeB.Order()[j];
                            const Bounds2 primitiveB =
                                cacheable ? mappedB[j - nodeB.start]
                                          : pair.bToA.ApplyEnclosure(
                                                treeB.PrimitiveBounds(second)).Box();
                            if (!boundsA.Overlaps(primitiveB, separation))
                                continue;
                            if (!visit(first, second))
                                return false;
                        }
                    }
                    continue;
                }

                if (!nodeA.IsLeaf() && (nodeB.IsLeaf() || nodeA.bounds.Area() >= boxB.Area()))
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

            return true;
        }

        // Minimum distance needs the opposite traversal discipline from an any-hit
        // query. TraversePairs prunes against `separation`, but for a distance search
        // that bound starts at infinity and only tightens once a leaf pair has actually
        // been measured, so a plain LIFO descent walks a large part of both hierarchies
        // before it can prune anything - which is how the prepared path ended up slower
        // than the raw O(n*m) loop it was meant to replace.
        //
        // Expanding the nearer child pair first reaches a genuinely close leaf pair
        // early, so `separation` is tight for the rest of the search. Bounds are kept
        // squared: the ordering only needs a monotone comparison, and the descent runs
        // often enough that a sqrt per node pair is worth avoiding.
        //
        // Kept separate from TraversePairs deliberately. Intersection queries report the
        // first hit they find, so reordering their traversal would change which
        // intersection they report - a behavioural change with no benefit to them.
        //
        // Precondition: `separation` must never grow during the traversal. A pair whose
        // lower bound exceeds it is dropped and not revisited, which is only sound while
        // the bound is non-increasing. Both callers satisfy this - the running minimum
        // only shrinks, and a threshold is constant.
        template <typename Visitor>
        bool TraverseNearestFirst(const PairContext& pair, const double& separation, Visitor&& visit)
        {
            const SegmentBVH& treeA = *pair.hierarchyA;
            const SegmentBVH& treeB = *pair.hierarchyB;

            if (treeA.Empty() || treeB.Empty())
                return true;

            struct Entry
            {
                std::uint32_t a;
                std::uint32_t b;
                double lowerBoundSquared;
            };

            // Bounded by depth(A) + depth(B): the stack only ever holds the not-yet-
            // visited sibling of each level on the current descent path, so twice the
            // build depth is the true ceiling and this is three times that. Same sizing
            // as TraversePairs.
            Entry stack[SegmentBVH::kMaxBuildDepth * 6];
            int top = 0;
            stack[top++] = { 0u, 0u, 0.0 };

            while (top > 0)
            {
                const Entry entry = stack[--top];

                // The bound was recorded when the pair was pushed; `separation` has
                // usually shrunk since, and re-testing here is what the ordering buys.
                if (entry.lowerBoundSquared > separation * separation)
                    continue;

                const SegmentBVH::Node& nodeA = treeA.Nodes()[entry.a];
                const SegmentBVH::Node& nodeB = treeB.Nodes()[entry.b];

                const Bounds2 boxB = pair.bToA.ApplyEnclosure(nodeB.bounds).Box();
                if (!nodeA.bounds.Overlaps(boxB, separation))
                    continue;

                if (nodeA.IsLeaf() && nodeB.IsLeaf())
                {
                    const std::uint32_t endA = nodeA.start + nodeA.count;
                    const std::uint32_t endB = nodeB.start + nodeB.count;
                    for (std::uint32_t i = nodeA.start; i < endA; ++i)
                    {
                        const std::uint32_t first = treeA.Order()[i];
                        for (std::uint32_t j = nodeB.start; j < endB; ++j)
                        {
                            const std::uint32_t second = treeB.Order()[j];
                            const Bounds2 primitiveB =
                                pair.bToA.ApplyEnclosure(treeB.PrimitiveBounds(second)).Box();
                            if (!treeA.PrimitiveBounds(first).Overlaps(primitiveB, separation))
                                continue;
                            if (!visit(first, second))
                                return false;
                        }
                    }
                    continue;
                }

                std::uint32_t childA[2];
                std::uint32_t childB[2];
                double bound[2];

                if (!nodeA.IsLeaf() && (nodeB.IsLeaf() || nodeA.bounds.Area() >= boxB.Area()))
                {
                    // Splitting A: B's box is unchanged, so the transform already done
                    // for this node pair is reused rather than repeated per child.
                    childA[0] = nodeA.LeftChild();  childB[0] = entry.b;
                    childA[1] = nodeA.RightChild(); childB[1] = entry.b;
                    bound[0] = treeA.Nodes()[childA[0]].bounds.DistanceSquared(boxB);
                    bound[1] = treeA.Nodes()[childA[1]].bounds.DistanceSquared(boxB);
                }
                else
                {
                    childA[0] = entry.a; childB[0] = nodeB.LeftChild();
                    childA[1] = entry.a; childB[1] = nodeB.RightChild();
                    for (int k = 0; k < 2; ++k)
                    {
                        const Bounds2 other =
                            pair.bToA.ApplyEnclosure(treeB.Nodes()[childB[k]].bounds).Box();
                        bound[k] = nodeA.bounds.DistanceSquared(other);
                    }
                }

                // LIFO: push the farther pair first so the nearer one is popped first
                // and tightens `separation` before the farther one is reconsidered.
                const int nearer = bound[0] <= bound[1] ? 0 : 1;
                const int farther = 1 - nearer;

                if (bound[farther] <= separation * separation)
                    stack[top++] = { childA[farther], childB[farther], bound[farther] };
                stack[top++] = { childA[nearer], childB[nearer], bound[nearer] };
            }

            return true;
        }
    }

    namespace
    {
        // Lemma B4 with the margin of BOUNDS_EVIDENCE_PROOF.md section 8.3.
        //
        // The witness is already shrunk by its own construction error, and the enclosure
        // is now widened by its own: `ApplyEnclosure` counts its four roundings and
        // widens by them, which is the F35a work this comment used to defer ("closing
        // that is F35's job, not this one"). Both sides of the comparison are therefore
        // certified in the direction they are used.
        //
        // The margin is KEPT anyway. It is not redundant: BOUNDS_EVIDENCE_PROOF.md
        // section 8.3 requires the refutation to clear the construction error of the
        // CONTAINER, and a margin costs only work - it moves the predicate the
        // conservative way, refuting less and handing more to the exact route. Removing
        // it would be trading a proof for a performance gain nobody measured.
        [[nodiscard]] bool RefutesContainment(const BoundsWitness& contained,
                                              const BoundsEnclosure& container,
                                              const GeometryContext& context) noexcept
        {
            if (contained.IsEmpty() || container.IsEmpty())
                return ProvesNotContained(contained, container, 0.0);

            const Bounds2& w = contained.Box();
            const Bounds2& e = container.Box();
            double scale = 1.0;
            for (const double v : { w.min.x, w.max.x, w.min.y, w.max.y,
                                    e.min.x, e.max.x, e.min.y, e.max.y })
            {
                const double m = std::fabs(v);
                if (m > scale) scale = m;
            }
            return ProvesNotContained(contained, container,
                                      context.Tolerance().ScaledEpsilon(scale));
        }
    }

    bool BoundsCouldTouch(const GeometryInstance& a, const GeometryInstance& b,
                          double tolerance) noexcept
    {
        // Lemma B2: enclosures apart prove the shapes apart. Only the negative
        // branch is a proof, and only the negative branch is read - by every caller.
        return a.Bounds().CouldTouch(b.Bounds(), tolerance);
    }

    namespace
    {
        // Checked before the world-AABB fast path, not inside Bind.
        //
        // The bounds reject is computed from world bounds and is sound under any
        // transform, so leaving the rigidity check downstream of it would answer
        // non-isometric pairs that happen to sit far apart and refuse the ones that sit
        // close. A precondition that depends on how far apart the operands are is not a
        // contract a caller can rely on, so the refusal comes first and applies always.
        [[nodiscard]] GeometryStatus ValidatePlacement(const GeometryInstance& a,
                                                       const GeometryInstance& b) noexcept
        {
            // Valid() covers finiteness: GeometryInstance establishes that invariant at
            // construction, so a non-finite placement can never reach a metric query and
            // the hot path pays one bool instead of twelve isfinite calls.
            if (!a.Valid() || !b.Valid())
                return GeometryStatus::InvalidInput;

            if (!a.ToWorld().IsIsometry() || !b.ToWorld().IsIsometry())
                return GeometryStatus::Unsupported;
            return GeometryStatus::Success;
        }

        // THE SINGLE ENTRY GATE FOR EVERY PREPARED COLLISION QUERY.
        //
        // Placement validity was already checked here by all five routes. Two more things
        // belong in the same place, and V8.1 had neither:
        //
        // CANCELLATION, UP FRONT. The raw routes report Cancelled for an already-cancelled
        // token; the prepared ones answered Success from their bounds and topology guards
        // without ever looking (evidence 892). A caller that cancelled a batch cannot tell
        // a finished answer from a short-circuited one, and prepared is supposed to be an
        // optimisation of raw, not a different product.
        //
        // LIMITS THE CALLER TIGHTENED AFTER PREPARATION. A derivative built under generous
        // limits must not let a later, stricter query be answered anyway - section 49
        // states it directly. Measured: four of six limits diverged, with raw returning
        // ComplexityLimit and prepared answering.
        //
        // The comparison is PreparationLimitsAreTighter - the limits that govern what a
        // definition CONTAINS. Query-time limits like maxConvexPairs are enforced by the
        // query itself on both routes, and gating on them here would refuse work that
        // would have finished inside the caller's own ceiling.
        //
        // Cost on the hot path: one relaxed atomic load for cancellation, and an integer
        // comparison chain that is skipped entirely when both operands were prepared by
        // the same context the query is using - the overwhelmingly common case, since the
        // session hands out both.
        GeometryStatus ScreenedPair::Screen(const GeometryInstance& a,
                                            const GeometryInstance& b,
                                            const GeometryContext& context,
                                            std::optional<ScreenedPair>& out) noexcept
        {
            if (const GeometryStatus placement = ValidatePlacement(a, b);
                !IsSuccess(placement))
            {
                return placement;
            }

            if (context.IsCancelled())
                return GeometryStatus::Cancelled;

            const ComplexityLimits& query = context.Limits();
            for (const GeometryInstance* instance : { &a, &b })
            {
                const PreparedShapeDefinitionPtr& definition = instance->Definition();
                if (definition == nullptr)
                    continue;
                if (PreparationLimitsAreTighter(query, definition->Context().Limits()))
                    return GeometryStatus::ComplexityLimit;
            }

            // The token is minted HERE and nowhere else. Copy-assignment into the
            // optional, because the constructor is private and `emplace` is not a friend.
            //
            // It carries the PAIR and not the context: the context it was screened against
            // is the one in scope at the call site, because screening and binding happen in
            // the same function body. That scoping is a convention, not a proof - a token
            // does not stop somebody from screening under one context and working under
            // another. Carrying the context would only move the convention, since nothing
            // would compare it either.
            out = ScreenedPair(a, b);
            return GeometryStatus::Success;
        }
    }

    namespace
    {
        // EVERY CALLER-VISIBLE ANSWER CARRIES THE ERROR THE CALLER DECLARED.
        //
        // A point of A may sit up to A provenance from where its caller believes it is,
        // and a point of B likewise, so a relation between the two carries the SUM.
        // MergeSequential, not Merge: both displacements happen, they do not compete.
        //
        // WHAT THE bool MEANS, now that the budget travels with it. The value is the
        // EXACT answer for the geometry the kernel represents; the budget is how far that
        // representation may be from the caller geometry. It is deliberately NOT a
        // morphological answer about the original - see section 4 of
        // PREPARED_BOOLEAN_QUERY_CERTIFICATION_CONTRACT.md for why topology admits no
        // such bound, and DistanceLessThanCertified for the query that does.
        //
        // V8.1.1 published exact=true and Total()=0 from all five of these while both
        // operands declared 0.5 mm, which made a represented false indistinguishable from
        // a certified one (evidence 1016). MinimumDistance already did this correctly and
        // is the control the differential measures the others against.
        // `Answer` and `QueryProvenance` used to live here. They were not wrong; they were
        // a SECOND name for what `ScreenedPair::Compose` does, and a second name is how a
        // third implementation gets written without anyone noticing. It did: the distance
        // solver composed by hand, and one of its exits returned before doing so (F40).
    }

    namespace
    {
    GeometryResult<bool> IntersectsUnguarded(const GeometryInstance& a, const GeometryInstance& b,
                                             const GeometryContext& context)
    {
        std::optional<ScreenedPair> screened;
        if (const GeometryStatus status = ScreenedPair::Screen(a, b, context, screened);
            !IsSuccess(status))
        {
            return GeometryResult<bool>::Failure(status);
        }

        if (!BoundsCouldTouch(a, b, context.Tolerance().intersection))
            return screened->Publish(false);

        PairContext pair;
        if (const GeometryStatus status = Bind(*screened, pair); !IsSuccess(status))
            return GeometryResult<bool>::Failure(status);

        const GeometryTolerance& tolerance = context.Tolerance();
        const double separation = tolerance.intersection;

        bool found = false;
        bool cancelled = false;
        std::uint64_t tested = 0;

        TraversePairs(pair, separation, [&](std::uint32_t i, std::uint32_t j)
        {
            if (context.ShouldCheckCancellation(static_cast<std::size_t>(tested)) && context.IsCancelled())
            {
                cancelled = true;
                return false;
            }
            ++tested;

            // Allocation-free any-hit, stopping at the first contact.
            const Segment transformed = TransformSegment((*pair.segmentsB)[j], pair.bToA);
            if (Intersect::AnySegmentIntersection((*pair.segmentsA)[i], transformed, tolerance))
            {
                found = true;
                return false;
            }
            return true;
        });

        CountStat(context, &GeometryStatistics::bvhCandidatePairs, tested);

        if (cancelled)
            return GeometryResult<bool>::Failure(GeometryStatus::Cancelled);

        return screened->Publish(found);
    }
    }

    GeometryResult<bool> Intersects(const GeometryInstance& a, const GeometryInstance& b,
                                    const GeometryContext& context)
    {
        return TranslateExhaustion<bool>([&] { return IntersectsUnguarded(a, b, context); });
    }

    namespace
    {
    GeometryResult<bool> FindFirstIntersectionUnguarded(const GeometryInstance& a,
                                                        const GeometryInstance& b,
                                                        const GeometryContext& context,
                                                        IntersectionPoint& out)
    {
        std::optional<ScreenedPair> screened;
        if (const GeometryStatus status = ScreenedPair::Screen(a, b, context, screened);
            !IsSuccess(status))
        {
            return GeometryResult<bool>::Failure(status);
        }

        if (!BoundsCouldTouch(a, b, context.Tolerance().intersection))
            return screened->Publish(false);

        PairContext pair;
        if (const GeometryStatus status = Bind(*screened, pair); !IsSuccess(status))
            return GeometryResult<bool>::Failure(status);

        const double separation = context.Tolerance().intersection;

        bool found = false;
        bool cancelled = false;
        std::uint64_t tested = 0;

        const GeometryTolerance& tolerance = context.Tolerance();

        TraversePairs(pair, separation, [&](std::uint32_t i, std::uint32_t j)
        {
            if (context.ShouldCheckCancellation(static_cast<std::size_t>(tested)) && context.IsCancelled())
            {
                cancelled = true;
                return false;
            }
            ++tested;

            const Segment transformed = TransformSegment((*pair.segmentsB)[j], pair.bToA);

            // Allocation-free: the narrow phase writes one point and stops, instead of
            // collecting every intersection of the pair into a vector and keeping the
            // first. A heap allocation per candidate pair is the one thing this query
            // cannot afford.
            IntersectionPoint hit;
            if (!Intersect::FindFirstSegmentIntersection((*pair.segmentsA)[i], transformed,
                                                         tolerance, hit))
            {
                return true;
            }

            // Map the contact back into world coordinates for the caller.
            out = hit;
            out.point = a.ToWorld().Apply(hit.point);
            found = true;
            return false;
        });

        if (cancelled)
            return GeometryResult<bool>::Failure(GeometryStatus::Cancelled);

        return screened->Publish(found);
    }
    }

    GeometryResult<bool> FindFirstIntersection(const GeometryInstance& a, const GeometryInstance& b,
                                               const GeometryContext& context,
                                               IntersectionPoint& out)
    {
        return TranslateExhaustion<bool>(
            [&] { return FindFirstIntersectionUnguarded(a, b, context, out); });
    }

    // THE ONE PLACE A THRESHOLD IS COMPARED AGAINST A DISTANCE.
    //
    // Both public forms come out of here. The bool form used to carry its own copy of the
    // comparison, which is how a kernel ends up with two answers to one question.
    static GeometryResult<DistanceDecision> DecideDistanceLessThan(const GeometryInstance& a,
                                                                   const GeometryInstance& b,
                                                                   double threshold,
                                                                   const GeometryContext& context)
    {
        // WHAT THE OPERANDS DECLARED ABOUT THEIR OWN SOURCES, subtracted from the decision
        // rather than only reported next to it.
        //
        // The solver's interval is about the geometry as REPRESENTED. A point of A may sit
        // up to A's declared error from where its caller believes it is, and a point of B
        // likewise, so a claim about the CALLER's geometry has to give both back:
        //
        //     upper + E < threshold  ->  True       lower - E >= threshold  ->  False
        //
        // The False side is `>=` and not `>`, and the two are not the same question. The
        // query asks whether the distance is LESS THAN the threshold, so a distance proved
        // equal to it answers False - and the world-AABB reject a few lines below has
        // always behaved that way. Using `>` on this path and `>=` on that one made the
        // same geometry answer differently depending on which route decided it.
        //
        // With E = 0 - which is every operand that declared nothing - these are exactly
        // the tests V8.1.2 already performed, so nothing moves. With E > 0 the False
        // region shrinks and those cases become Ambiguous, which the bool form answers as
        // `true`: the conservative side, and now a visibly conservative one.
        if (!(threshold >= 0.0) || !std::isfinite(threshold))
            return GeometryResult<DistanceDecision>::Failure(GeometryStatus::InvalidInput);

        std::optional<ScreenedPair> screened;
        if (const GeometryStatus status = ScreenedPair::Screen(a, b, context, screened);
            !IsSuccess(status))
        {
            return GeometryResult<DistanceDecision>::Failure(status);
        }

        // How much declared error the comparison below has to clear. Same composition the
        // finalizer applies, read as a number instead of published as a budget - and read
        // from the same place, so the two cannot drift apart.
        ErrorBudget queryProvenance;
        screened->Compose(queryProvenance);
        const double provenance = queryProvenance.Total();

        // ONE PLACE PRODUCES THE RESULT, and it is the finalizer every sibling uses.
        const auto publish = [&](DistanceDecisionState state, double lower, double upper)
        {
            DistanceDecision decision;
            decision.state = state;
            decision.lower = lower;
            decision.upper = upper;
            decision.appliedProvenance = provenance;
            return screened->Publish(decision);
        };

        // The world-AABB reject has to clear the threshold AND the provenance, or a pair
        // whose true geometry could be within the threshold would be rejected because its
        // represented geometry is not.
        if (!BoundsCouldTouch(a, b, threshold + provenance))
        {
            return publish(DistanceDecisionState::False,
                           threshold + provenance,
                           std::numeric_limits<double>::infinity());
        }

        PairContext pair;
        if (const GeometryStatus status = Bind(*screened, pair); !IsSuccess(status))
            return GeometryResult<DistanceDecision>::Failure(status);

        const double curveTolerance = RefinementTolerance(a, b, context);

        DistanceDecisionState state = DistanceDecisionState::False;
        bool cancelled = false;

        // The solver can fail, and a query that could not certify its own answer does not
        // get to publish a decision. Before F27 this variable had nothing to hold: an
        // allocation failure inside CurveCurveBounds killed the process (evidence 1144).
        GeometryStatus solverStatus = GeometryStatus::Success;
        std::uint64_t tested = 0;

        // THE REPORTED INTERVAL, accumulated without adding a square root to the hot path.
        //
        // Pairs settled by the probe are tracked in SQUARED space and converted once at
        // the end; `probeSlack` is the largest widening any of them needed, so
        // sqrt(min s_i) - max slack_i is a valid lower bound on min (s_i - slack_i).
        // Pairs that reached the solver already carry a millimetre lower bound.
        double witnessSquared = std::numeric_limits<double>::infinity();
        double probeMinSquared = std::numeric_limits<double>::infinity();
        double probeSlack = 0.0;
        double solverLower = std::numeric_limits<double>::infinity();

        // Fixed pruning radius: any pair already farther apart than the threshold cannot
        // satisfy it, and the first pair that decides ends the query.
        //
        // Nearest-first, like MinimumDistance. A generous threshold lets a large number
        // of primitive pairs through the bounds test, and each survivor costs a full
        // cubic-cubic proximity solve, so the order the candidates are examined in
        // decides whether the early exit fires on the first one or the thousandth.
        TraverseNearestFirst(pair, threshold + provenance, [&](std::uint32_t i, std::uint32_t j)
        {
            if (context.ShouldCheckCancellation(static_cast<std::size_t>(tested)) && context.IsCancelled())
            {
                cancelled = true;
                return false;
            }
            ++tested;

            const Segment transformed = TransformSegment((*pair.segmentsB)[j], pair.bToA);
            CountStat(context, &GeometryStatistics::distanceCandidatePairs);

            const double sampledSquared =
                OnCurveUpperBoundSquared((*pair.segmentsA)[i], transformed);
            CountStat(context, &GeometryStatistics::distanceSeedProbes);
            if (sampledSquared < witnessSquared)
                witnessSquared = sampledSquared;

            // The probe returns the distance between two points it COMPUTED on the two
            // curves. Over the reals those points are ON the curves and the value needs no
            // tolerance term; in IEEE-754 it can sit a rounding below the distance between
            // the points it stands for. `True` is a claim of proof, so the witness is
            // widened before it is allowed to make one - the same absolute, coordinate
            // keyed widening F36 needed on the other side of the same routine.
            const double slack = ClosedFormSlack((*pair.segmentsA)[i], transformed);

            // FILTERED ACCEPT, in squared space so no square root joins the hot path:
            // with margin = threshold - E - slack >= 0,
            //     sampled + slack + E < threshold   <=>   sampledSquared < margin^2
            const double acceptMargin = threshold - provenance - slack;
            if (acceptMargin > 0.0 && sampledSquared < acceptMargin * acceptMargin)
            {
                CountStat(context, &GeometryStatistics::distanceBoundedAccepts);
                state = DistanceDecisionState::True;
                return false;
            }

            // For two lines the probe returned the closed form, so the solver would only
            // recompute it. Already-flattened operands are nothing but lines, which is why
            // this is worth a branch rather than a comment.
            if ((*pair.segmentsA)[i].IsLine() && transformed.IsLine())
            {
                // Proves BEYOND when sampled - slack - E > threshold, again squared:
                //     sampledSquared > (threshold + E + slack)^2
                const double beyond = threshold + provenance + slack;
                if (sampledSquared >= beyond * beyond)
                {
                    CountStat(context, &GeometryStatistics::distanceBoundedRejects);
                    if (sampledSquared < probeMinSquared)
                        probeMinSquared = sampledSquared;
                    if (slack > probeSlack)
                        probeSlack = slack;
                    return true;
                }

                // Neither proved. The closed form has nothing finer to offer, so this pair
                // is where the query stops being able to answer.
                state = DistanceDecisionState::Ambiguous;
                return false;
            }

            // DECIDED BY INTERVAL, never by an uncertified scalar.
            //
            // CurveCurveBounds returns [lower, upper] for this pair with no depth cap, so
            // the two decisive tests below are exactly the ones DISTANCE_INTERVAL_PROOF.md
            // section 9 licenses. V8.1.1 resolved the ambiguous band by calling the SAME
            // solver at a finer tolerance and comparing the returned double against the
            // threshold - with no band, and without asking for the deviation the signature
            // offered. The cap fires on real geometry: 6 of 6 adversarial fixtures,
            // overstating clearance by up to 3.74e-06 mm (evidence 1025).
            CountStat(context, &GeometryStatistics::distanceExactRefinements);

            // SOFT AND HARD, SIDE BY SIDE AND NAMED. maxDistanceRefinements is a soft
            // preference living in a struct whose first sentence says "hard ceilings:
            // exceeded => ComplexityLimit" - I08, still open.
            //
            // decideAt and upperSeed stay at the bare threshold rather than threshold + E.
            // The seed is a pruning aid and the decision point is only a place to stop
            // early; the E-aware comparison happens below, on the published interval. For
            // E = 0 the two are the same number, which is why nothing moves there.
            Proximity::DistanceSolveControl control;
            control.targetWidth = curveTolerance;
            control.upperSeed = threshold;
            control.decideAt = threshold;
            control.refinementBudget = context.Limits().maxDistanceRefinements;

            Proximity::DistanceBounds bounds = Proximity::CurveCurveBounds(
                (*pair.segmentsA)[i], transformed, control, context);
            CountStat(context, &GeometryStatistics::curveSubdivisions, bounds.subdivisions);
            if (!IsSuccess(bounds.status))
            {
                solverStatus = bounds.status;
                return false;
            }

            // Still overlapping the threshold after refinement. Refine once more at the
            // analytic target before giving up on separating it.
            if (!(bounds.upper + provenance < threshold) &&
                !(bounds.lower - provenance >= threshold))
            {
                Proximity::DistanceSolveControl fine = control;
                fine.targetWidth = context.Tolerance().intersection;
                fine.upperSeed = bounds.upper;

                const Proximity::DistanceBounds refined = Proximity::CurveCurveBounds(
                    (*pair.segmentsA)[i], transformed, fine, context);
                CountStat(context, &GeometryStatistics::curveSubdivisions, refined.subdivisions);
                if (!IsSuccess(refined.status))
                {
                    solverStatus = refined.status;
                    return false;
                }

                // ONLY THE FINAL INTERVAL FOR THIS PAIR IS KEPT. Feeding both to the
                // accumulator below would be F30 in a new function: the first, looser
                // bound would survive in a minimum that never rises.
                bounds = refined;
            }

            if (bounds.upper + provenance < threshold)
            {
                state = DistanceDecisionState::True;
                return false;
            }
            if (bounds.lower - provenance >= threshold)
            {
                CountStat(context, &GeometryStatistics::distanceBoundedRejects);
                if (bounds.lower < solverLower)
                    solverLower = bounds.lower;
                return true;
            }

            state = DistanceDecisionState::Ambiguous;
            return false;
        });

        if (cancelled)
            return GeometryResult<DistanceDecision>::Failure(GeometryStatus::Cancelled);
        if (!IsSuccess(solverStatus))
            return GeometryResult<DistanceDecision>::Failure(solverStatus);

        // ONE square root, after the traversal rather than inside it.
        double lower = solverLower;
        if (probeMinSquared < std::numeric_limits<double>::infinity())
        {
            double fromProbe = std::sqrt(probeMinSquared) - probeSlack;
            if (fromProbe < 0.0)
                fromProbe = 0.0;
            if (fromProbe < lower)
                lower = fromProbe;
        }
        if (lower == std::numeric_limits<double>::infinity())
            lower = 0.0;

        const double upper = witnessSquared < std::numeric_limits<double>::infinity()
                                 ? std::sqrt(witnessSquared)
                                 : std::numeric_limits<double>::infinity();

        return publish(state, lower, upper);
    }

    GeometryResult<DistanceDecision> DistanceLessThanCertified(const GeometryInstance& a,
                                                               const GeometryInstance& b,
                                                               double threshold,
                                                               const GeometryContext& context)
    {
        // F37's mould, now the shared one. This body does not allocate today, and the
        // translation is here so that it may - a query whose contract is a status must
        // not become one whose contract is an exception because someone added a
        // container.
        return TranslateExhaustion<DistanceDecision>(
            [&] { return DecideDistanceLessThan(a, b, threshold, context); });
    }

    GeometryResult<bool> DistanceLessThan(const GeometryInstance& a, const GeometryInstance& b,
                                          double threshold, const GeometryContext& context)
    {
        const GeometryResult<DistanceDecision> decision =
            DistanceLessThanCertified(a, b, threshold, context);
        if (!decision.Ok())
            return GeometryResult<bool>::Failure(decision.Status());

        // AMBIGUOUS -> true, and this line is the only place that policy exists.
        //
        // For a proximity question the conservative side is "might be near": saying two
        // parts are close when they are not costs material, saying they are apart when
        // they might not be puts one on top of the other. That is a POLICY and not a
        // proof, which is exactly why DistanceDecisionState exists - a caller that must
        // tell the two apart asks the certified form.
        // THE ONE PLACE THAT BUILDS A RESULT WITHOUT THE FINALIZER, on purpose.
        //
        // The certified form already went through `Publish`, so its budget already carries
        // the composed caller provenance. Running the finalizer again here would compose it
        // a SECOND time and double what the caller reads - the failure mode
        // `ProvenanceIsComposedExactlyOnce` measures as slope 4 instead of 2. This forwards
        // a finished budget; it does not make one.
        GeometryResult<bool> result = GeometryResult<bool>::Success(
            decision.Value().state != DistanceDecisionState::False);
        result.Budget() = decision.Budget();
        return result;
    }

    // NOT the entry point: see the wrapper below. This body allocates - the certificate
    // ledger holds one entry per curved leaf pair - and F37 is that the prepared query
    // family never translated a failed allocation into a status.
    static GeometryResult<double> SolveMinimumDistance(const GeometryInstance& a,
                                                       const GeometryInstance& b,
                                                       const GeometryContext& context)
    {
        // THE SAME GATE AS ITS FIVE SIBLINGS.
        //
        // V8.1.1 gave ScreenQuery to Intersects, FindFirstIntersection, DistanceLessThan,
        // Overlaps and Contains, and left this one opening with Bind. Bind checks
        // placement; it does not check cancellation and it does not compare the query
        // limits against the limits the derivative was prepared under.
        //
        // Measured (evidence 1025): with maxSegments, maxContours, maxFlattenPoints or
        // maxPolygonVertices tightened AFTER preparation, Overlaps returned
        // ComplexityLimit on all four and MinimumDistance returned Success on all four.
        // Four divergences out of four, inside the family whose closure F13 was.
        //
        // Cancellation was already covered here, but incidentally: the in-traversal check
        // fires because the minimum-distance traversal has no separation bound and always
        // reaches a leaf pair. That is a property of this query, not a guarantee, and it
        // would vanish the day the fast path gained an early exit.
        std::optional<ScreenedPair> screened;
        if (const GeometryStatus status = ScreenedPair::Screen(a, b, context, screened);
            !IsSuccess(status))
        {
            return GeometryResult<double>::Failure(status);
        }

        PairContext pair;
        if (const GeometryStatus status = Bind(*screened, pair); !IsSuccess(status))
            return GeometryResult<double>::Failure(status);

        const double curveTolerance = RefinementTolerance(a, b, context);

        // THE GLOBAL INTERVAL, assembled in ONE place.
        //
        // `upper` is the smallest distance between two points that lie on the two
        // outlines, so it is an upper bound with no tolerance term at all, and a running
        // minimum is right for it: a witness is final by construction.
        //
        // `lower` is where V8.1.2 had the defect. It was a running minimum too - over
        // quantities that get REFINED. A pair solved early was seeded with whatever upper
        // bound existed at the time, and a loose seed produces a loose lower bound; the
        // second pass then recomputes that pair without a seed and gets a HIGHER bound,
        // which `min` cannot accept. Measured: 120 pairs whose lower a refresh raises,
        // worst raise 75.36 (evidence 1152). F30.
        //
        // GlobalDistanceLedger separates the two kinds of contribution and REPLACES the
        // refinable ones per owner. See the long comment on the type in Distance.hpp.
        //
        // WHY THAT MATTERS ONE LEVEL DOWN TOO (F20). V8.1.1 tracked the chord deviation of
        // the pair that owned `best` and REASSIGNED it on every change of owner, so a flat
        // pair winning last erased the deviation of a capped pair accepted earlier. Here a
        // leaf that cannot be resolved holds a certificate of its own until something
        // replaces it.
        Proximity::GlobalDistanceLedger ledger;
        bool anyFallback = false;

        bool cancelled = false;
        GeometryStatus solverStatus = GeometryStatus::Success;
        std::uint64_t tested = 0;

        // Read by the traversal on every node pair, so each improvement immediately
        // prunes the rest of the search. It only ever shrinks, which is the precondition
        // TraverseNearestFirst documents.
        double separation = std::numeric_limits<double>::infinity();

        TraverseNearestFirst(pair, separation, [&](std::uint32_t i, std::uint32_t j)
        {
            if (context.ShouldCheckCancellation(static_cast<std::size_t>(tested)) && context.IsCancelled())
            {
                cancelled = true;
                return false;
            }
            ++tested;

            const Segment transformed = TransformSegment((*pair.segmentsB)[j], pair.bToA);
            CountStat(context, &GeometryStatistics::distanceCandidatePairs);

            // BOUND THE SOLVER BEFORE CALLING IT. The running upper bound is handed to the
            // pair solver as well as to the traversal: the traversal prunes at the box
            // level - measured, 23 leaf pairs out of 40,000 - but each of those was then
            // resolved in isolation at 762 us because the solver had no bound of its own.
            // That, not the traversal, was the 17 ms.
            const double sampled = std::sqrt(
                OnCurveUpperBoundSquared((*pair.segmentsA)[i], transformed));
            CountStat(context, &GeometryStatistics::distanceSeedProbes);
            ledger.Witness(sampled);

            // Two lines: the probe IS the answer for this pair, and it can never be
            // refined - so it is a FINAL contribution, not a certificate. That is also
            // what keeps a line-line query at zero allocations.
            //
            // F36, and it is the F31 class in the one place stage B did not reach.
            // OnCurveUpperBoundSquared returns SegmentSegmentSquared for two lines, and
            // its own comment says the value "is still a valid UPPER bound". V8.1.2 fed
            // that same number to globalLower as if it were a lower bound too. Over the
            // reals they coincide; in IEEE-754 the closed form returns the distance
            // between two points it COMPUTED on the segments, which can only sit at or
            // above the true minimum. Stage B widened exactly this quantity inside
            // CurveCurveBounds and left the parallel shortcut here untouched.
            if ((*pair.segmentsA)[i].IsLine() && transformed.IsLine())
            {
                const double slack = ClosedFormSlack((*pair.segmentsA)[i], transformed);
                double exact = sampled - slack;
                if (exact < 0.0)
                    exact = 0.0;
                ledger.Final(exact);
                separation = ledger.Upper() > 0.0 ? ledger.Upper() : 0.0;
                return ledger.Upper() > 0.0;
            }

            CountStat(context, &GeometryStatistics::distanceExactRefinements);

            // No decideAt: this is a MEASUREMENT, so the solver narrows the interval
            // rather than settling a side. The running upper bound is the pruning seed
            // and nothing more - a pruned piece contributes the seed as its lower bound,
            // never as a witness.
            Proximity::DistanceSolveControl control;
            control.targetWidth = curveTolerance;
            control.upperSeed = ledger.Upper();
            control.refinementBudget = context.Limits().maxDistanceRefinements;

            const Proximity::DistanceBounds bounds = Proximity::CurveCurveBounds(
                (*pair.segmentsA)[i], transformed, control, context);
            CountStat(context, &GeometryStatistics::curveSubdivisions, bounds.subdivisions);
            if (!IsSuccess(bounds.status))
            {
                solverStatus = bounds.status;
                return false;
            }
            if (bounds.usedFallback)
                anyFallback = true;

            // ONE certificate per owner. This pair may be refined below, and when it is,
            // the certificate is REPLACED rather than merged into a minimum.
            ledger.Certify({ i, j }, bounds.lower);
            ledger.Witness(bounds.upper);

            separation = ledger.Upper() > 0.0 ? ledger.Upper() : 0.0;
            return ledger.Upper() > 0.0;
        });

        if (cancelled)
            return GeometryResult<double>::Failure(GeometryStatus::Cancelled);
        if (!IsSuccess(solverStatus))
            return GeometryResult<double>::Failure(solverStatus);

        // Neither hierarchy delivered a pair - both outlines are empty.
        //
        // This exit used to build its own result and return before the two composition
        // lines at the bottom of the function, so it published `Total()=0` and
        // `exact=true` for operands that had declared error - F40, measured in 1324. It
        // cannot happen through the finalizer, which is the point of there being one.
        if (ledger.Upper() == std::numeric_limits<double>::infinity())
            return screened->Publish(std::numeric_limits<double>::infinity());

        // SECOND PASS. Only the pairs that could still hold the minimum, and only when
        // their first answer is loose enough to matter.
        //
        // Read by index and copied by value: Certify() below writes into the vector this
        // walks. It cannot grow it - the owner already exists, so no entry is appended -
        // but a reference into a container being written to is a habit worth not having.
        for (std::size_t k = 0; k < ledger.Count(); ++k)
        {
            const Proximity::PairCertificate v = ledger.At(k);

            if (v.lower >= ledger.Upper())
                continue;                       // cannot hold the minimum (Lemma 4)
            if (ledger.Upper() - v.lower <= curveTolerance)
                continue;                       // already tight enough to publish

            const Segment transformed =
                TransformSegment((*pair.segmentsB)[v.owner.b], pair.bToA);
            // NO SEED. The seed is a pruning bound, and a pruned piece contributes the
            // seed itself as its lower bound - so seeding this pass would reintroduce
            // exactly the pollution it exists to remove. Without a seed the pair is
            // resolved on its own merits and its lower bound is its own.
            Proximity::DistanceSolveControl refresh;
            refresh.targetWidth = curveTolerance;
            refresh.upperSeed = std::numeric_limits<double>::infinity();
            refresh.refinementBudget = context.Limits().maxDistanceRefinements;

            const Proximity::DistanceBounds refreshed = Proximity::CurveCurveBounds(
                (*pair.segmentsA)[v.owner.a], transformed, refresh, context);
            CountStat(context, &GeometryStatistics::curveSubdivisions, refreshed.subdivisions);
            if (!IsSuccess(refreshed.status))
                return GeometryResult<double>::Failure(refreshed.status);
            if (refreshed.usedFallback)
                anyFallback = true;

            ledger.Witness(refreshed.upper);

            // THE REPLACEMENT. This is the line F30 is about. The old contribution of this
            // owner does not survive anywhere - not in a running minimum, not in a cache.
            ledger.Certify(v.owner, refreshed.lower);
        }

        if (ledger.Spilled())
            CountStat(context, &GeometryStatistics::distanceLedgerSpills);

        // ONE PLACE PRODUCES THE PUBLISHED INTERVAL, and the clamps live inside it.
        const Proximity::GlobalDistanceInterval interval = ledger.Publish();
        const double globalLower = interval.lower;
        const double globalUpper = interval.upper;

        // Already world millimetres: Bind refused anything but an isometric placement, so
        // A's local frame and world space measure the same lengths.
        //
        // THE MIDPOINT, so that `value +/- Budget().Total()` is EXACTLY the interval that
        // was proved - no looser and no tighter. Publishing `upper` with a budget of
        // `upper - lower` would also be correct and would widen the published band to
        // [lower, 2*upper - lower], whose upper half was never demonstrated. Publishing
        // more uncertainty than was proved moves parts apart for a reason that does not
        // exist. Derivation in DISTANCE_INTERVAL_PROOF.md, section 7.
        //
        // FOR A NESTING CONSUMER: the guaranteed clearance is
        //     value - Budget().Total()
        // and it is that, never `value`, that a clearance must be compared against.
        ErrorBudget intrinsic;
        intrinsic.AddFlatten((globalUpper - globalLower) * 0.5);
        (void)anyFallback;

        // Caller provenance, composed EXACTLY ONCE, one operand at a time.
        //
        // A point of A may sit up to A's declared error from where its caller believes it
        // is, and a point of B likewise, so the distance between them carries the SUM.
        // This is a stage on top of the solver's own error rather than an alternative to
        // it, which is why it is sequential and not Merge.
        //
        // No double count with RefinementTolerance: that reads
        // FlattenedContour::toleranceUsed, which is the purely geometric deviation of the
        // flattened polyline from its curve. It deliberately does NOT read
        // FlattenedPath::budget precisely because that one already carries provenance -
        // see the comment on PreparedDeviation. The two quantities are different and each
        // is counted once.
        return screened->Publish((globalLower + globalUpper) * 0.5, intrinsic);
    }

    GeometryResult<double> MinimumDistance(const GeometryInstance& a, const GeometryInstance& b,
                                           const GeometryContext& context)
    {
        // F37. Exhaustion is a status, not an exception thrown at a caller that was
        // handed a GeometryResult. This route allocates 7 to 8 times per curved query
        // (evidence 1215). The two siblings that allocate far more heavily - Overlaps and
        // Contains, through WorldPath() and Boolean - used to be left out, on the grounds
        // that a wrapper should not appear in a function nobody measured. They are in now,
        // through the same gateway rather than through a copy of it.
        return TranslateExhaustion<double>([&] { return SolveMinimumDistance(a, b, context); });
    }

    namespace
    {
        // Maps a point from one instance's WORLD frame into another definition's LOCAL
        // frame, so it can be classified against that definition's prepared tree.
        [[nodiscard]] bool WorldToLocal(const GeometryInstance& instance, Vec2 world, Vec2& out)
        {
            Transform2 inverse;
            if (!instance.ToWorld().TryInvert(inverse))
                return false;
            out = inverse.Apply(world);
            return true;
        }

        // Both trees, or Unsupported. Overlaps and Contains are region queries and a
        // region needs a topology; CollisionReady does not build one.
        [[nodiscard]] GeometryStatus BindTrees(const ScreenedPair& screened,
                                               const ContainmentTree*& treeA,
                                               const ContainmentTree*& treeB)
        {
            const GeometryInstance& a = screened.A();
            const GeometryInstance& b = screened.B();

            if (!a.Valid() || !b.Valid())
                return GeometryStatus::InvalidInput;

            const GeometryResult<ContainmentTree>& resultA = a.Definition()->Topology();
            const GeometryResult<ContainmentTree>& resultB = b.Definition()->Topology();
            if (!resultA.Ok() || !resultB.Ok())
                return GeometryStatus::Unsupported;

            treeA = &resultA.Value();
            treeB = &resultB.Value();
            return GeometryStatus::Success;
        }

        // Whether the filled region is ONE connected component.
        //
        // This is the precondition every shortcut below actually needs, and the one V8.1
        // assumed without stating. A ring marked Outer contributes a filled component; its
        // holes subtract from that same component and cannot split it, because non-crossing
        // holes are strictly interior and disjoint from one another. So exactly one Outer
        // node means exactly one connected filled region.
        //
        // Counting `roots` instead would be wrong, and only just: an island of material
        // inside a hole is a CHILD of that hole, never a root, yet it is a separate
        // connected component of the set. roots.size() == 1 would call a ring-hole-island
        // stack connected. Counting Outer roles sees the island.
        [[nodiscard]] bool IsSingleConnectedRegion(const ContainmentTree& region) noexcept
        {
            std::size_t outerCount = 0;
            for (const ContainmentNode& node : region.nodes)
            {
                if (node.role != ContourRole::Outer)
                    continue;
                if (++outerCount > 1)
                    return false;
            }
            return outerCount == 1;
        }

        // An instance whose local geometry has no contours. The empty set is a value, not
        // a failure, and the raw route states what it means (Collision.cpp:130-133).
        [[nodiscard]] bool InstanceIsEmpty(const GeometryInstance& instance) noexcept
        {
            const PreparedShapeDefinitionPtr& definition = instance.Definition();
            return definition == nullptr || definition->LocalPath().contours.empty();
        }

        // Every vertex of a flattened ring, mapped to world, inside `window`.
        //
        // Sufficient AND exact: `window` is a box, boxes are convex, and a polyline lies
        // inside the convex hull of its own vertices. So all vertices in means the whole
        // ring is in, with no tolerance term and no approximation.
        //
        // Early exit on the first vertex outside. That is what keeps this affordable: a
        // large outer ring against a small window fails on its first or second point, and
        // the only rings walked to the end are the ones that really are inside - after
        // which the caller takes the exact Boolean route, which costs orders of magnitude
        // more than the walk.
        [[nodiscard]] bool RingIsWithin(const std::vector<Vec2>& points,
                                        const Transform2& toWorld, const Bounds2& window) noexcept
        {
            if (points.empty())
                return false;
            for (const Vec2 point : points)
                if (!window.Contains(toWorld.Apply(point)))
                    return false;
            return true;
        }

        // Sound only once Intersects has ruled out a crossing: a ring not inside
        // `window` cannot enter it, so the fill there is constant and one point decides.
        // A ring that IS inside partitions the window, and no point speaks for both.
        //
        // WHY THE BOX TEST IS NOT THE WHOLE ANSWER.
        //
        // `node.bounds` is a LOCAL AABB and Transform2::Apply(Bounds2) returns the AABB
        // OF the transformed AABB, which under rotation is strictly larger than the
        // ring's true world extent. Writing R for the ring's true world bounds and T for
        // that box, R is a subset of T, and the two directions are NOT symmetric:
        //
        //     window contains T          ->  window contains R  ->  ring is inside   SOUND
        //     window does not contain T  ->  says nothing about R                  UNSOUND
        //
        // V8.1.1 used only the second row, and it is the row that returns true - the
        // answer that authorises the single-point shortcut. Measured: a local diamond of
        // radius 10 rotated 45 degrees becomes a ring 14.142 wide inside a box 28.284
        // wide, so a window between those two widths contains the ring and not the box.
        // Contains then reported a part sitting over 200 mm^2 of void as contained, and
        // Overlaps reported two shapes sharing 200 mm^2 of material as disjoint. The
        // answer flipped exactly at 28.284/2, which is a property of the approximation
        // and not of the geometry (evidence 1013).
        //
        // The fix is not a wider tolerance - a larger over-approximation demonstrates
        // LESS, not more. Section 171: broad phase filters, and the exact test decides.
        // So the box keeps both of its SOUND uses, as a two-sided filter:
        //
        //     window contains T      ring is certainly inside      -> decided, no walk
        //     window misses T        ring is certainly not inside   -> decided, no walk
        //     otherwise              undecided                      -> ask the ring
        //
        // Under identity, translation, axis mirrors and quarter turns the transformed
        // AABB is exact, so the two filters decide every node and no ring is ever walked.
        // That is the whole of the nesting fast lane, unchanged.
        [[nodiscard]] bool FillIsUniformWithin(const ContainmentTree& region,
                                               const Transform2& toWorld, const Bounds2& window)
        {
            const std::size_t count = region.nodes.size();
            const BoundsEnclosure windowEvidence = BoundsEnclosure::Enclosing(window);
            for (std::size_t i = 0; i < count; ++i)
            {
                const BoundsEnclosure conservative =
                    toWorld.ApplyEnclosure(region.nodes[i].bounds);

                // Certainly inside: the ring is contained in a box the window contains.
                // Lemma B3, and only its true branch - which is the F16 fix, restated by
                // the type: `ProvesInside` is the only containment question an enclosure
                // is allowed to answer.
                if (conservative.ProvesInside(window))
                    return false;

                // Certainly not inside: the ring is contained in a box that does not
                // even reach the window. Lemma B2.
                if (!conservative.CouldTouch(windowEvidence, 0.0))
                    continue;

                // The approximation cannot decide this one, so the ring answers for
                // itself. `rings` is index-aligned with `nodes` by construction; if a
                // tree ever arrives without them the shortcut is DECLINED rather than
                // granted, because "I could not check" is not "I proved it is not there".
                if (i >= region.rings.size())
                    return false;

                if (RingIsWithin(region.rings[i].points, toWorld, window))
                    return false;
            }
            return true;
        }

        [[nodiscard]] bool RepresentativePointInside(const GeometryInstance& inner,
                                                     const ContainmentTree& innerTree,
                                                     const GeometryInstance& outer,
                                                     const ContainmentTree& outerTree,
                                                     const GeometryContext& context)
        {
            if (!innerTree.hasRepresentativePoint)
                return false;

            const Vec2 world = inner.ToWorld().Apply(innerTree.representativePoint);

            Vec2 local;
            if (!WorldToLocal(outer, world, local))
                return false;

            return Topology::ClassifyPoint(outerTree, local, context) != PointClassification::Outside;
        }
    }

    namespace
    {
    GeometryResult<bool> OverlapsUnguarded(const GeometryInstance& a, const GeometryInstance& b,
                                           const GeometryContext& context)
    {
        std::optional<ScreenedPair> screened;
        if (const GeometryStatus status = ScreenedPair::Screen(a, b, context, screened);
            !IsSuccess(status))
        {
            return GeometryResult<bool>::Failure(status);
        }

        // Same first reject as every other prepared query, so a disjoint pair costs
        // exactly what Intersects costs.
        if (!BoundsCouldTouch(a, b, context.Tolerance().intersection))
            return screened->Publish(false);

        // Unguarded on purpose: this call is ALREADY inside Overlaps's own
        // TranslateExhaustion frame, so routing it through the public Intersects
        // would establish a second, redundant unwind region on the hottest prepared
        // query in the kernel. The contract is unchanged - a bad_alloc from here
        // still lands on the outer gateway - and it is one measured frame cheaper.
        const auto crossing = IntersectsUnguarded(a, b, context);
        if (!crossing.Ok())
            return crossing;
        if (crossing.Value())
            return screened->Publish(true);

        // No crossing means "nested or disjoint" - for ONE connected region against
        // another. That dichotomy is what makes a global bounds box decisive, and it does
        // not survive a multipart operand, which can be PARTLY nested and PARTLY disjoint
        // at the same time.
        //
        // V8.1 drew the conclusion unconditionally. For A = [0,0]-[10,10] against
        // B = [1,1]-[2,2] union [20,20]-[21,21], no boundary crosses, B's box does not
        // contain A's (0 < 1) and A's does not contain B's (21 > 10), so it answered
        // false while the sets share the unit square [1,1]-[2,2]. A missed overlap is a
        // missed collision.
        //
        // The shortcut is kept where it is provably valid and the exact set question is
        // asked everywhere else. It is not replaced by a wider tolerance or a different
        // heuristic: below is the same answer the raw route publishes.
        // Topology is bound FIRST, and its absence is still Unsupported.
        //
        // Region queries need a ContainmentTree and say so rather than returning a
        // plausible false at CollisionReady - that is the documented contract, pinned by
        // HardeningV5_PreparedRegion::RegionQueriesNeedBooleanReady. Falling back to
        // Boolean on WorldPath() here would answer it instead, by MATERIALISING the world
        // geometry that this whole design exists to avoid building. The bounds shortcut
        // below also has to move behind this bind, because deciding whether it is sound
        // requires knowing how many components each operand has.
        const ContainmentTree* treeA = nullptr;
        const ContainmentTree* treeB = nullptr;
        if (const GeometryStatus status = BindTrees(*screened, treeA, treeB);
            !IsSuccess(status))
        {
            return GeometryResult<bool>::Failure(status);
        }

        if (IsSingleConnectedRegion(*treeA) && IsSingleConnectedRegion(*treeB))
        {
            // FILTERS. Enclosure against enclosure decides nothing on its own; a
            // `false` here only means "do not try this shortcut", and the exact route
            // below is what answers. Over-estimating costs work, never correctness.
            const bool aCouldBeInsideB = b.Bounds().Box().Contains(a.Bounds().Box());
            const bool bCouldBeInsideA = a.Bounds().Box().Contains(b.Bounds().Box());

            // DECISION. This one publishes an answer, so it needs lemma B4: a WITNESS on
            // the contained side. Reading the filters above as the proof is F38 - six of
            // eleven rotations answered `false` for a part sitting inside the plate
            // (1270), because the enclosure of a rotated shape is up to twice its extent.
            //
            // The filters gate the witness: `RefutesContainment` implies the filter, so
            // where the filter already says "could be nested" there is nothing to prove
            // and the O(n) witness is never built.
            if (!aCouldBeInsideB && !bCouldBeInsideA &&
                RefutesContainment(a.ExtentWitness(), b.Bounds(), context) &&
                RefutesContainment(b.ExtentWitness(), a.Bounds(), context))
            {
                return screened->Publish(false);
            }

            // An over-estimated WINDOW makes uniformity harder to establish, which is the
            // safe direction (lemma B3); that is why these keep the enclosure.
            if (aCouldBeInsideB &&
                FillIsUniformWithin(*treeB, b.ToWorld(), a.Bounds().Box()))
                return screened->Publish(RepresentativePointInside(a, *treeA, b, *treeB, context));
            if (bCouldBeInsideA &&
                FillIsUniformWithin(*treeA, a.ToWorld(), b.Bounds().Box()))
                return screened->Publish(RepresentativePointInside(b, *treeB, a, *treeA, context));
        }

        const auto shared = Boolean::Intersection(a.WorldPath(), b.WorldPath(), context);
        if (!shared.Ok() && shared.Status() != GeometryStatus::Empty)
            return GeometryResult<bool>::Failure(shared.Status());

        return screened->Publish(shared.Ok() && !shared.Value().contours.empty());
    }

    GeometryResult<bool> ContainsUnguarded(const GeometryInstance& outer,
                                           const GeometryInstance& inner,
                                           const GeometryContext& context)
    {
        std::optional<ScreenedPair> screened;
        if (const GeometryStatus status = ScreenedPair::Screen(outer, inner, context, screened);
            !IsSuccess(status))
        {
            return GeometryResult<bool>::Failure(status);
        }

        // Empty is a value, and the raw route states what it means
        // (Collision.cpp:130-133). The empty set is a subset of every set, and nothing
        // non-empty is a subset of it. V8.1's prepared route had neither guard: it fell
        // through to BindTrees, whose Topology() of an empty path fails, and published
        // Unsupported where raw publishes Success(true).
        if (InstanceIsEmpty(inner))
            return screened->Publish(true);
        if (InstanceIsEmpty(outer))
            return screened->Publish(false);

        // Lemma B2 first, because it is free and it is a proof: enclosures that do not
        // even touch prove the shapes disjoint, and `inner` is known non-empty by the
        // guard above, so disjoint settles it.
        //
        // This is not an optimisation bolted on afterwards - it was measured. Without it
        // every disjoint pair built an O(n) witness to refute what this line refutes in
        // four comparisons: `Contains disjoint bounds` went 12.0 -> 1967.4 ns/query.
        if (!outer.Bounds().CouldTouch(inner.Bounds(), context.Tolerance().intersection))
            return screened->Publish(false);

        // Then the box, which only refutes containment when the contained side
        // is a WITNESS. Lemma B4. The cheap enclosure test gates the O(n) witness: it is
        // implied by the refutation, so a pair it accepts has nothing to refute.
        if (!outer.Bounds().Box().Contains(inner.Bounds().Box()) &&
            RefutesContainment(inner.ExtentWitness(), outer.Bounds(), context))
        {
            return screened->Publish(false);
        }

        // Any boundary crossing means part of `inner` escapes.
        // Same reason as in Overlaps: already inside Contains's gateway frame.
        const auto crossing = IntersectsUnguarded(outer, inner, context);
        if (!crossing.Ok())
            return crossing;
        if (crossing.Value())
            return screened->Publish(false);

        const ContainmentTree* outerTree = nullptr;
        const ContainmentTree* innerTree = nullptr;
        if (const GeometryStatus status = BindTrees(*screened, outerTree, innerTree);
            !IsSuccess(status))
        {
            // Unchanged contract: a region query without topology reports Unsupported
            // rather than materialising WorldPath() to answer anyway.
            return GeometryResult<bool>::Failure(status);
        }

        // ONE representative point decides for `inner` - so `inner` has to BE one region.
        //
        // ContainmentTree stores a single representative point: the first root that
        // classifies Inside, then break. For a two-component `inner` that point speaks
        // only for whichever component came first, and the verdict silently depends on
        // the order the caller listed them in.
        //
        // The raw route already refused this shortcut, and said why
        // (Collision.cpp:148-157): a probe cannot see a hole of `outer` lying strictly
        // inside `inner`, and a 15x15 part over a 2x2 hole was reported contained with
        // 4 mm^2 of it over void. The prepared route reintroduced the probe; this
        // restores the precondition the raw comment was about.
        if (IsSingleConnectedRegion(*innerTree) &&
            FillIsUniformWithin(*outerTree, outer.ToWorld(), inner.Bounds().Box()))
        {
            return screened->Publish(RepresentativePointInside(inner, *innerTree, outer, *outerTree, context));
        }

        // A ring of `outer` sits inside `inner`, so ask the set question exactly.
        const auto residue = Boolean::Difference(inner.WorldPath(), outer.WorldPath(), context);
        if (!residue.Ok() && residue.Status() != GeometryStatus::Empty)
            return GeometryResult<bool>::Failure(residue.Status());

        return screened->Publish(!residue.Ok() || residue.Value().contours.empty());
    }
    }

    // The two heaviest allocators in the family. Both materialise WorldPath() for each
    // operand and hand the pair to Boolean, which is where F37's uncaught std::bad_alloc
    // came from.
    GeometryResult<bool> Overlaps(const GeometryInstance& a, const GeometryInstance& b,
                                  const GeometryContext& context)
    {
        return TranslateExhaustion<bool>([&] { return OverlapsUnguarded(a, b, context); });
    }

    GeometryResult<bool> Contains(const GeometryInstance& outer, const GeometryInstance& inner,
                                  const GeometryContext& context)
    {
        return TranslateExhaustion<bool>([&] { return ContainsUnguarded(outer, inner, context); });
    }
}
