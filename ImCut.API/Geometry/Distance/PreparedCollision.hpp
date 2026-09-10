#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../Intersection/Intersection.hpp"
#include "../PreparedGeometry.hpp"

namespace ImCut::Geometry::Collision
{
    // Collision queries between two prepared instances.
    //
    // This is the hot path the kernel exists for: prepare once, query millions of times.
    // Everything here runs against the shared definitions' existing hierarchies and a
    // pair of transforms. Nothing builds a segment list, nothing rebuilds a hierarchy,
    // and the any-hit forms allocate nothing at all.
    //
    // The two definitions live in their own local frames, so instead of transforming
    // geometry into world space the traversal maps B into A's frame once and works
    // there. Node bounds are transformed on the fly, which is conservative - a
    // transformed AABB can over-report but never under-report - so the broad phase
    // stays free of false negatives.
    //
    // PRECONDITION - both instances must be placed by an isometry.
    //
    // Every query here is metric: distances come back in millimetres and the narrow
    // phase compares against millimetre tolerances. Working in A's local frame only
    // preserves those millimetres when the placement preserves distance, so
    // translation, rotation and reflection are supported and scale, non-uniform scale
    // and skew return GeometryStatus::Unsupported. They are refused rather than
    // measured in the wrong units: under a 2x placement a 10 mm gap reads as 5, which
    // no caller could distinguish from a correct answer.
    //
    // This is the transform set nesting places pieces with, so the hot path is
    // unaffected. Callers needing a scaled or skewed instance should bake the transform
    // into the definition and place the result rigidly.

    // Any contact between the two OUTLINES. Stops at the first one.
    //
    // Boundary-only, deliberately and by name. A part sitting entirely inside another
    // has no boundary contact, so this returns false for it - which is the right answer
    // to the question it asks and the wrong answer to the question a nesting engine
    // asks. Use Overlaps for regions.
    [[nodiscard]] GeometryResult<bool> Intersects(const GeometryInstance& a,
                                                  const GeometryInstance& b,
                                                  const GeometryContext& context);

    // Do the two REGIONS share any area?
    //
    // True when the outlines cross, and also when one part lies wholly inside the other
    // with no boundary contact at all. That second case is the one a no-fit polygon
    // produces constantly and the one Intersects cannot see: the audit measured a
    // 40x40 part sitting inside a 100x100 one reported as not intersecting, with
    // MinimumDistance reporting 40 mm of clearance.
    //
    // Costs the same as Intersects in the disjoint case - the world-bounds reject
    // decides first, and the containment test only runs when one part's bounds enclose
    // the other's, which cannot happen for disjoint parts that merely sit close.
    //
    // Both definitions must be prepared to PreparationLevel::BooleanReady: the
    // containment test reads their ContainmentTrees. Anything less is Unsupported.
    // Do the two filled regions share interior area?
    //
    // Exact. Bounds rejection and boundary crossing decide almost every call; when the
    // two are nested with no crossing, one interior point decides it only where a bounds
    // proof says the other region's fill is constant across the window, and otherwise
    // the answer comes from Boolean::Intersection. Measured cost by regime is in
    // audit-v6/evidence-v6/23_overlap_cost.txt.
    [[nodiscard]] GeometryResult<bool> Overlaps(const GeometryInstance& a,
                                                const GeometryInstance& b,
                                                const GeometryContext& context);

    // Is `inner` wholly inside `outer`, as a set?
    //
    // True if and only if no part of `inner` falls outside `outer`, holes included. A
    // part laid over a cut-out in the container is NOT contained, however far from the
    // container's outer boundary it sits - that is the IFP predicate, and getting it
    // wrong means placing a part over void.
    //
    // Both definitions must be prepared to BooleanReady, for the same reason as above.
    [[nodiscard]] GeometryResult<bool> Contains(const GeometryInstance& outer,
                                                const GeometryInstance& inner,
                                                const GeometryContext& context);

    // First contact, reported in A's world coordinates.
    [[nodiscard]] GeometryResult<bool> FindFirstIntersection(const GeometryInstance& a,
                                                             const GeometryInstance& b,
                                                             const GeometryContext& context,
                                                             IntersectionPoint& out);

    // WHAT A THRESHOLD QUERY ACTUALLY PROVED.
    //
    // A bool has two values and the question has three answers. When refinement cannot
    // separate the interval from the threshold, the bool below answers `true` because for
    // a proximity question "might be near" is the safe side - and that conservative `true`
    // was indistinguishable from a proved one. `ErrorBudget.exact` does not help: it
    // describes PROVENANCE, which is what the operands declared about their own accuracy,
    // and with exact operands it stays true whether or not the decision was proved. Two
    // different quantities in one field is the same mistake ComplexityLimits makes with
    // hard and soft ceilings.
    //
    // So the certification of the DECISION gets its own channel. That is F29.
    enum class DistanceDecisionState : std::uint8_t
    {
        // PROVED nearer than the threshold, witnessed by two points that lie on the two
        // outlines, with the operands' declared provenance already subtracted.
        True,

        // PROVED at or beyond the threshold: no point of either outline can be nearer,
        // again with provenance already accounted for.
        False,

        // Neither was proved. NOT "unprovable" - this query stopped at the first pair it
        // could not separate, and a different pair might have decided it. The bool form
        // answers `true` here, and that is a POLICY, not a proof.
        Ambiguous
    };

    struct DistanceDecision
    {
        DistanceDecisionState state = DistanceDecisionState::Ambiguous;

        // The certified interval the query established around the threshold, in
        // millimetres, for the geometry as REPRESENTED. `lower` is proved by boxes that
        // contain the curves, `upper` by points that lie on them.
        double lower = 0.0;
        double upper = 0.0;

        // The provenance actually SUBTRACTED from the decision - the sum of what the two
        // operands declared about their own sources.
        //
        // This is not a second provenance channel. GeometryResult::Budget() carries the
        // operands' provenance exactly as it does for the five sibling bool queries, and
        // this field is the audit trail of what the comparison consumed, so a caller can
        // reconstruct the test that was performed rather than infer it.
        double appliedProvenance = 0.0;
    };

    // "Are these closer than d?" - with the third answer visible.
    //
    //     upper + provenance < threshold   ->  True
    //     lower - provenance > threshold   ->  False
    //     otherwise                        ->  Ambiguous
    //
    // Use this when Ambiguous is worth handling: a caller that can afford an exact route,
    // or one that must not treat a conservative answer as a measurement. Callers that
    // simply want the safe side want DistanceLessThan below, which delegates to this and
    // maps Ambiguous onto `true`.
    [[nodiscard]] GeometryResult<DistanceDecision> DistanceLessThanCertified(
        const GeometryInstance& a, const GeometryInstance& b, double threshold,
        const GeometryContext& context);

    // "Are these closer than d?" answered without measuring how close.
    //
    // A different question from MinimumDistance, and answering it by computing the exact
    // distance throws away most of the pruning the threshold makes possible.
    //
    // DELEGATES to DistanceLessThanCertified and maps `state != False` onto true. There is
    // no second threshold comparison anywhere in the kernel - that was the point of
    // introducing the certified form, and re-deriving the answer here would have recreated
    // the class it closed.
    [[nodiscard]] GeometryResult<bool> DistanceLessThan(const GeometryInstance& a,
                                                        const GeometryInstance& b,
                                                        double threshold,
                                                        const GeometryContext& context);

    // Boundary-to-boundary distance. Zero when the outlines touch or cross.
    [[nodiscard]] GeometryResult<double> MinimumDistance(const GeometryInstance& a,
                                                         const GeometryInstance& b,
                                                         const GeometryContext& context);

    // Cheap world-AABB reject, the first thing every query above does.
    [[nodiscard]] bool BoundsCouldTouch(const GeometryInstance& a, const GeometryInstance& b,
                                        double tolerance) noexcept;
}
