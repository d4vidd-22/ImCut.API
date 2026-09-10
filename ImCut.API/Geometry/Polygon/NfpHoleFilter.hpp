#pragma once

#include "../GeometryCertification.hpp"
#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Math/Bounds2.hpp"
#include "../Math/Transform2.hpp"
#include "../Topology/ContainmentTree.hpp"

#include <cstddef>
#include <cstdint>
#include <vector>

namespace ImCut::Geometry::Nfp
{
    enum class HoleFilterProof : std::uint8_t
    {
        None,
        BoundingBoxNonFit,
        RemovedWithProvenAncestor
    };

    struct HoleFilterDecision
    {
        std::uint32_t contourIndex = 0;
        HoleFilterProof proof = HoleFilterProof::None;
        // The hole is the CONTAINER of the question, so an enclosure is the sound
        // evidence for it: over-estimating a container only makes the filter fire less.
        // BOUNDS_EVIDENCE_PROOF.md lemma B4.2.
        BoundsEnclosure holeBounds{};
        bool widthProvesNonFit = false;
        bool heightProvesNonFit = false;
        bool removed = false;
    };

    struct OperandHoleFilterAudit
    {
        Path filtered{};
        std::vector<HoleFilterDecision> holes{};
        // The other operand is the CONTAINED side, so this is a WITNESS and not an
        // enclosure. It was an enclosure until V8.1.2 and that was F26: a hole the part
        // fits into was filled at every rotation where the enclosure inflated past it.
        BoundsWitness otherOperandBounds{};
        bool sourceTopologyAccepted = false;
        bool otherOperandTopologyAccepted = false;
        bool otherOperandConnected = false;
        std::size_t removedContours = 0;
        std::size_t removedHoles = 0;
        std::size_t preservedHoles = 0;
    };

    // Pair-specific derivative. It does not claim that a hole is absent from the
    // operand; it proves that filling it leaves this pair's Minkowski sum unchanged.
    struct ProvenHoleFilterAudit
    {
        OperandHoleFilterAudit stationary{};
        OperandHoleFilterAudit moving{};

        [[nodiscard]] std::size_t RemovedHoles() const noexcept
        {
            return stationary.removedHoles + moving.removedHoles;
        }
        [[nodiscard]] bool ProductionEligible() const noexcept { return true; }
        [[nodiscard]] GeometryCertification Certification() const noexcept
        {
            return GeometryCertification::Certified;
        }
    };

    // Shape-intrinsic proof derivative. Validation, containment and connectedness are
    // paid once; pair filtering only combines this metadata with the other bounds.
    struct PreparedHoleFilterAnalysis
    {
        ContainmentTree tree{};
        // Exact polygonal predicate per original contour. The pair plan may use it
        // only after proving that every other contour is removed; flattened curve
        // rings never become an exact convex-edge-merge certificate.
        std::vector<std::uint8_t> exactStrictlyConvexContours{};
        Bounds2 bounds{};
        // Points that lie ON this shape, in its own frame - the convex hull of the
        // contour nodes and of one evaluated point per cubic extremum. A hull has the
        // same support function as the set it spans, so the witness it yields under any
        // transform is the same one the full point set would yield, at hull cost.
        //
        // This is shape-intrinsic, which is why it lives in the prepared analysis; the
        // TRANSFORM is what varies per pair, and `Metrics::ComputeWitness` takes it.
        std::vector<Vec2> witnessPoints{};
        bool topologyAccepted = false;
        bool connected = false;
    };

    struct OperandHoleFilterPlan
    {
        std::size_t removedContours = 0;
        std::size_t removedHoles = 0;
        bool strictlyConvexAfterFilter = false;
    };

    struct ProvenHoleFilterPlan
    {
        OperandHoleFilterPlan stationary{};
        OperandHoleFilterPlan moving{};

        [[nodiscard]] bool Applied() const noexcept
        {
            return stationary.removedContours != 0 ||
                   moving.removedContours != 0;
        }
        [[nodiscard]] std::size_t RemovedHoles() const noexcept
        {
            return stationary.removedHoles + moving.removedHoles;
        }
    };

    [[nodiscard]] GeometryResult<PreparedHoleFilterAnalysis>
    PrepareHoleFilterAnalysis(
        const Path& path,
        const GeometryContext& context);

    [[nodiscard]] ProvenHoleFilterPlan PlanProvenIrrelevantHoles(
        const PreparedHoleFilterAnalysis& stationary,
        const Transform2& stationaryTransform,
        const PreparedHoleFilterAnalysis& moving,
        const Transform2& movingTransform,
        const GeometryContext& context) noexcept;

    // Conservative proof used by WP8B.
    //
    // For closed A and connected B:
    //   x is outside A (+) B  iff  x - B lies in one connected component of A's
    //   complement. A bounded hole can therefore affect the dilation only if some
    //   translation of B fits wholly inside that hole. If B's axis-aligned bounds are
    //   wider OR taller than the hole's bounds (with a numerical safety margin), such
    //   a translation is impossible. Filling that hole is then exact for this pair.
    //
    // The test is sufficient, not necessary. Ambiguous topology, disconnected B,
    // equal/near-equal bounds, and every case without proof preserve the hole.
    [[nodiscard]] GeometryResult<ProvenHoleFilterAudit> FilterProvenIrrelevantHoles(
        const Path& stationary,
        const Path& moving,
        const GeometryContext& context);

    [[nodiscard]] GeometryResult<ProvenHoleFilterAudit> FilterProvenIrrelevantHoles(
        const Path& stationary,
        const PreparedHoleFilterAnalysis& stationaryAnalysis,
        const Path& moving,
        const PreparedHoleFilterAnalysis& movingAnalysis,
        const GeometryContext& context);

    [[nodiscard]] GeometryResult<ProvenHoleFilterAudit> FilterProvenIrrelevantHoles(
        const Path& stationary,
        const PreparedHoleFilterAnalysis& stationaryAnalysis,
        const Transform2& stationaryTransform,
        const Path& moving,
        const PreparedHoleFilterAnalysis& movingAnalysis,
        const Transform2& movingTransform,
        const GeometryContext& context);
}
