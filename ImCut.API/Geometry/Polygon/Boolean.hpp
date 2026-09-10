#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"

namespace ImCut::Geometry
{
    enum class BooleanOperation : std::uint8_t { Union, Intersection, Difference, Xor };

    // How the fill rule for an operation is chosen.
    enum class BooleanFillRule : std::uint8_t
    {
        // Take it from the operands. This is the normal behaviour: a path already
        // records how its own rings are meant to be filled, and overriding that silently
        // discards holes on any artwork authored with even-odd and consistent winding.
        UseSource,

        EvenOdd,
        NonZero
    };

    struct BooleanOptions
    {
        BooleanFillRule fillRule = BooleanFillRule::UseSource;

        // Drop zero-area rings from the result and report them.
        bool pruneDegenerate = true;
    };

    struct BooleanReport
    {
        std::size_t inputRings = 0;
        std::size_t outputRings = 0;
        std::size_t prunedDegenerate = 0;
        std::size_t prunedEmpty = 0;
        bool nonFiniteRemoved = false;

        // Set when the lattice changed the TOPOLOGY rather than merely the coordinates.
        //
        // Quantisation can merge two components that were separate, or annihilate a
        // region entirely. Both come back as a perfectly ordinary status - Empty is a
        // success by design, and correctly so for Difference(A, A) - which left a caller
        // no way to tell "this region is genuinely empty" from "the lattice destroyed
        // it". The audit measured a 1e6 mm^2 region reduced to zero rings with
        // prunedDegenerate = 0 and nothing else to look at.
        //
        // V5 also moved quantisation into the operands' own frame, so the step no longer
        // depends on where the artwork sits and this fires far less often. It can still
        // fire for geometry whose features are genuinely below the lattice, which is a
        // fact about the input and has to be reportable.
        bool topologyChanged = false;

        // Lattice step actually used, in millimetres.
        double quantizationStep = 0.0;
    };

    // Boolean operations on filled REGIONS.
    //
    // Consumers - Nesting, GeometryDoctor, Cut, AutoBleeding - call only these. The
    // polygon engine sits behind an internal backend boundary and no backend type is
    // reachable from here, so it can be replaced without touching any caller.
    //
    // Contract:
    //   - Every contour must be closed. An open contour is not a region and is rejected
    //     with InvalidTopology. Treating one as a ring invents filled area that was
    //     never in the input; use Offset::StrokeOpenPath to turn a polyline into a
    //     region deliberately.
    //   - When the two operands disagree on fill rule under UseSource, the subject's
    //     rule wins, because the operation is defined as "modify the subject".
    //   - Results are polygonal: curved input is flattened at the context tolerance
    //     first, and the returned ErrorBudget reports the accumulated flattening and
    //     quantisation error so nobody can overstate the precision of the output.
    namespace Boolean
    {
        [[nodiscard]] GeometryResult<Path> Execute(BooleanOperation operation,
                                                   const Path& subject, const Path& clip,
                                                   const GeometryContext& context,
                                                   const BooleanOptions& options = {},
                                                   BooleanReport* report = nullptr);

        [[nodiscard]] GeometryResult<Path> Union(const Path& a, const Path& b,
                                                 const GeometryContext& context,
                                                 const BooleanOptions& options = {});

        [[nodiscard]] GeometryResult<Path> Intersection(const Path& a, const Path& b,
                                                        const GeometryContext& context,
                                                        const BooleanOptions& options = {});

        // Difference(A, A) is Empty - a correct answer, not a failure.
        [[nodiscard]] GeometryResult<Path> Difference(const Path& a, const Path& b,
                                                      const GeometryContext& context,
                                                      const BooleanOptions& options = {});

        [[nodiscard]] GeometryResult<Path> Xor(const Path& a, const Path& b,
                                               const GeometryContext& context,
                                               const BooleanOptions& options = {});

        // Self-union: resolves a path's own overlaps into clean, correctly nested rings.
        [[nodiscard]] GeometryResult<Path> Simplify(const Path& path,
                                                    const GeometryContext& context,
                                                    const BooleanOptions& options = {});

        // Name of the active backend, for reporting and diagnostics.
        [[nodiscard]] const char* BackendName() noexcept;
    }
}
