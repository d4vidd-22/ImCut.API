#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Quantization.hpp"
#include "PolygonBackend.hpp"

// INTERNAL. Shared mm-to-integer plumbing for Boolean, Offset and Minkowski.
//
// This is where the kernel keeps ownership of everything the backend must not decide:
// which lattice to use, whether the input fits it, how much error flattening and
// quantisation contributed, and how to turn the backend's rings back into kernel
// geometry with correct outer/hole winding.

namespace ImCut::Geometry::Convert
{
    // The lattice a set of integer rings lives on: a step, and the point that step is
    // measured from.
    //
    // The origin is the part that was missing. ChooseScale coarsens the step until the
    // largest ABSOLUTE coordinate fits the safe integer range, so the same 100 mm part
    // quantised at the origin and at 1e8 mm got two different steps - 1e-6 mm and
    // 1 mm - and the Boolean result depended on where the artwork happened to sit.
    // Measured: two components 0.05 mm apart merged into one ring at 1e8, and a 1e6 mm^2
    // region was annihilated to Empty at 1e15.
    //
    // Quantising relative to the operands' own centre makes the step depend on their
    // SIZE instead of their position, which is what the caller actually controls.
    struct LatticeFrame
    {
        QuantizationScale scale;
        Vec2 origin{};

        [[nodiscard]] Backend::IntPoint ToInteger(Vec2 point) const noexcept
        {
            return { scale.ToInteger(point.x - origin.x), scale.ToInteger(point.y - origin.y) };
        }

        [[nodiscard]] Vec2 ToMillimetres(std::int64_t x, std::int64_t y) const noexcept
        {
            return { scale.ToMillimetres(x) + origin.x, scale.ToMillimetres(y) + origin.y };
        }
    };

    struct ConversionResult
    {
        Backend::IntRings rings;

        // Ring range per input path: path i owns [offsets[i], offsets[i+1]).
        // Contours that flatten to fewer than three points are skipped, so counting
        // contours would not line up - the offsets are recorded during the same pass
        // that produced the rings.
        std::vector<std::size_t> offsets;

        // Kept for callers that only need the step - the offset request scales a
        // distance, which is translation invariant.
        QuantizationScale scale;

        // Step plus origin. Anything converting integers back to millimetres must use
        // this, not `scale` alone, or the result comes back at the wrong place.
        LatticeFrame frame;

        ErrorBudget budget;
        Bounds2 bounds;

        // Non-owning slice into `rings`. Callers that only submit an operand to the
        // backend must not deep-copy every IntRing merely to split the shared lattice.
        // The view remains valid while this ConversionResult and its rings are alive.
        [[nodiscard]] Backend::IntRingsView RingsOf(std::size_t pathIndex) const noexcept;
    };

    // Picks a lattice covering both operands so their integer coordinates are directly
    // comparable, flattens every contour at the context tolerance, and quantises.
    // Fails with Degenerate when the combined bounds cannot be represented safely.
    //
    // `extentHint`, when given, is unioned into the extent the lattice is chosen for.
    // Offset needs it: the result of inflating by d spans the input bounds grown by d,
    // and sizing the lattice for the input alone let the OUTPUT run past the documented
    // 1e9 coordinate ceiling while still reporting Success.
    [[nodiscard]] GeometryResult<ConversionResult> PathsToIntegers(
        const Path* const* paths, std::size_t pathCount, const GeometryContext& context,
        const Bounds2* extentHint = nullptr);

    [[nodiscard]] GeometryResult<ConversionResult> PathToIntegers(
        const Path& path, const GeometryContext& context);

    // Rebuilds kernel geometry from the backend's nesting hierarchy, applying the
    // outer-counter-clockwise / hole-clockwise convention from the tree depth rather
    // than trusting whatever direction the backend emitted.
    [[nodiscard]] Path PolyTreeToPath(const Backend::PolyTree& tree, const LatticeFrame& frame,
                                      FillRule fillRule, const GeometryContext& context);

    [[nodiscard]] Path RingsToPath(const Backend::IntRings& rings, const LatticeFrame& frame,
                                   FillRule fillRule);

    // Post-operation findings. A backend returning vertices is not proof the result is
    // usable: booleans and offsets routinely emit slivers and collapsed rings.
    struct ResultValidation
    {
        std::size_t emptyRings = 0;
        std::size_t degenerateRings = 0;
        bool nonFinite = false;

        [[nodiscard]] bool Clean() const noexcept
        {
            return emptyRings == 0 && degenerateRings == 0 && !nonFinite;
        }
    };

    // Drops rings that carry no area and reports what was removed. Repair policy is
    // deliberately not applied here - that belongs to the future GeometryDoctor.
    [[nodiscard]] ResultValidation ValidateAndPrune(Path& path, const GeometryContext& context);
}
