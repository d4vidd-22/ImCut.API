#pragma once

#include "GeometryResult.hpp"
#include "GeometryTolerance.hpp"
#include "Math/Bounds2.hpp"
#include "Math/Vec2.hpp"

#include <cstdint>

namespace ImCut::Geometry
{
    // The kernel's single policy for turning millimetres into integers and back.
    //
    // Everything that needs an integer lattice - canonicalisation, fingerprinting and
    // the polygon backend - goes through here, so there is exactly one place where
    // scale selection, range validation and overflow safety are decided, and exactly
    // one place that knows what resolution was actually achieved.
    struct QuantizationScale
    {
        // Integer units per millimetre.
        double scale = 0.0;

        // Millimetres per integer unit. This is the real resolution of the lattice and
        // the quantisation term of the error budget.
        double resolution = 0.0;

        bool valid = false;

        [[nodiscard]] std::int64_t ToInteger(double millimetres) const noexcept;
        [[nodiscard]] double ToMillimetres(std::int64_t units) const noexcept;

        // Euclidean displacement of a 2D point when both coordinates round by half a
        // lattice step. ErrorBudget is a spatial-distance contract, so reporting only
        // one coordinate's error understated the diagonal case by sqrt(2).
        [[nodiscard]] double MaxDisplacement() const noexcept
        {
            return resolution * 0.70710678118654752440;
        }
    };

    namespace Quantization
    {
        // Largest coordinate magnitude the lattice is allowed to produce.
        //
        // Robust polygon clipping evaluates cross products of coordinate differences.
        // With coordinates bounded by C those differences are bounded by 2C and their
        // products by 4C^2, so keeping C at 1e9 caps intermediate products at 4e18,
        // comfortably inside the 9.22e18 that a signed 64-bit integer holds. Choosing
        // a scale from the desired resolution alone, without this cap, is how integer
        // clipping libraries silently overflow on large artwork.
        inline constexpr double kMaxSafeCoordinate = 1.0e9;

        // Default lattice step in millimetres. One nanometre is four orders of
        // magnitude finer than the 0.01 mm flattening tolerance, so quantisation never
        // becomes the dominant term of the error budget.
        inline constexpr double kDefaultResolution = 1.0e-6;

        // Largest coordinate the canonical lattice may produce.
        //
        // Canonicalisation only compares and hashes integers; it never multiplies them,
        // so it is not bound by the cross-product limit that constrains clipping. Its
        // ceiling is simply "still exactly representable", which int64 gives with a very
        // wide margin here.
        inline constexpr double kMaxCanonicalCoordinate = 1.0e15;

        // Lattice step for canonical forms and fingerprints.
        //
        // Deliberately much coarser than kDefaultResolution. Translation invariance is
        // the whole point of the canonical form, and a translated coordinate cannot be
        // recovered to better than the double spacing at its own magnitude - roughly
        // 1e-11 mm for a coordinate of 1e5 mm. A 1e-4 mm lattice sits seven orders above
        // that noise floor, so relative coordinates land on the same integer no matter
        // where the shape was placed. A nanometre lattice would not, and that is exactly
        // how position leaked into fingerprints.
        //
        // It is also well below the 1e-3 mm duplicate tolerance, so nothing the kernel
        // considers distinct can collapse here.
        inline constexpr double kCanonicalResolution = 1.0e-4;

        // Lattice for canonical forms.
        //
        // Unlike ChooseScale this never coarsens with coordinate magnitude: doing so
        // made a 1000 mm part and a 3000 mm part quantise to identical integers and
        // share a fingerprint. The requested resolution is honoured outright, and only
        // an extent that would leave the exactly-representable range is coarsened - with
        // the achieved resolution reported as always.
        [[nodiscard]] QuantizationScale CanonicalScale(const Bounds2& extent,
                                                       double resolution = kCanonicalResolution) noexcept;

        // Chooses the finest lattice that keeps every coordinate inside `bounds`
        // within the safe range.
        //
        // The requested resolution is honoured when it fits and coarsened when it does
        // not; `QuantizationScale::resolution` always reports what was actually used,
        // so a caller can never claim precision the lattice does not provide.
        [[nodiscard]] QuantizationScale ChooseScale(const Bounds2& bounds,
                                                    double desiredResolution = kDefaultResolution) noexcept;

        // Scale derived from the tolerance in use, floored at the safe range.
        [[nodiscard]] QuantizationScale ChooseScale(const Bounds2& bounds,
                                                    const GeometryTolerance& tolerance) noexcept;

        // True when every corner of `bounds` maps inside the safe integer range.
        [[nodiscard]] bool FitsSafely(const Bounds2& bounds, const QuantizationScale& scale) noexcept;

        [[nodiscard]] std::int64_t Snap(double millimetres, double scale) noexcept;
    }
}
