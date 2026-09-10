#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

namespace ImCut::Geometry
{
    enum class PrecisionMode
    {
        // Cheaper, conservatively-approximated queries. Never means "wrong":
        // whenever a Fast decision lands in the numerically uncertain band the
        // caller is expected to fall through to the robust narrow phase.
        Fast,
        Production,
        Maximum
    };

    // The single tolerance vocabulary of the kernel for GEOMETRIC tolerances - every
    // quantity measured in millimetres against real geometry comes from here, and no
    // module invents its own.
    //
    // The qualifier is not a weasel word, it is the correction of a claim that was
    // simply false. This used to read "every epsilon used anywhere", and the kernel
    // contained at least seven that did not come from here. Most of them are not
    // geometric tolerances at all and do not belong in a millimetre vocabulary:
    //
    //   Transform2::IsIsometry(1e-9)     dimensionless, compares matrix entries whose
    //                                    magnitudes are ratios, not lengths
    //   Transform2 condition floor 1e-12 dimensionless, guards a determinant
    //   Polynomial solver epsilons       relative to coefficient magnitude, not to mm
    //   CubicBezier internal guards      parameter-space, t in [0,1], not millimetres
    //
    // Scaling any of those by a millimetre tolerance would be wrong, not more
    // principled. What the rule requires of them is that they be documented where they
    // are, and that none of them be SCALE DEPENDENT - a threshold compared against a
    // quantity whose units depend on the caller's coordinates. SolveQuadratic's was, and
    // is fixed; the rest are ratios and are not.
    //
    // All values are millimetres unless the name says otherwise. Defaults target
    // wide-format print-and-cut work, where the physical cutter is good for roughly
    // 0.1 mm, so geometric tolerances sit one to two orders of magnitude below that.
    struct GeometryTolerance
    {
        // Below this, two coordinates are the same number for comparison purposes.
        double coordinateEpsilon = 1e-9;

        // Endpoint gap under which an open contour is treated as closed.
        double closure = 1e-3;

        // Distance under which two nodes collapse into one.
        double nodeMerge = 1e-3;

        // Segment length at or below this carries no direction and is degenerate.
        double zeroLength = 1e-6;

        // Segment short enough to be numerically fragile but still directional.
        double tinySegment = 1e-3;

        // Positional agreement required to accept a computed intersection point.
        double intersection = 1e-7;

        // Normalised cross-product magnitude under which two directions are parallel.
        double collinearity = 1e-9;

        // Normalised cross-product magnitude under which a crossing is a tangency.
        double tangent = 1e-7;

        // Distance under which two points are considered the same vertex.
        double duplicate = 1e-3;

        // Maximum chord deviation allowed when flattening a curve to a polyline.
        double flatten = 1e-2;

        // Area at or below this is zero regardless of object size (mm^2).
        double absoluteArea = 1e-6;

        // Area at or below this fraction of the bounding-box area is zero.
        double relativeArea = 1e-9;

        // Clamps for scale-aware tolerances. Without an upper clamp a 3 m artwork
        // would silently grow its "same point" radius into the millimetre range and
        // start merging real nodes, so the growth is bounded on both ends.
        double scaledEpsilonMin = 1e-9;
        double scaledEpsilonMax = 1e-2;

        [[nodiscard]] static constexpr GeometryTolerance Production() noexcept { return {}; }

        [[nodiscard]] static constexpr GeometryTolerance Fast() noexcept
        {
            GeometryTolerance t;
            t.flatten = 5e-2;
            t.nodeMerge = 5e-3;
            t.duplicate = 5e-3;
            t.closure = 5e-3;
            t.tinySegment = 5e-3;
            return t;
        }

        [[nodiscard]] static constexpr GeometryTolerance Maximum() noexcept
        {
            GeometryTolerance t;
            t.flatten = 1e-3;
            t.nodeMerge = 1e-4;
            t.duplicate = 1e-4;
            t.closure = 1e-4;
            t.tinySegment = 1e-4;
            t.intersection = 1e-9;
            return t;
        }

        [[nodiscard]] static constexpr GeometryTolerance ForMode(PrecisionMode mode) noexcept
        {
            switch (mode)
            {
                case PrecisionMode::Fast:    return Fast();
                case PrecisionMode::Maximum: return Maximum();
                case PrecisionMode::Production:
                default:                     return Production();
            }
        }

        // Relative epsilon for a coordinate of the given magnitude, clamped so it can
        // neither vanish on huge coordinates nor swallow real geometry on small ones.
        [[nodiscard]] double ScaledEpsilon(double magnitude) const noexcept
        {
            const double scaled = coordinateEpsilon * std::fabs(magnitude);
            return std::clamp(scaled, scaledEpsilonMin, scaledEpsilonMax);
        }

        // Area threshold combining the absolute floor with the size-relative one.
        [[nodiscard]] double AreaThreshold(double referenceArea) const noexcept
        {
            const double relative = relativeArea * std::fabs(referenceArea);
            return relative > absoluteArea ? relative : absoluteArea;
        }

        [[nodiscard]] bool IsValid() const noexcept
        {
            // Finite as well as positive.
            //
            // The test used to be `> 0.0` alone, which infinity satisfies. A tolerance
            // of infinity is not a loose tolerance, it is a broken one - every flatness
            // test passes, every merge test passes, and the caller is told the settings
            // are valid. NaN fails `> 0.0` already; infinity did not.
            auto usable = [](double value) noexcept
            {
                return value > 0.0 && value < std::numeric_limits<double>::infinity();
            };

            return usable(coordinateEpsilon) && usable(closure) && usable(nodeMerge) &&
                   usable(zeroLength) && usable(tinySegment) && usable(intersection) &&
                   usable(collinearity) && usable(tangent) && usable(duplicate) &&
                   usable(flatten) && usable(absoluteArea) &&
                   relativeArea >= 0.0 && relativeArea < std::numeric_limits<double>::infinity() &&
                   usable(scaledEpsilonMin) && usable(scaledEpsilonMax) &&
                   scaledEpsilonMax >= scaledEpsilonMin;
        }
    };
}
