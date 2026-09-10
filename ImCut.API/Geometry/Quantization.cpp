#include "Quantization.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace ImCut::Geometry
{
    std::int64_t QuantizationScale::ToInteger(double millimetres) const noexcept
    {
        return Quantization::Snap(millimetres, scale);
    }

    double QuantizationScale::ToMillimetres(std::int64_t units) const noexcept
    {
        return scale != 0.0 ? static_cast<double>(units) / scale : 0.0;
    }

    namespace Quantization
    {
        std::int64_t Snap(double millimetres, double scale) noexcept
        {
            // Infinity saturates, like any other out-of-range magnitude. Mapping it to
            // the ORIGIN - which is what `!isfinite -> 0` did - put a point that is
            // infinitely far away at the exact centre of the lattice, which is the one
            // answer guaranteed to be wrong in a way nothing downstream can notice.
            if (millimetres == std::numeric_limits<double>::infinity()) return INT64_MAX;
            if (millimetres == -std::numeric_limits<double>::infinity()) return INT64_MIN;

            // NaN has no lattice point, and no saturation direction either. Zero is
            // returned as a defined, documented answer rather than as undefined
            // conversion behaviour - but it IS indistinguishable from a legitimate
            // origin, so callers must reject non-finite input first. Every kernel entry
            // point does: Contour::IsStructurallyValid checks HasFiniteCoordinates, and
            // Canonical::Canonicalize refuses a non-finite contour outright, so this
            // branch is unreachable from validated geometry.
            if (std::isnan(millimetres) || scale == 0.0)
                return 0;

            const double scaled = millimetres * scale;

            // Saturate rather than wrap. A value this far out is already invalid input,
            // and undefined conversion behaviour would turn it into silent corruption.
            if (scaled >= 9.2e18) return INT64_MAX;
            if (scaled <= -9.2e18) return INT64_MIN;

            // Round half away from zero, so the lattice is symmetric about the origin
            // and mirrored geometry quantises to mirrored integers.
            //
            // This matches llround below 2^52 and diverges above it, where the spacing
            // between doubles is already at least 1: adding 0.5 to an integral value
            // there rounds to even instead of being exact, so an odd `scaled` can come
            // back one larger. llround would return it unchanged. The comment here used
            // to claim equivalence without qualification. There is no practical
            // consequence - the canonical ceiling is 1e15 and kMaxSafeCoordinate is 1e9,
            // both far below 2^52 - but the claim was still false.
            return static_cast<std::int64_t>(scaled >= 0.0 ? scaled + 0.5 : scaled - 0.5);
        }

        QuantizationScale ChooseScale(const Bounds2& bounds, double desiredResolution) noexcept
        {
            QuantizationScale result;

            if (!(desiredResolution > 0.0) || !std::isfinite(desiredResolution))
                return result;

            if (bounds.IsEmpty())
            {
                // No geometry to bound the range, so the requested resolution is
                // unconditionally safe.
                result.scale = 1.0 / desiredResolution;
                result.resolution = desiredResolution;
                result.valid = true;
                return result;
            }

            if (!IsFinite(bounds))
                return result;

            const double extreme = std::max({ std::fabs(bounds.min.x), std::fabs(bounds.max.x),
                                              std::fabs(bounds.min.y), std::fabs(bounds.max.y) });

            double scale = 1.0 / desiredResolution;

            if (extreme > 0.0)
            {
                // Coarsen until the largest coordinate fits the safe range. This is the
                // step that keeps downstream integer cross products from overflowing.
                const double maximumScale = kMaxSafeCoordinate / extreme;
                scale = std::min(scale, maximumScale);
            }

            if (!(scale > 0.0) || !std::isfinite(scale))
                return result;

            result.scale = scale;
            result.resolution = 1.0 / scale;
            result.valid = true;
            return result;
        }

        QuantizationScale CanonicalScale(const Bounds2& extent, double resolution) noexcept
        {
            QuantizationScale result;

            if (!(resolution > 0.0) || !std::isfinite(resolution))
                return result;

            double scale = 1.0 / resolution;

            if (!extent.IsEmpty())
            {
                if (!IsFinite(extent))
                    return result;

                // Only the span matters: canonical coordinates are always relative to
                // the shape's own origin, so absolute position never enters here.
                const double span = std::max(extent.Width(), extent.Height());
                if (span > 0.0)
                {
                    const double maximumScale = kMaxCanonicalCoordinate / span;
                    if (maximumScale < scale)
                        scale = maximumScale;
                }
            }

            if (!(scale > 0.0) || !std::isfinite(scale))
                return result;

            result.scale = scale;
            result.resolution = 1.0 / scale;
            result.valid = true;
            return result;
        }

        QuantizationScale ChooseScale(const Bounds2& bounds, const GeometryTolerance& tolerance) noexcept
        {
            // Aim an order of magnitude finer than the coordinate epsilon, but never
            // finer than the default lattice, which is already far below any tolerance
            // the kernel works with.
            const double desired = std::max(kDefaultResolution, tolerance.coordinateEpsilon * 0.1);
            return ChooseScale(bounds, desired);
        }

        bool FitsSafely(const Bounds2& bounds, const QuantizationScale& scale) noexcept
        {
            if (!scale.valid || bounds.IsEmpty())
                return scale.valid;

            const double extreme = std::max({ std::fabs(bounds.min.x), std::fabs(bounds.max.x),
                                              std::fabs(bounds.min.y), std::fabs(bounds.max.y) });

            return extreme * scale.scale <= kMaxSafeCoordinate;
        }
    }
}
