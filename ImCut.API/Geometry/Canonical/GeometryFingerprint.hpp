#pragma once

#include "CanonicalGeometry.hpp"

#include <cstdint>
#include <functional>

namespace ImCut::Geometry
{
    // 128 bits, computed from the quantised canonical form. Raw doubles are never
    // hashed: bit-identical shapes would otherwise fingerprint differently after any
    // arithmetic that perturbs the last ulp.
    struct GeometryFingerprint
    {
        std::uint64_t low = 0;
        std::uint64_t high = 0;

        [[nodiscard]] bool operator==(const GeometryFingerprint& o) const noexcept
        {
            return low == o.low && high == o.high;
        }
        [[nodiscard]] bool operator!=(const GeometryFingerprint& o) const noexcept { return !(*this == o); }
        [[nodiscard]] bool operator<(const GeometryFingerprint& o) const noexcept
        {
            return high != o.high ? high < o.high : low < o.low;
        }

        [[nodiscard]] bool IsValid() const noexcept { return low != 0 || high != 0; }
    };

    namespace Fingerprint
    {
        [[nodiscard]] GeometryFingerprint Compute(const CanonicalContour& canonical) noexcept;
        [[nodiscard]] GeometryFingerprint Compute(const CanonicalPath& canonical) noexcept;

        [[nodiscard]] GeometryFingerprint Compute(const Contour& contour,
                                                  const CanonicalOptions& options = {});
        [[nodiscard]] GeometryFingerprint Compute(const Path& path,
                                                  const CanonicalOptions& options = {});

        // Equal fingerprints are a candidate, not a proof. Callers must confirm with
        // one of these before treating two geometries as the same.
        [[nodiscard]] bool Verify(const CanonicalContour& a, const CanonicalContour& b) noexcept;
        [[nodiscard]] bool Verify(const CanonicalPath& a, const CanonicalPath& b) noexcept;

        // Tolerance-based comparison against the real geometry, independent of the
        // lattice. Node counts must match; positions must agree within `tolerance`.
        [[nodiscard]] bool ApproximatelyEqual(const Contour& a, const Contour& b, double tolerance);

        [[nodiscard]] GeometryFingerprint Combine(GeometryFingerprint a, GeometryFingerprint b) noexcept;
    }
}

template <>
struct std::hash<ImCut::Geometry::GeometryFingerprint>
{
    [[nodiscard]] std::size_t operator()(const ImCut::Geometry::GeometryFingerprint& f) const noexcept
    {
        return static_cast<std::size_t>(f.low ^ (f.high * 0x9E3779B97F4A7C15ull));
    }
};
