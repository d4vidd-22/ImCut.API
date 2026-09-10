#include "GeometryFingerprint.hpp"

#include <cmath>
#include <cstring>
#include <type_traits>

namespace ImCut::Geometry::Fingerprint
{
    namespace
    {
        // Two independent SplitMix64 lanes with different constants. Running two lanes
        // over the same stream gives 128 bits without needing a wide multiply.
        constexpr std::uint64_t kSeedLow = 0x243F6A8885A308D3ull;
        constexpr std::uint64_t kSeedHigh = 0x13198A2E03707344ull;

        [[nodiscard]] inline std::uint64_t Mix(std::uint64_t value) noexcept
        {
            value ^= value >> 30;
            value *= 0xBF58476D1CE4E5B9ull;
            value ^= value >> 27;
            value *= 0x94D049BB133111EBull;
            value ^= value >> 31;
            return value;
        }

        struct Accumulator
        {
            std::uint64_t low = kSeedLow;
            std::uint64_t high = kSeedHigh;

            void FeedBits(std::uint64_t value) noexcept
            {
                low = Mix(low ^ value) + 0x9E3779B97F4A7C15ull;
                high = Mix(high + value * 0xC2B2AE3D27D4EB4Full) ^ (low >> 17);
            }

            // One template rather than uint64_t/int64_t overloads.
            //
            // On LP64, uint64_t is `unsigned long` and `unsigned long long` is a third,
            // distinct type, so a `1ull` literal matched neither overload exactly and the
            // call was ambiguous - the kernel only built on MSVC because LLP64 happens to
            // make those two types the same. A template accepts every integral type on
            // every data model and converts once, explicitly.
            template <typename T>
            void Feed(T value) noexcept
            {
                static_assert(std::is_integral_v<T>, "Feed takes integral values only");
                FeedBits(static_cast<std::uint64_t>(value));
            }

            [[nodiscard]] GeometryFingerprint Finish() const noexcept
            {
                GeometryFingerprint result;
                result.low = Mix(low ^ high);
                result.high = Mix(high + 0x9E3779B97F4A7C15ull) ^ low;
                if (!result.IsValid())
                    result.low = 1;
                return result;
            }
        };

        // The lattice divides the integers back into millimetres, so it is part of what
        // the fingerprint identifies. Hashing the bit pattern is exact and stable:
        // CanonicalScale produces the value deterministically, and the fingerprint must
        // separate anything operator== separates or a cache hit returns another shape.
        [[nodiscard]] std::uint64_t ResolutionBits(double resolution) noexcept
        {
            std::uint64_t bits = 0;
            std::memcpy(&bits, &resolution, sizeof(bits));
            return bits;
        }

        void FeedContour(Accumulator& accumulator, const CanonicalContour& canonical) noexcept
        {
            accumulator.Feed(ResolutionBits(canonical.resolution));
            accumulator.Feed(canonical.closed ? 0x1ull : 0x2ull);
            accumulator.Feed(static_cast<std::uint64_t>(canonical.coordinates.size()));
            accumulator.Feed(static_cast<std::uint64_t>(canonical.kinds.size()));

            for (const std::int64_t value : canonical.coordinates)
                accumulator.Feed(value);
            for (const std::int64_t value : canonical.handles)
                accumulator.Feed(value);
            for (const SegmentKind kind : canonical.kinds)
                accumulator.Feed(static_cast<std::uint64_t>(kind));
        }
    }

    GeometryFingerprint Compute(const CanonicalContour& canonical) noexcept
    {
        Accumulator accumulator;
        FeedContour(accumulator, canonical);
        return accumulator.Finish();
    }

    GeometryFingerprint Compute(const CanonicalPath& canonical) noexcept
    {
        Accumulator accumulator;
        accumulator.Feed(ResolutionBits(canonical.resolution));
        accumulator.Feed(static_cast<std::uint64_t>(canonical.fillRule));
        accumulator.Feed(static_cast<std::uint64_t>(canonical.contours.size()));

        // Canonicalize() already ordered the contours by content, so this is stable.
        for (const CanonicalContour& contour : canonical.contours)
            FeedContour(accumulator, contour);

        return accumulator.Finish();
    }

    GeometryFingerprint Compute(const Contour& contour, const CanonicalOptions& options)
    {
        return Compute(Canonical::Canonicalize(contour, options));
    }

    GeometryFingerprint Compute(const Path& path, const CanonicalOptions& options)
    {
        return Compute(Canonical::Canonicalize(path, options));
    }

    bool Verify(const CanonicalContour& a, const CanonicalContour& b) noexcept { return a == b; }
    bool Verify(const CanonicalPath& a, const CanonicalPath& b) noexcept { return a == b; }

    bool ApproximatelyEqual(const Contour& a, const Contour& b, double tolerance)
    {
        if (a.closed != b.closed) return false;
        if (a.nodes.size() != b.nodes.size()) return false;
        if (a.kinds != b.kinds) return false;
        if (a.handles.size() != b.handles.size()) return false;

        const double toleranceSquared = tolerance * tolerance;

        for (std::size_t i = 0; i < a.nodes.size(); ++i)
        {
            if (DistanceSquared(a.nodes[i], b.nodes[i]) > toleranceSquared)
                return false;
        }
        for (std::size_t i = 0; i < a.handles.size(); ++i)
        {
            if (DistanceSquared(a.handles[i], b.handles[i]) > toleranceSquared)
                return false;
        }

        return true;
    }

    GeometryFingerprint Combine(GeometryFingerprint a, GeometryFingerprint b) noexcept
    {
        Accumulator accumulator;
        accumulator.Feed(a.low);
        accumulator.Feed(a.high);
        accumulator.Feed(b.low);
        accumulator.Feed(b.high);
        return accumulator.Finish();
    }
}
