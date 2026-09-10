#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Quantization.hpp"

#include <cstdint>
#include <vector>

namespace ImCut::Geometry
{
    struct CanonicalOptions
    {
        // Lattice step for the canonical form, in millimetres. Two geometries that
        // differ by less than this become identical canonical forms.
        double resolution = Quantization::kCanonicalResolution;

        // Translate so the geometry's minimum corner sits at the origin, making the
        // form invariant to position.
        bool normalizeTranslation = true;

        // Force counter-clockwise traversal, making the form invariant to the direction
        // the artwork happened to be drawn in.
        bool normalizeWinding = true;

        // Rotate a closed ring to a canonical starting node, making the form invariant
        // to which node the authoring tool chose to start at.
        bool normalizeStart = true;

        // Also consider the mirrored form and keep whichever is lexicographically
        // smaller, so a shape and its reflection share a canonical form.
        //
        // Off by default. Reflection equivalence is useful for future symmetry-aware
        // caching, but treating a shape as equal to its mirror image is never
        // acceptable for artwork, which is not symmetric under reflection.
        bool allowReflection = false;
    };

    // Position-, direction- and start-node-invariant integer form of a contour.
    //
    // Canonicalisation never touches the source: it produces a separate description
    // used for caching, duplicate detection, fingerprinting and, later, non-fitting
    // polygon lookup.
    struct CanonicalContour
    {
        // Interleaved quantised x,y node coordinates.
        std::vector<std::int64_t> coordinates;

        // Interleaved quantised control points, four values per segment. Curve handles
        // belong in the canonical form: two contours can share every node and still be
        // completely different shapes, so omitting handles would make the fingerprint
        // collide on genuinely distinct geometry.
        std::vector<std::int64_t> handles;

        std::vector<SegmentKind> kinds;
        bool closed = false;

        // How the source was transformed to reach this form. Enough to map a canonical
        // result back onto the original.
        std::uint32_t startNode = 0;
        bool reversed = false;
        bool mirrored = false;
        std::int64_t translationX = 0;
        std::int64_t translationY = 0;

        double resolution = 0.0;

        [[nodiscard]] std::size_t NodeCount() const noexcept { return coordinates.size() / 2; }
        [[nodiscard]] bool Empty() const noexcept { return coordinates.empty(); }

        [[nodiscard]] bool operator==(const CanonicalContour& other) const noexcept;
        [[nodiscard]] bool operator!=(const CanonicalContour& other) const noexcept
        {
            return !(*this == other);
        }
    };

    struct CanonicalPath
    {
        std::vector<CanonicalContour> contours;
        FillRule fillRule = FillRule::EvenOdd;
        double resolution = 0.0;

        [[nodiscard]] bool operator==(const CanonicalPath& other) const noexcept;
    };

    namespace Canonical
    {
        // Canonicalises one contour. Idempotent: canonicalising an already-canonical
        // contour reproduces it exactly.
        [[nodiscard]] CanonicalContour Canonicalize(const Contour& contour,
                                                    const CanonicalOptions& options = {});

        // Canonicalises a whole path. Contours are ordered by their own canonical
        // content, so two paths that list the same rings in different order produce the
        // same canonical path.
        [[nodiscard]] CanonicalPath Canonicalize(const Path& path,
                                                 const CanonicalOptions& options = {});

        // Reconstructs an approximate contour from a canonical form, accurate to the
        // lattice resolution. Useful for debugging and for round-trip tests; it is not
        // a way to recover the exact original.
        [[nodiscard]] Contour Rebuild(const CanonicalContour& canonical);
    }
}
