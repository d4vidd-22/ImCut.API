#pragma once

#include "../GeometryTolerance.hpp"
#include "../GeometryTypes.hpp"

#include <vector>

namespace ImCut::Geometry::Winding
{
    // Kernel convention: outer rings run counter-clockwise, holes clockwise.
    //
    // Which convention is chosen matters far less than applying one consistently, and
    // winding is never used to *decide* what is a hole - that comes from containment.
    // Winding is normalised only after roles are known. Deriving the role from winding
    // instead is what breaks on artwork whose rings were drawn in arbitrary directions,
    // which is most real artwork.
    [[nodiscard]] constexpr Orientation ExpectedOrientation(ContourRole role) noexcept
    {
        return role == ContourRole::Outer ? Orientation::CounterClockwise : Orientation::Clockwise;
    }

    // Reverses traversal direction while preserving the exact geometry: node order is
    // inverted and each segment's two handles swap, so curves retrace identically
    // rather than approximately.
    void Reverse(Contour& contour);

    [[nodiscard]] Contour Reversed(const Contour& contour);

    void Reverse(std::vector<Vec2>& ring);

    // Forces the contour to the orientation its role requires. Returns true when the
    // contour was actually reversed. Degenerate (zero-area) contours are left alone,
    // since they have no meaningful orientation to correct.
    bool Normalize(Contour& contour, ContourRole role, const GeometryTolerance& tolerance);

    bool Normalize(std::vector<Vec2>& ring, ContourRole role, const GeometryTolerance& tolerance);
}
