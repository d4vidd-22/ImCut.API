#pragma once

#include "../GeometryTolerance.hpp"
#include "../GeometryTypes.hpp"
#include "../Math/Vec2.hpp"

#include <cstddef>
#include <vector>

namespace ImCut::Geometry
{
    namespace Hull
    {
        // Andrew's monotone chain.
        //
        // O(n log n), no floating-point branching beyond the exact orientation
        // predicate, and fully deterministic: the lexicographic sort has an explicit
        // total order, so equal points cannot reorder between runs.
        //
        // Output is counter-clockwise, starts at the lexicographically smallest point,
        // and excludes collinear interior points. Fewer than three distinct input
        // points yield the distinct points themselves.
        [[nodiscard]] std::vector<Vec2> Compute(const Vec2* points, std::size_t count);

        [[nodiscard]] inline std::vector<Vec2> Compute(const std::vector<Vec2>& points)
        {
            return Compute(points.data(), points.size());
        }

        // Hull of a contour's flattened outline. Curved segments are flattened at
        // `tolerance` first, so the hull is only as tight as that approximation - the
        // control points alone would give a hull that is correct but visibly loose.

        // Writes into `out` without freeing its capacity, so a caller looping over many
        // pieces reuses one buffer instead of allocating per hull.
        void ComputeInto(const Vec2* points, std::size_t count, std::vector<Vec2>& out);

        [[nodiscard]] double Area(const std::vector<Vec2>& hull) noexcept;
    }
}
