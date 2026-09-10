#include "ConvexHull.hpp"

#include "../Curves/Flatten.hpp"
#include "../Math/Predicates.hpp"

#include <algorithm>

namespace ImCut::Geometry::Hull
{
    void ComputeInto(const Vec2* points, std::size_t count, std::vector<Vec2>& out)
    {
        out.clear();

        if (points == nullptr || count == 0)
            return;

        std::vector<Vec2> sorted(points, points + count);

        // Total order with an explicit tie-break, so the result never depends on the
        // sort's handling of equal elements.
        std::sort(sorted.begin(), sorted.end(), [](Vec2 a, Vec2 b) noexcept
        {
            if (a.x != b.x) return a.x < b.x;
            return a.y < b.y;
        });

        sorted.erase(std::unique(sorted.begin(), sorted.end(),
                                 [](Vec2 a, Vec2 b) noexcept { return a.x == b.x && a.y == b.y; }),
                     sorted.end());

        const std::size_t unique = sorted.size();
        if (unique < 3)
        {
            out = std::move(sorted);
            return;
        }

        out.resize(unique * 2);
        std::size_t written = 0;

        // Lower hull, then upper hull. Popping on a non-left turn keeps only vertices
        // that are strictly extreme, which is what drops collinear interior points.
        for (std::size_t i = 0; i < unique; ++i)
        {
            while (written >= 2 &&
                   Predicates::Orient2D(out[written - 2], out[written - 1], sorted[i]) <= 0.0)
            {
                --written;
            }
            out[written++] = sorted[i];
        }

        const std::size_t lowerSize = written + 1;
        for (std::size_t i = unique - 1; i-- > 0;)
        {
            while (written >= lowerSize &&
                   Predicates::Orient2D(out[written - 2], out[written - 1], sorted[i]) <= 0.0)
            {
                --written;
            }
            out[written++] = sorted[i];
        }

        // The last point repeats the first; drop it so the ring is stored implicitly
        // closed, exactly like every other ring in the kernel.
        out.resize(written > 0 ? written - 1 : 0);
    }

    std::vector<Vec2> Compute(const Vec2* points, std::size_t count)
    {
        std::vector<Vec2> hull;
        ComputeInto(points, count, hull);
        return hull;
    }

    double Area(const std::vector<Vec2>& hull) noexcept
    {
        if (hull.size() < 3)
            return 0.0;

        // Referenced to the hull's own first vertex. The loop integral is translation
        // invariant, so this is exact; referenced to the absolute origin it lost
        // log10(d^2/s^2) digits and reported 2.0 for a 0.01 mm^2 square at 1e8 mm.
        // Same fault, same fix, as Metrics::SignedArea.
        const Vec2 origin = hull[0];

        double total = 0.0;
        for (std::size_t i = 0; i < hull.size(); ++i)
        {
            const Vec2 a = hull[i] - origin;
            const Vec2 b = hull[(i + 1) % hull.size()] - origin;
            total += a.x * b.y - b.x * a.y;
        }
        return total * 0.5;
    }
}
