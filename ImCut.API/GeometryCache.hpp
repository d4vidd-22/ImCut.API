#pragma once

#include <array>
#include <cmath>
#include <cstddef>
#include <vector>

namespace ImCut::GeometryCache
{
    struct Point { double x = 0, y = 0; };
    struct Segment
    {
        bool cubic = false;
        std::array<Point, 4> points{};
    };
    struct Path
    {
        bool closed = false;
        long nodes = 0;
        std::vector<Segment> segments;
    };
    using Curve = std::vector<Path>;
    inline constexpr std::size_t MaxSegments = 10000;

    inline bool Match(const Curve& a, const Curve& b, double tolerance,
        long& mode, double& p1, double& p2)
    {
        mode = 0; p1 = p2 = 0;
        if (a.empty() || a.size() != b.size() || !std::isfinite(tolerance) || tolerance < 0)
            return false;
        bool anchored = false, move = true, mirror = true;
        double dx = 0, dy = 0, axis = 0;
        std::size_t count = 0;
        for (std::size_t i = 0; i < a.size(); ++i)
        {
            if (a[i].closed != b[i].closed || a[i].nodes != b[i].nodes ||
                a[i].segments.empty() || a[i].segments.size() != b[i].segments.size())
                return false;
            for (std::size_t j = 0; j < a[i].segments.size(); ++j)
            {
                if (++count > MaxSegments) return false;
                const auto& as = a[i].segments[j];
                const auto& bs = b[i].segments[j];
                if (as.cubic != bs.cubic) return false;
                for (std::size_t k = 0; k < 4; ++k)
                {
                    if (!as.cubic && (k == 1 || k == 2)) continue;
                    const auto ap = as.points[k], bp = bs.points[k];
                    if (!std::isfinite(ap.x) || !std::isfinite(ap.y) ||
                        !std::isfinite(bp.x) || !std::isfinite(bp.y)) return false;
                    const double x = bp.x - ap.x, y = bp.y - ap.y, sum = bp.x + ap.x;
                    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(sum)) return false;
                    if (!anchored) { dx = x; dy = y; axis = sum; anchored = true; }
                    if (std::abs(y - dy) > tolerance) return false;
                    move = move && std::abs(x - dx) <= tolerance;
                    mirror = mirror && std::abs(sum - axis) <= tolerance;
                    if (!move && !mirror) return false;
                }
            }
        }
        if (!anchored) return false;
        mode = move ? 1 : 2;
        p1 = move ? dx : axis;
        p2 = dy;
        return true;
    }
}
