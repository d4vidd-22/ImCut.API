#pragma once

#include <cstddef>

namespace ImCut::Geometry::Polynomial
{
    // Real-root solvers used by curve extrema, curve/line intersection and closest
    // point. All of them:
    //   - return roots in ascending order, so downstream results are deterministic;
    //   - deduplicate roots that coincide within `epsilon`;
    //   - degrade gracefully when leading coefficients vanish, rather than dividing
    //     by a near-zero and producing garbage.

    // a*t^2 + b*t + c = 0. Returns the number of distinct real roots written.
    [[nodiscard]] int SolveQuadratic(double a, double b, double c,
                                     double roots[2], double epsilon = 1e-12) noexcept;

    // a*t^3 + b*t^2 + c*t + d = 0. Returns the number of distinct real roots written.
    [[nodiscard]] int SolveCubic(double a, double b, double c, double d,
                                 double roots[3], double epsilon = 1e-12) noexcept;

    // Filters roots to the closed interval [low, high], snapping values that sit
    // within `epsilon` of an endpoint onto it. Returns the surviving count.
    [[nodiscard]] int ClampToInterval(double* roots, int count,
                                      double low, double high, double epsilon) noexcept;
}
