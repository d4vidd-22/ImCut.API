#pragma once

#include <cmath>

namespace ImCut::Geometry
{
    // All kernel coordinates are millimetres (see Geometry/README.md, "Units").
    struct Vec2
    {
        double x = 0.0;
        double y = 0.0;

        constexpr Vec2() noexcept = default;
        constexpr Vec2(double px, double py) noexcept : x(px), y(py) {}
    };

    [[nodiscard]] constexpr Vec2 operator+(Vec2 a, Vec2 b) noexcept { return { a.x + b.x, a.y + b.y }; }
    [[nodiscard]] constexpr Vec2 operator-(Vec2 a, Vec2 b) noexcept { return { a.x - b.x, a.y - b.y }; }
    [[nodiscard]] constexpr Vec2 operator-(Vec2 a) noexcept { return { -a.x, -a.y }; }
    [[nodiscard]] constexpr Vec2 operator*(Vec2 a, double s) noexcept { return { a.x * s, a.y * s }; }
    [[nodiscard]] constexpr Vec2 operator*(double s, Vec2 a) noexcept { return { a.x * s, a.y * s }; }
    [[nodiscard]] constexpr Vec2 operator/(Vec2 a, double s) noexcept { return { a.x / s, a.y / s }; }

    constexpr Vec2& operator+=(Vec2& a, Vec2 b) noexcept { a.x += b.x; a.y += b.y; return a; }
    constexpr Vec2& operator-=(Vec2& a, Vec2 b) noexcept { a.x -= b.x; a.y -= b.y; return a; }
    constexpr Vec2& operator*=(Vec2& a, double s) noexcept { a.x *= s; a.y *= s; return a; }

    [[nodiscard]] constexpr bool operator==(Vec2 a, Vec2 b) noexcept { return a.x == b.x && a.y == b.y; }
    [[nodiscard]] constexpr bool operator!=(Vec2 a, Vec2 b) noexcept { return !(a == b); }

    [[nodiscard]] constexpr double Dot(Vec2 a, Vec2 b) noexcept { return a.x * b.x + a.y * b.y; }
    [[nodiscard]] constexpr double Cross(Vec2 a, Vec2 b) noexcept { return a.x * b.y - a.y * b.x; }

    [[nodiscard]] constexpr double LengthSquared(Vec2 a) noexcept { return a.x * a.x + a.y * a.y; }
    [[nodiscard]] inline double Length(Vec2 a) noexcept { return std::sqrt(LengthSquared(a)); }

    [[nodiscard]] constexpr double DistanceSquared(Vec2 a, Vec2 b) noexcept
    {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        return dx * dx + dy * dy;
    }

    [[nodiscard]] inline double Distance(Vec2 a, Vec2 b) noexcept { return std::sqrt(DistanceSquared(a, b)); }

    // Rotate 90 degrees counter-clockwise; yields normals without any trig.
    [[nodiscard]] constexpr Vec2 Perpendicular(Vec2 a) noexcept { return { -a.y, a.x }; }

    [[nodiscard]] constexpr Vec2 Lerp(Vec2 a, Vec2 b, double t) noexcept
    {
        return { a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t };
    }

    // Returns the zero vector when the input is too short to normalise safely, so
    // callers never propagate inf/NaN out of a degenerate direction.
    [[nodiscard]] inline Vec2 Normalized(Vec2 a, double minLength = 1e-300) noexcept
    {
        const double lengthSquared = LengthSquared(a);
        if (lengthSquared <= minLength * minLength)
            return {};
        return a * (1.0 / std::sqrt(lengthSquared));
    }

    [[nodiscard]] inline bool IsFinite(Vec2 a) noexcept
    {
        return std::isfinite(a.x) && std::isfinite(a.y);
    }
}
