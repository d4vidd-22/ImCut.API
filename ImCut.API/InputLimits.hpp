#pragma once
#include <cmath>

namespace ImCut::InputLimits
{
    inline constexpr double MinBleedMm = 0.001;
    inline constexpr double MaxBleedMm = 1000.0;
    inline bool ValidBleed(double value) noexcept
    {
        return std::isfinite(value) && value >= MinBleedMm && value <= MaxBleedMm;
    }
    inline float BleedOrDefault(double value, float fallback = 3.0f) noexcept
    {
        return ValidBleed(value) ? static_cast<float>(value) : fallback;
    }
}
