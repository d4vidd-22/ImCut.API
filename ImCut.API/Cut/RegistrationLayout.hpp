#pragma once
#include <algorithm>
#include <cmath>

namespace ImCut::Cut
{
    struct RegistrationDensity
    {
        int count = 0;
        double preferredSpacing = 25.0;
        double minimumSpacing = 25.0;
    };

    inline RegistrationDensity AdaptiveRegistrationDensity(double width, double height)
    {
        if (!std::isfinite(width) || !std::isfinite(height) || width < 5 || height < 5)
            return {};
        const double w = std::min(width, 100000.0), h = std::min(height, 100000.0);
        const int minimum = std::min(w, h) >= 150.0 ? 4 : 3;
        const double longSide = std::max(w, h);
        const double shortSide = std::min(w, h);
        const double perimeter = 2.0 * (w + h);
        const double compactness = std::min(1.0, 2.0 * shortSide / longSide);
        const double demand = perimeter / 180.0 * compactness + w * h / 250000.0;
        const double lengthFloor = std::ceil(longSide / 300.0) + 2.0;
        const int count = static_cast<int>(std::clamp(
            std::ceil(std::max(demand, lengthFloor)), double(minimum), 72.0));
        const double preferredSpacing = std::clamp(
            std::min(perimeter / count * 0.80, std::sqrt(w * h / count) * 0.50),
            25.0, 120.0);
        const double minimumSpacing = std::max(25.0, preferredSpacing * 0.60);
        return {count, preferredSpacing, minimumSpacing};
    }
}
