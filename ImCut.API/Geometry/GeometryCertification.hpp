#pragma once

#include <cstdint>

namespace ImCut::Geometry
{
    // Certification is relative to the prepared polygonal representation and its
    // accumulated ErrorBudget. It never claims a flattened polygon is identical to the
    // original analytic curve.
    enum class GeometryCertification : std::uint8_t
    {
        Certified,
        ConservativeSuperset,
        BoundedApproximation,
        CandidateOnly
    };
}
