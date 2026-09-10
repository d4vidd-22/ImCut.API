#include "GeometryResult.hpp"

#include <cstdio>
#include <cstdlib>

namespace ImCut::Geometry
{
    void OnResultInvariantViolation(GeometryStatus status, const char* what) noexcept
    {
        // Reading a value that does not exist is a programming error, not a geometric
        // one, so it is the one case that does not become a status. Failing loudly here
        // is the point: the alternative is a default-constructed answer travelling on as
        // if the operation had succeeded.
        std::fprintf(stderr, "ImCut::Geometry invariant violation: %s (status = %s)\n",
                     what, ToString(status));
        std::fflush(stderr);
        std::abort();
    }
}
