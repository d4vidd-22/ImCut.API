#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Intersection/Intersection.hpp"

#include <vector>

namespace ImCut::Geometry::Collision
{
    // Separate entry points per question, deliberately.
    //
    // Answering "do these touch?" by building the full intersection list is the single
    // most common way collision code wastes time. Each of these stops as soon as its
    // own answer is decided.

    // Any contact at all. Stops at the first one.
    [[nodiscard]] GeometryResult<bool> Intersects(const Contour& a, const Contour& b, const GeometryContext& context);

    [[nodiscard]] GeometryResult<bool> Intersects(const Path& a, const Path& b, const GeometryContext& context);

    // First contact found, in deterministic order. Stops there.
    [[nodiscard]] GeometryResult<bool> FindFirstIntersection(const Contour& a, const Contour& b,
                                                             const GeometryContext& context,
                                                             IntersectionPoint& out);

    // Every contact, sorted and de-duplicated.
    [[nodiscard]] GeometryResult<std::vector<IntersectionPoint>> FindAllIntersections(
        const Contour& a, const Contour& b, const GeometryContext& context);

    // Regions share interior area. Distinct from Intersects: one contour fully inside
    // another overlaps without their boundaries ever crossing.
    [[nodiscard]] GeometryResult<bool> Overlaps(const Path& a, const Path& b, const GeometryContext& context);

    // True when `inner` lies entirely within `outer`'s filled region.
    [[nodiscard]] GeometryResult<bool> Contains(const Path& outer, const Path& inner, const GeometryContext& context);

    // Boundary-to-boundary distance. Zero when the boundaries touch or cross; this
    // does not report penetration depth.
    //
    // NO STATUS CHANNEL, DELIBERATELY AND WITH A CONSEQUENCE.
    //
    // These two return a bare double, so unlike every other entry point in this header
    // they cannot report Cancelled or ComplexityLimit, and they cannot publish an
    // ErrorBudget. A caller that cancels mid-run gets a number back, not a refusal, and a
    // caller that needs to know how far the answer can be from the truth cannot ask.
    //
    // The prepared overload in PreparedCollision.hpp returns GeometryResult<double> and
    // does all three. It is the supported route for anything that has to react to a
    // status or reason about clearance - which is every nesting and inspection consumer.
    // These raw overloads remain for callers that want a plain measurement from a plain
    // Path and will not act on a status they cannot receive.
    //
    // Recorded rather than changed: widening the return type is a breaking change to a
    // published signature, and section 49 permits a raw/prepared asymmetry only when it
    // is explicitly documented. This is that documentation.
    [[nodiscard]] double MinimumDistance(const Contour& a, const Contour& b,
                                         const GeometryContext& context);

    [[nodiscard]] double MinimumDistance(const Path& a, const Path& b, const GeometryContext& context);

    // Cheap conservative reject shared by all of the above.
    [[nodiscard]] bool BoundsCouldTouch(const Path& a, const Path& b, double tolerance) noexcept;
}
