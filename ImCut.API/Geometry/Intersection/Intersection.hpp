#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryTypes.hpp"
#include "../Math/Vec2.hpp"

#include <cstdint>
#include <vector>

namespace ImCut::Geometry
{
    // Callers need to distinguish these cases, so intersection never reduces to bool.
    // "Do these two contours cross?" and "do they merely touch at a shared endpoint?"
    // have opposite answers for validity checking, and collapsing them is exactly how
    // a valid path gets reported as self-intersecting.
    enum class IntersectionKind : std::uint8_t
    {
        None = 0,

        // Transversal crossing strictly inside both primitives.
        ProperCross,

        // Contact at an endpoint of at least one primitive.
        EndpointTouch,

        // Contact where the two directions agree to within the tangent tolerance.
        Tangent,

        // Directions are parallel and the primitives lie on the same line, but their
        // spans do not overlap.
        Collinear,

        // Parallel, same line, and the spans do overlap over a positive length.
        Overlap
    };

    [[nodiscard]] constexpr const char* ToString(IntersectionKind kind) noexcept
    {
        switch (kind)
        {
            case IntersectionKind::None:          return "None";
            case IntersectionKind::ProperCross:   return "ProperCross";
            case IntersectionKind::EndpointTouch: return "EndpointTouch";
            case IntersectionKind::Tangent:       return "Tangent";
            case IntersectionKind::Collinear:     return "Collinear";
            case IntersectionKind::Overlap:       return "Overlap";
        }
        return "Unknown";
    }

    // True when the two primitives genuinely share geometry, as opposed to merely
    // lying on a common line without touching.
    [[nodiscard]] constexpr bool IsContact(IntersectionKind kind) noexcept
    {
        return kind == IntersectionKind::ProperCross ||
               kind == IntersectionKind::EndpointTouch ||
               kind == IntersectionKind::Tangent ||
               kind == IntersectionKind::Overlap;
    }

    struct IntersectionPoint
    {
        Vec2 point;

        // Parameters on the first and second primitive, both in [0,1].
        double parameterA = 0.0;
        double parameterB = 0.0;

        IntersectionKind kind = IntersectionKind::None;
    };

    struct LineIntersection
    {
        IntersectionKind kind = IntersectionKind::None;

        Vec2 point;
        double parameterA = 0.0;
        double parameterB = 0.0;

        // For Overlap: the shared span, expressed on A. Meaningless otherwise.
        double overlapMinA = 0.0;
        double overlapMaxA = 0.0;
        Vec2 overlapStart;
        Vec2 overlapEnd;
    };

    namespace Intersect
    {
        // Robust, fully classified segment-segment intersection.
        //
        // Classification is driven by the exact orientation predicate rather than by
        // comparing a computed intersection point against a tolerance, so the
        // cross / touch / collinear decision cannot flip under rounding. Only the
        // position of an already-classified crossing is computed in floating point.
        [[nodiscard]] LineIntersection Lines(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1,
                                             const GeometryTolerance& tolerance) noexcept;

        // Cheap boolean form with early exit; does not compute positions.
        [[nodiscard]] bool LinesIntersect(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1,
                                          const GeometryTolerance& tolerance) noexcept;

        // Line against cubic, solved analytically.
        //
        // Projecting the curve onto the line's normal turns the problem into a single
        // cubic polynomial in t whose real roots are exactly the intersections. That
        // is far more accurate than flattening the curve and intersecting the
        // resulting polyline, which would only ever be as good as its chord tolerance.
        // Returns the number of intersections written (at most 3).
        [[nodiscard]] int LineCubic(Vec2 a0, Vec2 a1, const Segment& curve,
                                    const GeometryTolerance& tolerance,
                                    IntersectionPoint out[3]) noexcept;

        // Cubic against cubic by adaptive subdivision under tight curve bounds.
        //
        // Bounds rejection prunes the pair tree, and recursion stops once both pieces
        // are flat enough to be treated as segments, at which point the robust
        // line-line predicate takes over. Results closer together than the duplicate
        // tolerance are merged, so a tangential contact yields one intersection rather
        // than a cluster of nearly-identical ones.
        void Cubics(const Segment& a, const Segment& b, const GeometryContext& context,
                    std::vector<IntersectionPoint>& out);

        // Dispatches on segment kind, taking the line-line and line-cubic fast paths
        // whenever it can.
        void Segments(const Segment& a, const Segment& b, const GeometryContext& context,
                      std::vector<IntersectionPoint>& out);

        // True as soon as any contact is found. Never builds the full result list, and
        // never allocates - it is the allocation-free any-hit under a context-shaped
        // signature. Not noexcept: nothing here allocates, but the predicates it calls
        // make no such declaration, and the previous version claimed the guarantee while
        // materialising a std::vector for the cubic-cubic case.
        [[nodiscard]] bool SegmentsIntersect(const Segment& a, const Segment& b,
                                             const GeometryContext& context);

        // Allocation-free any-hit test for two cubics.
        //
        // The general Cubics() path collects every intersection into a vector, which is
        // pure waste when the caller only wants to know whether the two touch at all -
        // and a heap allocation per pair is exactly what a million-query workload cannot
        // afford. This keeps its subdivision stack in automatic storage and returns the
        // instant the first contact appears.
        //
        // Not marked noexcept: nothing here allocates, but the predicates it calls are
        // not declared noexcept either, and claiming an exception guarantee the whole
        // call chain does not make would be false.
        [[nodiscard]] bool AnyCubicIntersection(const Segment& a, const Segment& b,
                                                const GeometryTolerance& tolerance,
                                                int maxDepth = 24);

        // Kind-dispatching allocation-free any-hit.
        [[nodiscard]] bool AnySegmentIntersection(const Segment& a, const Segment& b,
                                                  const GeometryTolerance& tolerance);

        // First contact with its location, without building a result list.
        //
        // Between AnySegmentIntersection (a bool) and Segments (every intersection, in a
        // vector) sits the query a collision sweep actually makes: where do these first
        // touch. Answering it through Segments costs a heap allocation per candidate
        // pair, which is what a million-query workload cannot afford.
        //
        // "First" is the first contact the subdivision reaches, not the smallest
        // parameterA. Callers needing the earliest contact along A must use Segments and
        // sort. The subdivision tracks each piece's parameter span, so the reported
        // parameters are on the original curves rather than on a fragment.
        //
        // Not marked noexcept for the same reason as AnyCubicIntersection: nothing here
        // allocates, but the predicates it calls make no such declaration.
        [[nodiscard]] bool FindFirstCubicIntersection(const Segment& a, const Segment& b,
                                                      const GeometryTolerance& tolerance,
                                                      IntersectionPoint& out,
                                                      int maxDepth = 24);

        [[nodiscard]] bool FindFirstSegmentIntersection(const Segment& a, const Segment& b,
                                                        const GeometryTolerance& tolerance,
                                                        IntersectionPoint& out);

        // Merges points closer than `tolerance`, keeping the earliest parameterA.
        // Sorting by (parameterA, parameterB) makes the output order deterministic.
        void MergeNearbyPoints(std::vector<IntersectionPoint>& points, double tolerance);
    }
}
