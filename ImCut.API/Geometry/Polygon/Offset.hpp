#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"

namespace ImCut::Geometry
{
    enum class JoinType : std::uint8_t { Miter, Round, Square, Bevel };
    enum class EndType : std::uint8_t { ClosedPolygon, ClosedJoined, OpenButt, OpenSquare, OpenRound };

    struct OffsetOptions
    {
        JoinType join = JoinType::Round;
        EndType end = EndType::ClosedPolygon;

        // Cap on how far a miter may extend, as a multiple of the offset distance.
        double miterLimit = 2.0;

        // Maximum deviation of a round join from the true arc, in millimetres.
        // Zero lets the kernel derive it from the offset distance.
        double arcTolerance = 0.0;

        bool pruneDegenerate = true;

        // Check the result for self-intersection and report it.
        //
        // On by default: a backend returning vertices is not proof of a valid result,
        // and offsetting a concave region really can fold it onto itself. It is the
        // dominant cost of a large offset though - each output ring needs its own
        // hierarchy - so a caller that will run its own validation downstream, or that
        // only needs bounds, can turn it off deliberately.
        bool validateSelfIntersection = true;
    };

    struct OffsetReport
    {
        std::size_t inputRings = 0;
        std::size_t outputRings = 0;
        std::size_t prunedDegenerate = 0;
        bool nonFiniteRemoved = false;
        bool selfIntersecting = false;
        double quantizationStep = 0.0;
        double arcToleranceUsed = 0.0;
    };

    // Polygon offsetting (inflate for positive distance, deflate for negative).
    //
    // A backend returning vertices is not proof of a usable result, so the output is
    // checked for finiteness, collapsed rings and topology damage before being
    // returned. Findings are reported; no repair policy is applied here - that belongs
    // to the future ImCut::GeometryDoctor.
    namespace Offset
    {
        // Grows or shrinks a filled REGION.
        //
        // Every contour must be closed; an open one is rejected with InvalidTopology.
        // Offsetting a polyline as though it bounded a region is a different operation
        // with a different answer, and doing it implicitly invents area.
        [[nodiscard]] GeometryResult<Path> Execute(const Path& path, double distance,
                                                   const GeometryContext& context,
                                                   const OffsetOptions& options = {},
                                                   OffsetReport* report = nullptr);

        // Offsetting by zero must reproduce the input region, not merely something
        // close to it. Short-circuited so it costs nothing.
        [[nodiscard]] GeometryResult<Path> Inflate(const Path& path, double distance,
                                                   const GeometryContext& context,
                                                   const OffsetOptions& options = {});

        // Turns an open polyline into a region by stroking it to the given half-width.
        //
        // The explicit counterpart to Execute: closing a path is something the caller
        // asks for by name, never something a region operation does behind their back.
        // `halfWidth` must be positive. Closed contours in the input are stroked as
        // closed outlines rather than filled.
        [[nodiscard]] GeometryResult<Path> StrokeOpenPath(const Path& path, double halfWidth,
                                                          const GeometryContext& context,
                                                          const OffsetOptions& options = {},
                                                          OffsetReport* report = nullptr);
    }
}
