#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

// INTERNAL. Not part of the public ImCut::Geometry surface and must not be included
// by consumers. Boolean.hpp / Offset.hpp / Minkowski.hpp are the only supported entry
// points; Nesting, GeometryDoctor, Cut and AutoBleeding call those and never touch a
// backend directly.
//
// Everything here is expressed in backend-neutral integers so the concrete polygon
// engine can be replaced without any consumer changing. Choosing the scale, checking
// range, converting mm to and from integers, rebuilding topology and validating the
// result are all the kernel's job, not the backend's - a backend only clips.

namespace ImCut::Geometry::Backend
{
    struct IntPoint
    {
        std::int64_t x = 0;
        std::int64_t y = 0;
    };

    using IntRing = std::vector<IntPoint>;
    using IntRings = std::vector<IntRing>;
    using IntRingsView = std::span<const IntRing>;
    using IntTriangle = std::array<IntPoint, 3>;
    using IntTriangles = std::vector<IntTriangle>;

    struct PolyNode
    {
        IntRing ring;
        std::uint32_t parent = 0xFFFFFFFFu;
        std::vector<std::uint32_t> children;
        int depth = 0;
        bool hole = false;

        static constexpr std::uint32_t kNoParent = 0xFFFFFFFFu;
    };

    // Nesting hierarchy as reported by the backend, kept so the kernel can rebuild
    // outer/hole roles without re-deriving containment from scratch.
    struct PolyTree
    {
        std::vector<PolyNode> nodes;
        std::vector<std::uint32_t> roots;

        void Clear() noexcept { nodes.clear(); roots.clear(); }
        [[nodiscard]] bool Empty() const noexcept { return nodes.empty(); }
    };

    enum class Operation : std::uint8_t { Union, Intersection, Difference, Xor };
    enum class TriangulationStatus : std::uint8_t
    {
        Success, Empty, PathsIntersect, Failed, OutOfMemory
    };
    enum class Fill : std::uint8_t { EvenOdd, NonZero };
    enum class Join : std::uint8_t { Miter, Round, Square, Bevel };
    enum class Cap : std::uint8_t { ClosedPolygon, ClosedJoined, OpenButt, OpenSquare, OpenRound };

    // Outcome of a tree-producing backend call.
    //
    // DepthLimit is separated from Failed because the two mean opposite things to a
    // caller: Failed is "the engine could not compute this", DepthLimit is "the answer
    // is well defined but nests deeper than the caller allowed". The kernel maps the
    // first to NumericalFailure and the second to ComplexityLimit, and neither may be
    // reported as the other.
    enum class BackendStatus : std::uint8_t { Success, Failed, DepthLimit, OutOfMemory };

    // Nesting depth a tree-producing call may return before it gives up. Depth is
    // input-controlled, so it has to be bounded: walking it used to be recursive on
    // both sides of the backend boundary and took the process down past ~4950 levels.
    struct TreeLimits
    {
        int maxNestingDepth = 256;
    };

    struct OffsetRequest
    {
        double delta = 0.0;
        Join join = Join::Round;
        Cap cap = Cap::ClosedPolygon;
        double miterLimit = 2.0;
        double arcTolerance = 0.0;
    };

    // Implementations must be stateless and reentrant: the kernel is thread-safe and
    // will call one instance from several workers at once.
    class IPolygonBackend
    {
    public:
        virtual ~IPolygonBackend() = default;

        [[nodiscard]] virtual const char* Name() const noexcept = 0;

        [[nodiscard]] virtual BackendStatus Execute(Operation operation, Fill fill,
                                                    IntRingsView subject, IntRingsView clip,
                                                    const TreeLimits& limits,
                                                    PolyTree& out) const = 0;

        // Intermediate Boolean stages often need only a flat ring set as input to the
        // next stage. Requiring a PolyTree there builds hierarchy, copies every ring
        // into PolyNode and then immediately flattens it again. ExecuteFlat keeps that
        // avoidable work behind the backend-neutral boundary; only a final result that
        // must recover outer/hole topology uses Execute above.
        [[nodiscard]] virtual BackendStatus ExecuteFlat(Operation operation, Fill fill,
                                                        IntRingsView subject, IntRingsView clip,
                                                        IntRings& out) const = 0;

        [[nodiscard]] virtual BackendStatus Offset(IntRingsView subject,
                                                   const OffsetRequest& request,
                                                   const TreeLimits& limits,
                                                   PolyTree& out) const = 0;

        // No Minkowski here. It used to be a backend call, and the identity the backend
        // route relied on - A (+) B == A union (dA (+) B) - is only valid when B is
        // star-shaped about the origin. Minkowski/ now decomposes both operands into
        // convex pieces and merges edges exactly, which needs no lattice, no clipping
        // and no engine support. See Polygon/Minkowski.cpp.

        // Rings describe one filled region: outer rings and holes together. Output is
        // one fixed-size triple per triangle. Fixed-size storage is deliberate: a
        // vector<IntRing> allocated one heap buffer for every triangle even though its
        // cardinality is statically three. The backend may merge duplicate and
        // collinear vertices, so output points are not guaranteed to be a subset of the
        // input points - the kernel re-indexes them.
        [[nodiscard]] virtual TriangulationStatus Triangulate(IntRingsView rings,
                                                              IntTriangles& out) const = 0;
    };

    // The backend the kernel currently uses. Swapping engines means changing this one
    // function; no consumer and no other kernel file is affected.
    [[nodiscard]] const IPolygonBackend& Default() noexcept;
}
