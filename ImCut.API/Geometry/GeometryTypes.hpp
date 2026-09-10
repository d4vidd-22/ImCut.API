#pragma once

#include "GeometryInstrumentation.hpp"
#include "GeometryResult.hpp"
#include "GeometryTolerance.hpp"
#include "Math/Bounds2.hpp"
#include "Math/Transform2.hpp"
#include "Math/Vec2.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <vector>

namespace ImCut::Geometry
{
    // Compact, deterministic identity that is independent of any COM pointer or heap
    // address, so results stay reproducible across runs and threads.
    template <typename Tag>
    struct GeometryId
    {
        static constexpr std::uint32_t kInvalid = 0xFFFFFFFFu;

        std::uint32_t value = kInvalid;

        constexpr GeometryId() noexcept = default;
        explicit constexpr GeometryId(std::uint32_t v) noexcept : value(v) {}

        [[nodiscard]] constexpr bool IsValid() const noexcept { return value != kInvalid; }
        [[nodiscard]] constexpr std::size_t Index() const noexcept { return static_cast<std::size_t>(value); }

        friend constexpr bool operator==(GeometryId a, GeometryId b) noexcept { return a.value == b.value; }
        friend constexpr bool operator!=(GeometryId a, GeometryId b) noexcept { return a.value != b.value; }
        friend constexpr bool operator<(GeometryId a, GeometryId b) noexcept { return a.value < b.value; }
    };

    using GeometryObjectId = GeometryId<struct GeometryObjectTag>;
    using PathId           = GeometryId<struct PathTag>;
    using ContourId        = GeometryId<struct ContourTag>;
    using SegmentId        = GeometryId<struct SegmentTag>;

    enum class SegmentKind : std::uint8_t
    {
        Line = 0,
        Cubic = 1
    };

    enum class FillRule : std::uint8_t
    {
        EvenOdd = 0,
        NonZero = 1
    };

    enum class Orientation : std::int8_t
    {
        Clockwise = -1,
        Degenerate = 0,
        CounterClockwise = 1
    };

    // Point-in-polygon is deliberately tri-state. Collapsing Boundary into a bool is
    // what makes containment classification flip under floating-point noise.
    enum class PointClassification : std::uint8_t
    {
        Outside = 0,
        Inside = 1,
        Boundary = 2
    };

    // Kernel winding convention: outer rings run counter-clockwise, holes clockwise.
    enum class ContourRole : std::uint8_t
    {
        Outer = 0,
        Hole = 1
    };

    // A single self-contained segment. Lines are stored as cubics whose handles sit
    // at the one-third points, which makes the cubic basis reproduce the straight
    // segment exactly and with uniform speed. Evaluation can therefore stay
    // branch-free, while `kind` remains available as a fast-path hint for code that
    // benefits from knowing a segment is straight.
    struct Segment
    {
        Vec2 p0;
        Vec2 c1;
        Vec2 c2;
        Vec2 p1;
        SegmentKind kind = SegmentKind::Line;

        [[nodiscard]] static constexpr Segment Line(Vec2 a, Vec2 b) noexcept
        {
            return { a, Lerp(a, b, 1.0 / 3.0), Lerp(a, b, 2.0 / 3.0), b, SegmentKind::Line };
        }

        [[nodiscard]] static constexpr Segment Cubic(Vec2 a, Vec2 h1, Vec2 h2, Vec2 b) noexcept
        {
            return { a, h1, h2, b, SegmentKind::Cubic };
        }

        [[nodiscard]] constexpr bool IsLine() const noexcept { return kind == SegmentKind::Line; }
    };

    // Storage form of a contour: nodes are the on-curve points, handles hold exactly
    // two control points per segment, and kinds one tag per segment. Parallel arrays
    // keep traversal sequential and let a whole contour live in a few allocations
    // instead of one per segment.
    struct Contour IMCUT_GEOMETRY_COUNTED(ContourTag)
    {
        std::vector<Vec2> nodes;
        std::vector<Vec2> handles;
        std::vector<SegmentKind> kinds;
        bool closed = false;

        [[nodiscard]] std::size_t NodeCount() const noexcept { return nodes.size(); }

        [[nodiscard]] std::size_t SegmentCount() const noexcept
        {
            if (nodes.size() < 2) return 0;
            return closed ? nodes.size() : nodes.size() - 1;
        }

        [[nodiscard]] Segment SegmentAt(std::size_t index) const noexcept
        {
            const std::size_t count = nodes.size();
            const Vec2 start = nodes[index];
            const Vec2 end = nodes[index + 1 == count ? 0 : index + 1];
            return { start, handles[index * 2], handles[index * 2 + 1], end, kinds[index] };
        }

        void Clear() noexcept
        {
            nodes.clear();
            handles.clear();
            kinds.clear();
            closed = false;
        }

        void Reserve(std::size_t segmentCount)
        {
            nodes.reserve(segmentCount + 1);
            handles.reserve(segmentCount * 2);
            kinds.reserve(segmentCount);
        }

        // Starts the contour. Must be called exactly once, before any Line/Cubic.
        void MoveTo(Vec2 point)
        {
            nodes.push_back(point);
        }

        void LineTo(Vec2 point)
        {
            const Vec2 from = nodes.back();
            handles.push_back(Lerp(from, point, 1.0 / 3.0));
            handles.push_back(Lerp(from, point, 2.0 / 3.0));
            kinds.push_back(SegmentKind::Line);
            nodes.push_back(point);
        }

        void CubicTo(Vec2 control1, Vec2 control2, Vec2 point)
        {
            handles.push_back(control1);
            handles.push_back(control2);
            kinds.push_back(SegmentKind::Cubic);
            nodes.push_back(point);
        }

        // Closes with a straight closing segment back to the first node. When the
        // last node already coincides with the first within `weldTolerance` it is
        // dropped instead, so the closing segment is never a zero-length stub.
        void Close(double weldTolerance)
        {
            if (nodes.size() < 2)
            {
                closed = nodes.size() == 1;
                return;
            }

            if (DistanceSquared(nodes.back(), nodes.front()) <= weldTolerance * weldTolerance)
            {
                // The duplicated node ended the final segment; dropping it promotes
                // that segment to the closing one, handles and all.
                nodes.pop_back();

                // Unless there is nothing left to close.
                //
                // MoveTo(p); LineTo(p); Close(0.0) is a legal sequence, and it welded
                // down to one node while keeping the segment that node terminated:
                // nodes=1, handles=2, kinds=1, SegmentCount()=0. That contour fails
                // HasConsistentArrays, so a legal caller could build something no
                // operation would accept. A single point has no segments, so the
                // segment goes with the node it ended.
                if (nodes.size() < 2)
                {
                    if (!kinds.empty()) kinds.pop_back();
                    if (handles.size() >= 2) { handles.pop_back(); handles.pop_back(); }
                    closed = false;
                    return;
                }

                closed = true;
                return;
            }

            const Vec2 from = nodes.back();
            const Vec2 to = nodes.front();
            handles.push_back(Lerp(from, to, 1.0 / 3.0));
            handles.push_back(Lerp(from, to, 2.0 / 3.0));
            kinds.push_back(SegmentKind::Line);
            closed = true;
        }

        [[nodiscard]] bool HasConsistentArrays() const noexcept
        {
            return kinds.size() * 2 == handles.size() && kinds.size() == SegmentCount();
        }

        // Every entry of `kinds` names a segment kind this kernel knows.
        //
        // An enum class does not constrain the VALUE - a std::uint8_t of 255 stored into
        // a SegmentKind is well-formed C++ and passed every check the kernel had. It
        // then reached the switch in the flattener, missed both labels, and fell through
        // to the cubic branch, so a garbage byte silently became "this is a curve".
        // Data arriving from a COM adapter, a file or a fuzzer is exactly where such a
        // value comes from.
        [[nodiscard]] bool HasKnownSegmentKinds() const noexcept
        {
            for (const SegmentKind kind : kinds)
            {
                if (kind != SegmentKind::Line && kind != SegmentKind::Cubic)
                    return false;
            }
            return true;
        }

        [[nodiscard]] bool HasFiniteCoordinates() const noexcept
        {
            for (const Vec2& node : nodes)
                if (!IsFinite(node)) return false;
            for (const Vec2& handle : handles)
                if (!IsFinite(handle)) return false;
            return true;
        }

        // The precondition every operation that indexes these arrays must check first.
        //
        // SegmentAt() reads handles[index*2 + 1] with no bounds check, because checking
        // per segment on a hot path is unacceptable. That is only safe if the structure
        // was validated once, up front - hence this gate rather than defensive checks
        // scattered through the traversals.
        [[nodiscard]] bool IsStructurallyValid() const noexcept
        {
            return HasConsistentArrays() && HasKnownSegmentKinds() && HasFiniteCoordinates();
        }
    };

    // One source path: a set of contours sharing a fill rule, plus the identity that
    // ties it back to whatever produced it (a Corel shape, an SVG element, a test).
    struct Path IMCUT_GEOMETRY_COUNTED(PathTag)
    {
        std::vector<Contour> contours;
        FillRule fillRule = FillRule::EvenOdd;
        GeometryObjectId sourceObject{};

        // Approximation this geometry already carries.
        //
        // The budget travels with the geometry, not just with the call that produced it.
        // Without that, feeding a boolean result back into another boolean loses the
        // first stage's error entirely and the second result claims to be as accurate as
        // if it had come straight from the source.
        ErrorBudget budget{};

        [[nodiscard]] std::size_t SegmentCount() const noexcept
        {
            std::size_t total = 0;
            for (const Contour& contour : contours)
                total += contour.SegmentCount();
            return total;
        }

        [[nodiscard]] bool Empty() const noexcept { return contours.empty(); }

        // A known fill rule, for the same reason a Contour checks its segment kinds:
        // the enum type does not constrain the value, and an out-of-range byte was
        // reaching the backend where it silently read as NonZero.
        [[nodiscard]] bool HasKnownFillRule() const noexcept
        {
            return fillRule == FillRule::EvenOdd || fillRule == FillRule::NonZero;
        }

        [[nodiscard]] bool IsStructurallyValid() const noexcept
        {
            if (!HasKnownFillRule())
                return false;
            for (const Contour& contour : contours)
                if (!contour.IsStructurallyValid()) return false;
            return true;
        }

        [[nodiscard]] bool HasOpenContours() const noexcept
        {
            for (const Contour& contour : contours)
                if (!contour.closed && contour.SegmentCount() > 0) return true;
            return false;
        }
    };
}

template <typename Tag>
struct std::hash<ImCut::Geometry::GeometryId<Tag>>
{
    [[nodiscard]] std::size_t operator()(ImCut::Geometry::GeometryId<Tag> id) const noexcept
    {
        return std::hash<std::uint32_t>{}(id.value);
    }
};
