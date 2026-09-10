// The one and only translation unit in ImCut::Geometry that includes Clipper2.
// Nothing here appears in any public header, so no Clipper2 type can reach a consumer.
//
// Clipper2 2.0.1, Boost Software License 1.0.
// Vendored under Geometry/ThirdParty/Clipper2 (see LICENSE there); no runtime DLL and
// no network access at build time.

#include "../GeometryInstrumentation.hpp"
#include "PolygonBackend.hpp"

#include "clipper2/clipper.h"

#include <cstddef>
#include <new>
#include <vector>

namespace ImCut::Geometry::Backend
{
    namespace
    {
        [[nodiscard]] Clipper2Lib::ClipType ToClipper(Operation operation) noexcept
        {
            switch (operation)
            {
                case Operation::Intersection: return Clipper2Lib::ClipType::Intersection;
                case Operation::Difference:   return Clipper2Lib::ClipType::Difference;
                case Operation::Xor:          return Clipper2Lib::ClipType::Xor;
                case Operation::Union:
                default:                      return Clipper2Lib::ClipType::Union;
            }
        }

        [[nodiscard]] Clipper2Lib::FillRule ToClipper(Fill fill) noexcept
        {
            return fill == Fill::NonZero ? Clipper2Lib::FillRule::NonZero
                                         : Clipper2Lib::FillRule::EvenOdd;
        }

        [[nodiscard]] Clipper2Lib::JoinType ToClipper(Join join) noexcept
        {
            switch (join)
            {
                case Join::Miter:  return Clipper2Lib::JoinType::Miter;
                case Join::Square: return Clipper2Lib::JoinType::Square;
                case Join::Bevel:  return Clipper2Lib::JoinType::Bevel;
                case Join::Round:
                default:           return Clipper2Lib::JoinType::Round;
            }
        }

        [[nodiscard]] Clipper2Lib::EndType ToClipper(Cap cap) noexcept
        {
            switch (cap)
            {
                case Cap::ClosedJoined: return Clipper2Lib::EndType::Joined;
                case Cap::OpenButt:     return Clipper2Lib::EndType::Butt;
                case Cap::OpenSquare:   return Clipper2Lib::EndType::Square;
                case Cap::OpenRound:    return Clipper2Lib::EndType::Round;
                case Cap::ClosedPolygon:
                default:                return Clipper2Lib::EndType::Polygon;
            }
        }

        [[nodiscard]] Clipper2Lib::Path64 ToClipper(const IntRing& ring)
        {
            Clipper2Lib::Path64 path;
            path.reserve(ring.size());
            for (const IntPoint& point : ring)
                path.emplace_back(point.x, point.y);
            return path;
        }

        [[nodiscard]] Clipper2Lib::Paths64 ToClipper(IntRingsView rings)
        {
            // The one place a ring's point data is genuinely duplicated: the backend
            // keeps its own Point64 layout, so crossing into it costs a copy per ring.
            // Counted here rather than on IntRing itself, which is a std::vector alias
            // the kernel does not own and cannot give a base class.
            IMCUT_GEOMETRY_COUNT_INT_RINGS(rings.size());
            Clipper2Lib::Paths64 paths;
            paths.reserve(rings.size());
            for (const IntRing& ring : rings)
                paths.push_back(ToClipper(ring));
            return paths;
        }

        // Rings with consecutive duplicate points removed, for the triangulator only.
        //
        // Hygiene, not a fix. A zero-length edge carries no geometry, so handing one to
        // a triangulator is meaningless work, and removing it here shrinks the input
        // before the engine sees it.
        //
        // It was ATTEMPTED as the fix for the heap-use-after-free that mutant 1602 of
        // the corpus fuzz probe triggers inside Clipper2's
        // Delaunay::MergeDupOrCollinearVertices, on the reasoning that only consecutive
        // ring points get an edge between them, so only they can merge into the
        // self-loop that corrupts the edge list. It did not work: the triangulator also
        // creates coincident vertices internally, because SplitEdge calls
        // CreateEdge(newT, oldT) and those two can be distinct but coincident. The
        // precondition is therefore not expressible on the input at all.
        //
        // The actual fix is ThirdParty/Clipper2/PATCHES/0001, which skips self-loop
        // edges inside the engine. This stayed because it is still worth doing.
        [[nodiscard]] Clipper2Lib::Paths64 ToClipperWithoutZeroLengthEdges(IntRingsView rings)
        {
            Clipper2Lib::Paths64 paths;
            paths.reserve(rings.size());

            for (const IntRing& ring : rings)
            {
                Clipper2Lib::Path64 path;
                path.reserve(ring.size());

                for (const IntPoint& point : ring)
                {
                    if (!path.empty() && path.back().x == point.x && path.back().y == point.y)
                        continue;
                    path.emplace_back(point.x, point.y);
                }

                // The ring is implicitly closed, so a first point repeated at the end is
                // the same zero-length edge seen from the other side.
                while (path.size() > 1 && path.front().x == path.back().x &&
                       path.front().y == path.back().y)
                {
                    path.pop_back();
                }

                if (path.size() >= 3)
                    paths.push_back(std::move(path));
            }

            return paths;
        }

        [[nodiscard]] IntRing FromClipper(const Clipper2Lib::Path64& path)
        {
            IntRing ring;
            ring.reserve(path.size());
            for (const Clipper2Lib::Point64& point : path)
                ring.push_back({ point.x, point.y });
            return ring;
        }

        void FromClipper(const Clipper2Lib::Paths64& paths, IntRings& out)
        {
            out.clear();
            out.reserve(paths.size());
            for (const Clipper2Lib::Path64& path : paths)
                out.push_back(FromClipper(path));
        }

        void FromClipperTriangles(const Clipper2Lib::Paths64& paths, IntTriangles& out)
        {
            out.clear();
            out.reserve(paths.size());
            for (const Clipper2Lib::Path64& path : paths)
            {
                if (path.size() != 3)
                    continue;
                out.push_back({
                    IntPoint{ path[0].x, path[0].y },
                    IntPoint{ path[1].x, path[1].y },
                    IntPoint{ path[2].x, path[2].y }
                });
            }
        }

        // Walks the Clipper2 tree with an explicit stack.
        //
        // This was recursive, one frame per nesting level, and nesting depth is
        // dictated by the input: past roughly 4950 levels the process died with
        // 0xC00000FD instead of returning a status. Depth is now capped, and exceeding
        // the cap is reported rather than attempted.
        //
        // Children are pushed in reverse so they pop in source order, which keeps the
        // emitted node indices identical to what the recursive version produced. That
        // matters: PolyTreeToPath walks `nodes` in order and the kernel's determinism
        // gate compares output byte for byte.
        [[nodiscard]] bool CollectTree(const Clipper2Lib::PolyPath64& source, PolyTree& tree,
                                       int maxDepth)
        {
            struct Frame
            {
                const Clipper2Lib::PolyPath64* node;
                std::uint32_t parent;
                int depth;
            };

            std::vector<Frame> stack;
            for (std::size_t i = source.Count(); i-- > 0; )
            {
                if (const Clipper2Lib::PolyPath64* child = source.Child(i); child != nullptr)
                    stack.push_back({ child, PolyNode::kNoParent, 0 });
            }

            while (!stack.empty())
            {
                const Frame frame = stack.back();
                stack.pop_back();

                if (frame.depth >= maxDepth)
                    return false;

                const auto index = static_cast<std::uint32_t>(tree.nodes.size());

                PolyNode node;
                node.ring = FromClipper(frame.node->Polygon());
                node.parent = frame.parent;
                node.depth = frame.depth;
                node.hole = (frame.depth % 2) != 0;
                tree.nodes.push_back(std::move(node));

                if (frame.parent == PolyNode::kNoParent)
                    tree.roots.push_back(index);
                else
                    tree.nodes[frame.parent].children.push_back(index);

                for (std::size_t i = frame.node->Count(); i-- > 0; )
                {
                    if (const Clipper2Lib::PolyPath64* child = frame.node->Child(i); child != nullptr)
                        stack.push_back({ child, index, frame.depth + 1 });
                }
            }

            return true;
        }

        // Clipper2Lib::PolyPath64::~PolyPath64 does childs_.resize(0), which destroys a
        // chain of unique_ptr children one stack frame per level. That recursion runs at
        // scope exit no matter what CollectTree decided, so capping CollectTree alone
        // only moves the crash: measured, the cap shifts it from ~4950 to ~6450 levels
        // (audit-v5/evidence-v5/04_p0_7_root_cause.txt).
        //
        // Clearing bottom-up fixes it without touching ThirdParty/Clipper2: by the time
        // a node is cleared its children are already childless, so every destructor that
        // does run recurses exactly one level.
        class ScopedPolyTree
        {
        public:
            ScopedPolyTree() = default;
            ScopedPolyTree(const ScopedPolyTree&) = delete;
            ScopedPolyTree& operator=(const ScopedPolyTree&) = delete;

            // A destructor is implicitly noexcept, and the traversal below allocates:
            // a std::bad_alloc escaping here would call std::terminate. Collect what
            // the allocator allows, then clear bottom-up whatever was collected.
            //
            // Descendants are always discovered after their ancestors, so they hold
            // higher indices and reverse iteration still clears children before
            // parents even when `order` holds only a prefix. Whatever was never
            // collected falls back to PolyPath64's own recursive destructor - the
            // behaviour this class exists to avoid, but reachable only when the
            // allocator is already exhausted, and strictly better than terminating.
            ~ScopedPolyTree()
            {
                // A destructor must let nothing escape, so both halves below are guarded
                // separately - and separately on purpose. If the traversal runs out of
                // memory partway, `order` still holds a prefix that is worth clearing;
                // folding the two into one try would throw that work away at the point
                // where it is needed most.
                std::vector<Clipper2Lib::PolyPath64*> order;

                try
                {
                    std::vector<Clipper2Lib::PolyPath64*> stack{ &tree_ };

                    while (!stack.empty())
                    {
                        Clipper2Lib::PolyPath64* node = stack.back();
                        stack.pop_back();
                        order.push_back(node);
                        for (std::size_t i = 0; i < node->Count(); ++i)
                        {
                            if (Clipper2Lib::PolyPath64* child = node->Child(i); child != nullptr)
                                stack.push_back(child);
                        }
                    }
                }
                catch (...)
                {
                    // std::bad_alloc from push_back, or std::length_error at the vector
                    // size limit. Not actionable; the partial clear below still helps.
                }

                try
                {
                    // Descendants are always discovered after their ancestors, so they
                    // hold higher indices and reverse iteration clears children before
                    // parents even when the traversal above was cut short. Anything left
                    // uncollected falls back to PolyPath64's own recursive destructor -
                    // the behaviour this class exists to avoid, but reachable only when
                    // the allocator is already exhausted, and strictly better than
                    // terminating.
                    for (std::size_t i = order.size(); i-- > 0; )
                        order[i]->Clear();
                }
                catch (...)
                {
                    // Clear() is virtual and Clipper2 does not declare its destructor
                    // chain noexcept, so this is guarded even though the implementation
                    // only resizes a vector to zero.
                }
            }

            [[nodiscard]] Clipper2Lib::PolyTree64& Get() noexcept { return tree_; }

        private:
            Clipper2Lib::PolyTree64 tree_;
        };

        class Clipper2Backend final : public IPolygonBackend
        {
        public:
            [[nodiscard]] const char* Name() const noexcept override { return "Clipper2 2.0.1"; }

            [[nodiscard]] BackendStatus Execute(Operation operation, Fill fill,
                                                IntRingsView subject, IntRingsView clip,
                                                const TreeLimits& limits,
                                                PolyTree& out) const override
            {
                out.Clear();

                try
                {
                    Clipper2Lib::Clipper64 clipper;
                    clipper.AddSubject(ToClipper(subject));
                    if (!clip.empty())
                        clipper.AddClip(ToClipper(clip));

                    ScopedPolyTree tree;
                    Clipper2Lib::Paths64 open;

                    if (!clipper.Execute(ToClipper(operation), ToClipper(fill), tree.Get(), open))
                        return BackendStatus::Failed;

                    if (!CollectTree(tree.Get(), out, limits.maxNestingDepth))
                    {
                        out.Clear();
                        return BackendStatus::DepthLimit;
                    }
                    return BackendStatus::Success;
                }
                catch (const Clipper2Lib::Clipper2Exception&)
                {
                    // Range or precision failure inside the engine. Reported as a
                    // failed operation so the kernel can return NumericalFailure; the
                    // exception itself must not escape into caller code.
                    out.Clear();
                    return BackendStatus::Failed;
                }
                catch (const std::bad_alloc&)
                {
                    // Ran out of memory, which is not the same as failing to compute.
                    // This used to escape into caller code from Execute and Offset while
                    // Triangulate caught it and reported a generic failure.
                    out.Clear();
                    return BackendStatus::OutOfMemory;
                }
            }

            [[nodiscard]] BackendStatus ExecuteFlat(Operation operation, Fill fill,
                                                    IntRingsView subject, IntRingsView clip,
                                                    IntRings& out) const override
            {
                out.clear();

                try
                {
                    Clipper2Lib::Clipper64 clipper;
                    clipper.AddSubject(ToClipper(subject));
                    if (!clip.empty())
                        clipper.AddClip(ToClipper(clip));

                    Clipper2Lib::Paths64 closed;
                    Clipper2Lib::Paths64 open;
                    if (!clipper.Execute(ToClipper(operation), ToClipper(fill), closed, open))
                        return BackendStatus::Failed;

                    FromClipper(closed, out);
                    return BackendStatus::Success;
                }
                catch (const Clipper2Lib::Clipper2Exception&)
                {
                    out.clear();
                    return BackendStatus::Failed;
                }
                catch (const std::bad_alloc&)
                {
                    out.clear();
                    return BackendStatus::OutOfMemory;
                }
            }

            [[nodiscard]] BackendStatus Offset(IntRingsView subject, const OffsetRequest& request,
                                               const TreeLimits& limits,
                                               PolyTree& out) const override
            {
                out.Clear();

                try
                {
                    Clipper2Lib::ClipperOffset offsetter;
                    offsetter.MiterLimit(request.miterLimit);
                    if (request.arcTolerance > 0.0)
                        offsetter.ArcTolerance(request.arcTolerance);

                    offsetter.AddPaths(ToClipper(subject), ToClipper(request.join), ToClipper(request.cap));

                    ScopedPolyTree tree;
                    offsetter.Execute(request.delta, tree.Get());

                    if (!CollectTree(tree.Get(), out, limits.maxNestingDepth))
                    {
                        out.Clear();
                        return BackendStatus::DepthLimit;
                    }
                    return BackendStatus::Success;
                }
                catch (const Clipper2Lib::Clipper2Exception&)
                {
                    out.Clear();
                    return BackendStatus::Failed;
                }
                catch (const std::bad_alloc&)
                {
                    out.Clear();
                    return BackendStatus::OutOfMemory;
                }
            }

            [[nodiscard]] TriangulationStatus Triangulate(IntRingsView rings,
                                                          IntTriangles& out) const override
            {
                out.clear();
                if (rings.empty())
                    return TriangulationStatus::Empty;

                try
                {
                    Clipper2Lib::Paths64 solution;
                    // useDelaunay: better-shaped triangles, which keeps the convex
                    // merge from producing slivers that Minkowski would amplify.
                    const auto status =
                        Clipper2Lib::Triangulate(ToClipperWithoutZeroLengthEdges(rings),
                                                 solution, true);

                    switch (status)
                    {
                        case Clipper2Lib::TriangulateResult::success:
                            FromClipperTriangles(solution, out);
                            return out.empty() ? TriangulationStatus::Empty
                                               : TriangulationStatus::Success;
                        case Clipper2Lib::TriangulateResult::no_polygons:
                            return TriangulationStatus::Empty;
                        case Clipper2Lib::TriangulateResult::paths_intersect:
                            return TriangulationStatus::PathsIntersect;
                        case Clipper2Lib::TriangulateResult::fail:
                        default:
                            return TriangulationStatus::Failed;
                    }
                }
                catch (const Clipper2Lib::Clipper2Exception&)
                {
                    out.clear();
                    return TriangulationStatus::Failed;
                }
                catch (const std::bad_alloc&)
                {
                    out.clear();
                    return TriangulationStatus::OutOfMemory;
                }
            }
        };

        const Clipper2Backend kBackend;
    }

    const IPolygonBackend& Default() noexcept { return kBackend; }
}
