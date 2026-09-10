#include "ConvexDecomposition.hpp"

#include "../Math/Predicates.hpp"
#include "PolygonConversion.hpp"
#include "PolygonMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <utility>

namespace ImCut::Geometry::Decompose
{
    namespace
    {
        constexpr std::uint32_t kNone = 0xFFFFFFFFu;

        [[nodiscard]] bool PointInTriangle(Vec2 p, Vec2 a, Vec2 b, Vec2 c) noexcept
        {
            const double d1 = Predicates::Orient2D(a, b, p);
            const double d2 = Predicates::Orient2D(b, c, p);
            const double d3 = Predicates::Orient2D(c, a, p);

            const bool anyNegative = d1 < 0.0 || d2 < 0.0 || d3 < 0.0;
            const bool anyPositive = d1 > 0.0 || d2 > 0.0 || d3 > 0.0;
            return !(anyNegative && anyPositive);
        }

        // ------------------------------------------------------------------
        // Hertel-Mehlhorn over a half-edge mesh.
        //
        // The previous implementation rebuilt a full edge->owner map after every single
        // merge, which is O(P^2 log P) and unusable past a few hundred triangles. Here
        // the mesh is built once; removing a diagonal is a constant number of pointer
        // updates plus two orientation tests, because only the two junction vertices
        // can have turned reflex - every other vertex of both faces keeps the exact
        // neighbourhood it already had.
        // ------------------------------------------------------------------
        struct Mesh
        {
            std::vector<std::uint32_t> origin;
            std::vector<std::uint32_t> next;
            std::vector<std::uint32_t> prev;
            std::vector<std::uint32_t> twin;
            std::vector<bool> alive;

            // Union-find over triangles, so a diagonal whose two sides already belong to
            // the same (possibly hole-encircling) face is never removed.
            std::vector<std::uint32_t> face;

            [[nodiscard]] std::uint32_t Find(std::uint32_t node) noexcept
            {
                while (face[node] != node)
                {
                    face[node] = face[face[node]];
                    node = face[node];
                }
                return node;
            }
        };

        struct EdgeKey
        {
            std::uint64_t key;
            std::uint32_t half;

            [[nodiscard]] bool operator<(const EdgeKey& other) const noexcept
            {
                return key != other.key ? key < other.key : half < other.half;
            }
        };

        void BuildMesh(const std::vector<Triangle>& triangles, Mesh& mesh)
        {
            const std::size_t count = triangles.size() * 3;
            mesh.origin.resize(count);
            mesh.next.resize(count);
            mesh.prev.resize(count);
            mesh.twin.assign(count, kNone);
            mesh.alive.assign(count, true);

            mesh.face.resize(triangles.size());
            for (std::size_t t = 0; t < triangles.size(); ++t)
            {
                mesh.face[t] = static_cast<std::uint32_t>(t);
                const auto base = static_cast<std::uint32_t>(t * 3);
                for (std::uint32_t i = 0; i < 3; ++i)
                {
                    mesh.origin[base + i] = triangles[t][i];
                    mesh.next[base + i] = base + (i + 1) % 3;
                    mesh.prev[base + i] = base + (i + 2) % 3;
                }
            }

            std::vector<EdgeKey> keys;
            keys.reserve(count);
            for (std::uint32_t h = 0; h < count; ++h)
            {
                const std::uint32_t a = mesh.origin[h];
                const std::uint32_t b = mesh.origin[mesh.next[h]];
                const std::uint32_t low = a < b ? a : b;
                const std::uint32_t high = a < b ? b : a;
                keys.push_back({ (static_cast<std::uint64_t>(low) << 32) | high, h });
            }
            std::sort(keys.begin(), keys.end());

            for (std::size_t i = 0; i + 1 < keys.size(); )
            {
                if (keys[i].key != keys[i + 1].key)
                {
                    ++i;
                    continue;
                }

                // Three or more faces on one edge is non-manifold; leaving them all
                // untwinned blocks merging there instead of corrupting the mesh.
                if (i + 2 < keys.size() && keys[i + 2].key == keys[i].key)
                {
                    const std::uint64_t shared = keys[i].key;
                    while (i < keys.size() && keys[i].key == shared)
                        ++i;
                    continue;
                }

                mesh.twin[keys[i].half] = keys[i + 1].half;
                mesh.twin[keys[i + 1].half] = keys[i].half;
                i += 2;
            }
        }

        [[nodiscard]] bool ConvexTurn(const std::vector<Vec2>& vertices, std::uint32_t a,
                                      std::uint32_t b, std::uint32_t c) noexcept
        {
            return Predicates::Orient2D(vertices[a], vertices[b], vertices[c]) >= 0.0;
        }

        [[nodiscard]] bool MergeDiagonal(const std::vector<Vec2>& vertices, Mesh& mesh,
                                         std::uint32_t h)
        {
            if (!mesh.alive[h])
                return false;

            const std::uint32_t t = mesh.twin[h];
            if (t == kNone || !mesh.alive[t] || t < h)
                return false;

            const std::uint32_t rootA = mesh.Find(h / 3);
            const std::uint32_t rootB = mesh.Find(t / 3);
            if (rootA == rootB)
                return false;

            const std::uint32_t prevH = mesh.prev[h];
            const std::uint32_t nextH = mesh.next[h];
            const std::uint32_t prevT = mesh.prev[t];
            const std::uint32_t nextT = mesh.next[t];

            // After removal the merged face runs prevH -> nextT and prevT -> nextH.
            // Those two joins are the only places the boundary changes direction.
            if (!ConvexTurn(vertices, mesh.origin[prevH], mesh.origin[nextT],
                            mesh.origin[mesh.next[nextT]]))
                return false;
            if (!ConvexTurn(vertices, mesh.origin[prevT], mesh.origin[nextH],
                            mesh.origin[mesh.next[nextH]]))
                return false;

            mesh.next[prevH] = nextT;
            mesh.prev[nextT] = prevH;
            mesh.next[prevT] = nextH;
            mesh.prev[nextH] = prevT;

            mesh.alive[h] = false;
            mesh.alive[t] = false;
            mesh.face[rootB] = rootA;
            return true;
        }

        void MergeDiagonals(const std::vector<Vec2>& vertices, Mesh& mesh,
                            DecompositionMergeStrategy strategy)
        {
            if (strategy == DecompositionMergeStrategy::None)
                return;

            const std::uint32_t count = static_cast<std::uint32_t>(mesh.origin.size());
            std::vector<std::uint32_t> order;
            order.reserve(count / 2);

            for (std::uint32_t h = 0; h < count; ++h)
            {
                const std::uint32_t twin = mesh.twin[h];
                if (twin != kNone && h < twin)
                    order.push_back(h);
            }

            // Ascending half-edge index exactly preserves the historical
            // Hertel-Mehlhorn traversal. The length orders are deterministic because
            // the half-edge index is the explicit final tie-breaker.
            if (strategy != DecompositionMergeStrategy::HertelMehlhorn)
            {
                const bool longest =
                    strategy == DecompositionMergeStrategy::LongestSharedEdgeFirst;
                std::sort(order.begin(), order.end(), [&](std::uint32_t a, std::uint32_t b)
                {
                    const double lengthA = DistanceSquared(
                        vertices[mesh.origin[a]], vertices[mesh.origin[mesh.next[a]]]);
                    const double lengthB = DistanceSquared(
                        vertices[mesh.origin[b]], vertices[mesh.origin[mesh.next[b]]]);
                    if (lengthA != lengthB)
                        return longest ? lengthA > lengthB : lengthA < lengthB;
                    return a < b;
                });
            }

            for (const std::uint32_t h : order)
                (void)MergeDiagonal(vertices, mesh, h);
        }

        void CollectFaces(const std::vector<Vec2>& vertices, const Mesh& mesh,
                          std::vector<std::vector<Vec2>>& out)
        {
            const std::uint32_t count = static_cast<std::uint32_t>(mesh.origin.size());
            std::vector<bool> visited(count, false);

            for (std::uint32_t h = 0; h < count; ++h)
            {
                if (!mesh.alive[h] || visited[h])
                    continue;

                std::vector<Vec2> piece;
                std::uint32_t walk = h;
                std::uint32_t guard = 0;
                do
                {
                    visited[walk] = true;
                    piece.push_back(vertices[mesh.origin[walk]]);
                    walk = mesh.next[walk];
                } while (walk != h && ++guard <= count);

                if (piece.size() >= 3)
                    out.push_back(std::move(piece));
            }
        }

        struct PartitionScore
        {
            std::size_t pieces = 0;
            std::size_t vertices = 0;
            double perimeter = 0.0;
        };

        [[nodiscard]] PartitionScore ScorePartition(
            const std::vector<std::vector<Vec2>>& pieces) noexcept
        {
            PartitionScore score;
            score.pieces = pieces.size();
            for (const std::vector<Vec2>& piece : pieces)
            {
                score.vertices += piece.size();
                for (std::size_t i = 0; i < piece.size(); ++i)
                    score.perimeter += std::sqrt(DistanceSquared(
                        piece[i], piece[(i + 1) % piece.size()]));
            }
            return score;
        }

        [[nodiscard]] bool BetterPartition(const PartitionScore& candidate,
                                           const PartitionScore& current) noexcept
        {
            if (candidate.pieces != current.pieces)
                return candidate.pieces < current.pieces;
            if (candidate.vertices != current.vertices)
                return candidate.vertices < current.vertices;
            return candidate.perimeter < current.perimeter;
        }

        [[nodiscard]] GeometryStatus BuildPieces(
            const std::vector<Vec2>& vertices,
            const std::vector<Triangle>& triangles,
            const DecompositionOptions& options,
            std::vector<std::vector<Vec2>>& pieces,
            DecompositionMergeStrategy& applied)
        {
            if (!options.mergeToConvex)
            {
                Mesh mesh;
                BuildMesh(triangles, mesh);
                CollectFaces(vertices, mesh, pieces);
                applied = DecompositionMergeStrategy::None;
                return GeometryStatus::Success;
            }

            auto build = [&](DecompositionMergeStrategy strategy)
            {
                Mesh mesh;
                BuildMesh(triangles, mesh);
                MergeDiagonals(vertices, mesh, strategy);
                std::vector<std::vector<Vec2>> candidate;
                CollectFaces(vertices, mesh, candidate);
                return candidate;
            };

            if (options.mergeStrategy != DecompositionMergeStrategy::SmallNBestOfThree)
            {
                pieces = build(options.mergeStrategy);
                applied = options.mergeStrategy;
                return GeometryStatus::Success;
            }

            if (triangles.size() > options.smallNPortfolioTriangleLimit)
                return GeometryStatus::ComplexityLimit;

            constexpr DecompositionMergeStrategy strategies[] = {
                DecompositionMergeStrategy::HertelMehlhorn,
                DecompositionMergeStrategy::LongestSharedEdgeFirst,
                DecompositionMergeStrategy::ShortestSharedEdgeFirst
            };

            bool haveBest = false;
            PartitionScore bestScore{};
            for (const DecompositionMergeStrategy strategy : strategies)
            {
                std::vector<std::vector<Vec2>> candidate = build(strategy);
                const PartitionScore score = ScorePartition(candidate);
                if (!haveBest || BetterPartition(score, bestScore))
                {
                    haveBest = true;
                    bestScore = score;
                    pieces = std::move(candidate);
                    applied = strategy;
                }
            }
            return GeometryStatus::Success;
        }

        // ------------------------------------------------------------------
        // Sweep backend plumbing.
        // ------------------------------------------------------------------
        class FlatLatticeIndex
        {
        public:
            explicit FlatLatticeIndex(std::size_t maximumEntries)
            {
                std::size_t capacity = 8;
                const std::size_t requested = maximumEntries <=
                    (std::numeric_limits<std::size_t>::max)() / 2
                    ? maximumEntries * 2
                    : (std::numeric_limits<std::size_t>::max)();
                while (capacity < requested)
                {
                    if (capacity > (std::numeric_limits<std::size_t>::max)() / 2)
                        throw std::bad_alloc();
                    capacity *= 2;
                }
                slots_.resize(capacity);
                mask_ = capacity - 1;
            }

            [[nodiscard]] std::uint32_t FindOrInsert(
                const Backend::IntPoint& point, std::uint32_t nextIndex,
                bool& inserted) noexcept
            {
                std::size_t slotIndex = static_cast<std::size_t>(
                    Mix(static_cast<std::uint64_t>(point.x)) ^
                    (Mix(static_cast<std::uint64_t>(point.y)) << 1)) & mask_;
                for (;;)
                {
                    Slot& slot = slots_[slotIndex];
                    if (!slot.occupied)
                    {
                        slot.x = point.x;
                        slot.y = point.y;
                        slot.index = nextIndex;
                        slot.occupied = true;
                        inserted = true;
                        return nextIndex;
                    }
                    if (slot.x == point.x && slot.y == point.y)
                    {
                        inserted = false;
                        return slot.index;
                    }
                    slotIndex = (slotIndex + 1) & mask_;
                }
            }

        private:
            struct Slot
            {
                std::int64_t x = 0;
                std::int64_t y = 0;
                std::uint32_t index = 0;
                bool occupied = false;
            };

            [[nodiscard]] static std::uint64_t Mix(std::uint64_t value) noexcept
            {
                value += 0x9E3779B97F4A7C15ull;
                value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ull;
                value = (value ^ (value >> 27)) * 0x94D049BB133111EBull;
                return value ^ (value >> 31);
            }

            std::vector<Slot> slots_;
            std::size_t mask_ = 0;
        };

        // Builds the union of the region so the triangulator receives non-intersecting,
        // consistently wound rings - the only input it accepts. The rings are kept
        // exactly as the backend emitted them: re-orienting them here would flip the
        // triangulator's own inner/outer test.
        [[nodiscard]] GeometryStatus NormalizeRegion(const Convert::ConversionResult& converted,
                                                     FillRule fillRule,
                                                     const GeometryContext& context,
                                                     Backend::IntRings& out)
        {
            out.clear();

            Backend::PolyTree tree;
            const Backend::Fill fill =
                fillRule == FillRule::NonZero ? Backend::Fill::NonZero : Backend::Fill::EvenOdd;

            const Backend::TreeLimits treeLimits{ context.Limits().maxNestingDepth };
            switch (Backend::Default().Execute(Backend::Operation::Union, fill, converted.rings,
                                               Backend::IntRingsView{}, treeLimits, tree))
            {
                case Backend::BackendStatus::Success:     break;
                case Backend::BackendStatus::DepthLimit:  return GeometryStatus::ComplexityLimit;
                case Backend::BackendStatus::OutOfMemory: return GeometryStatus::OutOfMemory;
                case Backend::BackendStatus::Failed:
                default:                                  return GeometryStatus::NumericalFailure;
            }

            out.reserve(tree.nodes.size());
            for (Backend::PolyNode& node : tree.nodes)
            {
                if (node.ring.size() >= 3)
                    out.push_back(std::move(node.ring));
            }

            return out.empty() ? GeometryStatus::Empty : GeometryStatus::Success;
        }

        [[nodiscard]] GeometryStatus SweepTriangulate(const Backend::IntRings& rings,
                                                      const Convert::LatticeFrame& frame,
                                                      const GeometryContext& context,
                                                      std::vector<Vec2>& vertices,
                                                      std::vector<Triangle>& triangles)
        {
            Backend::IntTriangles raw;
            switch (Backend::Default().Triangulate(rings, raw))
            {
                case Backend::TriangulationStatus::Success:  break;
                case Backend::TriangulationStatus::Empty:    return GeometryStatus::Empty;
                case Backend::TriangulationStatus::PathsIntersect:
                    return GeometryStatus::InvalidTopology;
                case Backend::TriangulationStatus::OutOfMemory:
                    return GeometryStatus::OutOfMemory;
                case Backend::TriangulationStatus::Failed:
                default:                                     return GeometryStatus::NumericalFailure;
            }

            if (raw.size() > context.Limits().maxPolygonVertices)
                return GeometryStatus::ComplexityLimit;

            if (raw.size() > (std::numeric_limits<std::size_t>::max)() / 3)
                return GeometryStatus::ComplexityLimit;
            // Open-addressed exact key table. Hash collisions probe until the full
            // int64 pair matches; new indices retain first-occurrence order, so mesh
            // half-edge numbering and deterministic merge order are unchanged.
            FlatLatticeIndex lookup(raw.size() * 3);

            vertices.clear();
            triangles.clear();
            triangles.reserve(raw.size());

            auto indexOf = [&](const Backend::IntPoint& point) -> std::uint32_t
            {
                bool inserted = false;
                const std::uint32_t index = lookup.FindOrInsert(
                    point, static_cast<std::uint32_t>(vertices.size()), inserted);
                if (inserted)
                    vertices.push_back(frame.ToMillimetres(point.x, point.y));
                return index;
            };

            for (const Backend::IntTriangle& triangle : raw)
            {
                const std::uint32_t a = indexOf(triangle[0]);
                const std::uint32_t b = indexOf(triangle[1]);
                const std::uint32_t c = indexOf(triangle[2]);
                if (a == b || b == c || a == c)
                    continue;

                // Millimetres are y-up; the backend works y-down, so orientation is
                // normalised here rather than trusted.
                const double area = Predicates::Orient2D(vertices[a], vertices[b], vertices[c]);
                if (area == 0.0)
                    continue;

                if (area > 0.0)
                    triangles.push_back({ a, b, c });
                else
                    triangles.push_back({ a, c, b });
            }

            if (context.IsCancelled())
                return GeometryStatus::Cancelled;

            return triangles.empty() ? GeometryStatus::Empty : GeometryStatus::Success;
        }

        [[nodiscard]] Path RingToPath(const std::vector<Vec2>& ring)
        {
            Contour contour;
            contour.MoveTo(ring.front());
            for (std::size_t i = 1; i < ring.size(); ++i)
                contour.LineTo(ring[i]);
            contour.Close(0.0);

            Path path;
            path.fillRule = FillRule::NonZero;
            path.contours.push_back(std::move(contour));
            return path;
        }

        // Douglas-Peucker on a CLOSED ring.
        //
        // The classic algorithm is defined on an open polyline, so the ring is split at
        // its two extreme points - the pair that is furthest apart along the dominant
        // axis - and each half is simplified independently. Splitting at a fixed index
        // instead would let the tolerance behave differently depending on where the ring
        // happened to start, and the canonical form exists precisely so that start node
        // does not matter.
        //
        // Every kept vertex is an input vertex, and no kept vertex moves, so the only
        // error is the perpendicular distance of the DISCARDED vertices - bounded by
        // `tolerance` by construction, which is what makes the budget term honest.
        void SimplifyOpen(const std::vector<Vec2>& points, std::size_t first, std::size_t last,
                          double toleranceSquared, std::vector<bool>& keep)
        {
            if (last <= first + 1)
                return;

            const Vec2 a = points[first];
            const Vec2 b = points[last];
            const Vec2 axis = b - a;
            const double axisLengthSquared = LengthSquared(axis);

            double worst = -1.0;
            std::size_t worstIndex = first;

            for (std::size_t i = first + 1; i < last; ++i)
            {
                const Vec2 d = points[i] - a;
                double distanceSquared;
                if (axisLengthSquared <= 0.0)
                {
                    distanceSquared = LengthSquared(d);
                }
                else
                {
                    const double cross = axis.x * d.y - axis.y * d.x;
                    distanceSquared = (cross * cross) / axisLengthSquared;
                }

                if (distanceSquared > worst)
                {
                    worst = distanceSquared;
                    worstIndex = i;
                }
            }

            if (worst <= toleranceSquared)
                return;

            keep[worstIndex] = true;
            SimplifyOpen(points, first, worstIndex, toleranceSquared, keep);
            SimplifyOpen(points, worstIndex, last, toleranceSquared, keep);
        }

        [[nodiscard]] std::vector<Vec2> SimplifyRing(const std::vector<Vec2>& ring, double tolerance)
        {
            if (tolerance <= 0.0 || ring.size() < 4)
                return ring;

            // Two anchors that cannot both be discarded, chosen by content so the
            // result does not depend on where the ring starts.
            std::size_t lowest = 0;
            for (std::size_t i = 1; i < ring.size(); ++i)
            {
                if (ring[i].y < ring[lowest].y ||
                    (ring[i].y == ring[lowest].y && ring[i].x < ring[lowest].x))
                {
                    lowest = i;
                }
            }

            std::vector<Vec2> rotated;
            rotated.reserve(ring.size() + 1);
            for (std::size_t i = 0; i < ring.size(); ++i)
                rotated.push_back(ring[(lowest + i) % ring.size()]);
            rotated.push_back(rotated.front());

            std::size_t farthest = 0;
            double best = -1.0;
            for (std::size_t i = 1; i + 1 < rotated.size(); ++i)
            {
                const double d = DistanceSquared(rotated.front(), rotated[i]);
                if (d > best) { best = d; farthest = i; }
            }

            std::vector<bool> keep(rotated.size(), false);
            keep.front() = true;
            keep.back() = true;
            keep[farthest] = true;

            const double toleranceSquared = tolerance * tolerance;
            SimplifyOpen(rotated, 0, farthest, toleranceSquared, keep);
            SimplifyOpen(rotated, farthest, rotated.size() - 1, toleranceSquared, keep);

            std::vector<Vec2> simplified;
            simplified.reserve(rotated.size());
            for (std::size_t i = 0; i + 1 < rotated.size(); ++i)
            {
                if (keep[i])
                    simplified.push_back(rotated[i]);
            }

            // A ring that simplifies below three vertices has been destroyed, not
            // simplified. Keeping the original is the conservative answer.
            return simplified.size() >= 3 ? simplified : ring;
        }

        // Records what simplification cost. It does NOT judge `maxPieces`.
        //
        // F24. This function used to end with
        //
        //     if (options.maxPieces != 0 && result.pieces.size() > options.maxPieces)
        //         return GeometryStatus::ComplexityLimit;
        //
        // and that line is the whole of the finding. It fires AFTER the triangulation and
        // the merge have run, so it saves no work at all: every byte and every ear-clip
        // scan the ceiling claims to be protecting has already been spent when it
        // returns. What it does instead is throw away a partition that is complete,
        // convex and EXACT, and label the result with a status that reads as a hard
        // resource contract. Measured: a 96-arm star decomposes into 98 pieces, and the
        // shipping default of 64 turned that into ComplexityLimit (baseline log,
        // [F24a]/[F24b]).
        //
        // The real ceiling on this cost is ComplexityLimits::maxConvexPairs, which is in
        // the unit that actually limits - Minkowski evaluates one exact sum per PAIR -
        // and is checked with overflow-free division BEFORE any pair is enumerated, at
        // all three enumeration sites (Minkowski.cpp:176, Minkowski.cpp:377,
        // NfpCover.cpp:478). Nothing was left unguarded by removing this.
        //
        // `maxPieces` keeps exactly one honest job, and Escalate() below is the ONE place
        // that does it: it is the TARGET a caller who also supplied a simplifyTolerance
        // asks the escalation to reach. Judging it here as well was the second copy of a
        // transversal rule, which is how the two readings - "target" and "refusal" -
        // came to disagree.
        void ApplyPieceBudget(ConvexDecompositionResult& result,
                              const DecompositionOptions& options)
        {
            if (options.simplifyTolerance > 0.0)
                result.budget.AddSequentialFlatten(options.simplifyTolerance);
        }

        [[nodiscard]] GeometryResult<ConvexDecompositionResult> FinishEarClipping(
            const std::vector<Vec2>& ring, const GeometryContext& context,
            const DecompositionOptions& options)
        {
            ConvexDecompositionResult result;
            result.backend = DecompositionBackend::EarClipping;
            result.vertices = SimplifyRing(ring, options.simplifyTolerance);

            auto triangulation = Triangulate(result.vertices, context);
            if (!triangulation.Ok())
                return GeometryResult<ConvexDecompositionResult>::Failure(triangulation.Status());

            result.triangles = triangulation.Value();
            if (result.triangles.empty())
                return GeometryResult<ConvexDecompositionResult>::Empty(std::move(result));

            if (const GeometryStatus pieceStatus = BuildPieces(
                    result.vertices, result.triangles, options, result.pieces,
                    result.appliedMergeStrategy);
                !IsSuccess(pieceStatus))
            {
                return GeometryResult<ConvexDecompositionResult>::Failure(pieceStatus);
            }

            ApplyPieceBudget(result, options);

            return GeometryResult<ConvexDecompositionResult>::Success(std::move(result));
        }
    }

    GeometryResult<std::vector<Triangle>> Triangulate(const std::vector<Vec2>& ring,
                                                      const GeometryContext& context)
    {
        std::vector<Triangle> triangles;

        const std::size_t count = ring.size();
        if (count < 3)
            return GeometryResult<std::vector<Triangle>>::Empty(std::move(triangles));

        if (count > context.Limits().maxPolygonVertices)
            return GeometryResult<std::vector<Triangle>>::Failure(GeometryStatus::ComplexityLimit);

        // Work counter-clockwise so "convex vertex" is simply a left turn.
        std::vector<std::uint32_t> indices(count);
        const bool clockwise = Metrics::SignedArea(ring.data(), count) < 0.0;
        for (std::size_t i = 0; i < count; ++i)
            indices[i] = static_cast<std::uint32_t>(clockwise ? count - 1 - i : i);

        triangles.reserve(count - 2);

        std::size_t remaining = count;
        std::size_t cursor = 0;

        // Without this the loop could spin forever on a degenerate or self-intersecting
        // ring that never presents an ear.
        std::size_t sinceLastEar = 0;

        while (remaining > 3)
        {
            if (context.ShouldCheckCancellation(triangles.size()) && context.IsCancelled())
                return GeometryResult<std::vector<Triangle>>::Failure(GeometryStatus::Cancelled);

            const std::size_t previous = (cursor + remaining - 1) % remaining;
            const std::size_t next = (cursor + 1) % remaining;

            const std::uint32_t ia = indices[previous];
            const std::uint32_t ib = indices[cursor];
            const std::uint32_t ic = indices[next];

            const Vec2 a = ring[ia];
            const Vec2 b = ring[ib];
            const Vec2 c = ring[ic];

            bool isEar = Predicates::Orient2D(a, b, c) > 0.0;

            if (isEar)
            {
                for (std::size_t i = 0; i < remaining; ++i)
                {
                    if (i == previous || i == cursor || i == next)
                        continue;

                    if (PointInTriangle(ring[indices[i]], a, b, c))
                    {
                        isEar = false;
                        break;
                    }
                }
            }

            if (isEar)
            {
                triangles.push_back({ ia, ib, ic });
                indices.erase(indices.begin() + static_cast<std::ptrdiff_t>(cursor));
                --remaining;
                if (cursor >= remaining)
                    cursor = 0;
                sinceLastEar = 0;
                continue;
            }

            cursor = next;
            if (++sinceLastEar > remaining)
                return GeometryResult<std::vector<Triangle>>::Failure(GeometryStatus::InvalidTopology);
        }

        triangles.push_back({ indices[0], indices[1], indices[2] });

        return GeometryResult<std::vector<Triangle>>::Success(std::move(triangles));
    }

    namespace
    {
        // One attempt at the requested tolerance. The escalation lives in the public
        // entry points below so both of them share it.
        [[nodiscard]] GeometryResult<ConvexDecompositionResult> ConvexOnce(
            const std::vector<Vec2>& ring, const GeometryContext& context,
            const DecompositionOptions& options);

        [[nodiscard]] GeometryResult<ConvexDecompositionResult> ConvexRegionOnce(
            const Path& path, const GeometryContext& context,
            const DecompositionOptions& options);

        // Doubles `simplifyTolerance` until the piece ceiling is met.
        //
        // Returning ComplexityLimit when a ceiling is missed is honest but not useful to
        // a no-fit polygon, which needs SOME decomposition it can afford. With a
        // tolerance to work with, the ceiling becomes a guarantee: keep coarsening until
        // it fits, and declare the tolerance that actually did it. Without one there is
        // nothing to escalate, so the refusal stands.
        template <typename Attempt>
        [[nodiscard]] GeometryResult<ConvexDecompositionResult> Escalate(
            const DecompositionOptions& options, Attempt&& attempt)
        {
            if (options.maxPieces == 0 || options.simplifyTolerance <= 0.0)
                return attempt(options);

            DecompositionOptions current = options;
            current.maxPieces = 0;   // let each attempt through, judge it here

            const int attempts = options.maxSimplifyAttempts > 0 ? options.maxSimplifyAttempts : 1;
            for (int i = 0; i < attempts; ++i)
            {
                auto result = attempt(current);
                if (!result.Ok())
                    return result;

                if (result.Value().PieceCount() <= options.maxPieces)
                    return result;

                if (current.simplifyTolerance > kMaxSupportedSimplifyTolerance * 0.5)
                    break;
                current.simplifyTolerance *= 2.0;
            }

            return GeometryResult<ConvexDecompositionResult>::Failure(
                GeometryStatus::ComplexityLimit);
        }
    }

    GeometryResult<ConvexDecompositionResult> Convex(const std::vector<Vec2>& ring,
                                                     const GeometryContext& context,
                                                     const DecompositionOptions& options)
    {
        if (!(options.simplifyTolerance >= 0.0) ||
            !std::isfinite(options.simplifyTolerance) ||
            options.simplifyTolerance > kMaxSupportedSimplifyTolerance ||
            options.mergeStrategy > DecompositionMergeStrategy::SmallNBestOfThree ||
            options.maxSimplifyAttempts < 0)
        {
            return GeometryResult<ConvexDecompositionResult>::Failure(
                GeometryStatus::InvalidInput);
        }
        return Escalate(options, [&](const DecompositionOptions& attempt)
        {
            return ConvexOnce(ring, context, attempt);
        });
    }

    GeometryResult<ConvexDecompositionResult> ConvexRegion(const Path& path,
                                                           const GeometryContext& context,
                                                           const DecompositionOptions& options)
    {
        if (!(options.simplifyTolerance >= 0.0) ||
            !std::isfinite(options.simplifyTolerance) ||
            options.simplifyTolerance > kMaxSupportedSimplifyTolerance ||
            options.mergeStrategy > DecompositionMergeStrategy::SmallNBestOfThree ||
            options.maxSimplifyAttempts < 0)
        {
            return GeometryResult<ConvexDecompositionResult>::Failure(
                GeometryStatus::InvalidInput);
        }
        return Escalate(options, [&](const DecompositionOptions& attempt)
        {
            return ConvexRegionOnce(path, context, attempt);
        });
    }

    namespace
    {
    GeometryResult<ConvexDecompositionResult> ConvexOnce(const std::vector<Vec2>& ring,
                                                         const GeometryContext& context,
                                                         const DecompositionOptions& options)
    {
        if (ring.size() < 3)
        {
            ConvexDecompositionResult empty;
            empty.vertices = ring;
            return GeometryResult<ConvexDecompositionResult>::Empty(std::move(empty));
        }

        if (ring.size() <= options.earClippingLimit)
            return FinishEarClipping(ring, context, options);

        return ConvexRegionOnce(RingToPath(ring), context, options);
    }

    GeometryResult<ConvexDecompositionResult> ConvexRegionOnce(const Path& path,
                                                               const GeometryContext& context,
                                                               const DecompositionOptions& options)
    {
        if (!path.IsStructurallyValid())
            return GeometryResult<ConvexDecompositionResult>::Failure(GeometryStatus::InvalidInput);

        if (path.HasOpenContours())
            return GeometryResult<ConvexDecompositionResult>::Failure(GeometryStatus::InvalidTopology);

        if (path.contours.empty())
            return GeometryResult<ConvexDecompositionResult>::Empty(ConvexDecompositionResult{});

        auto converted = Convert::PathToIntegers(path, context);
        if (!converted.Ok())
            return GeometryResult<ConvexDecompositionResult>::Failure(converted.Status());

        ConvexDecompositionResult result;
        result.budget = converted.Value().budget;
        result.backend = DecompositionBackend::SweepDelaunay;

        Backend::IntRings normalized;
        const GeometryStatus normalizeStatus =
            NormalizeRegion(converted.Value(), path.fillRule, context, normalized);
        if (normalizeStatus == GeometryStatus::Empty)
            return GeometryResult<ConvexDecompositionResult>::Empty(std::move(result));
        if (!IsSuccess(normalizeStatus))
            return GeometryResult<ConvexDecompositionResult>::Failure(normalizeStatus);

        // A single small ring still goes through ear clipping: exact lattice vertices,
        // no sweep set-up cost, and the same convex merge afterwards.
        if (normalized.size() == 1 && normalized.front().size() <= options.earClippingLimit)
        {
            const Convert::LatticeFrame& frame = converted.Value().frame;
            std::vector<Vec2> ring;
            ring.reserve(normalized.front().size());
            for (const Backend::IntPoint& point : normalized.front())
                ring.push_back(frame.ToMillimetres(point.x, point.y));

            auto eared = FinishEarClipping(ring, context, options);
            if (eared.Ok())
            {
                const GeometryStatus earStatus = eared.Status();
                ConvexDecompositionResult small = std::move(eared).Value();

                // MERGE the conversion budget in; do not replace what ear clipping
                // declared. Assigning here dropped the simplification term entirely, so
                // a decomposition coarsened to 25 mm reported 0.01 mm - the budget
                // understating the error, which is exactly what INV-14 forbids. The two
                // stages happen in sequence, so they add.
                const ErrorBudget earBudget = small.budget;
                small.budget = result.budget;
                small.budget.MergeSequential(earBudget);

                return earStatus == GeometryStatus::Empty
                           ? GeometryResult<ConvexDecompositionResult>::Empty(std::move(small))
                           : GeometryResult<ConvexDecompositionResult>::Success(std::move(small));
            }
            // Ear clipping refused this ring; the sweep backend below is the fallback.
        }

        // Simplify on the lattice, before triangulating.
        //
        // The sweep backend produces one triangle per vertex pair, so the piece count
        // is set by the vertex count long before Hertel-Mehlhorn gets to merge
        // anything. Thinning here is what moves the number; thinning afterwards would
        // only be able to merge faces that already exist.
        if (options.simplifyTolerance > 0.0)
        {
            const Convert::LatticeFrame& frame = converted.Value().frame;
            Backend::IntRings thinned;
            thinned.reserve(normalized.size());

            for (const Backend::IntRing& ring : normalized)
            {
                std::vector<Vec2> points;
                points.reserve(ring.size());
                for (const Backend::IntPoint& point : ring)
                    points.push_back(frame.ToMillimetres(point.x, point.y));

                const std::vector<Vec2> simplified =
                    SimplifyRing(points, options.simplifyTolerance);
                if (simplified.size() < 3)
                    continue;

                Backend::IntRing out;
                out.reserve(simplified.size());
                for (const Vec2& point : simplified)
                    out.push_back(frame.ToInteger(point));
                thinned.push_back(std::move(out));
            }

            if (thinned.empty())
                return GeometryResult<ConvexDecompositionResult>::Empty(std::move(result));

            // Re-resolve the region after thinning.
            //
            // Douglas-Peucker moves each ring independently, so two rings that were
            // merely close can end up crossing, and a ring with a tight concavity can
            // cross itself. Measured on the corpus: KTM.svg and CRF 250R came back
            // InvalidTopology from the triangulator at 0.05 and 0.25 mm. Unioning the
            // thinned rings under the same fill rule resolves those intersections the
            // same way the original normalisation did, so the triangulator sees a valid
            // region again. The displacement is still bounded by the tolerance - the
            // union only re-cuts along the rings, it does not move them further.
            Backend::IntRings resolved;
            Backend::PolyTree resolvedTree;
            const Backend::Fill fill = path.fillRule == FillRule::NonZero
                ? Backend::Fill::NonZero : Backend::Fill::EvenOdd;
            const Backend::TreeLimits treeLimits{ context.Limits().maxNestingDepth };

            switch (Backend::Default().Execute(Backend::Operation::Union, fill, thinned,
                                               Backend::IntRingsView{}, treeLimits, resolvedTree))
            {
                case Backend::BackendStatus::Success:
                {
                    resolved.reserve(resolvedTree.nodes.size());
                    for (Backend::PolyNode& node : resolvedTree.nodes)
                    {
                        if (node.ring.size() >= 3)
                            resolved.push_back(std::move(node.ring));
                    }
                    break;
                }
                case Backend::BackendStatus::DepthLimit:
                    return GeometryResult<ConvexDecompositionResult>::Failure(
                        GeometryStatus::ComplexityLimit);
                case Backend::BackendStatus::Failed:
                default:
                    return GeometryResult<ConvexDecompositionResult>::Failure(
                        GeometryStatus::NumericalFailure);
            }

            if (resolved.empty())
                return GeometryResult<ConvexDecompositionResult>::Empty(std::move(result));

            normalized = std::move(resolved);
        }

        const GeometryStatus status = SweepTriangulate(normalized, converted.Value().frame, context,
                                                       result.vertices, result.triangles);
        if (status == GeometryStatus::Empty)
            return GeometryResult<ConvexDecompositionResult>::Empty(std::move(result));
        if (!IsSuccess(status))
            return GeometryResult<ConvexDecompositionResult>::Failure(status);

        if (const GeometryStatus pieceStatus = BuildPieces(
                result.vertices, result.triangles, options, result.pieces,
                result.appliedMergeStrategy);
            !IsSuccess(pieceStatus))
        {
            return GeometryResult<ConvexDecompositionResult>::Failure(pieceStatus);
        }

        ApplyPieceBudget(result, options);

        return GeometryResult<ConvexDecompositionResult>::Success(std::move(result));
    }
    }
}
