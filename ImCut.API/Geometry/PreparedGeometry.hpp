#pragma once

#include "Canonical/CanonicalGeometry.hpp"
#include "Canonical/GeometryFingerprint.hpp"
#include "Curves/Flatten.hpp"
#include "GeometryContext.hpp"
#include "GeometryTypes.hpp"
#include "Polygon/ConvexDecomposition.hpp"
#include "Spatial/SegmentBVH.hpp"
#include "Topology/ContainmentTree.hpp"

#include <memory>
#include <atomic>
#include <mutex>
#include <optional>
#include <vector>

namespace ImCut::Geometry
{
    // How much derived data to materialise. Preparing everything for a piece that only
    // ever gets a bounds test wastes most of the work, so cost is opt-in.
    enum class PreparationLevel : std::uint8_t
    {
        // Bounds only. Always available, computed in the constructor.
        Basic,

        // Adds flattening, the segment hierarchy and the convex hull: enough for
        // collision queries and distance.
        CollisionReady,

        // Adds containment topology and canonical form with fingerprint.
        BooleanReady,

        // Adds convex decomposition. Needed for Minkowski and, later, no-fit polygons.
        Maximum
    };

    // Intrinsic geometry of a shape plus its expensive derivatives, in the shape's own
    // local frame.
    //
    // A definition knows nothing about where it sits in the world. That separation is
    // the point: two placements of the same part share one definition and therefore one
    // flattening, one hierarchy, one decomposition - while each placement keeps its own
    // position. Folding the two together is what made the cache hand back the first
    // instance's world bounds for every later one.
    //
    // Threading: preparation is serialised by the definition's own lock; reads of a
    // PUBLISHED derivative take no lock at all.
    //
    // A derivative that succeeds is never recomputed or replaced, so its storage is
    // immutable for the definition's lifetime. Publication is an atomic release store
    // performed under the lock once the value is final; a reader that sees the acquire
    // load is guaranteed to see the fully constructed value. That is what makes
    // "prepare once, query millions" real: PreparedCollision::Bind touches four
    // accessors per query, and each one used to lock and unlock a mutex.
    //
    // A derivative that FAILED is not published and is recomputed on the next Prepare(),
    // which replaces it - reads of an unpublished derivative therefore still take the
    // lock. Check Ok() and consume the value in the same expression rather than holding
    // a reference to a failed result across a retry.
    class PreparedShapeDefinition
    {
    public:
        PreparedShapeDefinition();
        PreparedShapeDefinition(Path localPath, GeometryContext context);

        // Seeded with an identity the caller has already computed.
        //
        // GeometrySession canonicalises and fingerprints a path to look it up, and on a
        // miss the definition used to canonicalise and fingerprint the very same
        // geometry all over again. Handing the work over costs nothing and removes a
        // full duplicate canonicalisation from every cold prepare.
        PreparedShapeDefinition(Path localPath, GeometryContext context,
                                CanonicalPath canonical, GeometryFingerprint fingerprint);

        [[nodiscard]] const Path& LocalPath() const noexcept { return *local_; }

        // Immutable ownership of the intrinsic path. GeometrySession's NFP identity
        // registry keeps this small payload alive without retaining the definition's
        // much larger prepared derivatives. Returning a const-qualified owner also
        // makes sharing a one-way operation: no cache can mutate the geometry behind
        // another cache's identity.
        [[nodiscard]] const std::shared_ptr<const Path>& LocalPathHandle() const noexcept
        {
            return local_;
        }

        // Bytes owned by LocalPathHandle(), measured once at construction. This lets
        // cache accounting move the shared allocation from the definition cache to the
        // NFP registry if the definition is evicted while NFP still retains the path.
        [[nodiscard]] std::size_t LocalPathApproximateBytes() const noexcept
        {
            return localPathBytes_;
        }
        // Process-unique immutable identity used only as an O(1) cache fast lane.
        // Different identities can still describe equal geometry and must then pass
        // fingerprint-candidate plus exact verification.
        [[nodiscard]] std::uint64_t Identity() const noexcept { return identity_; }
        // TIGHT local extents. Reporting, sizing, comparison.
        [[nodiscard]] const Bounds2& LocalBounds() const noexcept { return localBounds_; }

        // The same extents as EVIDENCE: a box proved to contain the local geometry.
        //
        // F35a. GeometryInstance builds its world BoundsEnclosure from this rather than
        // from LocalBounds(), because BoundsEnclosure::Enclosing() is an assertion and
        // the tight box does not satisfy it: ExactBounds falls short of the true extrema
        // by up to 4 ULP of the coordinate. Every prepared query that proves two shapes
        // DISJOINT from their world boxes was resting on that assertion.
        [[nodiscard]] const BoundsEnclosure& LocalEnclosure() const noexcept
        {
            return localEnclosure_;
        }
        [[nodiscard]] const GeometryContext& Context() const noexcept { return context_; }
        [[nodiscard]] std::size_t SegmentCount() const noexcept { return segmentCount_; }
        [[nodiscard]] bool Empty() const noexcept { return local_->contours.empty(); }

        // Materialises everything the level requires and reports what happened.
        //
        // Not void: preparation runs real geometry that can be cancelled, blow a
        // complexity budget or be handed structurally invalid input, and swallowing that
        // left callers using derivatives that were never actually built.
        [[nodiscard]] GeometryStatus Prepare(PreparationLevel level);

        [[nodiscard]] bool IsPrepared(PreparationLevel level) const noexcept;

        // Accessors return the stored result, including its status. They do not compute:
        // call Prepare() first. An underived accessor reports Unsupported rather than
        // silently producing an empty answer.
        [[nodiscard]] const GeometryResult<std::vector<Segment>>& Segments() const;
        [[nodiscard]] const GeometryResult<FlattenedPath>& Flattened() const;
        [[nodiscard]] const GeometryResult<SegmentBVH>& Hierarchy() const;
        [[nodiscard]] const GeometryResult<std::vector<Vec2>>& ConvexHull() const;
        [[nodiscard]] const GeometryResult<ContainmentTree>& Topology() const;
        [[nodiscard]] const GeometryResult<CanonicalPath>& Canonical() const;
        [[nodiscard]] const GeometryResult<GeometryFingerprint>& Fingerprint() const;
        [[nodiscard]] const GeometryResult<std::vector<ConvexDecompositionResult>>& Decomposition() const;

        // Filled area with holes subtracted, in the local frame.
        [[nodiscard]] const GeometryResult<double>& NetArea() const;

        // Approximate heap bytes this definition holds, counting the derivatives that
        // have actually been prepared. Approximate because it counts payload and not
        // allocator overhead: it is a floor on what is held, which is the safe direction
        // for a ceiling - it can let one extra definition through, but it will never
        // claim to hold more than it does and evict something for bytes that are not
        // there.
        //
        // Exists so GeometrySession can honour ComplexityLimits::maxSessionBytes. A
        // ceiling nothing can measure against is a comment, not a limit.
        //
        // O(1), and deliberately so. The obvious implementation walks the derivatives
        // and sums them, which is O(decomposition pieces) - thousands for a Maximum
        // definition - and the session calls this on every cache HIT. Instead the total
        // is accumulated as each derivative is published, under the lock that already
        // serialises publication, and read here as a relaxed atomic load.
        // True when the session served this definition to a caller whose geometry was
        // not bit-identical to it - the served coordinates are then within one canonical
        // lattice step of what was asked for, never equal to it.
        [[nodiscard]] bool IsShared() const noexcept
        {
            return shared_.load(std::memory_order_relaxed);
        }

        void MarkShared() noexcept { shared_.store(true, std::memory_order_relaxed); }

        [[nodiscard]] std::size_t ApproximateBytes() const noexcept
        {
            return approximateBytes_.load(std::memory_order_relaxed);
        }

    private:
        template <typename T>
        struct Derivative
        {
            std::optional<GeometryResult<T>> result;

            // Set once, under the lock, after `result` holds a successful value that will
            // never be replaced. Readers acquire-load it and skip the lock entirely.
            std::atomic<bool> published{ false };

            [[nodiscard]] bool Ready() const noexcept { return result.has_value() && result->Ok(); }

            [[nodiscard]] bool Published() const noexcept
            {
                return published.load(std::memory_order_acquire);
            }

            void Publish() noexcept { published.store(true, std::memory_order_release); }
        };

        [[nodiscard]] GeometryStatus PrepareLocked(PreparationLevel level);

        std::shared_ptr<const Path> local_;
        std::size_t localPathBytes_ = 0;
        std::uint64_t identity_ = 0;
        GeometryContext context_;
        Bounds2 localBounds_;
        BoundsEnclosure localEnclosure_;
        std::size_t segmentCount_ = 0;
        bool structurallyValid_ = false;

        mutable std::mutex mutex_;

        Derivative<std::vector<Segment>> segments_;
        Derivative<FlattenedPath> flattened_;
        Derivative<SegmentBVH> hierarchy_;
        Derivative<std::vector<Vec2>> hull_;
        Derivative<ContainmentTree> topology_;
        Derivative<CanonicalPath> canonical_;
        Derivative<GeometryFingerprint> fingerprint_;
        Derivative<std::vector<ConvexDecompositionResult>> decomposition_;
        Derivative<double> netArea_;

        // Running total for ApproximateBytes(). Written only under `mutex_`, by
        // RecomputeBytesLocked at the end of PrepareLocked, and read without the lock.
        std::atomic<std::size_t> approximateBytes_{ 0 };
        std::atomic<bool> shared_{ false };

        // Sums what the prepared derivatives hold. Called once per preparation, not per
        // query. Must be called with `mutex_` held.
        void RecomputeBytesLocked() noexcept;
    };

    using PreparedShapeDefinitionPtr = std::shared_ptr<PreparedShapeDefinition>;

    // One placement of a definition in world space.
    //
    // Carries identity, the local-to-world transform and the world bounds. Everything
    // expensive lives in the shared definition; an instance is cheap enough to create
    // per placement, which is exactly the shape of the future nesting workload.
    class GeometryInstance
    {
    public:
        GeometryInstance() = default;
        GeometryInstance(PreparedShapeDefinitionPtr definition, GeometryObjectId id,
                         const Transform2& toWorld, std::int64_t sourceTag = 0,
                         const ErrorBudget& provenance = ErrorBudget{});

        [[nodiscard]] const PreparedShapeDefinitionPtr& Definition() const noexcept { return definition_; }

        // The error the CALLER declared about the geometry it handed in.
        //
        // It lives here, and not on the definition, because a definition is SHARED: two
        // callers with the same shape and different source quality get the same
        // PreparedShapeDefinition, and neither may see the other's error. An instance is
        // the opposite - PrepareInstance mints a fresh one per call and the session never
        // retains it - so it is exactly the query-bound place for query-bound data.
        //
        // Before this existed the session computed the provenance and published it on the
        // RESULT wrapper, where `.Value()` dropped it: the hot APIs take a
        // GeometryInstance, so a caller with lossy input was handed a distance whose
        // budget described only the curve solver. A clearance built from that number is
        // tighter than the geometry justifies, which is the dangerous direction.
        [[nodiscard]] const ErrorBudget& Provenance() const noexcept { return provenance_; }
        [[nodiscard]] GeometryObjectId Id() const noexcept { return id_; }
        [[nodiscard]] std::int64_t SourceTag() const noexcept { return sourceTag_; }

        // Definition-local to world.
        [[nodiscard]] const Transform2& ToWorld() const noexcept { return toWorld_; }

        // Axis-aligned world bounds of this placement, as EVIDENCE and not as a
        // rectangle. BOUNDS_EVIDENCE_PROOF.md.
        //
        // It is `toWorld.ApplyEnclosure(definition->LocalBounds())`: the AABB of a
        // transformed AABB, which under rotation is strictly larger than the shape - a
        // factor of 2 at 45 degrees (1270). That makes it an ENCLOSURE. It over-estimates,
        // so it is sound for "these cannot touch" and for "this is inside that window",
        // and it is NOT sound on the contained side of a containment refutation.
        //
        // Until V8.1.2 this returned a bare `Bounds2` and two call sites used it in that
        // last position - F38 - while a class census read the same accessor and recorded
        // both as safe. The type is what stops that from being a matter of remembering.
        [[nodiscard]] const BoundsEnclosure& Bounds() const noexcept { return worldBounds_; }

        // What the shape provably REACHES, for the other side of the refutation.
        //
        // Costs one pass over the local segments, so it is computed on demand rather than
        // at construction: the cheap queries never ask for it, and `PrepareInstance` is on
        // a measured path. Nothing is cached - a caller asking twice pays twice.
        [[nodiscard]] BoundsWitness ExtentWitness() const noexcept;

        // Finiteness is an invariant of the type, established once at construction.
        //
        // A non-finite placement is not a geometry: it produced infinite world bounds,
        // Success(false) from Intersects and Success(infinity) from MinimumDistance.
        // Checking it per query instead cost ~9 ns on the bounds-reject path - twelve
        // isfinite calls on the cheapest query in the kernel - so the check happens where
        // the instance is built and Valid() carries the answer.
        [[nodiscard]] bool Valid() const noexcept { return definition_ != nullptr && finite_; }

        // True when every transform coefficient and both world-bounds corners are finite.
        [[nodiscard]] bool IsFinite() const noexcept { return finite_; }

        // Materialises the world-space geometry. Deliberately explicit and on demand:
        // the whole design exists so queries can run against the shared local data plus
        // a transform, without ever building this.
        [[nodiscard]] Path WorldPath() const;

        // Same definition, different placement.
        [[nodiscard]] GeometryInstance WithTransform(const Transform2& toWorld,
                                                     GeometryObjectId id) const;

    private:
        PreparedShapeDefinitionPtr definition_;

        // Next to the pointer it is tested with: Valid() reads both on the cheapest
        // query in the kernel, and the bounds-reject path runs millions of times.
        bool finite_ = false;

        GeometryObjectId id_;
        Transform2 toWorld_ = Transform2::Identity();
        BoundsEnclosure worldBounds_;
        std::int64_t sourceTag_ = 0;

        // Query-bound, never shared. See Provenance().
        ErrorBudget provenance_{};
    };

    // Applies a transform to a path, including curve handles.
    [[nodiscard]] Path TransformPath(const Path& path, const Transform2& transform);
    [[nodiscard]] Contour TransformContour(const Contour& contour, const Transform2& transform);

    // Splits a path into the local geometry a definition holds and the translation that
    // places it. The local form has its bounds minimum at the origin, which is what
    // makes two placements of one part share a definition.
    void SplitLocalAndPlacement(const Path& path, Path& localOut, Transform2& placementOut);
}
