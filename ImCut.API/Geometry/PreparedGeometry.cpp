#include "PreparedGeometry.hpp"

#include "GeometryTestHooks.hpp"
#include "Polygon/ConvexHull.hpp"
#include "Polygon/PolygonMetrics.hpp"

#include <array>
#include <cmath>
#include <iterator>

namespace ImCut::Geometry
{
    namespace
    {
        [[nodiscard]] std::uint64_t NextPreparedDefinitionIdentity() noexcept
        {
            static std::atomic<std::uint64_t> next{ 1 };
            return next.fetch_add(1, std::memory_order_relaxed);
        }

        [[nodiscard]] std::size_t ImmutablePathBytes(const Path& path) noexcept
        {
            // The Path object itself moved out of PreparedShapeDefinition when its
            // ownership became shareable, so count it explicitly. Capacity reflects
            // retained allocations more faithfully than size and remains O(contours).
            std::size_t total = sizeof(Path) + path.contours.capacity() * sizeof(Contour);
            for (const Contour& contour : path.contours)
            {
                total += contour.nodes.capacity() * sizeof(Vec2);
                total += contour.handles.capacity() * sizeof(Vec2);
                total += contour.kinds.capacity() * sizeof(SegmentKind);
            }
            return total;
        }
    }

    Contour TransformContour(const Contour& contour, const Transform2& transform)
    {
        Contour result = contour;
        transform.ApplyBatch(contour.nodes.data(), result.nodes.data(), contour.nodes.size());
        transform.ApplyBatch(contour.handles.data(), result.handles.data(), contour.handles.size());
        return result;
    }

    Path TransformPath(const Path& path, const Transform2& transform)
    {
        Path result;
        result.fillRule = path.fillRule;
        result.sourceObject = path.sourceObject;
        result.budget = path.budget;
        result.contours.reserve(path.contours.size());
        for (const Contour& contour : path.contours)
            result.contours.push_back(TransformContour(contour, transform));

        // A mirroring transform reverses every ring's direction, so the outer/hole
        // convention has to be re-established by whoever consumes the result.
        return result;
    }

    void SplitLocalAndPlacement(const Path& path, Path& localOut, Transform2& placementOut)
    {
        const Bounds2 bounds = Metrics::ComputeBounds(path);

        if (bounds.IsEmpty())
        {
            localOut = path;
            placementOut = Transform2::Identity();
            return;
        }

        // Local frame has its minimum corner at the origin; the placement puts it back.
        placementOut = Transform2::Translation(bounds.min);
        localOut = TransformPath(path, Transform2::Translation({ -bounds.min.x, -bounds.min.y }));
    }

    PreparedShapeDefinition::PreparedShapeDefinition()
        : local_(std::make_shared<const Path>()),
          localPathBytes_(ImmutablePathBytes(*local_)),
          identity_(NextPreparedDefinitionIdentity())
    {
        approximateBytes_.store(
            sizeof(PreparedShapeDefinition) + localPathBytes_,
            std::memory_order_relaxed);
    }

    PreparedShapeDefinition::PreparedShapeDefinition(Path localPath, GeometryContext context)
        : local_(std::make_shared<const Path>(std::move(localPath))),
          localPathBytes_(ImmutablePathBytes(*local_)),
          identity_(NextPreparedDefinitionIdentity()),
          context_(std::move(context))
    {
        structurallyValid_ = local_->IsStructurallyValid();
        if (structurallyValid_)
        {
            localBounds_ = Metrics::ComputeBounds(*local_);
            localEnclosure_ = Metrics::ComputeEnclosure(*local_);
            segmentCount_ = local_->SegmentCount();
        }
        approximateBytes_.store(
            sizeof(PreparedShapeDefinition) + localPathBytes_,
            std::memory_order_relaxed);
    }

    PreparedShapeDefinition::PreparedShapeDefinition(Path localPath, GeometryContext context,
                                                     CanonicalPath canonical,
                                                     GeometryFingerprint fingerprint)
        : PreparedShapeDefinition(std::move(localPath), std::move(context))
    {
        // The session already canonicalised and fingerprinted this exact geometry to
        // look it up. Publishing that work here instead of repeating it removes one full
        // canonicalisation and one fingerprint from every cold prepare, and guarantees
        // the definition's identity is bit-identical to the key it was filed under.
        // Empty is a valid geometry status, not a missing identity. Publishing the
        // canonical value for an empty path lets downstream prepared caches preserve
        // Empty cold-to-warm instead of failing Fingerprint() with Unsupported.
        if (structurallyValid_)
        {
            canonical_.result = GeometryResult<CanonicalPath>::Success(std::move(canonical));
            canonical_.Publish();
            fingerprint_.result = GeometryResult<GeometryFingerprint>::Success(fingerprint);
            fingerprint_.Publish();
        }
    }

    bool PreparedShapeDefinition::IsPrepared(PreparationLevel level) const noexcept
    {
        std::lock_guard<std::mutex> lock(mutex_);

        if (level >= PreparationLevel::CollisionReady &&
            !(segments_.Ready() && flattened_.Ready() && hierarchy_.Ready() && hull_.Ready()))
        {
            return false;
        }
        if (level >= PreparationLevel::BooleanReady &&
            !(topology_.Ready() && canonical_.Ready() && fingerprint_.Ready() && netArea_.Ready()))
        {
            return false;
        }
        if (level >= PreparationLevel::Maximum && !decomposition_.Ready())
            return false;

        return true;
    }

    GeometryStatus PreparedShapeDefinition::Prepare(PreparationLevel level)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return PrepareLocked(level);
    }

    GeometryStatus PreparedShapeDefinition::PrepareLocked(PreparationLevel level)
    {
        // Structure first: every derivative below indexes the contour arrays.
        if (!structurallyValid_)
            return GeometryStatus::InvalidInput;

        if (local_->contours.empty())
            return GeometryStatus::Empty;

        // maxSegments, checked before anything sized by the segment count is allocated.
        // The field existed in ComplexityLimits since the first version and was never
        // read, so a path with millions of segments walked straight into the segment
        // array, the BVH and the flattener.
        if (segmentCount_ > context_.Limits().maxSegments)
            return GeometryStatus::ComplexityLimit;

        if (local_->contours.size() > context_.Limits().maxContours)
            return GeometryStatus::ComplexityLimit;

        // A derivative is cached only when it succeeded. Caching a failure would turn a
        // cancelled or budget-limited run into a permanent empty answer for the rest of
        // the definition's life.
        auto failed = [](GeometryStatus status) { return !IsSuccess(status); };

        if (level >= PreparationLevel::CollisionReady)
        {
            if (!segments_.Ready())
            {
                if (context_.IsCancelled())
                    return GeometryStatus::Cancelled;

                std::vector<Segment> segments;
                segments.reserve(segmentCount_);
                for (const Contour& contour : local_->contours)
                {
                    const std::size_t count = contour.SegmentCount();
                    for (std::size_t i = 0; i < count; ++i)
                        segments.push_back(contour.SegmentAt(i));
                }
                segments_.result = GeometryResult<std::vector<Segment>>::Success(std::move(segments));
                segments_.Publish();
            }

            if (!flattened_.Ready())
            {
                // Test-only injection point. Disarmed in every shipped call - one relaxed
                // load on a path that already holds the mutex. It stands in for Flatten
                // mapping std::bad_alloc to OutOfMemory (Flatten.cpp:255), a status no
                // input can provoke and which the accessors below used to lose.
                GeometryStatus injected = GeometryStatus::Success;
                if (Testing::ShouldInjectFailure(Testing::FailurePoint::DefinitionFlatten,
                                                 injected))
                {
                    flattened_.result = GeometryResult<FlattenedPath>::Failure(injected);
                    return injected;
                }

                auto result = Flatten::PathWith(*local_, context_);
                if (failed(result.Status()))
                {
                    flattened_.result = GeometryResult<FlattenedPath>::Failure(result.Status());
                    return result.Status();
                }
                flattened_.result = std::move(result);
                flattened_.Publish();
            }

            if (!hierarchy_.Ready())
            {
                if (context_.IsCancelled())
                    return GeometryStatus::Cancelled;

                const std::vector<Segment>& segments = segments_.result->Value();
                SegmentBVH bvh;
                bvh.BuildFromSegments(segments.data(), segments.size(), context_);
                hierarchy_.result = GeometryResult<SegmentBVH>::Success(std::move(bvh));
                hierarchy_.Publish();
            }

            if (!hull_.Ready())
            {
                std::vector<Vec2> points;
                for (const FlattenedContour& contour : flattened_.result->Value().contours)
                    points.insert(points.end(), contour.points.begin(), contour.points.end());

                std::vector<Vec2> hull;
                Hull::ComputeInto(points.data(), points.size(), hull);
                hull_.result = GeometryResult<std::vector<Vec2>>::Success(std::move(hull));
                hull_.Publish();
            }
        }

        if (level >= PreparationLevel::BooleanReady)
        {
            if (!topology_.Ready())
            {
                auto result = Topology::BuildContainmentTree(*local_, context_);
                if (failed(result.Status()))
                {
                    topology_.result = GeometryResult<ContainmentTree>::Failure(result.Status());
                    return result.Status();
                }
                topology_.result = std::move(result);
                topology_.Publish();
            }

            if (!canonical_.Ready())
            {
                if (context_.IsCancelled())
                    return GeometryStatus::Cancelled;
                canonical_.result =
                    GeometryResult<CanonicalPath>::Success(Canonical::Canonicalize(*local_));
                canonical_.Publish();
            }

            if (!fingerprint_.Ready())
            {
                fingerprint_.result = GeometryResult<GeometryFingerprint>::Success(
                    Fingerprint::Compute(canonical_.result->Value()));
                fingerprint_.Publish();
            }

            if (!netArea_.Ready())
            {
                // Net area is the sum of the FILLED annuli: for every ring whose role is
                // Outer, its own area minus the area of the rings immediately inside it.
                //
                // The previous form, sum of +|area| for Outer and -|area| for Hole, is
                // the same number whenever roles strictly alternate - which is exactly
                // what EvenOdd guarantees, and why it was never wrong before. It is not
                // the same number under NonZero, where a ring nested inside another can
                // also be Outer: five concentric CCW rings would have summed to 22000
                // for a region that is 10000.
                const ContainmentTree& tree = topology_.result->Value();

                std::vector<double> areas(tree.nodes.size(), 0.0);
                for (std::size_t i = 0; i < tree.nodes.size(); ++i)
                {
                    const ContainmentNode& node = tree.nodes[i];
                    if (node.contour < local_->contours.size())
                        areas[i] = std::fabs(Metrics::SignedArea(local_->contours[node.contour]));
                }

                double total = 0.0;
                for (std::size_t i = 0; i < tree.nodes.size(); ++i)
                {
                    const ContainmentNode& node = tree.nodes[i];
                    if (node.role != ContourRole::Outer)
                        continue;

                    double annulus = areas[i];
                    for (const std::uint32_t child : node.children)
                        annulus -= areas[child];
                    total += annulus;
                }

                netArea_.result = GeometryResult<double>::Success(total);
                netArea_.Publish();
            }
        }

        if (level >= PreparationLevel::Maximum && !decomposition_.Ready())
        {
            // Hole-aware decomposition of the REGION, not of its outer rings.
            //
            // This used to walk tree.roots and decompose each outer ring on its own,
            // which filled every hole: an outer 100x100 with a 20x20 hole has a net area
            // of 9600 and the decomposition covered 10000. For collision or nesting
            // against a part that genuinely allows material in its holes, that is not a
            // conservative approximation - it is different geometry.
            //
            // Decompose::ConvexRegion resolves the region through the Boolean backend
            // first, so holes, islands, multiple components and inconsistent winding all
            // come out as the filled area actually is. It already existed; Maximum was
            // simply not using it.
            std::vector<ConvexDecompositionResult> pieces;

            auto result = Decompose::ConvexRegion(*local_, context_);
            if (failed(result.Status()))
            {
                decomposition_.result =
                    GeometryResult<std::vector<ConvexDecompositionResult>>::Failure(result.Status());
                return result.Status();
            }

            if (result.Status() == GeometryStatus::Success)
                pieces.push_back(std::move(result).Value());

            // Empty in, Empty out.
            //
            // GeometryStatus::Empty satisfies IsSuccess, so it slipped past the failure
            // check above, but it is not == Success, so it also skipped the push. What
            // got published was a Success carrying no pieces - and a consumer reading
            // Decomposition().Value() on an empty vector cannot tell "the region is
            // degenerate" from "decomposition was not needed". NFP will consume this.
            using DecompositionResult = GeometryResult<std::vector<ConvexDecompositionResult>>;
            decomposition_.result = pieces.empty()
                ? DecompositionResult::Empty(std::move(pieces))
                : DecompositionResult::Success(std::move(pieces));
            decomposition_.Publish();
        }

        // One sweep per preparation, with the lock already held. Every early return
        // above leaves the previous total in place, which is correct: nothing was
        // published on that path.
        RecomputeBytesLocked();
        return GeometryStatus::Success;
    }

    namespace
    {
        // A reference to a failure result with STATIC lifetime, one per status.
        //
        // Accessors return a reference. For a published derivative that is safe: the
        // value is immutable from publication on and nothing ever reassigns it. For a
        // derivative that FAILED it was not - the failure lived in a
        // std::optional<GeometryResult<T>> that the next Prepare() reassigns, so
        //
        //     if (d->Flattened().Ok()) use(d->Flattened().Value());
        //
        // two separate calls, as the header itself pointed out, could straddle a retry
        // and read a destroyed object. The header asked callers for a discipline the
        // type could not enforce.
        //
        // Handing back a static removes the hazard by construction: the only non-static
        // reference an accessor can return is a published, immutable value. The real
        // status still travels with it, so ComplexityLimit is still distinguishable from
        // Cancelled.
        // The status each slot publishes, in enum order.
        //
        // Success and Empty carry a value and can never be a failure slot; an accessor
        // arriving here with one is a bug, and Unsupported is the honest answer rather
        // than a fabricated empty value.
        //
        // WHY THIS IS A NAMED CONSTANT WITH A static_assert RATHER THAN A LITERAL ARRAY.
        //
        // V8.1 wrote nine entries and clamped out-of-range indices to table[0]. When
        // OutOfMemory was appended to GeometryStatus, its ordinal became 9, the guard
        // 9 < 9 failed, and every accessor reported Unsupported for it - a silent
        // collapse that broke no build and no test, because nothing produced the status
        // yet. Measured across all eight failure statuses, seven survived and only that
        // one did not (evidence 865).
        //
        // The static_assert below ties the table to the last enumerator, so appending a
        // status - or inserting one, which shifts the last ordinal up - stops the build
        // instead of quietly losing the new value. That is the compile-time binding
        // section 37 asks for, and it does not require adding a sentinel to the public
        // enum.
        //
        // GeometrySession.cpp:109-121 solves the same problem with an exhaustive switch
        // and has always handled OutOfMemory correctly. The two are kept consistent
        // deliberately; this one stays a table because it is a template instantiated per
        // derivative type and the statics must be per-T.
        inline constexpr GeometryStatus kFailureSlotStatus[] = {
            GeometryStatus::Unsupported,       // Success  - not a failure; caller bug
            GeometryStatus::Unsupported,       // Empty    - not a failure; caller bug
            GeometryStatus::InvalidInput,
            GeometryStatus::Degenerate,
            GeometryStatus::InvalidTopology,
            GeometryStatus::NumericalFailure,
            GeometryStatus::ComplexityLimit,
            GeometryStatus::Cancelled,
            GeometryStatus::Unsupported,
            GeometryStatus::OutOfMemory,
        };

        static_assert(
            sizeof(kFailureSlotStatus) / sizeof(kFailureSlotStatus[0]) ==
                static_cast<std::size_t>(GeometryStatus::OutOfMemory) + 1,
            "FailureRef must publish a slot for every GeometryStatus. A status was added "
            "to or inserted into the enum without extending kFailureSlotStatus, which is "
            "exactly how OutOfMemory was silently collapsed into Unsupported in V8.1.");

        template <typename T>
        const GeometryResult<T>& FailureRef(GeometryStatus status)
        {
            static const auto table = [] {
                std::array<GeometryResult<T>, std::size(kFailureSlotStatus)> built{};
                for (std::size_t i = 0; i < std::size(kFailureSlotStatus); ++i)
                    built[i] = GeometryResult<T>::Failure(kFailureSlotStatus[i]);
                return built;
            }();

            const auto index = static_cast<std::size_t>(status);
            // Still guarded: the enum could gain a value in a header the build did not
            // recompile. Unsupported remains the honest answer for an unknown status.
            return index < table.size() ? table[index] : table[0];
        }

        // Returned when an accessor is called before the derivative was prepared. The
        // status says "not built" rather than handing back a plausible empty answer.
        template <typename T>
        const GeometryResult<T>& NotPrepared()
        {
            return FailureRef<T>(GeometryStatus::Unsupported);
        }
    }

    const GeometryResult<std::vector<Segment>>& PreparedShapeDefinition::Segments() const
    {
        // Lock-free once published: the value is immutable from that point on and
        // the acquire load pairs with the release store in PrepareLocked.
        if (segments_.Published())
            return *segments_.result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (segments_.result && segments_.result->Ok())
            return *segments_.result;
        const GeometryStatus status =
            segments_.result ? segments_.result->Status() : GeometryStatus::Unsupported;
        return FailureRef<std::vector<Segment>>(status);
    }

    const GeometryResult<FlattenedPath>& PreparedShapeDefinition::Flattened() const
    {
        // Lock-free once published: the value is immutable from that point on and
        // the acquire load pairs with the release store in PrepareLocked.
        if (flattened_.Published())
            return *flattened_.result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (flattened_.result && flattened_.result->Ok())
            return *flattened_.result;
        const GeometryStatus status =
            flattened_.result ? flattened_.result->Status() : GeometryStatus::Unsupported;
        return FailureRef<FlattenedPath>(status);
    }

    const GeometryResult<SegmentBVH>& PreparedShapeDefinition::Hierarchy() const
    {
        // Lock-free once published: the value is immutable from that point on and
        // the acquire load pairs with the release store in PrepareLocked.
        if (hierarchy_.Published())
            return *hierarchy_.result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (hierarchy_.result && hierarchy_.result->Ok())
            return *hierarchy_.result;
        const GeometryStatus status =
            hierarchy_.result ? hierarchy_.result->Status() : GeometryStatus::Unsupported;
        return FailureRef<SegmentBVH>(status);
    }

    const GeometryResult<std::vector<Vec2>>& PreparedShapeDefinition::ConvexHull() const
    {
        // Lock-free once published: the value is immutable from that point on and
        // the acquire load pairs with the release store in PrepareLocked.
        if (hull_.Published())
            return *hull_.result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (hull_.result && hull_.result->Ok())
            return *hull_.result;
        const GeometryStatus status =
            hull_.result ? hull_.result->Status() : GeometryStatus::Unsupported;
        return FailureRef<std::vector<Vec2>>(status);
    }

    const GeometryResult<ContainmentTree>& PreparedShapeDefinition::Topology() const
    {
        // Lock-free once published: the value is immutable from that point on and
        // the acquire load pairs with the release store in PrepareLocked.
        if (topology_.Published())
            return *topology_.result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (topology_.result && topology_.result->Ok())
            return *topology_.result;
        const GeometryStatus status =
            topology_.result ? topology_.result->Status() : GeometryStatus::Unsupported;
        return FailureRef<ContainmentTree>(status);
    }

    const GeometryResult<CanonicalPath>& PreparedShapeDefinition::Canonical() const
    {
        // Lock-free once published: the value is immutable from that point on and
        // the acquire load pairs with the release store in PrepareLocked.
        if (canonical_.Published())
            return *canonical_.result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (canonical_.result && canonical_.result->Ok())
            return *canonical_.result;
        const GeometryStatus status =
            canonical_.result ? canonical_.result->Status() : GeometryStatus::Unsupported;
        return FailureRef<CanonicalPath>(status);
    }

    const GeometryResult<GeometryFingerprint>& PreparedShapeDefinition::Fingerprint() const
    {
        // Lock-free once published: the value is immutable from that point on and
        // the acquire load pairs with the release store in PrepareLocked.
        if (fingerprint_.Published())
            return *fingerprint_.result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (fingerprint_.result && fingerprint_.result->Ok())
            return *fingerprint_.result;
        const GeometryStatus status =
            fingerprint_.result ? fingerprint_.result->Status() : GeometryStatus::Unsupported;
        return FailureRef<GeometryFingerprint>(status);
    }

    const GeometryResult<std::vector<ConvexDecompositionResult>>&
    PreparedShapeDefinition::Decomposition() const
    {
        // Lock-free once published: the value is immutable from that point on and
        // the acquire load pairs with the release store in PrepareLocked.
        if (decomposition_.Published())
            return *decomposition_.result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (decomposition_.result && decomposition_.result->Ok())
            return *decomposition_.result;
        const GeometryStatus status =
            decomposition_.result ? decomposition_.result->Status() : GeometryStatus::Unsupported;
        return FailureRef<std::vector<ConvexDecompositionResult>>(status);
    }

    void PreparedShapeDefinition::RecomputeBytesLocked() noexcept
    {
        // Payload only: allocator overhead, spare capacity and object headers are not
        // counted. See the header for why that is the right direction to be wrong in.
        std::size_t total = sizeof(PreparedShapeDefinition) + localPathBytes_;

        if (segments_.Ready())
            total += segments_.result->Value().size() * sizeof(Segment);

        if (flattened_.Ready())
            for (const FlattenedContour& contour : flattened_.result->Value().contours)
                total += contour.points.size() * sizeof(Vec2);

        if (hierarchy_.Ready())
            total += hierarchy_.result->Value().NodeCount() * sizeof(SegmentBVH::Node) +
                     hierarchy_.result->Value().Order().size() * sizeof(std::uint32_t);

        if (hull_.Ready())
            total += hull_.result->Value().size() * sizeof(Vec2);

        if (topology_.Ready())
            total += topology_.result->Value().nodes.size() * sizeof(ContainmentNode);

        if (canonical_.Ready())
        {
            for (const CanonicalContour& contour : canonical_.result->Value().contours)
            {
                total += contour.coordinates.size() * sizeof(std::int64_t);
                total += contour.handles.size() * sizeof(std::int64_t);
                total += contour.kinds.size() * sizeof(SegmentKind);
            }
        }

        if (decomposition_.Ready())
        {
            for (const ConvexDecompositionResult& region : decomposition_.result->Value())
            {
                total += region.vertices.size() * sizeof(Vec2);
                total += region.triangles.size() * sizeof(Triangle);
                for (const std::vector<Vec2>& piece : region.pieces)
                    total += piece.size() * sizeof(Vec2);
            }
        }

        approximateBytes_.store(total, std::memory_order_relaxed);
    }

    const GeometryResult<double>& PreparedShapeDefinition::NetArea() const
    {
        // Lock-free once published: the value is immutable from that point on and
        // the acquire load pairs with the release store in PrepareLocked.
        if (netArea_.Published())
            return *netArea_.result;

        std::lock_guard<std::mutex> lock(mutex_);
        if (netArea_.result && netArea_.result->Ok())
            return *netArea_.result;
        const GeometryStatus status =
            netArea_.result ? netArea_.result->Status() : GeometryStatus::Unsupported;
        return FailureRef<double>(status);
    }

    GeometryInstance::GeometryInstance(PreparedShapeDefinitionPtr definition, GeometryObjectId id,
                                       const Transform2& toWorld, std::int64_t sourceTag,
                                       const ErrorBudget& provenance)
        : definition_(std::move(definition)), id_(id), toWorld_(toWorld), sourceTag_(sourceTag),
          provenance_(provenance)
    {
        if (definition_)
            // From the ENCLOSURE, not from the tight box. ApplyEnclosure is
            // lemma B1a and it is sound only if what it is given already contains the
            // shape; handed the tight box it produced a world "enclosure" that was
            // short of the geometry by a few ULP.
            worldBounds_ = toWorld_.ApplyEnclosure(definition_->LocalEnclosure().Box());

        // Established here so every query can trust it: a transform with a NaN or
        // infinite coefficient, or bounds that came out non-finite, makes the instance
        // invalid rather than merely unusual.
        finite_ = toWorld_.IsFinite() && ImCut::Geometry::IsFinite(worldBounds_.Box());
    }

    BoundsWitness GeometryInstance::ExtentWitness() const noexcept
    {
        if (!definition_)
            return {};
        return Metrics::ComputeWitness(definition_->LocalPath(), toWorld_);
    }

    Path GeometryInstance::WorldPath() const
    {
        if (!definition_)
            return {};

        Path world = TransformPath(definition_->LocalPath(), toWorld_);
        world.sourceObject = id_;

        // TransformPath already carried LocalPath().budget, which is the INTRINSIC error
        // of representing this shape - zero for a polygon, the flatten deviation for a
        // curve. What it could not carry is the CALLER provenance, because that lives on
        // the instance and not on the shared definition: two placements of one part share
        // a definition and may declare different source quality.
        //
        // Sequential, not Merge: the shape is reconstructed FROM an already-imprecise
        // source, so both displacements happen and they compose. Exactly once, here, at
        // the point the Path becomes visible to a caller - the same discipline F08 and
        // F15 established for the erosion and alias routes.
        //
        // This is not cosmetic. WorldPath() is the operand of the exact Boolean fallback
        // in Overlaps and Contains, so without it that Boolean runs on a Path declaring
        // exact=true over geometry the caller said was 0.5 mm out, and publishes the
        // result (evidence 1016).
        world.budget.MergeSequential(provenance_);
        return world;
    }

    GeometryInstance GeometryInstance::WithTransform(const Transform2& toWorld,
                                                     GeometryObjectId id) const
    {
        // Provenance travels with the placement: moving a part does not make the source
        // it was reconstructed from any more certain.
        return GeometryInstance(definition_, id, toWorld, sourceTag_, provenance_);
    }
}
