#include "GeometrySession.hpp"

#include "Polygon/PolygonMetrics.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace ImCut::Geometry
{
    GeometryObjectId GeometrySnapshot::Add(Path path, std::int64_t sourceTag)
    {
        const auto id = GeometryObjectId(static_cast<std::uint32_t>(objects_.size()));
        path.sourceObject = id;
        objects_.push_back({ id, std::move(path), sourceTag });
        return id;
    }

    const GeometrySnapshot::Object* GeometrySnapshot::Find(GeometryObjectId id) const noexcept
    {
        if (!id.IsValid() || id.Index() >= objects_.size())
            return nullptr;
        return &objects_[id.Index()];
    }

    Bounds2 GeometrySnapshot::Bounds() const
    {
        Bounds2 bounds;
        for (const Object& object : objects_)
            bounds.Add(Metrics::ComputeBounds(object.path));
        return bounds;
    }

    std::size_t GeometrySnapshot::SegmentCount() const noexcept
    {
        std::size_t total = 0;
        for (const Object& object : objects_)
            total += object.path.SegmentCount();
        return total;
    }

    void GeometrySnapshot::Clear() noexcept { objects_.clear(); }

    std::size_t GeometrySession::FlattenKeyHash::operator()(const FlattenKey& key) const noexcept
    {
        std::size_t hash = static_cast<std::size_t>(key.contentHash);
        hash ^= std::hash<double>{}(key.tolerance) + 0x9E3779B97F4A7C15ull + (hash << 6) + (hash >> 2);
        hash ^= std::hash<int>{}(key.maxDepth) + 0x9E3779B97F4A7C15ull + (hash << 6) + (hash >> 2);
        return hash;
    }

    namespace
    {
        [[nodiscard]] std::uint64_t NextGeometrySessionIdentity() noexcept
        {
            static std::atomic<std::uint64_t> next{ 1 };
            return next.fetch_add(1, std::memory_order_relaxed);
        }

        [[nodiscard]] GeometrySessionConfig SessionConfigFrom(
            const GeometryContext& context) noexcept
        {
            GeometrySessionConfig config;
            config.tolerance = context.Tolerance();
            config.precision = context.Precision();
            config.limits = context.Limits();
            return config;
        }

        [[nodiscard]] GeometryContext SessionContextFrom(const GeometrySessionConfig& config)
        {
            GeometryContext context(config.tolerance, config.precision);
            context.SetLimits(config.limits);
            return context;
        }

        [[nodiscard]] GeometryResult<PreparedShapeDefinitionPtr> DefinitionResult(
            PreparedShapeDefinitionPtr definition, GeometryStatus status)
        {
            if (status == GeometryStatus::Empty)
            {
                return GeometryResult<PreparedShapeDefinitionPtr>::Empty(
                    std::move(definition));
            }
            return GeometryResult<PreparedShapeDefinitionPtr>::Success(
                std::move(definition));
        }

        [[nodiscard]] const GeometryResult<FlattenedContour>& FlattenFailure(
            GeometryStatus status) noexcept
        {
            static const GeometryResult<FlattenedContour> invalidInput =
                GeometryResult<FlattenedContour>::Failure(GeometryStatus::InvalidInput);
            static const GeometryResult<FlattenedContour> invalidTopology =
                GeometryResult<FlattenedContour>::Failure(GeometryStatus::InvalidTopology);
            static const GeometryResult<FlattenedContour> degenerate =
                GeometryResult<FlattenedContour>::Failure(GeometryStatus::Degenerate);
            static const GeometryResult<FlattenedContour> limited =
                GeometryResult<FlattenedContour>::Failure(GeometryStatus::ComplexityLimit);
            static const GeometryResult<FlattenedContour> cancelled =
                GeometryResult<FlattenedContour>::Failure(GeometryStatus::Cancelled);
            static const GeometryResult<FlattenedContour> numerical =
                GeometryResult<FlattenedContour>::Failure(GeometryStatus::NumericalFailure);
            static const GeometryResult<FlattenedContour> unsupported =
                GeometryResult<FlattenedContour>::Failure(GeometryStatus::Unsupported);
            static const GeometryResult<FlattenedContour> outOfMemory =
                GeometryResult<FlattenedContour>::Failure(GeometryStatus::OutOfMemory);

            switch (status)
            {
                case GeometryStatus::InvalidTopology: return invalidTopology;
                case GeometryStatus::Degenerate: return degenerate;
                case GeometryStatus::ComplexityLimit: return limited;
                case GeometryStatus::Cancelled: return cancelled;
                case GeometryStatus::NumericalFailure: return numerical;
                case GeometryStatus::Unsupported: return unsupported;
                case GeometryStatus::OutOfMemory: return outOfMemory;
                case GeometryStatus::InvalidInput:
                default: return invalidInput;
            }
        }
    }

    GeometrySession::GeometrySession()
        : GeometrySession(GeometrySessionConfig{})
    {
    }

    GeometrySession::GeometrySession(const GeometrySessionConfig& config)
        : sessionIdentity_(NextGeometrySessionIdentity()), config_(config),
          context_(SessionContextFrom(config_))
    {
    }

    GeometrySession::GeometrySession(const GeometryContext& context)
        : sessionIdentity_(NextGeometrySessionIdentity()),
          config_(SessionConfigFrom(context)), context_(context)
    {
    }

    namespace
    {
        [[nodiscard]] std::uint64_t MixBits(std::uint64_t value) noexcept
        {
            // splitmix64 finaliser: full avalanche in four instructions.
            value += 0x9E3779B97F4A7C15ull;
            value = (value ^ (value >> 30)) * 0xBF58476D1CE4E5B9ull;
            value = (value ^ (value >> 27)) * 0x94D049BB133111EBull;
            return value ^ (value >> 31);
        }

        [[nodiscard]] std::uint64_t MixDouble(std::uint64_t hash, double value) noexcept
        {
            std::uint64_t bits = 0;
            std::memcpy(&bits, &value, sizeof(bits));
            return MixBits(hash ^ MixBits(bits));
        }

        // Bucket key for the flatten cache: O(1) in the contour's size.
        //
        // Deliberately a SAMPLE, not a digest of the whole contour. The bucket is
        // verified with an exact comparison, so a collision costs one extra memcmp and
        // never a wrong answer - which means the hash only has to separate contours
        // well, not prove them equal.
        //
        // That distinction is the whole point. Hashing all the bytes of a 237-segment
        // contour is 11 KB through a serial multiply chain, measured at 2.7 us - as
        // expensive as the canonicalisation it replaced, and still more than the 0.69 us
        // it costs to just flatten the thing again. Sampling the sizes, the closed flag
        // and a dozen spread-out coordinates costs tens of nanoseconds, while the memcmp
        // that actually decides the answer runs at cache-line speed.
        [[nodiscard]] std::uint64_t HashContour(const Contour& contour) noexcept
        {
            std::uint64_t hash = 0xCBF29CE484222325ull;
            hash = MixBits(hash ^ contour.nodes.size());
            hash = MixBits(hash ^ contour.handles.size());
            hash = MixBits(hash ^ contour.kinds.size());
            hash = MixBits(hash ^ (contour.closed ? 0x9E37u : 0x85EBu));

            // Up to twelve nodes spread across the ring, so two contours that share a
            // prefix - a common shape authored from the same corner - still separate.
            constexpr std::size_t kSamples = 12;
            const std::size_t nodeCount = contour.nodes.size();
            if (nodeCount != 0)
            {
                const std::size_t stride = nodeCount > kSamples ? nodeCount / kSamples : 1;
                for (std::size_t i = 0; i < nodeCount; i += stride)
                {
                    hash = MixDouble(hash, contour.nodes[i].x);
                    hash = MixDouble(hash, contour.nodes[i].y);
                }
                hash = MixDouble(hash, contour.nodes.back().x);
                hash = MixDouble(hash, contour.nodes.back().y);
            }

            // Two contours can share every node and differ entirely in their curvature,
            // so the handles have to contribute something.
            const std::size_t handleCount = contour.handles.size();
            if (handleCount != 0)
            {
                const std::size_t stride = handleCount > kSamples ? handleCount / kSamples : 1;
                for (std::size_t i = 0; i < handleCount; i += stride)
                {
                    hash = MixDouble(hash, contour.handles[i].x);
                    hash = MixDouble(hash, contour.handles[i].y);
                }
            }

            return hash;
        }

        // Exact equality of two contours, used to verify a hash hit.
        [[nodiscard]] bool ContoursIdentical(const Contour& a, const Contour& b) noexcept
        {
            return a.closed == b.closed && a.nodes == b.nodes &&
                   a.handles == b.handles && a.kinds == b.kinds;
        }

        // The same hash and the same comparison, but against a translated VIEW of the
        // path rather than a materialised copy of it.
        //
        // PrepareInstance splits a world path into a local path plus a placement, and
        // the split allocates a whole new Path and transforms every node and handle -
        // 711 points for a 237-segment contour. On a cache hit that copy is thrown away
        // immediately. Reading through the offset instead costs nothing and lets the hit
        // path allocate nothing at all.
        //
        // Bit-exactness matters here, because a mismatch would make the fast lane always
        // miss. For a pure translation Transform2::Apply computes 1.0*x + 0.0*y + tx,
        // which for finite coordinates is exactly x + tx, and tx is -origin.x. So
        // x - origin.x is the same double, not merely a close one. A mismatch would
        // still be safe - it falls through to the canonical lookup - but it would cost
        // the speed this exists for.
        [[nodiscard]] std::uint64_t HashContourTranslated(const Contour& contour, Vec2 origin) noexcept
        {
            std::uint64_t hash = 0xCBF29CE484222325ull;
            hash = MixBits(hash ^ contour.nodes.size());
            hash = MixBits(hash ^ contour.handles.size());
            hash = MixBits(hash ^ contour.kinds.size());
            hash = MixBits(hash ^ (contour.closed ? 0x9E37u : 0x85EBu));

            constexpr std::size_t kSamples = 12;
            const std::size_t nodeCount = contour.nodes.size();
            if (nodeCount != 0)
            {
                const std::size_t stride = nodeCount > kSamples ? nodeCount / kSamples : 1;
                for (std::size_t i = 0; i < nodeCount; i += stride)
                {
                    hash = MixDouble(hash, contour.nodes[i].x - origin.x);
                    hash = MixDouble(hash, contour.nodes[i].y - origin.y);
                }
                hash = MixDouble(hash, contour.nodes.back().x - origin.x);
                hash = MixDouble(hash, contour.nodes.back().y - origin.y);
            }

            const std::size_t handleCount = contour.handles.size();
            if (handleCount != 0)
            {
                const std::size_t stride = handleCount > kSamples ? handleCount / kSamples : 1;
                for (std::size_t i = 0; i < handleCount; i += stride)
                {
                    hash = MixDouble(hash, contour.handles[i].x - origin.x);
                    hash = MixDouble(hash, contour.handles[i].y - origin.y);
                }
            }

            return hash;
        }

        [[nodiscard]] std::uint64_t HashPathTranslated(const Path& path, Vec2 origin) noexcept
        {
            std::uint64_t hash = MixBits(0x243F6A8885A308D3ull ^ path.contours.size());
            hash = MixBits(hash ^ static_cast<std::uint64_t>(path.fillRule));
            for (const Contour& contour : path.contours)
                hash = MixBits(hash ^ HashContourTranslated(contour, origin));
            return hash;
        }

        [[nodiscard]] bool PathMatchesTranslated(const Path& stored, const Path& world,
                                                 Vec2 origin) noexcept
        {
            if (stored.fillRule != world.fillRule ||
                stored.contours.size() != world.contours.size())
            {
                return false;
            }

            for (std::size_t c = 0; c < stored.contours.size(); ++c)
            {
                const Contour& a = stored.contours[c];
                const Contour& b = world.contours[c];
                if (a.closed != b.closed || a.nodes.size() != b.nodes.size() ||
                    a.handles.size() != b.handles.size() || a.kinds != b.kinds)
                {
                    return false;
                }

                for (std::size_t i = 0; i < a.nodes.size(); ++i)
                {
                    if (a.nodes[i].x != b.nodes[i].x - origin.x ||
                        a.nodes[i].y != b.nodes[i].y - origin.y)
                    {
                        return false;
                    }
                }
                for (std::size_t i = 0; i < a.handles.size(); ++i)
                {
                    if (a.handles[i].x != b.handles[i].x - origin.x ||
                        a.handles[i].y != b.handles[i].y - origin.y)
                    {
                        return false;
                    }
                }
            }
            return true;
        }

        [[nodiscard]] std::uint64_t HashPath(const Path& path) noexcept
        {
            std::uint64_t hash = MixBits(0x243F6A8885A308D3ull ^ path.contours.size());
            hash = MixBits(hash ^ static_cast<std::uint64_t>(path.fillRule));
            for (const Contour& contour : path.contours)
                hash = MixBits(hash ^ HashContour(contour));
            return hash;
        }

        // Approximate footprint of one cache entry, in bytes.
        //
        // Approximate is enough: it drives an eviction policy, not an allocator. What it
        // must do is track the real thing monotonically, so it counts the two vectors
        // that dominate - the stored source contour and the flattened points.
        [[nodiscard]] std::size_t EntryBytes(const Contour& source,
                                             const GeometryResult<FlattenedContour>& result) noexcept
        {
            std::size_t bytes = sizeof(Contour) + sizeof(GeometryResult<FlattenedContour>);
            bytes += source.nodes.capacity() * sizeof(Vec2);
            bytes += source.handles.capacity() * sizeof(Vec2);
            bytes += source.kinds.capacity() * sizeof(SegmentKind);
            if (result.Ok())
                bytes += result.Value().points.capacity() * sizeof(Vec2);
            return bytes;
        }

        // Coordinate-for-coordinate equality of two paths. Used by
        // PrepareDefinitionExact, where "the same shape" has to mean the same numbers
        // rather than the same canonical lattice cell.
        [[nodiscard]] bool PathsIdentical(const Path& a, const Path& b) noexcept
        {
            if (a.fillRule != b.fillRule || a.contours.size() != b.contours.size())
                return false;

            for (std::size_t i = 0; i < a.contours.size(); ++i)
            {
                const Contour& lhs = a.contours[i];
                const Contour& rhs = b.contours[i];
                if (lhs.closed != rhs.closed || lhs.nodes != rhs.nodes ||
                    lhs.handles != rhs.handles || lhs.kinds != rhs.kinds)
                {
                    return false;
                }
            }
            return true;
        }

        // How far the SERVED geometry actually sits from what the caller asked for, as an
        // upper bound in millimetres. Zero when the two are bit-identical.
        //
        // PathsIdentical answers "are these the same bytes"; this answers "and if not, by
        // how much", which is the number a budget should carry. Charging the width of the
        // matching lattice instead overstates by whatever margin the match had to spare.
        //
        // Measured motivation: a translated copy of the same rectangle differs from the
        // stored definition by ONE ULP in a curve handle - 7.11e-15 mm - because splitting
        // the local frame subtracts the origin from coordinates that were computed with it
        // included. kCanonicalResolution is 1e-4, so that hit was charged 1.4e10 times the
        // error it actually inherited.
        //
        // THE CAP IS LOAD-BEARING. Canonical identity is invariant to start node and
        // winding by design, so an index-wise walk can report a large difference for two
        // paths that are geometrically well inside the lattice. Capping at `resolution`
        // turns that case back into the conservative answer, and the result is a valid
        // upper bound either way:
        //
        //   ordering lines up   index-wise IS the deviation, and it is <= resolution
        //   ordering differs    index-wise > true deviation, so the cap >= true deviation
        //
        // Nothing about the matching lattice changes here. The same definitions are still
        // shared and identity still survives translation, which the header calls
        // deliberate and not to be tightened. Only the DECLARED error becomes accurate.
        [[nodiscard]] double PathAliasDeviation(const Path& served, const Path& requested,
                                                double resolution) noexcept
        {
            if (served.fillRule != requested.fillRule ||
                served.contours.size() != requested.contours.size())
            {
                return resolution;
            }

            double worst = 0.0;
            for (std::size_t i = 0; i < served.contours.size(); ++i)
            {
                const Contour& lhs = served.contours[i];
                const Contour& rhs = requested.contours[i];
                if (lhs.closed != rhs.closed || lhs.kinds != rhs.kinds ||
                    lhs.nodes.size() != rhs.nodes.size() ||
                    lhs.handles.size() != rhs.handles.size())
                {
                    return resolution;
                }

                for (std::size_t n = 0; n < lhs.nodes.size(); ++n)
                {
                    worst = (std::max)(worst, std::fabs(lhs.nodes[n].x - rhs.nodes[n].x));
                    worst = (std::max)(worst, std::fabs(lhs.nodes[n].y - rhs.nodes[n].y));
                }
                for (std::size_t h = 0; h < lhs.handles.size(); ++h)
                {
                    worst = (std::max)(worst,
                                       std::fabs(lhs.handles[h].x - rhs.handles[h].x));
                    worst = (std::max)(worst,
                                       std::fabs(lhs.handles[h].y - rhs.handles[h].y));
                }

                if (worst >= resolution)
                    return resolution;
            }
            return worst;
        }
    }

    GeometryResult<PreparedShapeDefinitionPtr> GeometrySession::PrepareDefinition(
        const Path& localPath, PreparationLevel level, ErrorBudget* budget)
    {
        if (!localPath.IsStructurallyValid())
            return GeometryResult<PreparedShapeDefinitionPtr>::Failure(GeometryStatus::InvalidInput);

        const ErrorBudget sourceProvenance = localPath.budget;
        if (budget != nullptr)
            *budget = sourceProvenance;

        // Fast lane: has this session already seen this exact geometry?
        //
        // Canonicalising costs 4.6 us and was 82% of a PrepareInstance that hit the
        // cache. A nesting run prepares the same part thousands of times, and for that
        // case the canonical form answers a question the raw coordinates already
        // answered. A miss here falls straight through to the canonical lookup, so
        // nothing that used to share a definition stops sharing one - this only skips
        // work, it never changes an answer.
        const std::uint64_t contentHash = HashPath(localPath);
        const auto contentFound = definitionsByContent_.find(contentHash);
        if (contentFound != definitionsByContent_.end())
        {
            for (const PreparedShapeDefinitionPtr& candidate : contentFound->second)
            {
                if (!PathsIdentical(candidate->LocalPath(), localPath))
                    continue;

                ++statistics_.preparedHits;
                const GeometryStatus status = candidate->Prepare(level);
                if (!IsSuccess(status))
                    return GeometryResult<PreparedShapeDefinitionPtr>::Failure(status);

                // Preparing to a higher level grows the definition, so its size is
                // re-measured on the way out rather than trusted from insertion time.
                TouchDefinition(candidate);

                // Bit-identical geometry: nothing was approximated for this caller.
                auto hit = DefinitionResult(candidate, status);
                hit.Budget() = sourceProvenance;
                return hit;
            }
        }

        const CanonicalPath canonical = Canonical::Canonicalize(localPath);
        const GeometryFingerprint fingerprint = Fingerprint::Compute(canonical);

        const auto canonicalFound = definitions_.find(fingerprint);

        // A fingerprint match is a candidate, never a proof. Verifying the canonical
        // form is what makes reuse safe; the bucket exists precisely so a collision
        // degrades to a second comparison rather than to a wrong answer.
        if (canonicalFound != definitions_.end())
        {
            for (const PreparedShapeDefinitionPtr& candidate : canonicalFound->second)
            {
                const GeometryResult<CanonicalPath>& cached = candidate->Canonical();
                if (cached.Ok() && Fingerprint::Verify(cached.Value(), canonical))
                {
                    ++statistics_.preparedHits;
                    const GeometryStatus status = candidate->Prepare(level);
                    if (!IsSuccess(status))
                        return GeometryResult<PreparedShapeDefinitionPtr>::Failure(status);

                TouchDefinition(candidate);
                auto hit = DefinitionResult(candidate, status);
                hit.Budget() = sourceProvenance;

                    // Declared on the RESULT, not only in the out-param: a caller composing
                    // a clearance reads Budget(), where a silent exact=true is worse than a
                    // number. A bit-identical hit aliased nothing and stays exact.
                    const double aliasDeviation = PathAliasDeviation(
                        candidate->LocalPath(), localPath, canonical.resolution);
                    if (aliasDeviation > 0.0)
                    {
                        hit.Budget().AddQuantization(aliasDeviation);
                        candidate->MarkShared();
                    }
                    if (budget != nullptr)
                        *budget = hit.Budget();

                    return hit;
                }
                ++statistics_.verificationRejects;
            }
        }

        ++statistics_.preparedMisses;

        // Hand the identity over instead of making the definition recompute it.
        //
        // The lookup above already canonicalised and fingerprinted this exact geometry.
        // The definition then canonicalised and fingerprinted it a second time, so every
        // cold miss paid for two full canonicalisations of the same path - on the 1.7k
        // segment contours in the reference corpus that is the most expensive part of
        // preparing at all.
        Path intrinsicPath = localPath;
        intrinsicPath.budget = ErrorBudget{};
        auto definition = std::make_shared<PreparedShapeDefinition>(std::move(intrinsicPath), context_,
                                                                    canonical, fingerprint);

        // Only the level the caller asked for.
        //
        // This used to force BooleanReady because the cache is keyed on the canonical
        // form and a definition without one could never be matched again. Seeding the
        // identity removes that reason: the canonical form and fingerprint are present
        // from construction, so Basic and CollisionReady are genuinely opt-in again and
        // a caller that only wants collision no longer pays for the containment tree and
        // the net area.
        const GeometryStatus status = definition->Prepare(level);
        if (!IsSuccess(status))
            return GeometryResult<PreparedShapeDefinitionPtr>::Failure(status);

        definitions_.try_emplace(fingerprint).first->second.push_back(definition);
        definitionsByContent_.try_emplace(contentHash).first->second.push_back(definition);
        TrackDefinition(definition);
        auto result = DefinitionResult(std::move(definition), status);
        result.Budget() = sourceProvenance;
        return result;
    }

    GeometryResult<PreparedShapeDefinitionPtr> GeometrySession::PrepareDefinitionExact(
        const Path& localPath, PreparationLevel level, ErrorBudget* budget)
    {
        if (!localPath.IsStructurallyValid())
            return GeometryResult<PreparedShapeDefinitionPtr>::Failure(GeometryStatus::InvalidInput);

        // Exact controls geometry sharing, not whether the source already carries a
        // declared approximation. Caller provenance stays query-bound on every path.
        const ErrorBudget sourceProvenance = localPath.budget;
        if (budget != nullptr)
            *budget = sourceProvenance;

        // The canonical form still keys the bucket - it is the only cheap way to find
        // candidates - but it does not decide reuse. Two paths inside one lattice cell
        // land in the same bucket and are then separated by comparing coordinates.
        const CanonicalPath canonical = Canonical::Canonicalize(localPath);
        const GeometryFingerprint fingerprint = Fingerprint::Compute(canonical);

        const auto found = definitions_.find(fingerprint);
        if (found != definitions_.end())
        {
            for (const PreparedShapeDefinitionPtr& candidate : found->second)
            {
                if (PathsIdentical(candidate->LocalPath(), localPath))
                {
                    ++statistics_.preparedHits;
                    const GeometryStatus status = candidate->Prepare(level);
                    if (!IsSuccess(status))
                        return GeometryResult<PreparedShapeDefinitionPtr>::Failure(status);
                    TouchDefinition(candidate);
                    auto hit = DefinitionResult(candidate, status);
                    hit.Budget() = sourceProvenance;
                    return hit;
                }
                ++statistics_.verificationRejects;
            }
        }

        ++statistics_.preparedMisses;

        Path intrinsicPath = localPath;
        intrinsicPath.budget = ErrorBudget{};
        auto definition = std::make_shared<PreparedShapeDefinition>(std::move(intrinsicPath), context_,
                                                                    canonical, fingerprint);

        const GeometryStatus status = definition->Prepare(level);
        if (!IsSuccess(status))
            return GeometryResult<PreparedShapeDefinitionPtr>::Failure(status);

        definitions_.try_emplace(fingerprint).first->second.push_back(definition);
        definitionsByContent_.try_emplace(HashPath(localPath)).first->second.push_back(definition);
        TrackDefinition(definition);
        auto result = DefinitionResult(std::move(definition), status);
        result.Budget() = sourceProvenance;
        return result;
    }

    GeometryResult<GeometryInstance> GeometrySession::PrepareInstance(const Path& path,
                                                                      const Transform2& placement,
                                                                      PreparationLevel level,
                                                                      std::int64_t sourceTag)
    {
        if (!path.IsStructurallyValid())
            return GeometryResult<GeometryInstance>::Failure(GeometryStatus::InvalidInput);

        // A non-finite placement is not a geometry, so it never becomes an instance.
        // Checked here, at the boundary, rather than left for a metric query to turn
        // into an infinite distance that looks like a measurement.
        if (!placement.IsFinite())
            return GeometryResult<GeometryInstance>::Failure(GeometryStatus::InvalidInput);
        // Split without materialising, when the definition is already here.
        //
        // SplitLocalAndPlacement allocates a whole new Path and transforms every node
        // and handle into it. On a cache hit that copy is built and immediately thrown
        // away, and it was the largest remaining cost in a PrepareInstance that hits.
        const Bounds2 bounds = Metrics::ComputeBounds(path);
        const bool degenerate = bounds.IsEmpty();
        const Vec2 origin = degenerate ? Vec2{} : bounds.min;
        const Transform2 fromLocal =
            degenerate ? Transform2::Identity() : Transform2::Translation(bounds.min);

        auto makeInstance = [&](PreparedShapeDefinitionPtr definition,
                                GeometryStatus definitionStatus = GeometryStatus::Success,
                                ErrorBudget provenance = ErrorBudget{})
        {
            const Transform2 toWorld = Transform2::Compose(placement, fromLocal);
            const GeometryObjectId id{ nextInstanceId_++ };

            // The provenance goes INTO the instance, not only onto the result wrapper.
            // A caller that writes `.Value()` and passes the instance to a query used to
            // drop it here; the query APIs take a GeometryInstance, so that was the end
            // of it.
            GeometryInstance instance(std::move(definition), id, toWorld, sourceTag,
                                      provenance);
            auto result = definitionStatus == GeometryStatus::Empty
                ? GeometryResult<GeometryInstance>::Empty(std::move(instance))
                : GeometryResult<GeometryInstance>::Success(std::move(instance));
            result.Budget() = provenance;
            return result;
        };

        const std::uint64_t contentHash = HashPathTranslated(path, origin);
        const auto contentFound = definitionsByContent_.find(contentHash);
        if (contentFound != definitionsByContent_.end())
        {
            for (const PreparedShapeDefinitionPtr& candidate : contentFound->second)
            {
                if (!PathMatchesTranslated(candidate->LocalPath(), path, origin))
                    continue;

                ++statistics_.preparedHits;
                const GeometryStatus status = candidate->Prepare(level);
                if (!IsSuccess(status))
                    return GeometryResult<GeometryInstance>::Failure(status);

                return makeInstance(candidate, status, path.budget);
            }
        }

        Path local;
        Transform2 splitPlacement = Transform2::Identity();
        SplitLocalAndPlacement(path, local, splitPlacement);

        auto definition = PrepareDefinition(local, level);
        if (!definition.Ok())
            return GeometryResult<GeometryInstance>::Failure(definition.Status());

        const ErrorBudget declared = definition.Budget();
        const GeometryStatus definitionStatus = definition.Status();
        auto instance = makeInstance(
            std::move(definition).Value(), definitionStatus, declared);
        return instance;
    }

    const GeometryResult<FlattenedContour>& GeometrySession::Flatten(const Contour& contour,
                                                                      const FlattenOptions& options)
    {
        // Exact identity: nothing about position or traversal is normalised away.
        //
        // The payload is a list of absolute points in a specific order, so the key has
        // to identify the exact geometry. The shape fingerprint is invariant to
        // translation, start node and winding BY DESIGN, which is why it must never key
        // an absolute payload - the same square at X=0 and X=100 shared an entry and the
        // second caller was handed the first caller's coordinates.
        //
        // The identity is now a cheap content hash plus an exact comparison, not a
        // canonical form. Canonicalising per lookup copied the contour, allocated node
        // keys and Booth's failure table, and made a cache HIT cost 4x a recompute.
        // Nothing about the identity's strictness changed: comparing raw coordinates is
        // stricter than comparing canonical forms, because the canonical form quantises.
        FlattenKey key;
        key.contentHash = HashContour(contour);
        key.tolerance = options.tolerance;
        key.maxDepth = options.maxDepth;

        const auto found = flattened_.find(key);
        if (found != flattened_.end())
        {
            for (FlattenEntry& entry : found->second)
            {
                if (ContoursIdentical(entry.source, contour))
                {
                    ++statistics_.flattenHits;
                    entry.lastUsed = ++flattenClock_;
                    return entry.result;
                }
                ++statistics_.verificationRejects;
            }
        }

        ++statistics_.flattenMisses;

        // Budget-aware overload: the unlimited one existed only to avoid threading a
        // context through, and it let a pathological curve allocate without bound on
        // exactly this path.
        FlattenedContour flattened;
        const GeometryStatus status = Flatten::ContourInto(contour, options, context_, flattened);

        // A cancelled run is not an answer about this geometry, so it is not cached.
        //
        // Cancellation says something about the CALLER - it asked to stop - not about
        // the contour. Storing it would make the next caller, with a live token and all
        // the time in the world, get Cancelled forever from a cache it never asked to
        // poison. The same reasoning applies to ComplexityLimit under a temporarily
        // tightened budget, so neither is retained.
        //
        // Returned by value through a static, because the contract is a reference and
        // there is no entry to point at.
        if (status != GeometryStatus::Success && status != GeometryStatus::Empty)
            return FlattenFailure(status);

        FlattenEntry entry;
        entry.source = contour;
        entry.result = status == GeometryStatus::Empty
            ? GeometryResult<FlattenedContour>::Empty(std::move(flattened))
            : GeometryResult<FlattenedContour>::Success(std::move(flattened));
        entry.lastUsed = ++flattenClock_;
        entry.bytes = EntryBytes(entry.source, entry.result);

        // Evict FIRST, insert second.
        //
        // The other order does not work, and the first version of this got it wrong:
        // holding a reference to the freshly pushed entry and then pruning can erase an
        // older entry from the SAME bucket, which shifts the vector and leaves that
        // reference pointing at a moved-from slot. The corpus fuzz probe found it as a
        // segmentation fault. Excluding the new entry from the victim search does not
        // help - the entry that gets erased is a different one, and erasing it is what
        // moves the new one.
        //
        // Making room before inserting removes the problem instead of guarding it: at
        // the point the reference is taken, nothing further will touch the container.
        // `bucket` is re-acquired because eviction may have erased that bucket outright.
        // TWO GUARDS REMOVED, BOTH OF WHICH MEANT "GIVE UP AND KEEP EVERYTHING".
        //
        // `ceiling != 0` made the tightest ceiling retain the most, the same inversion
        // EvictDefinitionsTo had. `entry.bytes < ceiling` skipped eviction entirely when
        // ONE entry was larger than the whole ceiling - so an oversized entry did not
        // merely exceed the ceiling itself, it also spared every other entry.
        //
        // Making room for as much as will fit is right in both cases. What cannot be
        // avoided is the entry being inserted: this function returns a reference INTO the
        // cache, so under a ceiling of 0 the least it can retain is the one result it is
        // handing back, evicted on the next call. That is a limit of the API shape and it
        // is declared in V8_1_1_SESSION_MEMORY_CONTRACT.md rather than left as a surprise.
        const std::size_t ceiling = context_.Limits().maxCacheBytes;
        EvictFlattenTo(ceiling > entry.bytes ? ceiling - entry.bytes : 0);

        std::vector<FlattenEntry>& target = flattened_.try_emplace(key).first->second;
        flattenBytes_ += entry.bytes;
        target.push_back(std::move(entry));
        statistics_.flattenBytes = flattenBytes_;
        ++statistics_.flattenEntries;
        statistics_.flattenBuckets = flattened_.size();
        return target.back().result;
    }

    void GeometrySession::TrackDefinition(const PreparedShapeDefinitionPtr& definition)
    {
        if (definition == nullptr)
            return;

        DefinitionRecord record;
        record.definition = definition;
        record.lastUsed = ++definitionClock_;
        record.bytes = definition->ApproximateBytes();

        definitionBytes_ += record.bytes;
        definitionSlots_[definition.get()] = definitionOrder_.size();
        definitionOrder_.push_back(std::move(record));
        statistics_.definitionBytes = definitionBytes_;
        statistics_.definitionEntries = definitionOrder_.size();
        statistics_.definitionCanonicalBuckets = definitions_.size();
        statistics_.definitionContentBuckets = definitionsByContent_.size();

        // Enforced AFTER insertion rather than before, unlike the flatten cache, and the
        // reason is a difference in what the caller is holding. A flatten result is
        // returned by reference into the cache, so evicting around a live reference is
        // the hazard the flatten path is arranged to avoid. A definition is a shared_ptr:
        // dropping the cache's copy of the one just inserted is harmless because the
        // caller's copy keeps it alive. So the ceiling can be applied to the true total,
        // including the new entry, which is what the caller asked it to bound.
        EvictDefinitionsTo(context_.Limits().maxSessionBytes);
    }

    void GeometrySession::TouchDefinition(const PreparedShapeDefinitionPtr& definition)
    {
        if (definition == nullptr)
            return;

        const auto slot = definitionSlots_.find(definition.get());
        if (slot == definitionSlots_.end())
            return;

        DefinitionRecord& record = definitionOrder_[slot->second];
        record.lastUsed = ++definitionClock_;

        // Re-measured because Prepare(level) on a hit can add derivatives, so the size
        // recorded at insertion is a lower bound afterwards. ApproximateBytes is an
        // atomic load, not a walk of the derivatives.
        const std::size_t now = record.definition->ApproximateBytes();
        definitionBytes_ -= (std::min)(definitionBytes_, record.bytes);
        record.bytes = now;
        definitionBytes_ += now;
        statistics_.definitionBytes = definitionBytes_;

        // Preparing a cached definition to a higher level can allocate derivatives.
        // A hit is therefore capable of crossing the session ceiling just like a miss.
        EvictDefinitionsTo(context_.Limits().maxSessionBytes);
    }

    void GeometrySession::EraseDefinitionFromLookups(
        const PreparedShapeDefinitionPtr& definition) noexcept
    {
        auto drop = [&definition](auto& map)
        {
            for (auto it = map.begin(); it != map.end(); )
            {
                auto& bucket = it->second;
                bucket.erase(std::remove(bucket.begin(), bucket.end(), definition),
                             bucket.end());
                it = bucket.empty() ? map.erase(it) : std::next(it);
            }
        };

        drop(definitions_);
        drop(definitionsByContent_);
        statistics_.definitionCanonicalBuckets = definitions_.size();
        statistics_.definitionContentBuckets = definitionsByContent_.size();
    }

    void GeometrySession::EvictDefinitionsTo(std::size_t ceiling)
    {
        // NO ZERO ESCAPE HATCH.
        //
        // V8.1 returned here when the ceiling was 0, so the tightest ceiling a caller
        // could express retained MORE than any other value - everything, for the life of
        // the session. Measured across seven descending ceilings, retention fell
        // monotonically from 2 942 720 bytes at 1 GB to 0 at 1 byte, and then jumped back
        // to 2 942 720 at 0 (evidence 883). One inverted point, at exactly the value a
        // caller reaches for when it wants no cache at all.
        //
        // Every other field of ComplexityLimits treats its value as the ceiling itself
        // with no zero case - maxContours == 0 refuses a path with one contour
        // (Flatten.cpp:277) - so zero means zero here too. See
        // V8_1_1_SESSION_MEMORY_CONTRACT.md.
        //
        // The ceiling bounds what the session RETAINS, never what it can compute:
        // PrepareDefinition still succeeds under a ceiling of 0 and the caller keeps its
        // own shared_ptr. Only the cache's copy goes.
        //
        // Definitions are dropped before flatten entries because they are the larger and
        // the more recomputable of the two - re-preparing costs microseconds and is
        // cached again immediately, whereas thrashing the flatten cache costs the same
        // work on every contour of every subsequent operation.
        while (CacheBytes() > ceiling && !definitionOrder_.empty())
        {
            std::size_t victim = 0;
            for (std::size_t i = 1; i < definitionOrder_.size(); ++i)
                if (definitionOrder_[i].lastUsed < definitionOrder_[victim].lastUsed)
                    victim = i;

            const PreparedShapeDefinitionPtr dropped = definitionOrder_[victim].definition;

            // NFP may share this definition's immutable Path. Move that allocation's
            // accounting to NFP before the definition cache releases its owner; the
            // total then falls only by the derivatives/base object actually released.
            TransferNfpGeometryAccountingFrom(*dropped);
            definitionBytes_ -= (std::min)(definitionBytes_, definitionOrder_[victim].bytes);

            // Swap with the back and repair the moved element's slot, so removal stays
            // O(1) and every surviving index remains valid.
            definitionSlots_.erase(dropped.get());
            const std::size_t last = definitionOrder_.size() - 1u;
            if (victim != last)
            {
                definitionOrder_[victim] = std::move(definitionOrder_[last]);
                definitionSlots_[definitionOrder_[victim].definition.get()] = victim;
            }
            definitionOrder_.pop_back();
            ++statistics_.definitionEvictions;
            statistics_.definitionEntries = definitionOrder_.size();

            // Remove it from both lookup maps. It appears in exactly one bucket of each,
            // but which one is not recorded, so both are swept. This is the cost of
            // keeping the LRU out of the maps, and it is paid only on eviction.
            EraseDefinitionFromLookups(dropped);
        }

        // Still over after every definition is gone: the flatten cache is the remainder,
        // and it is asked to fit what is left. Zero ceiling means the session cannot
        // hold anything, which is a legitimate thing to configure - and which the early
        // return this function used to open with made impossible.
        if (CacheBytes() > ceiling)
            EvictNfpTo(ceiling);

        if (CacheBytes() > ceiling)
        {
            const std::size_t occupied = CacheBytes() - flattenBytes_;
            EvictFlattenTo(ceiling > occupied ? ceiling - occupied : 0);
        }

        // Hand back the empty hash tables too, so a session evicted to nothing accounts
        // for nothing. Without this the ceiling has a floor it can never reach.
        ShrinkEmptyIndexes();

        statistics_.definitionBytes = definitionBytes_;
    }

    void GeometrySession::ShrinkEmptyIndexes() noexcept
    {
        // std::vector default construction and swap are both noexcept, so the vector
        // shrinks cannot fail. std::unordered_map default construction is NOT: an
        // implementation is free to allocate its bucket array there, and a throw inside a
        // noexcept function is std::terminate.
        //
        // This function is noexcept because eviction and ClearCaches are, and neither can
        // usefully react to a failure to hand back an empty hash table. So the map shrinks
        // are guarded and a failure simply leaves the buckets in place - which costs
        // nothing that matters: IndexBytes() already reports zero for a map with no
        // entries, so the accounting reaches zero whether or not the array was released.
        // The vector capacities are the terms that MUST go, and they cannot throw.
        const auto shrinkMap = [](auto& container) noexcept
        {
            if (!container.empty())
                return;
            try
            {
                std::decay_t<decltype(container)>{}.swap(container);
            }
            catch (...)
            {
                // Keeping an empty bucket array is not a correctness problem. Terminating
                // the process to avoid it would be.
            }
        };

        const auto shrinkVector = [](auto& container) noexcept
        {
            if (container.empty())
                std::decay_t<decltype(container)>{}.swap(container);
        };

        // ONE list, shared with IndexBytes() - see ForEachIndexContainer in the header.
        // A container that is shrunk here but not counted there was F23; naming them in
        // two places is what let that happen. const_cast because the walk is declared
        // const for the accounting, which is its other and more common consumer.
        ForEachIndexContainer(
            [&](const auto& map, std::size_t) noexcept
            {
                shrinkMap(const_cast<std::remove_const_t<std::remove_reference_t<decltype(map)>>&>(map));
            },
            [&](const auto& vec, std::size_t) noexcept
            {
                shrinkVector(const_cast<std::remove_const_t<std::remove_reference_t<decltype(vec)>>&>(vec));
            });

        statistics_.definitionCanonicalBuckets = definitions_.size();
        statistics_.definitionContentBuckets = definitionsByContent_.size();
        statistics_.flattenBuckets = flattened_.size();
    }

    void GeometrySession::EvictFlattenTo(std::size_t ceiling)
    {
        if (flattenBytes_ <= ceiling)
        {
            statistics_.flattenBytes = flattenBytes_;
            return;
        }

        // Least-recently-used. Called before the new entry is inserted, so every entry
        // present is a legitimate victim and no reference into the container is live.
        while (flattenBytes_ > ceiling)
        {
            auto victimBucket = flattened_.end();
            std::size_t victimIndex = 0;
            std::uint64_t oldest = 0;
            bool found = false;

            for (auto it = flattened_.begin(); it != flattened_.end(); ++it)
            {
                for (std::size_t i = 0; i < it->second.size(); ++i)
                {
                    const FlattenEntry& entry = it->second[i];
                    if (!found || entry.lastUsed < oldest)
                    {
                        found = true;
                        oldest = entry.lastUsed;
                        victimBucket = it;
                        victimIndex = i;
                    }
                }
            }

            if (!found)
                break;   // the cache is empty and still over the ceiling

            std::vector<FlattenEntry>& entries = victimBucket->second;
            flattenBytes_ -= (std::min)(flattenBytes_, entries[victimIndex].bytes);
            entries.erase(entries.begin() + static_cast<std::ptrdiff_t>(victimIndex));
            ++statistics_.flattenEvictions;
            --statistics_.flattenEntries;

            if (entries.empty())
                flattened_.erase(victimBucket);
        }

        statistics_.flattenBytes = flattenBytes_;
        statistics_.flattenBuckets = flattened_.size();
    }

    namespace
    {
        [[nodiscard]] std::uint64_t Bits(double value) noexcept
        {
            std::uint64_t bits = 0;
            std::memcpy(&bits, &value, sizeof(bits));
            return bits;
        }

        [[nodiscard]] std::size_t PathBytes(const Path& path) noexcept
        {
            std::size_t total = path.contours.capacity() * sizeof(Contour);
            for (const Contour& c : path.contours)
                total += c.nodes.capacity() * sizeof(Vec2) +
                         c.handles.capacity() * sizeof(Vec2) +
                         c.kinds.capacity() * sizeof(SegmentKind);
            return total;
        }

        [[nodiscard]] std::size_t HoleAnalysisBytes(
            const Nfp::PreparedHoleFilterAnalysis& analysis) noexcept
        {
            const ContainmentTree& tree = analysis.tree;
            std::size_t total = sizeof(Nfp::PreparedHoleFilterAnalysis) +
                tree.nodes.capacity() * sizeof(ContainmentNode) +
                tree.roots.capacity() * sizeof(std::uint32_t) +
                tree.rings.capacity() * sizeof(FlattenedContour) +
                tree.interiorPoints.capacity() * sizeof(Vec2) +
                tree.hasInteriorPoint.capacity() / 8 + 1 +
                analysis.exactStrictlyConvexContours.capacity() *
                    sizeof(std::uint8_t);
            for (const ContainmentNode& node : tree.nodes)
                total += node.children.capacity() * sizeof(std::uint32_t);
            for (const FlattenedContour& ring : tree.rings)
                total += ring.points.capacity() * sizeof(Vec2);
            return total;
        }
    }

    GeometryContext GeometrySession::QueryContext(const GeometryQueryControl& control) const
    {
        GeometryContext context = context_;
        if (control.cancellation.CanCancel())
            context.SetCancellation(control.cancellation);
        if (control.statistics != nullptr)
            context.SetStatistics(control.statistics);
        if (control.diagnostics != nullptr)
            context.SetDiagnostics(control.diagnostics);
        return context;
    }

    std::optional<std::size_t> GeometrySession::FindNfpGeometry(
        const PreparedShapeDefinition& definition,
        const GeometryFingerprint& fingerprint)
    {
        // The overwhelmingly common warm lane: two integer comparisons and one vector
        // index. Identity is checked as well as the address so allocator address reuse
        // after an evicted definition can never alias an earlier object.
        if (const auto alias = nfpDefinitionAliases_.find(&definition);
            alias != nfpDefinitionAliases_.end() &&
            alias->second.definitionIdentity == definition.Identity() &&
            alias->second.geometryIndex < nfpGeometries_.size())
        {
            return alias->second.geometryIndex;
        }

        const std::uint64_t candidateBucket =
            fingerprint.low ^
            (fingerprint.high * 0x9E3779B97F4A7C15ull);
        const auto candidates = nfpGeometryCandidates_.find(candidateBucket);
        if (candidates == nfpGeometryCandidates_.end())
            return std::nullopt;

        for (const std::size_t index : candidates->second)
        {
            if (index >= nfpGeometries_.size())
                continue;
            const NfpGeometryRecord& record = nfpGeometries_[index];
            if (record.fingerprint != fingerprint)
                continue;
            if (!record.exactGeometry ||
                !PathsIdentical(*record.exactGeometry, definition.LocalPath()))
            {
                ++statistics_.verificationRejects;
                continue;
            }

            nfpDefinitionAliases_[&definition] = { definition.Identity(), index };
            return index;
        }
        return std::nullopt;
    }

    std::size_t GeometrySession::AddNfpGeometry(
        const PreparedShapeDefinition& definition,
        const GeometryFingerprint& fingerprint)
    {
        NfpGeometryRecord record;
        record.geometryIdentity = nextNfpGeometryIdentity_++;
        record.fingerprint = fingerprint;
        record.exactGeometry = definition.LocalPathHandle();
        // Prepared definitions already store intrinsic geometry with a zero budget;
        // caller provenance belongs to NfpQueryOperand and is composed on delivery.
        auto features = Nfp::AnalyzeOperandFeatures(*record.exactGeometry, context_);
        record.featureStatus = features.Status();
        if (features.Ok())
            record.features = std::move(features).Value();
        record.exactGeometryBytes = definition.LocalPathApproximateBytes();

        const auto definitionSlot = definitionSlots_.find(&definition);
        const bool accountedByThisSessionDefinition =
            definitionSlot != definitionSlots_.end() &&
            definitionSlot->second < definitionOrder_.size() &&
            definitionOrder_[definitionSlot->second].definition.get() == &definition;
        if (accountedByThisSessionDefinition)
        {
            record.accountingDefinition = &definition;
            record.accountingDefinitionIdentity = definition.Identity();
        }
        else
        {
            record.exactGeometryBytesCharged = true;
        }

        record.bytes = sizeof(NfpGeometryRecord) +
            (record.exactGeometryBytesCharged ? record.exactGeometryBytes : 0u);
        const std::size_t identityBytes = record.bytes;
        const std::size_t chargedExactGeometryBytes =
            record.exactGeometryBytesCharged ? record.exactGeometryBytes : 0u;

        const std::size_t index = nfpGeometries_.size();
        nfpBytes_ += record.bytes;
        nfpGeometries_.push_back(std::move(record));

        const std::uint64_t candidateBucket =
            fingerprint.low ^ (fingerprint.high * 0x9E3779B97F4A7C15ull);
        nfpGeometryCandidates_[candidateBucket].push_back(index);
        nfpDefinitionAliases_[&definition] = { definition.Identity(), index };
        statistics_.nfpBytes = nfpBytes_;
        statistics_.nfpIdentityBytes += identityBytes;
        statistics_.nfpExactGeometryBytes += chargedExactGeometryBytes;
        ++statistics_.nfpIdentityEntries;
        return index;
    }

    void GeometrySession::TransferNfpGeometryAccountingFrom(
        const PreparedShapeDefinition& definition) noexcept
    {
        for (NfpGeometryRecord& record : nfpGeometries_)
        {
            if (record.exactGeometryBytesCharged ||
                record.accountingDefinition != &definition ||
                record.accountingDefinitionIdentity != definition.Identity())
            {
                continue;
            }

            record.accountingDefinition = nullptr;
            record.accountingDefinitionIdentity = 0;
            record.exactGeometryBytesCharged = true;
            record.bytes += record.exactGeometryBytes;
            nfpBytes_ += record.exactGeometryBytes;
            statistics_.nfpIdentityBytes += record.exactGeometryBytes;
            statistics_.nfpExactGeometryBytes += record.exactGeometryBytes;
        }
        statistics_.nfpBytes = nfpBytes_;
    }

    GeometryStatus GeometrySession::EnsureNfpHoleAnalysis(
        std::size_t geometryIndex,
        const GeometryContext& queryContext)
    {
        if (geometryIndex >= nfpGeometries_.size())
            return GeometryStatus::InvalidInput;
        NfpGeometryRecord& record = nfpGeometries_[geometryIndex];
        if (record.holeAnalysis)
            return GeometryStatus::Success;
        if (record.holeAnalysisStatus != GeometryStatus::Unsupported)
            return record.holeAnalysisStatus;

        auto prepared = Nfp::PrepareHoleFilterAnalysis(
            *record.exactGeometry, queryContext);
        record.holeAnalysisStatus = prepared.Status();
        if (!prepared.Ok())
            return prepared.Status();

        auto analysis = std::make_shared<const Nfp::PreparedHoleFilterAnalysis>(
            std::move(prepared).Value());
        const std::size_t bytes = HoleAnalysisBytes(*analysis);
        record.holeAnalysis = std::move(analysis);
        record.bytes += bytes;
        nfpBytes_ += bytes;
        statistics_.nfpBytes = nfpBytes_;
        statistics_.nfpIdentityBytes += bytes;
        return GeometryStatus::Success;
    }

    GeometryResult<Nfp::NfpResult> GeometrySession::ComposeNfpResult(
        const GeometryResult<Nfp::NfpResult>& intrinsic,
        const ErrorBudget& provenanceA, const ErrorBudget& provenanceB) const
    {
        if (!intrinsic.Ok())
            return intrinsic;

        GeometryResult<Nfp::NfpResult> composed = intrinsic;
        composed.Value().budget.MergeSequential(provenanceA);
        composed.Value().budget.MergeSequential(provenanceB);
        composed.Value().region.budget = composed.Value().budget;
        composed.Budget() = composed.Value().budget;
        return composed;
    }

    namespace
    {
        // THE single place caller-visible NFP metadata is composed.
        //
        // Two routes publish an NFP - the value API and the immutable handle - and V8.1
        // composed the budget in both while leaving `exact` and `certification` at their
        // intrinsic values. A caller with lossy input was handed an object that said
        // "budget not exact" and "answer exact, Certified" at the same time.
        //
        // The rule is not invented here: NfpCover::ComposeQueryProvenance
        // (NfpCover.cpp:612-629) has done exactly this for the Cover all along. This is
        // that rule, applied where it was missing, in one function so the routes cannot
        // drift apart again.
        //
        //   exact(caller)   = intrinsic.exact AND finalBudget.exact
        //   Certified + !exact -> BoundedApproximation
        //
        // CERTIFICATION IS NOT DERIVED FROM EXACT. A non-convex cover can be legitimately
        // Certified with exact=false: the two come from different conditions - `exact`
        // needs strictly convex operands with no simplify and no clearance, while
        // certification asks whether the COVER was certified. Only the downgrade above is
        // implied, and ConservativeSuperset / BoundedApproximation / CandidateOnly are
        // left exactly as the algorithm reported them.
        void ComposeNfpQueryMetadata(ErrorBudget& budget, bool& exact,
                                     GeometryCertification& certification,
                                     const ErrorBudget& provenanceA,
                                     const ErrorBudget& provenanceB) noexcept
        {
            budget.MergeSequential(provenanceA);
            budget.MergeSequential(provenanceB);

            exact = exact && budget.exact;

            if (!budget.exact && certification == GeometryCertification::Certified)
                certification = GeometryCertification::BoundedApproximation;
        }
    }

    GeometryResult<Nfp::NfpResultHandle> GeometrySession::ComposeNfpHandle(
        GeometryStatus status,
        std::shared_ptr<const Nfp::NfpResult> intrinsic,
        const ErrorBudget& provenanceA, const ErrorBudget& provenanceB) const
    {
        if (!IsSuccess(status) || intrinsic == nullptr)
        {
            return GeometryResult<Nfp::NfpResultHandle>::Failure(
                IsSuccess(status) ? GeometryStatus::NumericalFailure : status);
        }

        ErrorBudget budget = intrinsic->budget;
        bool exact = intrinsic->exact;
        GeometryCertification certification = intrinsic->certification;
        ComposeNfpQueryMetadata(budget, exact, certification, provenanceA, provenanceB);

        // The payload stays untouched and shared; the caller-visible metadata rides on
        // the handle.
        Nfp::NfpResultHandle handle(std::move(intrinsic), budget, exact, certification);
        auto result = status == GeometryStatus::Empty
            ? GeometryResult<Nfp::NfpResultHandle>::Empty(std::move(handle))
            : GeometryResult<Nfp::NfpResultHandle>::Success(std::move(handle));
        result.Budget() = budget;
        return result;
    }

    GeometryResult<Nfp::NfpResult> GeometrySession::ComposeStoredNfpResult(
        GeometryStatus status, const Nfp::NfpResult& intrinsic,
        const ErrorBudget& provenanceA, const ErrorBudget& provenanceB) const
    {
        if (!IsSuccess(status))
            return GeometryResult<Nfp::NfpResult>::Failure(status);

        Nfp::NfpResult copy = intrinsic;
        ComposeNfpQueryMetadata(copy.budget, copy.exact, copy.certification,
                                provenanceA, provenanceB);
        copy.region.budget = copy.budget;
        auto result = status == GeometryStatus::Empty
            ? GeometryResult<Nfp::NfpResult>::Empty(std::move(copy))
            : GeometryResult<Nfp::NfpResult>::Success(std::move(copy));
        result.Budget() = result.Value().budget;
        return result;
    }

    GeometryResult<Nfp::PreparedNfpOperandPtr> GeometrySession::PrepareNfpOperand(
        const PreparedShapeDefinition& definition,
        double rotationRadians,
        bool reflected,
        const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        if (!std::isfinite(rotationRadians) || !(options.simplifyTolerance >= 0.0) ||
            !std::isfinite(options.simplifyTolerance) ||
            options.simplifyTolerance > kMaxSupportedSimplifyTolerance ||
            options.routingPolicy > Nfp::NfpRoutingPolicy::Forced ||
            options.coverMode > Nfp::NfpCoverMode::Product ||
            options.decompositionMergeStrategy >
                DecompositionMergeStrategy::SmallNBestOfThree)
        {
            return GeometryResult<Nfp::PreparedNfpOperandPtr>::Failure(
                GeometryStatus::InvalidInput);
        }

        const GeometryContext queryContext = QueryContext(control);

        const GeometryResult<GeometryFingerprint>& fingerprint = definition.Fingerprint();
        if (!fingerprint.Ok())
            return GeometryResult<Nfp::PreparedNfpOperandPtr>::Failure(fingerprint.Status());

        const std::optional<std::size_t> found =
            FindNfpGeometry(definition, fingerprint.Value());
        const std::size_t geometryIndex = found.has_value()
            ? *found : AddNfpGeometry(definition, fingerprint.Value());

        return PrepareNfpOperandResolved(
            definition, nfpGeometries_[geometryIndex].geometryIdentity, rotationRadians,
            reflected, options, queryContext);
    }

    GeometryResult<Nfp::PreparedNfpOperandPtr>
        GeometrySession::PrepareNfpOperandResolved(
            const PreparedShapeDefinition& definition,
            std::uint64_t geometryIdentity,
            double rotationRadians,
            bool reflected,
            const Nfp::NfpOptions& options,
            const GeometryContext& queryContext,
            const Nfp::NfpOperandFeatures* knownFeatures)
    {
        // Takes the IDENTITY, not an index. An index into nfpGeometries_ does not
        // survive the EvictNfpTo at the end of this very function, so the second of two
        // consecutive calls used to index a vector the first call had already cleared.

        NfpPreparedKey key;
        key.geometryIdentity = geometryIdentity;
        key.rotationBits = Bits(rotationRadians);
        key.simplifyBits = Bits(options.simplifyTolerance);
        key.maxPieces = options.maxPieces;
        key.decompositionMergeStrategy = options.decompositionMergeStrategy;
        key.reflected = reflected;

        const std::uint64_t bucket = key.geometryIdentity ^
            (key.rotationBits * 0xD6E8FEB86659FD93ull) ^
            (key.simplifyBits * 0x9E3779B97F4A7C15ull) ^
            (key.maxPieces * 31ull) ^
            (static_cast<std::uint64_t>(key.decompositionMergeStrategy) *
                0x94D049BB133111EBull) ^
            (reflected ? 1ull : 0ull);
        if (const auto found = nfpPrepared_.find(bucket); found != nfpPrepared_.end())
        {
            for (NfpPreparedEntry& entry : found->second)
            {
                if (!(entry.key == key))
                    continue;
                ++statistics_.nfpPreparedHits;
                CountStat(queryContext, &GeometryStatistics::operandPrepareHits);
                entry.lastUsed = ++nfpClock_;
                auto hit = entry.status == GeometryStatus::Empty
                    ? GeometryResult<Nfp::PreparedNfpOperandPtr>::Empty(entry.operand)
                    : GeometryResult<Nfp::PreparedNfpOperandPtr>::Success(entry.operand);
                hit.Budget().Merge(entry.operand->IntrinsicBudget());
                return hit;
            }
        }

        ++statistics_.nfpPreparedMisses;
        CountStat(queryContext, &GeometryStatistics::operandPrepareMisses);
        auto built = Nfp::PreparedNfpOperand::Build(
            definition, sessionIdentity_, geometryIdentity, rotationRadians,
            reflected, config_.nfpLattice, queryContext, options, knownFeatures);
        if (!built.Ok())
            return GeometryResult<Nfp::PreparedNfpOperandPtr>::Failure(built.Status());

        const GeometryStatus builtStatus = built.Status();
        auto operand = std::make_shared<const Nfp::PreparedNfpOperand>(
            std::move(built).Value());
        NfpPreparedEntry entry;
        entry.key = key;
        entry.operand = operand;
        entry.status = builtStatus;
        entry.lastUsed = ++nfpClock_;
        entry.bytes = sizeof(NfpPreparedEntry) + operand->ApproximateBytes();
        nfpBytes_ += entry.bytes;
        nfpPrepared_[bucket].push_back(std::move(entry));
        ++statistics_.nfpPreparedEntries;
        statistics_.nfpPreparedBuckets = nfpPrepared_.size();
        statistics_.nfpBytes = nfpBytes_;

        EvictNfpTo(context_.Limits().maxSessionBytes);
        auto result = builtStatus == GeometryStatus::Empty
            ? GeometryResult<Nfp::PreparedNfpOperandPtr>::Empty(std::move(operand))
            : GeometryResult<Nfp::PreparedNfpOperandPtr>::Success(std::move(operand));
        result.Budget().Merge(result.Value()->IntrinsicBudget());
        return result;
    }

    GeometryResult<NfpPreparedQueryOperand> GeometrySession::PrepareNfpOperand(
        const NfpQueryOperand& definition,
        double rotationRadians,
        bool reflected,
        const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        if (definition.definition == nullptr)
        {
            return GeometryResult<NfpPreparedQueryOperand>::Failure(
                GeometryStatus::InvalidInput);
        }

        auto intrinsic = PrepareNfpOperand(
            *definition.definition, rotationRadians, reflected, options, control);
        if (!intrinsic.Ok())
        {
            return GeometryResult<NfpPreparedQueryOperand>::Failure(
                intrinsic.Status());
        }

        const GeometryStatus status = intrinsic.Status();
        ErrorBudget declared = intrinsic.Budget();
        declared.MergeSequential(definition.provenance);
        NfpPreparedQueryOperand query(
            std::move(intrinsic).Value(), definition.provenance);
        auto result = status == GeometryStatus::Empty
            ? GeometryResult<NfpPreparedQueryOperand>::Empty(std::move(query))
            : GeometryResult<NfpPreparedQueryOperand>::Success(std::move(query));
        result.Budget() = declared;
        return result;
    }

    GeometryResult<Nfp::NfpResult> GeometrySession::Nfp(
        const Nfp::PreparedNfpOperand& stationary,
        const Nfp::PreparedNfpOperand& reflectedMoving,
        const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        const GeometryContext queryContext = QueryContext(control);
        return ComputePreparedNfp(
            stationary, reflectedMoving, options, queryContext);
    }

    GeometryResult<Nfp::NfpResult> GeometrySession::ComputePreparedNfp(
        const Nfp::PreparedNfpOperand& stationary,
        const Nfp::PreparedNfpOperand& reflectedMoving,
        const Nfp::NfpOptions& options,
        const GeometryContext& queryContext,
        const Nfp::NfpExecutionPlan* plan)
    {
        if (stationary.OwnerSessionIdentity() != sessionIdentity_ ||
            reflectedMoving.OwnerSessionIdentity() != sessionIdentity_ ||
            stationary.Reflected() || !reflectedMoving.Reflected() ||
            !(stationary.Profile() == reflectedMoving.Profile()) ||
            stationary.Profile().decomposition.simplifyTolerance !=
                options.simplifyTolerance ||
            stationary.Profile().decomposition.maxPieces != options.maxPieces ||
            stationary.Profile().decomposition.mergeStrategy !=
                options.decompositionMergeStrategy)
        {
            return GeometryResult<Nfp::NfpResult>::Failure(GeometryStatus::InvalidInput);
        }

        Nfp::NfpExecutionPlan forced;
        forced.algorithm = stationary.StrictlyConvex() &&
                           reflectedMoving.StrictlyConvex()
            ? Nfp::NfpAlgorithm::ConvexEdgeMerge
            : Nfp::NfpAlgorithm::ConvexDecomposition;
        forced.reason = Nfp::NfpSelectionReason::Forced;
        forced.decompositionStrategy = options.decompositionMergeStrategy;
        return Nfp::ExecutePreparedPlan(
            stationary, reflectedMoving, queryContext, options,
            plan == nullptr ? forced : *plan);
    }

    GeometryResult<Nfp::NfpResult> GeometrySession::Nfp(
        const NfpPreparedQueryOperand& stationary,
        const NfpPreparedQueryOperand& reflectedMoving,
        const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        if (stationary.operand == nullptr || reflectedMoving.operand == nullptr)
            return GeometryResult<Nfp::NfpResult>::Failure(GeometryStatus::InvalidInput);
        const auto intrinsic = Nfp(
            *stationary.operand, *reflectedMoving.operand, options, control);
        return ComposeNfpResult(
            intrinsic, stationary.provenance, reflectedMoving.provenance);
    }

    GeometryResult<Nfp::NfpCover> GeometrySession::NfpCover(
        const Nfp::PreparedNfpOperand& stationary,
        const Nfp::PreparedNfpOperand& reflectedMoving,
        const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        if (stationary.OwnerSessionIdentity() != sessionIdentity_ ||
            reflectedMoving.OwnerSessionIdentity() != sessionIdentity_)
        {
            return GeometryResult<Nfp::NfpCover>::Failure(
                GeometryStatus::InvalidInput);
        }
        return Nfp::ComputeCover(
            stationary, reflectedMoving, QueryContext(control), options);
    }

    GeometryResult<Nfp::NfpCover> GeometrySession::NfpCover(
        const NfpPreparedQueryOperand& stationary,
        const NfpPreparedQueryOperand& reflectedMoving,
        const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        if (stationary.operand == nullptr || reflectedMoving.operand == nullptr)
            return GeometryResult<Nfp::NfpCover>::Failure(GeometryStatus::InvalidInput);
        auto result = NfpCover(
            *stationary.operand, *reflectedMoving.operand, options, control);
        return Nfp::ComposeQueryProvenance(
            std::move(result), stationary.provenance, reflectedMoving.provenance);
    }

    GeometryResult<Nfp::NfpContactCandidates> GeometrySession::NfpContacts(
        const Nfp::PreparedNfpOperand& stationary,
        const Nfp::PreparedNfpOperand& reflectedMoving,
        const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        if (stationary.OwnerSessionIdentity() != sessionIdentity_ ||
            reflectedMoving.OwnerSessionIdentity() != sessionIdentity_)
        {
            return GeometryResult<Nfp::NfpContactCandidates>::Failure(
                GeometryStatus::InvalidInput);
        }
        return Nfp::ComputeContactCandidates(
            stationary, reflectedMoving, QueryContext(control), options);
    }

    GeometryResult<Nfp::NfpContactCandidates> GeometrySession::NfpContacts(
        const NfpPreparedQueryOperand& stationary,
        const NfpPreparedQueryOperand& reflectedMoving,
        const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        if (stationary.operand == nullptr || reflectedMoving.operand == nullptr)
        {
            return GeometryResult<Nfp::NfpContactCandidates>::Failure(
                GeometryStatus::InvalidInput);
        }
        auto result = NfpContacts(
            *stationary.operand, *reflectedMoving.operand, options, control);
        return Nfp::ComposeQueryProvenance(
            std::move(result), stationary.provenance, reflectedMoving.provenance);
    }

    GeometrySession::NfpCachedPayload GeometrySession::NfpCachedCore(
        const NfpQueryOperand& a, const NfpQueryOperand& b,
        double rotationRadians, const Nfp::NfpOptions& options,
        const GeometryQueryControl& control, bool innerFit,
        bool retainPayload)
    {
        if (a.definition == nullptr || b.definition == nullptr)
            return { GeometryStatus::InvalidInput };

        const GeometryContext queryContext = QueryContext(control);
        ScopedGeometryTotal totalTimer(queryContext.Diagnostics());
        {
            ScopedGeometryStage validationStage(
                queryContext.Diagnostics(), GeometryDiagnosticStage::ValidationExclusive);
            // Validate the complete query before registering or preparing either
            // operand. A rejected moving rotation must not leave a stationary
            // identity/variant behind in the session cache.
            if (!std::isfinite(rotationRadians) ||
                !(options.clearance >= 0.0) || !std::isfinite(options.clearance) ||
                !(options.simplifyTolerance >= 0.0) ||
                !std::isfinite(options.simplifyTolerance) ||
                options.simplifyTolerance > kMaxSupportedSimplifyTolerance ||
                options.routingPolicy > Nfp::NfpRoutingPolicy::Forced ||
                options.coverMode > Nfp::NfpCoverMode::Product ||
                options.decompositionMergeStrategy >
                    DecompositionMergeStrategy::SmallNBestOfThree)
            {
                return { GeometryStatus::InvalidInput };
            }
        }

        const GeometryResult<GeometryFingerprint>& fa = a.definition->Fingerprint();
        const GeometryResult<GeometryFingerprint>& fb = b.definition->Fingerprint();
        std::optional<std::size_t> geometryA = fa.Ok()
            ? FindNfpGeometry(*a.definition, fa.Value()) : std::nullopt;
        std::optional<std::size_t> geometryB = fb.Ok()
            ? FindNfpGeometry(*b.definition, fb.Value()) : std::nullopt;

        Nfp::NfpExecutionPlan executionPlan;
        executionPlan.reason = Nfp::NfpSelectionReason::Forced;
        executionPlan.decompositionStrategy = options.decompositionMergeStrategy;
        const bool autoRouting = !innerFit &&
            options.routingPolicy == Nfp::NfpRoutingPolicy::Auto &&
            options.decompositionMergeStrategy ==
                DecompositionMergeStrategy::HertelMehlhorn;

        if (autoRouting)
        {
            if (!fa.Ok())
                return { fa.Status() };
            if (!fb.Ok())
                return { fb.Status() };
            if (!geometryA.has_value())
                geometryA = AddNfpGeometry(*a.definition, fa.Value());
            if (!geometryB.has_value())
            {
                geometryB = FindNfpGeometry(*b.definition, fb.Value());
                if (!geometryB.has_value())
                    geometryB = AddNfpGeometry(*b.definition, fb.Value());
            }

            const NfpGeometryRecord& recordA = nfpGeometries_[*geometryA];
            const NfpGeometryRecord& recordB = nfpGeometries_[*geometryB];
            if (!IsSuccess(recordA.featureStatus))
                return { recordA.featureStatus };
            if (!IsSuccess(recordB.featureStatus))
                return { recordB.featureStatus };

            executionPlan = Nfp::ResolveCertifiedAutoExecutionPlan(
                recordA.features, recordB.features);
        }

        Nfp::ProvenHoleFilterPlan holeFilterPlan;
        std::optional<Transform2> holeMovingTransform;
        if (autoRouting && options.allowProvenHoleFilter &&
            (nfpGeometries_[*geometryA].features.holes != 0 ||
             nfpGeometries_[*geometryB].features.holes != 0))
        {
            const GeometryStatus analysisA = EnsureNfpHoleAnalysis(
                *geometryA, queryContext);
            if (!IsSuccess(analysisA))
                return { analysisA };
            const GeometryStatus analysisB = EnsureNfpHoleAnalysis(
                *geometryB, queryContext);
            if (!IsSuccess(analysisB))
                return { analysisB };

            const Transform2 stationaryTransform = Transform2::Identity();
            holeMovingTransform.emplace(rotationRadians == 0.0
                ? Transform2::Identity()
                : Transform2::Rotation(rotationRadians));
            holeFilterPlan = Nfp::PlanProvenIrrelevantHoles(
                *nfpGeometries_[*geometryA].holeAnalysis,
                stationaryTransform,
                *nfpGeometries_[*geometryB].holeAnalysis,
                *holeMovingTransform,
                queryContext);
            if (holeFilterPlan.Applied() &&
                holeFilterPlan.stationary.strictlyConvexAfterFilter &&
                holeFilterPlan.moving.strictlyConvexAfterFilter)
            {
                executionPlan.algorithm = Nfp::NfpAlgorithm::ConvexEdgeMerge;
                executionPlan.reason =
                    Nfp::NfpSelectionReason::ProvenHoleFilterMadePairConvex;
                executionPlan.decompositionStrategy =
                    DecompositionMergeStrategy::HertelMehlhorn;
            }
        }

        auto makeKey = [&](std::uint64_t identityA, std::uint64_t identityB)
        {
            NfpKey key;
            key.geometryA = identityA;
            key.geometryB = identityB;
            key.rotationBits = Bits(rotationRadians);
            key.clearanceBits = Bits(options.clearance);
            key.toleranceBits = Bits(options.simplifyTolerance);
            key.maxPieces = options.maxPieces;
            key.resolvedPlanIdentity =
                (static_cast<std::uint64_t>(executionPlan.policyVersion) << 32) |
                (static_cast<std::uint64_t>(executionPlan.algorithm) << 16) |
                (static_cast<std::uint64_t>(
                    executionPlan.decompositionStrategy) << 8) |
                (holeFilterPlan.Applied() ? 1ull : 0ull);
            key.innerFit = innerFit;
            return key;
        };
        auto keyBucket = [](const NfpKey& key) noexcept
        {
            return key.geometryA ^ (key.geometryB * 0xBF58476D1CE4E5B9ull) ^
                   (key.rotationBits * 0xD6E8FEB86659FD93ull) ^ key.clearanceBits ^
                   (key.toleranceBits * 3ull) ^ (key.maxPieces * 31ull) ^
                   (key.resolvedPlanIdentity * 0x9FB21C651E98DF25ull) ^
                   (key.innerFit ? 1ull : 0ull);
        };

        if (geometryA.has_value() && geometryB.has_value())
        {
            const NfpKey key = makeKey(
                nfpGeometries_[*geometryA].geometryIdentity,
                nfpGeometries_[*geometryB].geometryIdentity);
            const auto found = nfp_.find(keyBucket(key));
            if (found != nfp_.end())
            {
                for (NfpEntry& entry : found->second)
                {
                    if (!(entry.key == key))
                        continue;
                    ++statistics_.nfpHits;
                    CountStat(queryContext, &GeometryStatistics::cacheHits);
                    entry.lastUsed = ++nfpClock_;
                    return {
                        entry.status,
                        entry.result.get(),
                        retainPayload ? entry.result : nullptr
                    };
                }
            }
        }

        ++statistics_.nfpMisses;
        CountStat(queryContext, &GeometryStatistics::cacheMisses);

        Nfp::NfpOptions resolvedOptions = options;
        if (autoRouting)
        {
            resolvedOptions.routingPolicy = Nfp::NfpRoutingPolicy::Forced;
            resolvedOptions.decompositionMergeStrategy =
                executionPlan.decompositionStrategy;
        }

        GeometryResult<Nfp::NfpResult> computed;

        // Identities captured while the indices into nfpGeometries_ are still known
        // valid. See the snapshot note in the prepared branch: eviction can clear that
        // vector, and the cache key is built after every call that can evict.
        std::optional<std::uint64_t> capturedIdentityA;
        std::optional<std::uint64_t> capturedIdentityB;
        if (innerFit)
        {
            // The cached payload has to be intrinsic, so neither operand may contribute
            // its definition's budget here: caller provenance is composed exactly once,
            // later, in ComposeNfpHandle.
            //
            // The PART needs no copy any more. PrepareInnerFitPart strips provenance from
            // its own working copy before any intrinsic stage runs, and the operand below
            // carries an empty budget, so handing it the definition's path by reference is
            // already provenance-free. That removes one full Path deep copy per IFP miss -
            // 1006-1437 segments each on the real corpus.
            //
            // The CONTAINER still needs one: its Path::budget reaches the result through
            // Boolean::Difference inside the erosion, and zeroing a local copy is the only
            // way to keep the cached region provenance-free. Kept inside this branch so an
            // ordinary NFP miss never copies a geometry it will not use.
            auto preparedPart = Nfp::PrepareInnerFitPart(
                b.definition->LocalPath(), rotationRadians, queryContext, resolvedOptions);
            if (!preparedPart.Ok() && preparedPart.Status() != GeometryStatus::Empty)
            {
                computed = GeometryResult<Nfp::NfpResult>::Failure(preparedPart.Status());
            }
            else if (preparedPart.Status() == GeometryStatus::Empty ||
                     !preparedPart.Value().Valid())
            {
                computed = GeometryResult<Nfp::NfpResult>::Empty(Nfp::NfpResult{});
            }
            else
            {
                Path intrinsicContainer = a.definition->LocalPath();
                intrinsicContainer.budget = ErrorBudget{};
                computed = Nfp::InnerFitPrepared(
                    intrinsicContainer,
                    Nfp::PreparedInnerFitOperand(preparedPart.Value()),
                    queryContext, resolvedOptions);
            }
        }
        else
        {
            // A prepared derivative requires a sound exact identity. Resolve each
            // operand once, then pass the already-built query context and identity
            // through the rest of the miss pipeline.
            if (!fa.Ok())
                return { fa.Status() };
            if (!fb.Ok())
                return { fb.Status() };
            if (!geometryA.has_value())
                geometryA = AddNfpGeometry(*a.definition, fa.Value());
            if (!geometryB.has_value())
            {
                // Adding A can make B resolvable when both definitions describe the
                // same exact geometry. Recheck B after that state change so identity
                // sharing remains exact and order-independent.
                geometryB = FindNfpGeometry(*b.definition, fb.Value());
                if (!geometryB.has_value())
                    geometryB = AddNfpGeometry(*b.definition, fb.Value());
            }

            // SNAPSHOT BEFORE ANYTHING CAN EVICT.
            //
            // `geometryA`/`geometryB` are indices into nfpGeometries_, and that vector is
            // not stable across a cache eviction: EvictNfpTo can reach
            // ClearNfpGeometryIdentities, which calls nfpGeometries_.clear(). Every
            // PrepareNfpOperandResolved below ends in EvictNfpTo, so by the time the
            // cache key is built the indices can be past the end - and the features
            // POINTER handed into that same call can be dangling while the callee holds
            // it.
            //
            // Found by the WP13 Debug gate: "vector subscript out of range" in
            // V8SessionNfp.NfpEntriesObeyTheWholeSessionByteCeiling and
            // V8_1NfpCacheHandle.HandleOutlivesEvictionClearAndSession, both of which set
            // a byte ceiling low enough that eviction fires on every insert. Release
            // builds read past the end silently instead of asserting, which is why this
            // survived every green run until a Debug build was made.
            //
            // Both values are cheap: an integer identity and a small POD feature set.
            // Copying them removes the dangling index and the dangling pointer together.
            capturedIdentityA = nfpGeometries_[*geometryA].geometryIdentity;
            capturedIdentityB = nfpGeometries_[*geometryB].geometryIdentity;
            const Nfp::NfpOperandFeatures featuresA = nfpGeometries_[*geometryA].features;
            const Nfp::NfpOperandFeatures featuresB = nfpGeometries_[*geometryB].features;

            if (holeFilterPlan.Applied())
            {
                const Transform2 stationaryTransform = Transform2::Identity();
                const Transform2& movingTransform = *holeMovingTransform;
                Path effectiveA = *nfpGeometries_[*geometryA].exactGeometry;
                Path effectiveB = rotationRadians == 0.0
                    ? *nfpGeometries_[*geometryB].exactGeometry
                    : TransformPath(
                        *nfpGeometries_[*geometryB].exactGeometry,
                        movingTransform);
                auto filtered = Nfp::FilterProvenIrrelevantHoles(
                    effectiveA,
                    *nfpGeometries_[*geometryA].holeAnalysis,
                    stationaryTransform,
                    effectiveB,
                    *nfpGeometries_[*geometryB].holeAnalysis,
                    movingTransform,
                    queryContext);
                if (!filtered.Ok())
                {
                    return { filtered.Status() };
                }

                constexpr std::uint64_t kFilteredDefinitionA = 1;
                constexpr std::uint64_t kFilteredDefinitionB = 2;
                auto preparedA = Nfp::PreparedNfpOperand::BuildTransient(
                    filtered.Value().stationary.filtered,
                    kFilteredDefinitionA, sessionIdentity_,
                    nfpGeometries_[*geometryA].geometryIdentity,
                    0.0, false, config_.nfpLattice, queryContext,
                    resolvedOptions);
                if (!preparedA.Ok())
                {
                    return { preparedA.Status() };
                }
                auto preparedB = Nfp::PreparedNfpOperand::BuildTransient(
                    filtered.Value().moving.filtered,
                    kFilteredDefinitionB, sessionIdentity_,
                    nfpGeometries_[*geometryB].geometryIdentity,
                    0.0, true, config_.nfpLattice, queryContext,
                    resolvedOptions);
                if (!preparedB.Ok())
                {
                    return { preparedB.Status() };
                }
                computed = ComputePreparedNfp(
                    preparedA.Value(), preparedB.Value(), resolvedOptions,
                    queryContext, &executionPlan);
            }
            else
            {
                const auto preparedA = PrepareNfpOperandResolved(
                    *a.definition, *capturedIdentityA, 0.0, false,
                    resolvedOptions, queryContext,
                    autoRouting ? &featuresA : nullptr);
                if (!preparedA.Ok())
                {
                    return { preparedA.Status() };
                }
                const auto preparedB = PrepareNfpOperandResolved(
                    *b.definition, *capturedIdentityB, rotationRadians, true,
                    resolvedOptions, queryContext,
                    autoRouting ? &featuresB : nullptr);
                if (!preparedB.Ok())
                {
                    return { preparedB.Status() };
                }
                computed = ComputePreparedNfp(
                    *preparedA.Value(), *preparedB.Value(), resolvedOptions,
                    queryContext, &executionPlan);
            }
        }
        if (!computed.Ok())
            return { computed.Status() };

        // Missing fingerprint means there is no sound candidate identity. Return the
        // intrinsic computation composed with provenance, but do not guess a cache key.
        if (!fa.Ok() || !fb.Ok())
        {
            const GeometryStatus status = computed.Status();
            auto payload = std::make_shared<const Nfp::NfpResult>(
                std::move(computed).Value());
            return { status, payload.get(), std::move(payload) };
        }

        // The prepared branch captured both identities while its indices were still
        // valid; reading nfpGeometries_ again here would be reading through indices an
        // eviction may already have invalidated.
        //
        // The inner-fit branch captures nothing - it never resolves operands - so it
        // registers them here instead, which is safe because nothing between that branch
        // and this point can evict: Nfp::PrepareInnerFitPart and Nfp::InnerFitPrepared
        // are pure kernel calls that never touch the session.
        std::uint64_t keyIdentityA = 0;
        std::uint64_t keyIdentityB = 0;
        if (capturedIdentityA.has_value() && capturedIdentityB.has_value())
        {
            keyIdentityA = *capturedIdentityA;
            keyIdentityB = *capturedIdentityB;
        }
        else
        {
            const std::size_t indexA = geometryA.has_value()
                ? *geometryA : AddNfpGeometry(*a.definition, fa.Value());
            if (!geometryB.has_value())
            {
                geometryB = FindNfpGeometry(*b.definition, fb.Value());
                if (!geometryB.has_value())
                    geometryB = AddNfpGeometry(*b.definition, fb.Value());
            }
            keyIdentityA = nfpGeometries_[indexA].geometryIdentity;
            keyIdentityB = nfpGeometries_[*geometryB].geometryIdentity;
        }
        const NfpKey key = makeKey(keyIdentityA, keyIdentityB);
        const std::uint64_t bucket = keyBucket(key);

        NfpEntry entry;
        entry.key = key;
        entry.status = computed.Status();
        entry.result = std::make_shared<const Nfp::NfpResult>(
            std::move(computed).Value());
        entry.lastUsed = ++nfpClock_;
        entry.bytes = sizeof(NfpEntry) + sizeof(Nfp::NfpResult) +
            PathBytes(entry.result->region);

        const GeometryStatus resultStatus = entry.status;
        std::shared_ptr<const Nfp::NfpResult> resultPayload = entry.result;

        nfpBytes_ += entry.bytes;
        nfp_[bucket].push_back(std::move(entry));
        ++statistics_.nfpEntries;
        statistics_.nfpBytes = nfpBytes_;
        statistics_.nfpBuckets = nfp_.size();

        EvictNfpTo(context_.Limits().maxSessionBytes);
        const Nfp::NfpResult* resultValue = resultPayload.get();
        return { resultStatus, resultValue, std::move(resultPayload) };
    }

    GeometryResult<Nfp::NfpResult> GeometrySession::Nfp(
        const PreparedShapeDefinition& stationary, const PreparedShapeDefinition& moving,
        double rotationRadians, const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        return Nfp(NfpQueryOperand(stationary), NfpQueryOperand(moving),
                   rotationRadians, options, control);
    }

    GeometryResult<Nfp::NfpResult> GeometrySession::Nfp(
        const NfpQueryOperand& stationary, const NfpQueryOperand& moving,
        double rotationRadians, const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        const NfpCachedPayload cached = NfpCachedCore(
            stationary, moving, rotationRadians, options, control, false, false);
        if (!IsSuccess(cached.status) || cached.value == nullptr)
        {
            return GeometryResult<Nfp::NfpResult>::Failure(
                IsSuccess(cached.status)
                    ? GeometryStatus::NumericalFailure : cached.status);
        }
        return ComposeStoredNfpResult(
            cached.status, *cached.value,
            stationary.provenance, moving.provenance);
    }

    GeometryResult<Nfp::NfpResultHandle> GeometrySession::NfpHandle(
        const PreparedShapeDefinition& stationary,
        const PreparedShapeDefinition& moving,
        double rotationRadians, const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        return NfpHandle(
            NfpQueryOperand(stationary), NfpQueryOperand(moving),
            rotationRadians, options, control);
    }

    GeometryResult<Nfp::NfpResultHandle> GeometrySession::NfpHandle(
        const NfpQueryOperand& stationary, const NfpQueryOperand& moving,
        double rotationRadians, const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        NfpCachedPayload cached = NfpCachedCore(
            stationary, moving, rotationRadians, options, control, false, true);
        if (!IsSuccess(cached.status) || cached.owner == nullptr)
        {
            return GeometryResult<Nfp::NfpResultHandle>::Failure(
                IsSuccess(cached.status)
                    ? GeometryStatus::NumericalFailure : cached.status);
        }
        return ComposeNfpHandle(
            cached.status, std::move(cached.owner),
            stationary.provenance, moving.provenance);
    }

    GeometryResult<Nfp::NfpResult> GeometrySession::InnerFit(
        const PreparedShapeDefinition& container, const PreparedShapeDefinition& part,
        double rotationRadians, const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        return InnerFit(NfpQueryOperand(container), NfpQueryOperand(part),
                        rotationRadians, options, control);
    }

    GeometryResult<Nfp::NfpResult> GeometrySession::InnerFit(
        const NfpQueryOperand& container, const NfpQueryOperand& part,
        double rotationRadians, const Nfp::NfpOptions& options,
        const GeometryQueryControl& control)
    {
        const NfpCachedPayload cached = NfpCachedCore(
            container, part, rotationRadians, options, control, true, false);
        if (!IsSuccess(cached.status) || cached.value == nullptr)
        {
            return GeometryResult<Nfp::NfpResult>::Failure(
                IsSuccess(cached.status)
                    ? GeometryStatus::NumericalFailure : cached.status);
        }
        return ComposeStoredNfpResult(
            cached.status, *cached.value, container.provenance, part.provenance);
    }

    void GeometrySession::EvictNfpTo(std::size_t ceiling)
    {
        // The same zero escape hatch EvictDefinitionsTo had, and removed for the same
        // reason: the value is the ceiling, and zero is a ceiling of zero.
        while (CacheBytes() > ceiling && !nfp_.empty())
        {
            auto victimBucket = nfp_.end();
            std::size_t victimIndex = 0;
            std::uint64_t oldest = 0;
            bool found = false;

            for (auto it = nfp_.begin(); it != nfp_.end(); ++it)
            {
                for (std::size_t i = 0; i < it->second.size(); ++i)
                {
                    if (found && it->second[i].lastUsed >= oldest) continue;
                    found = true;
                    oldest = it->second[i].lastUsed;
                    victimBucket = it;
                    victimIndex = i;
                }
            }

            if (!found)
                break;

            std::vector<NfpEntry>& entries = victimBucket->second;
            nfpBytes_ -= (std::min)(nfpBytes_, entries[victimIndex].bytes);
            entries.erase(entries.begin() + static_cast<std::ptrdiff_t>(victimIndex));
            ++statistics_.nfpEvictions;
            --statistics_.nfpEntries;

            if (entries.empty())
                nfp_.erase(victimBucket);
        }

        // Exact-geometry records are stored once per distinct operand rather than once
        // per pair. If every pair entry was evicted, no hot key can refer to them and
        // retaining them would let identity memory alone exceed the ceiling.
        if (nfp_.empty() && CacheBytes() > ceiling)
            ClearNfpGeometryIdentities();

        // And give back the emptied tables, for the same reason EvictDefinitionsTo does:
        // memory the accounting reports but eviction cannot release turns the ceiling
        // into something the session is unable to honour.
        ShrinkEmptyIndexes();

        statistics_.nfpBytes = nfpBytes_;
        statistics_.nfpBuckets = nfp_.size();
    }

    void GeometrySession::ClearNfpGeometryIdentities() noexcept
    {
        for (const auto& bucket : nfpPrepared_)
            for (const NfpPreparedEntry& entry : bucket.second)
                nfpBytes_ -= (std::min)(nfpBytes_, entry.bytes);
        nfpPrepared_.clear();
        statistics_.nfpPreparedEntries = 0;
        statistics_.nfpPreparedBuckets = 0;
        for (const NfpGeometryRecord& record : nfpGeometries_)
            nfpBytes_ -= (std::min)(nfpBytes_, record.bytes);
        nfpGeometries_.clear();
        nfpGeometryCandidates_.clear();
        nfpDefinitionAliases_.clear();
        statistics_.nfpIdentityBytes = 0;
        statistics_.nfpExactGeometryBytes = 0;
        statistics_.nfpIdentityEntries = 0;
    }

    void GeometrySession::ClearCaches() noexcept
    {
        // Both lanes of the definition cache. Leaving the content lane populated would
        // keep serving definitions built under the old tolerance, which is exactly the
        // staleness this call exists to remove.
        definitions_.clear();
        definitionsByContent_.clear();
        definitionOrder_.clear();
        definitionSlots_.clear();
        definitionBytes_ = 0;
        statistics_.definitionBytes = 0;
        statistics_.definitionEntries = 0;
        statistics_.definitionCanonicalBuckets = 0;
        statistics_.definitionContentBuckets = 0;
        nfp_.clear();
        ClearNfpGeometryIdentities();
        nfpBytes_ = 0;
        statistics_.nfpBytes = 0;
        statistics_.nfpEntries = 0;
        statistics_.nfpBuckets = 0;
        flattened_.clear();
        flattenBytes_ = 0;
        statistics_.flattenBytes = 0;
        statistics_.flattenEntries = 0;
        statistics_.flattenBuckets = 0;

        // clear() keeps every bucket array. A caller who asks for the caches to be
        // cleared is asking for the memory back, so hand back the tables too - otherwise
        // CacheBytes() reports a floor it can never get below.
        ShrinkEmptyIndexes();
    }

}
