#pragma once

#include "Canonical/CanonicalGeometry.hpp"
#include "Canonical/GeometryFingerprint.hpp"
#include "Curves/Flatten.hpp"
#include "GeometryContext.hpp"
#include "GeometryResult.hpp"
#include "GeometryTypes.hpp"
#include "PreparedGeometry.hpp"
#include "Polygon/Nfp.hpp"
#include "Polygon/NfpAuto.hpp"
#include "Polygon/NfpCover.hpp"
#include "Polygon/NfpHoleFilter.hpp"
#include "Polygon/NfpPrepared.hpp"

#include <cstdint>
#include <memory>
#include <optional>
#include <unordered_map>
#include <vector>

namespace ImCut::Geometry
{
    struct GeometrySessionConfig
    {
        GeometryTolerance tolerance = GeometryTolerance::Production();
        PrecisionMode precision = PrecisionMode::Production;
        ComplexityLimits limits{};
        Nfp::NfpLatticeIdentity nfpLattice{};
    };

    // Execution state is intentionally outside GeometrySessionConfig. It may vary per
    // query without changing geometry semantics or cache identity.
    struct GeometryQueryControl
    {
        CancellationToken cancellation{};
        GeometryStatistics* statistics = nullptr;
        GeometryDiagnostics* diagnostics = nullptr;
    };

    struct NfpQueryOperand
    {
        const PreparedShapeDefinition* definition = nullptr;
        ErrorBudget provenance{};

        NfpQueryOperand() = default;
        explicit NfpQueryOperand(const PreparedShapeDefinition& value)
            : definition(&value), provenance(value.LocalPath().budget) {}
        NfpQueryOperand(const PreparedShapeDefinition& value, ErrorBudget sourceProvenance)
            : definition(&value), provenance(sourceProvenance) {}
    };

    // Query-bound view of an intrinsic prepared derivative. Geometry identity and
    // cache reuse remain provenance-free; caller error is composed exactly once when
    // a result crosses back out of the session.
    struct NfpPreparedQueryOperand
    {
        Nfp::PreparedNfpOperandPtr operand{};
        ErrorBudget provenance{};

        NfpPreparedQueryOperand() = default;
        explicit NfpPreparedQueryOperand(Nfp::PreparedNfpOperandPtr value)
            : operand(std::move(value)) {}
        NfpPreparedQueryOperand(Nfp::PreparedNfpOperandPtr value,
                                ErrorBudget sourceProvenance)
            : operand(std::move(value)), provenance(sourceProvenance) {}
    };

    // Independent snapshot of whatever a host application handed the kernel.
    //
    // The extraction boundary. Corel COM (or an SVG loader, or a test) is read exactly
    // once to build this; from here on every query runs against plain C++ memory. That
    // is what makes millions of queries affordable - re-reading the host per query is
    // the pattern this kernel exists to eliminate.
    class GeometrySnapshot
    {
    public:
        struct Object
        {
            GeometryObjectId id;
            Path path;

            // Opaque host-side identity (a Corel StaticID, an SVG element index).
            // The kernel never interprets it; it only carries it back out.
            std::int64_t sourceTag = 0;
        };

        GeometryObjectId Add(Path path, std::int64_t sourceTag = 0);

        [[nodiscard]] std::size_t Count() const noexcept { return objects_.size(); }
        [[nodiscard]] bool Empty() const noexcept { return objects_.empty(); }
        [[nodiscard]] const std::vector<Object>& Objects() const noexcept { return objects_; }

        [[nodiscard]] const Object* Find(GeometryObjectId id) const noexcept;
        [[nodiscard]] Bounds2 Bounds() const;
        [[nodiscard]] std::size_t SegmentCount() const noexcept;

        void Clear() noexcept;
        void Reserve(std::size_t count) { objects_.reserve(count); }

    private:
        std::vector<Object> objects_;
    };

    // Per-operation cache and scratch owner.
    //
    // Caches live here rather than in a global, so two concurrent operations never
    // contend and a session's lifetime bounds its memory. One session is meant to be
    // used by one worker; sessions are cheap to create per worker.
    class GeometrySession
    {
    public:
        GeometrySession();
        explicit GeometrySession(const GeometrySessionConfig& config);

        // Compatibility adapter: snapshots the semantic fields once at construction.
        // New code should construct with GeometrySessionConfig and pass execution state
        // through GeometryQueryControl.
        explicit GeometrySession(const GeometryContext& context);

        // Read-only by design. A mutable accessor let callers change tolerance or
        // precision behind the session, leaving every derivative cached under the old
        // settings and still being served. Anything that invalidates derivatives goes
        // through the setters below; anything that does not is operation-local and
        // belongs on a GeometryContext the caller owns.
        [[nodiscard]] const GeometryContext& Context() const noexcept { return context_; }
        [[nodiscard]] const GeometrySessionConfig& Config() const noexcept { return config_; }
        [[nodiscard]] std::uint64_t Identity() const noexcept { return sessionIdentity_; }

        // Prepares `path` as a placed instance of a shared shape definition.
        //
        // The definition is reused whenever the intrinsic geometry fingerprints
        // identically, which - because canonicalisation is translation invariant - means
        // two placements of the same part share all the expensive derivatives. The
        // instance keeps its own identity, transform and world bounds, so reuse can
        // never move a part to where an earlier one happened to sit.
        //
        // `placement` is applied on top of the path's own position.
        [[nodiscard]] GeometryResult<GeometryInstance> PrepareInstance(
            const Path& path,
            const Transform2& placement = Transform2::Identity(),
            PreparationLevel level = PreparationLevel::CollisionReady,
            std::int64_t sourceTag = 0);

        // Definition only, with no placement. For callers that work purely in the
        // shape's local frame.
        //
        // WHAT YOU GET BACK MAY NOT BE THE GEOMETRY YOU PASSED IN.
        //
        // Definitions are keyed by a canonical form quantised to
        // Quantization::kCanonicalResolution (1e-4 mm), so two paths whose coordinates
        // differ by less than that share one definition, and `LocalPath()` is the
        // geometry of whichever caller created it first. The measured collapse radius is
        // 5e-5 mm. That lattice is deliberate - it is what makes identity survive
        // translation above double noise - and it is not going to be tightened.
        //
        // The returned result's own Budget() carries the error THIS call inherited:
        // exact on a miss and on a bit-identical hit, AddQuantization(kCanonicalResolution)
        // when a different path was served. `budget` is the same value as an out-param,
        // kept for convenience. A caller composing a clearance reads Budget() and cannot
        // be told exact by omission. Use PrepareDefinitionExact when sharing is not
        // acceptable at all; PreparedShapeDefinition::IsShared() says whether it happened.
        [[nodiscard]] GeometryResult<PreparedShapeDefinitionPtr> PrepareDefinition(
            const Path& localPath, PreparationLevel level = PreparationLevel::CollisionReady,
            ErrorBudget* budget = nullptr);

        // Same cache, stricter reuse: a definition is shared only when its stored path
        // is coordinate-for-coordinate identical to `localPath`.
        //
        // The canonical form is not consulted for the decision, so nothing can alias
        // and `budget` always comes back exact. Two parts differing by 4e-5 mm get two
        // definitions and two sets of derivatives - that is the cost, and it is the
        // caller's to choose. Asking twice for the same path is still one object.
        [[nodiscard]] GeometryResult<PreparedShapeDefinitionPtr> PrepareDefinitionExact(
            const Path& localPath, PreparationLevel level = PreparationLevel::CollisionReady,
            ErrorBudget* budget = nullptr);

        // Flatten cache. The key includes the tolerance and options, so a coarse result
        // can never be handed to a caller that asked for a finer one.
        // Flattens a contour, caching the result for the exact geometry requested.
        //
        // Returns a status, not a bare reference: flattening can exhaust the context
        // point budget, and there was previously nowhere to say so - the caller received
        // a truncated contour that looked complete.
        //
        // The cache key identifies the PAYLOAD, not the shape. The shape fingerprint is
        // deliberately invariant to translation, start node and winding; keying absolute
        // flattened coordinates with it returned the first caller's coordinates to the
        // second. See the exact-identity key in the implementation.
        [[nodiscard]] const GeometryResult<FlattenedContour>& Flatten(const Contour& contour,
                                                                      const FlattenOptions& options);

        // NFP and IFP, memoised on the canonical identity of the two operands.
        //
        // A nesting run asks the same question thousands of times - the same pair of
        // parts at the same rotation from its fixed rotation set - and a cold concave
        // NFP measured 3 830 us (audit-v7/evidence/10_perf.txt). The answer depends only
        // on the key, so serving it twice is not an approximation.
        //
        // The rotation enters the key as its exact bit pattern, not quantised. Rounding
        // it would trade precision for hit rate, which is a contract decision this is
        // not entitled to make.
        [[nodiscard]] GeometryResult<Nfp::NfpResult> Nfp(
            const PreparedShapeDefinition& stationary, const PreparedShapeDefinition& moving,
            double rotationRadians, const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpResult> Nfp(
            const NfpQueryOperand& stationary, const NfpQueryOperand& moving,
            double rotationRadians, const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        // High-throughput cache API. The returned handle owns an immutable intrinsic
        // payload and survives eviction, ClearCaches() and session lifetime. Region()
        // is zero-copy; CopyValue() explicitly opts into compatibility cost.
        [[nodiscard]] GeometryResult<Nfp::NfpResultHandle> NfpHandle(
            const PreparedShapeDefinition& stationary,
            const PreparedShapeDefinition& moving,
            double rotationRadians, const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpResultHandle> NfpHandle(
            const NfpQueryOperand& stationary, const NfpQueryOperand& moving,
            double rotationRadians, const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::PreparedNfpOperandPtr> PrepareNfpOperand(
            const PreparedShapeDefinition& definition,
            double rotationRadians,
            bool reflected,
            const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<NfpPreparedQueryOperand> PrepareNfpOperand(
            const NfpQueryOperand& definition,
            double rotationRadians,
            bool reflected,
            const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpResult> Nfp(
            const Nfp::PreparedNfpOperand& stationary,
            const Nfp::PreparedNfpOperand& reflectedMoving,
            const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpResult> Nfp(
            const NfpPreparedQueryOperand& stationary,
            const NfpPreparedQueryOperand& reflectedMoving,
            const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpCover> NfpCover(
            const Nfp::PreparedNfpOperand& stationary,
            const Nfp::PreparedNfpOperand& reflectedMoving,
            const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpCover> NfpCover(
            const NfpPreparedQueryOperand& stationary,
            const NfpPreparedQueryOperand& reflectedMoving,
            const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpContactCandidates> NfpContacts(
            const Nfp::PreparedNfpOperand& stationary,
            const Nfp::PreparedNfpOperand& reflectedMoving,
            const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpContactCandidates> NfpContacts(
            const NfpPreparedQueryOperand& stationary,
            const NfpPreparedQueryOperand& reflectedMoving,
            const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpResult> InnerFit(
            const PreparedShapeDefinition& container, const PreparedShapeDefinition& part,
            double rotationRadians, const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<Nfp::NfpResult> InnerFit(
            const NfpQueryOperand& container, const NfpQueryOperand& part,
            double rotationRadians, const Nfp::NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        // Invalidates everything. Call when the source geometry changes: a stale
        // derivative is worse than no cache at all.
        void ClearCaches() noexcept;

        // Bytes currently held by the flatten cache, by the same accounting the
        // eviction policy uses. Exposed so a caller can size maxCacheBytes from
        // measurement rather than from a guess.
        [[nodiscard]] std::size_t FlattenCacheBytes() const noexcept { return flattenBytes_; }

        // What the session retains: the flatten, definition and NFP payloads, plus the
        // index structures and spare container capacity the session owns to manage them.
        // This is the quantity ComplexityLimits::maxSessionBytes bounds, and it is what a
        // caller should size that ceiling from.
        //
        // WHAT IT DOES NOT COVER, stated rather than implied.
        //
        // V8.1 said this "bounds EVERYTHING the session holds" while counting three
        // payload totals and nothing else. Measured against the operating system's own
        // view of the process under a 400-definition load, the accounted figure sat
        // 1.31x-1.41x below process private bytes (evidence 884). Part of that gap was
        // the lookup maps and LRU vectors below, which are now counted; the rest is:
        //
        //   - allocator overhead, which no portable C++ can measure. Claiming to bound
        //     it would replace one false statement with another.
        //   - objects a caller keeps alive through its own shared_ptr after the session
        //     has evicted them. That memory is the caller's, and the session cannot
        //     release it.
        //
        // See V8_1_1_SESSION_MEMORY_CONTRACT.md.
        [[nodiscard]] std::size_t CacheBytes() const noexcept
        {
            return flattenBytes_ + definitionBytes_ + nfpBytes_ + IndexBytes();
        }

        struct CacheStatistics
        {
            std::uint64_t preparedHits = 0;
            std::uint64_t preparedMisses = 0;
            std::uint64_t flattenHits = 0;
            std::uint64_t flattenMisses = 0;
            std::uint64_t verificationRejects = 0;

            // Entries dropped because the cache reached ComplexityLimits::maxCacheBytes.
            // Non-zero means the session is thrashing and the ceiling is too small for
            // the workload - a fact worth surfacing rather than absorbing silently.
            std::uint64_t flattenEvictions = 0;

            // Bytes the flatten cache currently holds, by the same accounting the
            // eviction policy uses.
            std::size_t flattenBytes = 0;
            std::size_t flattenEntries = 0;
            std::size_t flattenBuckets = 0;

            // Definitions dropped because the session reached
            // ComplexityLimits::maxSessionBytes. Same meaning as flattenEvictions: a
            // non-zero value says the ceiling is too small for the workload, which is
            // a fact worth surfacing rather than absorbing silently.
            std::uint64_t definitionEvictions = 0;

            // Bytes the definition cache currently holds.
            std::size_t definitionBytes = 0;
            std::size_t definitionEntries = 0;
            std::size_t definitionCanonicalBuckets = 0;
            std::size_t definitionContentBuckets = 0;

            std::uint64_t nfpHits = 0;
            std::uint64_t nfpMisses = 0;
            std::uint64_t nfpEvictions = 0;
            std::size_t nfpBytes = 0;
            std::size_t nfpEntries = 0;
            std::size_t nfpBuckets = 0;
            // Identity-registry subset of nfpBytes. Exact geometry is immutable and
            // shared with a cached definition whenever possible. exactGeometryBytes is
            // therefore only the payload currently charged to NFP after the owning
            // definition was evicted (or when the definition came from outside this
            // session); identityBytes also includes records and lazy hole analysis.
            std::size_t nfpIdentityBytes = 0;
            std::size_t nfpExactGeometryBytes = 0;
            std::size_t nfpIdentityEntries = 0;
            std::uint64_t nfpPreparedHits = 0;
            std::uint64_t nfpPreparedMisses = 0;
            std::size_t nfpPreparedEntries = 0;
            std::size_t nfpPreparedBuckets = 0;
        };

        [[nodiscard]] const CacheStatistics& Statistics() const noexcept { return statistics_; }

        // Reusable scratch, so repeated queries in a loop stop allocating.
        [[nodiscard]] std::vector<std::uint32_t>& IndexScratch() noexcept { return indexScratch_; }
        [[nodiscard]] std::vector<Vec2>& PointScratch() noexcept { return pointScratch_; }

    private:
        struct FlattenKey
        {
            // A cheap hash over the contour's RAW bytes, plus the options that change
            // the payload.
            //
            // This used to be a 128-bit fingerprint of the canonical form, computed on
            // every lookup. Canonicalising copies the whole contour per variant,
            // allocates a vector of 56-byte node keys, allocates 2n ptrdiff_t inside
            // Booth's algorithm and reverses through a temporary - which made a cache
            // HIT cost 2.8 us against 0.7 us to just flatten the thing again. A cache
            // that loses to not having a cache is not a cache.
            //
            // The payload is a list of absolute points in a specific order, so the key
            // has to identify the exact geometry - position and traversal included. A
            // hash of the raw arrays does that with no copy and no allocation, and the
            // exact comparison below turns a collision into a second probe rather than
            // into another contour's coordinates.
            std::uint64_t contentHash;
            double tolerance;
            int maxDepth;

            [[nodiscard]] bool operator==(const FlattenKey& other) const noexcept
            {
                return contentHash == other.contentHash &&
                       tolerance == other.tolerance &&
                       maxDepth == other.maxDepth;
            }
        };

        // A hash match is a candidate, never a proof - the same discipline the
        // prepared-definition cache applies. Verification is exact equality of the
        // source contour, which is both cheaper and stricter than comparing canonical
        // forms: the flatten payload is absolute, so anything short of identical
        // coordinates is a different answer.
        struct FlattenEntry
        {
            Contour source;
            GeometryResult<FlattenedContour> result =
                GeometryResult<FlattenedContour>::Failure(GeometryStatus::InvalidInput);

            // Monotonic stamp of the last hit. The eviction policy is least-recently
            // used, and LRU needs a clock.
            std::uint64_t lastUsed = 0;

            // Cached so eviction does not have to re-measure a contour it is discarding.
            std::size_t bytes = 0;
        };

        struct FlattenKeyHash
        {
            [[nodiscard]] std::size_t operator()(const FlattenKey& key) const noexcept;
        };

        const std::uint64_t sessionIdentity_;
        const GeometrySessionConfig config_;
        const GeometryContext context_;
        CacheStatistics statistics_;

        // Cached definitions, keyed two ways, plus the LRU bookkeeping the session
        // ceiling needs. A definition appears in both maps and is counted once.
        struct DefinitionRecord
        {
            PreparedShapeDefinitionPtr definition;
            std::uint64_t lastUsed = 0;
            std::size_t bytes = 0;
        };

        std::unordered_map<GeometryFingerprint, std::vector<PreparedShapeDefinitionPtr>> definitions_;

        // Fast lane in front of `definitions_`, keyed by a cheap hash of the path's raw
        // coordinates and verified by exact comparison.
        //
        // The canonical fingerprint is what makes two DIFFERENT-looking paths share a
        // definition, and it has to stay. But computing it costs 4.6 us, and 82% of a
        // PrepareInstance that HITS was spent deriving an identity for geometry the
        // session had already seen byte for byte. The overwhelmingly common case - the
        // same part prepared again, which is what a nesting run does thousands of times
        // per generation - never needs the canonical form at all.
        //
        // A miss here falls through to the canonical lookup, so nothing that used to
        // share a definition stops sharing one.
        std::unordered_map<std::uint64_t, std::vector<PreparedShapeDefinitionPtr>> definitionsByContent_;
        std::uint32_t nextInstanceId_ = 0;
        std::unordered_map<FlattenKey, std::vector<FlattenEntry>, FlattenKeyHash> flattened_;

        std::vector<std::uint32_t> indexScratch_;
        std::vector<Vec2> pointScratch_;

        // Approximate bytes held by `flattened_`, and the LRU clock.
        //
        // The caches used to grow monotonically with no ceiling and no eviction: the
        // only ways to release memory were ClearCaches() and destroying the session.
        // For a nesting run with thousands of rotation variants that is unbounded
        // growth by design.
        std::size_t flattenBytes_ = 0;
        std::uint64_t flattenClock_ = 0;

        // The definition LRU. Kept beside the two lookup maps rather than inside them
        // because a definition lives in both and must be counted, touched and evicted
        // once.
        //
        // `definitionSlots_` maps a definition to its index in `definitionOrder_`, so a
        // cache HIT touches the LRU in O(1). A linear scan would have been simpler and
        // is what this had first, but a hit is on the path section 6 holds to 1 us and
        // a nesting session holds thousands of definitions - paying a scan of all of
        // them per hit is the kind of cost that only shows up at the scale that matters.
        // Removal swaps with the back and repairs the moved element's slot; the LRU
        // victim search stays linear, and that runs only when something is evicted.
        std::vector<DefinitionRecord> definitionOrder_;
        std::unordered_map<const PreparedShapeDefinition*, std::size_t> definitionSlots_;

        // A hash match is a candidate, never a proof: the full key is compared before
        // any reuse, the same discipline the definition cache applies.
        struct NfpKey
        {
            std::uint64_t geometryA = 0;
            std::uint64_t geometryB = 0;
            std::uint64_t rotationBits = 0;
            std::uint64_t clearanceBits = 0;
            std::uint64_t toleranceBits = 0;
            std::size_t maxPieces = 0;
            // Version, algorithm, effective decomposition strategy and the resolved
            // pair-specific hole proof are one semantic identity. Packing them keeps
            // the warm-hit comparison compact without weakening cache correctness.
            std::uint64_t resolvedPlanIdentity = 0;
            bool innerFit = false;

            [[nodiscard]] bool operator==(const NfpKey& other) const noexcept
            {
                return geometryA == other.geometryA && geometryB == other.geometryB &&
                       rotationBits == other.rotationBits &&
                       clearanceBits == other.clearanceBits &&
                       toleranceBits == other.toleranceBits && maxPieces == other.maxPieces &&
                       resolvedPlanIdentity == other.resolvedPlanIdentity &&
                       innerFit == other.innerFit;
            }
        };

        struct NfpEntry
        {
            NfpKey key;
            // Intrinsic result only. Operand provenance is composed on each return and
            // therefore is deliberately absent from the key and cached value.
            std::shared_ptr<const Nfp::NfpResult> result{};
            GeometryStatus status = GeometryStatus::InvalidInput;
            std::uint64_t lastUsed = 0;
            std::size_t bytes = 0;
        };

        struct NfpGeometryRecord
        {
            std::uint64_t geometryIdentity = 0;
            GeometryFingerprint fingerprint{};
            std::shared_ptr<const Path> exactGeometry{};
            Nfp::NfpOperandFeatures features{};
            GeometryStatus featureStatus = GeometryStatus::Success;
            std::shared_ptr<const Nfp::PreparedHoleFilterAnalysis> holeAnalysis{};
            GeometryStatus holeAnalysisStatus = GeometryStatus::Unsupported;
            std::size_t bytes = 0;
            std::size_t exactGeometryBytes = 0;

            // While this definition remains in the session cache it already accounts
            // for the shared Path. On definition eviction the charge migrates here, so
            // CacheBytes() counts the retained allocation exactly once through every
            // ownership transition. The identity guards allocator address reuse.
            const PreparedShapeDefinition* accountingDefinition = nullptr;
            std::uint64_t accountingDefinitionIdentity = 0;
            bool exactGeometryBytesCharged = false;
        };

        struct NfpDefinitionAlias
        {
            std::uint64_t definitionIdentity = 0;
            std::size_t geometryIndex = 0;
        };

        struct NfpPreparedKey
        {
            std::uint64_t geometryIdentity = 0;
            std::uint64_t rotationBits = 0;
            std::uint64_t simplifyBits = 0;
            std::size_t maxPieces = 0;
            DecompositionMergeStrategy decompositionMergeStrategy =
                DecompositionMergeStrategy::HertelMehlhorn;
            bool reflected = false;

            [[nodiscard]] bool operator==(const NfpPreparedKey& other) const noexcept
            {
                return geometryIdentity == other.geometryIdentity &&
                       rotationBits == other.rotationBits &&
                       simplifyBits == other.simplifyBits &&
                       maxPieces == other.maxPieces &&
                       decompositionMergeStrategy == other.decompositionMergeStrategy &&
                       reflected == other.reflected;
            }
        };

        struct NfpPreparedEntry
        {
            NfpPreparedKey key;
            Nfp::PreparedNfpOperandPtr operand;
            GeometryStatus status = GeometryStatus::Success;
            std::uint64_t lastUsed = 0;
            std::size_t bytes = 0;
        };

        std::unordered_map<std::uint64_t, std::vector<NfpEntry>> nfp_;
        std::vector<NfpGeometryRecord> nfpGeometries_;
        std::unordered_map<std::uint64_t, std::vector<std::size_t>> nfpGeometryCandidates_;
        std::unordered_map<const PreparedShapeDefinition*, NfpDefinitionAlias> nfpDefinitionAliases_;
        std::unordered_map<std::uint64_t, std::vector<NfpPreparedEntry>> nfpPrepared_;
        std::size_t nfpBytes_ = 0;
        std::uint64_t nfpClock_ = 0;
        std::uint64_t nextNfpGeometryIdentity_ = 1;

        struct NfpCachedPayload
        {
            GeometryStatus status = GeometryStatus::InvalidInput;
            const Nfp::NfpResult* value = nullptr;
            std::shared_ptr<const Nfp::NfpResult> owner{};
        };

        [[nodiscard]] NfpCachedPayload NfpCachedCore(
            const NfpQueryOperand& a, const NfpQueryOperand& b,
            double rotationRadians, const Nfp::NfpOptions& options,
            const GeometryQueryControl& control, bool innerFit,
            bool retainPayload);

        [[nodiscard]] GeometryContext QueryContext(const GeometryQueryControl& control) const;
        [[nodiscard]] std::optional<std::size_t> FindNfpGeometry(
            const PreparedShapeDefinition& definition,
            const GeometryFingerprint& fingerprint);
        // Adds a geometry that the caller has already proved absent. Keeping lookup
        // and insertion separate lets a complete NFP miss resolve each operand once.
        [[nodiscard]] std::size_t AddNfpGeometry(
            const PreparedShapeDefinition& definition, const GeometryFingerprint& fingerprint);
        [[nodiscard]] GeometryStatus EnsureNfpHoleAnalysis(
            std::size_t geometryIndex,
            const GeometryContext& queryContext);
        [[nodiscard]] GeometryResult<Nfp::PreparedNfpOperandPtr>
            PrepareNfpOperandResolved(
                const PreparedShapeDefinition& definition,
                std::uint64_t geometryIdentity,
                double rotationRadians,
                bool reflected,
                const Nfp::NfpOptions& options,
                const GeometryContext& queryContext,
                const Nfp::NfpOperandFeatures* knownFeatures = nullptr);
        [[nodiscard]] GeometryResult<Nfp::NfpResult> ComputePreparedNfp(
            const Nfp::PreparedNfpOperand& stationary,
            const Nfp::PreparedNfpOperand& reflectedMoving,
            const Nfp::NfpOptions& options,
            const GeometryContext& queryContext,
            const Nfp::NfpExecutionPlan* plan = nullptr);
        [[nodiscard]] GeometryResult<Nfp::NfpResultHandle> ComposeNfpHandle(
            GeometryStatus status,
            std::shared_ptr<const Nfp::NfpResult> intrinsic,
            const ErrorBudget& provenanceA, const ErrorBudget& provenanceB) const;
        [[nodiscard]] GeometryResult<Nfp::NfpResult> ComposeStoredNfpResult(
            GeometryStatus status, const Nfp::NfpResult& intrinsic,
            const ErrorBudget& provenanceA, const ErrorBudget& provenanceB) const;
        [[nodiscard]] GeometryResult<Nfp::NfpResult> ComposeNfpResult(
            const GeometryResult<Nfp::NfpResult>& intrinsic,
            const ErrorBudget& provenanceA, const ErrorBudget& provenanceB) const;
        void TransferNfpGeometryAccountingFrom(
            const PreparedShapeDefinition& definition) noexcept;
        void ClearNfpGeometryIdentities() noexcept;

        void EvictNfpTo(std::size_t ceiling);
        std::size_t definitionBytes_ = 0;
        std::uint64_t definitionClock_ = 0;

        // Drops least-recently-used entries until the cache fits its ceiling.
        void EvictFlattenTo(std::size_t ceiling);

        // Records a newly cached definition and drops least-recently-used ones until
        // CacheBytes() fits ComplexityLimits::maxSessionBytes.
        void TrackDefinition(const PreparedShapeDefinitionPtr& definition);

        // Moves an existing definition to the front of the LRU on a cache hit.
        void TouchDefinition(const PreparedShapeDefinitionPtr& definition);

        void EvictDefinitionsTo(std::size_t ceiling);

        // Removes one definition from both lookup maps. Only called on eviction.
        void EraseDefinitionFromLookups(const PreparedShapeDefinitionPtr& definition) noexcept;

        // Bytes the session holds in its own index structures, as opposed to the payloads
        // those structures point at.
        //
        // Every term is O(1) - bucket_count(), size() and capacity() - because this is
        // read once per iteration of the eviction loop and a scan would turn eviction
        // into quadratic work.
        //
        // The per-bucket vectors inside the maps hold one pointer-sized handle per entry
        // and their heap buffers are not walked; that is roughly 16 bytes per cached item
        // against a payload measured in kilobytes. It is named here rather than silently
        // omitted.
        // THE ONE ENUMERATION OF THE SESSION'S RETAINED INDEX CONTAINERS.
        //
        // F23. There used to be three hand-written lists of these containers - one in
        // IndexBytes(), one in ShrinkEmptyIndexes(), one in AuditRetainedIndex() - and
        // they disagreed. `nfpGeometryCandidates_` and `nfpDefinitionAliases_` were added
        // with the NFP geometry index, registered in the shrink list and in ClearCaches,
        // and never registered in the accounting. So the session released them on demand,
        // which is the operational definition of "retained cache" this file uses, while
        // the ceiling was compared against a number that did not include them: 99 120
        // counted against 121 712 actually held, 22 592 bytes outside every ceiling at
        // 150 definitions (baseline log, [F23]).
        //
        // The fix is not to add two terms to one list. It is that a container now enters
        // the accounting, the shrinking and the audit by being named ONCE, here. Adding
        // an eleventh container without touching this function leaves it unshrunk,
        // uncleared and uncounted together - one visible omission instead of three
        // independent lists that can drift apart silently.
        //
        // `visitMap(container, valueSize)` and `visitVector(container, elementSize)` are
        // called for every retained container in a fixed order.
        template <typename VisitMap, typename VisitVector>
        void ForEachIndexContainer(VisitMap&& visitMap, VisitVector&& visitVector) const noexcept
        {
            visitMap(definitions_,
                     sizeof(GeometryFingerprint) + sizeof(std::vector<PreparedShapeDefinitionPtr>));
            visitMap(definitionsByContent_,
                     sizeof(std::uint64_t) + sizeof(std::vector<PreparedShapeDefinitionPtr>));
            visitMap(definitionSlots_,
                     sizeof(const PreparedShapeDefinition*) + sizeof(std::size_t));
            visitMap(flattened_, sizeof(FlattenKey) + sizeof(std::vector<FlattenEntry>));
            visitMap(nfp_, sizeof(std::uint64_t) + sizeof(std::vector<NfpEntry>));
            visitMap(nfpPrepared_, sizeof(std::uint64_t) + sizeof(std::vector<NfpPreparedEntry>));
            visitMap(nfpGeometryCandidates_,
                     sizeof(std::uint64_t) + sizeof(std::vector<std::size_t>));
            visitMap(nfpDefinitionAliases_,
                     sizeof(const PreparedShapeDefinition*) + sizeof(NfpDefinitionAlias));
            visitVector(definitionOrder_, sizeof(DefinitionRecord));
            visitVector(nfpGeometries_, sizeof(NfpGeometryRecord));
        }

        // The per-container byte rule, so the accounting and the audit cannot disagree
        // about arithmetic either.
        [[nodiscard]] static constexpr std::size_t MapBytes(std::size_t buckets,
                                                            std::size_t entries,
                                                            std::size_t valueSize) noexcept
        {
            // An EMPTY map contributes nothing, even though bucket_count() reports a
            // non-zero default. That default array is the cost of the session object
            // existing, not of anything cached - it is there before the first
            // PrepareDefinition and after ShrinkEmptyIndexes has given everything
            // back. Counting it would put a floor of eight default bucket arrays under
            // CacheBytes() that no ceiling could ever reach, and "the ceiling was
            // honoured" would stop being a statement the session can satisfy.
            if (entries == 0)
                return 0;

            // One bucket slot per bucket, plus a node per entry: the value, its key
            // hash slot and the two list pointers every node-based map carries.
            return buckets * sizeof(void*) + entries * (valueSize + 2 * sizeof(void*));
        }

        [[nodiscard]] std::size_t IndexBytes() const noexcept
        {
            std::size_t total = 0;

            ForEachIndexContainer(
                [&total](const auto& map, std::size_t valueSize) noexcept
                {
                    total += MapBytes(map.bucket_count(), map.size(), valueSize);
                },
                // Vectors are counted at CAPACITY, not size: a vector that grew and then
                // had entries evicted still holds the buffer, and a ceiling that ignored
                // it would be describing memory the session has already released when it
                // has not.
                [&total](const auto& vec, std::size_t elementSize) noexcept
                {
                    total += vec.capacity() * elementSize;
                });

            // The per-bucket vectors INSIDE the bucketed maps hold their own heap
            // buffers, and they are retained exactly as the nodes above are. They are
            // walked rather than estimated: at 150 cached definitions the candidate map's
            // nested vectors are 1200 bytes, small next to the nodes but not zero, and
            // "small" is not a term a ceiling can be compared against. The walk is over
            // entries, not payload bytes, and CacheBytes() is read once per eviction
            // iteration - see NestedIndexBytes for why that stays affordable.
            total += NestedIndexBytes();

            // indexScratch_ and pointScratch_ are DELIBERATELY absent. They are working
            // buffers for the operation in flight, not retained cache: their size is
            // bounded by the largest single operation rather than by accumulation, and
            // eviction cannot release them because the next operation needs them again.
            // Counting memory that eviction cannot free would make the ceiling
            // unreachable, which turns every "the ceiling was honoured" assertion into
            // something the session can no longer satisfy.
            //
            // That is the rule the whole of IndexBytes() obeys: it counts only what
            // eviction and ClearCaches can actually give back. ShrinkEmptyIndexes()
            // exists so that stays true for the bucket arrays above.
            return total;
        }

        // Heap held by the vectors stored INSIDE the bucketed maps.
        //
        // Only the candidate map has any: every other bucketed container above stores its
        // per-key vector as the map's value, and those vectors' buffers are charged where
        // the payload is charged (flattenBytes_, definitionBytes_, nfpBytes_), not here.
        // The candidate map's vectors hold bare indices, which no payload total covers.
        //
        // O(entries), and entries is the number of distinct NFP geometry buckets - one
        // per cached shape, not one per query. Kept incremental would mean a second
        // ledger to keep in step with push_back; measured at the ceiling's read rate this
        // walk does not show up, and a second ledger is exactly the class F23 is.
        [[nodiscard]] std::size_t NestedIndexBytes() const noexcept
        {
            std::size_t nested = 0;
            for (const auto& bucket : nfpGeometryCandidates_)
                nested += bucket.second.capacity() * sizeof(std::size_t);
            return nested;
        }

        // Releases the bucket arrays and buffers of containers that eviction has emptied.
        //
        // unordered_map::clear() and vector::clear() keep their allocations, so without
        // this a session evicted down to nothing still reported its empty hash tables -
        // 416 bytes that no ceiling could ever reach. Only empty containers are touched,
        // so this never thrashes a live cache.
        void ShrinkEmptyIndexes() noexcept;

    public:
        // INSTRUMENT, not policy. Reports what IndexBytes() counts against what the same
        // rule would count over EVERY container ShrinkEmptyIndexes() hands back, so the
        // difference between the two is a number rather than an argument.
        //
        // Compiled unconditionally, like GeometryTestHooks: a diagnostic that only exists
        // in the test build measures a binary nobody ships. It reads and returns; it
        // changes nothing, and no production path calls it.
        struct RetainedIndexAudit
        {
            std::size_t counted = 0;            // IndexBytes(), as the ceiling sees it
            std::size_t allContainers = 0;      // the same rule, over every index container
            std::size_t candidateBuckets = 0;
            std::size_t candidateEntries = 0;
            std::size_t candidateNestedBytes = 0;   // the per-bucket vectors themselves
            std::size_t aliasBuckets = 0;
            std::size_t aliasEntries = 0;
        };

        [[nodiscard]] RetainedIndexAudit AuditRetainedIndex() const noexcept
        {
            RetainedIndexAudit audit;
            audit.counted = IndexBytes();

            // MEASURED SEPARATELY, ON PURPOSE.
            //
            // The obvious tidy-up after F23 is to compute `allContainers` from the same
            // ForEachIndexContainer walk that IndexBytes() uses. That would make
            // `counted == allContainers` true by construction and this instrument would
            // stop being able to detect the next omission - it would assert that a sum
            // equals itself. So the two containers that were actually missing keep their
            // own hand-written measurement here, which is the form that caught the defect
            // in the first place, and the equality below stays a real comparison between
            // the ceiling's rule and a second reading of the same memory.
            audit.candidateBuckets = nfpGeometryCandidates_.bucket_count();
            audit.candidateEntries = nfpGeometryCandidates_.size();
            for (const auto& bucket : nfpGeometryCandidates_)
                audit.candidateNestedBytes += bucket.second.capacity() * sizeof(std::size_t);

            audit.aliasBuckets = nfpDefinitionAliases_.bucket_count();
            audit.aliasEntries = nfpDefinitionAliases_.size();

            // The same rule the ceiling obeys, applied to every container
            // ShrinkEmptyIndexes() hands back, written out independently of the walk.
            audit.allContainers =
                MapBytes(definitions_.bucket_count(), definitions_.size(),
                         sizeof(GeometryFingerprint) +
                             sizeof(std::vector<PreparedShapeDefinitionPtr>)) +
                MapBytes(definitionsByContent_.bucket_count(), definitionsByContent_.size(),
                         sizeof(std::uint64_t) +
                             sizeof(std::vector<PreparedShapeDefinitionPtr>)) +
                MapBytes(definitionSlots_.bucket_count(), definitionSlots_.size(),
                         sizeof(const PreparedShapeDefinition*) + sizeof(std::size_t)) +
                MapBytes(flattened_.bucket_count(), flattened_.size(),
                         sizeof(FlattenKey) + sizeof(std::vector<FlattenEntry>)) +
                MapBytes(nfp_.bucket_count(), nfp_.size(),
                         sizeof(std::uint64_t) + sizeof(std::vector<NfpEntry>)) +
                MapBytes(nfpPrepared_.bucket_count(), nfpPrepared_.size(),
                         sizeof(std::uint64_t) + sizeof(std::vector<NfpPreparedEntry>)) +
                MapBytes(audit.candidateBuckets, audit.candidateEntries,
                         sizeof(std::uint64_t) + sizeof(std::vector<std::size_t>)) +
                audit.candidateNestedBytes +
                MapBytes(audit.aliasBuckets, audit.aliasEntries,
                         sizeof(const PreparedShapeDefinition*) + sizeof(NfpDefinitionAlias)) +
                definitionOrder_.capacity() * sizeof(DefinitionRecord) +
                nfpGeometries_.capacity() * sizeof(NfpGeometryRecord);
            return audit;
        }
    };
}
