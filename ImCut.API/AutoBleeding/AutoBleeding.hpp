#pragma once

#include "..\Global.hpp"


#define IMCUT_AUTOBLEEDING_API_VERSION 2

namespace ImCut::AutoBleeding
{
    enum class HiddenObjectAction
    {
        MoveToLayer,
        LeaveInPlace,
        MoveAside
    };

    enum class HeavyShapeDecision
    {
        ProcessOnce,
        ProcessAll,
        SkipAll
    };

    struct Settings
    {
        double bleedMillimeters = 3.0;
        double bleedTimeBudgetSeconds = 20.0;
        double hiddenTimeBudgetSeconds = 45.0;
        double checkpointSeconds = 45.0;
        long heavyNodeLimit = 1500;
        long heavySubpathLimit = 40;
        bool flattenGroups = false;
        bool detectHidden = false;
        bool createCutline = true;
        bool allowRectangleCutlineFallback = false;
        bool showFinalSummary = true;
        HiddenObjectAction hiddenAction = HiddenObjectAction::MoveAside;
    };

    struct ProgressInfo
    {
        long current = 0;
        long total = 0;
        std::string message;
    };

    struct HeavyShapeInfo
    {
        long current = 0;
        long total = 0;
        long subpathCount = 0;
        long nodeCount = 0;
    };

    struct Result
    {
        long totalCount = 0;
        long processedCount = 0;
        long hiddenCount = 0;
        long successCount = 0;
        long failedCount = 0;
        long fallbackCount = 0;
        long skippedCount = 0;
        bool userAborted = false;
        std::string cacheReport;
        std::string fatalError;
        double elapsedMs = 0.0;
        double preparationMs = 0.0;
        double cacheMs = 0.0;
        double generationMs = 0.0;
        double validationMs = 0.0;
        double cutlineMs = 0.0;
        long generationAttempts = 0;
    };

    struct PerformanceStats
    {
        double preparationMs = 0.0;
        double cacheMs = 0.0;
        double generationMs = 0.0;
        double validationMs = 0.0;
        long generationAttempts = 0;
    };

    struct Callbacks
    {
        std::function<void(const ProgressInfo&)> onProgress;
        std::function<HeavyShapeDecision(const HeavyShapeInfo&)> onHeavyShape;
        std::function<bool(long processed, long total)> onCheckpoint;
    };

    namespace Detail
    {
        namespace Constants
        {
            inline constexpr double ContourMiterLimit = 15.0;
            inline constexpr double GrowthMinRatio = 0.95;
            inline constexpr double GrowthMaxRatio = 4.0;
            inline constexpr double AdaptiveRetryStepMm = 0.25;
            inline constexpr double AdaptiveRetryMaxExtraMm = 5.0;
            inline constexpr double LeftoverTolerance = 0.002;
            inline constexpr long BandBatchNodes = 250;
            inline constexpr long BandHeavyNodes = 100;
            inline constexpr long BandHeavySteps = 8;
            inline constexpr const char* EmergencyLayerName = "AutoBleeding - Recuperacao";
            inline constexpr std::size_t CacheMaxEntries = 250;
            inline constexpr long CacheSignatureSubpathDetail = 40;
            inline constexpr long CacheModeMove = 1;
            inline constexpr long CacheModeFlipH = 2;
            inline constexpr long StrategyCount = 8;
            inline constexpr const char* HiddenLayerName = "AutoBleeding - Ocultos";
            inline constexpr double MoveDistanceMm = 3000.0;
            inline constexpr double BboxToleranceMm = 0.025;
            inline constexpr double PointToleranceMm = 0.06;
            inline constexpr double AreaToleranceMm2 = 0.02;
            inline constexpr double OutlineRelevantMm = 0.15;
            inline constexpr long CoarseGrid = 9;
            inline constexpr long FineGrid = 23;
            inline constexpr long CoarseHalton = 192;
            inline constexpr long FineHalton = 1536;
            inline constexpr long BoundarySamplesMin = 64;
            inline constexpr long BoundarySamplesMax = 512;
            inline constexpr long MinConfidentSamples = 48;
            inline constexpr const char* CutContourSpotId = "13ed2790-c966-11de-8a39-0800200c9a66";
            inline constexpr double CutContourWidthMm = 0.1;
            inline constexpr double BleedOutlineWidthMm = 0.05;
        }

        void PumpMessages();

        struct Bounds
        {
            double left = 0.0;
            double right = 0.0;
            double bottom = 0.0;
            double top = 0.0;
        };

        struct CacheEntry
        {
            std::string signature;
            IVGCurvePtr sourceCurve;
            IVGCurvePtr resultCurve;
            cdrFillMode fillMode{};
        };

        struct HiddenSnapshot
        {
            IVGShapePtr shape;
            long staticId = 0;
            long pageIndex = 0;
            Bounds bounds;
            double area = 0.0;
            long zOrder = 0;
            bool visible = false;
            bool opaque = false;
            bool solid = false;
            bool visibilityInspected = false;
            bool opacityInspected = false;
        };

        class Deadline final
        {
        public:
            void Start(double seconds) noexcept;
            [[nodiscard]] bool Exceeded() const noexcept;

        private:
            std::chrono::steady_clock::time_point end_{};
            bool active_ = false;
        };

        class CorelExecutionGuard final
        {
        public:
            CorelExecutionGuard(IVGApplicationPtr app, IVGDocumentPtr doc, const char* commandName);
            ~CorelExecutionGuard() noexcept;

            CorelExecutionGuard(const CorelExecutionGuard&) = delete;
            CorelExecutionGuard& operator=(const CorelExecutionGuard&) = delete;

            void RestoreEventsTemporarily();
            void DisableEventsAgain();
            [[nodiscard]] VARIANT_BOOL PreviousEventsEnabled() const noexcept;

        private:
            void Restore() noexcept;
            IVGApplicationPtr app_;
            IVGDocumentPtr doc_;
            VARIANT_BOOL previousOptimization_ = VARIANT_FALSE;
            VARIANT_BOOL previousEventsEnabled_ = VARIANT_TRUE;
            bool commandGroupStarted_ = false;
            bool restored_ = false;
        };

        [[nodiscard]] IVGCurvePtr CurveCopy(const IVGShapePtr& shape);
        void TraceBleed(const std::string& message) noexcept;
        [[nodiscard]] double ClosedArea(const IVGShapePtr& shape);
        [[nodiscard]] double BoxSpan(const IVGShapePtr& shape);
        [[nodiscard]] double Mm(const IVGApplicationPtr& app, double millimeters);
        [[nodiscard]] bool HasPowerClip(const IVGShapePtr& shape);
        void DeleteShape(IVGShapePtr& shape) noexcept;
        void DeleteRange(IVGShapeRangePtr& range) noexcept;
        [[nodiscard]] std::string ComErrorText(const _com_error& error);
        [[nodiscard]] bool MeasureShape(const IVGShapePtr& shape, Bounds& bounds) noexcept;
        [[nodiscard]] std::string FormatDistance(const IVGApplicationPtr& app, double value);
        [[nodiscard]] std::string Narrow(const _bstr_t& value);
    }

    class UniversalBleedOffset final
    {
    public:
        explicit UniversalBleedOffset(IVGApplicationPtr app);

        void SetTimeBudgetSeconds(double value) noexcept;
        [[nodiscard]] double TimeBudgetSeconds() const noexcept;
        void SetQuickRetry(bool value) noexcept;
        void SetAdaptiveDistanceAvailable(bool value) noexcept;

        void BeginBatch();
        void EndBatch();

        [[nodiscard]] IVGShapePtr CreateOffsetShape(const IVGShapePtr& baseShape, double distance);
        [[nodiscard]] const std::string& GetLastDiagnostic() const noexcept;
        [[nodiscard]] const std::string& GetLastFailureDetail() const noexcept;
        [[nodiscard]] std::string GetCacheStats() const;
        [[nodiscard]] const PerformanceStats& GetPerformanceStats() const noexcept;
        [[nodiscard]] IVGCurvePtr PreparedClosedCurve() const noexcept;
        [[nodiscard]] bool QuickRetryRecommended() const noexcept;

    private:

        [[nodiscard]] double MinimumGrowth(
            const IVGShapePtr& shape,
            const Detail::Bounds& sourceBounds) const;

        [[nodiscard]] IVGShapePtr DilateByContour(
            IVGShapePtr& regionShape,
            double distance,
            const Detail::Bounds& sourceBounds,
            const IVGLayerPtr& targetLayer,
            std::string& reason);

        [[nodiscard]] IVGShapePtr DilateByBand(
            IVGShapePtr& regionShape,
            double distance,
            double regionArea,
            const IVGLayerPtr& targetLayer,
            std::string& reason);

        [[nodiscard]] IVGShapePtr DilateOneStep(
            const IVGShapePtr& sourceShape,
            double stepDistance,
            const IVGLayerPtr& targetLayer,
            long stepIndex,
            long stepTotal,
            std::string& reason);

        [[nodiscard]] long MaxSubPathNodes(const IVGShapePtr& shape) const;
        [[nodiscard]] IVGCurvePtr NewBoundCurve() const;
        [[nodiscard]] IVGShapePtr WeldPair(IVGShapePtr shapeA, IVGShapePtr shapeB) const;
        [[nodiscard]] IVGShapePtr BuildBandRaw(
            const IVGCurvePtr& boundaryCurve,
            double distance,
            const IVGLayerPtr& targetLayer) const;

        [[nodiscard]] IVGShapePtr DilateOpenGeometry(
            const IVGCurvePtr& sourceCurve,
            double distance,
            const IVGLayerPtr& targetLayer,
            std::string& reason) const;

        [[nodiscard]] IVGShapePtr BuildRegionShape(
            const IVGShapePtr& baseShape,
            const IVGCurvePtr& closedCurve,
            const IVGLayerPtr& targetLayer,
            bool& usedCurvePath) const;

        [[nodiscard]] IVGShapePtr BuildRegionFromDuplicate(const IVGShapePtr& baseShape) const;
        [[nodiscard]] IVGShapePtr BuildRegionFromGroup(
            const IVGShapePtr& baseShape,
            const IVGLayerPtr& targetLayer) const;

        [[nodiscard]] IVGShapeRangePtr FlattenGroupRange(const IVGShapePtr& groupShape) const;
        [[nodiscard]] IVGShapeRangePtr FlattenRange(const IVGShapeRangePtr& source) const;
        [[nodiscard]] IVGShapePtr WeldRange(const IVGShapeRangePtr& members) const;

        [[nodiscard]] bool ValidateDilation(
            const Detail::Bounds& sourceBounds,
            const IVGShapePtr& regionProbe,
            const IVGShapePtr& resultShape,
            double distance,
            std::string& reason) const;

        [[nodiscard]] bool ValidateCachedDilation(
            const Detail::Bounds& sourceBounds,
            const IVGShapePtr& resultShape,
            double distance,
            std::string& reason) const;

        [[nodiscard]] bool RegionFitsInside(
            const IVGShapePtr& regionProbe,
            const IVGShapePtr& container,
            std::string& reason) const;

        [[nodiscard]] long CountClosedSubPaths(const IVGShapePtr& shape) const;
        [[nodiscard]] bool RemoveUnexpectedInternalSubpaths(
            const IVGShapePtr& regionProbe,
            IVGShapePtr& resultShape,
            const IVGLayerPtr& targetLayer,
            double distance,
            std::string& reason) const;

        [[nodiscard]] bool FindSubPathInteriorPoint(
            const IVGSubPathPtr& subPath,
            double tolerance,
            double& x,
            double& y) const;

        [[nodiscard]] std::vector<long> FindUnexpectedInternalSubpaths(
            const IVGShapePtr& regionProbe,
            const IVGShapePtr& resultShape,
            double distance) const;

        [[nodiscard]] bool TopologyChecksPass(
            const IVGShapePtr& regionProbe,
            const IVGShapePtr& resultShape,
            double distance,
            std::string& reason) const;

        [[nodiscard]] std::string SideName(long sideIndex) const;
        void ClearCache();

        [[nodiscard]] IVGShapePtr TryReuseFromCache(
            const std::string& signature,
            const IVGCurvePtr& sourceCurve,
            double distance,
            const IVGLayerPtr& targetLayer);

        void StoreInCache(
            const std::string& signature,
            const IVGCurvePtr& sourceCurve,
            const IVGShapePtr& resultShape,
            double distance);

        [[nodiscard]] bool ApplyCacheTransform(
            const IVGShapePtr& shape,
            long mode,
            double p1,
            double p2) const;

        [[nodiscard]] std::string CurveSignature(
            const IVGCurvePtr& curve,
            double distance) const;

        [[nodiscard]] bool CurvesMatchByIsometry(
            const IVGCurvePtr& cachedCurve,
            const IVGCurvePtr& newCurve,
            long& mode,
            double& p1,
            double& p2) const;

        [[nodiscard]] IVGCurvePtr ExtractClosedSubPaths(
            const IVGCurvePtr& sourceCurve,
            long& closedCount) const;

        void MirrorShape(const IVGShapePtr& shape) const;
        [[nodiscard]] std::string StrategyName(long attempt) const;
        [[nodiscard]] IVGCurvePtr ReverseCurveDirection(const IVGCurvePtr& source) const;
        [[nodiscard]] bool IsGroupShape(const IVGShapePtr& shape) const;
        [[nodiscard]] IVGLayerPtr ResolveWritableLayer(const IVGShapePtr& baseShape) const;
        [[nodiscard]] IVGLayerPtr GetOrCreateEmergencyLayer(const IVGShapePtr& baseShape) const;
        void ForceNoFill(const IVGShapePtr& shape) const;

        IVGApplicationPtr app_;
        Detail::Deadline deadline_;
        double timeBudgetSeconds_ = 20.0;
        bool quickRetry_ = false;
        bool quickRetryRecommended_ = false;
        bool adaptiveDistanceAvailable_ = false;
        bool lastFailureDistanceLimited_ = false;
        bool cacheEnabled_ = false;
        long preferredStrategy_ = 0;
        long cacheHits_ = 0;
        long cacheMisses_ = 0;
        std::string lastDiagnostic_;
        std::string lastStrategy_;
        PerformanceStats performance_;
        IVGCurvePtr preparedClosedCurve_;
        std::vector<Detail::CacheEntry> cache_;
        std::unordered_map<std::string, std::size_t> cacheIndex_;
    };

    class HiddenObjectDetector final
    {
    public:
        explicit HiddenObjectDetector(IVGApplicationPtr app);

        void SetAction(HiddenObjectAction value) noexcept;
        [[nodiscard]] HiddenObjectAction Action() const noexcept;
        void SetTimeBudgetSeconds(double value) noexcept;
        [[nodiscard]] double TimeBudgetSeconds() const noexcept;
        [[nodiscard]] const std::string& GetLastReport() const noexcept;

        void Process(IVGShapeRangePtr& range, long& hiddenCount);

    private:
        [[nodiscard]] bool SnapshotSelection(const IVGShapeRangePtr& range);
        void CalibrateZOrderDirection();
        [[nodiscard]] IVGShapePtr ShapeAt(std::size_t index);
        void EnsureVisibility(std::size_t index);
        void EnsureOpacity(std::size_t index);
        [[nodiscard]] std::vector<std::size_t> BuildCandidateList(std::size_t targetIndex, double bboxTolerance);
        [[nodiscard]] bool IsAbove(std::size_t candidateIndex, std::size_t targetIndex);
        [[nodiscard]] bool IsShapeHidden(
            std::size_t targetIndex,
            const std::vector<std::size_t>& candidates,
            double bboxTolerance,
            double pointTolerance,
            double areaTolerance);

        [[nodiscard]] int BooleanCoverage(
            std::size_t targetIndex,
            const std::vector<std::size_t>& candidates,
            double areaTolerance);

        [[nodiscard]] IVGShapePtr BuildFootprint(std::size_t targetIndex);
        [[nodiscard]] IVGShapePtr ExpandByOutlineWeld(IVGShapePtr& baseShape, double offsetValue);
        [[nodiscard]] bool SampledCoverage(
            std::size_t targetIndex,
            const std::vector<std::size_t>& candidates,
            double tolerance);

        [[nodiscard]] bool SamplePass(
            const IVGShapePtr& targetShape,
            const std::vector<IVGShapePtr>& candidateShapes,
            double tolerance,
            long gridSize,
            long haltonCount,
            bool includeBoundary,
            long& tested);

        [[nodiscard]] std::vector<IVGShapePtr> ResolveCandidateShapes(
            const std::vector<std::size_t>& candidates);

        [[nodiscard]] bool PointCoveredByAny(
            const std::vector<IVGShapePtr>& candidateShapes,
            double x,
            double y,
            double tolerance);

        [[nodiscard]] bool PointBelongsToShape(
            const IVGShapePtr& shape,
            double x,
            double y,
            double tolerance);

        [[nodiscard]] bool PointCoveredByShape(
            const IVGShapePtr& shape,
            double x,
            double y,
            double tolerance);

        [[nodiscard]] bool HasExactDuplicateAbove(
            std::size_t targetIndex,
            const std::vector<std::size_t>& candidates,
            double bboxTolerance,
            double pointTolerance);

        [[nodiscard]] bool CurvesEquivalent(
            const IVGShapePtr& firstShape,
            const IVGShapePtr& secondShape,
            double tolerance);

        [[nodiscard]] bool SubPathsEquivalent(
            const IVGSubPathPtr& firstSubPath,
            const IVGSubPathPtr& secondSubPath,
            double tolerance);

        [[nodiscard]] bool SubPathSamplesMatch(
            const IVGSubPathPtr& sourceSubPath,
            const IVGSubPathPtr& testSubPath,
            double tolerance);

        [[nodiscard]] bool ShapeHasSolidInterior(const IVGShapePtr& shape);
        [[nodiscard]] bool ShapeIsOpaque(const IVGShapePtr& shape);
        void ApplyHiddenAction(const IVGShapeRangePtr& hiddenRange);
        [[nodiscard]] IVGLayerPtr GetOrCreateHiddenLayer();
        [[nodiscard]] bool BoxesIntersect(std::size_t a, std::size_t b, double tolerance) const noexcept;
        [[nodiscard]] bool BoxesEqual(std::size_t a, std::size_t b, double tolerance) const noexcept;
        [[nodiscard]] long BoundarySampleCount(const IVGSubPathPtr& subPath) const;
        [[nodiscard]] static double Halton(long index, long base) noexcept;
        void ClearContext();

        IVGApplicationPtr app_;
        IVGDocumentPtr doc_;
        std::vector<Detail::HiddenSnapshot> shapes_;
        Detail::Deadline deadline_;
        double timeBudgetSeconds_ = 45.0;
        HiddenObjectAction action_ = HiddenObjectAction::MoveAside;
        bool zOrderUsable_ = false;
        bool zOrderFrontIsHigher_ = false;
        std::string lastReport_;
    };

    class Processor final
    {
    public:
        explicit Processor(IVGApplicationPtr app);

        [[nodiscard]] Result Run(
            const Settings& settings,
            const Callbacks& callbacks = {});

    private:
        struct GroupBleedSources
        {
            std::vector<IVGShapePtr> geometrySources;
            IVGShapePtr appearanceSource;
            bool usesExistingCutline = false;
        };

        struct PendingPowerClipFinalize
        {
            IVGShapePtr sourceShape;
            IVGShapePtr generatedPowerClip;
            IVGShapePtr generatedCutline;
            bool preserveGroup = false;
            long itemNumber = 0;
            std::string strategyNote;
        };

        [[nodiscard]] IVGShapeRangePtr FlattenSelectedGroups(const IVGShapeRangePtr& sourceRange) const;
        [[nodiscard]] bool IsGroupShape(const IVGShapePtr& shape) const;
        [[nodiscard]] GroupBleedSources ResolveGroupBleedSources(const IVGShapePtr& groupShape) const;
        [[nodiscard]] IVGShapePtr CreateStandaloneGeometryProxy(
            const IVGShapePtr& sourceShape,
            const IVGLayerPtr& targetLayer) const;
        [[nodiscard]] IVGShapePtr CreateStandaloneGeometryProxy(
            const std::vector<IVGShapePtr>& sourceShapes,
            const IVGLayerPtr& targetLayer) const;
        [[nodiscard]] IVGShapePtr CreateDetachedDuplicate(
            const IVGShapePtr& sourceShape,
            const IVGLayerPtr& targetLayer) const;
        [[nodiscard]] IVGCurvePtr ExtractExteriorCutlineCurve(
            const IVGCurvePtr& sourceCurve) const;
        [[nodiscard]] bool CopyPowerClipContentsSafely(
            const IVGShapePtr& sourcePowerClip,
            const IVGShapePtr& targetPowerClip,
            const IVGLayerPtr& targetLayer,
            long& contentCount,
            std::string& diagnostic) const;
        [[nodiscard]] bool NormalizePowerClipContents(
            const IVGShapePtr& container,
            std::string& diagnostic) const;
        void RollbackPendingPowerClip(
            PendingPowerClipFinalize& pending) noexcept;
        void FindDominantGroupArtworkShapeRecursive(
            const IVGShapePtr& shape,
            IVGShapePtr& bestPrimary,
            double& bestPrimaryScore,
            IVGShapePtr& bestSecondary,
            double& bestSecondaryScore) const;
        void CollectGroupArtworkShapesRecursive(
            const IVGShapePtr& shape,
            std::vector<IVGShapePtr>& primary,
            std::vector<IVGShapePtr>& secondary) const;

        [[nodiscard]] bool SkipHeavyShape(
            const IVGShapePtr& shape,
            long itemNumber,
            long totalCount,
            const Settings& settings,
            const Callbacks& callbacks,
            Detail::CorelExecutionGuard& guard);

        [[nodiscard]] bool ProcessUniversalBleedShape(
            const IVGShapePtr& sourceShape,
            double bleedDistance,
            long itemNumber,
            UniversalBleedOffset& bleedEngine,
            const Settings& settings,
            IVGShapePtr& resultGroup,
            std::string& errorDescription,
            std::string& strategyNote);

        [[nodiscard]] IVGShapePtr CreateUniversalCutline(
            const IVGShapePtr& sourceShape,
            long itemNumber,
            IVGLayerPtr targetLayer,
            const Settings& settings,
            const IVGCurvePtr& preparedClosedCurve = nullptr) const;

        void ApplyBestBleedAppearance(
            const IVGShapePtr& sourceShape,
            const IVGShapePtr& bleedShape) const;

        [[nodiscard]] IVGShapePtr FindFillSource(const IVGShapePtr& sourceShape) const;
        void Progress(const Callbacks& callbacks, long current, long total, const std::string& message) const;

        IVGApplicationPtr app_;
        HeavyShapeDecision heavyDecision_ = HeavyShapeDecision::ProcessOnce;
        std::vector<PendingPowerClipFinalize> pendingPowerClipFinalizations_;
        double cutlineMs_ = 0.0;
    };

    [[nodiscard]] Result Run(
        IVGApplicationPtr& spApp,
        const Settings& settings,
        const Callbacks& callbacks = {});
}
