#define NOMINMAX 1

#include "AutoBleeding.hpp"
#include <cmath>
#include <sstream>
#include "../GeometryCache.hpp"
#include "../InputLimits.hpp"



namespace ImCut::AutoBleeding::Detail
{
    void PumpMessages()
    {
        MSG msg{};
        
        
        while (PeekMessageW(&msg, nullptr, WM_PAINT, WM_PAINT, PM_REMOVE))
        {
            TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }

    void Deadline::Start(double seconds) noexcept
    {
        seconds = std::isfinite(seconds) ? std::clamp(seconds, 0.0, 3600.0) : 20.0;
        if (seconds <= 0.0)
        {
            active_ = false;
            return;
        }

        end_ = std::chrono::steady_clock::now() +
            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                std::chrono::duration<double>(seconds));
        active_ = true;
    }

    bool Deadline::Exceeded() const noexcept
    {
        return active_ && std::chrono::steady_clock::now() > end_;
    }

    CorelExecutionGuard::CorelExecutionGuard(
        IVGApplicationPtr app,
        IVGDocumentPtr doc,
        const char* commandName)
        : app_(std::move(app)),
        doc_(std::move(doc))
    {
        previousOptimization_ = app_->GetOptimization();
        previousEventsEnabled_ = app_->GetEventsEnabled();
        try
        {
            doc_->BeginCommandGroup(commandName);
            commandGroupStarted_ = true;
            app_->PutOptimization(VARIANT_TRUE);
            app_->PutEventsEnabled(VARIANT_FALSE);
        }
        catch (...)
        {
            Restore();
            throw;
        }
    }

    CorelExecutionGuard::~CorelExecutionGuard() noexcept { Restore(); }

    void CorelExecutionGuard::Restore() noexcept
    {
        if (restored_)
            return;

        try
        {
            if (commandGroupStarted_)
                doc_->EndCommandGroup();
        }
        catch (...)
        {
        }

        try
        {
            app_->PutEventsEnabled(previousEventsEnabled_);
        }
        catch (...)
        {
        }

        try
        {
            app_->PutOptimization(previousOptimization_);
        }
        catch (...)
        {
        }

        restored_ = true;
    }

    void CorelExecutionGuard::RestoreEventsTemporarily()
    {
        app_->PutEventsEnabled(previousEventsEnabled_);
    }

    void CorelExecutionGuard::DisableEventsAgain()
    {
        app_->PutEventsEnabled(VARIANT_FALSE);
    }

    VARIANT_BOOL CorelExecutionGuard::PreviousEventsEnabled() const noexcept
    {
        return previousEventsEnabled_;
    }

    IVGCurvePtr CurveCopy(const IVGShapePtr& shape)
    {
        if (!shape)
            return nullptr;

        try
        {
            auto display = shape->DisplayCurve;
            return display ? display->GetCopy() : nullptr;
        }
        catch (...)
        {
            return nullptr;
        }
    }

    void TraceBleed(const std::string& message) noexcept
    {
        std::string line = "[ImCut.AutoBleeding] " + message + "\r\n";
        OutputDebugStringA(line.c_str());
    }

    double ClosedArea(const IVGShapePtr& shape)
    {
        auto curve = CurveCopy(shape);
        if (!curve)
            return 0.0;

        double total = 0.0;

        try
        {
            auto subPaths = curve->SubPaths;
            const long count = subPaths->Count;

            for (long i = 1; i <= count; ++i)
            {
                auto subPath = subPaths->Item[i];
                if (subPath && subPath->Closed)
                    total += std::abs(subPath->Area);
            }
        }
        catch (...)
        {
        }

        return total;
    }

    double BoxSpan(const IVGShapePtr& shape)
    {
        Bounds bounds;
        if (!MeasureShape(shape, bounds))
            return 0.0;
        return (bounds.right - bounds.left) + (bounds.top - bounds.bottom);
    }

    double Mm(const IVGApplicationPtr& app, double millimeters)
    {
        if (!app || !app->ActiveDocument)
            return 0.0;

        try
        {
            return app->ConvertUnits(
                millimeters,
                cdrMillimeter,
                app->ActiveDocument->Unit);
        }
        catch (...)
        {
            return 0.0;
        }
    }

    bool HasPowerClip(const IVGShapePtr& shape)
    {
        if (!shape)
            return false;

        try
        {
            return shape->PowerClip != nullptr;
        }
        catch (...)
        {
            return false;
        }
    }

    void DeleteShape(IVGShapePtr& shape) noexcept
    {
        try
        {
            if (shape)
                shape->Delete();
        }
        catch (...)
        {
        }
        shape = nullptr;
    }

    void DeleteRange(IVGShapeRangePtr& range) noexcept
    {
        try
        {
            if (range)
                range->Delete();
        }
        catch (...)
        {
        }
        range = nullptr;
    }

    std::string ComErrorText(const _com_error& error)
    {
        try
        {
            _bstr_t description = error.Description();
            if (description.length() > 0)
                return Narrow(description);
        }
        catch (...)
        {
        }

        return "CorelDRAW COM error";
    }

    bool MeasureShape(const IVGShapePtr& shape, Bounds& bounds) noexcept
    {
        if (!shape)
            return false;

        try
        {
            bounds.left = shape->LeftX;
            bounds.right = shape->RightX;
            bounds.bottom = shape->BottomY;
            bounds.top = shape->TopY;
            return bounds.right >= bounds.left && bounds.top >= bounds.bottom;
        }
        catch (...)
        {
            return false;
        }
    }

    std::string FormatDistance(const IVGApplicationPtr& app, double value)
    {
        std::ostringstream stream;
        stream << std::fixed << std::setprecision(3);

        try
        {
            const double mm = app->ConvertUnits(
                value,
                app->ActiveDocument->Unit,
                cdrMillimeter);
            stream << mm << " mm";
        }
        catch (...)
        {
            stream << value;
        }

        return stream.str();
    }

    std::string Narrow(const _bstr_t& value)
    {
        const char* text = static_cast<const char*>(value);
        return text ? text : "";
    }
}

namespace ImCut::AutoBleeding
{
    using namespace Detail::Constants;

    namespace
    {
        [[nodiscard]] double ElapsedMilliseconds(
            std::chrono::steady_clock::time_point start) noexcept
        {
            return std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();
        }
    }

    UniversalBleedOffset::UniversalBleedOffset(IVGApplicationPtr app)
        : app_(std::move(app))
    {
    }

    void UniversalBleedOffset::SetTimeBudgetSeconds(double value) noexcept
    {
        timeBudgetSeconds_ = value;
    }

    double UniversalBleedOffset::TimeBudgetSeconds() const noexcept
    {
        return timeBudgetSeconds_ > 0.0 ? timeBudgetSeconds_ : 20.0;
    }

    void UniversalBleedOffset::SetQuickRetry(bool value) noexcept
    {
        quickRetry_ = value;
    }

    void UniversalBleedOffset::SetAdaptiveDistanceAvailable(bool value) noexcept
    {
        adaptiveDistanceAvailable_ = value;
    }

    void UniversalBleedOffset::BeginBatch()
    {
        ClearCache();
        cacheEnabled_ = true;
        cacheHits_ = 0;
        cacheMisses_ = 0;
        
        
        preferredStrategy_ = 2;
        performance_ = {};
        preparedClosedCurve_ = nullptr;
    }

    void UniversalBleedOffset::EndBatch()
    {
        cacheEnabled_ = false;
        ClearCache();
    }

    const std::string& UniversalBleedOffset::GetLastDiagnostic() const noexcept
    {
        return lastStrategy_;
    }

    const std::string& UniversalBleedOffset::GetLastFailureDetail() const noexcept
    {
        return lastDiagnostic_;
    }

    std::string UniversalBleedOffset::GetCacheStats() const
    {
        const long total = cacheHits_ + cacheMisses_;
        if (total == 0)
            return {};
        return std::to_string(cacheHits_) + " de " + std::to_string(total) + " reaproveitados do cache";
    }

    const PerformanceStats& UniversalBleedOffset::GetPerformanceStats() const noexcept
    {
        return performance_;
    }

    IVGCurvePtr UniversalBleedOffset::PreparedClosedCurve() const noexcept
    {
        return preparedClosedCurve_;
    }

    bool UniversalBleedOffset::QuickRetryRecommended() const noexcept
    {
        return quickRetryRecommended_;
    }




    IVGShapePtr UniversalBleedOffset::CreateOffsetShape(
        const IVGShapePtr& baseShape,
        double distance)
    {
        const auto preparationStart = std::chrono::steady_clock::now();
        lastDiagnostic_.clear();
        lastStrategy_.clear();
        preparedClosedCurve_ = nullptr;
        quickRetryRecommended_ = false;

        if (!baseShape)
            return nullptr;

        if (!std::isfinite(distance) || distance <= 0.0)
            throw std::runtime_error("A distancia da sangria deve ser maior que zero.");

        deadline_.Start(TimeBudgetSeconds());

        auto targetLayer = ResolveWritableLayer(baseShape);

        if (!targetLayer)
            throw std::runtime_error("Nenhuma camada editavel disponivel para a sangria.");

        auto sourceCurve = Detail::CurveCopy(baseShape);

        if (!sourceCurve)
            throw std::runtime_error("Nao foi possivel obter a curva visual do objeto.");

        const std::string signature = cacheEnabled_
            ? CurveSignature(sourceCurve, distance) + ";fill=" + std::to_string(baseShape->FillMode)
            : std::string{};
        IVGShapePtr resultShape;

        long closedCount = 0;
        auto closedCurve = ExtractClosedSubPaths(sourceCurve, closedCount);
        preparedClosedCurve_ = closedCurve;
        std::string reason;

        if (closedCount == 0)
        {
            resultShape = DilateOpenGeometry(
                sourceCurve,
                distance,
                targetLayer,
                reason);

            if (!resultShape)
            {
                lastDiagnostic_ = reason;

                throw std::runtime_error(
                    "Nao foi possivel gerar a sangria desta curva aberta.\n"
                    "Diagnostico: " + reason);
            }

            lastStrategy_ = "curva aberta (faixa de traco)";

            ForceNoFill(resultShape);
            return resultShape;
        }

        bool usedCurvePath = false;

        auto regionMaster = BuildRegionShape(
            baseShape,
            closedCurve,
            targetLayer,
            usedCurvePath);

        if (!regionMaster)
        {
            lastDiagnostic_ = "nao foi possivel montar a regiao solida";

            throw std::runtime_error(
                "Nao foi possivel montar a regiao solida do objeto.");
        }

        Detail::Bounds sourceBounds;

        if (!Detail::MeasureShape(regionMaster, sourceBounds))
        {
            Detail::DeleteShape(regionMaster);
            lastDiagnostic_ = "nao foi possivel medir a regiao";

            throw std::runtime_error(
                "Nao foi possivel medir a regiao solida do objeto.");
        }

        performance_.preparationMs += ElapsedMilliseconds(preparationStart);

        const auto cacheStart = std::chrono::steady_clock::now();
        resultShape = TryReuseFromCache(signature, sourceCurve, distance, targetLayer);
        performance_.cacheMs += ElapsedMilliseconds(cacheStart);
        if (resultShape)
        {
            const auto validationStart = std::chrono::steady_clock::now();
            if (ValidateCachedDilation(sourceBounds, resultShape, distance, reason))
            {
                performance_.validationMs += ElapsedMilliseconds(validationStart);
                ++cacheHits_;
                lastStrategy_ = "cache validado";
                Detail::DeleteShape(regionMaster);
                return resultShape;
            }
            performance_.validationMs += ElapsedMilliseconds(validationStart);
            Detail::DeleteShape(resultShape);
        }

        std::string history;

        std::array<long, StrategyCount> attemptOrder{};
        long attemptCount = 0;
        const auto addAttempt = [&](long attempt)
        {
            if (attempt < 1 || attempt > StrategyCount)
                return;
            if (std::find(attemptOrder.begin(), attemptOrder.begin() + attemptCount,
                    attempt) != attemptOrder.begin() + attemptCount)
                return;
            attemptOrder[attemptCount++] = attempt;
        };

        addAttempt(preferredStrategy_);
        if (!quickRetry_)
        {
            addAttempt(2); 
            addAttempt(1); 
            for (long attempt = 3; attempt <= StrategyCount; ++attempt)
                addAttempt(attempt);
        }

        bool nativeDistanceLimited = false;
        for (long pass = 0; pass < attemptCount; ++pass)
        {
            const long attempt = attemptOrder[pass];
            lastFailureDistanceLimited_ = false;

            if (deadline_.Exceeded())
            {
                history += " | orcamento de tempo esgotado";
                break;
            }

            Detail::PumpMessages();

            const long variant = ((attempt - 1) / 2) + 1;
            const long method = ((attempt - 1) % 2) + 1;
            const bool flipAxis = variant == 3 || variant == 4;

            IVGShapePtr regionShape;

            if (variant == 2 || variant == 4)
            {
                if (!usedCurvePath)
                    continue;

                auto workCurve = ReverseCurveDirection(closedCurve);

                if (!workCurve)
                    continue;

                regionShape = BuildRegionShape(
                    baseShape,
                    workCurve,
                    targetLayer,
                    usedCurvePath);
            }
            else
            {
                try
                {
                    regionShape = regionMaster->Duplicate(0.0, 0.0);
                }
                catch (...)
                {
                    regionShape = nullptr;
                }
            }

            if (!regionShape)
                continue;

            if (flipAxis)
                MirrorShape(regionShape);

            if (method == 1)
            {
                const auto generationStart = std::chrono::steady_clock::now();
                ++performance_.generationAttempts;
                resultShape = DilateByBand(
                    regionShape,
                    distance,
                    0.0,
                    targetLayer,
                    reason);
                performance_.generationMs += ElapsedMilliseconds(generationStart);
            }
            else
            {
                const auto generationStart = std::chrono::steady_clock::now();
                ++performance_.generationAttempts;
                resultShape = DilateByContour(
                    regionShape,
                    distance,
                    sourceBounds,
                    targetLayer,
                    reason);
                performance_.generationMs += ElapsedMilliseconds(generationStart);
            }

            regionShape = nullptr;

            if (resultShape)
            {
                if (flipAxis)
                    MirrorShape(resultShape);

                const auto validationStart = std::chrono::steady_clock::now();
                bool valid = ValidateDilation(
                    sourceBounds,
                    regionMaster,
                    resultShape,
                    distance,
                    reason);

                if (!valid)
                {
                    const auto unexpected = FindUnexpectedInternalSubpaths(
                        regionMaster,
                        resultShape,
                        distance);
                    long maxSourceNodes = 0;
                    if (method == 2 && unexpected.size() > 1)
                    {
                        try
                        {
                            IVGCurvePtr sourceCurve;
                            if (regionMaster->Type == cdrCurveShape)
                                sourceCurve = regionMaster->Curve;
                            if (!sourceCurve)
                                sourceCurve = regionMaster->DisplayCurve;
                            if (sourceCurve && sourceCurve->SubPaths)
                            {
                                for (long index = 1;
                                    index <= sourceCurve->SubPaths->Count;
                                    ++index)
                                {
                                    auto path = sourceCurve->SubPaths->Item[index];
                                    if (path)
                                        maxSourceNodes = std::max(
                                            maxSourceNodes,
                                            path->Nodes->Count);
                                }
                            }
                        }
                        catch (...) {}
                    }

                    
                    
                    
                    
                    
                    constexpr long DensePathNodes = 128;
                    const bool canClean = !unexpected.empty() &&
                        (method == 1 || unexpected.size() == 1 ||
                            maxSourceNodes >= DensePathNodes);
                    if (canClean && RemoveUnexpectedInternalSubpaths(
                        regionMaster,
                        resultShape,
                        targetLayer,
                        distance,
                        reason))
                    {
                        reason.clear();
                        valid = ValidateDilation(
                            sourceBounds,
                            regionMaster,
                            resultShape,
                            distance,
                            reason);
                    }
                }

                if (valid)
                {
                    performance_.validationMs += ElapsedMilliseconds(validationStart);
                    
                    
                    
                    if (attempt == 2)
                        preferredStrategy_ = attempt;
                    lastStrategy_ = StrategyName(attempt);
                    break;
                }

                performance_.validationMs += ElapsedMilliseconds(validationStart);

                Detail::DeleteShape(resultShape);
            }

            history += " | " + StrategyName(attempt) + ": " + reason;

            if (attempt == 2)
            {
                nativeDistanceLimited = lastFailureDistanceLimited_;
            }
            else if (attempt == 1 && nativeDistanceLimited &&
                lastFailureDistanceLimited_ && adaptiveDistanceAvailable_)
            {
                
                
                
                quickRetryRecommended_ = true;
                break;
            }
        }

        if (quickRetry_ && nativeDistanceLimited && adaptiveDistanceAvailable_)
            quickRetryRecommended_ = true;

        Detail::DeleteShape(regionMaster);

        if (!resultShape)
        {
            lastDiagnostic_ = history;

            throw std::runtime_error(
                "Nao foi possivel gerar a sangria deste objeto.\n"
                "Diagnostico: " + history);
        }

        ForceNoFill(resultShape);
        StoreInCache(signature, sourceCurve, resultShape, distance);

        return resultShape;
    }

    void UniversalBleedOffset::ForceNoFill(const IVGShapePtr& shape) const
    {
        if (!shape)
            return;

        try
        {
            shape->Outline->SetNoOutline();
            shape->Fill->ApplyNoFill();
            if (shape->Fill->Type != cdrNoFill)
            {
                auto white = app_->CreateRGBColor(255, 255, 255);
                shape->Fill->ApplyUniformFill(white);
                shape->Fill->ApplyNoFill();
            }
        }
        catch (...)
        {
        }
    }



    double UniversalBleedOffset::MinimumGrowth(
        const IVGShapePtr& shape,
        const Detail::Bounds& sourceBounds) const
    {
        Detail::Bounds bounds;
        if (!Detail::MeasureShape(shape, bounds))
            return -1e30;

        return std::min({
            sourceBounds.left - bounds.left,
            bounds.right - sourceBounds.right,
            sourceBounds.bottom - bounds.bottom,
            bounds.top - sourceBounds.top
            });
    }



    IVGShapePtr UniversalBleedOffset::DilateByContour(
        IVGShapePtr& regionShape,
        double distance,
        const Detail::Bounds& sourceBounds,
        const IVGLayerPtr&,
        std::string& reason)
    {
        if (!regionShape)
            return nullptr;

        IVGShapeRangePtr parts;
        IVGShapeRangePtr flat;
        IVGShapePtr chosen;

        try
        {
            auto effect = regionShape->CreateContour(cdrContourOutside, distance, 1, cdrDirectFountainFillBlend, nullptr, nullptr, nullptr, 0, 0, cdrContourRoundCap, cdrContourCornerRound, ContourMiterLimit);

            if (!effect)
            {
                reason = "CreateContour nao devolveu um efeito";
                throw std::runtime_error(reason);
            }

            auto contour = effect->Contour;

            if (!contour)
            {
                reason = "CreateContour nao devolveu propriedades de contorno";
                throw std::runtime_error(reason);
            }

            parts = effect->Separate();

            if (!parts)
            {
                reason = "Separate nao devolveu as formas do contorno";
                throw std::runtime_error(reason);
            }

            bool containsGroup = false;
            for (long i = 1; i <= parts->Count; ++i)
            {
                auto shape = parts->Item[i];
                if (shape && shape->Type == cdrGroupShape)
                {
                    containsGroup = true;
                    break;
                }
            }

            flat = containsGroup ? FlattenRange(parts) : parts;

            if (!flat)
            {
                reason = "nao foi possivel achatar o resultado do contorno";
                throw std::runtime_error(reason);
            }

            double bestGrowth = -1e30;

            for (long i = 1; i <= flat->Count; ++i)
            {
                auto shape = flat->Item[i];

                if (!shape)
                    continue;

                const double currentGrowth =
                    MinimumGrowth(shape, sourceBounds);

                if (currentGrowth > bestGrowth)
                {
                    bestGrowth = currentGrowth;
                    chosen = shape;
                }
            }

            if (!chosen)
            {
                reason = "o contorno nao produziu nenhuma forma";
                throw std::runtime_error(reason);
            }

            if (bestGrowth < distance * GrowthMinRatio)
            {
                lastFailureDistanceLimited_ = true;
                reason =
                    "nenhuma forma do contorno cresceu o esperado "
                    "(melhor: " +
                    Detail::FormatDistance(app_, bestGrowth) + ")";

                throw std::runtime_error(reason);
            }

            for (long i = 1; i <= flat->Count; ++i)
            {
                auto shape = flat->Item[i];

                if (!shape)
                    continue;

                if (shape.GetInterfacePtr() != chosen.GetInterfacePtr())
                {
                    try
                    {
                        shape->Delete();
                    }
                    catch (...)
                    {
                    }
                }
            }

            ForceNoFill(chosen);
            regionShape = nullptr;

            return chosen;
        }
        catch (const _com_error& error)
        {
            if (reason.empty())
                reason = "erro COM: " + Detail::ComErrorText(error);
        }
        catch (const std::exception& error)
        {
            if (reason.empty())
                reason = error.what();
        }
        catch (...)
        {
            if (reason.empty())
                reason = "erro desconhecido no contorno";
        }

        if (!flat || !parts || flat.GetInterfacePtr() != parts.GetInterfacePtr())
            Detail::DeleteRange(flat);
        Detail::DeleteRange(parts);
        Detail::DeleteShape(regionShape);

        return nullptr;
    }


    IVGShapePtr UniversalBleedOffset::DilateByBand(
        IVGShapePtr& regionShape,
        double distance,
        double regionArea,
        const IVGLayerPtr& targetLayer,
        std::string& reason)
    {
        (void)regionArea;

        if (!regionShape)
            return nullptr;

        long steps = 1;

        if (MaxSubPathNodes(regionShape) > BandHeavyNodes)
            steps = BandHeavySteps;

        const double stepDistance =
            distance / static_cast<double>(steps);

        Detail::Bounds baseBounds;

        if (!Detail::MeasureShape(regionShape, baseBounds))
        {
            reason = "nao foi possivel medir a regiao antes da faixa";
            Detail::DeleteShape(regionShape);
            return nullptr;
        }

        IVGShapePtr accumulator = regionShape;
        regionShape = nullptr;

        for (long step = 1; step <= steps; ++step)
        {
            auto stepped = DilateOneStep(
                accumulator,
                stepDistance,
                targetLayer,
                step,
                steps,
                reason);

            if (!stepped)
            {
                Detail::DeleteShape(accumulator);
                return nullptr;
            }

            accumulator = stepped;

            const double grown =
                MinimumGrowth(accumulator, baseBounds);

            if (grown <
                stepDistance *
                static_cast<double>(step) *
                GrowthMinRatio)
            {
                lastFailureDistanceLimited_ = true;
                reason =
                    "faixa sem crescimento no passo " +
                    std::to_string(step) + " de " +
                    std::to_string(steps) + " (medido " +
                    Detail::FormatDistance(app_, grown) +
                    ", esperado " +
                    Detail::FormatDistance(
                        app_,
                        stepDistance * static_cast<double>(step)) +
                    ")";

                Detail::DeleteShape(accumulator);
                return nullptr;
            }

            Detail::PumpMessages();

            if (deadline_.Exceeded() && step < steps)
            {
                reason =
                    "orcamento de tempo esgotado no passo " +
                    std::to_string(step) + " de " +
                    std::to_string(steps);

                Detail::DeleteShape(accumulator);
                return nullptr;
            }
        }

        ForceNoFill(accumulator);
        return accumulator;
    }

    IVGShapePtr UniversalBleedOffset::DilateOneStep(
        const IVGShapePtr& sourceShape,
        double stepDistance,
        const IVGLayerPtr& targetLayer,
        long stepIndex,
        long stepTotal,
        std::string& reason)
    {
        if (!sourceShape)
            return nullptr;

        IVGShapePtr accumulator;
        IVGShapePtr bandShape;
        IVGShapePtr welded;

        try
        {
            auto boundaryCurve = Detail::CurveCopy(sourceShape);
            if (!boundaryCurve)
            {
                reason = "sem curva de fronteira no passo " + std::to_string(stepIndex);
                return nullptr;
            }

            auto subPaths = boundaryCurve->SubPaths;
            const long total = subPaths->Count;
            if (total == 0)
            {
                reason = "fronteira sem subpaths no passo " + std::to_string(stepIndex);
                return nullptr;
            }

            accumulator = sourceShape;
            auto batchCurve = NewBoundCurve();
            if (!batchCurve)
            {
                reason = "nao foi possivel montar a curva do lote";
                return nullptr;
            }

            long batchNodes = 0;
            long pending = 0;

            for (long i = 1; i <= total; ++i)
            {
                auto subPath = subPaths->Item[i];
                if (subPath)
                {
                    auto copy = subPath->GetCopy();
                    if (copy)
                        batchCurve->AppendCurve(copy);
                    batchNodes += subPath->Nodes->Count;
                    ++pending;
                }

                if ((batchNodes >= BandBatchNodes || i == total) && pending > 0)
                {
                    bandShape = BuildBandRaw(batchCurve, stepDistance, targetLayer);
                    if (!bandShape)
                    {
                        reason = "ConvertToObject nao produziu a faixa (passo " +
                            std::to_string(stepIndex) + "/" + std::to_string(stepTotal) +
                            ", ate a subpath " + std::to_string(i) + ")";
                        throw std::runtime_error(reason);
                    }

                    welded = WeldPair(bandShape, accumulator);

                    if (!welded)
                    {
                        reason = "a uniao do lote com a regiao falhou (passo " +
                            std::to_string(stepIndex) + "/" + std::to_string(stepTotal) +
                            ", ate a subpath " + std::to_string(i) + ")";
                        throw std::runtime_error(reason);
                    }

                    bandShape = nullptr;
                    accumulator = welded;
                    welded = nullptr;
                    batchCurve = NewBoundCurve();
                    batchNodes = 0;
                    pending = 0;
                    Detail::PumpMessages();
                }
            }

            return accumulator;
        }
        catch (const _com_error& error)
        {
            if (reason.empty())
                reason = "erro COM: " + Detail::ComErrorText(error);
        }
        catch (const std::exception& error)
        {
            if (reason.empty())
                reason = error.what();
        }
        catch (...)
        {
            if (reason.empty())
                reason = "erro desconhecido na faixa";
        }

        Detail::DeleteShape(welded);
        Detail::DeleteShape(bandShape);
        Detail::DeleteShape(accumulator);
        return nullptr;
    }

    long UniversalBleedOffset::MaxSubPathNodes(const IVGShapePtr& shape) const
    {
        auto curve = Detail::CurveCopy(shape);
        if (!curve)
            return 0;

        long best = 0;
        try
        {
            auto subPaths = curve->SubPaths;
            for (long i = 1; i <= subPaths->Count; ++i)
            {
                auto subPath = subPaths->Item[i];
                if (subPath)
                    best = std::max(best, subPath->Nodes->Count);
            }
        }
        catch (...)
        {
        }
        return best;
    }

    IVGCurvePtr UniversalBleedOffset::NewBoundCurve() const
    {
        try
        {
            return app_->CreateCurve(app_->ActiveDocument);
        }
        catch (...)
        {
            return nullptr;
        }
    }



    IVGShapePtr UniversalBleedOffset::WeldPair(
        IVGShapePtr shapeA,
        IVGShapePtr shapeB) const
    {
        if (!shapeA || !shapeB)
            return nullptr;

        for (long variantIndex = 1; variantIndex <= 2; ++variantIndex)
        {
            IVGShapePtr dupA;
            IVGShapePtr dupB;
            IVGShapePtr result;

            try
            {
                dupA = shapeA->Duplicate(0.0, 0.0);
                dupB = shapeB->Duplicate(0.0, 0.0);

                if (dupA && dupB)
                {
                    if (variantIndex == 1)
                    {
                        result = dupA->Weld(
                            dupB,
                            VARIANT_FALSE,
                            VARIANT_FALSE);
                    }
                    else
                    {
                        result = dupB->Weld(
                            dupA,
                            VARIANT_FALSE,
                            VARIANT_FALSE);
                    }
                }
            }
            catch (...)
            {
                result = nullptr;
            }

            if (result)
            {
                Detail::DeleteShape(dupA);
                Detail::DeleteShape(dupB);
                Detail::DeleteShape(shapeA);
                Detail::DeleteShape(shapeB);

                return result;
            }

            Detail::DeleteShape(dupA);
            Detail::DeleteShape(dupB);
        }

        return nullptr;
    }


    IVGShapePtr UniversalBleedOffset::BuildBandRaw(
        const IVGCurvePtr& boundaryCurve,
        double distance,
        const IVGLayerPtr& targetLayer) const
    {
        IVGShapePtr pathShape;
        IVGShapePtr converted;

        try
        {
            pathShape = targetLayer->CreateCurve(boundaryCurve);

            if (!pathShape)
                throw std::runtime_error("CreateCurve falhou");

            pathShape->Fill->ApplyNoFill();

            auto outline = pathShape->Outline;

            outline->Color->RGBAssign(0, 0, 0);
            outline->Width = distance * 2.0;
            outline->LineJoin = cdrOutlineRoundLineJoin;
            outline->LineCaps = cdrOutlineRoundLineCaps;
            outline->BehindFill = VARIANT_FALSE;

            converted = outline->ConvertToObject();

            if (!converted)
                throw std::runtime_error("ConvertToObject falhou");

            return converted;
        }
        catch (...)
        {
            Detail::DeleteShape(converted);
            Detail::DeleteShape(pathShape);
            return nullptr;
        }
    }

    IVGShapePtr UniversalBleedOffset::DilateOpenGeometry(
        const IVGCurvePtr& sourceCurve,
        double distance,
        const IVGLayerPtr& targetLayer,
        std::string& reason) const
    {
        IVGShapePtr pathShape;
        IVGShapePtr converted;

        try
        {
            pathShape = targetLayer->CreateCurve(sourceCurve);
            if (!pathShape)
            {
                reason = "nao foi possivel recriar a curva aberta";
                return nullptr;
            }

            pathShape->Fill->ApplyNoFill();
            auto outline = pathShape->Outline;
            outline->Color->RGBAssign(0, 0, 0);
            outline->Width = distance * 2.0;
            outline->LineJoin = cdrOutlineRoundLineJoin;
            outline->LineCaps = cdrOutlineRoundLineCaps;
            outline->BehindFill = VARIANT_FALSE;
            converted = outline->ConvertToObject();

            if (!converted)
            {
                reason = "ConvertToObject nao produziu a faixa do traco";
                Detail::DeleteShape(pathShape);
                return nullptr;
            }

            Detail::DeleteShape(pathShape);
            return converted;
        }
        catch (const _com_error& error)
        {
            if (reason.empty())
                reason = "erro COM: " + Detail::ComErrorText(error);
        }
        catch (...)
        {
            if (reason.empty())
                reason = "erro desconhecido na curva aberta";
        }

        Detail::DeleteShape(converted);
        Detail::DeleteShape(pathShape);
        return nullptr;
    }



    IVGShapePtr UniversalBleedOffset::BuildRegionShape(
        const IVGShapePtr& baseShape,
        const IVGCurvePtr& closedCurve,
        const IVGLayerPtr& targetLayer,
        bool& usedCurvePath) const
    {
        usedCurvePath = false;

        if (!closedCurve)
            return nullptr;

        
        
        if (IsGroupShape(baseShape))
        {
            auto regionShape =
                BuildRegionFromGroup(baseShape, targetLayer);

            if (regionShape)
                return regionShape;
        }

        
        
        try
        {
            if (baseShape->Type == cdrCurveShape && targetLayer)
            {
                IVGShapePtr regionShape = targetLayer->CreateCurve(closedCurve);
                if (regionShape)
                {
                    regionShape->FillMode = baseShape->FillMode;
                    regionShape->Fill->ApplyUniformFill(app_->CreateRGBColor(0, 0, 0));
                    regionShape->Outline->SetNoOutline();
                    usedCurvePath = true;
                    return regionShape;
                }
            }
        }
        catch (...)
        {
        }

        if (auto regionShape =
            BuildRegionFromDuplicate(baseShape))
        {
            return regionShape;
        }

        if (!targetLayer)
            return nullptr;

        IVGShapePtr regionShape;
        try
        {
            regionShape = targetLayer->CreateCurve(closedCurve);

            if (!regionShape)
                return nullptr;

            regionShape->FillMode = baseShape->FillMode;
            usedCurvePath = true;

            regionShape->Fill->ApplyUniformFill(
                app_->CreateRGBColor(0, 0, 0));

            regionShape->Outline->SetNoOutline();

            return regionShape;
        }
        catch (...)
        {
            Detail::DeleteShape(regionShape);
            return nullptr;
        }
    }








    IVGShapePtr UniversalBleedOffset::BuildRegionFromDuplicate(const IVGShapePtr& baseShape) const
    {
        if (!baseShape)
            return nullptr;

        IVGShapePtr duplicate;

        try
        {
            duplicate = baseShape->Duplicate(0.0, 0.0);
            if (!duplicate)
                return nullptr;

            duplicate->ConvertToCurves();
            if (CountClosedSubPaths(duplicate) == 0)
            {
                Detail::DeleteShape(duplicate);
                return nullptr;
            }

            duplicate->Fill->ApplyUniformFill(app_->CreateRGBColor(0, 0, 0));
            duplicate->Outline->SetNoOutline();
            return duplicate;
        }
        catch (...)
        {
            Detail::DeleteShape(duplicate);
            return nullptr;
        }
    }

    IVGShapePtr UniversalBleedOffset::BuildRegionFromGroup(
        const IVGShapePtr& baseShape,
        const IVGLayerPtr&) const
    {
        IVGShapePtr duplicate;
        IVGShapePtr welded;

        try
        {
            duplicate = baseShape->Duplicate(0.0, 0.0);
            if (!duplicate)
                return nullptr;

            auto members = FlattenGroupRange(duplicate);
            duplicate = nullptr;

            if (!members || members->Count == 0)
                return nullptr;

            try { members->ConvertToCurves(); }
            catch (...) {}

            if (members->Count == 1)
                welded = members->Item[1];
            else
                welded = WeldRange(members);

            if (!welded)
            {
                Detail::DeleteRange(members);
                return nullptr;
            }

            welded->Fill->ApplyUniformFill(app_->CreateRGBColor(0, 0, 0));
            welded->Outline->SetNoOutline();
            return welded;
        }
        catch (...)
        {
            Detail::DeleteShape(welded);
            Detail::DeleteShape(duplicate);
            return nullptr;
        }
    }

    IVGShapeRangePtr UniversalBleedOffset::FlattenGroupRange(const IVGShapePtr& groupShape) const
    {
        if (!groupShape)
            return nullptr;

        try
        {
            auto current = app_->CreateShapeRange();
            current->Add(groupShape);

            for (long guard = 0; guard < 64; ++guard)
            {
                bool foundGroup = false;
                auto next = app_->CreateShapeRange();

                for (long i = 1; i <= current->Count; ++i)
                {
                    auto shape = current->Item[i];
                    if (!shape)
                        continue;

                    if (shape->Type == cdrGroupShape)
                    {
                        auto ungrouped = shape->UngroupEx();
                        if (ungrouped)
                            next->AddRange(ungrouped);
                        foundGroup = true;
                    }
                    else
                    {
                        next->Add(shape);
                    }
                }

                current = next;
                if (!foundGroup)
                    break;
            }

            return current;
        }
        catch (...)
        {
            return nullptr;
        }
    }

    IVGShapeRangePtr UniversalBleedOffset::FlattenRange(const IVGShapeRangePtr& source) const
    {
        if (!source)
            return nullptr;

        try
        {
            auto current = source;

            for (long guard = 0; guard < 64; ++guard)
            {
                bool foundGroup = false;
                auto next = app_->CreateShapeRange();

                for (long i = 1; i <= current->Count; ++i)
                {
                    auto shape = current->Item[i];
                    if (!shape)
                        continue;

                    if (shape->Type == cdrGroupShape)
                    {
                        auto ungrouped = shape->UngroupEx();
                        if (ungrouped)
                            next->AddRange(ungrouped);
                        foundGroup = true;
                    }
                    else
                    {
                        next->Add(shape);
                    }
                }

                current = next;
                if (!foundGroup)
                    break;
            }

            return current;
        }
        catch (...)
        {
            return source;
        }
    }

    IVGShapePtr UniversalBleedOffset::WeldRange(const IVGShapeRangePtr& members) const
    {
        if (!members || members->Count == 0)
            return nullptr;

        IVGShapePtr accumulated;
        IVGShapePtr nextShape;

        try
        {
            accumulated = members->Item[1]->Duplicate(0.0, 0.0);
            if (!accumulated)
                return nullptr;

            for (long i = 2; i <= members->Count; ++i)
            {
                nextShape = members->Item[i]->Duplicate(0.0, 0.0);
                if (!nextShape)
                    throw std::runtime_error("duplicate failed");

                auto welded = nextShape->Weld(accumulated, VARIANT_FALSE, VARIANT_FALSE);
                if (!welded)
                    throw std::runtime_error("weld failed");

                accumulated = welded;
                nextShape = nullptr;
            }

            return accumulated;
        }
        catch (...)
        {
            Detail::DeleteShape(nextShape);
            Detail::DeleteShape(accumulated);
            return nullptr;
        }
    }

    bool UniversalBleedOffset::ValidateDilation(
        const Detail::Bounds& sourceBounds,
        const IVGShapePtr& regionProbe,
        const IVGShapePtr& resultShape,
        double distance,
        std::string& reason) const
    {
        if (!resultShape)
        {
            reason = "resultado vazio";
            return false;
        }

        Detail::Bounds resultBounds;
        if (!Detail::MeasureShape(resultShape, resultBounds))
        {
            reason = "nao foi possivel medir o resultado";
            return false;
        }

        const double growth[4] = {
            sourceBounds.left - resultBounds.left,
            resultBounds.right - sourceBounds.right,
            sourceBounds.bottom - resultBounds.bottom,
            resultBounds.top - sourceBounds.top
        };

        const double minAllowed = distance * GrowthMinRatio;
        const double maxAllowed = distance * GrowthMaxRatio;

        for (long i = 0; i < 4; ++i)
        {
            if (!std::isfinite(growth[i]) || growth[i] < minAllowed)
            {
                reason = "sangria insuficiente no lado " + SideName(i + 1) +
                    ": medido " + Detail::FormatDistance(app_, growth[i]) +
                    ", minimo " + Detail::FormatDistance(app_, minAllowed) +
                    " [E " + Detail::FormatDistance(app_, growth[0]) +
                    " / D " + Detail::FormatDistance(app_, growth[1]) +
                    " / B " + Detail::FormatDistance(app_, growth[2]) +
                    " / T " + Detail::FormatDistance(app_, growth[3]) + "]";
                return false;
            }
        }

        for (long i = 0; i < 4; ++i)
        {
            if (growth[i] > maxAllowed)
            {
                reason = "crescimento absurdo no lado " + SideName(i + 1) +
                    ": medido " + Detail::FormatDistance(app_, growth[i]) +
                    ", teto " + Detail::FormatDistance(app_, maxAllowed);
                return false;
            }
        }

        if (!TopologyChecksPass(regionProbe, resultShape, distance, reason))
            return false;

        if (!RegionFitsInside(regionProbe, resultShape, reason))
        {
            if (reason.empty()) reason = "o resultado nao contem o objeto original";
            return false;
        }

        return true;
    }

    bool UniversalBleedOffset::ValidateCachedDilation(
        const Detail::Bounds& sourceBounds,
        const IVGShapePtr& resultShape,
        double distance,
        std::string& reason) const
    {
        if (!resultShape)
        {
            reason = "resultado de cache vazio";
            return false;
        }

        Detail::Bounds resultBounds;
        if (!Detail::MeasureShape(resultShape, resultBounds))
        {
            reason = "nao foi possivel medir o resultado de cache";
            return false;
        }

        const double growth[4]
        {
            sourceBounds.left - resultBounds.left,
            resultBounds.right - sourceBounds.right,
            sourceBounds.bottom - resultBounds.bottom,
            resultBounds.top - sourceBounds.top
        };
        const double minimum = distance * GrowthMinRatio;
        const double maximum = distance * GrowthMaxRatio;
        for (double value : growth)
        {
            if (!std::isfinite(value) || value < minimum || value > maximum)
            {
                reason = "crescimento invalido no resultado de cache";
                return false;
            }
        }

        
        
        
        
        return true;
    }

    bool UniversalBleedOffset::RegionFitsInside(
        const IVGShapePtr& regionProbe, const IVGShapePtr& container,
        std::string& reason) const
    {
        IVGShapePtr probe, remainder;
        try
        {
            const auto area = [](const IVGShapePtr& shape)
            {
                if (!shape) throw std::runtime_error("shape indisponivel");
                auto curve = shape->DisplayCurve;
                if (!curve || !curve->SubPaths) throw std::runtime_error("curva indisponivel");
                double total = 0;
                auto paths = curve->SubPaths;
                for (long i = 1; i <= paths->Count; ++i)
                {
                    auto path = paths->Item[i];
                    if (!path) throw std::runtime_error("subpath indisponivel");
                    if (path->Closed) total += std::abs(path->Area);
                }
                if (!std::isfinite(total)) throw std::runtime_error("area nao finita");
                return total;
            };
            const double probeArea = area(regionProbe);
            if (!container || probeArea <= 0) throw std::runtime_error("regiao sem area valida");
            probe = regionProbe->Duplicate(0.0, 0.0);
            if (!probe) throw std::runtime_error("falha ao duplicar regiao");
            remainder = container->Trim(probe, VARIANT_TRUE, VARIANT_FALSE);
            
            const double leftover = remainder ? area(remainder) : 0.0;
            Detail::DeleteShape(remainder);
            Detail::DeleteShape(probe);
            if (leftover > probeArea * LeftoverTolerance)
            {
                reason = "o resultado nao contem o objeto original";
                return false;
            }
            return true;
        }
        catch (...)
        {
            Detail::DeleteShape(remainder);
            Detail::DeleteShape(probe);
            reason = "contencao nao verificavel: falha ao consultar area ou executar Trim";
            return false;
        }
    }

    long UniversalBleedOffset::CountClosedSubPaths(const IVGShapePtr& shape) const
    {
        long total = 0;
        try
        {
            IVGCurvePtr curve;
            if (shape && shape->Type == cdrCurveShape)
                curve = shape->Curve;
            if (!curve && shape)
                curve = shape->DisplayCurve;
            if (!curve)
                return 0;

            auto subPaths = curve->SubPaths;
            for (long i = 1; i <= subPaths->Count; ++i)
            {
                auto subPath = subPaths->Item[i];
                if (subPath && subPath->Closed)
                    ++total;
            }
        }
        catch (...)
        {
        }
        return total;
    }

    bool UniversalBleedOffset::FindSubPathInteriorPoint(
        const IVGSubPathPtr& subPath,
        double tolerance,
        double& x,
        double& y) const
    {
        if (!subPath)
            return false;

        try
        {
            auto box = subPath->BoundingBox;
            if (!box || box->Width <= 0.0 || box->Height <= 0.0)
                return false;

            x = box->Left + box->Width * 0.5;
            y = box->Bottom + box->Height * 0.5;
            if (subPath->IsOnSubPath(x, y, tolerance) == cdrInsideShape)
                return true;

            constexpr long Grid = 3;
            for (long row = 0; row < Grid; ++row)
            {
                const double v = (static_cast<double>(row) + 0.5) /
                    static_cast<double>(Grid);
                for (long column = 0; column < Grid; ++column)
                {
                    const double u = (static_cast<double>(column) + 0.5) /
                        static_cast<double>(Grid);
                    const double candidateX = box->Left + box->Width * u;
                    const double candidateY = box->Bottom + box->Height * v;
                    if (subPath->IsOnSubPath(
                        candidateX,
                        candidateY,
                        tolerance) == cdrInsideShape)
                    {
                        x = candidateX;
                        y = candidateY;
                        return true;
                    }
                }
            }
        }
        catch (...)
        {
        }

        return false;
    }

    std::vector<long> UniversalBleedOffset::FindUnexpectedInternalSubpaths(
        const IVGShapePtr& regionProbe,
        const IVGShapePtr& resultShape,
        double distance) const
    {
        std::vector<long> result;
        if (!regionProbe || !resultShape)
            return result;

        try
        {
            IVGCurvePtr sourceCurve;
            IVGCurvePtr generatedCurve;
            if (regionProbe->Type == cdrCurveShape)
                sourceCurve = regionProbe->Curve;
            if (!sourceCurve)
                sourceCurve = regionProbe->DisplayCurve;
            if (resultShape->Type == cdrCurveShape)
                generatedCurve = resultShape->Curve;
            if (!generatedCurve)
                generatedCurve = resultShape->DisplayCurve;
            if (!sourceCurve || !generatedCurve ||
                !sourceCurve->SubPaths || !generatedCurve->SubPaths)
            {
                return result;
            }

            auto sourcePaths = sourceCurve->SubPaths;
            auto generatedPaths = generatedCurve->SubPaths;
            if (generatedPaths->Count <= sourcePaths->Count)
                return result;

            long sourceCount = 0;
            for (long index = 1; index <= sourcePaths->Count; ++index)
            {
                auto path = sourcePaths->Item[index];
                if (path && path->Closed)
                    ++sourceCount;
            }
            long generatedCount = 0;
            for (long index = 1; index <= generatedPaths->Count; ++index)
            {
                auto path = generatedPaths->Item[index];
                if (path && path->Closed)
                    ++generatedCount;
            }
            if (sourceCount <= 0 || generatedCount <= sourceCount)
                return result;

            const double tolerance = std::max(distance * 0.001, 0.000001);
            struct SubPathBounds
            {
                bool valid = false;
                bool closed = false;
                bool degenerate = false;
                double left = 0.0;
                double bottom = 0.0;
                double right = 0.0;
                double top = 0.0;
                double area = 0.0;
            };

            const long pathCount = generatedPaths->Count;
            std::vector<SubPathBounds> bounds(
                static_cast<std::size_t>(pathCount + 1));
            for (long index = 1; index <= pathCount; ++index)
            {
                auto path = generatedPaths->Item[index];
                if (!path || !path->Closed)
                    continue;

                auto& current = bounds[static_cast<std::size_t>(index)];
                current.closed = true;
                auto box = path->BoundingBox;
                if (!box || std::abs(box->Width) <= tolerance ||
                    std::abs(box->Height) <= tolerance ||
                    path->Nodes->Count < 3)
                {
                    current.degenerate = true;
                    continue;
                }

                current.valid = true;
                current.left = box->Left;
                current.bottom = box->Bottom;
                current.right = box->Left + box->Width;
                current.top = box->Bottom + box->Height;
                current.area = box->Width * box->Height;
            }

            for (long generatedIndex = 1;
                generatedIndex <= pathCount;
                ++generatedIndex)
            {
                const auto& candidate = bounds[static_cast<std::size_t>(generatedIndex)];
                if (!candidate.closed)
                    continue;
                if (candidate.degenerate)
                {
                    result.push_back(generatedIndex);
                    continue;
                }
                if (!candidate.valid)
                    continue;

                
                
                
                bool enclosed = false;
                for (long otherIndex = 1; otherIndex <= pathCount; ++otherIndex)
                {
                    if (otherIndex == generatedIndex)
                        continue;

                    const auto& other = bounds[static_cast<std::size_t>(otherIndex)];
                    if (!other.valid || other.area <= candidate.area)
                        continue;

                    if (other.left <= candidate.left + tolerance &&
                        other.bottom <= candidate.bottom + tolerance &&
                        other.right >= candidate.right - tolerance &&
                        other.top >= candidate.top - tolerance)
                    {
                        enclosed = true;
                        break;
                    }
                }
                if (!enclosed)
                    continue;

                auto generatedPath = generatedPaths->Item[generatedIndex];
                double x = 0.0;
                double y = 0.0;
                if (!FindSubPathInteriorPoint(generatedPath, tolerance, x, y))
                    continue;

                if (resultShape->IsOnShape(x, y, tolerance) != cdrOutsideShape)
                    continue;

                bool matchesOriginalHole = false;
                const bool outsideOriginal =
                    regionProbe->IsOnShape(x, y, tolerance) == cdrOutsideShape;
                if (outsideOriginal)
                {
                    for (long sourceIndex = 1;
                        sourceIndex <= sourcePaths->Count;
                        ++sourceIndex)
                    {
                        auto sourcePath = sourcePaths->Item[sourceIndex];
                        if (sourcePath &&
                            sourcePath->IsOnSubPath(
                                x,
                                y,
                                tolerance) == cdrInsideShape)
                        {
                            matchesOriginalHole = true;
                            break;
                        }
                    }
                }

                if (!matchesOriginalHole)
                    result.push_back(generatedIndex);
            }
        }
        catch (...)
        {
            result.clear();
        }
        return result;
    }

    bool UniversalBleedOffset::RemoveUnexpectedInternalSubpaths(
        const IVGShapePtr& regionProbe,
        IVGShapePtr& resultShape,
        const IVGLayerPtr& targetLayer,
        double distance,
        std::string& reason) const
    {
        if (!regionProbe || !resultShape)
        {
            reason = "o offset nao forneceu geometria para limpeza";
            return false;
        }

        const long sourceCount = CountClosedSubPaths(regionProbe);
        const long resultCount = CountClosedSubPaths(resultShape);
        if (sourceCount <= 0 || resultCount <= 0)
        {
            reason = "o offset nao forneceu subpaths fechados para limpeza";
            return false;
        }
        if (resultCount <= sourceCount)
            return true;

        try
        {
            auto resultCurve = Detail::CurveCopy(resultShape);
            if (!resultCurve || !resultCurve->SubPaths)
            {
                reason = "nao foi possivel copiar a curva do offset para limpeza";
                return false;
            }

            const auto remove = FindUnexpectedInternalSubpaths(
                regionProbe,
                resultShape,
                distance);

            if (remove.empty())
            {
                reason = "o offset criou subpaths internos que nao puderam ser classificados";
                return false;
            }

            for (auto index = remove.rbegin(); index != remove.rend(); ++index)
                resultCurve->SubPaths->Item[*index]->Delete();

            if (resultCurve->SubPaths->Count <= 0)
            {
                reason = "a limpeza do offset removeu toda a geometria";
                return false;
            }

            IVGLayerPtr layer = targetLayer;
            if (!layer)
            {
                try { layer = resultShape->Layer; }
                catch (...) {}
            }
            if (!layer)
            {
                reason = "a camada de destino do offset nao esta disponivel";
                return false;
            }

            const cdrFillMode fillMode = resultShape->FillMode;
            auto cleaned = layer->CreateCurve(resultCurve);
            if (!cleaned)
            {
                reason = "nao foi possivel recriar a curva limpa do offset";
                return false;
            }

            cleaned->FillMode = fillMode;
            ForceNoFill(cleaned);
            Detail::DeleteShape(resultShape);
            resultShape = cleaned;

            if (!FindUnexpectedInternalSubpaths(
                regionProbe,
                resultShape,
                distance).empty())
            {
                reason = "a limpeza do offset ainda deixou subpaths internos extras";
                return false;
            }
            return true;
        }
        catch (...)
        {
            reason = "falha ao limpar subpaths internos do offset";
            return false;
        }
    }


    bool UniversalBleedOffset::TopologyChecksPass(
        const IVGShapePtr& regionProbe,
        const IVGShapePtr& resultShape,
        double distance,
        std::string& reason) const
    {
        if (!regionProbe || !resultShape)
        { reason = "topologia nao verificavel: shape indisponivel"; return false; }

        IVGCurvePtr curve;
        try
        {
            if (regionProbe && regionProbe->Type == cdrCurveShape)
                curve = regionProbe->Curve;
            if (!curve && regionProbe)
                curve = regionProbe->DisplayCurve;
        }
        catch (...) {}
        if (!curve)
        { reason = "topologia nao verificavel: curva indisponivel"; return false; }

        double tolerance = distance * 0.001;
        if (tolerance <= 0.0)
            tolerance = 0.000001;


        try
        {
            auto subPaths = curve->SubPaths;
            const auto unexpectedSubpaths = FindUnexpectedInternalSubpaths(
                regionProbe,
                resultShape,
                distance);
            if (!unexpectedSubpaths.empty())
            {
                reason = "a sangria criou " +
                    std::to_string(unexpectedSubpaths.size()) +
                    " subpath(s) interno(s) inexistente(s) no original";
                return false;
            }

            for (long i = 1; i <= subPaths->Count; ++i)
            {
                auto subPath = subPaths->Item[i];

                if (!subPath)
                { reason = "topologia nao verificavel: subpath indisponivel"; return false; }
                if (!subPath->Closed) continue;

                auto box = subPath->BoundingBox;
                if (!box)
                { reason = "topologia nao verificavel: limites indisponiveis"; return false; }

                const double minSide =
                    std::min(
                        std::abs(box->Width),
                        std::abs(box->Height));

                const double cx =
                    box->Left + box->Width * 0.5;

                const double cy =
                    box->Bottom + box->Height * 0.5;

                if (!std::isfinite(minSide) || !std::isfinite(cx) || !std::isfinite(cy))
                { reason = "topologia nao verificavel: coordenadas invalidas"; return false; }

                if (subPath->IsOnSubPath(
                    cx,
                    cy,
                    tolerance) != cdrInsideShape)
                {
                    continue;
                }

                const bool insideRegion =
                    regionProbe->IsOnShape(
                        cx,
                        cy,
                        tolerance) == cdrInsideShape;

                if (insideRegion)
                {
                    {

                        if (resultShape->IsOnShape(
                            cx,
                            cy,
                            tolerance) == cdrOutsideShape)
                        {
                            reason =
                                "o resultado nao cobre o interior "
                                "do objeto (saiu anel)";

                            return false;
                        }
                    }
                }
                else if (minSide > distance * 3.0)
                {
                    
                    
                    
                    if (resultShape->IsOnShape(
                        cx,
                        cy,
                        tolerance) == cdrInsideShape)
                    {
                        reason =
                            "a sangria preencheu um furo que "
                            "deveria continuar aberto";

                        return false;
                    }
                }
            }
        }
        catch (...)
        {
            reason = "topologia nao verificavel: falha COM";
            return false;
        }

        return true;
    }

    std::string UniversalBleedOffset::SideName(long sideIndex) const
    {
        switch (sideIndex)
        {
        case 1: return "esquerdo";
        case 2: return "direito";
        case 3: return "inferior";
        case 4: return "superior";
        default: return std::to_string(sideIndex);
        }
    }

    void UniversalBleedOffset::ClearCache()
    {
        cache_.clear();
        cacheIndex_.clear();
    }

    IVGShapePtr UniversalBleedOffset::TryReuseFromCache(
        const std::string& signature,
        const IVGCurvePtr& sourceCurve,
        double,
        const IVGLayerPtr& targetLayer)
    {
        if (!cacheEnabled_ || !sourceCurve || !targetLayer || signature.empty())
            return nullptr;

        const auto it = cacheIndex_.find(signature);
        if (it == cacheIndex_.end() || it->second >= cache_.size())
            return nullptr;

        const auto& entry = cache_[it->second];
        long mode = 0;
        double p1 = 0.0;
        double p2 = 0.0;

        if (!CurvesMatchByIsometry(entry.sourceCurve, sourceCurve, mode, p1, p2))
            return nullptr;

        IVGShapePtr shape;

        try
        {
            shape = targetLayer->CreateCurve(entry.resultCurve);
            if (!shape)
                return nullptr;

            shape->FillMode = entry.fillMode;
            shape->Fill->ApplyNoFill();
            shape->Outline->SetNoOutline();

            if (!ApplyCacheTransform(shape, mode, p1, p2))
            {
                Detail::DeleteShape(shape);
                return nullptr;
            }

            return shape;
        }
        catch (...)
        {
            Detail::DeleteShape(shape);
            return nullptr;
        }
    }

    void UniversalBleedOffset::StoreInCache(
        const std::string& signature,
        const IVGCurvePtr& sourceCurve,
        const IVGShapePtr& resultShape,
        double)
    {
        if (!cacheEnabled_ || !sourceCurve || !resultShape || signature.empty())
            return;
        if (cache_.size() >= CacheMaxEntries)
            return;

        try
        {
            auto resultCopy = Detail::CurveCopy(resultShape);
            if (!resultCopy)
                return;

            auto sourceCopy = sourceCurve->GetCopy();
            if (!sourceCopy)
                sourceCopy = sourceCurve;

            const std::size_t index = cache_.size();
            cache_.push_back({ signature, sourceCopy, resultCopy, resultShape->FillMode });
            cacheIndex_.emplace(signature, index);
            ++cacheMisses_;
        }
        catch (...)
        {
        }
    }

    bool UniversalBleedOffset::ApplyCacheTransform(
        const IVGShapePtr& shape,
        long mode,
        double p1,
        double p2) const
    {
        if (!shape)
            return false;

        try
        {
            if (mode == CacheModeMove)
            {
                if (p1 != 0.0 || p2 != 0.0)
                    shape->Move(p1, p2);
                return true;
            }

            if (mode == CacheModeFlipH)
            {
                Detail::Bounds bounds;
                if (!Detail::MeasureShape(shape, bounds))
                    return false;

                const double centerX = (bounds.left + bounds.right) * 0.5;
                shape->Flip(cdrFlipHorizontal);
                shape->Move(p1 - 2.0 * centerX, p2);
                return true;
            }
        }
        catch (...)
        {
            return false;
        }

        return false;
    }

    std::string UniversalBleedOffset::CurveSignature(
        const IVGCurvePtr& curve,
        double distance) const
    {
        if (!curve)
            return {};

        try
        {
            std::ostringstream stream;
            stream << std::fixed << std::setprecision(17) << "d" << distance;

            auto subPaths = curve->SubPaths;
            stream << ";n" << subPaths->Count;

            long totalNodes = 0;
            const long count = subPaths->Count;

            for (long i = 1; i <= count; ++i)
            {
                auto subPath = subPaths->Item[i];
                if (!subPath)
                    continue;

                const long nodes = subPath->Nodes->Count;
                totalNodes += nodes;

                if (i <= CacheSignatureSubpathDetail)
                {
                    auto box = subPath->BoundingBox;
                    stream << "|" << (subPath->Closed ? "C" : "O")
                        << nodes << ","
                        << subPath->Segments->Count;
                    if (box)
                    {
                        stream << ","
                            << std::fixed << std::setprecision(4)
                            << std::abs(box->Width) << "x"
                            << std::abs(box->Height);
                    }
                }
            }

            stream << "|T" << totalNodes;
            return stream.str();
        }
        catch (...)
        {
            return {};
        }
    }

    bool UniversalBleedOffset::CurvesMatchByIsometry(
        const IVGCurvePtr& cachedCurve,
        const IVGCurvePtr& newCurve,
        long& mode,
        double& p1,
        double& p2) const
    {
        mode = 0; p1 = p2 = 0;
        if (!cachedCurve || !newCurve) return false;
        try
        {
            const auto capture = [this](const IVGCurvePtr& curve)
            {
                GeometryCache::Curve snapshot;
                auto paths = curve->SubPaths;
                const long count = paths->Count;
                if (count <= 0 || count > static_cast<long>(GeometryCache::MaxSegments))
                    throw std::runtime_error("cache: limite de subpaths");
                std::size_t total = 0;
                for (long i = 1; i <= count; ++i)
                {
                    if (deadline_.Exceeded()) throw std::runtime_error("cache: prazo esgotado");
                    auto path = paths->Item[i];
                    if (!path) throw std::runtime_error("cache: subpath indisponivel");
                    GeometryCache::Path entry;
                    entry.closed = path->Closed != VARIANT_FALSE;
                    entry.nodes = path->Nodes->Count;
                    auto segments = path->Segments;
                    const long n = segments->Count;
                    if (n <= 0 || static_cast<std::size_t>(n) > GeometryCache::MaxSegments - total)
                        throw std::runtime_error("cache: limite de segmentos");
                    total += n;
                    for (long j = 1; j <= n; ++j)
                    {
                        if (deadline_.Exceeded()) throw std::runtime_error("cache: prazo esgotado");
                        auto seg = segments->Item[j];
                        if (!seg || (seg->Type != cdrLineSegment && seg->Type != cdrCurveSegment))
                            throw std::runtime_error("cache: segmento nao suportado");
                        GeometryCache::Segment item;
                        item.cubic = seg->Type == cdrCurveSegment;
                        auto start = seg->StartNode, end = seg->EndNode;
                        if (!start || !end) throw std::runtime_error("cache: no indisponivel");
                        item.points[0] = {start->PositionX, start->PositionY};
                        item.points[3] = {end->PositionX, end->PositionY};
                        if (item.cubic)
                        {
                            seg->GetStartingControlPointPosition(&item.points[1].x, &item.points[1].y);
                            seg->GetEndingControlPointPosition(&item.points[2].x, &item.points[2].y);
                        }
                        entry.segments.push_back(item);
                    }
                    snapshot.push_back(std::move(entry));
                }
                return snapshot;
            };
            return GeometryCache::Match(capture(cachedCurve), capture(newCurve),
                Detail::Mm(app_, 1e-8), mode, p1, p2);
        }
        catch (...) { return false; }
    }

    IVGCurvePtr UniversalBleedOffset::ExtractClosedSubPaths(
        const IVGCurvePtr& sourceCurve,
        long& closedCount) const
    {
        closedCount = 0;
        if (!sourceCurve)
            return nullptr;

        try
        {
            auto result = app_->CreateCurve(app_->ActiveDocument);
            auto subPaths = sourceCurve->SubPaths;

            for (long i = 1; i <= subPaths->Count; ++i)
            {
                auto subPath = subPaths->Item[i];
                if (!subPath || !subPath->Closed)
                    continue;

                auto component = subPath->GetCopy();
                if (component)
                {
                    result->AppendCurve(component);
                    ++closedCount;
                }
            }

            return closedCount > 0 ? result : nullptr;
        }
        catch (...)
        {
            closedCount = 0;
            return nullptr;
        }
    }

    void UniversalBleedOffset::MirrorShape(const IVGShapePtr& shape) const
    {
        if (!shape)
            return;
        try { shape->Flip(cdrFlipHorizontal); }
        catch (...) {}
    }



    std::string UniversalBleedOffset::StrategyName(long attempt) const
    {
        if (attempt == 2)
            return {};

        std::string variantName;

        switch (((attempt - 1) / 2) + 1)
        {
        case 1:
            break;

        case 2:
            variantName = " com sentido corrigido";
            break;

        case 3:
            variantName = " com espelhamento";
            break;

        case 4:
            variantName =
                " com espelhamento e sentido corrigido";
            break;

        default:
            break;
        }

        const std::string methodName =
            (((attempt - 1) % 2) + 1 == 1)
            ? "faixa de Minkowski"
            : "contorno nativo";

        return methodName + variantName;
    }

    IVGCurvePtr UniversalBleedOffset::ReverseCurveDirection(const IVGCurvePtr& source) const
    {
        if (!source)
            return nullptr;

        try
        {
            auto copy = source->GetCopy();
            if (!copy)
                return nullptr;

            auto subPaths = copy->SubPaths;
            for (long i = 1; i <= subPaths->Count; ++i)
            {
                auto subPath = subPaths->Item[i];
                if (subPath)
                    subPath->ReverseDirection();
            }
            return copy;
        }
        catch (...)
        {
            return nullptr;
        }
    }

    bool UniversalBleedOffset::IsGroupShape(const IVGShapePtr& shape) const
    {
        if (!shape)
            return false;
        try { return shape->Type == cdrGroupShape; }
        catch (...) { return false; }
    }

    IVGLayerPtr UniversalBleedOffset::ResolveWritableLayer(const IVGShapePtr& baseShape) const
    {
        try
        {
            if (baseShape)
            {
                auto layer = baseShape->Layer;
                if (layer && layer->Editable)
                    return layer;
            }
        }
        catch (...)
        {
        }

        try
        {
            auto layer = app_->ActiveLayer;
            if (layer && layer->Editable)
                return layer;
        }
        catch (...)
        {
        }

        return GetOrCreateEmergencyLayer(baseShape);
    }

    IVGLayerPtr UniversalBleedOffset::GetOrCreateEmergencyLayer(const IVGShapePtr& baseShape) const
    {
        IVGPagePtr page;

        try
        {
            if (baseShape)
                page = baseShape->Page;
        }
        catch (...)
        {
        }

        if (!page)
        {
            try { page = app_->ActivePage; }
            catch (...) {}
        }

        if (!page)
            return nullptr;

        try
        {
            auto layers = page->Layers;
            for (long i = 1; i <= layers->Count; ++i)
            {
                auto layer = layers->Item[i];
                if (!layer)
                    continue;

                if (_stricmp(Detail::Narrow(layer->Name).c_str(), EmergencyLayerName) == 0)
                {
                    layer->Editable = VARIANT_TRUE;
                    if (layer->Editable)
                        return layer;
                }
            }

            auto layer = page->CreateLayer(EmergencyLayerName);
            if (layer)
            {
                layer->Editable = VARIANT_TRUE;
                return layer;
            }
        }
        catch (...)
        {
        }

        return nullptr;
    }

    HiddenObjectDetector::HiddenObjectDetector(IVGApplicationPtr app)
        : app_(std::move(app))
    {
    }

    void HiddenObjectDetector::SetAction(HiddenObjectAction value) noexcept
    {
        action_ = value;
    }

    HiddenObjectAction HiddenObjectDetector::Action() const noexcept
    {
        return action_;
    }

    void HiddenObjectDetector::SetTimeBudgetSeconds(double value) noexcept
    {
        timeBudgetSeconds_ = value;
    }

    double HiddenObjectDetector::TimeBudgetSeconds() const noexcept
    {
        return timeBudgetSeconds_ > 0.0 ? timeBudgetSeconds_ : 45.0;
    }

    const std::string& HiddenObjectDetector::GetLastReport() const noexcept
    {
        return lastReport_;
    }

    void HiddenObjectDetector::Process(IVGShapeRangePtr& range, long& hiddenCount)
    {
        hiddenCount = 0;
        lastReport_.clear();

        if (!range || range->Count == 0)
            return;

        deadline_.Start(TimeBudgetSeconds());

        if (!SnapshotSelection(range))
        {
            lastReport_ = "deteccao de ocultos abortada; nada foi alterado";
            ClearContext();
            return;
        }

        const double bboxTolerance = Detail::Mm(app_, BboxToleranceMm);
        const double pointTolerance = Detail::Mm(app_, PointToleranceMm);
        double areaTolerance = Detail::Mm(app_, std::sqrt(AreaToleranceMm2));
        areaTolerance *= areaTolerance;

        std::vector<bool> hiddenFlags(shapes_.size(), false);
        bool budgetHit = false;

        try
        {
            for (std::size_t i = 0; i < shapes_.size(); ++i)
            {
                if (deadline_.Exceeded())
                {
                    budgetHit = true;
                    break;
                }

                auto candidates = BuildCandidateList(i, bboxTolerance);
                if (candidates.empty())
                    continue;

                EnsureVisibility(i);
                if (!shapes_[i].visible)
                    continue;

                hiddenFlags[i] = IsShapeHidden(
                    i,
                    candidates,
                    bboxTolerance,
                    pointTolerance,
                    areaTolerance);
            }

            auto hiddenRange = app_->CreateShapeRange();
            auto visibleRange = app_->CreateShapeRange();

            for (std::size_t i = 0; i < shapes_.size(); ++i)
            {
                auto shape = ShapeAt(i);
                if (!shape)
                    continue;

                if (hiddenFlags[i])
                {
                    hiddenRange->Add(shape);
                    ++hiddenCount;
                }
                else
                {
                    visibleRange->Add(shape);
                }
            }

            if (hiddenRange->Count > 0)
                ApplyHiddenAction(hiddenRange);

            range = visibleRange;
            lastReport_ = std::to_string(hiddenCount) + " oculto(s) de " + std::to_string(shapes_.size());
            if (budgetHit)
                lastReport_ += " (analise interrompida pelo limite de tempo)";
        }
        catch (...)
        {
            try
            {
                auto visibleRange = app_->CreateShapeRange();
                for (std::size_t i = 0; i < shapes_.size(); ++i)
                {
                    auto shape = ShapeAt(i);
                    if (shape)
                        visibleRange->Add(shape);
                }
                if (visibleRange->Count > 0)
                    range = visibleRange;
            }
            catch (...)
            {
            }

            hiddenCount = 0;
            lastReport_ = "deteccao de ocultos abortada; nada foi alterado";
        }

        ClearContext();
    }

    bool HiddenObjectDetector::SnapshotSelection(const IVGShapeRangePtr& range)
    {
        if (!range)
            return false;

        try
        {
            doc_ = app_->ActiveDocument;
            const long count = range->Count;
            shapes_.clear();
            shapes_.resize(static_cast<std::size_t>(count));

            bool zOk = true;
            bool zAllSame = true;
            bool sameLayer = true;
            std::string firstLayerName;

            for (long i = 1; i <= count; ++i)
            {
                auto shape = range->Item[i];
                if (!shape)
                    continue;

                auto& snapshot = shapes_[static_cast<std::size_t>(i - 1)];
                snapshot.shape = shape;

                try { snapshot.staticId = shape->StaticID; }
                catch (...) {}
                try { snapshot.pageIndex = shape->Page->Index; }
                catch (...) {}

                if (!Detail::MeasureShape(shape, snapshot.bounds))
                    continue;

                snapshot.area = std::abs(
                    (snapshot.bounds.right - snapshot.bounds.left) *
                    (snapshot.bounds.top - snapshot.bounds.bottom));
                try
                {
                    snapshot.zOrder = shape->ZOrder;
                    if (i > 1 && snapshot.zOrder != shapes_[0].zOrder)
                        zAllSame = false;
                }
                catch (...)
                {
                    zOk = false;
                }

                try
                {
                    const std::string layerName =
                        Detail::Narrow(shape->Layer->Name) + "@" + std::to_string(snapshot.pageIndex);
                    if (i == 1)
                        firstLayerName = layerName;
                    else if (layerName != firstLayerName)
                        sameLayer = false;
                }
                catch (...)
                {
                    sameLayer = false;
                }
            }

            if (count == 1)
                zAllSame = false;

            zOrderUsable_ = zOk && sameLayer && !zAllSame;
            if (zOrderUsable_)
                CalibrateZOrderDirection();

            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    void HiddenObjectDetector::CalibrateZOrderDirection()
    {
        zOrderUsable_ = false;

        try
        {
            for (std::size_t i = 0; i < shapes_.size(); ++i)
            {
                for (std::size_t j = i + 1; j < shapes_.size(); ++j)
                {
                    if (shapes_[i].zOrder == shapes_[j].zOrder)
                        continue;

                    auto a = ShapeAt(i);
                    auto b = ShapeAt(j);
                    if (!a || !b)
                        return;

                    const bool aheadIsInFront = a->OrderIsInFrontOf(b) != VARIANT_FALSE;
                    zOrderFrontIsHigher_ = aheadIsInFront == (shapes_[i].zOrder > shapes_[j].zOrder);
                    zOrderUsable_ = true;
                    return;
                }
            }
        }
        catch (...)
        {
            zOrderUsable_ = false;
        }
    }

    IVGShapePtr HiddenObjectDetector::ShapeAt(std::size_t index)
    {
        if (index >= shapes_.size())
            return nullptr;

        auto& snapshot = shapes_[index];

        try
        {
            if (snapshot.shape)
            {
                const long probe = snapshot.shape->StaticID;
                if (probe != 0)
                    return snapshot.shape;
            }
        }
        catch (...)
        {
            snapshot.shape = nullptr;
        }

        if (!doc_ || snapshot.staticId == 0 || snapshot.pageIndex < 1)
            return nullptr;

        try
        {
            if (snapshot.pageIndex > doc_->Pages->Count)
                return nullptr;

            auto page = doc_->Pages->Item[snapshot.pageIndex];
            snapshot.shape = page->FindShape(
                _bstr_t(),
                cdrNoShape,
                snapshot.staticId,
                VARIANT_TRUE);
            return snapshot.shape;
        }
        catch (...)
        {
            return nullptr;
        }
    }

    void HiddenObjectDetector::EnsureVisibility(std::size_t index)
    {
        if (index >= shapes_.size() || shapes_[index].visibilityInspected)
            return;

        auto& snapshot = shapes_[index];
        snapshot.visibilityInspected = true;
        auto shape = ShapeAt(index);
        if (!shape)
            return;

        snapshot.solid = ShapeHasSolidInterior(shape);
        snapshot.visible = snapshot.solid;
        if (!snapshot.visible)
        {
            try { snapshot.visible = shape->Outline->Type == cdrOutline; }
            catch (...) {}
        }
    }

    void HiddenObjectDetector::EnsureOpacity(std::size_t index)
    {
        if (index >= shapes_.size() || shapes_[index].opacityInspected)
            return;

        EnsureVisibility(index);
        auto& snapshot = shapes_[index];
        snapshot.opacityInspected = true;
        if (snapshot.visible)
            snapshot.opaque = ShapeIsOpaque(ShapeAt(index));
    }

    std::vector<std::size_t> HiddenObjectDetector::BuildCandidateList(
        std::size_t targetIndex,
        double bboxTolerance)
    {
        std::vector<std::size_t> ordered;

        for (std::size_t j = 0; j < shapes_.size(); ++j)
        {
            if (j == targetIndex)
                continue;
            if (!BoxesIntersect(targetIndex, j, bboxTolerance))
                continue;
            if (!IsAbove(j, targetIndex))
                continue;

            EnsureOpacity(j);
            if (!shapes_[j].opaque)
                continue;

            const auto position = std::find_if(
                ordered.begin(),
                ordered.end(),
                [&](std::size_t existing)
                {
                    return shapes_[j].area > shapes_[existing].area;
                });
            ordered.insert(position, j);
        }

        return ordered;
    }

    bool HiddenObjectDetector::IsAbove(
        std::size_t candidateIndex,
        std::size_t targetIndex)
    {
        if (candidateIndex == targetIndex)
            return false;

        if (zOrderUsable_)
        {
            return zOrderFrontIsHigher_
                ? shapes_[candidateIndex].zOrder > shapes_[targetIndex].zOrder
                : shapes_[candidateIndex].zOrder < shapes_[targetIndex].zOrder;
        }

        auto candidate = ShapeAt(candidateIndex);
        auto target = ShapeAt(targetIndex);
        if (!candidate || !target)
            return false;

        try
        {
            return candidate->OrderIsInFrontOf(target) != VARIANT_FALSE;
        }
        catch (...)
        {
            return false;
        }
    }

    bool HiddenObjectDetector::IsShapeHidden(
        std::size_t targetIndex,
        const std::vector<std::size_t>& candidates,
        double bboxTolerance,
        double pointTolerance,
        double areaTolerance)
    {
        
        
        
        
        if (targetIndex >= shapes_.size() || candidates.empty())
            return false;

        Detail::Bounds coverBounds{};
        bool haveCoverBounds = false;
        for (const auto candidate : candidates)
        {
            if (candidate >= shapes_.size())
                continue;

            const auto& bounds = shapes_[candidate].bounds;
            if (!haveCoverBounds)
            {
                coverBounds = bounds;
                haveCoverBounds = true;
            }
            else
            {
                coverBounds.left = std::min(coverBounds.left, bounds.left);
                coverBounds.right = std::max(coverBounds.right, bounds.right);
                coverBounds.bottom = std::min(coverBounds.bottom, bounds.bottom);
                coverBounds.top = std::max(coverBounds.top, bounds.top);
            }
        }

        const auto& targetBounds = shapes_[targetIndex].bounds;
        if (!haveCoverBounds ||
            coverBounds.left > targetBounds.left + bboxTolerance ||
            coverBounds.right < targetBounds.right - bboxTolerance ||
            coverBounds.bottom > targetBounds.bottom + bboxTolerance ||
            coverBounds.top < targetBounds.top - bboxTolerance)
        {
            return false;
        }

        if (HasExactDuplicateAbove(targetIndex, candidates, bboxTolerance, pointTolerance))
            return true;

        const int booleanResult = BooleanCoverage(targetIndex, candidates, areaTolerance);
        if (booleanResult == 1)
            return true;
        if (booleanResult == -1)
            return false;

        return SampledCoverage(targetIndex, candidates, pointTolerance);
    }

    int HiddenObjectDetector::BooleanCoverage(
        std::size_t targetIndex,
        const std::vector<std::size_t>& candidates,
        double areaTolerance)
    {
        if (!shapes_[targetIndex].solid)
            return 0;

        for (const auto candidate : candidates)
        {
            if (!shapes_[candidate].solid)
                return 0;
        }

        IVGShapePtr footprint;
        IVGShapePtr remaining;
        IVGShapePtr nextRemaining;

        try
        {
            footprint = BuildFootprint(targetIndex);
            if (!footprint)
                return 0;

            remaining = footprint;
            footprint = nullptr;
            bool usedAnyCover = false;

            for (const auto candidate : candidates)
            {
                if (deadline_.Exceeded())
                    throw std::runtime_error("deadline");

                auto coverShape = ShapeAt(candidate);
                if (!coverShape)
                    continue;

                nextRemaining = nullptr;
                try
                {
                    nextRemaining = coverShape->Trim(remaining, VARIANT_TRUE, VARIANT_TRUE);
                    usedAnyCover = true;
                }
                catch (...)
                {
                    continue;
                }

                if (!nextRemaining)
                {
                    Detail::DeleteShape(remaining);
                    return 1;
                }

                Detail::DeleteShape(remaining);
                remaining = nextRemaining;
                nextRemaining = nullptr;

                if (Detail::ClosedArea(remaining) <= areaTolerance)
                {
                    Detail::DeleteShape(remaining);
                    return 1;
                }
            }

            if (!usedAnyCover)
                throw std::runtime_error("no cover");

            const double remainingArea = Detail::ClosedArea(remaining);
            Detail::DeleteShape(remaining);
            return remainingArea <= areaTolerance ? 1 : -1;
        }
        catch (...)
        {
            Detail::DeleteShape(nextRemaining);
            Detail::DeleteShape(remaining);
            Detail::DeleteShape(footprint);
            return 0;
        }
    }

    IVGShapePtr HiddenObjectDetector::BuildFootprint(std::size_t targetIndex)
    {
        if (targetIndex >= shapes_.size() || !shapes_[targetIndex].solid)
            return nullptr;

        auto targetShape = ShapeAt(targetIndex);
        if (!targetShape)
            return nullptr;

        IVGShapePtr region;
        IVGShapePtr expanded;

        try
        {
            auto targetLayer = targetShape->Layer;
            if (!targetLayer)
                targetLayer = app_->ActiveLayer;
            if (!targetLayer)
                return nullptr;

            auto curveCopy = Detail::CurveCopy(targetShape);
            if (!curveCopy)
                return nullptr;

            region = targetLayer->CreateCurve(curveCopy);
            if (!region)
                return nullptr;

            region->Fill->ApplyUniformFill(app_->CreateRGBColor(0, 0, 0));
            region->Outline->SetNoOutline();

            double outlineWidth = 0.0;
            try
            {
                if (targetShape->Outline->Type == cdrOutline)
                    outlineWidth = targetShape->Outline->Width;
            }
            catch (...)
            {
            }

            if (outlineWidth < Detail::Mm(app_, OutlineRelevantMm))
                return region;

            expanded = ExpandByOutlineWeld(region, outlineWidth * 0.5);
            if (!expanded)
                return region;

            return expanded;
        }
        catch (...)
        {
            Detail::DeleteShape(expanded);
            Detail::DeleteShape(region);
            return nullptr;
        }
    }


    IVGShapePtr HiddenObjectDetector::ExpandByOutlineWeld(
        IVGShapePtr& baseShape,
        double offsetValue)
    {
        if (!baseShape || offsetValue <= 0.0)
            return nullptr;

        IVGShapePtr pathShape;
        IVGShapePtr bandShape;
        IVGShapePtr baseCopy;
        IVGShapePtr result;

        try
        {
            auto targetLayer = baseShape->Layer;
            if (!targetLayer)
                targetLayer = app_->ActiveLayer;
            if (!targetLayer)
                return nullptr;

            auto boundaryCurve = Detail::CurveCopy(baseShape);
            if (!boundaryCurve)
                return nullptr;

            pathShape = targetLayer->CreateCurve(boundaryCurve);
            if (!pathShape)
                return nullptr;

            pathShape->Fill->ApplyNoFill();

            auto outline = pathShape->Outline;
            outline->Color->RGBAssign(0, 0, 0);
            outline->Width = offsetValue * 2.0;
            outline->LineJoin = cdrOutlineRoundLineJoin;
            outline->LineCaps = cdrOutlineRoundLineCaps;
            outline->BehindFill = VARIANT_FALSE;

            bandShape = outline->ConvertToObject();
            if (!bandShape)
                return nullptr;

            try
            {
                if (bandShape->Type != cdrCurveShape)
                    bandShape->ConvertToCurves();
            }
            catch (...)
            {
            }

            bandShape->Fill->ApplyUniformFill(
                app_->CreateRGBColor(0, 0, 0));
            bandShape->Outline->SetNoOutline();

            baseCopy = baseShape->Duplicate(0.0, 0.0);
            if (!baseCopy)
                return nullptr;

            result = bandShape->Weld(
                baseCopy,
                VARIANT_FALSE,
                VARIANT_FALSE);

            if (!result)
                return nullptr;

            Detail::DeleteShape(pathShape);
            Detail::DeleteShape(baseShape);
            baseShape = nullptr;

            return result;
        }
        catch (...)
        {
            Detail::DeleteShape(result);
            Detail::DeleteShape(baseCopy);
            Detail::DeleteShape(bandShape);
            Detail::DeleteShape(pathShape);
            return nullptr;
        }
    }


    bool HiddenObjectDetector::SampledCoverage(
        std::size_t targetIndex,
        const std::vector<std::size_t>& candidates,
        double tolerance)
    {
        auto targetShape = ShapeAt(targetIndex);
        if (!targetShape)
            return false;

        auto candidateShapes = ResolveCandidateShapes(candidates);
        if (candidateShapes.empty())
            return false;

        long tested = 0;
        if (!SamplePass(targetShape, candidateShapes, tolerance, CoarseGrid, CoarseHalton, false, tested))
            return false;

        if (deadline_.Exceeded())
            return false;

        tested = 0;
        if (!SamplePass(targetShape, candidateShapes, tolerance, FineGrid, FineHalton, true, tested))
            return false;

        return tested >= MinConfidentSamples;
    }

    bool HiddenObjectDetector::SamplePass(
        const IVGShapePtr& targetShape,
        const std::vector<IVGShapePtr>& candidateShapes,
        double tolerance,
        long gridSize,
        long haltonCount,
        bool includeBoundary,
        long& tested)
    {
        if (!targetShape)
            return false;

        try
        {
            const double left = targetShape->LeftX;
            const double bottom = targetShape->BottomY;
            const double width = targetShape->SizeWidth;
            const double height = targetShape->SizeHeight;

            if (width <= 0.0 || height <= 0.0)
                return true;

            for (long gy = 0; gy < gridSize; ++gy)
            {
                const double v = (static_cast<double>(gy) + 0.5) / static_cast<double>(gridSize);
                const double y = bottom + height * v;

                for (long gx = 0; gx < gridSize; ++gx)
                {
                    const double u = (static_cast<double>(gx) + 0.5) / static_cast<double>(gridSize);
                    const double x = left + width * u;

                    if (PointBelongsToShape(targetShape, x, y, tolerance))
                    {
                        ++tested;
                        if (!PointCoveredByAny(candidateShapes, x, y, tolerance))
                            return false;
                    }
                }
            }

            for (long i = 1; i <= haltonCount; ++i)
            {
                const double x = left + width * Halton(i, 2);
                const double y = bottom + height * Halton(i, 3);

                if (PointBelongsToShape(targetShape, x, y, tolerance))
                {
                    ++tested;
                    if (!PointCoveredByAny(candidateShapes, x, y, tolerance))
                        return false;
                }
            }

            if (includeBoundary)
            {
                auto targetCurve = Detail::CurveCopy(targetShape);
                if (targetCurve)
                {
                    auto subPaths = targetCurve->SubPaths;
                    for (long s = 1; s <= subPaths->Count; ++s)
                    {
                        auto subPath = subPaths->Item[s];
                        if (!subPath)
                            continue;

                        const long boundaryCount = BoundarySampleCount(subPath);
                        for (long i = 1; i <= boundaryCount; ++i)
                        {
                            double x = 0.0;
                            double y = 0.0;
                            const double offset = (static_cast<double>(i) - 0.5) / static_cast<double>(boundaryCount);
                            subPath->GetPointPositionAt(&x, &y, offset, cdrRelativeSegmentOffset);

                            if (PointBelongsToShape(targetShape, x, y, tolerance))
                            {
                                ++tested;
                                if (!PointCoveredByAny(candidateShapes, x, y, tolerance))
                                    return false;
                            }
                        }
                    }
                }
            }

            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    std::vector<IVGShapePtr> HiddenObjectDetector::ResolveCandidateShapes(
        const std::vector<std::size_t>& candidates)
    {
        std::vector<IVGShapePtr> result;
        result.reserve(candidates.size());

        for (const auto index : candidates)
        {
            auto shape = ShapeAt(index);
            if (shape)
                result.push_back(shape);
        }

        return result;
    }

    bool HiddenObjectDetector::PointCoveredByAny(
        const std::vector<IVGShapePtr>& candidateShapes,
        double x,
        double y,
        double tolerance)
    {
        for (const auto& shape : candidateShapes)
        {
            if (!shape)
                continue;

            try
            {
                if (x >= shape->LeftX - tolerance &&
                    x <= shape->RightX + tolerance &&
                    y >= shape->BottomY - tolerance &&
                    y <= shape->TopY + tolerance &&
                    PointCoveredByShape(shape, x, y, tolerance))
                {
                    return true;
                }
            }
            catch (...)
            {
            }
        }

        return false;
    }

    bool HiddenObjectDetector::PointBelongsToShape(
        const IVGShapePtr& shape,
        double x,
        double y,
        double tolerance)
    {
        if (!shape)
            return false;

        try
        {
            if (shape->Type == cdrGroupShape)
            {
                auto children = shape->Shapes;
                for (long i = 1; i <= children->Count; ++i)
                {
                    if (PointBelongsToShape(children->Item[i], x, y, tolerance))
                        return true;
                }
                return false;
            }

            const auto position = shape->IsOnShape(x, y, tolerance);
            if (position == cdrInsideShape)
                return ShapeHasSolidInterior(shape);
            if (position == cdrOnMarginOfShape)
                return ShapeHasSolidInterior(shape) || shape->Outline->Type == cdrOutline;
        }
        catch (...)
        {
        }

        return false;
    }

    bool HiddenObjectDetector::PointCoveredByShape(
        const IVGShapePtr& shape,
        double x,
        double y,
        double tolerance)
    {
        if (!shape)
            return false;

        try
        {
            if (shape->Type == cdrGroupShape)
            {
                auto children = shape->Shapes;
                for (long i = 1; i <= children->Count; ++i)
                {
                    auto child = children->Item[i];
                    if (ShapeIsOpaque(child) && PointCoveredByShape(child, x, y, tolerance))
                        return true;
                }
                return false;
            }

            if (!ShapeIsOpaque(shape))
                return false;

            const auto position = shape->IsOnShape(x, y, tolerance);
            if (position == cdrInsideShape)
                return ShapeHasSolidInterior(shape);
            if (position == cdrOnMarginOfShape)
                return ShapeHasSolidInterior(shape) || shape->Outline->Type == cdrOutline;
        }
        catch (...)
        {
        }

        return false;
    }

    bool HiddenObjectDetector::HasExactDuplicateAbove(
        std::size_t targetIndex,
        const std::vector<std::size_t>& candidates,
        double bboxTolerance,
        double pointTolerance)
    {
        for (const auto candidate : candidates)
        {
            if (!BoxesEqual(targetIndex, candidate, bboxTolerance))
                continue;

            auto targetShape = ShapeAt(targetIndex);
            auto candidateShape = ShapeAt(candidate);
            if (targetShape && candidateShape && CurvesEquivalent(targetShape, candidateShape, pointTolerance))
                return true;
        }

        return false;
    }

    bool HiddenObjectDetector::CurvesEquivalent(
        const IVGShapePtr& firstShape,
        const IVGShapePtr& secondShape,
        double tolerance)
    {
        auto firstCurve = Detail::CurveCopy(firstShape);
        auto secondCurve = Detail::CurveCopy(secondShape);
        if (!firstCurve || !secondCurve)
            return false;

        try
        {
            auto firstSubPaths = firstCurve->SubPaths;
            auto secondSubPaths = secondCurve->SubPaths;

            if (firstSubPaths->Count != secondSubPaths->Count || firstSubPaths->Count == 0)
                return false;

            std::vector<bool> matched(static_cast<std::size_t>(secondSubPaths->Count), false);

            for (long i = 1; i <= firstSubPaths->Count; ++i)
            {
                bool found = false;
                for (long j = 1; j <= secondSubPaths->Count; ++j)
                {
                    if (matched[static_cast<std::size_t>(j - 1)])
                        continue;

                    if (SubPathsEquivalent(firstSubPaths->Item[i], secondSubPaths->Item[j], tolerance))
                    {
                        matched[static_cast<std::size_t>(j - 1)] = true;
                        found = true;
                        break;
                    }
                }

                if (!found)
                    return false;
            }

            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool HiddenObjectDetector::SubPathsEquivalent(
        const IVGSubPathPtr& firstSubPath,
        const IVGSubPathPtr& secondSubPath,
        double tolerance)
    {
        if (!firstSubPath || !secondSubPath)
            return false;

        try
        {
            if (firstSubPath->Closed != secondSubPath->Closed)
                return false;
            if (firstSubPath->Nodes->Count != secondSubPath->Nodes->Count)
                return false;
            if (firstSubPath->Segments->Count != secondSubPath->Segments->Count)
                return false;

            auto firstBox = firstSubPath->BoundingBox;
            auto secondBox = secondSubPath->BoundingBox;
            if (!firstBox || !secondBox)
                return false;

            if (std::abs(firstBox->Left - secondBox->Left) > tolerance)
                return false;
            if (std::abs(firstBox->Right - secondBox->Right) > tolerance)
                return false;
            if (std::abs(firstBox->Bottom - secondBox->Bottom) > tolerance)
                return false;
            if (std::abs(firstBox->Top - secondBox->Top) > tolerance)
                return false;
            if (std::abs(firstSubPath->Length - secondSubPath->Length) > tolerance * 8.0)
                return false;

            return SubPathSamplesMatch(firstSubPath, secondSubPath, tolerance);
        }
        catch (...)
        {
            return false;
        }
    }

    bool HiddenObjectDetector::SubPathSamplesMatch(
        const IVGSubPathPtr& sourceSubPath,
        const IVGSubPathPtr& testSubPath,
        double tolerance)
    {
        if (!sourceSubPath || !testSubPath)
            return false;

        try
        {
            const long sampleCount = BoundarySampleCount(sourceSubPath);
            for (long i = 1; i <= sampleCount; ++i)
            {
                double x = 0.0;
                double y = 0.0;
                const double offset = (static_cast<double>(i) - 0.5) / static_cast<double>(sampleCount);
                sourceSubPath->GetPointPositionAt(&x, &y, offset, cdrRelativeSegmentOffset);
                if (testSubPath->IsOnSubPath(x, y, tolerance) != cdrOnMarginOfShape)
                    return false;
            }
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool HiddenObjectDetector::ShapeHasSolidInterior(const IVGShapePtr& shape)
    {
        if (!shape)
            return false;

        try
        {
            if (shape->Type == cdrGroupShape)
            {
                auto children = shape->Shapes;
                for (long i = 1; i <= children->Count; ++i)
                {
                    if (ShapeHasSolidInterior(children->Item[i]))
                        return true;
                }
                return false;
            }

            if (shape->Type == cdrBitmapShape)
                return true;
            if (shape->Fill->Type != cdrNoFill)
                return true;
            return Detail::HasPowerClip(shape);
        }
        catch (...)
        {
            return false;
        }
    }

    bool HiddenObjectDetector::ShapeIsOpaque(const IVGShapePtr& shape)
    {
        if (!shape)
            return false;

        try
        {
            if (shape->Type == cdrGroupShape)
            {
                auto children = shape->Shapes;
                for (long i = 1; i <= children->Count; ++i)
                {
                    if (ShapeIsOpaque(children->Item[i]))
                        return true;
                }
                return false;
            }

            return shape->Transparency->Type == cdrNoTransparency;
        }
        catch (...)
        {
            return false;
        }
    }

    void HiddenObjectDetector::ApplyHiddenAction(const IVGShapeRangePtr& hiddenRange)
    {
        if (!hiddenRange)
            return;

        try
        {
            if (action_ == HiddenObjectAction::LeaveInPlace)
                return;

            if (action_ == HiddenObjectAction::MoveToLayer)
            {
                auto hiddenLayer = GetOrCreateHiddenLayer();
                if (hiddenLayer)
                    hiddenRange->MoveToLayer(hiddenLayer);
                return;
            }

            hiddenRange->Move(Detail::Mm(app_, MoveDistanceMm), 0.0);
            try
            {
                app_->ActiveWindow->ActiveView->ToFitShapeRange(hiddenRange);
            }
            catch (...)
            {
            }
        }
        catch (...)
        {
        }
    }

    IVGLayerPtr HiddenObjectDetector::GetOrCreateHiddenLayer()
    {
        IVGPagePtr page;
        try { page = app_->ActivePage; }
        catch (...) {}
        if (!page)
            return nullptr;

        try
        {
            auto layers = page->Layers;
            for (long i = 1; i <= layers->Count; ++i)
            {
                auto layer = layers->Item[i];
                if (!layer)
                    continue;

                if (_stricmp(Detail::Narrow(layer->Name).c_str(), HiddenLayerName) == 0)
                {
                    layer->Editable = VARIANT_TRUE;
                    return layer;
                }
            }

            auto layer = page->CreateLayer(HiddenLayerName);
            if (layer)
            {
                layer->Editable = VARIANT_TRUE;
                layer->Printable = VARIANT_FALSE;
                return layer;
            }
        }
        catch (...)
        {
        }

        return nullptr;
    }

    bool HiddenObjectDetector::BoxesIntersect(
        std::size_t a,
        std::size_t b,
        double tolerance) const noexcept
    {
        if (a >= shapes_.size() || b >= shapes_.size())
            return false;

        const auto& boxA = shapes_[a].bounds;
        const auto& boxB = shapes_[b].bounds;

        return
            boxA.left <= boxB.right + tolerance &&
            boxA.right >= boxB.left - tolerance &&
            boxA.bottom <= boxB.top + tolerance &&
            boxA.top >= boxB.bottom - tolerance;
    }

    bool HiddenObjectDetector::BoxesEqual(
        std::size_t a,
        std::size_t b,
        double tolerance) const noexcept
    {
        if (a >= shapes_.size() || b >= shapes_.size())
            return false;

        const auto& boxA = shapes_[a].bounds;
        const auto& boxB = shapes_[b].bounds;

        return
            std::abs(boxA.left - boxB.left) <= tolerance &&
            std::abs(boxA.right - boxB.right) <= tolerance &&
            std::abs(boxA.bottom - boxB.bottom) <= tolerance &&
            std::abs(boxA.top - boxB.top) <= tolerance;
    }

    long HiddenObjectDetector::BoundarySampleCount(const IVGSubPathPtr& subPath) const
    {
        if (!subPath)
            return BoundarySamplesMin;

        try
        {
            auto box = subPath->BoundingBox;
            if (!box)
                return BoundarySamplesMin;

            const double perimeter = 2.0 * (std::abs(box->Width) + std::abs(box->Height));
            const double desiredStep = Detail::Mm(app_, 0.5);
            long result = BoundarySamplesMin;

            if (desiredStep > 0.0)
                result = static_cast<long>(perimeter / desiredStep);

            return std::clamp(result, BoundarySamplesMin, BoundarySamplesMax);
        }
        catch (...)
        {
            return BoundarySamplesMin;
        }
    }

    double HiddenObjectDetector::Halton(long index, long base) noexcept
    {
        double fraction = 1.0;
        double result = 0.0;
        long current = index;

        while (current > 0)
        {
            fraction /= static_cast<double>(base);
            result += fraction * static_cast<double>(current % base);
            current /= base;
        }

        return result;
    }

    void HiddenObjectDetector::ClearContext()
    {
        doc_ = nullptr;
        shapes_.clear();
        zOrderUsable_ = false;
        zOrderFrontIsHigher_ = false;
    }

    Processor::Processor(IVGApplicationPtr app)
        : app_(std::move(app))
    {
    }

    Result Processor::Run(
        const Settings& settings,
        const Callbacks& callbacks)
    {
        const auto operationStart = std::chrono::steady_clock::now();
        Result result;
        heavyDecision_ = callbacks.onHeavyShape
            ? HeavyShapeDecision::ProcessOnce
            : HeavyShapeDecision::ProcessAll;
        pendingPowerClipFinalizations_.clear();
        cutlineMs_ = 0.0;

        if (!app_)
        {
            result.fatalError = "Application CorelDRAW invalida.";
            return result;
        }

        IVGDocumentPtr doc;
        IVGShapeRangePtr selected;
        IVGShapeRangePtr resultRange;
        std::vector<IVGShapePtr> completedResults;
        UniversalBleedOffset bleedEngine(app_);
        bool batchStarted = false;

        try
        {
            if (!app_->Documents || app_->Documents->Count == 0)
                throw std::runtime_error("Abra um documento antes de criar a sangria.");

            doc = app_->ActiveDocument;
            if (!doc)
                throw std::runtime_error("Nenhum documento ativo.");

            selected = app_->ActiveSelectionRange;
            if (!selected || selected->Count == 0)
                throw std::runtime_error("Selecione pelo menos um objeto.");


            if (!InputLimits::ValidBleed(settings.bleedMillimeters))
                throw std::runtime_error("A distancia de sangria deve ser finita e estar entre 0,001 e 1000 mm.");

            const double bleedDistance = app_->ConvertUnits(
                settings.bleedMillimeters,
                cdrMillimeter,
                doc->Unit);

            if (!std::isfinite(bleedDistance) || bleedDistance <= 0.0)
                throw std::runtime_error("Nao foi possivel converter a distancia de sangria.");

            {
                Detail::CorelExecutionGuard guard(app_, doc, "AutoBleed Universal");

                bleedEngine.SetTimeBudgetSeconds(settings.bleedTimeBudgetSeconds);
                bleedEngine.BeginBatch();
                batchStarted = true;

                if (settings.flattenGroups)
                {
                    Progress(callbacks, 0, selected->Count, "Desagrupando objetos selecionados...");
                    selected = FlattenSelectedGroups(selected);

                }

                if (settings.detectHidden)
                {
                    Progress(callbacks, 0, selected->Count, "Analisando objetos totalmente cobertos...");
                    HiddenObjectDetector detector(app_);
                    detector.SetTimeBudgetSeconds(settings.hiddenTimeBudgetSeconds);
                    detector.SetAction(settings.hiddenAction);
                    detector.Process(selected, result.hiddenCount);

                    if (!detector.GetLastReport().empty())
                        Progress(callbacks, 0, selected ? selected->Count : 0, detector.GetLastReport());
                }

                std::vector<IVGShapePtr> workItems;
                if (selected && selected->Count > 0)
                {
                    workItems.reserve(static_cast<std::size_t>(selected->Count));

                    for (long index = 1; index <= selected->Count; ++index)
                    {
                        try
                        {
                            auto item = selected->Item[index];
                            if (item)
                                workItems.push_back(item);
                        }
                        catch (...)
                        {
                        }
                    }
                }

                result.totalCount = static_cast<long>(workItems.size());

                Progress(
                    callbacks,
                    0,
                    result.totalCount,
                    "Processando 0/" + std::to_string(result.totalCount) + "...");

                auto nextCheckpoint = std::chrono::steady_clock::now() +
                    std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                        std::chrono::duration<double>(std::max(1.0, settings.checkpointSeconds)));

                for (long i = result.totalCount; i >= 1; --i)
                {
                    IVGShapePtr sourceShape;

                    const auto workIndex = static_cast<std::size_t>(i - 1);
                    if (workIndex < workItems.size())
                        sourceShape = workItems[workIndex];

                    if (!sourceShape)
                    {
                        ++result.skippedCount;
                    }
                    else
                    {
                        ++result.processedCount;

                        if (SkipHeavyShape(
                            sourceShape,
                            result.processedCount,
                            result.totalCount,
                            settings,
                            callbacks,
                            guard))
                        {
                            ++result.skippedCount;
                            Progress(
                                callbacks,
                                result.processedCount,
                                result.totalCount,
                                "Ignorado a pedido: objeto muito complexo");
                        }
                        else
                        {
                            IVGShapePtr resultGroup;
                            std::string objectError;
                            std::string strategyNote;
                            const std::size_t pendingBefore =
                                pendingPowerClipFinalizations_.size();

                            if (ProcessUniversalBleedShape(
                                sourceShape,
                                bleedDistance,
                                result.processedCount,
                                bleedEngine,
                                settings,
                                resultGroup,
                                objectError,
                                strategyNote))
                            {
                                const bool waitsPowerClipFinalize =
                                    pendingPowerClipFinalizations_.size() > pendingBefore;

                                if (!waitsPowerClipFinalize)
                                {
                                    ++result.successCount;

                                    if (resultGroup)
                                        completedResults.push_back(resultGroup);

                                    if (!strategyNote.empty())
                                    {
                                        ++result.fallbackCount;
                                        Progress(
                                            callbacks,
                                            result.processedCount,
                                            result.totalCount,
                                            "Fallback: " + strategyNote);
                                    }
                                }
                                else if (!strategyNote.empty())
                                {
                                    pendingPowerClipFinalizations_.back().strategyNote = strategyNote;
                                }
                            }
                            else
                            {
                                ++result.failedCount;
                                Progress(
                                    callbacks,
                                    result.processedCount,
                                    result.totalCount,
                                    "Falha " + std::to_string(result.processedCount) + ": " + objectError);
                            }
                        }
                    }

                    Progress(
                        callbacks,
                        result.processedCount,
                        result.totalCount,
                        "Processando " + std::to_string(result.processedCount) +
                        "/" + std::to_string(result.totalCount) + "...");

                    Detail::PumpMessages();

                    if (settings.checkpointSeconds > 0.0 &&
                        std::chrono::steady_clock::now() > nextCheckpoint)
                    {
                        guard.RestoreEventsTemporarily();

                        bool shouldContinue = true;
                        if (callbacks.onCheckpoint)
                        {
                            shouldContinue = callbacks.onCheckpoint(
                                result.processedCount,
                                result.totalCount);
                        }

                        guard.DisableEventsAgain();

                        if (!shouldContinue)
                        {
                            result.userAborted = true;
                            result.skippedCount += i - 1;
                            break;
                        }

                        nextCheckpoint = std::chrono::steady_clock::now() +
                            std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                std::chrono::duration<double>(settings.checkpointSeconds));
                    }
                }

                result.cacheReport = bleedEngine.GetCacheStats();
                bleedEngine.EndBatch();
                batchStarted = false;

                
                
                
                
                
                if (!pendingPowerClipFinalizations_.empty())
                {
                    guard.RestoreEventsTemporarily();

                    try
                    {
                        app_->PutOptimization(VARIANT_FALSE);
                        if (app_->ActiveWindow)
                            app_->ActiveWindow->Refresh();
                        app_->Refresh();
                    }
                    catch (...)
                    {
                    }

                    Detail::PumpMessages();
                }

                for (auto& pending : pendingPowerClipFinalizations_)
                {
                    std::string normalizeDiagnostic;




                    if (!NormalizePowerClipContents(
                        pending.generatedPowerClip,
                        normalizeDiagnostic))
                    {
                        ++result.failedCount;

                        Progress(
                            callbacks,
                            pending.itemNumber,
                            result.totalCount,
                            "Falha " + std::to_string(pending.itemNumber) +
                            " ao finalizar PowerClip: " + normalizeDiagnostic);

                        RollbackPendingPowerClip(pending);
                        continue;
                    }


                    IVGShapePtr finalizedResult;
                    bool finalizeFailed = false;
                    std::string finalizeDiagnostic;

                    try
                    {
                        if (pending.preserveGroup)
                        {
                            auto groupRange = app_->CreateShapeRange();
                            if (!groupRange)
                                throw std::runtime_error("nao foi possivel criar o agrupamento final do PowerClip");

                            groupRange->Add(pending.generatedPowerClip);
                            groupRange->Add(pending.sourceShape);

                            if (pending.generatedCutline)
                                groupRange->Add(pending.generatedCutline);

                            finalizedResult = groupRange->Group();
                        }
                        else if (pending.generatedCutline)
                        {
                            auto groupRange = app_->CreateShapeRange();
                            if (!groupRange)
                                throw std::runtime_error("nao foi possivel criar o agrupamento final da sangria");

                            groupRange->Add(pending.generatedPowerClip);
                            groupRange->Add(pending.generatedCutline);
                            finalizedResult = groupRange->Group();
                        }
                        else
                        {
                            finalizedResult = pending.generatedPowerClip;
                        }

                        if (!finalizedResult)
                            throw std::runtime_error("o CorelDRAW nao retornou o resultado final da sangria");
                    }
                    catch (const _com_error& error)
                    {
                        finalizeFailed = true;
                        finalizeDiagnostic = Detail::ComErrorText(error);
                    }
                    catch (const std::exception& error)
                    {
                        finalizeFailed = true;
                        finalizeDiagnostic = error.what();
                    }
                    catch (...)
                    {
                        finalizeFailed = true;
                        finalizeDiagnostic = "erro desconhecido no agrupamento final";
                    }

                    if (finalizeFailed)
                    {
                        ++result.failedCount;
                        Progress(
                            callbacks,
                            pending.itemNumber,
                            result.totalCount,
                            "Falha " + std::to_string(pending.itemNumber) +
                            " ao agrupar resultado: " + finalizeDiagnostic);
                        RollbackPendingPowerClip(pending);
                        continue;
                    }

                    bool sourceDeleteFailed = false;

                    if (!pending.preserveGroup && pending.sourceShape)
                    {
                        try
                        {
                            pending.sourceShape->Delete();
                        }
                        catch (...)
                        {
                            sourceDeleteFailed = true;
                        }
                    }


                    ++result.successCount;
                    completedResults.push_back(finalizedResult);

                    pending.generatedPowerClip = nullptr;
                    pending.generatedCutline = nullptr;
                    pending.sourceShape = nullptr;

                    if (!pending.strategyNote.empty())
                    {
                        ++result.fallbackCount;
                        Progress(
                            callbacks,
                            pending.itemNumber,
                            result.totalCount,
                            "Fallback: " + pending.strategyNote);
                    }

                    if (sourceDeleteFailed)
                    {
                        Progress(
                            callbacks,
                            pending.itemNumber,
                            result.totalCount,
                            "Aviso: a sangria foi concluida, mas o objeto original nao pode ser removido.");
                    }
                }

                pendingPowerClipFinalizations_.clear();
            }
        }
        catch (const _com_error& error)
        {
            result.fatalError = "CorelDRAW COM error: " + Detail::ComErrorText(error);
        }
        catch (const std::exception& error)
        {
            result.fatalError = error.what();
        }
        catch (...)
        {
            result.fatalError = "Erro desconhecido durante o Auto Bleeding.";
        }

        if (batchStarted)
        {
            result.cacheReport = bleedEngine.GetCacheStats();
            bleedEngine.EndBatch();
        }

        if (!pendingPowerClipFinalizations_.empty())
        {
            for (auto& pending : pendingPowerClipFinalizations_)
                RollbackPendingPowerClip(pending);

            pendingPowerClipFinalizations_.clear();
        }

        try
        {
            resultRange = app_->CreateShapeRange();

            if (resultRange)
            {
                for (const auto& shape : completedResults)
                {
                    if (!shape)
                        continue;

                    try
                    {
                        resultRange->Add(shape);
                    }
                    catch (...)
                    {
                    }
                }

                if (resultRange->Count > 0)
                    resultRange->CreateSelection();
            }

            if (app_->ActiveWindow)
                app_->ActiveWindow->Refresh();
        }
        catch (...)
        {
        }

        Progress(callbacks, result.processedCount, result.totalCount, "Concluidos: " + std::to_string(result.successCount));
        Progress(callbacks, result.processedCount, result.totalCount, "Falhas preservadas: " + std::to_string(result.failedCount));

        if (result.skippedCount > 0)
            Progress(callbacks, result.processedCount, result.totalCount, "Nao processados: " + std::to_string(result.skippedCount));

        Progress(callbacks, result.processedCount, result.totalCount, "Ocultos ignorados: " + std::to_string(result.hiddenCount));
        Progress(callbacks, result.processedCount, result.totalCount, "Fallbacks geometricos: " + std::to_string(result.fallbackCount));

        if (!result.cacheReport.empty())
            Progress(callbacks, result.processedCount, result.totalCount, "Cache: " + result.cacheReport);

        Progress(callbacks, result.processedCount, result.totalCount, "Pronto");

        const auto& performance = bleedEngine.GetPerformanceStats();
        result.preparationMs = performance.preparationMs;
        result.cacheMs = performance.cacheMs;
        result.generationMs = performance.generationMs;
        result.validationMs = performance.validationMs;
        result.cutlineMs = cutlineMs_;
        result.generationAttempts = performance.generationAttempts;
        result.elapsedMs = ElapsedMilliseconds(operationStart);

        return result;
    }

    IVGShapeRangePtr Processor::FlattenSelectedGroups(const IVGShapeRangePtr& sourceRange) const
    {
        if (!sourceRange)
            return nullptr;

        try
        {
            auto resultRange = sourceRange;

            for (long guard = 0; guard < 64; ++guard)
            {
                bool foundGroup = false;
                auto nextRange = app_->CreateShapeRange();

                for (long i = 1; i <= resultRange->Count; ++i)
                {
                    auto shape = resultRange->Item[i];
                    if (!shape)
                        continue;

                    if (shape->Type == cdrGroupShape)
                    {
                        auto ungrouped = shape->UngroupEx();
                        if (ungrouped)
                            nextRange->AddRange(ungrouped);
                        foundGroup = true;
                    }
                    else
                    {
                        nextRange->Add(shape);
                    }
                }

                resultRange = nextRange;
                if (!foundGroup)
                    break;
            }

            return resultRange;
        }
        catch (...)
        {
            return sourceRange;
        }
    }

    bool Processor::IsGroupShape(const IVGShapePtr& shape) const
    {
        if (!shape)
            return false;

        try
        {
            return shape->Type == cdrGroupShape;
        }
        catch (...)
        {
            return false;
        }
    }

    void Processor::FindDominantGroupArtworkShapeRecursive(
        const IVGShapePtr& shape,
        IVGShapePtr& bestPrimary,
        double& bestPrimaryScore,
        IVGShapePtr& bestSecondary,
        double& bestSecondaryScore) const
    {
        if (!shape)
            return;

        try
        {
            if (shape->Type == cdrGroupShape)
            {
                auto children = shape->Shapes;
                if (!children)
                    return;

                for (long i = 1; i <= children->Count; ++i)
                {
                    FindDominantGroupArtworkShapeRecursive(
                        children->Item[i],
                        bestPrimary,
                        bestPrimaryScore,
                        bestSecondary,
                        bestSecondaryScore);
                }
                return;
            }

            if (shape->Type == cdrTextShape)
                return;
        }
        catch (...)
        {
            return;
        }

        const double closedArea = Detail::ClosedArea(shape);
        if (closedArea <= 0.0)
            return;

        Detail::Bounds bounds;
        const bool measured = Detail::MeasureShape(shape, bounds);
        const double bboxArea = measured
            ? std::max(0.0, bounds.right - bounds.left) *
            std::max(0.0, bounds.top - bounds.bottom)
            : 0.0;

        bool hasVisibleFill = false;
        try
        {
            hasVisibleFill =
                shape->CanHaveFill &&
                shape->Fill &&
                shape->Fill->Type != cdrNoFill;
        }
        catch (...)
        {
            hasVisibleFill = false;
        }

        const bool hasPowerClip = Detail::HasPowerClip(shape);
        double score = closedArea + bboxArea * 0.05;

        if (hasPowerClip)
            score *= 1.20;
        else if (hasVisibleFill)
            score *= 1.05;

        if (hasPowerClip || hasVisibleFill)
        {
            if (score > bestPrimaryScore)
            {
                bestPrimaryScore = score;
                bestPrimary = shape;
            }
            return;
        }

        if (score > bestSecondaryScore)
        {
            bestSecondaryScore = score;
            bestSecondary = shape;
        }
    }

    void Processor::CollectGroupArtworkShapesRecursive(
        const IVGShapePtr& shape,
        std::vector<IVGShapePtr>& primary,
        std::vector<IVGShapePtr>& secondary) const
    {
        if (!shape)
            return;

        try
        {
            if (shape->Type == cdrGroupShape)
            {
                auto children = shape->Shapes;
                if (!children)
                    return;

                for (long i = 1; i <= children->Count; ++i)
                {
                    CollectGroupArtworkShapesRecursive(
                        children->Item[i],
                        primary,
                        secondary);
                }
                return;
            }

            if (shape->Type == cdrTextShape)
                return;
        }
        catch (...)
        {
            return;
        }

        if (Detail::ClosedArea(shape) <= 0.0)
            return;

        bool hasVisibleFill = false;
        try
        {
            hasVisibleFill =
                shape->CanHaveFill &&
                shape->Fill &&
                shape->Fill->Type != cdrNoFill;
        }
        catch (...)
        {
            hasVisibleFill = false;
        }

        if (Detail::HasPowerClip(shape) || hasVisibleFill)
            primary.push_back(shape);
        else
            secondary.push_back(shape);
    }

    Processor::GroupBleedSources Processor::ResolveGroupBleedSources(
        const IVGShapePtr& groupShape) const
    {
        GroupBleedSources result;

        if (!IsGroupShape(groupShape))
            return result;

        try
        {
            auto cutShapes = groupShape->Shapes->FindShapes(
                _bstr_t(),
                cdrNoShape,
                VARIANT_TRUE,
                _bstr_t("@outline.color.name = 'CutContour'"));

            if (cutShapes && cutShapes->Count > 0)
            {
                result.geometrySources.reserve(
                    static_cast<std::size_t>(cutShapes->Count));

                for (long i = 1; i <= cutShapes->Count; ++i)
                {
                    auto shape = cutShapes->Item[i];
                    if (!shape)
                        continue;

                    auto curve = Detail::CurveCopy(shape);
                    if (!curve || !curve->SubPaths || curve->SubPaths->Count <= 0)
                        continue;

                    result.geometrySources.push_back(shape);
                }

                result.usesExistingCutline =
                    !result.geometrySources.empty();
            }
        }
        catch (...)
        {
        }

        std::vector<IVGShapePtr> primary;
        std::vector<IVGShapePtr> secondary;

        CollectGroupArtworkShapesRecursive(
            groupShape,
            primary,
            secondary);

        IVGShapePtr bestPrimary;
        IVGShapePtr bestSecondary;
        double bestPrimaryScore = -1.0;
        double bestSecondaryScore = -1.0;

        FindDominantGroupArtworkShapeRecursive(
            groupShape,
            bestPrimary,
            bestPrimaryScore,
            bestSecondary,
            bestSecondaryScore);

        result.appearanceSource =
            bestPrimary ? bestPrimary : bestSecondary;

        if (result.geometrySources.empty())
        {
            result.geometrySources.reserve(
                primary.size() + secondary.size());

            result.geometrySources.insert(
                result.geometrySources.end(),
                primary.begin(),
                primary.end());

            result.geometrySources.insert(
                result.geometrySources.end(),
                secondary.begin(),
                secondary.end());
        }

        if (!result.appearanceSource &&
            !result.geometrySources.empty())
        {
            result.appearanceSource =
                result.geometrySources.front();
        }

        return result;
    }

    IVGShapePtr Processor::CreateStandaloneGeometryProxy(
        const IVGShapePtr& sourceShape,
        const IVGLayerPtr& targetLayer) const
    {
        if (!sourceShape)
            return nullptr;

        return CreateStandaloneGeometryProxy(
            std::vector<IVGShapePtr>{ sourceShape },
            targetLayer);
    }

    IVGShapePtr Processor::CreateStandaloneGeometryProxy(
        const std::vector<IVGShapePtr>& sourceShapes,
        const IVGLayerPtr& targetLayer) const
    {
        if (sourceShapes.empty())
            return nullptr;

        IVGLayerPtr layer = targetLayer;
        IVGShapePtr accumulated;
        IVGShapePtr current;

        try
        {
            if (!layer)
            {
                for (const auto& source : sourceShapes)
                {
                    if (!source)
                        continue;

                    try
                    {
                        layer = source->Layer;
                    }
                    catch (...)
                    {
                    }

                    if (layer)
                        break;
                }
            }

            if (!layer)
                layer = app_->ActiveLayer;
            if (!layer)
                return nullptr;

            for (const auto& source : sourceShapes)
            {
                if (!source)
                    continue;

                auto curve = Detail::CurveCopy(source);
                if (!curve || !curve->SubPaths || curve->SubPaths->Count <= 0)
                    continue;

                current = layer->CreateCurve(curve);
                if (!current)
                    continue;

                current->Fill->ApplyUniformFill(
                    app_->CreateRGBColor(0, 0, 0));
                current->Outline->SetNoOutline();

                if (!accumulated)
                {
                    accumulated = current;
                    current = nullptr;
                    continue;
                }

                IVGShapePtr merged;

                try
                {
                    merged = current->Weld(
                        accumulated,
                        VARIANT_FALSE,
                        VARIANT_FALSE);
                }
                catch (...)
                {
                }

                if (!merged)
                {
                    auto mergeRange = app_->CreateShapeRange();
                    if (mergeRange)
                    {
                        mergeRange->Add(accumulated);
                        mergeRange->Add(current);

                        try
                        {
                            merged = mergeRange->Combine();
                        }
                        catch (...)
                        {
                        }
                    }
                }

                if (!merged)
                {
                    Detail::DeleteShape(current);
                    throw std::runtime_error(
                        "Nao foi possivel unir a geometria completa do grupo.");
                }

                accumulated = merged;
                current = nullptr;
            }

            if (!accumulated)
                return nullptr;

            try
            {
                accumulated->Fill->ApplyUniformFill(
                    app_->CreateRGBColor(0, 0, 0));
                accumulated->Outline->SetNoOutline();
                accumulated->Name =
                    _bstr_t("ImCut Group Bleed Geometry Proxy");
            }
            catch (...)
            {
            }

            return accumulated;
        }
        catch (...)
        {
            Detail::DeleteShape(current);
            Detail::DeleteShape(accumulated);
            return nullptr;
        }
    }

    IVGCurvePtr Processor::ExtractExteriorCutlineCurve(
        const IVGCurvePtr& sourceCurve) const
    {
        if (!sourceCurve || !sourceCurve->SubPaths)
            return nullptr;

        struct PathInfo
        {
            IVGSubPathPtr path;
            double left = 0.0;
            double right = 0.0;
            double bottom = 0.0;
            double top = 0.0;
            double boxArea = 0.0;
            double area = 0.0;
            bool valid = false;
        };

        try
        {
            const double tolerance = std::max(
                Detail::Mm(app_, 0.01),
                0.000001);
            const double areaTolerance = tolerance * tolerance;
            auto paths = sourceCurve->SubPaths;
            std::vector<PathInfo> info(
                static_cast<std::size_t>(paths->Count + 1));

            for (long index = 1; index <= paths->Count; ++index)
            {
                auto path = paths->Item[index];
                auto& current = info[static_cast<std::size_t>(index)];
                current.path = path;

                if (!path || !path->Closed)
                {
                    continue;
                }

                auto box = path->BoundingBox;
                if (!box || box->Width <= tolerance ||
                    box->Height <= tolerance)
                {
                    continue;
                }

                current.left = box->Left;
                current.right = box->Left + box->Width;
                current.bottom = box->Bottom;
                current.top = box->Bottom + box->Height;
                current.boxArea = box->Width * box->Height;
                current.area = std::abs(path->Area);
                current.valid = current.area > areaTolerance;
            }

            std::vector<long> remove;
            long exteriorCount = 0;

            for (long candidateIndex = 1;
                candidateIndex <= paths->Count;
                ++candidateIndex)
            {
                const auto& candidate =
                    info[static_cast<std::size_t>(candidateIndex)];

                if (!candidate.valid)
                {
                    remove.push_back(candidateIndex);
                    continue;
                }

                bool enclosed = false;

                for (long containerIndex = 1;
                    containerIndex <= paths->Count && !enclosed;
                    ++containerIndex)
                {
                    if (containerIndex == candidateIndex)
                        continue;

                    const auto& container =
                        info[static_cast<std::size_t>(containerIndex)];

                    if (!container.valid ||
                        container.boxArea <= candidate.boxArea + areaTolerance ||
                        container.left > candidate.left + tolerance ||
                        container.right < candidate.right - tolerance ||
                        container.bottom > candidate.bottom + tolerance ||
                        container.top < candidate.top - tolerance)
                    {
                        continue;
                    }

                    constexpr long SampleCount = 12;
                    bool allSamplesInside = true;

                    for (long sample = 0; sample < SampleCount; ++sample)
                    {
                        double x = 0.0;
                        double y = 0.0;
                        candidate.path->GetPointPositionAt(
                            &x,
                            &y,
                            (static_cast<double>(sample) + 0.5) /
                                static_cast<double>(SampleCount),
                            cdrRelativeSegmentOffset);

                        if (container.path->IsOnSubPath(
                            x,
                            y,
                            tolerance) == cdrOutsideShape)
                        {
                            allSamplesInside = false;
                            break;
                        }
                    }

                    enclosed = allSamplesInside;
                }

                if (enclosed)
                    remove.push_back(candidateIndex);
                else
                    ++exteriorCount;
            }

            if (exteriorCount <= 0)
                return nullptr;

            auto exterior = sourceCurve->GetCopy();
            if (!exterior || !exterior->SubPaths)
                return nullptr;

            for (auto index = remove.rbegin(); index != remove.rend(); ++index)
                exterior->SubPaths->Item[*index]->Delete();

            return exterior->SubPaths->Count > 0
                ? exterior
                : IVGCurvePtr();
        }
        catch (...)
        {
            return nullptr;
        }
    }

    IVGShapePtr Processor::CreateDetachedDuplicate(
        const IVGShapePtr& sourceShape,
        const IVGLayerPtr& targetLayer) const
    {
        if (!sourceShape)
            return nullptr;

        IVGShapePtr duplicate;

        try
        {
            duplicate = sourceShape->Duplicate(0.0, 0.0);
            if (!duplicate)
                return nullptr;

            if (targetLayer)
                duplicate->MoveToLayer(targetLayer);

            return duplicate;
        }
        catch (...)
        {
            Detail::DeleteShape(duplicate);
            return nullptr;
        }
    }

    bool Processor::CopyPowerClipContentsSafely(
        const IVGShapePtr& sourcePowerClip,
        const IVGShapePtr& targetPowerClip,
        const IVGLayerPtr& targetLayer,
        long& contentCount,
        std::string& diagnostic) const
    {
        contentCount = 0;
        diagnostic.clear();

        if (!sourcePowerClip || !Detail::HasPowerClip(sourcePowerClip))
        {
            diagnostic = "fonte PowerClip invalida";
            return false;
        }

        if (!targetPowerClip)
        {
            diagnostic = "container de sangria invalido";
            return false;
        }

        
        
        
        if (Detail::HasPowerClip(targetPowerClip))
        {
            diagnostic = "o container novo da sangria nao esta vazio";
            return false;
        }

        IVGShapePtr duplicate;
        IVGShapeRangePtr detachedContents;
        bool transferred = false;
        long sourceContentCount = 0;

        try
        {
            auto sourcePowerClipObject = sourcePowerClip->PowerClip;
            if (!sourcePowerClipObject || !sourcePowerClipObject->Shapes)
                throw std::runtime_error("conteudo do PowerClip original indisponivel");

            const VARIANT_BOOL sourceContentsLocked =
                sourcePowerClipObject->ContentsLocked;

            sourceContentCount = sourcePowerClipObject->Shapes->Count;
            if (sourceContentCount <= 0)
                return true;

            
            
            duplicate = CreateDetachedDuplicate(sourcePowerClip, targetLayer);
            if (!duplicate || !Detail::HasPowerClip(duplicate))
                throw std::runtime_error("nao foi possivel criar a copia temporaria do PowerClip");

            auto duplicatePowerClip = duplicate->PowerClip;
            if (!duplicatePowerClip || !duplicatePowerClip->Shapes ||
                duplicatePowerClip->Shapes->Count <= 0)
            {
                throw std::runtime_error("a copia temporaria perdeu o conteudo do PowerClip");
            }

            detachedContents = duplicatePowerClip->ExtractShapes();
            if (!detachedContents || detachedContents->Count <= 0)
                throw std::runtime_error("nao foi possivel isolar o conteudo da copia temporaria");

            const long transferredCount = detachedContents->Count;
            detachedContents->AddToPowerClip(targetPowerClip, cdrFalse);
            transferred = true;

            auto targetPowerClipObject = targetPowerClip->PowerClip;
            if (!targetPowerClipObject || !targetPowerClipObject->Shapes)
                throw std::runtime_error("a sangria nao recebeu um PowerClip valido");

            if (targetPowerClipObject->Shapes->Count != transferredCount)
            {
                throw std::runtime_error(
                    "a sangria recebeu uma quantidade incompleta de objetos no PowerClip");
            }

            targetPowerClipObject->ContentsLocked = sourceContentsLocked;
            contentCount = transferredCount;

            const long targetStaticId = targetPowerClip->StaticID;
            for (long i = 1; i <= targetPowerClipObject->Shapes->Count; ++i)
            {
                auto child = targetPowerClipObject->Shapes->Item[i];
                if (!child)
                    throw std::runtime_error("a sangria possui um objeto interno invalido");

                auto parent = child->PowerClipParent;
                if (!parent || parent->StaticID != targetStaticId)
                    throw std::runtime_error("um objeto interno nao ficou vinculado ao PowerClip da sangria");
            }

            auto sourcePowerClipAfter = sourcePowerClip->PowerClip;
            if (!sourcePowerClipAfter ||
                !sourcePowerClipAfter->Shapes ||
                sourcePowerClipAfter->Shapes->Count != sourceContentCount)
            {
                throw std::runtime_error("o PowerClip original mudou durante a copia; operacao cancelada");
            }

            detachedContents = nullptr;
            Detail::DeleteShape(duplicate);
            return true;
        }
        catch (const _com_error& error)
        {
            diagnostic = "copia PowerClip: " + Detail::ComErrorText(error);
        }
        catch (const std::exception& error)
        {
            diagnostic = error.what();
        }
        catch (...)
        {
            diagnostic = "erro desconhecido ao copiar o conteudo do PowerClip";
        }

        if (!transferred && detachedContents)
            Detail::DeleteRange(detachedContents);

        Detail::DeleteShape(duplicate);
        return false;
    }

    bool Processor::NormalizePowerClipContents(
        const IVGShapePtr& container,
        std::string& diagnostic) const
    {
        diagnostic.clear();

        if (!container)
        {
            diagnostic = "container PowerClip invalido";
            return false;
        }

        IVGShapeRangePtr contents;
        bool detached = false;
        bool lockCaptured = false;
        VARIANT_BOOL previousContentsLocked = VARIANT_FALSE;
        bool appStateCaptured = false;
        VARIANT_BOOL previousEventsEnabled = VARIANT_TRUE;
        VARIANT_BOOL previousOptimization = VARIANT_FALSE;

        const auto restoreApplicationState = [&]() noexcept
            {
                if (!appStateCaptured || !app_)
                    return;

                try
                {
                    app_->PutEventsEnabled(previousEventsEnabled);
                }
                catch (...)
                {
                }

                try
                {
                    app_->PutOptimization(previousOptimization);
                }
                catch (...)
                {
                }
            };

        const auto restoreDetachedContents = [&]() -> bool
            {
                if (!detached || !contents)
                    return true;

                try
                {
                    contents->AddToPowerClip(container, cdrFalse);
                    detached = false;
                    return true;
                }
                catch (...)
                {
                }

                try
                {
                    Detail::PumpMessages();
                    contents->AddToPowerClip(container, cdrFalse);
                    detached = false;
                    return true;
                }
                catch (...)
                {
                }

                return false;
            };

        try
        {
            if (!Detail::HasPowerClip(container))
                throw std::runtime_error("a sangria perdeu o PowerClip antes da finalizacao");

            auto powerClip = container->PowerClip;
            if (!powerClip || !powerClip->Shapes)
                throw std::runtime_error("PowerClip da sangria indisponivel");

            if (powerClip->Shapes->Count <= 0)
                return true;


            previousContentsLocked = powerClip->ContentsLocked;
            lockCaptured = true;

            if (app_)
            {
                previousEventsEnabled = app_->GetEventsEnabled();
                previousOptimization = app_->GetOptimization();
                appStateCaptured = true;
                app_->PutOptimization(VARIANT_FALSE);
                app_->PutEventsEnabled(VARIANT_TRUE);
            }

            powerClip->ContentsLocked = VARIANT_FALSE;

            contents = powerClip->Shapes->All();
            if (!contents || contents->Count <= 0)
                throw std::runtime_error("nao foi possivel obter o conteudo da sangria para o rebind");

            const long reboundCount = contents->Count;

            contents->RemoveFromContainer(0);
            detached = true;



            if (!restoreDetachedContents())
                throw std::runtime_error("nao foi possivel recolocar o conteudo no PowerClip apos RemoveFromContainer");



            powerClip = container->PowerClip;
            if (!powerClip || !powerClip->Shapes || powerClip->Shapes->Count <= 0)
                throw std::runtime_error("PowerClip nao foi reconstruido apos o rebind");

            powerClip->ContentsLocked = previousContentsLocked;

            if (powerClip->Shapes->Count != reboundCount)
                throw std::runtime_error("o rebind retornou quantidade diferente de objetos internos");

            const long containerStaticId = container->StaticID;
            for (long i = 1; i <= powerClip->Shapes->Count; ++i)
            {
                auto child = powerClip->Shapes->Item[i];
                if (!child)
                    throw std::runtime_error("objeto interno invalido apos o rebind");

                auto parent = child->PowerClipParent;
                if (!parent || parent->StaticID != containerStaticId)
                    throw std::runtime_error("PowerClipParent inconsistente apos o rebind");
            }

            contents = nullptr;
            restoreApplicationState();

            Detail::TraceBleed(
                "PowerClip normalizado via RemoveFromContainer/AddToPowerClip; filhos=" +
                std::to_string(reboundCount));

            return true;
        }
        catch (const _com_error& error)
        {
            diagnostic = "normalizacao PowerClip: " + Detail::ComErrorText(error);
        }
        catch (const std::exception& error)
        {
            diagnostic = error.what();
        }
        catch (...)
        {
            diagnostic = "erro desconhecido durante a normalizacao do PowerClip";
        }

        if (detached && contents)
        {
            if (!restoreDetachedContents())
            {
                Detail::DeleteRange(contents);
                detached = false;

                if (!diagnostic.empty())
                    diagnostic += " | ";

                diagnostic += "rollback removeu apenas as copias da sangria que nao puderam voltar ao PowerClip";
            }
        }

        if (lockCaptured && Detail::HasPowerClip(container))
        {
            try
            {
                container->PowerClip->ContentsLocked = previousContentsLocked;
            }
            catch (...)
            {
            }
        }

        restoreApplicationState();
        return false;
    }

    void Processor::RollbackPendingPowerClip(
        PendingPowerClipFinalize& pending) noexcept
    {
        Detail::DeleteShape(pending.generatedPowerClip);
        Detail::DeleteShape(pending.generatedCutline);
        pending.generatedPowerClip = nullptr;
        pending.generatedCutline = nullptr;
    }

    bool Processor::SkipHeavyShape(
        const IVGShapePtr& shape,
        long itemNumber,
        long totalCount,
        const Settings& settings,
        const Callbacks& callbacks,
        Detail::CorelExecutionGuard& guard)
    {
        if (!shape)
            return false;
        if (heavyDecision_ == HeavyShapeDecision::ProcessAll)
            return false;
        if (heavyDecision_ == HeavyShapeDecision::SkipAll)
            return true;

        long nodeCount = 0;
        long subpathCount = 0;
        std::vector<IVGShapePtr> complexityShapes;

        if (!settings.flattenGroups && IsGroupShape(shape))
        {
            const auto groupSources = ResolveGroupBleedSources(shape);
            complexityShapes = groupSources.geometrySources;
        }

        if (complexityShapes.empty())
            complexityShapes.push_back(shape);

        try
        {
            for (const auto& complexityShape : complexityShapes)
            {
                if (!complexityShape)
                    continue;

                auto curve = Detail::CurveCopy(complexityShape);
                if (!curve)
                    continue;

                auto subPaths = curve->SubPaths;
                if (!subPaths)
                    continue;

                subpathCount += subPaths->Count;

                for (long i = 1; i <= subPaths->Count; ++i)
                {
                    auto subPath = subPaths->Item[i];
                    if (subPath)
                        nodeCount += subPath->Nodes->Count;
                }
            }
        }
        catch (...)
        {
            return false;
        }

        if (subpathCount < settings.heavySubpathLimit && nodeCount < settings.heavyNodeLimit)
            return false;

        guard.RestoreEventsTemporarily();

        HeavyShapeDecision decision = HeavyShapeDecision::ProcessAll;

        if (callbacks.onHeavyShape)
        {
            decision = callbacks.onHeavyShape({ itemNumber, totalCount, subpathCount, nodeCount });
        }

        guard.DisableEventsAgain();

        if (decision == HeavyShapeDecision::ProcessAll)
        {
            heavyDecision_ = HeavyShapeDecision::ProcessAll;
            return false;
        }

        if (decision == HeavyShapeDecision::SkipAll)
        {
            heavyDecision_ = HeavyShapeDecision::SkipAll;
            return true;
        }

        return false;
    }

    bool Processor::ProcessUniversalBleedShape(
        const IVGShapePtr& sourceShape,
        double bleedDistance,
        long itemNumber,
        UniversalBleedOffset& bleedEngine,
        const Settings& settings,
        IVGShapePtr& resultGroup,
        std::string& errorDescription,
        std::string& strategyNote)
    {
        if (!sourceShape)
            return false;

        IVGShapePtr cutlineShape;
        IVGShapePtr bleedShape;
        IVGShapePtr geometryProxy;
        IVGShapePtr generatedPowerClipShape;
        long powerClipContentCount = 0;

        try
        {
            const bool preserveGroup =
                !settings.flattenGroups &&
                IsGroupShape(sourceShape);

            GroupBleedSources groupSources;
            IVGShapePtr appearanceSource = sourceShape;
            IVGShapePtr workingGeometry = sourceShape;
            IVGLayerPtr outputLayer;

            try
            {
                outputLayer = sourceShape->Layer;
            }
            catch (...)
            {
            }

            if (!outputLayer)
                outputLayer = app_->ActiveLayer;

            if (preserveGroup)
            {
                groupSources = ResolveGroupBleedSources(sourceShape);

                if (groupSources.geometrySources.empty())
                {
                    throw std::runtime_error(
                        "O grupo nao possui geometria fechada adequada para gerar a sangria.");
                }

                appearanceSource = groupSources.appearanceSource
                    ? groupSources.appearanceSource
                    : groupSources.geometrySources.front();

                geometryProxy = CreateStandaloneGeometryProxy(
                    groupSources.geometrySources,
                    outputLayer);

                if (!geometryProxy)
                {
                    throw std::runtime_error(
                        "Nao foi possivel isolar a geometria interna do grupo para gerar a sangria com seguranca.");
                }

                workingGeometry = geometryProxy;
            }

            
            
            double effectiveBleedDistance = bleedDistance;
            const double adaptiveStep = Detail::Mm(app_, AdaptiveRetryStepMm);
            const double adaptiveMaxExtra = Detail::Mm(app_, AdaptiveRetryMaxExtraMm);
            const double adaptiveLimit = bleedDistance + adaptiveMaxExtra;
            std::string lastBleedError;
            long adaptiveAttempts = 0;
            bool quickRetryRecommended = false;

            for (;;)
            {
                ++adaptiveAttempts;
                lastBleedError.clear();

                const bool lastAdaptiveDistance =
                    adaptiveStep <= 0.0 ||
                    effectiveBleedDistance + adaptiveStep > adaptiveLimit + 1e-9;
                bleedEngine.SetQuickRetry(
                    adaptiveAttempts > 1 &&
                    !lastAdaptiveDistance &&
                    quickRetryRecommended);
                bleedEngine.SetAdaptiveDistanceAvailable(!lastAdaptiveDistance);

                try
                {
                    bleedShape = bleedEngine.CreateOffsetShape(
                        workingGeometry,
                        effectiveBleedDistance);
                }
                catch (const _com_error& error)
                {
                    lastBleedError = "Erro COM: " + Detail::ComErrorText(error);
                    bleedShape = nullptr;
                }
                catch (const std::exception& error)
                {
                    lastBleedError = error.what();
                    bleedShape = nullptr;
                }
                catch (...)
                {
                    lastBleedError = "erro desconhecido no gerador de sangria";
                    bleedShape = nullptr;
                }

                quickRetryRecommended = bleedEngine.QuickRetryRecommended();
                if (bleedShape)
                    break;

                if (adaptiveStep <= 0.0 ||
                    adaptiveMaxExtra <= 0.0 ||
                    effectiveBleedDistance + adaptiveStep > adaptiveLimit + 1e-9)
                {
                    if (!lastBleedError.empty())
                        throw std::runtime_error(lastBleedError);

                    throw std::runtime_error(
                        "O gerador universal nao retornou uma sangria.");
                }

                effectiveBleedDistance += adaptiveStep;

                Detail::TraceBleed(
                    "Item #" + std::to_string(itemNumber) +
                    ": nova tentativa com " +
                    Detail::FormatDistance(app_, effectiveBleedDistance));

                Detail::PumpMessages();
            }

            strategyNote = bleedEngine.GetLastDiagnostic();

            if (effectiveBleedDistance > bleedDistance + 1e-9)
            {
                if (!strategyNote.empty())
                    strategyNote += " | ";

                strategyNote +=
                    "sangria adaptativa: " +
                    Detail::FormatDistance(app_, bleedDistance) +
                    " -> " +
                    Detail::FormatDistance(app_, effectiveBleedDistance) +
                    " (" + std::to_string(adaptiveAttempts) + " tentativas)";
            }

            if (settings.createCutline &&
                !(preserveGroup && groupSources.usesExistingCutline))
            {
                const auto cutlineStart = std::chrono::steady_clock::now();
                auto cutlineCurve = bleedEngine.PreparedClosedCurve();
                bool cutlineGeometrySafe = true;

                if (preserveGroup)
                {
                    
                    
                    
                    
                    cutlineCurve =
                        ExtractExteriorCutlineCurve(cutlineCurve);
                    cutlineGeometrySafe = cutlineCurve != nullptr;
                }

                if (cutlineGeometrySafe)
                {
                    cutlineShape = CreateUniversalCutline(
                        workingGeometry,
                        itemNumber,
                        outputLayer ? outputLayer : bleedShape->Layer,
                        settings,
                        cutlineCurve);
                }
                cutlineMs_ += ElapsedMilliseconds(cutlineStart);

                if (!cutlineShape)
                {
                    if (!strategyNote.empty())
                        strategyNote += " | ";

                    strategyNote +=
                        "aviso: a sangria foi criada, mas a borda Spot CutContour nao pode ser criada";
                }
            }

            bleedShape->Name =
                _bstr_t(("Sangria #" + std::to_string(itemNumber)).c_str());

            IVGShapePtr powerClipSource;

            if (Detail::HasPowerClip(sourceShape))
            {
                powerClipSource = sourceShape;
            }
            else if (preserveGroup &&
                appearanceSource &&
                Detail::HasPowerClip(appearanceSource))
            {
                powerClipSource = appearanceSource;
            }

            if (powerClipSource)
            {


                
                
                
                
                
                

                IVGCurvePtr cleanBleedCurve;

                try
                {
                    
                    
                    
                    if (bleedShape->Type == cdrCurveShape && bleedShape->Curve)
                        cleanBleedCurve = bleedShape->Curve->GetCopy();
                }
                catch (...)
                {
                    cleanBleedCurve = nullptr;
                }

                
                if (!cleanBleedCurve)
                    cleanBleedCurve = Detail::CurveCopy(bleedShape);

                
                
                
                
                
                
                
                cdrFillMode generatedBleedFillMode = cdrFillAlternate;
                bool generatedBleedFillModeCaptured = false;

                try
                {
                    generatedBleedFillMode = bleedShape->FillMode;
                    generatedBleedFillModeCaptured = true;
                }
                catch (...)
                {
                    try
                    {
                        generatedBleedFillMode = powerClipSource->FillMode;
                        generatedBleedFillModeCaptured = true;
                    }
                    catch (...)
                    {
                    }
                }

                if (!cleanBleedCurve ||
                    !cleanBleedCurve->SubPaths ||
                    cleanBleedCurve->SubPaths->Count <= 0)
                {
                    throw std::runtime_error(
                        "Nao foi possivel obter a geometria limpa da sangria PowerClip.");
                }

                IVGLayerPtr cleanBleedLayer = outputLayer;
                if (!cleanBleedLayer)
                {
                    try
                    {
                        cleanBleedLayer = bleedShape->Layer;
                    }
                    catch (...)
                    {
                    }
                }

                if (!cleanBleedLayer)
                    cleanBleedLayer = app_->ActiveLayer;

                if (!cleanBleedLayer)
                    throw std::runtime_error(
                        "Nao foi possivel localizar a camada para recriar a sangria PowerClip.");

                IVGShapePtr cleanBleedShape =
                    cleanBleedLayer->CreateCurve(cleanBleedCurve);

                if (!cleanBleedShape)
                    throw std::runtime_error(
                        "O CorelDRAW nao criou o container limpo da sangria PowerClip.");

                
                
                
                
                try
                {
                    auto fillModeRange = app_->CreateShapeRange();
                    if (fillModeRange)
                    {
                        fillModeRange->Add(cleanBleedShape);
                        fillModeRange->SetFillMode(
                            generatedBleedFillModeCaptured
                            ? generatedBleedFillMode
                            : cdrFillAlternate);
                    }
                }
                catch (...)
                {
                    
                    
                    
                    try
                    {
                        auto fillModeRange = app_->CreateShapeRange();
                        if (fillModeRange)
                        {
                            fillModeRange->Add(cleanBleedShape);
                            fillModeRange->SetFillMode(cdrFillAlternate);
                        }
                    }
                    catch (...)
                    {
                    }
                }

                try
                {
                    cleanBleedShape->Name =
                        _bstr_t(("Sangria #" + std::to_string(itemNumber)).c_str());
                }
                catch (...)
                {
                }

                if (Detail::HasPowerClip(cleanBleedShape))
                {
                    Detail::DeleteShape(cleanBleedShape);
                    throw std::runtime_error(
                        "O container limpo herdou PowerClip inesperadamente; operacao cancelada.");
                }

                
                IVGShapePtr inheritedBleedShape = bleedShape;
                bleedShape = cleanBleedShape;
                cleanBleedShape = nullptr;

                Detail::DeleteShape(inheritedBleedShape);



                std::string powerClipDiagnostic;

                if (!CopyPowerClipContentsSafely(
                    powerClipSource,
                    bleedShape,
                    outputLayer,
                    powerClipContentCount,
                    powerClipDiagnostic))
                {
                    throw std::runtime_error(
                        "O conteudo do PowerClip nao pode ser preparado com seguranca: " +
                        powerClipDiagnostic);
                }

                if (powerClipContentCount > 0)
                {
                    generatedPowerClipShape = bleedShape;

                }
            }

            ApplyBestBleedAppearance(appearanceSource, bleedShape);

            Detail::DeleteShape(geometryProxy);

            if (outputLayer)
            {
                try
                {
                    if (bleedShape)
                        bleedShape->MoveToLayer(outputLayer);
                }
                catch (...)
                {
                }

                try
                {
                    if (cutlineShape)
                        cutlineShape->MoveToLayer(outputLayer);
                }
                catch (...)
                {
                }
            }

            bleedShape->OrderToBack();

            if (cutlineShape)
                cutlineShape->OrderToFront();

            if (generatedPowerClipShape)
            {
                PendingPowerClipFinalize pending;
                pending.sourceShape = sourceShape;
                pending.generatedPowerClip = generatedPowerClipShape;
                pending.generatedCutline = cutlineShape;
                pending.preserveGroup = preserveGroup;
                pending.itemNumber = itemNumber;
                pending.strategyNote = strategyNote;
                pendingPowerClipFinalizations_.push_back(std::move(pending));


                bleedShape = nullptr;
                cutlineShape = nullptr;
                resultGroup = nullptr;
                return true;
            }

            if (preserveGroup)
            {
                auto groupRange = app_->CreateShapeRange();
                if (!groupRange)
                    throw std::runtime_error("Nao foi possivel criar o intervalo de agrupamento.");

                groupRange->Add(bleedShape);
                groupRange->Add(sourceShape);

                if (cutlineShape)
                    groupRange->Add(cutlineShape);

                resultGroup = groupRange->Group();

                if (!resultGroup)
                    throw std::runtime_error("Nao foi possivel agrupar a arte original, a sangria e a borda de corte.");

                bleedShape = nullptr;
                cutlineShape = nullptr;
                return true;
            }

            if (cutlineShape)
            {
                auto groupRange = app_->CreateShapeRange();
                if (!groupRange)
                    throw std::runtime_error("Nao foi possivel criar o intervalo da sangria final.");

                groupRange->Add(bleedShape);
                groupRange->Add(cutlineShape);
                resultGroup = groupRange->Group();

                if (!resultGroup)
                    throw std::runtime_error("Nao foi possivel agrupar a sangria e a borda de corte.");

                bleedShape = nullptr;
                cutlineShape = nullptr;
            }
            else
            {
                resultGroup = bleedShape;
                bleedShape = nullptr;
            }

            sourceShape->Delete();
            return true;
        }
        catch (const _com_error& error)
        {
            errorDescription = "Erro COM: " + Detail::ComErrorText(error);
        }
        catch (const std::exception& error)
        {
            errorDescription = std::string("Erro: ") + error.what();
        }
        catch (...)
        {
            errorDescription = "Erro desconhecido ao processar o objeto.";
        }

        Detail::DeleteShape(geometryProxy);
        Detail::DeleteShape(resultGroup);
        Detail::DeleteShape(bleedShape);
        Detail::DeleteShape(cutlineShape);
        strategyNote.clear();
        return false;
    }


    IVGShapePtr Processor::CreateUniversalCutline(
        const IVGShapePtr& sourceShape,
        long itemNumber,
        IVGLayerPtr targetLayer,
        const Settings& settings,
        const IVGCurvePtr& preparedClosedCurve) const
    {
        if (!sourceShape)
            return nullptr;

        IVGShapePtr cutlineShape;

        try
        {
            if (!targetLayer)
                targetLayer = sourceShape->Layer;

            if (!targetLayer)
                targetLayer = app_->ActiveLayer;

            if (!targetLayer)
                return nullptr;

            auto sourceCurve = preparedClosedCurve
                ? preparedClosedCurve
                : Detail::CurveCopy(sourceShape);

            if (sourceCurve && sourceCurve->SubPaths)
            {
                
                
                
                
                IVGCurvePtr cutCurve;
                bool hasOpenSubpath = false;

                for (long i = 1; i <= sourceCurve->SubPaths->Count; ++i)
                {
                    auto subPath = sourceCurve->SubPaths->Item[i];
                    if (subPath && !subPath->Closed)
                    {
                        hasOpenSubpath = true;
                        break;
                    }
                }

                if (!hasOpenSubpath && sourceCurve->SubPaths->Count > 0)
                {
                    try { cutlineShape = targetLayer->CreateCurve(sourceCurve); }
                    catch (...) { cutlineShape = nullptr; }
                }

                try
                {
                    if (!cutlineShape)
                        cutCurve = sourceCurve->GetCopy();
                }
                catch (...)
                {
                    cutCurve = nullptr;
                }

                if (!cutlineShape && cutCurve && cutCurve->SubPaths)
                {
                    auto subPaths = cutCurve->SubPaths;

                    for (long i = subPaths->Count; i >= 1; --i)
                    {
                        auto subPath = subPaths->Item[i];

                        if (subPath && !subPath->Closed)
                        {
                            try
                            {
                                subPath->Delete();
                            }
                            catch (...)
                            {
                            }
                        }
                    }

                    if (cutCurve->SubPaths->Count > 0)
                    {
                        try
                        {
                            cutlineShape = targetLayer->CreateCurve(cutCurve);
                        }
                        catch (...)
                        {
                            cutlineShape = nullptr;
                        }
                    }
                }

                
                
                if (!cutlineShape && sourceCurve->SubPaths->Count > 0)
                {
                    try
                    {
                        cutlineShape = targetLayer->CreateCurve(sourceCurve);
                    }
                    catch (...)
                    {
                        cutlineShape = nullptr;
                    }
                }
            }

            if (!cutlineShape)
            {
                if (!settings.allowRectangleCutlineFallback)
                    return nullptr;

                cutlineShape = targetLayer->CreateRectangle(
                    sourceShape->LeftX,
                    sourceShape->TopY,
                    sourceShape->RightX,
                    sourceShape->BottomY,
                    0, 0, 0, 0);
            }

            if (!cutlineShape)
                return nullptr;

            const double cutWidth = app_->ConvertUnits(
                CutContourWidthMm,
                cdrMillimeter,
                app_->ActiveDocument->Unit);

            cutlineShape->Name =
                _bstr_t(
                    ("Borda de Corte #" +
                        std::to_string(itemNumber)).c_str());

            cutlineShape->Fill->ApplyNoFill();

            auto outline = cutlineShape->Outline;

            outline->Width = cutWidth;
            outline->LineJoin = cdrOutlineRoundLineJoin;
            outline->LineCaps = cdrOutlineRoundLineCaps;

            auto color = outline->Color;
            color->SpotAssign(_bstr_t(CutContourSpotId), 1, 100);
            if (color->Type != cdrColorSpot ||
                std::wstring(static_cast<const wchar_t*>(color->GetName(VARIANT_FALSE))) != L"CutContour")
                throw std::runtime_error("A paleta nao forneceu a cor Spot CutContour.");
            cutlineShape->FillMode = sourceShape->FillMode;

            return cutlineShape;
        }
        catch (...)
        {
            Detail::DeleteShape(cutlineShape);
            return nullptr;
        }
    }


    void Processor::ApplyBestBleedAppearance(
        const IVGShapePtr& sourceShape,
        const IVGShapePtr& bleedShape) const
    {
        if (!bleedShape)
            return;

        try
        {
            auto fillSource = FindFillSource(sourceShape);

            if (!fillSource)
            {
                bleedShape->Fill->ApplyNoFill();
                if (bleedShape->Fill->Type != cdrNoFill)
                {
                    bleedShape->Fill->ApplyUniformFill(app_->CreateRGBColor(255, 255, 255));
                    bleedShape->Fill->ApplyNoFill();
                }
            }
            else
            {
                try
                {
                    bleedShape->Fill->CopyAssign(fillSource->Fill);
                }
                catch (...)
                {
                    bleedShape->Fill->ApplyNoFill();
                }
            }
        }
        catch (...)
        {
        }

        try
        {
            const double outlineWidth = app_->ConvertUnits(
                BleedOutlineWidthMm,
                cdrMillimeter,
                app_->ActiveDocument->Unit);
            auto outline = bleedShape->Outline;
            outline->Color->RGBAssign(0, 0, 0);
            outline->Width = outlineWidth;
            outline->LineJoin = cdrOutlineRoundLineJoin;
            outline->LineCaps = cdrOutlineRoundLineCaps;
            outline->BehindFill = VARIANT_FALSE;
        }
        catch (...)
        {
        }
    }

    IVGShapePtr Processor::FindFillSource(const IVGShapePtr& sourceShape) const
    {
        if (!sourceShape)
            return nullptr;

        try
        {
            if (sourceShape->Type == cdrGroupShape)
            {
                auto children = sourceShape->Shapes;
                for (long i = 1; i <= children->Count; ++i)
                {
                    auto found = FindFillSource(children->Item[i]);
                    if (found)
                        return found;
                }
                return nullptr;
            }

            if (sourceShape->CanHaveFill && sourceShape->Fill->Type != cdrNoFill)
                return sourceShape;
        }
        catch (...)
        {
        }

        return nullptr;
    }

    void Processor::Progress(
        const Callbacks& callbacks,
        long current,
        long total,
        const std::string& message) const
    {
        if (callbacks.onProgress)
            callbacks.onProgress({ current, total, message });
    }

    Result Run(
        IVGApplicationPtr& spApp,
        const Settings& settings,
        const Callbacks& callbacks)
    {
        Processor processor(spApp);
        return processor.Run(settings, callbacks);
    }
}
