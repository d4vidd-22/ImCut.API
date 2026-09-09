#define NOMINMAX 1
#include "Cut.hpp"
#include "RegistrationLayout.hpp"
#include "../OperationCancellation.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cwchar>
#include <cstring>
#include <limits>
#include <unordered_set>


namespace ImCut::Cut
{
    namespace
    {
        constexpr double kEpsilon = 1e-9;

        struct Point2D
        {
            double x = 0.0;
            double y = 0.0;
        };

        struct Bounds
        {
            double left = 0.0;
            double bottom = 0.0;
            double right = 0.0;
            double top = 0.0;

            bool Contains(double x, double y) const noexcept
            {
                return x >= left && x <= right && y >= bottom && y <= top;
            }

            double Width() const noexcept
            {
                return right - left;
            }

            double Height() const noexcept
            {
                return top - bottom;
            }
        };

        struct RegmarkSettings
        {
            double radius = 2.5;
            double hardArtworkMargin = 0.75;
            double artworkMargin = 5.0;
            double preferredSpacing = 60.0;
            double absoluteMinSpacing = 25.0;
            double physicalMinSpacing = 6.0;
            double candidateStep = 0.5;
            double spacingRelaxFactor = 0.88;
            double pageInset = 2.5;
            double corridorProbeDistance = 40.0;
            bool forceBottomRightAnchor = true;
            std::size_t maxCandidates = 2000000;
            std::size_t maxSelectionCandidates = 120000;
        };

        struct RegmarkPlan
        {
            std::vector<Point2D> marks;
            std::size_t validCandidateCount = 0;
            std::size_t evaluatedCandidateCount = 0;
            double smallestSpacing = std::numeric_limits<double>::infinity();
        };

        double DistanceSq(const Point2D& a, const Point2D& b) noexcept
        {
            const double dx = a.x - b.x;
            const double dy = a.y - b.y;
            return dx * dx + dy * dy;
        }

        class CorelExecutionGuard
        {
        public:
            CorelExecutionGuard(IVGApplicationPtr app, IVGDocumentPtr doc,
                bool& rollbackFailed, bool& rollbackPerformed)
                : app_(std::move(app)),
                doc_(std::move(doc)),
                oldOptimization_(app_->GetOptimization()),
                oldEventsEnabled_(app_->GetEventsEnabled()),
                oldUnit_(doc_->Unit), rollbackFailed_(rollbackFailed), rollbackPerformed_(rollbackPerformed)
            {
            }

            CorelExecutionGuard(const CorelExecutionGuard&) = delete;
            CorelExecutionGuard& operator=(const CorelExecutionGuard&) = delete;

            void Begin(const char* commandName)
            {
                app_->PutOptimization(VARIANT_TRUE);
                optimizationChanged_ = true;

                app_->PutEventsEnabled(VARIANT_FALSE);
                eventsChanged_ = true;

                doc_->BeginCommandGroup(commandName);
                commandGroupOpen_ = true;

                doc_->Unit = cdrMillimeter;
                unitChanged_ = true;
                sentinel_ = app_->ActiveLayer->CreateRectangle(0, 0.001, 0.001, 0, 0, 0, 0, 0);
                if (!sentinel_) throw std::runtime_error("Unable to start a recoverable cut command.");
                hasUndoRecord_ = true;
            }

            void Commit()
            {
                if (sentinel_) { sentinel_->Delete(); sentinel_ = nullptr; }
                committed_ = true;
            }

            ~CorelExecutionGuard() noexcept
            {
                if (unitChanged_)
                {
                    try
                    {
                        doc_->Unit = oldUnit_;
                    }
                    catch (...)
                    {
                    }
                }

                if (commandGroupOpen_)
                {
                    try
                    {
                        doc_->EndCommandGroup();
                        if (hasUndoRecord_ && !committed_)
                        {
                            doc_->Undo(1);
                            rollbackPerformed_ = true;
                        }
                    }
                    catch (...)
                    {
                        rollbackFailed_ = true;
                    }
                }

                if (eventsChanged_)
                {
                    try
                    {
                        app_->PutEventsEnabled(oldEventsEnabled_);
                    }
                    catch (...)
                    {
                    }
                }

                if (optimizationChanged_)
                {
                    try
                    {
                        app_->PutOptimization(oldOptimization_);
                    }
                    catch (...)
                    {
                    }
                }

                try
                {
                    if (oldOptimization_ == VARIANT_FALSE) app_->ActiveWindow->Refresh();
                }
                catch (...)
                {
                }
            }

        private:
            IVGApplicationPtr app_;
            IVGDocumentPtr doc_;
            VARIANT_BOOL oldOptimization_ = VARIANT_FALSE;
            VARIANT_BOOL oldEventsEnabled_ = VARIANT_TRUE;
            cdrUnit oldUnit_ = cdrMillimeter;
            bool optimizationChanged_ = false;
            bool eventsChanged_ = false;
            bool commandGroupOpen_ = false;
            bool unitChanged_ = false;
            IVGShapePtr sentinel_;
            bool hasUndoRecord_ = false, committed_ = false;
            bool& rollbackFailed_;
            bool& rollbackPerformed_;
        };

        struct NativeSubPath
        {
            std::vector<Point2D> points;
            Bounds bounds;
            bool closed = false;
        };

        struct NativeSegment
        {
            Point2D a;
            Point2D b;
            Bounds bounds;
        };

        struct NativeObstacle
        {
            Bounds bounds;
            cdrFillMode fillMode = cdrFillAlternate;
            std::vector<std::size_t> closedSubPaths;
        };

        struct CollisionGeometry
        {
            std::vector<NativeSubPath> subPaths;
            std::vector<NativeSegment> segments;
            std::vector<NativeObstacle> obstacles;
            std::vector<Bounds> fallbackBoxes;
            bool fromCutContours = false;
            std::size_t sourceShapeCount = 0;
            std::size_t pointCount = 0;
        };

        struct ClosureGeometrySnapshot
        {
            IVGShapeRangePtr cutShapes;
            CollisionGeometry collision;
        };

        [[nodiscard]] Bounds ExpandBounds(
            const Bounds& bounds,
            double amount) noexcept
        {
            return
            {
                bounds.left - amount,
                bounds.bottom - amount,
                bounds.right + amount,
                bounds.top + amount
            };
        }

        [[nodiscard]] Bounds TranslateBounds(
            const Bounds& bounds,
            double dx,
            double dy) noexcept
        {
            return
            {
                bounds.left + dx,
                bounds.bottom + dy,
                bounds.right + dx,
                bounds.top + dy
            };
        }

        [[nodiscard]] Bounds ComputeBounds(
            const std::vector<Point2D>& points) noexcept
        {
            Bounds result;

            if (points.empty())
                return result;

            result.left = points.front().x;
            result.right = points.front().x;
            result.bottom = points.front().y;
            result.top = points.front().y;

            for (std::size_t i = 1; i < points.size(); ++i)
            {
                result.left = std::min(result.left, points[i].x);
                result.right = std::max(result.right, points[i].x);
                result.bottom = std::min(result.bottom, points[i].y);
                result.top = std::max(result.top, points[i].y);
            }

            return result;
        }

        void IncludeBounds(
            Bounds& destination,
            const Bounds& source,
            bool& initialized) noexcept
        {
            if (!initialized)
            {
                destination = source;
                initialized = true;
                return;
            }

            destination.left = std::min(destination.left, source.left);
            destination.bottom = std::min(destination.bottom, source.bottom);
            destination.right = std::max(destination.right, source.right);
            destination.top = std::max(destination.top, source.top);
        }

        [[nodiscard]] double PointSegmentDistanceSq(
            const Point2D& point,
            const Point2D& a,
            const Point2D& b) noexcept
        {
            const double vx = b.x - a.x;
            const double vy = b.y - a.y;
            const double wx = point.x - a.x;
            const double wy = point.y - a.y;

            const double lengthSq = vx * vx + vy * vy;

            if (lengthSq <= kEpsilon)
                return DistanceSq(point, a);

            const double projection =
                std::clamp(
                    (wx * vx + wy * vy) / lengthSq,
                    0.0,
                    1.0);

            const Point2D closest
            {
                a.x + projection * vx,
                a.y + projection * vy
            };

            return DistanceSq(point, closest);
        }

        [[nodiscard]] double Cross(
            const Point2D& a,
            const Point2D& b,
            const Point2D& point) noexcept
        {
            return
                (b.x - a.x) * (point.y - a.y) -
                (point.x - a.x) * (b.y - a.y);
        }

        [[nodiscard]] bool PointInRingAlternate(
            const Point2D& point,
            const NativeSubPath& ring) noexcept
        {
            if (!ring.closed ||
                ring.points.size() < 3 ||
                !ring.bounds.Contains(point.x, point.y))
            {
                return false;
            }

            bool inside = false;
            std::size_t previous = ring.points.size() - 1;

            for (std::size_t current = 0;
                current < ring.points.size();
                previous = current++)
            {
                const Point2D& a = ring.points[previous];
                const Point2D& b = ring.points[current];

                const bool crosses =
                    (a.y > point.y) !=
                    (b.y > point.y);

                if (!crosses)
                    continue;

                const double denominator = b.y - a.y;

                if (std::abs(denominator) <= kEpsilon)
                    continue;

                const double xIntersection =
                    a.x +
                    (point.y - a.y) *
                    (b.x - a.x) /
                    denominator;

                if (point.x < xIntersection)
                    inside = !inside;
            }

            return inside;
        }

        [[nodiscard]] int RingWindingNumber(
            const Point2D& point,
            const NativeSubPath& ring) noexcept
        {
            if (!ring.closed ||
                ring.points.size() < 3 ||
                !ring.bounds.Contains(point.x, point.y))
            {
                return 0;
            }

            int winding = 0;
            const std::size_t count = ring.points.size();

            for (std::size_t i = 0; i < count; ++i)
            {
                const Point2D& a = ring.points[i];
                const Point2D& b = ring.points[(i + 1) % count];

                if (a.y <= point.y)
                {
                    if (b.y > point.y &&
                        Cross(a, b, point) > 0.0)
                    {
                        ++winding;
                    }
                }
                else if (b.y <= point.y &&
                    Cross(a, b, point) < 0.0)
                {
                    --winding;
                }
            }

            return winding;
        }

        [[nodiscard]] bool PointInsideObstacle(
            const Point2D& point,
            const NativeObstacle& obstacle,
            const CollisionGeometry& geometry) noexcept
        {
            if (obstacle.closedSubPaths.empty())
                return false;

            if (obstacle.fillMode == cdrFillWinding)
            {
                int winding = 0;

                for (const std::size_t subPathIndex :
                obstacle.closedSubPaths)
                {
                    winding +=
                        RingWindingNumber(
                            point,
                            geometry.subPaths[subPathIndex]);
                }

                return winding != 0;
            }

            bool inside = false;

            for (const std::size_t subPathIndex :
            obstacle.closedSubPaths)
            {
                if (PointInRingAlternate(
                    point,
                    geometry.subPaths[subPathIndex]))
                {
                    inside = !inside;
                }
            }

            return inside;
        }

        [[nodiscard]] bool TryGetShapeBounds(
            const IVGShapePtr& shape,
            Bounds& bounds)
        {
            try
            {
                double x = 0.0;
                double y = 0.0;
                double width = 0.0;
                double height = 0.0;

                shape->GetBoundingBox(
                    &x,
                    &y,
                    &width,
                    &height,
                    VARIANT_TRUE);

                bounds.left = x;
                bounds.bottom = y;
                bounds.right = x + width;
                bounds.top = y + height;

                return std::isfinite(x) && std::isfinite(y) && std::isfinite(bounds.right) &&
                    std::isfinite(bounds.top) && width >= 0 && height >= 0;
            }
            catch (...)
            {
                return false;
            }
        }

        void AddSegments(
            CollisionGeometry& geometry,
            const NativeSubPath& subPath)
        {
            if (subPath.points.size() < 2)
                return;

            const std::size_t pointCount =
                subPath.points.size();

            const std::size_t segmentCount =
                subPath.closed
                ? pointCount
                : pointCount - 1;

            geometry.segments.reserve(
                geometry.segments.size() +
                segmentCount);

            for (std::size_t i = 0;
                i < segmentCount;
                ++i)
            {
                const Point2D& a =
                    subPath.points[i];

                const Point2D& b =
                    subPath.points[
                        (i + 1) % pointCount];

                if (DistanceSq(a, b) <= 1e-12)
                    continue;

                NativeSegment segment;
                segment.a = a;
                segment.b = b;
                segment.bounds =
                {
                    std::min(a.x, b.x),
                    std::min(a.y, b.y),
                    std::max(a.x, b.x),
                    std::max(a.y, b.y)
                };

                geometry.segments.push_back(
                    std::move(segment));
            }
        }

        void AppendPointUnique(
            std::vector<Point2D>& points,
            const Point2D& point)
        {
            if (!points.empty() &&
                DistanceSq(
                    points.back(),
                    point) <= 1e-12)
            {
                return;
            }

            points.push_back(point);
        }

        void FlattenCubicRecursive(
            const Point2D& p0,
            const Point2D& p1,
            const Point2D& p2,
            const Point2D& p3,
            double toleranceSq,
            int depth,
            std::vector<Point2D>& output)
        {
            const double d1 =
                PointSegmentDistanceSq(
                    p1,
                    p0,
                    p3);

            const double d2 =
                PointSegmentDistanceSq(
                    p2,
                    p0,
                    p3);

            if (depth >= 12 ||
                std::max(d1, d2) <= toleranceSq)
            {
                AppendPointUnique(
                    output,
                    p3);
                return;
            }

            const Point2D p01
            {
                (p0.x + p1.x) * 0.5,
                (p0.y + p1.y) * 0.5
            };

            const Point2D p12
            {
                (p1.x + p2.x) * 0.5,
                (p1.y + p2.y) * 0.5
            };

            const Point2D p23
            {
                (p2.x + p3.x) * 0.5,
                (p2.y + p3.y) * 0.5
            };

            const Point2D p012
            {
                (p01.x + p12.x) * 0.5,
                (p01.y + p12.y) * 0.5
            };

            const Point2D p123
            {
                (p12.x + p23.x) * 0.5,
                (p12.y + p23.y) * 0.5
            };

            const Point2D midpoint
            {
                (p012.x + p123.x) * 0.5,
                (p012.y + p123.y) * 0.5
            };

            FlattenCubicRecursive(
                p0,
                p01,
                p012,
                midpoint,
                toleranceSq,
                depth + 1,
                output);

            FlattenCubicRecursive(
                midpoint,
                p123,
                p23,
                p3,
                toleranceSq,
                depth + 1,
                output);
        }

        void FinalizeExtractedSubPath(
            NativeSubPath& path,
            std::vector<NativeSubPath>& output)
        {
            if (path.points.size() >= 2 &&
                DistanceSq(
                    path.points.front(),
                    path.points.back()) <= 1e-12)
            {
                path.closed = true;
                path.points.pop_back();
            }

            if (path.points.size() < 2)
            {
                path = NativeSubPath{};
                return;
            }

            path.bounds =
                ComputeBounds(
                    path.points);

            output.push_back(
                std::move(path));

            path = NativeSubPath{};
        }

        [[nodiscard]] bool TryGetCurveElementsBulk(
            const IVGCurvePtr& curve,
            std::vector<CurveElement>& elements)
        {
            if (!curve)
                return false;

            IDispatch* dispatch = nullptr;

            if (FAILED(
                curve->QueryInterface(
                    IID_IDispatch,
                    reinterpret_cast<void**>(
                        &dispatch))) ||
                !dispatch)
            {
                return false;
            }

            OLECHAR methodName[] =
                L"GetCurveInfo";

            LPOLESTR name = methodName;
            DISPID dispid = DISPID_UNKNOWN;

            HRESULT hr =
                dispatch->GetIDsOfNames(
                    IID_NULL,
                    &name,
                    1,
                    LOCALE_USER_DEFAULT,
                    &dispid);

            if (FAILED(hr))
            {
                dispatch->Release();
                return false;
            }

            DISPPARAMS params{};
            VARIANT result;
            VariantInit(&result);

            EXCEPINFO exception{};
            UINT argumentError = 0;

            hr =
                dispatch->Invoke(
                    dispid,
                    IID_NULL,
                    LOCALE_USER_DEFAULT,
                    DISPATCH_METHOD,
                    &params,
                    &result,
                    &exception,
                    &argumentError);

            dispatch->Release();

            if (FAILED(hr))
            {
                VariantClear(&result);
                return false;
            }

            SAFEARRAY* array = nullptr;

            if ((result.vt & VT_ARRAY) != 0)
            {
                if ((result.vt & VT_BYREF) != 0)
                {
                    if (result.pparray)
                        array = *result.pparray;
                }
                else
                {
                    array = result.parray;
                }
            }

            if (!array ||
                SafeArrayGetDim(array) != 1 ||
                array->cbElements != sizeof(CurveElement))
            {
                VariantClear(&result);
                return false;
            }

            LONG lower = 0;
            LONG upper = -1;

            if (FAILED(
                SafeArrayGetLBound(
                    array,
                    1,
                    &lower)) ||
                FAILED(
                    SafeArrayGetUBound(
                        array,
                        1,
                        &upper)) ||
                upper < lower)
            {
                VariantClear(&result);
                return false;
            }

            void* rawData = nullptr;

            if (FAILED(
                SafeArrayAccessData(
                    array,
                    &rawData)) ||
                !rawData)
            {
                VariantClear(&result);
                return false;
            }

            const std::size_t count =
                static_cast<std::size_t>(
                    upper - lower + 1);

            elements.resize(count);

            std::memcpy(
                elements.data(),
                rawData,
                count * sizeof(CurveElement));

            SafeArrayUnaccessData(array);
            VariantClear(&result);

            return !elements.empty();
        }

        [[nodiscard]] bool ExtractBulkCurveGeometry(
            const IVGCurvePtr& curve,
            double translateX,
            double translateY,
            std::vector<NativeSubPath>& output)
        {
            std::vector<CurveElement> elements;

            if (!TryGetCurveElementsBulk(
                curve,
                elements))
            {
                return false;
            }

            constexpr double flattenTolerance = 0.25;
            constexpr double flattenToleranceSq =
                flattenTolerance * flattenTolerance;

            NativeSubPath current;
            std::vector<Point2D> controls;
            bool hasCurrent = false;

            const auto closeFlag =
                static_cast<unsigned char>(
                    cdrFlagClosed);

            for (const CurveElement& element :
                elements)
            {
                const Point2D point
                {
                    element.PositionX + translateX,
                    element.PositionY + translateY
                };

                const bool elementClosed =
                    (element.Flags & closeFlag) != 0;

                if (element.ElementType == cdrElementStart)
                {
                    if (hasCurrent)
                    {
                        FinalizeExtractedSubPath(
                            current,
                            output);
                    }

                    current = NativeSubPath{};
                    current.closed = elementClosed;
                    current.points.push_back(point);
                    controls.clear();
                    hasCurrent = true;
                    continue;
                }

                if (!hasCurrent)
                    continue;

                current.closed =
                    current.closed ||
                    elementClosed;

                if (element.ElementType == cdrElementControl)
                {
                    controls.push_back(point);

                    if (controls.size() > 2)
                    {
                        controls.erase(
                            controls.begin(),
                            controls.end() - 2);
                    }

                    continue;
                }

                if (element.ElementType == cdrElementCurve)
                {
                    if (!current.points.empty() &&
                        controls.size() >= 2)
                    {
                        FlattenCubicRecursive(
                            current.points.back(),
                            controls[controls.size() - 2],
                            controls[controls.size() - 1],
                            point,
                            flattenToleranceSq,
                            0,
                            current.points);
                    }
                    else
                    {
                        AppendPointUnique(
                            current.points,
                            point);
                    }

                    controls.clear();
                    continue;
                }

                if (element.ElementType == cdrElementLine)
                {
                    controls.clear();
                    AppendPointUnique(
                        current.points,
                        point);
                }
            }

            if (hasCurrent)
            {
                FinalizeExtractedSubPath(
                    current,
                    output);
            }

            return !output.empty();
        }

        void CommitExtractedSubPaths(
            std::vector<NativeSubPath>& extracted,
            cdrFillMode fillMode,
            CollisionGeometry& geometry)
        {
            NativeObstacle obstacle;
            obstacle.fillMode = fillMode;
            bool obstacleBoundsInitialized = false;

            for (NativeSubPath& nativePath :
                extracted)
            {
                if (nativePath.points.size() < 2)
                    continue;

                if (nativePath.bounds.Width() <= 0.0 &&
                    nativePath.bounds.Height() <= 0.0)
                {
                    nativePath.bounds =
                        ComputeBounds(
                            nativePath.points);
                }

                const std::size_t globalSubPathIndex =
                    geometry.subPaths.size();

                if (nativePath.closed &&
                    nativePath.points.size() >= 3)
                {
                    obstacle.closedSubPaths.push_back(
                        globalSubPathIndex);

                    IncludeBounds(
                        obstacle.bounds,
                        nativePath.bounds,
                        obstacleBoundsInitialized);
                }

                geometry.pointCount +=
                    nativePath.points.size();

                AddSegments(
                    geometry,
                    nativePath);

                geometry.subPaths.push_back(
                    std::move(nativePath));
            }

            if (obstacleBoundsInitialized &&
                !obstacle.closedSubPaths.empty())
            {
                geometry.obstacles.push_back(
                    std::move(obstacle));
            }
        }

        [[nodiscard]] bool ExtractPolylineGeometry(
            const IVGShapePtr& shape,
            double translateX,
            double translateY,
            CollisionGeometry& geometry)
        {
            try
            {
                auto displayCurve =
                    shape->DisplayCurve;

                if (!displayCurve)
                    return false;

                cdrFillMode fillMode =
                    cdrFillAlternate;

                try
                {
                    fillMode = shape->FillMode;
                }
                catch (...)
                {
                }

                std::vector<NativeSubPath> extracted;

                if (ExtractBulkCurveGeometry(
                    displayCurve,
                    translateX,
                    translateY,
                    extracted))
                {
                    CommitExtractedSubPaths(
                        extracted,
                        fillMode,
                        geometry);

                    return true;
                }

                auto polyline =
                    displayCurve->GetPolyline(0);

                if (!polyline)
                    return false;

                auto subPaths =
                    polyline->SubPaths;

                if (!subPaths)
                    return false;

                const long subPathCount =
                    subPaths->Count;

                if (subPathCount <= 0)
                    return false;

                extracted.reserve(
                    static_cast<std::size_t>(
                        subPathCount));

                for (long i = 1;
                    i <= subPathCount;
                    ++i)
                {
                    auto subPath =
                        subPaths->Item[i];

                    if (!subPath)
                        continue;

                    auto nodes =
                        subPath->Nodes;

                    if (!nodes)
                        continue;

                    const long nodeCount =
                        nodes->Count;

                    if (nodeCount < 2)
                        continue;

                    NativeSubPath nativePath;
                    nativePath.closed =
                        subPath->Closed != VARIANT_FALSE;

                    nativePath.points.reserve(
                        static_cast<std::size_t>(
                            nodeCount));

                    for (long n = 1;
                        n <= nodeCount;
                        ++n)
                    {
                        auto node =
                            nodes->Item[n];

                        if (!node)
                            continue;

                        double px = 0.0;
                        double py = 0.0;

                        node->GetPosition(
                            &px,
                            &py);

                        AppendPointUnique(
                            nativePath.points,
                            {
                                px + translateX,
                                py + translateY
                            });
                    }

                    FinalizeExtractedSubPath(
                        nativePath,
                        extracted);
                }

                if (extracted.empty())
                    return false;

                CommitExtractedSubPaths(
                    extracted,
                    fillMode,
                    geometry);

                return true;
            }
            catch (...)
            {
                return false;
            }
        }

        void AddFallbackBounds(
            CollisionGeometry& geometry,
            const IVGShapePtr& shape,
            double translateX,
            double translateY)
        {
            Bounds bounds;

            if (!TryGetShapeBounds(
                shape,
                bounds))
            {
                throw std::runtime_error("Unable to validate artwork bounds for registration marks.");
            }

            geometry.fallbackBoxes.push_back(
                TranslateBounds(
                    bounds,
                    translateX,
                    translateY));
        }

        [[nodiscard]] bool IsCutContourShape(const IVGShapePtr& shape)
        {
            if (!shape)
                return false;

            try
            {
                auto outline = shape->Outline;
                auto color = outline ? outline->Color : IVGColorPtr();
                if (!color || color->IsSpot == VARIANT_FALSE)
                    return false;

                const auto matches = [](const _bstr_t& value)
                {
                    const wchar_t* text = static_cast<const wchar_t*>(value);
                    return text && _wcsicmp(text, L"CutContour") == 0;
                };

                if (matches(color->SpotColorName))
                    return true;

                return matches(color->GetName(VARIANT_FALSE));
            }
            catch (...)
            {
                return false;
            }
        }

        void CollectCutContourShapes(const IVGShapePtr& shape,
            const IVGShapeRangePtr& result, int depth = 0)
        {
            if (!shape || !result || depth >= 64)
                return;

            if (IsCutContourShape(shape))
            {
                result->Add(shape);
                return;
            }

            if (shape->Type != cdrGroupShape)
                return;

            try
            {
                auto children = shape->Shapes;
                const long count = children ? children->Count : 0;
                for (long index = 1; index <= count; ++index)
                    CollectCutContourShapes(children->Item[index], result, depth + 1);
            }
            catch (...)
            {
            }
        }

        [[nodiscard]] IVGShapeRangePtr FindCutContourShapes(
            const IVGShapeRangePtr& range)
        {
            if (!range || range->Count <= 0)
                return nullptr;

            auto app = range->Application;
            auto result = app ? app->CreateShapeRange() : IVGShapeRangePtr();
            if (!result)
                return nullptr;

            const long count = range->Count;
            for (long index = 1; index <= count; ++index)
            {
                try { CollectCutContourShapes(range->Item[index], result); }
                catch (...) {}
            }

            return result->Count > 0 ? result : IVGShapeRangePtr();
        }

        void AddArtworkGeometry(CollisionGeometry& geometry, const IVGShapePtr& shape,
            double translateX, double translateY, const std::unordered_set<long>& cutIds, int depth = 0)
        {
            if (!shape) throw std::runtime_error("Unable to inspect closure artwork.");
            if (cutIds.contains(shape->StaticID)) return;
            if (depth < 64 && shape->Type == cdrGroupShape)
            {
                auto effects = shape->Effects;
                if (!effects || effects->Count == 0)
                {
                    auto children = shape->Shapes;
                    const long count = children ? children->Count : 0;
                    if (count > 0)
                    {
                        for (long i = 1; i <= count; ++i)
                            AddArtworkGeometry(geometry, children->Item[i], translateX, translateY, cutIds, depth + 1);
                        return;
                    }
                }
            }

            if (shape->Type == cdrCurveShape &&
                ExtractPolylineGeometry(shape, translateX, translateY, geometry))
            {
                return;
            }

            AddFallbackBounds(geometry, shape, translateX, translateY);
        }

        [[nodiscard]] ClosureGeometrySnapshot CaptureClosureGeometry(
            const IVGShapeRangePtr& range,
            double translateX,
            double translateY)
        {
            ClosureGeometrySnapshot snapshot;
            std::unordered_set<long> cutIds;

            if (!range || range->Count <= 0)
                return snapshot;

            snapshot.cutShapes = FindCutContourShapes(range);

            if (snapshot.cutShapes &&
                snapshot.cutShapes->Count > 0)
            {
                snapshot.collision.fromCutContours = true;
                snapshot.collision.sourceShapeCount =
                    static_cast<std::size_t>(
                        snapshot.cutShapes->Count);

                const long count =
                    snapshot.cutShapes->Count;

                for (long i = 1;
                    i <= count;
                    ++i)
                {
                    IVGShapePtr shape;

                    try
                    {
                        shape =
                            snapshot.cutShapes->Item[i];
                    }
                    catch (...)
                    {
                        continue;
                    }

                    if (!shape)
                        throw std::runtime_error("Unable to inspect CutContour geometry.");

                    cutIds.insert(shape->StaticID);

                    if (!ExtractPolylineGeometry(
                        shape,
                        translateX,
                        translateY,
                        snapshot.collision))
                    {
                        AddFallbackBounds(
                            snapshot.collision,
                            shape,
                            translateX,
                            translateY);
                    }
                }

                const long selectedCount = range->Count;
                for (long i = 1; i <= selectedCount; ++i)
                {
                    auto selected = range->Item[i];
                    if (!selected) throw std::runtime_error("Unable to inspect selected artwork bounds.");
                    AddArtworkGeometry(snapshot.collision, selected, translateX, translateY, cutIds);
                }
                return snapshot;
            }

            snapshot.collision.fromCutContours = false;
            snapshot.collision.sourceShapeCount =
                static_cast<std::size_t>(
                    range->Count);

            const long count = range->Count;

            for (long i = 1;
                i <= count;
                ++i)
            {
                IVGShapePtr shape;

                try
                {
                    shape = range->Item[i];
                }
                catch (...)
                {
                    continue;
                }

                if (!shape)
                    continue;

                AddArtworkGeometry(
                    snapshot.collision,
                    shape,
                    translateX,
                    translateY, cutIds);
            }

            return snapshot;
        }

        [[nodiscard]] bool PointBlockedByGeometry(
            const CollisionGeometry& geometry,
            const Point2D& point,
            double hotArea) noexcept
        {
            const double hotAreaSq =
                hotArea * hotArea;

            for (const Bounds& bounds :
                geometry.fallbackBoxes)
            {
                if (ExpandBounds(
                    bounds,
                    hotArea).Contains(
                        point.x,
                        point.y))
                {
                    return true;
                }
            }

            for (const NativeSegment& segment :
                geometry.segments)
            {
                const Bounds expanded =
                    ExpandBounds(
                        segment.bounds,
                        hotArea);

                if (!expanded.Contains(
                    point.x,
                    point.y))
                {
                    continue;
                }

                if (PointSegmentDistanceSq(
                    point,
                    segment.a,
                    segment.b) <= hotAreaSq + kEpsilon)
                {
                    return true;
                }
            }

            for (const NativeObstacle& obstacle :
                geometry.obstacles)
            {
                if (!obstacle.bounds.Contains(
                    point.x,
                    point.y))
                {
                    continue;
                }

                if (PointInsideObstacle(
                    point,
                    obstacle,
                    geometry))
                {
                    return true;
                }
            }

            return false;
        }

        class BlockedMaskBuilder final
        {
        public:
            BlockedMaskBuilder(
                const std::vector<double>& x,
                const std::vector<double>& y,
                const CollisionGeometry& geometry,
                double hotArea)
                : x_(x),
                y_(y),
                geometry_(geometry),
                hotArea_(std::max(0.0, hotArea)),
                hotAreaSq_(hotArea_* hotArea_),
                nx_(x_.size()),
                ny_(y_.size()),
                blocked_(nx_* ny_, 0)
            {
            }

            [[nodiscard]] std::vector<std::uint8_t> Build()
            {
                RasterizeFallbackBoxes();
                RasterizeFilledObstacles();
                RasterizeBoundaryMargins();
                return std::move(blocked_);
            }

        private:
            [[nodiscard]] bool CoordinateRange(
                const std::vector<double>& values,
                double minimum,
                double maximum,
                std::size_t& first,
                std::size_t& last) const noexcept
            {
                if (values.empty() ||
                    maximum < values.front() - kEpsilon ||
                    minimum > values.back() + kEpsilon)
                {
                    return false;
                }

                const auto begin =
                    std::lower_bound(
                        values.begin(),
                        values.end(),
                        minimum - kEpsilon);

                const auto end =
                    std::upper_bound(
                        values.begin(),
                        values.end(),
                        maximum + kEpsilon);

                if (begin == values.end() ||
                    begin >= end)
                {
                    return false;
                }

                first =
                    static_cast<std::size_t>(
                        begin - values.begin());

                last =
                    static_cast<std::size_t>(
                        end - values.begin() - 1);

                return true;
            }

            void BlockRect(
                const Bounds& bounds) noexcept
            {
                std::size_t firstX = 0;
                std::size_t lastX = 0;
                std::size_t firstY = 0;
                std::size_t lastY = 0;

                if (!CoordinateRange(
                    x_,
                    bounds.left,
                    bounds.right,
                    firstX,
                    lastX) ||
                    !CoordinateRange(
                        y_,
                        bounds.bottom,
                        bounds.top,
                        firstY,
                        lastY))
                {
                    return;
                }

                for (std::size_t iy = firstY;
                    iy <= lastY;
                    ++iy)
                {
                    const std::size_t row =
                        iy * nx_;

                    std::fill(
                        blocked_.begin() +
                        static_cast<std::ptrdiff_t>(
                            row + firstX),
                        blocked_.begin() +
                        static_cast<std::ptrdiff_t>(
                            row + lastX + 1),
                        static_cast<std::uint8_t>(1));
                }
            }

            void BlockXInterval(
                std::size_t iy,
                double firstX,
                double lastX) noexcept
            {
                if (firstX > lastX)
                    std::swap(firstX, lastX);

                std::size_t beginIndex = 0;
                std::size_t endIndex = 0;

                if (!CoordinateRange(
                    x_,
                    firstX,
                    lastX,
                    beginIndex,
                    endIndex))
                {
                    return;
                }

                const std::size_t row =
                    iy * nx_;

                std::fill(
                    blocked_.begin() +
                    static_cast<std::ptrdiff_t>(
                        row + beginIndex),
                    blocked_.begin() +
                    static_cast<std::ptrdiff_t>(
                        row + endIndex + 1),
                    static_cast<std::uint8_t>(1));
            }

            void RasterizeFallbackBoxes() noexcept
            {
                for (const Bounds& bounds :
                    geometry_.fallbackBoxes)
                {
                    BlockRect(
                        ExpandBounds(
                            bounds,
                            hotArea_));
                }
            }

            void RasterizeFilledObstacles()
            {
                struct Crossing
                {
                    double x = 0.0;
                    int windingDelta = 0;
                };

                std::vector<Crossing> crossings;

                for (const NativeObstacle& obstacle :
                    geometry_.obstacles)
                {
                    if (obstacle.closedSubPaths.empty())
                        continue;

                    std::size_t firstY = 0;
                    std::size_t lastY = 0;

                    if (!CoordinateRange(
                        y_,
                        obstacle.bounds.bottom,
                        obstacle.bounds.top,
                        firstY,
                        lastY))
                    {
                        continue;
                    }

                    for (std::size_t iy = firstY;
                        iy <= lastY;
                        ++iy)
                    {
                        const double scanY = y_[iy];
                        crossings.clear();

                        for (const std::size_t subPathIndex :
                        obstacle.closedSubPaths)
                        {
                            const NativeSubPath& ring =
                                geometry_.subPaths[subPathIndex];

                            if (!ring.closed ||
                                ring.points.size() < 3 ||
                                scanY < ring.bounds.bottom - kEpsilon ||
                                scanY > ring.bounds.top + kEpsilon)
                            {
                                continue;
                            }

                            const std::size_t count =
                                ring.points.size();

                            for (std::size_t i = 0;
                                i < count;
                                ++i)
                            {
                                const Point2D& a =
                                    ring.points[i];

                                const Point2D& b =
                                    ring.points[
                                        (i + 1) % count];

                                const bool upward =
                                    a.y <= scanY &&
                                    b.y > scanY;

                                const bool downward =
                                    b.y <= scanY &&
                                    a.y > scanY;

                                if (!upward && !downward)
                                    continue;

                                const double denominator =
                                    b.y - a.y;

                                if (std::abs(denominator) <= kEpsilon)
                                    continue;

                                Crossing crossing;
                                crossing.x =
                                    a.x +
                                    (scanY - a.y) *
                                    (b.x - a.x) /
                                    denominator;

                                crossing.windingDelta =
                                    upward ? 1 : -1;

                                crossings.push_back(
                                    crossing);
                            }
                        }

                        if (crossings.size() < 2)
                            continue;

                        std::sort(
                            crossings.begin(),
                            crossings.end(),
                            [](const Crossing& a,
                                const Crossing& b)
                            {
                                if (a.x != b.x)
                                    return a.x < b.x;

                                return
                                    a.windingDelta <
                                    b.windingDelta;
                            });

                        if (obstacle.fillMode != cdrFillWinding)
                        {
                            for (std::size_t i = 0;
                                i + 1 < crossings.size();
                                i += 2)
                            {
                                BlockXInterval(
                                    iy,
                                    crossings[i].x,
                                    crossings[i + 1].x);
                            }

                            continue;
                        }

                        int winding = 0;
                        double previousX =
                            crossings.front().x;
                        std::size_t i = 0;

                        while (i < crossings.size())
                        {
                            const double currentX =
                                crossings[i].x;

                            if (winding != 0 &&
                                currentX > previousX + kEpsilon)
                            {
                                BlockXInterval(
                                    iy,
                                    previousX,
                                    currentX);
                            }

                            int delta = 0;

                            while (i < crossings.size() &&
                                std::abs(
                                    crossings[i].x -
                                    currentX) <= kEpsilon)
                            {
                                delta +=
                                    crossings[i].windingDelta;
                                ++i;
                            }

                            winding += delta;
                            previousX = currentX;
                        }
                    }
                }
            }

            void RasterizeBoundaryMargins() noexcept
            {
                if (geometry_.segments.empty())
                    return;

                for (const NativeSegment& segment :
                    geometry_.segments)
                {
                    const Bounds expanded =
                        ExpandBounds(
                            segment.bounds,
                            hotArea_);

                    std::size_t firstX = 0;
                    std::size_t lastX = 0;
                    std::size_t firstY = 0;
                    std::size_t lastY = 0;

                    if (!CoordinateRange(
                        x_,
                        expanded.left,
                        expanded.right,
                        firstX,
                        lastX) ||
                        !CoordinateRange(
                            y_,
                            expanded.bottom,
                            expanded.top,
                            firstY,
                            lastY))
                    {
                        continue;
                    }

                    for (std::size_t iy = firstY;
                        iy <= lastY;
                        ++iy)
                    {
                        const std::size_t row =
                            iy * nx_;

                        for (std::size_t ix = firstX;
                            ix <= lastX;
                            ++ix)
                        {
                            const std::size_t flatIndex =
                                row + ix;

                            if (blocked_[flatIndex] != 0)
                                continue;

                            const Point2D point
                            {
                                x_[ix],
                                y_[iy]
                            };

                            if (PointSegmentDistanceSq(
                                point,
                                segment.a,
                                segment.b) <= hotAreaSq_ + kEpsilon)
                            {
                                blocked_[flatIndex] = 1;
                            }
                        }
                    }
                }
            }

            const std::vector<double>& x_;
            const std::vector<double>& y_;
            const CollisionGeometry& geometry_;
            double hotArea_ = 0.0;
            double hotAreaSq_ = 0.0;
            std::size_t nx_ = 0;
            std::size_t ny_ = 0;
            std::vector<std::uint8_t> blocked_;
        };

        class CandidateGrid final
        {
        public:
            CandidateGrid(
                const Bounds& page,
                const CollisionGeometry& collision,
                double hardArea,
                double preferredArea,
                const RegmarkSettings& settings)
            {
                const double inset =
                    std::max(
                        settings.radius,
                        settings.pageInset);

                minX_ = page.left + inset;
                maxX_ = page.right - inset;
                minY_ = page.bottom + inset;
                maxY_ = page.top - inset;

                if (maxX_ < minX_ || maxY_ < minY_)
                    return;

                const double availableWidth = maxX_ - minX_;
                const double availableHeight = maxY_ - minY_;
                double step = std::max(0.5, settings.candidateStep);

                const auto computeCount = [](double size, double gridStep)
                    {
                        if (size <= kEpsilon)
                            return std::size_t{ 1 };

                        return
                            static_cast<std::size_t>(
                                std::floor(size / gridStep)) +
                            1;
                    };

                nx_ = computeCount(availableWidth, step);
                ny_ = computeCount(availableHeight, step);

                const double estimated =
                    static_cast<double>(nx_) *
                    static_cast<double>(ny_);

                if (settings.maxCandidates > 0 &&
                    estimated > static_cast<double>(settings.maxCandidates))
                {
                    const double scale =
                        std::sqrt(
                            estimated /
                            static_cast<double>(settings.maxCandidates));

                    step *= scale;
                    nx_ = computeCount(availableWidth, step);
                    ny_ = computeCount(availableHeight, step);
                }

                if (nx_ == 0 || ny_ == 0)
                    return;

                if (settings.maxCandidates > 0)
                {
                    nx_ = std::min(nx_, settings.maxCandidates);
                    ny_ = std::min(
                        ny_,
                        std::max(
                            std::size_t{ 1 },
                            settings.maxCandidates / nx_));
                }

                x_.resize(nx_);
                y_.resize(ny_);

                for (std::size_t ix = 0; ix < nx_; ++ix)
                {
                    const double tx =
                        nx_ <= 1
                        ? 0.5
                        : static_cast<double>(ix) /
                        static_cast<double>(nx_ - 1);

                    x_[ix] = minX_ + availableWidth * tx;
                }

                for (std::size_t iy = 0; iy < ny_; ++iy)
                {
                    const double ty =
                        ny_ <= 1
                        ? 0.5
                        : static_cast<double>(iy) /
                        static_cast<double>(ny_ - 1);

                    y_[iy] = minY_ + availableHeight * ty;
                }

                const std::size_t total = nx_ * ny_;

                BlockedMaskBuilder maskBuilder(
                    x_,
                    y_,
                    collision,
                    hardArea);

                blocked_ = maskBuilder.Build();

                BlockedMaskBuilder preferredMaskBuilder(
                    x_,
                    y_,
                    collision,
                    std::max(hardArea, preferredArea));

                preferredBlocked_ =
                    preferredMaskBuilder.Build();

                BuildCorridorScores(
                    settings.corridorProbeDistance);

                BuildSelectionCandidates(
                    settings.maxSelectionCandidates);

                selected_.assign(total, 0);
                nearestMarkDistanceSq_.assign(
                    total,
                    std::numeric_limits<double>::infinity());

                blockedCandidateCount_ =
                    static_cast<std::size_t>(
                        std::count(
                            blocked_.begin(),
                            blocked_.end(),
                            static_cast<std::uint8_t>(1)));

                validCandidateCount_ =
                    total - blockedCandidateCount_;
            }

            [[nodiscard]] bool Empty() const noexcept
            {
                return nx_ == 0 || ny_ == 0;
            }

            [[nodiscard]] std::size_t Width() const noexcept
            {
                return nx_;
            }

            [[nodiscard]] std::size_t Height() const noexcept
            {
                return ny_;
            }

            [[nodiscard]] std::size_t Total() const noexcept
            {
                return nx_ * ny_;
            }

            [[nodiscard]] std::size_t FlatIndex(
                std::size_t ix,
                std::size_t iy) const noexcept
            {
                return iy * nx_ + ix;
            }

            [[nodiscard]] Point2D Position(
                std::size_t ix,
                std::size_t iy) const noexcept
            {
                return { x_[ix], y_[iy] };
            }

            [[nodiscard]] Point2D Position(
                std::size_t flatIndex) const noexcept
            {
                return Position(
                    flatIndex % nx_,
                    flatIndex / nx_);
            }

            [[nodiscard]] const std::vector<double>& XCoordinates() const noexcept
            {
                return x_;
            }

            [[nodiscard]] const std::vector<double>& YCoordinates() const noexcept
            {
                return y_;
            }

            [[nodiscard]] bool IsSelected(
                std::size_t flatIndex) const noexcept
            {
                return selected_[flatIndex] != 0;
            }

            void Select(std::size_t flatIndex) noexcept
            {
                selected_[flatIndex] = 1;
            }

            [[nodiscard]] bool HasMarks() const noexcept
            {
                return markCount_ > 0;
            }

            [[nodiscard]] double SpacingSq(
                std::size_t flatIndex) const noexcept
            {
                return nearestMarkDistanceSq_[flatIndex];
            }

            [[nodiscard]] double CorridorScore(
                std::size_t flatIndex) const noexcept
            {
                return
                    static_cast<double>(
                        corridorScore_[flatIndex]) /
                    2.0;
            }

            [[nodiscard]] bool HasPreferredClearance(
                std::size_t flatIndex) const noexcept
            {
                return preferredBlocked_[flatIndex] == 0;
            }

            [[nodiscard]] const std::vector<std::size_t>& Candidates() const noexcept
            {
                return selectionCandidates_;
            }

            void AddMark(
                const Point2D& mark) noexcept
            {
                ++markCount_;

                for (const std::size_t flatIndex :
                    selectionCandidates_)
                {
                    const std::size_t iy = flatIndex / nx_;
                    const std::size_t ix = flatIndex % nx_;
                    const double dy =
                        y_[iy] - mark.y;
                    const double dySq =
                        dy * dy;
                    const double dx =
                        x_[ix] - mark.x;

                    const double distanceSq =
                        dx * dx + dySq;

                    double& cached =
                        nearestMarkDistanceSq_[
                            flatIndex];

                    if (distanceSq < cached)
                        cached = distanceSq;
                }
            }

            [[nodiscard]] bool IsValid(
                std::size_t flatIndex) const noexcept
            {
                return blocked_[flatIndex] == 0;
            }

            [[nodiscard]] std::size_t ValidCandidateCount() const noexcept
            {
                return validCandidateCount_;
            }

            [[nodiscard]] std::size_t EvaluatedCandidateCount() const noexcept
            {
                return selectionCandidates_.size();
            }

            [[nodiscard]] std::size_t BlockedCandidateCount() const noexcept
            {
                return blockedCandidateCount_;
            }

        private:
            void BuildCorridorScores(
                double probeDistance)
            {
                const std::size_t total = Total();
                corridorScore_.assign(total, 0);

                if (total == 0 ||
                    probeDistance <= kEpsilon)
                {
                    return;
                }

                std::vector<float> firstDistance(
                    total,
                    std::numeric_limits<float>::infinity());

                const double gridStepX =
                    nx_ > 1 ? x_[1] - x_[0] : 1.0;

                const double gridStepY =
                    ny_ > 1 ? y_[1] - y_[0] : 1.0;

                const double balanceTolerance =
                    std::max(
                        1.0,
                        std::max(gridStepX, gridStepY) * 1.5);

                for (std::size_t iy = 0; iy < ny_; ++iy)
                {
                    const std::size_t row = iy * nx_;
                    std::size_t lastBlocked = nx_;

                    for (std::size_t ix = 0; ix < nx_; ++ix)
                    {
                        const std::size_t index = row + ix;

                        if (blocked_[index] != 0)
                        {
                            lastBlocked = ix;
                        }
                        else if (lastBlocked != nx_)
                        {
                            const double distance =
                                x_[ix] - x_[lastBlocked];

                            if (distance <= probeDistance)
                                firstDistance[index] =
                                    static_cast<float>(distance);
                        }
                    }

                    std::size_t nextBlocked = nx_;

                    for (std::size_t ix = nx_; ix-- > 0;)
                    {
                        const std::size_t index = row + ix;

                        if (blocked_[index] != 0)
                        {
                            nextBlocked = ix;
                            continue;
                        }

                        if (nextBlocked == nx_ ||
                            !std::isfinite(firstDistance[index]))
                        {
                            continue;
                        }

                        const double secondDistance =
                            x_[nextBlocked] - x_[ix];

                        if (secondDistance > probeDistance)
                            continue;

                        const double first =
                            firstDistance[index];

                        if (std::abs(first - secondDistance) <=
                            std::max(
                                balanceTolerance,
                                (first + secondDistance) * 0.30))
                        {
                            ++corridorScore_[index];
                        }
                    }
                }

                std::fill(
                    firstDistance.begin(),
                    firstDistance.end(),
                    std::numeric_limits<float>::infinity());

                for (std::size_t ix = 0; ix < nx_; ++ix)
                {
                    std::size_t lastBlocked = ny_;

                    for (std::size_t iy = 0; iy < ny_; ++iy)
                    {
                        const std::size_t index = iy * nx_ + ix;

                        if (blocked_[index] != 0)
                        {
                            lastBlocked = iy;
                        }
                        else if (lastBlocked != ny_)
                        {
                            const double distance =
                                y_[iy] - y_[lastBlocked];

                            if (distance <= probeDistance)
                                firstDistance[index] =
                                    static_cast<float>(distance);
                        }
                    }

                    std::size_t nextBlocked = ny_;

                    for (std::size_t iy = ny_; iy-- > 0;)
                    {
                        const std::size_t index = iy * nx_ + ix;

                        if (blocked_[index] != 0)
                        {
                            nextBlocked = iy;
                            continue;
                        }

                        if (nextBlocked == ny_ ||
                            !std::isfinite(firstDistance[index]))
                        {
                            continue;
                        }

                        const double secondDistance =
                            y_[nextBlocked] - y_[iy];

                        if (secondDistance > probeDistance)
                            continue;

                        const double first =
                            firstDistance[index];

                        if (std::abs(first - secondDistance) <=
                            std::max(
                                balanceTolerance,
                                (first + secondDistance) * 0.30))
                        {
                            ++corridorScore_[index];
                        }
                    }
                }
            }

            void BuildSelectionCandidates(
                std::size_t preferredBudget)
            {
                selectionCandidates_.clear();

                if (Total() == 0)
                    return;

                std::size_t stride = 1;

                if (preferredBudget > 0 &&
                    Total() > preferredBudget)
                {
                    stride =
                        std::max(
                            std::size_t{ 1 },
                            static_cast<std::size_t>(
                                std::ceil(
                                    std::sqrt(
                                        static_cast<double>(Total()) /
                                        static_cast<double>(preferredBudget)))));
                }

                selectionCandidates_.reserve(
                    std::min(Total(), preferredBudget * 2));

                for (std::size_t iy = 0; iy < ny_; ++iy)
                {
                    for (std::size_t ix = 0; ix < nx_; ++ix)
                    {
                        const std::size_t index =
                            FlatIndex(ix, iy);

                        if (blocked_[index] != 0)
                            continue;

                        const bool coarseSample =
                            stride <= 1 ||
                            (ix % stride == 0 &&
                                iy % stride == 0) ||
                            ix == 0 || iy == 0 ||
                            ix + 1 == nx_ || iy + 1 == ny_;

                        const bool tightSpace =
                            preferredBlocked_[index] != 0;

                        if (coarseSample ||
                            tightSpace ||
                            corridorScore_[index] != 0)
                        {
                            selectionCandidates_.push_back(index);
                        }
                    }
                }
            }

            double minX_ = 0.0;
            double maxX_ = 0.0;
            double minY_ = 0.0;
            double maxY_ = 0.0;
            std::size_t nx_ = 0;
            std::size_t ny_ = 0;
            std::vector<double> x_;
            std::vector<double> y_;
            std::vector<std::uint8_t> blocked_;
            std::vector<std::uint8_t> preferredBlocked_;
            std::vector<std::uint8_t> corridorScore_;
            std::vector<std::uint8_t> selected_;
            std::vector<double> nearestMarkDistanceSq_;
            std::vector<std::size_t> selectionCandidates_;
            std::size_t markCount_ = 0;
            std::size_t validCandidateCount_ = 0;
            std::size_t blockedCandidateCount_ = 0;
        };

        [[nodiscard]] std::vector<std::size_t> BuildCoordinateOrder(
            const std::vector<double>& coordinates,
            double target)
        {
            std::vector<std::size_t> order;
            order.reserve(coordinates.size());

            if (coordinates.empty())
                return order;

            const auto firstRight =
                std::lower_bound(
                    coordinates.begin(),
                    coordinates.end(),
                    target);

            std::ptrdiff_t left =
                static_cast<std::ptrdiff_t>(
                    firstRight - coordinates.begin()) -
                1;

            std::size_t right =
                static_cast<std::size_t>(
                    firstRight - coordinates.begin());

            while (left >= 0 || right < coordinates.size())
            {
                if (left < 0)
                {
                    order.push_back(right++);
                    continue;
                }

                if (right >= coordinates.size())
                {
                    order.push_back(
                        static_cast<std::size_t>(left--));
                    continue;
                }

                const std::size_t leftIndex =
                    static_cast<std::size_t>(left);

                const double leftDistance =
                    std::abs(
                        coordinates[leftIndex] -
                        target);

                const double rightDistance =
                    std::abs(
                        coordinates[right] -
                        target);

                if (leftDistance <= rightDistance)
                {
                    order.push_back(leftIndex);
                    --left;
                }
                else
                {
                    order.push_back(right);
                    ++right;
                }
            }

            return order;
        }

        struct CandidateSearchOrder
        {
            std::vector<std::size_t> x;
            std::vector<std::size_t> y;
        };

        [[nodiscard]] CandidateSearchOrder BuildCandidateSearchOrder(
            const CandidateGrid& grid,
            const Point2D& target)
        {
            CandidateSearchOrder result;

            result.x =
                BuildCoordinateOrder(
                    grid.XCoordinates(),
                    target.x);

            result.y =
                BuildCoordinateOrder(
                    grid.YCoordinates(),
                    target.y);

            return result;
        }

        [[nodiscard]] std::size_t FindNearestCandidateForTarget(
            CandidateGrid& grid,
            const Point2D& target,
            const CandidateSearchOrder& searchOrder,
            double requiredSpacingSq)
        {
            if (grid.Empty())
                return std::numeric_limits<std::size_t>::max();

            const bool hasMarks = grid.HasMarks();
            std::size_t bestIndex = std::numeric_limits<std::size_t>::max();
            double bestTargetDistanceSq = std::numeric_limits<double>::infinity();
            double bestSpacingSq = -1.0;

            for (const std::size_t iy : searchOrder.y)
            {
                const double dy =
                    grid.YCoordinates()[iy] -
                    target.y;

                const double dySq =
                    dy * dy;

                if (std::isfinite(bestTargetDistanceSq) &&
                    dySq > bestTargetDistanceSq + kEpsilon)
                {
                    break;
                }

                for (const std::size_t ix : searchOrder.x)
                {
                    const double dx =
                        grid.XCoordinates()[ix] -
                        target.x;

                    const double targetDistanceSq =
                        dx * dx + dySq;

                    if (std::isfinite(bestTargetDistanceSq) &&
                        targetDistanceSq >
                        bestTargetDistanceSq + kEpsilon)
                    {
                        break;
                    }

                    const std::size_t flatIndex =
                        grid.FlatIndex(ix, iy);

                    if (grid.IsSelected(flatIndex))
                        continue;

                    const double spacingSq =
                        grid.SpacingSq(flatIndex);

                    if (hasMarks &&
                        spacingSq + kEpsilon <
                        requiredSpacingSq)
                    {
                        continue;
                    }

                    if (!grid.IsValid(flatIndex))
                        continue;

                    if (targetDistanceSq + kEpsilon <
                        bestTargetDistanceSq ||
                        (std::abs(
                            targetDistanceSq -
                            bestTargetDistanceSq) <= kEpsilon &&
                            (spacingSq >
                                bestSpacingSq + kEpsilon ||
                                (std::abs(
                                    spacingSq -
                                    bestSpacingSq) <= kEpsilon &&
                                    flatIndex < bestIndex))))
                    {
                        bestIndex = flatIndex;
                        bestTargetDistanceSq =
                            targetDistanceSq;
                        bestSpacingSq =
                            spacingSq;
                    }
                }
            }

            return bestIndex;
        }

        [[nodiscard]] std::size_t FindNearestCandidateWithRelaxation(
            CandidateGrid& grid,
            const Point2D& target,
            double preferredSpacing,
            double minimumSpacing,
            double relaxFactor)
        {
            const auto order = BuildCandidateSearchOrder(grid, target);
            double requiredSpacing = preferredSpacing;

            for (;;)
            {
                const auto candidate = FindNearestCandidateForTarget(
                    grid, target, order, requiredSpacing * requiredSpacing);

                if (candidate != std::numeric_limits<std::size_t>::max())
                    return candidate;

                if (requiredSpacing <= minimumSpacing + kEpsilon)
                    return std::numeric_limits<std::size_t>::max();

                const double nextSpacing = std::max(
                    minimumSpacing,
                    requiredSpacing * relaxFactor);

                if (std::abs(nextSpacing - requiredSpacing) < kEpsilon)
                    return std::numeric_limits<std::size_t>::max();

                requiredSpacing = nextSpacing;
            }
        }

        struct SpatialRegions
        {
            Bounds page;
            std::size_t columns = 1;
            std::size_t rows = 1;
            double cellWidth = 1.0;
            double cellHeight = 1.0;

            SpatialRegions(
                const Bounds& bounds,
                std::size_t requestedCount)
                : page(bounds)
            {
                const double width =
                    std::max(kEpsilon, page.Width());

                const double height =
                    std::max(kEpsilon, page.Height());

                const double aspect = width / height;

                columns =
                    std::max(
                        std::size_t{ 1 },
                        static_cast<std::size_t>(
                            std::llround(
                                std::sqrt(
                                    static_cast<double>(
                                        std::max(
                                            std::size_t{ 1 },
                                            requestedCount)) *
                                    aspect))));

                columns =
                    std::min(
                        columns,
                        std::max(
                            std::size_t{ 1 },
                            requestedCount));

                rows =
                    std::max(
                        std::size_t{ 1 },
                        (requestedCount + columns - 1) /
                        columns);

                cellWidth = width /
                    static_cast<double>(columns);

                cellHeight = height /
                    static_cast<double>(rows);
            }

            [[nodiscard]] std::size_t Count() const noexcept
            {
                return columns * rows;
            }

            [[nodiscard]] double Diagonal() const noexcept
            {
                return std::hypot(
                    page.Width(),
                    page.Height());
            }

            [[nodiscard]] std::size_t Index(
                const Point2D& point) const noexcept
            {
                const std::size_t column =
                    std::min(
                        columns - 1,
                        static_cast<std::size_t>(
                            std::max(
                                0.0,
                                std::floor(
                                    (point.x - page.left) /
                                    cellWidth))));

                const std::size_t row =
                    std::min(
                        rows - 1,
                        static_cast<std::size_t>(
                            std::max(
                                0.0,
                                std::floor(
                                    (point.y - page.bottom) /
                                    cellHeight))));

                return row * columns + column;
            }

            [[nodiscard]] Point2D Center(
                std::size_t index) const noexcept
            {
                const std::size_t column =
                    index % columns;

                const std::size_t row =
                    index / columns;

                return
                {
                    page.left +
                        (static_cast<double>(column) + 0.5) *
                        cellWidth,
                    page.bottom +
                        (static_cast<double>(row) + 0.5) *
                        cellHeight
                };
            }

            [[nodiscard]] double CenteringScore(
                std::size_t index,
                const Point2D& point) const noexcept
            {
                const Point2D center = Center(index);
                const double halfDiagonal =
                    std::max(
                        kEpsilon,
                        0.5 * std::hypot(
                            cellWidth,
                            cellHeight));

                return
                    1.0 -
                    std::clamp(
                        std::sqrt(
                            DistanceSq(
                                point,
                                center)) /
                        halfDiagonal,
                        0.0,
                        1.0);
            }

            [[nodiscard]] double InteriorScore(
                const Point2D& point) const noexcept
            {
                const double edgeDistance =
                    std::min(
                        {
                            point.x - page.left,
                            page.right - point.x,
                            point.y - page.bottom,
                            page.top - point.y
                        });

                const double scale =
                    std::max(
                        kEpsilon,
                        std::min(
                            page.Width(),
                            page.Height()) *
                        0.25);

                return
                    std::clamp(
                        edgeDistance / scale,
                        0.0,
                        1.0);
            }
        };

        [[nodiscard]] std::size_t FindBestTightCorridorCandidate(
            CandidateGrid& grid,
            const SpatialRegions& regions,
            double physicalSpacing,
            double preferredSpacing)
        {
            const double requiredSpacingSq =
                physicalSpacing * physicalSpacing;
            std::size_t bestIndex =
                std::numeric_limits<std::size_t>::max();
            double bestScore =
                -std::numeric_limits<double>::infinity();
            std::size_t inspected = 0;

            for (const std::size_t flatIndex : grid.Candidates())
            {
                Cancellation::Checkpoint(inspected++);

                if (grid.IsSelected(flatIndex) ||
                    !grid.IsValid(flatIndex) ||
                    grid.HasPreferredClearance(flatIndex) ||
                    grid.CorridorScore(flatIndex) <= 0.0)
                {
                    continue;
                }

                const double spacingSq = grid.SpacingSq(flatIndex);
                if (grid.HasMarks() &&
                    spacingSq + kEpsilon < requiredSpacingSq)
                {
                    continue;
                }

                const Point2D point = grid.Position(flatIndex);
                const double spacing = grid.HasMarks()
                    ? std::clamp(
                        std::sqrt(spacingSq) /
                            std::max(preferredSpacing, regions.Diagonal() * 0.55),
                        0.0,
                        1.0)
                    : 0.5;
                const double score =
                    grid.CorridorScore(flatIndex) * 0.55 +
                    regions.InteriorScore(point) * 0.25 +
                    spacing * 0.20;

                if (score > bestScore + kEpsilon ||
                    (std::abs(score - bestScore) <= kEpsilon &&
                        flatIndex < bestIndex))
                {
                    bestIndex = flatIndex;
                    bestScore = score;
                }
            }

            return bestIndex;
        }

        [[nodiscard]] std::size_t FindBestRegionalCandidate(
            CandidateGrid& grid,
            const SpatialRegions& regions,
            const std::vector<std::uint8_t>& servedRegions,
            double physicalSpacing,
            double preferredSpacing,
            bool prioritizeGeometry)
        {
            const bool hasMarks = grid.HasMarks();
            const bool scoreSpacing = hasMarks && !prioritizeGeometry;
            const double physicalSpacingSq =
                physicalSpacing * physicalSpacing;

            std::size_t bestIndex =
                std::numeric_limits<std::size_t>::max();

            double bestScore =
                -std::numeric_limits<double>::infinity();

            double bestSpacingSq = -1.0;
            std::size_t inspected = 0;

            for (const std::size_t flatIndex :
                grid.Candidates())
            {
                Cancellation::Checkpoint(inspected++);

                if (grid.IsSelected(flatIndex) ||
                    !grid.IsValid(flatIndex))
                {
                    continue;
                }

                const Point2D point =
                    grid.Position(flatIndex);

                const std::size_t region =
                    regions.Index(point);

                if (servedRegions[region] != 0)
                    continue;

                const double spacingSq =
                    grid.SpacingSq(flatIndex);

                if (hasMarks &&
                    spacingSq + kEpsilon < physicalSpacingSq)
                {
                    continue;
                }

                const double spacingScore =
                    scoreSpacing
                    ? std::clamp(
                        std::sqrt(spacingSq) /
                        std::max(
                            preferredSpacing,
                            regions.Diagonal() * 0.55),
                        0.0,
                        1.0)
                    : 0.5;

                const double corridor =
                    grid.CorridorScore(flatIndex);

                const double preferredClearance =
                    grid.HasPreferredClearance(flatIndex)
                    ? 1.0
                    : 0.0;

                const double centering =
                    regions.CenteringScore(
                        region,
                        point);

                const double interior =
                    regions.InteriorScore(point);

                const double score =
                    scoreSpacing
                    ? spacingScore * 0.52 +
                        corridor * 0.20 +
                        centering * 0.12 +
                        preferredClearance * 0.08 +
                        interior * 0.08
                    : corridor * 0.30 +
                        centering * 0.18 +
                        preferredClearance * 0.12 +
                        interior * 0.40;

                if (score > bestScore + kEpsilon ||
                    (std::abs(score - bestScore) <= kEpsilon &&
                        (spacingSq > bestSpacingSq + kEpsilon ||
                            (std::abs(spacingSq - bestSpacingSq) <= kEpsilon &&
                                flatIndex < bestIndex))))
                {
                    bestIndex = flatIndex;
                    bestScore = score;
                    bestSpacingSq = spacingSq;
                }
            }

            return bestIndex;
        }

        [[nodiscard]] std::size_t FindBestWeightedCandidate(
            CandidateGrid& grid,
            const SpatialRegions& regions,
            const std::vector<std::size_t>& regionCounts,
            double requiredSpacing,
            double preferredSpacing)
        {
            const bool hasMarks = grid.HasMarks();
            const double requiredSpacingSq =
                requiredSpacing * requiredSpacing;

            std::size_t bestIndex =
                std::numeric_limits<std::size_t>::max();

            double bestScore =
                -std::numeric_limits<double>::infinity();

            double bestSpacingSq = -1.0;
            std::size_t inspected = 0;

            for (const std::size_t flatIndex :
                grid.Candidates())
            {
                Cancellation::Checkpoint(inspected++);

                if (grid.IsSelected(flatIndex) ||
                    !grid.IsValid(flatIndex))
                {
                    continue;
                }

                const double distanceSq =
                    grid.SpacingSq(flatIndex);

                if (hasMarks &&
                    distanceSq + kEpsilon <
                    requiredSpacingSq)
                {
                    continue;
                }

                const Point2D point =
                    grid.Position(flatIndex);

                const std::size_t region =
                    regions.Index(point);

                const double spacingScore =
                    hasMarks
                    ? std::clamp(
                        std::sqrt(distanceSq) /
                        std::max(
                            preferredSpacing,
                            regions.Diagonal() * 0.55),
                        0.0,
                        1.0)
                    : 0.5;

                const double regionNeed =
                    1.0 /
                    (1.0 +
                        static_cast<double>(
                            regionCounts[region]));

                const double score =
                    spacingScore * 0.62 +
                    grid.CorridorScore(flatIndex) * 0.18 +
                    regionNeed * 0.10 +
                    (grid.HasPreferredClearance(flatIndex) ? 0.06 : 0.0) +
                    regions.InteriorScore(point) * 0.04;

                if (score > bestScore + kEpsilon ||
                    (std::abs(score - bestScore) <= kEpsilon &&
                        (distanceSq > bestSpacingSq + kEpsilon ||
                            (std::abs(distanceSq - bestSpacingSq) <= kEpsilon &&
                                flatIndex < bestIndex))))
                {
                    bestIndex = flatIndex;
                    bestScore = score;
                    bestSpacingSq = distanceSq;
                }
            }

            return bestIndex;
        }

        void CommitCandidate(
            CandidateGrid& grid,
            std::size_t selectedIndex,
            RegmarkPlan& result)
        {
            const Point2D newMark =
                grid.Position(selectedIndex);

            const double selectedSpacingSq =
                grid.SpacingSq(selectedIndex);

            grid.Select(selectedIndex);
            result.marks.push_back(newMark);
            grid.AddMark(newMark);

            if (std::isfinite(selectedSpacingSq))
            {
                result.smallestSpacing =
                    std::min(
                        result.smallestSpacing,
                        std::sqrt(selectedSpacingSq));
            }
        }

        RegmarkPlan GenerateRegmarkPlan(
            const Bounds& page,
            const CollisionGeometry& collision,
            int requestedCount,
            const RegmarkSettings& settings)
        {
            RegmarkPlan result;

            if (requestedCount <= 0)
                return result;

            result.marks.reserve(static_cast<std::size_t>(requestedCount));

            Cancellation::ThrowIfRequested();

            const Point2D bottomRightAnchor
            {
                page.right - settings.radius,
                page.bottom + settings.radius
            };

            const double hardArea =
                settings.radius +
                std::max(
                    0.0,
                    settings.hardArtworkMargin);

            const bool anchorFitsPage =
                bottomRightAnchor.x - settings.radius >= page.left - kEpsilon &&
                bottomRightAnchor.x + settings.radius <= page.right + kEpsilon &&
                bottomRightAnchor.y - settings.radius >= page.bottom - kEpsilon &&
                bottomRightAnchor.y + settings.radius <= page.top + kEpsilon;

            const bool useBottomRightAnchor =
                settings.forceBottomRightAnchor &&
                anchorFitsPage &&
                !PointBlockedByGeometry(
                    collision,
                    bottomRightAnchor,
                    hardArea);

            CandidateGrid grid(
                page,
                collision,
                hardArea,
                settings.radius +
                    std::max(
                        settings.hardArtworkMargin,
                        settings.artworkMargin),
                settings);

            Cancellation::ThrowIfRequested();

            if (useBottomRightAnchor)
                result.marks.push_back(bottomRightAnchor);

            if (grid.Empty())
                return result;

            if (useBottomRightAnchor)
                grid.AddMark(bottomRightAnchor);

            const double physicalMinSpacing =
                std::max(
                    settings.radius * 2.0,
                    settings.physicalMinSpacing);

            const double absoluteMinSpacing =
                std::max(
                    physicalMinSpacing,
                    settings.absoluteMinSpacing);

            const double preferredSpacing =
                std::max(
                    absoluteMinSpacing,
                    settings.preferredSpacing);

            const double relaxFactor = std::clamp(
                settings.spacingRelaxFactor,
                0.5,
                0.99);

            const std::size_t requested = static_cast<std::size_t>(requestedCount);

            const SpatialRegions regions(
                page,
                requested);

            std::vector<std::uint8_t> servedRegions(
                regions.Count(),
                0);

            std::vector<std::size_t> regionCounts(
                regions.Count(),
                0);

            if (result.marks.size() < requested)
            {
                const std::size_t corridorIndex =
                    FindBestTightCorridorCandidate(
                        grid,
                        regions,
                        physicalMinSpacing,
                        preferredSpacing);

                if (corridorIndex !=
                    std::numeric_limits<std::size_t>::max())
                {
                    const std::size_t region =
                        regions.Index(grid.Position(corridorIndex));
                    CommitCandidate(grid, corridorIndex, result);
                    servedRegions[region] = 1;
                    ++regionCounts[region];
                }
            }

            while (result.marks.size() < requested)
            {
                Cancellation::ThrowIfRequested();

                const std::size_t selectedIndex =
                    FindBestRegionalCandidate(
                        grid,
                        regions,
                        servedRegions,
                        physicalMinSpacing,
                        preferredSpacing,
                        result.marks.size() ==
                            (useBottomRightAnchor ? 1u : 0u));

                if (selectedIndex ==
                    std::numeric_limits<std::size_t>::max())
                {
                    break;
                }

                const std::size_t region =
                    regions.Index(
                        grid.Position(selectedIndex));

                CommitCandidate(
                    grid,
                    selectedIndex,
                    result);

                servedRegions[region] = 1;
                ++regionCounts[region];
            }

            while (result.marks.size() < requested)
            {
                Cancellation::ThrowIfRequested();

                double requiredSpacing = preferredSpacing;
                std::size_t selectedIndex = std::numeric_limits<std::size_t>::max();

                for (;;)
                {
                    selectedIndex = FindBestWeightedCandidate(
                        grid,
                        regions,
                        regionCounts,
                        requiredSpacing,
                        preferredSpacing);

                    if (selectedIndex != std::numeric_limits<std::size_t>::max() ||
                        requiredSpacing <= absoluteMinSpacing + kEpsilon)
                        break;

                    const double nextSpacing = std::max(
                        absoluteMinSpacing,
                        requiredSpacing * relaxFactor);

                    if (std::abs(nextSpacing - requiredSpacing) < kEpsilon)
                        break;

                    requiredSpacing = nextSpacing;
                }

                if (selectedIndex == std::numeric_limits<std::size_t>::max())
                    break;

                const std::size_t region =
                    regions.Index(
                        grid.Position(selectedIndex));

                CommitCandidate(
                    grid,
                    selectedIndex,
                    result);

                ++regionCounts[region];
            }

            result.validCandidateCount = grid.ValidCandidateCount();
            result.evaluatedCandidateCount = grid.EvaluatedCandidateCount();

            return result;
        }

        std::string ComErrorToString(const _com_error& error)
        {
            try
            {
                _bstr_t description = error.Description();

                if (description.length() > 0)
                    return static_cast<const char*>(description);
            }
            catch (...)
            {
            }

            return "Unknown CorelDRAW COM error.";
        }
    }


    namespace
    {
        struct ClosureProcessStats
        {
            int regmarksCreated = 0;
            int warnings = 0;
            double geometryMs = 0.0;
            double planningMs = 0.0;
            double documentMs = 0.0;
        };

        [[nodiscard]] IVGShapeRangePtr MakeSingleShapeRange(
            const IVGApplicationPtr& app,
            const IVGShapePtr& shape)
        {
            auto range = app->CreateShapeRange();

            if (shape)
                range->Add(shape);

            return range;
        }

        [[nodiscard]] std::vector<IVGShapeRangePtr> BuildClosureRanges(
            const IVGApplicationPtr& app,
            const IVGShapeRangePtr& selection,
            ClosureMode requestedMode)
        {
            std::vector<IVGShapeRangePtr> result;

            if (!app || !selection || selection->Count <= 0)
                return result;

            ClosureMode mode = requestedMode;

            if (mode == ClosureMode::Auto)
            {
                bool allGroups = selection->Count > 1;

                for (long i = 1; i <= selection->Count && allGroups; ++i)
                {
                    auto shape = selection->Item[i];

                    if (!shape || shape->Type != cdrGroupShape)
                        allGroups = false;
                }

                mode = allGroups
                    ? ClosureMode::EachSelectedGroup
                    : ClosureMode::WholeSelection;
            }

            if (mode == ClosureMode::WholeSelection)
            {
                result.push_back(selection);
                return result;
            }

            result.reserve(
                static_cast<std::size_t>(
                    selection->Count));

            for (long i = 1; i <= selection->Count; ++i)
            {
                auto shape = selection->Item[i];

                if (!shape)
                    continue;

                if (mode == ClosureMode::EachSelectedGroup &&
                    shape->Type != cdrGroupShape)
                {
                    throw std::runtime_error(
                        "Each selected group mode requires all selected top-level objects to be groups.");
                }

                auto range =
                    MakeSingleShapeRange(
                        app,
                        shape);

                if (range && range->Count > 0)
                    result.push_back(range);
            }

            return result;
        }

        [[nodiscard]] ClosureProcessStats ProcessClosureRange(
            const IVGApplicationPtr& app,
            const IVGDocumentPtr& doc,
            const IVGShapeRangePtr& range,
            const Settings& publicSettings,
            int closureIndex,
            const IVGPagePtr& existingPrintPage,
            const IVGPagePtr& previousCutPage,
            IVGPagePtr& printPageOut,
            IVGPagePtr& cutPageOut)
        {
            Cancellation::ThrowIfRequested();

            if (!app || !doc || !range || range->Count <= 0)
                throw std::runtime_error("Invalid closure.");

            const double pageWidth =
                static_cast<double>(
                    publicSettings.pageWidth);

            const double pageHeight =
                range->SizeHeight;

            constexpr double minimumPageDimensionMm = 0.001;
            if (!std::isfinite(pageWidth) || !std::isfinite(pageHeight) ||
                pageWidth < minimumPageDimensionMm || pageHeight < minimumPageDimensionMm)
            {
                throw std::runtime_error(
                    "Invalid closure page dimensions.");
            }

            IVGPagePtr printPage =
                existingPrintPage;

            if (!printPage)
            {
                if (!previousCutPage)
                {
                    throw std::runtime_error(
                        "Unable to resolve the page insertion point.");
                }

                printPage =
                    doc->InsertPagesEx(
                        1,
                        VARIANT_FALSE,
                        previousCutPage->Index,
                        pageWidth,
                        pageHeight);

                if (!printPage)
                {
                    throw std::runtime_error(
                        "Unable to create PRINT page.");
                }
            }

            const auto documentStart =
                std::chrono::steady_clock::now();

            printPage->Activate();
            printPage->SetSize(
                pageWidth,
                pageHeight);

            const double moveX =
                printPage->CenterX -
                range->CenterX;

            const double moveY =
                printPage->CenterY -
                range->CenterY;

            const auto geometryStart =
                std::chrono::steady_clock::now();

            ClosureGeometrySnapshot geometrySnapshot =
                CaptureClosureGeometry(
                    range,
                    moveX,
                    moveY);

            const auto geometryEnd =
                std::chrono::steady_clock::now();

            bool alreadyOnPrintPage = false;

            try
            {
                auto firstShape =
                    range->Item[1];

                alreadyOnPrintPage =
                    firstShape &&
                    firstShape->Page &&
                    firstShape->Page->Index ==
                    printPage->Index;
            }
            catch (...)
            {
                alreadyOnPrintPage = false;
            }

            if (!alreadyOnPrintPage)
            {
                range->MoveToLayer(
                    printPage->ActiveLayer);
            }

            range->Move(
                moveX,
                moveY);

            const Bounds pageBounds
            {
                printPage->LeftX,
                printPage->BottomY,
                printPage->RightX,
                printPage->TopY
            };

            RegmarkSettings markSettings;
            markSettings.radius = 2.5;
            markSettings.hardArtworkMargin = 0.75;
            markSettings.artworkMargin = 5.0;
            markSettings.preferredSpacing = 60.0;
            markSettings.absoluteMinSpacing = 25.0;
            markSettings.physicalMinSpacing = 6.0;
            markSettings.candidateStep = 0.5;
            markSettings.spacingRelaxFactor = 0.88;
            const double requestedMargin =
                std::isfinite(
                    publicSettings.registrationMarginMillimeters)
                ? std::clamp(
                    publicSettings.registrationMarginMillimeters,
                    0.0,
                    1000.0)
                : 5.0;

            markSettings.pageInset =
                markSettings.radius + requestedMargin;
            markSettings.corridorProbeDistance = 40.0;
            markSettings.forceBottomRightAnchor = true;
            markSettings.maxCandidates = 2000000;
            markSettings.maxSelectionCandidates = 120000;
            const auto density = AdaptiveRegistrationDensity(pageWidth, pageHeight);
            markSettings.preferredSpacing = density.preferredSpacing;
            markSettings.absoluteMinSpacing = density.minimumSpacing;

            const auto planningStart =
                std::chrono::steady_clock::now();

            const RegmarkPlan plan =
                GenerateRegmarkPlan(
                    pageBounds,
                    geometrySnapshot.collision,
                    density.count,
                    markSettings);

            const auto planningEnd =
                std::chrono::steady_clock::now();

            auto regMarks =
                app->CreateShapeRange();

            auto black =
                app->CreateCMYKColor(
                    0,
                    0,
                    0,
                    100);

            auto printLayer =
                printPage->ActiveLayer;

            long markNumber = 0;
            for (const Point2D& point : plan.marks)
            {
                auto mark =
                    printLayer->CreateEllipse2(
                        point.x,
                        point.y,
                        markSettings.radius,
                        markSettings.radius,
                        90,
                        90,
                        VARIANT_FALSE);

                mark->Fill->ApplyUniformFill(
                    black);

                mark->Outline->SetNoOutline();
                mark->Name = _bstr_t(("Marca de registro #" +
                    std::to_string(++markNumber)).c_str());

                regMarks->Add(mark);
            }

            auto cutPage =
                doc->InsertPagesEx(
                    1,
                    VARIANT_FALSE,
                    printPage->Index,
                    pageWidth,
                    pageHeight);

            if (!cutPage)
            {
                throw std::runtime_error(
                    "Unable to create CUT page.");
            }

            if (regMarks &&
                regMarks->Count > 0)
            {
                auto duplicatedMarks =
                    regMarks->Duplicate(
                        0.0,
                        0.0);

                duplicatedMarks->MoveToLayer(
                    cutPage->ActiveLayer);
            }

            auto cutShapes =
                geometrySnapshot.cutShapes;

            if (cutShapes &&
                cutShapes->Count > 0)
            {
                auto duplicatedCut =
                    cutShapes->Duplicate(
                        0.0,
                        0.0);

                duplicatedCut->MoveToLayer(
                    cutPage->ActiveLayer);

                cutShapes->Delete();
            }

            if (publicSettings.namePages)
            {
                const std::string printName =
                    "PRINT " +
                    std::to_string(
                        closureIndex);

                const std::string cutName =
                    "CUT " +
                    std::to_string(
                        closureIndex);

                printPage->PutName(
                    _bstr_t(
                        printName.c_str()));

                cutPage->PutName(
                    _bstr_t(
                        cutName.c_str()));
            }

            ClosureProcessStats stats;

            stats.regmarksCreated =
                static_cast<int>(
                    plan.marks.size());

            if (plan.marks.size() < 3) ++stats.warnings;

            const auto documentEnd =
                std::chrono::steady_clock::now();

            stats.geometryMs =
                std::chrono::duration<double, std::milli>(
                    geometryEnd - geometryStart).count();

            stats.planningMs =
                std::chrono::duration<double, std::milli>(
                    planningEnd - planningStart).count();

            stats.documentMs =
                std::chrono::duration<double, std::milli>(
                    documentEnd - documentStart).count() -
                stats.geometryMs -
                stats.planningMs;

            {
                std::ostringstream perf;
                perf
                    << "[ImCut.Cut] Closure "
                    << closureIndex
                    << " | geometry="
                    << std::fixed
                    << std::setprecision(1)
                    << stats.geometryMs
                    << " ms | planning="
                    << stats.planningMs
                    << " ms | document="
                    << stats.documentMs
                    << " ms | collision="
                    << (geometrySnapshot.collision.fromCutContours
                        ? "CutContour"
                        : "TopLevelBBox")
                    << " | shapes="
                    << geometrySnapshot.collision.sourceShapeCount
                    << " | points="
                    << geometrySnapshot.collision.pointCount
                    << " | segments="
                    << geometrySnapshot.collision.segments.size()
                    << "\n";

                OutputDebugStringA(
                    perf.str().c_str());
            }

            printPageOut = printPage;
            cutPageOut = cutPage;

            return stats;
        }
    }

    Result Process(
        IVGApplicationPtr& spApp,
        const Settings& settings)
    {
        Result result;
        const auto operationStart = std::chrono::steady_clock::now();
        bool rollbackFailed = false;
        bool rollbackPerformed = false;

        Cancellation::ThrowIfRequested();

        if (!spApp)
        {
            result.error =
                "CorelDRAW application is not available.";

            return result;
        }

        try
        {
            auto doc =
                spApp->ActiveDocument;

            if (!doc)
            {
                result.error =
                    "No active document.";

                return result;
            }

            auto selection =
                spApp->ActiveSelectionRange;

            if (!selection ||
                selection->Count <= 0)
            {
                result.error =
                    "Select at least one object.";

                return result;
            }

            auto selectedShapes =
                selection->Shapes->All();

            if (!selectedShapes ||
                selectedShapes->Count <= 0)
            {
                result.error =
                    "Select at least one object.";

                return result;
            }

            auto closures =
                BuildClosureRanges(
                    spApp,
                    selectedShapes,
                    settings.closureMode);

            if (closures.empty())
            {
                result.error =
                    "No valid closures were detected.";

                return result;
            }

            result.closureCount =
                static_cast<int>(
                    closures.size());

            IVGPagePtr firstPrintPage;
            IVGPagePtr previousCutPage;
            IVGPagePtr sourcePrintPage =
                spApp->ActivePage;

            if (!sourcePrintPage)
            {
                result.error =
                    "No active page.";

                return result;
            }

            double geometryTotalMs = 0.0;
            double planningTotalMs = 0.0;
            double documentTotalMs = 0.0;

            {
                CorelExecutionGuard guard(
                    spApp,
                    doc,
                    rollbackFailed,
                    rollbackPerformed);

                guard.Begin(
                    "ImCut - Prepare Cut");

                for (std::size_t i = 0;
                    i < closures.size();
                    ++i)
                {
                    Cancellation::ThrowIfRequested();

                    IVGPagePtr printPage;
                    IVGPagePtr cutPage;

                    const IVGPagePtr existingPrintPage =
                        i == 0
                        ? sourcePrintPage
                        : IVGPagePtr();

                    const auto stats =
                        ProcessClosureRange(
                            spApp,
                            doc,
                            closures[i],
                            settings,
                            static_cast<int>(
                                i + 1),
                            existingPrintPage,
                            previousCutPage,
                            printPage,
                            cutPage);

                    if (!firstPrintPage)
                    {
                        firstPrintPage =
                            printPage;
                    }

                    previousCutPage =
                        cutPage;

                    ++result.processedCount;

                    result.regmarksCreated +=
                        stats.regmarksCreated;

                    result.warnings +=
                        stats.warnings;

                    geometryTotalMs +=
                        stats.geometryMs;

                    planningTotalMs +=
                        stats.planningMs;

                    documentTotalMs +=
                        stats.documentMs;
                }

                if (firstPrintPage)
                    firstPrintPage->Activate();
                guard.Commit();
            }

            if (rollbackFailed)
                throw std::runtime_error("CorelDRAW could not close the cut command group.");

            result.elapsedMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - operationStart).count();
            result.geometryMs = geometryTotalMs;
            result.planningMs = planningTotalMs;
            result.documentMs = documentTotalMs;
            if (settings.showSummary)
            {
                std::wostringstream message;
                message << result.processedCount << L" fechamento(s) concluído(s).\n"
                    << result.regmarksCreated << L" marcas de registro.\n"
                    << std::fixed << std::setprecision(2) << result.elapsedMs / 1000.0 << L" s";
                if (result.warnings > 0)
                    message << L"\n\n" << result.warnings
                        << L" fechamento(s) com menos de 3 marcas. Confira antes de produzir.";
                HWND owner = nullptr;
                try { owner = reinterpret_cast<HWND>(static_cast<INT_PTR>(spApp->AppWindow->Handle)); }
                catch (...) {}
                MessageBoxW(owner, message.str().c_str(), L"ImCut - Fechamento", MB_OK |
                    (result.warnings > 0 ? MB_ICONWARNING : MB_ICONINFORMATION));
            }

            return result;
        }
        catch (const OperationCancelled&)
        {
            throw;
        }
        catch (const _com_error& error)
        {
            result.error =
                std::string(
                    "CorelDRAW COM error: ") +
                ComErrorToString(error);
        }
        catch (const std::exception& error)
        {
            result.error =
                error.what();
        }
        catch (...)
        {
            result.error =
                "Unknown error while preparing cut closures.";
        }

        if (rollbackFailed)
            result.error += " A restauracao automatica falhou; confira o documento e use Desfazer antes de continuar.";
        else if (rollbackPerformed)
        {
            result.processedCount = 0;
            result.regmarksCreated = 0;
            result.error += " As alteracoes do lote foram desfeitas.";
        }
        return result;
    }


    void StartCutMarks(
        IVGApplicationPtr& spApp,
        int width,
        int regmarks)
    {
        Settings settings;
        settings.pageWidth = width;
        settings.regmarks = regmarks;
        settings.closureMode =
            ClosureMode::WholeSelection;
        settings.namePages = false;
        settings.showSummary = true;
        settings.forceBottomRightAnchor = true;

        const Result result =
            Process(
                spApp,
                settings);

        if (!result.error.empty())
        {
            MessageBoxA(
                nullptr,
                result.error.c_str(),
                "ImCut.Cut",
                MB_OK | MB_ICONERROR);
        }
    }
}
