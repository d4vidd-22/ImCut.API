#define NOMINMAX 1

#include "CorelGeometryAdapter.hpp"

#include <cmath>
#include <comdef.h>

namespace ImCut::Geometry::CorelAdapter
{
    namespace
    {
        [[nodiscard]] std::string Narrow(const _bstr_t& value)
        {
            const char* text = static_cast<const char*>(value);
            return text != nullptr ? std::string(text) : std::string();
        }

        [[nodiscard]] std::string ComErrorText(const _com_error& error)
        {
            std::string message = Narrow(error.Description());
            if (message.empty())
            {
                const TCHAR* fallback = error.ErrorMessage();
                if (fallback != nullptr)
                    message = Narrow(_bstr_t(fallback));
            }
            return message;
        }

        // StaticID identifies the source object in diagnostics. Read before the
        // geometry so a failure can still name the shape it happened on.
        [[nodiscard]] std::int64_t ShapeIdentity(const IVGShapePtr& shape, ExtractionReport& report)
        {
            if (!shape)
                return 0;

            try
            {
                ++report.comCalls;
                return static_cast<std::int64_t>(shape->StaticID);
            }
            catch (const _com_error& error)
            {
                report.Note(ExtractionIssue::ShapeIdentityUnavailable, 0, ComErrorText(error));
                return 0;
            }
        }

        // Reads the whole curve in one call and unpacks the SAFEARRAY of CurveElement.
        //
        // CurveElement layout is fixed by the type library:
        //   PositionX, PositionY, ElementType, NodeType, Flags
        // with ElementType in {Start, Line, Curve, Control} and cdrFlagClosed marking a
        // closed subpath.
        [[nodiscard]] bool ReadCurveElements(const IVGCurvePtr& curve, std::int64_t sourceObject,
                                             std::vector<CurveElementView>& elements,
                                             ExtractionReport& report)
        {
            elements.clear();

            if (!curve)
                return false;

            SAFEARRAY* array = nullptr;
            try
            {
                ++report.comCalls;
                array = curve->GetCurveInfo();
            }
            catch (const _com_error& error)
            {
                report.lastError = "GetCurveInfo failed: " + ComErrorText(error);
                report.Note(ExtractionIssue::CurveInfoUnavailable, sourceObject, ComErrorText(error));
                return false;
            }

            if (array == nullptr)
            {
                report.Note(ExtractionIssue::CurveInfoUnavailable, sourceObject,
                            "GetCurveInfo returned no array");
                return false;
            }

            // Guard the element size: a type-library change would otherwise be
            // reinterpreted as garbage coordinates rather than reported.
            if (SafeArrayGetDim(array) != 1 || array->cbElements != sizeof(CurveElement))
            {
                SafeArrayDestroy(array);
                report.lastError = "GetCurveInfo returned an unexpected element layout";
                report.Note(ExtractionIssue::CurveInfoLayoutUnexpected, sourceObject,
                            "element size does not match CurveElement");
                return false;
            }

            LONG lower = 0;
            LONG upper = -1;
            if (FAILED(SafeArrayGetLBound(array, 1, &lower)) ||
                FAILED(SafeArrayGetUBound(array, 1, &upper)) || upper < lower)
            {
                SafeArrayDestroy(array);
                report.Note(ExtractionIssue::CurveInfoUnavailable, sourceObject,
                            "GetCurveInfo returned empty bounds");
                return false;
            }

            CurveElement* data = nullptr;
            if (FAILED(SafeArrayAccessData(array, reinterpret_cast<void**>(&data))) || data == nullptr)
            {
                SafeArrayDestroy(array);
                report.lastError = "SafeArrayAccessData failed on GetCurveInfo result";
                report.Note(ExtractionIssue::CurveInfoUnavailable, sourceObject,
                            "SafeArrayAccessData failed");
                return false;
            }

            const std::size_t count = static_cast<std::size_t>(upper - lower + 1);
            elements.resize(count);
            for (std::size_t i = 0; i < count; ++i)
            {
                elements[i].x = data[i].PositionX;
                elements[i].y = data[i].PositionY;
                elements[i].elementType = static_cast<std::int32_t>(data[i].ElementType);
                elements[i].flags = data[i].Flags;
            }

            SafeArrayUnaccessData(array);
            SafeArrayDestroy(array);
            return true;
        }

        [[nodiscard]] bool IsGroup(const IVGShapePtr& shape, std::int64_t sourceObject,
                                   ExtractionReport& report)
        {
            try
            {
                ++report.comCalls;
                return shape->Type == cdrGroupShape;
            }
            catch (const _com_error& error)
            {
                report.Note(ExtractionIssue::GroupTraversalFailed, sourceObject, ComErrorText(error));
                return false;
            }
        }

    }

    // Extraction with an already-resolved source identity. ExtractRecursive reads
    // StaticID to label diagnostics before it knows whether the shape is a group, and
    // re-reading it inside the extraction would spend a second COM call per shape for a
    // value already in hand.
    [[nodiscard]] bool ExtractShapeWithIdentity(const IVGShapePtr& shape, double millimetresPerUnit,
                                                std::int64_t sourceObject, Path& out,
                                                ExtractionReport& report,
                                                const ExtractionOptions& options);

    namespace
    {
        void ExtractRecursive(const IVGShapePtr& shape, double scale, GeometrySnapshot& out,
                              ExtractionReport& report, const ExtractionOptions& options)
        {
            if (!shape)
                return;

            ++report.shapesVisited;

            const std::int64_t sourceObject = ShapeIdentity(shape, report);

            if (options.skipInvisible)
            {
                try
                {
                    ++report.comCalls;
                    if (shape->Visible == VARIANT_FALSE)
                    {
                        ++report.shapesSkipped;
                        return;
                    }
                }
                catch (const _com_error& error)
                {
                    // Visibility is a filter, not geometry: it is reported, and the
                    // shape is still extracted rather than dropped on a COM hiccup.
                    report.Note(ExtractionIssue::VisibilityUnavailable, sourceObject,
                                ComErrorText(error));
                }
            }

            if (options.expandGroups && IsGroup(shape, sourceObject, report))
            {
                try
                {
                    // Two calls, not one: the property fetch and the method on its
                    // result. Counting the chain as a single call understates the COM
                    // traffic the extract-once contract is supposed to be measured by.
                    ++report.comCalls;
                    IVGShapesPtr shapes = shape->Shapes;
                    if (shapes)
                    {
                        ++report.comCalls;
                        IVGShapeRangePtr children = shapes->All();
                        if (children)
                        {
                            ++report.comCalls;
                            const long count = children->Count;
                            for (long i = 1; i <= count; ++i)
                            {
                                ++report.comCalls;
                                ExtractRecursive(children->Item[i], scale, out, report, options);
                            }
                        }
                    }
                    return;
                }
                catch (const _com_error& error)
                {
                    report.lastError = "Group traversal failed: " + ComErrorText(error);
                    report.Note(ExtractionIssue::GroupTraversalFailed, sourceObject,
                                ComErrorText(error));
                    ++report.shapesSkipped;
                    return;
                }
            }

            Path path;
            if (!ExtractShapeWithIdentity(shape, scale, sourceObject, path, report, options) ||
                path.contours.empty())
            {
                ++report.shapesSkipped;
                return;
            }

            for (const Contour& contour : path.contours)
            {
                report.segmentsExtracted += static_cast<long>(contour.SegmentCount());
                ++report.contoursExtracted;
            }

            out.Add(std::move(path), sourceObject);
            ++report.shapesExtracted;
        }
    }

    const char* ToString(ExtractionIssue issue) noexcept
    {
        switch (issue)
        {
            case ExtractionIssue::DocumentUnavailable:       return "DocumentUnavailable";
            case ExtractionIssue::UnitConversionFailed:      return "UnitConversionFailed";
            case ExtractionIssue::UnitConversionImplausible: return "UnitConversionImplausible";
            case ExtractionIssue::CurveUnavailable:          return "CurveUnavailable";
            case ExtractionIssue::CurveInfoUnavailable:      return "CurveInfoUnavailable";
            case ExtractionIssue::CurveInfoLayoutUnexpected: return "CurveInfoLayoutUnexpected";
            case ExtractionIssue::FillModeUnavailable:       return "FillModeUnavailable";
            case ExtractionIssue::FillModeUnknown:           return "FillModeUnknown";
            case ExtractionIssue::MalformedCurveStream:      return "MalformedCurveStream";
            case ExtractionIssue::VisibilityUnavailable:     return "VisibilityUnavailable";
            case ExtractionIssue::GroupTraversalFailed:      return "GroupTraversalFailed";
            case ExtractionIssue::ShapeIdentityUnavailable:  return "ShapeIdentityUnavailable";
        }
        return "Unknown";
    }

    void ExtractionReport::Note(ExtractionIssue issue, std::int64_t sourceObject, std::string detail)
    {
        ExtractionDiagnostic diagnostic;
        diagnostic.issue = issue;
        diagnostic.sourceObject = sourceObject;
        diagnostic.detail = std::move(detail);
        diagnostics.push_back(std::move(diagnostic));
    }

    GeometryResult<double> MillimetresPerDocumentUnit(const IVGApplicationPtr& application,
                                                      const IVGDocumentPtr& document,
                                                      ExtractionReport& report)
    {
        if (!application || !document)
        {
            report.lastError = "Unit conversion requires an application and a document";
            report.Note(ExtractionIssue::DocumentUnavailable, 0, report.lastError);
            return GeometryResult<double>::Failure(GeometryStatus::InvalidInput);
        }

        try
        {
            ++report.comCalls;
            const cdrUnit unit = document->Unit;
            if (unit == cdrMillimeter)
                return GeometryResult<double>::Success(1.0);

            // One call yields the factor; converting per coordinate would put a COM
            // round trip on every point of every path.
            ++report.comCalls;
            const double scale = application->ConvertUnits(1.0, unit, cdrMillimeter);

            if (!std::isfinite(scale) || scale <= 0.0)
            {
                report.lastError = "ConvertUnits returned an unusable factor";
                report.Note(ExtractionIssue::UnitConversionImplausible, 0, report.lastError);
                return GeometryResult<double>::Failure(GeometryStatus::NumericalFailure);
            }

            return GeometryResult<double>::Success(scale);
        }
        catch (const _com_error& error)
        {
            report.lastError = "Unit conversion failed: " + ComErrorText(error);
            report.Note(ExtractionIssue::UnitConversionFailed, 0, ComErrorText(error));
            return GeometryResult<double>::Failure(GeometryStatus::Unsupported);
        }
    }

    GeometryResult<FillRule> ShapeFillRule(const IVGShapePtr& shape, ExtractionReport& report,
                                           std::int64_t sourceObject)
    {
        if (!shape)
            return GeometryResult<FillRule>::Failure(GeometryStatus::InvalidInput);

        try
        {
            ++report.comCalls;
            const cdrFillMode mode = shape->FillMode;

            switch (mode)
            {
                case cdrFillAlternate: return GeometryResult<FillRule>::Success(FillRule::EvenOdd);
                case cdrFillWinding:   return GeometryResult<FillRule>::Success(FillRule::NonZero);
            }

            report.lastError = "Unknown cdrFillMode reported by the document";
            report.Note(ExtractionIssue::FillModeUnknown, sourceObject, report.lastError);
            return GeometryResult<FillRule>::Failure(GeometryStatus::Unsupported);
        }
        catch (const _com_error& error)
        {
            report.lastError = "FillMode unavailable: " + ComErrorText(error);
            report.Note(ExtractionIssue::FillModeUnavailable, sourceObject, ComErrorText(error));
            return GeometryResult<FillRule>::Failure(GeometryStatus::Unsupported);
        }
    }

    bool ExtractShape(const IVGShapePtr& shape, double millimetresPerUnit, Path& out,
                      ExtractionReport& report, const ExtractionOptions& options)
    {
        if (!shape)
            return false;

        // Public entry point: nobody has resolved the identity yet, so pay for it here.
        return ExtractShapeWithIdentity(shape, millimetresPerUnit,
                                        ShapeIdentity(shape, report), out, report, options);
    }

    bool ExtractShapeWithIdentity(const IVGShapePtr& shape, double millimetresPerUnit,
                                  std::int64_t sourceObject, Path& out,
                                  ExtractionReport& report, const ExtractionOptions& options)
    {
        if (!shape)
            return false;

        if (!std::isfinite(millimetresPerUnit) || millimetresPerUnit <= 0.0)
        {
            report.lastError = "Extraction called with an unresolved millimetre factor";
            report.Note(ExtractionIssue::UnitConversionImplausible, 0, report.lastError);
            return false;
        }

        IVGCurvePtr curve;
        try
        {
            ++report.comCalls;
            curve = options.useDisplayCurve ? shape->DisplayCurve : shape->Curve;
        }
        catch (const _com_error& error)
        {
            report.lastError = "Curve access failed: " + ComErrorText(error);
            report.Note(ExtractionIssue::CurveUnavailable, sourceObject, ComErrorText(error));
            return false;
        }

        if (!curve)
            return false;

        std::vector<CurveElementView> elements;
        if (!ReadCurveElements(curve, sourceObject, elements, report))
            return false;

        if (elements.empty())
            return false;

        // The fill rule decides whether an inner ring is a hole or a solid, so it is
        // resolved before any geometry is committed and a failure rejects the shape.
        const GeometryResult<FillRule> fillRule = ShapeFillRule(shape, report, sourceObject);
        if (!fillRule.Ok())
            return false;

        out.contours.clear();
        out.fillRule = fillRule.Value();

        const BuildOutcome outcome = BuildContours(elements.data(), elements.size(),
                                                   millimetresPerUnit, options.closureTolerance,
                                                   out);

        if (!outcome.ok)
        {
            ExtractionDiagnostic diagnostic;
            diagnostic.issue = ExtractionIssue::MalformedCurveStream;
            diagnostic.sourceObject = sourceObject;
            diagnostic.contourIndex = outcome.contourIndex;
            diagnostic.elementIndex = outcome.elementIndex;
            diagnostic.elementType = outcome.elementType;
            diagnostic.detail = ToString(outcome.issue);
            report.diagnostics.push_back(std::move(diagnostic));

            report.lastError = std::string("Malformed curve stream: ") + ToString(outcome.issue);
            out.contours.clear();
            return false;
        }

        return !out.contours.empty();
    }

    bool ExtractShapes(const IVGApplicationPtr& application, const IVGShapePtr* shapes,
                       std::size_t count, GeometrySnapshot& out, ExtractionReport& report,
                       const ExtractionOptions& options)
    {
        if (shapes == nullptr || count == 0)
            return false;

        IVGDocumentPtr document;
        try
        {
            ++report.comCalls;
            document = application ? application->ActiveDocument : IVGDocumentPtr();
        }
        catch (const _com_error& error)
        {
            report.lastError = ComErrorText(error);
            report.Note(ExtractionIssue::DocumentUnavailable, 0, report.lastError);
            return false;
        }

        const GeometryResult<double> scale = MillimetresPerDocumentUnit(application, document, report);
        if (!scale.Ok())
            return false;

        out.Reserve(out.Count() + count);
        for (std::size_t i = 0; i < count; ++i)
            ExtractRecursive(shapes[i], scale.Value(), out, report, options);

        return report.shapesExtracted > 0;
    }

    bool ExtractRange(const IVGApplicationPtr& application, const IVGShapeRangePtr& range,
                      GeometrySnapshot& out, ExtractionReport& report,
                      const ExtractionOptions& options)
    {
        if (!range)
            return false;

        IVGDocumentPtr document;
        try
        {
            ++report.comCalls;
            document = application ? application->ActiveDocument : IVGDocumentPtr();
        }
        catch (const _com_error& error)
        {
            report.lastError = ComErrorText(error);
            report.Note(ExtractionIssue::DocumentUnavailable, 0, report.lastError);
            return false;
        }

        const GeometryResult<double> scale = MillimetresPerDocumentUnit(application, document, report);
        if (!scale.Ok())
            return false;

        long count = 0;
        try
        {
            ++report.comCalls;
            count = range->Count;
        }
        catch (const _com_error& error)
        {
            report.lastError = ComErrorText(error);
            report.Note(ExtractionIssue::GroupTraversalFailed, 0, report.lastError);
            return false;
        }

        out.Reserve(out.Count() + static_cast<std::size_t>(count > 0 ? count : 0));

        for (long i = 1; i <= count; ++i)
        {
            try
            {
                ++report.comCalls;
                ExtractRecursive(range->Item[i], scale.Value(), out, report, options);
            }
            catch (const _com_error& error)
            {
                report.lastError = ComErrorText(error);
                report.Note(ExtractionIssue::GroupTraversalFailed, 0, ComErrorText(error));
                ++report.shapesSkipped;
            }
        }

        return report.shapesExtracted > 0;
    }
}
