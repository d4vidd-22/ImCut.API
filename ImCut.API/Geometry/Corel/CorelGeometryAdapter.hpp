#pragma once

// The ONLY component of ImCut::Geometry that knows CorelDRAW exists.
//
// Dependency direction is one-way: consumers depend on the kernel, the kernel depends
// on nothing. This adapter sits at the boundary and does exactly one job - move
// geometry out of COM and into plain memory. No mathematics lives here.
//
// Compiled only inside ImCut.API.dll; it is excluded from the standalone kernel build
// because it needs #import <VGCoreAuto.tlb>. The path assembly it drives lives in
// CurvePathBuilder, which is COM-free and covered by the normal test suite.

#include "../../Global.hpp"

#include "../GeometryResult.hpp"
#include "../GeometrySession.hpp"
#include "../GeometryTolerance.hpp"
#include "../GeometryTypes.hpp"
#include "CurvePathBuilder.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace ImCut::Geometry::CorelAdapter
{
    struct ExtractionOptions
    {
        // DisplayCurve reflects what the user sees, effects included. The raw Curve of
        // a shape ignores contours, envelopes and similar, which is rarely what a cut
        // or bleed operation should act on.
        bool useDisplayCurve = true;

        // Skip shapes the document marks invisible.
        bool skipInvisible = false;

        // Recurse into groups instead of skipping them.
        bool expandGroups = true;

        // Distance below which a closed subpath's final node is treated as coincident
        // with its first. Zero would leave a sub-nanometre stub segment behind whenever
        // Corel's closing node is not bit-identical to the start.
        double closureTolerance = GeometryTolerance::Production().closure;
    };

    // Why an object was rejected or a fact could not be established. Every one of these
    // used to be a silent fallback that made bad geometry look like good geometry.
    enum class ExtractionIssue : std::uint8_t
    {
        DocumentUnavailable,
        UnitConversionFailed,
        UnitConversionImplausible,
        CurveUnavailable,
        CurveInfoUnavailable,
        CurveInfoLayoutUnexpected,
        FillModeUnavailable,
        FillModeUnknown,
        MalformedCurveStream,
        VisibilityUnavailable,
        GroupTraversalFailed,
        ShapeIdentityUnavailable
    };

    [[nodiscard]] const char* ToString(ExtractionIssue issue) noexcept;

    struct ExtractionDiagnostic
    {
        ExtractionIssue issue = ExtractionIssue::DocumentUnavailable;

        // The source object the problem belongs to (Corel StaticID), preserved so a
        // caller can point the user at the actual shape. Zero when the identity itself
        // could not be read.
        std::int64_t sourceObject = 0;

        // Position inside the shape's curve stream, when the issue has one: which
        // subpath, which element, and what that element claimed to be. Enough for a
        // report to name the offending segment without re-reading the document.
        std::size_t contourIndex = 0;
        std::size_t elementIndex = 0;
        std::int32_t elementType = 0;

        std::string detail;
    };

    struct ExtractionReport
    {
        long shapesVisited = 0;
        long shapesExtracted = 0;
        long shapesSkipped = 0;
        long contoursExtracted = 0;
        long segmentsExtracted = 0;

        // COM invocations ATTEMPTED, counted immediately before each call, so a failed
        // call is counted too - the round trip was paid for either way.
        //
        // One increment per actual invocation, not per statement: a chained expression
        // like `shape->Shapes->All()` is two calls and counts as two. This is the number
        // the "extract once, query millions" contract is measured against, so it has to
        // mean what it says.
        long comCalls = 0;

        std::vector<ExtractionDiagnostic> diagnostics;

        std::string lastError;

        [[nodiscard]] bool HasError() const noexcept { return !lastError.empty(); }
        [[nodiscard]] bool HasDiagnostics() const noexcept { return !diagnostics.empty(); }

        void Note(ExtractionIssue issue, std::int64_t sourceObject, std::string detail);
    };

    // Millimetres per unit of the document's active unit, obtained with a single
    // ConvertUnits call. The document is never modified: switching its unit to get
    // millimetre coordinates would be a visible side effect of a read-only query.
    //
    // Failure is a status, never the number 1.0. Silently treating an unknown unit as
    // millimetres turns a 10 inch part into a 10 mm part, which is a scrapped sheet
    // rather than an error message.
    [[nodiscard]] GeometryResult<double> MillimetresPerDocumentUnit(const IVGApplicationPtr& application,
                                                                    const IVGDocumentPtr& document,
                                                                    ExtractionReport& report);

    // The shape's fill rule as CorelDRAW reports it. cdrFillMode has exactly two
    // members (Alternate, Winding); anything else, including a COM failure, is a
    // failure status rather than a guess, because guessing decides whether a ring is a
    // hole or a solid.
    // `sourceObject` is only used to label diagnostics; pass the shape's StaticID when
    // the caller already has it, so a rejection can name the shape it came from.
    [[nodiscard]] GeometryResult<FillRule> ShapeFillRule(const IVGShapePtr& shape,
                                                         ExtractionReport& report,
                                                         std::int64_t sourceObject = 0);

    // Extracts one shape's outline into a Path in millimetres.
    //
    // Geometry comes back through IVGCurve::GetCurveInfo, which returns the entire
    // path - nodes and Bezier control points alike - in one COM call. Walking
    // SubPaths/Nodes/Segments instead would cost thousands of round trips per shape
    // for the same data.
    [[nodiscard]] bool ExtractShape(const IVGShapePtr& shape, double millimetresPerUnit,
                                    Path& out, ExtractionReport& report,
                                    const ExtractionOptions& options = {});

    // Extracts a caller-supplied range into a snapshot. The caller decides what to
    // process: the adapter never consults the clipboard or the active selection.
    //
    // A unit that cannot be resolved fails the whole extraction; a single malformed
    // shape is recorded and skipped, since the scale is global and the malformation is
    // not.
    [[nodiscard]] bool ExtractRange(const IVGApplicationPtr& application,
                                    const IVGShapeRangePtr& range,
                                    GeometrySnapshot& out, ExtractionReport& report,
                                    const ExtractionOptions& options = {});

    [[nodiscard]] bool ExtractShapes(const IVGApplicationPtr& application,
                                     const IVGShapePtr* shapes, std::size_t count,
                                     GeometrySnapshot& out, ExtractionReport& report,
                                     const ExtractionOptions& options = {});
}
