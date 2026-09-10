#pragma once

// Assembly of CorelDRAW's flat CurveElement stream into kernel Contours.
//
// Split out of the COM adapter deliberately: the translation is where the interesting
// failure modes live (a cubic arriving without its control points, an element type the
// type library no longer matches), and none of them need CorelDRAW running to be
// exercised. Keeping this COM-free means the semantics are covered by the normal test
// suite instead of being asserted and left unverified.
//
// No COM, no VGCore, no #import - GeometryTypes only.

#include "../GeometryTypes.hpp"

#include <cstddef>
#include <cstdint>

namespace ImCut::Geometry::CorelAdapter
{
    // Mirrors CorelDRAW's CurveElement (PositionX, PositionY, ElementType, NodeType,
    // Flags) minus the fields path assembly does not read. Values are document units;
    // the builder applies the millimetre factor.
    struct CurveElementView
    {
        double x = 0.0;
        double y = 0.0;
        std::int32_t elementType = 0;
        unsigned char flags = 0;
    };

    // cdrCurveElementType, verified against VGCoreAuto.tlb (CorelDRAW Graphics Suite 27).
    enum class ElementKind : std::int32_t
    {
        Start   = 0,
        Line    = 1,
        Curve   = 2,
        Control = 3
    };

    // cdrFlagClosed from cdrCurveElementFlags.
    inline constexpr unsigned char kClosedFlag = 8;

    enum class BuildIssue : std::uint8_t
    {
        None,

        // A Curve element arrived without the two Control elements that describe it.
        // Emitting a straight line here would silently replace a curved cut path with
        // a chord, so the shape is rejected instead.
        CurveWithoutControls,

        // A third Control before a Curve. A cubic takes exactly two; a third means this
        // is not the element layout compiled against, and keeping the last two would
        // invent a curve out of data of unknown meaning.
        InvalidCurveStructure,

        // Controls followed by a Line. The controls describe a curve that never came,
        // and discarding them loses whatever the file actually said.
        ControlsBeforeLine,

        // Controls still pending when the subpath ended.
        DanglingControls,

        // Geometry before any Start element: the stream is not a valid path.
        ElementBeforeStart,

        // An ElementType outside cdrCurveElementType. Skipping it would drop geometry
        // without telling anyone, which is how a type-library change becomes a wrong
        // cut instead of an error.
        UnknownElementType,

        // A coordinate that is not finite, either as it arrived or after the unit
        // conversion: two finite operands can still overflow to infinity.
        NonFiniteCoordinate,

        InvalidUnitScale,
        InvalidTolerance
    };

    // Largest millimetres-per-unit factor treated as a real document unit. The coarsest
    // unit CorelDRAW reports is the mile, at 1.6e6 mm; anything past this is a corrupt
    // or misinterpreted factor rather than a unit, and multiplying coordinates by it
    // produces numbers no cutter could act on.
    inline constexpr double kMaxMillimetresPerUnit = 1.0e7;

    // A weld tolerance is a distance between adjacent nodes. Beyond this it would start
    // collapsing whole contours instead of duplicate nodes.
    inline constexpr double kMaxWeldTolerance = 1.0e6;

    [[nodiscard]] const char* ToString(BuildIssue issue) noexcept;

    struct BuildOutcome
    {
        bool ok = false;
        BuildIssue issue = BuildIssue::None;

        // Where the problem was, precise enough for a diagnostic to name it without the
        // caller re-parsing anything: which subpath, which element inside the stream,
        // and what that element claimed to be.
        //
        // `contourIndex` counts subpaths encountered in this stream, so it stays correct
        // after the rollback empties the output. On success it holds the number of
        // contours appended.
        std::size_t contourIndex = 0;
        std::size_t elementIndex = 0;
        std::int32_t elementType = 0;
    };

    // Appends the stream's subpaths to `out.contours`.
    //
    // Strict by design: anything structurally unexpected fails rather than being
    // repaired into plausible geometry. `millimetresPerUnit` must be finite, positive
    // and below kMaxMillimetresPerUnit; `weldTolerance` must be finite, non-negative and
    // below kMaxWeldTolerance.
    [[nodiscard]] BuildOutcome BuildContours(const CurveElementView* elements, std::size_t count,
                                             double millimetresPerUnit, double weldTolerance,
                                             Path& out);
}
