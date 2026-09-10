#pragma once

#include "../GeometryContext.hpp"
#include "../GeometryResult.hpp"
#include "../GeometryTypes.hpp"
#include "../Intersection/Intersection.hpp"

#include <cstdint>
#include <vector>

namespace ImCut::Geometry
{
    // Geometric facts, not verdicts.
    //
    // The kernel reports what is measurably true about a path. It deliberately does not
    // decide whether any of it is acceptable, how severe it is, or how to fix it:
    // Safe/Moderate/Aggressive policy, health scoring and repair belong to the future
    // ImCut::GeometryDoctor, which will consume these findings.
    // How much of the requested analysis actually ran.
    //
    // A report that looks clean because a sub-operation was cancelled is worse than no
    // report: the caller reads "no findings" as "no problems".
    enum class ValidationCompleteness : std::uint8_t
    {
        // Every requested check ran to completion.
        Complete,

        // Some checks were skipped or truncated. Findings that ARE present are real;
        // their absence proves nothing.
        Partial,

        Cancelled,
        Failed
    };

    [[nodiscard]] constexpr const char* ToString(ValidationCompleteness completeness) noexcept
    {
        switch (completeness)
        {
            case ValidationCompleteness::Complete:  return "Complete";
            case ValidationCompleteness::Partial:   return "Partial";
            case ValidationCompleteness::Cancelled: return "Cancelled";
            case ValidationCompleteness::Failed:    return "Failed";
        }
        return "Unknown";
    }

    enum class ValidationFinding : std::uint16_t
    {
        NonFiniteCoordinate,

        // Parallel arrays out of step: the contour cannot be indexed at all.
        InconsistentStorage,

        EmptyGeometry,
        OpenContour,
        InsufficientNodes,
        ZeroLengthSegment,
        TinySegment,
        DuplicateNode,
        DegenerateCurve,
        ZeroArea,
        SelfIntersection,
        ContourIntersection,
        CoincidentContours,
        WindingInconsistency,

        // Reserved compatibility values. Validation::Validate does not emit these.
        // They shipped in the public enum before their required input semantics existed,
        // so removing them would break source/ABI compatibility. Callers must use
        // FindingAvailability() rather than infer support from enum membership.

        // Requires an authored outer/hole role, which Path intentionally does not store.
        HoleOutsideOuter,

        // Requires an authored outer/hole role. Geometry currently reports the generic
        // ContourIntersection plus its IntersectionKind instead.
        HoleTouchingOuter,

        // An operation-specific before/after fact, not an input-validation fact.
        QuantizationCollapse,

        // Reserved distinct-edge identity. Consecutive duplicate nodes are reported by
        // DuplicateNode and ZeroLengthSegment.
        DuplicateEdge,

        // Collinear overlaps are carried by SelfIntersection/ContourIntersection with
        // IntersectionKind::CollinearOverlap.
        CoincidentEdge,

        // Unsupported representations are reported through GeometryStatus::Unsupported.
        UnsupportedTopology,

        // No policy-free feature definition exists beyond the active TinySegment fact.
        TinyFeature
    };

    enum class ValidationFindingAvailability : std::uint8_t
    {
        EmittedByValidate,
        ReservedCompatibility
    };

    // The enum is public ABI; this table is the authoritative promise about which facts
    // Validation::Validate can produce. ReservedCompatibility values remain named and
    // printable but are not promises to the future GeometryDoctor.
    [[nodiscard]] constexpr ValidationFindingAvailability FindingAvailability(
        ValidationFinding finding) noexcept
    {
        switch (finding)
        {
            case ValidationFinding::NonFiniteCoordinate:
            case ValidationFinding::InconsistentStorage:
            case ValidationFinding::EmptyGeometry:
            case ValidationFinding::OpenContour:
            case ValidationFinding::InsufficientNodes:
            case ValidationFinding::ZeroLengthSegment:
            case ValidationFinding::TinySegment:
            case ValidationFinding::DuplicateNode:
            case ValidationFinding::DegenerateCurve:
            case ValidationFinding::ZeroArea:
            case ValidationFinding::SelfIntersection:
            case ValidationFinding::ContourIntersection:
            case ValidationFinding::CoincidentContours:
            case ValidationFinding::WindingInconsistency:
                return ValidationFindingAvailability::EmittedByValidate;

            case ValidationFinding::HoleOutsideOuter:
            case ValidationFinding::HoleTouchingOuter:
            case ValidationFinding::QuantizationCollapse:
            case ValidationFinding::DuplicateEdge:
            case ValidationFinding::CoincidentEdge:
            case ValidationFinding::UnsupportedTopology:
            case ValidationFinding::TinyFeature:
                return ValidationFindingAvailability::ReservedCompatibility;
        }
        return ValidationFindingAvailability::ReservedCompatibility;
    }

    [[nodiscard]] constexpr const char* ToString(ValidationFinding finding) noexcept
    {
        switch (finding)
        {
            case ValidationFinding::NonFiniteCoordinate: return "NonFiniteCoordinate";
            case ValidationFinding::InconsistentStorage: return "InconsistentStorage";
            case ValidationFinding::EmptyGeometry:       return "EmptyGeometry";
            case ValidationFinding::OpenContour:         return "OpenContour";
            case ValidationFinding::InsufficientNodes:   return "InsufficientNodes";
            case ValidationFinding::ZeroLengthSegment:   return "ZeroLengthSegment";
            case ValidationFinding::TinySegment:         return "TinySegment";
            case ValidationFinding::DuplicateNode:       return "DuplicateNode";
            case ValidationFinding::DegenerateCurve:     return "DegenerateCurve";
            case ValidationFinding::ZeroArea:            return "ZeroArea";
            case ValidationFinding::SelfIntersection:    return "SelfIntersection";
            case ValidationFinding::ContourIntersection: return "ContourIntersection";
            case ValidationFinding::CoincidentContours:  return "CoincidentContours";
            case ValidationFinding::HoleOutsideOuter:    return "HoleOutsideOuter";
            case ValidationFinding::HoleTouchingOuter:   return "HoleTouchingOuter";
            case ValidationFinding::QuantizationCollapse: return "QuantizationCollapse";
            case ValidationFinding::DuplicateEdge:       return "DuplicateEdge";
            case ValidationFinding::CoincidentEdge:      return "CoincidentEdge";
            case ValidationFinding::UnsupportedTopology: return "UnsupportedTopology";
            case ValidationFinding::TinyFeature:         return "TinyFeature";
            case ValidationFinding::WindingInconsistency:return "WindingInconsistency";
        }
        return "Unknown";
    }

    struct ValidationIssue
    {
        ValidationFinding finding = ValidationFinding::EmptyGeometry;
        std::uint32_t contour = 0;
        std::uint32_t segment = 0;
        Vec2 location;

        // How the two primitives met, for intersection findings.
        //
        // The intersection routines already classify this - ProperCross, EndpointTouch,
        // Tangent, CollinearOverlap and the rest - and the value was computed and then
        // dropped on the floor. A GeometryDoctor deciding between "these outlines
        // genuinely cross and the shape is self-intersecting" and "these two rings share
        // an endpoint, which is how the artwork was drawn" cannot tell them apart from a
        // location alone. Kind::None for findings that are not about a meeting.
        IntersectionKind kind = IntersectionKind::None;
    };

    struct ValidationOptions
    {
        bool checkSelfIntersections = true;
        bool checkContourIntersections = true;
        bool checkTopology = true;

        // Stops accumulating and marks the analysis Partial once reaching the limit
        // causes a requested check to be skipped or a finding to be dropped. Merely
        // storing exactly this many findings does not by itself make a fully executed
        // analysis Partial. Zero means collect none; any non-empty requested analysis
        // is therefore Partial.
        std::size_t maxIssues = 4096;
    };

    struct ValidationReport
    {
        std::vector<ValidationIssue> issues;
        bool truncated = false;

        // Always check this before concluding anything from an empty issue list.
        ValidationCompleteness completeness = ValidationCompleteness::Complete;

        // Cheap summary so callers need not scan the list.
        std::size_t contourCount = 0;
        std::size_t segmentCount = 0;
        std::size_t openContours = 0;
        std::size_t selfIntersections = 0;

        [[nodiscard]] bool Empty() const noexcept { return issues.empty(); }
        [[nodiscard]] bool Has(ValidationFinding finding) const noexcept;
        [[nodiscard]] std::size_t Count(ValidationFinding finding) const noexcept;
    };

    namespace Validation
    {
        [[nodiscard]] GeometryResult<ValidationReport> Validate(const Path& path,
                                                                const GeometryContext& context,
                                                                const ValidationOptions& options = {});

        [[nodiscard]] GeometryResult<ValidationReport> Validate(const Contour& contour,
                                                                const GeometryContext& context,
                                                                const ValidationOptions& options = {});

        // Fast structural screen: finite coordinates, enough nodes, consistent arrays.
        // No intersection or topology work.
        [[nodiscard]] bool IsStructurallySound(const Path& path) noexcept;
    }
}
