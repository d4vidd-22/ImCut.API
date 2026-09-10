#include "CurvePathBuilder.hpp"

#include <cmath>

namespace ImCut::Geometry::CorelAdapter
{
    const char* ToString(BuildIssue issue) noexcept
    {
        switch (issue)
        {
            case BuildIssue::None:                 return "None";
            case BuildIssue::CurveWithoutControls: return "CurveWithoutControls";
            case BuildIssue::InvalidCurveStructure: return "InvalidCurveStructure";
            case BuildIssue::ControlsBeforeLine:   return "ControlsBeforeLine";
            case BuildIssue::DanglingControls:     return "DanglingControls";
            case BuildIssue::ElementBeforeStart:   return "ElementBeforeStart";
            case BuildIssue::UnknownElementType:   return "UnknownElementType";
            case BuildIssue::NonFiniteCoordinate:  return "NonFiniteCoordinate";
            case BuildIssue::InvalidUnitScale:     return "InvalidUnitScale";
            case BuildIssue::InvalidTolerance:     return "InvalidTolerance";
        }
        return "Unknown";
    }

    namespace
    {
        // Parser state. The point of naming these is that every transition is either
        // listed or an error: there is no branch left where an unexpected element can be
        // absorbed into something that looks like valid geometry.
        enum class State : std::uint8_t
        {
            AwaitStart,
            InContour,
            OneControl,
            TwoControls
        };
    }

    BuildOutcome BuildContours(const CurveElementView* elements, std::size_t count,
                               double millimetresPerUnit, double weldTolerance, Path& out)
    {
        BuildOutcome outcome;

        if (!std::isfinite(millimetresPerUnit) || millimetresPerUnit <= 0.0 ||
            millimetresPerUnit > kMaxMillimetresPerUnit)
        {
            outcome.issue = BuildIssue::InvalidUnitScale;
            return outcome;
        }

        if (!std::isfinite(weldTolerance) || weldTolerance < 0.0 ||
            weldTolerance > kMaxWeldTolerance)
        {
            outcome.issue = BuildIssue::InvalidTolerance;
            return outcome;
        }

        if (elements == nullptr || count == 0)
        {
            outcome.ok = true;
            return outcome;
        }

        const std::size_t firstContour = out.contours.size();

        Contour current;
        Vec2 controls[2]{};
        State state = State::AwaitStart;
        bool closed = false;

        // Counts subpaths seen in this stream, not contours committed to `out`. A
        // rejected subpath still advances it, so a diagnostic names the subpath the file
        // actually contains rather than an index into a vector the rollback has emptied.
        std::size_t subpath = 0;

        auto fail = [&](BuildIssue issue, std::size_t index) -> BuildOutcome
        {
            // Anything already appended from this stream is discarded: half a shape is
            // more dangerous than none, because it still looks like a valid outline.
            out.contours.resize(firstContour);
            outcome.ok = false;
            outcome.issue = issue;
            outcome.elementIndex = index;
            outcome.elementType = elements[index].elementType;
            outcome.contourIndex = subpath;
            return outcome;
        };

        auto commit = [&]()
        {
            if (current.nodes.size() >= 2)
            {
                if (closed)
                    current.Close(weldTolerance);

                out.contours.push_back(std::move(current));
            }

            current = Contour{};
            closed = false;
        };

        for (std::size_t i = 0; i < count; ++i)
        {
            const CurveElementView& element = elements[i];

            if (!std::isfinite(element.x) || !std::isfinite(element.y))
                return fail(BuildIssue::NonFiniteCoordinate, i);

            // Both operands can be finite and the product still overflow, so the check
            // has to be on the converted value, not only on what arrived.
            const Vec2 point{ element.x * millimetresPerUnit, element.y * millimetresPerUnit };
            if (!std::isfinite(point.x) || !std::isfinite(point.y))
                return fail(BuildIssue::NonFiniteCoordinate, i);

            const bool elementClosed = (element.flags & kClosedFlag) != 0;
            const ElementKind kind = static_cast<ElementKind>(element.elementType);

            switch (kind)
            {
                case ElementKind::Start:
                    if (state == State::OneControl || state == State::TwoControls)
                        return fail(BuildIssue::DanglingControls, i);

                    if (state != State::AwaitStart)
                    {
                        commit();
                        ++subpath;
                    }

                    current.MoveTo(point);
                    closed = elementClosed;
                    state = State::InContour;
                    break;

                case ElementKind::Control:
                    switch (state)
                    {
                        case State::AwaitStart:  return fail(BuildIssue::ElementBeforeStart, i);
                        case State::InContour:   controls[0] = point; state = State::OneControl; break;
                        case State::OneControl:  controls[1] = point; state = State::TwoControls; break;
                        case State::TwoControls: return fail(BuildIssue::InvalidCurveStructure, i);
                    }
                    closed = closed || elementClosed;
                    break;

                case ElementKind::Curve:
                    if (state == State::AwaitStart)
                        return fail(BuildIssue::ElementBeforeStart, i);
                    if (state != State::TwoControls)
                        return fail(BuildIssue::CurveWithoutControls, i);

                    current.CubicTo(controls[0], controls[1], point);
                    state = State::InContour;
                    closed = closed || elementClosed;
                    break;

                case ElementKind::Line:
                    switch (state)
                    {
                        case State::AwaitStart:  return fail(BuildIssue::ElementBeforeStart, i);
                        case State::OneControl:
                        case State::TwoControls: return fail(BuildIssue::ControlsBeforeLine, i);
                        case State::InContour:   break;
                    }

                    current.LineTo(point);
                    closed = closed || elementClosed;
                    break;

                default:
                    return fail(BuildIssue::UnknownElementType, i);
            }
        }

        if (state == State::OneControl || state == State::TwoControls)
            return fail(BuildIssue::DanglingControls, count - 1);

        if (state != State::AwaitStart)
        {
            commit();
            ++subpath;
        }

        outcome.ok = true;
        outcome.contourIndex = out.contours.size() - firstContour;
        return outcome;
    }
}
