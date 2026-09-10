#include "Winding.hpp"

#include "../Polygon/PolygonMetrics.hpp"

#include <algorithm>

namespace ImCut::Geometry::Winding
{
    void Reverse(Contour& contour)
    {
        // Section 15A. This walks SegmentAt() over the parallel arrays, which is only
        // sound behind IsStructurallyValid, and it is `void` - there is no channel to
        // report a refusal, so the refusal has to be to LEAVE THE CONTOUR ALONE.
        // Reversing the node list of a structurally broken contour would additionally
        // scramble the correspondence between nodes and the handles that no longer
        // match them, turning a detectable defect into an undetectable one.
        if (!contour.IsStructurallyValid())
            return;

        const std::size_t segmentCount = contour.SegmentCount();
        if (segmentCount == 0)
        {
            std::reverse(contour.nodes.begin(), contour.nodes.end());
            return;
        }

        // Walk the segments backwards, mirroring each one in place. Swapping c1 and c2
        // together with p0 and p1 is what makes the reversed cubic retrace the original
        // exactly rather than merely approximately.
        std::vector<Segment> reversed;
        reversed.reserve(segmentCount);
        for (std::size_t i = segmentCount; i-- > 0;)
        {
            const Segment segment = contour.SegmentAt(i);
            reversed.push_back({ segment.p1, segment.c2, segment.c1, segment.p0, segment.kind });
        }

        const bool closed = contour.closed;

        contour.nodes.clear();
        contour.handles.clear();
        contour.kinds.clear();

        contour.nodes.reserve(segmentCount + 1);
        contour.handles.reserve(segmentCount * 2);
        contour.kinds.reserve(segmentCount);

        contour.nodes.push_back(reversed.front().p0);

        for (std::size_t i = 0; i < reversed.size(); ++i)
        {
            contour.handles.push_back(reversed[i].c1);
            contour.handles.push_back(reversed[i].c2);
            contour.kinds.push_back(reversed[i].kind);

            // On a closed contour the final segment returns to the first node, which is
            // stored implicitly.
            const bool isLast = (i + 1 == reversed.size());
            if (!(isLast && closed))
                contour.nodes.push_back(reversed[i].p1);
        }

        contour.closed = closed;
    }

    Contour Reversed(const Contour& contour)
    {
        Contour copy = contour;
        Reverse(copy);
        return copy;
    }

    void Reverse(std::vector<Vec2>& ring)
    {
        std::reverse(ring.begin(), ring.end());
    }

    bool Normalize(Contour& contour, ContourRole role, const GeometryTolerance& tolerance)
    {
        const Orientation current = Metrics::OrientationOf(contour, tolerance);

        // A zero-area contour has no orientation to fix; reversing it would only churn
        // the data without changing anything meaningful.
        if (current == Orientation::Degenerate)
            return false;

        if (current == ExpectedOrientation(role))
            return false;

        Reverse(contour);
        return true;
    }

    bool Normalize(std::vector<Vec2>& ring, ContourRole role, const GeometryTolerance& tolerance)
    {
        const Orientation current = Metrics::OrientationOf(ring.data(), ring.size(), tolerance);

        if (current == Orientation::Degenerate)
            return false;

        if (current == ExpectedOrientation(role))
            return false;

        Reverse(ring);
        return true;
    }
}
