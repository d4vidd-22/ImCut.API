#include "PolygonConversion.hpp"

#include "../Curves/Flatten.hpp"
#include "../Topology/Winding.hpp"
#include "PolygonMetrics.hpp"

#include <cmath>

namespace ImCut::Geometry::Convert
{
    namespace
    {
        void AppendRing(const FlattenedContour& contour, const LatticeFrame& frame,
                        Backend::IntRings& out)
        {
            if (contour.points.size() < 3)
                return;

            Backend::IntRing ring;
            ring.reserve(contour.points.size());

            for (const Vec2& point : contour.points)
                ring.push_back(frame.ToInteger(point));

            out.push_back(std::move(ring));
        }
    }

    Backend::IntRingsView ConversionResult::RingsOf(std::size_t pathIndex) const noexcept
    {
        if (pathIndex + 1 >= offsets.size())
            return {};

        const std::size_t begin = offsets[pathIndex];
        const std::size_t count = offsets[pathIndex + 1] - begin;
        if (count == 0 && rings.empty())
            return {};

        return Backend::IntRingsView{
            rings.data() + begin, count };
    }

    GeometryResult<ConversionResult> PathsToIntegers(const Path* const* paths, std::size_t pathCount,
                                                     const GeometryContext& context,
                                                     const Bounds2* extentHint)
    {
        ConversionResult result;

        if (paths == nullptr || pathCount == 0)
            return GeometryResult<ConversionResult>::Empty(std::move(result));

        for (std::size_t i = 0; i < pathCount; ++i)
        {
            if (paths[i] == nullptr)
                continue;
            result.bounds.Add(Metrics::ComputeBounds(*paths[i]));
        }

        // The caller may know the result spans more than the input does.
        if (extentHint != nullptr && !extentHint->IsEmpty())
            result.bounds.Add(*extentHint);

        if (!IsFinite(result.bounds))
            return GeometryResult<ConversionResult>::Failure(GeometryStatus::InvalidInput);

        // One lattice shared by every operand: separate scales would make the integer
        // coordinates incomparable and silently misplace the result.
        //
        // Chosen in the operands' OWN frame. ChooseScale coarsens the step until the
        // largest absolute coordinate fits int64, so quantising in world coordinates
        // made the step a function of where the artwork sits rather than how big it is:
        // the same geometry got 1e-6 mm at the origin and 1 mm at 1e9. Centring first
        // means a 100 mm part is quantised at the step a 100 mm part deserves, wherever
        // it happens to be.
        result.frame.origin = result.bounds.IsEmpty() ? Vec2{} : result.bounds.Center();

        Bounds2 local;
        if (!result.bounds.IsEmpty())
        {
            local.Add(result.bounds.min - result.frame.origin);
            local.Add(result.bounds.max - result.frame.origin);
        }

        result.frame.scale = Quantization::ChooseScale(local, context.Tolerance());
        result.scale = result.frame.scale;
        if (!result.frame.scale.valid)
            return GeometryResult<ConversionResult>::Failure(GeometryStatus::Degenerate);

        // Input-size guards before any ring is built.
        {
            std::size_t inputSegments = 0;
            std::size_t inputContours = 0;
            for (std::size_t i = 0; i < pathCount; ++i)
            {
                if (paths[i] == nullptr) continue;
                inputContours += paths[i]->contours.size();
                inputSegments += paths[i]->SegmentCount();
            }
            if (inputContours > context.Limits().maxContours ||
                inputSegments > context.Limits().maxSegments)
            {
                return GeometryResult<ConversionResult>::Failure(GeometryStatus::ComplexityLimit);
            }
        }

        FlattenOptions options;
        options.tolerance = context.Tolerance().flatten;
        options.maxDepth = context.Limits().maxSubdivisionDepth;

        FlattenedContour scratch;
        std::size_t vertices = 0;

        result.offsets.reserve(pathCount + 1);

        double worstTolerance = options.tolerance;

        for (std::size_t i = 0; i < pathCount; ++i)
        {
            result.offsets.push_back(result.rings.size());

            if (paths[i] == nullptr)
                continue;

            for (const Contour& contour : paths[i]->contours)
            {
                if (context.ShouldCheckCancellation(vertices) && context.IsCancelled())
                    return GeometryResult<ConversionResult>::Failure(GeometryStatus::Cancelled);

                // Budget checked DURING emission, not after. The previous form
                // flattened the whole contour first and only then compared the total,
                // so a single pathological curve allocated everything it wanted before
                // the limit was consulted.
                const std::size_t vertexBudget = context.Limits().maxPolygonVertices;
                const std::size_t remainingVertices =
                    vertices >= vertexBudget ? 0 : vertexBudget - vertices;

                GeometryContext scoped = context;
                ComplexityLimits scopedLimits = context.Limits();
                scopedLimits.maxFlattenPoints =
                    (std::min)(scopedLimits.maxFlattenPoints, remainingVertices);
                scoped.SetLimits(scopedLimits);

                const GeometryStatus status = Flatten::ContourInto(contour, options, scoped, scratch);
                if (!IsSuccess(status))
                    return GeometryResult<ConversionResult>::Failure(status);

                vertices += scratch.points.size();
                worstTolerance = (std::max)(worstTolerance, scratch.toleranceUsed);

                if (vertices > vertexBudget)
                    return GeometryResult<ConversionResult>::Failure(GeometryStatus::ComplexityLimit);

                AppendRing(scratch, result.frame, result.rings);
            }
        }

        result.offsets.push_back(result.rings.size());

        // Both stages are lossy and both must be declared, so no caller can claim the
        // lattice resolution as the accuracy of the whole pipeline.
        //
        // The flatten term is the tolerance ACHIEVED, not the one requested. When the
        // subdivision depth cap fires the flattener stops short of the request and says
        // so through FlattenedContour::toleranceUsed; reporting options.tolerance
        // instead declared 0.01 mm for a run the audit measured at 891.78 mm.
        result.budget.AddFlatten(worstTolerance);
        result.budget.AddQuantization(result.frame.scale.MaxDisplacement());

        if (result.rings.empty())
            return GeometryResult<ConversionResult>::Empty(std::move(result));

        return GeometryResult<ConversionResult>::Success(std::move(result));
    }

    GeometryResult<ConversionResult> PathToIntegers(const Path& path, const GeometryContext& context)
    {
        const Path* pointer = &path;
        return PathsToIntegers(&pointer, 1, context);
    }

    namespace
    {
        Contour RingToContour(const Backend::IntRing& ring, const LatticeFrame& frame)
        {
            Contour contour;
            if (ring.size() < 3)
                return contour;

            contour.Reserve(ring.size());
            contour.MoveTo(frame.ToMillimetres(ring[0].x, ring[0].y));
            for (std::size_t i = 1; i < ring.size(); ++i)
                contour.LineTo(frame.ToMillimetres(ring[i].x, ring[i].y));

            contour.Close(0.0);
            return contour;
        }
    }

    Path PolyTreeToPath(const Backend::PolyTree& tree, const LatticeFrame& frame,
                        FillRule fillRule, const GeometryContext& context)
    {
        Path path;
        path.fillRule = fillRule;
        path.contours.reserve(tree.nodes.size());

        // The session's tolerance, not a hardcoded production default: winding
        // normalisation compares against an area threshold, and a caller running under
        // Fast or a custom tolerance must get that tolerance applied here too.
        const GeometryTolerance& tolerance = context.Tolerance();

        for (const Backend::PolyNode& node : tree.nodes)
        {
            Contour contour = RingToContour(node.ring, frame);
            if (contour.SegmentCount() < 3)
                continue;

            // Depth parity from the backend's own hierarchy decides the role; the
            // direction it happened to emit is then rewritten to match.
            Winding::Normalize(contour, node.hole ? ContourRole::Hole : ContourRole::Outer, tolerance);
            path.contours.push_back(std::move(contour));
        }

        return path;
    }

    Path RingsToPath(const Backend::IntRings& rings, const LatticeFrame& frame, FillRule fillRule)
    {
        Path path;
        path.fillRule = fillRule;
        path.contours.reserve(rings.size());

        for (const Backend::IntRing& ring : rings)
        {
            Contour contour = RingToContour(ring, frame);
            if (contour.SegmentCount() >= 3)
                path.contours.push_back(std::move(contour));
        }

        return path;
    }

    ResultValidation ValidateAndPrune(Path& path, const GeometryContext& context)
    {
        ResultValidation validation;

        const GeometryTolerance& tolerance = context.Tolerance();

        std::vector<Contour> kept;
        kept.reserve(path.contours.size());

        for (Contour& contour : path.contours)
        {
            if (contour.SegmentCount() < 3)
            {
                ++validation.emptyRings;
                continue;
            }

            bool finite = true;
            for (const Vec2& node : contour.nodes)
            {
                if (!IsFinite(node)) { finite = false; break; }
            }

            if (!finite)
            {
                validation.nonFinite = true;
                continue;
            }

            const Bounds2 bounds = Metrics::ComputeBounds(contour);
            const double area = std::fabs(Metrics::SignedArea(contour));

            // Slivers and collapsed rings are a normal by-product of clipping and
            // offsetting; they are reported and dropped, never silently kept.
            if (area <= tolerance.AreaThreshold(bounds.Area()))
            {
                ++validation.degenerateRings;
                continue;
            }

            kept.push_back(std::move(contour));
        }

        path.contours = std::move(kept);
        return validation;
    }
}
