#include "Offset.hpp"

#include "../Intersection/SelfIntersection.hpp"
#include "PolygonBackend.hpp"
#include "PolygonConversion.hpp"
#include "PolygonMetrics.hpp"

#include <algorithm>
#include <cmath>

namespace ImCut::Geometry::Offset
{
    namespace
    {
        [[nodiscard]] Backend::Join ToBackend(JoinType join) noexcept
        {
            switch (join)
            {
                case JoinType::Miter:  return Backend::Join::Miter;
                case JoinType::Square: return Backend::Join::Square;
                case JoinType::Bevel:  return Backend::Join::Bevel;
                case JoinType::Round:
                default:               return Backend::Join::Round;
            }
        }

        [[nodiscard]] Backend::Cap ToBackend(EndType end) noexcept
        {
            switch (end)
            {
                case EndType::ClosedJoined: return Backend::Cap::ClosedJoined;
                case EndType::OpenButt:     return Backend::Cap::OpenButt;
                case EndType::OpenSquare:   return Backend::Cap::OpenSquare;
                case EndType::OpenRound:    return Backend::Cap::OpenRound;
                case EndType::ClosedPolygon:
                default:                    return Backend::Cap::ClosedPolygon;
            }
        }
    }

    namespace
    {
        [[nodiscard]] GeometryResult<Path> RunOffset(const Path& path, double distance,
                                                     const GeometryContext& context,
                                                     const OffsetOptions& options,
                                                     OffsetReport* report, bool allowOpen);
    }

    GeometryResult<Path> Execute(const Path& path, double distance, const GeometryContext& context,
                                 const OffsetOptions& options, OffsetReport* report)
    {
        // Region offset: closed contours only.
        if (path.HasOpenContours())
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidTopology);

        // And a closed END TYPE only.
        //
        // The comment that used to sit further down claimed Execute "constrains the end
        // type to a polygon cap". It did not constrain anything: an EndType::OpenButt
        // reached the backend as the cap for a closed ring, which then traced the ring
        // as a polyline and returned a band. Measured on a 10x10 square inflated by 1:
        // area 59.56 against the analytic 143.14 - not an approximation, a different
        // shape. Silently substituting a closed cap would be no better, because the
        // caller asked for something this operation does not do.
        if (options.end != EndType::ClosedPolygon && options.end != EndType::ClosedJoined)
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidInput);

        return RunOffset(path, distance, context, options, report, false);
    }

    GeometryResult<Path> StrokeOpenPath(const Path& path, double halfWidth,
                                        const GeometryContext& context,
                                        const OffsetOptions& options, OffsetReport* report)
    {
        if (!(halfWidth > 0.0) || !std::isfinite(halfWidth))
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidInput);

        OffsetOptions stroke = options;
        // Stroking sweeps the outline; a polygon end type would fill it instead.
        //
        // This only sets the cap used for OPEN contours. Closed contours are stroked as
        // closed bands and take their own end type - substituting an open cap for all of
        // them is what made a closed ring and an open polyline with the same nodes
        // produce byte-identical output.
        if (stroke.end == EndType::ClosedPolygon)
            stroke.end = EndType::OpenRound;

        return RunOffset(path, halfWidth, context, stroke, report, true);
    }

    namespace
    {
    GeometryResult<Path> RunOffset(const Path& path, double distance, const GeometryContext& context,
                                   const OffsetOptions& options, OffsetReport* report, bool allowOpen)
    {
        if (!std::isfinite(distance))
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidInput);

        if (!path.IsStructurallyValid())
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidInput);

        // F45. AN EARLY RETURN STILL OWES THE CALLER THE OPERAND'S OWN ERROR.
        //
        // Boolean.cpp fixed exactly this for itself in V8.1.1 - see EmptyWithBudget and
        // the comment above it, "V8.1 returned a default Path here, so Simplify() of a
        // lossy-but-empty path came back declaring exact and zero" - and the fix never
        // reached its sibling. Offset had three early returns that dropped the input's
        // declared error on the floor: an empty operand, a zero distance, and an empty
        // conversion. `Boolean::Difference(A, A)` returns Empty carrying a real budget,
        // and feeding that straight into Offset erased it.
        //
        // Both channels are written, because both are read: GeometryResult::Budget() is
        // what the caller inspects and Path::budget is what travels on into the next
        // operation. The success route at the bottom of this function already sets both.
        if (path.contours.empty())
        {
            Path passthrough;
            passthrough.fillRule = path.fillRule;
            passthrough.budget = path.budget;
            auto empty = GeometryResult<Path>::Empty(std::move(passthrough));
            empty.Budget().MergeSequential(path.budget);
            return empty;
        }

        // Zero offset is the identity on the region. Round-tripping it through the
        // backend would only add flattening and lattice error for no benefit - but the
        // identity of a lossy region is still lossy, and the wrapper has to say so.
        if (distance == 0.0)
        {
            auto identity = GeometryResult<Path>::Success(path);
            identity.Budget().MergeSequential(path.budget);
            if (report != nullptr)
            {
                report->inputRings = path.contours.size();
                report->outputRings = path.contours.size();
            }
            return identity;
        }

        // Closed and open contours are different geometry and take different end
        // types, so they are separated BEFORE conversion and offset in their own runs.
        //
        // The backend takes one cap for the whole ring set. Handing it a single end type
        // meant FlattenedContour::closed never reached the decision: a closed ring was
        // stroked as though it were an open polyline, and the audit measured a closed
        // and an open contour with identical nodes producing the same area, 62.6818.
        Path closedPart;
        Path openPart;
        closedPart.fillRule = path.fillRule;
        openPart.fillRule = path.fillRule;

        for (const Contour& contour : path.contours)
        {
            if (contour.closed)
                closedPart.contours.push_back(contour);
            else
                openPart.contours.push_back(contour);
        }

        const bool haveClosed = !closedPart.contours.empty();
        const bool haveOpen = !openPart.contours.empty();

        // Region offset already refuses open contours upstream, so this only ever sees
        // the stroke case; the assertion of that contract lives in Execute().
        if (!allowOpen && haveOpen)
            return GeometryResult<Path>::Failure(GeometryStatus::InvalidTopology);

        // One conversion for both groups keeps them on a single lattice, which is what
        // makes the union of the two offset results meaningful.
        //
        // The lattice is chosen for the INFLATED extent, not the input extent. The
        // kernel's whole integer-range argument is that coordinates stay under 1e9 so
        // cross products of differences stay inside int64; the offset distance was never
        // part of that budget, so a delta far larger than the input pushed the result
        // out of range. Measured: distance/extent around 100 already breaches the cap,
        // and 1e12 mm of offset produced coordinates of 1e18 - nine orders past the
        // documented ceiling, reported as Success. Expanding the extent first makes the
        // result fit by construction rather than by luck.
        const Bounds2 inflated =
            Metrics::ComputeBounds(path).Expanded(std::fabs(distance));

        const Path* operands[2] = { &closedPart, &openPart };
        auto conversion = Convert::PathsToIntegers(operands, 2, context, &inflated);
        if (conversion.Status() == GeometryStatus::Empty)
        {
            Path passthrough;
            passthrough.fillRule = path.fillRule;
            passthrough.budget = path.budget;
            auto empty = GeometryResult<Path>::Empty(std::move(passthrough));
            empty.Budget().MergeSequential(path.budget);
            return empty;
        }
        if (!conversion.Ok())
            return GeometryResult<Path>::Failure(conversion.Status());

        const Convert::ConversionResult& converted = conversion.Value();

        Backend::OffsetRequest request;
        request.delta = distance * converted.scale.scale;
        request.join = ToBackend(options.join);
        request.miterLimit = options.miterLimit;

        // Round joins are approximated by arcs; without an explicit tolerance, tie it
        // to the offset distance so the arc error stays proportionate rather than
        // absolute, and never finer than the lattice can express.
        const double arcTolerance = options.arcTolerance > 0.0
            ? options.arcTolerance
            : std::max(std::fabs(distance) * 0.005, context.Tolerance().flatten * 0.5);

        request.arcTolerance = arcTolerance * converted.scale.scale;

        // A closed contour being STROKED is a closed band, not a filled region and not
        // an open polyline, so it takes ClosedJoined. A closed contour being OFFSET as a
        // region keeps the caller's end type - and Execute really does constrain that to
        // a closed cap now, which is what this sentence used to claim without it being
        // true anywhere in the code.
        const EndType closedEnd = allowOpen ? EndType::ClosedJoined : options.end;

        Backend::PolyTree tree;
        const Backend::TreeLimits treeLimits{ context.Limits().maxNestingDepth };

        // A backend outcome becomes a kernel status in one place, so a caller cannot
        // collapse DepthLimit into NumericalFailure by testing only for "not success".
        const auto toStatus = [](Backend::BackendStatus status) noexcept
        {
            switch (status)
            {
                case Backend::BackendStatus::Success:     return GeometryStatus::Success;
                case Backend::BackendStatus::DepthLimit:  return GeometryStatus::ComplexityLimit;
                case Backend::BackendStatus::OutOfMemory: return GeometryStatus::OutOfMemory;
                case Backend::BackendStatus::Failed:
                default:                                  return GeometryStatus::NumericalFailure;
            }
        };

        if (haveClosed && haveOpen)
        {
            Backend::OffsetRequest closedRequest = request;
            closedRequest.cap = ToBackend(closedEnd);

            Backend::OffsetRequest openRequest = request;
            openRequest.cap = ToBackend(options.end);

            Backend::PolyTree closedTree;
            Backend::PolyTree openTree;

            if (const auto status = Backend::Default().Offset(converted.RingsOf(0), closedRequest,
                                                              treeLimits, closedTree);
                status != Backend::BackendStatus::Success)
            {
                return GeometryResult<Path>::Failure(toStatus(status));
            }
            if (const auto status = Backend::Default().Offset(converted.RingsOf(1), openRequest,
                                                              treeLimits, openTree);
                status != Backend::BackendStatus::Success)
            {
                return GeometryResult<Path>::Failure(toStatus(status));
            }

            // Both groups are already offset outlines on one lattice; unioning them
            // under NonZero merges overlaps without reinterpreting either group.
            Backend::IntRings merged;
            merged.reserve(closedTree.nodes.size() + openTree.nodes.size());
            for (const Backend::PolyNode& node : closedTree.nodes) merged.push_back(node.ring);
            for (const Backend::PolyNode& node : openTree.nodes) merged.push_back(node.ring);

            if (const auto status =
                    Backend::Default().Execute(Backend::Operation::Union, Backend::Fill::NonZero,
                                               merged, Backend::IntRingsView{}, treeLimits, tree);
                status != Backend::BackendStatus::Success)
            {
                return GeometryResult<Path>::Failure(toStatus(status));
            }
        }
        else
        {
            request.cap = ToBackend(haveClosed ? closedEnd : options.end);
            const Backend::IntRingsView rings =
                haveClosed ? converted.RingsOf(0) : converted.RingsOf(1);
            if (const auto status = Backend::Default().Offset(rings, request, treeLimits, tree);
                status != Backend::BackendStatus::Success)
            {
                return GeometryResult<Path>::Failure(toStatus(status));
            }
        }

        Path output = Convert::PolyTreeToPath(tree, converted.frame, path.fillRule, context);

        Convert::ResultValidation validation;
        if (options.pruneDegenerate)
            validation = Convert::ValidateAndPrune(output, context);

        // Offsetting can fold a concave region onto itself. Report it as a finding
        // rather than pretending the result is clean.
        bool selfIntersecting = false;
        if (options.validateSelfIntersection)
        {
            for (const Contour& contour : output.contours)
            {
                // Read the answer, not the status. GeometryResult converts to bool as
                // "did the query complete", so testing the result directly would report
                // every successful check as a self-intersection.
                const auto check = Intersect::HasSelfIntersection(contour, context);
                if (!check.Ok())
                {
                    // The check did not finish, so the result is unverified. Failing the
                    // whole offset is better than shipping a silent "clean" claim.
                    return GeometryResult<Path>::Failure(check.Status());
                }
                if (check.Value())
                {
                    selfIntersecting = true;
                    break;
                }
            }
        }

        if (report != nullptr)
        {
            report->inputRings = converted.rings.size();
            report->outputRings = output.contours.size();
            report->prunedDegenerate = validation.degenerateRings;
            report->nonFiniteRemoved = validation.nonFinite;
            report->selfIntersecting = selfIntersecting;
            report->quantizationStep = converted.scale.resolution;
            report->arcToleranceUsed = arcTolerance;
        }

        CountStat(context, &GeometryStatistics::offsetOperations);

        // Whatever the input already carried, plus this stage's own flattening,
        // quantisation and (for round joins) arc approximation.
        ErrorBudget budget = path.budget;
        budget.MergeSequential(converted.budget);
        if (options.join == JoinType::Round)
            budget.AddSequentialFlatten(arcTolerance);
        output.budget = budget;

        if (output.contours.empty())
        {
            // Deflating past the region's own width legitimately erases it.
            auto empty = GeometryResult<Path>::Empty(std::move(output));
            empty.Budget().MergeSequential(budget);
            return empty;
        }

        auto success = GeometryResult<Path>::Success(std::move(output));
        success.Budget().MergeSequential(budget);
        return success;
    }
    }

    GeometryResult<Path> Inflate(const Path& path, double distance, const GeometryContext& context,
                                 const OffsetOptions& options)
    {
        return Execute(path, distance, context, options, nullptr);
    }
}
