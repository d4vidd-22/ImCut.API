#include "PolygonMetrics.hpp"

#include "../Curves/CubicBezier.hpp"
#include "../Math/CertifiedInterval.hpp"
#include "../Math/Predicates.hpp"
#include "ConvexHull.hpp"

#include <cmath>

namespace ImCut::Geometry::Metrics
{
    namespace
    {
        // SECTION 15A, FOR THE FUNCTIONS THAT HAVE NO STATUS CHANNEL.
        //
        // Every Contour-taking entry in this file is public, noexcept, and returns a
        // plain double, a Bounds2, a Vec2 or an Orientation. There is nowhere to put
        // InvalidInput, so the precondition cannot be reported - it can only be OBEYED.
        // And it was not: all of them called SegmentAt() straight off the caller's
        // contour, which indexes three parallel arrays with no bounds check.
        //
        // A contour with four nodes, `closed = true` and empty handles/kinds reports
        // SegmentCount() == 4 and reads handles[0] of a zero-length vector. That is the
        // same defect that made Intersect::SelfIntersections take the process down with
        // 0xC0000005, and these functions had no guard at all.
        //
        // The answer for a garbage input is a NEUTRAL value, not a crash and not a
        // number: zero area, zero perimeter, an empty box, the origin, Degenerate. A
        // caller that wants to know WHY asks Validation::Validate, which is the entry
        // point that does have a channel for it and is required to survive this input.
        [[nodiscard]] bool Traversable(const Contour& contour) noexcept
        {
            return contour.IsStructurallyValid();
        }
    }

    double SignedArea(const Contour& contour) noexcept
    {
        if (!Traversable(contour))
            return 0.0;

        const std::size_t count = contour.SegmentCount();
        if (count == 0)
            return 0.0;

        // One origin for the whole contour, taken from the contour itself. Both this
        // sum and the polyline shoelace below used to be referenced to the absolute
        // origin, which cost log10(d^2/s^2) digits for a feature of side s at distance
        // d and inverted the sign outright past d^2/s^2 ~ 1e13. The loop integral is
        // translation invariant, so this is exact, and it costs one subtraction.
        //
        // It has to be the same origin for every segment: per-segment origins leave
        // residual terms that do not telescope. See CubicBezier.hpp.
        const Vec2 origin = contour.nodes.front();

        double total = 0.0;
        for (std::size_t i = 0; i < count; ++i)
            total += CubicBezier::SignedAreaContribution(contour.SegmentAt(i), origin);

        // An open contour has no enclosed area; reporting the integral over its
        // segments would silently invent one by implicitly closing it.
        return contour.closed ? total : 0.0;
    }

    double SignedArea(const Vec2* ring, std::size_t count) noexcept
    {
        if (ring == nullptr || count < 3)
            return 0.0;

        // Referenced to the ring's own first vertex, for the reason above.
        const Vec2 origin = ring[0];

        double total = 0.0;
        for (std::size_t i = 0; i < count; ++i)
        {
            const Vec2 a = ring[i] - origin;
            const Vec2 b = ring[i + 1 == count ? 0 : i + 1] - origin;
            total += a.x * b.y - b.x * a.y;
        }
        return total * 0.5;
    }

    double Perimeter(const Contour& contour, double tolerance) noexcept
    {
        if (!Traversable(contour))
            return 0.0;

        const std::size_t count = contour.SegmentCount();
        double total = 0.0;
        for (std::size_t i = 0; i < count; ++i)
        {
            const Segment segment = contour.SegmentAt(i);
            total += segment.IsLine() ? Distance(segment.p0, segment.p1)
                                      : CubicBezier::LengthApprox(segment, tolerance);
        }
        return total;
    }

    double Perimeter(const Vec2* ring, std::size_t count, bool closed) noexcept
    {
        if (ring == nullptr || count < 2)
            return 0.0;

        double total = 0.0;
        const std::size_t limit = closed ? count : count - 1;
        for (std::size_t i = 0; i < limit; ++i)
            total += Distance(ring[i], ring[i + 1 == count ? 0 : i + 1]);
        return total;
    }

    Bounds2 ComputeBounds(const Contour& contour) noexcept
    {
        Bounds2 bounds;
        if (!Traversable(contour))
            return bounds;

        const std::size_t count = contour.SegmentCount();
        if (count == 0)
        {
            for (const Vec2& node : contour.nodes)
                bounds.Add(node);
            return bounds;
        }

        // TIGHT, and only tight. This is a MEASUREMENT - "how far does this contour
        // reach" - not a proof object, and its name and documented meaning say so.
        //
        // The audit briefly made this return CertifiedExactBounds, to close the last
        // claim-bearing use of the short box. That was the wrong layer and the tests
        // said so immediately: widening every reported extent by ~1.4e-12 mm on a 100 mm
        // shape changed a ROUTE SELECTION - Nfp::InnerFit stopped recognising an exact
        // rectangle and silently dropped its exact closed form for the general Boolean
        // window, losing `exact` on all four corpus parts (29 failures).
        //
        // A measurement that has to be conservative is a different object from a
        // measurement that has to be tight, which is the whole argument BoundsEnclosure
        // and BoundsWitness are built on. So the enclosure lives in ComputeEnclosure
        // below, PreparedShapeDefinition takes ITS local enclosure from there, and this
        // one keeps reporting the tight box every route that needs a tight box relies on.
        for (std::size_t i = 0; i < count; ++i)
            bounds.Add(CubicBezier::ExactBounds(contour.SegmentAt(i)));

        return bounds;
    }

    Bounds2 ComputeBounds(const Path& path) noexcept
    {
        Bounds2 bounds;
        for (const Contour& contour : path.contours)
            bounds.Add(ComputeBounds(contour));
        return bounds;
    }

    // THE ENCLOSURE COUNTERPART. F35a.
    //
    // Same geometry, opposite obligation: this one has to CONTAIN the contour, and it is
    // returned as a BoundsEnclosure so a caller cannot silently use it where a witness is
    // required or vice versa.
    //
    // ComputeBounds above cannot serve here. `ExactBounds` evaluates the curve at the
    // extremum parameters it solved for in double, so the point it adds is on the curve
    // but is not the extremum, and the box falls short: 9807 of 59 359 measured true
    // extrema fall outside it, by up to 4 ULP of the coordinate (evidence 1354, and
    // reproduced directly by V8_1_3BoundsDirection, which also shows that 4 ULP EXCEEDS
    // tolerance.intersection at the documented 1e9 mm coordinate ceiling).
    //
    // PreparedShapeDefinition used to build its BoundsEnclosure from the tight box, so
    // `BoundsEnclosure::Enclosing()` - which the header calls "an ASSERTION by the
    // caller" and "the one place where the invariant enters" - was asserting something
    // false by a few ULP for every prepared shape with a curve in it. Everything
    // downstream that proves DISJOINT from that enclosure inherited the gap.
    BoundsEnclosure ComputeEnclosure(const Contour& contour) noexcept
    {
        Bounds2 bounds;
        if (!Traversable(contour))
            return BoundsEnclosure::Enclosing(bounds);

        const std::size_t count = contour.SegmentCount();
        if (count == 0)
        {
            for (const Vec2& node : contour.nodes)
                bounds.Add(node);
            return BoundsEnclosure::Enclosing(bounds);
        }

        for (std::size_t i = 0; i < count; ++i)
            bounds.Add(CubicBezier::CertifiedExactBounds(contour.SegmentAt(i)));

        return BoundsEnclosure::Enclosing(bounds);
    }

    BoundsEnclosure ComputeEnclosure(const Path& path) noexcept
    {
        Bounds2 bounds;
        for (const Contour& contour : path.contours)
            bounds.Add(ComputeEnclosure(contour).Box());
        return BoundsEnclosure::Enclosing(bounds);
    }

    namespace
    {
        // `a*x + c*y + tx`: two multiplications and two additions, so four roundings per
        // coordinate. Counted, not guessed - the same discipline the certified curve
        // bounds use, and the same primitive.
        constexpr int kTransformOps = 4;

        // Shrinks the box of computed points to a box the shape provably reaches.
        // Minima move UP and maxima move DOWN - the opposite of an enclosure, which is
        // why this cannot be the same type with a flag on it.
        [[nodiscard]] BoundsWitness Shrink(const Bounds2& sampled) noexcept
        {
            if (sampled.IsEmpty())
                return {};

            Bounds2 inward;
            inward.min = { Certified::WidenUp(sampled.min.x, kTransformOps),
                           Certified::WidenUp(sampled.min.y, kTransformOps) };
            inward.max = { Certified::WidenDown(sampled.max.x, kTransformOps),
                           Certified::WidenDown(sampled.max.y, kTransformOps) };

            // A shape thinner than its own rounding error witnesses nothing rather than
            // witnessing a box turned inside out.
            if (inward.min.x > inward.max.x || inward.min.y > inward.max.y)
                return {};
            return BoundsWitness::Reached(inward);
        }
    }

    BoundsWitness ComputeWitness(const Contour& contour, const Transform2& toWorld) noexcept
    {
        Bounds2 sampled;
        if (!Traversable(contour))
            return {};

        const std::size_t count = contour.SegmentCount();
        if (count == 0)
        {
            for (const Vec2& node : contour.nodes)
                sampled.Add(toWorld.Apply(node));
            return Shrink(sampled);
        }

        for (std::size_t i = 0; i < count; ++i)
        {
            const Segment segment = contour.SegmentAt(i);

            // Endpoints are on the shape by construction.
            sampled.Add(toWorld.Apply(segment.p0));
            sampled.Add(toWorld.Apply(segment.p1));

            // And so is the curve at each extremum parameter. `Extrema` solves a
            // quadratic in the LOCAL frame, so its roots are the extrema of the local
            // curve rather than of the transformed one - which costs tightness under
            // rotation and costs nothing else, because every point it names is still a
            // point of the curve.
            double t[4];
            const int found = CubicBezier::Extrema(segment, t);
            for (int k = 0; k < found; ++k)
                sampled.Add(toWorld.Apply(CubicBezier::Evaluate(segment, t[k])));
        }

        return Shrink(sampled);
    }

    BoundsWitness ComputeWitness(const Vec2* points, std::size_t count,
                                 const Transform2& toWorld) noexcept
    {
        Bounds2 sampled;
        for (std::size_t i = 0; i < count; ++i)
            sampled.Add(toWorld.Apply(points[i]));
        return Shrink(sampled);
    }

    void CollectWitnessPoints(const Path& path, std::vector<Vec2>& out)
    {
        for (const Contour& contour : path.contours)
        {
            const std::size_t count = contour.SegmentCount();
            if (count == 0)
            {
                out.insert(out.end(), contour.nodes.begin(), contour.nodes.end());
                continue;
            }

            for (std::size_t i = 0; i < count; ++i)
            {
                const Segment segment = contour.SegmentAt(i);
                out.push_back(segment.p0);
                out.push_back(segment.p1);

                double t[4];
                const int found = CubicBezier::Extrema(segment, t);
                for (int k = 0; k < found; ++k)
                    out.push_back(CubicBezier::Evaluate(segment, t[k]));
            }
        }
    }

    BoundsWitness ComputeWitness(const Path& path, const Transform2& toWorld) noexcept
    {
        Bounds2 sampled;
        for (const Contour& contour : path.contours)
        {
            const BoundsWitness one = ComputeWitness(contour, toWorld);
            if (!one.IsEmpty())
                sampled.Add(one.Box());
        }
        // Already shrunk per contour; the union of witness boxes of parts of the shape is
        // still inside the shape's extent, so no second shrink is owed.
        return sampled.IsEmpty() ? BoundsWitness{} : BoundsWitness::Reached(sampled);
    }

    Vec2 Centroid(const Contour& contour) noexcept
    {
        const std::size_t count = contour.SegmentCount();
        if (count == 0)
            return contour.nodes.empty() ? Vec2{} : contour.nodes.front();

        // Area-weighted centroid of the polygon through the contour's nodes. Curved
        // segments shift this slightly; callers needing the exact curved centroid
        // should flatten first and pass the ring.
        //
        // Accumulated in the contour's own frame and shifted back at the end, for the
        // reason given in SignedArea: referenced to the absolute origin, the centroid
        // of a 0.1 mm square at 1e8 mm came out 33 km away from the square.
        const std::size_t nodeCount = contour.nodes.size();
        const Vec2 origin = contour.nodes.front();

        double area = 0.0;
        double cx = 0.0;
        double cy = 0.0;

        for (std::size_t i = 0; i < nodeCount; ++i)
        {
            const Vec2 a = contour.nodes[i] - origin;
            const Vec2 b = contour.nodes[i + 1 == nodeCount ? 0 : i + 1] - origin;
            const double cross = a.x * b.y - b.x * a.y;
            area += cross;
            cx += (a.x + b.x) * cross;
            cy += (a.y + b.y) * cross;
        }

        if (area == 0.0)
            return ComputeBounds(contour).Center();

        const double scale = 1.0 / (3.0 * area);
        return { cx * scale + origin.x, cy * scale + origin.y };
    }

    Orientation OrientationOf(const Contour& contour, const GeometryTolerance& tolerance) noexcept
    {
        const double area = SignedArea(contour);
        const double threshold = tolerance.AreaThreshold(ComputeBounds(contour).Area());

        if (area > threshold) return Orientation::CounterClockwise;
        if (area < -threshold) return Orientation::Clockwise;
        return Orientation::Degenerate;
    }

    Orientation OrientationOf(const Vec2* ring, std::size_t count,
                              const GeometryTolerance& tolerance) noexcept
    {
        const double area = SignedArea(ring, count);

        Bounds2 bounds;
        for (std::size_t i = 0; i < count; ++i)
            bounds.Add(ring[i]);

        const double threshold = tolerance.AreaThreshold(bounds.Area());

        if (area > threshold) return Orientation::CounterClockwise;
        if (area < -threshold) return Orientation::Clockwise;
        return Orientation::Degenerate;
    }

    bool IsConvex(const Vec2* ring, std::size_t count, const GeometryTolerance& tolerance) noexcept
    {
        if (ring == nullptr || count < 3)
            return false;

        int sign = 0;
        for (std::size_t i = 0; i < count; ++i)
        {
            const Vec2 a = ring[i];
            const Vec2 b = ring[(i + 1) % count];
            const Vec2 c = ring[(i + 2) % count];

            const double turn = Predicates::Orient2D(a, b, c);

            // Collinear triples are allowed: a convex polygon may carry redundant
            // vertices along a straight edge.
            const Vec2 ab = b - a;
            const Vec2 bc = c - b;
            const double magnitude = std::sqrt(LengthSquared(ab) * LengthSquared(bc));
            if (magnitude <= 0.0 || std::fabs(turn) <= tolerance.collinearity * magnitude)
                continue;

            const int currentSign = turn > 0.0 ? 1 : -1;
            if (sign == 0)
                sign = currentSign;
            else if (sign != currentSign)
                return false;
        }

        return sign != 0;
    }

    bool IsStrictlyConvex(const Vec2* ring, std::size_t count) noexcept
    {
        if (ring == nullptr || count < 3)
            return false;

        int sign = 0;
        for (std::size_t i = 0; i < count; ++i)
        {
            const Orientation turn = Predicates::Orientation2D(
                ring[i], ring[(i + 1) % count], ring[(i + 2) % count]);
            if (turn == Orientation::Degenerate)
                return false;

            const int current = turn == Orientation::CounterClockwise ? 1 : -1;
            if (sign == 0)
                sign = current;
            else if (sign != current)
                return false;
        }
        return true;
    }

    double Convexity(const std::vector<Vec2>& ring)
    {
        if (ring.size() < 3)
            return 0.0;

        const double area = std::fabs(SignedArea(ring.data(), ring.size()));
        if (area <= 0.0)
            return 0.0;

        const double hullArea = std::fabs(Hull::Area(Hull::Compute(ring)));
        if (hullArea <= 0.0)
            return 0.0;

        return area / hullArea;
    }
}
