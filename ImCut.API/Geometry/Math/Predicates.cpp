#include "Predicates.hpp"

#include <algorithm>

namespace ImCut::Geometry::Predicates
{
    namespace
    {
        // Error-free transformations after Shewchuk, "Adaptive Precision Floating-Point
        // Arithmetic and Fast Robust Geometric Predicates" (1997). Every routine below
        // depends on strict IEEE-754 double rounding with no reassociation and no
        // fused multiply-add contraction; MSVC /fp:precise (the default, and what
        // ImCut.API.vcxproj uses) provides exactly that.

        // 2^-53: half an ulp at 1.0.
        constexpr double kEpsilon = 1.1102230246251565e-16;

        // 2^27 + 1, the Dekker splitting constant for doubles.
        constexpr double kSplitter = 134217729.0;

        constexpr double kResultErrBound = (3.0 + 8.0 * kEpsilon) * kEpsilon;
        constexpr double kCcwErrBoundA   = (3.0 + 16.0 * kEpsilon) * kEpsilon;
        constexpr double kCcwErrBoundB   = (2.0 + 12.0 * kEpsilon) * kEpsilon;
        constexpr double kCcwErrBoundC   = (9.0 + 64.0 * kEpsilon) * kEpsilon * kEpsilon;

        // x = a + b exactly, with |a| >= |b| assumed; y carries the rounding error.
        inline void FastTwoSum(double a, double b, double& x, double& y) noexcept
        {
            x = a + b;
            const double bvirt = x - a;
            y = b - bvirt;
        }

        // x = a + b exactly, no magnitude assumption.
        inline void TwoSum(double a, double b, double& x, double& y) noexcept
        {
            x = a + b;
            const double bvirt = x - a;
            const double avirt = x - bvirt;
            const double bround = b - bvirt;
            const double around = a - avirt;
            y = around + bround;
        }

        // y is the exact error of the already-computed difference x = a - b.
        inline void TwoDiffTail(double a, double b, double x, double& y) noexcept
        {
            const double bvirt = a - x;
            const double avirt = x + bvirt;
            const double bround = bvirt - b;
            const double around = a - avirt;
            y = around + bround;
        }

        inline void Split(double a, double& ahi, double& alo) noexcept
        {
            const double c = kSplitter * a;
            const double abig = c - a;
            ahi = c - abig;
            alo = a - ahi;
        }

        // x = a * b rounded, y = the exact error, so x + y == a * b exactly.
        inline void TwoProduct(double a, double b, double& x, double& y) noexcept
        {
            x = a * b;
            double ahi, alo, bhi, blo;
            Split(a, ahi, alo);
            Split(b, bhi, blo);
            const double err1 = x - (ahi * bhi);
            const double err2 = err1 - (alo * bhi);
            const double err3 = err2 - (ahi * blo);
            y = (alo * blo) - err3;
        }

        inline void TwoOneDiff(double a1, double a0, double b,
                               double& x2, double& x1, double& x0) noexcept
        {
            double i;
            i = a0 - b;
            TwoDiffTail(a0, b, i, x0);
            TwoSum(a1, i, x2, x1);
        }

        // (a1,a0) - (b1,b0) as a four-component expansion.
        inline void TwoTwoDiff(double a1, double a0, double b1, double b0,
                               double& x3, double& x2, double& x1, double& x0) noexcept
        {
            double j, k;
            TwoOneDiff(a1, a0, b0, j, k, x0);
            TwoOneDiff(j, k, b1, x3, x2, x1);
        }

        [[nodiscard]] inline double Estimate(int length, const double* e) noexcept
        {
            double sum = e[0];
            for (int i = 1; i < length; ++i)
                sum += e[i];
            return sum;
        }

        // Merges two non-overlapping, increasing-magnitude expansions into one,
        // dropping zero components. `h` must hold at least elen + flen doubles.
        // Bounds are checked explicitly here; Shewchuk's original reads one past the
        // end of the input arrays, which is undefined behaviour in C++.
        [[nodiscard]] int FastExpansionSumZeroElim(int elen, const double* e,
                                                   int flen, const double* f,
                                                   double* h) noexcept
        {
            int eindex = 0;
            int findex = 0;
            int hindex = 0;

            double enow = e[0];
            double fnow = f[0];
            double q;

            if ((fnow > enow) == (fnow > -enow))
            {
                q = enow;
                ++eindex;
                enow = eindex < elen ? e[eindex] : 0.0;
            }
            else
            {
                q = fnow;
                ++findex;
                fnow = findex < flen ? f[findex] : 0.0;
            }

            double qnew, hh;

            if (eindex < elen && findex < flen)
            {
                if ((fnow > enow) == (fnow > -enow))
                {
                    FastTwoSum(enow, q, qnew, hh);
                    ++eindex;
                    enow = eindex < elen ? e[eindex] : 0.0;
                }
                else
                {
                    FastTwoSum(fnow, q, qnew, hh);
                    ++findex;
                    fnow = findex < flen ? f[findex] : 0.0;
                }
                q = qnew;
                if (hh != 0.0) h[hindex++] = hh;

                while (eindex < elen && findex < flen)
                {
                    if ((fnow > enow) == (fnow > -enow))
                    {
                        TwoSum(q, enow, qnew, hh);
                        ++eindex;
                        enow = eindex < elen ? e[eindex] : 0.0;
                    }
                    else
                    {
                        TwoSum(q, fnow, qnew, hh);
                        ++findex;
                        fnow = findex < flen ? f[findex] : 0.0;
                    }
                    q = qnew;
                    if (hh != 0.0) h[hindex++] = hh;
                }
            }

            while (eindex < elen)
            {
                TwoSum(q, enow, qnew, hh);
                ++eindex;
                enow = eindex < elen ? e[eindex] : 0.0;
                q = qnew;
                if (hh != 0.0) h[hindex++] = hh;
            }

            while (findex < flen)
            {
                TwoSum(q, fnow, qnew, hh);
                ++findex;
                fnow = findex < flen ? f[findex] : 0.0;
                q = qnew;
                if (hh != 0.0) h[hindex++] = hh;
            }

            if (q != 0.0 || hindex == 0)
                h[hindex++] = q;

            return hindex;
        }

        // Stage 2 and beyond: recompute the determinant to whatever precision the
        // input actually demands, stopping as soon as the sign is certain.
        [[nodiscard]] double Orient2DAdapt(Vec2 pa, Vec2 pb, Vec2 pc, double detsum) noexcept
        {
            const double acx = pa.x - pc.x;
            const double bcx = pb.x - pc.x;
            const double acy = pa.y - pc.y;
            const double bcy = pb.y - pc.y;

            double detleft, detlefttail, detright, detrighttail;
            TwoProduct(acx, bcy, detleft, detlefttail);
            TwoProduct(acy, bcx, detright, detrighttail);

            double b[4];
            TwoTwoDiff(detleft, detlefttail, detright, detrighttail, b[3], b[2], b[1], b[0]);

            double det = Estimate(4, b);
            double errbound = kCcwErrBoundB * detsum;
            if (det >= errbound || -det >= errbound)
                return det;

            double acxtail, bcxtail, acytail, bcytail;
            TwoDiffTail(pa.x, pc.x, acx, acxtail);
            TwoDiffTail(pb.x, pc.x, bcx, bcxtail);
            TwoDiffTail(pa.y, pc.y, acy, acytail);
            TwoDiffTail(pb.y, pc.y, bcy, bcytail);

            // The coordinate differences were exact, so the stage-1 expansion already
            // is the exact determinant.
            if (acxtail == 0.0 && acytail == 0.0 && bcxtail == 0.0 && bcytail == 0.0)
                return det;

            errbound = kCcwErrBoundC * detsum + kResultErrBound * std::fabs(det);
            det += (acx * bcytail + bcy * acxtail) - (acy * bcxtail + bcx * acytail);
            if (det >= errbound || -det >= errbound)
                return det;

            double s1, s0, t1, t0;
            double u[4];
            double c1[8], c2[12], d[16];

            TwoProduct(acxtail, bcy, s1, s0);
            TwoProduct(acytail, bcx, t1, t0);
            TwoTwoDiff(s1, s0, t1, t0, u[3], u[2], u[1], u[0]);
            const int c1length = FastExpansionSumZeroElim(4, b, 4, u, c1);

            TwoProduct(acx, bcytail, s1, s0);
            TwoProduct(acy, bcxtail, t1, t0);
            TwoTwoDiff(s1, s0, t1, t0, u[3], u[2], u[1], u[0]);
            const int c2length = FastExpansionSumZeroElim(c1length, c1, 4, u, c2);

            TwoProduct(acxtail, bcytail, s1, s0);
            TwoProduct(acytail, bcxtail, t1, t0);
            TwoTwoDiff(s1, s0, t1, t0, u[3], u[2], u[1], u[0]);
            const int dlength = FastExpansionSumZeroElim(c2length, c2, 4, u, d);

            // The most significant component carries the exact sign.
            return d[dlength - 1];
        }
    }

    double Orient2D(Vec2 a, Vec2 b, Vec2 c) noexcept
    {
        const double detleft = (a.x - c.x) * (b.y - c.y);
        const double detright = (a.y - c.y) * (b.x - c.x);
        const double det = detleft - detright;

        double detsum;
        if (detleft > 0.0)
        {
            if (detright <= 0.0) return det;
            detsum = detleft + detright;
        }
        else if (detleft < 0.0)
        {
            if (detright >= 0.0) return det;
            detsum = -detleft - detright;
        }
        else
        {
            return det;
        }

        // Opposite-sign products cancel, so only here can rounding change the sign.
        const double errbound = kCcwErrBoundA * detsum;
        if (det >= errbound || -det >= errbound)
            return det;

        return Orient2DAdapt(a, b, c, detsum);
    }

    Orientation Orientation2D(Vec2 a, Vec2 b, Vec2 c) noexcept
    {
        const double det = Orient2D(a, b, c);
        if (det > 0.0) return Orientation::CounterClockwise;
        if (det < 0.0) return Orientation::Clockwise;
        return Orientation::Degenerate;
    }

    bool Collinear(Vec2 a, Vec2 b, Vec2 c, const GeometryTolerance& tolerance) noexcept
    {
        // Compare against the normalised cross product so the test means "the angle
        // between the two edges is negligible" rather than "the raw determinant is
        // small", which would scale with segment length.
        const Vec2 ab = b - a;
        const Vec2 ac = c - a;
        const double lengths = std::sqrt(LengthSquared(ab) * LengthSquared(ac));
        if (lengths <= tolerance.zeroLength * tolerance.zeroLength)
            return true;

        const double det = Orient2D(a, b, c);
        return std::fabs(det) <= tolerance.collinearity * lengths;
    }

    double ProjectionParameter(Vec2 p, Vec2 a, Vec2 b) noexcept
    {
        const Vec2 ab = b - a;
        const double lengthSquared = LengthSquared(ab);
        if (lengthSquared <= 0.0)
            return 0.0;
        return std::clamp(Dot(p - a, ab) / lengthSquared, 0.0, 1.0);
    }

    bool PointOnSegment(Vec2 p, Vec2 a, Vec2 b, double tolerance) noexcept
    {
        const Vec2 ab = b - a;
        const double lengthSquared = LengthSquared(ab);
        if (lengthSquared <= 0.0)
            return DistanceSquared(p, a) <= tolerance * tolerance;

        const double t = std::clamp(Dot(p - a, ab) / lengthSquared, 0.0, 1.0);
        const Vec2 closest{ a.x + ab.x * t, a.y + ab.y * t };
        return DistanceSquared(p, closest) <= tolerance * tolerance;
    }

    PointClassification PointInRing(Vec2 point, const Vec2* ring, std::size_t count,
                                    double boundaryTolerance) noexcept
    {
        if (ring == nullptr || count < 3)
            return PointClassification::Outside;

        // Boundary first: a point on an edge has no meaningful crossing parity.
        for (std::size_t i = 0, previous = count - 1; i < count; previous = i++)
        {
            if (PointOnSegment(point, ring[previous], ring[i], boundaryTolerance))
                return PointClassification::Boundary;
        }

        bool inside = false;
        for (std::size_t i = 0, previous = count - 1; i < count; previous = i++)
        {
            const Vec2 a = ring[previous];
            const Vec2 b = ring[i];

            // Half-open rule on y: each edge covers [min, max), so a vertex exactly on
            // the ray is counted by exactly one of its two edges.
            if ((a.y > point.y) == (b.y > point.y))
                continue;

            // Sign of the orientation determinant replaces the divided x-intersection,
            // so no division and no rounding can flip the parity.
            const double side = Orient2D(a, b, point);
            if (side == 0.0)
                return PointClassification::Boundary;

            // Crossing counts when the point lies left of an upward edge or right of
            // a downward edge.
            if ((b.y > a.y) == (side > 0.0))
                inside = !inside;
        }

        return inside ? PointClassification::Inside : PointClassification::Outside;
    }

    int WindingNumber(Vec2 point, const Vec2* ring, std::size_t count,
                      double boundaryTolerance, bool* onBoundary) noexcept
    {
        if (onBoundary != nullptr)
            *onBoundary = false;

        if (ring == nullptr || count < 3)
            return 0;

        for (std::size_t i = 0, previous = count - 1; i < count; previous = i++)
        {
            if (PointOnSegment(point, ring[previous], ring[i], boundaryTolerance))
            {
                if (onBoundary != nullptr)
                    *onBoundary = true;
                return 0;
            }
        }

        int winding = 0;
        for (std::size_t i = 0, previous = count - 1; i < count; previous = i++)
        {
            const Vec2 a = ring[previous];
            const Vec2 b = ring[i];

            if (a.y <= point.y)
            {
                if (b.y > point.y && Orient2D(a, b, point) > 0.0)
                    ++winding;
            }
            else if (b.y <= point.y && Orient2D(a, b, point) < 0.0)
            {
                --winding;
            }
        }

        return winding;
    }
}
