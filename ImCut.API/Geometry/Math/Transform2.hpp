#pragma once

#include "Bounds2.hpp"
#include "CertifiedInterval.hpp"
#include "Vec2.hpp"

#include <cmath>
#include <cstddef>

namespace ImCut::Geometry
{
    // 2D affine transform stored as the 2x3 matrix
    //     | a  c  tx |
    //     | b  d  ty |
    // acting on column vectors: x' = a*x + c*y + tx, y' = b*x + d*y + ty.
    //
    // Rotation angles are turned into cos/sin exactly once, inside Rotation() /
    // FromComponents(); no transform application ever calls a trig function. This
    // is what keeps batch rotation of a prepared piece cheap for the future
    // nesting workload, where one piece is rotated into many variants.
    struct Transform2
    {
        double a = 1.0, b = 0.0;
        double c = 0.0, d = 1.0;
        double tx = 0.0, ty = 0.0;

        constexpr Transform2() noexcept = default;
        constexpr Transform2(double ma, double mb, double mc, double md, double mtx, double mty) noexcept
            : a(ma), b(mb), c(mc), d(md), tx(mtx), ty(mty) {}

        [[nodiscard]] static constexpr Transform2 Identity() noexcept { return {}; }

        [[nodiscard]] static constexpr Transform2 Translation(Vec2 t) noexcept
        {
            return { 1.0, 0.0, 0.0, 1.0, t.x, t.y };
        }

        [[nodiscard]] static constexpr Transform2 Scaling(double sx, double sy) noexcept
        {
            return { sx, 0.0, 0.0, sy, 0.0, 0.0 };
        }

        [[nodiscard]] static constexpr Transform2 Scaling(double s) noexcept { return Scaling(s, s); }

        [[nodiscard]] static Transform2 Rotation(double radians) noexcept
        {
            const double cosine = std::cos(radians);
            const double sine = std::sin(radians);
            return { cosine, sine, -sine, cosine, 0.0, 0.0 };
        }

        [[nodiscard]] static Transform2 RotationAbout(double radians, Vec2 pivot) noexcept
        {
            const double cosine = std::cos(radians);
            const double sine = std::sin(radians);
            return { cosine, sine, -sine, cosine,
                     pivot.x - cosine * pivot.x + sine * pivot.y,
                     pivot.y - sine * pivot.x - cosine * pivot.y };
        }

        // Mirror about the line through the origin at the given angle.
        [[nodiscard]] static Transform2 Mirror(double radians) noexcept
        {
            const double cosine = std::cos(2.0 * radians);
            const double sine = std::sin(2.0 * radians);
            return { cosine, sine, sine, -cosine, 0.0, 0.0 };
        }

        [[nodiscard]] static constexpr Transform2 MirrorX() noexcept { return { -1.0, 0.0, 0.0, 1.0, 0.0, 0.0 }; }
        [[nodiscard]] static constexpr Transform2 MirrorY() noexcept { return { 1.0, 0.0, 0.0, -1.0, 0.0, 0.0 }; }

        // Single trig evaluation for a full translate/rotate/scale/mirror stack.
        [[nodiscard]] static Transform2 FromComponents(Vec2 translation, double radians,
                                                       double scaleX, double scaleY,
                                                       bool mirrorX = false) noexcept
        {
            const double cosine = std::cos(radians);
            const double sine = std::sin(radians);
            const double sx = mirrorX ? -scaleX : scaleX;
            return { cosine * sx, sine * sx, -sine * scaleY, cosine * scaleY, translation.x, translation.y };
        }

        [[nodiscard]] constexpr double Determinant() const noexcept { return a * d - b * c; }

        [[nodiscard]] constexpr bool IsIdentity() const noexcept
        {
            return a == 1.0 && b == 0.0 && c == 0.0 && d == 1.0 && tx == 0.0 && ty == 0.0;
        }

        // True when the transform flips orientation, which also flips polygon winding.
        [[nodiscard]] constexpr bool IsMirroring() const noexcept { return Determinant() < 0.0; }

        // Every coefficient, translation included.
        //
        // IsIsometry examines only a, b, c, d, so a transform with tx = +infinity is a
        // perfectly good rotation matrix by that test. One reached PrepareInstance,
        // produced infinite world bounds, and turned into Success(false) from Intersects
        // and Success(infinity) from MinimumDistance - mathematically invalid input
        // arriving as an apparently valid answer. Finiteness is therefore checked
        // separately, and before isometry.
        [[nodiscard]] bool IsFinite() const noexcept
        {
            return std::isfinite(a) && std::isfinite(b) && std::isfinite(c) &&
                   std::isfinite(d) && std::isfinite(tx) && std::isfinite(ty);
        }

        // True when the linear part preserves distance: rotation, reflection, identity,
        // and any composition of them, with or without translation.
        //
        // This is the precondition for measuring in a local frame and reporting the
        // answer in millimetres. The columns must be unit length and orthogonal; scale,
        // non-uniform scale and skew all fail one of the two and would make a local
        // distance a different number from the world distance it claims to be.
        //
        // Says nothing about finiteness: NaN coefficients fail the comparisons and an
        // infinite translation is invisible to it. Pair it with IsFinite().
        [[nodiscard]] bool IsIsometry(double epsilon = 1e-9) const noexcept
        {
            const double columnX = a * a + b * b;
            const double columnY = c * c + d * d;
            const double dot = a * c + b * d;

            return std::fabs(columnX - 1.0) <= epsilon &&
                   std::fabs(columnY - 1.0) <= epsilon &&
                   std::fabs(dot) <= epsilon;
        }

        [[nodiscard]] constexpr Vec2 Apply(Vec2 p) const noexcept
        {
            return { a * p.x + c * p.y + tx, b * p.x + d * p.y + ty };
        }

        // Direction vectors ignore translation.
        [[nodiscard]] constexpr Vec2 ApplyVector(Vec2 v) const noexcept
        {
            return { a * v.x + c * v.y, b * v.x + d * v.y };
        }

        void ApplyBatch(const Vec2* source, Vec2* destination, std::size_t count) const noexcept
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                const double x = source[i].x;
                const double y = source[i].y;
                destination[i].x = a * x + c * y + tx;
                destination[i].y = b * x + d * y + ty;
            }
        }

        // Lemma B1a of BOUNDS_EVIDENCE_PROOF.md: transforms the box CORNERS, so the
        // result is the AABB of the transformed AABB. That contains the transformed
        // shape, so it is an enclosure - and under rotation it is strictly larger than
        // the shape's true extent, by a factor that reaches 2 at 45 degrees (1270).
        //
        // The return type says so. There is deliberately no overload returning a bare
        // `Bounds2`: this operation cannot produce a witness (lemma B1b gives the
        // counterexample), and the previous signature let three call sites read it as if
        // it could - F26, F38, and the class census 1138 that declared them safe.
        // F35a, the remaining half. PreparedCollision.cpp used to say of this function
        // "`ApplyEnclosure` does not widen, and closing that is F35's job, not this one",
        // and worked around it with a margin in ONE of its nine consumers.
        //
        // The name is a claim: the transformed shape is inside the returned box. It was
        // not, quite. `Apply` is `a*x + c*y + tx` - two multiplications and two additions,
        // so four roundings per coordinate - and the hull of four ROUNDED corners can sit
        // inside the hull of the four exact ones. The consumers that only over-accept
        // paid nothing for that; `FillIsUniformWithin` reads `ProvesInside` and
        // `CouldTouch(..., 0.0)` with NO margin at all, and both of those are proofs.
        //
        // Widened by the counted-operations rule the witness side has used since F26:
        // Certified::Widen*(v, 4), where 4 is this function's own operation count, not a
        // constant fitted to an observed escape. Costs four fused terms per box.
        //
        // No longer constexpr - Certified::WidenUp is not - and nothing called it in a
        // constant expression.
        [[nodiscard]] BoundsEnclosure ApplyEnclosure(const Bounds2& box) const noexcept
        {
            if (box.IsEmpty()) return {};
            Bounds2 result;
            result.Add(Apply(box.min));
            result.Add(Apply({ box.max.x, box.min.y }));
            result.Add(Apply(box.max));
            result.Add(Apply({ box.min.x, box.max.y }));

            // ONE pad for all four sides, from the largest magnitude in the box.
            //
            // Certified::WidenDown(v, 4) is v - (|v| * 4u + a). Using the box's largest
            // |coordinate| in place of each |v| can only make the pad LARGER, so this
            // stays sound, and it costs one multiply and one add for the whole box
            // instead of four of each plus four fabs. This is a hot path - the dual
            // hierarchy traversal calls it per node pair - so the cheaper form is worth
            // the slightly looser box, which costs candidate pairs and never an answer.
            constexpr int kApplyOps = 4;
            double scale = std::fabs(result.min.x);
            const double my = std::fabs(result.min.y);
            const double Mx = std::fabs(result.max.x);
            const double My = std::fabs(result.max.y);
            if (my > scale) scale = my;
            if (Mx > scale) scale = Mx;
            if (My > scale) scale = My;

            const double pad =
                scale * (Certified::kRelativeWiden * static_cast<double>(kApplyOps)) +
                Certified::kAbsoluteWiden;

            result.min = { result.min.x - pad, result.min.y - pad };
            result.max = { result.max.x + pad, result.max.y + pad };
            return BoundsEnclosure::Enclosing(result);
        }

        // Returns the transform equivalent to applying `inner` first, then `outer`.
        [[nodiscard]] static constexpr Transform2 Compose(const Transform2& outer, const Transform2& inner) noexcept
        {
            return {
                outer.a * inner.a + outer.c * inner.b,
                outer.b * inner.a + outer.d * inner.b,
                outer.a * inner.c + outer.c * inner.d,
                outer.b * inner.c + outer.d * inner.d,
                outer.a * inner.tx + outer.c * inner.ty + outer.tx,
                outer.b * inner.tx + outer.d * inner.ty + outer.ty
            };
        }

        // Rejects singular AND numerically singular transforms.
        //
        // `determinant == 0` alone is not enough: a matrix whose determinant is merely
        // tiny relative to its own entries inverts to finite garbage, which is worse
        // than failing because the caller has no way to notice. The test is therefore
        // relative - a uniformly tiny scale has a tiny determinant and is perfectly
        // invertible, while an ill-conditioned one is not, and only the ratio tells them
        // apart.
        //
        // The threshold costs roughly 12 of the 16 available decimal digits before a
        // transform is refused.
        [[nodiscard]] bool TryInvert(Transform2& out) const noexcept
        {
            const double determinant = Determinant();
            if (!std::isfinite(determinant) || determinant == 0.0)
                return false;

            constexpr double kConditionFloor = 1e-12;
            const double magnitude = a * a + b * b + c * c + d * d;
            if (!std::isfinite(magnitude) || magnitude <= 0.0)
                return false;
            if (std::fabs(determinant) <= kConditionFloor * magnitude)
                return false;

            const double inverse = 1.0 / determinant;
            out.a =  d * inverse;
            out.b = -b * inverse;
            out.c = -c * inverse;
            out.d =  a * inverse;
            out.tx = (c * ty - d * tx) * inverse;
            out.ty = (b * tx - a * ty) * inverse;
            return true;
        }
    };

    [[nodiscard]] inline bool IsFinite(const Transform2& t) noexcept
    {
        return std::isfinite(t.a) && std::isfinite(t.b) && std::isfinite(t.c) &&
               std::isfinite(t.d) && std::isfinite(t.tx) && std::isfinite(t.ty);
    }
}
