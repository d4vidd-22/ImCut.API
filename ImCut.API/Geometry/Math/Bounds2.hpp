#pragma once

#include "Vec2.hpp"

#include <limits>

namespace ImCut::Geometry
{
    // Axis-aligned bounds in millimetres. Empty is encoded as min > max, so that
    // Add() of an empty box is the identity and no separate validity flag is needed.
    struct Bounds2
    {
        Vec2 min{  std::numeric_limits<double>::infinity(),  std::numeric_limits<double>::infinity() };
        Vec2 max{ -std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity() };

        constexpr Bounds2() noexcept = default;
        constexpr Bounds2(Vec2 lo, Vec2 hi) noexcept : min(lo), max(hi) {}

        [[nodiscard]] constexpr bool IsEmpty() const noexcept { return min.x > max.x || min.y > max.y; }

        [[nodiscard]] constexpr double Width() const noexcept { return IsEmpty() ? 0.0 : max.x - min.x; }
        [[nodiscard]] constexpr double Height() const noexcept { return IsEmpty() ? 0.0 : max.y - min.y; }
        [[nodiscard]] constexpr double Area() const noexcept { return Width() * Height(); }
        [[nodiscard]] constexpr double Perimeter() const noexcept { return 2.0 * (Width() + Height()); }

        [[nodiscard]] constexpr Vec2 Center() const noexcept
        {
            return { (min.x + max.x) * 0.5, (min.y + max.y) * 0.5 };
        }

        [[nodiscard]] constexpr Vec2 Extent() const noexcept { return { Width(), Height() }; }

        [[nodiscard]] constexpr double LongestAxisLength() const noexcept
        {
            const double w = Width();
            const double h = Height();
            return w > h ? w : h;
        }

        // 0 = x, 1 = y. Ties resolve to x so BVH splits stay deterministic.
        [[nodiscard]] constexpr int LongestAxis() const noexcept { return Height() > Width() ? 1 : 0; }

        constexpr void Add(Vec2 p) noexcept
        {
            if (p.x < min.x) min.x = p.x;
            if (p.y < min.y) min.y = p.y;
            if (p.x > max.x) max.x = p.x;
            if (p.y > max.y) max.y = p.y;
        }

        constexpr void Add(const Bounds2& b) noexcept
        {
            if (b.IsEmpty()) return;
            if (b.min.x < min.x) min.x = b.min.x;
            if (b.min.y < min.y) min.y = b.min.y;
            if (b.max.x > max.x) max.x = b.max.x;
            if (b.max.y > max.y) max.y = b.max.y;
        }

        [[nodiscard]] constexpr bool Contains(Vec2 p) const noexcept
        {
            return p.x >= min.x && p.x <= max.x && p.y >= min.y && p.y <= max.y;
        }

        [[nodiscard]] constexpr bool Contains(Vec2 p, double tolerance) const noexcept
        {
            return p.x >= min.x - tolerance && p.x <= max.x + tolerance &&
                   p.y >= min.y - tolerance && p.y <= max.y + tolerance;
        }

        [[nodiscard]] constexpr bool Contains(const Bounds2& b) const noexcept
        {
            if (b.IsEmpty()) return true;
            if (IsEmpty()) return false;
            return b.min.x >= min.x && b.max.x <= max.x && b.min.y >= min.y && b.max.y <= max.y;
        }

        [[nodiscard]] constexpr bool Overlaps(const Bounds2& b) const noexcept
        {
            return !(IsEmpty() || b.IsEmpty() ||
                     b.min.x > max.x || b.max.x < min.x ||
                     b.min.y > max.y || b.max.y < min.y);
        }

        [[nodiscard]] constexpr bool Overlaps(const Bounds2& b, double tolerance) const noexcept
        {
            return !(IsEmpty() || b.IsEmpty() ||
                     b.min.x > max.x + tolerance || b.max.x < min.x - tolerance ||
                     b.min.y > max.y + tolerance || b.max.y < min.y - tolerance);
        }

        [[nodiscard]] constexpr Bounds2 Expanded(double amount) const noexcept
        {
            if (IsEmpty()) return {};
            return { { min.x - amount, min.y - amount }, { max.x + amount, max.y + amount } };
        }

        [[nodiscard]] constexpr Bounds2 Translated(Vec2 delta) const noexcept
        {
            if (IsEmpty()) return {};
            return { min + delta, max + delta };
        }

        // Squared distance from p to the box; 0 when inside. Keeps sqrt off the
        // BVH nearest-candidate path.
        [[nodiscard]] constexpr double DistanceSquared(Vec2 p) const noexcept
        {
            if (IsEmpty()) return std::numeric_limits<double>::infinity();
            const double dx = p.x < min.x ? min.x - p.x : (p.x > max.x ? p.x - max.x : 0.0);
            const double dy = p.y < min.y ? min.y - p.y : (p.y > max.y ? p.y - max.y : 0.0);
            return dx * dx + dy * dy;
        }

        [[nodiscard]] constexpr double DistanceSquared(const Bounds2& b) const noexcept
        {
            if (IsEmpty() || b.IsEmpty()) return std::numeric_limits<double>::infinity();
            const double dx = b.min.x > max.x ? b.min.x - max.x : (min.x > b.max.x ? min.x - b.max.x : 0.0);
            const double dy = b.min.y > max.y ? b.min.y - max.y : (min.y > b.max.y ? min.y - b.max.y : 0.0);
            return dx * dx + dy * dy;
        }

        [[nodiscard]] static constexpr Bounds2 Union(const Bounds2& a, const Bounds2& b) noexcept
        {
            Bounds2 r = a;
            r.Add(b);
            return r;
        }

        [[nodiscard]] static constexpr Bounds2 Intersection(const Bounds2& a, const Bounds2& b) noexcept
        {
            if (a.IsEmpty() || b.IsEmpty()) return {};
            Bounds2 r;
            r.min = { a.min.x > b.min.x ? a.min.x : b.min.x, a.min.y > b.min.y ? a.min.y : b.min.y };
            r.max = { a.max.x < b.max.x ? a.max.x : b.max.x, a.max.y < b.max.y ? a.max.y : b.max.y };
            if (r.min.x > r.max.x || r.min.y > r.max.y) return {};
            return r;
        }

        [[nodiscard]] static constexpr Bounds2 FromPoints(Vec2 a, Vec2 b) noexcept
        {
            Bounds2 r;
            r.min = { a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y };
            r.max = { a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y };
            return r;
        }
    };

    // -----------------------------------------------------------------------------------
    // THE TWO KINDS OF EVIDENCE A BOX CAN CARRY.   BOUNDS_EVIDENCE_PROOF.md
    //
    // A `Bounds2` is a rectangle. It does not say what it is evidence OF, and the kernel
    // asks two opposite things of a box:
    //
    //     "the shape is inside this"     an ENCLOSURE, which over-estimates
    //     "the shape reaches this far"   a WITNESS,    which under-estimates
    //
    // Both are boxes and they are NOT interchangeable. Using an enclosure where the proof
    // needs a witness is the defect behind F16, F26 and F38 - and behind a class census
    // (1138) that read `.Bounds()` and had to REMEMBER which one it was holding.
    //
    // So neither type converts to `Bounds2` implicitly. Getting the raw rectangle is
    // spelled `.Box()`, which is legitimate - a BVH stores rectangles - but it is written
    // down at every site that does it.

    // Evidence that S is a subset of Box(). Proof, section 2.
    //
    // Constructed by `Transform2::ApplyEnclosure` (lemma B1a) and by anything that already
    // holds a box known to contain the shape. `Enclosing` is an ASSERTION by the caller:
    // it is the one place where the invariant enters, so it is named rather than implicit.
    class BoundsEnclosure
    {
    public:
        constexpr BoundsEnclosure() noexcept = default;

        [[nodiscard]] static constexpr BoundsEnclosure Enclosing(const Bounds2& box) noexcept
        {
            return BoundsEnclosure(box);
        }

        [[nodiscard]] constexpr const Bounds2& Box() const noexcept { return box_; }
        [[nodiscard]] constexpr bool IsEmpty() const noexcept { return box_.IsEmpty(); }

        // Lemma B2. The useful branch is the negative one: boxes apart prove sets apart.
        [[nodiscard]] constexpr bool CouldTouch(const BoundsEnclosure& other,
                                                double tolerance) const noexcept
        {
            return box_.Overlaps(other.box_, tolerance);
        }

        // Lemma B3, and ONLY its true branch. `window` contains this enclosure, therefore
        // it contains the shape. A false result says nothing about the shape - which is
        // exactly what F16 read as if it did.
        [[nodiscard]] constexpr bool ProvesInside(const Bounds2& window) const noexcept
        {
            return window.Contains(box_);
        }

    private:
        explicit constexpr BoundsEnclosure(const Bounds2& box) noexcept : box_(box) {}
        Bounds2 box_{};
    };

    // Evidence that Box() is a subset of the shape's TRUE extent - the shape reaches at
    // least this far in each of the four axis directions. Proof, section 2 and 8.2.
    //
    // It cannot be produced by transforming an enclosure: lemma B1b gives the
    // counterexample. It is built from points OF the shape, and shrunk outward-error-wise
    // so that it stays a witness in double.
    class BoundsWitness
    {
    public:
        constexpr BoundsWitness() noexcept = default;

        [[nodiscard]] static constexpr BoundsWitness Reached(const Bounds2& box) noexcept
        {
            return BoundsWitness(box);
        }

        [[nodiscard]] constexpr const Bounds2& Box() const noexcept { return box_; }
        [[nodiscard]] constexpr bool IsEmpty() const noexcept { return box_.IsEmpty(); }
        [[nodiscard]] constexpr double Width() const noexcept { return box_.Width(); }
        [[nodiscard]] constexpr double Height() const noexcept { return box_.Height(); }

    private:
        explicit constexpr BoundsWitness(const Bounds2& box) noexcept : box_(box) {}
        Bounds2 box_{};
    };

    // Lemma B4: the shape behind `contained` is NOT a subset of the shape behind
    // `container`. The asymmetry is the whole point - a witness on the contained side, an
    // enclosure on the container side - and the signature is what enforces it.
    //
    // `margin` must dominate the construction error of `container`; a false answer means
    // "not proven", never "contained".
    [[nodiscard]] inline constexpr bool ProvesNotContained(const BoundsWitness& contained,
                                                           const BoundsEnclosure& container,
                                                           double margin) noexcept
    {
        // An empty witness asserts nothing, and nothing is refuted from it.
        if (contained.IsEmpty()) return false;
        // A non-empty shape is not inside an empty one, and the witness says it is not empty.
        if (container.IsEmpty()) return true;

        const Bounds2& w = contained.Box();
        const Bounds2& e = container.Box();
        return w.min.x < e.min.x - margin || w.max.x > e.max.x + margin ||
               w.min.y < e.min.y - margin || w.max.y > e.max.y + margin;
    }

    // Lemma B4.2: no TRANSLATION of the shape behind `contained` fits inside the shape
    // behind `container`. Same direction, position-independent question.
    [[nodiscard]] inline constexpr bool ProvesCannotFitByTranslation(
        const BoundsWitness& contained, const BoundsEnclosure& container,
        double margin) noexcept
    {
        if (contained.IsEmpty()) return false;
        if (container.IsEmpty()) return true;

        return contained.Width() > container.Box().Width() + margin ||
               contained.Height() > container.Box().Height() + margin;
    }

    [[nodiscard]] inline bool IsFinite(const Bounds2& b) noexcept
    {
        return b.IsEmpty() || (IsFinite(b.min) && IsFinite(b.max));
    }
}
