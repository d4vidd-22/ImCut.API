#include "CanonicalGeometry.hpp"

#include "../Polygon/PolygonMetrics.hpp"
#include "../Topology/Winding.hpp"

#include <algorithm>
#include <cmath>

namespace ImCut::Geometry
{
    bool CanonicalContour::operator==(const CanonicalContour& other) const noexcept
    {
        // The lattice is part of the identity, not metadata about it. The integers are
        // physical millimetres divided by `resolution`, so the same integers under two
        // lattices describe two different shapes: 10 mm at 1e-4 and 100 mm at 1e-3 both
        // quantise to 0..100000. Comparing only the payload called those equal.
        //
        // Exact comparison is correct here: the resolution is produced deterministically
        // by CanonicalScale for a given extent and request, so equal shapes reach
        // bit-identical values rather than merely close ones.
        return closed == other.closed &&
               resolution == other.resolution &&
               coordinates == other.coordinates &&
               handles == other.handles &&
               kinds == other.kinds;
    }

    bool CanonicalPath::operator==(const CanonicalPath& other) const noexcept
    {
        return fillRule == other.fillRule &&
               resolution == other.resolution &&
               contours == other.contours;
    }

    namespace Canonical
    {
        namespace
        {
            // Total order over canonical forms, used to pick between the variants
            // (forward/reversed, original/mirrored) that the options allow. Any total
            // order works as long as it is deterministic; this one compares the cheap
            // discriminators first.
            [[nodiscard]] int Compare(const CanonicalContour& a, const CanonicalContour& b) noexcept
            {
                if (a.closed != b.closed) return a.closed ? 1 : -1;
                if (a.coordinates.size() != b.coordinates.size())
                    return a.coordinates.size() < b.coordinates.size() ? -1 : 1;

                if (a.coordinates != b.coordinates)
                    return a.coordinates < b.coordinates ? -1 : 1;
                if (a.handles != b.handles)
                    return a.handles < b.handles ? -1 : 1;
                if (a.kinds != b.kinds)
                    return a.kinds < b.kinds ? -1 : 1;

                return 0;
            }

            // Position, segment kind and both control points of the segment leaving the
            // node.
            //
            // Handles have to be in the key. Two nodes can share a position and a kind
            // while their outgoing segments curve completely differently; with a
            // position-only key Booth's algorithm cannot separate those candidates and
            // the winning rotation ends up decided by whichever node the input happened
            // to start at - which is exactly the invariance the canonical form exists to
            // provide.
            struct NodeKey
            {
                std::int64_t x;
                std::int64_t y;
                std::int64_t c1x;
                std::int64_t c1y;
                std::int64_t c2x;
                std::int64_t c2y;
                std::uint8_t kind;

                [[nodiscard]] bool operator<(const NodeKey& other) const noexcept
                {
                    if (x != other.x) return x < other.x;
                    if (y != other.y) return y < other.y;
                    if (kind != other.kind) return kind < other.kind;
                    if (c1x != other.c1x) return c1x < other.c1x;
                    if (c1y != other.c1y) return c1y < other.c1y;
                    if (c2x != other.c2x) return c2x < other.c2x;
                    return c2y < other.c2y;
                }

                [[nodiscard]] bool operator==(const NodeKey& other) const noexcept
                {
                    return x == other.x && y == other.y && kind == other.kind &&
                           c1x == other.c1x && c1y == other.c1y &&
                           c2x == other.c2x && c2y == other.c2y;
                }

                [[nodiscard]] bool operator!=(const NodeKey& other) const noexcept
                {
                    return !(*this == other);
                }
            };

            // Booth's algorithm for the least lexicographic rotation, in O(n).
            //
            // A ring has no inherent first node, so a canonical start has to be derived
            // from content. Comparing all n rotations pairwise would be O(n^2), which on
            // a 1.6k-segment contour from the reference corpus is 2.5M comparisons per
            // canonicalisation - and canonicalisation runs on every cache lookup.
            [[nodiscard]] std::size_t LeastRotation(const std::vector<NodeKey>& keys)
            {
                const std::size_t n = keys.size();
                if (n < 2)
                    return 0;

                std::vector<std::ptrdiff_t> failure(2 * n, -1);
                std::size_t k = 0;

                auto at = [&](std::size_t index) noexcept -> const NodeKey&
                {
                    return keys[index % n];
                };

                for (std::size_t j = 1; j < 2 * n; ++j)
                {
                    const NodeKey& sj = at(j);
                    std::ptrdiff_t i = failure[j - k - 1];

                    while (i != -1 && sj != at(k + static_cast<std::size_t>(i) + 1))
                    {
                        if (sj < at(k + static_cast<std::size_t>(i) + 1))
                            k = j - static_cast<std::size_t>(i) - 1;
                        i = failure[i];
                    }

                    if (sj != at(k + static_cast<std::size_t>(i) + 1))
                    {
                        if (sj < at(k))
                            k = j;
                        failure[j - k] = -1;
                    }
                    else
                    {
                        failure[j - k] = i + 1;
                    }
                }

                return k % n;
            }

            // Produces the canonical form for one specific choice of direction and
            // mirroring. The caller picks the best variant.
            // The frame a contour is quantised in: one origin and one lattice.
            //
            // For a lone Contour the frame is its own bounds, which is what makes a
            // single ring translation-invariant. For a Path the frame is the WHOLE
            // path's bounds, shared by every contour, so the offsets between contours
            // survive quantisation. Deriving the origin per contour is exactly what
            // erased those offsets and let two different objects share one identity.
            struct CanonicalFrame
            {
                double originX = 0.0;
                double originY = 0.0;
                QuantizationScale scale{};
            };

            [[nodiscard]] CanonicalFrame MakeFrame(const Bounds2& extent, const CanonicalOptions& options)
            {
                CanonicalFrame frame;
                frame.scale = Quantization::CanonicalScale(extent, options.resolution);
                if (options.normalizeTranslation && !extent.IsEmpty())
                {
                    frame.originX = extent.min.x;
                    frame.originY = extent.min.y;
                }
                return frame;
            }

            // `frame` is optional. A lone contour passes nullptr so the frame is derived
            // from the variant actually being quantised - mirroring negates x, and the
            // mirrored variant has to be normalised against its OWN extent or reflection
            // invariance breaks. A Path passes its shared frame instead.
            [[nodiscard]] CanonicalContour Build(const Contour& source, const CanonicalOptions& options,
                                                 bool reverse, bool mirror, const CanonicalFrame* frame)
            {
                CanonicalContour result;

                // Non-finite input has no canonical form.
                //
                // Quantization::Snap maps NaN to the origin, so a contour carrying one
                // used to canonicalise to a shape sitting at (0,0) - a fingerprint for
                // geometry that does not exist. An empty form is the honest answer, and
                // every entry point that keys a cache on this already refuses non-finite
                // input through IsStructurallyValid.
                if (!source.HasFiniteCoordinates())
                    return result;

                Contour working = source;
                if (mirror)
                {
                    for (Vec2& node : working.nodes) node.x = -node.x;
                    for (Vec2& handle : working.handles) handle.x = -handle.x;
                    // Mirroring flips traversal direction, so undo that to keep the
                    // reverse flag meaning what it says.
                    Winding::Reverse(working);
                }

                if (reverse)
                    Winding::Reverse(working);

                const std::size_t segmentCount = working.SegmentCount();
                const std::size_t nodeCount = working.nodes.size();

                result.closed = working.closed;
                result.reversed = reverse;
                result.mirrored = mirror;

                if (nodeCount == 0)
                    return result;

                // The canonical lattice is fixed by resolution alone. It must not be
                // derived from coordinate magnitude the way the clipping lattice is:
                // that coarsening collapsed every part from 1000 mm upward onto the same
                // integers, so different physical sizes shared a fingerprint.
                const CanonicalFrame ownFrame =
                    frame != nullptr ? *frame : MakeFrame(Metrics::ComputeBounds(working), options);

                const QuantizationScale& scale = ownFrame.scale;
                if (!scale.valid)
                    return result;

                result.resolution = scale.resolution;

                // Subtract in double, then quantise once. Quantising each coordinate and
                // then subtracting quantised origins lets the two roundings disagree by a
                // lattice step, which is precisely how position leaked into the
                // fingerprint.
                const double originX = ownFrame.originX;
                const double originY = ownFrame.originY;
                result.translationX = scale.ToInteger(originX);
                result.translationY = scale.ToInteger(originY);

                auto quantizeX = [&](double value) noexcept { return scale.ToInteger(value - originX); };
                auto quantizeY = [&](double value) noexcept { return scale.ToInteger(value - originY); };

                // Rotate a closed ring to its canonical start. Open contours have a real
                // first node, so rotating them would destroy information.
                std::size_t start = 0;
                if (options.normalizeStart && working.closed && segmentCount > 1 &&
                    nodeCount == segmentCount)
                {
                    std::vector<NodeKey> keys;
                    keys.reserve(nodeCount);
                    for (std::size_t i = 0; i < nodeCount; ++i)
                    {
                        keys.push_back({ quantizeX(working.nodes[i].x),
                                         quantizeY(working.nodes[i].y),
                                         quantizeX(working.handles[i * 2].x),
                                         quantizeY(working.handles[i * 2].y),
                                         quantizeX(working.handles[i * 2 + 1].x),
                                         quantizeY(working.handles[i * 2 + 1].y),
                                         static_cast<std::uint8_t>(working.kinds[i]) });
                    }
                    start = LeastRotation(keys);
                }

                result.startNode = static_cast<std::uint32_t>(start);

                result.coordinates.reserve(nodeCount * 2);
                result.handles.reserve(segmentCount * 4);
                result.kinds.reserve(segmentCount);

                for (std::size_t i = 0; i < nodeCount; ++i)
                {
                    const Vec2& node = working.nodes[(i + start) % nodeCount];
                    result.coordinates.push_back(quantizeX(node.x));
                    result.coordinates.push_back(quantizeY(node.y));
                }

                for (std::size_t i = 0; i < segmentCount; ++i)
                {
                    const std::size_t index = (i + start) % segmentCount;
                    result.kinds.push_back(working.kinds[index]);
                    result.handles.push_back(quantizeX(working.handles[index * 2].x));
                    result.handles.push_back(quantizeY(working.handles[index * 2].y));
                    result.handles.push_back(quantizeX(working.handles[index * 2 + 1].x));
                    result.handles.push_back(quantizeY(working.handles[index * 2 + 1].y));
                }

                return result;
            }
        }

        CanonicalContour Canonicalize(const Contour& contour, const CanonicalOptions& options)
        {
            // Deliberately not the session tolerance. Canonical form is an identity: it
            // keys the prepared cache and the fingerprint, so the same contour has to
            // canonicalise identically no matter which PrecisionMode the caller happens
            // to be running under. Taking the orientation from the context would make a
            // shape's identity depend on the settings of whoever looked at it first, and
            // two sessions would then disagree about a cache hit. The lattice is fixed
            // for the same reason - see CanonicalOptions::resolution.
            const GeometryTolerance tolerance = GeometryTolerance::Production();

            // Which direction variants are worth building depends on whether the
            // contour has a well-defined orientation at all.
            bool tryForward = true;
            bool tryReversed = false;

            if (options.normalizeWinding)
            {
                const Orientation orientation = Metrics::OrientationOf(contour, tolerance);
                if (orientation == Orientation::Clockwise)
                {
                    tryForward = false;
                    tryReversed = true;
                }
                else if (orientation == Orientation::Degenerate)
                {
                    // Zero area means no orientation to normalise against, so both
                    // directions are equally valid; pick deterministically by content.
                    tryReversed = true;
                }
            }

            CanonicalContour best;
            bool haveBest = false;

            auto consider = [&](bool reverse, bool mirror)
            {
                // No shared frame: each variant is normalised against its own extent,
                // which is what makes a lone ring invariant to translation and, when
                // enabled, to reflection.
                CanonicalContour candidate = Build(contour, options, reverse, mirror, nullptr);
                if (!haveBest || Compare(candidate, best) < 0)
                {
                    best = std::move(candidate);
                    haveBest = true;
                }
            };

            if (tryForward) consider(false, false);
            if (tryReversed) consider(true, false);

            if (options.allowReflection)
            {
                if (tryForward) consider(false, true);
                if (tryReversed) consider(true, true);
            }

            return best;
        }

        CanonicalPath Canonicalize(const Path& path, const CanonicalOptions& options)
        {
            CanonicalPath result;
            result.fillRule = path.fillRule;
            result.contours.reserve(path.contours.size());

            // ONE frame for the whole path.
            //
            // Canonicalising each contour against its own bounds made every ring
            // individually translation-invariant, which erased the offsets BETWEEN
            // rings: two squares 20 mm apart and two squares 200 mm apart produced the
            // same canonical form, the same fingerprint, and therefore the same
            // PreparedShapeDefinition. A hole in a different place became the same part.
            //
            // Quantising every contour against the path origin keeps the contract on
            // both sides: translating the whole path moves the origin with it and the
            // identity is unchanged, while moving one contour relative to another
            // changes the integers and the identity with them.
            const Bounds2 extent = Metrics::ComputeBounds(path);
            const CanonicalFrame frame = MakeFrame(extent, options);
            result.resolution = frame.scale.resolution;

            const GeometryTolerance tolerance = GeometryTolerance::Production();

            // Under NonZero the direction each ring was drawn in is SEMANTIC, not
            // incidental.
            //
            // Normalising every ring to counter-clockwise makes the identity invariant
            // to traversal, which is exactly right for EvenOdd - there the region does
            // not depend on direction at all - and wrong for NonZero, where an inner
            // ring wound with its parent is solid and wound against it is a hole. Both
            // normalised to the same integers, so a donut and a solid produced the same
            // fingerprint and GeometrySession handed the second caller the first
            // caller's prepared object: its bounds, its BVH, its net area.
            //
            // What NonZero IS invariant to is reversing every ring together, which
            // negates the winding number everywhere and leaves the filled set alone. So
            // the normalisation to keep is one global flip for the whole path, not one
            // decision per ring. Direction then survives in the node ORDER inside
            // `coordinates`, which both operator== and FeedContour already compare -
            // nothing has to be added to the identity payload.
            //
            // The flip is chosen from the signed-area sum: it is one deterministic
            // number, it is dominated by the outer ring for any sane region, and since
            // V5 it is well conditioned at any distance from the origin (P0-8). A sum of
            // exactly zero is genuinely ambiguous, so those paths build both variants
            // and keep the lexicographically smaller one.
            const bool directionIsSemantic =
                options.normalizeWinding && path.fillRule == FillRule::NonZero;

            double signedAreaSum = 0.0;
            if (directionIsSemantic)
            {
                for (const Contour& contour : path.contours)
                    signedAreaSum += Metrics::SignedArea(contour);
            }
            const bool ambiguousDirection = directionIsSemantic && signedAreaSum == 0.0;

            // Reflection is decided for the path as a whole: mirroring one ring about
            // the frame origin would move it relative to the others.
            auto buildAll = [&](bool mirror, bool globalFlip)
            {
                std::vector<CanonicalContour> contours;
                contours.reserve(path.contours.size());

                for (const Contour& contour : path.contours)
                {
                    // Direction stays per contour - reversing a traversal does not move
                    // any point, so it cannot disturb the shared frame.
                    bool tryForward = true;
                    bool tryReversed = false;
                    if (directionIsSemantic)
                    {
                        // Exactly one variant: the same flip for every ring, so the
                        // relative directions that carry the meaning are preserved.
                        tryForward = !globalFlip;
                        tryReversed = globalFlip;
                    }
                    else if (options.normalizeWinding)
                    {
                        const Orientation orientation = Metrics::OrientationOf(contour, tolerance);
                        if (orientation == Orientation::Clockwise)
                        {
                            tryForward = false;
                            tryReversed = true;
                        }
                        else if (orientation == Orientation::Degenerate)
                        {
                            tryReversed = true;
                        }
                    }

                    CanonicalContour best;
                    bool haveBest = false;
                    auto consider = [&](bool reverse)
                    {
                        CanonicalContour candidate = Build(contour, options, reverse, mirror, &frame);
                        if (!haveBest || Compare(candidate, best) < 0)
                        {
                            best = std::move(candidate);
                            haveBest = true;
                        }
                    };

                    if (tryForward) consider(false);
                    if (tryReversed) consider(true);

                    contours.push_back(std::move(best));
                }

                // Order contours by content, so the same rings listed in a different
                // order canonicalise identically. The content now carries each ring
                // position within the path, so ordering stays deterministic without
                // flattening the geometry that distinguishes two paths.
                std::sort(contours.begin(), contours.end(),
                          [](const CanonicalContour& a, const CanonicalContour& b) noexcept
                          {
                              return Compare(a, b) < 0;
                          });
                return contours;
            };

            // Lexicographic order over whole canonicalised paths, used to pick between
            // the two global-flip variants when the signed-area sum cannot choose.
            auto isSmaller = [](const std::vector<CanonicalContour>& a,
                                const std::vector<CanonicalContour>& b) noexcept
            {
                if (a.size() != b.size())
                    return a.size() < b.size();
                for (std::size_t i = 0; i < a.size(); ++i)
                {
                    const int order = Compare(a[i], b[i]);
                    if (order != 0)
                        return order < 0;
                }
                return false;
            };

            const bool globalFlip = directionIsSemantic && signedAreaSum < 0.0;
            result.contours = buildAll(false, globalFlip);

            if (ambiguousDirection)
            {
                std::vector<CanonicalContour> flipped = buildAll(false, true);
                if (isSmaller(flipped, result.contours))
                    result.contours = std::move(flipped);
            }

            if (options.allowReflection)
            {
                // The mirrored variant needs its own frame, so it is canonicalised as a
                // whole path rather than ring by ring.
                Path mirrored = path;
                for (Contour& contour : mirrored.contours)
                {
                    for (Vec2& node : contour.nodes) node.x = -node.x;
                    for (Vec2& handle : contour.handles) handle.x = -handle.x;
                    Winding::Reverse(contour);
                }

                CanonicalOptions withoutReflection = options;
                withoutReflection.allowReflection = false;
                const CanonicalPath other = Canonicalize(mirrored, withoutReflection);

                bool otherIsSmaller = other.contours.size() < result.contours.size();
                if (other.contours.size() == result.contours.size())
                {
                    for (std::size_t i = 0; i < other.contours.size(); ++i)
                    {
                        const int order = Compare(other.contours[i], result.contours[i]);
                        if (order != 0) { otherIsSmaller = order < 0; break; }
                    }
                }
                if (otherIsSmaller)
                    result.contours = other.contours;
            }

            return result;
        }

        Contour Rebuild(const CanonicalContour& canonical)
        {
            Contour contour;

            const std::size_t nodeCount = canonical.NodeCount();
            if (nodeCount == 0 || canonical.resolution <= 0.0)
                return contour;

            const double resolution = canonical.resolution;
            auto toMillimetres = [&](std::int64_t value) noexcept
            {
                return static_cast<double>(value) * resolution;
            };

            contour.closed = canonical.closed;
            contour.nodes.reserve(nodeCount);
            for (std::size_t i = 0; i < nodeCount; ++i)
            {
                contour.nodes.push_back({ toMillimetres(canonical.coordinates[i * 2]),
                                          toMillimetres(canonical.coordinates[i * 2 + 1]) });
            }

            const std::size_t segmentCount = canonical.kinds.size();
            contour.kinds = canonical.kinds;
            contour.handles.reserve(segmentCount * 2);
            for (std::size_t i = 0; i < segmentCount; ++i)
            {
                contour.handles.push_back({ toMillimetres(canonical.handles[i * 4]),
                                            toMillimetres(canonical.handles[i * 4 + 1]) });
                contour.handles.push_back({ toMillimetres(canonical.handles[i * 4 + 2]),
                                            toMillimetres(canonical.handles[i * 4 + 3]) });
            }

            return contour;
        }
    }
}
