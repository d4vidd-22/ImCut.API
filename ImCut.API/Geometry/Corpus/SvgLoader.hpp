#pragma once

// Test and benchmark utility, not part of the kernel and not compiled into the DLL.
//
// Scope is deliberately narrow. The reference corpus in TESTES/ is pure <path> data
// using only M/m, l, c and z, with no transform attributes and no other primitives, so
// a full SVG implementation would be a large dependency bought for nothing. What is
// supported is stated up front and anything else is a hard, reported failure - a
// loader that silently mis-parses a command it does not understand would quietly
// corrupt every measurement taken with it.
//
// Supported: M m L l H h V v C c S s Z z
// Rejected:  Q q T t A a  (reported, never guessed at)

#include "../GeometryTypes.hpp"

#include <string>
#include <vector>

namespace ImCut::Geometry::Corpus
{
    struct SvgDocument
    {
        std::string name;

        // Millimetre size declared by the width/height attributes.
        double widthMillimetres = 0.0;
        double heightMillimetres = 0.0;

        // viewBox extents in user units.
        double viewBoxWidth = 0.0;
        double viewBoxHeight = 0.0;

        // Uniform user-units-to-millimetres factor derived from the two above.
        double scale = 1.0;

        // One path per <path> element, already converted to millimetres.
        std::vector<Path> paths;

        // Measured while parsing, never estimated.
        std::size_t lineSegments = 0;
        std::size_t cubicSegments = 0;
        std::size_t closedContours = 0;
        std::size_t openContours = 0;

        [[nodiscard]] std::size_t ContourCount() const noexcept
        {
            return closedContours + openContours;
        }

        [[nodiscard]] std::size_t SegmentCount() const noexcept
        {
            return lineSegments + cubicSegments;
        }

        [[nodiscard]] std::size_t FileBytes() const noexcept { return fileBytes; }

        std::size_t fileBytes = 0;
    };

    struct LoadResult
    {
        bool ok = false;
        std::string error;
        SvgDocument document;
    };

    // Reads and parses one SVG file. The file is only ever read, never written.
    [[nodiscard]] LoadResult LoadFile(const std::string& path);

    // Parses a path `d` attribute into contours in user units.
    // Returns false and fills `error` on any unsupported command.
    [[nodiscard]] bool ParsePathData(const std::string& data, double scale,
                                     std::vector<Contour>& out, std::string& error,
                                     std::size_t& lineSegments, std::size_t& cubicSegments);

    // Loads every *.svg in a directory, sorted by name for deterministic reporting.
    [[nodiscard]] std::vector<LoadResult> LoadDirectory(const std::string& directory);
}
