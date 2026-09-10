#include "SvgLoader.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>

namespace ImCut::Geometry::Corpus
{
    namespace
    {
        struct Cursor
        {
            const char* current;
            const char* end;

            [[nodiscard]] bool AtEnd() const noexcept { return current >= end; }

            void SkipSeparators() noexcept
            {
                while (current < end && (std::isspace(static_cast<unsigned char>(*current)) || *current == ','))
                    ++current;
            }

            [[nodiscard]] bool ReadNumber(double& value) noexcept
            {
                SkipSeparators();
                if (AtEnd())
                    return false;

                char* stop = nullptr;
                const double parsed = std::strtod(current, &stop);
                if (stop == current)
                    return false;

                current = stop;
                value = parsed;
                return true;
            }
        };

        [[nodiscard]] std::string AttributeValue(const std::string& text, std::size_t elementStart,
                                                 const std::string& attribute)
        {
            const std::size_t elementEnd = text.find('>', elementStart);
            const std::string needle = attribute + "=\"";

            // The match must start an attribute name, not end one: searching for d="
            // alone also finds the d in id=", which silently reads the wrong value.
            std::size_t at = elementStart;
            while ((at = text.find(needle, at)) != std::string::npos)
            {
                if (elementEnd != std::string::npos && at > elementEnd)
                    return {};

                const bool atNameStart =
                    at == 0 || std::isspace(static_cast<unsigned char>(text[at - 1])) != 0;

                if (atNameStart)
                    break;

                at += needle.size();
            }

            if (at == std::string::npos)
                return {};

            at += needle.size();
            const std::size_t close = text.find('"', at);
            if (close == std::string::npos)
                return {};

            return text.substr(at, close - at);
        }

        [[nodiscard]] double ParseLengthMillimetres(const std::string& text)
        {
            if (text.empty())
                return 0.0;

            char* stop = nullptr;
            const double value = std::strtod(text.c_str(), &stop);
            if (stop == text.c_str())
                return 0.0;

            std::string unit(stop);
            unit.erase(std::remove_if(unit.begin(), unit.end(),
                                      [](unsigned char c) { return std::isspace(c) != 0; }),
                       unit.end());

            // The corpus declares millimetres explicitly. Other absolute units are
            // converted; anything unrecognised is treated as user units by the caller,
            // which then falls back to a scale of 1.
            if (unit.empty() || unit == "mm") return value;
            if (unit == "cm") return value * 10.0;
            if (unit == "in") return value * 25.4;
            if (unit == "pt") return value * 25.4 / 72.0;
            if (unit == "pc") return value * 25.4 / 6.0;
            if (unit == "px") return value * 25.4 / 96.0;

            return 0.0;
        }
    }

    bool ParsePathData(const std::string& data, double scale, std::vector<Contour>& out,
                       std::string& error, std::size_t& lineSegments, std::size_t& cubicSegments)
    {
        Cursor cursor{ data.data(), data.data() + data.size() };

        Contour current;
        bool building = false;

        Vec2 position{ 0.0, 0.0 };
        Vec2 subpathStart{ 0.0, 0.0 };

        // Reflection of the previous cubic's second control point, for S/s.
        Vec2 lastControl{ 0.0, 0.0 };
        bool lastWasCubic = false;

        char command = 0;

        auto finish = [&](bool closed)
        {
            if (!building)
                return;

            if (current.nodes.size() >= 2)
            {
                if (closed)
                    current.Close(0.0);
                out.push_back(std::move(current));
            }
            current = Contour{};
            building = false;
        };

        auto toMillimetres = [&](Vec2 point) noexcept { return Vec2{ point.x * scale, point.y * scale }; };

        while (true)
        {
            cursor.SkipSeparators();
            if (cursor.AtEnd())
                break;

            const char c = *cursor.current;
            if (std::isalpha(static_cast<unsigned char>(c)))
            {
                command = c;
                ++cursor.current;
            }
            else if (command == 0)
            {
                error = "path data does not start with a command";
                return false;
            }
            else if (command == 'M')
            {
                // A repeated coordinate pair after moveto is an implicit lineto.
                command = 'L';
            }
            else if (command == 'm')
            {
                command = 'l';
            }

            const bool relative = std::islower(static_cast<unsigned char>(command)) != 0;
            const char upper = static_cast<char>(std::toupper(static_cast<unsigned char>(command)));

            switch (upper)
            {
                case 'M':
                {
                    double x = 0.0, y = 0.0;
                    if (!cursor.ReadNumber(x) || !cursor.ReadNumber(y))
                    {
                        error = "moveto is missing coordinates";
                        return false;
                    }

                    finish(false);

                    position = relative ? Vec2{ position.x + x, position.y + y } : Vec2{ x, y };
                    subpathStart = position;

                    current = Contour{};
                    current.MoveTo(toMillimetres(position));
                    building = true;
                    lastWasCubic = false;
                    break;
                }

                case 'L':
                {
                    double x = 0.0, y = 0.0;
                    if (!cursor.ReadNumber(x) || !cursor.ReadNumber(y))
                    {
                        error = "lineto is missing coordinates";
                        return false;
                    }
                    if (!building) { error = "lineto before moveto"; return false; }

                    position = relative ? Vec2{ position.x + x, position.y + y } : Vec2{ x, y };
                    current.LineTo(toMillimetres(position));
                    ++lineSegments;
                    lastWasCubic = false;
                    break;
                }

                case 'H':
                case 'V':
                {
                    double value = 0.0;
                    if (!cursor.ReadNumber(value))
                    {
                        error = "horizontal/vertical lineto is missing its coordinate";
                        return false;
                    }
                    if (!building) { error = "lineto before moveto"; return false; }

                    if (upper == 'H')
                        position.x = relative ? position.x + value : value;
                    else
                        position.y = relative ? position.y + value : value;

                    current.LineTo(toMillimetres(position));
                    ++lineSegments;
                    lastWasCubic = false;
                    break;
                }

                case 'C':
                {
                    double x1 = 0, y1 = 0, x2 = 0, y2 = 0, x = 0, y = 0;
                    if (!cursor.ReadNumber(x1) || !cursor.ReadNumber(y1) ||
                        !cursor.ReadNumber(x2) || !cursor.ReadNumber(y2) ||
                        !cursor.ReadNumber(x) || !cursor.ReadNumber(y))
                    {
                        error = "curveto is missing coordinates";
                        return false;
                    }
                    if (!building) { error = "curveto before moveto"; return false; }

                    const Vec2 base = relative ? position : Vec2{ 0.0, 0.0 };
                    const Vec2 control1{ base.x + x1, base.y + y1 };
                    const Vec2 control2{ base.x + x2, base.y + y2 };
                    const Vec2 target{ base.x + x, base.y + y };

                    current.CubicTo(toMillimetres(control1), toMillimetres(control2), toMillimetres(target));
                    ++cubicSegments;

                    lastControl = control2;
                    lastWasCubic = true;
                    position = target;
                    break;
                }

                case 'S':
                {
                    double x2 = 0, y2 = 0, x = 0, y = 0;
                    if (!cursor.ReadNumber(x2) || !cursor.ReadNumber(y2) ||
                        !cursor.ReadNumber(x) || !cursor.ReadNumber(y))
                    {
                        error = "smooth curveto is missing coordinates";
                        return false;
                    }
                    if (!building) { error = "curveto before moveto"; return false; }

                    // The first control mirrors the previous curve's second control;
                    // with no previous curve it coincides with the current point.
                    const Vec2 control1 = lastWasCubic
                        ? Vec2{ 2.0 * position.x - lastControl.x, 2.0 * position.y - lastControl.y }
                        : position;

                    const Vec2 base = relative ? position : Vec2{ 0.0, 0.0 };
                    const Vec2 control2{ base.x + x2, base.y + y2 };
                    const Vec2 target{ base.x + x, base.y + y };

                    current.CubicTo(toMillimetres(control1), toMillimetres(control2), toMillimetres(target));
                    ++cubicSegments;

                    lastControl = control2;
                    lastWasCubic = true;
                    position = target;
                    break;
                }

                case 'Z':
                {
                    finish(true);
                    position = subpathStart;
                    lastWasCubic = false;
                    break;
                }

                case 'Q':
                case 'T':
                case 'A':
                    error = std::string("unsupported path command '") + command +
                            "' (quadratic and arc segments are not implemented)";
                    return false;

                default:
                    error = std::string("unknown path command '") + command + "'";
                    return false;
            }
        }

        finish(false);
        return true;
    }

    LoadResult LoadFile(const std::string& path)
    {
        LoadResult result;
        result.document.name = std::filesystem::path(path).filename().string();

        std::ifstream file(path, std::ios::binary);
        if (!file)
        {
            result.error = "cannot open " + path;
            return result;
        }

        std::string text((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
        result.document.fileBytes = text.size();

        const std::size_t svgAt = text.find("<svg");
        if (svgAt == std::string::npos)
        {
            result.error = "no <svg> element";
            return result;
        }

        result.document.widthMillimetres = ParseLengthMillimetres(AttributeValue(text, svgAt, "width"));
        result.document.heightMillimetres = ParseLengthMillimetres(AttributeValue(text, svgAt, "height"));

        const std::string viewBox = AttributeValue(text, svgAt, "viewBox");
        if (!viewBox.empty())
        {
            Cursor cursor{ viewBox.data(), viewBox.data() + viewBox.size() };
            double minX = 0, minY = 0, width = 0, height = 0;
            if (cursor.ReadNumber(minX) && cursor.ReadNumber(minY) &&
                cursor.ReadNumber(width) && cursor.ReadNumber(height))
            {
                result.document.viewBoxWidth = width;
                result.document.viewBoxHeight = height;
            }
        }

        // Uniform scale from user units to millimetres. Falling back to 1.0 keeps the
        // geometry in user units rather than inventing a conversion.
        if (result.document.viewBoxWidth > 0.0 && result.document.widthMillimetres > 0.0)
            result.document.scale = result.document.widthMillimetres / result.document.viewBoxWidth;

        // Reject anything the corpus is not: a transform or a non-path primitive would
        // silently shift or drop geometry.
        if (text.find("transform=") != std::string::npos)
        {
            result.error = "transform attributes are not supported";
            return result;
        }

        for (const char* primitive : { "<rect", "<circle", "<ellipse", "<polygon", "<polyline", "<line " })
        {
            if (text.find(primitive) != std::string::npos)
            {
                result.error = std::string("unsupported primitive ") + primitive;
                return result;
            }
        }

        std::size_t at = 0;
        while ((at = text.find("<path", at)) != std::string::npos)
        {
            const std::string data = AttributeValue(text, at, "d");
            at += 5;

            if (data.empty())
                continue;

            Path element;
            element.fillRule = FillRule::EvenOdd;

            std::string error;
            if (!ParsePathData(data, result.document.scale, element.contours, error,
                               result.document.lineSegments, result.document.cubicSegments))
            {
                result.error = result.document.name + ": " + error;
                return result;
            }

            for (const Contour& contour : element.contours)
            {
                if (contour.closed) ++result.document.closedContours;
                else ++result.document.openContours;
            }

            if (!element.contours.empty())
                result.document.paths.push_back(std::move(element));
        }

        result.ok = true;
        return result;
    }

    std::vector<LoadResult> LoadDirectory(const std::string& directory)
    {
        std::vector<std::string> files;

        std::error_code code;
        for (const auto& entry : std::filesystem::directory_iterator(directory, code))
        {
            if (!entry.is_regular_file())
                continue;

            std::string extension = entry.path().extension().string();
            std::transform(extension.begin(), extension.end(), extension.begin(),
                           [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

            if (extension == ".svg")
                files.push_back(entry.path().string());
        }

        // Sorted so reports and benchmarks list files in the same order every run.
        std::sort(files.begin(), files.end());

        std::vector<LoadResult> results;
        results.reserve(files.size());
        for (const std::string& file : files)
            results.push_back(LoadFile(file));

        return results;
    }
}
