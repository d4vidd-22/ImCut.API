#define NOMINMAX 1
#include "ColorConverter.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cwctype>
#include <fstream>
#include <limits>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#pragma warning(push)
#pragma warning(disable: 5033)
#include <lcms2.h>
#pragma warning(pop)

#pragma comment(lib, "Mscms.lib")
#pragma comment(lib, "lcms2.lib")

namespace ImCut::Color
{
    namespace
    {
        constexpr std::uint32_t CacheVersion = 3;
        constexpr std::array<char, 8> CacheMagic{ 'I', 'M', 'C', 'U', 'T', '4', 'D', 'L' };
        constexpr std::size_t TranslateBatchLimit = 65536;
        static_assert(sizeof(Rgb16) == 6);

#pragma pack(push, 1)
        struct CacheHeader
        {
            char magic[8];
            std::uint32_t version;
            std::uint32_t gridSize;
            std::uint32_t intent;
            std::uint32_t storageBits;
            std::uint64_t sourceFingerprint;
            std::uint64_t targetFingerprint;
            std::uint64_t entryCount;
            std::uint64_t payloadHash;
        };
#pragma pack(pop)

        [[nodiscard]] std::string WindowsError(const char* operation)
        {
            const DWORD error = GetLastError();
            LPSTR buffer = nullptr;

            const DWORD length = FormatMessageA(
                FORMAT_MESSAGE_ALLOCATE_BUFFER |
                FORMAT_MESSAGE_FROM_SYSTEM |
                FORMAT_MESSAGE_IGNORE_INSERTS,
                nullptr,
                error,
                MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
                reinterpret_cast<LPSTR>(&buffer),
                0,
                nullptr);

            std::ostringstream stream;
            stream << operation << " failed";

            if (error != ERROR_SUCCESS)
                stream << " (" << error << ")";

            if (length > 0 && buffer)
            {
                stream << ": " << buffer;
                LocalFree(buffer);
            }

            return stream.str();
        }

        [[nodiscard]] std::uint64_t HashBytes(const void* data, std::size_t size) noexcept
        {
            constexpr std::uint64_t offsetBasis = 14695981039346656037ull;
            constexpr std::uint64_t prime = 1099511628211ull;

            std::uint64_t hash = offsetBasis;
            const auto* bytes = static_cast<const std::uint8_t*>(data);

            for (std::size_t i = 0; i < size; ++i)
            {
                hash ^= bytes[i];
                hash *= prime;
            }

            return hash;
        }

        [[nodiscard]] std::vector<std::uint8_t> ReadProfileBytes(HPROFILE profile)
        {
            DWORD size = 0;
            SetLastError(ERROR_SUCCESS);
            GetColorProfileFromHandle(profile, nullptr, &size);

            if (size == 0)
                throw std::runtime_error(WindowsError("GetColorProfileFromHandle"));

            std::vector<std::uint8_t> bytes(size);

            if (!GetColorProfileFromHandle(profile, bytes.data(), &size))
                throw std::runtime_error(WindowsError("GetColorProfileFromHandle"));

            bytes.resize(size);
            return bytes;
        }

        [[nodiscard]] bool IsRawIcc(const std::vector<std::uint8_t>& bytes) noexcept
        {
            return
                bytes.size() >= 40 &&
                bytes[36] == 'a' &&
                bytes[37] == 'c' &&
                bytes[38] == 's' &&
                bytes[39] == 'p';
        }

        [[nodiscard]] std::filesystem::path ResolveProfilePath(const std::filesystem::path& input)
        {
            std::error_code ec;

            if (input.is_absolute() && std::filesystem::exists(input, ec))
                return input;

            if (std::filesystem::exists(input, ec))
            {
                const auto absolute = std::filesystem::absolute(input, ec);
                if (!ec)
                    return absolute;
            }

            return input;
        }

        [[nodiscard]] std::wstring DisplayNameFor(const std::filesystem::path& path)
        {
            auto name = path.stem().wstring();

            if (name.empty())
                name = path.filename().wstring();

            return name;
        }

        [[nodiscard]] DWORD ToWindowsIntent(RenderingIntent intent) noexcept
        {
            switch (intent)
            {
            case RenderingIntent::RelativeColorimetric:
                return INTENT_RELATIVE_COLORIMETRIC;
            case RenderingIntent::Saturation:
                return INTENT_SATURATION;
            case RenderingIntent::AbsoluteColorimetric:
                return INTENT_ABSOLUTE_COLORIMETRIC;
            case RenderingIntent::Perceptual:
            default:
                return INTENT_PERCEPTUAL;
            }
        }

        [[nodiscard]] std::uint16_t PercentToWord(double value) noexcept
        {
            const double clamped = std::clamp(value, 0.0, 100.0);
            return static_cast<std::uint16_t>(std::lround(clamped * 65535.0 / 100.0));
        }

        [[nodiscard]] std::uint8_t WordToByte(std::uint16_t value) noexcept
        {
            return static_cast<std::uint8_t>((static_cast<std::uint32_t>(value) + 128u) / 257u);
        }

        [[nodiscard]] std::uint16_t ByteToWord(std::uint8_t value) noexcept
        {
            return static_cast<std::uint16_t>(
                static_cast<std::uint32_t>(value) * 257u);
        }

        [[nodiscard]] std::uint16_t ClampWord(double value) noexcept
        {
            return static_cast<std::uint16_t>(std::clamp(std::lround(value), 0L, 65535L));
        }

        [[nodiscard]] Rgb8 ApplyBlackFloor(Rgb8 value, const BuildOptions& options) noexcept
        {
            if (!options.blackFloorEnabled)
                return value;

            const std::uint8_t floor = options.blackFloorRgb;

            if (std::max({ value.r, value.g, value.b }) < floor)
                return { floor, floor, floor };

            return value;
        }

        [[nodiscard]] Rgb16 ApplyBlackFloor(Rgb16 value, const BuildOptions& options) noexcept
        {
            if (!options.blackFloorEnabled)
                return value;

            const std::uint16_t floor =
                static_cast<std::uint16_t>(
                    static_cast<std::uint16_t>(options.blackFloorRgb) * 257u);

            if (std::max({ value.r, value.g, value.b }) < floor)
                return { floor, floor, floor };

            return value;
        }

        [[nodiscard]] std::filesystem::path StandardSrgbProfilePath()
        {
            DWORD bytes = 0;
            SetLastError(ERROR_SUCCESS);
            (void)GetStandardColorSpaceProfileW(
                nullptr,
                LCS_sRGB,
                nullptr,
                &bytes);

            if (bytes > 0)
            {
                std::vector<wchar_t> buffer(
                    (static_cast<std::size_t>(bytes) + sizeof(wchar_t) - 1) /
                    sizeof(wchar_t));

                DWORD bufferBytes = bytes;

                if (GetStandardColorSpaceProfileW(
                    nullptr,
                    LCS_sRGB,
                    buffer.data(),
                    &bufferBytes))
                {
                    const std::filesystem::path path(buffer.data());
                    if (!path.empty())
                        return path;
                }
            }

            const auto profiles = ProfileCatalog::EnumerateRGB();

            const auto normalize = [](const std::wstring& value)
            {
                std::wstring result;
                result.reserve(value.size());

                for (const wchar_t ch : value)
                {
                    if (std::iswalnum(ch))
                    {
                        result.push_back(
                            static_cast<wchar_t>(
                                std::towupper(ch)));
                    }
                }

                return result;
            };

            for (const auto& profile : profiles)
            {
                const std::wstring name = normalize(profile.displayName);
                const std::wstring file = normalize(profile.path.filename().wstring());

                if (name == L"SRGB" ||
                    name == L"SRGBCOLORSPACEPROFILE" ||
                    file == L"SRGBCOLORSPACEPROFILEICM" ||
                    file == L"SRGBCOLORSPACEPROFILEICC")
                {
                    return profile.path;
                }
            }

            throw std::runtime_error(
                "Unable to resolve the Windows standard sRGB color profile.");
        }

        [[nodiscard]] bool IsUnsupportedProfileClass(DWORD profileClass) noexcept
        {
            return
                profileClass == CLASS_LINK ||
                profileClass == CLASS_NAMED ||
                profileClass == CLASS_ABSTRACT;
        }

        [[nodiscard]] std::vector<std::uint32_t> GridSequence(const BuildOptions& options)
        {
            std::vector<std::uint32_t> grids;

            if (!options.adaptiveGrid)
            {
                grids.push_back(std::max<std::uint32_t>(2, options.preferredGrid));
                return grids;
            }

            const std::uint32_t count = std::min<std::uint32_t>(
                options.adaptiveGridCount,
                static_cast<std::uint32_t>(options.adaptiveGrids.size()));

            for (std::uint32_t i = 0; i < count; ++i)
                grids.push_back(std::max<std::uint32_t>(2, options.adaptiveGrids[i]));

            if (grids.empty())
                grids.push_back(std::max<std::uint32_t>(2, options.preferredGrid));

            std::sort(grids.begin(), grids.end());
            grids.erase(std::unique(grids.begin(), grids.end()), grids.end());
            return grids;
        }

    }

    std::vector<ProfileInfo> ProfileCatalog::EnumerateCMYK()
    {
        return Enumerate(SPACE_CMYK);
    }

    std::vector<ProfileInfo> ProfileCatalog::EnumerateRGB()
    {
        return Enumerate(SPACE_RGB);
    }

    std::vector<ProfileInfo> ProfileCatalog::EnumerateGray()
    {
        return Enumerate(SPACE_GRAY);
    }

    ProfileInfo ProfileCatalog::Inspect(const std::filesystem::path& profilePath)
    {
        IccProfile profile(profilePath);
        return profile.Info();
    }

    std::vector<ProfileInfo> ProfileCatalog::Enumerate(std::uint32_t colorSpace)
    {
        ENUMTYPEW enumeration{};
        enumeration.dwSize = sizeof(enumeration);
        enumeration.dwVersion = ENUM_TYPE_VERSION;
        enumeration.dwFields = ET_DATACOLORSPACE;
        enumeration.dwDataColorSpace = colorSpace;

        DWORD bytes = 0;
        DWORD count = 0;

        SetLastError(ERROR_SUCCESS);
        EnumColorProfilesW(nullptr, &enumeration, nullptr, &bytes, &count);

        if (bytes == 0)
            return {};

        std::vector<std::uint8_t> buffer(bytes);

        if (!EnumColorProfilesW(nullptr, &enumeration, buffer.data(), &bytes, &count))
            throw std::runtime_error(WindowsError("EnumColorProfilesW"));

        std::vector<ProfileInfo> profiles;
        std::set<std::wstring> uniquePaths;

        const wchar_t* current = reinterpret_cast<const wchar_t*>(buffer.data());

        for (DWORD i = 0; i < count && current && *current; ++i)
        {
            const std::filesystem::path path = ResolveProfilePath(current);
            current += std::wcslen(current) + 1;

            const auto key = path.wstring();
            if (!uniquePaths.insert(key).second)
                continue;

            try
            {
                IccProfile profile(path);
                const auto info = profile.Info();

                if (!info.isIcc || info.colorSpace != colorSpace || IsUnsupportedProfileClass(info.profileClass))
                    continue;

                profiles.push_back(info);
            }
            catch (...)
            {
            }
        }

        std::sort(
            profiles.begin(),
            profiles.end(),
            [](const ProfileInfo& a, const ProfileInfo& b)
            {
                return a.displayName < b.displayName;
            });

        return profiles;
    }

    IccProfile::IccProfile(const ProfileInfo& info)
    {
        Open(info.path);
    }

    IccProfile::IccProfile(const std::filesystem::path& path)
    {
        Open(path);
    }

    IccProfile::~IccProfile() noexcept
    {
        Close();
    }

    IccProfile::IccProfile(IccProfile&& other) noexcept
        : info_(std::move(other.info_)),
        handle_(other.handle_)
    {
        other.handle_ = nullptr;
    }

    IccProfile& IccProfile::operator=(IccProfile&& other) noexcept
    {
        if (this == &other)
            return *this;

        Close();
        info_ = std::move(other.info_);
        handle_ = other.handle_;
        other.handle_ = nullptr;
        return *this;
    }

    const ProfileInfo& IccProfile::Info() const noexcept
    {
        return info_;
    }

    void* IccProfile::NativeHandle() const noexcept
    {
        return handle_;
    }

    void IccProfile::Open(const std::filesystem::path& path)
    {
        Close();

        const auto resolved = ResolveProfilePath(path);
        std::wstring pathText = resolved.wstring();

        PROFILE profile{};
        profile.dwType = PROFILE_FILENAME;
        profile.pProfileData = const_cast<wchar_t*>(pathText.c_str());
        profile.cbDataSize = static_cast<DWORD>((pathText.size() + 1) * sizeof(wchar_t));

        HPROFILE handle = WcsOpenColorProfileW(
            &profile,
            nullptr,
            nullptr,
            PROFILE_READ,
            FILE_SHARE_READ,
            OPEN_EXISTING,
            DONT_USE_EMBEDDED_WCS_PROFILES);

        if (!handle)
            throw std::runtime_error(WindowsError("WcsOpenColorProfileW"));

        BOOL valid = FALSE;

        if (!IsColorProfileValid(handle, &valid) || !valid)
        {
            CloseColorProfile(handle);
            throw std::runtime_error("The selected profile is not a valid ICC color profile.");
        }

        handle_ = handle;
        info_.path = resolved;

        try
        {
            ReadMetadata();
        }
        catch (...)
        {
            Close();
            throw;
        }
    }

    void IccProfile::Close() noexcept
    {
        if (!handle_)
            return;

        CloseColorProfile(static_cast<HPROFILE>(handle_));
        handle_ = nullptr;
    }

    void IccProfile::ReadMetadata()
    {
        const auto handle = static_cast<HPROFILE>(handle_);
        PROFILEHEADER header{};

        if (!GetColorProfileHeader(handle, &header))
            throw std::runtime_error(WindowsError("GetColorProfileHeader"));

        const auto raw = ReadProfileBytes(handle);

        info_.displayName = DisplayNameFor(info_.path);
        info_.colorSpace = header.phDataColorSpace;
        info_.profileClass = header.phClass;
        info_.renderingIntent = header.phRenderingIntent;
        info_.fingerprint = HashBytes(raw.data(), raw.size());
        info_.isIcc = IsRawIcc(raw);

        if (!info_.isIcc)
        {
            throw std::runtime_error(
                "The selected profile resolved to a WCS/DMP profile. A pure ICC profile is required for the CMYK pipeline.");
        }

        if (IsUnsupportedProfileClass(header.phClass))
        {
            throw std::runtime_error(
                "The selected ICC profile class is not supported by this CMYK to RGB pipeline.");
        }
    }

    IccTransform::IccTransform(
        const IccProfile& source,
        const IccProfile& target,
        RenderingIntent intent)
    {
        if (source.Info().colorSpace != SPACE_CMYK)
            throw std::invalid_argument("Source ICC profile must use the CMYK data color space.");

        if (target.Info().colorSpace != SPACE_RGB)
            throw std::invalid_argument("Target ICC profile must use the RGB data color space.");

        HPROFILE profiles[2]
        {
            static_cast<HPROFILE>(source.NativeHandle()),
            static_cast<HPROFILE>(target.NativeHandle())
        };

        DWORD intents[1]
        {
            ToWindowsIntent(intent)
        };

        HTRANSFORM transform = CreateMultiProfileTransform(
            profiles,
            2,
            intents,
            1,
            BEST_MODE,
            INDEX_DONT_CARE);

        if (!transform)
            throw std::runtime_error(WindowsError("CreateMultiProfileTransform"));

        handle_ = transform;
    }

    IccTransform::~IccTransform() noexcept
    {
        Close();
    }

    IccTransform::IccTransform(IccTransform&& other) noexcept
        : handle_(other.handle_)
    {
        other.handle_ = nullptr;
    }

    IccTransform& IccTransform::operator=(IccTransform&& other) noexcept
    {
        if (this == &other)
            return *this;

        Close();
        handle_ = other.handle_;
        other.handle_ = nullptr;
        return *this;
    }

    Rgb16 IccTransform::Translate(const Cmyk& color) const
    {
        if (!handle_)
            throw std::runtime_error("ICC transform is not initialized.");

        COLOR source{};
        COLOR target{};

        source.cmyk.cyan = PercentToWord(color.c);
        source.cmyk.magenta = PercentToWord(color.m);
        source.cmyk.yellow = PercentToWord(color.y);
        source.cmyk.black = PercentToWord(color.k);

        if (!TranslateColors(
            static_cast<HTRANSFORM>(handle_),
            &source,
            1,
            COLOR_CMYK,
            &target,
            COLOR_RGB))
        {
            throw std::runtime_error(WindowsError("TranslateColors"));
        }

        return
        {
            target.rgb.red,
            target.rgb.green,
            target.rgb.blue
        };
    }

    void IccTransform::Translate(const Cmyk* input, Rgb16* output, std::size_t count) const
    {
        if (!handle_)
            throw std::runtime_error("ICC transform is not initialized.");

        if ((!input || !output) && count != 0)
            throw std::invalid_argument("Input and output buffers are required.");

        std::size_t offset = 0;

        while (offset < count)
        {
            const std::size_t batch = std::min(count - offset, TranslateBatchLimit);
            std::vector<COLOR> source(batch);
            std::vector<COLOR> target(batch);

            for (std::size_t i = 0; i < batch; ++i)
            {
                const Cmyk& value = input[offset + i];
                source[i].cmyk.cyan = PercentToWord(value.c);
                source[i].cmyk.magenta = PercentToWord(value.m);
                source[i].cmyk.yellow = PercentToWord(value.y);
                source[i].cmyk.black = PercentToWord(value.k);
            }

            if (!TranslateColors(
                static_cast<HTRANSFORM>(handle_),
                source.data(),
                static_cast<DWORD>(batch),
                COLOR_CMYK,
                target.data(),
                COLOR_RGB))
            {
                throw std::runtime_error(WindowsError("TranslateColors"));
            }

            for (std::size_t i = 0; i < batch; ++i)
            {
                output[offset + i] =
                {
                    target[i].rgb.red,
                    target[i].rgb.green,
                    target[i].rgb.blue
                };
            }

            offset += batch;
        }
    }

    void* IccTransform::NativeHandle() const noexcept
    {
        return handle_;
    }

    void IccTransform::Close() noexcept
    {
        if (!handle_)
            return;

        DeleteColorTransform(static_cast<HTRANSFORM>(handle_));
        handle_ = nullptr;
    }

    struct AppearanceTransform::Impl
    {
        struct ScoredCandidate
        {
            Rgb8 rgb;
            cmsCIELab lab{};
            double deltaE = std::numeric_limits<double>::infinity();
            double deltaE76 = std::numeric_limits<double>::infinity();
            double labDistance2 = std::numeric_limits<double>::infinity();
            double chromaAxisError2 = std::numeric_limits<double>::infinity();
            double temperatureError = std::numeric_limits<double>::infinity();
            double primaryError = std::numeric_limits<double>::infinity();
            double secondaryError = std::numeric_limits<double>::infinity();
        };

        struct LabKey
        {
            std::uint64_t l = 0;
            std::uint64_t a = 0;
            std::uint64_t b = 0;

            [[nodiscard]] bool operator==(const LabKey& other) const noexcept
            {
                return l == other.l &&
                    a == other.a &&
                    b == other.b;
            }
        };

        struct CmykKey
        {
            std::uint64_t c = 0;
            std::uint64_t m = 0;
            std::uint64_t y = 0;
            std::uint64_t k = 0;

            [[nodiscard]] bool operator==(const CmykKey& other) const noexcept
            {
                return c == other.c &&
                    m == other.m &&
                    y == other.y &&
                    k == other.k;
            }
        };

        struct LabKeyHasher
        {
            [[nodiscard]] std::size_t operator()(const LabKey& key) const noexcept
            {
                std::uint64_t hash = 14695981039346656037ull;
                Mix(hash, key.l);
                Mix(hash, key.a);
                Mix(hash, key.b);
                return static_cast<std::size_t>(hash);
            }

        private:
            static void Mix(std::uint64_t& hash, std::uint64_t value) noexcept
            {
                constexpr std::uint64_t prime = 1099511628211ull;
                for (int shift = 0; shift < 64; shift += 8)
                {
                    hash ^= (value >> shift) & 0xFFull;
                    hash *= prime;
                }
            }
        };

        struct CmykKeyHasher
        {
            [[nodiscard]] std::size_t operator()(const CmykKey& key) const noexcept
            {
                std::uint64_t hash = 14695981039346656037ull;
                Mix(hash, key.c);
                Mix(hash, key.m);
                Mix(hash, key.y);
                Mix(hash, key.k);
                return static_cast<std::size_t>(hash);
            }

        private:
            static void Mix(std::uint64_t& hash, std::uint64_t value) noexcept
            {
                constexpr std::uint64_t prime = 1099511628211ull;
                for (int shift = 0; shift < 64; shift += 8)
                {
                    hash ^= (value >> shift) & 0xFFull;
                    hash *= prime;
                }
            }
        };

        cmsContext context = nullptr;
        std::vector<std::uint8_t> sourceBytes;
        std::vector<std::uint8_t> targetBytes;
        cmsHPROFILE sourceProfile = nullptr;
        cmsHPROFILE targetProfile = nullptr;
        cmsHPROFILE labProfile = nullptr;
        cmsHTRANSFORM sourceToLab = nullptr;
        cmsHTRANSFORM labToTarget = nullptr;
        cmsHTRANSFORM targetToLab = nullptr;
        std::uint32_t searchRadius = 6;
        bool forceExhaustive = false;
        mutable std::unordered_map<CmykKey, AppearanceMatch, CmykKeyHasher> cache;
        mutable std::unordered_map<LabKey, AppearanceMatch, LabKeyHasher> labCache;

        ~Impl() noexcept
        {
            if (targetToLab)
                cmsDeleteTransform(targetToLab);
            if (labToTarget)
                cmsDeleteTransform(labToTarget);
            if (sourceToLab)
                cmsDeleteTransform(sourceToLab);
            if (labProfile)
                cmsCloseProfile(labProfile);
            if (targetProfile)
                cmsCloseProfile(targetProfile);
            if (sourceProfile)
                cmsCloseProfile(sourceProfile);
            if (context)
                cmsDeleteContext(context);
        }

        [[nodiscard]] bool Ready() const noexcept
        {
            return
                context &&
                sourceProfile &&
                targetProfile &&
                labProfile &&
                sourceToLab &&
                labToTarget &&
                targetToLab;
        }

        [[nodiscard]] static std::uint64_t DoubleBits(double value) noexcept
        {
            if (value == 0.0)
                value = 0.0;

            std::uint64_t bits = 0;
            static_assert(sizeof(bits) == sizeof(value));
            std::memcpy(&bits, &value, sizeof(bits));
            return bits;
        }

        [[nodiscard]] static LabKey MakeLabKey(const Lab& lab) noexcept
        {
            return
            {
                DoubleBits(lab.l),
                DoubleBits(lab.a),
                DoubleBits(lab.b)
            };
        }

        [[nodiscard]] static CmykKey MakeCmykKey(const Cmyk& color) noexcept
        {
            return
            {
                DoubleBits(color.c),
                DoubleBits(color.m),
                DoubleBits(color.y),
                DoubleBits(color.k)
            };
        }

        [[nodiscard]] static double NormalizePercent(double value)
        {
            if (!std::isfinite(value))
            {
                throw std::invalid_argument(
                    "CMYK input contains a non-finite component.");
            }

            return std::clamp(value, 0.0, 100.0);
        }

        [[nodiscard]] static std::uint8_t UnitToByte(double value) noexcept
        {
            if (!std::isfinite(value))
                return 0;

            return static_cast<std::uint8_t>(
                std::clamp(
                    std::lround(
                        std::clamp(value, 0.0, 1.0) * 255.0),
                    0L,
                    255L));
        }

        [[nodiscard]] static std::uint32_t PackRgb(
            const Rgb8& rgb) noexcept
        {
            return
                (static_cast<std::uint32_t>(rgb.r) << 16) |
                (static_cast<std::uint32_t>(rgb.g) << 8) |
                static_cast<std::uint32_t>(rgb.b);
        }

        static void AddCandidate(
            std::vector<Rgb8>& output,
            std::unordered_set<std::uint32_t>& seen,
            int r,
            int g,
            int b)
        {
            const Rgb8 rgb
            {
                static_cast<std::uint8_t>(
                    std::clamp(r, 0, 255)),
                static_cast<std::uint8_t>(
                    std::clamp(g, 0, 255)),
                static_cast<std::uint8_t>(
                    std::clamp(b, 0, 255))
            };

            if (seen.insert(PackRgb(rgb)).second)
                output.push_back(rgb);
        }

        [[nodiscard]] static bool UsesPointExactPriority(
            const cmsCIELab& targetLab) noexcept
        {
            constexpr double pointExactChromaLimit = 16.0;
            const double targetChroma =
                std::hypot(targetLab.a, targetLab.b);

            return
                std::isfinite(targetChroma) &&
                targetChroma <= pointExactChromaLimit;
        }

        [[nodiscard]] static ScoredCandidate BuildScoredCandidate(
            const cmsCIELab& targetLab,
            const Rgb8& rgb,
            const cmsCIELab& lab,
            bool pointExactPriority) noexcept
        {
            ScoredCandidate candidate;
            candidate.rgb = rgb;
            candidate.lab = lab;

            if (!std::isfinite(lab.L) ||
                !std::isfinite(lab.a) ||
                !std::isfinite(lab.b))
            {
                return candidate;
            }

            const double dL = lab.L - targetLab.L;
            const double dA = lab.a - targetLab.a;
            const double dB = lab.b - targetLab.b;

            const double deltaE =
                cmsCIE2000DeltaE(
                    &targetLab,
                    &lab,
                    1.0,
                    1.0,
                    1.0);

            if (!std::isfinite(deltaE))
                return candidate;

            candidate.deltaE = deltaE;
            candidate.labDistance2 =
                dL * dL +
                dA * dA +
                dB * dB;
            candidate.chromaAxisError2 =
                dA * dA +
                dB * dB;
            candidate.temperatureError =
                std::abs(dB);

            if (!std::isfinite(candidate.labDistance2))
                candidate.labDistance2 =
                    std::numeric_limits<double>::infinity();

            candidate.deltaE76 =
                std::sqrt(candidate.labDistance2);

            if (!std::isfinite(candidate.deltaE76))
                candidate.deltaE76 =
                    std::numeric_limits<double>::infinity();

            if (!std::isfinite(candidate.chromaAxisError2))
                candidate.chromaAxisError2 =
                    std::numeric_limits<double>::infinity();

            if (!std::isfinite(candidate.temperatureError))
                candidate.temperatureError =
                    std::numeric_limits<double>::infinity();

            candidate.primaryError =
                pointExactPriority
                ? candidate.deltaE76
                : candidate.deltaE;
            candidate.secondaryError =
                pointExactPriority
                ? candidate.deltaE
                : candidate.deltaE76;

            return candidate;
        }

        [[nodiscard]] static bool Better(
            const ScoredCandidate& left,
            const ScoredCandidate& right) noexcept
        {
            if (left.primaryError != right.primaryError)
                return left.primaryError < right.primaryError;

            if (left.secondaryError != right.secondaryError)
                return left.secondaryError < right.secondaryError;

            if (left.chromaAxisError2 != right.chromaAxisError2)
                return left.chromaAxisError2 < right.chromaAxisError2;

            if (left.temperatureError != right.temperatureError)
                return left.temperatureError < right.temperatureError;

            return PackRgb(left.rgb) < PackRgb(right.rgb);
        }

        [[nodiscard]] std::vector<ScoredCandidate> ScoreCandidates(
            const cmsCIELab& targetLab,
            const std::vector<Rgb8>& candidates,
            std::size_t keep) const
        {
            if (candidates.empty() || keep == 0)
                return {};

            const bool pointExactPriority =
                UsesPointExactPriority(targetLab);

            constexpr std::size_t batchLimit = 65536;
            std::vector<ScoredCandidate> best;
            best.reserve(keep);

            std::size_t offset = 0;

            while (offset < candidates.size())
            {
                Cancellation::ThrowIfRequested();

                const std::size_t batch =
                    std::min(
                        batchLimit,
                        candidates.size() - offset);

                std::vector<std::array<double, 3>> input(batch);
                std::vector<cmsCIELab> labs(batch);

                for (std::size_t i = 0;
                    i < batch;
                    ++i)
                {
                    const auto& rgb =
                        candidates[offset + i];

                    input[i] =
                    {
                        static_cast<double>(rgb.r) / 255.0,
                        static_cast<double>(rgb.g) / 255.0,
                        static_cast<double>(rgb.b) / 255.0
                    };
                }

                cmsDoTransform(
                    targetToLab,
                    input.data(),
                    labs.data(),
                    static_cast<cmsUInt32Number>(batch));

                std::vector<ScoredCandidate> scored;
                scored.reserve(best.size() + batch);
                scored.insert(
                    scored.end(),
                    best.begin(),
                    best.end());

                for (std::size_t i = 0;
                    i < batch;
                    ++i)
                {
                    scored.push_back(
                        BuildScoredCandidate(
                            targetLab,
                            candidates[offset + i],
                            labs[i],
                            pointExactPriority));
                }

                const std::size_t retained =
                    std::min(keep, scored.size());

                if (scored.size() > retained)
                {
                    std::nth_element(
                        scored.begin(),
                        scored.begin() +
                            static_cast<std::ptrdiff_t>(retained),
                        scored.end(),
                        Better);
                    scored.resize(retained);
                }

                std::sort(
                    scored.begin(),
                    scored.end(),
                    Better);

                best = std::move(scored);
                offset += batch;
            }

            return best;
        }

        [[nodiscard]] static std::vector<int> AxisValues(int step)
        {
            step = std::clamp(step, 1, 255);

            std::vector<int> values;
            for (int value = 0;
                value <= 255;
                value += step)
            {
                values.push_back(value);
            }

            if (values.empty() ||
                values.back() != 255)
            {
                values.push_back(255);
            }

            return values;
        }

        [[nodiscard]] static std::vector<Rgb8> FullGrid(
            int step,
            const std::vector<ScoredCandidate>& extra = {})
        {
            const auto axis = AxisValues(step);

            std::vector<Rgb8> output;
            output.reserve(
                axis.size() *
                axis.size() *
                axis.size() +
                extra.size());

            for (const int r : axis)
            {
                Cancellation::ThrowIfRequested();

                for (const int g : axis)
                {
                    for (const int b : axis)
                    {
                        output.push_back(
                            {
                                static_cast<std::uint8_t>(r),
                                static_cast<std::uint8_t>(g),
                                static_cast<std::uint8_t>(b)
                            });
                    }
                }
            }

            for (const auto& candidate : extra)
                output.push_back(candidate.rgb);

            return output;
        }

        [[nodiscard]] static std::vector<Rgb8> RefineAround(
            const std::vector<ScoredCandidate>& seeds,
            int step,
            int radiusSteps)
        {
            std::vector<Rgb8> output;
            std::unordered_set<std::uint32_t> seen;

            const int width =
                radiusSteps * 2 + 1;
            output.reserve(
                seeds.size() *
                static_cast<std::size_t>(
                    width * width * width));
            seen.reserve(output.capacity() * 2 + 1);

            for (const auto& seed : seeds)
            {
                Cancellation::ThrowIfRequested();

                for (int dr = -radiusSteps;
                    dr <= radiusSteps;
                    ++dr)
                {
                    for (int dg = -radiusSteps;
                        dg <= radiusSteps;
                        ++dg)
                    {
                        for (int db = -radiusSteps;
                            db <= radiusSteps;
                            ++db)
                        {
                            AddCandidate(
                                output,
                                seen,
                                static_cast<int>(seed.rgb.r) +
                                    dr * step,
                                static_cast<int>(seed.rgb.g) +
                                    dg * step,
                                static_cast<int>(seed.rgb.b) +
                                    db * step);
                        }
                    }
                }
            }

            return output;
        }

        [[nodiscard]] static AppearanceMatch MakeMatch(
            const cmsCIELab& targetLab,
            const ScoredCandidate& candidate,
            bool usedGlobalSearch,
            bool usedDenseSearch,
            bool usedExhaustiveSearch)
        {
            AppearanceMatch result;
            result.rgb = candidate.rgb;
            result.targetLab =
            {
                targetLab.L,
                targetLab.a,
                targetLab.b
            };
            result.resultingLab =
            {
                candidate.lab.L,
                candidate.lab.a,
                candidate.lab.b
            };
            result.deltaE00 = candidate.deltaE;
            result.deltaE76 = candidate.deltaE76;
            result.deltaL = candidate.lab.L - targetLab.L;
            result.deltaA = candidate.lab.a - targetLab.a;
            result.deltaB = candidate.lab.b - targetLab.b;
            result.usedGlobalSearch = usedGlobalSearch;
            result.usedDenseSearch = usedDenseSearch;
            result.usedExhaustiveSearch = usedExhaustiveSearch;
            return result;
        }

        [[nodiscard]] ScoredCandidate ExhaustiveRgb8(
            const cmsCIELab& targetLab) const
        {
            constexpr std::size_t slabSize = 256u * 256u;

            std::vector<std::array<double, 3>> input(slabSize);
            std::vector<cmsCIELab> labs(slabSize);
            const bool pointExactPriority =
                UsesPointExactPriority(targetLab);

            for (int g = 0; g <= 255; ++g)
            {
                for (int b = 0; b <= 255; ++b)
                {
                    const std::size_t index =
                        static_cast<std::size_t>(g) * 256u +
                        static_cast<std::size_t>(b);

                    input[index][1] =
                        static_cast<double>(g) / 255.0;
                    input[index][2] =
                        static_cast<double>(b) / 255.0;
                }
            }

            ScoredCandidate globalBest;
            bool haveBest = false;

            for (int r = 0; r <= 255; ++r)
            {
                Cancellation::PumpPaintAndThrow();
                const double normalizedR =
                    static_cast<double>(r) / 255.0;

                for (auto& value : input)
                    value[0] = normalizedR;

                cmsDoTransform(
                    targetToLab,
                    input.data(),
                    labs.data(),
                    static_cast<cmsUInt32Number>(slabSize));

                for (int g = 0; g <= 255; ++g)
                {
                    for (int b = 0; b <= 255; ++b)
                    {
                        const std::size_t index =
                            static_cast<std::size_t>(g) * 256u +
                            static_cast<std::size_t>(b);

                        const ScoredCandidate candidate =
                            BuildScoredCandidate(
                                targetLab,
                                {
                                    static_cast<std::uint8_t>(r),
                                    static_cast<std::uint8_t>(g),
                                    static_cast<std::uint8_t>(b)
                                },
                                labs[index],
                                pointExactPriority);

                        if (!haveBest ||
                            Better(candidate, globalBest))
                        {
                            globalBest = candidate;
                            haveBest = true;
                        }
                    }
                }
            }

            if (!haveBest ||
                !std::isfinite(globalBest.deltaE))
            {
                throw std::runtime_error(
                    "Unable to exhaustively evaluate the RGB8 destination space.");
            }

            return globalBest;
        }

        [[nodiscard]] AppearanceMatch OptimizeDetailed(
            const cmsCIELab& targetLab,
            const double directRgb[3]) const
        {
            if (!std::isfinite(targetLab.L) ||
                !std::isfinite(targetLab.a) ||
                !std::isfinite(targetLab.b))
            {
                throw std::runtime_error(
                    "The source ICC profile produced an invalid Lab color.");
            }

            const Rgb8 initial
            {
                UnitToByte(directRgb[0]),
                UnitToByte(directRgb[1]),
                UnitToByte(directRgb[2])
            };

            const int finalRadius =
                static_cast<int>(
                    std::clamp<std::uint32_t>(
                        searchRadius,
                        2u,
                        12u));

            ScoredCandidate initialSeed;
            initialSeed.rgb = initial;

            std::vector<ScoredCandidate> seeds
            {
                initialSeed
            };

            auto localCandidates =
                RefineAround(
                    seeds,
                    1,
                    finalRadius);

            seeds =
                ScoreCandidates(
                    targetLab,
                    localCandidates,
                    32);

            if (seeds.empty() ||
                !std::isfinite(seeds.front().deltaE))
            {
                throw std::runtime_error(
                    "Unable to evaluate RGB candidates for appearance preservation.");
            }

            constexpr double exactZero = 1e-12;

            if (forceExhaustive &&
                seeds.front().primaryError > exactZero)
            {
                const auto exactBest =
                    ExhaustiveRgb8(targetLab);

                return MakeMatch(
                    targetLab,
                    exactBest,
                    true,
                    true,
                    true);
            }

            if (seeds.front().primaryError <= exactZero)
            {
                return MakeMatch(
                    targetLab,
                    seeds.front(),
                    false,
                    false,
                    false);
            }

            bool usedGlobalSearch = true;
            bool usedDenseSearch = false;
            bool usedExhaustiveSearch = false;

            seeds =
                ScoreCandidates(
                    targetLab,
                    FullGrid(16, seeds),
                    48);

            constexpr int refinementSteps[]
            {
                8,
                4,
                2,
                1
            };

            for (const int step : refinementSteps)
            {
                Cancellation::ThrowIfRequested();
                seeds =
                    ScoreCandidates(
                        targetLab,
                        RefineAround(
                            seeds,
                            step,
                            2),
                        48);
            }

            if (seeds.empty())
            {
                throw std::runtime_error(
                    "Global RGB appearance search produced no candidates.");
            }

            if (seeds.front().primaryError > 0.10)
            {
                usedDenseSearch = true;

                seeds =
                    ScoreCandidates(
                        targetLab,
                        FullGrid(4, seeds),
                        64);

                for (const int step : { 2, 1 })
                {
                    Cancellation::ThrowIfRequested();
                    seeds =
                        ScoreCandidates(
                            targetLab,
                            RefineAround(
                                seeds,
                                step,
                                2),
                            64);
                }
            }

            if (seeds.empty())
            {
                throw std::runtime_error(
                    "Dense RGB appearance search produced no candidates.");
            }

            if (forceExhaustive)
            {
                usedDenseSearch = true;
                usedExhaustiveSearch = true;
                seeds.front() =
                    ExhaustiveRgb8(targetLab);
            }
            else
            {
                const std::size_t finalSeedCount =
                    std::min<std::size_t>(
                        seeds.size(),
                        16);

                std::vector<ScoredCandidate> finalSeeds(
                    seeds.begin(),
                    seeds.begin() +
                        static_cast<std::ptrdiff_t>(
                            finalSeedCount));

                const auto finalBest =
                    ScoreCandidates(
                        targetLab,
                        RefineAround(
                            finalSeeds,
                            1,
                            finalRadius),
                        1);

                if (!finalBest.empty() &&
                    Better(
                        finalBest.front(),
                        seeds.front()))
                {
                    seeds.front() =
                        finalBest.front();
                }
            }

            return MakeMatch(
                targetLab,
                seeds.front(),
                usedGlobalSearch,
                usedDenseSearch,
                usedExhaustiveSearch);
        }

        [[nodiscard]] AppearanceMatch TranslateLabDetailed(
            const Lab& color) const
        {
            Lab normalized = color;

            if (!std::isfinite(normalized.l) ||
                !std::isfinite(normalized.a) ||
                !std::isfinite(normalized.b))
            {
                throw std::invalid_argument(
                    "Lab input contains a non-finite component.");
            }

            normalized.l =
                std::clamp(
                    normalized.l,
                    0.0,
                    100.0);

            const LabKey key =
                MakeLabKey(normalized);

            const auto found =
                labCache.find(key);

            if (found != labCache.end())
                return found->second;

            const cmsCIELab targetLab
            {
                normalized.l,
                normalized.a,
                normalized.b
            };

            double directRgb[3]{};

            cmsDoTransform(
                labToTarget,
                &targetLab,
                directRgb,
                1);

            const AppearanceMatch result =
                OptimizeDetailed(
                    targetLab,
                    directRgb);

            if (labCache.size() >= 32768)
                labCache.clear();

            labCache.emplace(
                key,
                result);

            return result;
        }

        [[nodiscard]] AppearanceMatch TranslateDetailed(
            const Cmyk& color) const
        {
            const Cmyk normalized
            {
                NormalizePercent(color.c),
                NormalizePercent(color.m),
                NormalizePercent(color.y),
                NormalizePercent(color.k)
            };

            const CmykKey key =
                MakeCmykKey(normalized);

            const auto found =
                cache.find(key);

            if (found != cache.end())
                return found->second;

            const double cmyk[4]
            {
                normalized.c,
                normalized.m,
                normalized.y,
                normalized.k
            };

            cmsCIELab targetLab{};

            cmsDoTransform(
                sourceToLab,
                cmyk,
                &targetLab,
                1);

            const AppearanceMatch result =
                TranslateLabDetailed(
                    Lab
                    {
                        targetLab.L,
                        targetLab.a,
                        targetLab.b
                    });

            if (cache.size() >= 32768)
                cache.clear();

            cache.emplace(
                key,
                result);

            return result;
        }

        [[nodiscard]] Rgb8 TranslateLab(
            const Lab& color) const
        {
            return TranslateLabDetailed(color).rgb;
        }

        [[nodiscard]] Rgb8 Translate(
            const Cmyk& color) const
        {
            return TranslateDetailed(color).rgb;
        }
    };

    AppearanceTransform::AppearanceTransform(
        const IccProfile& source,
        const IccProfile& target,
        std::uint32_t searchRadius,
        bool forceExhaustive)
        : impl_(std::make_unique<Impl>())
    {
        impl_->searchRadius =
            std::clamp<std::uint32_t>(
                searchRadius,
                2u,
                12u);
        impl_->forceExhaustive =
            forceExhaustive;


        const auto sourceHandle =
            static_cast<HPROFILE>(
                source.NativeHandle());
        const auto targetHandle =
            static_cast<HPROFILE>(
                target.NativeHandle());

        if (!sourceHandle || !targetHandle)
        {
            impl_.reset();
            throw std::runtime_error(
                "The selected ICC profiles are not open.");
        }

        impl_->sourceBytes =
            ReadProfileBytes(
                sourceHandle);
        impl_->targetBytes =
            ReadProfileBytes(
                targetHandle);

        impl_->context =
            cmsCreateContext(
                nullptr,
                nullptr);

        if (!impl_->context)
        {
            impl_.reset();
            throw std::runtime_error(
                "LittleCMS could not create an isolated appearance context.");
        }

        cmsSetAdaptationStateTHR(
            impl_->context,
            0.0);

        impl_->sourceProfile =
            cmsOpenProfileFromMemTHR(
                impl_->context,
                impl_->sourceBytes.data(),
                static_cast<cmsUInt32Number>(
                    impl_->sourceBytes.size()));
        impl_->targetProfile =
            cmsOpenProfileFromMemTHR(
                impl_->context,
                impl_->targetBytes.data(),
                static_cast<cmsUInt32Number>(
                    impl_->targetBytes.size()));
        impl_->labProfile =
            cmsCreateLab4ProfileTHR(
                impl_->context,
                nullptr);

        if (!impl_->sourceProfile ||
            !impl_->targetProfile ||
            !impl_->labProfile)
        {
            impl_.reset();
            throw std::runtime_error(
                "LittleCMS could not open the selected ICC profiles.");
        }

        constexpr cmsUInt32Number intent = INTENT_ABSOLUTE_COLORIMETRIC;
        constexpr cmsUInt32Number flags =
            cmsFLAGS_NOOPTIMIZE |
            cmsFLAGS_NOCACHE;

        impl_->sourceToLab =
            cmsCreateTransformTHR(
                impl_->context,
                impl_->sourceProfile,
                TYPE_CMYK_DBL,
                impl_->labProfile,
                TYPE_Lab_DBL,
                intent,
                flags);

        impl_->labToTarget =
            cmsCreateTransformTHR(
                impl_->context,
                impl_->labProfile,
                TYPE_Lab_DBL,
                impl_->targetProfile,
                TYPE_RGB_DBL,
                intent,
                flags);

        impl_->targetToLab =
            cmsCreateTransformTHR(
                impl_->context,
                impl_->targetProfile,
                TYPE_RGB_DBL,
                impl_->labProfile,
                TYPE_Lab_DBL,
                intent,
                flags);

        if (!impl_->Ready())
        {
            impl_.reset();
            throw std::runtime_error(
                "LittleCMS could not build the Lab appearance-preserving transforms.");
        }
    }

    AppearanceTransform::~AppearanceTransform() noexcept = default;

    AppearanceTransform::AppearanceTransform(
        AppearanceTransform&& other) noexcept = default;

    AppearanceTransform& AppearanceTransform::operator=(
        AppearanceTransform&& other) noexcept = default;

    bool AppearanceTransform::Ready() const noexcept
    {
        return impl_ && impl_->Ready();
    }

    Rgb8 AppearanceTransform::Translate(
        const Cmyk& color) const
    {
        if (!Ready())
        {
            throw std::runtime_error(
                "Appearance-preserving transform is not initialized.");
        }

        return impl_->Translate(color);
    }

    Rgb8 AppearanceTransform::TranslateLab(
        const Lab& color) const
    {
        if (!Ready())
        {
            throw std::runtime_error(
                "Appearance-preserving Lab transform is not initialized.");
        }

        return impl_->TranslateLab(color);
    }

    AppearanceMatch AppearanceTransform::TranslateDetailed(
        const Cmyk& color) const
    {
        if (!Ready())
        {
            throw std::runtime_error(
                "Appearance-preserving transform is not initialized.");
        }

        return impl_->TranslateDetailed(color);
    }

    AppearanceMatch AppearanceTransform::TranslateLabDetailed(
        const Lab& color) const
    {
        if (!Ready())
        {
            throw std::runtime_error(
                "Appearance-preserving Lab transform is not initialized.");
        }

        return impl_->TranslateLabDetailed(color);
    }

    RgbToSrgbConverter::~RgbToSrgbConverter() noexcept
    {
        Reset();
    }

    RgbToSrgbConverter::RgbToSrgbConverter(
        RgbToSrgbConverter&& other) noexcept
        : source_(std::move(other.source_)),
        srgb_(std::move(other.srgb_)),
        transform_(other.transform_)
    {
        other.transform_ = nullptr;
    }

    RgbToSrgbConverter& RgbToSrgbConverter::operator=(
        RgbToSrgbConverter&& other) noexcept
    {
        if (this == &other)
            return *this;

        Reset();
        source_ = std::move(other.source_);
        srgb_ = std::move(other.srgb_);
        transform_ = other.transform_;
        other.transform_ = nullptr;
        return *this;
    }

    void RgbToSrgbConverter::Initialize(
        const ProfileInfo& sourceRgb,
        RenderingIntent intent)
    {
        Reset();

        auto source = std::make_unique<IccProfile>(sourceRgb);
        auto srgb = std::make_unique<IccProfile>(StandardSrgbProfilePath());

        if (source->Info().colorSpace != SPACE_RGB)
            throw std::invalid_argument("Source ICC profile must use the RGB data color space.");

        if (srgb->Info().colorSpace != SPACE_RGB)
            throw std::runtime_error("The Windows standard sRGB profile is not an RGB ICC profile.");

        HPROFILE profiles[2]
        {
            static_cast<HPROFILE>(source->NativeHandle()),
            static_cast<HPROFILE>(srgb->NativeHandle())
        };

        DWORD intents[1]
        {
            ToWindowsIntent(intent)
        };

        HTRANSFORM transform = CreateMultiProfileTransform(
            profiles,
            2,
            intents,
            1,
            BEST_MODE,
            INDEX_DONT_CARE);

        if (!transform)
            throw std::runtime_error(WindowsError("CreateMultiProfileTransform"));

        source_ = std::move(source);
        srgb_ = std::move(srgb);
        transform_ = transform;
    }

    void RgbToSrgbConverter::Reset() noexcept
    {
        CloseTransform();
        srgb_.reset();
        source_.reset();
    }

    bool RgbToSrgbConverter::Ready() const noexcept
    {
        return transform_ != nullptr && source_ && srgb_;
    }

    Rgb8 RgbToSrgbConverter::Convert(const Rgb8& color) const
    {
        if (!Ready())
            throw std::runtime_error("RGB-to-sRGB transform is not initialized.");

        COLOR source{};
        COLOR target{};

        source.rgb.red =
            static_cast<std::uint16_t>(
                static_cast<std::uint16_t>(color.r) * 257u);
        source.rgb.green =
            static_cast<std::uint16_t>(
                static_cast<std::uint16_t>(color.g) * 257u);
        source.rgb.blue =
            static_cast<std::uint16_t>(
                static_cast<std::uint16_t>(color.b) * 257u);

        if (!TranslateColors(
            static_cast<HTRANSFORM>(transform_),
            &source,
            1,
            COLOR_RGB,
            &target,
            COLOR_RGB))
        {
            throw std::runtime_error(WindowsError("TranslateColors"));
        }

        return
        {
            WordToByte(target.rgb.red),
            WordToByte(target.rgb.green),
            WordToByte(target.rgb.blue)
        };
    }

    const ProfileInfo& RgbToSrgbConverter::SourceProfile() const
    {
        if (!source_)
            throw std::runtime_error("RGB-to-sRGB source profile is not initialized.");

        return source_->Info();
    }

    const ProfileInfo& RgbToSrgbConverter::SrgbProfile() const
    {
        if (!srgb_)
            throw std::runtime_error("sRGB profile is not initialized.");

        return srgb_->Info();
    }

    void RgbToSrgbConverter::CloseTransform() noexcept
    {
        if (!transform_)
            return;

        DeleteColorTransform(
            static_cast<HTRANSFORM>(transform_));

        transform_ = nullptr;
    }

    void IccLut4D::Build(
        const IccTransform& transform,
        std::uint32_t gridSize,
        std::uint32_t chunkSize)
    {
        if (gridSize < 2)
            throw std::invalid_argument("LUT grid size must be at least 2.");

        gridSize_ = gridSize;

        const std::uint64_t n = gridSize_;
        const std::uint64_t count64 = n * n * n * n;

        if (count64 > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
            throw std::overflow_error("LUT is too large for this process.");

        data_.assign(static_cast<std::size_t>(count64), {});

        const std::size_t chunk = std::max<std::size_t>(1, chunkSize);
        std::vector<Cmyk> input;
        std::vector<Rgb16> output(chunk);

        input.reserve(chunk);
        std::size_t writeOffset = 0;

        const auto flush = [&]()
            {
                if (input.empty())
                    return;

                transform.Translate(input.data(), output.data(), input.size());
                std::copy_n(output.data(), input.size(), data_.data() + writeOffset);
                writeOffset += input.size();
                input.clear();
            };

        const double denominator = static_cast<double>(gridSize_ - 1);

        for (std::uint32_t c = 0; c < gridSize_; ++c)
        {
            Cancellation::PumpPaintAndThrow();
            const double cyan = 100.0 * static_cast<double>(c) / denominator;

            for (std::uint32_t m = 0; m < gridSize_; ++m)
            {
                const double magenta = 100.0 * static_cast<double>(m) / denominator;

                for (std::uint32_t y = 0; y < gridSize_; ++y)
                {
                    const double yellow = 100.0 * static_cast<double>(y) / denominator;

                    for (std::uint32_t k = 0; k < gridSize_; ++k)
                    {
                        const double black = 100.0 * static_cast<double>(k) / denominator;
                        input.push_back({ cyan, magenta, yellow, black });

                        if (input.size() >= chunk)
                            flush();
                    }
                }
            }
        }

        flush();
    }

    bool IccLut4D::Load(
        const std::filesystem::path& file,
        std::uint64_t sourceFingerprint,
        std::uint64_t targetFingerprint,
        RenderingIntent intent,
        std::uint32_t gridSize)
    {
        std::ifstream stream(file, std::ios::binary);

        if (!stream)
            return false;

        CacheHeader header{};
        stream.read(reinterpret_cast<char*>(&header), sizeof(header));

        if (!stream)
            return false;

        if (!std::equal(CacheMagic.begin(), CacheMagic.end(), header.magic))
            return false;

        if (header.version != CacheVersion ||
            header.gridSize != gridSize ||
            header.intent != static_cast<std::uint32_t>(intent) ||
            header.storageBits != 16 ||
            header.sourceFingerprint != sourceFingerprint ||
            header.targetFingerprint != targetFingerprint)
        {
            return false;
        }

        const std::uint64_t n = gridSize;
        const std::uint64_t expectedCount = n * n * n * n;

        if (header.entryCount != expectedCount)
            return false;

        if (expectedCount > static_cast<std::uint64_t>(std::numeric_limits<std::size_t>::max()))
            return false;

        std::vector<Rgb16> candidate(static_cast<std::size_t>(expectedCount));

        stream.read(
            reinterpret_cast<char*>(candidate.data()),
            static_cast<std::streamsize>(candidate.size() * sizeof(Rgb16)));

        if (!stream)
            return false;

        const auto payloadHash = HashBytes(candidate.data(), candidate.size() * sizeof(Rgb16));

        if (payloadHash != header.payloadHash)
            return false;

        gridSize_ = gridSize;
        data_ = std::move(candidate);
        return true;
    }

    bool IccLut4D::Save(
        const std::filesystem::path& file,
        std::uint64_t sourceFingerprint,
        std::uint64_t targetFingerprint,
        RenderingIntent intent) const
    {
        if (Empty())
            return false;

        std::error_code ec;
        const auto parent = file.parent_path();

        if (!parent.empty())
        {
            std::filesystem::create_directories(parent, ec);
            if (ec)
                return false;
        }

        CacheHeader header{};
        std::copy(CacheMagic.begin(), CacheMagic.end(), header.magic);
        header.version = CacheVersion;
        header.gridSize = gridSize_;
        header.intent = static_cast<std::uint32_t>(intent);
        header.storageBits = 16;
        header.sourceFingerprint = sourceFingerprint;
        header.targetFingerprint = targetFingerprint;
        header.entryCount = data_.size();
        header.payloadHash = HashBytes(data_.data(), data_.size() * sizeof(Rgb16));

        const std::filesystem::path temporary(
            file.wstring() +
            L".tmp." +
            std::to_wstring(GetCurrentProcessId()) +
            L"." +
            std::to_wstring(GetCurrentThreadId()));

        std::ofstream stream(temporary, std::ios::binary | std::ios::trunc);

        if (!stream)
            return false;

        stream.write(reinterpret_cast<const char*>(&header), sizeof(header));
        stream.write(
            reinterpret_cast<const char*>(data_.data()),
            static_cast<std::streamsize>(data_.size() * sizeof(Rgb16)));
        stream.close();

        if (!stream)
        {
            std::filesystem::remove(temporary, ec);
            return false;
        }

        const std::wstring temporaryText = temporary.wstring();
        const std::wstring fileText = file.wstring();

        if (!MoveFileExW(
            temporaryText.c_str(),
            fileText.c_str(),
            MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        {
            std::filesystem::remove(temporary, ec);
            return false;
        }

        return true;
    }

    Rgb16 IccLut4D::Sample16(const Cmyk& color, InterpolationMode mode) const
    {
        if (Empty())
            throw std::runtime_error("ICC LUT is not initialized.");

        const Axis c = ResolveAxis(color.c);
        const Axis m = ResolveAxis(color.m);
        const Axis y = ResolveAxis(color.y);
        const Axis k = ResolveAxis(color.k);

        if (mode == InterpolationMode::Simplex4D)
            return SampleSimplex(c, m, y, k);

        return SampleMultilinear(c, m, y, k);
    }

    Rgb8 IccLut4D::Sample8(const Cmyk& color, InterpolationMode mode) const
    {
        const auto value = Sample16(color, mode);
        return { WordToByte(value.r), WordToByte(value.g), WordToByte(value.b) };
    }

    std::uint32_t IccLut4D::GridSize() const noexcept
    {
        return gridSize_;
    }

    std::size_t IccLut4D::EntryCount() const noexcept
    {
        return data_.size();
    }

    bool IccLut4D::Empty() const noexcept
    {
        return gridSize_ < 2 || data_.empty();
    }

    void IccLut4D::Clear() noexcept
    {
        gridSize_ = 0;
        data_.clear();
    }

    IccLut4D::Axis IccLut4D::ResolveAxis(double percent) const noexcept
    {
        const double value = std::clamp(percent, 0.0, 100.0);
        const double scaled = value * static_cast<double>(gridSize_ - 1) / 100.0;
        const std::uint32_t low = static_cast<std::uint32_t>(std::floor(scaled));

        if (low >= gridSize_ - 1)
            return { gridSize_ - 1, gridSize_ - 1, 0.0 };

        return { low, low + 1, scaled - static_cast<double>(low) };
    }

    std::size_t IccLut4D::Offset(
        std::uint32_t c,
        std::uint32_t m,
        std::uint32_t y,
        std::uint32_t k) const noexcept
    {
        const std::size_t n = gridSize_;
        return (((static_cast<std::size_t>(c) * n + m) * n + y) * n + k);
    }

    Rgb16 IccLut4D::SampleMultilinear(
        const Axis& c,
        const Axis& m,
        const Axis& y,
        const Axis& k) const
    {
        const Axis axes[4]{ c, m, y, k };
        double red = 0.0;
        double green = 0.0;
        double blue = 0.0;

        for (std::uint32_t vertex = 0; vertex < 16; ++vertex)
        {
            std::uint32_t index[4]{};
            double weight = 1.0;

            for (std::uint32_t axis = 0; axis < 4; ++axis)
            {
                const bool high = (vertex & (1u << (3u - axis))) != 0;
                index[axis] = high ? axes[axis].high : axes[axis].low;
                weight *= high ? axes[axis].fraction : 1.0 - axes[axis].fraction;
            }

            if (weight <= 0.0)
                continue;

            const auto& value = data_[Offset(index[0], index[1], index[2], index[3])];
            red += static_cast<double>(value.r) * weight;
            green += static_cast<double>(value.g) * weight;
            blue += static_cast<double>(value.b) * weight;
        }

        return { ClampWord(red), ClampWord(green), ClampWord(blue) };
    }

    Rgb16 IccLut4D::SampleSimplex(
        const Axis& c,
        const Axis& m,
        const Axis& y,
        const Axis& k) const
    {
        const Axis axes[4]{ c, m, y, k };

        struct RankedAxis
        {
            std::uint32_t dimension;
            double fraction;
        };

        std::array<RankedAxis, 4> rank
        { {
            { 0, c.fraction },
            { 1, m.fraction },
            { 2, y.fraction },
            { 3, k.fraction }
        } };

        std::sort(
            rank.begin(),
            rank.end(),
            [](const RankedAxis& a, const RankedAxis& b)
            {
                return a.fraction > b.fraction;
            });

        const double weights[5]
        {
            1.0 - rank[0].fraction,
            rank[0].fraction - rank[1].fraction,
            rank[1].fraction - rank[2].fraction,
            rank[2].fraction - rank[3].fraction,
            rank[3].fraction
        };

        std::uint32_t index[4]
        {
            axes[0].low,
            axes[1].low,
            axes[2].low,
            axes[3].low
        };

        double red = 0.0;
        double green = 0.0;
        double blue = 0.0;

        const auto accumulate = [&](double weight)
            {
                if (weight <= 0.0)
                    return;

                const auto& value = data_[Offset(index[0], index[1], index[2], index[3])];
                red += static_cast<double>(value.r) * weight;
                green += static_cast<double>(value.g) * weight;
                blue += static_cast<double>(value.b) * weight;
            };

        accumulate(weights[0]);

        for (std::uint32_t step = 0; step < 4; ++step)
        {
            const std::uint32_t dimension = rank[step].dimension;
            index[dimension] = axes[dimension].high;
            accumulate(weights[step + 1]);
        }

        return { ClampWord(red), ClampWord(green), ClampWord(blue) };
    }

    void Converter::Initialize(
        const ProfileInfo& sourceCmyk,
        const ProfileInfo& targetRgb,
        const BuildOptions& options)
    {
        Reset();
        options_ = options;

        if (options_.conversionMode ==
            ConversionMode::PreserveAppearance)
        {
            options_.intent =
                RenderingIntent::AbsoluteColorimetric;

            options_.blackFloorEnabled = false;
        }

        source_ = std::make_unique<IccProfile>(sourceCmyk);
        target_ = std::make_unique<IccProfile>(targetRgb);

        if (source_->Info().colorSpace != SPACE_CMYK)
        {
            Reset();
            throw std::invalid_argument("Source profile is not CMYK.");
        }

        if (target_->Info().colorSpace != SPACE_RGB)
        {
            Reset();
            throw std::invalid_argument("Target profile is not RGB.");
        }

        transform_ = std::make_unique<IccTransform>(*source_, *target_, options_.intent);

        if (options_.conversionMode ==
                ConversionMode::PreserveAppearance ||
            options_.enableLabTargetTransform)
        {
            appearanceTransform_ =
                std::make_unique<AppearanceTransform>(
                    *source_,
                    *target_,
                    options_.appearanceSearchRadius,
                    options_.appearanceExhaustive);
        }

        if (options_.conversionMode ==
            ConversionMode::PreserveAppearance)
        {
            loadedFromCache_ = false;
            directFallback_ = false;
            return;
        }

        const auto grids = GridSequence(options_);
        const std::uint32_t samples = std::max<std::uint32_t>(1, options_.validationSamples);

        IccLut4D lastCandidate;
        ValidationStats lastStats{};
        bool lastFromCache = false;

        for (const std::uint32_t grid : grids)
        {
            Cancellation::PumpPaintAndThrow();
            IccLut4D candidate;
            bool fromCache = false;

            if (options_.enableDiskCache)
            {
                fromCache = candidate.Load(
                    CacheFileFor(grid),
                    source_->Info().fingerprint,
                    target_->Info().fingerprint,
                    options_.intent,
                    grid);
            }

            if (!fromCache)
            {
                candidate.Build(*transform_, grid, options_.buildChunkSize);

                if (options_.enableDiskCache)
                {
                    (void)candidate.Save(
                        CacheFileFor(grid),
                        source_->Info().fingerprint,
                        target_->Info().fingerprint,
                        options_.intent);
                }
            }

            const auto stats = ValidateLut(candidate, samples);

            if (MeetsQualityTarget(stats, options_))
            {
                lut_ = std::move(candidate);
                lastValidation_ = stats;
                loadedFromCache_ = fromCache;
                directFallback_ = false;
                return;
            }

            lastCandidate = std::move(candidate);
            lastStats = stats;
            lastFromCache = fromCache;
        }

        if (options_.fallbackToDirectOnQualityFailure)
        {
            lut_ = std::move(lastCandidate);
            lastValidation_ = lastStats;
            loadedFromCache_ = lastFromCache;
            directFallback_ = true;
            return;
        }

        std::ostringstream message;
        message
            << "ICC LUT quality target was not reached. Max channel error: "
            << lastStats.maxChannelError8
            << ", RMS error: "
            << lastStats.rmsError8
            << ".";

        Reset();
        throw std::runtime_error(message.str());
    }

    void Converter::Reset() noexcept
    {
        lut_.Clear();
        appearanceTransform_.reset();
        transform_.reset();
        target_.reset();
        source_.reset();
        lastValidation_ = {};
        loadedFromCache_ = false;
        directFallback_ = false;
    }

    bool Converter::Ready() const noexcept
    {
        if (!source_ || !target_ || !transform_)
            return false;

        if (options_.conversionMode ==
            ConversionMode::PreserveAppearance)
        {
            return appearanceTransform_ &&
                appearanceTransform_->Ready();
        }

        return !lut_.Empty();
    }

    bool Converter::LoadedFromCache() const noexcept
    {
        return loadedFromCache_;
    }

    bool Converter::UsingDirectFallback() const noexcept
    {
        return directFallback_;
    }

    Rgb8 Converter::Convert(const Cmyk& color) const
    {
        if (!Ready())
            throw std::runtime_error("Color converter is not initialized.");

        if (options_.conversionMode ==
            ConversionMode::PreserveAppearance)
        {
            return appearanceTransform_->Translate(color);
        }

        if (directFallback_)
        {
            const auto value = transform_->Translate(color);
            return ApplyBlackFloor(
                Rgb8{ WordToByte(value.r), WordToByte(value.g), WordToByte(value.b) },
                options_);
        }

        return ApplyBlackFloor(
            lut_.Sample8(color, options_.interpolation),
            options_);
    }

    Rgb16 Converter::Convert16(const Cmyk& color) const
    {
        if (!Ready())
            throw std::runtime_error("Color converter is not initialized.");

        if (options_.conversionMode ==
            ConversionMode::PreserveAppearance)
        {
            const auto value =
                appearanceTransform_->Translate(color);

            return
            {
                ByteToWord(value.r),
                ByteToWord(value.g),
                ByteToWord(value.b)
            };
        }

        if (directFallback_)
            return ApplyBlackFloor(
                transform_->Translate(color),
                options_);

        return ApplyBlackFloor(
            lut_.Sample16(color, options_.interpolation),
            options_);
    }

    AppearanceMatch Converter::ConvertDetailed(
        const Cmyk& color) const
    {
        if (!Ready() ||
            options_.conversionMode !=
                ConversionMode::PreserveAppearance ||
            !appearanceTransform_)
        {
            throw std::runtime_error(
                "Detailed appearance matching requires PreserveAppearance mode.");
        }

        return appearanceTransform_->TranslateDetailed(color);
    }

    Rgb8 Converter::ConvertLab(const Lab& color) const
    {
        if (!appearanceTransform_ ||
            !appearanceTransform_->Ready())
        {
            throw std::runtime_error(
                "Lab-to-RGB appearance transform is not initialized.");
        }

        return appearanceTransform_->TranslateLab(color);
    }

    Rgb16 Converter::ConvertLab16(const Lab& color) const
    {
        const auto value =
            ConvertLab(color);

        return
        {
            ByteToWord(value.r),
            ByteToWord(value.g),
            ByteToWord(value.b)
        };
    }

    AppearanceMatch Converter::ConvertLabDetailed(
        const Lab& color) const
    {
        if (!appearanceTransform_ ||
            !appearanceTransform_->Ready())
        {
            throw std::runtime_error(
                "Detailed Lab-to-RGB appearance transform is not initialized.");
        }

        return appearanceTransform_->TranslateLabDetailed(color);
    }

    void Converter::Convert(const Cmyk* input, Rgb8* output, std::size_t count) const
    {
        if (!Ready())
            throw std::runtime_error("Color converter is not initialized.");

        if ((!input || !output) && count != 0)
            throw std::invalid_argument("Input and output buffers are required.");

        if (count == 0)
            return;

        if (options_.conversionMode ==
            ConversionMode::PreserveAppearance)
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                output[i] =
                    appearanceTransform_->Translate(input[i]);
            }

            return;
        }

        if (directFallback_)
        {
            std::vector<Rgb16> temporary(TranslateBatchLimit);
            std::size_t offset = 0;

            while (offset < count)
            {
                Cancellation::ThrowIfRequested();
                const std::size_t batch = std::min(count - offset, TranslateBatchLimit);
                transform_->Translate(input + offset, temporary.data(), batch);

                for (std::size_t i = 0; i < batch; ++i)
                {
                    output[offset + i] =
                        ApplyBlackFloor(
                            Rgb8
                            {
                                WordToByte(temporary[i].r),
                                WordToByte(temporary[i].g),
                                WordToByte(temporary[i].b)
                            },
                            options_);
                }

                offset += batch;
            }

            return;
        }

        for (std::size_t i = 0; i < count; ++i)
        {
            output[i] =
                ApplyBlackFloor(
                    lut_.Sample8(
                        input[i],
                        options_.interpolation),
                    options_);
        }
    }

    void Converter::Convert16(const Cmyk* input, Rgb16* output, std::size_t count) const
    {
        if (!Ready())
            throw std::runtime_error("Color converter is not initialized.");

        if ((!input || !output) && count != 0)
            throw std::invalid_argument("Input and output buffers are required.");

        if (count == 0)
            return;

        if (options_.conversionMode ==
            ConversionMode::PreserveAppearance)
        {
            for (std::size_t i = 0; i < count; ++i)
            {
                const auto value =
                    appearanceTransform_->Translate(input[i]);

                output[i] =
                {
                    ByteToWord(value.r),
                    ByteToWord(value.g),
                    ByteToWord(value.b)
                };
            }

            return;
        }

        if (directFallback_)
        {
            transform_->Translate(input, output, count);

            for (std::size_t i = 0; i < count; ++i)
                output[i] = ApplyBlackFloor(output[i], options_);

            return;
        }

        for (std::size_t i = 0; i < count; ++i)
        {
            output[i] =
                ApplyBlackFloor(
                    lut_.Sample16(
                        input[i],
                        options_.interpolation),
                    options_);
        }
    }

    Rgb16 Converter::ConvertDirect16(const Cmyk& color) const
    {
        if (options_.conversionMode ==
            ConversionMode::PreserveAppearance)
        {
            if (!appearanceTransform_ ||
                !appearanceTransform_->Ready())
            {
                throw std::runtime_error(
                    "Appearance-preserving transform is not initialized.");
            }

            const auto value =
                appearanceTransform_->Translate(color);

            return
            {
                ByteToWord(value.r),
                ByteToWord(value.g),
                ByteToWord(value.b)
            };
        }

        if (!transform_)
            throw std::runtime_error("ICC transform is not initialized.");

        return transform_->Translate(color);
    }

    ValidationStats Converter::Validate(std::uint32_t samples) const
    {
        if (!Ready())
            throw std::runtime_error("Color converter is not initialized.");

        if (options_.conversionMode ==
            ConversionMode::PreserveAppearance)
        {
            return {};
        }

        if (samples == 0)
            samples = std::max<std::uint32_t>(1, options_.validationSamples);

        return ValidateLut(lut_, samples);
    }

    const ProfileInfo& Converter::SourceProfile() const
    {
        if (!source_)
            throw std::runtime_error("Source ICC profile is not initialized.");

        return source_->Info();
    }

    const ProfileInfo& Converter::TargetProfile() const
    {
        if (!target_)
            throw std::runtime_error("Target ICC profile is not initialized.");

        return target_->Info();
    }

    const BuildOptions& Converter::Options() const noexcept
    {
        return options_;
    }

    const ValidationStats& Converter::LastValidation() const noexcept
    {
        return lastValidation_;
    }

    std::uint32_t Converter::GridSize() const noexcept
    {
        return lut_.GridSize();
    }

    std::filesystem::path Converter::DefaultCacheDirectory()
    {
        DWORD size = GetEnvironmentVariableW(L"LOCALAPPDATA", nullptr, 0);

        if (size > 1)
        {
            std::wstring value(size, L'\0');
            const DWORD written = GetEnvironmentVariableW(L"LOCALAPPDATA", value.data(), size);

            if (written > 0)
            {
                value.resize(written);
                return std::filesystem::path(value) / L"ImCut" / L"ColorLUT";
            }
        }

        std::error_code ec;
        const auto temp = std::filesystem::temp_directory_path(ec);

        if (!ec)
            return temp / L"ImCut" / L"ColorLUT";

        return std::filesystem::path(L".") / L"ImCutColorLUT";
    }

    ValidationStats Converter::ValidateLut(
        const IccLut4D& lut,
        std::uint32_t samples) const
    {
        ValidationStats stats{};

        if (!transform_ || lut.Empty() || samples == 0)
            return stats;

        std::vector<Cmyk> input(samples);
        std::vector<Rgb16> reference(samples);

        for (std::uint32_t i = 0; i < samples; ++i)
        {
            Cancellation::Checkpoint(i);
            const std::uint32_t index = i + 1;
            input[i] =
            {
                Halton(index, 2) * 100.0,
                Halton(index, 3) * 100.0,
                Halton(index, 5) * 100.0,
                Halton(index, 7) * 100.0
            };
        }

        transform_->Translate(input.data(), reference.data(), input.size());

        double absoluteSum = 0.0;
        double squareSum = 0.0;
        double maximum = 0.0;

        const std::size_t channelCount = static_cast<std::size_t>(samples) * 3;

        for (std::uint32_t i = 0; i < samples; ++i)
        {
            Cancellation::Checkpoint(i);
            const auto estimated = lut.Sample16(input[i], options_.interpolation);

            const double errors[3]
            {
                std::abs(static_cast<double>(estimated.r) - static_cast<double>(reference[i].r)) / 257.0,
                std::abs(static_cast<double>(estimated.g) - static_cast<double>(reference[i].g)) / 257.0,
                std::abs(static_cast<double>(estimated.b) - static_cast<double>(reference[i].b)) / 257.0
            };

            for (const double error : errors)
            {
                maximum = std::max(maximum, error);
                absoluteSum += error;
                squareSum += error * error;
            }
        }

        stats.samples = samples;
        stats.maxChannelError8 = maximum;
        stats.meanAbsoluteError8 = absoluteSum / static_cast<double>(channelCount);
        stats.rmsError8 = std::sqrt(squareSum / static_cast<double>(channelCount));
        return stats;
    }

    std::filesystem::path Converter::CacheFileFor(std::uint32_t gridSize) const
    {
        if (!source_ || !target_)
            return {};

        auto directory = options_.cacheDirectory;

        if (directory.empty())
            directory = DefaultCacheDirectory();

        std::wostringstream name;
        name
            << L"lut4d_"
            << std::hex
            << std::setw(16)
            << std::setfill(L'0')
            << source_->Info().fingerprint
            << L"_"
            << std::setw(16)
            << target_->Info().fingerprint
            << std::dec
            << L"_i"
            << static_cast<std::uint32_t>(options_.intent)
            << L"_n"
            << gridSize
            << L".bin";

        return directory / name.str();
    }

    double Converter::Halton(std::uint32_t index, std::uint32_t base) noexcept
    {
        double result = 0.0;
        double factor = 1.0 / static_cast<double>(base);

        while (index > 0)
        {
            result += factor * static_cast<double>(index % base);
            index /= base;
            factor /= static_cast<double>(base);
        }

        return result;
    }

    bool Converter::MeetsQualityTarget(
        const ValidationStats& stats,
        const BuildOptions& options) noexcept
    {
        if (stats.samples == 0)
            return true;

        return
            stats.maxChannelError8 <= options.maxAllowedChannelError8 &&
            stats.rmsError8 <= options.maxAllowedRmsError8;
    }
}
