#define NOMINMAX 1

#include "SettingsStore.hpp"
#include "AtomicIni.hpp"
#include "BufferedIni.hpp"
#include "InputLimits.hpp"
#include <limits>

#include <Windows.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <cwchar>
#include <string>
#include <system_error>
#include <utility>

namespace ImCut
{
    namespace
    {
        constexpr wchar_t SectionGeneral[] = L"General";
        constexpr wchar_t SectionCut[] = L"Cut";
        constexpr wchar_t SectionBleed[] = L"Bleed";
        constexpr wchar_t SectionColor[] = L"Color";
        constexpr wchar_t SectionNesting[] = L"Nesting";
        constexpr wchar_t SectionColorMappingFile[] = L"ImCutColorMapping";
        constexpr wchar_t SectionSettingsFile[] = L"ImCutSettings";

        [[nodiscard]] std::wstring ReadString(
            const std::filesystem::path& file,
            const wchar_t* section,
            const wchar_t* key,
            const wchar_t* fallback)
        {
            std::array<wchar_t, 512> buffer{};

            GetPrivateProfileStringW(
                section,
                key,
                fallback,
                buffer.data(),
                static_cast<DWORD>(buffer.size()),
                file.c_str());

            return buffer.data();
        }

        [[nodiscard]] int ReadInt(
            const std::filesystem::path& file,
            const wchar_t* section,
            const wchar_t* key,
            int fallback)
        {
            return static_cast<int>(
                GetPrivateProfileIntW(
                    section,
                    key,
                    fallback,
                    file.c_str()));
        }

        [[nodiscard]] double ReadDouble(
            const std::filesystem::path& file,
            const wchar_t* section,
            const wchar_t* key,
            double fallback)
        {
            const std::wstring value =
                ReadString(
                    file,
                    section,
                    key,
                    L"");

            if (value.empty())
                return fallback;

            wchar_t* end = nullptr;
            errno = 0;

            const double parsed =
                std::wcstod(
                    value.c_str(),
                    &end);

            if (errno != 0 ||
                end == value.c_str() ||
                !std::isfinite(parsed))
            {
                return fallback;
            }

            return parsed;
        }

        [[nodiscard]] std::uint64_t ReadUInt64(
            const std::filesystem::path& file,
            const wchar_t* section,
            const wchar_t* key,
            std::uint64_t fallback)
        {
            const std::wstring value =
                ReadString(
                    file,
                    section,
                    key,
                    L"");

            if (value.empty())
                return fallback;

            wchar_t* end = nullptr;
            errno = 0;

            const unsigned long long parsed =
                std::wcstoull(
                    value.c_str(),
                    &end,
                    16);

            if (errno != 0 ||
                end == value.c_str())
            {
                return fallback;
            }

            return static_cast<std::uint64_t>(
                parsed);
        }

        [[nodiscard]] bool WriteString(
            BufferedIni& file, const wchar_t* section, const wchar_t* key,
            const std::wstring& value)
        {
            if (std::wcscmp(key, L"SpotBlacklistNames") == 0)
            {
                
                
                auto aliases = value;
                std::replace(aliases.begin(), aliases.end(), L'\r', L'|');
                std::replace(aliases.begin(), aliases.end(), L'\n', L'|');
                return file.Set(section, key, aliases);
            }
            return file.Set(section, key, value);
        }

        [[nodiscard]] bool WriteInt(
            BufferedIni& file,
            const wchar_t* section,
            const wchar_t* key,
            int value)
        {
            return WriteString(
                file,
                section,
                key,
                std::to_wstring(value));
        }

        [[nodiscard]] bool WriteDouble(
            BufferedIni& file,
            const wchar_t* section,
            const wchar_t* key,
            double value)
        {
            wchar_t buffer[64]{};

            const int count =
                swprintf_s(
                    buffer,
                    _countof(buffer),
                    L"%.6f",
                    value);

            if (count <= 0)
                return false;

            return WriteString(
                file,
                section,
                key,
                buffer);
        }

        [[nodiscard]] bool WriteUInt64(
            BufferedIni& file,
            const wchar_t* section,
            const wchar_t* key,
            std::uint64_t value)
        {
            wchar_t buffer[32]{};

            const int count =
                swprintf_s(
                    buffer,
                    _countof(buffer),
                    L"%016llX",
                    static_cast<unsigned long long>(
                        value));

            if (count <= 0)
                return false;

            return WriteString(
                file,
                section,
                key,
                buffer);
        }

        [[nodiscard]] std::wstring Utf8ToWide(
            const char* value)
        {
            if (!value || *value == '\0')
                return {};

            const int sourceLength =
                static_cast<int>(
                    std::char_traits<char>::length(value));

            const int length =
                MultiByteToWideChar(
                    CP_UTF8,
                    0,
                    value,
                    sourceLength,
                    nullptr,
                    0);

            if (length <= 0)
                return {};

            std::wstring output(
                static_cast<std::size_t>(length),
                L'\0');

            MultiByteToWideChar(
                CP_UTF8,
                0,
                value,
                sourceLength,
                output.data(),
                length);

            return output;
        }

        template <std::size_t N>
        void CopyUtf8(
            std::array<char, N>& destination,
            const std::wstring& value)
        {
            destination.fill('\0');

            if (value.empty() || N == 0)
                return;

            const int required =
                WideCharToMultiByte(
                    CP_UTF8,
                    0,
                    value.data(),
                    static_cast<int>(value.size()),
                    nullptr,
                    0,
                    nullptr,
                    nullptr);

            if (required <= 0)
                return;

            std::string utf8(
                static_cast<std::size_t>(required),
                '\0');

            WideCharToMultiByte(
                CP_UTF8,
                0,
                value.data(),
                static_cast<int>(value.size()),
                utf8.data(),
                required,
                nullptr,
                nullptr);

            const std::size_t count =
                (std::min)(
                    utf8.size(),
                    N - 1);

            std::copy_n(
                utf8.data(),
                count,
                destination.data());
        }

        [[nodiscard]] std::wstring ColorMapKey(
            int index,
            const wchar_t* suffix)
        {
            return L"Map" +
                std::to_wstring(index) +
                suffix;
        }

        template <typename T>
        void HashValue(
            std::uint64_t& hash,
            const T& value) noexcept
        {
            constexpr std::uint64_t prime =
                1099511628211ull;

            const auto* bytes =
                reinterpret_cast<const unsigned char*>(
                    &value);

            for (std::size_t i = 0;
                i < sizeof(T);
                ++i)
            {
                hash ^= bytes[i];
                hash *= prime;
            }
        }

        [[nodiscard]] bool EnsureDirectory(
            const std::filesystem::path& file)
        {
            std::error_code error;

            const auto directory =
                file.parent_path();

            if (directory.empty())
                return true;

            std::filesystem::create_directories(
                directory,
                error);

            return !error;
        }

        [[nodiscard]] bool InitializeUnicodeProfileFile(
            const std::filesystem::path& file)
        {
            HANDLE handle =
                CreateFileW(
                    file.c_str(),
                    GENERIC_WRITE,
                    FILE_SHARE_READ,
                    nullptr,
                    CREATE_ALWAYS,
                    FILE_ATTRIBUTE_NORMAL,
                    nullptr);

            if (handle == INVALID_HANDLE_VALUE)
                return false;

            constexpr wchar_t bom =
                static_cast<wchar_t>(0xFEFF);

            DWORD written = 0;

            const BOOL ok =
                WriteFile(
                    handle,
                    &bom,
                    sizeof(bom),
                    &written,
                    nullptr);

            CloseHandle(handle);

            return
                ok != FALSE &&
                written == sizeof(bom);
        }
    }

    SettingsStore::SettingsStore()
        : filePath_(
            ResolveFilePath())
    {
    }

    SettingsStore::SettingsStore(
        std::filesystem::path filePath)
        : filePath_(
            std::move(filePath))
    {
    }

    void SettingsStore::Load(
        UI::State& state,
        StoredProfiles& profiles) const
    {
        if (filePath_.empty())
            return;

        const int language =
            ReadInt(
                filePath_,
                SectionGeneral,
                L"Language",
                static_cast<int>(
                    state.language));

        state.language =
            static_cast<UI::Language>(
                std::clamp(
                    language,
                    0,
                    1));

        const int page =
            ReadInt(
                filePath_,
                SectionGeneral,
                L"Page",
                static_cast<int>(
                    state.page));

        switch (page)
        {
            case static_cast<int>(
            UI::Page::Cut):
                state.page =
                    UI::Page::Cut;
                break;

                case static_cast<int>(
                UI::Page::Bleed):
                    state.page =
                        UI::Page::Bleed;
                    break;

                    case static_cast<int>(
                    UI::Page::Color):
                        state.page =
                            UI::Page::Color;
                        break;

                        case static_cast<int>(
                        UI::Page::Settings):
                            state.page =
                                UI::Page::Settings;
                            break;

                        default:
                            state.page =
                                UI::Page::Cut;
                            break;
        }

        state.cut.pageWidthMm =
            std::max(
                ReadInt(
                    filePath_,
                    SectionCut,
                    L"PageWidthMm",
                    state.cut.pageWidthMm),
                1);

        state.cut.registrationMarks =
            std::max(
                ReadInt(
                    filePath_,
                    SectionCut,
                    L"RegistrationMarks",
                    state.cut.registrationMarks),
                0);

        state.cut.closureMode =
            static_cast<UI::ClosureMode>(
                std::clamp(
                    ReadInt(
                        filePath_,
                        SectionCut,
                        L"ClosureMode",
                        static_cast<int>(
                            state.cut.closureMode)),
                    0,
                    3));

        state.cut.namePages =
            ReadInt(
                filePath_,
                SectionCut,
                L"NamePages",
                state.cut.namePages ? 1 : 0) != 0;

        state.cut.showSummary =
            ReadInt(
                filePath_,
                SectionCut,
                L"ShowSummary",
                state.cut.showSummary ? 1 : 0) != 0;

        state.cut.forceBottomRightAnchor =
            ReadInt(
                filePath_,
                SectionCut,
                L"ForceBottomRightAnchor",
                state.cut.forceBottomRightAnchor ? 1 : 0) != 0;

        state.bleed.distanceMm = InputLimits::BleedOrDefault(
            ReadDouble(filePath_, SectionBleed, L"DistanceMm", state.bleed.distanceMm));

        state.bleed.ungroupBeforeProcessing =
            ReadInt(
                filePath_,
                SectionBleed,
                L"UngroupBeforeProcessing",
                state.bleed.ungroupBeforeProcessing ? 1 : 0) != 0;

        state.bleed.detectHiddenObjects =
            ReadInt(
                filePath_,
                SectionBleed,
                L"DetectHiddenObjects",
                state.bleed.detectHiddenObjects ? 1 : 0) != 0;

        state.bleed.createCutline =
            ReadInt(
                filePath_,
                SectionBleed,
                L"CreateCutline",
                state.bleed.createCutline ? 1 : 0) != 0;

        state.color.intent =
            static_cast<UI::RenderingIntent>(
                std::clamp(
                    ReadInt(
                        filePath_,
                        SectionColor,
                        L"RenderingIntent",
                        static_cast<int>(
                            state.color.intent)),
                    0,
                    3));

        state.color.conversionMode =
            static_cast<UI::ColorConversionMode>(
                std::clamp(
                    ReadInt(
                        filePath_,
                        SectionColor,
                        L"ConversionMode",
                        static_cast<int>(
                            state.color.conversionMode)),
                    0,
                    1));

        state.color.assignDocumentProfiles = ReadInt(filePath_, SectionColor,
            L"AssignDocumentProfiles", state.color.assignDocumentProfiles ? 1 : 0) != 0;

        state.color.exhaustiveAppearanceSearch =
            ReadInt(
                filePath_,
                SectionColor,
                L"ExhaustiveAppearanceSearch",
                state.color.exhaustiveAppearanceSearch ? 1 : 0) != 0;

        state.color.adaptiveLut =
            ReadInt(
                filePath_,
                SectionColor,
                L"AdaptiveLut",
                state.color.adaptiveLut ? 1 : 0) != 0;

        state.color.preferredGrid =
            std::clamp(
                ReadInt(
                    filePath_,
                    SectionColor,
                    L"PreferredGrid",
                    state.color.preferredGrid),
                2,
                65);

        state.color.showSummary =
            ReadInt(
                filePath_,
                SectionColor,
                L"ShowSummary",
                state.color.showSummary ? 1 : 0) != 0;

        state.color.blackFloorEnabled =
            ReadInt(
                filePath_,
                SectionColor,
                L"BlackFloorEnabled",
                state.color.blackFloorEnabled ? 1 : 0) != 0;

        state.color.blackFloorRgb =
            std::clamp(
                ReadInt(
                    filePath_,
                    SectionColor,
                    L"BlackFloorRgb",
                    state.color.blackFloorRgb),
                0,
                255);

        state.color.spotTintWhiteEnabled =
            ReadInt(
                filePath_,
                SectionColor,
                L"SpotTintWhiteEnabled",
                state.color.spotTintWhiteEnabled ? 1 : 0) != 0;

        state.color.spotTintWhiteThreshold =
            std::clamp(
                ReadInt(
                    filePath_,
                    SectionColor,
                    L"SpotTintWhiteThreshold",
                    state.color.spotTintWhiteThreshold),
                0,
                100);

        state.color.convertSpotsToRgb =
            ReadInt(
                filePath_,
                SectionColor,
                L"ConvertSpotsToRgb",
                state.color.convertSpotsToRgb ? 1 : 0) != 0;

        state.color.preserveSpotAppearance =
            ReadInt(
                filePath_,
                SectionColor,
                L"PreserveSpotAppearance",
                state.color.preserveSpotAppearance ? 1 : 0) != 0;

        state.color.spotBlacklistEnabled =
            ReadInt(
                filePath_,
                SectionColor,
                L"SpotBlacklistEnabled",
                state.color.spotBlacklistEnabled ? 1 : 0) != 0;

        CopyUtf8(
            state.color.spotBlacklistNames,
            ReadString(
                filePath_,
                SectionColor,
                L"SpotBlacklistNames",
                L""));

        state.color.colorMappingEnabled =
            ReadInt(
                filePath_,
                SectionColor,
                L"ColorMappingEnabled",
                state.color.colorMappingEnabled ? 1 : 0) != 0;

        state.color.colorMappingTolerance =
            std::clamp(
                ReadInt(
                    filePath_,
                    SectionColor,
                    L"ColorMappingTolerance",
                    state.color.colorMappingTolerance),
                0,
                255);

        const int colorMapCount =
            std::clamp(
                ReadInt(
                    filePath_,
                    SectionColor,
                    L"ColorMapCount",
                    0),
                0,
                64);

        state.color.colorMappings.clear();
        state.color.colorMappings.reserve(
            static_cast<std::size_t>(
                colorMapCount));

        for (int i = 0; i < colorMapCount; ++i)
        {
            UI::ColorMapEntry entry;

            entry.enabled =
                ReadInt(
                    filePath_,
                    SectionColor,
                    ColorMapKey(i, L"Enabled").c_str(),
                    1) != 0;

            entry.sourceKind =
                static_cast<UI::ColorMapSourceKind>(
                    std::clamp(
                        ReadInt(
                            filePath_,
                            SectionColor,
                            ColorMapKey(i, L"SourceKind").c_str(),
                            0),
                        0,
                        2));

            for (int channel = 0; channel < 4; ++channel)
            {
                const std::wstring suffix =
                    L"Source" +
                    std::to_wstring(channel);

                entry.sourceChannels[
                    static_cast<std::size_t>(channel)] =
                    ReadInt(
                        filePath_,
                        SectionColor,
                        ColorMapKey(i, suffix.c_str()).c_str(),
                        0);
            }

            CopyUtf8(
                entry.sourceSpotPalette,
                ReadString(
                    filePath_,
                    SectionColor,
                    ColorMapKey(i, L"SourceSpotPalette").c_str(),
                    L""));

            CopyUtf8(
                entry.sourceSpotName,
                ReadString(
                    filePath_,
                    SectionColor,
                    ColorMapKey(i, L"SourceSpotName").c_str(),
                    L""));

            entry.sourceSpotTint =
                std::clamp(
                    ReadInt(
                        filePath_,
                        SectionColor,
                        ColorMapKey(i, L"SourceSpotTint").c_str(),
                        100),
                    0,
                    100);

            entry.targetKind =
                static_cast<UI::ColorMapTargetKind>(
                    std::clamp(
                        ReadInt(
                            filePath_,
                            SectionColor,
                            ColorMapKey(i, L"TargetKind").c_str(),
                            0),
                        0,
                        1));

            for (int channel = 0; channel < 3; ++channel)
            {
                const std::wstring suffix =
                    L"TargetRgb" +
                    std::to_wstring(channel);

                entry.targetRgb[
                    static_cast<std::size_t>(channel)] =
                    std::clamp(
                        ReadInt(
                            filePath_,
                            SectionColor,
                            ColorMapKey(i, suffix.c_str()).c_str(),
                            0),
                        0,
                        255);
            }

            CopyUtf8(
                entry.targetSpotPalette,
                ReadString(
                    filePath_,
                    SectionColor,
                    ColorMapKey(i, L"TargetSpotPalette").c_str(),
                    L""));

            CopyUtf8(
                entry.targetSpotName,
                ReadString(
                    filePath_,
                    SectionColor,
                    ColorMapKey(i, L"TargetSpotName").c_str(),
                    L""));

            entry.targetSpotTint =
                std::clamp(
                    ReadInt(
                        filePath_,
                        SectionColor,
                        ColorMapKey(i, L"TargetSpotTint").c_str(),
                        100),
                    0,
                    100);

            if (entry.sourceKind ==
                UI::ColorMapSourceKind::CMYK)
            {
                for (int channel = 0; channel < 4; ++channel)
                {
                    entry.sourceChannels[
                        static_cast<std::size_t>(channel)] =
                        std::clamp(
                            entry.sourceChannels[
                                static_cast<std::size_t>(channel)],
                            0,
                            100);
                }
            }
            else
            {
                for (int channel = 0; channel < 3; ++channel)
                {
                    entry.sourceChannels[
                        static_cast<std::size_t>(channel)] =
                        std::clamp(
                            entry.sourceChannels[
                                static_cast<std::size_t>(channel)],
                            0,
                            255);
                }
            }

            state.color.colorMappings.push_back(
                std::move(entry));
        }

        state.nesting.spacingMm = static_cast<float>(std::clamp(
            ReadDouble(filePath_, SectionNesting, L"SpacingMm", state.nesting.spacingMm), 0.0, 1000.0));

        state.nesting.rotationStepDeg =
            std::clamp(
                ReadInt(
                    filePath_,
                    SectionNesting,
                    L"RotationStepDeg",
                    state.nesting.rotationStepDeg),
                1,
                360);

        state.nesting.generations =
            std::max(
                ReadInt(
                    filePath_,
                    SectionNesting,
                    L"Generations",
                    state.nesting.generations),
                1);

        state.nesting.population =
            std::max(
                ReadInt(
                    filePath_,
                    SectionNesting,
                    L"Population",
                    state.nesting.population),
                1);

        state.nesting.allowMirror =
            ReadInt(
                filePath_,
                SectionNesting,
                L"AllowMirror",
                state.nesting.allowMirror ? 1 : 0) != 0;

        state.nesting.useTrueShape =
            ReadInt(
                filePath_,
                SectionNesting,
                L"UseTrueShape",
                state.nesting.useTrueShape ? 1 : 0) != 0;

        profiles.cmykFingerprint =
            ReadUInt64(
                filePath_,
                SectionColor,
                L"CmykFingerprint",
                0);

        profiles.rgbFingerprint =
            ReadUInt64(
                filePath_,
                SectionColor,
                L"RgbFingerprint",
                0);
    }

    bool SettingsStore::Save(
        const UI::State& state,
        const StoredProfiles& profiles) const
    {
        if (filePath_.empty() ||
            !EnsureDirectory(
                filePath_))
        {
            return false;
        }

        AtomicIni transaction(filePath_);
        if (!transaction.Ready()) return false;
        const auto& stagedPath = transaction.Path();

        BufferedIni writer(stagedPath);
        bool ok = true;
        ok = WriteInt(writer, SectionColor, L"AssignDocumentProfiles",
            state.color.assignDocumentProfiles ? 1 : 0) && ok;


        ok = WriteInt(
            writer,
            SectionGeneral,
            L"Language",
            static_cast<int>(
                state.language)) && ok;

        ok = WriteInt(
            writer,
            SectionGeneral,
            L"Page",
            static_cast<int>(
                state.page)) && ok;

        ok = WriteInt(
            writer,
            SectionCut,
            L"PageWidthMm",
            state.cut.pageWidthMm) && ok;

        ok = WriteInt(
            writer,
            SectionCut,
            L"RegistrationMarks",
            state.cut.registrationMarks) && ok;

        ok = WriteInt(
            writer,
            SectionCut,
            L"ClosureMode",
            static_cast<int>(
                state.cut.closureMode)) && ok;

        ok = WriteInt(
            writer,
            SectionCut,
            L"NamePages",
            state.cut.namePages ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionCut,
            L"ShowSummary",
            state.cut.showSummary ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionCut,
            L"ForceBottomRightAnchor",
            state.cut.forceBottomRightAnchor ? 1 : 0) && ok;

        ok = WriteDouble(
            writer,
            SectionBleed,
            L"DistanceMm",
            state.bleed.distanceMm) && ok;

        ok = WriteInt(
            writer,
            SectionBleed,
            L"UngroupBeforeProcessing",
            state.bleed.ungroupBeforeProcessing ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionBleed,
            L"DetectHiddenObjects",
            state.bleed.detectHiddenObjects ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionBleed,
            L"CreateCutline",
            state.bleed.createCutline ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"RenderingIntent",
            static_cast<int>(
                state.color.intent)) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"ConversionMode",
            static_cast<int>(
                state.color.conversionMode)) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"ExhaustiveAppearanceSearch",
            state.color.exhaustiveAppearanceSearch ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"AdaptiveLut",
            state.color.adaptiveLut ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"PreferredGrid",
            state.color.preferredGrid) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"ShowSummary",
            state.color.showSummary ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"BlackFloorEnabled",
            state.color.blackFloorEnabled ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"BlackFloorRgb",
            std::clamp(
                state.color.blackFloorRgb,
                0,
                255)) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"SpotTintWhiteEnabled",
            state.color.spotTintWhiteEnabled ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"SpotTintWhiteThreshold",
            std::clamp(
                state.color.spotTintWhiteThreshold,
                0,
                100)) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"ConvertSpotsToRgb",
            state.color.convertSpotsToRgb ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"PreserveSpotAppearance",
            state.color.preserveSpotAppearance ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"SpotBlacklistEnabled",
            state.color.spotBlacklistEnabled ? 1 : 0) && ok;

        ok = WriteString(
            writer,
            SectionColor,
            L"SpotBlacklistNames",
            Utf8ToWide(
                state.color.spotBlacklistNames.data())) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"ColorMappingEnabled",
            state.color.colorMappingEnabled ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColor,
            L"ColorMappingTolerance",
            std::clamp(
                state.color.colorMappingTolerance,
                0,
                255)) && ok;

        const int colorMapCount =
            static_cast<int>(
                (std::min)(
                    state.color.colorMappings.size(),
                    std::size_t{ 64 }));

        ok = WriteInt(
            writer,
            SectionColor,
            L"ColorMapCount",
            colorMapCount) && ok;

        for (int i = 0; i < colorMapCount; ++i)
        {
            const auto& entry =
                state.color.colorMappings[
                    static_cast<std::size_t>(i)];

            ok = WriteInt(
                writer,
                SectionColor,
                ColorMapKey(i, L"Enabled").c_str(),
                entry.enabled ? 1 : 0) && ok;

            ok = WriteInt(
                writer,
                SectionColor,
                ColorMapKey(i, L"SourceKind").c_str(),
                static_cast<int>(entry.sourceKind)) && ok;

            for (int channel = 0; channel < 4; ++channel)
            {
                const std::wstring suffix =
                    L"Source" +
                    std::to_wstring(channel);

                ok = WriteInt(
                    writer,
                    SectionColor,
                    ColorMapKey(i, suffix.c_str()).c_str(),
                    entry.sourceChannels[
                        static_cast<std::size_t>(channel)]) && ok;
            }

            ok = WriteString(
                writer,
                SectionColor,
                ColorMapKey(i, L"SourceSpotPalette").c_str(),
                Utf8ToWide(
                    entry.sourceSpotPalette.data())) && ok;

            ok = WriteString(
                writer,
                SectionColor,
                ColorMapKey(i, L"SourceSpotName").c_str(),
                Utf8ToWide(
                    entry.sourceSpotName.data())) && ok;

            ok = WriteInt(
                writer,
                SectionColor,
                ColorMapKey(i, L"SourceSpotTint").c_str(),
                std::clamp(
                    entry.sourceSpotTint,
                    0,
                    100)) && ok;

            ok = WriteInt(
                writer,
                SectionColor,
                ColorMapKey(i, L"TargetKind").c_str(),
                static_cast<int>(entry.targetKind)) && ok;

            for (int channel = 0; channel < 3; ++channel)
            {
                const std::wstring suffix =
                    L"TargetRgb" +
                    std::to_wstring(channel);

                ok = WriteInt(
                    writer,
                    SectionColor,
                    ColorMapKey(i, suffix.c_str()).c_str(),
                    std::clamp(
                        entry.targetRgb[
                            static_cast<std::size_t>(channel)],
                        0,
                        255)) && ok;
            }

            ok = WriteString(
                writer,
                SectionColor,
                ColorMapKey(i, L"TargetSpotPalette").c_str(),
                Utf8ToWide(
                    entry.targetSpotPalette.data())) && ok;

            ok = WriteString(
                writer,
                SectionColor,
                ColorMapKey(i, L"TargetSpotName").c_str(),
                Utf8ToWide(
                    entry.targetSpotName.data())) && ok;

            ok = WriteInt(
                writer,
                SectionColor,
                ColorMapKey(i, L"TargetSpotTint").c_str(),
                std::clamp(
                    entry.targetSpotTint,
                    0,
                    100)) && ok;
        }

        ok = WriteDouble(
            writer,
            SectionNesting,
            L"SpacingMm",
            state.nesting.spacingMm) && ok;

        ok = WriteInt(
            writer,
            SectionNesting,
            L"RotationStepDeg",
            state.nesting.rotationStepDeg) && ok;

        ok = WriteInt(
            writer,
            SectionNesting,
            L"Generations",
            state.nesting.generations) && ok;

        ok = WriteInt(
            writer,
            SectionNesting,
            L"Population",
            state.nesting.population) && ok;

        ok = WriteInt(
            writer,
            SectionNesting,
            L"AllowMirror",
            state.nesting.allowMirror ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionNesting,
            L"UseTrueShape",
            state.nesting.useTrueShape ? 1 : 0) && ok;

        ok = WriteUInt64(
            writer,
            SectionColor,
            L"CmykFingerprint",
            profiles.cmykFingerprint) && ok;

        ok = WriteUInt64(
            writer,
            SectionColor,
            L"RgbFingerprint",
            profiles.rgbFingerprint) && ok;

        if (ok)
        {
            WritePrivateProfileStringW(
                nullptr,
                nullptr,
                nullptr,
                stagedPath.c_str());
        }

        return ok && writer.Flush() && transaction.Commit();
    }

    std::uint64_t SettingsStore::Signature(
        const UI::State& state,
        const StoredProfiles& profiles) const noexcept
    {
        std::uint64_t hash =
            14695981039346656037ull;

        const int page =
            static_cast<int>(
                state.page);

        const int language =
            static_cast<int>(
                state.language);

        const int closureMode =
            static_cast<int>(
                state.cut.closureMode);

        const int intent =
            static_cast<int>(
                state.color.intent);

        const int conversionMode =
            static_cast<int>(
                state.color.conversionMode);

        HashValue(
            hash,
            page);

        HashValue(
            hash,
            language);

        HashValue(
            hash,
            state.cut.pageWidthMm);

        HashValue(
            hash,
            state.cut.registrationMarks);

        HashValue(
            hash,
            closureMode);

        HashValue(
            hash,
            state.cut.namePages);

        HashValue(
            hash,
            state.cut.showSummary);

        HashValue(
            hash,
            state.cut.forceBottomRightAnchor);

        HashValue(
            hash,
            state.bleed.distanceMm);

        HashValue(
            hash,
            state.bleed.ungroupBeforeProcessing);

        HashValue(
            hash,
            state.bleed.detectHiddenObjects);

        HashValue(
            hash,
            state.bleed.createCutline);

        HashValue(
            hash,
            intent);

        HashValue(
            hash,
            conversionMode);

        HashValue(
            hash,
            state.color.exhaustiveAppearanceSearch);
        HashValue(hash, state.color.assignDocumentProfiles);


        HashValue(
            hash,
            state.color.adaptiveLut);

        HashValue(
            hash,
            state.color.preferredGrid);

        HashValue(
            hash,
            state.color.showSummary);

        HashValue(
            hash,
            state.color.blackFloorEnabled);

        HashValue(
            hash,
            state.color.blackFloorRgb);

        HashValue(
            hash,
            state.color.spotTintWhiteEnabled);

        HashValue(
            hash,
            state.color.spotTintWhiteThreshold);

        HashValue(
            hash,
            state.color.convertSpotsToRgb);

        HashValue(
            hash,
            state.color.preserveSpotAppearance);

        HashValue(
            hash,
            state.color.spotBlacklistEnabled);

        HashValue(
            hash,
            state.color.spotBlacklistNames);

        HashValue(
            hash,
            state.color.colorMappingEnabled);

        HashValue(
            hash,
            state.color.colorMappingTolerance);

        const std::size_t colorMapCount =
            (std::min)(
                state.color.colorMappings.size(),
                std::size_t{ 64 });

        HashValue(
            hash,
            colorMapCount);

        for (std::size_t i = 0; i < colorMapCount; ++i)
        {
            const auto& entry =
                state.color.colorMappings[i];

            const int sourceKind =
                static_cast<int>(
                    entry.sourceKind);

            const int targetKind =
                static_cast<int>(
                    entry.targetKind);

            HashValue(hash, entry.enabled);
            HashValue(hash, sourceKind);
            HashValue(hash, entry.sourceChannels);
            HashValue(hash, entry.sourceSpotPalette);
            HashValue(hash, entry.sourceSpotName);
            HashValue(hash, entry.sourceSpotTint);
            HashValue(hash, targetKind);
            HashValue(hash, entry.targetRgb);
            HashValue(hash, entry.targetSpotPalette);
            HashValue(hash, entry.targetSpotName);
            HashValue(hash, entry.targetSpotTint);
        }

        HashValue(
            hash,
            state.nesting.spacingMm);

        HashValue(
            hash,
            state.nesting.rotationStepDeg);

        HashValue(
            hash,
            state.nesting.generations);

        HashValue(
            hash,
            state.nesting.population);

        HashValue(
            hash,
            state.nesting.allowMirror);

        HashValue(
            hash,
            state.nesting.useTrueShape);

        HashValue(
            hash,
            profiles.cmykFingerprint);

        HashValue(
            hash,
            profiles.rgbFingerprint);

        return hash;
    }


    bool SettingsStore::ExportColorMappings(
        const std::filesystem::path& filePath,
        const UI::ColorState& color,
        std::wstring& error)
    {
        error.clear();

        if (filePath.empty())
        {
            error = L"Invalid Color Mapping file path.";
            return false;
        }

        if (!EnsureDirectory(filePath))
        {
            error = L"Unable to create the destination directory.";
            return false;
        }

        AtomicIni transaction(filePath, false);
        if (!transaction.Ready())
        {
            error = L"Unable to stage the settings file.";
            return false;
        }
        const auto& stagedPath = transaction.Path();

        BufferedIni writer(stagedPath);
        bool ok = true;

        ok = WriteInt(
            writer,
            SectionColorMappingFile,
            L"Version",
            1) && ok;

        ok = WriteInt(
            writer,
            SectionColorMappingFile,
            L"Enabled",
            color.colorMappingEnabled ? 1 : 0) && ok;

        ok = WriteInt(
            writer,
            SectionColorMappingFile,
            L"Tolerance",
            std::clamp(
                color.colorMappingTolerance,
                0,
                255)) && ok;

        const int count =
            static_cast<int>(
                (std::min)(
                    color.colorMappings.size(),
                    std::size_t{ 64 }));

        ok = WriteInt(
            writer,
            SectionColorMappingFile,
            L"Count",
            count) && ok;

        for (int i = 0; i < count; ++i)
        {
            const auto& entry =
                color.colorMappings[
                    static_cast<std::size_t>(i)];

            ok = WriteInt(
                writer,
                SectionColorMappingFile,
                ColorMapKey(i, L"Enabled").c_str(),
                entry.enabled ? 1 : 0) && ok;

            ok = WriteInt(
                writer,
                SectionColorMappingFile,
                ColorMapKey(i, L"SourceKind").c_str(),
                static_cast<int>(
                    entry.sourceKind)) && ok;

            for (int channel = 0; channel < 4; ++channel)
            {
                const std::wstring suffix =
                    L"Source" +
                    std::to_wstring(channel);

                ok = WriteInt(
                    writer,
                    SectionColorMappingFile,
                    ColorMapKey(
                        i,
                        suffix.c_str()).c_str(),
                    entry.sourceChannels[
                        static_cast<std::size_t>(
                            channel)]) && ok;
            }

            ok = WriteString(
                writer,
                SectionColorMappingFile,
                ColorMapKey(i, L"SourceSpotPalette").c_str(),
                Utf8ToWide(
                    entry.sourceSpotPalette.data())) && ok;

            ok = WriteString(
                writer,
                SectionColorMappingFile,
                ColorMapKey(i, L"SourceSpotName").c_str(),
                Utf8ToWide(
                    entry.sourceSpotName.data())) && ok;

            ok = WriteInt(
                writer,
                SectionColorMappingFile,
                ColorMapKey(i, L"SourceSpotTint").c_str(),
                std::clamp(
                    entry.sourceSpotTint,
                    0,
                    100)) && ok;

            ok = WriteInt(
                writer,
                SectionColorMappingFile,
                ColorMapKey(i, L"TargetKind").c_str(),
                static_cast<int>(
                    entry.targetKind)) && ok;

            for (int channel = 0; channel < 3; ++channel)
            {
                const std::wstring suffix =
                    L"TargetRgb" +
                    std::to_wstring(channel);

                ok = WriteInt(
                    writer,
                    SectionColorMappingFile,
                    ColorMapKey(
                        i,
                        suffix.c_str()).c_str(),
                    std::clamp(
                        entry.targetRgb[
                            static_cast<std::size_t>(
                                channel)],
                        0,
                        255)) && ok;
            }

            ok = WriteString(
                writer,
                SectionColorMappingFile,
                ColorMapKey(i, L"TargetSpotPalette").c_str(),
                Utf8ToWide(
                    entry.targetSpotPalette.data())) && ok;

            ok = WriteString(
                writer,
                SectionColorMappingFile,
                ColorMapKey(i, L"TargetSpotName").c_str(),
                Utf8ToWide(
                    entry.targetSpotName.data())) && ok;

            ok = WriteInt(
                writer,
                SectionColorMappingFile,
                ColorMapKey(i, L"TargetSpotTint").c_str(),
                std::clamp(
                    entry.targetSpotTint,
                    0,
                    100)) && ok;
        }

        if (!ok)
        {
            error = L"Unable to write the Color Mapping file.";
            return false;
        }

        WritePrivateProfileStringW(
            nullptr,
            nullptr,
            nullptr,
            stagedPath.c_str());

        if (!writer.Flush() || !transaction.Commit())
        {
            error = L"Unable to replace the destination file. The previous file was preserved.";
            return false;
        }
        return true;
    }

    bool SettingsStore::ImportColorMappings(
        const std::filesystem::path& filePath,
        UI::ColorState& color,
        std::wstring& error)
    {
        error.clear();

        if (filePath.empty())
        {
            error = L"Invalid Color Mapping file path.";
            return false;
        }

        std::error_code existsError;

        if (!std::filesystem::exists(
            filePath,
            existsError) ||
            existsError)
        {
            error = L"Color Mapping file was not found.";
            return false;
        }

        const int version =
            ReadInt(
                filePath,
                SectionColorMappingFile,
                L"Version",
                0);

        if (version != 1)
        {
            error = L"Unsupported or invalid Color Mapping file.";
            return false;
        }

        const int rawCount =
            ReadInt(
                filePath,
                SectionColorMappingFile,
                L"Count",
                0);

        if (rawCount < 0 ||
            rawCount > 64)
        {
            error = L"Invalid number of mappings in the Color Mapping file.";
            return false;
        }

        std::vector<UI::ColorMapEntry> imported;
        imported.reserve(
            static_cast<std::size_t>(
                rawCount));

        for (int i = 0; i < rawCount; ++i)
        {
            UI::ColorMapEntry entry;

            entry.enabled =
                ReadInt(
                    filePath,
                    SectionColorMappingFile,
                    ColorMapKey(i, L"Enabled").c_str(),
                    1) != 0;

            entry.sourceKind =
                static_cast<UI::ColorMapSourceKind>(
                    std::clamp(
                        ReadInt(
                            filePath,
                            SectionColorMappingFile,
                            ColorMapKey(i, L"SourceKind").c_str(),
                            0),
                        0,
                        2));

            for (int channel = 0; channel < 4; ++channel)
            {
                const std::wstring suffix =
                    L"Source" +
                    std::to_wstring(channel);

                entry.sourceChannels[
                    static_cast<std::size_t>(
                        channel)] =
                    ReadInt(
                        filePath,
                        SectionColorMappingFile,
                        ColorMapKey(
                            i,
                            suffix.c_str()).c_str(),
                        0);
            }

            if (entry.sourceKind ==
                UI::ColorMapSourceKind::CMYK)
            {
                for (int channel = 0; channel < 4; ++channel)
                {
                    entry.sourceChannels[
                        static_cast<std::size_t>(
                            channel)] =
                        std::clamp(
                            entry.sourceChannels[
                                static_cast<std::size_t>(
                                    channel)],
                            0,
                            100);
                }
            }
            else
            {
                for (int channel = 0; channel < 3; ++channel)
                {
                    entry.sourceChannels[
                        static_cast<std::size_t>(
                            channel)] =
                        std::clamp(
                            entry.sourceChannels[
                                static_cast<std::size_t>(
                                    channel)],
                            0,
                            255);
                }
            }

            CopyUtf8(
                entry.sourceSpotPalette,
                ReadString(
                    filePath,
                    SectionColorMappingFile,
                    ColorMapKey(
                        i,
                        L"SourceSpotPalette").c_str(),
                    L""));

            CopyUtf8(
                entry.sourceSpotName,
                ReadString(
                    filePath,
                    SectionColorMappingFile,
                    ColorMapKey(
                        i,
                        L"SourceSpotName").c_str(),
                    L""));

            entry.sourceSpotTint =
                std::clamp(
                    ReadInt(
                        filePath,
                        SectionColorMappingFile,
                        ColorMapKey(
                            i,
                            L"SourceSpotTint").c_str(),
                        100),
                    0,
                    100);

            entry.targetKind =
                static_cast<UI::ColorMapTargetKind>(
                    std::clamp(
                        ReadInt(
                            filePath,
                            SectionColorMappingFile,
                            ColorMapKey(i, L"TargetKind").c_str(),
                            0),
                        0,
                        1));

            for (int channel = 0; channel < 3; ++channel)
            {
                const std::wstring suffix =
                    L"TargetRgb" +
                    std::to_wstring(channel);

                entry.targetRgb[
                    static_cast<std::size_t>(
                        channel)] =
                    std::clamp(
                        ReadInt(
                            filePath,
                            SectionColorMappingFile,
                            ColorMapKey(
                                i,
                                suffix.c_str()).c_str(),
                            0),
                        0,
                        255);
            }

            CopyUtf8(
                entry.targetSpotPalette,
                ReadString(
                    filePath,
                    SectionColorMappingFile,
                    ColorMapKey(
                        i,
                        L"TargetSpotPalette").c_str(),
                    L""));

            CopyUtf8(
                entry.targetSpotName,
                ReadString(
                    filePath,
                    SectionColorMappingFile,
                    ColorMapKey(
                        i,
                        L"TargetSpotName").c_str(),
                    L""));

            entry.targetSpotTint =
                std::clamp(
                    ReadInt(
                        filePath,
                        SectionColorMappingFile,
                        ColorMapKey(
                            i,
                            L"TargetSpotTint").c_str(),
                        100),
                    0,
                    100);

            imported.push_back(
                std::move(entry));
        }

        color.colorMappingEnabled =
            ReadInt(
                filePath,
                SectionColorMappingFile,
                L"Enabled",
                color.colorMappingEnabled ? 1 : 0) != 0;

        color.colorMappingTolerance =
            std::clamp(
                ReadInt(
                    filePath,
                    SectionColorMappingFile,
                    L"Tolerance",
                    color.colorMappingTolerance),
                0,
                255);

        color.colorMappings =
            std::move(imported);

        return true;
    }

    bool SettingsStore::ExportSettings(
        const std::filesystem::path& filePath,
        const UI::State& state,
        const StoredProfiles& profiles,
        std::wstring& error)
    {
        error.clear();

        if (filePath.empty())
        {
            error = L"Invalid ImCut settings file path.";
            return false;
        }

        if (!EnsureDirectory(filePath))
        {
            error = L"Unable to create the destination directory.";
            return false;
        }

        AtomicIni transaction(filePath, false);
        if (!transaction.Ready())
        {
            error = L"Unable to stage the settings file.";
            return false;
        }
        const auto& stagedPath = transaction.Path();

        SettingsStore exportStore(stagedPath);

        if (!exportStore.Save(
            state,
            profiles))
        {
            error = L"Unable to write the ImCut settings file.";
            return false;
        }

        BufferedIni writer(stagedPath);
        bool ok = true;

        ok = WriteInt(
            writer,
            SectionSettingsFile,
            L"Version",
            1) && ok;

        ok = WriteInt(
            writer,
            SectionSettingsFile,
            L"IncludesColorMapping",
            1) && ok;

        if (!ok)
        {
            error = L"Unable to finalize the ImCut settings file.";
            return false;
        }

        WritePrivateProfileStringW(
            nullptr,
            nullptr,
            nullptr,
            stagedPath.c_str());

        if (!writer.Flush() || !transaction.Commit())
        {
            error = L"Unable to replace the destination file. The previous file was preserved.";
            return false;
        }
        return true;
    }

    bool SettingsStore::ImportSettings(
        const std::filesystem::path& filePath,
        UI::State& state,
        StoredProfiles& profiles,
        std::wstring& error)
    {
        error.clear();

        if (filePath.empty())
        {
            error = L"Invalid ImCut settings file path.";
            return false;
        }

        std::error_code existsError;

        if (!std::filesystem::exists(
            filePath,
            existsError) ||
            existsError)
        {
            error = L"ImCut settings file was not found.";
            return false;
        }

        const int version =
            ReadInt(
                filePath,
                SectionSettingsFile,
                L"Version",
                0);

        if (version != 1)
        {
            error = L"Unsupported or invalid ImCut settings file.";
            return false;
        }

        const int rawColorMapCount =
            ReadInt(
                filePath,
                SectionColor,
                L"ColorMapCount",
                -1);

        if (rawColorMapCount < 0 ||
            rawColorMapCount > 64)
        {
            error = L"Invalid Color Mapping data in the ImCut settings file.";
            return false;
        }

        UI::State importedState = state;
        StoredProfiles importedProfiles = profiles;

        const UI::Page currentPage =
            state.page;

        const UI::RuntimeState currentRuntime =
            state.runtime;

        const UI::UpdateState currentUpdate =
            state.update;

        const auto cmykProfiles =
            state.color.cmykProfiles;

        const auto rgbProfiles =
            state.color.rgbProfiles;

        SettingsStore importStore(filePath);

        importStore.Load(
            importedState,
            importedProfiles);

        importedState.page =
            currentPage;

        importedState.runtime =
            currentRuntime;

        importedState.update =
            currentUpdate;

        importedState.color.cmykProfiles =
            cmykProfiles;

        importedState.color.rgbProfiles =
            rgbProfiles;

        state =
            std::move(importedState);

        profiles =
            importedProfiles;

        return true;
    }

    const std::filesystem::path& SettingsStore::FilePath() const noexcept
    {
        return filePath_;
    }

    std::filesystem::path SettingsStore::ResolveFilePath()
    {
        DWORD size =
            GetEnvironmentVariableW(
                L"LOCALAPPDATA",
                nullptr,
                0);

        if (size > 1)
        {
            std::wstring value(
                size,
                L'\0');

            const DWORD written =
                GetEnvironmentVariableW(
                    L"LOCALAPPDATA",
                    value.data(),
                    size);

            if (written > 0)
            {
                value.resize(
                    written);

                return
                    std::filesystem::path(
                        value) /
                    L"ImCut" /
                    L"settings.ini";
            }
        }

        std::error_code error;

        const auto temp =
            std::filesystem::temp_directory_path(
                error);

        if (!error)
        {
            return
                temp /
                L"ImCut" /
                L"settings.ini";
        }

        return
            std::filesystem::path(
                L"settings.ini");
    }
}
