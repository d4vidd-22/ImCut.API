#pragma once

#include "Version.hpp"
#include <array>
#include <cstddef>
#include <functional>
#include <string>
#include <vector>

struct ImFont;

namespace ImCut::UI
{
    enum class Page
    {
        Cut = 0,
        Bleed = 1,
        Color = 2,
        Nesting = 3,
        Settings = 4
    };

    enum class Language
    {
        Portuguese = 0,
        English = 1
    };

    enum class ClosureMode
    {
        Automatic = 0,
        WholeSelection,
        EachSelectedGroup,
        EachSelectedObject
    };

    enum class RenderingIntent
    {
        Perceptual = 0,
        RelativeColorimetric,
        Saturation,
        AbsoluteColorimetric
    };

    enum class ColorConversionMode
    {
        StandardIcc = 0,
        PreserveAppearance = 1
    };

    struct CutState
    {
        int pageWidthMm = 1352;
        int registrationMarks = 20;
        float registrationMarginMm = 5.0f;
        ClosureMode closureMode = ClosureMode::Automatic;
        bool namePages = true;
        bool showSummary = true;
        bool forceBottomRightAnchor = true;
    };

    struct BleedState
    {
        float distanceMm = 2.0f;
        bool ungroupBeforeProcessing = true;
        bool detectHiddenObjects = true;
        bool createCutline = true;
    };

    enum class ColorMapSourceKind
    {
        RGB = 0,
        CMYK = 1,
        Spot = 2
    };

    enum class ColorMapTargetKind
    {
        RGB = 0,
        Spot = 1
    };

    struct ColorMapEntry
    {
        bool enabled = true;
        ColorMapSourceKind sourceKind = ColorMapSourceKind::RGB;
        std::array<int, 4> sourceChannels{ 0, 0, 0, 0 };
        std::array<char, 128> sourceSpotPalette{};
        std::array<char, 512> sourceSpotName{};
        int sourceSpotTint = 100;
        ColorMapTargetKind targetKind = ColorMapTargetKind::RGB;
        std::array<int, 3> targetRgb{ 0, 0, 0 };
        std::array<char, 128> targetSpotPalette{};
        std::array<char, 128> targetSpotName{};
        int targetSpotTint = 100;
    };


    struct ColorTargetPreview
    {
        bool valid = false;
        std::array<int, 3> srgb{ 0, 0, 0 };
        std::string sourceProfile;
        std::string message;
    };

    struct ColorState
    {
        std::vector<std::string> cmykProfiles;
        std::vector<std::string> rgbProfiles;
        int selectedCmyk = -1;
        int selectedRgb = -1;
        RenderingIntent intent = RenderingIntent::Perceptual;
        ColorConversionMode conversionMode = ColorConversionMode::StandardIcc;
        bool exhaustiveAppearanceSearch = false;
        bool assignDocumentProfiles = false;
        bool adaptiveLut = true;
        int preferredGrid = 33;
        bool showSummary = true;
        bool blackFloorEnabled = true;
        int blackFloorRgb = 28;
        bool spotTintWhiteEnabled = true;
        int spotTintWhiteThreshold = 0;
        bool convertSpotsToRgb = true;
        bool preserveSpotAppearance = true;
        bool spotBlacklistEnabled = false;
        std::array<char, 512> spotBlacklistNames{};
        bool colorMappingEnabled = false;
        int colorMappingTolerance = 0;
        std::vector<ColorMapEntry> colorMappings;
    };

    struct NestingState
    {
        float spacingMm = 3.0f;
        int rotationStepDeg = 90;
        int generations = 250;
        int population = 80;
        bool allowMirror = false;
        bool useTrueShape = true;
    };

    enum class UpdateStatus
    {
        Idle = 0,
        Checking,
        UpToDate,
        UpdateAvailable,
        ResolvingDownload,
        Downloading,
        Downloaded,
        Error
    };

    struct UpdateState
    {
        UpdateStatus status = UpdateStatus::Idle;
        std::string currentVersion = IMCUT_VERSION_STRING;
        std::string remoteVersion;
        std::string message;
        float progress = 0.0f;
        bool updateAvailable = false;
    };

    struct RuntimeState
    {
        bool busy = false;
        float progress = 0.0f;
        int detectedClosures = 0;
        std::string status = "Ready";
    };

    struct State
    {
        Page page = Page::Cut;
        Language language = Language::Portuguese;
        CutState cut;
        BleedState bleed;
        ColorState color;
        NestingState nesting;
        RuntimeState runtime;
        UpdateState update;
    };

    struct Callbacks
    {
        std::function<void(const CutState&)> runCut;
        std::function<void(const BleedState&)> runBleed;
        std::function<void(const ColorState&)> runColor;
        std::function<void(const NestingState&)> runNesting;
        std::function<void()> refreshColorProfiles;
        std::function<void()> importColorMappings;
        std::function<void()> exportColorMappings;
        std::function<void()> importSettings;
        std::function<void()> exportSettings;
        std::function<ColorTargetPreview(const ColorMapEntry&, const ColorState&)> previewColorMappingTarget;
        std::function<void()> checkForUpdates;
        std::function<void()> downloadUpdate;
        std::function<void()> openDownloadedUpdate;
    };

    void RequestPrimaryActionFocus(Page page) noexcept;
    void SetFonts(ImFont* regular, ImFont* semibold) noexcept;
    void ApplyCorelTheme(float dpiScale = 1.0f);
    void Draw(State& state, const Callbacks& callbacks, bool* open = nullptr);
}
