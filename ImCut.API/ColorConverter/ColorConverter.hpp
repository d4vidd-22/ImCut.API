#pragma once

#include "..\Global.hpp"
#include <exception>

namespace ImCut::Color
{
    class OperationCancelled final : public std::exception
    {
    public:
        const char* what() const noexcept override
        {
            return "Conversao cancelada com Esc. As cores ja concluidas foram mantidas; use Desfazer para reverter o lote.";
        }
    };
    enum class RenderingIntent : std::uint32_t
    {
        Perceptual = 0,
        RelativeColorimetric = 1,
        Saturation = 2,
        AbsoluteColorimetric = 3
    };

    enum class InterpolationMode : std::uint32_t
    {
        Multilinear4D = 0,
        Simplex4D = 1
    };

    enum class ConversionMode : std::uint32_t
    {
        StandardIcc = 0,
        PreserveAppearance = 1
    };

    struct Cmyk
    {
        double c = 0.0;
        double m = 0.0;
        double y = 0.0;
        double k = 0.0;
    };

    struct Rgb8
    {
        std::uint8_t r = 0;
        std::uint8_t g = 0;
        std::uint8_t b = 0;
    };

    struct Lab
    {
        double l = 0.0;
        double a = 0.0;
        double b = 0.0;
    };

    struct AppearanceMatch
    {
        Rgb8 rgb;
        Lab targetLab;
        Lab resultingLab;
        double deltaE00 = 0.0;
        double deltaE76 = 0.0;
        double deltaL = 0.0;
        double deltaA = 0.0;
        double deltaB = 0.0;
        bool usedGlobalSearch = false;
        bool usedDenseSearch = false;
        bool usedExhaustiveSearch = false;
    };

    struct Rgb16
    {
        std::uint16_t r = 0;
        std::uint16_t g = 0;
        std::uint16_t b = 0;
    };

    struct ProfileInfo
    {
        std::filesystem::path path;
        std::wstring displayName;
        std::uint32_t colorSpace = 0;
        std::uint32_t profileClass = 0;
        std::uint32_t renderingIntent = 0;
        std::uint64_t fingerprint = 0;
        bool isIcc = false;
    };

    struct ValidationStats
    {
        std::uint32_t samples = 0;
        double maxChannelError8 = 0.0;
        double meanAbsoluteError8 = 0.0;
        double rmsError8 = 0.0;
    };

    struct BuildOptions
    {
        RenderingIntent intent = RenderingIntent::Perceptual;
        InterpolationMode interpolation = InterpolationMode::Multilinear4D;
        ConversionMode conversionMode = ConversionMode::StandardIcc;
        std::uint32_t appearanceSearchRadius = 6;
        bool appearanceExhaustive = false;
        bool enableLabTargetTransform = false;
        bool adaptiveGrid = true;
        bool enableDiskCache = true;
        bool fallbackToDirectOnQualityFailure = true;
        bool blackFloorEnabled = true;
        std::uint8_t blackFloorRgb = 28;
        std::uint32_t preferredGrid = 33;
        std::array<std::uint32_t, 4> adaptiveGrids{ 17, 25, 33, 41 };
        std::uint32_t adaptiveGridCount = 4;
        std::uint32_t validationSamples = 2048;
        std::uint32_t buildChunkSize = 32768;
        double maxAllowedChannelError8 = 1.25;
        double maxAllowedRmsError8 = 0.30;
        std::filesystem::path cacheDirectory;
    };

    class ProfileCatalog final
    {
    public:
        [[nodiscard]] static std::vector<ProfileInfo> EnumerateCMYK();
        [[nodiscard]] static std::vector<ProfileInfo> EnumerateRGB();
        [[nodiscard]] static std::vector<ProfileInfo> EnumerateGray();
        [[nodiscard]] static ProfileInfo Inspect(const std::filesystem::path& profilePath);

    private:
        [[nodiscard]] static std::vector<ProfileInfo> Enumerate(std::uint32_t colorSpace);
    };

    class IccProfile final
    {
    public:
        explicit IccProfile(const ProfileInfo& info);
        explicit IccProfile(const std::filesystem::path& path);
        ~IccProfile() noexcept;

        IccProfile(const IccProfile&) = delete;
        IccProfile& operator=(const IccProfile&) = delete;
        IccProfile(IccProfile&& other) noexcept;
        IccProfile& operator=(IccProfile&& other) noexcept;

        [[nodiscard]] const ProfileInfo& Info() const noexcept;
        [[nodiscard]] void* NativeHandle() const noexcept;

    private:
        void Open(const std::filesystem::path& path);
        void Close() noexcept;
        void ReadMetadata();

        ProfileInfo info_;
        void* handle_ = nullptr;
    };

    class IccTransform final
    {
    public:
        IccTransform(const IccProfile& source, const IccProfile& target, RenderingIntent intent);
        ~IccTransform() noexcept;

        IccTransform(const IccTransform&) = delete;
        IccTransform& operator=(const IccTransform&) = delete;
        IccTransform(IccTransform&& other) noexcept;
        IccTransform& operator=(IccTransform&& other) noexcept;

        [[nodiscard]] Rgb16 Translate(const Cmyk& color) const;
        void Translate(const Cmyk* input, Rgb16* output, std::size_t count) const;
        [[nodiscard]] void* NativeHandle() const noexcept;

    private:
        void Close() noexcept;
        void* handle_ = nullptr;
    };


    class AppearanceTransform final
    {
    public:
        AppearanceTransform(
            const IccProfile& source,
            const IccProfile& target,
            std::uint32_t searchRadius,
            bool forceExhaustive = false);
        ~AppearanceTransform() noexcept;

        AppearanceTransform(const AppearanceTransform&) = delete;
        AppearanceTransform& operator=(const AppearanceTransform&) = delete;
        AppearanceTransform(AppearanceTransform&& other) noexcept;
        AppearanceTransform& operator=(AppearanceTransform&& other) noexcept;

        [[nodiscard]] bool Ready() const noexcept;
        [[nodiscard]] Rgb8 Translate(const Cmyk& color) const;
        [[nodiscard]] Rgb8 TranslateLab(const Lab& color) const;
        [[nodiscard]] AppearanceMatch TranslateDetailed(const Cmyk& color) const;
        [[nodiscard]] AppearanceMatch TranslateLabDetailed(const Lab& color) const;

    private:
        struct Impl;
        std::unique_ptr<Impl> impl_;
    };


    class RgbToSrgbConverter final
    {
    public:
        RgbToSrgbConverter() = default;
        ~RgbToSrgbConverter() noexcept;

        RgbToSrgbConverter(const RgbToSrgbConverter&) = delete;
        RgbToSrgbConverter& operator=(const RgbToSrgbConverter&) = delete;
        RgbToSrgbConverter(RgbToSrgbConverter&& other) noexcept;
        RgbToSrgbConverter& operator=(RgbToSrgbConverter&& other) noexcept;

        void Initialize(
            const ProfileInfo& sourceRgb,
            RenderingIntent intent = RenderingIntent::RelativeColorimetric);

        void Reset() noexcept;

        [[nodiscard]] bool Ready() const noexcept;
        [[nodiscard]] Rgb8 Convert(const Rgb8& color) const;
        [[nodiscard]] const ProfileInfo& SourceProfile() const;
        [[nodiscard]] const ProfileInfo& SrgbProfile() const;

    private:
        void CloseTransform() noexcept;

        std::unique_ptr<IccProfile> source_;
        std::unique_ptr<IccProfile> srgb_;
        void* transform_ = nullptr;
    };

    class IccLut4D final
    {
    public:
        IccLut4D() = default;

        void Build(const IccTransform& transform, std::uint32_t gridSize, std::uint32_t chunkSize);

        [[nodiscard]] bool Load(
            const std::filesystem::path& file,
            std::uint64_t sourceFingerprint,
            std::uint64_t targetFingerprint,
            RenderingIntent intent,
            std::uint32_t gridSize);

        [[nodiscard]] bool Save(
            const std::filesystem::path& file,
            std::uint64_t sourceFingerprint,
            std::uint64_t targetFingerprint,
            RenderingIntent intent) const;

        [[nodiscard]] Rgb16 Sample16(const Cmyk& color, InterpolationMode mode) const;
        [[nodiscard]] Rgb8 Sample8(const Cmyk& color, InterpolationMode mode) const;
        [[nodiscard]] std::uint32_t GridSize() const noexcept;
        [[nodiscard]] std::size_t EntryCount() const noexcept;
        [[nodiscard]] bool Empty() const noexcept;
        void Clear() noexcept;

    private:
        struct Axis
        {
            std::uint32_t low = 0;
            std::uint32_t high = 0;
            double fraction = 0.0;
        };

        [[nodiscard]] Axis ResolveAxis(double percent) const noexcept;
        [[nodiscard]] std::size_t Offset(std::uint32_t c, std::uint32_t m, std::uint32_t y, std::uint32_t k) const noexcept;
        [[nodiscard]] Rgb16 SampleMultilinear(const Axis& c, const Axis& m, const Axis& y, const Axis& k) const;
        [[nodiscard]] Rgb16 SampleSimplex(const Axis& c, const Axis& m, const Axis& y, const Axis& k) const;

        std::uint32_t gridSize_ = 0;
        std::vector<Rgb16> data_;
    };

    class Converter final
    {
    public:
        Converter() = default;

        void Initialize(
            const ProfileInfo& sourceCmyk,
            const ProfileInfo& targetRgb,
            const BuildOptions& options = {});

        void Reset() noexcept;

        [[nodiscard]] bool Ready() const noexcept;
        [[nodiscard]] bool LoadedFromCache() const noexcept;
        [[nodiscard]] bool UsingDirectFallback() const noexcept;
        [[nodiscard]] Rgb8 Convert(const Cmyk& color) const;
        [[nodiscard]] Rgb16 Convert16(const Cmyk& color) const;
        [[nodiscard]] AppearanceMatch ConvertDetailed(const Cmyk& color) const;
        [[nodiscard]] Rgb8 ConvertLab(const Lab& color) const;
        [[nodiscard]] Rgb16 ConvertLab16(const Lab& color) const;
        [[nodiscard]] AppearanceMatch ConvertLabDetailed(const Lab& color) const;
        void Convert(const Cmyk* input, Rgb8* output, std::size_t count) const;
        void Convert16(const Cmyk* input, Rgb16* output, std::size_t count) const;
        [[nodiscard]] Rgb16 ConvertDirect16(const Cmyk& color) const;
        [[nodiscard]] ValidationStats Validate(std::uint32_t samples = 0) const;
        [[nodiscard]] const ProfileInfo& SourceProfile() const;
        [[nodiscard]] const ProfileInfo& TargetProfile() const;
        [[nodiscard]] const BuildOptions& Options() const noexcept;
        [[nodiscard]] const ValidationStats& LastValidation() const noexcept;
        [[nodiscard]] std::uint32_t GridSize() const noexcept;
        [[nodiscard]] static std::filesystem::path DefaultCacheDirectory();

    private:
        [[nodiscard]] ValidationStats ValidateLut(const IccLut4D& lut, std::uint32_t samples) const;
        [[nodiscard]] std::filesystem::path CacheFileFor(std::uint32_t gridSize) const;
        [[nodiscard]] static double Halton(std::uint32_t index, std::uint32_t base) noexcept;
        [[nodiscard]] static bool MeetsQualityTarget(const ValidationStats& stats, const BuildOptions& options) noexcept;

        BuildOptions options_;
        std::unique_ptr<IccProfile> source_;
        std::unique_ptr<IccProfile> target_;
        std::unique_ptr<IccTransform> transform_;
        std::unique_ptr<AppearanceTransform> appearanceTransform_;
        IccLut4D lut_;
        ValidationStats lastValidation_;
        bool loadedFromCache_ = false;
        bool directFallback_ = false;
    };
}
