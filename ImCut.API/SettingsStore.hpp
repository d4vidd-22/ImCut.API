#pragma once

#include "UI.hpp"

#include <cstdint>
#include <filesystem>

namespace ImCut
{
    struct StoredProfiles
    {
        std::uint64_t cmykFingerprint = 0;
        std::uint64_t rgbFingerprint = 0;
    };

    class SettingsStore final
    {
    public:
        SettingsStore();
        explicit SettingsStore(
            std::filesystem::path filePath);

        void Load(
            UI::State& state,
            StoredProfiles& profiles) const;

        [[nodiscard]] bool Save(
            const UI::State& state,
            const StoredProfiles& profiles) const;

        [[nodiscard]] std::uint64_t Signature(
            const UI::State& state,
            const StoredProfiles& profiles) const noexcept;

        [[nodiscard]] const std::filesystem::path& FilePath() const noexcept;

        [[nodiscard]] static bool ExportColorMappings(
            const std::filesystem::path& filePath,
            const UI::ColorState& color,
            std::wstring& error);

        [[nodiscard]] static bool ImportColorMappings(
            const std::filesystem::path& filePath,
            UI::ColorState& color,
            std::wstring& error);

        [[nodiscard]] static bool ExportSettings(
            const std::filesystem::path& filePath,
            const UI::State& state,
            const StoredProfiles& profiles,
            std::wstring& error);

        [[nodiscard]] static bool ImportSettings(
            const std::filesystem::path& filePath,
            UI::State& state,
            StoredProfiles& profiles,
            std::wstring& error);

    private:
        [[nodiscard]] static std::filesystem::path ResolveFilePath();

        std::filesystem::path filePath_;
    };
}