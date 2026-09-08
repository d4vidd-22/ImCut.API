#pragma once
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace ImCut::Updater
{
    
    bool ValidateImage(const std::vector<std::uint8_t>& bytes, std::string& error);
    std::filesystem::path RunningModulePath();
    bool ValidateArtifact(const std::filesystem::path& file,
        const std::filesystem::path& installedModule,
        const std::string& expectedVersion, std::string& error);
}
