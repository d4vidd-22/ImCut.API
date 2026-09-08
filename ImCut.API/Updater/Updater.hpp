#pragma once

#include "../Version.hpp"

#define NOMINMAX 1
#include <Windows.h>

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>

namespace ImCut::Updater
{
    enum class Status
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

    struct Snapshot
    {
        Status status = Status::Idle;
        std::string currentVersion = IMCUT_VERSION_STRING;
        std::string remoteVersion;
        std::string message;
        float progress = 0.0f;
        bool updateAvailable = false;
        std::wstring downloadedPath;
    };

    class Service final
    {
    public:
        Service();
        ~Service();

        Service(const Service&) = delete;
        Service& operator=(const Service&) = delete;

        void CheckAsync();
        void DownloadAsync();
        void Cancel() noexcept;
        void Stop();

        [[nodiscard]] Snapshot GetSnapshot() const;
        [[nodiscard]] bool OpenDownloaded(HWND owner) const;
        [[nodiscard]] bool IsBusy() const noexcept;

        [[nodiscard]] static constexpr const char* CurrentVersion() noexcept
        {
            return IMCUT_VERSION_STRING;
        }

        [[nodiscard]] static constexpr const wchar_t* CurrentVersionWide() noexcept
        {
            return IMCUT_VERSION_WSTRING;
        }

        [[nodiscard]] static constexpr const wchar_t* VersionUrl() noexcept
        {
            return L"https://raw.githubusercontent.com/tebaldigraphicspcfour-lgtm/tebaldi-macros/main/versao";
        }

        [[nodiscard]] static constexpr const wchar_t* DownloadManifestUrl() noexcept
        {
            return L"https://raw.githubusercontent.com/tebaldigraphicspcfour-lgtm/tebaldi-macros/main/dl";
        }

    private:
        void JoinWorker();
        void RunCheck();
        void RunDownload();

        void SetSnapshot(
            Status status,
            std::string remoteVersion,
            std::string message,
            float progress = 0.0f,
            std::wstring downloadedPath = {});

        mutable std::mutex mutex_;
        Snapshot snapshot_;
        std::jthread worker_;
        std::atomic_bool cancelRequested_{ false };
        std::atomic_bool busy_{ false };
    };
}
