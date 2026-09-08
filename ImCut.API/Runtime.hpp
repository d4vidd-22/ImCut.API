#pragma once

#include "Exports.hpp"

#include <Windows.h>
#include <Unknwn.h>

#include <memory>

namespace ImCut
{
    class Runtime final
    {
    public:
        static Runtime& Instance();

        void ReportBoundaryError(HRESULT hr, const wchar_t* message) noexcept;

        HRESULT Initialize(
            IUnknown* corelApplication);

        HRESULT ConnectCorel2026();
        HRESULT ConnectCorel2026Hidden();

        HRESULT Show();
        HRESULT Hide();
        HRESULT Toggle();
        HRESULT Shutdown();

        [[nodiscard]] bool IsInitialized() const noexcept;
        [[nodiscard]] bool IsVisible() const noexcept;

        HRESULT RunCut(
            int pageWidthMm,
            int registrationMarks,
            int closureMode);

        HRESULT RunBleed(
            double distanceMm,
            bool ungroupBeforeProcessing,
            bool detectHiddenObjects,
            bool createCutline);

        HRESULT RefreshColorProfiles();

        HRESULT ConvertSelection(
            int cmykProfileIndex,
            int rgbProfileIndex,
            int renderingIntent,
            bool adaptiveLut,
            int preferredGrid);

        HRESULT RunCutSaved();
        HRESULT RunBleedSaved();
        HRESULT ConvertSelectionSaved();

        void SetNestingCallback(
            ImCutNestingCallback callback) noexcept;

        int CopyLastError(
            wchar_t* buffer,
            int capacity) const;

        [[nodiscard]] const wchar_t* Version() const noexcept;

        LRESULT WindowProc(
            HWND hwnd,
            UINT message,
            WPARAM wParam,
            LPARAM lParam);

    private:
        class Impl;

        Runtime();
        ~Runtime();

        Runtime(const Runtime&) = delete;
        Runtime& operator=(const Runtime&) = delete;

        std::unique_ptr<Impl> impl_;
    };
}