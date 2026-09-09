#include "Version.hpp"
#define NOMINMAX 1

#include "Runtime.hpp"
#include "ScopeExit.hpp"
#include "InputLimits.hpp"
#include "ApiBoundary.hpp"
#include "OperationCancellation.hpp"

#include "AutoBleeding/AutoBleeding.hpp"
#include "ColorConverter/ColorConverter.hpp"
#include "Cut/Cut.hpp"
#include "UI.hpp"
#include "SettingsStore.hpp"
#include "Updater/Updater.hpp"
#include "Updater/ArtifactValidation.hpp"

#if !defined(IMCUT_CUT_API_VERSION) || IMCUT_CUT_API_VERSION != 2
#error "ImCutRuntime requires the Cut.hpp shipped with this package."
#endif

#if !defined(IMCUT_AUTOBLEEDING_API_VERSION) || IMCUT_AUTOBLEEDING_API_VERSION != 2
#error "ImCutRuntime requires the AutoBleeding.hpp shipped with this package."
#endif

#include <imgui.h>
#include <imgui_impl_dx11.h>
#include <imgui_impl_win32.h>

#include <Windows.h>
#include <OleAuto.h>
#include <commdlg.h>



extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND hwnd,
    UINT message,
    WPARAM wParam,
    LPARAM lParam);
#include <d3d11.h>
#include <dxgi.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <cmath>
#include <cwchar>
#include <cwctype>
#include <filesystem>
#include <functional>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <unordered_map>
#include <vector>

#pragma comment(lib, "d3d11.lib")
#pragma comment(lib, "dxgi.lib")
#pragma comment(lib, "Ole32.lib")
#pragma comment(lib, "OleAut32.lib")
#pragma comment(lib, "Comdlg32.lib")

namespace ImCut
{
    namespace
    {
        constexpr wchar_t WindowClassName[] =
            L"ImCut.Runtime.Window";

        constexpr wchar_t WindowTitle[] =
            L"ImCut";

        constexpr UINT_PTR RenderTimerId = 0x494D4355;
        constexpr UINT ActiveRenderTimerMs = 16;
        constexpr UINT UpdateRenderTimerMs = 50;
        constexpr UINT IdleRenderTimerMs = 200;
        constexpr UINT BackgroundRenderTimerMs = 500;
        constexpr ULONGLONG UiActivityHoldMs = 360;
        constexpr ULONGLONG SelectionProbeIdleDelayMs = 450;
        constexpr ULONGLONG PreviewComputeIdleDelayMs = 700;
        constexpr ULONGLONG SelectionProbeIntervalMs = 1800;
        constexpr ULONGLONG UpdateSyncIntervalMs = 150;
        constexpr ULONGLONG SettingsSignatureIntervalMs = 650;
        constexpr ULONGLONG SettingsSaveDebounceMs = 850;
        constexpr ULONGLONG SettingsIdleDelayMs = 500;

        constexpr UINT MessageRunCut =
            WM_APP + 0x120;

        constexpr UINT MessageRunBleed =
            WM_APP + 0x121;

        constexpr UINT MessageRunColor =
            WM_APP + 0x122;

        constexpr UINT MessageRunNesting =
            WM_APP + 0x123;

        constexpr UINT MessageRefreshProfiles =
            WM_APP + 0x124;

        constexpr UINT MessageImportColorMappings =
            WM_APP + 0x125;

        constexpr UINT MessageExportColorMappings =
            WM_APP + 0x126;

        constexpr UINT MessageShutdownForUpdate =
            WM_APP + 0x127;

        constexpr UINT MessageImportSettings =
            WM_APP + 0x128;

        constexpr UINT MessageExportSettings =
            WM_APP + 0x129;

        constexpr UINT MessageAutoInstallUpdate =
            WM_APP + 0x12A;

        constexpr UINT MessageRenderNow =
            WM_APP + 0x12B;

        constexpr wchar_t UpdaterProjectName[] =
            L"ImCutUpdater";

        constexpr wchar_t UpdaterScheduleMacro[] =
            L"Updater.ScheduleInstall";

        constexpr float UiClientWidth = 680.0f;
        constexpr float UiClientHeight = 690.0f;

        [[nodiscard]] std::wstring GetModuleFilePath(HMODULE module)
        {
            if (!module)
                return {};

            std::vector<wchar_t> buffer(1024);

            for (;;)
            {
                const DWORD length = GetModuleFileNameW(
                    module,
                    buffer.data(),
                    static_cast<DWORD>(buffer.size()));

                if (length == 0)
                    return {};

                if (length < buffer.size())
                    return std::wstring(buffer.data(), length);

                if (buffer.size() >= 32768)
                    return {};

                buffer.resize(buffer.size() * 2);
            }
        }

        [[nodiscard]] std::filesystem::path LocalAppDataGmsFallback()
        {
            DWORD size = GetEnvironmentVariableW(
                L"LOCALAPPDATA",
                nullptr,
                0);

            if (size > 1)
            {
                std::wstring value(size, L'\0');
                const DWORD written = GetEnvironmentVariableW(
                    L"LOCALAPPDATA",
                    value.data(),
                    size);

                if (written > 0)
                {
                    value.resize(written);
                    return std::filesystem::path(value) /
                        L"ImCut" /
                        L"GMSPath";
                }
            }

            std::error_code ec;
            const auto temp = std::filesystem::temp_directory_path(ec);

            if (!ec)
                return temp / L"ImCut" / L"GMSPath";

            return std::filesystem::path(L".") / L"ImCut" / L"GMSPath";
        }

        [[nodiscard]] std::filesystem::path LegacyTempUpdateControlDirectory()
        {
            std::error_code ec;
            const auto temp = std::filesystem::temp_directory_path(ec);

            if (ec)
                return {};

            auto path = temp / L"ImCut" / L"Updates";
            std::filesystem::create_directories(path, ec);
            return path;
        }

        [[nodiscard]] std::filesystem::path ResolveUserGmsPath(
            const IVGApplicationPtr& app)
        {
            if (!app)
                return {};

            try
            {
                app->InitializeVBA();

                auto manager = app->GMSManager;
                if (!manager)
                    return {};

                IDispatch* dispatch = nullptr;
                const HRESULT qi = manager->QueryInterface(
                    IID_IDispatch,
                    reinterpret_cast<void**>(&dispatch));

                if (FAILED(qi) || !dispatch)
                    return {};

                LPOLESTR propertyName =
                    const_cast<LPOLESTR>(L"UserGMSPath");

                DISPID dispid = DISPID_UNKNOWN;
                HRESULT hr = dispatch->GetIDsOfNames(
                    IID_NULL,
                    &propertyName,
                    1,
                    LOCALE_USER_DEFAULT,
                    &dispid);

                if (FAILED(hr))
                {
                    dispatch->Release();
                    return {};
                }

                DISPPARAMS parameters{};
                VARIANT result;
                VariantInit(&result);

                hr = dispatch->Invoke(
                    dispid,
                    IID_NULL,
                    LOCALE_USER_DEFAULT,
                    DISPATCH_PROPERTYGET,
                    &parameters,
                    &result,
                    nullptr,
                    nullptr);

                dispatch->Release();

                if (FAILED(hr))
                {
                    VariantClear(&result);
                    return {};
                }

                std::filesystem::path path;

                if (result.vt == VT_BSTR &&
                    result.bstrVal &&
                    SysStringLen(result.bstrVal) > 0)
                {
                    path = std::filesystem::path(
                        std::wstring(
                            result.bstrVal,
                            SysStringLen(result.bstrVal)));
                }

                VariantClear(&result);
                return path;
            }
            catch (...)
            {
                return {};
            }
        }

        [[nodiscard]] std::filesystem::path EffectiveGmsPath(
            const IVGApplicationPtr& app)
        {
            auto path = ResolveUserGmsPath(app);

            if (path.empty())
                path = LocalAppDataGmsFallback();

            std::error_code ec;
            std::filesystem::create_directories(path, ec);
            return path;
        }

        [[nodiscard]] std::filesystem::path DefaultImCutDllPath(
            const IVGApplicationPtr& app)
        {
            return EffectiveGmsPath(app) / L"ImCut.API.dll";
        }

        [[nodiscard]] std::filesystem::path UpdateControlDirectory(
            const IVGApplicationPtr& app)
        {
            auto path = EffectiveGmsPath(app) / L"ImCutUpdater";

            std::error_code ec;
            std::filesystem::create_directories(path, ec);
            return path;
        }

        [[nodiscard]] bool WriteUtf16LeFile(
            const std::filesystem::path& path,
            const std::wstring& text,
            std::wstring& error)
        {
            error.clear();

            HANDLE file = CreateFileW(
                path.c_str(),
                GENERIC_WRITE,
                FILE_SHARE_READ,
                nullptr,
                CREATE_ALWAYS,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);

            if (file == INVALID_HANDLE_VALUE)
            {
                error = L"Não foi possível criar o arquivo de controle da atualização.";
                return false;
            }

            const unsigned char bom[2] = { 0xFF, 0xFE };
            DWORD written = 0;
            bool ok = WriteFile(file, bom, 2, &written, nullptr) != FALSE && written == 2;

            if (ok && !text.empty())
            {
                const DWORD bytes = static_cast<DWORD>(
                    text.size() * sizeof(wchar_t));

                written = 0;
                ok = WriteFile(
                    file,
                    text.data(),
                    bytes,
                    &written,
                    nullptr) != FALSE && written == bytes;
            }

            FlushFileBuffers(file);
            CloseHandle(file);

            if (!ok)
            {
                error = L"Falha ao gravar o arquivo de controle da atualização.";
                return false;
            }

            return true;
        }

        [[nodiscard]] bool TouchFile(
            const std::filesystem::path& path) noexcept
        {
            HANDLE file = CreateFileW(
                path.c_str(),
                GENERIC_WRITE,
                FILE_SHARE_READ | FILE_SHARE_WRITE | FILE_SHARE_DELETE,
                nullptr,
                CREATE_ALWAYS,
                FILE_ATTRIBUTE_NORMAL,
                nullptr);

            if (file == INVALID_HANDLE_VALUE)
                return false;

            FlushFileBuffers(file);
            CloseHandle(file);
            return true;
        }

        [[nodiscard]] bool InvokeGmsMacroNoArgs(
            const IVGApplicationPtr& app,
            const wchar_t* projectName,
            const wchar_t* macroName,
            std::wstring& error)
        {
            error.clear();

            if (!app || !projectName || !macroName)
            {
                error = L"Parâmetros inválidos para executar o macro do atualizador.";
                return false;
            }

            try
            {
                app->InitializeVBA();

                auto manager = app->GMSManager;
                if (!manager)
                {
                    error = L"O GMSManager do CorelDRAW não está disponível.";
                    return false;
                }

                IDispatch* dispatch = nullptr;
                const HRESULT qi = manager->QueryInterface(
                    IID_IDispatch,
                    reinterpret_cast<void**>(&dispatch));

                if (FAILED(qi) || !dispatch)
                {
                    error = L"Não foi possível acessar o GMSManager via IDispatch.";
                    return false;
                }

                LPOLESTR methodName = const_cast<LPOLESTR>(L"RunMacro");
                DISPID dispid = DISPID_UNKNOWN;
                HRESULT hr = dispatch->GetIDsOfNames(
                    IID_NULL,
                    &methodName,
                    1,
                    LOCALE_USER_DEFAULT,
                    &dispid);

                if (FAILED(hr))
                {
                    dispatch->Release();
                    error = L"O método GMSManager.RunMacro não foi encontrado.";
                    return false;
                }

                VARIANTARG args[2];
                VariantInit(&args[0]);
                VariantInit(&args[1]);

                args[0].vt = VT_BSTR;
                args[0].bstrVal = SysAllocString(macroName);
                args[1].vt = VT_BSTR;
                args[1].bstrVal = SysAllocString(projectName);

                DISPPARAMS parameters{};
                parameters.rgvarg = args;
                parameters.cArgs = 2;

                VARIANT result;
                VariantInit(&result);
                EXCEPINFO exceptionInfo{};
                UINT argumentError = 0;

                hr = dispatch->Invoke(
                    dispid,
                    IID_NULL,
                    LOCALE_USER_DEFAULT,
                    DISPATCH_METHOD,
                    &parameters,
                    &result,
                    &exceptionInfo,
                    &argumentError);

                VariantClear(&result);
                VariantClear(&args[0]);
                VariantClear(&args[1]);
                dispatch->Release();

                if (FAILED(hr))
                {
                    if (exceptionInfo.bstrDescription)
                    {
                        error.assign(
                            exceptionInfo.bstrDescription,
                            SysStringLen(exceptionInfo.bstrDescription));
                    }
                    else
                    {
                        std::wostringstream stream;
                        stream << L"Falha ao executar o macro do atualizador (HRESULT 0x"
                               << std::hex
                               << static_cast<unsigned long>(hr)
                               << L").";
                        error = stream.str();
                    }

                    if (exceptionInfo.bstrSource)
                        SysFreeString(exceptionInfo.bstrSource);
                    if (exceptionInfo.bstrDescription)
                        SysFreeString(exceptionInfo.bstrDescription);
                    if (exceptionInfo.bstrHelpFile)
                        SysFreeString(exceptionInfo.bstrHelpFile);

                    return false;
                }

                if (exceptionInfo.bstrSource)
                    SysFreeString(exceptionInfo.bstrSource);
                if (exceptionInfo.bstrDescription)
                    SysFreeString(exceptionInfo.bstrDescription);
                if (exceptionInfo.bstrHelpFile)
                    SysFreeString(exceptionInfo.bstrHelpFile);

                return true;
            }
            catch (const _com_error& comError)
            {
                try
                {
                    const _bstr_t description = comError.Description();
                    if (description.length() > 0)
                    {
                        error = static_cast<const wchar_t*>(description);
                        return false;
                    }
                }
                catch (...)
                {
                }

                error = L"Falha COM ao iniciar o macro do atualizador.";
                return false;
            }
            catch (...)
            {
                error = L"Falha desconhecida ao iniciar o macro do atualizador.";
                return false;
            }
        }

        [[nodiscard]] UINT ResolveDpi(HWND hwnd)
        {
            HMODULE user32 = GetModuleHandleW(L"user32.dll");

            if (user32 && hwnd)
            {
                using GetDpiForWindowFn = UINT(WINAPI*)(HWND);
                const auto getDpiForWindow =
                    reinterpret_cast<GetDpiForWindowFn>(
                        GetProcAddress(user32, "GetDpiForWindow"));

                if (getDpiForWindow)
                {
                    const UINT dpi = getDpiForWindow(hwnd);
                    if (dpi > 0)
                        return dpi;
                }
            }

            HWND dcOwner = hwnd;
            HDC dc = GetDC(dcOwner);

            if (!dc)
            {
                dcOwner = nullptr;
                dc = GetDC(nullptr);
            }

            if (dc)
            {
                const int dpi = GetDeviceCaps(dc, LOGPIXELSX);
                ReleaseDC(dcOwner, dc);

                if (dpi > 0)
                    return static_cast<UINT>(dpi);
            }

            return 96;
        }

        [[nodiscard]] float ResolveScale(HWND hwnd)
        {
            return static_cast<float>(ResolveDpi(hwnd)) / 96.0f;
        }

        void AdjustWindowRectForDpi(
            RECT& rect,
            DWORD style,
            DWORD exStyle,
            UINT dpi)
        {
            HMODULE user32 = GetModuleHandleW(L"user32.dll");

            if (user32)
            {
                using AdjustWindowRectExForDpiFn =
                    BOOL(WINAPI*)(LPRECT, DWORD, BOOL, DWORD, UINT);

                const auto adjustForDpi =
                    reinterpret_cast<AdjustWindowRectExForDpiFn>(
                        GetProcAddress(user32, "AdjustWindowRectExForDpi"));

                if (adjustForDpi &&
                    adjustForDpi(&rect, style, FALSE, exStyle, dpi))
                {
                    return;
                }
            }

            (void)AdjustWindowRectEx(&rect, style, FALSE, exStyle);
        }

        void ApplyDarkTitleBar(HWND hwnd)
        {
            if (!hwnd)
                return;

            HMODULE dwm = LoadLibraryW(L"dwmapi.dll");

            if (!dwm)
                return;

            using DwmSetWindowAttributeFn =
                HRESULT(WINAPI*)(HWND, DWORD, LPCVOID, DWORD);

            const auto setAttribute =
                reinterpret_cast<DwmSetWindowAttributeFn>(
                    GetProcAddress(dwm, "DwmSetWindowAttribute"));

            if (setAttribute)
            {
                const BOOL enabled = TRUE;
                HRESULT hr = setAttribute(hwnd, 20, &enabled, sizeof(enabled));

                if (FAILED(hr))
                    (void)setAttribute(hwnd, 19, &enabled, sizeof(enabled));
            }

            FreeLibrary(dwm);
        }

        void LoadUiFonts(float scale)
        {
            ImGuiIO& io = ImGui::GetIO();
            char windowsDirectory[MAX_PATH]{};
            ImFont* regular = nullptr;
            ImFont* semibold = nullptr;

            static const ImWchar glyphRanges[]
            {
                0x0020, 0x00FF,
                0x0100, 0x017F,
                0x2000, 0x206F,
                0x20AC, 0x20AC,
                0
            };

            if (GetWindowsDirectoryA(windowsDirectory, MAX_PATH) > 0)
            {
                const std::string base =
                    std::string(windowsDirectory) + "\\Fonts\\";

                const std::string regularPath =
                    base + "segoeui.ttf";

                const std::string semiboldPath =
                    base + "seguisb.ttf";

                ImFontConfig regularConfig{};
                regularConfig.OversampleH = 2;
                regularConfig.OversampleV = 2;
                regularConfig.PixelSnapH = false;

                if (GetFileAttributesA(regularPath.c_str()) != INVALID_FILE_ATTRIBUTES)
                {
                    regular = io.Fonts->AddFontFromFileTTF(
                        regularPath.c_str(),
                        15.5f * scale,
                        &regularConfig,
                        glyphRanges);
                }

                ImFontConfig semiboldConfig{};
                semiboldConfig.OversampleH = 2;
                semiboldConfig.OversampleV = 2;
                semiboldConfig.PixelSnapH = false;

                if (GetFileAttributesA(semiboldPath.c_str()) != INVALID_FILE_ATTRIBUTES)
                {
                    semibold = io.Fonts->AddFontFromFileTTF(
                        semiboldPath.c_str(),
                        15.5f * scale,
                        &semiboldConfig,
                        glyphRanges);
                }
            }

            if (!regular)
                regular = io.Fonts->AddFontDefault();

            if (!semibold)
                semibold = regular;

            io.FontDefault = regular;
            UI::SetFonts(regular, semibold);
        }

        [[nodiscard]] std::wstring Utf8ToWide(
            const std::string& value)
        {
            if (value.empty())
                return {};

            const int length =
                MultiByteToWideChar(
                    CP_UTF8,
                    0,
                    value.data(),
                    static_cast<int>(
                        value.size()),
                    nullptr,
                    0);

            if (length <= 0)
                return {};

            std::wstring output(
                static_cast<std::size_t>(
                    length),
                L'\0');

            MultiByteToWideChar(
                CP_UTF8,
                0,
                value.data(),
                static_cast<int>(
                    value.size()),
                output.data(),
                length);

            return output;
        }

        [[nodiscard]] std::string WideToUtf8(
            const std::wstring& value)
        {
            if (value.empty())
                return {};

            const int length =
                WideCharToMultiByte(
                    CP_UTF8,
                    0,
                    value.data(),
                    static_cast<int>(
                        value.size()),
                    nullptr,
                    0,
                    nullptr,
                    nullptr);

            if (length <= 0)
                return {};

            std::string output(
                static_cast<std::size_t>(
                    length),
                '\0');

            WideCharToMultiByte(
                CP_UTF8,
                0,
                value.data(),
                static_cast<int>(
                    value.size()),
                output.data(),
                length,
                nullptr,
                nullptr);

            return output;
        }

        [[nodiscard]] std::wstring ComErrorText(
            const _com_error& error)
        {
            try
            {
                const _bstr_t description =
                    error.Description();

                if (description.length() > 0)
                {
                    return std::wstring(
                        static_cast<const wchar_t*>(
                            description));
                }
            }
            catch (...)
            {
            }

            wchar_t* buffer = nullptr;

            const DWORD length =
                FormatMessageW(
                    FORMAT_MESSAGE_ALLOCATE_BUFFER |
                    FORMAT_MESSAGE_FROM_SYSTEM |
                    FORMAT_MESSAGE_IGNORE_INSERTS,
                    nullptr,
                    static_cast<DWORD>(error.Error()),
                    MAKELANGID(
                        LANG_NEUTRAL,
                        SUBLANG_DEFAULT),
                    reinterpret_cast<LPWSTR>(&buffer),
                    0,
                    nullptr);

            if (length > 0 && buffer)
            {
                std::wstring message(
                    buffer,
                    static_cast<std::size_t>(length));

                LocalFree(buffer);

                while (!message.empty() &&
                    (message.back() == L'\r' ||
                        message.back() == L'\n' ||
                        message.back() == L' ' ||
                        message.back() == L'\t'))
                {
                    message.pop_back();
                }

                if (!message.empty())
                    return message;
            }

            return L"Unknown COM error.";
        }

        LRESULT CALLBACK RuntimeWindowProc(
            HWND hwnd,
            UINT message,
            WPARAM wParam,
            LPARAM lParam)
        {
            try
            {
                return Runtime::Instance().WindowProc(
                    hwnd,
                    message,
                    wParam,
                    lParam);
            }
            catch (...)
            {
                (void)ReportCurrentException();
                if (message == WM_PAINT) ValidateRect(hwnd, nullptr);
                return message == WM_CREATE ? -1 : 0;
            }
        }

        class ColorExecutionGuard final
        {
        public:
            ColorExecutionGuard(
                IVGApplicationPtr app,
                IVGDocumentPtr doc)
                : app_(std::move(app)),
                doc_(std::move(doc))
            {
                oldOptimization_ =
                    app_->GetOptimization();

                oldEventsEnabled_ =
                    app_->GetEventsEnabled();

                try
                {
                    app_->PutOptimization(
                        VARIANT_TRUE);

                    app_->PutEventsEnabled(
                        VARIANT_FALSE);

                    doc_->BeginCommandGroup(
                        "ImCut - Color Converter");

                    commandGroupOpen_ = true;
                }
                catch (...) { Restore(); throw; }
            }

            ~ColorExecutionGuard() noexcept { Restore(); }
            void Restore() noexcept
            {
                if (commandGroupOpen_)
                {
                    try
                    {
                        doc_->EndCommandGroup();
                    }
                    catch (...)
                    {
                    }
                }

                try
                {
                    app_->PutEventsEnabled(
                        oldEventsEnabled_);
                }
                catch (...)
                {
                }

                try
                {
                    app_->PutOptimization(
                        oldOptimization_);
                }
                catch (...)
                {
                }

                try
                {
                    app_->ActiveWindow->Refresh();
                }
                catch (...)
                {
                }

                try
                {
                    app_->Refresh();
                }
                catch (...)
                {
                }
                commandGroupOpen_ = false;
            }

            ColorExecutionGuard(
                const ColorExecutionGuard&) = delete;

            ColorExecutionGuard& operator=(
                const ColorExecutionGuard&) = delete;

        private:
            IVGApplicationPtr app_;
            IVGDocumentPtr doc_;
            VARIANT_BOOL oldOptimization_ =
                VARIANT_FALSE;
            VARIANT_BOOL oldEventsEnabled_ =
                VARIANT_TRUE;
            bool commandGroupOpen_ = false;
        };
    }


    [[nodiscard]] static bool SelectColorMappingFile(
        HWND owner,
        bool save,
        std::filesystem::path& filePath,
        std::wstring& error)
    {
        filePath.clear();
        error.clear();

        std::vector<wchar_t> buffer(32768, L'\0');

        OPENFILENAMEW dialog{};
        dialog.lStructSize = sizeof(dialog);
        dialog.hwndOwner = owner;
        dialog.lpstrFilter =
            L"ImCut Color Mapping (*.imcutmap)\0*.imcutmap\0"
            L"All files (*.*)\0*.*\0";
        dialog.lpstrFile = buffer.data();
        dialog.nMaxFile =
            static_cast<DWORD>(
                buffer.size());
        dialog.lpstrDefExt = L"imcutmap";
        dialog.Flags =
            OFN_EXPLORER |
            OFN_NOCHANGEDIR |
            OFN_PATHMUSTEXIST;

        BOOL selected = FALSE;

        if (save)
        {
            dialog.Flags |=
                OFN_OVERWRITEPROMPT;

            selected =
                GetSaveFileNameW(
                    &dialog);
        }
        else
        {
            dialog.Flags |=
                OFN_FILEMUSTEXIST;

            selected =
                GetOpenFileNameW(
                    &dialog);
        }

        if (selected)
        {
            filePath =
                buffer.data();

            return true;
        }

        const DWORD extended =
            CommDlgExtendedError();

        if (extended != 0)
        {
            wchar_t message[128]{};

            swprintf_s(
                message,
                L"Windows file dialog failed (0x%08lX).",
                extended);

            error = message;
        }

        return false;
    }

    [[nodiscard]] static bool SelectSettingsFile(
        HWND owner,
        bool save,
        std::filesystem::path& filePath,
        std::wstring& error)
    {
        filePath.clear();
        error.clear();

        std::vector<wchar_t> buffer(32768, L'\0');

        OPENFILENAMEW dialog{};
        dialog.lStructSize = sizeof(dialog);
        dialog.hwndOwner = owner;
        dialog.lpstrFilter =
            L"ImCut Settings (*.imcutcfg)\0*.imcutcfg\0"
            L"All files (*.*)\0*.*\0";
        dialog.lpstrFile = buffer.data();
        dialog.nMaxFile =
            static_cast<DWORD>(
                buffer.size());
        dialog.lpstrDefExt = L"imcutcfg";
        dialog.Flags =
            OFN_EXPLORER |
            OFN_NOCHANGEDIR |
            OFN_PATHMUSTEXIST;

        BOOL selected = FALSE;

        if (save)
        {
            wcscpy_s(
                buffer.data(),
                buffer.size(),
                L"ImCut-Settings.imcutcfg");

            dialog.Flags |=
                OFN_OVERWRITEPROMPT;

            selected =
                GetSaveFileNameW(
                    &dialog);
        }
        else
        {
            dialog.Flags |=
                OFN_FILEMUSTEXIST;

            selected =
                GetOpenFileNameW(
                    &dialog);
        }

        if (selected)
        {
            filePath =
                buffer.data();

            return true;
        }

        const DWORD extended =
            CommDlgExtendedError();

        if (extended != 0)
        {
            wchar_t message[128]{};

            swprintf_s(
                message,
                L"Windows file dialog failed (0x%08lX).",
                extended);

            error = message;
        }

        return false;
    }

    namespace
    {
        struct AppearanceAccuracyStats
        {
            long samples = 0;
            long deltaELe025 = 0;
            long deltaELe050 = 0;
            long deltaELe100 = 0;
            long globalSearches = 0;
            long denseSearches = 0;
            long exhaustiveSearches = 0;
            double deltaESum = 0.0;
            double deltaEMax = 0.0;
            double deltaE76Sum = 0.0;
            double deltaE76Max = 0.0;
            double absDeltaLSum = 0.0;
            double absDeltaASum = 0.0;
            double absDeltaBSum = 0.0;
            double absDeltaLMax = 0.0;
            double absDeltaAMax = 0.0;
            double absDeltaBMax = 0.0;

            void Add(const Color::AppearanceMatch& match) noexcept
            {
                if (!std::isfinite(match.deltaE00) ||
                    !std::isfinite(match.deltaE76) ||
                    !std::isfinite(match.deltaL) ||
                    !std::isfinite(match.deltaA) ||
                    !std::isfinite(match.deltaB))
                {
                    return;
                }

                ++samples;
                deltaESum += match.deltaE00;
                deltaEMax =
                    (std::max)(
                        deltaEMax,
                        match.deltaE00);
                deltaE76Sum += match.deltaE76;
                deltaE76Max =
                    (std::max)(
                        deltaE76Max,
                        match.deltaE76);

                const double absL = std::abs(match.deltaL);
                const double absA = std::abs(match.deltaA);
                const double absB = std::abs(match.deltaB);

                absDeltaLSum += absL;
                absDeltaASum += absA;
                absDeltaBSum += absB;
                absDeltaLMax = (std::max)(absDeltaLMax, absL);
                absDeltaAMax = (std::max)(absDeltaAMax, absA);
                absDeltaBMax = (std::max)(absDeltaBMax, absB);

                if (match.deltaE00 <= 0.25)
                    ++deltaELe025;
                if (match.deltaE00 <= 0.50)
                    ++deltaELe050;
                if (match.deltaE00 <= 1.00)
                    ++deltaELe100;
                if (match.usedGlobalSearch)
                    ++globalSearches;
                if (match.usedDenseSearch)
                    ++denseSearches;
                if (match.usedExhaustiveSearch)
                    ++exhaustiveSearches;
            }

            [[nodiscard]] double MeanDeltaE() const noexcept
            {
                return samples > 0
                    ? deltaESum /
                        static_cast<double>(samples)
                    : 0.0;
            }

            [[nodiscard]] double MeanDeltaE76() const noexcept
            {
                return samples > 0
                    ? deltaE76Sum /
                        static_cast<double>(samples)
                    : 0.0;
            }

            [[nodiscard]] double MeanAbsDeltaL() const noexcept
            {
                return samples > 0
                    ? absDeltaLSum / static_cast<double>(samples)
                    : 0.0;
            }

            [[nodiscard]] double MeanAbsDeltaA() const noexcept
            {
                return samples > 0
                    ? absDeltaASum / static_cast<double>(samples)
                    : 0.0;
            }

            [[nodiscard]] double MeanAbsDeltaB() const noexcept
            {
                return samples > 0
                    ? absDeltaBSum / static_cast<double>(samples)
                    : 0.0;
            }

            [[nodiscard]] double PercentWithin(long count) const noexcept
            {
                return samples > 0
                    ? 100.0 *
                        static_cast<double>(count) /
                        static_cast<double>(samples)
                    : 0.0;
            }
        };

        struct ColorConversionStats
        {
            long examined = 0;
            long converted = 0;
            long mapped = 0;
            long spotTintWhite = 0;
            long spotToRgb = 0;
            long spotAppearancePreserved = 0;
            long cmykToRgb = 0;
            long blackFloorAdjusted = 0;
            long blacklisted = 0;
            long unchanged = 0;
            long errors = 0;
            AppearanceAccuracyStats cmykAccuracy;
            AppearanceAccuracyStats spotAccuracy;
        };

        using SpotAppearanceCache =
            std::unordered_map<std::wstring, Color::Lab>;
    }

    class Runtime::Impl final
    {
    public:
        HRESULT Initialize(IUnknown* corelApplication)
        {
            if (!corelApplication)
                return SetError(E_POINTER, L"CorelDRAW application pointer is null.");
            if (!OnOwnerThread()) return RPC_E_WRONG_THREAD;
            if (operationRunning_) return SetError(HRESULT_FROM_WIN32(ERROR_BUSY), L"An ImCut operation is running.");
            IVGApplicationPtr app;
            const HRESULT hr = corelApplication->QueryInterface(__uuidof(IVGApplication),
                reinterpret_cast<void**>(&app));
            if (FAILED(hr) || !app) return SetError(FAILED(hr) ? hr : E_NOINTERFACE,
                L"The supplied object does not implement IVGApplication.");
            return InitializeFromSmartPointer(app);
        }

        HRESULT ConnectCorel2026()
        {
            if (!OnOwnerThread()) return RPC_E_WRONG_THREAD;
            if (operationRunning_) return SetError(HRESULT_FROM_WIN32(ERROR_BUSY), L"An ImCut operation is running.");
            if (initialized_) return S_OK;
            const HRESULT comHr = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
            if (FAILED(comHr)) return SetError(comHr, L"ImCut requires a COM STA thread.");
            ScopeExit uninitialize([] { CoUninitialize(); });
            IVGApplicationPtr app(L"CorelDRAW.Application.27");
            if (!app) return SetError(E_FAIL, L"CorelDRAW 2026 application could not be obtained.");
            const HRESULT hr = InitializeFromSmartPointer(app);
            if (SUCCEEDED(hr))
            {
                comInitializationOwned_ = true;
                uninitialize.Release();
            }
            return hr;
        }

        HRESULT ConnectCorel2026Hidden()
        {
            const HRESULT hr = ConnectCorel2026();
            return SUCCEEDED(hr) && visible_ ? Hide() : hr;
        }

        HRESULT Show()
        {
            if (!OnOwnerThread()) return RPC_E_WRONG_THREAD;
            return ShowPage(state_.page);
        }

        HRESULT ShowPage(UI::Page page)
        {
            if (!OnOwnerThread()) return RPC_E_WRONG_THREAD;
            if (!initialized_)
                return SetError(
                    E_UNEXPECTED,
                    L"ImCut is not initialized.");

            const HRESULT uiHr = EnsureUi();
            if (FAILED(uiHr)) return uiHr;
            state_.page = page;

            ShowWindow(
                hwnd_,
                SW_SHOWNORMAL);

            SetForegroundWindow(hwnd_);
            BringWindowToTop(hwnd_);
            SetFocus(hwnd_);

            visible_ = true;
            lastSelectionProbe_ = 0;
            selectionProbePending_ = true;

            RequestAutomaticUpdateCheck();
            SyncUpdateState(true);

            NoteUiActivity();
            ClearError();

            return S_OK;
        }

        HRESULT Hide()
        {
            if (!OnOwnerThread()) return RPC_E_WRONG_THREAD;
            if (!initialized_) return E_UNEXPECTED;
            if (hwnd_)
            {
                const bool wasForeground = GetForegroundWindow() == hwnd_;
                const HWND owner = GetWindow(hwnd_, GW_OWNER);
                visible_ = false;
                renderPosted_ = false;
                StopRenderTimer();
                ShowWindow(hwnd_, SW_HIDE);
                if (wasForeground && IsWindow(owner) && !IsIconic(owner))
                    SetForegroundWindow(owner);
            }
            SavePersistentSettings();
            ClearError();
            return S_OK;
        }

        HRESULT Toggle()
        {
            if (!OnOwnerThread()) return RPC_E_WRONG_THREAD;
            return visible_
                ? Hide()
                : Show();
        }

        HRESULT Shutdown()
        {
            if (!OnOwnerThread()) return RPC_E_WRONG_THREAD;
            if (!initialized_)
                return S_FALSE;

            if (operationRunning_)
            {
                return SetError(
                    HRESULT_FROM_WIN32(
                        ERROR_BUSY),
                    L"An ImCut operation is still running.");
            }

            if (GetCurrentThreadId() !=
                ownerThreadId_)
            {
                return SetError(
                    RPC_E_WRONG_THREAD,
                    L"ImCut must be shut down from the thread that initialized its window.");
            }

            SavePersistentSettings();
            updater_.Stop();

            DestroyImGui();
            CleanupDevice();
            DestroyNativeWindow();

            colorConverter_.Reset();
            profilesLoaded_ = false;
            profileLoadPosted_ = false;
            cmykProfiles_.clear();
            rgbProfiles_.clear();

            app_ = nullptr;

            initialized_ = false;
            visible_ = false;
            ownerThreadId_ = 0;

            if (comInitializationOwned_)
            {
                CoUninitialize();
                comInitializationOwned_ =
                    false;
            }

            ClearError();

            return S_OK;
        }

        [[nodiscard]] bool IsInitialized() const noexcept
        {
            return initialized_;
        }

        [[nodiscard]] bool IsVisible() const noexcept { return initialized_ && visible_; }

        void ReportBoundaryError(HRESULT hr, const wchar_t* message) noexcept
        {
            try { SetError(hr, message); } catch (...) { lastHresult_.store(hr); }
            OutputDebugStringW(message);
        }

        HRESULT RunCut(
            int pageWidthMm,
            int registrationMarks,
            int closureMode)
        {
            if (!ValidateOperationContext())
                return LastHresult();

            state_.cut.pageWidthMm =
                (std::max)(
                    pageWidthMm,
                    1);

            state_.cut.registrationMarks =
                (std::max)(
                    registrationMarks,
                    0);

            state_.cut.closureMode =
                static_cast<UI::ClosureMode>(
                    (std::clamp)(
                        closureMode,
                        0,
                        3));

            SavePersistentSettings();
            RequestAutomaticUpdateCheck();
            UI::RequestPrimaryActionFocus(
                UI::Page::Cut);

            return ShowPage(
                UI::Page::Cut);
        }

        HRESULT RunBleed(
            double distanceMm,
            bool ungroupBeforeProcessing,
            bool detectHiddenObjects,
            bool createCutline)
        {
            if (!ValidateOperationContext())
                return LastHresult();

            if (!InputLimits::ValidBleed(distanceMm))
                return SetError(E_INVALIDARG, L"A distancia de sangria deve estar entre 0,001 e 1000 mm e ser finita.");
            state_.bleed.distanceMm = static_cast<float>(distanceMm);

            state_.bleed.ungroupBeforeProcessing =
                ungroupBeforeProcessing;

            state_.bleed.detectHiddenObjects =
                detectHiddenObjects;

            state_.bleed.createCutline =
                createCutline;

            SavePersistentSettings();
            RequestAutomaticUpdateCheck();
            UI::RequestPrimaryActionFocus(
                UI::Page::Bleed);

            return ShowPage(
                UI::Page::Bleed);
        }

        HRESULT RefreshColorProfiles()
        {
            if (!OnOwnerThread()) return RPC_E_WRONG_THREAD;
            if (!initialized_)
                return SetError(
                    E_UNEXPECTED,
                    L"ImCut is not initialized.");

            return ExecuteOperation(
                L"Refreshing ICC profiles",
                [this]()
                {
                    return RefreshColorProfilesInternal();
                });
        }

        HRESULT ConvertSelection(
            int cmykProfileIndex,
            int rgbProfileIndex,
            int renderingIntent,
            bool adaptiveLut,
            int preferredGrid)
        {
            if (!ValidateOperationContext())
                return LastHresult();
            const HRESULT profilesHr = EnsureColorProfiles();
            if (FAILED(profilesHr)) return profilesHr;

            UI::ColorState request =
                state_.color;

            request.selectedCmyk =
                cmykProfileIndex;

            request.selectedRgb =
                rgbProfileIndex;

            request.intent =
                static_cast<UI::RenderingIntent>(
                    (std::clamp)(
                        renderingIntent,
                        0,
                        3));

            request.adaptiveLut =
                adaptiveLut;

            request.preferredGrid =
                preferredGrid;

            return RunColorInternal(request, true, false);
        }

        HRESULT RunCutSaved()
        {
            if (!ValidateOperationContext()) return LastHresult();
            RequestAutomaticUpdateCheck();
            UI::RequestPrimaryActionFocus(
                UI::Page::Cut);

            return ShowPage(
                UI::Page::Cut);
        }

        HRESULT RunBleedSaved()
        {
            if (!ValidateOperationContext()) return LastHresult();
            RequestAutomaticUpdateCheck();
            UI::RequestPrimaryActionFocus(
                UI::Page::Bleed);

            return ShowPage(
                UI::Page::Bleed);
        }

        HRESULT ConvertSelectionSaved()
        {
            if (!ValidateOperationContext())
                return LastHresult();
            const HRESULT profilesHr = EnsureColorProfiles();
            if (FAILED(profilesHr)) return profilesHr;

            return RunColorInternal(
                state_.color, true, false);
        }

        void SetNestingCallback(
            ImCutNestingCallback callback) noexcept
        {
            nestingCallback_ = callback;
        }

        int CopyLastError(
            wchar_t* buffer,
            int capacity) const
        {
            std::lock_guard<std::mutex> lock(
                errorMutex_);

            const int required =
                static_cast<int>(
                    lastError_.size()) +
                1;

            if (!buffer ||
                capacity <= 0)
            {
                return required;
            }

            const int count =
                (std::min)(
                    capacity - 1,
                    static_cast<int>(
                        lastError_.size()));

            if (count > 0)
            {
                std::memcpy(
                    buffer,
                    lastError_.data(),
                    static_cast<std::size_t>(
                        count) *
                    sizeof(wchar_t));
            }

            buffer[count] = L'\0';

            return required;
        }

        [[nodiscard]] HRESULT LastHresult() const noexcept
        {
            return lastHresult_.load(
                std::memory_order_relaxed);
        }

        [[nodiscard]] static bool IsUiInputMessage(
            UINT message) noexcept
        {
            switch (message)
            {
            case WM_MOUSEMOVE:
            case WM_MOUSEWHEEL:
            case WM_MOUSEHWHEEL:
            case WM_LBUTTONDOWN:
            case WM_LBUTTONUP:
            case WM_LBUTTONDBLCLK:
            case WM_RBUTTONDOWN:
            case WM_RBUTTONUP:
            case WM_RBUTTONDBLCLK:
            case WM_MBUTTONDOWN:
            case WM_MBUTTONUP:
            case WM_MBUTTONDBLCLK:
            case WM_XBUTTONDOWN:
            case WM_XBUTTONUP:
            case WM_XBUTTONDBLCLK:
            case WM_KEYDOWN:
            case WM_KEYUP:
            case WM_SYSKEYDOWN:
            case WM_SYSKEYUP:
            case WM_CHAR:
            case WM_SETFOCUS:
                return true;

            default:
                return false;
            }
        }

        void SetRenderTimerInterval(
            UINT intervalMs) noexcept
        {
            if (!hwnd_ ||
                renderTimerIntervalMs_ == intervalMs)
            {
                return;
            }

            if (SetTimer(
                    hwnd_,
                    RenderTimerId,
                    intervalMs,
                    nullptr) != 0)
            {
                renderTimerIntervalMs_ = intervalMs;
            }
        }

        void StopRenderTimer() noexcept
        {
            if (!hwnd_ ||
                renderTimerIntervalMs_ == 0)
            {
                return;
            }

            KillTimer(
                hwnd_,
                RenderTimerId);

            renderTimerIntervalMs_ = 0;
        }

        void NoteUiActivity() noexcept
        {
            lastUiActivity_ = GetTickCount64();
            if (!visible_)
                return;

            SetRenderTimerInterval(ActiveRenderTimerMs);

            if (!rendering_ && !renderPosted_ && hwnd_)
            {
                renderPosted_ = PostMessageW(
                    hwnd_,
                    MessageRenderNow,
                    0,
                    0) != FALSE;
            }
        }

        [[nodiscard]] bool IsUiRecentlyActive(
            ULONGLONG now) const noexcept
        {
            return now - lastUiActivity_ <
                UiActivityHoldMs;
        }

        [[nodiscard]] bool IsUpdaterVisuallyActive() const noexcept
        {
            switch (state_.update.status)
            {
            case UI::UpdateStatus::Checking:
            case UI::UpdateStatus::ResolvingDownload:
            case UI::UpdateStatus::Downloading:
                return true;

            default:
                return false;
            }
        }

        void UpdateRenderCadence(
            bool imguiInteractive) noexcept
        {
            const ULONGLONG now =
                GetTickCount64();

            if (imguiInteractive ||
                IsUiRecentlyActive(now))
            {
                SetRenderTimerInterval(
                    ActiveRenderTimerMs);
                return;
            }

            if (IsUpdaterVisuallyActive())
            {
                SetRenderTimerInterval(
                    UpdateRenderTimerMs);
                return;
            }

            if (GetForegroundWindow() != hwnd_)
            {
                SetRenderTimerInterval(
                    BackgroundRenderTimerMs);
                return;
            }

            SetRenderTimerInterval(
                IdleRenderTimerMs);
        }

        LRESULT WindowProc(
            HWND hwnd,
            UINT message,
            WPARAM wParam,
            LPARAM lParam)
        {
            if (IsUiInputMessage(message))
                NoteUiActivity();

            if (imguiInitialized_ &&
                ImGui_ImplWin32_WndProcHandler(
                    hwnd,
                    message,
                    wParam,
                    lParam))
            {
                return TRUE;
            }

            switch (message)
            {
            case WM_ACTIVATE:
                if (LOWORD(wParam) != WA_INACTIVE)
                {
                    selectionProbePending_ = true;
                    previewContextKey_.clear();
                    lastPreviewContextProbe_ = 0;
                }
                break;

            case WM_TIMER:
            {
                if (wParam ==
                    RenderTimerId)
                {
                    RenderFrame();
                    return 0;
                }

                break;
            }

            case MessageRenderNow:
            {
                renderPosted_ = false;
                RenderFrame();
                return 0;
            }

            case WM_SIZE:
            {
                NoteUiActivity();

                if (device_ &&
                    wParam !=
                    SIZE_MINIMIZED)
                {
                    CleanupRenderTarget();

                    if (swapChain_)
                    {
                        (void)swapChain_->ResizeBuffers(
                            0,
                            static_cast<UINT>(
                                LOWORD(lParam)),
                            static_cast<UINT>(
                                HIWORD(lParam)),
                            DXGI_FORMAT_UNKNOWN,
                            0);
                    }

                    CreateRenderTarget();
                }

                return 0;
            }

            case WM_CLOSE:
                (void)Hide();
                return 0;

            case WM_SYSCOMMAND:
            {
                const WPARAM command = wParam & 0xFFF0;
                if (command == SC_CLOSE || command == SC_MINIMIZE)
                {
                    (void)Hide();
                    return 0;
                }

                if (command == SC_SIZE ||
                    command == SC_MAXIMIZE)
                {
                    return 0;
                }

                break;
            }

            case WM_NCLBUTTONDBLCLK:
            {
                if (wParam == HTCAPTION)
                    return 0;

                break;
            }

            case MessageRunCut:
            {
                (void)RunCutInternal(
                    pendingCut_);

                return 0;
            }

            case MessageRunBleed:
            {
                (void)RunBleedInternal(
                    pendingBleed_);

                return 0;
            }

            case MessageRunColor:
            {
                (void)RunColorInternal(
                    pendingColor_);

                return 0;
            }

            case MessageRunNesting:
            {
                (void)RunNestingInternal(
                    pendingNesting_);

                return 0;
            }

            case MessageRefreshProfiles:
            {
                (void)RefreshColorProfiles();

                return 0;
            }

            case MessageImportColorMappings:
            {
                (void)ImportColorMappingsInternal();

                return 0;
            }

            case MessageExportColorMappings:
            {
                (void)ExportColorMappingsInternal();

                return 0;
            }

            case MessageImportSettings:
            {
                (void)ImportSettingsInternal();

                return 0;
            }

            case MessageExportSettings:
            {
                (void)ExportSettingsInternal();

                return 0;
            }

            case MessageAutoInstallUpdate:
            {
                autoUpdateInstallPosted_ = false;

                if (!hotUpdateInstallPending_)
                    BeginHotUpdateInstall();

                if (!hotUpdateInstallPending_)
                    installAfterUserDownload_ = false;

                return 0;
            }

            case MessageShutdownForUpdate:
            {
                if (!hotUpdateInstallPending_)
                    return 0;

                const std::filesystem::path readyFlag(
                    pendingUpdateReadyFlag_);

                const HRESULT shutdownHr = Shutdown();

                if (SUCCEEDED(shutdownHr))
                    (void)TouchFile(readyFlag);

                return 0;
            }

            case WM_DESTROY:
                return 0;
            }

            return DefWindowProcW(
                hwnd,
                message,
                wParam,
                lParam);
        }

        [[nodiscard]] const wchar_t* Version() const noexcept
        {
            return L"ImCut Runtime " IMCUT_VERSION_WSTRING;
        }

    private:
        HRESULT InitializeFromSmartPointer(
            const IVGApplicationPtr& app)
        {
            if (!app)
                return E_POINTER;

            if (!OnOwnerThread()) return RPC_E_WRONG_THREAD;
            if (initialized_)
            {
                IUnknownPtr oldIdentity(app_), newIdentity(app);
                if (oldIdentity != newIdentity)
                    return SetError(E_INVALIDARG, L"Shut down ImCut before attaching another CorelDRAW instance.");
                return S_OK;
            }
            DWORD expected = 0;
            if (!ownerThreadId_.compare_exchange_strong(expected, GetCurrentThreadId()))
                return SetError(expected == GetCurrentThreadId() ? HRESULT_FROM_WIN32(ERROR_BUSY) : RPC_E_WRONG_THREAD,
                    L"ImCut initialization is already in progress.");
            ScopeExit rollback([this]
            {
                updater_.Stop();
                DestroyImGui(); CleanupDevice(); DestroyNativeWindow();
                app_ = nullptr; initialized_ = false; visible_ = false; ownerThreadId_ = 0;
            });
            app_ = app;

            LoadPersistentSettings();
            initialized_ = true;
            visible_ = false;
            ClearError();
            rollback.Release();
            return S_OK;
        }

        HRESULT EnsureUi()
        {
            if (hwnd_ && imguiInitialized_) return S_OK;
            ScopeExit rollback([this] { DestroyImGui(); CleanupDevice(); DestroyNativeWindow(); });
            HRESULT hr = CreateNativeWindow();
            if (FAILED(hr)) return hr;
            hr = CreateDevice();
            if (FAILED(hr)) return hr;
            hr = CreateImGui();
            if (FAILED(hr)) return hr;
            SetupCallbacks();
            rollback.Release();
            return S_OK;
        }

        HRESULT AttachApplication(
            IUnknown* unknown)
        {
            IVGApplication* raw = nullptr;

            const HRESULT hr =
                unknown->QueryInterface(
                    __uuidof(IVGApplication),
                    reinterpret_cast<void**>(
                        &raw));

            if (FAILED(hr) || !raw)
            {
                return SetError(
                    FAILED(hr)
                    ? hr
                    : E_NOINTERFACE,
                    L"The supplied COM object does not implement IVGApplication.");
            }

            app_ = raw;
            raw->Release();

            return S_OK;
        }

        HRESULT CreateNativeWindow()
        {
            HMODULE module = nullptr;

            if (!GetModuleHandleExW(
                GET_MODULE_HANDLE_EX_FLAG_FROM_ADDRESS |
                GET_MODULE_HANDLE_EX_FLAG_UNCHANGED_REFCOUNT,
                reinterpret_cast<LPCWSTR>(&RuntimeWindowProc),
                &module) ||
                !module)
            {
                return SetError(
                    HRESULT_FROM_WIN32(GetLastError()),
                    L"Unable to resolve the ImCut DLL module handle.");
            }

            moduleInstance_ =
                reinterpret_cast<HINSTANCE>(module);

            WNDCLASSEXW windowClass{};
            windowClass.cbSize = sizeof(windowClass);
            windowClass.style = 0;
            windowClass.lpfnWndProc = RuntimeWindowProc;
            windowClass.hInstance = moduleInstance_;
            windowClass.hCursor =
                static_cast<HCURSOR>(
                    LoadImageW(
                        nullptr,
                        MAKEINTRESOURCEW(32512),
                        IMAGE_CURSOR,
                        0,
                        0,
                        LR_DEFAULTSIZE | LR_SHARED));
            windowClass.hbrBackground = nullptr;
            windowClass.lpszClassName = WindowClassName;

            if (!RegisterClassExW(&windowClass))
            {
                const DWORD error = GetLastError();

                if (error != ERROR_CLASS_ALREADY_EXISTS)
                {
                    return SetError(
                        HRESULT_FROM_WIN32(error),
                        L"Unable to register the ImCut window class.");
                }
            }

            HWND owner = nullptr;

            try
            {
                if (app_ && app_->AppWindow)
                {
                    owner =
                        reinterpret_cast<HWND>(
                            static_cast<INT_PTR>(
                                app_->AppWindow->Handle));
                }
            }
            catch (...)
            {
                owner = nullptr;
            }

            const DWORD exStyle = WS_EX_TOOLWINDOW;
            const DWORD style =
                WS_OVERLAPPED |
                WS_CAPTION |
                WS_SYSMENU;

            const UINT dpi = ResolveDpi(owner);
            const float scale = static_cast<float>(dpi) / 96.0f;
            const int clientWidth =
                static_cast<int>(std::lround(UiClientWidth * scale));
            const int clientHeight =
                static_cast<int>(std::lround(UiClientHeight * scale));

            RECT windowRect
            {
                0,
                0,
                clientWidth,
                clientHeight
            };

            AdjustWindowRectForDpi(
                windowRect,
                style,
                exStyle,
                dpi);

            const int windowWidth = windowRect.right - windowRect.left;
            const int windowHeight = windowRect.bottom - windowRect.top;

            RECT workArea{};
            MONITORINFO monitorInfo{};
            monitorInfo.cbSize = sizeof(monitorInfo);
            const HMONITOR monitor = owner
                ? MonitorFromWindow(owner, MONITOR_DEFAULTTONEAREST)
                : nullptr;
            if (monitor && GetMonitorInfoW(monitor, &monitorInfo))
            {
                workArea = monitorInfo.rcWork;
            }
            else
            {
                workArea = {
                    0,
                    0,
                    GetSystemMetrics(SM_CXSCREEN),
                    GetSystemMetrics(SM_CYSCREEN)
                };
                (void)SystemParametersInfoW(
                    SPI_GETWORKAREA,
                    0,
                    &workArea,
                    0);
            }

            RECT reference = workArea;

            if (owner)
            {
                RECT ownerRect{};
                if (GetWindowRect(owner, &ownerRect))
                    reference = ownerRect;
            }

            int x = reference.left +
                ((reference.right - reference.left) - windowWidth) / 2;
            int y = reference.top +
                ((reference.bottom - reference.top) - windowHeight) / 2;

            const int maxX = std::max(workArea.left, workArea.right - windowWidth);
            const int maxY = std::max(workArea.top, workArea.bottom - windowHeight);
            x = std::clamp<int>(x, workArea.left, maxX);
            y = std::clamp<int>(y, workArea.top, maxY);

            hwnd_ =
                CreateWindowExW(
                    exStyle,
                    WindowClassName,
                    WindowTitle,
                    style,
                    x,
                    y,
                    windowWidth,
                    windowHeight,
                    owner,
                    nullptr,
                    moduleInstance_,
                    nullptr);

            if (!hwnd_)
            {
                return SetError(
                    HRESULT_FROM_WIN32(GetLastError()),
                    L"Unable to create the ImCut window.");
            }

            ApplyDarkTitleBar(hwnd_);

            renderTimerIntervalMs_ = SetTimer(
                hwnd_,
                RenderTimerId,
                ActiveRenderTimerMs,
                nullptr) != 0
                ? ActiveRenderTimerMs
                : 0;

            lastUiActivity_ =
                GetTickCount64();

            return S_OK;
        }

        void DestroyNativeWindow() noexcept
        {
            if (hwnd_)
            {
                KillTimer(
                    hwnd_,
                    RenderTimerId);

                DestroyWindow(hwnd_);
                hwnd_ = nullptr;
            }

            if (moduleInstance_)
            {
                UnregisterClassW(
                    WindowClassName,
                    moduleInstance_);

                moduleInstance_ = nullptr;
            }
        }

        HRESULT CreateDevice()
        {
            DXGI_SWAP_CHAIN_DESC description{};
            description.BufferCount = 2;
            description.BufferDesc.Format =
                DXGI_FORMAT_R8G8B8A8_UNORM;
            description.BufferUsage =
                DXGI_USAGE_RENDER_TARGET_OUTPUT;
            description.OutputWindow = hwnd_;
            description.SampleDesc.Count = 1;
            description.Windowed = TRUE;
            description.SwapEffect =
                DXGI_SWAP_EFFECT_FLIP_DISCARD;

            UINT flags =
                D3D11_CREATE_DEVICE_SINGLETHREADED;

            D3D_FEATURE_LEVEL featureLevel{};
            const D3D_FEATURE_LEVEL featureLevels[]
            {
                D3D_FEATURE_LEVEL_11_0,
                D3D_FEATURE_LEVEL_10_0
            };

            const auto createDevice =
                [&](D3D_DRIVER_TYPE driverType)
                {
                    return D3D11CreateDeviceAndSwapChain(
                        nullptr,
                        driverType,
                        nullptr,
                        flags,
                        featureLevels,
                        static_cast<UINT>(
                            sizeof(featureLevels) /
                            sizeof(featureLevels[0])),
                        D3D11_SDK_VERSION,
                        &description,
                        &swapChain_,
                        &device_,
                        &featureLevel,
                        &deviceContext_);
                };

            HRESULT hr =
                createDevice(
                    D3D_DRIVER_TYPE_HARDWARE);

            if (FAILED(hr))
            {
                CleanupDevice();
                description.SwapEffect =
                    DXGI_SWAP_EFFECT_DISCARD;

                hr = createDevice(
                    D3D_DRIVER_TYPE_HARDWARE);
            }

            if (FAILED(hr))
            {
                CleanupDevice();
                description.SwapEffect =
                    DXGI_SWAP_EFFECT_FLIP_DISCARD;

                hr = createDevice(
                    D3D_DRIVER_TYPE_WARP);
            }

            if (FAILED(hr))
            {
                CleanupDevice();
                description.SwapEffect =
                    DXGI_SWAP_EFFECT_DISCARD;

                hr = createDevice(
                    D3D_DRIVER_TYPE_WARP);
            }

            if (FAILED(hr))
            {
                return SetError(
                    hr,
                    L"Unable to initialize Direct3D 11.");
            }

            IDXGIDevice1* dxgiDevice = nullptr;
            if (SUCCEEDED(device_->QueryInterface(
                    __uuidof(IDXGIDevice1),
                    reinterpret_cast<void**>(&dxgiDevice))) &&
                dxgiDevice)
            {
                (void)dxgiDevice->SetMaximumFrameLatency(1);
                dxgiDevice->Release();
            }

            hr = CreateRenderTarget();

            if (FAILED(hr))
            {
                CleanupDevice();
                return hr;
            }

            return S_OK;
        }

        HRESULT CreateRenderTarget()
        {
            if (!swapChain_ ||
                !device_)
            {
                return E_UNEXPECTED;
            }

            ID3D11Texture2D* backBuffer =
                nullptr;

            HRESULT hr =
                swapChain_->GetBuffer(
                    0,
                    IID_PPV_ARGS(
                        &backBuffer));

            if (FAILED(hr))
            {
                return SetError(
                    hr,
                    L"Unable to obtain the Direct3D back buffer.");
            }

            hr =
                device_->CreateRenderTargetView(
                    backBuffer,
                    nullptr,
                    &renderTarget_);

            backBuffer->Release();

            if (FAILED(hr))
            {
                return SetError(
                    hr,
                    L"Unable to create the Direct3D render target.");
            }

            return S_OK;
        }

        void CleanupRenderTarget() noexcept
        {
            if (renderTarget_)
            {
                renderTarget_->Release();
                renderTarget_ = nullptr;
            }
        }

        void CleanupDevice() noexcept
        {
            CleanupRenderTarget();

            if (swapChain_)
            {
                swapChain_->Release();
                swapChain_ = nullptr;
            }

            if (deviceContext_)
            {
                deviceContext_->Release();
                deviceContext_ = nullptr;
            }

            if (device_)
            {
                device_->Release();
                device_ = nullptr;
            }
        }

        HRESULT CreateImGui()
        {
            IMGUI_CHECKVERSION();
            auto* context = ImGui::CreateContext();
            if (!context) return E_OUTOFMEMORY;
            bool win32Ready = false, dx11Ready = false;
            ScopeExit rollback([&]
            {
                if (dx11Ready) ImGui_ImplDX11_Shutdown();
                if (win32Ready) ImGui_ImplWin32_Shutdown();
                ImGui::DestroyContext(context);
            });
            auto& io = ImGui::GetIO();
            io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
            io.ConfigWindowsMoveFromTitleBarOnly = true;
            io.IniFilename = nullptr; io.LogFilename = nullptr;
            const float scale = ResolveScale(hwnd_);
            LoadUiFonts(scale);
            UI::ApplyCorelTheme(scale);
            win32Ready = ImGui_ImplWin32_Init(hwnd_);
            if (!win32Ready) return SetError(E_FAIL, L"Unable to initialize the ImGui Win32 backend.");
            dx11Ready = ImGui_ImplDX11_Init(device_, deviceContext_);
            if (!dx11Ready) return SetError(E_FAIL, L"Unable to initialize the ImGui DirectX 11 backend.");
            imguiInitialized_ = true;
            rollback.Release();
            return S_OK;
        }

        void DestroyImGui() noexcept
        {
            if (!imguiInitialized_)
                return;

            ImGui_ImplDX11_Shutdown();
            ImGui_ImplWin32_Shutdown();
            ImGui::DestroyContext();

            imguiInitialized_ = false;
        }

        void SetupCallbacks()
        {
            previewSrgbConverters_.reserve(16);
            colorTargetPreviewCache_.reserve(256);
            pendingColorPreviewRequests_.reserve(64);

            callbacks_.runCut =
                [this](
                    const UI::CutState& request)
                {
                    RequestAutomaticUpdateCheck();
                    pendingCut_ = request;

                    PostMessageW(
                        hwnd_,
                        MessageRunCut,
                        0,
                        0);
                };

            callbacks_.runBleed =
                [this](
                    const UI::BleedState& request)
                {
                    RequestAutomaticUpdateCheck();
                    pendingBleed_ = request;

                    PostMessageW(
                        hwnd_,
                        MessageRunBleed,
                        0,
                        0);
                };

            callbacks_.runColor =
                [this](
                    const UI::ColorState& request)
                {
                    pendingColor_ = request;

                    PostMessageW(
                        hwnd_,
                        MessageRunColor,
                        0,
                        0);
                };

            callbacks_.runNesting =
                [this](
                    const UI::NestingState& request)
                {
                    pendingNesting_ = request;

                    PostMessageW(
                        hwnd_,
                        MessageRunNesting,
                        0,
                        0);
                };

            callbacks_.refreshColorProfiles =
                [this]()
                {
                    PostMessageW(
                        hwnd_,
                        MessageRefreshProfiles,
                        0,
                        0);
                };

            callbacks_.importColorMappings =
                [this]()
                {
                    PostMessageW(
                        hwnd_,
                        MessageImportColorMappings,
                        0,
                        0);
                };

            callbacks_.exportColorMappings =
                [this]()
                {
                    PostMessageW(
                        hwnd_,
                        MessageExportColorMappings,
                        0,
                        0);
                };

            callbacks_.importSettings =
                [this]()
                {
                    PostMessageW(
                        hwnd_,
                        MessageImportSettings,
                        0,
                        0);
                };

            callbacks_.exportSettings =
                [this]()
                {
                    PostMessageW(
                        hwnd_,
                        MessageExportSettings,
                        0,
                        0);
                };

            callbacks_.previewColorMappingTarget =
                [this](
                    const UI::ColorMapEntry& mapping,
                    const UI::ColorState& colorState)
                {
                    return GetColorMappingTargetPreview(
                        mapping,
                        colorState);
                };

            callbacks_.checkForUpdates =
                [this]()
                {
                    RequestAutomaticUpdateCheck();
                };

            callbacks_.downloadUpdate =
                [this]()
                {
                    installAfterUserDownload_ = true;
                    autoUpdateInstallPosted_ = false;
                    updater_.DownloadAsync();
                };

            callbacks_.openDownloadedUpdate =
                [this]()
                {
                    BeginHotUpdateInstall();
                };
        }

        void BeginHotUpdateInstall()
        {
            const auto snapshot = updater_.GetSnapshot();

            if (snapshot.status != Updater::Status::Downloaded ||
                snapshot.downloadedPath.empty())
            {
                MessageBoxW(
                    hwnd_,
                    L"Nenhuma atualização foi baixada.",
                    L"ImCut",
                    MB_OK | MB_ICONWARNING);
                return;
            }

            const std::filesystem::path source(
                snapshot.downloadedPath);

            std::error_code ec;
            if (!std::filesystem::exists(source, ec) || ec)
            {
                MessageBoxW(
                    hwnd_,
                    L"O arquivo baixado da atualização não foi encontrado.",
                    L"ImCut",
                    MB_OK | MB_ICONERROR);
                return;
            }

            std::wstring extension = source.extension().wstring();
            std::transform(
                extension.begin(),
                extension.end(),
                extension.begin(),
                [](wchar_t ch)
                {
                    return static_cast<wchar_t>(std::towlower(ch));
                });

            std::string validationError;
            if (extension != L".dll" || !Updater::ValidateArtifact(source,
                Updater::RunningModulePath(), snapshot.remoteVersion, validationError))
            {
                const auto message = Utf8ToWide(validationError.empty()
                    ? "Somente uma DLL ImCut autenticada pode ser instalada." : validationError);
                MessageBoxW(hwnd_, message.c_str(), L"ImCut - Atualizacao bloqueada", MB_OK | MB_ICONERROR);
                return;
            }

            if (!app_ || !moduleInstance_ || !hwnd_)
            {
                MessageBoxW(
                    hwnd_,
                    L"O ImCut não está em um estado válido para atualização em tempo real.",
                    L"ImCut",
                    MB_OK | MB_ICONERROR);
                return;
            }

            std::wstring target = GetModuleFilePath(
                moduleInstance_);

            if (target.empty())
            {
                target = DefaultImCutDllPath(app_).wstring();
            }

            const auto controlDirectory = UpdateControlDirectory(app_);
            const auto ticketPath = controlDirectory / L"pending-update.txt";
            const auto readyFlag = controlDirectory / L"shutdown-ready.flag";

            std::filesystem::remove(readyFlag, ec);
            ec.clear();

            std::wostringstream ticket;
            ticket
                << L"IMCUT_UPDATE_V1\r\n"
                << L"Source=" << source.wstring() << L"\r\n"
                << L"Target=" << target << L"\r\n"
                << L"Ready=" << readyFlag.wstring() << L"\r\n"
                << L"Version=";

            const std::wstring remoteVersion(
                snapshot.remoteVersion.begin(),
                snapshot.remoteVersion.end());

            ticket << remoteVersion << L"\r\n";

            const std::wstring ticketText = ticket.str();

            std::wstring ticketError;
            if (!WriteUtf16LeFile(
                ticketPath,
                ticketText,
                ticketError))
            {
                MessageBoxW(
                    hwnd_,
                    ticketError.c_str(),
                    L"ImCut",
                    MB_OK | MB_ICONERROR);
                return;
            }

            SetEnvironmentVariableW(
                L"IMCUT_UPDATE_TICKET",
                ticketPath.c_str());

            {
                std::wstring ignored;

                const auto legacyTemp =
                    LegacyTempUpdateControlDirectory();

                if (!legacyTemp.empty())
                {
                    (void)WriteUtf16LeFile(
                        legacyTemp / L"pending-update.txt",
                        ticketText,
                        ignored);
                }

                ignored.clear();

                auto legacyLocal = LocalAppDataGmsFallback();
                legacyLocal /= L"Updates";

                std::error_code legacyEc;
                std::filesystem::create_directories(
                    legacyLocal,
                    legacyEc);

                (void)WriteUtf16LeFile(
                    legacyLocal / L"pending-update.txt",
                    ticketText,
                    ignored);
            }

            std::wstring macroError;
            if (!InvokeGmsMacroNoArgs(
                app_,
                UpdaterProjectName,
                UpdaterScheduleMacro,
                macroError))
            {
                const std::wstring message =
                    L"Não foi possível iniciar o ImCutUpdater.gms.\n\n"
                    L"Instale o projeto VBA separado 'ImCutUpdater' e importe o módulo Updater.bas.\n\nDetalhes: " +
                    macroError;

                MessageBoxW(
                    hwnd_,
                    message.c_str(),
                    L"ImCut",
                    MB_OK | MB_ICONERROR);
                return;
            }

            pendingUpdateReadyFlag_ = readyFlag.wstring();
            hotUpdateInstallPending_ = true;

            state_.update.message =
                "Atualização preparada. O ImCut será descarregado, atualizado e carregado novamente.";

            if (!PostMessageW(
                hwnd_,
                MessageShutdownForUpdate,
                0,
                0))
            {
                hotUpdateInstallPending_ = false;

                MessageBoxW(
                    hwnd_,
                    L"Não foi possível iniciar o desligamento seguro do ImCut.",
                    L"ImCut",
                    MB_OK | MB_ICONERROR);
            }
        }

        void StartAutomaticUpdateCheck()
        {
            RequestAutomaticUpdateCheck();
        }

        void RequestAutomaticUpdateCheck()
        {
            const auto snapshot =
                updater_.GetSnapshot();

            const bool userRequestedUpdateInFlight =
                installAfterUserDownload_ &&
                (snapshot.status == Updater::Status::ResolvingDownload ||
                 snapshot.status == Updater::Status::Downloading ||
                 snapshot.status == Updater::Status::Downloaded ||
                 updater_.IsBusy());

            if (!userRequestedUpdateInFlight)
            {
                installAfterUserDownload_ = false;
                autoUpdateInstallPosted_ = false;
            }

            if (snapshot.status == Updater::Status::Downloaded ||
                snapshot.status == Updater::Status::UpdateAvailable ||
                updater_.IsBusy())
            {
                return;
            }

            updater_.CheckAsync();
        }

        void SyncUpdateState(
            bool force = false)
        {
            const ULONGLONG now =
                GetTickCount64();

            if (!force &&
                now - lastUpdateSync_ <
                    UpdateSyncIntervalMs)
            {
                return;
            }

            lastUpdateSync_ = now;

            const auto snapshot =
                updater_.GetSnapshot();

            state_.update.currentVersion =
                snapshot.currentVersion;
            state_.update.remoteVersion =
                snapshot.remoteVersion;
            state_.update.message =
                snapshot.message;
            state_.update.progress =
                snapshot.progress;
            state_.update.updateAvailable =
                snapshot.updateAvailable;

            switch (snapshot.status)
            {
            case Updater::Status::Checking:
                state_.update.status = UI::UpdateStatus::Checking;
                break;
            case Updater::Status::UpToDate:
                state_.update.status = UI::UpdateStatus::UpToDate;
                break;
            case Updater::Status::UpdateAvailable:
                state_.update.status = UI::UpdateStatus::UpdateAvailable;
                break;
            case Updater::Status::ResolvingDownload:
                state_.update.status = UI::UpdateStatus::ResolvingDownload;
                break;
            case Updater::Status::Downloading:
                state_.update.status = UI::UpdateStatus::Downloading;
                break;
            case Updater::Status::Downloaded:
                state_.update.status = UI::UpdateStatus::Downloaded;
                break;
            case Updater::Status::Error:
                state_.update.status = UI::UpdateStatus::Error;
                break;
            case Updater::Status::Idle:
            default:
                state_.update.status = UI::UpdateStatus::Idle;
                break;
            }

            if (!installAfterUserDownload_ ||
                hotUpdateInstallPending_)
            {
                return;
            }

            if (snapshot.status ==
                    Updater::Status::Downloaded &&
                !operationRunning_ &&
                !autoUpdateInstallPosted_)
            {
                autoUpdateInstallPosted_ =
                    PostMessageW(
                        hwnd_,
                        MessageAutoInstallUpdate,
                        0,
                        0) != FALSE;
            }
        }

        void RenderFrame()
        {
            if (!initialized_ ||
                !visible_ ||
                !hwnd_ ||
                !imguiInitialized_ ||
                rendering_ ||
                IsIconic(hwnd_))
            {
                return;
            }

            rendering_ = true;

            try
            {
                const ULONGLONG frameNow =
                    GetTickCount64();

                bool backgroundWorkPerformed = false;

                if (!state_.runtime.busy &&
                    !IsUiRecentlyActive(frameNow))
                {
                    if (state_.page == UI::Page::Color) RefreshPreviewContext();
                    backgroundWorkPerformed = ProcessOnePendingColorPreview();

                    if (!backgroundWorkPerformed)
                        UpdateSelectionProbe();
                }

                ImGui_ImplDX11_NewFrame();
                ImGui_ImplWin32_NewFrame();
                ImGui::NewFrame();

                if (state_.page == UI::Page::Color && !profilesLoaded_ && !profileLoadPosted_)
                {
                    profileLoadPosted_ = PostMessageW(hwnd_, MessageRefreshProfiles, 0, 0) != FALSE;
                }
                bool open = true;

                SyncUpdateState();

                UI::Draw(
                    state_,
                    callbacks_,
                    &open);

                const ImGuiIO& io =
                    ImGui::GetIO();

                const bool imguiInteractive =
                    ImGui::IsAnyItemActive() ||
                    io.WantTextInput ||
                    io.MouseDown[0] ||
                    io.MouseDown[1] ||
                    io.MouseDown[2];

                if (!backgroundWorkPerformed &&
                    !state_.runtime.busy &&
                    !imguiInteractive &&
                    !IsUiRecentlyActive(
                        GetTickCount64()))
                {
                    TrackPersistentSettings();
                }

                ImGui::Render();

                constexpr float clearColor[4]
                {
                    38.0f / 255.0f,
                    38.0f / 255.0f,
                    38.0f / 255.0f,
                    1.0f
                };

                if (renderTarget_)
                {
                    deviceContext_->OMSetRenderTargets(
                        1,
                        &renderTarget_,
                        nullptr);

                    deviceContext_->ClearRenderTargetView(
                        renderTarget_,
                        clearColor);
                }

                ImGui_ImplDX11_RenderDrawData(
                    ImGui::GetDrawData());

                if (swapChain_)
                {
                    (void)swapChain_->Present(
                        0,
                        0);
                }

                if (!open)
                {
                    (void)Hide();
                }
                else
                {
                    UpdateRenderCadence(
                        imguiInteractive);
                }
            }
            catch (...)
            {
            }

            rendering_ = false;
        }

        void UpdateSelectionProbe()
        {
            if (state_.page != UI::Page::Cut ||
                GetForegroundWindow() != hwnd_)
            {
                return;
            }

            const ULONGLONG now =
                GetTickCount64();

            if (now - lastUiActivity_ <
                    SelectionProbeIdleDelayMs ||
                now - lastSelectionProbe_ <
                    SelectionProbeIntervalMs)
            {
                return;
            }

            if (!selectionProbePending_ && lastProbedClosureMode_ == state_.cut.closureMode) return;
            selectionProbePending_ = false;
            lastProbedClosureMode_ = state_.cut.closureMode;
            lastSelectionProbe_ = now;

            state_.runtime.detectedClosures =
                DetectClosureCount();
        }

        [[nodiscard]] int DetectClosureCount()
        {
            try
            {
                auto selection = app_->ActiveSelectionRange;
                if (!selection) return 0;
                const long count = selection->Count;
                if (count <= 0) return 0;
                switch (state_.cut.closureMode)
                {
                case UI::ClosureMode::WholeSelection: return 1;
                case UI::ClosureMode::EachSelectedObject: return count;
                default:
                    auto groups = selection->Shapes->FindShapes(_bstr_t(), cdrGroupShape, VARIANT_FALSE, _bstr_t());
                    const bool allGroups = groups && groups->Count == count;
                    return state_.cut.closureMode == UI::ClosureMode::EachSelectedGroup
                        ? (allGroups ? count : 0) : (allGroups ? count : 1);
                }
            }
            catch (...) { return 0; }
        }

        HRESULT RunCutInternal(
            const UI::CutState& request)
        {
            return ExecuteOperation(
                L"Preparing cut",
                [this, request]()
                {
                    Cut::Settings settings;
                    settings.pageWidth =
                        (std::max)(
                            request.pageWidthMm,
                            1);

                    settings.regmarks =
                        (std::max)(
                            request.registrationMarks,
                            0);

                    settings.registrationMarginMillimeters =
                        std::isfinite(request.registrationMarginMm)
                        ? (std::clamp)(
                            static_cast<double>(
                                request.registrationMarginMm),
                            0.0,
                            1000.0)
                        : 5.0;

                    settings.namePages =
                        request.namePages;

                    settings.showSummary =
                        request.showSummary;

                    settings.forceBottomRightAnchor =
                        request.forceBottomRightAnchor;

                    switch (
                        request.closureMode)
                    {
                    case UI::ClosureMode::WholeSelection:
                        settings.closureMode =
                            Cut::ClosureMode::WholeSelection;
                        break;

                    case UI::ClosureMode::EachSelectedGroup:
                        settings.closureMode =
                            Cut::ClosureMode::EachSelectedGroup;
                        break;

                    case UI::ClosureMode::EachSelectedObject:
                        settings.closureMode =
                            Cut::ClosureMode::EachSelectedObject;
                        break;

                    case UI::ClosureMode::Automatic:
                    default:
                        settings.closureMode =
                            Cut::ClosureMode::Auto;
                        break;
                    }

                    auto result =
                        Cut::Process(
                            app_,
                            settings);

                    if (!result.error.empty())
                    {
                        return SetError(
                            E_FAIL,
                            Utf8ToWide(
                                result.error));
                    }

                    state_.runtime.progress =
                        1.0f;

                    state_.runtime.status = "Corte: " + std::to_string(result.processedCount) +
                        " fechamento(s), " + std::to_string(result.warnings) + " aviso(s).";
                    if (result.warnings && !request.showSummary)
                        MessageBoxW(hwnd_, L"O corte terminou com avisos: confira as marcas e os contornos antes de produzir.",
                            L"ImCut - Corte", MB_OK | MB_ICONWARNING);
                    return result.warnings ? S_FALSE : S_OK;
                });
        }

        HRESULT RunBleedInternal(
            const UI::BleedState& request)
        {
            return ExecuteOperation(
                L"Creating bleed",
                [this, request]()
                {
                    AutoBleeding::Settings settings;
                    settings.bleedMillimeters =
                        (std::max)(
                            static_cast<double>(
                                request.distanceMm),
                            0.0);

                    settings.flattenGroups =
                        request.ungroupBeforeProcessing;

                    settings.detectHidden =
                        request.detectHiddenObjects;

                    settings.createCutline =
                        request.createCutline;

                    settings.showFinalSummary =
                        false;

                    std::string bleedDiagnostics;
                    long renderedProgress = -1;
                    long renderedTotal = -1;
                    AutoBleeding::Callbacks callbacks;

                    callbacks.onProgress =
                        [this, &bleedDiagnostics, &renderedProgress, &renderedTotal](
                            const AutoBleeding::ProgressInfo& info)
                        {
                            Cancellation::ThrowIfRequested();

                            if (info.total > 0)
                            {
                                state_.runtime.progress =
                                    static_cast<float>(
                                        info.current) /
                                    static_cast<float>(
                                        info.total);

                                state_.runtime.status =
                                    "Processando " +
                                    std::to_string(info.current) +
                                    "/" +
                                    std::to_string(info.total) +
                                    "...";
                            }
                            else
                            {
                                state_.runtime.status = info.message;
                            }

                            if ((info.message.rfind("Falha", 0) == 0 ||
                                 info.message.rfind("Fallback:", 0) == 0) && bleedDiagnostics.size() < 12000)
                                bleedDiagnostics += info.message + "\n";

                            if (visible_ &&
                                (info.current != renderedProgress ||
                                 info.total != renderedTotal))
                            {
                                renderedProgress = info.current;
                                renderedTotal = info.total;
                                RenderFrame();
                            }
                        };

                    callbacks.onCheckpoint =
                        [](
                            long,
                            long)
                        {
                            Cancellation::ThrowIfRequested();
                            return true;
                        };

                    const auto result =
                        AutoBleeding::Run(
                            app_,
                            settings,
                            callbacks);

                    if (!result.fatalError.empty())
                    {
                        return SetError(
                            E_FAIL,
                            Utf8ToWide(
                                result.fatalError));
                    }

                    state_.runtime.progress =
                        1.0f;

                    std::ostringstream status;
                    status
                        << "Bleed: "
                        << result.successCount
                        << " completed";

                    if (result.failedCount > 0)
                    {
                        status
                            << ", "
                            << result.failedCount
                            << " failed";
                    }

                    status << "; distancia " << request.distanceMm << " mm";
                    state_.runtime.status = status.str();
                    if (!bleedDiagnostics.empty())
                    {
                        const auto details = Utf8ToWide(bleedDiagnostics);
                        (void)SetError(S_FALSE, details);
                    }

                    return
                        result.failedCount > 0
                        ? S_FALSE
                        : S_OK;
                }, false);
        }

        HRESULT RunColorInternal(
            const UI::ColorState& request,
            bool showCompletionDialog = true,
            bool showErrorDialog = true)
        {
            return ExecuteOperation(
                L"Converting colors",
                [this, request, showCompletionDialog]()
                {
                    if (request.selectedCmyk < 0 ||
                        request.selectedCmyk >=
                        static_cast<int>(
                            cmykProfiles_.size()))
                    {
                        return SetError(
                            E_INVALIDARG,
                            L"Select a valid CMYK ICC profile.");
                    }

                    if (request.selectedRgb < 0 ||
                        request.selectedRgb >=
                        static_cast<int>(
                            rgbProfiles_.size()))
                    {
                        return SetError(
                            E_INVALIDARG,
                            L"Select a valid RGB ICC profile.");
                    }

                    if (request.colorMappingEnabled)
                    {
                        for (const auto& mapping :
                            request.colorMappings)
                        {
                            if (!mapping.enabled)
                                continue;

                            if (mapping.sourceKind ==
                                UI::ColorMapSourceKind::Spot &&
                                mapping.sourceSpotName[0] == '\0')
                            {
                                return SetError(
                                    E_INVALIDARG,
                                    L"A Spot source mapping requires at least one Spot color name or alias.");
                            }

                            if (mapping.targetKind ==
                                UI::ColorMapTargetKind::Spot &&
                                (mapping.targetSpotPalette[0] == '\0' ||
                                    mapping.targetSpotName[0] == '\0'))
                            {
                                return SetError(
                                    E_INVALIDARG,
                                    L"A Spot target mapping requires both palette identifier and Spot color name.");
                            }
                        }
                    }

                    auto doc =
                        app_->ActiveDocument;

                    if (!doc)
                    {
                        return SetError(
                            E_FAIL,
                            L"No active document.");
                    }

                    auto selection =
                        app_->ActiveSelectionRange;

                    if (!selection ||
                        selection->Count <= 0)
                    {
                        return SetError(
                            E_FAIL,
                            L"Select at least one object.");
                    }

                    Color::BuildOptions options;

                    const UI::RenderingIntent effectiveIntent =
                        request.conversionMode ==
                            UI::ColorConversionMode::PreserveAppearance
                        ? UI::RenderingIntent::AbsoluteColorimetric
                        : request.intent;

                    options.conversionMode =
                        request.conversionMode ==
                            UI::ColorConversionMode::PreserveAppearance
                        ? Color::ConversionMode::PreserveAppearance
                        : Color::ConversionMode::StandardIcc;

                    options.enableLabTargetTransform =
                        request.convertSpotsToRgb &&
                        request.preserveSpotAppearance;

                    options.appearanceExhaustive =
                        request.exhaustiveAppearanceSearch;

                    options.adaptiveGrid =
                        request.adaptiveLut;

                    options.preferredGrid =
                        static_cast<std::uint32_t>(
                            (std::clamp)(
                                request.preferredGrid,
                                2,
                                65));

                    options.blackFloorEnabled =
                        request.blackFloorEnabled &&
                        request.conversionMode ==
                            UI::ColorConversionMode::StandardIcc;

                    options.blackFloorRgb =
                        static_cast<std::uint8_t>(
                            (std::clamp)(
                                request.blackFloorRgb,
                                0,
                                255));

                    switch (effectiveIntent)
                    {
                    case UI::RenderingIntent::RelativeColorimetric:
                        options.intent =
                            Color::RenderingIntent::RelativeColorimetric;
                        break;

                    case UI::RenderingIntent::Saturation:
                        options.intent =
                            Color::RenderingIntent::Saturation;
                        break;

                    case UI::RenderingIntent::AbsoluteColorimetric:
                        options.intent =
                            Color::RenderingIntent::AbsoluteColorimetric;
                        break;

                    case UI::RenderingIntent::Perceptual:
                    default:
                        options.intent =
                            Color::RenderingIntent::Perceptual;
                        break;
                    }

                    colorConverter_.Initialize(
                        cmykProfiles_[
                            static_cast<std::size_t>(
                                request.selectedCmyk)],
                        rgbProfiles_[
                            static_cast<std::size_t>(
                                request.selectedRgb)],
                        options);

                    ColorExecutionGuard guard(
                        app_,
                        doc);

                    const auto& selectedCmykProfile =
                        cmykProfiles_[
                            static_cast<std::size_t>(
                                request.selectedCmyk)];

                    const auto& selectedRgbProfile =
                        rgbProfiles_[
                            static_cast<std::size_t>(
                                request.selectedRgb)];

                    clrRenderingIntent corelIntent =
                        clrRenderPerceptual;

                    switch (effectiveIntent)
                    {
                    case UI::RenderingIntent::RelativeColorimetric:
                        corelIntent =
                            clrRenderRelative;
                        break;

                    case UI::RenderingIntent::Saturation:
                        corelIntent =
                            clrRenderSaturation;
                        break;

                    case UI::RenderingIntent::AbsoluteColorimetric:
                        corelIntent =
                            clrRenderAbsolute;
                        break;

                    case UI::RenderingIntent::Perceptual:
                    default:
                        corelIntent =
                            clrRenderPerceptual;
                        break;
                    }

                    auto currentDocumentColorContext =
                        doc->ColorContext;

                    if (!currentDocumentColorContext)
                    {
                        return SetError(
                            E_FAIL,
                            L"Unable to read the CorelDRAW document color context.");
                    }

                    const std::wstring selectedRgbCorelName =
                        ResolveCorelProfileName(
                            selectedRgbProfile,
                            clrColorModelRGB);

                    const std::wstring selectedCmykCorelName =
                        ResolveCorelProfileName(
                            selectedCmykProfile,
                            clrColorModelCMYK);

                    const std::wstring selectedProfiles =
                        selectedRgbCorelName +
                        L"," +
                        selectedCmykCorelName;

                    auto selectedColorContext =
                        app_->CreateColorContext2(
                            _bstr_t(
                                selectedProfiles.c_str()),
                            corelIntent,
                            clrColorModelRGB);

                    if (!selectedColorContext)
                    {
                        return SetError(
                            E_FAIL,
                            L"Unable to create the selected RGB/CMYK color context.");
                    }

                    if (!request.assignDocumentProfiles &&
                        (!currentDocumentColorContext->RGBColorProfile || !selectedColorContext->RGBColorProfile ||
                         !currentDocumentColorContext->RGBColorProfile->IsSame(selectedColorContext->RGBColorProfile)))
                        return SetError(E_INVALIDARG,
                            L"Escolha o perfil RGB atual do documento ou habilite Aplicar perfis ao documento inteiro. Nenhum objeto foi alterado.");

                    auto documentColorContext =
                        currentDocumentColorContext->GetCopy();

                    if (!documentColorContext)
                    {
                        return SetError(
                            E_FAIL,
                            L"Unable to copy the CorelDRAW document color context.");
                    }

                    documentColorContext->RGBColorProfile =
                        selectedColorContext->RGBColorProfile;

                    documentColorContext->CMYKColorProfile =
                        selectedColorContext->CMYKColorProfile;

                    documentColorContext->RenderingIntent =
                        corelIntent;

                    documentColorContext->BlendingColorModel =
                        clrColorModelRGB;

                    SpotAppearanceCache spotAppearanceCache;

                    if (request.convertSpotsToRgb &&
                        request.preserveSpotAppearance)
                    {
                        for (long i = 1;
                            i <= selection->Count;
                            ++i)
                        {
                            Cancellation::ThrowIfRequested();
                            CaptureSpotAppearanceRecursive(
                                selection->Item[i],
                                spotAppearanceCache);
                        }
                    }

                    if (request.assignDocumentProfiles)
                    {
                        doc->AssignColorContext(documentColorContext);
                    }

                    colorTargetPreviewCache_.clear();
                    pendingColorPreviewRequests_.clear();

                    ColorConversionStats stats;

                    const SpotAppearanceCache* const activeSpotAppearance =
                        spotAppearanceCache.empty()
                        ? nullptr
                        : &spotAppearanceCache;

                    for (long i = 1;
                        i <= selection->Count;
                        ++i)
                    {
                        Cancellation::ThrowIfRequested();
                        ConvertShapeRecursive(
                            selection->Item[i],
                            request,
                            stats,
                            activeSpotAppearance);
                    }

                    state_.runtime.progress =
                        1.0f;

                    state_.runtime.status =
                        "Converted " +
                        std::to_string(
                            stats.converted) +
                        " color" +
                        (stats.converted == 1 ? "" : "s") +
                        "; " + std::to_string(stats.errors) + " erro(s)";

                    if (showCompletionDialog && request.showSummary)
                    {
                        std::wostringstream message;

                        message
                            << L"Conversão de cores concluída.\n\n"
                            << L"CMYK: "
                            << selectedCmykCorelName
                            << L"\nRGB: "
                            << selectedRgbCorelName
                            << L"\nMétodo: "
                            << (request.conversionMode ==
                                    UI::ColorConversionMode::PreserveAppearance
                                ? L"Preservar aparência (ICC-Absolute / Lab exato / DeltaE)"
                                : L"ICC padrão")
                            << L"\nBusca RGB8 exaustiva: "
                            << (request.exhaustiveAppearanceSearch
                                ? L"ativada (Esc para cancelar)"
                                : L"desativada")
                            << L"\n\nCores analisadas: "
                            << stats.examined
                            << L"\nCores alteradas: "
                            << stats.converted
                            << L"\n\nDetalhamento:"
                            << L"\n  Color Mapping: "
                            << stats.mapped
                            << L"\n  Spot Tint -> branco: "
                            << stats.spotTintWhite
                            << L"\n  Spot -> RGB (Lab exato / DeltaE): "
                            << stats.spotAppearancePreserved
                            << L"\n  Spot -> RGB (Corel/fallback): "
                            << stats.spotToRgb
                            << L"\n  CMYK -> RGB ("
                            << (request.conversionMode ==
                                    UI::ColorConversionMode::PreserveAppearance
                                ? L"Lab absoluto / Lab exato / DeltaE"
                                : L"ICC")
                            << L"): "
                            << stats.cmykToRgb
                            << L"\n  Limite de preto em RGB/Spot: "
                            << stats.blackFloorAdjusted
                            << L"\n  Protegidas pela blacklist: "
                            << stats.blacklisted
                            << L"\n\nSem alteração: "
                            << stats.unchanged
                            << L"\nErros durante o processamento: "
                            << stats.errors;

                        const auto appendAccuracy =
                            [&](
                                const wchar_t* label,
                                const AppearanceAccuracyStats& accuracy)
                            {
                                if (accuracy.samples <= 0)
                                    return;

                                message
                                    << L"\n\n"
                                    << label
                                    << L" - fidelidade medida:"
                                    << std::fixed
                                    << std::setprecision(3)
                                    << L"\n  DeltaE00 médio: "
                                    << accuracy.MeanDeltaE()
                                    << L"\n  DeltaE00 máximo: "
                                    << accuracy.deltaEMax
                                    << L"\n  DeltaE76 (distância Lab) médio: "
                                    << accuracy.MeanDeltaE76()
                                    << L"\n  DeltaE76 (distância Lab) máximo: "
                                    << accuracy.deltaE76Max
                                    << L"\n  |Delta L*| médio/máx: "
                                    << accuracy.MeanAbsDeltaL()
                                    << L" / "
                                    << accuracy.absDeltaLMax
                                    << L"\n  |Delta a*| médio/máx: "
                                    << accuracy.MeanAbsDeltaA()
                                    << L" / "
                                    << accuracy.absDeltaAMax
                                    << L"\n  |Delta b*| médio/máx: "
                                    << accuracy.MeanAbsDeltaB()
                                    << L" / "
                                    << accuracy.absDeltaBMax
                                    << std::setprecision(1)
                                    << L"\n  <= 0.25: "
                                    << accuracy.PercentWithin(
                                        accuracy.deltaELe025)
                                    << L"%"
                                    << L"\n  <= 0.50: "
                                    << accuracy.PercentWithin(
                                        accuracy.deltaELe050)
                                    << L"%"
                                    << L"\n  <= 1.00: "
                                    << accuracy.PercentWithin(
                                        accuracy.deltaELe100)
                                    << L"%"
                                    << L"\n  Cores com busca global: "
                                    << accuracy.globalSearches
                                    << L"/"
                                    << accuracy.samples
                                    << L"\n  Cores com busca densa: "
                                    << accuracy.denseSearches
                                    << L"/"
                                    << accuracy.samples
                                    << L"\n  Cores com busca RGB8 exaustiva: "
                                    << accuracy.exhaustiveSearches
                                    << L"/"
                                    << accuracy.samples;
                            };

                        appendAccuracy(
                            L"CMYK -> RGB",
                            stats.cmykAccuracy);
                        appendAccuracy(
                            L"Spot -> RGB",
                            stats.spotAccuracy);

                        const bool accuracyWarning =
                            stats.cmykAccuracy.deltaEMax > 1.0 ||
                            stats.spotAccuracy.deltaEMax > 1.0 ||
                            stats.cmykAccuracy.deltaE76Max > 1.0 ||
                            stats.spotAccuracy.deltaE76Max > 1.0;

                        if (accuracyWarning)
                        {
                            message
                                << L"\n\nAviso: pelo menos uma cor terminou acima de 1.0 em DeltaE00 ou distância Lab. "
                                << L"Isso normalmente indica limite de gamut/quantização do RGB destino; "
                                << L"o ImCut já selecionou o melhor candidato encontrado pela busca de máxima precisão.";
                        }

                        OutputDebugStringW(message.str().c_str());
                        std::wostringstream summary;
                        summary << stats.converted << L" cores alteradas.\n"
                            << stats.mapped << L" por regras de mapeamento.\n"
                            << stats.unchanged << L" sem alteração.\n"
                            << stats.errors << L" erros.";
                        if (accuracyWarning) summary << L"\n\nAlgumas cores ficaram fora da tolerância. Confira o resultado.";
                        MessageBoxW(
                            DialogOwnerWindow(),
                            summary.str().c_str(),
                            L"ImCut - Conversão",
                            MB_OK |
                            ((stats.errors > 0 ||
                                accuracyWarning)
                                ? MB_ICONWARNING
                                : MB_ICONINFORMATION));
                    }

                    return stats.errors > 0
                        ? S_FALSE
                        : S_OK;
                }, showErrorDialog);
        }

        HRESULT RunNestingInternal(
            const UI::NestingState& request)
        {
            return ExecuteOperation(
                L"Running nesting",
                [this, request]()
                {
                    const auto nestingCallback = nestingCallback_.load();
                    if (!nestingCallback)
                    {
                        return SetError(
                            E_NOTIMPL,
                            L"No True Shape Nesting backend is registered.");
                    }

                    ImCutNestingRequest nativeRequest{};
                    nativeRequest.spacingMm =
                        request.spacingMm;
                    nativeRequest.rotationStepDeg =
                        request.rotationStepDeg;
                    nativeRequest.generations =
                        request.generations;
                    nativeRequest.population =
                        request.population;
                    nativeRequest.allowMirror =
                        request.allowMirror
                        ? TRUE
                        : FALSE;
                    nativeRequest.useTrueShape =
                        request.useTrueShape
                        ? TRUE
                        : FALSE;

                    IUnknown* unknown =
                        app_
                        ? static_cast<IUnknown*>(
                            app_.GetInterfacePtr())
                        : nullptr;

                    const HRESULT hr =
                        nestingCallback(
                            unknown,
                            &nativeRequest);

                    if (FAILED(hr))
                    {
                        return SetError(
                            hr,
                            L"The nesting backend returned an error.");
                    }

                    state_.runtime.progress =
                        1.0f;
                    state_.runtime.status =
                        "Nesting completed";

                    return hr;
                });
        }

        HRESULT ImportColorMappingsInternal()
        {
            std::filesystem::path filePath;
            std::wstring dialogError;

            if (!SelectColorMappingFile(
                hwnd_,
                false,
                filePath,
                dialogError))
            {
                if (!dialogError.empty())
                {
                    (void)SetError(
                        E_FAIL,
                        dialogError);

                    ShowErrorMessage();
                    return E_FAIL;
                }

                return S_FALSE;
            }

            std::wstring importError;

            if (!SettingsStore::ImportColorMappings(
                filePath,
                state_.color,
                importError))
            {
                (void)SetError(
                    E_FAIL,
                    importError.empty()
                    ? L"Unable to import Color Mapping."
                    : importError);

                ShowErrorMessage();
                return E_FAIL;
            }

            ClearError();

            state_.runtime.status =
                "Color mapping imported";

            SavePersistentSettings();

            return S_OK;
        }

        HRESULT ExportColorMappingsInternal()
        {
            std::filesystem::path filePath;
            std::wstring dialogError;

            if (!SelectColorMappingFile(
                hwnd_,
                true,
                filePath,
                dialogError))
            {
                if (!dialogError.empty())
                {
                    (void)SetError(
                        E_FAIL,
                        dialogError);

                    ShowErrorMessage();
                    return E_FAIL;
                }

                return S_FALSE;
            }

            std::wstring exportError;

            if (!SettingsStore::ExportColorMappings(
                filePath,
                state_.color,
                exportError))
            {
                (void)SetError(
                    E_FAIL,
                    exportError.empty()
                    ? L"Unable to export Color Mapping."
                    : exportError);

                ShowErrorMessage();
                return E_FAIL;
            }

            ClearError();

            state_.runtime.status =
                "Color mapping exported";

            return S_OK;
        }

        HRESULT ImportSettingsInternal()
        {
            std::filesystem::path filePath;
            std::wstring dialogError;

            if (!SelectSettingsFile(
                hwnd_,
                false,
                filePath,
                dialogError))
            {
                if (!dialogError.empty())
                {
                    (void)SetError(
                        E_FAIL,
                        dialogError);

                    ShowErrorMessage();
                    return E_FAIL;
                }

                return S_FALSE;
            }

            UI::State importedState =
                state_;

            StoredProfiles importedProfiles =
                storedProfiles_;

            std::wstring importError;

            if (!SettingsStore::ImportSettings(
                filePath,
                importedState,
                importedProfiles,
                importError))
            {
                (void)SetError(
                    E_FAIL,
                    importError.empty()
                    ? L"Unable to import ImCut settings."
                    : importError);

                ShowErrorMessage();
                return E_FAIL;
            }

            state_ =
                std::move(importedState);

            storedProfiles_ =
                importedProfiles;

            state_.color.selectedCmyk = -1;
            state_.color.selectedRgb = -1;

            previewSrgbConverters_.clear();
            colorTargetPreviewCache_.clear();
            pendingColorPreviewRequests_.clear();

            const HRESULT refreshHr =
                RefreshColorProfilesInternal();

            if (FAILED(refreshHr))
            {
                state_.runtime.status =
                    "Settings imported; ICC profile refresh failed";
            }
            else
            {
                state_.runtime.status =
                    "ImCut settings imported";
            }

            SavePersistentSettings();
            ClearError();

            return S_OK;
        }

        HRESULT ExportSettingsInternal()
        {
            std::filesystem::path filePath;
            std::wstring dialogError;

            if (!SelectSettingsFile(
                hwnd_,
                true,
                filePath,
                dialogError))
            {
                if (!dialogError.empty())
                {
                    (void)SetError(
                        E_FAIL,
                        dialogError);

                    ShowErrorMessage();
                    return E_FAIL;
                }

                return S_FALSE;
            }

            UpdateStoredProfileSelection();

            std::wstring exportError;

            if (!SettingsStore::ExportSettings(
                filePath,
                state_,
                storedProfiles_,
                exportError))
            {
                (void)SetError(
                    E_FAIL,
                    exportError.empty()
                    ? L"Unable to export ImCut settings."
                    : exportError);

                ShowErrorMessage();
                return E_FAIL;
            }

            ClearError();

            state_.runtime.status =
                "ImCut settings exported";

            return S_OK;
        }

        void LoadPersistentSettings()
        {
            if (settingsLoaded_)
                return;

            settingsStore_.Load(
                state_,
                storedProfiles_);

            if (state_.page ==
                UI::Page::Nesting)
            {
                state_.page =
                    UI::Page::Cut;
            }

            observedSettingsSignature_ =
                settingsStore_.Signature(
                    state_,
                    storedProfiles_);

            settingsLoaded_ = true;
        }

        void UpdateStoredProfileSelection() noexcept
        {
            if (!profilesLoaded_) return;
            storedProfiles_.cmykFingerprint =
                SelectedFingerprint(
                    cmykProfiles_,
                    state_.color.selectedCmyk);

            storedProfiles_.rgbFingerprint =
                SelectedFingerprint(
                    rgbProfiles_,
                    state_.color.selectedRgb);
        }

        void TrackPersistentSettings()
        {
            if (!settingsLoaded_)
                return;

            const ULONGLONG now =
                GetTickCount64();

            if (now - lastUiActivity_ <
                SettingsIdleDelayMs)
            {
                return;
            }

            if (now -
                lastSettingsSignatureCheck_ >=
                    SettingsSignatureIntervalMs)
            {
                lastSettingsSignatureCheck_ = now;

                StoredProfiles currentProfiles =
                    storedProfiles_;

                if (profilesLoaded_)
                {
                    currentProfiles.cmykFingerprint = SelectedFingerprint(cmykProfiles_, state_.color.selectedCmyk);
                    currentProfiles.rgbFingerprint = SelectedFingerprint(rgbProfiles_, state_.color.selectedRgb);
                }

                const std::uint64_t signature =
                    settingsStore_.Signature(
                        state_,
                        currentProfiles);

                if (signature ==
                    observedSettingsSignature_)
                {
                    settingsSavePending_ = false;
                    pendingSettingsSignature_ = 0;
                }
                else if (!settingsSavePending_ ||
                    signature !=
                        pendingSettingsSignature_)
                {
                    pendingSettingsSignature_ =
                        signature;

                    settingsDirtySince_ =
                        now;

                    settingsSavePending_ =
                        true;
                }
            }

            if (!settingsSavePending_ ||
                now - settingsDirtySince_ <
                    SettingsSaveDebounceMs)
            {
                return;
            }

            StoredProfiles currentProfiles =
                storedProfiles_;

            if (profilesLoaded_)
            {
                currentProfiles.cmykFingerprint = SelectedFingerprint(cmykProfiles_, state_.color.selectedCmyk);
                currentProfiles.rgbFingerprint = SelectedFingerprint(rgbProfiles_, state_.color.selectedRgb);
            }

            const std::uint64_t currentSignature =
                settingsStore_.Signature(
                    state_,
                    currentProfiles);

            if (currentSignature !=
                pendingSettingsSignature_)
            {
                pendingSettingsSignature_ =
                    currentSignature;

                settingsDirtySince_ =
                    now;

                return;
            }

            if (!settingsStore_.Save(
                state_,
                currentProfiles))
            {
                state_.runtime.status = "Falha ao salvar configuracoes; a versao anterior foi preservada. Nova tentativa pendente.";
                settingsDirtySince_ = now;
                return;
            }

            storedProfiles_ =
                currentProfiles;

            observedSettingsSignature_ =
                currentSignature;

            settingsSavePending_ = false;
            pendingSettingsSignature_ = 0;
        }

        void SavePersistentSettings()
        {
            if (!settingsLoaded_)
                return;

            UpdateStoredProfileSelection();

            const std::uint64_t signature =
                settingsStore_.Signature(
                    state_,
                    storedProfiles_);

            if (signature == observedSettingsSignature_)
            {
                settingsSavePending_ = false;
                pendingSettingsSignature_ = 0;
                settingsDirtySince_ = 0;
                lastSettingsSignatureCheck_ =
                    GetTickCount64();
                return;
            }

            if (!settingsStore_.Save(
                state_,
                storedProfiles_))
            {
                state_.runtime.status = "Falha ao salvar configuracoes; a versao anterior foi preservada.";
                settingsSavePending_ = true;
                settingsDirtySince_ = GetTickCount64();
                return;
            }

            observedSettingsSignature_ =
                signature;

            settingsSavePending_ = false;
            pendingSettingsSignature_ = 0;
            settingsDirtySince_ = 0;
            lastSettingsSignatureCheck_ =
                GetTickCount64();
        }

        HRESULT EnsureColorProfiles()
        {
            return profilesLoaded_ ? S_OK : RefreshColorProfilesInternal();
        }

        HRESULT RefreshColorProfilesInternal()
        {
            try
            {
                std::uint64_t oldCmykFingerprint =
                    SelectedFingerprint(
                        cmykProfiles_,
                        state_.color.selectedCmyk);

                std::uint64_t oldRgbFingerprint =
                    SelectedFingerprint(
                        rgbProfiles_,
                        state_.color.selectedRgb);

                if (oldCmykFingerprint == 0)
                    oldCmykFingerprint =
                    storedProfiles_.cmykFingerprint;

                if (oldRgbFingerprint == 0)
                    oldRgbFingerprint =
                    storedProfiles_.rgbFingerprint;

                auto windowsCmykProfiles =
                    Color::ProfileCatalog::EnumerateCMYK();

                auto windowsRgbProfiles =
                    Color::ProfileCatalog::EnumerateRGB();

                cmykProfiles_ =
                    BuildCorelProfileCatalog(
                        windowsCmykProfiles,
                        clrColorModelCMYK);

                rgbProfiles_ =
                    BuildCorelProfileCatalog(
                        windowsRgbProfiles,
                        clrColorModelRGB);

                if (cmykProfiles_.empty())
                    cmykProfiles_ = std::move(windowsCmykProfiles);

                if (rgbProfiles_.empty())
                    rgbProfiles_ = std::move(windowsRgbProfiles);

                previewSrgbConverters_.clear();
                colorTargetPreviewCache_.clear();
                pendingColorPreviewRequests_.clear();

                state_.color.cmykProfiles.clear();
                state_.color.rgbProfiles.clear();

                state_.color.cmykProfiles.reserve(
                    cmykProfiles_.size());

                state_.color.rgbProfiles.reserve(
                    rgbProfiles_.size());

                for (const auto& profile :
                    cmykProfiles_)
                {
                    state_.color.cmykProfiles.push_back(
                        WideToUtf8(
                            profile.displayName));
                }

                for (const auto& profile :
                    rgbProfiles_)
                {
                    state_.color.rgbProfiles.push_back(
                        WideToUtf8(
                            profile.displayName));
                }

                state_.color.selectedCmyk =
                    FindProfileIndex(
                        cmykProfiles_,
                        oldCmykFingerprint);

                state_.color.selectedRgb =
                    FindProfileIndex(
                        rgbProfiles_,
                        oldRgbFingerprint);

                if (state_.color.selectedCmyk < 0 &&
                    !cmykProfiles_.empty())
                {
                    state_.color.selectedCmyk = 0;
                }

                if (state_.color.selectedRgb < 0 &&
                    !rgbProfiles_.empty())
                {
                    state_.color.selectedRgb = 0;
                }

                UpdateStoredProfileSelection();

                state_.runtime.status =
                    "Ready";

                profilesLoaded_ = true;
                return S_OK;
            }
            catch (const _com_error& error)
            {
                return SetError(
                    error.Error(),
                    L"Unable to enumerate ICC profiles: " +
                    ComErrorText(error));
            }
            catch (const std::exception& error)
            {
                return SetError(
                    E_FAIL,
                    L"Unable to enumerate ICC profiles: " +
                    Utf8ToWide(
                        error.what()));
            }
            catch (...)
            {
                return SetError(
                    E_FAIL,
                    L"Unable to enumerate ICC profiles.");
            }
        }

        struct CorelProfileRecord
        {
            std::wstring name;
            std::wstring fileName;
        };

        [[nodiscard]] static std::wstring NormalizeLookupText(
            const std::wstring& value)
        {
            std::wstring normalized;
            normalized.reserve(value.size());

            for (const wchar_t ch : value)
            {
                if (std::iswalnum(ch))
                {
                    normalized.push_back(
                        static_cast<wchar_t>(
                            std::towupper(ch)));
                }
            }

            return normalized;
        }

        [[nodiscard]] static std::wstring FileNameKey(
            const std::wstring& value)
        {
            if (value.empty())
                return {};

            try
            {
                return NormalizeLookupText(
                    std::filesystem::path(value)
                        .filename()
                        .wstring());
            }
            catch (...)
            {
                return NormalizeLookupText(value);
            }
        }

        [[nodiscard]] static int ProfileMatchScore(
            const Color::ProfileInfo& windowsProfile,
            const CorelProfileRecord& corelProfile)
        {
            const std::wstring windowsFile =
                FileNameKey(
                    windowsProfile.path
                        .filename()
                        .wstring());

            const std::wstring corelFile =
                FileNameKey(corelProfile.fileName);

            if (!windowsFile.empty() &&
                !corelFile.empty() &&
                windowsFile == corelFile)
            {
                return 0;
            }

            const std::wstring windowsName =
                NormalizeLookupText(
                    windowsProfile.displayName);

            const std::wstring corelName =
                NormalizeLookupText(
                    corelProfile.name);

            if (windowsName.empty() ||
                corelName.empty())
            {
                return -1;
            }

            if (windowsName == corelName)
                return 10;

            const std::size_t shorter =
                (std::min)(
                    windowsName.size(),
                    corelName.size());

            if (shorter < 4)
                return -1;

            if (windowsName.find(corelName) !=
                    std::wstring::npos ||
                corelName.find(windowsName) !=
                    std::wstring::npos)
            {
                const std::size_t delta =
                    windowsName.size() > corelName.size()
                    ? windowsName.size() - corelName.size()
                    : corelName.size() - windowsName.size();

                return 100 +
                    static_cast<int>(
                        (std::min)(
                            delta,
                            static_cast<std::size_t>(10000)));
            }

            return -1;
        }

        [[nodiscard]] std::vector<CorelProfileRecord>
            EnumerateCorelProfiles(
                clrColorModel colorModel) const
        {
            std::vector<CorelProfileRecord> result;

            if (!app_)
                return result;

            try
            {
                auto manager = app_->ColorManager;

                if (!manager)
                    return result;

                auto profiles = manager->ColorProfiles;

                if (!profiles ||
                    profiles->Count <= 0)
                {
                    return result;
                }

                result.reserve(
                    static_cast<std::size_t>(
                        profiles->Count));

                for (long i = 1;
                    i <= profiles->Count;
                    ++i)
                {
                    try
                    {
                        auto profile = profiles->Item[i];

                        if (!profile ||
                            profile->ColorModel != colorModel ||
                            profile->Installed == VARIANT_FALSE)
                        {
                            continue;
                        }

                        CorelProfileRecord record;
                        record.name =
                            BstrToWide(
                                _bstr_t(profile->Name));
                        record.fileName =
                            BstrToWide(
                                _bstr_t(profile->FileName));

                        if (!record.name.empty())
                            result.push_back(std::move(record));
                    }
                    catch (...)
                    {
                    }
                }
            }
            catch (...)
            {
                result.clear();
            }

            return result;
        }

        [[nodiscard]] std::vector<Color::ProfileInfo>
            BuildCorelProfileCatalog(
                const std::vector<Color::ProfileInfo>& windowsProfiles,
                clrColorModel colorModel) const
        {
            const auto corelProfiles =
                EnumerateCorelProfiles(colorModel);

            if (corelProfiles.empty() ||
                windowsProfiles.empty())
            {
                return {};
            }

            std::vector<Color::ProfileInfo> result;
            std::vector<bool> used(
                windowsProfiles.size(),
                false);

            result.reserve(
                (std::min)(
                    corelProfiles.size(),
                    windowsProfiles.size()));

            for (const auto& corelProfile : corelProfiles)
            {
                int bestScore = -1;
                std::size_t bestIndex =
                    windowsProfiles.size();

                for (std::size_t i = 0;
                    i < windowsProfiles.size();
                    ++i)
                {
                    if (used[i])
                        continue;

                    const int score =
                        ProfileMatchScore(
                            windowsProfiles[i],
                            corelProfile);

                    if (score < 0)
                        continue;

                    if (bestScore < 0 ||
                        score < bestScore)
                    {
                        bestScore = score;
                        bestIndex = i;
                    }
                }

                if (bestIndex >= windowsProfiles.size())
                    continue;

                auto profile =
                    windowsProfiles[bestIndex];

                profile.displayName =
                    corelProfile.name;

                result.push_back(
                    std::move(profile));

                used[bestIndex] = true;
            }

            std::sort(
                result.begin(),
                result.end(),
                [](const Color::ProfileInfo& a,
                    const Color::ProfileInfo& b)
                {
                    return _wcsicmp(
                        a.displayName.c_str(),
                        b.displayName.c_str()) < 0;
                });

            return result;
        }

        [[nodiscard]] std::wstring ResolveCorelProfileName(
            const Color::ProfileInfo& profile,
            clrColorModel colorModel) const
        {
            const auto corelProfiles =
                EnumerateCorelProfiles(colorModel);

            int bestScore = -1;
            const CorelProfileRecord* best = nullptr;

            for (const auto& corelProfile : corelProfiles)
            {
                const int score =
                    ProfileMatchScore(
                        profile,
                        corelProfile);

                if (score < 0)
                    continue;

                if (bestScore < 0 ||
                    score < bestScore)
                {
                    bestScore = score;
                    best = &corelProfile;
                }
            }

            return best && !best->name.empty()
                ? best->name
                : profile.displayName;
        }

        struct MappingPreviewCacheEntry
        {
            UI::ColorTargetPreview preview;
        };

        struct PendingColorPreviewRequest
        {
            UI::ColorMapEntry mapping;
            int selectedRgb = -1;
            UI::RenderingIntent intent =
                UI::RenderingIntent::Perceptual;
            bool blackFloorEnabled = true;
            int blackFloorRgb = 28;
            std::string cacheKey;
        };

        [[nodiscard]] static Color::RenderingIntent ToColorRenderingIntent(
            UI::RenderingIntent intent) noexcept
        {
            switch (intent)
            {
            case UI::RenderingIntent::RelativeColorimetric:
                return Color::RenderingIntent::RelativeColorimetric;
            case UI::RenderingIntent::Saturation:
                return Color::RenderingIntent::Saturation;
            case UI::RenderingIntent::AbsoluteColorimetric:
                return Color::RenderingIntent::AbsoluteColorimetric;
            case UI::RenderingIntent::Perceptual:
            default:
                return Color::RenderingIntent::Perceptual;
            }
        }

        [[nodiscard]] Color::ProfileInfo ResolveColorRgbProfile(
            const IVGColorPtr& color) const
        {
            if (!color)
                throw std::runtime_error("Corel color is not available.");

            auto context = color->ColorContext;

            if (!context)
                throw std::runtime_error("Corel color has no color context.");

            auto corelProfile = context->RGBColorProfile;

            if (!corelProfile)
                throw std::runtime_error("Corel color context has no RGB profile.");

            CorelProfileRecord record;
            record.name =
                BstrToWide(
                    _bstr_t(corelProfile->Name));
            record.fileName =
                BstrToWide(
                    _bstr_t(corelProfile->FileName));

            int bestScore = -1;
            const Color::ProfileInfo* best = nullptr;

            for (const auto& profile : rgbProfiles_)
            {
                const int score =
                    ProfileMatchScore(
                        profile,
                        record);

                if (score < 0)
                    continue;

                if (bestScore < 0 || score < bestScore)
                {
                    bestScore = score;
                    best = &profile;
                }
            }

            if (best)
                return *best;

            if (!record.fileName.empty())
            {
                try
                {
                    auto profile =
                        Color::ProfileCatalog::Inspect(
                            std::filesystem::path(
                                record.fileName));

                    if (!record.name.empty())
                        profile.displayName = record.name;

                    return profile;
                }
                catch (...)
                {
                }
            }

            const auto windowsProfiles =
                Color::ProfileCatalog::EnumerateRGB();

            bestScore = -1;
            best = nullptr;

            for (const auto& profile : windowsProfiles)
            {
                const int score =
                    ProfileMatchScore(
                        profile,
                        record);

                if (score < 0)
                    continue;

                if (bestScore < 0 || score < bestScore)
                {
                    bestScore = score;
                    best = &profile;
                }
            }

            if (best)
                return *best;

            throw std::runtime_error(
                "Unable to resolve the RGB ICC profile used by CorelDRAW for the color preview.");
        }

        Color::RgbToSrgbConverter& PreviewSrgbConverter(
            const Color::ProfileInfo& sourceProfile,
            UI::RenderingIntent intent)
        {
            const std::string key =
                std::to_string(sourceProfile.fingerprint) +
                ":" +
                std::to_string(
                    static_cast<int>(intent));

            const auto found =
                previewSrgbConverters_.find(key);

            if (found != previewSrgbConverters_.end() &&
                found->second &&
                found->second->Ready())
            {
                return *found->second;
            }

            auto converter =
                std::make_unique<Color::RgbToSrgbConverter>();

            converter->Initialize(
                sourceProfile,
                ToColorRenderingIntent(intent));

            auto* result = converter.get();
            previewSrgbConverters_[key] =
                std::move(converter);

            return *result;
        }

        void RefreshPreviewContext()
        {
            const auto now = GetTickCount64();
            if (now - lastPreviewContextProbe_ < SelectionProbeIntervalMs) return;
            lastPreviewContextProbe_ = now;
            std::string key;
            try
            {
                auto doc = app_->ActiveDocument;
                if (doc)
                {
                    auto context = doc->ColorContext;
                    auto rgb = context->RGBColorProfile;
                    auto cmyk = context->CMYKColorProfile;
                    if (rgb && cmyk)
                        key = std::to_string(reinterpret_cast<std::uintptr_t>(doc.GetInterfacePtr())) + "|" +
                            WideToUtf8(static_cast<const wchar_t*>(rgb->ID)) + "|" +
                            WideToUtf8(static_cast<const wchar_t*>(cmyk->ID)) + "|" +
                            std::to_string(context->RenderingIntent) + "|" + std::to_string(context->BlendingColorModel);
                }
            }
            catch (...) {}
            if (key != previewContextKey_)
            {
                previewContextKey_ = std::move(key);
                colorTargetPreviewCache_.clear();
                pendingColorPreviewRequests_.clear();
            }
        }

        [[nodiscard]] std::string SpotPreviewCacheKey(
            const UI::ColorMapEntry& mapping,
            UI::RenderingIntent intent)
        {
            return
                previewContextKey_ + "|SPOT|" +
                std::string(mapping.targetSpotPalette.data()) +
                "|" +
                std::string(mapping.targetSpotName.data()) +
                "|" +
                std::to_string(
                    (std::clamp)(
                        mapping.targetSpotTint,
                        0,
                        100)) +
                "|" +
                std::to_string(
                    static_cast<int>(intent));
        }

        [[nodiscard]] static std::string RgbPreviewCacheKey(
            const UI::ColorMapEntry& mapping,
            const UI::ColorState& colorState,
            std::uint64_t profileFingerprint)
        {
            return
                std::string("RGB|") +
                std::to_string(profileFingerprint) +
                "|" +
                std::to_string(
                    static_cast<int>(
                        colorState.intent)) +
                "|" +
                ((colorState.blackFloorEnabled && colorState.conversionMode == UI::ColorConversionMode::StandardIcc) ? "1" : "0") +
                "|" +
                std::to_string(
                    (std::clamp)(
                        colorState.blackFloorRgb,
                        0,
                        255)) +
                "|" +
                std::to_string(
                    (std::clamp)(
                        mapping.targetRgb[0],
                        0,
                        255)) +
                "|" +
                std::to_string(
                    (std::clamp)(
                        mapping.targetRgb[1],
                        0,
                        255)) +
                "|" +
                std::to_string(
                    (std::clamp)(
                        mapping.targetRgb[2],
                        0,
                        255));
        }

        [[nodiscard]] UI::ColorTargetPreview GetColorMappingTargetPreview(
            const UI::ColorMapEntry& mapping,
            const UI::ColorState& colorState)
        {
            UI::ColorTargetPreview result;
            std::string cacheKey;

            if (mapping.targetKind ==
                UI::ColorMapTargetKind::RGB)
            {
                if (colorState.selectedRgb < 0 ||
                    colorState.selectedRgb >=
                        static_cast<int>(
                            rgbProfiles_.size()))
                {
                    result.message =
                        "Select an RGB profile to calculate the preview.";
                    return result;
                }

                const auto& sourceProfile =
                    rgbProfiles_[
                        static_cast<std::size_t>(
                            colorState.selectedRgb)];

                cacheKey =
                    RgbPreviewCacheKey(
                        mapping,
                        colorState,
                        sourceProfile.fingerprint);
            }
            else
            {
                if (!app_)
                {
                    result.message =
                        "CorelDRAW is not available.";
                    return result;
                }

                const char* palette =
                    mapping.targetSpotPalette.data();
                const char* name =
                    mapping.targetSpotName.data();

                if (!palette || !*palette ||
                    !name || !*name)
                {
                    result.message =
                        "Enter the exact Spot name and palette ID to preview it.";
                    return result;
                }

                if (previewContextKey_.empty())
                {
                    result.message = "Aguardando documento...";
                    return result;
                }
                cacheKey =
                    SpotPreviewCacheKey(
                        mapping,
                        colorState.intent);
            }

            const auto cached =
                colorTargetPreviewCache_.find(
                    cacheKey);

            if (cached !=
                colorTargetPreviewCache_.end())
            {
                return cached->second.preview;
            }

            const ULONGLONG now =
                GetTickCount64();

            if (now - lastUiActivity_ <
                UiActivityHoldMs)
            {
                result.message =
                    "Preview pending...";
                return result;
            }

            const bool alreadyQueued =
                std::any_of(
                    pendingColorPreviewRequests_.begin(),
                    pendingColorPreviewRequests_.end(),
                    [&](const PendingColorPreviewRequest& request)
                    {
                        return request.cacheKey ==
                            cacheKey;
                    });

            if (!alreadyQueued)
            {
                if (pendingColorPreviewRequests_.size() >= 64)
                {
                    pendingColorPreviewRequests_.erase(
                        pendingColorPreviewRequests_.begin(),
                        pendingColorPreviewRequests_.begin() + 16);
                }

                PendingColorPreviewRequest request;
                request.mapping = mapping;
                request.selectedRgb =
                    colorState.selectedRgb;
                request.intent =
                    colorState.intent;
                request.blackFloorEnabled =
                    (colorState.blackFloorEnabled && colorState.conversionMode == UI::ColorConversionMode::StandardIcc);
                request.blackFloorRgb =
                    colorState.blackFloorRgb;
                request.cacheKey =
                    std::move(cacheKey);

                pendingColorPreviewRequests_.push_back(
                    std::move(request));
            }

            result.message =
                "Preview pending...";
            return result;
        }

        bool ProcessOnePendingColorPreview()
        {
            if (state_.page != UI::Page::Color)
            {
                pendingColorPreviewRequests_.clear();
                return false;
            }

            if (GetForegroundWindow() != hwnd_ ||
                pendingColorPreviewRequests_.empty())
            {
                return false;
            }

            const ULONGLONG now =
                GetTickCount64();

            if (now - lastUiActivity_ <
                PreviewComputeIdleDelayMs)
            {
                return false;
            }

            PendingColorPreviewRequest request =
                std::move(
                    pendingColorPreviewRequests_.front());

            pendingColorPreviewRequests_.erase(
                pendingColorPreviewRequests_.begin());

            if (colorTargetPreviewCache_.find(
                    request.cacheKey) !=
                colorTargetPreviewCache_.end())
            {
                return true;
            }

            UI::ColorState previewState;
            previewState.selectedRgb =
                request.selectedRgb;
            previewState.intent =
                request.intent;
            previewState.blackFloorEnabled =
                request.blackFloorEnabled;
            previewState.blackFloorRgb =
                request.blackFloorRgb;

            (void)ComputeColorMappingTargetPreview(
                request.mapping,
                previewState);

            return true;
        }

        [[nodiscard]] UI::ColorTargetPreview ComputeColorMappingTargetPreview(
            const UI::ColorMapEntry& mapping,
            const UI::ColorState& colorState)
        {
            UI::ColorTargetPreview result;

            if (!app_)
            {
                result.message = "CorelDRAW is not available.";
                return result;
            }

            try
            {
                Color::Rgb8 profileRgb{};
                Color::ProfileInfo sourceProfile;
                std::string cacheKey;

                if (mapping.targetKind ==
                    UI::ColorMapTargetKind::RGB)
                {
                    if (colorState.selectedRgb < 0 ||
                        colorState.selectedRgb >=
                            static_cast<int>(
                                rgbProfiles_.size()))
                    {
                        result.message =
                            "Select an RGB profile to calculate the preview.";
                        return result;
                    }

                    sourceProfile =
                        rgbProfiles_[
                            static_cast<std::size_t>(
                                colorState.selectedRgb)];

                    cacheKey =
                        RgbPreviewCacheKey(
                            mapping,
                            colorState,
                            sourceProfile.fingerprint);

                    const auto cached =
                        colorTargetPreviewCache_.find(
                            cacheKey);

                    if (cached !=
                        colorTargetPreviewCache_.end())
                    {
                        return cached->second.preview;
                    }

                    int targetR =
                        (std::clamp)(
                            mapping.targetRgb[0],
                            0,
                            255);
                    int targetG =
                        (std::clamp)(
                            mapping.targetRgb[1],
                            0,
                            255);
                    int targetB =
                        (std::clamp)(
                            mapping.targetRgb[2],
                            0,
                            255);

                    if ((colorState.blackFloorEnabled && colorState.conversionMode == UI::ColorConversionMode::StandardIcc))
                    {
                        const int floor =
                            (std::clamp)(
                                colorState.blackFloorRgb,
                                0,
                                255);

                        if ((std::max)({
                            targetR,
                            targetG,
                            targetB }) < floor)
                        {
                            targetR = floor;
                            targetG = floor;
                            targetB = floor;
                        }
                    }

                    profileRgb =
                    {
                        static_cast<std::uint8_t>(targetR),
                        static_cast<std::uint8_t>(targetG),
                        static_cast<std::uint8_t>(targetB)
                    };
                }
                else
                {
                    const std::wstring palette =
                        Utf8ToWide(
                            std::string(
                                mapping.targetSpotPalette.data()));

                    const std::wstring name =
                        Utf8ToWide(
                            std::string(
                                mapping.targetSpotName.data()));

                    if (palette.empty() || name.empty())
                    {
                        result.message =
                            "Enter the exact Spot name and palette ID to preview it.";
                        return result;
                    }

                    cacheKey =
                        SpotPreviewCacheKey(
                            mapping,
                            colorState.intent);

                    const auto cached =
                        colorTargetPreviewCache_.find(
                            cacheKey);

                    if (cached !=
                        colorTargetPreviewCache_.end())
                    {
                        return cached->second.preview;
                    }

                    auto color =
                        app_->CreateSpotColorByName(
                            _bstr_t(palette.c_str()),
                            _bstr_t(name.c_str()),
                            (std::clamp)(
                                mapping.targetSpotTint,
                                0,
                                100));

                    if (!color)
                    {
                        result.message =
                            "CorelDRAW could not create the requested Spot color.";
                        return result;
                    }

                    color->ConvertToRGB();

                    profileRgb =
                    {
                        static_cast<std::uint8_t>(
                            (std::clamp)(
                                static_cast<int>(color->RGBRed),
                                0,
                                255)),
                        static_cast<std::uint8_t>(
                            (std::clamp)(
                                static_cast<int>(color->RGBGreen),
                                0,
                                255)),
                        static_cast<std::uint8_t>(
                            (std::clamp)(
                                static_cast<int>(color->RGBBlue),
                                0,
                                255))
                    };

                    sourceProfile =
                        ResolveColorRgbProfile(color);
                }

                auto& converter =
                    PreviewSrgbConverter(
                        sourceProfile,
                        colorState.intent);

                const Color::Rgb8 srgb =
                    converter.Convert(profileRgb);

                result.valid = true;
                result.srgb =
                {
                    static_cast<int>(srgb.r),
                    static_cast<int>(srgb.g),
                    static_cast<int>(srgb.b)
                };
                result.sourceProfile =
                    WideToUtf8(
                        sourceProfile.displayName);

                if (mapping.targetKind ==
                    UI::ColorMapTargetKind::Spot)
                {
                    result.message =
                        "Corel ConvertToRGB: " +
                        std::to_string(profileRgb.r) +
                        ", " +
                        std::to_string(profileRgb.g) +
                        ", " +
                        std::to_string(profileRgb.b) +
                        "  |  " +
                        result.sourceProfile +
                        " -> sRGB";
                }
                else
                {
                    result.message =
                        result.sourceProfile +
                        " -> sRGB";
                }

                if (!cacheKey.empty())
                {
                    if (colorTargetPreviewCache_.size() > 512)
                    {
                        colorTargetPreviewCache_.clear();
                        pendingColorPreviewRequests_.clear();
                    }

                    colorTargetPreviewCache_[
                        std::move(cacheKey)] =
                    {
                        result
                    };
                }

                return result;
            }
            catch (const _com_error& error)
            {
                result.message =
                    WideToUtf8(
                        ComErrorText(error));
            }
            catch (const std::exception& error)
            {
                result.message = error.what();
            }
            catch (...)
            {
                result.message =
                    "Unable to calculate the destination color preview.";
            }

            return result;
        }

        [[nodiscard]] static std::uint64_t SelectedFingerprint(
            const std::vector<Color::ProfileInfo>& profiles,
            int index) noexcept
        {
            if (index < 0 ||
                index >=
                static_cast<int>(
                    profiles.size()))
            {
                return 0;
            }

            return profiles[
                static_cast<std::size_t>(
                    index)].fingerprint;
        }

        [[nodiscard]] static int FindProfileIndex(
            const std::vector<Color::ProfileInfo>& profiles,
            std::uint64_t fingerprint) noexcept
        {
            if (fingerprint == 0)
                return -1;

            for (std::size_t i = 0;
                i < profiles.size();
                ++i)
            {
                if (profiles[i].fingerprint ==
                    fingerprint)
                {
                    return static_cast<int>(i);
                }
            }

            return -1;
        }

        [[nodiscard]] static std::wstring SpotAppearanceKey(
            const IVGColorPtr& color)
        {
            if (!color)
                return {};

            try
            {
                const std::wstring palette =
                    BstrToWide(
                        _bstr_t(
                            color->PaletteIdentifier));

                const std::wstring name =
                    BstrToWide(
                        _bstr_t(
                            color->SpotColorName));

                const int tint =
                    (std::clamp)(
                        static_cast<int>(
                            color->Tint),
                        0,
                        100);

                long spotId = 0;
                try
                {
                    spotId = color->SpotColorID;
                }
                catch (...)
                {
                    spotId = 0;
                }

                if (name.empty())
                    return {};

                return
                    palette +
                    L"\x1F" +
                    name +
                    L"\x1F" +
                    std::to_wstring(spotId) +
                    L"\x1F" +
                    std::to_wstring(tint);
            }
            catch (...)
            {
                return {};
            }
        }

        bool CaptureSpotAppearance(
            const IVGColorPtr& color,
            SpotAppearanceCache& cache) const
        {
            if (!color ||
                !IsSpotColor(color))
            {
                return false;
            }

            const std::wstring key =
                SpotAppearanceKey(color);

            if (key.empty())
                return false;

            if (cache.find(key) !=
                cache.end())
            {
                return true;
            }

            const auto extractLab =
                [](IVGColorPtr working,
                    Color::Lab& lab) -> bool
                {
                    if (!working)
                        return false;

                    try
                    {
                        if (working->Type !=
                            cdrColorLab)
                        {
                            working->ConvertToLab();
                        }

                        if (working->Type !=
                            cdrColorLab)
                        {
                            return false;
                        }

                        lab =
                        {
                            static_cast<double>(
                                working->LabLuminance) *
                                (100.0 / 255.0),
                            static_cast<double>(
                                working->LabComponentA),
                            static_cast<double>(
                                working->LabComponentB)
                        };

                        return
                            std::isfinite(lab.l) &&
                            std::isfinite(lab.a) &&
                            std::isfinite(lab.b);
                    }
                    catch (...)
                    {
                        return false;
                    }
                };

            Color::Lab lab;

            try
            {
                auto copy =
                    color->GetCopy();

                if (extractLab(
                    copy,
                    lab))
                {
                    cache.emplace(
                        key,
                        lab);
                    return true;
                }
            }
            catch (...)
            {
            }

            try
            {
                const std::wstring palette =
                    BstrToWide(
                        _bstr_t(
                            color->PaletteIdentifier));
                const std::wstring name =
                    BstrToWide(
                        _bstr_t(
                            color->SpotColorName));
                const int tint =
                    (std::clamp)(
                        static_cast<int>(
                            color->Tint),
                        0,
                        100);

                if (app_ &&
                    !palette.empty() &&
                    !name.empty())
                {
                    auto paletteSpot =
                        app_->CreateSpotColorByName(
                            _bstr_t(palette.c_str()),
                            _bstr_t(name.c_str()),
                            tint);

                    if (extractLab(
                        paletteSpot,
                        lab))
                    {
                        cache.emplace(
                            key,
                            lab);
                        return true;
                    }
                }
            }
            catch (...)
            {
            }

            try
            {
                auto copy =
                    color->GetCopy();

                if (copy)
                {
                    copy->ConvertToRGB();

                    if (extractLab(
                        copy,
                        lab))
                    {
                        cache.emplace(
                            key,
                            lab);
                        return true;
                    }
                }
            }
            catch (...)
            {
            }

            return false;
        }

        void CaptureSpotAppearanceRecursive(
            const IVGShapePtr& shape,
            SpotAppearanceCache& cache) const
        {
            Cancellation::ThrowIfRequested();

            if (!shape)
                return;

            try
            {
                if (shape->Type ==
                    cdrGroupShape)
                {
                    auto children =
                        shape->Shapes;

                    if (children)
                    {
                        for (long i = 1;
                            i <= children->Count;
                            ++i)
                        {
                            CaptureSpotAppearanceRecursive(
                                children->Item[i],
                                cache);
                        }
                    }
                }
            }
            catch (const OperationCancelled&) { throw; }
            catch (...)
            {
            }

            try
            {
                auto powerClip =
                    shape->PowerClip;

                if (powerClip)
                {
                    auto contents =
                        powerClip->Shapes;

                    if (contents)
                    {
                        for (long i = 1;
                            i <= contents->Count;
                            ++i)
                        {
                            CaptureSpotAppearanceRecursive(
                                contents->Item[i],
                                cache);
                        }
                    }
                }
            }
            catch (const OperationCancelled&) { throw; }
            catch (...)
            {
            }

            try
            {
                auto fill =
                    shape->Fill;

                if (fill)
                {
                    if (fill->Type ==
                        cdrUniformFill)
                    {
                        (void)CaptureSpotAppearance(
                            fill->UniformColor,
                            cache);
                    }
                    else if (
                        fill->Type ==
                        cdrFountainFill)
                    {
                        auto fountain =
                            fill->Fountain;

                        if (fountain &&
                            fountain->Colors)
                        {
                            auto colors =
                                fountain->Colors;

                            const long last =
                                colors->Count + 1;

                            for (long i = 0;
                                i <= last;
                                ++i)
                            {
                                auto fountainColor =
                                    colors->Item[i];

                                if (fountainColor)
                                {
                                    (void)CaptureSpotAppearance(
                                        fountainColor->Color,
                                        cache);
                                }
                            }
                        }
                    }
                }
            }
            catch (const OperationCancelled&) { throw; }
            catch (...)
            {
            }

            try
            {
                auto outline =
                    shape->Outline;

                if (outline &&
                    outline->Type ==
                    cdrOutline)
                {
                    (void)CaptureSpotAppearance(
                        outline->Color,
                        cache);
                }
            }
            catch (const OperationCancelled&) { throw; }
            catch (...)
            {
            }
        }

        long ConvertShapeRecursive(
            const IVGShapePtr& shape,
            const UI::ColorState& request,
            ColorConversionStats& stats,
            const SpotAppearanceCache* spotAppearanceCache)
        {
            Cancellation::ThrowIfRequested();

            if (!shape)
                return 0;

            long converted = 0;

            try
            {
                if (shape->Type ==
                    cdrGroupShape)
                {
                    auto children =
                        shape->Shapes;

                    if (children)
                    {
                        for (long i = 1;
                            i <= children->Count;
                            ++i)
                        {
                            converted +=
                                ConvertShapeRecursive(
                                    children->Item[i],
                                    request,
                                    stats,
                                    spotAppearanceCache);
                        }
                    }
                }
            }
            catch (const Color::OperationCancelled&) { throw; }
            catch (...)
            {
                ++stats.errors;
            }

            try
            {
                auto powerClip =
                    shape->PowerClip;

                if (powerClip)
                {
                    auto contents =
                        powerClip->Shapes;

                    if (contents)
                    {
                        for (long i = 1;
                            i <= contents->Count;
                            ++i)
                        {
                            converted +=
                                ConvertShapeRecursive(
                                    contents->Item[i],
                                    request,
                                    stats,
                                    spotAppearanceCache);
                        }
                    }
                }
            }
            catch (const Color::OperationCancelled&) { throw; }
            catch (...)
            {
                ++stats.errors;
            }

            try
            {
                auto fill =
                    shape->Fill;

                if (fill)
                {
                    if (fill->Type ==
                        cdrUniformFill)
                    {
                        converted +=
                            ConvertCorelColor(
                                fill->UniformColor,
                                request,
                                stats,
                                spotAppearanceCache);
                    }
                    else if (
                        fill->Type ==
                        cdrFountainFill)
                    {
                        auto fountain =
                            fill->Fountain;

                        if (fountain &&
                            fountain->Colors)
                        {
                            auto colors =
                                fountain->Colors;

                            const long last =
                                colors->Count + 1;

                            for (long i = 0;
                                i <= last;
                                ++i)
                            {
                                auto fountainColor =
                                    colors->Item[i];

                                if (fountainColor)
                                {
                                    converted +=
                                        ConvertCorelColor(
                                            fountainColor->Color,
                                            request,
                                            stats,
                                            spotAppearanceCache);
                                }
                            }
                        }
                    }
                }
            }
            catch (const Color::OperationCancelled&) { throw; }
            catch (...)
            {
                ++stats.errors;
            }

            try
            {
                auto outline =
                    shape->Outline;

                if (outline &&
                    outline->Type ==
                    cdrOutline)
                {
                    converted +=
                        ConvertCorelColor(
                            outline->Color,
                            request,
                            stats,
                            spotAppearanceCache);
                }
            }
            catch (const Color::OperationCancelled&) { throw; }
            catch (...)
            {
                ++stats.errors;
            }

            return converted;
        }

        [[nodiscard]] static bool EqualInsensitive(
            const std::wstring& a,
            const std::wstring& b) noexcept
        {
            return _wcsicmp(
                a.c_str(),
                b.c_str()) == 0;
        }

        struct SpotNameKey
        {
            std::wstring normalized;
            std::wstring base;
            std::wstring words;
            std::wstring coating;
            std::vector<std::wstring> numbers;
        };

        [[nodiscard]] static bool EndsWithText(
            const std::wstring& value,
            const wchar_t* suffix) noexcept
        {
            if (!suffix)
                return false;

            const std::size_t suffixLength =
                std::wcslen(suffix);

            return value.size() >= suffixLength &&
                value.compare(
                    value.size() - suffixLength,
                    suffixLength,
                    suffix) == 0;
        }

        static void RemoveAllText(
            std::wstring& value,
            const wchar_t* needle)
        {
            if (!needle || !*needle)
                return;

            const std::size_t needleLength =
                std::wcslen(needle);

            std::size_t position = 0;
            while ((position = value.find(
                needle,
                position)) != std::wstring::npos)
            {
                value.erase(position, needleLength);
            }
        }

        [[nodiscard]] static std::wstring CanonicalSpotNumber(
            const std::wstring& value)
        {
            if (value.empty())
                return {};

            std::size_t firstNonZero = 0;
            while (firstNonZero + 1 < value.size() &&
                value[firstNonZero] == L'0')
            {
                ++firstNonZero;
            }

            return value.substr(firstNonZero);
        }

        [[nodiscard]] static SpotNameKey BuildSpotNameKey(
            const std::wstring& value)
        {
            SpotNameKey key;
            key.normalized = NormalizeLookupText(value);

            if (key.normalized.empty())
                return key;

            std::wstring semantic = key.normalized;

            if (EndsWithText(semantic, L"UNCOATED"))
                key.coating = L"U";
            else if (EndsWithText(semantic, L"COATED"))
                key.coating = L"C";

            static constexpr const wchar_t* noiseWords[]
            {
                L"FORMULAGUIDE",
                L"SOLIDUNCOATED",
                L"SOLIDCOATED",
                L"PANTONEPLUS",
                L"PANTONE",
                L"UNCOATED",
                L"COATED",
                L"SOLID",
                L"PALETTE",
                L"COLOUR",
                L"COLOR",
                L"SPOT",
                L"PLUS",
                L"PMS"
            };

            for (const wchar_t* noise : noiseWords)
                RemoveAllText(semantic, noise);

            if (key.coating.empty())
            {
                if (EndsWithText(semantic, L"CVC"))
                {
                    key.coating = L"C";
                    semantic.resize(semantic.size() - 3);
                }
                else if (EndsWithText(semantic, L"CVU"))
                {
                    key.coating = L"U";
                    semantic.resize(semantic.size() - 3);
                }
                else if (EndsWithText(semantic, L"CP") &&
                    semantic.size() > 2)
                {
                    key.coating = L"CP";
                    semantic.resize(semantic.size() - 2);
                }
                else if (EndsWithText(semantic, L"UP") &&
                    semantic.size() > 2)
                {
                    key.coating = L"UP";
                    semantic.resize(semantic.size() - 2);
                }
                else if (EndsWithText(semantic, L"C") &&
                    semantic.size() > 1)
                {
                    key.coating = L"C";
                    semantic.resize(semantic.size() - 1);
                }
                else if (EndsWithText(semantic, L"U") &&
                    semantic.size() > 1)
                {
                    key.coating = L"U";
                    semantic.resize(semantic.size() - 1);
                }
            }
            else
            {
                RemoveAllText(semantic, L"UNCOATED");
                RemoveAllText(semantic, L"COATED");
            }

            key.base = semantic;
            key.words.reserve(semantic.size());

            std::size_t i = 0;
            while (i < semantic.size())
            {
                if (!std::iswdigit(semantic[i]))
                {
                    key.words.push_back(semantic[i]);
                    ++i;
                    continue;
                }

                const std::size_t start = i;
                while (i < semantic.size() &&
                    std::iswdigit(semantic[i]))
                {
                    ++i;
                }

                key.numbers.push_back(
                    CanonicalSpotNumber(
                        semantic.substr(
                            start,
                            i - start)));
            }

            return key;
        }

        [[nodiscard]] static bool SameSpotNumbers(
            const std::vector<std::wstring>& actual,
            const std::vector<std::wstring>& expected) noexcept
        {
            if (actual.size() != expected.size())
                return false;

            for (std::size_t i = 0; i < actual.size(); ++i)
            {
                if (actual[i] != expected[i])
                    return false;
            }

            return true;
        }

        [[nodiscard]] static bool LooseIdentifierEquivalent(
            const std::wstring& actual,
            const std::wstring& expected)
        {
            if (expected.empty())
                return true;

            const std::wstring normalizedActual =
                NormalizeLookupText(actual);

            const std::wstring normalizedExpected =
                NormalizeLookupText(expected);

            if (normalizedActual.empty() ||
                normalizedExpected.empty())
            {
                return false;
            }

            if (normalizedActual == normalizedExpected)
                return true;

            const std::size_t shorter =
                (std::min)(
                    normalizedActual.size(),
                    normalizedExpected.size());

            if (shorter < 4)
                return false;

            return normalizedActual.find(normalizedExpected) !=
                    std::wstring::npos ||
                normalizedExpected.find(normalizedActual) !=
                    std::wstring::npos;
        }

        [[nodiscard]] static bool SpotNamesEquivalent(
            const std::wstring& actual,
            const std::wstring& expected)
        {
            if (actual.empty() ||
                expected.empty())
            {
                return false;
            }

            if (EqualInsensitive(actual, expected))
                return true;

            const SpotNameKey actualKey =
                BuildSpotNameKey(actual);

            const SpotNameKey expectedKey =
                BuildSpotNameKey(expected);

            if (actualKey.normalized.empty() ||
                expectedKey.normalized.empty() ||
                actualKey.base.empty() ||
                expectedKey.base.empty())
            {
                return false;
            }

            if (actualKey.normalized ==
                expectedKey.normalized)
            {
                return true;
            }

            if (!expectedKey.coating.empty())
            {
                if (actualKey.coating.empty() ||
                    actualKey.coating != expectedKey.coating)
                {
                    return false;
                }
            }

            if (!expectedKey.numbers.empty())
            {
                if (!SameSpotNumbers(
                    actualKey.numbers,
                    expectedKey.numbers))
                {
                    return false;
                }
            }
            else if (!actualKey.numbers.empty())
            {
                return false;
            }

            if (expectedKey.words.empty())
            {
                return !expectedKey.numbers.empty();
            }

            if (actualKey.words.empty())
                return false;

            if (actualKey.words == expectedKey.words)
                return true;

            if (expectedKey.words.size() >= 3 &&
                actualKey.words.find(expectedKey.words) !=
                    std::wstring::npos)
            {
                return true;
            }

            return false;
        }

        [[nodiscard]] static std::wstring TrimSpotAlias(
            const std::wstring& value)
        {
            std::size_t first = 0;
            while (first < value.size() &&
                std::iswspace(value[first]))
            {
                ++first;
            }

            std::size_t last = value.size();
            while (last > first &&
                std::iswspace(value[last - 1]))
            {
                --last;
            }

            return value.substr(first, last - first);
        }

        [[nodiscard]] static std::vector<std::wstring> SplitSpotAliases(
            const std::wstring& value)
        {
            std::vector<std::wstring> aliases;
            std::wstring current;
            current.reserve(value.size());

            const auto flush = [&aliases, &current]()
            {
                std::wstring alias =
                    TrimSpotAlias(current);

                if (!alias.empty())
                    aliases.push_back(std::move(alias));

                current.clear();
            };

            for (const wchar_t ch : value)
            {
                if (ch == L',' ||
                    ch == L';' ||
                    ch == L'|' ||
                    ch == L'\n' ||
                    ch == L'\r')
                {
                    flush();
                    continue;
                }

                current.push_back(ch);
            }

            flush();
            return aliases;
        }

        [[nodiscard]] static bool SpotNamesEquivalentAny(
            const std::wstring& actual,
            const std::wstring& aliasesText)
        {
            if (actual.empty() || aliasesText.empty())
                return false;

            const auto aliases =
                SplitSpotAliases(aliasesText);

            for (const auto& alias : aliases)
            {
                if (SpotNamesEquivalent(actual, alias))
                    return true;
            }

            return false;
        }

        [[nodiscard]] static std::wstring BstrToWide(
            const _bstr_t& value)
        {
            const wchar_t* text =
                static_cast<const wchar_t*>(value);

            return text
                ? std::wstring(text)
                : std::wstring();
        }

        [[nodiscard]] bool IsSpotColor(
            const IVGColorPtr& color) const noexcept
        {
            if (!color)
                return false;

            try
            {
                return color->IsSpot !=
                    VARIANT_FALSE;
            }
            catch (...)
            {
                return false;
            }
        }

        [[nodiscard]] std::array<int, 3> CorelRgbOf(
            const IVGColorPtr& color) const
        {
            if (!color)
                return { 0, 0, 0 };

            auto copy =
                app_->CreateRGBColor(
                    0,
                    0,
                    0);

            copy->CopyAssign(color);
            copy->ConvertToRGB();

            return
            {
                static_cast<int>(copy->RGBRed),
                static_cast<int>(copy->RGBGreen),
                static_cast<int>(copy->RGBBlue)
            };
        }

        [[nodiscard]] bool MatchesColorMapping(
            const IVGColorPtr& color,
            const UI::ColorMapEntry& mapping,
            int rgbTolerance) const
        {
            if (!color ||
                !mapping.enabled)
            {
                return false;
            }

            switch (mapping.sourceKind)
            {
            case UI::ColorMapSourceKind::RGB:
            {
                if (color->Type != cdrColorRGB || IsSpotColor(color))
                    return false;

                const auto rgb =
                    CorelRgbOf(color);

                const int tolerance =
                    (std::clamp)(
                        rgbTolerance,
                        0,
                        255);

                for (int channel = 0; channel < 3; ++channel)
                {
                    if (std::abs(
                        rgb[static_cast<std::size_t>(channel)] -
                        (std::clamp)(
                            mapping.sourceChannels[
                                static_cast<std::size_t>(channel)],
                            0,
                            255)) > tolerance)
                    {
                        return false;
                    }
                }

                return true;
            }

            case UI::ColorMapSourceKind::CMYK:
                if (color->Type !=
                    cdrColorCMYK)
                {
                    return false;
                }

                return
                    color->CMYKCyan ==
                        (std::clamp)(mapping.sourceChannels[0], 0, 100) &&
                    color->CMYKMagenta ==
                        (std::clamp)(mapping.sourceChannels[1], 0, 100) &&
                    color->CMYKYellow ==
                        (std::clamp)(mapping.sourceChannels[2], 0, 100) &&
                    color->CMYKBlack ==
                        (std::clamp)(mapping.sourceChannels[3], 0, 100);

            case UI::ColorMapSourceKind::Spot:
            {
                if (!IsSpotColor(color))
                    return false;

                const std::wstring expectedName =
                    Utf8ToWide(
                        std::string(
                            mapping.sourceSpotName.data()));

                if (expectedName.empty())
                    return false;

                const std::wstring actualName =
                    BstrToWide(
                        _bstr_t(
                            color->SpotColorName));

                if (!SpotNamesEquivalentAny(
                    actualName,
                    expectedName))
                {
                    return false;
                }

                const std::wstring expectedPalette =
                    Utf8ToWide(
                        std::string(
                            mapping.sourceSpotPalette.data()));

                if (!expectedPalette.empty())
                {
                    const std::wstring actualPalette =
                        BstrToWide(
                            _bstr_t(
                                color->PaletteIdentifier));

                    if (!LooseIdentifierEquivalent(
                        actualPalette,
                        expectedPalette))
                    {
                        return false;
                    }
                }

                return
                    static_cast<int>(color->Tint) ==
                    (std::clamp)(
                        mapping.sourceSpotTint,
                        0,
                        100);
            }

            default:
                return false;
            }
        }

        bool ApplySpotTintWhite(
            const IVGColorPtr& color,
            const UI::ColorState& request) const
        {
            if (!color ||
                !request.spotTintWhiteEnabled ||
                !IsSpotColor(color))
            {
                return false;
            }

            const int threshold =
                (std::clamp)(
                    request.spotTintWhiteThreshold,
                    0,
                    100);

            const int tint =
                (std::clamp)(
                    static_cast<int>(
                        color->Tint),
                    0,
                    100);

            if (tint > threshold)
                return false;

            color->RGBAssign(
                255,
                255,
                255);

            return true;
        }

        bool ApplyBlackFloorToCorelRgb(
            const IVGColorPtr& color,
            const UI::ColorState& request) const
        {
            if (!color ||
                request.conversionMode == UI::ColorConversionMode::PreserveAppearance ||
                !request.blackFloorEnabled ||
                color->Type !=
                cdrColorRGB)
            {
                return false;
            }

            const int floor =
                (std::clamp)(
                    request.blackFloorRgb,
                    0,
                    255);

            const int r =
                static_cast<int>(
                    color->RGBRed);

            const int g =
                static_cast<int>(
                    color->RGBGreen);

            const int b =
                static_cast<int>(
                    color->RGBBlue);

            if ((std::max)({ r, g, b }) >= floor)
                return false;

            color->RGBAssign(
                floor,
                floor,
                floor);

            return true;
        }

        bool ApplyColorMapping(
            const IVGColorPtr& color,
            const UI::ColorMapEntry& mapping,
            const UI::ColorState& request)
        {
            if (!MatchesColorMapping(
                color,
                mapping,
                request.colorMappingTolerance))
            {
                return false;
            }

            if (mapping.targetKind ==
                UI::ColorMapTargetKind::RGB)
            {
                color->RGBAssign(
                    (std::clamp)(
                        mapping.targetRgb[0],
                        0,
                        255),
                    (std::clamp)(
                        mapping.targetRgb[1],
                        0,
                        255),
                    (std::clamp)(
                        mapping.targetRgb[2],
                        0,
                        255));

                ApplyBlackFloorToCorelRgb(
                    color,
                    request);

                return true;
            }

            const std::wstring palette =
                Utf8ToWide(
                    std::string(
                        mapping.targetSpotPalette.data()));

            const std::wstring name =
                Utf8ToWide(
                    std::string(
                        mapping.targetSpotName.data()));

            if (palette.empty() ||
                name.empty())
            {
                return false;
            }

            color->SpotAssignByName(
                _bstr_t(palette.c_str()),
                _bstr_t(name.c_str()),
                (std::clamp)(
                    mapping.targetSpotTint,
                    0,
                    100));

            return true;
        }

        [[nodiscard]] bool IsColorBlacklisted(
            const IVGColorPtr& color,
            const UI::ColorState& request) const
        {
            if (!color ||
                !request.spotBlacklistEnabled ||
                request.spotBlacklistNames[0] == '\0' ||
                !IsSpotColor(color))
            {
                return false;
            }

            try
            {
                const std::wstring actual =
                    BstrToWide(
                        _bstr_t(
                            color->SpotColorName));

                const std::wstring blacklist =
                    Utf8ToWide(
                        std::string(
                            request.spotBlacklistNames.data()));

                return !actual.empty() &&
                    !blacklist.empty() &&
                    SpotNamesEquivalentAny(
                        actual,
                        blacklist);
            }
            catch (const Color::OperationCancelled&) { throw; }
            catch (...)
            {
                return false;
            }
        }

        long ConvertCorelColor(
            const IVGColorPtr& color,
            const UI::ColorState& request,
            ColorConversionStats& stats,
            const SpotAppearanceCache* spotAppearanceCache)
        {
            Cancellation::ThrowIfRequested();

            if (!color)
                return 0;

            ++stats.examined;

            try
            {
                if (IsColorBlacklisted(
                    color,
                    request))
                {
                    ++stats.blacklisted;
                    ++stats.unchanged;
                    return 0;
                }

                if (request.colorMappingEnabled)
                {
                    for (const auto& mapping :
                        request.colorMappings)
                    {
                        if (!mapping.enabled)
                            continue;

                        if (ApplyColorMapping(
                            color,
                            mapping,
                            request))
                        {
                            if (mapping.targetKind ==
                                    UI::ColorMapTargetKind::RGB &&
                                request.blackFloorEnabled &&
                                request.conversionMode != UI::ColorConversionMode::PreserveAppearance)
                            {
                                const int floor =
                                    (std::clamp)(
                                        request.blackFloorRgb,
                                        0,
                                        255);

                                if ((std::max)({
                                    mapping.targetRgb[0],
                                    mapping.targetRgb[1],
                                    mapping.targetRgb[2] }) < floor)
                                {
                                    ++stats.blackFloorAdjusted;
                                }
                            }

                            ++stats.converted;
                            ++stats.mapped;
                            return 1;
                        }
                    }
                }

                if (ApplySpotTintWhite(
                    color,
                    request))
                {
                    ++stats.converted;
                    ++stats.spotTintWhite;
                    return 1;
                }

                if (IsSpotColor(color))
                {
                    if (!request.convertSpotsToRgb)
                    {
                        ++stats.unchanged;
                        return 0;
                    }

                    if (request.preserveSpotAppearance &&
                        spotAppearanceCache)
                    {
                        const std::wstring key =
                            SpotAppearanceKey(color);

                        const auto found =
                            spotAppearanceCache->find(key);

                        if (!key.empty() &&
                            found != spotAppearanceCache->end())
                        {
                            const auto match =
                                colorConverter_.ConvertLabDetailed(
                                    found->second);

                            color->RGBAssign(
                                match.rgb.r,
                                match.rgb.g,
                                match.rgb.b);

                            stats.spotAccuracy.Add(match);
                            ++stats.converted;
                            ++stats.spotAppearancePreserved;
                            return 1;
                        }
                    }

                    color->ConvertToRGB();

                    if (ApplyBlackFloorToCorelRgb(
                        color,
                        request))
                    {
                        ++stats.blackFloorAdjusted;
                    }

                    ++stats.converted;
                    ++stats.spotToRgb;
                    return 1;
                }

                if (color->Type ==
                    cdrColorRGB)
                {
                    if (ApplyBlackFloorToCorelRgb(
                        color,
                        request))
                    {
                        ++stats.converted;
                        ++stats.blackFloorAdjusted;
                        return 1;
                    }

                    ++stats.unchanged;
                    return 0;
                }

                if (color->Type !=
                    cdrColorCMYK)
                {
                    ++stats.unchanged;
                    return 0;
                }

                const Color::Cmyk source
                {
                    static_cast<double>(
                        color->CMYKCyan),
                    static_cast<double>(
                        color->CMYKMagenta),
                    static_cast<double>(
                        color->CMYKYellow),
                    static_cast<double>(
                        color->CMYKBlack)
                };

                if (request.conversionMode ==
                    UI::ColorConversionMode::PreserveAppearance)
                {
                    const auto match =
                        colorConverter_.ConvertDetailed(
                            source);

                    color->RGBAssign(
                        match.rgb.r,
                        match.rgb.g,
                        match.rgb.b);

                    stats.cmykAccuracy.Add(match);
                }
                else
                {
                    const auto rgb =
                        colorConverter_.Convert(
                            source);

                    color->RGBAssign(
                        rgb.r,
                        rgb.g,
                        rgb.b);
                }

                ++stats.converted;
                ++stats.cmykToRgb;
                return 1;
            }
            catch (const Color::OperationCancelled&) { throw; }
            catch (...)
            {
                ++stats.errors;
                return 0;
            }
        }

        template <typename Function>
        HRESULT ExecuteOperation(
            const wchar_t* status,
            Function&& function, bool interactive = true)
        {
            if (!ValidateOperationContext())
                return LastHresult();

            if (operationRunning_)
            {
                return SetError(
                    HRESULT_FROM_WIN32(
                        ERROR_BUSY),
                    L"Another ImCut operation is already running.");
            }

            selectionProbePending_ = true;
            operationRunning_ = true;
            ScopeExit clearBusy([this] { operationRunning_ = false; state_.runtime.busy = false; });
            state_.runtime.busy = true;
            state_.runtime.progress = 0.0f;
            state_.runtime.status =
                WideToUtf8(
                    std::wstring(status));

            if (visible_)
                RenderFrame();

            HRESULT hr = E_FAIL;
            bool cancelled = false;
            Cancellation::OperationScope cancellationScope;

            try
            {
                Cancellation::ThrowIfRequested();
                hr = function();
                Cancellation::ThrowIfRequested();
            }
            catch (const OperationCancelled&)
            {
                cancelled = true;

                try { app_->PutEventsEnabled(VARIANT_TRUE); }
                catch (...) {}
                try { app_->PutOptimization(VARIANT_FALSE); }
                catch (...) {}
                try
                {
                    if (app_->ActiveWindow)
                        app_->ActiveWindow->Refresh();
                    app_->Refresh();
                }
                catch (...) {}

                hr = SetError(
                    HRESULT_FROM_WIN32(ERROR_CANCELLED),
                    L"Operacao cancelada com Esc. O CorelDRAW foi reativado.");
            }
            catch (const _com_error& error)
            {
                hr =
                    SetError(
                        error.Error(),
                        ComErrorText(
                            error));
            }
            catch (const std::exception& error)
            {
                hr =
                    SetError(
                        E_FAIL,
                        Utf8ToWide(
                            error.what()));
            }
            catch (...)
            {
                hr =
                    SetError(
                        E_FAIL,
                        L"Unknown ImCut operation error.");
            }

            state_.runtime.busy = false;

            if (SUCCEEDED(hr) &&
                state_.runtime.status.empty())
            {
                state_.runtime.status =
                    "Ready";
            }

            if (FAILED(hr))
            {
                state_.runtime.status =
                    WideToUtf8(
                        LastErrorCopy());

                if (interactive && !cancelled) ShowErrorMessage();
            }

            operationRunning_ = false;

            if (visible_)
            {
                lastUiActivity_ = GetTickCount64();
                RenderFrame();
            }

            return hr;
        }

        bool OnOwnerThread()
        {
            const DWORD owner = ownerThreadId_.load();
            if (owner && owner != GetCurrentThreadId())
            {
                SetError(RPC_E_WRONG_THREAD, L"ImCut must run on the thread that initialized it.");
                return false;
            }
            return true;
        }

        [[nodiscard]] bool ValidateOperationContext()
        {
            if (!OnOwnerThread()) return false;
            if (!initialized_)
            {
                (void)SetError(
                    E_UNEXPECTED,
                    L"ImCut is not initialized.");

                return false;
            }

            if (!app_)
            {
                (void)SetError(
                    E_POINTER,
                    L"CorelDRAW application is not available.");

                return false;
            }

            if (GetCurrentThreadId() !=
                ownerThreadId_)
            {
                (void)SetError(
                    RPC_E_WRONG_THREAD,
                    L"CorelDRAW operations must run on the ImCut/Corel UI thread.");

                return false;
            }

            return true;
        }

        HRESULT SetError(
            HRESULT hr,
            std::wstring message)
        {
            {
                std::lock_guard<std::mutex> lock(
                    errorMutex_);

                lastError_ =
                    std::move(
                        message);
            }

            lastHresult_.store(
                hr,
                std::memory_order_relaxed);

            return hr;
        }

        void ClearError()
        {
            {
                std::lock_guard<std::mutex> lock(
                    errorMutex_);

                lastError_.clear();
            }

            lastHresult_.store(
                S_OK,
                std::memory_order_relaxed);
        }

        [[nodiscard]] std::wstring LastErrorCopy() const
        {
            std::lock_guard<std::mutex> lock(
                errorMutex_);

            return lastError_;
        }

        [[nodiscard]] HWND DialogOwnerWindow() const noexcept
        {
            if (visible_ && IsWindow(hwnd_))
                return hwnd_;

            try
            {
                if (app_ && app_->AppWindow)
                {
                    const HWND corel = reinterpret_cast<HWND>(
                        static_cast<INT_PTR>(app_->AppWindow->Handle));
                    if (IsWindow(corel))
                        return corel;
                }
            }
            catch (...)
            {
            }

            return IsWindow(hwnd_) ? hwnd_ : nullptr;
        }

        void ShowErrorMessage()
        {
            const std::wstring error =
                LastErrorCopy();

            if (error.empty())
                return;

            MessageBoxW(
                DialogOwnerWindow(),
                error.c_str(),
                L"ImCut",
                MB_OK |
                MB_ICONERROR);
        }

        IVGApplicationPtr app_;

        UI::State state_;
        UI::Callbacks callbacks_;

        UI::CutState pendingCut_;
        UI::BleedState pendingBleed_;
        UI::ColorState pendingColor_;
        UI::NestingState pendingNesting_;

        SettingsStore settingsStore_;
        StoredProfiles storedProfiles_;
        Updater::Service updater_;
        std::wstring pendingUpdateReadyFlag_;
        bool hotUpdateInstallPending_ = false;
        bool installAfterUserDownload_ = false;
        bool autoUpdateInstallPosted_ = false;

        Color::Converter colorConverter_;
        std::vector<Color::ProfileInfo> cmykProfiles_;
        std::vector<Color::ProfileInfo> rgbProfiles_;
        std::unordered_map<std::string, std::unique_ptr<Color::RgbToSrgbConverter>> previewSrgbConverters_;
        std::unordered_map<std::string, MappingPreviewCacheEntry> colorTargetPreviewCache_;
        std::vector<PendingColorPreviewRequest> pendingColorPreviewRequests_;

        std::atomic<ImCutNestingCallback> nestingCallback_ =
            nullptr;

        HWND hwnd_ = nullptr;
        HINSTANCE moduleInstance_ = nullptr;

        ID3D11Device* device_ = nullptr;
        ID3D11DeviceContext* deviceContext_ = nullptr;
        IDXGISwapChain* swapChain_ = nullptr;
        ID3D11RenderTargetView* renderTarget_ = nullptr;

        std::atomic<DWORD> ownerThreadId_{0};
        ULONGLONG lastSelectionProbe_ = 0;
        ULONGLONG lastUpdateSync_ = 0;
        ULONGLONG lastSettingsSignatureCheck_ = 0;
        ULONGLONG settingsDirtySince_ = 0;
        ULONGLONG lastUiActivity_ = 0;
        UINT renderTimerIntervalMs_ = 0;

        std::uint64_t pendingSettingsSignature_ = 0;
        bool settingsSavePending_ = false;

        std::atomic_bool initialized_{false};
        std::atomic_bool visible_{false};
        bool imguiInitialized_ = false;
        bool rendering_ = false;
        bool renderPosted_ = false;
        bool operationRunning_ = false;
        bool comInitializationOwned_ = false;
        bool profilesLoaded_ = false;
        bool profileLoadPosted_ = false;
        bool selectionProbePending_ = true;
        UI::ClosureMode lastProbedClosureMode_ = UI::ClosureMode::Automatic;
        std::string previewContextKey_;
        ULONGLONG lastPreviewContextProbe_ = 0;
        bool settingsLoaded_ = false;

        std::uint64_t observedSettingsSignature_ = 0;

        mutable std::mutex errorMutex_;
        std::wstring lastError_;
        std::atomic<HRESULT> lastHresult_
        {
            S_OK
        };
    };

    Runtime& Runtime::Instance()
    {
        static Runtime* instance =
            new Runtime();

        return *instance;
    }

    Runtime::Runtime()
        : impl_(
            std::make_unique<Impl>())
    {
    }

    Runtime::~Runtime() = default;

    void Runtime::ReportBoundaryError(HRESULT hr, const wchar_t* message) noexcept
    {
        impl_->ReportBoundaryError(hr, message);
    }

    HRESULT Runtime::Initialize(
        IUnknown* corelApplication)
    {
        return impl_->Initialize(
            corelApplication);
    }

    HRESULT Runtime::ConnectCorel2026()
    {
        return impl_->ConnectCorel2026();
    }

    HRESULT Runtime::ConnectCorel2026Hidden()
    {
        return impl_->ConnectCorel2026Hidden();
    }

    HRESULT Runtime::Show()
    {
        return impl_->Show();
    }

    HRESULT Runtime::Hide()
    {
        return impl_->Hide();
    }

    HRESULT Runtime::Toggle()
    {
        return impl_->Toggle();
    }

    HRESULT Runtime::Shutdown()
    {
        return impl_->Shutdown();
    }

    bool Runtime::IsInitialized() const noexcept
    {
        return impl_->IsInitialized();
    }

    bool Runtime::IsVisible() const noexcept
    {
        return impl_->IsVisible();
    }

    HRESULT Runtime::RunCut(
        int pageWidthMm,
        int registrationMarks,
        int closureMode)
    {
        return impl_->RunCut(
            pageWidthMm,
            registrationMarks,
            closureMode);
    }

    HRESULT Runtime::RunBleed(
        double distanceMm,
        bool ungroupBeforeProcessing,
        bool detectHiddenObjects,
        bool createCutline)
    {
        return impl_->RunBleed(
            distanceMm,
            ungroupBeforeProcessing,
            detectHiddenObjects,
            createCutline);
    }

    HRESULT Runtime::RefreshColorProfiles()
    {
        return impl_->RefreshColorProfiles();
    }

    HRESULT Runtime::ConvertSelection(
        int cmykProfileIndex,
        int rgbProfileIndex,
        int renderingIntent,
        bool adaptiveLut,
        int preferredGrid)
    {
        return impl_->ConvertSelection(
            cmykProfileIndex,
            rgbProfileIndex,
            renderingIntent,
            adaptiveLut,
            preferredGrid);
    }

    HRESULT Runtime::RunCutSaved()
    {
        return impl_->RunCutSaved();
    }

    HRESULT Runtime::RunBleedSaved()
    {
        return impl_->RunBleedSaved();
    }

    HRESULT Runtime::ConvertSelectionSaved()
    {
        return impl_->ConvertSelectionSaved();
    }

    void Runtime::SetNestingCallback(
        ImCutNestingCallback callback) noexcept
    {
        impl_->SetNestingCallback(
            callback);
    }

    int Runtime::CopyLastError(
        wchar_t* buffer,
        int capacity) const
    {
        return impl_->CopyLastError(
            buffer,
            capacity);
    }

    const wchar_t* Runtime::Version() const noexcept
    {
        return impl_->Version();
    }

    LRESULT Runtime::WindowProc(
        HWND hwnd,
        UINT message,
        WPARAM wParam,
        LPARAM lParam)
    {
        return impl_->WindowProc(
            hwnd,
            message,
            wParam,
            lParam);
    }
}
