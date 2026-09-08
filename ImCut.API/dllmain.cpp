#include "pch.h"

#define IMCUT_IMPLEMENTATION
#include "Exports.hpp"
#include "Runtime.hpp"
#include "ApiBoundary.hpp"
#include "Version.hpp"

#include <Windows.h>

BOOL WINAPI DllMain(
    HINSTANCE module,
    DWORD reason,
    LPVOID)
{
    if (reason ==
        DLL_PROCESS_ATTACH)
    {
        DisableThreadLibraryCalls(
            module);
    }

    return TRUE;
}

HRESULT __stdcall ImCut_Initialize(
    IUnknown* corelApplication)
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().Initialize(
        corelApplication);
    });
}

HRESULT __stdcall ImCut_ConnectCorel2026()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().ConnectCorel2026();
    });
}

HRESULT __stdcall ImCut_ConnectCorel2026Hidden()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().ConnectCorel2026Hidden();
    });
}

HRESULT __stdcall ImCut_Show()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().Show();
    });
}

HRESULT __stdcall ImCut_Hide()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().Hide();
    });
}

HRESULT __stdcall ImCut_Toggle()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().Toggle();
    });
}

HRESULT __stdcall ImCut_Shutdown()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().Shutdown();
    });
}

BOOL __stdcall ImCut_IsInitialized()
{
    try
    {
    return ImCut::Runtime::Instance().IsInitialized()
        ? TRUE
        : FALSE;
    } catch (...) { (void)ImCut::ReportCurrentException(); return FALSE; }
}

BOOL __stdcall ImCut_IsVisible()
{
    try
    {
    return ImCut::Runtime::Instance().IsVisible()
        ? TRUE
        : FALSE;
    } catch (...) { (void)ImCut::ReportCurrentException(); return FALSE; }
}

HRESULT __stdcall ImCut_RunCut(
    int pageWidthMm,
    int registrationMarks,
    int closureMode)
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().RunCut(
        pageWidthMm,
        registrationMarks,
        closureMode);
    });
}

HRESULT __stdcall ImCut_RunBleed(
    double distanceMm,
    BOOL ungroupBeforeProcessing,
    BOOL detectHiddenObjects,
    BOOL createCutline)
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().RunBleed(
        distanceMm,
        ungroupBeforeProcessing != FALSE,
        detectHiddenObjects != FALSE,
        createCutline != FALSE);
    });
}

HRESULT __stdcall ImCut_RefreshColorProfiles()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().RefreshColorProfiles();
    });
}

HRESULT __stdcall ImCut_ConvertSelection(
    int cmykProfileIndex,
    int rgbProfileIndex,
    int renderingIntent,
    BOOL adaptiveLut,
    int preferredGrid)
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().ConvertSelection(
        cmykProfileIndex,
        rgbProfileIndex,
        renderingIntent,
        adaptiveLut != FALSE,
        preferredGrid);
    });
}

HRESULT __stdcall ImCut_RunCutSaved()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().RunCutSaved();
    });
}

HRESULT __stdcall ImCut_RunBleedSaved()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().RunBleedSaved();
    });
}

HRESULT __stdcall ImCut_ConvertSelectionSaved()
{
    return ImCut::ApiCall([&]() -> HRESULT
    {
    return ImCut::Runtime::Instance().ConvertSelectionSaved();
    });
}

void __stdcall ImCut_SetNestingCallback(
    ImCutNestingCallback callback)
{
    try { ImCut::Runtime::Instance().SetNestingCallback(callback); }
    catch (...) { (void)ImCut::ReportCurrentException(); }
}

int __stdcall ImCut_GetLastError(
    wchar_t* buffer,
    int capacity)
{
    try { return ImCut::Runtime::Instance().CopyLastError(buffer, capacity); }
    catch (...) { if (buffer && capacity > 0) buffer[0] = L'\0'; return 0; }
}

const wchar_t* __stdcall ImCut_GetVersion()
{
    return L"ImCut Runtime " IMCUT_VERSION_WSTRING;
}