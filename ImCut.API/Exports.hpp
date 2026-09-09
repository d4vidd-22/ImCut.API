#pragma once

#include <Windows.h>
#include <Unknwn.h>

#ifdef IMCUT_IMPLEMENTATION
#define IMCUT_API extern "C" __declspec(dllexport)
#else
#define IMCUT_API extern "C" __declspec(dllimport)
#endif

struct ImCutNestingRequest
{
    float spacingMm;
    int rotationStepDeg;
    int generations;
    int population;
    BOOL allowMirror;
    BOOL useTrueShape;
};

using ImCutNestingCallback =
HRESULT(__stdcall*)(
    IUnknown* corelApplication,
    const ImCutNestingRequest* request);

IMCUT_API HRESULT __stdcall ImCut_Initialize(
    IUnknown* corelApplication);

IMCUT_API HRESULT __stdcall ImCut_ConnectCorel2026();
IMCUT_API HRESULT __stdcall ImCut_ConnectCorel2026Hidden();
IMCUT_API HRESULT __stdcall ImCut_RunCutSaved();
IMCUT_API HRESULT __stdcall ImCut_RunBleedSaved();
IMCUT_API HRESULT __stdcall ImCut_ConvertSelectionSaved();

IMCUT_API HRESULT __stdcall ImCut_Show();

IMCUT_API HRESULT __stdcall ImCut_Hide();

IMCUT_API HRESULT __stdcall ImCut_Toggle();

IMCUT_API HRESULT __stdcall ImCut_Shutdown();

IMCUT_API BOOL __stdcall ImCut_IsInitialized();

IMCUT_API BOOL __stdcall ImCut_IsVisible();

IMCUT_API HRESULT __stdcall ImCut_RunCut(
    int pageWidthMm,
    int registrationMarks,
    int closureMode);

IMCUT_API HRESULT __stdcall ImCut_RunBleed(
    double distanceMm,
    BOOL ungroupBeforeProcessing,
    BOOL detectHiddenObjects,
    BOOL createCutline);

IMCUT_API HRESULT __stdcall ImCut_RefreshColorProfiles();

IMCUT_API HRESULT __stdcall ImCut_ConvertSelection(
    int cmykProfileIndex,
    int rgbProfileIndex,
    int renderingIntent,
    BOOL adaptiveLut,
    int preferredGrid);

IMCUT_API void __stdcall ImCut_SetNestingCallback(
    ImCutNestingCallback callback);

IMCUT_API int __stdcall ImCut_GetLastError(
    wchar_t* buffer,
    int capacity);

IMCUT_API const wchar_t* __stdcall ImCut_GetVersion();
