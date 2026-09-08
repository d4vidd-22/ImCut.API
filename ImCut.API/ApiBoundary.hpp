#pragma once
#include "Runtime.hpp"
#include <comdef.h>
#include <new>

namespace ImCut
{
    inline HRESULT ReportCurrentException() noexcept
    {
        HRESULT hr = E_FAIL;
        const wchar_t* message = L"Falha interna inesperada no ImCut.";
        try { throw; }
        catch (const _com_error& error) { hr = error.Error(); message = L"Falha COM na chamada ao ImCut."; }
        catch (const std::bad_alloc&) { hr = E_OUTOFMEMORY; message = L"Memoria insuficiente no ImCut."; }
        catch (...) {}
        try { Runtime::Instance().ReportBoundaryError(hr, message); } catch (...) {}
        return hr;
    }
    template<class F> HRESULT ApiCall(F&& fn) noexcept
    {
        try { return fn(); } catch (...) { return ReportCurrentException(); }
    }
}
