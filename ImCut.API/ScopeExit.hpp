#pragma once
#include <utility>

namespace ImCut
{
    template<class F> class ScopeExit final
    {
    public:
        explicit ScopeExit(F fn) : fn_(std::move(fn)) {}
        ~ScopeExit() noexcept { if (active_) { try { fn_(); } catch (...) {} } }
        ScopeExit(const ScopeExit&) = delete;
        ScopeExit& operator=(const ScopeExit&) = delete;
        void Release() noexcept { active_ = false; }
    private:
        F fn_;
        bool active_ = true;
    };
}
