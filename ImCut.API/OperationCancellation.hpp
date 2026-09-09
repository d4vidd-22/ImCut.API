#pragma once

#include <Windows.h>

#include <cstddef>
#include <exception>

namespace ImCut
{
    class OperationCancelled final : public std::exception
    {
    public:
        const char* what() const noexcept override
        {
            return "Operacao cancelada com Esc.";
        }
    };

    namespace Cancellation
    {
        inline thread_local unsigned int operationDepth = 0;
        inline thread_local bool cancelRequested = false;

        class OperationScope final
        {
        public:
            OperationScope() noexcept
            {
                owner_ = operationDepth++ == 0;
                if (owner_)
                {
                    cancelRequested = false;
                    (void)GetAsyncKeyState(VK_ESCAPE);
                }
            }

            ~OperationScope() noexcept
            {
                if (operationDepth > 0)
                    --operationDepth;

                if (owner_)
                    cancelRequested = false;
            }

            OperationScope(const OperationScope&) = delete;
            OperationScope& operator=(const OperationScope&) = delete;

        private:
            bool owner_ = false;
        };

        [[nodiscard]] inline bool Requested() noexcept
        {
            if (cancelRequested)
                return true;

            if (operationDepth == 0)
                return false;

            const SHORT state = GetAsyncKeyState(VK_ESCAPE);
            if ((state & 0x8001) != 0)
                cancelRequested = true;

            return cancelRequested;
        }

        inline void Request() noexcept
        {
            cancelRequested = true;
        }

        inline void ThrowIfRequested()
        {
            if (Requested())
                throw OperationCancelled();
        }

        inline void Checkpoint(
            std::size_t iteration,
            std::size_t intervalMask = 0x3ff)
        {
            if ((iteration & intervalMask) == 0)
                ThrowIfRequested();
        }

        inline void PumpPaintAndThrow()
        {
            MSG message{};
            while (PeekMessageW(
                &message,
                nullptr,
                WM_PAINT,
                WM_PAINT,
                PM_REMOVE))
            {
                TranslateMessage(&message);
                DispatchMessageW(&message);
            }

            ThrowIfRequested();
        }
    }
}
