#pragma once

// Copy/move instrumentation for the heavy geometry payload types.
//
// This exists to answer one question with evidence rather than assertion: when the
// session serves a cached NFP result, how many times is the payload actually copied?
// An allocation counter cannot answer it - a copy that reuses spare capacity allocates
// nothing, and one allocation can back many logical objects.
//
// Compiled ONLY when IMCUT_GEOMETRY_COPY_COUNTERS is defined, which neither the
// shipping DLL (ImCut.API.vcxproj) nor the DEFAULT standalone harness defines. Both
// exclusions are deliberate:
//
//   - the production DLL must never carry counters;
//   - the default harness must not either, because geometry_bench.exe is built from
//     the same switches as geometry_tests.exe. A branch and an atomic bump on every
//     Path copy would silently tax every number in the performance matrix - exactly
//     the invalid benchmark the plan forbids.
//
// So the counters get their own build:  build_tests.cmd <dir> counters
//
// Mechanism: an empty base class. The instrumented type keeps the rule of zero, so its
// own members are still copied by the compiler-generated member-wise code and no member
// list is duplicated here. Adding a field to Path can therefore never desynchronise the
// instrumentation. When the macro is off the base disappears entirely and the types are
// bit-for-bit the ones the kernel ships.

#if defined(IMCUT_GEOMETRY_COPY_COUNTERS)

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace ImCut::Geometry::Instrumentation
{
    // Relaxed atomics: batch/executor work copies payloads on several worker threads,
    // so a plain counter would be a genuine data race. Relaxed ordering is sufficient -
    // these are totals read after workers join, never used to establish happens-before.
    struct CopyMoveCounter
    {
        std::atomic<std::uint64_t> copies{ 0 };
        std::atomic<std::uint64_t> moves{ 0 };

        void CountCopy() noexcept { copies.fetch_add(1, std::memory_order_relaxed); }
        void CountMove() noexcept { moves.fetch_add(1, std::memory_order_relaxed); }

        void Reset() noexcept
        {
            copies.store(0, std::memory_order_relaxed);
            moves.store(0, std::memory_order_relaxed);
        }
        [[nodiscard]] std::uint64_t Copies() const noexcept
        {
            return copies.load(std::memory_order_relaxed);
        }
        [[nodiscard]] std::uint64_t Moves() const noexcept
        {
            return moves.load(std::memory_order_relaxed);
        }
    };

    struct PathTag;
    struct ContourTag;
    struct NfpResultTag;
    struct NfpCoverTag;
    struct IntRingTag;

    template <class Tag>
    [[nodiscard]] inline CopyMoveCounter& Counter() noexcept
    {
        static CopyMoveCounter counter;
        return counter;
    }

    // Empty base. Copying/moving the derived object copies/moves this subobject, which
    // is the only thing it does.
    template <class Tag>
    struct CopyMoveCounted
    {
        CopyMoveCounted() = default;
        ~CopyMoveCounted() = default;

        CopyMoveCounted(const CopyMoveCounted&) noexcept { Counter<Tag>().CountCopy(); }
        CopyMoveCounted(CopyMoveCounted&&) noexcept { Counter<Tag>().CountMove(); }

        CopyMoveCounted& operator=(const CopyMoveCounted&) noexcept
        {
            Counter<Tag>().CountCopy();
            return *this;
        }
        CopyMoveCounted& operator=(CopyMoveCounted&&) noexcept
        {
            Counter<Tag>().CountMove();
            return *this;
        }
    };

    // IntRing is std::vector<IntPoint>, an alias the kernel does not own, so it cannot
    // carry a base. It is counted explicitly at the only boundary able to duplicate
    // one: the backend conversion layer.
    inline void CountIntRingCopies(std::size_t rings) noexcept
    {
        Counter<IntRingTag>().copies.fetch_add(rings, std::memory_order_relaxed);
    }

    struct Totals
    {
        std::uint64_t pathCopies = 0;
        std::uint64_t pathMoves = 0;
        std::uint64_t contourCopies = 0;
        std::uint64_t contourMoves = 0;
        std::uint64_t nfpResultCopies = 0;
        std::uint64_t nfpResultMoves = 0;
        std::uint64_t nfpCoverCopies = 0;
        std::uint64_t nfpCoverMoves = 0;
        std::uint64_t intRingCopies = 0;
    };

    [[nodiscard]] inline Totals Read() noexcept
    {
        Totals totals;
        totals.pathCopies = Counter<PathTag>().Copies();
        totals.pathMoves = Counter<PathTag>().Moves();
        totals.contourCopies = Counter<ContourTag>().Copies();
        totals.contourMoves = Counter<ContourTag>().Moves();
        totals.nfpResultCopies = Counter<NfpResultTag>().Copies();
        totals.nfpResultMoves = Counter<NfpResultTag>().Moves();
        totals.nfpCoverCopies = Counter<NfpCoverTag>().Copies();
        totals.nfpCoverMoves = Counter<NfpCoverTag>().Moves();
        totals.intRingCopies = Counter<IntRingTag>().Copies();
        return totals;
    }

    inline void ResetAll() noexcept
    {
        Counter<PathTag>().Reset();
        Counter<ContourTag>().Reset();
        Counter<NfpResultTag>().Reset();
        Counter<NfpCoverTag>().Reset();
        Counter<IntRingTag>().Reset();
    }

    // RAII zeroing scope, so a measurement reads only its own window.
    struct Scope
    {
        Scope() noexcept { ResetAll(); }
        Scope(const Scope&) = delete;
        Scope& operator=(const Scope&) = delete;
        [[nodiscard]] Totals Read() const noexcept
        {
            return ::ImCut::Geometry::Instrumentation::Read();
        }
    };
}

#define IMCUT_GEOMETRY_COUNTED(TagName)                                              \
    : public ::ImCut::Geometry::Instrumentation::CopyMoveCounted<                    \
          ::ImCut::Geometry::Instrumentation::TagName>

#define IMCUT_GEOMETRY_COUNT_INT_RINGS(n)                                            \
    ::ImCut::Geometry::Instrumentation::CountIntRingCopies(n)

#else  // !IMCUT_GEOMETRY_COPY_COUNTERS

#define IMCUT_GEOMETRY_COUNTED(TagName)
#define IMCUT_GEOMETRY_COUNT_INT_RINGS(n) ((void)0)

#endif  // IMCUT_GEOMETRY_COPY_COUNTERS
