#pragma once

#include <array>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace ImCut::Geometry
{
    enum class GeometryDiagnosticStage : std::uint8_t
    {
        ValidationExclusive,
        TransformExclusive,
        FlattenExclusive,
        ConvexityExclusive,
        DecompositionExclusive,
        ConvexSumExclusive,
        BooleanConversionExclusive,
        BooleanBackendExclusive,
        ResultConversionExclusive,
        BudgetExclusive,
        Count
    };

    // Query-local timing sink. Stages may be nested: End() subtracts all child time
    // before accumulating the parent, so summing ExclusiveNanoseconds() is additive.
    // One sink belongs to one executing query/thread; aggregate completed sinks outside
    // the timed kernel when parallel statistics are needed.
    class GeometryDiagnostics
    {
    public:
        using Clock = std::chrono::steady_clock;
        static constexpr std::size_t kStageCount =
            static_cast<std::size_t>(GeometryDiagnosticStage::Count);

        void Reset() noexcept
        {
            exclusiveNanoseconds_.fill(0);
            totalNanoseconds_ = 0;
            queryCount_ = 0;
            totalDepth_ = 0;
            depth_ = 0;
            stackOverflow_ = false;
        }

        [[nodiscard]] std::uint64_t ExclusiveNanoseconds(
            GeometryDiagnosticStage stage) const noexcept
        {
            return exclusiveNanoseconds_[static_cast<std::size_t>(stage)];
        }

        [[nodiscard]] std::uint64_t ExclusiveSumNanoseconds() const noexcept
        {
            std::uint64_t total = 0;
            for (const std::uint64_t value : exclusiveNanoseconds_)
                total += value;
            return total;
        }

        [[nodiscard]] std::uint64_t TotalNanoseconds() const noexcept
        {
            return totalNanoseconds_;
        }

        [[nodiscard]] std::uint64_t UnaccountedNanoseconds() const noexcept
        {
            const std::uint64_t stages = ExclusiveSumNanoseconds();
            return totalNanoseconds_ > stages ? totalNanoseconds_ - stages : 0;
        }

        [[nodiscard]] std::uint64_t QueryCount() const noexcept { return queryCount_; }
        [[nodiscard]] bool StackOverflowed() const noexcept { return stackOverflow_; }

        [[nodiscard]] bool Begin(GeometryDiagnosticStage stage) noexcept
        {
            if (depth_ >= frames_.size())
            {
                stackOverflow_ = true;
                return false;
            }
            Frame& frame = frames_[depth_++];
            frame.stage = stage;
            frame.start = Clock::now();
            frame.childNanoseconds = 0;
            return true;
        }

        void End(GeometryDiagnosticStage stage) noexcept
        {
            if (depth_ == 0)
                return;
            const Clock::time_point stop = Clock::now();
            Frame frame = frames_[--depth_];
            if (frame.stage != stage)
            {
                stackOverflow_ = true;
                return;
            }

            const std::uint64_t elapsed = static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(stop - frame.start).count());
            const std::uint64_t exclusive = elapsed > frame.childNanoseconds
                ? elapsed - frame.childNanoseconds : 0;
            exclusiveNanoseconds_[static_cast<std::size_t>(stage)] += exclusive;
            if (depth_ != 0)
                frames_[depth_ - 1].childNanoseconds += elapsed;
        }

        void AddTotal(Clock::duration elapsed) noexcept
        {
            totalNanoseconds_ += static_cast<std::uint64_t>(
                std::chrono::duration_cast<std::chrono::nanoseconds>(elapsed).count());
            ++queryCount_;
        }

        // Total scopes may be nested by public/session wrappers. Only the outermost
        // scope owns a clock sample and publishes one completed query.
        [[nodiscard]] bool BeginTotal() noexcept
        {
            const bool outermost = totalDepth_ == 0;
            ++totalDepth_;
            if (outermost)
                totalStart_ = Clock::now();
            return outermost;
        }

        void EndTotal(bool outermost) noexcept
        {
            if (totalDepth_ == 0)
            {
                stackOverflow_ = true;
                return;
            }
            --totalDepth_;
            if (!outermost)
                return;
            if (totalDepth_ != 0)
            {
                stackOverflow_ = true;
                return;
            }
            AddTotal(Clock::now() - totalStart_);
        }

    private:
        struct Frame
        {
            GeometryDiagnosticStage stage = GeometryDiagnosticStage::ValidationExclusive;
            Clock::time_point start{};
            std::uint64_t childNanoseconds = 0;
        };

        std::array<std::uint64_t, kStageCount> exclusiveNanoseconds_{};
        std::array<Frame, 32> frames_{};
        std::uint64_t totalNanoseconds_ = 0;
        std::uint64_t queryCount_ = 0;
        Clock::time_point totalStart_{};
        std::size_t totalDepth_ = 0;
        std::size_t depth_ = 0;
        bool stackOverflow_ = false;
    };

    class ScopedGeometryStage
    {
    public:
        ScopedGeometryStage(GeometryDiagnostics* diagnostics,
                            GeometryDiagnosticStage stage) noexcept
            : diagnostics_(diagnostics), stage_(stage), active_(diagnostics != nullptr &&
                  diagnostics->Begin(stage))
        {
        }

        ~ScopedGeometryStage()
        {
            if (active_) diagnostics_->End(stage_);
        }

        ScopedGeometryStage(const ScopedGeometryStage&) = delete;
        ScopedGeometryStage& operator=(const ScopedGeometryStage&) = delete;

    private:
        GeometryDiagnostics* diagnostics_ = nullptr;
        GeometryDiagnosticStage stage_ = GeometryDiagnosticStage::ValidationExclusive;
        bool active_ = false;
    };

    class ScopedGeometryTotal
    {
    public:
        explicit ScopedGeometryTotal(GeometryDiagnostics* diagnostics) noexcept
            : diagnostics_(diagnostics),
              outermost_(diagnostics != nullptr && diagnostics->BeginTotal()) {}

        ~ScopedGeometryTotal()
        {
            if (diagnostics_ != nullptr)
                diagnostics_->EndTotal(outermost_);
        }

        ScopedGeometryTotal(const ScopedGeometryTotal&) = delete;
        ScopedGeometryTotal& operator=(const ScopedGeometryTotal&) = delete;

    private:
        GeometryDiagnostics* diagnostics_ = nullptr;
        bool outermost_ = false;
    };
}
