#include "GeometryExecutor.hpp"

#include "GeometryTestHooks.hpp"

#include <algorithm>
#include <system_error>

namespace ImCut::Geometry
{
    GeometryExecutor::GeometryExecutor(std::size_t workerCount)
        : workerCount_((std::max)(std::size_t{ 1 }, workerCount))
    {
        if (workerCount_ <= 1)
            return;

        threads_.reserve(workerCount_ - 1);

        // A CONSTRUCTOR THAT THROWS DOES NOT GET ITS OWN DESTRUCTOR.
        //
        // V8.1 asserted the opposite here, in a comment: "If thread creation throws
        // part-way, the destructor still shuts down and joins whatever was created."
        // That is not what C++ does. When a constructor body throws, only the destructors
        // of already-constructed MEMBERS run - ~GeometryExecutor never does. So
        // ~vector<std::thread> destroyed threads that were still joinable, and
        // ~thread() on a joinable thread calls std::terminate.
        //
        // The helpers really were joinable: nothing in the constructor set stopping_ or
        // notified wake_, so every thread already created was parked in WorkerLoop
        // waiting on the condition variable. Measured: the process aborted rather than
        // propagating (evidence 873).
        //
        // So the cleanup the comment described has to be written out. Basic exception
        // safety, and the executor is not silently downgraded to fewer helpers than
        // WorkerCount() reports - the caller is told, and decides.
        try
        {
            // Helper index 0 is the caller, so spawned helpers take 1..workerCount_-1.
            for (std::size_t i = 1; i < workerCount_; ++i)
            {
                // Test-only injection point; disarmed in every shipped call. Thread
                // creation failure cannot be provoked by input, and section 41 asks for a
                // fail-after-N rather than actually exhausting the system.
                GeometryStatus injected = GeometryStatus::Success;
                if (Testing::ShouldInjectFailure(Testing::FailurePoint::ExecutorThread,
                                                 injected))
                {
                    throw std::system_error(
                        std::make_error_code(std::errc::resource_unavailable_try_again),
                        "injected thread creation failure");
                }

                threads_.emplace_back([this, i] { WorkerLoop(i); });
            }
        }
        catch (...)
        {
            ShutdownWorkers();
            throw;
        }
    }

    // Signal, wake, join. Shared by the constructor's failure path and the destructor so
    // the two cannot drift apart - the drift is what F10 was.
    void GeometryExecutor::ShutdownWorkers() noexcept
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stopping_ = true;
        }
        wake_.notify_all();
        for (std::thread& thread : threads_)
        {
            if (thread.joinable())
                thread.join();
        }
        threads_.clear();
    }

    GeometryExecutor::~GeometryExecutor()
    {
        ShutdownWorkers();
    }

    void GeometryExecutor::DrainChunks(std::size_t workerIndex) noexcept
    {
        // body_/itemCount_/chunkSize_ are written under the mutex before the workers
        // are woken and are not touched again until every worker has reported, so they
        // are safe to read unsynchronised here. Only the cursor is contended.
        ChunkBody* const body = body_;
        const std::size_t itemCount = itemCount_;
        const std::size_t chunkSize = chunkSize_;
        if (body == nullptr)
            return;

        for (;;)
        {
            const std::size_t begin =
                cursor_.fetch_add(chunkSize, std::memory_order_relaxed);
            if (begin >= itemCount)
                return;
            const std::size_t end = (std::min)(begin + chunkSize, itemCount);
            body->Run(workerIndex, begin, end);
        }
    }

    void GeometryExecutor::WorkerLoop(std::size_t workerIndex)
    {
        std::uint64_t seen = 0;
        for (;;)
        {
            {
                std::unique_lock<std::mutex> lock(mutex_);
                wake_.wait(lock, [this, seen]
                {
                    return stopping_ || generation_ != seen;
                });
                if (stopping_)
                    return;
                seen = generation_;
            }

            DrainChunks(workerIndex);

            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (--pending_ == 0)
                    done_.notify_one();
            }
        }
    }

    void GeometryExecutor::Run(std::size_t itemCount, std::size_t chunkSize,
                               ChunkBody& body)
    {
        if (itemCount == 0)
            return;

        const std::size_t effectiveChunk = (std::max)(std::size_t{ 1 }, chunkSize);

        if (threads_.empty())
        {
            // Inline: no threads exist, so there is nothing to wake and nothing to
            // wait for. Keeps the one-worker path free of all synchronisation.
            body_ = &body;
            itemCount_ = itemCount;
            chunkSize_ = effectiveChunk;
            cursor_.store(0, std::memory_order_relaxed);
            DrainChunks(0);
            body_ = nullptr;
            ++generationsRun_;
            return;
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            body_ = &body;
            itemCount_ = itemCount;
            chunkSize_ = effectiveChunk;
            cursor_.store(0, std::memory_order_relaxed);
            pending_ = threads_.size();
            ++generation_;
        }
        wake_.notify_all();

        // The caller is worker 0 and pulls chunks like everyone else, so no core sits
        // idle waiting for the pool it just dispatched to.
        DrainChunks(0);

        {
            std::unique_lock<std::mutex> lock(mutex_);
            done_.wait(lock, [this] { return pending_ == 0; });
            body_ = nullptr;
            itemCount_ = 0;
        }
        ++generationsRun_;
    }
}
