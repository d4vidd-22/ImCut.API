#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace ImCut::Geometry
{
    // A caller-owned pool of reusable worker threads.
    //
    // The kernel used to create a fresh std::jthread per parallel call. For a nesting
    // run that asks for thousands of small batches that is the wrong shape: thread
    // creation is a syscall and a stack allocation, paid again on every generation,
    // and it lands entirely in the latency of short batches.
    //
    // What this deliberately is NOT:
    //
    //   - not a singleton and not a global. There is no Default(), no lazily created
    //     process-wide pool, and no thread_local cache. If you want workers, you
    //     construct an executor and you own it.
    //   - not a hidden service inside GeometrySession. A session still has no threads.
    //   - not a background service. Between calls the workers are blocked on a
    //     condition variable and consume nothing.
    //
    // Lifetime is explicit and shutdown is deterministic: the destructor wakes every
    // worker, waits for each to leave its loop, and joins it. No detached threads.
    //
    // Thread-safety: one executor is driven by one caller thread at a time. Concurrent
    // Run() calls on the SAME executor are not supported and not needed - the point is
    // to reuse workers across sequential generations, not to multiplex schedulers.
    class GeometryExecutor
    {
    public:
        // The body is invoked once per CHUNK, never once per item.
        //
        // That matters for §97: a virtual call (or a std::function) per geometry item
        // would put heap polymorphism in the hot loop. Per chunk it is amortised over
        // however many items the chunk holds, and the per-item loop lives inside the
        // caller's own code where the compiler can see it.
        class ChunkBody
        {
        public:
            virtual ~ChunkBody() = default;

            // [begin, end) of the caller's index space. workerIndex is stable within a
            // single Run() and is < WorkerCount(), so it can key worker-local storage.
            //
            // Must not throw: the executor catches and swallows to keep shutdown
            // deterministic, so an escaping exception silently loses that chunk's work.
            // Record failures in your own output slots instead.
            virtual void Run(std::size_t workerIndex,
                             std::size_t begin,
                             std::size_t end) noexcept = 0;
        };

        // workerCount is the TOTAL degree of parallelism including the calling thread,
        // so an executor of N spawns N-1 helpers and the caller works as one of them.
        // The caller would otherwise sit blocked while N threads run, wasting a core.
        // workerCount 0 and 1 are both "run inline", and spawn nothing.
        explicit GeometryExecutor(std::size_t workerCount);
        ~GeometryExecutor();

        GeometryExecutor(const GeometryExecutor&) = delete;
        GeometryExecutor& operator=(const GeometryExecutor&) = delete;
        GeometryExecutor(GeometryExecutor&&) = delete;
        GeometryExecutor& operator=(GeometryExecutor&&) = delete;

        [[nodiscard]] std::size_t WorkerCount() const noexcept { return workerCount_; }

        // Number of OS threads this executor owns (WorkerCount() - 1, or 0 when inline).
        [[nodiscard]] std::size_t ThreadCount() const noexcept { return threads_.size(); }

        // Splits [0, itemCount) into chunks of chunkSize and runs them across the pool,
        // returning only once every chunk has completed. chunkSize 0 is treated as 1.
        //
        // Chunks are claimed from a shared atomic cursor rather than statically
        // partitioned, because per-pair cost in this kernel varies by an order of
        // magnitude with geometry complexity and a static split leaves workers idle.
        // The claim order is therefore NOT deterministic - determinism comes from each
        // item owning a fixed output slot, never from which worker ran it.
        void Run(std::size_t itemCount, std::size_t chunkSize, ChunkBody& body);

        // Generations completed by this executor; lets a test prove threads were
        // actually reused rather than recreated.
        [[nodiscard]] std::uint64_t GenerationsRun() const noexcept
        {
            return generationsRun_;
        }

    private:
        void WorkerLoop(std::size_t workerIndex);
        void DrainChunks(std::size_t workerIndex) noexcept;

        // Signal stop, wake every helper, join, and drop them. Called by the destructor
        // and by the constructor's failure path, which is the only way those two can be
        // guaranteed to agree - a constructor that throws never runs its own destructor.
        void ShutdownWorkers() noexcept;

        std::size_t workerCount_ = 1;
        std::vector<std::thread> threads_;

        std::mutex mutex_;
        std::condition_variable wake_;
        std::condition_variable done_;

        ChunkBody* body_ = nullptr;
        std::size_t itemCount_ = 0;
        std::size_t chunkSize_ = 1;
        std::atomic<std::size_t> cursor_{ 0 };

        std::uint64_t generation_ = 0;
        std::uint64_t generationsRun_ = 0;
        std::size_t pending_ = 0;
        bool stopping_ = false;
    };
}
