#include "NfpBatch.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <limits>
#include <new>


namespace ImCut::Geometry::Nfp
{
    const PreparedNfpBatchVariant* PreparedNfpBatch::Variant(
        std::size_t shapeIndex, std::size_t rotationIndex) const noexcept
    {
        if (shapeIndex >= shapeCount_ || rotationIndex >= rotationCount_)
            return nullptr;
        return &variants_[shapeIndex * rotationCount_ + rotationIndex];
    }

    std::size_t PreparedNfpBatch::ApproximateBytes() const noexcept
    {
        std::size_t bytes = sizeof(PreparedNfpBatch) +
                            variants_.capacity() * sizeof(PreparedNfpBatchVariant);
        // This is a conservative O(N) upper bound. Repeated shape/rotation inputs may
        // make shared_ptrs alias and are then counted more than once; an approximate
        // memory query must not turn into an O(N^2) hot walk or allocate inside noexcept.
        for (const PreparedNfpBatchVariant& current : variants_)
        {
            if (current.stationary)
                bytes += current.stationary->ApproximateBytes();
            if (current.reflected)
                bytes += current.reflected->ApproximateBytes();
        }
        return bytes;
    }

    bool NfpCoverBatchResult::AllSucceeded() const noexcept
    {
        for (const GeometryResult<NfpCover>& result : results)
            if (!result.Ok()) return false;
        return true;
    }

    std::size_t NfpCoverBatchResult::ApproximateBytes() const noexcept
    {
        std::size_t bytes = sizeof(NfpCoverBatchResult) +
                            results.capacity() * sizeof(GeometryResult<NfpCover>);
        for (const GeometryResult<NfpCover>& result : results)
            if (result.Ok()) bytes += result.Value().ApproximateBytes();
        return bytes;
    }

    GeometryResult<PreparedNfpBatch> PrepareNfpBatch(
        GeometrySession& session,
        const std::vector<const PreparedShapeDefinition*>& definitions,
        const std::vector<double>& rotations,
        const NfpOptions& options,
        const GeometryQueryControl& control)
    {
        std::vector<NfpQueryOperand> queries;
        queries.reserve(definitions.size());
        for (const PreparedShapeDefinition* definition : definitions)
        {
            if (definition == nullptr)
            {
                return GeometryResult<PreparedNfpBatch>::Failure(
                    GeometryStatus::InvalidInput);
            }
            queries.emplace_back(*definition);
        }
        return PrepareNfpBatch(session, queries, rotations, options, control);
    }

    GeometryResult<PreparedNfpBatch> PrepareNfpBatch(
        GeometrySession& session,
        const std::vector<NfpQueryOperand>& definitions,
        const std::vector<double>& rotations,
        const NfpOptions& options,
        const GeometryQueryControl& control)
    {
        if (definitions.empty() || rotations.empty())
            return GeometryResult<PreparedNfpBatch>::Empty(PreparedNfpBatch{});
        if (rotations.size() >
            (std::numeric_limits<std::size_t>::max)() / definitions.size())
        {
            return GeometryResult<PreparedNfpBatch>::Failure(
                GeometryStatus::ComplexityLimit);
        }
        const std::size_t variantCount = definitions.size() * rotations.size();
        if (variantCount > session.Config().limits.maxNfpBatchVariants)
        {
            return GeometryResult<PreparedNfpBatch>::Failure(
                GeometryStatus::ComplexityLimit);
        }
        for (const NfpQueryOperand& definition : definitions)
            if (definition.definition == nullptr)
                return GeometryResult<PreparedNfpBatch>::Failure(
                    GeometryStatus::InvalidInput);
        for (const double rotation : rotations)
            if (!std::isfinite(rotation))
                return GeometryResult<PreparedNfpBatch>::Failure(
                    GeometryStatus::InvalidInput);

        PreparedNfpBatch batch;
        batch.shapeCount_ = definitions.size();
        batch.rotationCount_ = rotations.size();
        batch.variants_.reserve(variantCount);

        for (std::size_t shape = 0; shape < definitions.size(); ++shape)
        {
            for (std::size_t rotation = 0; rotation < rotations.size(); ++rotation)
            {
                if (control.cancellation.IsCancelled())
                    return GeometryResult<PreparedNfpBatch>::Failure(
                        GeometryStatus::Cancelled);

                auto stationary = session.PrepareNfpOperand(
                    definitions[shape], rotations[rotation], false, options, control);
                if (!stationary.Ok())
                    return GeometryResult<PreparedNfpBatch>::Failure(stationary.Status());
                auto reflected = session.PrepareNfpOperand(
                    definitions[shape], rotations[rotation], true, options, control);
                if (!reflected.Ok())
                    return GeometryResult<PreparedNfpBatch>::Failure(reflected.Status());

                PreparedNfpBatchVariant variant;
                variant.shapeIndex = shape;
                variant.rotationIndex = rotation;
                variant.rotationRadians = rotations[rotation];
                NfpPreparedQueryOperand stationaryQuery =
                    std::move(stationary).Value();
                NfpPreparedQueryOperand reflectedQuery =
                    std::move(reflected).Value();
                variant.stationary = std::move(stationaryQuery.operand);
                variant.reflected = std::move(reflectedQuery.operand);
                variant.provenance = stationaryQuery.provenance;
                batch.variants_.push_back(std::move(variant));
            }
        }
        return GeometryResult<PreparedNfpBatch>::Success(std::move(batch));
    }

    namespace
    {
        // Shared admission check for every batch entry point, so the one-shot form and
        // the executor form can never drift apart on what they accept.
        [[nodiscard]] GeometryStatus ValidateBatchRequest(
            const PreparedNfpBatch& prepared,
            const std::vector<NfpBatchPair>& pairs,
            std::size_t workerCount,
            const GeometryContext& context)
        {
            if (pairs.size() > context.Limits().maxNfpBatchPairs)
                return GeometryStatus::ComplexityLimit;
            if (workerCount > context.Limits().maxNfpBatchWorkers)
                return GeometryStatus::ComplexityLimit;
            // Diagnostics collect into one shared sink with no ordering, so a parallel
            // run would produce a stage timeline that never happened. Rejected rather
            // than silently dropped.
            if (workerCount == 0 ||
                (workerCount > 1 && pairs.size() > 1 && context.Diagnostics() != nullptr))
            {
                return GeometryStatus::InvalidInput;
            }
            for (const NfpBatchPair& pair : pairs)
            {
                if (pair.stationaryVariant >= prepared.VariantCount() ||
                    pair.movingVariant >= prepared.VariantCount())
                {
                    return GeometryStatus::InvalidInput;
                }
            }
            if (!prepared.Variants().empty())
            {
                const GeometryStatus profileStatus = prepared.Variants().front()
                    .stationary->Profile().ContextStatus(context);
                if (profileStatus != GeometryStatus::Success)
                    return profileStatus;
            }
            return GeometryStatus::Success;
        }

        [[nodiscard]] GeometryContext MakeWorkerContext(
            const GeometryContext& context, bool serial)
        {
            GeometryContext local(context.Tolerance(), context.Precision());
            local.SetLimits(context.Limits());
            local.SetCancellation(context.Cancellation());
            local.SetStatistics(context.Statistics());
            if (serial)
                local.SetDiagnostics(context.Diagnostics());
            return local;
        }

        // One pair. Failure is isolated to the pair's own output slot: one bad pair in
        // a batch of hundreds must not discard the other results.
        void RunOnePair(const PreparedNfpBatch& prepared,
                        const NfpBatchPair& request,
                        const GeometryContext& local,
                        const NfpOptions& options,
                        NfpCoverScratch& scratch,
                        GeometryResult<NfpCover>& slot) noexcept
        {
            if (local.IsCancelled())
            {
                slot = GeometryResult<NfpCover>::Failure(GeometryStatus::Cancelled);
                return;
            }
            const PreparedNfpBatchVariant& a =
                prepared.Variants()[request.stationaryVariant];
            const PreparedNfpBatchVariant& b =
                prepared.Variants()[request.movingVariant];
            try
            {
                slot = ComposeQueryProvenance(
                    ComputeCover(*a.stationary, *b.reflected, local, options,
                                 &scratch),
                    a.provenance, b.provenance);
            }
            catch (const std::bad_alloc&)
            {
                slot = GeometryResult<NfpCover>::Failure(GeometryStatus::OutOfMemory);
            }
            catch (...)
            {
                // Classified: the only catch-all that SWALLOWS in the production kernel.
                // It exists because this frame runs on a worker thread, where an escaping
                // exception is std::terminate for the whole process. It is deliberately
                // the LAST resort after bad_alloc is handled distinctly, and it cannot
                // mask a programming error into a wrong answer - the pair is reported
                // failed, never silently succeeded, and no other pair is affected.
                //
                // V8.1 called this "the only catch-all in the production kernel". That
                // stopped being true in V8.1.1: GeometryExecutor.cpp:55 added one, and
                // Clipper2Backend.cpp has two. The distinction that still holds, and the
                // one that matters, is that this is the only one that does not rethrow -
                // the executor one restores the invariant and rethrows, and the backend
                // pair guard destructors that are not declared noexcept.
                slot = GeometryResult<NfpCover>::Failure(
                    GeometryStatus::NumericalFailure);
            }
        }

        // Chunk body shared by the executor path. One virtual call per chunk; the
        // per-pair loop is right here, inlineable, with no indirection.
        class CoverBatchBody final : public GeometryExecutor::ChunkBody
        {
        public:
            CoverBatchBody(const PreparedNfpBatch& prepared,
                           const std::vector<NfpBatchPair>& pairs,
                           const GeometryContext& context,
                           const NfpOptions& options,
                           std::vector<GeometryResult<NfpCover>>& results,
                           bool serial,
                           std::size_t workerCount)
                : prepared_(prepared), pairs_(pairs), options_(options),
                  results_(results), local_(MakeWorkerContext(context, serial)),
                  scratch_(workerCount) {}

            void Run(std::size_t worker, std::size_t begin,
                     std::size_t end) noexcept override
            {
                // One scratch per worker index, so two workers never touch the same
                // buffers and no synchronisation is needed to reuse them.
                NfpCoverScratch& scratch =
                    scratch_[worker < scratch_.size() ? worker : 0];
                for (std::size_t index = begin; index < end; ++index)
                {
                    // Fixed output slot per request index: which worker ran it, and in
                    // what order chunks were claimed, cannot affect the result order.
                    RunOnePair(prepared_, pairs_[index], local_, options_, scratch,
                               results_[index]);
                }
            }

        private:
            const PreparedNfpBatch& prepared_;
            const std::vector<NfpBatchPair>& pairs_;
            const NfpOptions& options_;
            std::vector<GeometryResult<NfpCover>>& results_;
            GeometryContext local_;
            std::vector<NfpCoverScratch> scratch_;
        };
    }

    GeometryResult<NfpCoverBatchResult> ComputeNfpCoverBatch(
        const PreparedNfpBatch& prepared,
        const std::vector<NfpBatchPair>& pairs,
        std::size_t workerCount,
        const GeometryContext& context,
        const NfpOptions& options)
    {
        const GeometryStatus admission =
            ValidateBatchRequest(prepared, pairs, workerCount, context);
        if (admission != GeometryStatus::Success)
            return GeometryResult<NfpCoverBatchResult>::Failure(admission);

        NfpCoverBatchResult batch;
        batch.results.resize(pairs.size());
        if (pairs.empty())
            return GeometryResult<NfpCoverBatchResult>::Empty(std::move(batch));

        batch.workersUsed = (std::min)(workerCount, pairs.size());
        CountStat(context, &GeometryStatistics::nfpBatchRuns);
        CountStat(context, &GeometryStatistics::nfpBatchPairs, pairs.size());

        // The one-shot form owns a private executor for the duration of the call. The
        // threads still get created once per call - that is what "one-shot" means - but
        // the scheduling, chunking and failure isolation are now the same code the
        // reusable path runs, so the two can never diverge in behaviour.
        GeometryExecutor executor(batch.workersUsed);
        CoverBatchBody body(prepared, pairs, context, options, batch.results,
                            batch.workersUsed == 1, executor.WorkerCount());
        executor.Run(pairs.size(), kDefaultNfpBatchChunkSize, body);

        return GeometryResult<NfpCoverBatchResult>::Success(std::move(batch));
    }

    GeometryResult<NfpCoverBatchResult> ComputeNfpCoverBatch(
        const PreparedNfpBatch& prepared,
        const std::vector<NfpBatchPair>& pairs,
        GeometryExecutor& executor,
        const GeometryContext& context,
        const NfpOptions& options,
        std::size_t chunkSize)
    {
        NfpCoverBatchResult batch;
        const GeometryStatus status = ComputeNfpCoverBatchInto(
            prepared, pairs, executor, context, batch, options, chunkSize);
        if (!IsSuccess(status))
            return GeometryResult<NfpCoverBatchResult>::Failure(status);
        if (status == GeometryStatus::Empty)
            return GeometryResult<NfpCoverBatchResult>::Empty(std::move(batch));
        return GeometryResult<NfpCoverBatchResult>::Success(std::move(batch));
    }

    GeometryStatus ComputeNfpCoverBatchInto(
        const PreparedNfpBatch& prepared,
        const std::vector<NfpBatchPair>& pairs,
        GeometryExecutor& executor,
        const GeometryContext& context,
        NfpCoverBatchResult& out,
        const NfpOptions& options,
        std::size_t chunkSize)
    {
        const GeometryStatus admission =
            ValidateBatchRequest(prepared, pairs, executor.WorkerCount(), context);
        if (admission != GeometryStatus::Success)
        {
            // Defined state on rejection: a caller reusing storage must never read a
            // previous generation's results after a failed call.
            out.results.clear();
            out.workersUsed = 0;
            return admission;
        }

        // resize() keeps the existing capacity, which is the whole point of Into(): a
        // caller running one batch per generation stops reallocating. Every slot is
        // then overwritten below, so no stale result can survive.
        out.results.resize(pairs.size());
        if (pairs.empty())
        {
            out.workersUsed = 0;
            return GeometryStatus::Empty;
        }

        out.workersUsed = (std::min)(executor.WorkerCount(), pairs.size());
        CountStat(context, &GeometryStatistics::nfpBatchRuns);
        CountStat(context, &GeometryStatistics::nfpBatchPairs, pairs.size());

        CoverBatchBody body(prepared, pairs, context, options, out.results,
                            out.workersUsed == 1, executor.WorkerCount());
        executor.Run(pairs.size(), chunkSize, body);
        return GeometryStatus::Success;
    }
}
