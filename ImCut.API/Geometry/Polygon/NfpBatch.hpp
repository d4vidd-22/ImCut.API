#pragma once

#include "../GeometryExecutor.hpp"
#include "../GeometrySession.hpp"
#include "NfpCover.hpp"

#include <cstddef>
#include <vector>

namespace ImCut::Geometry
{
    namespace Nfp
    {
        // Chunk size for the shared work cursor, chosen by measurement, not by guess:
        // WP10 swept 1/4/8/16/32 against 1/2/4/6/8/12 workers over 435 real pairs,
        // three runs each (evidence 426-428).
        //
        // 1 won at every worker count. Medians (ms): at 4 workers 2.504 against 2.750
        // for chunk 4; at 8, 1.730 against 1.805; at 12, 1.533 against 1.843. Coarse
        // chunks lost steadily - chunk 32 cost 19% at 8 workers and 39% at 12.
        //
        // The reason is the cost distribution, not the synchronisation: a pair here
        // takes on the order of 20us and per-pair cost varies about tenfold with
        // geometry complexity, so one relaxed fetch_add per pair is noise while a
        // coarse chunk strands a worker holding the expensive tail. An initial guess of
        // 4 was measured and rejected.
        inline constexpr std::size_t kDefaultNfpBatchChunkSize = 1;
        struct PreparedNfpBatchVariant
        {
            std::size_t shapeIndex = 0;
            std::size_t rotationIndex = 0;
            double rotationRadians = 0.0;
            PreparedNfpOperandPtr stationary{};
            PreparedNfpOperandPtr reflected{};
            ErrorBudget provenance{};
        };

        // Owns immutable orientation variants. It deliberately owns no session and no
        // worker pool: preparation happens while the caller-owned session is present;
        // pair execution later reads only the shared immutable operands.
        class PreparedNfpBatch
        {
        public:
            [[nodiscard]] std::size_t ShapeCount() const noexcept { return shapeCount_; }
            [[nodiscard]] std::size_t RotationCount() const noexcept
            {
                return rotationCount_;
            }
            [[nodiscard]] std::size_t VariantCount() const noexcept
            {
                return variants_.size();
            }
            [[nodiscard]] const std::vector<PreparedNfpBatchVariant>& Variants() const noexcept
            {
                return variants_;
            }
            [[nodiscard]] const PreparedNfpBatchVariant* Variant(
                std::size_t shapeIndex, std::size_t rotationIndex) const noexcept;
            [[nodiscard]] std::size_t ApproximateBytes() const noexcept;

        private:
            std::size_t shapeCount_ = 0;
            std::size_t rotationCount_ = 0;
            std::vector<PreparedNfpBatchVariant> variants_{};

            friend GeometryResult<PreparedNfpBatch> PrepareNfpBatch(
                GeometrySession&, const std::vector<const PreparedShapeDefinition*>&,
                const std::vector<double>&, const NfpOptions&,
                const GeometryQueryControl&);
            friend GeometryResult<PreparedNfpBatch> PrepareNfpBatch(
                GeometrySession&, const std::vector<NfpQueryOperand>&,
                const std::vector<double>&, const NfpOptions&,
                const GeometryQueryControl&);
        };

        struct NfpBatchPair
        {
            // Indices into PreparedNfpBatch::Variants(). The first operand uses the
            // stationary derivative and the second uses its reflected derivative.
            std::size_t stationaryVariant = 0;
            std::size_t movingVariant = 0;
        };

        struct NfpCoverBatchResult
        {
            // One result per request, always in request order regardless of scheduling.
            std::vector<GeometryResult<NfpCover>> results{};
            std::size_t workersUsed = 0;

            [[nodiscard]] bool AllSucceeded() const noexcept;
            [[nodiscard]] std::size_t ApproximateBytes() const noexcept;
        };

        // Prepares every shape/rotation exactly once per reflection state: 2*N*R
        // immutable derivatives, never N^2*R^2 pair objects. GeometrySession is
        // intentionally used by one caller thread because its mutable caches are
        // worker-owned.
        [[nodiscard]] GeometryResult<PreparedNfpBatch> PrepareNfpBatch(
            GeometrySession& session,
            const std::vector<const PreparedShapeDefinition*>& definitions,
            const std::vector<double>& rotations,
            const NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        [[nodiscard]] GeometryResult<PreparedNfpBatch> PrepareNfpBatch(
            GeometrySession& session,
            const std::vector<NfpQueryOperand>& definitions,
            const std::vector<double>& rotations,
            const NfpOptions& options = {},
            const GeometryQueryControl& control = {});

        // Explicit caller-controlled parallelism. No threads are created by primitive
        // NFP APIs. Shared operands are immutable; each request writes one fixed output
        // slot, preserving deterministic result order.
        //
        // This one-shot form creates and destroys its workers per call. It is kept for
        // convenience and for callers that run a batch once; a caller running many
        // generations should own a GeometryExecutor and use the overload below.
        [[nodiscard]] GeometryResult<NfpCoverBatchResult> ComputeNfpCoverBatch(
            const PreparedNfpBatch& prepared,
            const std::vector<NfpBatchPair>& pairs,
            std::size_t workerCount,
            const GeometryContext& context,
            const NfpOptions& options = {});

        // Reuses the caller's executor instead of creating threads. Same semantics,
        // same deterministic output order, same per-pair failure isolation.
        [[nodiscard]] GeometryResult<NfpCoverBatchResult> ComputeNfpCoverBatch(
            const PreparedNfpBatch& prepared,
            const std::vector<NfpBatchPair>& pairs,
            GeometryExecutor& executor,
            const GeometryContext& context,
            const NfpOptions& options = {},
            std::size_t chunkSize = kDefaultNfpBatchChunkSize);

        // Writes into storage the caller already owns, so a nesting run doing one batch
        // per generation stops reallocating the result vector every time. `out.results`
        // is resized to pairs.size() and every slot is overwritten, so nothing from a
        // previous generation can survive into this one.
        //
        // Returns the status the equivalent value-returning call would return; `out` is
        // left in a defined (cleared) state on rejection.
        [[nodiscard]] GeometryStatus ComputeNfpCoverBatchInto(
            const PreparedNfpBatch& prepared,
            const std::vector<NfpBatchPair>& pairs,
            GeometryExecutor& executor,
            const GeometryContext& context,
            NfpCoverBatchResult& out,
            const NfpOptions& options = {},
            std::size_t chunkSize = kDefaultNfpBatchChunkSize);
    }
}
