#pragma once

#include <cstdint>
#include <utility>

namespace ImCut::Geometry
{
    // Geometric outcomes are statuses, not exceptions. Exceptions stay reserved for
    // genuinely extraordinary failures (allocation, broken internal invariant).
    enum class GeometryStatus : std::uint8_t
    {
        Success,

        // A well-defined, correct, empty answer. Difference(A, A) is Empty, not a
        // failure, and callers must treat it as such.
        Empty,

        InvalidInput,
        Degenerate,
        InvalidTopology,
        NumericalFailure,
        ComplexityLimit,
        Cancelled,
        Unsupported,

        // Allocation failed.
        //
        // Distinct from ComplexityLimit, which means "this would have exceeded a limit
        // the caller set". OutOfMemory means the limits were satisfied and the machine
        // still could not provide the memory - a different problem with a different
        // response. Before this existed, std::bad_alloc escaped Boolean::Execute,
        // Offset and Minkowski uncaught while Triangulate quietly swallowed it as a
        // generic failure, so the same condition surfaced three different ways.
        OutOfMemory
    };

    [[nodiscard]] constexpr const char* ToString(GeometryStatus status) noexcept
    {
        switch (status)
        {
            case GeometryStatus::Success:          return "Success";
            case GeometryStatus::Empty:            return "Empty";
            case GeometryStatus::InvalidInput:     return "InvalidInput";
            case GeometryStatus::Degenerate:       return "Degenerate";
            case GeometryStatus::InvalidTopology:  return "InvalidTopology";
            case GeometryStatus::NumericalFailure: return "NumericalFailure";
            case GeometryStatus::ComplexityLimit:  return "ComplexityLimit";
            case GeometryStatus::Cancelled:        return "Cancelled";
            case GeometryStatus::Unsupported:      return "Unsupported";
            case GeometryStatus::OutOfMemory:      return "OutOfMemory";
        }
        return "Unknown";
    }

    // Empty counts as success: the operation ran and produced a valid answer.
    [[nodiscard]] constexpr bool IsSuccess(GeometryStatus status) noexcept
    {
        return status == GeometryStatus::Success || status == GeometryStatus::Empty;
    }

    // How much approximation the value carries, so a caller can never claim more
    // precision than the pipeline actually delivered. A cubic flattened at 0.01 mm
    // and then quantised at 0.001 mm is accurate to ~0.011 mm, not to 0.001 mm.
    struct ErrorBudget
    {
        // Worst-case chord deviation introduced by curve flattening (mm).
        double flattenTolerance = 0.0;

        // Worst-case displacement introduced by coordinate quantisation (mm).
        double quantizationStep = 0.0;

        // False once any lossy stage has run.
        bool exact = true;

        [[nodiscard]] constexpr double Total() const noexcept
        {
            return flattenTolerance + quantizationStep;
        }

        // Two flavours, and using the wrong one understates the real error.
        //
        // Add*    - alternatives through the SAME stage. Only one of them happens, so
        //           the worst case is the maximum.
        // AddSequential* - stages that run one after another. Both displacements happen
        //           and they compose, so the worst case is the sum.
        //
        // Flattening a curve and then flattening the result again is sequential; taking
        // the maximum there would report a single stage's error for a two-stage pipeline.

        constexpr void AddFlatten(double tolerance) noexcept
        {
            if (tolerance > flattenTolerance) flattenTolerance = tolerance;
            if (tolerance > 0.0) exact = false;
        }

        constexpr void AddSequentialFlatten(double tolerance) noexcept
        {
            if (tolerance > 0.0)
            {
                flattenTolerance += tolerance;
                exact = false;
            }
        }

        constexpr void AddQuantization(double step) noexcept
        {
            if (step > quantizationStep) quantizationStep = step;
            if (step > 0.0) exact = false;
        }

        constexpr void AddSequentialQuantization(double step) noexcept
        {
            if (step > 0.0)
            {
                quantizationStep += step;
                exact = false;
            }
        }

        // Worst of two alternative paths.
        constexpr void Merge(const ErrorBudget& other) noexcept
        {
            AddFlatten(other.flattenTolerance);
            AddQuantization(other.quantizationStep);
            if (!other.exact) exact = false;
        }

        // Error of a stage applied on top of this one.
        constexpr void MergeSequential(const ErrorBudget& other) noexcept
        {
            AddSequentialFlatten(other.flattenTolerance);
            AddSequentialQuantization(other.quantizationStep);
            if (!other.exact) exact = false;
        }
    };

    // Fired when Value() is read on a result that carries none. Reading past a failure
    // used to hand back a default-constructed T, which is indistinguishable from a real
    // answer - an empty Path returned for a NumericalFailure looked exactly like a
    // legitimately empty result.
    [[noreturn]] void OnResultInvariantViolation(GeometryStatus status, const char* what) noexcept;

    // Status plus payload. Deliberately not a monad: geometry code reads better when the
    // caller checks the status and then uses the value.
    //
    // A value exists only for Success and Empty. Every other status has no value at all,
    // and asking for one is an invariant violation rather than a silent default.
    template <typename T>
    class [[nodiscard]] GeometryResult
    {
    public:
        // A default-constructed result is deliberately NOT a success. Somewhere between
        // "declared but not yet assigned" and "valid empty answer" is exactly where a
        // swallowed failure hides.
        GeometryResult() = default;

        [[nodiscard]] static GeometryResult Success(T value)
        {
            GeometryResult result;
            result.status_ = GeometryStatus::Success;
            result.value_ = std::move(value);
            result.hasValue_ = true;
            return result;
        }

        [[nodiscard]] static GeometryResult Empty(T value = T{})
        {
            GeometryResult result;
            result.status_ = GeometryStatus::Empty;
            result.value_ = std::move(value);
            result.hasValue_ = true;
            return result;
        }

        [[nodiscard]] static GeometryResult Failure(GeometryStatus status)
        {
            GeometryResult result;
            // Guard against constructing a "failure" that is actually a success status.
            result.status_ = IsSuccess(status) ? GeometryStatus::NumericalFailure : status;
            result.hasValue_ = false;
            return result;
        }

        [[nodiscard]] GeometryStatus Status() const noexcept { return status_; }
        [[nodiscard]] bool Ok() const noexcept { return IsSuccess(status_) && hasValue_; }
        [[nodiscard]] bool HasValue() const noexcept { return hasValue_; }
        [[nodiscard]] explicit operator bool() const noexcept { return Ok(); }

        [[nodiscard]] const T& Value() const&
        {
            if (!hasValue_) OnResultInvariantViolation(status_, "Value() on a result without a value");
            return value_;
        }

        [[nodiscard]] T& Value() &
        {
            if (!hasValue_) OnResultInvariantViolation(status_, "Value() on a result without a value");
            return value_;
        }

        [[nodiscard]] T&& Value() &&
        {
            if (!hasValue_) OnResultInvariantViolation(status_, "Value() on a result without a value");
            return std::move(value_);
        }

        // The only way to read past a failure, and it says so at the call site.
        [[nodiscard]] T ValueOr(T fallback) const&
        {
            return hasValue_ ? value_ : std::move(fallback);
        }

        [[nodiscard]] const ErrorBudget& Budget() const noexcept { return budget_; }
        [[nodiscard]] ErrorBudget& Budget() noexcept { return budget_; }

    private:
        GeometryStatus status_ = GeometryStatus::InvalidInput;
        T value_{};
        ErrorBudget budget_{};
        bool hasValue_ = false;
    };
}
