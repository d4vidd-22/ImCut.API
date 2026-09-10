#include "Boolean.hpp"

#include "PolygonBackend.hpp"
#include "PolygonConversion.hpp"

namespace ImCut::Geometry::Boolean
{
    namespace
    {
        [[nodiscard]] Backend::Operation ToBackend(BooleanOperation operation) noexcept
        {
            switch (operation)
            {
                case BooleanOperation::Intersection: return Backend::Operation::Intersection;
                case BooleanOperation::Difference:   return Backend::Operation::Difference;
                case BooleanOperation::Xor:          return Backend::Operation::Xor;
                case BooleanOperation::Union:
                default:                             return Backend::Operation::Union;
            }
        }

        [[nodiscard]] Backend::Fill ToBackend(FillRule fillRule) noexcept
        {
            return fillRule == FillRule::NonZero ? Backend::Fill::NonZero : Backend::Fill::EvenOdd;
        }

        // An explicit EvenOdd/NonZero is an override the caller asked for and applies to
        // both operands. UseSource is different: it means each operand keeps the rule it
        // arrived with, which is only expressible as a single backend fill when the two
        // rules already agree.
        [[nodiscard]] FillRule ResolveFillRule(BooleanFillRule requested, const Path& subject) noexcept
        {
            switch (requested)
            {
                case BooleanFillRule::EvenOdd: return FillRule::EvenOdd;
                case BooleanFillRule::NonZero: return FillRule::NonZero;
                case BooleanFillRule::UseSource:
                default:                       return subject.fillRule;
            }
        }

        // Resolves one operand's rings under ITS OWN fill rule.
        //
        // The backend takes a single fill for both operands, so passing the subject rule
        // reinterpreted the clip as well. An EvenOdd clip whose inner ring wound the same
        // way as its outer one is a ring with a hole; read as NonZero it is a solid, and
        // the hole simply vanished from the result - the audit measured area 10000 where
        // 7500 was correct.
        //
        // Unioning an operand against nothing under its own rule turns "these rings mean
        // this region" into an explicit outer/hole set. The backend emits those with
        // outer and hole orientations opposed, which is unambiguous under NonZero, so the
        // combining pass can use one canonical rule for both operands.
        [[nodiscard]] Backend::BackendStatus NormaliseOperand(Backend::IntRingsView rings,
                                                              FillRule rule,
                                                              Backend::IntRings& out)
        {
            if (rings.empty())
            {
                out.clear();
                return Backend::BackendStatus::Success;
            }

            return Backend::Default().ExecuteFlat(
                Backend::Operation::Union, ToBackend(rule), rings,
                Backend::IntRingsView{}, out);
        }

        // A backend outcome becomes a kernel status here and nowhere else, so the
        // DepthLimit / NumericalFailure distinction cannot be lost by a caller that
        // only tested for "not success".
        [[nodiscard]] GeometryStatus ToKernelStatus(Backend::BackendStatus status) noexcept
        {
            switch (status)
            {
                case Backend::BackendStatus::Success:     return GeometryStatus::Success;
                case Backend::BackendStatus::DepthLimit:  return GeometryStatus::ComplexityLimit;
                case Backend::BackendStatus::OutOfMemory: return GeometryStatus::OutOfMemory;
                case Backend::BackendStatus::Failed:
                default:                                  return GeometryStatus::NumericalFailure;
            }
        }

        // Region operations are undefined on open contours, and silently closing one
        // fabricates filled area. Structural validity is checked in the same pass so
        // nothing downstream indexes a broken contour.
        [[nodiscard]] GeometryStatus ScreenOperand(const Path& path) noexcept
        {
            if (!path.IsStructurallyValid())
                return GeometryStatus::InvalidInput;
            if (path.HasOpenContours())
                return GeometryStatus::InvalidTopology;
            return GeometryStatus::Success;
        }

        // Empty with a real budget, published to the payload and mirrored on the wrapper -
        // the ownership rule the success route at the bottom of Run() already follows.
        [[nodiscard]] GeometryResult<Path> EmptyWithBudget(const ErrorBudget& budget)
        {
            Path payload;
            payload.budget = budget;
            auto empty = GeometryResult<Path>::Empty(std::move(payload));
            empty.Budget() = budget;
            return empty;
        }

        [[nodiscard]] GeometryResult<Path> Run(BooleanOperation operation,
                                               const Path& subject, const Path& clip,
                                               const GeometryContext& context,
                                               const BooleanOptions& options,
                                               BooleanReport* report)
        {
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::ValidationExclusive);
                if (const GeometryStatus screened = ScreenOperand(subject); !IsSuccess(screened))
                    return GeometryResult<Path>::Failure(screened);
                if (const GeometryStatus screened = ScreenOperand(clip); !IsSuccess(screened))
                    return GeometryResult<Path>::Failure(screened);
            }

            const FillRule fillRule = ResolveFillRule(options.fillRule, subject);

            // Both operands share one lattice so their integers are comparable.
            const Path* operands[2] = { &subject, &clip };
            GeometryResult<Convert::ConversionResult> conversion;
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::BooleanConversionExclusive);
                conversion = Convert::PathsToIntegers(operands, 2, context);
            }

            // AN EARLY EMPTY STILL OWES THE CALLER THE OPERANDS' OWN ERROR.
            //
            // V8.1 returned a default Path here, so Simplify() of a lossy-but-empty path
            // came back declaring exact and zero: measured 0.0 against an input carrying
            // 0.51 mm (evidence 857). Empty is the assertion "no point qualifies", and it
            // was reached from geometry the caller said it only knows to some tolerance.
            //
            // Merge, not MergeSequential, for the same reason the success route uses it a
            // few lines below: a point of a Boolean result comes from one operand's
            // boundary or the other's, never both, so the two are alternatives.
            ErrorBudget operandBudget = subject.budget;
            operandBudget.Merge(clip.budget);

            if (conversion.Status() == GeometryStatus::Empty)
                return EmptyWithBudget(operandBudget);

            if (!conversion.Ok())
                return GeometryResult<Path>::Failure(conversion.Status());

            const Convert::ConversionResult& converted = conversion.Value();

            const Backend::IntRingsView subjectRings = converted.RingsOf(0);
            const Backend::IntRingsView clipRings = converted.RingsOf(1);

            if (report != nullptr)
            {
                report->inputRings = subjectRings.size() + clipRings.size();
                report->quantizationStep = converted.scale.resolution;
            }

            if (subjectRings.empty())
            {
                // Nothing to clip. Union still has to yield the clip side.
                if (operation != BooleanOperation::Union && operation != BooleanOperation::Xor)
                {
                    // The conversion did run here, so its quantisation counts as well.
                    ErrorBudget emptyBudget = operandBudget;
                    emptyBudget.MergeSequential(converted.budget);
                    return EmptyWithBudget(emptyBudget);
                }
            }

            // Fill semantics belong to each operand, not to the pair.
            //
            // With UseSource and two different source rules there is no single backend
            // fill that expresses both, so each side is resolved under its own rule first
            // and the combination then runs under one canonical rule. When the rules
            // already agree - the overwhelmingly common case, and every explicit
            // override - the single pass below is exactly equivalent and costs nothing
            // extra.
            const bool perOperandRules = options.fillRule == BooleanFillRule::UseSource &&
                                         subject.fillRule != clip.fillRule;

            const Backend::TreeLimits treeLimits{ context.Limits().maxNestingDepth };

            Backend::PolyTree tree;
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::BooleanBackendExclusive);
                if (perOperandRules)
                {
                    Backend::IntRings normalisedSubject;
                    Backend::IntRings normalisedClip;

                    if (const auto status = NormaliseOperand(subjectRings, subject.fillRule,
                                                             normalisedSubject);
                        status != Backend::BackendStatus::Success)
                    {
                        return GeometryResult<Path>::Failure(ToKernelStatus(status));
                    }
                    if (const auto status = NormaliseOperand(clipRings, clip.fillRule,
                                                             normalisedClip);
                        status != Backend::BackendStatus::Success)
                    {
                        return GeometryResult<Path>::Failure(ToKernelStatus(status));
                    }

                    if (const auto status =
                            Backend::Default().Execute(ToBackend(operation), Backend::Fill::NonZero,
                                                       normalisedSubject, normalisedClip,
                                                       treeLimits, tree);
                        status != Backend::BackendStatus::Success)
                    {
                        return GeometryResult<Path>::Failure(ToKernelStatus(status));
                    }
                }
                else if (const auto status =
                             Backend::Default().Execute(ToBackend(operation), ToBackend(fillRule),
                                                        subjectRings, clipRings, treeLimits, tree);
                         status != Backend::BackendStatus::Success)
                {
                    return GeometryResult<Path>::Failure(ToKernelStatus(status));
                }
            }

            // The per-operand path emits an explicit outer/hole set, which is a NonZero
            // region regardless of what the operands started as. Labelling it with the
            // subject rule would misdescribe the geometry that was actually produced.
            const FillRule outputRule = perOperandRules ? FillRule::NonZero : fillRule;
            Convert::ResultValidation validation;
            Path output;
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::ResultConversionExclusive);
                output = Convert::PolyTreeToPath(tree, converted.frame, outputRule, context);
                if (options.pruneDegenerate)
                    validation = Convert::ValidateAndPrune(output, context);
            }

            if (report != nullptr)
            {
                report->outputRings = output.contours.size();
                report->prunedDegenerate = validation.degenerateRings;
                report->prunedEmpty = validation.emptyRings;
                report->nonFiniteRemoved = validation.nonFinite;

                // Input rings existed and none survived: whatever the operation meant to
                // produce, the lattice is what removed it. Union and Xor of non-empty
                // operands cannot legitimately be empty, and for Intersection and
                // Difference an empty result IS legitimate, so the flag says "the
                // topology changed", not "this is wrong" - the caller decides.
                const bool annihilated = report->inputRings > 0 && output.contours.empty() &&
                                         (operation == BooleanOperation::Union ||
                                          operation == BooleanOperation::Xor);
                report->topologyChanged = annihilated;
            }

            CountStat(context, &GeometryStatistics::booleanOperations);

            // The operands may already carry error from earlier stages; this one adds
            // to it rather than replacing it.
            ErrorBudget budget;
            {
                ScopedGeometryStage stage(
                    context.Diagnostics(), GeometryDiagnosticStage::BudgetExclusive);
                budget = subject.budget;
                budget.Merge(clip.budget);
                budget.MergeSequential(converted.budget);
                output.budget = budget;
            }

            if (output.contours.empty())
            {
                auto empty = GeometryResult<Path>::Empty(std::move(output));
                empty.Budget().MergeSequential(budget);
                return empty;
            }

            auto success = GeometryResult<Path>::Success(std::move(output));
            success.Budget().MergeSequential(budget);
            return success;
        }
    }

    GeometryResult<Path> Execute(BooleanOperation operation, const Path& subject, const Path& clip,
                                 const GeometryContext& context, const BooleanOptions& options,
                                 BooleanReport* report)
    {
        return Run(operation, subject, clip, context, options, report);
    }

    GeometryResult<Path> Union(const Path& a, const Path& b, const GeometryContext& context,
                               const BooleanOptions& options)
    {
        return Run(BooleanOperation::Union, a, b, context, options, nullptr);
    }

    GeometryResult<Path> Intersection(const Path& a, const Path& b, const GeometryContext& context,
                                      const BooleanOptions& options)
    {
        return Run(BooleanOperation::Intersection, a, b, context, options, nullptr);
    }

    GeometryResult<Path> Difference(const Path& a, const Path& b, const GeometryContext& context,
                                    const BooleanOptions& options)
    {
        return Run(BooleanOperation::Difference, a, b, context, options, nullptr);
    }

    GeometryResult<Path> Xor(const Path& a, const Path& b, const GeometryContext& context,
                             const BooleanOptions& options)
    {
        return Run(BooleanOperation::Xor, a, b, context, options, nullptr);
    }

    GeometryResult<Path> Simplify(const Path& path, const GeometryContext& context,
                                  const BooleanOptions& options)
    {
        return Run(BooleanOperation::Union, path, Path{}, context, options, nullptr);
    }

    const char* BackendName() noexcept { return Backend::Default().Name(); }
}
