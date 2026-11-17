package Navigation.benchmark;

import Navigation.Navigation3D.fsa.CandidateEvaluation217;
import Navigation.model.GridPoint;
import Navigation.Navigation3D.fsa.SearchWorkLedger;
import java.util.List;
import java.util.Objects;
import java.util.function.DoubleSupplier;

/** One run-owned, synchronous binding at the native five-argument evaluation boundary.
 * The supplier is the native evaluator, never the already-wrapped old Fair evaluator.
 * Its caller owns resource checks and terminal exception classification.
 */
public final class CandidateBinding250 {
    private final RouteIncumbent249 incumbent;
    private final RouteIncumbent249.Identity identity;
    private final TimedCandidateLog258 timing;

    public CandidateBinding250(RouteIncumbent249 incumbent,RouteIncumbent249.Identity identity){
        throw new IllegalStateException("TIMED258_EXPLICIT_LEDGER_REQUIRED");
    }
    public CandidateBinding250(RouteIncumbent249 incumbent,RouteIncumbent249.Identity identity,SearchWorkLedger ledger){
        this.incumbent=Objects.requireNonNull(incumbent,"INCUMBENT250_NULL");
        this.identity=Objects.requireNonNull(identity,"IDENTITY250_NULL");
        requireOpen(identity);
        timing=TimedObservationScope258.current(incumbent,identity,ledger);
    }
    private void requireOpen(RouteIncumbent249.Identity actual){
        if(!identity.equals(actual))throw new IllegalArgumentException("BINDING250_IDENTITY_MISMATCH");
        RouteIncumbent249.Snapshot before=incumbent.snapshot();
        if(!identity.equals(before.identity))throw new IllegalArgumentException("BINDING250_WAREHOUSE_IDENTITY");
        if(before.state!=RouteIncumbent249.State.OPEN)throw new IllegalStateException("BINDING250_TERMINAL:"+before.state);
    }

    /** actual must be derived from this call's objective and fixed run identities.
     * Route coordinates must remain stable for this synchronous native call.
     * A complete result is saved before the launcher may check exhausted resources.
     */
    public double evaluate(RouteIncumbent249.Identity actual,String candidateId,List<GridPoint> route,
                           DoubleSupplier nativeCall){
        requireOpen(actual);
        if(candidateId==null||candidateId.trim().isEmpty())throw new IllegalArgumentException("CANDIDATE250_ID");
        Objects.requireNonNull(nativeCall,"NATIVE250_CALL");
        TimedCandidateLog258.Token token=timing.beforeNative(actual,candidateId);
        CandidateEvaluation217.Result outcome=CandidateEvaluation217.evaluate(route,nativeCall);
        switch(outcome.status){
            case COMPLETE:
                incumbent.observe(actual,RouteIncumbent249.Outcome.COMPLETE,candidateId,route,outcome.objective);
                timing.afterNative(token,outcome.status,outcome.objective);
                return outcome.legacyScalar();
            case UNPRICED:
                incumbent.observe(actual,RouteIncumbent249.Outcome.UNPRICED,candidateId,route,null);
                timing.afterNative(token,outcome.status,outcome.objective);
                return Double.NaN;
            case NATIVE_REJECTED:
                incumbent.observe(actual,RouteIncumbent249.Outcome.NATIVE_REJECTED,candidateId,route,null);
                timing.afterNative(token,outcome.status,outcome.objective);
                // Preserve the old external adapter's rejection sentinel, never a finite cost.
                return Double.POSITIVE_INFINITY;
            default:
                // Frozen217 propagates these exceptions; they cannot be normal return values.
                throw new IllegalStateException("BINDING250_UNEXPECTED_RETURN:"+outcome.status);
        }
    }
}
