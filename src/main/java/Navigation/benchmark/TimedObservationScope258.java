package Navigation.benchmark;

import Navigation.Navigation3D.fsa.SearchWorkLedger;
import java.util.Objects;

/** Outer launcher scope. Close after257 returns/throws; never finalizes the warehouse. */
public final class TimedObservationScope258 implements AutoCloseable {
    private static final ThreadLocal<TimedObservationScope258> ACTIVE=new ThreadLocal<>();
    private final TimedCandidateLog258 log;
    private final Thread owner=Thread.currentThread();
    private boolean closed;
    private TimedObservationScope258(TimedCandidateLog258 log){this.log=Objects.requireNonNull(log,"TIMED258_LOG");}
    public static TimedObservationScope258 open(TimedCandidateLog258 log){
        if(ACTIVE.get()!=null)throw new IllegalStateException("TIMED258_NESTED_SCOPE");
        TimedObservationScope258 scope=new TimedObservationScope258(log);ACTIVE.set(scope);return scope;
    }
    static TimedCandidateLog258 current(RouteIncumbent249 incumbent,RouteIncumbent249.Identity identity,SearchWorkLedger ledger){
        TimedObservationScope258 scope=ACTIVE.get();
        if(scope==null||scope.closed||scope.owner!=Thread.currentThread())throw new IllegalStateException("TIMED258_NO_LIVE_SCOPE");
        scope.log.requireBinding(incumbent,identity,ledger);return scope.log;
    }
    @Override public void close(){
        if(closed)return;
        if(owner!=Thread.currentThread()||ACTIVE.get()!=this)throw new IllegalStateException("TIMED258_CLOSE_OWNER");
        log.seal();ACTIVE.remove();closed=true;
    }
}
