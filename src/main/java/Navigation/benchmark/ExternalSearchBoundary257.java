package Navigation.benchmark;

import Navigation.Navigation3D.fsa.TruthBudgetStop211;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/** One launcher-owned terminal boundary. Source-only until native checks pass. */
public final class ExternalSearchBoundary257 {
    public interface ScopeClose { void close() throws Exception; }
    public interface ScopeOpen { ScopeClose open() throws Exception; }
    public interface Search { void run() throws Exception; }
    public interface FinalAudit { void verify(RouteIncumbent249.Snapshot candidate) throws Exception; }
    public interface Phases {
        void search() throws Exception;
        void finalAudit() throws Exception;
        void done() throws Exception;
    }
    public enum State { READY, RUNNING, VERIFIED_COMPLETE, VERIFIED_GLOBAL_STOP, FAILED }
    private final RouteIncumbent249 incumbent;
    private final RouteIncumbent249.Identity identity;
    private final ExternalRun166 run;
    private final Thread owner=Thread.currentThread();
    private State state=State.READY;
    private Throwable failure;
    private String stopReason;
    private boolean searchEntered,scopeClosed;

    public ExternalSearchBoundary257(RouteIncumbent249 incumbent,RouteIncumbent249.Identity identity,ExternalRun166 run){
        this.incumbent=Objects.requireNonNull(incumbent,"INCUMBENT257_NULL");
        this.identity=Objects.requireNonNull(identity,"IDENTITY257_NULL");
        this.run=Objects.requireNonNull(run,"RUN257_NULL");
        RouteIncumbent249.Snapshot seed=incumbent.snapshot();
        if(!identity.equals(seed.identity)||seed.state!=RouteIncumbent249.State.OPEN||!"seed".equals(seed.candidateId)
                ||Double.doubleToLongBits(seed.objective)!=Double.doubleToLongBits(run.seedCost)||run.finished())
            throw new IllegalArgumentException("FRESH_COMPLETE_SEED257_REQUIRED");
        for(Long count:seed.observations.values())if(count!=0L)throw new IllegalArgumentException("REUSED257_INCUMBENT");
    }
    private static Throwable suppressed(Throwable first,Throwable next){
        if(first==null)return next;
        if(next==null)return first;
        if(first!=next)first.addSuppressed(next);return first;
    }
    private static TruthBudgetStop211 globalControl(Throwable error){
        return error instanceof RuntimeException?TruthBudgetStop211.find(error):null;
    }
    private void fail(Throwable error) throws Exception {
        state=State.FAILED;failure=Objects.requireNonNull(error,"FAILURE257_NULL");
        //249 can represent RuntimeException. Fatal/checked errors retain their exact type;
        //the OPEN warehouse is deliberately not promoted and result() remains forbidden.
        if(error instanceof RuntimeException){
            if(incumbent.snapshot().state==RouteIncumbent249.State.OPEN)incumbent.contractFailure(identity,(RuntimeException)error);
            throw (RuntimeException)error;
        }
        if(error instanceof Error)throw (Error)error;
        if(error instanceof Exception)throw (Exception)error;
        throw new AssertionError("UNEXPECTED257_THROWABLE",error);
    }
    public RouteIncumbent249.Snapshot execute(ScopeOpen scopeOpen,Search search,Phases phases,FinalAudit audit) throws Exception {
        if(owner!=Thread.currentThread()||state!=State.READY)throw new IllegalStateException("BOUNDARY257_OWNER_OR_REUSE");
        Objects.requireNonNull(scopeOpen,"SCOPE257_NULL");Objects.requireNonNull(search,"SEARCH257_NULL");
        Objects.requireNonNull(phases,"PHASES257_NULL");Objects.requireNonNull(audit,"AUDIT257_NULL");
        state=State.RUNNING;ScopeClose scope=null;Throwable searchError=null,cleanupError=null;
        try{
            scope=Objects.requireNonNull(scopeOpen.open(),"SCOPE257_OPEN_NULL");
            phases.search();searchEntered=true;
            search.run();
        }catch(Throwable error){searchError=error;}
        finally{
            //Do not restart a clock or call the non-idempotent legacy run.finish().
            //The original algorithms own normal finish; typed stop may occur before run.start().
            try{run.ledger.setActive(false);}catch(Throwable error){cleanupError=error;}
            if(scope!=null){
                try{scope.close();scopeClosed=true;}
                catch(Throwable error){cleanupError=suppressed(cleanupError,error);}
            }
        }
        //A cleanup failure must never be consumed as a legitimate global budget signal.
        if(cleanupError!=null){
            if(searchError!=null&&(!searchEntered||globalControl(searchError)==null)){
                fail(suppressed(searchError,cleanupError));return null;
            }
            fail(suppressed(cleanupError,searchError));return null;
        }
        TruthBudgetStop211 stop=searchEntered?globalControl(searchError):null;
        if(searchError!=null&&stop==null){fail(searchError);return null;}
        stopReason=stop==null?null:stop.getMessage();
        if(stop!=null&&(stopReason==null||stopReason.trim().isEmpty())){
            fail(new IllegalStateException("EMPTY_TYPED257_STOP_REASON",searchError));return null;
        }
        try{
            //The complete native champion is read after closing the search scope.
            //FINAL_AUDIT must use the original native evaluator, not the active Fair wrapper.
            RouteIncumbent249.Snapshot candidate=incumbent.snapshot();
            phases.finalAudit();audit.verify(candidate);phases.done();
            RouteIncumbent249.Snapshot after=incumbent.snapshot();
            if(after.state!=RouteIncumbent249.State.OPEN||!after.identity.equals(candidate.identity)
                    ||!after.candidateId.equals(candidate.candidateId)
                    ||Double.doubleToLongBits(after.objective)!=Double.doubleToLongBits(candidate.objective))
                throw new IllegalStateException("FINAL_AUDIT257_CHANGED_INCUMBENT");
        }catch(Throwable error){fail(suppressed(error,searchError));return null;}
        if(stop==null){incumbent.complete(identity);state=State.VERIFIED_COMPLETE;}
        else{incumbent.globalStop(identity,stopReason);state=State.VERIFIED_GLOBAL_STOP;}
        return incumbent.resultForFinalAudit();
    }
    public RouteIncumbent249.Snapshot result(){
        if(state!=State.VERIFIED_COMPLETE&&state!=State.VERIFIED_GLOBAL_STOP)throw new IllegalStateException("BOUNDARY257_NOT_VERIFIED:"+state,failure);
        return incumbent.resultForFinalAudit();
    }
    public Map<String,Object> diagnostics(){
        Map<String,Object> x=new LinkedHashMap<>();x.put("state",state.name());x.put("search_entered",searchEntered);x.put("scope_closed",scopeClosed);
        x.put("legacy_run_finished",run.finished());x.put("typed_stop_reason",stopReason);x.put("failure_class",failure==null?null:failure.getClass().getName());
        x.put("warehouse_state",incumbent.snapshot().state.name());x.put("cost_scope","Full process Job measured separately; ledger and observer are not additional Jobs.");
        return x;
    }
}
