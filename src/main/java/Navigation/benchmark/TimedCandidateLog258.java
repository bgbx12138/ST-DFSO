package Navigation.benchmark;

import Navigation.Navigation3D.fsa.CandidateEvaluation217;
import Navigation.Navigation3D.fsa.FullRouteAudit211;
import Navigation.Navigation3D.fsa.SearchWorkLedger;
import Navigation.model.GridPoint;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.LongSupplier;

/** Observation only. No native call, clock reset, fee, phase change or terminal decision. */
public final class TimedCandidateLog258 {
    public static final class Token {
        private final TimedCandidateLog258 owner;
        public final long sequence,nativeBeforeNano;
        public final String candidateId;
        private Token(TimedCandidateLog258 owner,long sequence,long nano,String candidateId){
            this.owner=owner;this.sequence=sequence;nativeBeforeNano=nano;this.candidateId=candidateId;
        }
    }
    private static final class Point {
        final long sequence,lower,upper,work;
        final String id;
        final double objective;
        final int[][] coordinates;
        Point(Token token,long lower,long upper,long work,double value,int[][] coordinates){
            sequence=token.sequence;id=token.candidateId;this.lower=lower;this.upper=upper;
            this.work=work;objective=value;this.coordinates=copy(coordinates);
        }
        Map<String,Object> map(){
            Map<String,Object> x=new LinkedHashMap<>();x.put("attempt_sequence",sequence);x.put("candidate_id",id);
            x.put("nano_lower",lower);x.put("nano_upper",upper);x.put("subsegments",work);
            x.put("objective",objective);x.put("coordinates",copy(coordinates));return x;
        }
    }
    private final RouteIncumbent249 incumbent;
    private final RouteIncumbent249.Identity identity;
    private final SearchWorkLedger ledger;
    private final Thread owner=Thread.currentThread();
    private final LongSupplier clock;
    private final String clockSource,bindingId;
    private final int capacity;
    private final long pid=ProcessHandle.current().pid();
    private final double seedCost;
    private final int[][] seedCoordinates;
    private final List<Point> points=new ArrayList<>();
    private long attempts,completed,unpriced,rejected,previousNano=Long.MIN_VALUE;
    private double bestCost;
    private String bestId="seed";
    private Token pending;
    private boolean sealed;

    public TimedCandidateLog258(RouteIncumbent249 incumbent,RouteIncumbent249.Identity identity,
            SearchWorkLedger ledger,String bindingId,int maximumImprovements){
        this(incumbent,identity,ledger,bindingId,maximumImprovements,System::nanoTime,"SYSTEM_NANO_TIME");
    }
    private TimedCandidateLog258(RouteIncumbent249 incumbent,RouteIncumbent249.Identity identity,
            SearchWorkLedger ledger,String bindingId,int capacity,LongSupplier clock,String source){
        this.incumbent=Objects.requireNonNull(incumbent,"TIMED258_INCUMBENT");
        this.identity=Objects.requireNonNull(identity,"TIMED258_IDENTITY");
        this.ledger=Objects.requireNonNull(ledger,"TIMED258_LEDGER");
        if(bindingId==null||bindingId.trim().isEmpty()||capacity<=0)throw new IllegalArgumentException("TIMED258_LIMIT_OR_BINDING");
        this.bindingId=bindingId;this.capacity=capacity;this.clock=Objects.requireNonNull(clock);clockSource=source;
        RouteIncumbent249.Snapshot s=incumbent.snapshot();
        if(!identity.equals(s.identity)||s.state!=RouteIncumbent249.State.OPEN||!"seed".equals(s.candidateId))
            throw new IllegalArgumentException("TIMED258_REQUIRES_ACCEPTED_SEED_BEFORE_SEARCH");
        for(long count:s.observations.values())if(count!=0)throw new IllegalArgumentException("TIMED258_LATE_ATTACHMENT");
        seedCost=s.objective;bestCost=s.objective;seedCoordinates=coordinates(s.route());
    }
    /** Package-local deterministic fixture. Its output is forbidden by the production QPC join. */
    static TimedCandidateLog258 synthetic(RouteIncumbent249 incumbent,RouteIncumbent249.Identity identity,
            SearchWorkLedger ledger,String bindingId,int capacity,LongSupplier clock){
        return new TimedCandidateLog258(incumbent,identity,ledger,bindingId,capacity,clock,"SYNTHETIC_FIXTURE_CLOCK");
    }
    void requireBinding(RouteIncumbent249 actual,RouteIncumbent249.Identity id,SearchWorkLedger work){
        requireOpen();
        if(actual!=incumbent||!identity.equals(id)||ledger!=work)throw new IllegalArgumentException("TIMED258_DIFFERENT_RUN");
    }
    private void requireOwner(){if(owner!=Thread.currentThread())throw new IllegalStateException("TIMED258_THREAD");}
    private void requireOpen(){requireOwner();if(sealed)throw new IllegalStateException("TIMED258_SEALED");}
    private long now(){
        long value=clock.getAsLong();if(value<previousNano)throw new IllegalStateException("TIMED258_NONMONOTONE_CLOCK");
        previousNano=value;return value;
    }
    Token beforeNative(RouteIncumbent249.Identity actual,String candidateId){
        requireOpen();if(!identity.equals(actual))throw new IllegalArgumentException("TIMED258_GOAL_OR_IDENTITY");
        if(!"SEARCH_EXECUTE".equals(FullRouteAudit211.phase))throw new IllegalStateException("TIMED258_NOT_SEARCH");
        if(pending!=null)throw new IllegalStateException("TIMED258_UNRESOLVED_PREVIOUS_CALL");
        long stamp=now();pending=new Token(this,++attempts,stamp,candidateId);return pending;
    }
    /** Called only after the one native217 return and the one incumbent249 observation. */
    void afterNative(Token token,CandidateEvaluation217.Status status,Double objective){
        requireOpen();if(token==null||pending!=token||token.owner!=this)throw new IllegalArgumentException("TIMED258_TOKEN");
        long lower=now(); // The complete route is already in the warehouse at this instant.
        if(status==CandidateEvaluation217.Status.COMPLETE){
            if(objective==null||!Double.isFinite(objective)||objective<0||objective>=1e17)
                throw new IllegalArgumentException("TIMED258_INCOMPLETE_COST");
            if(objective<bestCost){
                if(points.size()>=capacity)throw new IllegalStateException("TIMED258_LOG_CAPACITY_CONTRACT");
                RouteIncumbent249.Snapshot s=incumbent.snapshot();
                if(!identity.equals(s.identity)||s.state!=RouteIncumbent249.State.OPEN||s.objective!=objective||!token.candidateId.equals(s.candidateId))
                    throw new IllegalStateException("TIMED258_WAREHOUSE_NOT_THIS_COMPLETE_CANDIDATE");
                int[][] route=coordinates(s.route());long work=ledger.snapshot().edgeSubsegmentAttempts;
                long upper=now();points.add(new Point(token,lower,upper,work,objective,route));
                bestCost=objective;bestId=token.candidateId;
            }
            completed++;
        }else{
            if(objective!=null)throw new IllegalArgumentException("TIMED258_UNKNOWN_FINITE_VALUE");
            if(status==CandidateEvaluation217.Status.UNPRICED)unpriced++;
            else if(status==CandidateEvaluation217.Status.NATIVE_REJECTED)rejected++;
            else throw new IllegalArgumentException("TIMED258_NOT_A_NORMAL_RETURN");
        }
        pending=null;
    }
    /** An in-flight exception stays unclassified here; the unique257 launcher owns that decision. */
    void seal(){requireOwner();sealed=true;}
    public Map<String,Object> snapshot(){
        requireOwner();RouteIncumbent249.Snapshot s=incumbent.snapshot();Map<String,Object> x=new LinkedHashMap<>();
        x.put("schema","TIMED_COMPLETE_CANDIDATES258_V1");x.put("clock_source",clockSource);x.put("pid",pid);
        x.put("clock_binding_id",bindingId);x.put("identity",identityMap(identity));x.put("sealed",sealed);
        x.put("seed_objective",seedCost);x.put("seed_coordinates",copy(seedCoordinates));x.put("seed_is_shared_problem_input",true);
        x.put("attempts",attempts);x.put("complete_returns",completed);x.put("unpriced_returns",unpriced);x.put("rejected_returns",rejected);
        x.put("unresolved_attempts",pending==null?0:1);x.put("maximum_improvements",capacity);
        List<Map<String,Object>> saved=new ArrayList<>();for(Point p:points)saved.add(p.map());x.put("improvements",saved);
        x.put("recorded_best_objective",bestCost);x.put("recorded_best_candidate_id",bestId);
        x.put("warehouse_state",s.state.name());x.put("warehouse_matches_recorded_best",s.objective==bestCost&&s.candidateId.equals(bestId));
        x.put("cpu_seconds_inferred_from_java_clock",false);return x;
    }
    static Map<String,Object> identityMap(RouteIncumbent249.Identity id){
        Map<String,Object> x=new LinkedHashMap<>();x.put("objective",id.objective);x.put("fields",id.fields());return x;
    }
    private static int[][] coordinates(List<GridPoint> route){
        int[][] result=new int[route.size()][2];for(int i=0;i<route.size();i++){result[i][0]=route.get(i).i;result[i][1]=route.get(i).j;}return result;
    }
    private static int[][] copy(int[][] x){int[][] y=new int[x.length][];for(int i=0;i<x.length;i++)y[i]=x[i].clone();return y;}
}
