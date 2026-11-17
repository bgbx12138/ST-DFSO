package Navigation.benchmark;

import Navigation.model.GridPoint;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.TreeMap;

/** Run-owned observation. Native evaluation and search updates remain with their callers. */
public final class RouteIncumbent249 {
    public enum Outcome { COMPLETE, NATIVE_REJECTED, UNPRICED, GLOBAL_STOP, CONTRACT_ERROR }
    public enum State { OPEN, COMPLETED, GLOBAL_STOPPED, CONTRACT_FAILED }

    public static final class Identity {
        public final String objective;
        private final Map<String,String> fields;
        private static final Set<String> REQUIRED=Collections.unmodifiableSet(new HashSet<>(Arrays.asList(
            "vessel","scenario","environment","graph","seed_route","run","supply","normalizer","evaluator")));
        public Identity(String objective,Map<String,String> fields) {
            if(!Arrays.asList("TIME","FUEL","SAFETY_DENSITY","SAFETY_EXPOSURE","COMPOSITE").contains(objective))
                throw new IllegalArgumentException("IDENTITY249_OBJECTIVE");
            if(fields==null||!fields.keySet().equals(REQUIRED))throw new IllegalArgumentException("IDENTITY249_KEYS");
            TreeMap<String,String> copy=new TreeMap<>();
            for(String k:REQUIRED)copy.put(k,text(fields.get(k),"IDENTITY249_"+k));
            this.objective=objective;this.fields=Collections.unmodifiableMap(copy);
        }
        public Map<String,String> fields(){return fields;}
        @Override public boolean equals(Object other){
            if(!(other instanceof Identity))return false;
            Identity that=(Identity)other;return objective.equals(that.objective)&&fields.equals(that.fields);
        }
        @Override public int hashCode(){return Objects.hash(objective,fields);}
    }

    public static final class Snapshot {
        public final Identity identity;
        public final State state;
        public final double objective;
        public final String candidateId;
        public final String terminationReason;
        public final String contractErrorClass;
        public final Map<Outcome,Long> observations;
        private final int[][] coordinates;
        private Snapshot(Identity identity,State state,double objective,String id,int[][] points,
                         String reason,RuntimeException error,Map<Outcome,Long> counts){
            this.identity=identity;this.state=state;this.objective=objective;candidateId=id;
            coordinates=copy(points);terminationReason=reason;
            contractErrorClass=error==null?null:error.getClass().getName();
            observations=Collections.unmodifiableMap(new EnumMap<>(counts));
        }
        /** Every call returns fresh points; mutating a returned point cannot mutate this snapshot. */
        public List<GridPoint> route(){
            List<GridPoint> result=new ArrayList<>();
            for(int[] p:coordinates)result.add(new GridPoint(p[0],p[1]));
            return Collections.unmodifiableList(result);
        }
    }

    private final Identity identity;
    private final int[] first,last;
    private final EnumMap<Outcome,Long> counts=new EnumMap<>(Outcome.class);
    private State state=State.OPEN;
    private int[][] best;
    private double bestCost;
    private String bestId="seed",reason;
    private RuntimeException failure;

    /** Caller must supply a native-complete seed and its cost under this exact identity. */
    public RouteIncumbent249(Identity identity,List<GridPoint> seed,double seedCost){
        this.identity=Objects.requireNonNull(identity,"IDENTITY249_NULL");
        requireCost(seedCost);best=coordinates(seed);bestCost=seedCost;
        first=best[0].clone();last=best[best.length-1].clone();
        for(Outcome x:Outcome.values())counts.put(x,0L);
    }
    private static String text(String x,String label){
        if(x==null||x.trim().isEmpty())throw new IllegalArgumentException(label);return x;
    }
    private static void requireCost(double x){
        if(!Double.isFinite(x)||x<0||x>=1e17)throw new IllegalArgumentException("INCOMPLETE249_COST");
    }
    private static int[][] copy(int[][] points){
        int[][] result=new int[points.length][];
        for(int i=0;i<points.length;i++)result[i]=points[i].clone();return result;
    }
    private static int[][] coordinates(List<GridPoint> points){
        if(points==null||points.size()<2)throw new IllegalArgumentException("INCOMPLETE249_ROUTE");
        int[][] result=new int[points.size()][2];Set<Long> seen=new HashSet<>();
        for(int i=0;i<points.size();i++){
            GridPoint p=points.get(i);
            if(p==null||p.i<0||p.j<0)throw new IllegalArgumentException("INVALID249_POINT");
            long key=((long)p.i<<32)|(p.j&0xffffffffL);
            if(!seen.add(key))throw new IllegalArgumentException("REPEATED249_POINT");
            result[i][0]=p.i;result[i][1]=p.j;
        }return result;
    }
    private void open(Identity actual){
        if(!identity.equals(actual))throw new IllegalArgumentException("INCUMBENT249_IDENTITY_MISMATCH");
        if(state!=State.OPEN)throw new IllegalStateException("INCUMBENT249_TERMINAL:"+state);
    }
    private void count(Outcome x){counts.put(x,counts.get(x)+1);}

    /** Return the observed candidate value unchanged, including worse finite candidates. */
    public synchronized Double observe(Identity actual,Outcome outcome,String candidateId,
                                       List<GridPoint> route,Double objective){
        open(actual);Objects.requireNonNull(outcome,"OUTCOME249_NULL");text(candidateId,"CANDIDATE249_ID");
        if(outcome==Outcome.COMPLETE){
            if(objective==null)throw new IllegalArgumentException("COMPLETE249_NULL_COST");
            requireCost(objective);int[][] points=coordinates(route);
            if(!Arrays.equals(first,points[0])||!Arrays.equals(last,points[points.length-1]))
                throw new IllegalArgumentException("INCUMBENT249_ENDPOINT_MISMATCH");
            count(outcome);
            if(objective<bestCost){best=points;bestCost=objective;bestId=candidateId;}
            return objective;
        }
        if(outcome!=Outcome.UNPRICED&&outcome!=Outcome.NATIVE_REJECTED)
            throw new IllegalArgumentException("USE249_EXPLICIT_TERMINATION");
        if(objective!=null)throw new IllegalArgumentException("UNPRICED249_FINITE_VALUE");
        count(outcome);return null;
    }
    public synchronized void complete(Identity actual){open(actual);state=State.COMPLETED;reason="NORMAL_COMPLETION";}
    /** Caller must establish a genuine global resource stop before invoking this method. */
    public synchronized void globalStop(Identity actual,String resourceReason){
        open(actual);reason=text(resourceReason,"STOP249_REASON");count(Outcome.GLOBAL_STOP);state=State.GLOBAL_STOPPED;
    }
    /** A contract error stays a failed run and propagates as the original exception. */
    public synchronized void contractFailure(Identity actual,RuntimeException error){
        open(actual);failure=Objects.requireNonNull(error,"CONTRACT249_ERROR");reason=error.getMessage();
        count(Outcome.CONTRACT_ERROR);state=State.CONTRACT_FAILED;throw error;
    }
    /** Diagnostics remain readable on a failed run; state must accompany any serialized snapshot. */
    public synchronized Snapshot snapshot(){return new Snapshot(identity,state,bestCost,bestId,best,reason,failure,counts);}
    /** Only normal completion or a genuine global stop may supply a final-audit route. */
    public synchronized Snapshot resultForFinalAudit(){
        if(state!=State.COMPLETED&&state!=State.GLOBAL_STOPPED)
            throw new IllegalStateException("INCUMBENT249_NOT_SUCCESSFUL:"+state,failure);
        return snapshot();
    }
}
