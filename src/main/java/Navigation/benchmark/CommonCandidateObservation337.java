package Navigation.benchmark;
import Navigation.Navigation3D.fsa.*;
import Navigation.model.GridPoint;
import java.util.*;

/** Common observation only: never evaluates, prunes, changes a returned cost or starts a clock. */
public final class CommonCandidateObservation337 implements AutoCloseable {
    private static CommonCandidateObservation337 ACTIVE;
    private final Thread owner=Thread.currentThread();
    private final SearchWorkLedger ledger; private final FsaContext context;
    private final List<Map<String,Object>> improvements=new ArrayList<>();
    private final Map<String,Long> counts=new LinkedHashMap<>();
    private List<GridPoint> best; private double bestCost; private boolean closed;
    private long invalidComplete; private final long pid=ProcessHandle.current().pid();
    private CommonCandidateObservation337(List<GridPoint> seed,double cost,SearchWorkLedger ledger,FsaContext context){
        if(seed==null||seed.size()<2||!Double.isFinite(cost)||cost<0||ledger==null||context==null)throw new IllegalArgumentException("OBS337_SEED");
        this.ledger=ledger;this.context=context;best=copy(seed);bestCost=cost;
    }
    public static synchronized CommonCandidateObservation337 open(List<GridPoint> seed,double cost,SearchWorkLedger ledger,FsaContext context){
        if(ACTIVE!=null)throw new IllegalStateException("OBS337_NESTED");ACTIVE=new CommonCandidateObservation337(seed,cost,ledger,context);return ACTIVE;
    }
    private static List<GridPoint> copy(List<GridPoint> p){List<GridPoint> out=new ArrayList<>();for(GridPoint q:p)out.add(new GridPoint(q.i,q.j));return out;}
    /** Called only after the unchanged217 classification/recording, for every method. */
    public static synchronized void record(String status,List<GridPoint> route,Double objective){
        CommonCandidateObservation337 a=ACTIVE;if(a==null||a.closed||!"SEARCH_EXECUTE".equals(FullRouteAudit211.phase))return;
        a.counts.merge(status,1L,Long::sum);
        if(!"COMPLETE".equals(status))return;
        if(objective==null||!Double.isFinite(objective)||objective<0||objective>=1e17)throw new IllegalStateException("OBS337_INVALID_COMPLETE");
        if(route==null||route.size()<2||!IntegrationAudit.practical(route,a.context)){a.invalidComplete++;return;}
        if(objective>=a.bestCost)return;
        if(a.improvements.size()>=8192)throw new IllegalStateException("OBS337_LOG_CAP_NOT_SILENT_TRUNCATION");
        long lower=System.nanoTime();a.best=copy(route);a.bestCost=objective;Map<String,Object> p=new LinkedHashMap<>();
        p.put("nano_lower",lower);p.put("nano_upper",System.nanoTime());p.put("objective",objective);
        p.put("subsegments",a.ledger.snapshot().edgeSubsegmentAttempts);p.put("points",copy(a.best));a.improvements.add(p);
    }
    public synchronized List<GridPoint> bestRoute(){return copy(best);}
    public synchronized double bestCost(){return bestCost;}
    public synchronized Map<String,Object> snapshot(){Map<String,Object> m=new LinkedHashMap<>();m.put("schema","COMMON_COMPLETE_CANDIDATE337");m.put("pid",pid);m.put("clock_source","SYSTEM_NANO_TIME");m.put("closed",closed);m.put("counts",new LinkedHashMap<>(counts));m.put("invalid_complete_geometry",invalidComplete);m.put("improvements",new ArrayList<>(improvements));m.put("best_cost",bestCost);m.put("best_route",copy(best));return m;}
    @Override public void close(){synchronized(CommonCandidateObservation337.class){if(closed)return;if(owner!=Thread.currentThread()||ACTIVE!=this)throw new IllegalStateException("OBS337_CLOSE_OWNER");closed=true;ACTIVE=null;}}
}
