package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import Navigation.model.GridRouteGraph;
import java.util.*;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.LongAdder;

/** Static geometric cues only. Never caches weather, travel time or risk. */
public final class StaticRouteGeometry {
    private static final int CACHE_LIMIT=65536;
    private final GridRouteGraph graph;
    private final int nx,ny;
    private final double[][] clearance;
    private final Map<Long,Boolean> legalCache=new ConcurrentHashMap<>();
    private final Map<Long,Double> cueCache=new ConcurrentHashMap<>();
    private final LongAdder legalQueries=new LongAdder(),legalMisses=new LongAdder(),cueQueries=new LongAdder(),cueMisses=new LongAdder();
    private final long preprocessingNanos;

    public StaticRouteGeometry(GridRouteGraph graph,GridRouteGraph.BlockedCell blocked,boolean buildClearance){
        long start=System.nanoTime();this.graph=Objects.requireNonNull(graph);nx=graph.getNx();ny=graph.getNy();
        if((long)nx*ny>Integer.MAX_VALUE)throw new IllegalArgumentException("grid key overflow");
        clearance=buildClearance?new double[ny][nx]:null;
        if(buildClearance)for(int r=0;r<ny;r++)for(int c=0;c<nx;c++){
            double distance=2;
            for(int dr=-2;dr<=2;dr++)for(int dc=-2;dc<=2;dc++){
                double d=Math.hypot(dr,dc);if(d>=distance)continue;
                int rr=r+dr,cc=c+dc;
                if(rr<0||rr>=ny||cc<0||cc>=nx||blocked.isBlocked(rr,cc))distance=d;
            }
            clearance[r][c]=Math.min(1,distance/2);
        }
        preprocessingNanos=System.nanoTime()-start;
    }
    private long directedKey(GridPoint a,GridPoint b){return (((long)a.i*nx+a.j)<<32)|(((long)b.i*nx+b.j)&0xffffffffL);}
    public boolean legal(GridPoint a,GridPoint b){
        legalQueries.increment();
        if(a==null||b==null||a.i<0||a.i>=ny||a.j<0||a.j>=nx||b.i<0||b.i>=ny||b.j<0||b.j>=nx)return false;
        long x=(long)a.i*nx+a.j,y=(long)b.i*nx+b.j;
        long key=(Math.min(x,y)<<32)|(Math.max(x,y)&0xffffffffL);
        Boolean old=legalCache.get(key);if(old!=null)return old;
        legalMisses.increment();boolean valid=graph.isValidEdge(a,b);
        if(legalCache.size()<CACHE_LIMIT)legalCache.putIfAbsent(key,valid);
        return valid;
    }
    /** Mean near-field clearance over the exact closed-cell supercover, excluding the shared origin. */
    public double segmentCue(GridPoint a,GridPoint b){
        if(clearance==null)throw new IllegalStateException("clearance not enabled");
        if(!legal(a,b))throw new IllegalArgumentException("cue requires a legal edge");
        cueQueries.increment();long key=directedKey(a,b);Double old=cueCache.get(key);if(old!=null)return old;
        cueMisses.increment();Set<Integer> cells=new HashSet<>();
        // Reuse the graph's public supercover enumerator without changing its obstacle predicate.
        GridRouteGraph.hasSupercoverClearance(a.i,a.j,b.i,b.j,ny,nx,(r,c)->{
            if(r!=a.i||c!=a.j)cells.add(r*nx+c);return false;
        });
        double sum=0;for(int cell:cells)sum+=clearance[cell/nx][cell%nx];
        double cue=.05+.95*(cells.isEmpty()?clearance[b.i][b.j]:sum/cells.size());
        if(cueCache.size()<CACHE_LIMIT)cueCache.putIfAbsent(key,cue);
        return cue;
    }
    public Map<String,Long> statistics(){
        Map<String,Long>x=new LinkedHashMap<>();x.put("preprocessing_nanos",preprocessingNanos);
        x.put("legal_queries",legalQueries.sum());x.put("legal_cache_misses",legalMisses.sum());
        x.put("segment_cue_queries",cueQueries.sum());x.put("segment_cue_cache_misses",cueMisses.sum());
        x.put("legal_cache_entries",(long)legalCache.size());x.put("cue_cache_entries",(long)cueCache.size());return x;
    }
}
