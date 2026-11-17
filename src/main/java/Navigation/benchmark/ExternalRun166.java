package Navigation.benchmark;

import Navigation.Navigation3D.fsa.SearchWorkLedger;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/** Run-scoped observation and budget. Never supplies a new route or changes an update equation. */
public final class ExternalRun166 {
    public final SearchWorkLedger ledger;
    public final double seedCost;
    private final long millis;
    private boolean started,finished;
    private final List<Map<String,Object>> milestones=new ArrayList<>();
    public ExternalRun166(long cap,long millis,double seedCost) {
        if(cap<=0 || millis<=0 || !valid(seedCost))throw new IllegalArgumentException("bounded valid seed required");
        ledger=new SearchWorkLedger(cap);this.millis=millis;this.seedCost=seedCost;
    }
    public static boolean valid(double cost){return Double.isFinite(cost)&&cost>=0&&cost<1e17;}
    public void start(long expectedCap) {
        if(started||finished||expectedCap!=ledger.getMaxEdgeSubsegmentAttempts())throw new IllegalStateException("run ownership/cap mismatch");
        ledger.startWallClock(millis);started=true;
    }
    public double checkedSeed(double value) {
        if(!valid(value)) {
            // A pre-validated shared seed remains the incumbent if a deliberately tiny
            // correctness-check budget is exhausted during its internal re-evaluation.
            if(ledger.shouldStopSearch())return seedCost;
            throw new AssertionError("EXTERNAL166_NATIVE_SEED_INVALID");
        }
        if(Math.abs(value-seedCost)>1e-8*Math.max(1,Math.abs(seedCost)))throw new AssertionError("EXTERNAL166_NATIVE_SEED_MISMATCH");
        return value;
    }
    public void checkpoint(double best,boolean force,String label) {
        if(!started||!valid(best)||best>seedCost+1e-8*Math.max(1,seedCost))throw new AssertionError("invalid incumbent");
        if(!force&&!milestones.isEmpty()&&best>=((Number)milestones.get(milestones.size()-1).get("cost")).doubleValue())return;
        if(milestones.size()>=2048)return;
        Map<String,Object> row=new LinkedHashMap<>();row.put("cost",best);
        row.put("wall_nanos",ledger.elapsedWallNanos());row.put("subsegments",ledger.snapshot().edgeSubsegmentAttempts);
        row.put("checkpoint",label);milestones.add(row);
    }
    public void finish(double best) {
        if(!started||finished)throw new IllegalStateException("finish lifecycle");
        checkpoint(best,true,"final_boundary");ledger.setActive(false);finished=true;
    }
    public boolean finished(){return finished;}
    public List<Map<String,Object>> milestones(){return new ArrayList<>(milestones);}
}
