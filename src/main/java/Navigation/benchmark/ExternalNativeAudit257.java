package Navigation.benchmark;

import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.IntegrationAudit;
import Navigation.model.GridPoint;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Original native five-goal reception. Does not replace the response/physics gates. */
public final class ExternalNativeAudit257 implements ExternalSearchBoundary257.FinalAudit {
    private final PathEvaluator evaluator;
    private final String revision,normalizer;
    private final double maxDistance;
    private Map<String,Object> received;
    public ExternalNativeAudit257(PathEvaluator evaluator,double maxDistance,String revision,String normalizer){
        this.evaluator=Objects.requireNonNull(evaluator,"NATIVE_AUDITOR257_NULL");
        if(evaluator.getClass()!=PathEvaluator.class)throw new IllegalArgumentException("RAW_NATIVE257_EVALUATOR_REQUIRED");
        if(!Double.isFinite(maxDistance)||maxDistance<=0.)throw new IllegalArgumentException("NATIVE_AUDIT257_DISTANCE");
        this.maxDistance=maxDistance;this.revision=Objects.requireNonNull(revision,"REVISION257_NULL");
        this.normalizer=Objects.requireNonNull(normalizer,"NORMALIZER257_NULL");
    }
    /** One phase transition per boundary, using the original three-party metadata helper. */
    public static ExternalSearchBoundary257.Phases phases(){
        return new ExternalSearchBoundary257.Phases(){
            public void search() throws Exception {AvailabilityLauncher211.mark("SEARCH_EXECUTE");}
            public void finalAudit() throws Exception {AvailabilityLauncher211.mark("FINAL_AUDIT");}
            public void done() throws Exception {AvailabilityLauncher211.mark("DONE");}
        };
    }
    private static void near(double actual,double expected,String label){
        if(!ExternalRun166.valid(actual)||!ExternalRun166.valid(expected)||Math.abs(actual-expected)>1e-7*Math.max(1.,Math.abs(expected)))
            throw new IllegalStateException(label);
    }
    @Override public void verify(RouteIncumbent249.Snapshot candidate) throws Exception {
        if(received!=null)throw new IllegalStateException("NATIVE_AUDIT257_ALREADY_RUN");
        if(!revision.equals(System.getProperty("nav.r1.tableRevision")))throw new IllegalStateException("NATIVE_AUDIT257_SUPPLY_CHANGED");
        String actualNormalizer=ExternalEvaluationScope251.normalizerFingerprint(IntegrationAudit.context(evaluator),BenchmarkLauncher.COMPOSITE_W);
        if(!normalizer.equals(actualNormalizer)||!normalizer.equals(candidate.identity.fields().get("normalizer")))
            throw new IllegalStateException("NATIVE_AUDIT257_NORMALIZER_CHANGED");
        List<GridPoint> route=candidate.route();Map<String,Object> after=AvailabilityLauncher211.audit(evaluator,route);
        if(!Boolean.TRUE.equals(after.get("complete"))||!Boolean.TRUE.equals(after.get("practical"))){
            PaidFailureAudit416.write("NATIVE_AUDIT257_INCOMPLETE_OR_GEOMETRY",after,route);
            throw new IllegalStateException("NATIVE_AUDIT257_INCOMPLETE_OR_GEOMETRY");
        }
        String[] names={"COMPOSITE","TIME","FUEL","SAFETY_DENSITY","SAFETY_EXPOSURE"};
        String[] keys={"composite","time_hours","fuel_j","risk_density","risk_total"};
        Map<String,Object> nativeGoals=new LinkedHashMap<>();
        for(int i=0;i<names.length;i++){
            HybridFSA3D.ObjectiveMode mode=HybridFSA3D.ObjectiveMode.valueOf(names[i]);
            ObjectiveWeights weights=mode==HybridFSA3D.ObjectiveMode.COMPOSITE?BenchmarkLauncher.COMPOSITE_W:null;
            double value=evaluator.evaluateStrict(route,mode,weights,evaluator.getRouteGraph(),maxDistance);
            Object raw=after.get(keys[i]);if(!(raw instanceof Number))throw new IllegalStateException("NATIVE_AUDIT257_NULL_"+names[i]);
            near(value,((Number)raw).doubleValue(),"NATIVE_AUDIT257_MAP_"+names[i]);nativeGoals.put(names[i],value);
            if(candidate.identity.objective.equals(names[i]))near(value,candidate.objective,"NATIVE_AUDIT257_ACTIVE_GOAL");
        }
        after.put("native_five_goal_costs",nativeGoals);after.put("native_five_goal_match",true);
        received=after;
    }
    public Map<String,Object> report(){
        if(received==null)throw new IllegalStateException("NATIVE_AUDIT257_NOT_RECEIVED");
        return new LinkedHashMap<>(received);
    }
}
