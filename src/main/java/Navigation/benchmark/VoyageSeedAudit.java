package Navigation.benchmark;
import java.nio.file.*;
import java.util.*;
import Navigation.model.GridPoint;
import Navigation.object.NavigationConfig;
import Navigation.Navigation3D.fsa.*;
/** Method-independent preparation: only the six common input routes, no optimiser. */
public final class VoyageSeedAudit {
    public static void main(String[] args)throws Exception {
        if(args.length!=2)throw new IllegalArgumentException("SCENARIO OUTPUT");
        BenchmarkLauncher.Scenario chosen=null;
        for(BenchmarkLauncher.Scenario s:BenchmarkLauncher.SCENARIOS)if(s.id.equals(args[0]))chosen=s;
        if(chosen==null)throw new IllegalArgumentException("SCENARIO");
        NavigationConfig cfg=BenchmarkLauncher.buildConfig(chosen);cfg.showVisualization=false;
        cfg.enableCompositeNormalization=true;cfg.enableCompositeTimeNormalization=true;
        cfg.enableCompositeFuelNormalization=true;cfg.enableCompositeShipClassCorrection=true;cfg.enableCompositeSafetyNormalization=false;
        List<GridPoint> seed=Arrays.asList(NativeFinalRunner351.J.readValue(Paths.get("config/seed_routes/"+args[0]+".json").toFile(),GridPoint[].class));
        FullRouteAudit211.configure(seed);AvailabilityLauncher211.mark("SEED_ONLY_PRECOMPUTATION");
        try(PathEvaluator raw=new PathEvaluator(cfg)) {
            double max=raw.configureDistanceBudgetFromSeed(seed,args[0].equals("S6")?1.5:2.);
            Map<String,Object> audit=NativeFinalRunner351.audit(raw,seed,max);
            NativeFinalRunner351.J.writeValue(Paths.get(args[1]).toFile(),audit);
        }
        AvailabilityLauncher211.mark("DONE");
    }
}
