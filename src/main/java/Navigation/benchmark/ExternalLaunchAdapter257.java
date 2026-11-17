package Navigation.benchmark;

import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.FsaContext;
import Navigation.benchmark.algo.AcoOptimizer166;
import Navigation.benchmark.algo.AsvIaFsaOptimizer166;
import Navigation.benchmark.algo.WindowIlsOptimizer169;
import Navigation.model.GridPoint;
import Navigation.object.NavigationConfig;
import java.util.Arrays;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** Source-only wiring: original three algorithms,251 scope,249 complete-route champion. */
public final class ExternalLaunchAdapter257 {
    public static final class Result {
        public final RouteIncumbent249.Snapshot verified;
        public final SwarmOptResult legacyResult; //null on a legitimate typed stop
        public final Map<String,Object> lifecycle;
        private Result(RouteIncumbent249.Snapshot verified,SwarmOptResult legacyResult,Map<String,Object> lifecycle){
            this.verified=verified;this.legacyResult=legacyResult;this.lifecycle=lifecycle;
        }
    }
    public static Result run(String algorithm,int population,int iterations,double kappa,long cap,long random,
            List<GridPoint> seed,NavigationConfig config,HybridFSA3D.ObjectiveMode goal,ObjectiveWeights compositeWeights,
            FsaContext completeSeedContext,double maxDistance,String revision,RouteIncumbent249.Identity identity,
            RouteIncumbent249 incumbent,ExternalRun166 run,ExternalSearchBoundary257.Phases phases,
            ExternalSearchBoundary257.FinalAudit nativeFinalAudit) throws Exception {
        if(!Arrays.asList("WINDOW_ILS","ACO","ASV_IAFSA").contains(algorithm))throw new IllegalArgumentException("ALGORITHM257_UNKNOWN");
        if(population<1||iterations<1||!Double.isFinite(kappa)||kappa<1||cap<=0||cap!=run.ledger.getMaxEdgeSubsegmentAttempts())
            throw new IllegalArgumentException("BOUNDED257_SETTINGS_REQUIRED");
        if(!"false".equals(System.getProperty("nav220.screen")))throw new IllegalStateException("R2257_REQUIRES_EXPLICIT_SCREEN_FALSE");
        Objects.requireNonNull(goal,"GOAL257_NULL");
        if(!identity.objective.equals(goal.name()))throw new IllegalArgumentException("GOAL257_IDENTITY");
        List<GridPoint> acceptedSeed=incumbent.snapshot().route();
        if(seed==null||seed.size()!=acceptedSeed.size())throw new IllegalArgumentException("SEED257_COORDINATES");
        for(int i=0;i<seed.size();i++)if(seed.get(i)==null||seed.get(i).i!=acceptedSeed.get(i).i||seed.get(i).j!=acceptedSeed.get(i).j)
            throw new IllegalArgumentException("SEED257_COORDINATES");
        ExternalSearchBoundary257 boundary=new ExternalSearchBoundary257(incumbent,identity,run);
        final SwarmOptResult[] legacy={null};
        RouteIncumbent249.Snapshot verified=boundary.execute(
            ()->{
                ExternalEvaluationScope251 scope=ExternalEvaluationScope251.open(identity,incumbent,config,completeSeedContext,
                    run.ledger,compositeWeights,maxDistance,revision);
                return scope::close;
            },
            ()->{
                HybridSwarmOptimizer optimizer=algorithm.equals("WINDOW_ILS")
                    ?new WindowIlsOptimizer169(iterations,kappa,cap).withRun(run)
                    :algorithm.equals("ACO")?new AcoOptimizer166(population,iterations,kappa,cap).withRun(run)
                    :new AsvIaFsaOptimizer166(population,iterations,kappa,cap).withRun(run);
                ObjectiveWeights weights=goal==HybridFSA3D.ObjectiveMode.COMPOSITE?compositeWeights:null;
                legacy[0]=optimizer.optimize(acceptedSeed,config,goal,weights,random);
                if(legacy[0]==null||!ExternalRun166.valid(legacy[0].compositeCost)||!run.finished())
                    throw new IllegalStateException("EXTERNAL257_NORMAL_RETURN_CONTRACT");
            },phases,nativeFinalAudit);
        //The independent native audit governs success. Legacy scalars/routes are diagnostic.
        return new Result(verified,legacy[0],boundary.diagnostics());
    }
}
