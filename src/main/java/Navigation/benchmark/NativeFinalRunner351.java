package Navigation.benchmark;
import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.*;
import Navigation.model.GridPoint;
import Navigation.object.NavigationConfig;
import com.fasterxml.jackson.databind.*;
import java.nio.file.*;
import java.security.MessageDigest;
import java.util.*;

/** One common configured risk run. Native gates and original external update equations retained. */
public final class NativeFinalRunner351 {
    static final ObjectMapper J=new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
    static String hash(String x)throws Exception{StringBuilder s=new StringBuilder();for(byte z:MessageDigest.getInstance("SHA-256").digest(x.getBytes(java.nio.charset.StandardCharsets.UTF_8)))s.append(String.format("%02x",z&255));return s.toString();}
    static Map<String,String> loaded()throws Exception{
        Map<String,String> out=new LinkedHashMap<>();
        for(String name:new String[]{"Navigation.benchmark.NativeFinalRunner351","Navigation.benchmark.FinalArm351","Navigation.benchmark.CommonCandidateObservation337","Navigation.benchmark.BenchmarkLauncher","Navigation.benchmark.PathEvaluator","Navigation.benchmark.FairPathEvaluator166","Navigation.benchmark.ExternalRun166","Navigation.benchmark.ExternalEvaluationScope251","Navigation.benchmark.ExternalLaunchAdapter257","Navigation.benchmark.ExternalSearchBoundary257","Navigation.benchmark.ExternalNativeAudit257","Navigation.benchmark.CandidateBinding250","Navigation.benchmark.ClockBridge164","Navigation.benchmark.algo.WindowIlsOptimizer169","Navigation.benchmark.algo.AcoOptimizer166","Navigation.benchmark.algo.AsvIaFsaOptimizer166","Navigation.Navigation3D.HybridFSA3D","Navigation.Navigation3D.fsa.CandidateEvaluation217","Navigation.Navigation3D.fsa.CandidateEvaluation217$Status","Navigation.Navigation3D.fsa.IntegrationAudit","Navigation.Navigation3D.fsa.ResearchResponsePort","Navigation.Navigation3D.VoyageSpeedModel","Navigation.Navigation3D.Normalization.Normalizers","Navigation.Navigation3D.Normalization.TimeNormalizer","Navigation.Navigation3D.Normalization.FuelNormalizer"}){
            Class<?> c=Class.forName(name,false,NativeFinalRunner351.class.getClassLoader());
            try(java.io.InputStream in=c.getResourceAsStream("/"+name.replace('.','/')+".class")){
                if(in==null)throw new IllegalStateException("LOADED337_CLASS_MISSING:"+name);StringBuilder h=new StringBuilder();for(byte b:MessageDigest.getInstance("SHA-256").digest(in.readAllBytes()))h.append(String.format("%02x",b&255));out.put(name,h.toString());
            }
        }return out;
    }
    static void need(boolean x,String s){if(!x)throw new IllegalStateException(s);}
    static void near(double x,double y,String s){need(Double.isFinite(x)&&Double.isFinite(y)&&Math.abs(x-y)<=1e-7*Math.max(1,Math.abs(y)),s);}
    static Map<String,Object> audit(PathEvaluator p,List<GridPoint> route,double max)throws Exception{
        Map<String,Object> a=AvailabilityLauncher211.audit(p,route);if(!Boolean.TRUE.equals(a.get("complete"))||!Boolean.TRUE.equals(a.get("practical")))PaidFailureAudit416.write("NATIVE337_COMPLETE_PRACTICAL",a,route);need(Boolean.TRUE.equals(a.get("complete"))&&Boolean.TRUE.equals(a.get("practical")),"NATIVE337_COMPLETE_PRACTICAL");
        String[] names={"TIME","FUEL","SAFETY_DENSITY","SAFETY_EXPOSURE","COMPOSITE"};String[] keys={"time_hours","fuel_j","risk_density","risk_total","composite"};Map<String,Object> nativeGoals=new LinkedHashMap<>();
        for(int i=0;i<names.length;i++){HybridFSA3D.ObjectiveMode g=HybridFSA3D.ObjectiveMode.valueOf(names[i]);double v=p.evaluateStrict(route,g,g==HybridFSA3D.ObjectiveMode.COMPOSITE?BenchmarkLauncher.COMPOSITE_W:null,p.getRouteGraph(),max);near(v,((Number)a.get(keys[i])).doubleValue(),"NATIVE337_FIVE_GOAL_"+names[i]);nativeGoals.put(names[i],v);}
        a.put("native_five_goal_match",true);a.put("native_five_goal_costs",nativeGoals);return a;
    }
    public static void main(String[] args)throws Exception{
        Map<String,String> o=new HashMap<>();for(int i=0;i<args.length;i+=2){need(i+1<args.length&&args[i].startsWith("--")&&o.put(args[i].substring(2),args[i+1])==null,"OPTIONS337");}
        String scene=o.get("scenario"),algorithm=o.get("algorithm"),arm=o.get("arm"),runId=o.get("run-id"),goalName=o.get("objective");
        need(Arrays.asList("FSA_SELECTED","WINDOW_ILS","ACO","ASV_IAFSA").contains(algorithm),"METHOD337");need(Arrays.asList("OFF_SF","MINUS_LL","MINUS_AV","MINUS_FG","MINUS_ER","CORE").contains(arm),"FINAL_ARM337");
        need(Arrays.asList("TIME","FUEL","SAFETY_DENSITY","SAFETY_EXPOSURE","COMPOSITE").contains(goalName),"RISK_GOALS337");need("false".equals(System.getProperty("nav220.screen")),"COMMON337_SCREEN_FALSE");
        Path out=Paths.get(o.get("output"));Files.createDirectory(out);long random=Long.parseLong(o.get("seed")),cap=Long.parseLong(o.get("subsegments")),millis=Long.parseLong(o.get("millis"));int population=Integer.parseInt(o.get("population")),iterations=Integer.parseInt(o.get("iterations"));
        BenchmarkLauncher.Scenario chosen=null;for(BenchmarkLauncher.Scenario s:BenchmarkLauncher.SCENARIOS)if(s.id.equals(scene))chosen=s;need(chosen!=null,"SCENE337");
        NavigationConfig cfg=BenchmarkLauncher.buildConfig(chosen);cfg.showVisualization=false;cfg.enableCompositeNormalization=true;cfg.enableCompositeTimeNormalization=true;cfg.enableCompositeFuelNormalization=true;cfg.enableCompositeShipClassCorrection=true;cfg.enableCompositeSafetyNormalization=false;
        need(Math.abs(cfg.shipParameters.shipLength-142)<1e-7&&Math.abs(cfg.shipParameters.shipWidth-19.06)<1e-7&&Math.abs(cfg.shipParameters.shipDraft-6.15)<1e-7&&Math.abs(cfg.shipParameters.shipDisplacement-8431.8)<1e-7&&Math.abs(cfg.shipParameters.shipSpeed*.5144-9.256149419710122)<1e-10,"VESSEL337");
        J.writeValue(out.resolve("actual_config.json").toFile(),cfg);
        List<GridPoint> seed=Arrays.asList(J.readValue(Paths.get(o.get("seed-json")).toFile(),GridPoint[].class));FullRouteAudit211.configure(seed);System.setProperty("nav211.searchMillis",Long.toString(millis));CandidateEvaluation217.reset();CandidateScreen220.reset();AvailabilityLauncher211.mark("SEED_AUDIT");
        HybridFSA3D.ObjectiveMode goal=HybridFSA3D.ObjectiveMode.valueOf(goalName);ObjectiveWeights weights=goal==HybridFSA3D.ObjectiveMode.COMPOSITE?BenchmarkLauncher.COMPOSITE_W:null;double kappa=scene.equals("S6")?1.5:2.;
        try(PathEvaluator raw=new PathEvaluator(cfg)){
            double max=raw.configureDistanceBudgetFromSeed(seed,kappa);Map<String,Object> before=audit(raw,seed,max);double seedCost=((Number)((Map)before.get("native_five_goal_costs")).get(goalName)).doubleValue();
            J.writeValue(out.resolve("seed_audit.json").toFile(),before);J.writeValue(out.resolve("seed_points.json").toFile(),seed);
            FsaContext context=IntegrationAudit.context(raw);String revision=ComponentProcessProvider211.CURRENT.revision,normalizer=ExternalEvaluationScope251.normalizerFingerprint(context,BenchmarkLauncher.COMPOSITE_W);
            Map<String,String> ids=new TreeMap<>();ids.put("vessel",ExternalEvaluationScope251.vesselFingerprint(cfg));ids.put("scenario",scene);ids.put("environment",o.get("environment-sha"));ids.put("graph",o.get("graph-sha"));ids.put("seed_route",o.get("route-sha"));ids.put("run",runId);ids.put("supply",o.get("supply-sha"));ids.put("normalizer",normalizer);ids.put("evaluator",o.get("evaluator-sha"));
            RouteIncumbent249.Identity identity=new RouteIncumbent249.Identity(goalName,ids);ExternalRun166 run=new ExternalRun166(cap,millis,seedCost);RouteIncumbent249 warehouse=new RouteIncumbent249(identity,seed,seedCost);
            TimedCandidateLog258 timed=new TimedCandidateLog258(warehouse,identity,run.ledger,runId,8192);ExternalNativeAudit257 finalNative=new ExternalNativeAudit257(raw,max,revision,normalizer);
            CommonCandidateObservation337 observed=CommonCandidateObservation337.open(seed,seedCost,run.ledger,context);ClockBridge164 bridge=ClockBridge164.capture();String typedStop=null;Map<String,Object> fsaFlags=null;Map<String,Object> lifecycle=null;
            try{
                if(algorithm.equals("FSA_SELECTED")){
                    HybridFSA3D f=new HybridFSA3D(cfg).setObjectiveMode(goal).setObjectiveWeights(weights).setPopulationSize(population).setMaxIterations(iterations).setThreadPoolSize(1).setBaseSeed(random).setSeedPath(seed).setRouteDistanceBudgetMultiplier(kappa).setSearchWorkLedger(run.ledger).setV5ExperimentContract(true).setImprovementWallMillis(millis).setMeasureSearchCpu(true);
                    FinalArm351.configure(f,arm);fsaFlags=AvailabilityLauncher211.flags(f);AvailabilityLauncher211.mark("SEARCH_EXECUTE");
                    try{f.execute();}catch(Exception error){
                        TruthBudgetStop211 stop=TruthBudgetStop211.find(error);
                        boolean noBest=f.getBestPath()==null||f.getBestPath().size()<2||!Double.isFinite(f.getBestCost())||f.getBestCost()>=1e17;
                        boolean stopped=run.ledger.snapshot().edgeSubsegmentAttempts>=cap||run.ledger.elapsedWallNanos()>=millis*1000000L;
                        if(stop!=null){typedStop=stop.getMessage();need(typedStop!=null&&!typedStop.trim().isEmpty(),"EMPTY_TYPED337_STOP_REASON");}
                        else if(noBest&&stopped&&"HybridFSA3D \u672a\u627e\u5230\u53ef\u884c\u8def\u5f84".equals(error.getMessage()))typedStop="ORIGINAL_AVAILABILITY211_RESOURCE_SEED_FALLBACK";
                        else throw error;
                    }
                }else try(TimedObservationScope258 scope=TimedObservationScope258.open(timed)){
                    ExternalLaunchAdapter257.Result result=ExternalLaunchAdapter257.run(algorithm,population,iterations,kappa,cap,random,seed,cfg,goal,BenchmarkLauncher.COMPOSITE_W,context,max,revision,identity,warehouse,run,ExternalNativeAudit257.phases(),finalNative);lifecycle=result.lifecycle;near(result.verified.objective,observed.bestCost(),"COMMON337_EXTERNAL_CHAMPION_MATCH");
                }
            }finally{run.ledger.setActive(false);observed.close();bridge.write(out.resolve("search_clock164.json"),run.ledger);J.writeValue(out.resolve("common_observations.json").toFile(),observed.snapshot());}
            AvailabilityLauncher211.mark("FINAL_AUDIT");List<GridPoint> route=observed.bestRoute();Map<String,Object> after=audit(raw,route,max);double objective=((Number)((Map)after.get("native_five_goal_costs")).get(goalName)).doubleValue();near(objective,observed.bestCost(),"COMMON337_FINAL_CHAMPION");need(objective<=seedCost+1e-8*Math.max(1,seedCost),"COMMON337_SEED_GUARD");
            Map<String,Object> report=new LinkedHashMap<>();report.put("voyage_speed_model",Navigation.Navigation3D.VoyageSpeedModel.MODEL);report.put("scenario",scene);report.put("objective",goalName);report.put("algorithm",algorithm);report.put("final_arm",arm);report.put("flags",fsaFlags);report.put("seed",random);report.put("run_id",runId);report.put("seed_cost",seedCost);report.put("external_cost",objective);report.put("audit",after);report.put("ship",cfg.shipParameters);report.put("work",run.ledger.snapshot());report.put("screen",false);report.put("typed_fsa_stop",typedStop);report.put("external_lifecycle",lifecycle);report.put("candidate_evaluations217",CandidateEvaluation217.snapshot());report.put("normalizer_sha256",normalizer);report.put("config_sha256",ExternalEvaluationScope251.configFingerprint(cfg));report.put("identity_fields",ids);report.put("provider_revision",revision);report.put("loaded_class_sha256",loaded());report.put("common_observation_policy","Same217 classified complete/practical candidates; seed plus best already evaluated complete route. No extra search or raw call from observer; all observation cost in complete Job.");
            J.writeValue(out.resolve("report.json").toFile(),report);J.writeValue(out.resolve("result_points.json").toFile(),route);AvailabilityLauncher211.mark("DONE");System.out.println("RISK337_NATIVE_COMPLETE "+out);
        }
    }
}
