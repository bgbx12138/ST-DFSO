package Navigation.benchmark.algo;

import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.SearchWorkLedger;
import Navigation.benchmark.*;
import Navigation.model.GridPoint;
import Navigation.model.GridRouteGraph;
import Navigation.object.NavigationConfig;
import java.util.*;

/** Ordinary ILS with variable-neighborhood descent; no social cue or fish mechanism. */
public final class WindowIlsOptimizer169 implements HybridSwarmOptimizer {
    private final int cycles;
    private final double kappa;
    private final long cap;
    private ExternalRun166 run;
    private final Map<String,Double> cache = new HashMap<>();
    private static final int MAX_CACHE = 100000;
    private static final class Candidate {
        final List<GridPoint> route; final double cost;
        Candidate(List<GridPoint> route,double cost){this.route=route;this.cost=cost;}
    }
    public WindowIlsOptimizer169(int cycles,double kappa,long cap){
        if(cycles<1 || kappa<1 || cap<1)throw new IllegalArgumentException("bounded ILS settings");
        this.cycles=cycles;this.kappa=kappa;this.cap=cap;
    }
    public WindowIlsOptimizer169 withRun(ExternalRun166 run){this.run=Objects.requireNonNull(run);return this;}
    @Override public String algorithmName(){return "Ordinary-Window-ILS169";}

    @Override public SwarmOptResult optimize(List<GridPoint> seed,NavigationConfig config,
            HybridFSA3D.ObjectiveMode mode,ObjectiveWeights weights,long randomSeed){
        if(seed==null || seed.size()<2 || run==null)throw new IllegalArgumentException("valid seed/run required");
        cache.clear();SearchWorkLedger work=run.ledger;Random rng=new Random(randomSeed);
        try(FairPathEvaluator166 evaluator=new FairPathEvaluator166(config,work)){
            GridRouteGraph graph=evaluator.getRouteGraph();double max=evaluator.configureDistanceBudgetFromSeed(seed,kappa);
            run.start(cap);double initial=run.checkedSeed(evaluator.evaluateStrict(seed,mode,weights,graph,max));
            Candidate best=new Candidate(copy(seed),initial),current=best;
            cache.put(key(seed),initial);List<Double> history=new ArrayList<>();history.add(initial);
            run.checkpoint(initial,true,"initialization_boundary");long start=System.currentTimeMillis();
            for(int cycle=0;cycle<cycles && !work.shouldStopSearch();cycle++){
                work.recordOperatorEvent("ordinary_descent_cycles");
                boolean changed=false;
                for(int neighborhood=0;neighborhood<3 && !work.shouldStopSearch();neighborhood++){
                    Candidate next=improve(current,neighborhood,evaluator,graph,max,mode,weights,rng,work);
                    if(next!=null){
                        current=next;changed=true;work.recordOperatorEvent("ordinary_descent_accepts_N"+neighborhood);
                        if(current.cost<best.cost){best=current;run.checkpoint(best.cost,false,"improvement_boundary");}
                        break;
                    }
                }
                if(!changed && !work.shouldStopSearch()){
                    // Perturb the champion, retain the global champion, and descend again.
                    // Accepting a worse current state is ordinary ILS diversification.
                    current=perturb(best,evaluator,graph,max,mode,weights,rng,work);
                    work.recordOperatorEvent("ordinary_perturbation_cycles");
                    if(current.cost<best.cost){best=current;run.checkpoint(best.cost,false,"improvement_boundary");}
                }
                history.add(best.cost);
            }
            run.finish(best.cost);
            return new SwarmOptResult(algorithmName(),"",mode.name(),0,randomSeed,
                Double.NaN,Double.NaN,Double.NaN,Double.NaN,best.cost,System.currentTimeMillis()-start,
                1,cycles,history,null,copy(best.route),false,work.snapshot());
        }catch(Exception e){throw new RuntimeException("ordinary window ILS169 failed",e);}
    }

    private Candidate improve(Candidate current,int neighborhood,FairPathEvaluator166 evaluator,
            GridRouteGraph graph,double max,HybridFSA3D.ObjectiveMode goal,ObjectiveWeights weights,
            Random rng,SearchWorkLedger work){
        List<int[]> windows=new ArrayList<>();int n=current.route.size();
        for(int a=0;a<n-2;a++)for(int b=a+2;b<n;b++){
            if(neighborhood==1 && b!=a+2)continue;
            if(neighborhood==2 && b<a+3)continue;
            if(neighborhood==0 && !graph.isValidEdge(current.route.get(a),current.route.get(b)))continue;
            if(neighborhood>0 && distance(current.route.get(a),current.route.get(b))>24+1e-9)continue;
            windows.add(new int[]{a,b});
        }
        Collections.shuffle(windows,rng);
        for(int[] window:windows){
            if(work.shouldStopSearch())break;
            int a=window[0],b=window[1];
            if(neighborhood==0){
                work.recordOperatorEvent("ordinary_shortcut_trials");
                Candidate trial=evaluate(splice(current.route,a,b,null),evaluator,graph,max,goal,weights,work);
                if(trial!=null && trial.cost<current.cost)return trial;
            }else{
                List<GridPoint> choices=lens(current.route,a,b,graph,work);Collections.shuffle(choices,rng);
                for(GridPoint q:choices){
                    if(work.shouldStopSearch())break;
                    work.recordOperatorEvent(neighborhood==1?"ordinary_relocation_trials":"ordinary_bridge_trials");
                    Candidate trial=evaluate(splice(current.route,a,b,q),evaluator,graph,max,goal,weights,work);
                    if(trial!=null && trial.cost<current.cost)return trial;
                }
            }
        }
        return null;
    }

    private Candidate perturb(Candidate champion,FairPathEvaluator166 evaluator,GridRouteGraph graph,double max,
            HybridFSA3D.ObjectiveMode goal,ObjectiveWeights weights,Random rng,SearchWorkLedger work){
        Candidate current=champion;
        int changes=1+rng.nextInt(3);
        for(int c=0;c<changes && !work.shouldStopSearch();c++){
            int n=current.route.size();if(n<3)break;
            int a=rng.nextInt(n-2);int b=Math.min(n-1,a+2+rng.nextInt(2));
            List<GridPoint> choices=lens(current.route,a,b,graph,work);Collections.shuffle(choices,rng);
            for(GridPoint q:choices){
                if(work.shouldStopSearch())break;
                Candidate trial=evaluate(splice(current.route,a,b,q),evaluator,graph,max,goal,weights,work);
                if(trial!=null){current=trial;work.recordOperatorEvent("ordinary_perturbation_moves");break;}
            }
        }
        return current;
    }

    private List<GridPoint> lens(List<GridPoint> route,int a,int b,GridRouteGraph graph,SearchWorkLedger work){
        Set<GridPoint> fixed=new HashSet<>();
        for(int i=0;i<=a;i++)fixed.add(route.get(i));
        for(int i=b;i<route.size();i++)fixed.add(route.get(i));
        List<GridPoint> choices=new ArrayList<>();
        for(GridPoint q:graph.neighbors(route.get(a))){
            if(work.shouldStopSearch())break;
            if(!fixed.contains(q) && graph.isValidEdge(q,route.get(b)))choices.add(q);
        }
        return choices;
    }

    private Candidate evaluate(List<GridPoint> route,FairPathEvaluator166 evaluator,GridRouteGraph graph,double max,
            HybridFSA3D.ObjectiveMode goal,ObjectiveWeights weights,SearchWorkLedger work){
        if(work.shouldStopSearch())return null;
        String key=key(route);Double cost=cache.get(key);
        if(cost!=null)work.recordOperatorEvent("ordinary_exact_route_cache_hits");
        else{
            // Every novel ordered complete route is re-evaluated from departure time0.
            // This per-run cache cannot collide across scene/day/hour or partial windows.
            cost=evaluator.evaluateStrict(route,goal,weights,graph,max);
            if(ExternalRun166.valid(cost) && cache.size()<MAX_CACHE)cache.put(key,cost);
        }
        return ExternalRun166.valid(cost)?new Candidate(route,cost):null;
    }
    private static String key(List<GridPoint> route){
        StringBuilder b=new StringBuilder();for(GridPoint p:route)b.append(p.i).append(',').append(p.j).append(';');return b.toString();
    }
    private static List<GridPoint> splice(List<GridPoint> route,int a,int b,GridPoint middle){
        List<GridPoint> next=new ArrayList<>();next.addAll(route.subList(0,a+1));
        if(middle!=null)next.add(middle);next.addAll(route.subList(b,route.size()));return next;
    }
    private static List<GridPoint> copy(List<GridPoint> route){
        List<GridPoint> next=new ArrayList<>();for(GridPoint p:route)next.add(new GridPoint(p.i,p.j));return next;
    }
    private static double distance(GridPoint a,GridPoint b){return Math.hypot(a.i-b.i,a.j-b.j);}
}
