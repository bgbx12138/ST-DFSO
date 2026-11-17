package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import java.util.*;

/** Optional long-range route renewal. All sources are already paid, valid complete routes.
 * Route transmission is a biological inspiration, not a claim that this kernel is new mathematics. */
public final class MigrationRenewal {
    public enum Mode { PERSONAL, SOCIAL }
    public static final class State {
        public final int owner,generation; public final List<GridPoint> path; public final double cost;
        State(int owner,int generation,List<GridPoint> path,double cost){this.owner=owner;this.generation=generation;this.path=copy(path);this.cost=cost;}
    }
    public static final class Plan {
        public final List<GridPoint> prefix,source; public final GridPoint start;public final int cut;
        public final Map<String,Object> witness=new LinkedHashMap<>();
        Plan(State s,int cut,int recipient,int generation,Mode mode,String kind,double fraction){
            this.source=s.path;this.cut=cut;prefix=copy(s.path.subList(0,cut+1));start=prefix.get(prefix.size()-1);
            witness.put("mode",mode.name());witness.put("recipient",recipient);witness.put("generation",generation);
            witness.put("source_owner",s.owner);witness.put("source_generation",s.generation);witness.put("source_kind",kind);
            witness.put("source_path",source);witness.put("source_cost",s.cost);witness.put("cut",cut);witness.put("cut_target_arc_fraction",fraction);witness.put("construction_prefix",prefix);
        }
    }
    private final Mode mode;private final List<GridPoint> seed;private final double seedCost,dx,dy;
    private final Map<Integer,State> recent=new TreeMap<>();private List<State> frozen=Collections.emptyList();
    private final List<Map<String,Object>> witnesses=new ArrayList<>();private int generation;
    private final Map<String,Long> totals=new LinkedHashMap<>();
    private void count(String key){totals.put(key,totals.getOrDefault(key,0L)+1);}
    public MigrationRenewal(Mode mode,List<GridPoint> seed,double seedCost,double dx,double dy){
        this.mode=Objects.requireNonNull(mode);this.seed=copy(seed);this.seedCost=seedCost;this.dx=dx;this.dy=dy;
    }
    private static List<GridPoint> copy(List<GridPoint> p){return Collections.unmodifiableList(new ArrayList<>(p));}
    public void beginGeneration(int current){
        generation=current;recent.values().removeIf(s->current-s.generation>2);
        frozen=new ArrayList<>(recent.values()); // Frozen before workers: never read this generation's new arrivals.
    }
    public void observe(int owner,List<GridPoint> path,double cost){
        if(path==null||path.size()<2||path.equals(seed)||!Double.isFinite(cost)||cost>=1e17||!path.get(0).equals(seed.get(0))||!path.get(path.size()-1).equals(seed.get(seed.size()-1))||new HashSet<>(path).size()!=path.size())return;
        recent.put(owner,new State(owner,generation,path,cost)); // A valid transient can be worse; global best is separate.
    }
    private double distance(GridPoint a,GridPoint b){return Math.hypot((a.i-b.i)*dy,(a.j-b.j)*dx);}
    private double pointToRoute(GridPoint q,List<GridPoint> p){
        double best=Double.POSITIVE_INFINITY;
        for(int k=1;k<p.size();k++){
            GridPoint a=p.get(k-1),b=p.get(k);double vx=(b.j-a.j)*dx,vy=(b.i-a.i)*dy;
            double qx=(q.j-a.j)*dx,qy=(q.i-a.i)*dy,t=Math.max(0,Math.min(1,(qx*vx+qy*vy)/Math.max(1e-20,vx*vx+vy*vy)));
            best=Math.min(best,Math.hypot(qx-t*vx,qy-t*vy));
        }return best;
    }
    private double difference(List<GridPoint> a,List<GridPoint> b){
        double d=0;for(GridPoint p:a)d=Math.max(d,pointToRoute(p,b));for(GridPoint p:b)d=Math.max(d,pointToRoute(p,a));return d;
    }
    public Plan plan(Fish fish){
        if(fish.id%3==0)return null; // Original cohort retains all original construction/imitation behavior.
        State own=null;for(State s:frozen)if(s.owner==fish.id)own=s;
        if(own==null){List<GridPoint> p=fish.personalBestPath;own=p!=null&&p.size()>=2&&Double.isFinite(fish.personalBestCost)?new State(fish.id,generation-1,p,fish.personalBestCost):new State(fish.id,0,seed,seedCost);}
        State chosen=own;String kind="PERSONAL";
        if(mode==Mode.SOCIAL&&fish.id%3==2){
            State best=null;double diversity=1e-9;
            for(State s:frozen)if(s.owner!=fish.id){double d=difference(s.path,own.path);if(d>diversity+1e-12||(Math.abs(d-diversity)<1e-12&&best!=null&&s.cost<best.cost)){diversity=d;best=s;}}
            if(best!=null){chosen=best;kind="SOCIAL_PREVIOUS_GENERATION";}
        }
        // Renew >= half the source's physical arclength, rather than a fixed pair of legs.
        double fraction=new double[]{0,.25,.5}[Math.floorMod(fish.id+generation,3)];
        double total=0;for(int k=1;k<chosen.path.size();k++)total+=distance(chosen.path.get(k-1),chosen.path.get(k));
        int cut=0;double arc=0;
        for(int k=1;k<chosen.path.size()-1;k++){double next=arc+distance(chosen.path.get(k-1),chosen.path.get(k));if(next>fraction*total+1e-12)break;arc=next;cut=k;}
        Plan plan=new Plan(chosen,cut,fish.id,generation,mode,kind,fraction);
        plan.witness.put("actual_prefix_arc_fraction",total>0?arc/total:0);return plan;
    }
    public void record(Plan plan,List<GridPoint> result,double cost,double personalBefore,double incumbentBefore){
        if(plan==null)return;Map<String,Object> w=plan.witness;
        w.put("result_path",result==null?null:new ArrayList<>(result));w.put("result_cost",Double.isFinite(cost)?cost:null);
        w.put("same_as_source",result!=null&&result.equals(plan.source));w.put("fallback_seed",result!=null&&result.equals(seed));
        boolean valid=result!=null&&result.size()>=2&&Double.isFinite(cost)&&cost<1e17;
        w.put("valid",valid);w.put("new_geometry",valid&&!result.equals(seed)&&!RequestMigration.shapeKey(result).equals(RequestMigration.shapeKey(plan.source)));
        w.put("transient_worse_than_personal",valid&&!result.equals(seed)&&cost>personalBefore+1e-9);
        w.put("personal_before",Double.isFinite(personalBefore)?personalBefore:null);w.put("incumbent_before",incumbentBefore);
        w.put("committed_incumbent",valid&&cost<incumbentBefore-1e-9);w.put("incumbent_gain",valid?Math.max(0,incumbentBefore-cost):0);
        count("completed_results");if(valid)count("valid_results");
        if(Boolean.TRUE.equals(w.get("fallback_seed")))count("seed_fallbacks");
        if(Boolean.TRUE.equals(w.get("new_geometry")))count("new_geometry_results");
        if(Boolean.TRUE.equals(w.get("transient_worse_than_personal")))count("valid_worse_transients");
        if("SOCIAL_PREVIOUS_GENERATION".equals(w.get("source_kind")))count("social_source_results");
        if(Boolean.TRUE.equals(w.get("committed_incumbent"))){count("global_commits");if(((Number)w.get("source_cost")).doubleValue()>incumbentBefore+1e-9)count("global_commits_from_worse_source");}
        if(witnesses.size()<128)witnesses.add(w);
        else {count("witnesses_omitted");if(Boolean.TRUE.equals(w.get("committed_incumbent")))for(int k=0;k<witnesses.size();k++)if(!Boolean.TRUE.equals(witnesses.get(k).get("committed_incumbent"))){witnesses.set(k,w);break;}}
    }
    public List<Map<String,Object>> witnesses(){return new ArrayList<>(witnesses);}
    public Map<String,Long> statistics(){return new LinkedHashMap<>(totals);}
}
