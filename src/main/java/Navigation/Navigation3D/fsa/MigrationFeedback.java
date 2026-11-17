package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import java.util.*;
import java.nio.charset.StandardCharsets;
import java.security.MessageDigest;

/** Small opt-in TIME operator. Biological inspiration never changes validity or acceptance. */
public final class MigrationFeedback {
    public enum Mode { FEEDBACK, PLAIN, RANDOM, SHORTCUT }
    public interface Host extends PassageRejoin.Host { double distanceBudget(); }
    public static final class Result {
        public final List<GridPoint> path;public final double cost;public final Map<String,Object> witness;
        Result(TimeEvaluator.Trace t,Map<String,Object> w){path=t.path;cost=t.cost;witness=w;}
    }
    private static final double EPS=1e-9;
    private final Host h;private final Mode mode;private final long seed;
    private long ordinary,slots;
    private final LinkedHashMap<String,Set<String>> history=new LinkedHashMap<>();
    private final List<Map<String,Object>> witnesses=new ArrayList<>();
    public MigrationFeedback(Mode mode,Host h,long seed){this.mode=mode;this.h=h;this.seed=seed;}
    public void ordinaryEvaluation(){ordinary++;}
    public List<Map<String,Object>> witnesses(){return Collections.unmodifiableList(witnesses);}
    private void count(String k){h.count("mf_"+k);}
    private double d(GridPoint a,GridPoint b){return h.distance(a,b);}
    private static final class Pair {
        final GridPoint q;final int b;final double length;
        Pair(GridPoint q,int b,double length){this.q=q;this.b=b;this.length=length;}
        String key(){return q.i+","+q.j+":"+b;}
    }
    private List<Pair> feasible(List<GridPoint> parent,double[] arc,int a,int b,double radius){
        GridPoint from=parent.get(a),end=parent.get(b);List<Pair> out=new ArrayList<>();
        for(GridPoint q:h.neighbors(from)){
            if(!h.allowed())break;count("geometry_points");
            if(parent.contains(q)||d(from,q)>radius+EPS||d(q,end)>radius+EPS||!h.edgeLegal(from,q)||!h.edgeLegal(q,end))continue;
            double length=d(from,q)+d(q,end);
            if(arc[a]+length+arc[arc.length-1]-arc[b]>h.distanceBudget()+EPS)continue;
            out.add(new Pair(q,b,length));
        }
        out.sort(Comparator.comparingDouble((Pair p)->p.length).thenComparingInt(p->p.q.i).thenComparingInt(p->p.q.j));return out;
    }
    private static String signature(List<Pair> pool){
        try{StringBuilder s=new StringBuilder();for(Pair p:pool)s.append(p.key()).append(';');
            byte[] bytes=MessageDigest.getInstance("SHA-256").digest(s.toString().getBytes(StandardCharsets.UTF_8));StringBuilder x=new StringBuilder();
            for(byte b:bytes)x.append(String.format("%02x",b&255));return x.toString();
        }catch(Exception e){throw new IllegalStateException(e);}
    }
    private List<GridPoint> pack(List<GridPoint> p,int a,Pair pair){
        List<GridPoint> out=new ArrayList<>(p.subList(0,a+1));if(pair.q!=null)out.add(pair.q);out.addAll(p.subList(pair.b,p.size()));return out;
    }
    private TimeEvaluator.Trace evaluate(List<GridPoint> parent,int a,Pair pair,String action,List<Map<String,Object>> logs){
        if(!h.allowed())return null;long work=h.work();TimeEvaluator.Trace t=h.evaluate(pack(parent,a,pair));
        Map<String,Object> row=new LinkedHashMap<>();row.put("action",action);row.put("q",pair.q);row.put("b",pair.b);row.put("path",t.path);
        row.put("status",t.status);row.put("cost",t.valid()?t.cost:null);row.put("arrival",t.arrival);row.put("work",h.work()-work);
        logs.add(row);count(action+"_evaluated");return t.valid()&&h.completedAllowed()?t:null;
    }
    /** The two actual leg averages are proposal signals, not a causal decomposition. */
    public static double[] response(double entry,double atGate,double atRejoin,double firstDistance,double secondDistance){
        if(!(firstDistance>0&&secondDistance>0&&atGate>entry&&atRejoin>atGate))throw new IllegalArgumentException("positive complete bridge required");
        return new double[]{(atGate-entry)/firstDistance,(atRejoin-atGate)/secondDistance};
    }
    public static boolean shouldRetarget(double out,double back){return back>out+1e-12*Math.max(1,Math.max(out,back));}

    public Result poll(List<GridPoint> parent,double parentCost){
        count("polls");if(!h.allowed()||ordinary<6*(slots+1))return null;slots++;count("opportunities");
        if(parent==null||parent.size()<3||!Double.isFinite(parentCost))return null;
        String version=parent.toString()+"@"+Double.toHexString(h.radius());
        if(!history.containsKey(version)&&history.size()>=128)history.remove(history.keySet().iterator().next());
        Set<String> seenWindows=history.computeIfAbsent(version,k->new HashSet<>());
        if(seenWindows.contains("EXHAUSTED")){count("no_fresh_window");return null;}
        TimeEvaluator.Trace base=h.evaluate(parent);count("parent_replays");
        if(!base.valid()||!h.completedAllowed())return null;
        if(Math.abs(base.cost-parentCost)>1e-7*Math.max(1,Math.abs(parentCost)))throw new IllegalStateException(
                "incumbent TIME mismatch: stored="+parentCost+", replay="+base.cost+", path="+parent);
        double radius=h.radius();double[] arc=new double[parent.size()];
        for(int k=1;k<arc.length;k++)arc[k]=arc[k-1]+d(parent.get(k-1),parent.get(k));
        List<int[]> windows=new ArrayList<>();Set<Integer> starts=new HashSet<>();
        for(int k=0;k<8;k++){
            int a=0;while(a<parent.size()-2&&arc[a]<arc[arc.length-1]*k/8.0)a++;
            if(!starts.add(a))continue;
            for(int b=a+2;b<parent.size();b++)if(d(parent.get(a),parent.get(b))<=2*radius+EPS&&!seenWindows.contains(a+":"+b))windows.add(new int[]{a,b});
        }
        windows.sort(Comparator.<int[]>comparingDouble(w->-(base.arrival[w[1]]-base.arrival[w[0]]-d(parent.get(w[0]),parent.get(w[1]))/h.nominalSpeed())).thenComparingInt(w->w[0]).thenComparingInt(w->w[1]));
        int scans=0,a=-1,b0=-1;List<Pair> pool=new ArrayList<>();List<Integer> bs=new ArrayList<>();
        for(int[] win:windows){
            if(scans>=6||!h.allowed())break;scans++;count("geometry_windows");
            List<Pair> f=feasible(parent,arc,win[0],win[1],radius);seenWindows.add(win[0]+":"+win[1]);
            if(!f.isEmpty()){a=win[0];b0=win[1];pool.addAll(f);bs.add(b0);break;}
        }
        if(a<0){if(windows.size()<=scans)seenWindows.add("EXHAUSTED");count("empty_supply");return null;}
        final int firstB=b0;List<Integer> more=new ArrayList<>();
        for(int b=a+2;b<parent.size();b++)if(b!=b0&&d(parent.get(a),parent.get(b))<=2*radius+EPS)more.add(b);
        more.sort(Comparator.<Integer>comparingInt(b->Math.abs(b-firstB)).thenComparingInt(b->b));
        for(int b:more){if(scans>=6||bs.size()>=3||!h.allowed())break;scans++;count("geometry_windows");
            List<Pair> f=feasible(parent,arc,a,b,radius);if(!f.isEmpty()){pool.addAll(f);bs.add(b);}}
        Map<String,Object> w=new LinkedHashMap<>();w.put("mode",mode.name());w.put("parent",new ArrayList<>(parent));w.put("parent_cost",parentCost);
        w.put("parent_arrival",base.arrival);w.put("a",a);w.put("b0",b0);w.put("rejoins",bs);w.put("radius_nm",radius);
        w.put("pool_signature",signature(pool));w.put("pair_supply_size",pool.size());w.put("geometry_checks",scans);
        List<Map<String,Object>> logs=new ArrayList<>();w.put("candidates",logs);List<Map<String,Object>> probes=new ArrayList<>();w.put("extra_probes",probes);
        if(witnesses.size()<256)witnesses.add(w);else count("witness_overflow");count("windows");count("legal_supply");
        TimeEvaluator.Trace best=base;
        for(int b:bs){
            if(!h.allowed())break;
            if(!h.edgeLegal(parent.get(a),parent.get(b)))continue;
            TimeEvaluator.Trace t=evaluate(parent,a,new Pair(null,b,d(parent.get(a),parent.get(b))),"shortcut",logs);
            if(t!=null&&t.cost<best.cost-EPS)best=t;
        }
        w.put("shortcut_best",best.cost);
        if(best.cost<parentCost-EPS)return finish(best,w,"shortcut",null);
        if(mode==Mode.SHORTCUT||!h.allowed())return null;
        Pair pair=pool.get(0);Set<String> used=new HashSet<>();Set<GridPoint> retargeted=new HashSet<>();
        Random rng=new Random(seed^parent.hashCode()*0x9E3779B97F4A7C15L^a^b0);int probeCount=0;
        double firstCost=Double.POSITIVE_INFINITY;
        for(int attempt=0;attempt<3&&h.allowed()&&pair!=null;attempt++){
            used.add(pair.key());String action=attempt==0?"initial":String.valueOf(w.get("next_action"));
            TimeEvaluator.Trace t=evaluate(parent,a,pair,action,logs);count("bridge_evaluations");
            if(t==null)return null;
            if(attempt==0){firstCost=t.cost;w.put("initial_cost",firstCost);}
            double[] rho=response(base.arrival[a],t.arrival[a+1],t.arrival[a+2],d(parent.get(a),pair.q),d(pair.q,parent.get(pair.b)));
            Map<String,Object> last=logs.get(logs.size()-1);last.put("rho_out",rho[0]);last.put("rho_return",rho[1]);
            last.put("bridge_arrival_difference",t.arrival[a+2]-base.arrival[pair.b]);
            last.put("tail_duration_difference",(t.cost-t.arrival[a+2])-(base.cost-base.arrival[pair.b]));
            if(t.cost<parentCost-EPS){w.put("extra_gain_over_initial",Math.max(0,Math.min(parentCost,firstCost)-t.cost));return finish(t,w,attempt==0?"initial_bridge":"continuation",action);}
            if(attempt==2)break;
            Pair next=null;
            if(mode==Mode.RANDOM){List<Pair> available=new ArrayList<>();for(Pair p:pool)if(!used.contains(p.key()))available.add(p);
                if(!available.isEmpty())next=available.get(rng.nextInt(available.size()));w.put("next_action","random");
            }else if(mode==Mode.PLAIN){
                // Ordinary local empirical-speed surrogate, given the SAME full trace and feasible relation.
                double score=Double.POSITIVE_INFINITY;
                for(Pair p:pool)if(!used.contains(p.key())){
                    double estimate=base.arrival[a]+rho[0]*d(parent.get(a),p.q)+rho[1]*d(p.q,parent.get(p.b))+base.cost-base.arrival[p.b];
                    if(estimate<score-EPS){score=estimate;next=p;}}
                w.put("next_action","model");
            }else{
                if(shouldRetarget(rho[0],rho[1])&&retargeted.add(pair.q)&&probeCount<2){
                    List<Pair> alternatives=new ArrayList<>();
                    for(Pair p:pool)if(p.q.equals(pair.q)&&!used.contains(p.key()))alternatives.add(p);
                    Pair lower=null,upper=null;
                    for(Pair p:alternatives){if(p.b<pair.b&&(lower==null||p.b>lower.b))lower=p;if(p.b>pair.b&&(upper==null||p.b<upper.b))upper=p;}
                    double score=Double.POSITIVE_INFINITY;
                    for(Pair p:Arrays.asList(lower,upper)){
                        if(p==null||probeCount>=2||!h.allowed())continue;
                        long before=h.work();double at=h.edgeArrival(pair.q,parent.get(p.b),t.arrival[a+1]);probeCount++;count("return_probe_samples");
                        Map<String,Object> log=new LinkedHashMap<>();log.put("q",pair.q);log.put("b",p.b);log.put("entry",t.arrival[a+1]);log.put("arrival",Double.isFinite(at)?at:null);log.put("work",h.work()-before);probes.add(log);
                        if(Double.isFinite(at)&&h.completedAllowed()&&at-base.arrival[p.b]<score-EPS){score=at-base.arrival[p.b];next=p;}
                    }
                    if(next!=null){w.put("next_action","retarget");count("response_retarget");}
                }
                if(next==null&&h.allowed()){
                    GridPoint from=parent.get(a);double x=(pair.q.j-from.j)*h.dx(),y=(pair.q.i-from.i)*h.dy(),progress=x*x+y*y;
                    double bestDistance=Double.POSITIVE_INFINITY,bestLength=Double.POSITIVE_INFINITY;
                    for(Pair p:pool)if(!used.contains(p.key())&&!p.q.equals(pair.q)){
                        double px=(p.q.j-from.j)*h.dx(),py=(p.q.i-from.i)*h.dy();
                        if(px*x+py*y>=progress-EPS)continue;
                        double distance=Math.hypot(px-x/2,py-y/2);
                        if(distance<bestDistance-EPS||(Math.abs(distance-bestDistance)<=EPS&&p.length<bestLength-EPS)){
                            next=p;bestDistance=distance;bestLength=p.length;}}
                    if(next!=null){w.put("next_action","retreat");count("response_retreat");}
                }
            }
            pair=next;
        }
        count("no_improvement");return null;
    }
    private Result finish(TimeEvaluator.Trace t,Map<String,Object> w,String origin,String action){
        w.put("origin",origin);w.put("action",action);w.put("result_path",t.path);w.put("result_cost",t.cost);count("improved_"+origin);
        return new Result(t,w);
    }
}
