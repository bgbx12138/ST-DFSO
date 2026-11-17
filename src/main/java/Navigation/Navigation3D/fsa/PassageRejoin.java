package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import java.util.*;

/** Opt-in, single-parent bridge operator. Biology motivates proposals, never acceptance. */
public final class PassageRejoin {
    public enum Mode { RANKED, FIXED, RANDOM, SHORTCUT, REPLAN }
    public interface Host extends AlarmWrapOperator.Host {
        double distance(GridPoint a, GridPoint b);
        double radius();
        long work();
    }
    public static final class Result {
        public final List<GridPoint> path;
        public final double cost;
        public final Map<String,Object> witness;
        Result(TimeEvaluator.Trace t, Map<String,Object> w) {path=t.path;cost=t.cost;witness=w;}
    }
    private final Host h;
    private final Mode mode;
    private final long seed;
    private long ordinary, slots;
    private final List<Map<String,Object>> witnesses=new ArrayList<>();
    // Bounded history; rotate through the ranked windows of an unchanged parent instead of redoing one.
    private final LinkedHashMap<List<GridPoint>,Integer> cursors=new LinkedHashMap<>();
    private static final double EPS=1e-9;
    public PassageRejoin(Mode mode,Host host,long seed){this.mode=mode;h=host;this.seed=seed;}
    public void ordinaryEvaluation(){ordinary++;}
    public List<Map<String,Object>> witnesses(){return Collections.unmodifiableList(witnesses);}
    private void count(String s){h.count("pr_"+s);}
    private double d(GridPoint a,GridPoint b){return h.distance(a,b);}
    private boolean bridgeEdge(GridPoint a,GridPoint b,double r){return !a.equals(b)&&d(a,b)<=r+EPS&&h.edgeLegal(a,b);}
    private static int advance(double[] arc,int start,double distance){
        if(start>=arc.length-1)return arc.length-1;
        int b=start+1;while(b<arc.length-1 && arc[b]-arc[start]<distance)b++;return b;
    }
    private static final class Pair {
        final int q,b;final double score,length;
        Pair(int q,int b,double score,double length){this.q=q;this.b=b;this.score=score;this.length=length;}
    }
    private List<GridPoint> pack(List<GridPoint> p,int a,List<GridPoint> middle,int b){
        List<GridPoint> out=new ArrayList<>(p.subList(0,a+1));out.addAll(middle);out.addAll(p.subList(b,p.size()));return out;
    }
    private TimeEvaluator.Trace evaluate(List<GridPoint> p,List<Map<String,Object>> logs,String kind,int b,GridPoint q){
        if(!h.allowed())return null;
        long start=h.work();TimeEvaluator.Trace t=h.evaluate(p);
        Map<String,Object> row=new LinkedHashMap<>();row.put("kind",kind);row.put("b",b);row.put("gate",q);
        row.put("path",p);row.put("status",t.status);row.put("cost",t.valid()?t.cost:null);
        row.put("arrival",t.arrival);row.put("work",h.work()-start);logs.add(row);count(kind+"_evaluated");
        return t.valid()&&h.completedAllowed()?t:null;
    }

    /** Regular opportunity: every six ordinary evaluations, with no risk/stagnation threshold. */
    public Result poll(int owner,List<GridPoint> parent,double parentCost){
        count("polls");
        if(!h.allowed() || ordinary<6*(slots+1))return null;
        slots++;count("opportunities");
        if(parent==null || parent.size()<3 || !Double.isFinite(parentCost))return null;
        double r=h.radius(),speed=h.nominalSpeed();
        if(!(r>0) || !(speed>0))throw new IllegalStateException("physical radius/speed required");
        TimeEvaluator.Trace base=h.evaluate(parent);count("parent_replays");
        if(!base.valid() || !h.completedAllowed())return null;
        if(Math.abs(base.cost-parentCost)>1e-7*Math.max(1,Math.abs(parentCost)))throw new IllegalStateException("parent TIME mismatch");
        long startWork=h.work();
        double[] arc=new double[parent.size()];for(int i=1;i<arc.length;i++)arc[i]=arc[i-1]+d(parent.get(i-1),parent.get(i));
        List<int[]> windows=new ArrayList<>();Set<Integer> starts=new HashSet<>();
        for(int k=0;k<8;k++){
            int a=0;double target=arc[arc.length-1]*k/8.0;
            while(a<arc.length-2 && arc[a]<target)a++;
            if(!starts.add(a))continue;
            int b=advance(arc,a,r);
            if(b>a+1 && d(parent.get(a),parent.get(b))>EPS)windows.add(new int[]{a,b});
        }
        windows.sort(Comparator.<int[]>comparingDouble(w->-(base.arrival[w[1]]-base.arrival[w[0]]-d(parent.get(w[0]),parent.get(w[1]))/speed)).thenComparingInt(w->w[0]));
        int cursor=cursors.getOrDefault(parent,0);
        if(cursor>=windows.size()){count("parent_windows_exhausted");return null;}
        if(!cursors.containsKey(parent)&&cursors.size()>=128)cursors.remove(cursors.keySet().iterator().next());
        cursors.put(new ArrayList<>(parent),cursor+1);
        int a=windows.get(cursor)[0],b0=windows.get(cursor)[1];GridPoint from=parent.get(a);
        List<Integer> bs=new ArrayList<>();
        for(int k=0;k<3;k++){
            int b=k==0?b0:advance(arc,b0,k*r);
            if(b>a && !bs.contains(b) && d(from,parent.get(b))<=2*r+EPS)bs.add(b);
        }
        Map<String,Object> w=new LinkedHashMap<>();w.put("owner",owner);w.put("mode",mode.name());w.put("parent",new ArrayList<>(parent));
        w.put("parent_cost",parentCost);w.put("parent_arrival",base.arrival);w.put("a",a);w.put("b0",b0);w.put("rejoins",bs);w.put("radius_nm",r);
        w.put("window_rank",cursor);w.put("parent_replay_work_included",true);
        List<Map<String,Object>> logs=new ArrayList<>();w.put("candidates",logs);
        if(witnesses.size()<256)witnesses.add(w);else count("witness_overflow");
        count("windows");if(bs.size()>1)count("multiple_rejoins");
        TimeEvaluator.Trace best=base;String origin="parent";
        for(int b:bs){
            if(!h.allowed())break;
            // The direct bridge is common to ALL increment arms, using exactly the same rejoin supply.
            if(!bridgeEdge(from,parent.get(b),2*r))continue;
            TimeEvaluator.Trace t=evaluate(pack(parent,a,Collections.emptyList(),b),logs,"shortcut",b,null);
            if(t!=null && t.cost<best.cost-EPS){best=t;origin="shortcut";}
        }
        double shortcutBest=best.cost;w.put("shortcut_best",shortcutBest);
        if(mode!=Mode.SHORTCUT && h.allowed()){
            if(mode==Mode.REPLAN){
                TimeEvaluator.Trace t=replan(parent,a,bs,base,logs,r);
                if(t!=null && t.cost<best.cost-EPS){best=t;origin="replan";}
            }else{
                List<GridPoint> qs=new ArrayList<>();GridPoint end=parent.get(b0);
                double x=(end.j-from.j)*h.dx(),y=(end.i-from.i)*h.dy(),length=Math.hypot(x,y);
                if(length>EPS)for(double offset:new double[]{r/4,-r/4,r/2,-r/2}){
                    // Work in nautical miles, then project; negative half ties go to the lower index.
                    double gx=(from.j+end.j)/2.0-y/length*offset/h.dx();
                    double gy=(from.i+end.i)/2.0+x/length*offset/h.dy();
                    GridPoint q=new GridPoint((int)Math.ceil(gy-0.5),(int)Math.ceil(gx-0.5));
                    if(parent.contains(q)||qs.contains(q)||!bridgeEdge(from,q,r))continue;
                    boolean reaches=false;for(int b:bs)if(bridgeEdge(q,parent.get(b),r)){reaches=true;break;}
                    if(reaches)qs.add(q);
                }
                w.put("gates",qs);if(!qs.isEmpty())count("legal_gate_supply");
                List<Pair> supply=new ArrayList<>();List<Double> arrivals=new ArrayList<>();
                for(int qi=0;qi<qs.size();qi++){
                    if(!h.allowed())break;
                    double time=h.edgeArrival(from,qs.get(qi),base.arrival[a]);count("scout_samples");
                    arrivals.add(Double.isFinite(time)?time:null);
                    if(!Double.isFinite(time)||!h.completedAllowed())continue;
                    for(int b:bs)if(bridgeEdge(qs.get(qi),parent.get(b),r))supply.add(new Pair(qi,b,time+d(qs.get(qi),parent.get(b))/speed+base.cost-base.arrival[b],d(from,qs.get(qi))+d(qs.get(qi),parent.get(b))));
                }
                w.put("scout_arrivals",arrivals);
                supply.sort(Comparator.comparingDouble((Pair p)->p.score).thenComparingDouble(p->p.length).thenComparingInt(p->p.q).thenComparingInt(p->p.b));
                List<List<Integer>> ids=new ArrayList<>();for(Pair p:supply)ids.add(Arrays.asList(p.q,p.b));w.put("pair_supply",ids);
                if(mode==Mode.RANDOM)Collections.shuffle(supply,new Random(seed^parent.hashCode()*0x9E3779B97F4A7C15L^a));
                if(mode==Mode.FIXED)supply.removeIf(p->p.b!=b0);
                int tested=0;
                for(Pair pair:supply){
                    if(tested>=3||!h.allowed())break;tested++;count("bridge_proposals");
                    GridPoint q=qs.get(pair.q);
                    // Public evaluator replays prefix, both bridge legs and the entire shifted tail.
                    // Scouts are charged as extra work; no old tail duration is used for acceptance.
                    TimeEvaluator.Trace t=evaluate(pack(parent,a,Collections.singletonList(q),pair.b),logs,"bridge",pair.b,q);
                    if(t!=null && t.cost<best.cost-EPS){best=t;origin="bridge";}
                }
                w.put("pairs_tested",tested);
            }
        }
        w.put("best_cost",best.cost);w.put("origin",origin);w.put("extra_gain_over_shortcut",Math.max(0,shortcutBest-best.cost));
        w.put("parent_gain",Math.max(0,parentCost-best.cost));w.put("post_parent_work",h.work()-startWork);
        if(best.cost<parentCost-EPS && h.completedAllowed()){
            w.put("result_path",best.path);count("improved_"+origin);return new Result(best,w);
        }
        return null;
    }

    private static final class Label {
        final List<GridPoint> route;final double time,priority;final long id;
        Label(List<GridPoint> p,double t,double f,long id){route=p;time=t;priority=f;this.id=id;}
    }
    /** Ordinary bounded time-dependent best-first search, no fish/gate advice.
     * Keeps path states (no invalid FIFO/earliest-arrival dominance assumption), up to 256 expansions.
     * Same B, current-time sampler, physical local box and global work/wall caps; own grid nodes allowed.
     */
    private TimeEvaluator.Trace replan(List<GridPoint> parent,int a,List<Integer> bs,TimeEvaluator.Trace base,List<Map<String,Object>> logs,double r){
        GridPoint from=parent.get(a);long id=0;
        PriorityQueue<Label> open=new PriorityQueue<>(Comparator.comparingDouble((Label l)->l.priority).thenComparingLong(l->l.id));
        open.add(new Label(Collections.singletonList(from),base.arrival[a],base.arrival[a],id++));
        Set<GridPoint> forbidden=new HashSet<>(parent.subList(0,a));
        Set<List<GridPoint>> seen=new HashSet<>();TimeEvaluator.Trace best=null;int expanded=0,tested=0;
        while(!open.isEmpty()&&expanded<256&&h.allowed()){
            Label l=open.poll();expanded++;GridPoint last=l.route.get(l.route.size()-1);count("replan_expansions");
            List<GridPoint> next=new ArrayList<>();for(int b:bs)if(!next.contains(parent.get(b)))next.add(parent.get(b));
            for(GridPoint p:h.neighbors(last))if(!next.contains(p))next.add(p);
            for(GridPoint p:next){
                if(!h.allowed())break;
                if(forbidden.contains(p)||l.route.contains(p)||d(from,p)>2*r+EPS||!bridgeEdge(last,p,r))continue;
                List<GridPoint> chain=new ArrayList<>(l.route);chain.add(p);if(!seen.add(chain))continue;
                double time=h.edgeArrival(last,p,l.time);count("replan_edge_samples");
                if(!Double.isFinite(time)||!h.completedAllowed())continue;
                int b=parent.indexOf(p);
                if(bs.contains(b)&&chain.size()>2){
                    List<GridPoint> path=pack(parent,a,chain.subList(1,chain.size()-1),b);
                    TimeEvaluator.Trace t=evaluate(path,logs,"replan",b,null);tested++;
                    if(t!=null&&(best==null||t.cost<best.cost-EPS))best=t;
                }
                // Goal labels may continue to another goal; preserve multiple arrival-time/path states.
                if(chain.size()<=32){double tail=Double.POSITIVE_INFINITY;
                    for(int goal:bs)tail=Math.min(tail,d(p,parent.get(goal))/h.nominalSpeed()+base.cost-base.arrival[goal]);
                    open.add(new Label(chain,time,time+tail,id++));}
            }
        }
        return best;
    }
}
