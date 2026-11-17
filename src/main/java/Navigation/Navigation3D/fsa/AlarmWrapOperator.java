package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import java.util.*;

/** Bounded, opt-in TIME increment. All probes and search edges use the host's budget. */
public final class AlarmWrapOperator {
    public interface Host {
        TimeEvaluator.Trace evaluate(List<GridPoint> path);
        boolean allowed();
        default boolean completedAllowed() { return allowed(); }
        boolean edgeLegal(GridPoint a, GridPoint b);
        double edgeArrival(GridPoint a, GridPoint b, double elapsed);
        List<GridPoint> neighbors(GridPoint point);
        void count(String name);
        double dx(); double dy(); double nominalSpeed();
    }
    public static final class Result {
        public final List<GridPoint> path;
        public final double cost;
        public final Map<String,Object> witness;
        Result(TimeEvaluator.Trace t, Map<String,Object> w) { path=t.path;cost=t.cost;witness=w; }
    }
    private static final double EPS=1e-9;
    private final String mode;
    private final Host host;
    private long ordinary, spent, nextId;
    private final Deque<Event> queue=new ArrayDeque<>();
    private final Set<String> seenEvents=new HashSet<>();
    private final Map<String,Integer> cursors=new LinkedHashMap<>();
    private final Map<String,Deque<int[]>> pendingWindows=new HashMap<>();
    private final List<Map<String,Object>> witnesses=new ArrayList<>();
    private boolean fillNext=true;
    private final SelectiveFollowOperator selective;

    public AlarmWrapOperator(String mode, Host host) {
        this(mode,host,0L);
    }
    public AlarmWrapOperator(String mode, Host host,long randomSeed) {
        if (!Arrays.asList("ALARM_WRAP","SINGLE_SECTION","LOCAL_REPLAN","LOCAL_FOLLOW","RANDOM_SECTION").contains(mode))
            throw new IllegalArgumentException("unknown alarm/control mode: "+mode);
        this.mode=mode;this.host=host;
        selective=isSelectiveMode(mode)?new SelectiveFollowOperator(mode,host,randomSeed):null;
    }
    public static boolean isSelectiveMode(String mode){return "LOCAL_FOLLOW".equals(mode)||"RANDOM_SECTION".equals(mode);}
    public void ordinaryEvaluation() { ordinary++; }
    public List<Map<String,Object>> witnesses() { return selective==null?Collections.unmodifiableList(witnesses):selective.witnesses(); }
    public List<Map<String,Object>> transactions(){return selective==null?Collections.emptyList():selective.transactions();}
    private void count(String key) { host.count("aw_"+key); }

    /** One metered opportunity per three ordinary route evaluations, shared by all new arms. */
    public Result poll(int owner, List<GridPoint> parent, double parentCost) {
        count("polls");
        if (!host.allowed() || ordinary < 3*(spent+1)) return null;
        spent++; count("quota_slots");
        if(selective!=null){
            Event e=scan(owner,parent,parentCost);
            return e==null?null:selective.apply(e.owner,e.id,e.parent,e.parentCost,e.a,e.b,e.chains[0]);
        }
        if (queue.isEmpty() || (queue.size()<2 && fillNext)) {
            fillNext=false;
            Event fresh=scan(owner,parent,parentCost);
            if (fresh!=null) {
                count("shortcut_evaluations");
                TimeEvaluator.Trace t=host.evaluate(fresh.shortcut);
                if ("ABORTED".equals(t.status) || !host.completedAllowed()) {count("aborted");return null;}
                if (t.valid() && t.cost < parentCost-EPS) {
                    count("direct_shortcut_improvements");
                    return result(fresh,"shortcut",-1,null,t,0);
                }
                fresh.trigger=t.status;queue.addLast(fresh);count("events_opened");
                count(t.valid()?"cost_alarms":"invalid_alarms");
                return null;
            }
        }
        if (queue.isEmpty()) {count("empty_supply_slots");return null;}
        fillNext=true;
        Event e=queue.removeFirst();e.visits++;count("event_continuations");
        Result r;
        if (e.base==null) {
            count("parent_trace_evaluations");e.base=host.evaluate(e.parent);e.calls++;
            if (!e.base.valid()) {e.closed=true;count("parent_trace_rejected");}
            else if (Math.abs(e.base.cost-e.parentCost)>1e-7*Math.max(1,Math.abs(e.parentCost)))
                throw new IllegalStateException("alarm parent differs from authoritative TIME score");
            r=null;
        } else if (mode.equals("LOCAL_REPLAN")) r=replan(e);
        else if (mode.equals("SINGLE_SECTION")) r=single(e);
        else r=swarm(e);
        int visitCap=mode.equals("LOCAL_REPLAN")?266:18;
        if (e.closed || e.visits>=visitCap || e.calls>=9 || !host.allowed()) {
            count("events_closed");if(e.visits>=visitCap)count("event_visit_cap");
        } else queue.addLast(e);
        return r;
    }

    private Event scan(int owner,List<GridPoint> p,double cost) {
        count("active_scans");
        if(seenEvents.size()>=4096){count("event_history_cap");return null;}
        if (p==null || p.size()<4 || !Double.isFinite(cost)) {count("short_parent");return null;}
        String hash=rawKey(p);int cursor=cursors.getOrDefault(hash,0), n=p.size();
        if (!cursors.containsKey(hash) && cursors.size()>=64){String oldest=cursors.keySet().iterator().next();cursors.remove(oldest);pendingWindows.remove(oldest);}
        Deque<int[]> pending=pendingWindows.computeIfAbsent(hash,k->new ArrayDeque<>());
        List<int[]> eligible=new ArrayList<>();Map<Integer,Double> gains=new HashMap<>();
        int total=(n-1)*5;
        for(int scan=0;pending.isEmpty() && scan<24 && cursor<total && host.allowed();scan++,cursor++) {
            int a=cursor/5, m=6-cursor%5, b=a+m+1;
            if(b>=n)continue;
            String eventKey=hash+":"+a+":"+b;
            if(seenEvents.contains(eventKey))continue;
            double length=0;for(int k=a;k<b;k++)length+=distance(p.get(k),p.get(k+1));
            double gain=length-distance(p.get(a),p.get(b));
            if(gain<=EPS)continue;
            Event e=makeEvent(owner,p,cost,a,b);
            if(e==null)continue;
            if(key(e.shortcut).equals(key(p)))continue;
            double shortcutLength=routeLength(e.shortcut),parentLength=routeLength(p);
            if(shortcutLength>=parentLength-EPS)continue;
            eligible.add(new int[]{a,b});gains.put(a*8+b-a,gain);
        }
        eligible.sort(Comparator.<int[]>comparingDouble(w->-gains.get(w[0]*8+w[1]-w[0])).thenComparingInt(w->w[0]).thenComparingInt(w->w[1]));
        pending.addAll(eligible);
        cursors.put(hash,cursor);
        if(pending.isEmpty()){count(cursor>=total?"windows_exhausted":"no_eligible_window");return null;}
        int[] window=pending.removeFirst();Event best=makeEvent(owner,p,cost,window[0],window[1]);
        seenEvents.add(hash+":"+best.a+":"+best.b);
        best.id=++nextId;count("eligible_windows");return best;
    }

    private Event makeEvent(int owner,List<GridPoint> path,double cost,int a,int b) {
        GridPoint start=path.get(a),end=path.get(b);
        double vx=(end.j-start.j)*host.dx(),vy=(end.i-start.i)*host.dy(),den=vx*vx+vy*vy;
        if(den<EPS)return null;
        int m=b-a-1;Event e=new Event(owner,path,cost,a,b,m);
        double sum=0;for(int k=a;k<b;k++)sum+=distance(path.get(k),path.get(k+1));
        double traversed=0,previous=-EPS;int sign=0,nonzero=0;
        for(int j=0;j<m;j++) {
            GridPoint p=path.get(a+j+1);
            traversed+=distance(path.get(a+j),p);
            double px=(p.j-start.j)*host.dx(),py=(p.i-start.i)*host.dy();
            double projection=(px*vx+py*vy)/den,cross=vx*py-vy*px;
            if(projection<previous-EPS || projection>1+EPS || projection< -EPS){count("nonmonotone_window");return null;}
            previous=projection;
            int s=cross>EPS?1:cross< -EPS?-1:0;
            if(s!=0 && sign!=0 && s!=sign){count("crossing_window");return null;}if(s!=0)sign=s;
            double fraction=traversed/sum;
            GridPoint q=new GridPoint((int)Math.round(start.i+fraction*(end.i-start.i)),(int)Math.round(start.j+fraction*(end.j-start.j)));
            GridPoint mirror=new GridPoint((int)Math.round(start.i+(2*projection*vy-py)/host.dy()),
                    (int)Math.round(start.j+(2*projection*vx-px)/host.dx()));
            e.chains[0].add(line(q,p));if(selective==null)e.chains[1].add(line(q,mirror));
            if(!q.equals(p))nonzero++;
        }
        if(nonzero==0){count("zero_retreat_window");return null;}
        List<GridPoint> raw=new ArrayList<>(path.subList(0,a+1));raw.add(end);raw.addAll(path.subList(b+1,path.size()));
        e.shortcut=pack(raw).path;e.seen.add(key(path));e.seen.add(key(e.shortcut));return e;
    }

    private Result swarm(Event e) {
        while(e.stage<8 && host.allowed()) {
            int stage=e.stage++,side=stage<4?stage/2:(stage-4)%2;
            int[] k;
            if(stage<4) {
                k=new int[e.m];boolean reverse=stage%2==1;
                for(int j=0;j<e.m;j++) {
                    int d=e.chains[side].get(j).size()-1;
                    boolean tight=(j<(e.m+1)/2)!=reverse;
                    // Floor at the tight end deliberately makes one-cell chains actionable.
                    k[j]=tight?d/4:(3*d+3)/4;
                }
            } else if(stage<6) k=fuse(e,side);
            else k=respond(e,side);
            if(k==null)continue;
            Packed packed=build(e,side,k);
            if(!e.seen.add(key(packed.path))){count("duplicate_candidates");continue;}
            count("candidate_generated");e.calls++;
            if(stage>=4 && stage<6 && e.lastFusionSources[side]>=2)count("multi_source_generated");
            TimeEvaluator.Trace trace=host.evaluate(packed.path);
            if(!host.completedAllowed() || "ABORTED".equals(trace.status)){e.closed=true;count("aborted");return null;}
            Probe probe=new Probe(side,k,trace,packed.map,stage);e.probes.add(probe);
            if(stage>=4 && stage<6)e.fused[side]=probe;
            if(trace.valid())count("candidate_valid");
            return trace.valid() && trace.cost<e.parentCost-EPS?
                    result(e,stage<4?"probe":stage<6?"fusion":"response",side,k,trace,probe.sources):null;
        }
        e.closed=true;return null;
    }
    private int[] fuse(Event e,int side) {
        int[] k=new int[e.m];Set<Integer> sources=new HashSet<>();
        for(int j=0;j<e.m;j++) {
            Probe best=null;double bestDelay=Double.POSITIVE_INFINITY;
            for(Probe p:e.probes)if(p.side==side && p.stage<4) {
                double delay=local(p,e,j);
                if(Double.isFinite(delay) && (delay<bestDelay-EPS || (Math.abs(delay-bestDelay)<=EPS && (best==null || p.k[j]<best.k[j])))) {
                    best=p;bestDelay=delay;
                }
            }
            int d=e.chains[side].get(j).size()-1;
            if(best!=null){k[j]=best.k[j];sources.add(best.stage);}
            else k[j]=d;
        }
        e.lastFusionSources[side]=sources.size();count("fusion_proposals");
        if(sources.size()>=2)count("multi_source_fusion");return k;
    }
    private int[] respond(Event e,int side) {
        Probe p=e.fused[side];if(p==null)return null;
        int[] k=p.k.clone();boolean changed=false;
        if(p.trace.firstInvalidEdge>=0) {
            for(int j=0;j<e.m;j++) {
                int logical=e.a+j+1,edge=p.trace.firstInvalidEdge;
                if(edge>=p.map[logical-1] && edge<p.map[logical+1]) {
                    int d=e.chains[side].get(j).size()-1,next=d;
                    for(Probe q:e.probes)if(q.side==side && q.k[j]>k[j] && Double.isFinite(local(q,e,j)))next=Math.min(next,q.k[j]);
                    if(next==d)next=Math.min(d,k[j]+Math.max(1,(d-k[j]+1)/2));
                    changed|=next!=k[j];k[j]=next;
                }
            }
        } else if(p.trace.valid() && p.trace.cost<e.parentCost-EPS) {
            for(int j=0;j<e.m;j++)if(local(p,e,j)<=EPS && k[j]>0){k[j]--;changed=true;}
        } else if(p.trace.valid() || "DEADLINE".equals(p.trace.status)) {
            int worst=-1;double delay=EPS;
            for(int j=0;j<e.m;j++){double v=local(p,e,j);if(v>delay){delay=v;worst=j;}}
            if(worst>=0) {
                for(Probe q:e.probes)if(q.side==side && local(q,e,worst)<delay-EPS) {
                    delay=local(q,e,worst);k[worst]=q.k[worst];
                }
                changed=k[worst]!=p.k[worst];
            } else count("suffix_timing_reject");
        }
        return changed?k:null;
    }
    private double local(Probe p,Event e,int j) {
        int l=p.map[e.a+j],r=p.map[e.a+j+2];
        double observed=p.trace.arrival[r]-p.trace.arrival[l];
        double base=e.base.arrival[e.a+j+2]-e.base.arrival[e.a+j];
        return observed-base;
    }

    /** Ordinary single-queue coordinate search: no cross-probe feedback or fusion. */
    private Result single(Event e) {
        if(e.singleK==null) {
            e.singleK=new int[e.m];for(int j=0;j<e.m;j++)e.singleK[j]=e.chains[0].get(j).size()-1;
            e.singleCost=e.parentCost;
        }
        while(e.singleAttempts<8 && host.allowed()) {
            int attempt=e.singleAttempts++,j=attempt%e.m;
            int[] k=e.singleK.clone();k[j]=Math.max(0,k[j]-Math.max(1,(k[j]+1)/2));
            Packed p=build(e,0,k);
            if(!e.seen.add(key(p.path))){count("duplicate_candidates");continue;}
            e.calls++;count("candidate_generated");TimeEvaluator.Trace t=host.evaluate(p.path);
            if(!host.completedAllowed() || "ABORTED".equals(t.status)){e.closed=true;return null;}
            if(t.valid())count("candidate_valid");
            if(t.valid() && t.cost<e.singleCost-EPS) {
                e.singleK=k;e.singleCost=t.cost;
                return result(e,"single_section",0,k,t,1);
            }
            return null;
        }
        e.closed=true;return null;
    }

    /** Bounded time-dependent A*: warm-started at the actual prefix arrival, not static edge weights.
     *  Fifteen-minute bins and four labels/cell make this an approximate, explicitly capped control. */
    private Result replan(Event e) {
        if(e.search==null)e.search=new Search(e);
        Search s=e.search;
        if(s.pending!=null) {
            List<GridPoint> route=s.pending;s.pending=null;e.calls++;count("replan_full_replays");
            if(s.goals>=4)e.closed=true;
            TimeEvaluator.Trace t=host.evaluate(route);
            return host.completedAllowed() && t.valid() && t.cost<e.parentCost-EPS?result(e,"local_replan",-1,null,t,0):null;
        }
        if(++s.slices>256){e.closed=true;count("replan_slice_cap");return null;}
        int attempts=0;
        while(host.allowed() && attempts<512 && s.expanded<256) {
            if(s.current==null) {
                if(s.open.isEmpty()){e.closed=true;break;}
                s.current=s.open.poll();
                if(s.current.time>s.best.getOrDefault(labelKey(s.current.point,s.current.time),Double.POSITIVE_INFINITY)+EPS){s.current=null;continue;}
                if(s.current.point.equals(e.parent.get(e.b))) {
                    List<GridPoint> segment=new ArrayList<>();for(Label x=s.current;x!=null;x=x.prev)segment.add(x.point);
                    Collections.reverse(segment);List<GridPoint> route=new ArrayList<>(e.parent.subList(0,e.a));
                    route.addAll(segment);route.addAll(e.parent.subList(e.b+1,e.parent.size()));
                    s.current=null;s.goals++;
                    if(e.seen.add(key(route))){s.pending=route;return null;}
                    if(s.goals>=4){e.closed=true;break;}continue;
                }
                s.edges=host.neighbors(s.current.point);s.edgeIndex=0;s.expanded++;count("replan_expansions");
            }
            if(s.edgeIndex>=s.edges.size()){s.current=null;continue;}
            GridPoint q=s.edges.get(s.edgeIndex++);attempts++;
            if(q.i<s.minI || q.i>s.maxI || q.j<s.minJ || q.j>s.maxJ)continue;
            boolean cycle=false;for(Label x=s.current;x!=null;x=x.prev)if(x.point.equals(q)){cycle=true;break;}
            if(cycle)continue;
            count("replan_edge_evaluations");
            double arrival=host.edgeArrival(s.current.point,q,s.current.time);
            if(!Double.isFinite(arrival) || arrival<=s.current.time)continue;
            String lk=labelKey(q,arrival);
            if(arrival>=s.best.getOrDefault(lk,Double.POSITIVE_INFINITY)-EPS)continue;
            String cell=q.i+","+q.j;
            if(!s.best.containsKey(lk) && s.cellLabels.getOrDefault(cell,0)>=4){count("replan_label_cap");continue;}
            if(!s.best.containsKey(lk))s.cellLabels.put(cell,s.cellLabels.getOrDefault(cell,0)+1);
            s.best.put(lk,arrival);s.open.add(new Label(q,arrival,s.current,++s.serial));
        }
        if(s.expanded>=256){e.closed=true;count("replan_expansion_cap");}
        return null;
    }
    private static String labelKey(GridPoint p,double time){return p.i+","+p.j+":"+(long)Math.floor(time*4);}
    private final class Search {
        final PriorityQueue<Label> open;
        final Map<String,Double> best=new HashMap<>();final Map<String,Integer> cellLabels=new HashMap<>();
        Label current;List<GridPoint> edges,pending;int edgeIndex,expanded,slices,goals,minI,maxI,minJ,maxJ;long serial;
        Search(Event e) {
            GridPoint goal=e.parent.get(e.b);
            open=new PriorityQueue<>(Comparator.<Label>comparingDouble(l->l.time+distance(l.point,goal)/Math.max(.1,host.nominalSpeed())).thenComparingLong(l->l.serial));
            minI=maxI=e.parent.get(e.a).i;minJ=maxJ=e.parent.get(e.a).j;
            for(int k=e.a;k<=e.b;k++){GridPoint p=e.parent.get(k);minI=Math.min(minI,p.i);maxI=Math.max(maxI,p.i);minJ=Math.min(minJ,p.j);maxJ=Math.max(maxJ,p.j);}
            minI-=8;maxI+=8;minJ-=8;maxJ+=8;
            Label first=new Label(e.parent.get(e.a),e.base.arrival[e.a],null,serial++);open.add(first);
            best.put(labelKey(first.point,first.time),first.time);cellLabels.put(first.point.i+","+first.point.j,1);
        }
    }
    private static final class Label {
        final GridPoint point;final double time;final Label prev;final long serial;
        Label(GridPoint p,double time,Label prev,long serial){this.point=p;this.time=time;this.prev=prev;this.serial=serial;}
    }
    private Result result(Event e,String stage,int side,int[] k,TimeEvaluator.Trace t,int sources) {
        Map<String,Object> w=new LinkedHashMap<>();w.put("event",e.id);w.put("mode",mode);w.put("source_owner",e.owner);
        w.put("stage",stage);w.put("side",side);w.put("from",e.a);w.put("to",e.b);w.put("retreat_steps",k);
        w.put("parent_cost",e.parentCost);w.put("candidate_cost",t.cost);w.put("parent",copy(e.parent));w.put("candidate",copy(t.path));
        w.put("trigger",e.trigger);w.put("fusion_sources",side<0?0:e.lastFusionSources[side]);
        w.put("committed_personal",false);w.put("committed_incumbent",false);
        List<Map<String,Object>> feedback=new ArrayList<>();
        for(Probe p:e.probes) {
            Map<String,Object> item=new LinkedHashMap<>();item.put("stage",p.stage);item.put("side",p.side);
            item.put("retreat_steps",p.k);item.put("status",p.trace.status);item.put("first_invalid_edge",p.trace.firstInvalidEdge);
            item.put("path",copy(p.trace.path));List<Double> arrivals=new ArrayList<>();
            for(double t0:p.trace.arrival)arrivals.add(Double.isFinite(t0)?t0:null);
            item.put("arrival_hours",arrivals);feedback.add(item);
        }
        w.put("feedback",feedback);
        if(witnesses.size()<256)witnesses.add(w);count("strict_parent_improvements");return new Result(t,w);
    }
    private double distance(GridPoint a,GridPoint b){return Math.hypot((a.i-b.i)*host.dy(),(a.j-b.j)*host.dx());}
    private double routeLength(List<GridPoint> p){double v=0;for(int k=1;k<p.size();k++)v+=distance(p.get(k-1),p.get(k));return v;}
    public static List<GridPoint> copy(List<GridPoint> p){List<GridPoint> r=new ArrayList<>();for(GridPoint q:p)r.add(new GridPoint(q.i,q.j));return r;}
    // Deduplicate geometric no-ops without changing the route passed to the evaluator.
    static String key(List<GridPoint> p){
        List<GridPoint> simple=new ArrayList<>();
        for(GridPoint q:p){
            if(!simple.isEmpty() && simple.get(simple.size()-1).equals(q))continue;
            while(simple.size()>=2){
                GridPoint a=simple.get(simple.size()-2),b=simple.get(simple.size()-1);
                long ux=b.i-a.i,uy=b.j-a.j,vx=q.i-b.i,vy=q.j-b.j;
                if(ux*vy-uy*vx!=0 || ux*vx+uy*vy<=0)break;
                simple.remove(simple.size()-1);
            }
            simple.add(q);
        }
        StringBuilder b=new StringBuilder();for(GridPoint q:simple)b.append(q.i).append(',').append(q.j).append(';');return b.toString();
    }
    private static String rawKey(List<GridPoint> p){StringBuilder b=new StringBuilder();for(GridPoint q:p)b.append(q.i).append(',').append(q.j).append(';');return b.toString();}
    static List<GridPoint> line(GridPoint a,GridPoint b) {
        List<GridPoint> out=new ArrayList<>();int x=a.j,y=a.i,dx=Math.abs(b.j-x),dy=-Math.abs(b.i-y),sx=Integer.compare(b.j,x),sy=Integer.compare(b.i,y),err=dx+dy;
        for(;;){out.add(new GridPoint(y,x));if(x==b.j && y==b.i)break;int twice=2*err;if(twice>=dy){err+=dy;x+=sx;}if(twice<=dx){err+=dx;y+=sy;}}
        return out;
    }
    static final class Packed {final List<GridPoint> path;final int[] map;Packed(List<GridPoint> p,int[] m){path=p;map=m;}}
    /** Representation subdivision only, shared by the three arms; no obstacle repair. */
    static Packed pack(List<GridPoint> raw) {
        List<GridPoint> out=new ArrayList<>();int[] map=new int[raw.size()];out.add(raw.get(0));
        for(int i=1;i<raw.size();i++) {
            GridPoint a=out.get(out.size()-1),b=raw.get(i);
            if(!a.equals(b)) {
                if(Math.hypot(a.i-b.i,a.j-b.j)>12) {
                    List<GridPoint> cells=line(a,b);for(int j=8;j<cells.size()-1;j+=8)out.add(cells.get(j));
                }
                out.add(b);
            }
            map[i]=out.size()-1;
        }return new Packed(out,map);
    }
    private static Packed build(Event e,int side,int[] k) {
        List<GridPoint> p=copy(e.parent);for(int j=0;j<e.m;j++)p.set(e.a+j+1,e.chains[side].get(j).get(k[j]));return pack(p);
    }
    private static final class Probe {
        final int side,stage;final int[] k,map;final TimeEvaluator.Trace trace;int sources;
        Probe(int side,int[] k,TimeEvaluator.Trace t,int[] map,int stage){this.side=side;this.k=k.clone();trace=t;this.map=map;this.stage=stage;}
    }
    private static final class Event {
        long id;final int owner,a,b,m;final List<GridPoint> parent;final double parentCost;
        final List<List<GridPoint>>[] chains;
        List<GridPoint> shortcut;TimeEvaluator.Trace base;String trigger="";int stage,visits,calls,singleAttempts;
        int[] singleK;double singleCost;boolean closed;final Set<String> seen=new HashSet<>();
        final List<Probe> probes=new ArrayList<>();final Probe[] fused=new Probe[2];final int[] lastFusionSources=new int[2];
        Search search;
        @SuppressWarnings("unchecked") Event(int owner,List<GridPoint> p,double cost,int a,int b,int m){this.owner=owner;parent=copy(p);parentCost=cost;this.a=a;this.b=b;this.m=m;chains=new List[]{new ArrayList<>(),new ArrayList<>()};}
    }
}
