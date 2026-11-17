package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import Navigation.model.GridRouteGraph;
import java.util.*;

/** Optional TIME proposal operator. All arms share the connector, evidence and acceptance. */
public final class RequestMigration {
    public enum Mode { PLAIN3, B3, BH1, SHUFFLED }
    public interface Host {
        boolean allowed();
        boolean completedAllowed();
        long remainingWork();
        long work();
        TimeEvaluator.Trace evaluate(List<GridPoint> path);
        double edgeArrival(GridPoint a, GridPoint b, double elapsed);
        boolean edgeLegal(GridPoint a, GridPoint b);
        List<GridPoint> neighbors(GridPoint p);
        double distance(GridPoint a, GridPoint b);
        double dx();
        double dy();
        double radius();
        double nominalSpeed();
        double distanceBudget();
        default double maxEdgeSpanCells(){return GridRouteGraph.DEFAULT_MAX_EDGE_SPAN_CELLS;}
        int weatherDay(double elapsed);
        void count(String name);
    }
    public static final class Result {
        public final List<GridPoint> path;
        public final double cost;
        public final Map<String,Object> witness;
        Result(TimeEvaluator.Trace trace, Map<String,Object> witness) {
            path=trace.path; cost=trace.cost; this.witness=witness;
        }
    }
    private static final double EPS=1e-9;
    private final Host host;
    private final Mode mode;
    private final java.util.concurrent.atomic.AtomicLong ordinary=new java.util.concurrent.atomic.AtomicLong();
    private long opportunities;
    private int windowCursor;
    private final LinkedHashMap<String,List<GridPoint>> observed=new LinkedHashMap<>();
    private final LinkedHashMap<String,TimeEvaluator.Trace> traces=new LinkedHashMap<>();
    private final LinkedHashSet<String> handled=new LinkedHashSet<>();
    private final List<Map<String,Object>> witnesses=new ArrayList<>();

    public RequestMigration(Mode mode, Host host) { this.mode=mode; this.host=host; }
    public void ordinaryEvaluation() { ordinary.incrementAndGet(); }
    public List<Map<String,Object>> witnesses() { return Collections.unmodifiableList(witnesses); }
    private void count(String name) { host.count("rm_"+name); }
    private void markHandled(String key){handled.add(key);while(handled.size()>2048)handled.remove(handled.iterator().next());}
    public static String key(List<GridPoint> p) {
        StringBuilder out=new StringBuilder();
        for(GridPoint q:p) out.append(q.i).append(',').append(q.j).append(';');
        return out.toString();
    }
    public synchronized void observe(List<GridPoint> path, double cost) {
        if(path==null || path.size()<3 || !Double.isFinite(cost)) return;
        String key=key(path);
        if(observed.containsKey(key)) return;
        observed.put(key,Collections.unmodifiableList(new ArrayList<>(path)));
        // Preserve the input seed; a rolling recent-only pool would silently lose this history.
        while(observed.size()>10){Iterator<String> it=observed.keySet().iterator();it.next();observed.remove(it.next());}
    }
    private synchronized List<List<GridPoint>> observedPaths() { return new ArrayList<>(observed.values()); }
    private void remember(TimeEvaluator.Trace t) {
        if(t!=null && t.valid()) {
            traces.put(key(t.path),t);
            while(traces.size()>8) traces.remove(traces.keySet().iterator().next());
        }
    }
    private static long edgeWork(GridPoint a,GridPoint b) {
        return Math.max(1L,(long)Math.ceil(Math.max(Math.abs(a.i-b.i),Math.abs(a.j-b.j))/FsaContext.SAFETY_SAMPLE_STEP_CELLS));
    }
    private static long routeWork(List<GridPoint> p) {
        long n=0; for(int k=1;k<p.size();k++) n+=edgeWork(p.get(k-1),p.get(k)); return n;
    }
    private boolean affordable(long n) { return host.allowed() && n<=host.remainingWork(); }
    private static final class Transaction {
        int calls, finalCalls, donorReplays, expanded, side, h1Queries, h1Different, h1Changed, shuffledQueries, maxSlots, slotEdges;
        final Set<String> evaluated=new HashSet<>();
        final List<Map<String,Object>> evaluations=new ArrayList<>();
    }
    private TimeEvaluator.Trace evaluate(List<GridPoint> p, String role, Transaction tx) {
        String k=key(p);
        if(tx.calls>=6 || tx.evaluated.contains(k) || !affordable(routeWork(p))) return null;
        tx.evaluated.add(k);tx.calls++;
        long before=host.work();TimeEvaluator.Trace t=host.evaluate(p);
        Map<String,Object> row=new LinkedHashMap<>();row.put("role",role);row.put("path",p);
        row.put("status",t.status);row.put("cost",t.valid()?t.cost:null);row.put("work",host.work()-before);
        tx.evaluations.add(row);remember(t);
        return t.valid() && host.completedAllowed()?t:null;
    }
    /** Shared rotation: no success-score sorting or preference for route terminal cells. */
    private List<int[]> windows(List<GridPoint> p,double radius) {
        double[] arc=new double[p.size()];
        for(int k=1;k<p.size();k++)arc[k]=arc[k-1]+host.distance(p.get(k-1),p.get(k));
        List<int[]> out=new ArrayList<>();Set<Integer> starts=new HashSet<>();
        for(int k=0;k<8;k++) {
            int a=0;while(a<p.size()-3 && arc[a]<arc[arc.length-1]*k/8.0)a++;
            if(!starts.add(a))continue;
            int b=a+2;while(b<p.size()-1 && arc[b]-arc[a]<2*radius)b++;
            out.add(new int[]{a,b});
        }
        return out;
    }
    private List<GridPoint> pack(List<GridPoint> parent,int a,int b,List<GridPoint> middle) {
        List<GridPoint> out=new ArrayList<>(parent.subList(0,a));out.addAll(middle);out.addAll(parent.subList(b+1,parent.size()));
        return new HashSet<>(out).size()==out.size()?out:null;
    }
    private List<GridPoint> commonProposal(List<GridPoint> p,int a,int b) {
        GridPoint from=p.get(a),end=p.get(b);
        if(host.edgeLegal(from,end))return pack(p,a,b,Arrays.asList(from,end));
        GridPoint best=null;double length=Double.POSITIVE_INFINITY;
        Set<GridPoint> outside=new HashSet<>(p.subList(0,a));outside.addAll(p.subList(b+1,p.size()));
        for(GridPoint q:host.neighbors(from)) {
            if(!host.allowed())break;
            if(q.equals(end)||outside.contains(q)||!host.edgeLegal(q,end))continue;
            double candidate=host.distance(from,q)+host.distance(q,end);
            if(candidate<length-EPS){length=candidate;best=q;}
        }
        return best==null?null:pack(p,a,b,Arrays.asList(from,best,end));
    }
    private static final class Relation {
        final GridPoint from,to;final double time,hx,hy;final int source,layer;
        Relation(GridPoint from,GridPoint to,double time,double hx,double hy,int source,int layer) {
            this.from=from;this.to=to;this.time=time;this.hx=hx;this.hy=hy;this.source=source;this.layer=layer;
        }
    }
    /** Ordered original-node matching, not interpolated arrival times or independently mixed layers. */
    private List<Relation> align(TimeEvaluator.Trace donor,List<GridPoint> refs,int source,double radius) {
        int layers=refs.size(),n=donor.path.size();List<Relation> out=new ArrayList<>();
        if(n<layers)return out;
        double[][] dp=new double[layers][n];int[][] prev=new int[layers][n];
        for(double[] row:dp)Arrays.fill(row,Double.POSITIVE_INFINITY);
        for(int[] row:prev)Arrays.fill(row,-1);
        for(int l=0;l<layers;l++)for(int j=l;j<n;j++) {
            if(!host.allowed())return Collections.emptyList();
            double d=host.distance(refs.get(l),donor.path.get(j));if(d>radius+EPS)continue;
            if(l==0)dp[l][j]=d;
            else for(int k=l-1;k<j;k++)if(dp[l-1][k]+d<dp[l][j]){dp[l][j]=dp[l-1][k]+d;prev[l][j]=k;}
        }
        int last=-1;double cost=Double.POSITIVE_INFINITY;
        for(int j=layers-1;j<n;j++)if(dp[layers-1][j]<cost){cost=dp[layers-1][j];last=j;}
        if(last<0)return out;
        int[] indices=new int[layers];indices[layers-1]=last;
        for(int l=layers-1;l>0;l--)indices[l-1]=prev[l][indices[l]];
        for(int l=0;l<layers-1;l++) {
            int j=indices[l];GridPoint from=donor.path.get(j),to=donor.path.get(indices[l+1]);
            GridPoint old=donor.path.get(j==0?Math.min(1,n-1):j-1);
            double hx=(j==0?old.j-from.j:from.j-old.j)*host.dx(),hy=(j==0?old.i-from.i:from.i-old.i)*host.dy();
            out.add(new Relation(from,to,donor.arrival[j],hx,hy,source,l));
        }
        return out;
    }
    public static String shapeKey(List<GridPoint> path) {
        List<GridPoint> p=new ArrayList<>();
        for(GridPoint q:path){
            while(p.size()>=2){GridPoint a=p.get(p.size()-2),b=p.get(p.size()-1);
                long x=b.j-a.j,y=b.i-a.i,u=q.j-b.j,v=q.i-b.i;
                if(x*v-y*u!=0||x*u+y*v<0)break;p.remove(p.size()-1);}
            p.add(q);
        }
        return key(p);
    }
    private boolean canAlign(List<GridPoint> donor,List<GridPoint> refs) {
        int j=0;for(GridPoint ref:refs){while(j<donor.size()&&host.distance(ref,donor.get(j))>host.radius()+EPS)j++;if(j>=donor.size())return false;j++;}return true;
    }
    private List<TimeEvaluator.Trace> supply(TimeEvaluator.Trace parent,List<GridPoint> refs,Transaction tx) {
        List<TimeEvaluator.Trace> out=new ArrayList<>();out.add(parent);
        List<List<GridPoint>> paths=observedPaths();String parentShape=shapeKey(parent.path);
        paths.removeIf(p->shapeKey(p).equals(parentShape)||!canAlign(p,refs));
        paths.sort(Comparator.comparingDouble((List<GridPoint> p)->-geometryDistance(p,parent.path)).thenComparing(RequestMigration::key));
        for(List<GridPoint> p:paths) {
            if(!host.allowed()||out.size()>=3)break;
            TimeEvaluator.Trace t=traces.get(key(p));
            if(t==null && tx.donorReplays<1) {int calls=tx.calls;t=evaluate(p,"common_donor_replay",tx);if(tx.calls>calls)tx.donorReplays++;}
            if(t!=null)out.add(t);
        }
        return out;
    }
    private double geometryDistance(List<GridPoint> p,List<GridPoint> q) {
        double sum=0;for(GridPoint v:p){double best=Double.POSITIVE_INFINITY;
            for(int k=1;k<q.size();k++){
                GridPoint a=q.get(k-1),b=q.get(k);double x=(b.j-a.j)*host.dx(),y=(b.i-a.i)*host.dy();
                double px=(v.j-a.j)*host.dx(),py=(v.i-a.i)*host.dy();double at=Math.max(0,Math.min(1,(px*x+py*y)/Math.max(1e-15,x*x+y*y)));
                best=Math.min(best,Math.hypot(px-at*x,py-at*y));
            }
            sum+=best;}
        return sum/p.size();
    }
    private static final class Node {
        final GridPoint p;final double time,length,priority;final int layer;final long sampledWork;final Node parent;
        boolean active=true;
        Node(GridPoint p,double time,double length,double priority,int layer,long sampledWork,Node parent){this.p=p;this.time=time;this.length=length;this.priority=priority;this.layer=layer;this.sampledWork=sampledWork;this.parent=parent;}
    }
    private double angleScore(Node x,GridPoint q,double targetI,double targetJ) {
        double ax=(q.j-x.p.j)*host.dx(),ay=(q.i-x.p.i)*host.dy();
        double bx=(targetJ-x.p.j)*host.dx(),by=(targetI-x.p.i)*host.dy();
        return -(ax*bx+ay*by)/Math.max(1e-15,Math.hypot(ax,ay)*Math.hypot(bx,by));
    }
    private GridPoint representative(List<GridPoint> pool,Node x,double i,double j,Set<GridPoint> used) {
        GridPoint best=null;double score=Double.POSITIVE_INFINITY;
        for(GridPoint q:pool)if(!used.contains(q)) {
            double s=angleScore(x,q,i,j)+1e-6*Math.hypot((q.i-i)*host.dy(),(q.j-j)*host.dx());
            if(s<score){score=s;best=q;}
        }
        return best;
    }
    private List<Relation> activeRelations(Node x,List<Relation> relations) {
        List<Relation> out=new ArrayList<>();
        for(Relation r:relations)if(r.layer==x.layer && host.weatherDay(x.time)==host.weatherDay(r.time))out.add(r);
        return out;
    }
    private double guide(Node x,GridPoint q,List<Relation> active,boolean shuffled,double entryHx,double entryHy) {
        double sum=0,weights=0,radius=Math.max(1e-9,host.radius());
        for(int k=0;k<active.size();k++) {
            Relation r=active.get(k);GridPoint next=shuffled && active.size()>1?active.get((k+1)%active.size()).to:r.to;
            double weight=Math.exp(-Math.pow(host.distance(x.p,r.from)/radius,2)/2);
            double hx=x.parent==null?entryHx:(x.p.j-x.parent.p.j)*host.dx(),hy=x.parent==null?entryHy:(x.p.i-x.parent.p.i)*host.dy();
            if(Math.hypot(hx,hy)>1e-12) {
                weight*=Math.max(0,(hx*r.hx+hy*r.hy)/Math.max(1e-15,Math.hypot(hx,hy)*Math.hypot(r.hx,r.hy)));
            }
            weights+=weight;sum+=weight*Math.exp(-Math.pow(host.distance(q,next)/radius,2)/2);
        }
        return weights>0?sum/weights:Double.NEGATIVE_INFINITY;
    }
    private static boolean inChain(Node x,GridPoint q) {for(Node n=x;n!=null;n=n.parent)if(n.p.equals(q))return true;return false;}
    private static List<GridPoint> middle(Node x) {List<GridPoint> out=new ArrayList<>();for(Node n=x;n!=null;n=n.parent)out.add(n.p);Collections.reverse(out);return out;}
    private int layer(GridPoint p,int previous,List<GridPoint> refs) {
        int best=previous;double d=Double.POSITIVE_INFINITY;
        for(int l=previous;l<refs.size()-1;l++){double candidate=host.distance(p,refs.get(l));if(candidate<d){d=candidate;best=l;}}
        return best;
    }
    private TimeEvaluator.Trace connect(TimeEvaluator.Trace base,int a,int b,int focus,List<GridPoint> refs,List<Relation> relations,Transaction tx) {
        List<GridPoint> p=base.path;GridPoint from=p.get(a),goal=p.get(b);
        double entryHx=a==0?0:(from.j-p.get(a-1).j)*host.dx(),entryHy=a==0?0:(from.i-p.get(a-1).i)*host.dy();
        double radius=host.radius(),outsideLength=0;long outsideWork=0;
        for(int k=1;k<p.size();k++)if(k<=a||k>b){outsideLength+=host.distance(p.get(k-1),p.get(k));outsideWork+=edgeWork(p.get(k-1),p.get(k));}
        int i0=Integer.MAX_VALUE,i1=-1,j0=Integer.MAX_VALUE,j1=-1;
        for(int k=a;k<=b;k++){i0=Math.min(i0,p.get(k).i);i1=Math.max(i1,p.get(k).i);j0=Math.min(j0,p.get(k).j);j1=Math.max(j1,p.get(k).j);}
        int ri=(int)Math.ceil(radius/host.dy()),rj=(int)Math.ceil(radius/host.dx());i0-=ri;i1+=ri;j0-=rj;j1+=rj;
        Set<GridPoint> outside=new HashSet<>(p.subList(0,a));outside.addAll(p.subList(b+1,p.size()));
        GridPoint fa=p.get(focus),fb=p.get(Math.min(focus+1,b));
        double vx=(fb.j-fa.j)*host.dx(),vy=(fb.i-fa.i)*host.dy(),norm=Math.max(1e-12,Math.hypot(vx,vy));
        double centerI=(fa.i+fb.i)/2.0,centerJ=(fa.j+fb.j)/2.0;
        double plusI=centerI+vx/norm*radius/2/host.dy(),plusJ=centerJ-vy/norm*radius/2/host.dx();
        double minusI=centerI-vx/norm*radius/2/host.dy(),minusJ=centerJ+vy/norm*radius/2/host.dx();
        PriorityQueue<Node> queue=new PriorityQueue<>(Comparator.comparingDouble((Node n)->n.priority).thenComparingDouble(n->n.time).thenComparingInt(n->n.p.i).thenComparingInt(n->n.p.j));
        Map<GridPoint,List<Node>> labels=new HashMap<>();Node start=new Node(from,base.arrival[a],0,0,0,0,null);queue.add(start);labels.put(from,new ArrayList<>(Collections.singleton(start)));
        TimeEvaluator.Trace best=null;
        while(!queue.isEmpty()&&tx.expanded<256&&host.allowed()&&tx.calls<6&&tx.finalCalls<3) {
            Node x=queue.poll();if(!x.active)continue;
            if(x.p.equals(goal)) {
                List<GridPoint> candidate=pack(p,a,b,middle(x));
                if(candidate!=null && !tx.evaluated.contains(key(candidate)) && !key(candidate).equals(key(p))) {
                    int oldCalls=tx.calls;TimeEvaluator.Trace t=evaluate(candidate,"connector_final",tx);
                    if(tx.calls>oldCalls)tx.finalCalls++;
                    if(t!=null && t.cost<base.cost-EPS && (best==null||t.cost<best.cost))best=t;
                }
                continue;
            }
            tx.expanded++;
            List<GridPoint> pool=new ArrayList<>();
            for(GridPoint q:host.neighbors(x.p)) {
                if(!host.allowed())break;
                if(q.i<i0||q.i>i1||q.j<j0||q.j>j1||outside.contains(q)||inChain(x,q))continue;
                if(outsideLength+x.length+host.distance(x.p,q)+host.distance(q,goal)>host.distanceBudget()+EPS)continue;
                pool.add(q);
            }
            if(pool.isEmpty())continue;
            Set<GridPoint> selected=new LinkedHashSet<>();GridPoint q=representative(pool,x,goal.i,goal.j,selected);if(q!=null)selected.add(q);
            if(mode==Mode.PLAIN3) {
                // Same three slots and state information; ordinary goal-oriented breadth.
                while(selected.size()<3 && (q=representative(pool,x,goal.i,goal.j,selected))!=null)selected.add(q);
            } else {
                boolean right=(tx.side++&1)==0;
                q=representative(pool,x,right?plusI:minusI,right?plusJ:minusJ,selected);if(q!=null)selected.add(q);
                GridPoint h1=null;List<Relation> active=activeRelations(x,relations);
                if((mode==Mode.BH1||mode==Mode.SHUFFLED)&&!active.isEmpty()) {
                    tx.h1Queries++;if(mode==Mode.SHUFFLED&&active.size()>1)tx.shuffledQueries++;
                    double score=Double.NEGATIVE_INFINITY;
                    for(GridPoint v:pool)if(!selected.contains(v)){double s=guide(x,v,active,mode==Mode.SHUFFLED,entryHx,entryHy);if(s>score){score=s;h1=v;}}
                }
                if(h1!=null){GridPoint fallback=representative(pool,x,right?minusI:plusI,right?minusJ:plusJ,selected);if(!h1.equals(fallback))tx.h1Changed++;selected.add(h1);tx.h1Different++;}
                else {q=representative(pool,x,right?minusI:plusI,right?minusJ:plusJ,selected);if(q!=null)selected.add(q);}
            }
            tx.maxSlots=Math.max(tx.maxSlots,selected.size());tx.slotEdges+=selected.size();
            for(GridPoint v:selected) {
                long needed=edgeWork(x.p,v),finalReserve=outsideWork+x.sampledWork+needed+edgeWork(v,goal);
                if(!affordable(needed+finalReserve))continue;
                double at=host.edgeArrival(x.p,v,x.time);if(!Double.isFinite(at)||!host.completedAllowed())continue;
                double length=x.length+host.distance(x.p,v),priority=at+host.distance(v,goal)/Math.max(.1,host.nominalSpeed());
                Node next=new Node(v,at,length,priority,layer(v,x.layer,refs),x.sampledWork+needed,x);
                List<Node> cell=labels.computeIfAbsent(v,k->new ArrayList<>());
                if(cell.size()>=4){Node worst=Collections.max(cell,Comparator.comparingDouble(n->n.priority));if(worst.priority<=priority)continue;worst.active=false;cell.remove(worst);}
                cell.add(next);queue.add(next);
            }
        }
        return best;
    }
    public Result poll(List<GridPoint> parent,double parentCost) {
        count("polls");if(!host.allowed()||ordinary.get()<3*(opportunities+1))return null;
        if(parent==null||parent.size()<3||!Double.isFinite(parentCost))return null;
        long reserve=3*routeWork(parent)+3*(long)Math.ceil(host.maxEdgeSpanCells()/FsaContext.SAFETY_SAMPLE_STEP_CELLS);
        if(host.remainingWork()<reserve){count("unfunded_transaction");return null;}
        opportunities++;count("opportunities");
        List<int[]> windows=windows(parent,host.radius());if(windows.isEmpty())return null;
        String version=key(parent)+"@"+Double.toHexString(host.radius());int[] window=null;String requestKey=null;
        for(int k=0;k<windows.size();k++){int[] w=windows.get((windowCursor+k)%windows.size());String key=version+":"+w[0]+":"+w[1];if(!handled.contains(key)){window=w;requestKey=key;windowCursor=(windowCursor+k+1)%windows.size();break;}}
        if(window==null){count("no_fresh_window");return null;}
        Transaction tx=new Transaction();TimeEvaluator.Trace base=traces.get(key(parent));
        if(base==null)base=evaluate(Collections.unmodifiableList(new ArrayList<>(parent)),"parent_replay",tx);
        if(base==null){count("unfunded_or_invalid_parent");return null;}
        if(Math.abs(base.cost-parentCost)>1e-7*Math.max(1,Math.abs(parentCost)))throw new IllegalStateException("request parent path/cost mismatch");
        int a=window[0],b=window[1];Map<String,Object> w=new LinkedHashMap<>();
        w.put("mode",mode.name());w.put("parent",base.path);w.put("parent_cost",base.cost);w.put("parent_arrival",base.arrival);w.put("a",a);w.put("b",b);w.put("entry",base.arrival[a]);w.put("request_key",requestKey);w.put("evaluations",tx.evaluations);
        if(witnesses.size()<256)witnesses.add(w);else count("witness_overflow");
        try {
        List<GridPoint> proposal=commonProposal(parent,a,b);TimeEvaluator.Trace common=null;
        if(proposal!=null) {
            if(key(proposal).equals(key(parent)))common=base;
            else {int calls=tx.calls;common=evaluate(proposal,"common_proposal",tx);if(common==null){w.put("origin",tx.calls==calls?"unfunded_common":"invalid_common");if(tx.calls==calls)witnesses.remove(w);else markHandled(requestKey);count("incomplete_common");return null;}}
        }
        if(!host.completedAllowed()){witnesses.remove(w);return null;}
        markHandled(requestKey);
        if(common!=null&&common.cost<base.cost-EPS){w.put("origin","common_proposal");w.put("result_cost",common.cost);w.put("result_path",common.path);w.put("full_calls",tx.calls);count("common_improved");return new Result(common,w);}
        if(!host.allowed())return null;
        int focus=(a+b-1)/2;
        if(common!=null){
            double slow=Double.NEGATIVE_INFINITY;int slowLeg=a,legs=common.path.size()-parent.size()+b-a;
            for(int k=a;k<a+legs;k++){double ratio=common.duration[k]/Math.max(1e-9,host.distance(common.path.get(k),common.path.get(k+1)));if(ratio>slow){slow=ratio;slowLeg=k;}}
            GridPoint u=common.path.get(slowLeg),v=common.path.get(slowLeg+1);double best=Double.POSITIVE_INFINITY;
            for(int k=a;k<b;k++){double d=Math.hypot((parent.get(k).i+parent.get(k+1).i-u.i-v.i)*host.dy(),(parent.get(k).j+parent.get(k+1).j-u.j-v.j)*host.dx());if(d<best){best=d;focus=k;}}
        }
        w.put("reason",common==null?"EMPTY_LOCAL":"VALID_NO_GAIN");w.put("focus_edge",focus);count("requests");
        List<GridPoint> refs=new ArrayList<>();for(int l=0;l<Math.min(4,b-a+1);l++)refs.add(parent.get(a+(b-a)*l/(Math.min(4,b-a+1)-1)));
        List<TimeEvaluator.Trace> donors=supply(base,refs,tx);List<Relation> relations=new ArrayList<>();
        if(donors.size()>=2)for(int k=0;k<donors.size();k++)relations.addAll(align(donors.get(k),refs,k,host.radius()));
        Set<Integer> aligned=new HashSet<>();for(Relation r:relations)aligned.add(r.source);
        w.put("traced_sources",donors.size());w.put("aligned_sources",aligned.size());
        if(aligned.size()<2)relations.clear();
        w.put("relations",relations.size());
        TimeEvaluator.Trace best=connect(base,a,b,focus,refs,relations,tx);
        w.put("full_calls",tx.calls);w.put("final_calls",tx.finalCalls);w.put("donor_replays",tx.donorReplays);w.put("expanded",tx.expanded);w.put("h1_queries",tx.h1Queries);w.put("h1_distinct_slots",tx.h1Different);w.put("shuffled_queries",tx.shuffledQueries);
        for(int k=0;k<tx.expanded;k++)count("connector_expansions");for(int k=0;k<tx.h1Different;k++)count("h1_distinct_slots");
        if(best!=null){w.put("origin","connector");w.put("result_cost",best.cost);w.put("result_path",best.path);count("connector_improved");return new Result(best,w);}
        count("no_improvement");return null;
        }finally{
            w.put("full_calls",tx.calls);w.put("final_calls",tx.finalCalls);w.put("donor_replays",tx.donorReplays);w.put("expanded",tx.expanded);
            w.put("h1_queries",tx.h1Queries);w.put("h1_distinct_slots",tx.h1Different);w.put("shuffled_queries",tx.shuffledQueries);
            w.put("h1_changed_from_b",tx.h1Changed);
            w.put("max_slots",tx.maxSlots);w.put("slot_edges",tx.slotEdges);
        }
    }
}
