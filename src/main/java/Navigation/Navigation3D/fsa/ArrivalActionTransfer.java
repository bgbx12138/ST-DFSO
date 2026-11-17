package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import java.util.*;

/** Opt-in, TIME-only observational edit transfer. No objective or feasibility relaxation. */
public final class ArrivalActionTransfer {
    public interface Host extends ConditionalCooperation.Host {
        /** Descriptor at existing vertices and their replayed arrival times; not interpolated physics. */
        double[] environment(ConditionalCooperation.Trace trace, int left, int right);
    }
    private static final double EPS = 1e-9;
    private final Host host;
    private final String mode;
    private final double dx, dy;
    private final boolean residual, conditioned, selfOnly;
    private long serial;
    private final Map<Integer,Deque<Edit>> events = new TreeMap<>();
    private final Map<Integer,String> versions = new HashMap<>();
    private final Map<Integer,Set<String>> tried = new HashMap<>();
    private final LinkedHashMap<String,ConditionalCooperation.Trace> traces = new LinkedHashMap<>();
    private final List<Map<String,Object>> witnesses = new ArrayList<>();

    public ArrivalActionTransfer(String mode, Host host, double dx, double dy) {
        if (!Arrays.asList("A0","A1","G0","G1","NF_A1").contains(mode))
            throw new IllegalArgumentException("unknown action-transfer mode: " + mode);
        if (!(dx>0 && dy>0)) throw new IllegalArgumentException("positive physical grid scales required");
        this.mode=mode; this.host=host; this.dx=dx; this.dy=dy;
        residual=mode.startsWith("A") || mode.equals("NF_A1");
        conditioned=mode.endsWith("1"); selfOnly=mode.equals("NF_A1");
    }
    private static final class Edit {
        long id; int owner,left,right,newRight;
        List<GridPoint> before,after; double beforeCost,afterCost;
    }
    private static final class Match {
        Edit edit; double score;
        Match(Edit e,double s){edit=e;score=s;}
    }
    public List<Map<String,Object>> witnesses(){return new ArrayList<>(witnesses);}

    /** Only called with an actual evaluated parent/offspring pair, never two unrelated best routes. */
    public void observe(int owner,List<GridPoint> before,double beforeCost,List<GridPoint> after,double afterCost) {
        if (before==null || after==null || before.size()<3 || after.size()<2
                || !Double.isFinite(beforeCost) || !Double.isFinite(afterCost) || !(afterCost<beforeCost-EPS)) return;
        if (!before.get(0).equals(after.get(0)) || !before.get(before.size()-1).equals(after.get(after.size()-1))) return;
        int prefix=0;
        while(prefix<Math.min(before.size(),after.size()) && before.get(prefix).equals(after.get(prefix)))prefix++;
        if(prefix==Math.min(before.size(),after.size()))return;
        int suffix=0;
        while(suffix<Math.min(before.size(),after.size())-prefix
                && before.get(before.size()-1-suffix).equals(after.get(after.size()-1-suffix)))suffix++;
        int l=prefix-1,r=before.size()-suffix,nr=after.size()-suffix;
        if(l<0 || r>=before.size() || nr>=after.size() || r-l<2 || nr<=l)return;
        if(chord(before,l,r)<EPS)return;
        String fingerprint=key(before)+">"+key(after);
        Deque<Edit> q=events.computeIfAbsent(owner,k->new ArrayDeque<>());
        for(Edit e:q)if((key(e.before)+">"+key(e.after)).equals(fingerprint))return;
        Edit e=new Edit();e.id=++serial;e.owner=owner;e.left=l;e.right=r;e.newRight=nr;
        e.before=copy(before);e.after=copy(after);e.beforeCost=beforeCost;e.afterCost=afterCost;
        q.addLast(e);while(q.size()>2){
            String expired=q.removeFirst().id+":";
            for(Set<String> attempts:tried.values())attempts.removeIf(k->k.startsWith(expired));
        }host.event("events_stored");
    }

    /** One selected window, at most four shortlisted events, one generated route. */
    public ConditionalCooperation.Trace step(int owner,List<GridPoint> recipient,double cost,long opportunity) {
        host.event("opportunities");
        if(recipient==null || recipient.size()<3 || !host.allowed())return null;
        String version=key(recipient);
        if(!version.equals(versions.put(owner,version)))tried.put(owner,new HashSet<>());
        List<int[]> windows=ConditionalCooperation.windows(recipient,host);
        windows.removeIf(w->w[1]-w[0]<2 || chord(recipient,w[0],w[1])<EPS);
        if(windows.isEmpty())return null;
        int[] window=windows.get((int)Math.floorMod(opportunity,windows.size()));int l=window[0],r=window[1];
        List<Match> shortlist=new ArrayList<>();
        for(Deque<Edit> queue:events.values())for(Edit e:queue){
            if(selfOnly ? e.owner!=owner : e.owner==owner)continue;
            double scale=chord(recipient,l,r)/chord(e.before,e.left,e.right);
            if(scale<0.5 || scale>2 || tried.get(owner).contains(e.id+":"+l+":"+r))continue;
            shortlist.add(new Match(e,geometryDistance(e.before,e.left,e.right,recipient,l,r)));
        }
        shortlist.sort(Comparator.comparingDouble((Match m)->m.score).thenComparingLong(m->m.edit.id));
        if(shortlist.size()>4)shortlist.subList(4,shortlist.size()).clear();
        if(shortlist.isEmpty()){host.event("no_supply");return null;}
        ConditionalCooperation.Trace rt=trace(recipient);
        if(rt==null || Math.abs(rt.cost-cost)>1e-7*Math.max(1,Math.abs(cost)))return null;
        double[] rc=host.environment(rt,l,r);
        if(!finite(rc)){host.event("context_unavailable");return null;}
        Match best=null;double bestScore=Double.POSITIVE_INFINITY;
        for(Match m:shortlist){
            if(!host.allowed())return null;
            Edit e=m.edit;ConditionalCooperation.Trace dt=trace(e.before);
            if(dt==null || Math.abs(dt.cost-e.beforeCost)>1e-7*Math.max(1,Math.abs(e.beforeCost)))continue;
            double[] dc=host.environment(dt,e.left,e.right);
            if(!finite(dc) || dc.length!=rc.length)continue;
            double score=m.score+(conditioned?meanSquare(dc,rc):0);
            if(score<bestScore){best=m;bestScore=score;}
        }
        if(best==null)return null;
        Edit e=best.edit;tried.get(owner).add(e.id+":"+l+":"+r);
        List<GridPoint> proposed=transport(e.before,e.left,e.right,e.after,e.left,e.newRight,recipient,l,r,residual,dx,dy);
        if(key(proposed).equals(version)){host.event("rounded_noop");return null;}
        host.event("generated");long workBefore=host.work();double incumbent=host.incumbentCost();
        ConditionalCooperation.Trace result=host.replay(proposed);
        if(result==null || !Double.isFinite(result.cost) || !host.allowed()){host.event("invalid_or_budget");return null;}
        boolean accepted=result.cost<cost-EPS;
        host.event(accepted?"strict_personal_candidates":"non_improving");
        if(witnesses.size()<256){
            Map<String,Object> w=new LinkedHashMap<>();w.put("mode",mode);w.put("owner",owner);w.put("donor",e.owner);
            w.put("event_id",e.id);w.put("window",window);w.put("before_cost",cost);w.put("after_cost",result.cost);
            w.put("strict_personal_candidate",accepted);w.put("incumbent_snapshot",incumbent);w.put("candidate_subsegments",host.work()-workBefore);
            w.put("recipient",copy(recipient));w.put("candidate",copy(proposed));witnesses.add(w);
        }
        // Do not turn transfer descendants into new observational evidence in this increment.
        return accepted?result:null;
    }
    private ConditionalCooperation.Trace trace(List<GridPoint> p){
        String k=key(p);ConditionalCooperation.Trace t=traces.get(k);
        if(t!=null){host.event("trace_cache_hits");return t;}
        t=host.replay(p);if(t!=null && host.allowed()){
            traces.put(k,t);if(traces.size()>128)traces.remove(traces.keySet().iterator().next());
        }return t;
    }
    private double chord(List<GridPoint> p,int l,int r){return Math.hypot((p.get(r).j-p.get(l).j)*dx,(p.get(r).i-p.get(l).i)*dy);}
    private double geometryDistance(List<GridPoint>a,int l,int r,List<GridPoint>b,int u,int v){
        double s=0;for(double f:new double[]{.25,.5,.75}){
            double[] x=local(a,l,r,f,dx,dy),y=local(b,u,v,f,dx,dy);
            s+=(x[0]-y[0])*(x[0]-y[0])+(x[1]-y[1])*(x[1]-y[1]);
        }return s/6;
    }
    /** Arc-length correspondence permits existing deletion edits without invented vertex identities. */
    public static List<GridPoint> transport(List<GridPoint> before,int l,int r,List<GridPoint> after,int al,int ar,
            List<GridPoint> recipient,int u,int v,boolean residual,double dx,double dy){
        List<GridPoint> out=copy(recipient);double[] fractions=fractions(recipient,u,v,dx,dy);
        GridPoint a=recipient.get(u),b=recipient.get(v);
        double x=(b.j-a.j)*dx,y=(b.i-a.i)*dy,len=Math.hypot(x,y);
        if(len<EPS)return out;
        for(int k=u+1;k<v;k++){
            double f=fractions[k-u];double[] old=local(before,l,r,f,dx,dy),next=local(after,al,ar,f,dx,dy);
            double ex=next[0]-(residual?old[0]:0),ey=next[1]-(residual?old[1]:0);
            GridPoint origin=residual?recipient.get(k):a;
            double pj=origin.j+(ex*x-ey*y)/dx,pi=origin.i+(ex*y+ey*x)/dy;
            if(!Double.isFinite(pi) || !Double.isFinite(pj) || Math.abs(pi)>Integer.MAX_VALUE-1.0 || Math.abs(pj)>Integer.MAX_VALUE-1.0)return copy(recipient);
            out.set(k,new GridPoint((int)Math.round(pi),(int)Math.round(pj)));
        }return out;
    }
    private static double[] fractions(List<GridPoint> p,int l,int r,double dx,double dy){
        double[] f=new double[r-l+1];for(int k=l+1;k<=r;k++)f[k-l]=f[k-l-1]+Math.hypot((p.get(k).j-p.get(k-1).j)*dx,(p.get(k).i-p.get(k-1).i)*dy);
        double length=f[f.length-1];if(length>EPS)for(int k=1;k<f.length;k++)f[k]/=length;return f;
    }
    private static double[] local(List<GridPoint> p,int l,int r,double f,double dx,double dy){
        double[] fs=fractions(p,l,r,dx,dy);int k=1;while(k<fs.length-1 && fs[k]<f)k++;
        double den=fs[k]-fs[k-1],t=den>EPS?(f-fs[k-1])/den:0;
        GridPoint a=p.get(l),b=p.get(r),c=p.get(l+k-1),d=p.get(l+k);
        double x=(b.j-a.j)*dx,y=(b.i-a.i)*dy,len2=x*x+y*y;
        if(len2<EPS)return new double[]{0,0};
        double px=(c.j+t*(d.j-c.j)-a.j)*dx,py=(c.i+t*(d.i-c.i)-a.i)*dy;
        return new double[]{(px*x+py*y)/len2,(-px*y+py*x)/len2};
    }
    private static boolean finite(double[] a){if(a==null || a.length==0)return false;for(double v:a)if(!Double.isFinite(v))return false;return true;}
    private static double meanSquare(double[] a,double[] b){double s=0;for(int k=0;k<a.length;k++)s+=(a[k]-b[k])*(a[k]-b[k]);return s/a.length;}
    private static String key(List<GridPoint> p){StringBuilder b=new StringBuilder();for(GridPoint g:p)b.append(g.i).append(',').append(g.j).append(';');return b.toString();}
    private static List<GridPoint> copy(List<GridPoint> p){List<GridPoint> q=new ArrayList<>();for(GridPoint g:p)q.add(new GridPoint(g.i,g.j));return q;}
}
