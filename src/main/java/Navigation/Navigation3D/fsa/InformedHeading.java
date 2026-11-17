package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import java.util.*;

/** Bounded heading proposals. No route splicing, risk adaptation, or unmetered edge sampling. */
public final class InformedHeading {
    public interface Host {
        boolean allowed();
        boolean completedAllowed();
        boolean legal(GridPoint a, GridPoint b);
        double distance(GridPoint a, GridPoint b);
        double bearing(GridPoint a, GridPoint b);
        double sample(GridPoint a, GridPoint b, double elapsed);
        List<GridPoint> candidates(GridPoint a, Set<GridPoint> visited);
        void count(String name);
    }
    public final Map<String,Object> witness=new LinkedHashMap<>();
    private final Host h;
    private final boolean social;
    private int changed,used;
    public InformedHeading(Host host,boolean useSocial){h=host;social=useSocial;}
    public static double angle(double a,double b){double d=Math.abs(a-b)%360;return Math.min(d,360-d);}
    public static double middle(double a,double b){double d=((b-a+540)%360)-180;return (a+d/2+360)%360;}

    /** Unique representatives; tie-breaks are geometry only and shared by both arms. */
    public static List<GridPoint> representatives(Host h,GridPoint from,List<GridPoint> pool,double heading,double goal,double third){
        List<GridPoint> result=new ArrayList<>();
        for(double target:new double[]{heading,goal,third}){
            GridPoint best=null;double error=Double.POSITIVE_INFINITY,turn=Double.POSITIVE_INFINITY,dist=-1;
            for(GridPoint p:pool){
                if(result.contains(p))continue;
                double e=angle(h.bearing(from,p),target),t=angle(h.bearing(from,p),heading),d=h.distance(from,p);
                if(e<error-1e-12 || (Math.abs(e-error)<1e-12 && (t<turn-1e-12 || (Math.abs(t-turn)<1e-12 && d>dist)))){
                    best=p;error=e;turn=t;dist=d;
                }
            }
            if(best!=null)result.add(best);
        }
        return result;
    }

    /** Both arms pay the same read-only leader query; plain discards its direction. */
    private Double leader(GridPoint from,List<EliteArchive.Entry> entries,double radius){
        double bestD=Double.POSITIVE_INFINITY,bestCost=Double.POSITIVE_INFINITY;Double heading=null;
        for(EliteArchive.Entry entry:entries){
            if(!h.allowed())return null;
            for(int i=0;i+1<entry.path.size();i++){
                h.count("ih_leader_segments_examined");
                GridPoint a=entry.path.get(i),b=entry.path.get(i+1);double d=h.distance(from,a);
                if(d>radius || (!from.equals(a) && !h.legal(from,a)))continue;
                if(d<bestD-1e-12 || (Math.abs(d-bestD)<1e-12 && entry.cost<bestCost)){
                    bestD=d;bestCost=entry.cost;heading=h.bearing(a,b);
                }
            }
        }
        return heading;
    }

    public List<GridPoint> build(List<GridPoint> prefix,GridPoint start,GridPoint end,double initialHeading,
                                  List<EliteArchive.Entry> entries,double radius,double nominalKnots,int maxSteps){
        h.count("ih_constructions");
        List<GridPoint> path=new ArrayList<>(prefix);if(path.isEmpty())path.add(start);
        witness.put("social",social);witness.put("prefix",new ArrayList<>(path));
        List<Map<String,Object>> logs=new ArrayList<>();witness.put("steps",logs);
        List<List<GridPoint>> socialEdges=new ArrayList<>();witness.put("social_edges",socialEdges);
        if(!path.get(path.size()-1).equals(start) || new HashSet<>(path).size()!=path.size())return fail("PREFIX");
        double time=0,heading=initialHeading;
        for(int i=1;i<path.size();i++){
            if(!h.allowed() || !h.legal(path.get(i-1),path.get(i)))return fail("PREFIX");
            h.count("ih_prefix_samples");time=h.sample(path.get(i-1),path.get(i),time);
            if(!Double.isFinite(time) || !h.completedAllowed())return fail("PREFIX_SAMPLE");
            heading=h.bearing(path.get(i-1),path.get(i));
        }
        witness.put("prefix_elapsed",time);
        Set<GridPoint> visited=new HashSet<>(path);GridPoint from=start;
        for(int step=0;step<maxSteps && !from.equals(end);step++){
            if(!h.allowed())return fail("BUDGET");
            List<GridPoint> pool=h.candidates(from,visited);
            pool.removeIf(p->visited.contains(p));
            // Host supplies only legal points; no arbitrary first-visible fallback.
            if(pool.isEmpty())return fail("NO_CANDIDATE");
            double goal=h.bearing(from,end),plain=middle(heading,goal);
            Double advice=leader(from,entries,radius);
            if(advice!=null)h.count("ih_direction_available");
            List<GridPoint> ordinary=representatives(h,from,pool,heading,goal,plain);
            List<GridPoint> actual=social && advice!=null?representatives(h,from,pool,heading,goal,advice):ordinary;
            boolean different=!actual.equals(ordinary);if(different){changed++;h.count("ih_candidate_set_changed");}
            h.count("ih_steps");
            GridPoint chosen=null;double best=Double.POSITIVE_INFINITY,bestTurn=Double.POSITIVE_INFINITY,nextTime=Double.NaN;
            List<Double> ends=new ArrayList<>();
            for(GridPoint p:actual){
                if(!h.allowed())return fail("BUDGET");
                h.count("ih_candidate_samples");double arrival=h.sample(from,p,time);
                ends.add(Double.isFinite(arrival)?arrival:null);
                if(!h.completedAllowed())return fail("BUDGET");
                if(!Double.isFinite(arrival) || arrival<time)continue;
                double score=arrival-time+h.distance(p,end)/Math.max(nominalKnots,0.1),turn=angle(h.bearing(from,p),heading);
                if(score<best-1e-12 || (Math.abs(score-best)<1e-12 && turn<bestTurn)){
                    best=score;bestTurn=turn;chosen=p;nextTime=arrival;
                }
            }
            if(chosen==null)return fail("NO_VALID_EDGE");
            boolean usedHere=different && !ordinary.contains(chosen);
            if(usedHere){used++;h.count("ih_social_direction_selected");socialEdges.add(Arrays.asList(from,chosen));}
            if(logs.size()<64){Map<String,Object> log=new LinkedHashMap<>();
                log.put("from",from);log.put("entry_elapsed",time);log.put("heading",heading);log.put("leader",advice);
                log.put("ordinary",ordinary);log.put("candidates",actual);log.put("arrivals",ends);log.put("selected",chosen);
                log.put("social_selected",usedHere);logs.add(log);}
            heading=h.bearing(from,chosen);from=chosen;time=nextTime;path.add(chosen);visited.add(chosen);
        }
        if(!from.equals(end))return fail("STEP_LIMIT");
        witness.put("raw_path",new ArrayList<>(path));witness.put("raw_elapsed",time);witness.put("status","VALID");
        witness.put("changed_steps",changed);witness.put("social_selected_steps",used);
        h.count("ih_completed_routes");return path;
    }
    private List<GridPoint> fail(String why){witness.put("status",why);witness.put("changed_steps",changed);witness.put("social_selected_steps",used);h.count("ih_failed_"+why);return null;}
}
