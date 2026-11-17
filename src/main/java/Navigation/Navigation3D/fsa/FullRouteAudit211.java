package Navigation.Navigation3D.fsa;
import Navigation.model.GridPoint;
import java.util.*;
import java.util.function.DoubleSupplier;
/** Observation only: return the original evaluator value and propagate every failure. */
public final class FullRouteAudit211 {
    public static volatile String phase="UNSET";
    public static final ThreadLocal<String> currentRoute=new ThreadLocal<>();
    private static List<GridPoint> seed;
    private static long calls,finite,incomplete,interrupted;
    private static final Map<String,Map<String,Object>> unique=new LinkedHashMap<>();
    public static synchronized void configure(List<GridPoint> s){seed=new ArrayList<>(s);calls=finite=incomplete=interrupted=0;unique.clear();}
    public static double evaluate(FsaContext ctx,List<GridPoint> route,DoubleSupplier original){
        boolean full=phase.equals("SEARCH_EXECUTE")&&seed!=null&&route!=null&&route.size()>1&&route.get(0).equals(seed.get(0))&&route.get(route.size()-1).equals(seed.get(seed.size()-1));
        if(full)synchronized(FullRouteAudit211.class){calls++;}
        double value;
        String previous=currentRoute.get();StringBuilder routeKey=new StringBuilder();if(route!=null)for(GridPoint p:route)routeKey.append(p.i).append(",").append(p.j).append(";");
        currentRoute.set(routeKey.toString());
        try{value=original.getAsDouble();}
        catch(RuntimeException error){if(full)synchronized(FullRouteAudit211.class){interrupted++;}throw error;}
        finally{if(previous==null)currentRoute.remove();else currentRoute.set(previous);}
        if(full)synchronized(FullRouteAudit211.class){
            boolean valid=Double.isFinite(value)&&value>=0&&value<1e17&&IntegrationAudit.practical(route,ctx);
            if(valid){finite++;StringBuilder key=new StringBuilder();for(GridPoint p:route)key.append(p.i).append(',').append(p.j).append(';');
                if(!unique.containsKey(key.toString())){Map<String,Object> r=new LinkedHashMap<>();r.put("points",new ArrayList<>(route));r.put("native_composite",value);r.put("is_seed",route.equals(seed));unique.put(key.toString(),r);}}
            else incomplete++;
        }
        return value;
    }
    public static synchronized Map<String,Object> snapshot(){Map<String,Object> r=new LinkedHashMap<>();r.put("full_endpoint_calls",calls);r.put("complete_practical_calls",finite);r.put("incomplete_or_infeasible_calls",incomplete);r.put("interrupted_calls",interrupted);r.put("unique_complete_routes",new ArrayList<>(unique.values()));return r;}
}
