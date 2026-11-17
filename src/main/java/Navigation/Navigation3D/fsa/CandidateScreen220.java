package Navigation.Navigation3D.fsa;
import Navigation.model.GridPoint;
import Navigation.Navigation3D.HybridFSA3D.ObjectiveMode;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import java.util.*;
import java.util.function.DoubleSupplier;
import com.fasterxml.jackson.databind.ObjectMapper;

/** Shared exact-view lower-bound screen; original evaluation alone supplies objectives. */
public final class CandidateScreen220 {
    private static final ObjectMapper J=new ObjectMapper();
    private static final List<Map<String,Object>> rows=new ArrayList<>();
    private static final Map<String,Long> counts=new LinkedHashMap<>();
    public static synchronized void reset(){rows.clear();counts.clear();}
    static double finite(Map<String,Object> a,String k){Object x=a.get(k);if(!(x instanceof Number)||!Double.isFinite(((Number)x).doubleValue()))throw new IllegalArgumentException("BOUND220_NONFINITE_"+k);return ((Number)x).doubleValue();}
    static String key(List<GridPoint> route){StringBuilder b=new StringBuilder();for(GridPoint p:route)b.append(p.i).append(',').append(p.j).append(';');return b.toString();}
    public static double reconstructedBound(Map<String,Object> audit){
        if(!Boolean.TRUE.equals(audit.get("kinematics_complete")))return Double.NaN;
        for(String k:new String[]{"failed_nm","unsupported_nm","approximated_nm"})if(finite(audit,k)!=0.)throw new IllegalArgumentException("BOUND220_UNSUPPORTED_SOURCE_"+k);
        if(Boolean.TRUE.equals(audit.get("response_missing_values_imputed")))throw new IllegalArgumentException("BOUND220_IMPUTATION");
        Object cs=audit.get("channel_exposure_partial");if(!(cs instanceof double[])){
            if(!(cs instanceof List))throw new IllegalArgumentException("BOUND220_CHANNELS");
            List list=(List)cs;double[] ar=new double[list.size()];for(int k=0;k<ar.length;k++)ar[k]=((Number)list.get(k)).doubleValue();cs=ar;
        }
        double[] c=(double[])cs;if(c.length!=4)throw new IllegalArgumentException("BOUND220_FOUR_CHANNELS");
        double risk=0;for(double x:c){if(!Double.isFinite(x)||x<0)throw new IllegalArgumentException("BOUND220_NEGATIVE_CHANNEL");risk+=x;}
        Map<String,Object> w=(Map<String,Object>)audit.get("objective_weights");double sw=finite(w,"safety"),tw=finite(w,"time"),fw=finite(w,"fuel");
        if(sw<0||tw<0||fw<0)throw new IllegalArgumentException("BOUND220_NEGATIVE_WEIGHT");
        double nt=finite(audit,"navigation_normalized_time_integral"),nf=finite(audit,"navigation_normalized_fuel_integral");
        if(nt<0||nf<0)throw new IllegalArgumentException("BOUND220_NEGATIVE_NAVIGATION");
        double bound=sw*risk+tw*nt+fw*nf;
        if(Math.abs(bound-finite(audit,"composite_lower_bound"))>1e-8*Math.max(1.,Math.abs(bound)))throw new IllegalArgumentException("BOUND220_DECOMPOSITION");
        return bound;
    }
    public static double reconstructedBound(Map<String,Object> audit,ObjectiveMode goal){
        if(goal==ObjectiveMode.COMPOSITE)return reconstructedBound(audit);
        if(goal!=ObjectiveMode.SAFETY_EXPOSURE && goal!=ObjectiveMode.SAFETY_DENSITY)throw new IllegalArgumentException("BOUND224_UNSUPPORTED_GOAL");
        double common=reconstructedBound(audit);if(!Double.isFinite(common))return Double.NaN;
        Object values=audit.get("channel_exposure_partial");double risk=0.;
        if(values instanceof double[])for(double v:(double[])values)risk+=v;
        else for(Object v:(List)values)risk+=((Number)v).doubleValue();
        if(Math.abs(risk-finite(audit,"cumulative_risk_lower_bound"))>1e-8*Math.max(1.,Math.abs(risk)))throw new IllegalArgumentException("BOUND224_RISK_DECOMPOSITION");
        double distance=finite(audit,"distance_nm");if(distance<=0.)throw new IllegalArgumentException("BOUND224_POSITIVE_COMPLETE_DISTANCE");
        double density=risk/distance;
        if(Math.abs(density-finite(audit,"risk_density_lower_bound"))>1e-8*Math.max(1.,Math.abs(density)))throw new IllegalArgumentException("BOUND224_DENSITY_DECOMPOSITION");
        return goal==ObjectiveMode.SAFETY_DENSITY?density:risk;
    }
    public static boolean sourceEligible230(Map<String,Object> audit){
        for(String name:new String[]{"failed_nm","unsupported_nm","approximated_nm"}){
            double value=finite(audit,name);if(value<0.)throw new IllegalArgumentException("BOUND230_NEGATIVE_SOURCE_"+name);
            if(value>0.)return false;
        }
        return true;
    }
    public static boolean excludes(double bound,double incumbent){
        return Double.isFinite(bound)&&Double.isFinite(incumbent)&&incumbent>=0&&incumbent<1e17&&bound>incumbent+1e-7*Math.max(1.,Math.abs(incumbent));
    }
    public static void verifyIdentity(String routeBefore,String routeAfter,String normBefore,String normAfter,String supplyBefore,String supplyAfter){
        if(!routeBefore.equals(routeAfter)||!normBefore.equals(normAfter)||!supplyBefore.equals(supplyAfter))throw new IllegalStateException("BOUND220_ROUTE_SUPPLY_NORMALIZER_CHANGED");
    }
    private static synchronized void record(String state,String route,Double bound,Double incumbent,String revision,long nanos,Map<String,Object> audit,ObjectiveMode goal){
        counts.merge(state,1L,Long::sum);Map<String,Object> r=new LinkedHashMap<>();r.put("status",state);r.put("route_key",route);r.put("objective",null);r.put("objective_mode",goal.name());
        r.put("bound",bound);r.put("incumbent",incumbent);r.put("supply_revision",revision);r.put("wall_nanos",nanos);r.put("physical_unsafe_inferred",false);
        if(audit!=null){r.put("exact_nm",audit.get("exact_nm"));r.put("missing_nm",audit.get("missing_nm"));r.put("unassessed_nm",audit.get("unassessed_nm"));
            r.put("unsupported_nm",audit.get("unsupported_nm"));r.put("failed_nm",audit.get("failed_nm"));r.put("approximated_nm",audit.get("approximated_nm"));r.put("kinematics_complete",audit.get("kinematics_complete"));r.put("complete",audit.get("complete"));r.put("query_count",((Map)audit.get("response_queries")).size());}
        if(rows.size()<4096)rows.add(r);
    }
    public static double evaluate(FsaContext ctx,List<GridPoint> route,ObjectiveWeights weights,double incumbent,DoubleSupplier original){
        return evaluate(ctx,route,ObjectiveMode.COMPOSITE,weights,incumbent,original);
    }
    public static double evaluate(FsaContext ctx,List<GridPoint> route,ObjectiveMode goal,ObjectiveWeights weights,double incumbent,DoubleSupplier original){
        if(!Boolean.getBoolean("nav220.screen")||(goal!=ObjectiveMode.COMPOSITE&&goal!=ObjectiveMode.SAFETY_EXPOSURE&&goal!=ObjectiveMode.SAFETY_DENSITY)||(goal==ObjectiveMode.COMPOSITE&&weights==null)||!Double.isFinite(incumbent)||incumbent<0||incumbent>=1e17)return original.getAsDouble();
        if(!FullRouteAudit211.phase.equals("SEARCH_EXECUTE")||ComponentProcessProvider211.CURRENT==null||!ResearchResponsePort.installed())throw new IllegalStateException("BOUND220_SEARCH_PROVIDER_REQUIRED");
        if(!IntegrationAudit.practical(route,ctx))return original.getAsDouble();
        long start=System.nanoTime();String routeBefore=key(route),revision=ComponentProcessProvider211.CURRENT.revision;
        Map<String,Object> audit;String normBefore;
        try{
            normBefore=J.writeValueAsString(IntegrationAudit.normalizers(ctx));
            try(ComponentProcessProvider211.ScreenScope220 scope=ComponentProcessProvider211.readOnlyScreen220()){
                String previous=FullRouteAudit211.currentRoute.get();FullRouteAudit211.currentRoute.set(routeBefore);
                try{audit=IntegrationAudit.replay(ctx,route,weights!=null?weights:new ObjectiveWeights(.5,.3,.2));}
                finally{if(previous==null)FullRouteAudit211.currentRoute.remove();else FullRouteAudit211.currentRoute.set(previous);}
            }
            verifyIdentity(routeBefore,key(route),normBefore,J.writeValueAsString(IntegrationAudit.normalizers(ctx)),revision,ComponentProcessProvider211.CURRENT.revision);
        }catch(RuntimeException e){record(TruthBudgetStop211.find(e)!=null?"GLOBAL_STOP":"CONTRACT_ERROR",routeBefore,null,incumbent,revision,System.nanoTime()-start,null,goal);throw e;
        }catch(Exception e){record("CONTRACT_ERROR",routeBefore,null,incumbent,revision,System.nanoTime()-start,null,goal);throw new IllegalStateException("BOUND220_AUDIT_CONTRACT",e);}
        if(!sourceEligible230(audit)){
            record("NO_BOUND_SOURCE230",routeBefore,null,incumbent,revision,System.nanoTime()-start,audit,goal);
            return original.getAsDouble();
        }
        double bound=reconstructedBound(audit,goal);
        if(excludes(bound,incumbent)){
            record("PRUNED_NONCOMPETITIVE",routeBefore,bound,incumbent,revision,System.nanoTime()-start,audit,goal);throw new CandidatePruned220(bound,incumbent);
        }
        record(Double.isFinite(bound)?"NOT_EXCLUDED":"NAVIGATION_UNPRICED",routeBefore,Double.isFinite(bound)?bound:null,incumbent,revision,System.nanoTime()-start,audit,goal);
        return original.getAsDouble();
    }
    public static synchronized Map<String,Object> snapshot(){Map<String,Object>x=new LinkedHashMap<>();x.put("counts",new LinkedHashMap<>(counts));x.put("rows",new ArrayList<>(rows));
        x.put("screening_work_in_shared_ledger",true);x.put("missing_objective_is_null",true);x.put("new_physical_solves_in_screen",0);return x;}
}
