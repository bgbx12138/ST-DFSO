package Navigation.Navigation3D.fsa;

import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.safety.ResolvedDynamicResponseCost;
import Navigation.model.GridPoint;
import java.util.*;
import java.lang.reflect.*;

/** Observation only: no response formula, input rounding, cache or acceptance changes. */
public final class IntegrationAudit {
    private static final ThreadLocal<Capture> ACTIVE=new ThreadLocal<>();
    public static final class Capture {
        double valid,failed,missing,unsupported,total,exact,approximated;
        final Map<String,Object> responseQueries=new LinkedHashMap<>();
        final double[] channels=new double[4];
        final Map<String,Double> reasons=new LinkedHashMap<>();
    }
    public static void observe(double nm,ResearchResponsePort.Response r){
        Capture c=ACTIVE.get();if(c==null)return;c.total+=nm;
        Map<String,Object> detail=new LinkedHashMap<>();detail.put("status",r.status.name());detail.put("sd",r.sd);detail.put("cost",r.cost);
        try{detail.put("provenance",new com.fasterxml.jackson.databind.ObjectMapper().readValue(r.provenance,Map.class));}catch(Exception e){throw new IllegalArgumentException("BAD_PROVENANCE",e);}
        detail.put("input_sha256",r.inputSha);
        Map<String,Object> previous=(Map<String,Object>)c.responseQueries.get(r.inputSha);
        double prior=previous==null?0.:((Number)previous.get("sampled_nm")).doubleValue();
        int count=previous==null?0:((Number)previous.get("sample_count")).intValue();
        if(previous!=null&&(!previous.get("status").equals(r.status.name())||!Objects.equals(previous.get("cost"),r.cost)))
            throw new IllegalStateException("RESPONSE_CHANGED_WITHIN_FROZEN_ROUTE_AUDIT");
        detail.put("sampled_nm",prior+nm);detail.put("sample_count",count+1);
        c.responseQueries.put(r.inputSha,detail);
        switch(r.status){case OK:c.valid+=nm;c.exact+=nm;break;case APPROXIMATED:c.valid+=nm;c.approximated+=nm;break;case MISSING_RESPONSE:c.missing+=nm;break;
            case UNSUPPORTED_INPUT:c.unsupported+=nm;break;default:c.failed+=nm;}
        if(ResearchResponsePort.isPriced(r.status)){
            double[] w=ResolvedDynamicResponseCost.authorWeights(),q=ResolvedDynamicResponseCost.referenceScales();
            for(int j=0;j<4;j++)c.channels[j]+=nm*w[j]*r.sd[j]/q[j];
        }else c.reasons.merge(r.status+":"+r.reason,nm,Double::sum);
    }
    public static FsaContext context(Object external)throws Exception{
        Field f=external.getClass().getDeclaredField("ctxComposite");f.setAccessible(true);return (FsaContext)f.get(external);
    }
    private static Object fields(Object o)throws Exception{
        Map<String,Object> v=new TreeMap<>();if(o==null)return v;
        for(Field f:o.getClass().getDeclaredFields())if(!Modifier.isStatic(f.getModifiers())){f.setAccessible(true);v.put(f.getName(),f.get(o));}return v;
    }
    public static Map<String,Object> normalizers(FsaContext c)throws Exception{
        Map<String,Object> x=new LinkedHashMap<>();x.put("time",fields(c.timeNormalizer));x.put("fuel",fields(c.fuelNormalizer));
        x.put("time_enabled",c.config.enableCompositeNormalization&&c.config.enableCompositeTimeNormalization);
        x.put("fuel_enabled",c.config.enableCompositeNormalization&&c.config.enableCompositeFuelNormalization);
        x.put("safety_rule","raw unbounded response cost per nm; no 1-r or clipping");
        x.put("weights",ResolvedDynamicResponseCost.authorWeights());x.put("scales",ResolvedDynamicResponseCost.referenceScales());return x;
    }
    public static Map<String,Object> replay(FsaContext c,List<GridPoint> p,ObjectiveWeights w){
        if(ACTIVE.get()!=null)throw new IllegalStateException("nested audit");
        Capture capture=new Capture();ACTIVE.set(capture);
        double length=0,time=0,fuel=0,risk=0,nt=0,nf=0,unassessed=0;boolean valid=p!=null&&p.size()>1;
        List<Map<String,Object>> edges=new ArrayList<>();int clippedTime=0,clippedFuel=0;
        double clockNt=0,clockNf=0;boolean allClocks=true;
        CompositeEvaluator comp=new CompositeEvaluator(c,w.normalized());
        try{
            if(valid)for(int k=0;k<p.size()-1;k++){
                GridPoint a=p.get(k),b=p.get(k+1);double distance=c.gridDistanceNm(a.i,a.j,b.i,b.j);length+=distance;
                Map<String,Object> row=new LinkedHashMap<>();row.put("index",k);row.put("start_hours",time);row.put("distance_nm",distance);
                boolean legal=c.routeGraph.isValidEdge(a,b);
                ACTIVE.remove();
                EdgeSampler.EdgeResult clock=legal?ResearchResponsePort.sample(c,a.i,a.j,b.i,b.j,time,new EdgeSampler.NeedComponents(true,true,false),true):EdgeSampler.EdgeResult.invalid();
                ACTIVE.set(capture);
                double before=capture.total;
                if(!clock.valid){allClocks=false;valid=false;unassessed+=distance;row.put("status","GEOMETRY_WEATHER_OR_TIME_UNSUPPORTED");edges.add(row);break;}
                int clockDir=FsaContext.nearestBearingIndex(c.bearingOf(a.i,a.j,b.i,b.j));
                clockNt+=c.timeNormalizer.normalize(clock.timeHours,distance,clockDir)*distance;
                clockNf+=c.fuelNormalizer.normalize(clock.fuelEnergyJ,distance,clockDir)*distance;
                EdgeSampler.EdgeResult e=EdgeSampler.sample(c,a.i,a.j,b.i,b.j,time,EdgeSampler.NeedComponents.all(),true);
                double observed=capture.total-before;unassessed+=Math.max(0,distance-observed);
                if(!e.valid)valid=false;
                if(e.valid){
                    risk+=e.riskWeighted;int dir=FsaContext.nearestBearingIndex(c.bearingOf(a.i,a.j,b.i,b.j));
                    double tv=c.timeNormalizer.normalize(e.timeHours,distance,dir),fv=c.fuelNormalizer.normalize(e.fuelEnergyJ,distance,dir);
                    nt+=tv*distance;nf+=fv*distance;if(tv<=0||tv>=1)clippedTime++;if(fv<=0||fv>=1)clippedFuel++;
                    double reconstructed=w.normalized().safety*e.riskWeighted+w.normalized().time*tv*distance+w.normalized().fuel*fv*distance;
                    if(Math.abs(comp.edgeCost(e,a.i,a.j,b.i,b.j,c.clampDay(time))-reconstructed)>1e-7*Math.max(1,reconstructed))throw new AssertionError("COMPOSITE_DECOMPOSITION");
                }
                row.put("status",e.valid?"OK":"UNPRICED_RESPONSE");row.put("end_hours",clock.endElapsedHours);
                row.put("response_sampled_nm",observed);edges.add(row);time=clock.endElapsedHours;fuel+=clock.fuelEnergyJ;
            }
        }finally{ACTIVE.remove();}
        // Keep all remaining geometry in the denominator even if the navigation clock failed.
        double expected=0;if(p!=null)for(int k=1;k<p.size();k++)expected+=c.gridDistanceNm(p.get(k-1).i,p.get(k-1).j,p.get(k).i,p.get(k).j);
        unassessed+=Math.max(0,expected-length);length=expected;
        boolean complete=valid&&capture.failed==0&&capture.missing==0&&capture.unsupported==0&&unassessed<1e-7&&Math.abs(capture.total-length)<1e-7&&c.meetsArrivalDeadline(time);
        double channelSum=Arrays.stream(capture.channels).sum();
        if(complete&&Math.abs(channelSum-risk)>1e-8*Math.max(1,risk))throw new AssertionError("CHANNEL_EXPOSURE_SUM");
        Map<String,Object>x=new LinkedHashMap<>();x.put("complete",complete);x.put("distance_nm",length);
        Map<String,Double> objectiveWeights=new LinkedHashMap<>();objectiveWeights.put("safety",w.normalized().safety);
        objectiveWeights.put("time",w.normalized().time);objectiveWeights.put("fuel",w.normalized().fuel);
        x.put("objective_weights",objectiveWeights);x.put("valid_nm",capture.valid);
        x.put("exact_nm",capture.exact);x.put("approximated_nm",capture.approximated);x.put("contains_approximation",capture.approximated>0);x.put("response_queries",capture.responseQueries);
        x.put("failed_nm",capture.failed);x.put("missing_nm",capture.missing);x.put("unsupported_nm",capture.unsupported);x.put("unassessed_nm",unassessed);
        x.put("reason_nm",capture.reasons);x.put("channel_exposure_partial",capture.channels);x.put("risk_total",complete?risk:null);x.put("risk_density",complete?risk/length:null);
        x.put("time_hours",complete?time:null);x.put("fuel_j",complete?fuel:null);x.put("normalized_time_integral",complete?nt:null);x.put("normalized_fuel_integral",complete?nf:null);
        x.put("composite",complete?w.normalized().safety*risk+w.normalized().time*nt+w.normalized().fuel*nf:null);
        boolean clockComplete=allClocks&&p!=null&&edges.size()==p.size()-1&&c.meetsArrivalDeadline(time);
        if(complete&&(Math.abs(nt-clockNt)>1e-8||Math.abs(nf-clockNf)>1e-8))throw new AssertionError("COMPLETE_CLOCK_NORMALIZATION_CHANGED");
        x.put("kinematics_complete",clockComplete);
        x.put("navigation_time_hours",clockComplete?time:null);x.put("navigation_fuel_j",clockComplete?fuel:null);
        x.put("navigation_normalized_time_integral",clockComplete?clockNt:null);
        x.put("navigation_normalized_fuel_integral",clockComplete?clockNf:null);
        x.put("cumulative_risk_lower_bound",clockComplete?channelSum:null);
        x.put("risk_density_lower_bound",clockComplete?channelSum/length:null);
        x.put("composite_lower_bound",clockComplete?w.normalized().safety*channelSum+w.normalized().time*clockNt+w.normalized().fuel*clockNf:null);
        x.put("response_missing_values_imputed",false);
        x.put("bound_scope","Nonnegative unknown response rates; saved fixed route only; not a completed score or mechanism win");
        x.put("normalized_time_boundary_edges",clippedTime);x.put("normalized_fuel_boundary_edges",clippedFuel);x.put("edges",edges);return x;
    }
    public static boolean practical(List<GridPoint> p,FsaContext c){
        if(p==null||p.size()<2||new HashSet<>(p).size()!=p.size())return false;
        for(int k=1;k<p.size()-1;k++){
            GridPoint a=p.get(k-1),b=p.get(k),d=p.get(k+1);
            double x=(b.j-a.j)*c.dLonNm,y=(b.i-a.i)*c.dLatNm,u=(d.j-b.j)*c.dLonNm,v=(d.i-b.i)*c.dLatNm;
            if(x*u+y*v<0&&Math.abs(x*v-y*u)<1e-10*Math.max(1,Math.hypot(x,y)*Math.hypot(u,v)))return false;
        }return true;
    }
}
