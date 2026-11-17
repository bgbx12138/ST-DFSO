package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;
import java.util.*;
import java.util.function.DoubleSupplier;

/** Explicit result first. NaN is only the legacy double API's absence sentinel. */
public final class CandidateEvaluation217 {
    public enum Status { COMPLETE, NATIVE_REJECTED, UNPRICED, GLOBAL_STOP, CONTRACT_ERROR }
    public static final class Result {
        public final Status status;public final Double objective;private final double nativeScalar;
        Result(Status status,Double objective,double nativeScalar){this.status=status;this.objective=objective;this.nativeScalar=nativeScalar;}
        public boolean unpriced(){return status==Status.UNPRICED;}
        public double legacyScalar(){return unpriced()?Double.NaN:nativeScalar;}
    }
    private static final List<Map<String,Object>> rows=new ArrayList<>();
    private static final EnumMap<Status,Long> counts=new EnumMap<>(Status.class);
    private static long sequence;
    public static synchronized void reset(){rows.clear();counts.clear();sequence=0;}
    private static synchronized void record(Status status,List<GridPoint> route,Double value,RuntimeException error){
        long id=++sequence;counts.merge(status,1L,Long::sum);Map<String,Object> r=new LinkedHashMap<>();
        r.put("sequence",id);r.put("status",status.name());r.put("objective",value);r.put("thread",Thread.currentThread().getName());
        StringBuilder key=new StringBuilder();if(route!=null)for(GridPoint p:route)key.append(p.i).append(',').append(p.j).append(';');
        r.put("route_key",key.toString());r.put("physical_unsafe_inferred",false);
        if(error!=null){r.put("reason",error.getMessage());r.put("error_class",error.getClass().getName());}
        if(error instanceof CandidateUnpriced217){CandidateUnpriced217 u=(CandidateUnpriced217)error;
            r.put("request_key",u.requestKey);r.put("input_sha256",u.inputSha);r.put("source_status",u.sourceStatus);}
        if(rows.size()<4096)rows.add(r);
        Navigation.benchmark.CommonCandidateObservation337.record(status.name(),route,value);
    }
    public static Result evaluate(List<GridPoint> route,DoubleSupplier call){
        try{
            double value=call.getAsDouble();boolean finite=Double.isFinite(value)&&value>=0&&value<1e17;
            Status status=finite?Status.COMPLETE:Status.NATIVE_REJECTED;record(status,route,finite?value:null,null);
            return new Result(status,finite?value:null,value);
        }catch(CandidateUnpriced217 missing){
            record(Status.UNPRICED,route,null,missing);return new Result(Status.UNPRICED,null,Double.NaN);
        }catch(RuntimeException error){
            record(TruthBudgetStop211.find(error)!=null?Status.GLOBAL_STOP:Status.CONTRACT_ERROR,route,null,error);throw error;
        }
    }
    public static synchronized Map<String,Object> snapshot(){
        Map<String,Object> x=new LinkedHashMap<>();Map<String,Long> c=new LinkedHashMap<>();
        for(Status s:Status.values())c.put(s.name(),counts.getOrDefault(s,0L));
        x.put("counts",c);x.put("attempts",sequence);x.put("rows",new ArrayList<>(rows));x.put("rows_truncated",sequence>rows.size());
        x.put("unpriced_objective_is_null",true);x.put("finite_failure_penalty_used",false);return x;
    }
}
