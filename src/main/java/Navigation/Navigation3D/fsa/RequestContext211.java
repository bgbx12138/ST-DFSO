package Navigation.Navigation3D.fsa;
import java.nio.file.*;
import java.util.*;
import com.fasterxml.jackson.databind.ObjectMapper;
/** Common resource metadata; it never supplies an objective or physical label. */
public final class RequestContext211 {
    private static final ObjectMapper J=new ObjectMapper();
    private static volatile String phase="UNSET";
    private static volatile long deadline=Long.MAX_VALUE;
    public static void phase(String p){
        phase=p;
        if(p.equals("SEARCH_EXECUTE"))deadline=System.nanoTime()+Long.parseLong(System.getProperty("nav211.searchMillis"))*1000000L;
    }
    private static final java.util.concurrent.atomic.AtomicLong moveRetries228=new java.util.concurrent.atomic.AtomicLong();
    public static long retryCount228(){return moveRetries228.get();}
    public static void publish(ResearchResponsePort.Request r){
        String file=System.getenv("FSA_REQUEST_CONTEXT211");if(file==null)return;
        Map<String,Object> m=new LinkedHashMap<>();m.put("schema","MONOTONIC_SEARCH_REMAINING211");m.put("phase",phase);
        m.put("remaining_seconds",phase.equals("SEARCH_EXECUTE")?Math.max(0,(deadline-System.nanoTime())*1e-9):null);
        m.put("issued_epoch_ms",System.currentTimeMillis());m.put("request_key",r.key());
        String route=FullRouteAudit211.currentRoute.get();m.put("route_key",route);m.put("role",route==null?"EDGE_OR_NATIVE_AUDIT":"COMPLETE_CANDIDATE_OBJECTIVE");
        try{
            Path p=Paths.get(file),tmp=p.resolveSibling(p.getFileName()+".partial");J.writeValue(tmp.toFile(),m);
            for(int attempt=0;;attempt++){
                try{Files.move(tmp,p,StandardCopyOption.ATOMIC_MOVE,StandardCopyOption.REPLACE_EXISTING);break;}
                catch(java.nio.file.AccessDeniedException sharing){
                    if(attempt>=5)throw sharing;
                    moveRetries228.incrementAndGet();Thread.sleep(1L<<attempt);
                }
            }
        }catch(Exception e){throw new IllegalStateException("REQUEST_CONTEXT211_TRANSPORT_CONTRACT",e);}
    }
}
