package Navigation.benchmark;
import com.fasterxml.jackson.databind.ObjectMapper;
import Navigation.model.GridPoint;
import java.nio.file.*;
import java.nio.charset.StandardCharsets;
import java.util.*;
/** Serialize only the already-paid rejected audit; never evaluate a route again. */
public final class PaidFailureAudit416 {
 public static void write(String origin,Map<String,Object> audit,List<GridPoint> route)throws Exception {
  String file=System.getProperty("nav416.failureEvidence");if(file==null)return;
  Map<String,Object> result=new LinkedHashMap<>();result.put("origin",origin);
  result.put("audit",audit);result.put("route",route);
  result.put("new_model_or_route_evaluations",0);result.put("original_rejection_unchanged",true);
  Files.writeString(Paths.get(file),new ObjectMapper().writerWithDefaultPrettyPrinter().writeValueAsString(result),
                    StandardCharsets.UTF_8,StandardOpenOption.CREATE_NEW,StandardOpenOption.WRITE);
 }
}
