package Navigation.Navigation3D.fsa;
import java.nio.file.*;
import java.util.*;
import java.lang.reflect.*;
import com.fasterxml.jackson.databind.*;

/** Same frozen backend and scope, with process lifetime/cost receipts. */
public final class TableEntry211 {
    public static void main(String[] args)throws Exception{
        if(args.length<5)throw new IllegalArgumentException("PYTHON PROVIDER CACHE REPORT DELEGATE args");
        Path report=Paths.get(args[3]);if(Files.exists(report))throw new IllegalStateException("fresh report required");
        Files.createDirectories(report.toAbsolutePath().getParent());System.setProperty("nav.r1.component","true");
        long start=System.nanoTime();
        try(ComponentProcessProvider211 provider=new ComponentProcessProvider211(args[0],Paths.get(args[1]),Paths.get(args[2]),report.resolveSibling(report.getFileName()+".stderr.log"));
            ResearchResponsePort.Session session=new ResearchResponsePort.Session(provider)){
            if(!provider.revision.equals(System.getProperty("nav.r1.tableRevision","")))throw new IllegalStateException("TABLE_REVISION_REQUIRED");
            try{Class.forName(args[4]).getMethod("main",String[].class).invoke(null,(Object)Arrays.copyOfRange(args,5,args.length));}
            catch(InvocationTargetException e){Throwable c=e.getCause();if(c instanceof Exception)throw (Exception)c;throw e;}
            finally{
                ObjectMapper j=new ObjectMapper();Map<String,Object> s=j.readValue(provider.statistics(),Map.class);
                s.put("truth_control_stops",provider.controlStops);s.put("java_process_cpu_seconds",ProcessHandle.current().info().totalCpuDuration().orElse(java.time.Duration.ZERO).toNanos()*1e-9);
                s.put("entry_wall_seconds",(System.nanoTime()-start)*1e-9);s.put("process_tree_accounting","outer Windows Job receipt is authoritative for all descendants");
                j.writerWithDefaultPrettyPrinter().writeValue(report.toFile(),s);
            }
            if(provider.transportErrors>0)throw new IllegalStateException("COMPONENT_TRANSPORT_ERROR_NO_VERDICT");
        }finally{System.clearProperty("nav.r1.component");}
    }
}
