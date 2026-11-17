package Navigation.benchmark;

import Navigation.Navigation3D.fsa.SearchWorkLedger;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;
import java.lang.reflect.Field;
import java.nio.file.Path;
import java.time.Instant;
import java.util.LinkedHashMap;
import java.util.Map;

/** Observation only. Reads the real ledger clock; does not start or reset it. */
public final class ClockBridge164 {
    private final Map<String,Long> before;
    private ClockBridge164(){before=anchor();}
    public static ClockBridge164 capture(){return new ClockBridge164();}
    private static Map<String,Long> anchor(){
        long a=System.nanoTime();Instant epoch=Instant.now();long b=System.nanoTime();
        Map<String,Long> m=new LinkedHashMap<>();m.put("nano_before",a);m.put("nano_after",b);
        m.put("epoch_ns",Math.addExact(Math.multiplyExact(epoch.getEpochSecond(),1000000000L),epoch.getNano()));
        return m;
    }
    public void write(Path path,SearchWorkLedger ledger)throws Exception{
        Map<String,Long> after=anchor();
        Field field=SearchWorkLedger.class.getDeclaredField("clockStartNanos");field.setAccessible(true);
        long start=field.getLong(ledger),guard=2000000L;
        long low=Math.max(before.get("epoch_ns")-before.get("nano_after")-guard,after.get("epoch_ns")-after.get("nano_after")-guard);
        long high=Math.min(before.get("epoch_ns")-before.get("nano_before")+guard,after.get("epoch_ns")-after.get("nano_before")+guard);
        boolean consistent=low<=high,started=start!=0;
        Map<String,Object> m=new LinkedHashMap<>();m.put("schema","SEARCH_LEDGER_CLOCK_BRIDGE164_V1");
        m.put("pid",ProcessHandle.current().pid());m.put("before",before);m.put("after",after);
        m.put("clock_resolution_guard_ns",guard);m.put("ledger_start_nano",start);
        m.put("ledger_elapsed_nanos",ledger.elapsedWallNanos());m.put("clock_consistent",consistent);
        m.put("search_started",started);m.put("join_eligible",consistent&&started);
        m.put("search_start_epoch_lower_ns",consistent&&started?start+low:null);
        m.put("search_start_epoch_upper_ns",consistent&&started?start+high:null);
        m.put("scope","Actual private ledger start via reflection, monotonic-to-epoch bracket. Guard is engineering clock tolerance, not CPU interpolation or inferred execute-start offset.");
        new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT).writeValue(path.toFile(),m);
    }
    public static void main(String[]args)throws Exception{
        ClockBridge164 bridge=capture();SearchWorkLedger ledger=new SearchWorkLedger(1000);
        ledger.startWallClock(5000);Thread.sleep(35);ledger.setActive(false);bridge.write(Path.of(args[0]),ledger);
        System.out.println("CLOCK_FIXTURE_ONLY_NO_OPTIMIZER");
    }
}
