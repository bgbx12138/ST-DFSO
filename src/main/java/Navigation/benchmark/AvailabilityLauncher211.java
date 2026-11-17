package Navigation.benchmark;
import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.*;
import Navigation.model.GridPoint;
import Navigation.object.NavigationConfig;
import Navigation.object.ShipParameters;
import java.lang.reflect.Field;
import java.nio.file.*;
import java.util.*;
import com.fasterxml.jackson.databind.*;

/** Registered 5415M candidates, exact-route native audit and frozen common evaluator. */
public final class AvailabilityLauncher211 {
    static final ObjectMapper J=new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
    static Map<String,Object> flags(HybridFSA3D f)throws Exception{
        Map<String,Object> x=new LinkedHashMap<>();
        for(String name:Arrays.asList("enableLateralLine","enableAdaptiveVision","enableSwarmFusion","enableEscapeResponse","enableScatteredForaging","enableForagingImitation","enableForagingExploration","enablePreyInspection","enableMultiplicativeSuccessorSelection","legalSuccessorCandidates","segmentLateralLine","completeCycleVision","arclengthForaging","reachableEscape","fusionRefinement","successorFormula")){
            Field field=HybridFSA3D.class.getDeclaredField(name);field.setAccessible(true);x.put(name,field.get(f));
        }return x;
    }
    static void configure(HybridFSA3D f,String arm){
        boolean core=arm.equals("CORE_OFF5"); boolean fgOnly=arm.equals("FG_ONLY");
        boolean ll=!core&&!fgOnly&&!arm.equals("OFF_LL"),av=!core&&!fgOnly&&!arm.equals("OFF_AV"),sf=!core&&!fgOnly&&!arm.equals("OFF_SF");
        boolean fg=!core&&!arm.equals("OFF_FG"),er=!core&&!fgOnly&&!arm.equals("OFF_ER");
        f.setSuccessorFormula("LOCAL_PROGRESS").setLegalSuccessorCandidates(true)
            .setSegmentLateralLine(!arm.equals("ALL_OLD_LL")).setCompleteCycleVision(true).setArclengthForaging(true)
            .setReachableEscape(true).setFusionRefinement("LOCAL")
            .setEnableLateralLine(ll).setEnableAdaptiveVision(av).setEnableSwarmFusion(sf)
            .setEnableScatteredForaging(fg).setEnableForagingImitation(fg).setEnableForagingExploration(fg)
            .setEnableEscapeResponse(er).setEnablePreyInspection(er);
    }
    static void mark(String phase)throws Exception{
        FullRouteAudit211.phase=phase;RequestContext211.phase(phase);ComponentProcessProvider211.auditPhase(phase);
        String s=System.getenv("FSA_AVAILABILITY_PHASE");if(s==null)throw new IllegalStateException("PHASE204_PATH_REQUIRED");
        Path p=Paths.get(s),t=p.resolveSibling(p.getFileName()+".partial");
        Files.writeString(t,phase,java.nio.charset.StandardCharsets.UTF_8,StandardOpenOption.CREATE_NEW);
        Files.move(t,p,StandardCopyOption.ATOMIC_MOVE,StandardCopyOption.REPLACE_EXISTING);
    }
    static Map<String,Object> audit(PathEvaluator p,List<GridPoint> route)throws Exception{
        FsaContext c=IntegrationAudit.context(p);Map<String,Object> x=IntegrationAudit.replay(c,route,BenchmarkLauncher.COMPOSITE_W);
        x.put("normalization",IntegrationAudit.normalizers(c));x.put("shape",p.analyzeRouteShape(route));
        x.put("practical",IntegrationAudit.practical(route,c));return x;
    }
}
