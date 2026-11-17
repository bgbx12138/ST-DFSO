package Navigation.benchmark;
import Navigation.Navigation3D.HybridFSA3D;
import java.util.*;
/** Configuration only. Original algorithm, common repair and evaluation stay untouched. */
public final class FinalArm351 {
    static final String[] ARMS={"OFF_SF","MINUS_LL","MINUS_AV","MINUS_FG","MINUS_ER","CORE"};
    static void configure(HybridFSA3D f,String arm){
        if(!Arrays.asList(ARMS).contains(arm))throw new IllegalArgumentException("FINAL351_ARM");
        AvailabilityLauncher211.configure(f,"OFF_SF");
        boolean core=arm.equals("CORE"),fg=!core&&!arm.equals("MINUS_FG"),er=!core&&!arm.equals("MINUS_ER");
        f.setEnableLateralLine(!core&&!arm.equals("MINUS_LL")).setEnableAdaptiveVision(!core&&!arm.equals("MINUS_AV"))
          .setEnableSwarmFusion(false).setEnableScatteredForaging(fg).setEnableForagingImitation(fg).setEnableForagingExploration(fg)
          .setEnableEscapeResponse(er).setEnablePreyInspection(er);
    }
}
