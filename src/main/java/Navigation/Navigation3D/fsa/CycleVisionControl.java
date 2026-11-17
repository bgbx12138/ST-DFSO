package Navigation.Navigation3D.fsa;

/** Original thresholds/multipliers, applied once after a complete search cycle. */
public final class CycleVisionControl {
    private int stalled;
    public double next(double radius,double startCost,double finalCost){
        if(finalCost<startCost*.999){stalled=0;return Math.max(4,radius*.85);}
        if(++stalled>=3){stalled=0;return Math.min(12,radius*1.5);}
        return radius;
    }
}
