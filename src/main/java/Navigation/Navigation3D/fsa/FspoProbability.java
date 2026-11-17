package Navigation.Navigation3D.fsa;

/** Optional refinements of the existing multi-cue successor formula; no extra model queries. */
public final class FspoProbability {
    public enum Mode { LEGACY, UNCLIPPED, LOCAL_PROGRESS }
    private FspoProbability() {}

    public static double legacyWeight(double self,double social,double distance,double clearance,double penalty) {
        return Math.max(.001,Math.max(self,.01)*Math.max(social,.01)
                *Math.max(clearance/(1+distance),.01)*penalty);
    }
    public static double logWeight(Mode mode,double self,double social,double currentDistance,
                                  double nextDistance,double edgeDistance,double cellDiagonal,
                                  double clearance,double penalty) {
        if(mode==Mode.LEGACY)throw new IllegalArgumentException("Legacy uses its original arithmetic.");
        positive(self);positive(social);positive(clearance);positive(penalty);positive(edgeDistance);positive(cellDiagonal);
        if(!Double.isFinite(currentDistance)||currentDistance<0||!Double.isFinite(nextDistance)||nextDistance<0)
            throw new IllegalArgumentException("Finite nonnegative goal distances required");
        // The diagnostic mode changes only clipping/normalization of the original inverse distance.
        double goal=mode==Mode.UNCLIPPED?-Math.log1p(nextDistance):
                Math.max(-1,Math.min(1,(currentDistance-nextDistance)/Math.max(edgeDistance,cellDiagonal)));
        return Math.log(self)+Math.log(social)+goal+Math.log(clearance)+Math.log(penalty);
    }
    private static void positive(double x){if(!(x>0)||!Double.isFinite(x))throw new IllegalArgumentException("Finite positive cue required");}
    public static double[] normalize(double[] logs){
        if(logs.length==0)throw new IllegalArgumentException("No candidates");
        double max=Double.NEGATIVE_INFINITY;
        for(double x:logs){if(!Double.isFinite(x))throw new IllegalArgumentException("Nonfinite log weight");max=Math.max(max,x);}
        double[] p=new double[logs.length];double sum=0;
        for(int k=0;k<p.length;k++){p[k]=Math.exp(logs[k]-max);sum+=p[k];}
        for(int k=0;k<p.length;k++)p[k]/=sum;
        return p;
    }
}
