package Navigation.Navigation3D.fsa;
/** A proved noncompetitive candidate; this carries no objective or physical hazard label. */
public final class CandidatePruned220 extends RuntimeException {
    public final double bound,incumbent;
    public CandidatePruned220(double bound,double incumbent){super("FIXED_ROUTE_NONNEGATIVE_BOUND220");this.bound=bound;this.incumbent=incumbent;}
}
