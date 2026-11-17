package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;

import java.util.List;

/**
 * SAFETY_DENSITY objective: minimize the route-level risk density R/L.
 *
 * R = Σ riskWeighted and L = Σ distanceNm are accumulated over the complete
 * departure-relative route. edgeCost is only a local construction cue; it is
 * never summed to define the objective. In particular, this class does not use
 * either an unweighted mean of edge ratios or a sum of edge ratios.
 */
public final class SafetyDensityEvaluator implements ObjectiveEvaluator {

    private final FsaContext ctx;
    private final EdgeSampler.NeedComponents need = new EdgeSampler.NeedComponents(true, false, true);
    private final boolean useShipClassCorr;

    public SafetyDensityEvaluator(FsaContext ctx, boolean useShipClassCorr) {
        this.ctx = ctx;
        this.useShipClassCorr = useShipClassCorr;
    }

    @Override
    public Mode mode() { return Mode.SAFETY_DENSITY; }

    @Override
    public EdgeSampler.EdgeResult sampleEdge(int fromI, int fromJ, int candI, int candJ,
                                             double startElapsedHours) {
        return EdgeSampler.sample(ctx, fromI, fromJ, candI, candJ,
                startElapsedHours, need, useShipClassCorr);
    }

    @Override
    public double edgeCost(EdgeSampler.EdgeResult edge, int fromI, int fromJ,
                           int candI, int candJ, int dayAtEdge) {
        if (edge == null || !edge.valid || !Double.isFinite(edge.riskWeighted)
                || edge.riskWeighted < 0.0 || !Double.isFinite(edge.distanceNm)
                || edge.distanceNm <= FsaContext.EPSILON) {
            return Double.POSITIVE_INFINITY;
        }
        // Local risk-exposure contribution for edge-level callers; the route objective is R/L below.
        return edge.riskWeighted;
    }

    @Override
    public double heuristic(int candI, int candJ, int goalI, int goalJ, int dayAtCand) {
        int steps = Math.max(Math.abs(goalI - candI), Math.abs(goalJ - candJ));
        return 1.0 / Math.max(steps, 1);
    }

    @Override
    public double pathFinalCost(List<GridPoint> path) {
        if (path == null || path.size() < 2) return Double.POSITIVE_INFINITY;
        double elapsed = 0.0;
        double totalRiskDistance = 0.0;
        double totalDistance = 0.0;
        for (int k = 0; k < path.size() - 1; k++) {
            GridPoint a = path.get(k);
            GridPoint b = path.get(k + 1);
            EdgeSampler.EdgeResult edge = sampleEdge(a.i, a.j, b.i, b.j, elapsed);
            if (edge == null || !edge.valid || !Double.isFinite(edge.riskWeighted)
                    || !Double.isFinite(edge.distanceNm) || edge.distanceNm <= FsaContext.EPSILON) {
                return Double.POSITIVE_INFINITY;
            }
            totalRiskDistance += edge.riskWeighted;
            totalDistance += edge.distanceNm;
            elapsed = edge.endElapsedHours;
        }
        if (!Double.isFinite(totalRiskDistance) || !Double.isFinite(totalDistance)
                || totalDistance <= FsaContext.EPSILON || !ctx.meetsArrivalDeadline(elapsed)) {
            return Double.POSITIVE_INFINITY;
        }
        return totalRiskDistance / totalDistance;
    }
}
