package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;

import java.util.List;

/**
 * FUEL 目标：累计油耗能量（J）。启发式 = 1 / 粗估距离。
 */
public final class FuelEvaluator implements ObjectiveEvaluator {

    private final FsaContext ctx;
    private final EdgeSampler.NeedComponents need = new EdgeSampler.NeedComponents(true, true, false);
    private final boolean useShipClassCorr;

    public FuelEvaluator(FsaContext ctx, boolean useShipClassCorr) {
        this.ctx = ctx;
        this.useShipClassCorr = useShipClassCorr;
    }

    @Override
    public Mode mode() { return Mode.FUEL; }

    @Override
    public EdgeSampler.EdgeResult sampleEdge(int fromI, int fromJ, int candI, int candJ, double startElapsedHours) {
        return EdgeSampler.sample(ctx, fromI, fromJ, candI, candJ, startElapsedHours, need, useShipClassCorr);
    }

    @Override
    public double edgeCost(EdgeSampler.EdgeResult edge, int fromI, int fromJ, int candI, int candJ, int dayAtEdge) {
        if (edge == null || !edge.valid) return Double.POSITIVE_INFINITY;
        double j = edge.fuelEnergyJ;
        if (!Double.isFinite(j) || j < 0) {
            // 保底兜底：按距离×固定功率估（与 NavigationObjective3D FUEL fallback 一致量级）
            return Math.max(edge.distanceNm, FsaContext.EPSILON) * 1_000_000.0;
        }
        return j;
    }

    @Override
    public double heuristic(int candI, int candJ, int goalI, int goalJ, int dayAtCand) {
        double distNm = ctx.gridDistanceNm(candI, candJ, goalI, goalJ);
        // 取 1 / (distNm + eps)：距离越近越吸引
        return 1.0 / Math.max(distNm, FsaContext.EPSILON);
    }

    @Override
    public double pathFinalCost(List<GridPoint> path) {
        RawPathCost raw = evaluateRawPathCost(path);
        return ctx.meetsArrivalDeadline(raw.elapsedHours)
                ? raw.cost : Double.POSITIVE_INFINITY;
    }

    /** Deadline-neutral fuel for reporting; edge/data validity is still enforced. */
    public double pathRawCost(List<GridPoint> path) {
        return evaluateRawPathCost(path).cost;
    }

    private RawPathCost evaluateRawPathCost(List<GridPoint> path) {
        if (path == null || path.size() < 2) return new RawPathCost(0.0, 0.0);
        double elapsed = 0.0;
        double total = 0.0;
        for (int k = 0; k < path.size() - 1; k++) {
            GridPoint a = path.get(k);
            GridPoint b = path.get(k + 1);
            EdgeSampler.EdgeResult e = sampleEdge(a.i, a.j, b.i, b.j, elapsed);
            if (!e.valid) return new RawPathCost(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
            total += edgeCost(e, a.i, a.j, b.i, b.j, ctx.clampDay(elapsed));
            elapsed = e.endElapsedHours;
        }
        return new RawPathCost(total, elapsed);
    }

    private static final class RawPathCost {
        final double cost;
        final double elapsedHours;

        RawPathCost(double cost, double elapsedHours) {
            this.cost = cost;
            this.elapsedHours = elapsedHours;
        }
    }
}
