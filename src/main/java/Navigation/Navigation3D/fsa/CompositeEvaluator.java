package Navigation.Navigation3D.fsa;

import Navigation.Navigation3D.Normalization.FuelNormalizer;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.Normalization.TimeNormalizer;
import Navigation.model.GridPoint;

import java.util.List;

/**
 * COMPOSITE 目标是归一化率沿航程的距离积分。
 * 每条边的归一化项均为无量纲单位距离率，乘以边长后再累加，
 * 因此把同一段直线路径拆成更多边不会只因边数增加而改变代价。
 *
 * 边代价的口径尽量与 NavigationObjective3D.calculateSegmentMetrics 一致：
 *   - safetyTerm = riskWeighted = meanRisk * distNm；
 *   - 开启归一化时，time/fuel 的归一化率乘以 distNm；关闭时使用可加的原始 time/fuel。
 * 将连续 bearing 映射到最近 8 邻域方向做归一化查表。
 */
public final class CompositeEvaluator implements ObjectiveEvaluator {

    private final FsaContext ctx;
    private final ObjectiveWeights weights;
    private final boolean useShipClassCorr;
    private final boolean useTimeNorm;
    private final boolean useFuelNorm;
    private final boolean useSafetyNorm;
    private final EdgeSampler.NeedComponents need = EdgeSampler.NeedComponents.all();

    public CompositeEvaluator(FsaContext ctx, ObjectiveWeights weights) {
        this.ctx = ctx;
        if (weights == null) {
            throw new IllegalArgumentException("COMPOSITE 需要 ObjectiveWeights");
        }
        this.weights = weights.normalized();
        this.useTimeNorm = ctx.config.enableCompositeNormalization && ctx.config.enableCompositeTimeNormalization
                && ctx.timeNormalizer != null;
        this.useFuelNorm = ctx.config.enableCompositeNormalization && ctx.config.enableCompositeFuelNormalization
                && ctx.fuelNormalizer != null;
        this.useSafetyNorm = ctx.config.enableCompositeNormalization && ctx.config.enableCompositeSafetyNormalization;
        this.useShipClassCorr = ctx.config.enableCompositeShipClassCorrection;
    }

    public ObjectiveWeights weights() {
        return weights;
    }

    @Override
    public Mode mode() { return Mode.COMPOSITE; }

    @Override
    public EdgeSampler.EdgeResult sampleEdge(int fromI, int fromJ, int candI, int candJ, double startElapsedHours) {
        return EdgeSampler.sample(ctx, fromI, fromJ, candI, candJ, startElapsedHours, need, useShipClassCorr);
    }

    @Override
    public double edgeCost(EdgeSampler.EdgeResult edge, int fromI, int fromJ, int candI, int candJ, int dayAtEdge) {
        if (edge == null || !edge.valid) return Double.POSITIVE_INFINITY;
        double distanceNm = edge.distanceNm;
        if (distanceNm <= FsaContext.EPSILON) return Double.POSITIVE_INFINITY;

        double bearing = ctx.bearingOf(fromI, fromJ, candI, candJ);
        int dirIdx = FsaContext.nearestBearingIndex(bearing);

        // safetyWeighted 是 Σ safetyIdx · subDist，riskWeighted 是 Σ (1-safetyIdx) · subDist；
        // 取平均 risk（无量纲）
        double avgRisk = edge.riskWeighted / Math.max(distanceNm, FsaContext.EPSILON);
        // 这是按距离积分的风险量，而不是“每条边一次”的平均风险。
        double safetyTerm = avgRisk * distanceNm;

        double timeTerm;
        if (useTimeNorm) {
            TimeNormalizer tn = ctx.timeNormalizer;
            timeTerm = tn.normalize(edge.timeHours, distanceNm, dirIdx) * distanceNm;
        } else {
            timeTerm = edge.timeHours;
        }

        double fuelTerm;
        if (useFuelNorm) {
            FuelNormalizer fn = ctx.fuelNormalizer;
            fuelTerm = fn.normalize(edge.fuelEnergyJ, distanceNm, dirIdx) * distanceNm;
        } else {
            fuelTerm = edge.fuelEnergyJ;
        }

        return weights.safety * safetyTerm
                + weights.time * timeTerm
                + weights.fuel * fuelTerm;
    }

    @Override
    public double heuristic(int candI, int candJ, int goalI, int goalJ, int dayAtCand) {
        double distNm = ctx.gridDistanceNm(candI, candJ, goalI, goalJ);
        double speedKn = Math.max(ctx.ship.shipSpeed, 0.1);
        double estHours = distNm / speedKn;
        // 启发式粗估：只用距离/速度驱动；目标越近越好，取 1/est
        return 1.0 / Math.max(estHours, FsaContext.EPSILON);
    }

    @Override
    public double pathFinalCost(List<GridPoint> path) {
        return FullRouteAudit211.evaluate(ctx,path,()->pathFinalCostOriginal211(path));
    }

    private double pathFinalCostOriginal211(List<GridPoint> path) {
        if (path == null || path.size() < 2) return 0.0;
        double elapsed = 0.0;
        double total = 0.0;
        for (int k = 0; k < path.size() - 1; k++) {
            GridPoint a = path.get(k);
            GridPoint b = path.get(k + 1);
            EdgeSampler.EdgeResult e = sampleEdge(a.i, a.j, b.i, b.j, elapsed);
            if (!e.valid) return Double.POSITIVE_INFINITY;
            int dayAtEdge = ctx.clampDay(elapsed);
            total += edgeCost(e, a.i, a.j, b.i, b.j, dayAtEdge);
            elapsed = e.endElapsedHours;
        }
        return ctx.meetsArrivalDeadline(elapsed) ? total : Double.POSITIVE_INFINITY;
    }
}
