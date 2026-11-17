package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;

import java.util.List;

/**
 * SAFETY_EXPOSURE 目标：累计风险暴露（沿航程对归一化风险密度积分）。
 *
 * 口径：
 *   edgeCost      = riskWeighted = Σ(risk·subDistance)（风险·海里）
 *   pathFinalCost = Σ riskWeighted（累计风险暴露，越小越好）
 *   SAFETY_DENSITY = Σ(risk·d)/Σd 是单独的路线级目标，不由本类优化。
 *   报告安全指数 S = 1 − SAFETY_DENSITY；它是描述性安全分数。
 *
 * 路径级平均风险虽然与分段数无关，但会因加入低风险绕行而被稀释；只优化平均值会让长回路
 * 获得虚假的安全收益。累计积分消除了该分母效应，并与 Dijkstra 的可加边代价统一。
 *
 * 其中 risk = 1 - safetyIndex，通过 EdgeSampler 在每 0.5 格子段上按连续 bearing 现算，
 * 不依赖 safetyCache，也不被鱼群视野大小影响。
 */
public final class SafetyEvaluator implements ObjectiveEvaluator {

    private final FsaContext ctx;
    private final EdgeSampler.NeedComponents need = new EdgeSampler.NeedComponents(true, false, true);
    private final boolean useShipClassCorr;

    public SafetyEvaluator(FsaContext ctx, boolean useShipClassCorr) {
        this.ctx = ctx;
        this.useShipClassCorr = useShipClassCorr;
    }

    @Override
    public Mode mode() { return Mode.SAFETY_EXPOSURE; }

    @Override
    public EdgeSampler.EdgeResult sampleEdge(int fromI, int fromJ, int candI, int candJ, double startElapsedHours) {
        return EdgeSampler.sample(ctx, fromI, fromJ, candI, candJ, startElapsedHours, need, useShipClassCorr);
    }

    @Override
    public double edgeCost(EdgeSampler.EdgeResult edge, int fromI, int fromJ, int candI, int candJ, int dayAtEdge) {
        if (edge == null || !edge.valid) return Double.POSITIVE_INFINITY;
        if (!Double.isFinite(edge.riskWeighted) || edge.riskWeighted < 0.0) {
            return Double.POSITIVE_INFINITY;
        }
        return edge.riskWeighted;
    }

    @Override
    public double heuristic(int candI, int candJ, int goalI, int goalJ, int dayAtCand) {
        // SAFETY 主要由步数主导：剩余步数越少越好。
        int dy = Math.abs(goalI - candI);
        int dx = Math.abs(goalJ - candJ);
        int steps = Math.max(dy, dx); // Chebyshev 距离（8 邻域最少步数）
        return 1.0 / Math.max(steps, 1);
    }

    @Override
    public double pathFinalCost(List<GridPoint> path) {
        if (path == null || path.size() < 2) return 0.0;
        double elapsed = 0.0;
        double sumRiskDistance = 0.0;
        for (int k = 0; k < path.size() - 1; k++) {
            GridPoint a = path.get(k);
            GridPoint b = path.get(k + 1);
            EdgeSampler.EdgeResult e = sampleEdge(a.i, a.j, b.i, b.j, elapsed);
            if (!e.valid) return Double.POSITIVE_INFINITY;
            if (!Double.isFinite(e.riskWeighted)) return Double.POSITIVE_INFINITY;
            sumRiskDistance += e.riskWeighted;
            elapsed = e.endElapsedHours;
        }
        return ctx.meetsArrivalDeadline(elapsed) ? sumRiskDistance : Double.POSITIVE_INFINITY;
    }
}
