package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;

import java.util.List;
import java.util.Arrays;

/**
 * TIME 目标：累计航行小时数。启发式 = 1 / 粗估剩余时间。
 */
public final class TimeEvaluator implements ObjectiveEvaluator {

    private final FsaContext ctx;
    private final EdgeSampler.NeedComponents need = EdgeSampler.NeedComponents.timeOnly();
    private final boolean useShipClassCorr;

    public TimeEvaluator(FsaContext ctx, boolean useShipClassCorr) {
        this.ctx = ctx;
        this.useShipClassCorr = useShipClassCorr;
    }

    @Override
    public Mode mode() { return Mode.TIME; }

    @Override
    public EdgeSampler.EdgeResult sampleEdge(int fromI, int fromJ, int candI, int candJ, double startElapsedHours) {
        return EdgeSampler.sample(ctx, fromI, fromJ, candI, candJ, startElapsedHours, need, useShipClassCorr);
    }

    @Override
    public double edgeCost(EdgeSampler.EdgeResult edge, int fromI, int fromJ, int candI, int candJ, int dayAtEdge) {
        if (edge == null || !edge.valid) return Double.POSITIVE_INFINITY;
        return edge.timeHours;
    }

    @Override
    public double heuristic(int candI, int candJ, int goalI, int goalJ, int dayAtCand) {
        double distNm = ctx.gridDistanceNm(candI, candJ, goalI, goalJ);
        double speedKn = Math.max(ctx.ship.shipSpeed, 0.1);
        double estHours = distNm / speedKn;
        return 1.0 / Math.max(estHours, FsaContext.EPSILON);
    }

    @Override
    public double pathFinalCost(List<GridPoint> path) {
        RawPathCost raw = evaluateRawPathCost(path);
        return ctx.meetsArrivalDeadline(raw.elapsedHours)
                ? raw.cost : Double.POSITIVE_INFINITY;
    }

    /** Deadline-neutral time for reporting; edge/data validity is still enforced. */
    public double pathRawCost(List<GridPoint> path) {
        return evaluateRawPathCost(path).cost;
    }

    private RawPathCost evaluateRawPathCost(List<GridPoint> path) {
        return evaluateRawPathCost(path, null, Double.POSITIVE_INFINITY);
    }

    /** Opt-in observation of the same TIME sampling loop; no safety-model adaptation. */
    public Trace tracePath(List<GridPoint> path, double maxDistanceNm) {
        Trace trace = new Trace(path);
        RawPathCost raw = evaluateRawPathCost(path, trace, maxDistanceNm);
        if ("VALID".equals(trace.status) && !ctx.meetsArrivalDeadline(raw.elapsedHours))
            trace.status = "DEADLINE";
        trace.cost = "VALID".equals(trace.status) ? raw.cost : Double.POSITIVE_INFINITY;
        return trace;
    }

    public static final class Trace {
        public final List<GridPoint> path;
        public final double[] arrival;
        public final double[] duration;
        public int firstInvalidEdge = -1;
        public String status = "VALID";
        public double cost = Double.POSITIVE_INFINITY;
        public Trace(List<GridPoint> path) {
            this.path = path;
            arrival = new double[path == null ? 0 : path.size()];
            duration = new double[Math.max(0, arrival.length - 1)];
            Arrays.fill(arrival, Double.NaN); Arrays.fill(duration, Double.NaN);
            if (arrival.length > 0) arrival[0] = 0;
        }
        public boolean valid() { return "VALID".equals(status) && Double.isFinite(cost); }
    }

    private RawPathCost evaluateRawPathCost(List<GridPoint> path, Trace trace, double maxDistanceNm) {
        if (trace != null && (path == null || path.size() < 2)) trace.status = "GEOMETRY";
        if (path == null || path.size() < 2) return new RawPathCost(0.0, 0.0);
        double elapsed = 0.0;
        double total = 0.0;
        double distance = 0.0;
        java.util.Set<GridPoint> visited = trace == null ? null : new java.util.HashSet<>();
        if (visited != null) visited.add(path.get(0));
        for (int k = 0; k < path.size() - 1; k++) {
            GridPoint a = path.get(k);
            GridPoint b = path.get(k + 1);
            if (trace != null) {
                if (ctx.searchWorkLedger != null && !ctx.searchWorkLedger.evaluationAllowed()) {
                    trace.status = "ABORTED";
                    return new RawPathCost(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
                }
                if (!ctx.routeGraph.isValidEdge(a, b) || !visited.add(b)) {
                    trace.status = "GEOMETRY"; trace.firstInvalidEdge = k;
                    return new RawPathCost(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
                }
                distance += ctx.routeGraph.edgeDistanceNm(a, b);
                if (distance > maxDistanceNm + FsaContext.EPSILON) {
                    trace.status = "DISTANCE";
                    return new RawPathCost(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
                }
            }
            EdgeSampler.EdgeResult e = sampleEdge(a.i, a.j, b.i, b.j, elapsed);
            if (!e.valid) {
                if (trace != null) {
                    trace.status = ctx.searchWorkLedger != null && !ctx.searchWorkLedger.evaluationAllowed()
                            ? "ABORTED" : "EDGE_INVALID";
                    if (!"ABORTED".equals(trace.status)) trace.firstInvalidEdge = k;
                }
                return new RawPathCost(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
            }
            total += e.timeHours;
            elapsed = e.endElapsedHours;
            if (trace != null) { trace.duration[k] = e.timeHours; trace.arrival[k+1] = elapsed; }
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
