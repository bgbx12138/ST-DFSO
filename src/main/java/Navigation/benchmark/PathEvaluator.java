package Navigation.benchmark;

import Navigation.model.GridPoint;
import Navigation.model.GridRouteGraph;
import Navigation.object.*;
import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.Normalization.Normalizers;
import Navigation.Navigation3D.Normalization.Normalizers.EnvironmentSampleProvider;
import Navigation.Navigation3D.fsa.*;
import Navigation.preprocess.object.*;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.time.LocalDate;
import java.time.format.DateTimeFormatter;
import java.util.*;

public class PathEvaluator implements AutoCloseable {

    private static final DateTimeFormatter FMT = DateTimeFormatter.ofPattern("yyyyMMdd");
    private static final ObjectMapper MAPPER = new ObjectMapper();

    private final FsaContext ctxNoNorm, ctxComposite;
    private final SearchWorkLedger searchWorkLedger;
    private final int ny, nx;
    private final NavigationConfig cfg;
    private final double dLat, dLon;
    private final GridRouteGraph routeGraph;
    private final GridPoint expectedStart, expectedEnd;
    private double maxRouteDistanceNm = Double.POSITIVE_INFINITY;

    public static final class RouteShapeMetrics {
        public final int pointCount;
        public final int distinctPointCount;
        public final int repeatedVisitCount;
        public final int revisitsAfterWeatherDayChange;
        public final int immediateBacktrackCount;
        public final int exactReverseTurnCount;
        public final double repeatedLoopDistanceNm;

        private RouteShapeMetrics(int pointCount, int distinctPointCount, int repeatedVisitCount,
                                  int revisitsAfterWeatherDayChange, int immediateBacktrackCount,
                                  int exactReverseTurnCount, double repeatedLoopDistanceNm) {
            this.pointCount = pointCount;
            this.distinctPointCount = distinctPointCount;
            this.repeatedVisitCount = repeatedVisitCount;
            this.revisitsAfterWeatherDayChange = revisitsAfterWeatherDayChange;
            this.immediateBacktrackCount = immediateBacktrackCount;
            this.exactReverseTurnCount = exactReverseTurnCount;
            this.repeatedLoopDistanceNm = repeatedLoopDistanceNm;
        }
    }

    private static final class PreviousVisit {
        final double distanceNm;
        final int weatherDay;

        PreviousVisit(double distanceNm, int weatherDay) {
            this.distanceNm = distanceNm;
            this.weatherDay = weatherDay;
        }
    }

    public PathEvaluator(NavigationConfig config) throws IOException {
        this(config, null);
    }

    public PathEvaluator(NavigationConfig config, SearchWorkLedger searchWorkLedger) throws IOException {
        this.cfg = config;
        config.resolveArrivalDeadlineHours();
        this.searchWorkLedger = searchWorkLedger;
        Path dataDir = Paths.get(config.dataFolder);
        if (!Files.isDirectory(dataDir))
            throw new IOException("data directory not found: " + dataDir);
        LocalDate base = LocalDate.parse(config.navigationDate, FMT);
        int numDays = countDays(dataDir, base);
        if (numDays == 0) throw new IOException("no env data found");
        List<EnvironmentFrame> frames = new ArrayList<>(numDays);
        for (int d = 0; d < numDays; d++)
            frames.add(loadFrame(dataDir, base.plusDays(d)));
        String obName = (config.obstacleFileName == null || config.obstacleFileName.isBlank())
                ? "obstacle_map.json" : config.obstacleFileName;
        ObstacleOutput obOut = MAPPER.readValue(dataDir.resolve(obName).toFile(), ObstacleOutput.class);
        boolean[][] obstacleMap = obOut.obstacleMap;
        double deltaLat = (config.area.latMax - config.area.latMin) / Math.max(1, config.grid.ny - 1);
        double deltaLon = (config.area.lonMax - config.area.lonMin) / Math.max(1, config.grid.nx - 1);
        double latCtr = (config.area.latMin + config.area.latMax) / 2.0;
        double dLatNm = deltaLat * 60.0;
        double dLonNm = deltaLon * 60.0 * Math.cos(Math.toRadians(latCtr));
        this.ny = config.grid.ny; this.nx = config.grid.nx;
        this.dLat = deltaLat; this.dLon = deltaLon;
        this.expectedStart = new GridPoint(config.grid.startY - 1, config.grid.startX - 1);
        this.expectedEnd = new GridPoint(config.grid.endY - 1, config.grid.endX - 1);
        boolean[][] obsBuf = copyBoolMap(obstacleMap);
        // Must match HybridFSA3D.buildContext() so SafetyEvaluator computes the same
        // cumulative risk for a given path — otherwise Elite Archive guarantee breaks.
        double depOff = config.resolveDepartureOffsetHours();
        Normalizers.Bundle bundle = Normalizers.build(config, config.shipParameters,
                FsaContext.DIRECTIONS, FsaContext.BEARINGS,
                new EnvProvider(frames, obstacleMap, numDays));
        this.ctxNoNorm = new FsaContext(config, config.shipParameters, ny, nx, numDays, depOff,
                Collections.unmodifiableList(new ArrayList<>(frames)), obstacleMap, obsBuf,
                dLatNm, dLonNm, deltaLat, deltaLon, null, null, searchWorkLedger);
        this.ctxComposite = new FsaContext(config, config.shipParameters, ny, nx, numDays, depOff,
                Collections.unmodifiableList(new ArrayList<>(frames)), obstacleMap, obsBuf,
                dLatNm, dLonNm, deltaLat, deltaLon,
                bundle.timeNormalizer(), bundle.fuelNormalizer(), searchWorkLedger);
        this.routeGraph = new GridRouteGraph(ny, nx, dLatNm, dLonNm,
                GridRouteGraph.DEFAULT_MAX_EDGE_SPAN_CELLS, this::isObstacle);
    }

    public double evaluate(List<GridPoint> path, HybridFSA3D.ObjectiveMode mode,
                           ObjectiveWeights weights) {
        return evaluateStrict(path, mode, weights, routeGraph, maxRouteDistanceNm);
    }

    /**
     * Score the exact supplied vertex sequence. Invalid routes receive the finite penalty;
     * this method never clamps, repairs, simplifies, deduplicates, or mutates the route.
     */
    public double evaluateStrict(List<GridPoint> path, HybridFSA3D.ObjectiveMode mode,
                                 ObjectiveWeights weights) {
        return evaluateStrict(path, mode, weights, routeGraph, maxRouteDistanceNm);
    }

    public double evaluateStrict(List<GridPoint> path, HybridFSA3D.ObjectiveMode mode,
                                 ObjectiveWeights weights, GridRouteGraph graph,
                                 double maxRouteDistanceNm) {
        if (searchWorkLedger != null) searchWorkLedger.recordRouteObjectiveEvaluation();
        if (searchWorkLedger != null && searchWorkLedger.shouldStopSearch()) {
            return finitePenalty(path);
        }
        if (graph == null || path == null || path.size() < 2
                || !graph.validate(path, expectedStart, expectedEnd, maxRouteDistanceNm).valid) {
            return finitePenalty(path);
        }
        double result;
        ObjectiveEvaluator eval;
        switch (mode) {
            case TIME:   eval = new TimeEvaluator(ctxNoNorm, true); break;
            case SAFETY_EXPOSURE:
            case SAFETY: // legacy in-process alias; new benchmark outputs use SAFETY_EXPOSURE
                // 累计风险暴露 R = Σ(risk·ds)，与可加安全搜索目标同口径。
                return computeCumulativeRiskExposure(path);
            case SAFETY_DENSITY:
                // 路线级 R/L。分子、分母分别累计后求比，不累加逐边平均或逐边比值。
                return finitePenalty(evaluateRiskSums(path, graph, maxRouteDistanceNm, true)[2], path);
            case FUEL:   eval = new FuelEvaluator(ctxNoNorm, true); break;
            case COMPOSITE:
                if (weights == null) weights = new ObjectiveWeights(0.5, 0.3, 0.2);
                return distNormalizedComposite(path, weights);
            default: return finitePenalty(path);
        }
        result = eval.pathFinalCost(path);
        return finitePenalty(result, path);
    }

    /** SAFETY_EXPOSURE 优化目标：∫risk ds = Σ(riskWeighted)。 */
    private double computeCumulativeRiskExposure(List<GridPoint> path) {
        double elapsed = 0.0;
        double sumRiskDistance = 0.0;
        EdgeSampler.NeedComponents needSafety = new EdgeSampler.NeedComponents(true, false, true);
        for (int k = 0; k < path.size() - 1; k++) {
            GridPoint a = path.get(k);
            GridPoint b = path.get(k + 1);
            EdgeSampler.EdgeResult e = EdgeSampler.sample(ctxNoNorm, a.i, a.j, b.i, b.j,
                    elapsed, needSafety, true);
            if (!e.valid || !Double.isFinite(e.riskWeighted)) return finitePenalty(path);
            sumRiskDistance += e.riskWeighted;
            elapsed = e.endElapsedHours;
        }
        return ctxNoNorm.meetsArrivalDeadline(elapsed)
                ? sumRiskDistance : finitePenalty(path);
    }

    /** 综合代价 = Σ(edgeCost)，与算法内部 CompositeEvaluator.pathFinalCost 口径一致。 */
    private double distNormalizedComposite(List<GridPoint> path, ObjectiveWeights weights) {
        CompositeEvaluator eval = new CompositeEvaluator(ctxComposite, weights.normalized());
        double elapsed = 0.0;
        double total = 0.0;
        int numEdges = path.size() - 1;
        for (int k = 0; k < numEdges; k++) {
            GridPoint a = path.get(k);
            GridPoint b = path.get(k + 1);
            EdgeSampler.EdgeResult e = EdgeSampler.sample(ctxComposite, a.i, a.j, b.i, b.j,
                    elapsed, new EdgeSampler.NeedComponents(true, true, true), true);
            if (!e.valid) return finitePenalty(path);
            int dayAtEdge = ctxComposite.clampDay(elapsed);
            total += eval.edgeCost(e, a.i, a.j, b.i, b.j, dayAtEdge);
            elapsed = e.endElapsedHours;
        }
        return Double.isFinite(total) && ctxComposite.meetsArrivalDeadline(elapsed)
                ? total : finitePenalty(path);
    }

    /** 原始累计风险暴露 Σ(risk·d)，与 SAFETY_EXPOSURE 搜索目标的代价语义一致。 */
    public double evaluateRawCumRisk(List<GridPoint> path) {
        if (path == null || path.size() < 2) return finitePenalty(null);
        double[] sums = evaluateRawRiskSums(path);
        if (!Double.isFinite(sums[0])) return finitePenalty(null);
        return sums[0];
    }

    /** 边积分风险累加量（只读报告指标，不受到达期限筛除）。
     *  返回 {sumRW, sumD, meanRiskDensity}；任一边无效返回 NaN 三元组。
     *  SAFETY_DENSITY 搜索仍通过 evaluateStrict 显式施加到达期限。 */
    public double[] evaluateRawRiskSums(List<GridPoint> path) {
        return evaluateRawRiskSums(path, routeGraph, Double.POSITIVE_INFINITY);
    }

    public double[] evaluateRawRiskSums(List<GridPoint> path, GridRouteGraph graph,
                                        double maxRouteDistanceNm) {
        return evaluateRiskSums(path, graph, maxRouteDistanceNm, false);
    }

    private double[] evaluateRiskSums(List<GridPoint> path, GridRouteGraph graph,
                                      double maxRouteDistanceNm, boolean requireDeadline) {
        if (path == null || path.size() < 2) return new double[]{Double.NaN, Double.NaN, Double.NaN};
        if (graph == null || !graph.validate(path, expectedStart, expectedEnd, maxRouteDistanceNm).valid)
            return new double[]{Double.NaN, Double.NaN, Double.NaN};
        double elapsed = 0.0, sumRW = 0.0, sumD = 0.0;
        EdgeSampler.NeedComponents needSafety = new EdgeSampler.NeedComponents(true, false, true);
        for (int k = 0; k < path.size() - 1; k++) {
            GridPoint a = path.get(k);
            GridPoint b = path.get(k + 1);
            EdgeSampler.EdgeResult e = EdgeSampler.sample(ctxNoNorm, a.i, a.j, b.i, b.j,
                    elapsed, needSafety, true);
            if (!e.valid) return new double[]{Double.NaN, Double.NaN, Double.NaN};
            sumRW += e.riskWeighted;
            sumD += e.distanceNm;
            elapsed = e.endElapsedHours;
        }
        if (requireDeadline && !ctxNoNorm.meetsArrivalDeadline(elapsed))
            return new double[]{Double.NaN, Double.NaN, Double.NaN};
        double meanRiskDensity = sumD > FsaContext.EPSILON ? sumRW / sumD : Double.NaN;
        return new double[]{sumRW, sumD, meanRiskDensity};
    }

    /** 报告用原始指标（不加有限惩罚）：[timeHours, fuelEnergyJ, distNm, cumRisk, safetyScore]。
     *  Infinity 原样返回，不包装为 1e18，便于诊断路径有效性。 */
    public double[] evaluateRawMetrics(List<GridPoint> path) {
        return evaluateRawMetrics(path, routeGraph, maxRouteDistanceNm);
    }

    public double[] evaluateRawMetrics(List<GridPoint> path, GridRouteGraph graph,
                                       double maxRouteDistanceNm) {
        if (path == null || path.size() < 2
                || graph == null || !graph.validate(path, expectedStart, expectedEnd, maxRouteDistanceNm).valid)
            return new double[]{Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY,
                    Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};
        TimeEvaluator timeEval = new TimeEvaluator(ctxNoNorm, true);
        double time = timeEval.pathRawCost(path);
        FuelEvaluator fuelEval = new FuelEvaluator(ctxNoNorm, true);
        double fuel = fuelEval.pathRawCost(path);
        // 报告口径固定为 Σ(risk·d)/Σd（与 Table 8 的 S 列同源），与搜索目标函数解耦
        double[] riskSums = evaluateRawRiskSums(path, graph, maxRouteDistanceNm);
        double dist = riskSums[1];
        double cumRisk = riskSums[0];
        double safety;
        if (Double.isFinite(cumRisk) && riskSums[1] > FsaContext.EPSILON) {
            safety = Math.max(0, Math.min(1, 1.0 - riskSums[2]));
        } else {
            safety = 0.0;
        }
        return new double[]{time, fuel, dist, cumRisk, safety};
    }

    public int getNy() { return ny; } public int getNx() { return nx; }
    public boolean isObstacle(int i, int j) { return ctxNoNorm.blocked(i, j); }
    public GridRouteGraph getRouteGraph() { return routeGraph; }
    public double getMaxRouteDistanceNm() { return maxRouteDistanceNm; }
    public Double getArrivalDeadlineHours() { return cfg.arrivalDeadlineHours; }
    public double configureDistanceBudgetFromSeed(List<GridPoint> seedPath, double kappa) {
        if (!Double.isFinite(kappa) || kappa < 1.0) {
            throw new IllegalArgumentException("detour kappa must be finite and at least 1.0");
        }
        GridRouteGraph.Validation seed = routeGraph.validate(
                seedPath, expectedStart, expectedEnd, Double.POSITIVE_INFINITY);
        if (!seed.valid || !Double.isFinite(seed.distanceNm) || seed.distanceNm <= 0.0) {
            throw new IllegalArgumentException("common seed is invalid for the shared distance budget: "
                    + seed.reason);
        }
        maxRouteDistanceNm = seed.distanceNm * kappa;
        if (!Double.isFinite(maxRouteDistanceNm)) {
            throw new IllegalArgumentException("detour distance budget is not finite");
        }
        return maxRouteDistanceNm;
    }
    public GridRouteGraph createRouteGraph(double maxEdgeSpanCells) {
        return new GridRouteGraph(ny, nx, ctxNoNorm.dLatNm, ctxNoNorm.dLonNm,
                maxEdgeSpanCells, this::isObstacle);
    }
    public GridRouteGraph.Validation validateRoute(List<GridPoint> path, GridPoint expectedStart,
                                                   GridPoint expectedEnd, double maxRouteDistanceNm) {
        return routeGraph.validate(path,
                expectedStart == null ? this.expectedStart : expectedStart,
                expectedEnd == null ? this.expectedEnd : expectedEnd,
                maxRouteDistanceNm);
    }

    /** 供可视化/导出使用：返回与评估所用完全一致的修复后（可航行）几何。 */
    public List<GridPoint> repairForExport(List<GridPoint> path) {
        if (path == null) return null;
        return repairPath(new ArrayList<>(path));
    }

    /**
     * Route-shape diagnostics. A revisit after a weather-day transition may be an implicit wait;
     * same-day loops and immediate reversals are reported separately for interpretation.
     */
    public RouteShapeMetrics analyzeRouteShape(List<GridPoint> path) {
        if (path == null || path.isEmpty()) {
            return new RouteShapeMetrics(0, 0, 0, 0, 0, 0, 0.0);
        }
        for (GridPoint point : path) {
            if (point == null) return new RouteShapeMetrics(path.size(), 0, 0, 0, 0, 0, Double.NaN);
        }
        List<GridPoint> points = new ArrayList<>(path);
        Map<Long, PreviousVisit> previousVisits = new HashMap<>();
        double elapsedHours = 0.0;
        double cumulativeDistanceNm = 0.0;
        double repeatedLoopDistanceNm = 0.0;
        int repeatedVisitCount = 0;
        int revisitsAfterWeatherDayChange = 0;
        int immediateBacktrackCount = 0;
        int exactReverseTurnCount = 0;

        for (int k = 0; k < points.size(); k++) {
            GridPoint current = points.get(k);
            long key = gpKey(current);
            int weatherDay = ctxNoNorm.clampDay(elapsedHours);
            PreviousVisit previous = previousVisits.put(key, new PreviousVisit(cumulativeDistanceNm, weatherDay));
            if (previous == null) {
                // First visit establishes the baseline for later loop detection.
            } else {
                repeatedVisitCount++;
                repeatedLoopDistanceNm += Math.max(0.0, cumulativeDistanceNm - previous.distanceNm);
                if (weatherDay > previous.weatherDay) revisitsAfterWeatherDayChange++;
            }

            if (k + 1 < points.size()) {
                GridPoint next = points.get(k + 1);
                EdgeSampler.EdgeResult edge = EdgeSampler.sample(ctxNoNorm, current.i, current.j,
                        next.i, next.j, elapsedHours,
                        new EdgeSampler.NeedComponents(true, false, false), true);
                if (edge.valid && Double.isFinite(edge.timeHours) && Double.isFinite(edge.distanceNm)) {
                    elapsedHours = edge.endElapsedHours;
                    cumulativeDistanceNm += edge.distanceNm;
                }
            }
        }

        for (int k = 1; k < points.size() - 1; k++) {
            GridPoint before = points.get(k - 1);
            GridPoint pivot = points.get(k);
            GridPoint after = points.get(k + 1);
            int ux = pivot.j - before.j;
            int uy = pivot.i - before.i;
            int vx = after.j - pivot.j;
            int vy = after.i - pivot.i;
            long dot = (long) ux * vx + (long) uy * vy;
            long cross = (long) ux * vy - (long) uy * vx;
            if (dot < 0 && cross == 0) exactReverseTurnCount++;
            if (before.i == after.i && before.j == after.j) immediateBacktrackCount++;
        }

        return new RouteShapeMetrics(points.size(), previousVisits.size(), repeatedVisitCount,
                revisitsAfterWeatherDayChange, immediateBacktrackCount, exactReverseTurnCount,
                repeatedLoopDistanceNm);
    }

    public NavigationResult buildResult(List<GridPoint> path) {
        return buildResultStrict(path, routeGraph, maxRouteDistanceNm);
    }

    public NavigationResult buildResultStrict(List<GridPoint> path, GridRouteGraph graph,
                                              double maxRouteDistanceNm) {
        if (path == null || path.size() < 2) return null;
        if (graph == null || !graph.validate(path, expectedStart, expectedEnd, maxRouteDistanceNm).valid) return null;
        double[] metrics = evaluateRawMetrics(path, graph, maxRouteDistanceNm);
        if (!Arrays.stream(metrics).allMatch(Double::isFinite)) return null;
        // Raw metrics remain available to the run ledger for late routes, but a
        // deadline-infeasible route must not be exported as a strict result.
        if (!ctxNoNorm.meetsArrivalDeadline(metrics[0])) return null;
        NavigationResult r = new NavigationResult();
        r.navigationDate = cfg.navigationDate; r.area = cfg.area; r.grid = cfg.grid;
        r.shipParameters = cfg.shipParameters; r.obstacleFile = cfg.obstacleFileName;
        // Export the exact ordered route that the optimizer returned and the evaluator scored.
        List<RoutePoint> pts = new ArrayList<>();
        double lat0 = cfg.area.latMin, lon0 = cfg.area.lonMin;
        for (int k = 0; k < path.size(); k++) {
            GridPoint g = path.get(k); RoutePoint pt = new RoutePoint();
            pt.index = k + 1; pt.x = g.j + 1; pt.y = g.i + 1;  // 1-based, 与 HybridFSA3D.buildRoutePoints 一致
            pt.lon = Math.round((lon0 + g.j * dLon) * 1_000_000.0) / 1_000_000.0;
            pt.lat = Math.round((lat0 + g.i * dLat) * 1_000_000.0) / 1_000_000.0;
            pts.add(pt);
        }
        r.pathPoints = pts; r.pathPointCount = pts.size();
        // Every reported metric is based on the same unchanged route sequence.
        r.totalTime = metrics[0];
        r.totalFuelEnergyJ = metrics[1];
        // Keep the numerator, denominator and reported index on the exact sampled route metric;
        // the same sums define SAFETY_DENSITY, so exported JSON can reproduce R/L.
        double[] riskSums = evaluateRawRiskSums(path, graph, maxRouteDistanceNm);
        boolean hasRiskMetric = Double.isFinite(riskSums[0]) && Double.isFinite(riskSums[1])
                && riskSums[1] > FsaContext.EPSILON && Double.isFinite(riskSums[2]);
        r.totalDistance = hasRiskMetric ? riskSums[1] : computeDist(path);
        double riskPerNm = hasRiskMetric ? riskSums[2] : Double.NaN;
        r.totalSafetyScore = Double.isFinite(riskPerNm)
                ? Math.max(0.0, Math.min(1.0, 1.0 - riskPerNm)) : 0.0;
        r.totalCumulativeRisk = hasRiskMetric ? riskSums[0] : evaluateRawCumRisk(path);
        r.projectName = "Benchmark"; r.calculationType = "swarm";
        return r;
    }

    @Override public void close() {}

    /** Infinity → 大有限惩罚值：避免 PSO/GWO/ACO 的 NaN 级联和不可逆坍缩
     *  必须大于所有目标的有效代价（FUEL ~26B, TIME ~50, 风险目标通常在 [0,1]，COMPOSITE ~0.01） */
    private double finitePenalty(List<GridPoint> path) {
        return 1e18;
    }
    private double finitePenalty(double val, List<GridPoint> path) {
        return Double.isFinite(val) ? val : finitePenalty(path);
    }

    // ── 路径修复：长边穿越障碍时 BFS 绕行 ──────────────────────────

    /**
     * 遍历路径中相邻航点对，用 Bresenham 检查中间是否穿越障碍。
     * 若穿越，用 BFS 在 8-邻域网格上找最短绕行路径并插入中间格子。
     * 使 PSO/GWO 等稀疏航点算法与 FSA 的逐格评估在同一规则下竞争。
     */
    private List<GridPoint> repairPath(List<GridPoint> path) {
        if (path.size() < 2) return path;
        List<GridPoint> result = new ArrayList<>();
        result.add(path.get(0));
        for (int k = 1; k < path.size(); k++) {
            GridPoint a = result.get(result.size() - 1);
            GridPoint b = path.get(k);
            if (a.i == b.i && a.j == b.j) continue;
            List<int[]> line = bresenhamLine(a.i, a.j, b.i, b.j);
            boolean crosses = false;
            for (int m = 1; m < line.size() - 1; m++) {
                if (ctxNoNorm.blocked(line.get(m)[0], line.get(m)[1])) {
                    crosses = true;
                    break;
                }
            }
            if (crosses) {
                List<GridPoint> detour = bfsRoute(a, b);
                if (detour != null) {
                    for (int m = 1; m < detour.size(); m++) result.add(detour.get(m));
                } else {
                    result.add(b);  // BFS 找不到则保留原边（评估器会返回 Infinity）
                }
            } else {
                result.add(b);
            }
        }
        return result;
    }

    /** Bresenham 直线插值：返回从 (r0,c0) 到 (r1,c1) 的所有整数格点 */
    private static List<int[]> bresenhamLine(int r0, int c0, int r1, int c1) {
        List<int[]> pts = new ArrayList<>();
        int dr = Math.abs(r1 - r0), dc = Math.abs(c1 - c0);
        int sr = r0 < r1 ? 1 : -1, sc = c0 < c1 ? 1 : -1;
        int err = dr - dc, r = r0, c = c0;
        while (true) {
            pts.add(new int[]{r, c});
            if (r == r1 && c == c1) break;
            int e2 = 2 * err;
            if (e2 > -dc) { err -= dc; r += sr; }
            if (e2 < dr)  { err += dr; c += sc; }
        }
        return pts;
    }

    /** BFS 8-邻域最短路：在网格上绕过障碍从 start 到 end，返回格子序列 */
    private List<GridPoint> bfsRoute(GridPoint start, GridPoint end) {
        if (ctxNoNorm.blocked(start.i, start.j) || ctxNoNorm.blocked(end.i, end.j))
            return null;
        if (start.i == end.i && start.j == end.j)
            return Arrays.asList(start);
        Map<Long, Long> parent = new HashMap<>();
        long sk = gpKey(start), ek = gpKey(end);
        parent.put(sk, -1L);
        ArrayDeque<Long> q = new ArrayDeque<>();
        q.add(sk);
        int[] di8 = {-1,-1,-1,0,0,1,1,1};
        int[] dj8 = {-1,0,1,-1,1,-1,0,1};
        while (!q.isEmpty()) {
            long cur = q.poll();
            if (cur == ek) break;
            int ci = (int)(cur >> 32), cj = (int)cur;
            for (int d = 0; d < 8; d++) {
                int ni = ci + di8[d], nj = cj + dj8[d];
                if (ni < 0 || ni >= ny || nj < 0 || nj >= nx) continue;
                if (ctxNoNorm.blocked(ni, nj)) continue;
                long nk = gpKey(ni, nj);
                if (!parent.containsKey(nk)) {
                    parent.put(nk, cur);
                    q.add(nk);
                }
            }
        }
        if (!parent.containsKey(ek)) return null;
        List<GridPoint> route = new ArrayList<>();
        long c = ek;
        while (c != -1L) {
            route.add(new GridPoint((int)(c >> 32), (int)c));
            c = parent.get(c);
        }
        Collections.reverse(route);
        return route;
    }

    private static long gpKey(GridPoint p) { return gpKey(p.i, p.j); }
    private static long gpKey(int i, int j) { return ((long)i << 32) | (j & 0xFFFFFFFFL); }

    private double computeDist(List<GridPoint> path) {
        double d = 0;
        for (int k = 1; k < path.size(); k++) {
            double di = (path.get(k - 1).i - path.get(k).i) * ctxNoNorm.dLatNm;
            double dj = (path.get(k - 1).j - path.get(k).j) * ctxNoNorm.dLonNm;
            d += Math.sqrt(di * di + dj * dj);
        }
        return d;
    }

    private static class EnvProvider implements EnvironmentSampleProvider {
        private final List<EnvironmentFrame> frames; private final boolean[][] obs; private final int n;
        EnvProvider(List<EnvironmentFrame> f, boolean[][] o, int n) { frames=f; obs=o; this.n=n; }
        @Override public int numDays() { return n; } @Override public int ny() { return obs.length; }
        @Override public int nx() { return obs[0].length; }
        @Override public boolean isObstacle(int i, int j) { return obs[i][j]; }
        @Override public double windU(int d, int i, int j) { return frames.get(d).windUData[i][j]; }
        @Override public double windV(int d, int i, int j) { return frames.get(d).windVData[i][j]; }
        @Override public double currentU(int d, int i, int j) { return frames.get(d).currentUData[i][j]; }
        @Override public double currentV(int d, int i, int j) { return frames.get(d).currentVData[i][j]; }
        @Override public double waveHeight(int d, int i, int j) { return frames.get(d).waveHeightData[i][j]; }
        @Override public double wavePeriod(int d, int i, int j) { return frames.get(d).wavePeriodData[i][j]; }
        @Override public double waveDir(int d, int i, int j) { return frames.get(d).waveDirData[i][j]; }
    }

    private static int countDays(Path dir, LocalDate base) {
        int n = 0;
        for (int off = 0; off < 30; off++) {
            String s = FMT.format(base.plusDays(off));
            if (Files.exists(dir.resolve(s + "_wind.json")) && Files.exists(dir.resolve(s + "_cur.json"))
                    && Files.exists(dir.resolve(s + "_wave.json"))) n++;
            else break;
        }
        return n;
    }
    private static EnvironmentFrame loadFrame(Path dir, LocalDate date) throws IOException {
        String s = FMT.format(date);
        WindOutput w = MAPPER.readValue(dir.resolve(s+"_wind.json").toFile(), WindOutput.class);
        CurrentOutput c = MAPPER.readValue(dir.resolve(s+"_cur.json").toFile(), CurrentOutput.class);
        WaveOutput v = MAPPER.readValue(dir.resolve(s+"_wave.json").toFile(), WaveOutput.class);
        return new EnvironmentFrame(to2D(w.windDataU,w.gridXY.ny,w.gridXY.nx), to2D(w.windDataV,w.gridXY.ny,w.gridXY.nx),
                to2D(c.currentDataU,c.gridXY.ny,c.gridXY.nx), to2D(c.currentDataV,c.gridXY.ny,c.gridXY.nx),
                to2D(v.waveDataD,v.gridXY.ny,v.gridXY.nx), to2D(v.waveDataP,v.gridXY.ny,v.gridXY.nx),
                to2D(v.waveDataH,v.gridXY.ny,v.gridXY.nx));
    }
    private static double[][] to2D(List<Double> d, int r, int c) {
        double[][] m = new double[r][c];
        if (d == null) {
            for (double[] row : m) Arrays.fill(row, Double.NaN);
            return m;
        }
        for(int i=0;i<r;i++) for(int j=0;j<c;j++) m[i][j]=sd(d.get(i*c+j)); return m;
    }
    private static boolean[][] copyBoolMap(boolean[][] s) {
        boolean[][] d = new boolean[s.length][s[0].length];
        for(int i=0;i<s.length;i++) System.arraycopy(s[i],0,d[i],0,s[i].length); return d;
    }
    private static double sd(Double v) { return v == null ? Double.NaN : v; }
}
