package Navigation.benchmark.algo;

import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.SearchWorkLedger;
import Navigation.benchmark.*;
import Navigation.model.GridPoint;
import Navigation.object.NavigationConfig;
import java.util.*;

/** 离散 ACO — 蚂蚁在网格上逐步构路，信息素在边上沉积。
 * Q 自适应缩放：deposit = Q × seedCost / cost，跨目标量级一致。 */
public class AcoOptimizer166 implements HybridSwarmOptimizer {

    private ExternalRun166 run166;
    public AcoOptimizer166 withRun(ExternalRun166 run){this.run166=Objects.requireNonNull(run);return this;}
    private final int ants, iters;
    private final double detourKappa;
    private final long maxSubsegmentAttempts;
    private final double evap = 0.1, alpha = 1.0, beta = 2.0, Q0 = 1.0;

    public AcoOptimizer166(int pop, int iters) {
        this(pop, iters, 0.0, 0L);
    }
    public AcoOptimizer166(int pop, int iters, double detourKappa, long maxSubsegmentAttempts) {
        if (detourKappa != 0.0 && (!Double.isFinite(detourKappa) || detourKappa < 1.0))
            throw new IllegalArgumentException("detour kappa must be zero (disabled) or at least 1.0");
        if (maxSubsegmentAttempts < 0L)
            throw new IllegalArgumentException("subsegment budget cannot be negative");
        if(pop<1||iters<1)throw new IllegalArgumentException("positive population/iterations required");
        this.ants = pop;
        this.iters = iters;
        this.detourKappa = detourKappa;
        this.maxSubsegmentAttempts = maxSubsegmentAttempts;
    }

    @Override public String algorithmName() { return "ACO"; }

    @Override
    public SwarmOptResult optimize(List<GridPoint> seedPath, NavigationConfig config,
                                   HybridFSA3D.ObjectiveMode mode,
                                   ObjectiveWeights weights, long runSeed) {
        Random rng = new Random(runSeed);
        SearchWorkLedger searchWork = Objects.requireNonNull(run166,"explicit run166 required").ledger;
        FairPathEvaluator166 eval;
        try { eval = new FairPathEvaluator166(config, searchWork); } catch (Exception e) { throw new RuntimeException(e); }
        if (detourKappa > 0.0) eval.configureDistanceBudgetFromSeed(seedPath, detourKappa);

        run166.start(maxSubsegmentAttempts);
        try {
        int ny = eval.getNy(), nx = eval.getNx();
        GridPoint start = seedPath.get(0), end = seedPath.get(seedPath.size() - 1);

        // Adaptive Q: seedCost normalises deposit across objectives (TIME ~30h, FUEL ~1e10J, etc.)
        double seedCost = run166.checkedSeed(eval.evaluateStrict(seedPath, mode, weights));
        double Q;
        if (Double.isFinite(seedCost)) {
            Q = Q0 * Math.max(seedCost, 1e-12);
        } else {
            // seedPath 不可行时，用路径长度估计合理 Q 值，防止 Infinity 传染信息素
            double estDist = 0;
            for (int k = 0; k < seedPath.size() - 1; k++)
                estDist += dist(seedPath.get(k), seedPath.get(k + 1));
            Q = Q0 * Math.max(estDist, 1.0);
        }

        Map<Long, Double> tau = new HashMap<>();
        // 强信息素播种：让蚂蚁初始时强烈跟随 Dijkstra 先验路径
        double tau0 = 1.0;
        for (int k = 0; k < seedPath.size() - 1; k++)
            tau.put(edgeKey(seedPath.get(k), seedPath.get(k+1), nx), tau0 * 100.0);

        double bestCost = seedCost;
        List<GridPoint> bestPath = copyRoute(seedPath);
        // Match FSA and IAFSA history semantics: row 0 is the shared seed,
        // followed by one best-so-far value after each completed ACO iteration.
        List<Double> deltaHist = new ArrayList<>(iters + 1);
        deltaHist.add(bestCost);
        run166.checkpoint(bestCost,true,"initialization_boundary");
        long t0 = System.currentTimeMillis();

        for (int t = 0; t < iters && !searchWork.shouldStopSearch(); t++) {
            List<GridPoint> iterBestPath = null;
            double iterBestCost = Double.POSITIVE_INFINITY;

            for (int a = 0; a < ants && !searchWork.shouldStopSearch(); a++) {
                List<GridPoint> path = constructPath(start, end, tau, eval, mode, weights, ny, nx, rng);
                if (path != null && path.size() >= 2
                        && path.get(path.size()-1).i == end.i
                        && path.get(path.size()-1).j == end.j) {
                    double cost = eval.evaluate(path, mode, weights);
                    if (ExternalRun166.valid(cost) && cost < iterBestCost) { iterBestCost = cost; iterBestPath = path; }
                    if (ExternalRun166.valid(cost) && cost < bestCost) { bestCost = cost; bestPath = path; }
                }
            }
            // Evaporate
            tau.replaceAll((k, v) -> v * (1.0 - evap));
            // Deposit pheromone on this iteration's best path
            if (iterBestPath != null && iterBestCost < Double.POSITIVE_INFINITY) {
                double deposit = Q / Math.max(iterBestCost, 1e-12);
                for (int k = 0; k < iterBestPath.size() - 1; k++)
                    tau.merge(edgeKey(iterBestPath.get(k), iterBestPath.get(k+1), nx), deposit, Double::sum);
            }
            deltaHist.add(bestCost);
            run166.checkpoint(bestCost,false,"iteration_boundary");
        }

        run166.finish(bestCost);
        long ms = System.currentTimeMillis() - t0;
        return new SwarmOptResult("ACO", "", mode.name(), 0, runSeed,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN, bestCost, ms, ants, iters, deltaHist,
                null, bestPath, false, searchWork.snapshot());
        } finally {eval.close();}
    }

    /** 局部搜索：从路径中采样 K 个中间航点，随机微调以改进代价。 */
    private List<GridPoint> localSearchImprove(List<GridPoint> path,
                                                PathEvaluator eval,
                                                HybridFSA3D.ObjectiveMode mode,
                                                ObjectiveWeights weights,
                                                int ny, int nx, Random rng) {
        // 采样 5 个中间航点索引
        int n = path.size();
        int K = Math.min(5, n - 2);
        if (K < 1) return path;
        int[] midIdx = new int[K];
        for (int k = 0; k < K; k++)
            midIdx[k] = 1 + (int) Math.round((double)(k + 1) * (n - 2) / (K + 1));

        // 构建精简路径 (start + K mid + end)
        List<GridPoint> compact = new ArrayList<>();
        compact.add(path.get(0));
        for (int k = 0; k < K; k++) compact.add(path.get(midIdx[k]));
        compact.add(path.get(n - 1));

        double curCost = eval.evaluate(compact, mode, weights);
        if (!Double.isFinite(curCost)) return path;

        // 每个中间航点尝试 10 次随机微调
        for (int k = 1; k <= K; k++) {
            GridPoint wp = compact.get(k);
            for (int trial = 0; trial < 10; trial++) {
                int di = (rng.nextInt(5) - 2);  // [-2, +2]
                int dj = (rng.nextInt(5) - 2);
                int ni = Math.max(0, Math.min(ny - 1, wp.i + di));
                int nj = Math.max(0, Math.min(nx - 1, wp.j + dj));
                if (eval.isObstacle(ni, nj)) continue;
                GridPoint old = compact.set(k, new GridPoint(ni, nj));
                double newCost = eval.evaluate(compact, mode, weights);
                if (Double.isFinite(newCost) && newCost < curCost) {
                    curCost = newCost;
                } else {
                    compact.set(k, old);  // 回退
                }
            }
        }
        return compact;
    }

    private List<GridPoint> constructPath(GridPoint start, GridPoint end,
                                           Map<Long, Double> tau,
                                           PathEvaluator eval,
                                           HybridFSA3D.ObjectiveMode mode,
                                           ObjectiveWeights weights,
                                           int ny, int nx, Random rng) {
        List<GridPoint> path = new ArrayList<>();
        path.add(start);
        Set<GridPoint> visited=new HashSet<>();visited.add(start);
        double routeDistanceNm = 0.0;

        GridPoint cur = start;
        int maxSteps = 5000;
        for (int step = 0; step < maxSteps && !run166.ledger.shouldStopSearch(); step++) {
            if (cur.i == end.i && cur.j == end.j) break;

            // Traverse the same bounded visibility graph used by FSA and strict evaluation.
            List<GridPoint> neigh = new ArrayList<>();
            double curDist = dist(cur, end);
            for (GridPoint n : eval.getRouteGraph().neighbors(cur)) {
                if(visited.contains(n))continue;
                GridPoint previous=path.size()>1?path.get(path.size()-2):null;
                if(!((FairPathEvaluator166)eval).extensionIsPractical(previous,cur,n))continue;
                double edgeDistanceNm = eval.getRouteGraph().edgeDistanceNm(cur, n);
                if (routeDistanceNm + edgeDistanceNm > eval.getMaxRouteDistanceNm() + 1e-9) continue;
                neigh.add(n);
            }
            if (neigh.isEmpty()) break;

            // Prefer closer-to-goal neighbours (bias, don't discard farther ones entirely)
            List<GridPoint> closer = new ArrayList<>();
            for (GridPoint n : neigh)
                if (dist(n, end) < curDist) closer.add(n);
            if (closer.size() >= 2) neigh = closer;  // only filter if at least 2 closer options exist

            // Probability
            double[] probs = new double[neigh.size()];
            double total = 0;
            for (int k = 0; k < neigh.size(); k++) {
                GridPoint n = neigh.get(k);
                long ek = edgeKey(cur, n, nx);
                double tauv = tau.getOrDefault(ek, 1.0);
                double heur = 1.0 / (1.0 + dist(n, end));
                probs[k] = Math.pow(tauv, alpha) * Math.pow(heur, beta);
                total += probs[k];
            }
            if (total <= 0) break;
            double r = rng.nextDouble() * total;
            double sum = 0; int pick = 0;
            for (int k = 0; k < neigh.size(); k++) { sum += probs[k]; if (r <= sum) { pick = k; break; } }

            GridPoint next = neigh.get(pick);
            routeDistanceNm += eval.getRouteGraph().edgeDistanceNm(cur, next);
            cur = next;
            path.add(cur);visited.add(cur);
        }
        return path;
    }

    private static List<GridPoint> copyRoute(List<GridPoint> route) {
        List<GridPoint> copy = new ArrayList<>(route.size());
        for (GridPoint point : route) copy.add(new GridPoint(point.i, point.j));
        return copy;
    }

    // long-encoded keys (avoids GC pressure from String concatenation)
    private static long edgeKey(GridPoint a, GridPoint b, int nx) {
        return pack(a.i, a.j, b.i, b.j, nx);
    }
    private static long cellKey(GridPoint p, int nx) {
        return ((long)p.i << 32) | (p.j & 0xFFFFFFFFL);
    }
    static long pack(int fi, int fj, int ti, int tj, int nx) {
        long from = ((long)fi * nx + fj);
        long to   = ((long)ti * nx + tj);
        return (from << 32) | (to & 0xFFFFFFFFL);
    }
    private static double dist(GridPoint a, GridPoint b) {
        double di = a.i - b.i, dj = a.j - b.j;
        return Math.sqrt(di*di + dj*dj);
    }
}
