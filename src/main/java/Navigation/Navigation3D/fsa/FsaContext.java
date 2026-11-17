package Navigation.Navigation3D.fsa;

import Navigation.Navigation3D.Normalization.FuelNormalizer;
import Navigation.Navigation3D.Normalization.TimeNormalizer;
import Navigation.object.NavigationConfig;
import Navigation.object.ShipParameters;
import Navigation.model.GridRouteGraph;

import java.util.List;

/**
 * GFS-3D 的共享只读上下文：持有网格、环境、障碍、船舶、归一化器等。
 * 所有 Evaluator / Fish / EdgeSampler 均从此上下文读数据，主类构造一次即可重复使用。
 */
public final class FsaContext {

    // 与 NavigationObjective3D 保持一致的 8 邻域方向与方位角
    public static final int[][] DIRECTIONS = {
            {0, 1}, {0, -1}, {1, 0}, {-1, 0},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };
    public static final double[] BEARINGS = {90, 270, 0, 180, 45, 315, 135, 225};

    public static final double DAY_HOURS = 24.0;
    public static final double EPSILON = 1e-9;
    public static final double KNOT_TO_MS = 0.5144;
    /** 安全采样物理步长：每 0.5 格采样一次，与 8 邻域退化一致。 */
    public static final double SAFETY_SAMPLE_STEP_CELLS = 0.5;

    public final NavigationConfig config;
    public final ShipParameters ship;
    public final int ny;
    public final int nx;
    public final int numDays;
    public final double departureOffsetHours;
    public final double arrivalDeadlineHours;

    public final List<EnvironmentFrame> frames;     // size == numDays
    public final boolean[][] obstacleMap;
    public final boolean[][] obstacleBufferMap;

    public final double dLatNm;
    public final double dLonNm;
    public final double diagDist;
    public final double deltaLat;
    public final double deltaLon;

    public final TimeNormalizer timeNormalizer;     // 可能为 null（非 COMPOSITE 或关闭归一化时）
    public final FuelNormalizer fuelNormalizer;     // 可能为 null
    public final SearchWorkLedger searchWorkLedger; // null outside metered optimizer runs
    public final GridRouteGraph routeGraph;

    public FsaContext(NavigationConfig config,
                      ShipParameters ship,
                      int ny, int nx, int numDays,
                      double departureOffsetHours,
                      List<EnvironmentFrame> frames,
                      boolean[][] obstacleMap,
                      boolean[][] obstacleBufferMap,
                      double dLatNm, double dLonNm,
                      double deltaLat, double deltaLon,
                      TimeNormalizer timeNormalizer,
                      FuelNormalizer fuelNormalizer) {
        this(config, ship, ny, nx, numDays, departureOffsetHours, frames, obstacleMap,
                obstacleBufferMap, dLatNm, dLonNm, deltaLat, deltaLon,
                timeNormalizer, fuelNormalizer, null);
    }

    public FsaContext(NavigationConfig config,
                      ShipParameters ship,
                      int ny, int nx, int numDays,
                      double departureOffsetHours,
                      List<EnvironmentFrame> frames,
                      boolean[][] obstacleMap,
                      boolean[][] obstacleBufferMap,
                      double dLatNm, double dLonNm,
                      double deltaLat, double deltaLon,
                      TimeNormalizer timeNormalizer,
                      FuelNormalizer fuelNormalizer,
                      SearchWorkLedger searchWorkLedger) {
        this.config = config;
        this.ship = ship;
        this.ny = ny;
        this.nx = nx;
        this.numDays = numDays;
        this.departureOffsetHours = departureOffsetHours;
        this.arrivalDeadlineHours = config.resolveArrivalDeadlineHours();
        this.frames = frames;
        this.obstacleMap = obstacleMap;
        this.obstacleBufferMap = obstacleBufferMap;
        this.dLatNm = dLatNm;
        this.dLonNm = dLonNm;
        this.diagDist = Math.sqrt(dLatNm * dLatNm + dLonNm * dLonNm);
        this.deltaLat = deltaLat;
        this.deltaLon = deltaLon;
        this.timeNormalizer = timeNormalizer;
        this.fuelNormalizer = fuelNormalizer;
        this.searchWorkLedger = searchWorkLedger;
        this.routeGraph = new GridRouteGraph(ny, nx, dLatNm, dLonNm,
                GridRouteGraph.DEFAULT_MAX_EDGE_SPAN_CELLS, this::blocked);
    }

    public void recordOperatorEvent(String name) {
        if (searchWorkLedger != null) searchWorkLedger.recordOperatorEvent(name);
    }

    public int clampDay(double elapsedHours) {
        double total = departureOffsetHours + Math.max(0.0, elapsedHours);
        int d = (int) Math.floor(total / DAY_HOURS);
        if (d < 0) return 0;
        if (d >= numDays) return numDays - 1;
        return d;
    }

    /** True only while an observed daily weather frame exists for this elapsed time. */
    public boolean hasWeatherAt(double elapsedHours) {
        double total = departureOffsetHours + Math.max(0.0, elapsedHours);
        return total < numDays * DAY_HOURS - EPSILON;
    }

    /** A route may end exactly at the data boundary, but may not travel beyond it. */
    public boolean weatherCoversThrough(double elapsedHours) {
        double total = departureOffsetHours + Math.max(0.0, elapsedHours);
        return total <= numDays * DAY_HOURS + EPSILON;
    }

    public boolean meetsArrivalDeadline(double elapsedHours) {
        return elapsedHours <= arrivalDeadlineHours + EPSILON;
    }

    public boolean inBounds(int i, int j) {
        return i >= 0 && i < ny && j >= 0 && j < nx;
    }

    public boolean blocked(int i, int j) {
        return obstacleBufferMap != null && obstacleBufferMap[i][j];
    }

    /** 物理距离：ny 方向步长 dLatNm, nx 方向步长 dLonNm（海里）。 */
    public double gridDistanceNm(int i1, int j1, int i2, int j2) {
        double dy = (i2 - i1) * dLatNm;
        double dx = (j2 - j1) * dLonNm;
        return Math.sqrt(dy * dy + dx * dx);
    }

    /**
     * 计算从 (i1,j1) → (i2,j2) 的方位角（度，0=正北，90=正东）。
     * 用网格单元海里数做正切，保证与 Dijkstra 的 8 邻域 bearings 口径一致：
     *   同一条 8 邻域边，atan2(dx*dLonNm, dy*dLatNm) 给出的 bearing 等于 BEARINGS[directionIndex]。
     */
    public double bearingOf(int i1, int j1, int i2, int j2) {
        double dy = (i2 - i1) * dLatNm;
        double dx = (j2 - j1) * dLonNm;
        if (Math.abs(dy) < EPSILON && Math.abs(dx) < EPSILON) return 0.0;
        double deg = Math.toDegrees(Math.atan2(dx, dy));
        if (deg < 0) deg += 360.0;
        return deg;
    }

    /** 找到 (di,dj) 对应的 DIRECTIONS 下标；找不到返回 -1。 */
    public static int findDirectionIndex(int di, int dj) {
        for (int k = 0; k < DIRECTIONS.length; k++) {
            if (DIRECTIONS[k][0] == di && DIRECTIONS[k][1] == dj) return k;
        }
        return -1;
    }

    /** 把连续 bearing 映射到最近的 8 邻域方向索引（用于归一化分位查询）。 */
    public static int nearestBearingIndex(double bearing) {
        double b = ((bearing % 360.0) + 360.0) % 360.0;
        int best = 0;
        double bestDiff = Double.MAX_VALUE;
        for (int k = 0; k < BEARINGS.length; k++) {
            double bb = BEARINGS[k];
            double diff = Math.abs(bb - b);
            if (diff > 180.0) diff = 360.0 - diff;
            if (diff < bestDiff) {
                bestDiff = diff;
                best = k;
            }
        }
        return best;
    }
}
