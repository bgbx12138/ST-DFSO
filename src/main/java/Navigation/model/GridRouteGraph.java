package Navigation.model;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;

/**
 * Shared spatial route graph over free raster-cell centers.
 *
 * Routes are ordered vertex sequences. Revisits are allowed; the graph never repairs,
 * simplifies, deduplicates, or otherwise rewrites a supplied route. Each directed
 * move must be within the configured Euclidean cell span and have strict supercover
 * clearance. The same geometry contract is used by FSA, the benchmark evaluator,
 * the common seed route, and graph-native comparator algorithms.
 */
public final class GridRouteGraph {

    public static final double DEFAULT_MAX_EDGE_SPAN_CELLS = 12.0;
    private static final double EPSILON = 1e-9;

    @FunctionalInterface
    public interface BlockedCell {
        boolean isBlocked(int row, int col);
    }

    public static final class Validation {
        public final boolean valid;
        public final String reason;
        public final double distanceNm;
        public final int pointCount;

        private Validation(boolean valid, String reason, double distanceNm, int pointCount) {
            this.valid = valid;
            this.reason = reason;
            this.distanceNm = distanceNm;
            this.pointCount = pointCount;
        }

        private static Validation ok(double distanceNm, int pointCount) {
            return new Validation(true, "", distanceNm, pointCount);
        }

        private static Validation invalid(String reason, int pointCount) {
            return new Validation(false, reason, Double.NaN, pointCount);
        }
    }

    private final int ny;
    private final int nx;
    private final double dRowNm;
    private final double dColNm;
    private final double maxEdgeSpanCells;
    private final BlockedCell blocked;

    public GridRouteGraph(int ny, int nx, double dRowNm, double dColNm,
                          double maxEdgeSpanCells, BlockedCell blocked) {
        if (ny <= 0 || nx <= 0) throw new IllegalArgumentException("grid dimensions must be positive");
        if (!Double.isFinite(dRowNm) || dRowNm <= 0.0
                || !Double.isFinite(dColNm) || dColNm <= 0.0) {
            throw new IllegalArgumentException("cell dimensions in nautical miles must be positive and finite");
        }
        if (!Double.isFinite(maxEdgeSpanCells) || maxEdgeSpanCells <= 0.0) {
            throw new IllegalArgumentException("max edge span must be positive and finite");
        }
        this.ny = ny;
        this.nx = nx;
        this.dRowNm = dRowNm;
        this.dColNm = dColNm;
        this.maxEdgeSpanCells = maxEdgeSpanCells;
        this.blocked = Objects.requireNonNull(blocked, "blocked-cell predicate");
    }

    public int getNy() { return ny; }
    public int getNx() { return nx; }
    public double getMaxEdgeSpanCells() { return maxEdgeSpanCells; }

    public boolean contains(GridPoint point) {
        return point != null && point.i >= 0 && point.i < ny && point.j >= 0 && point.j < nx
                && !blocked.isBlocked(point.i, point.j);
    }

    public boolean isValidEdge(GridPoint from, GridPoint to) {
        if (!contains(from) || !contains(to) || from.equals(to)) return false;
        double span = Math.hypot((double) to.i - from.i, (double) to.j - from.j);
        if (!Double.isFinite(span) || span > maxEdgeSpanCells + EPSILON) return false;
        return hasSupercoverClearance(from.i, from.j, to.i, to.j, ny, nx, blocked);
    }

    /** Deterministic row-major adjacency; route histories may revisit earlier vertices. */
    public List<GridPoint> neighbors(GridPoint from) {
        if (!contains(from)) return Collections.emptyList();
        int radius = (int) Math.ceil(maxEdgeSpanCells);
        int rowMin = Math.max(0, from.i - radius);
        int rowMax = Math.min(ny - 1, from.i + radius);
        int colMin = Math.max(0, from.j - radius);
        int colMax = Math.min(nx - 1, from.j + radius);
        List<GridPoint> result = new ArrayList<>();
        for (int row = rowMin; row <= rowMax; row++) {
            for (int col = colMin; col <= colMax; col++) {
                GridPoint next = new GridPoint(row, col);
                if (isValidEdge(from, next)) result.add(next);
            }
        }
        return result;
    }

    public double edgeDistanceNm(GridPoint from, GridPoint to) {
        if (from == null || to == null) return Double.NaN;
        double row = ((double) to.i - from.i) * dRowNm;
        double col = ((double) to.j - from.j) * dColNm;
        return Math.hypot(row, col);
    }

    public double routeDistanceNm(List<GridPoint> route) {
        if (route == null || route.size() < 2) return Double.NaN;
        double sum = 0.0;
        for (int k = 1; k < route.size(); k++) {
            double edge = edgeDistanceNm(route.get(k - 1), route.get(k));
            if (!Double.isFinite(edge) || edge <= 0.0) return Double.NaN;
            sum += edge;
        }
        return Double.isFinite(sum) ? sum : Double.NaN;
    }

    /**
     * Validate the exact supplied route. A positive finite distance limit is enforced
     * when provided; positive infinity means geometry-only validation. No path edits occur.
     */
    public Validation validate(List<GridPoint> route, GridPoint expectedStart,
                               GridPoint expectedEnd, double maxRouteDistanceNm) {
        if (Double.isNaN(maxRouteDistanceNm) || maxRouteDistanceNm <= 0.0) {
            throw new IllegalArgumentException("route distance limit must be positive or positive infinity");
        }
        if (route == null || route.size() < 2) return Validation.invalid("route needs at least two vertices", 0);
        if (expectedStart != null && !expectedStart.equals(route.get(0))) {
            return Validation.invalid("route start differs from the shared start", route.size());
        }
        if (expectedEnd != null && !expectedEnd.equals(route.get(route.size() - 1))) {
            return Validation.invalid("route end differs from the shared goal", route.size());
        }
        double totalNm = 0.0;
        for (int k = 0; k < route.size(); k++) {
            GridPoint point = route.get(k);
            if (!contains(point)) return Validation.invalid("vertex " + k + " is out of bounds or blocked", route.size());
            if (k == 0) continue;
            GridPoint previous = route.get(k - 1);
            if (!isValidEdge(previous, point)) {
                return Validation.invalid("edge " + (k - 1) + "->" + k + " violates span or supercover clearance", route.size());
            }
            totalNm += edgeDistanceNm(previous, point);
            if (!Double.isFinite(totalNm)) return Validation.invalid("route distance is not finite", route.size());
            if (totalNm > maxRouteDistanceNm + EPSILON) {
                return Validation.invalid("route exceeds the shared distance budget", route.size());
            }
        }
        return Validation.ok(totalNm, route.size());
    }

    public Validation validate(List<GridPoint> route) {
        return validate(route, null, null, Double.POSITIVE_INFINITY);
    }

    /** Exact closed-cell supercover; corner crossings require both side cells to be free. */
    public static boolean hasSupercoverClearance(int row0, int col0, int row1, int col1,
                                                  int ny, int nx, BlockedCell blocked) {
        if (blocked == null || !inBounds(row0, col0, ny, nx) || !inBounds(row1, col1, ny, nx)) return false;
        if (blocked.isBlocked(row0, col0) || blocked.isBlocked(row1, col1)) return false;
        int deltaCol = Math.abs(col1 - col0);
        int deltaRow = Math.abs(row1 - row0);
        int stepCol = Integer.compare(col1, col0);
        int stepRow = Integer.compare(row1, row0);
        int col = col0;
        int row = row0;
        int crossedCols = 0;
        int crossedRows = 0;

        while (crossedCols < deltaCol || crossedRows < deltaRow) {
            long decision = (1L + 2L * crossedCols) * deltaRow
                    - (1L + 2L * crossedRows) * deltaCol;
            if (decision == 0L) {
                int sideACol = col + stepCol;
                int sideBRow = row + stepRow;
                if (!isFree(sideACol, row, ny, nx, blocked)
                        || !isFree(col, sideBRow, ny, nx, blocked)) return false;
                col += stepCol;
                row += stepRow;
                crossedCols++;
                crossedRows++;
            } else if (decision < 0L) {
                col += stepCol;
                crossedCols++;
            } else {
                row += stepRow;
                crossedRows++;
            }
            if (!isFree(col, row, ny, nx, blocked)) return false;
        }
        return true;
    }

    private static boolean isFree(int col, int row, int ny, int nx, BlockedCell blocked) {
        return inBounds(row, col, ny, nx) && !blocked.isBlocked(row, col);
    }

    private static boolean inBounds(int row, int col, int ny, int nx) {
        return row >= 0 && row < ny && col >= 0 && col < nx;
    }
}
