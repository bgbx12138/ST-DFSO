package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * 精英解集：按 cost 升序保留 capacity 条最好的路径；
 * 支持查询"某格点在精英集合中的出现频率"（用于鱼群的社会学习项）。
 */
public final class EliteArchive {

    private final int capacity;
    private final boolean uniqueGeometry;
    private final List<Entry> entries = new ArrayList<>();
    private volatile boolean active = false;
    private int[][] frequencyGrid;   // 懒初始化
    /** 边计数：key = 边唯一编码，value = 含该边的精英解数。供 FSPO 社会学习项使用。 */
    private Map<Long, Integer> edgeCount = new HashMap<>();
    private int cachedNx = 0;

    public EliteArchive(int capacity) {
        this(capacity,false);
    }
    public EliteArchive(int capacity,boolean uniqueGeometry) {
        this.capacity = Math.max(1, capacity);
        this.uniqueGeometry=uniqueGeometry;
    }

    public int capacity() { return capacity; }
    public int size() { return entries.size(); }
    public boolean isActive() { return active; }
    public void setActive(boolean v) { this.active = v; }

    public synchronized void rebuildFrom(List<List<GridPoint>> candidatePaths,
                                         List<Double> candidateCosts,
                                         int ny, int nx) {
        entries.clear();
        int n = Math.min(candidatePaths.size(), candidateCosts.size());
        for (int k = 0; k < n; k++) {
            List<GridPoint> p = candidatePaths.get(k);
            Double c = candidateCosts.get(k);
            if (p == null || p.size() < 2 || c == null || !Double.isFinite(c)) continue;
            entries.add(new Entry(new ArrayList<>(p), c));
        }
        entries.sort(Comparator.comparingDouble(e -> e.cost));
        if(uniqueGeometry){
            java.util.Set<String> seen=new java.util.HashSet<>();
            entries.removeIf(e->!seen.add(RequestMigration.shapeKey(e.path)));
        }
        if (entries.size() > capacity) {
            entries.subList(capacity, entries.size()).clear();
        }
        cachedNx = nx;
        rebuildFrequencyGrid(ny, nx);
        rebuildEdgeCount(nx);
    }

    private void rebuildEdgeCount(int nx) {
        edgeCount.clear();
        for (Entry e : entries) {
            // 每条路径中出现过的边只计一次（避免同一解内重复边放大频率）
            java.util.Set<Long> seen = new java.util.HashSet<>();
            List<GridPoint> p = e.path;
            for (int k = 0; k < p.size() - 1; k++) {
                GridPoint a = p.get(k), b = p.get(k + 1);
                long key = edgeKey(a.i, a.j, b.i, b.j, nx);
                if (seen.add(key)) {
                    edgeCount.merge(key, 1, Integer::sum);
                }
            }
        }
    }

    private static long edgeKey(int fi, int fj, int ti, int tj, int nx) {
        long from = ((long) fi) * nx + fj;
        long to = ((long) ti) * nx + tj;
        return (from << 32) | (to & 0xFFFFFFFFL);
    }

    /**
     * 精英解集中含边 (fi,fj)→(ti,tj) 的解占比；最小回退 0.1（给新边最少探索机会）。
     * 未激活或 entries 为空时返回 0.1。
     */
    public double edgeFrequency(int fi, int fj, int ti, int tj, int nx) {
        if (!active || entries.isEmpty()) return 0.1;
        long key = edgeKey(fi, fj, ti, tj, nx);
        Integer c = edgeCount.get(key);
        if (c == null) return 0.1;
        return Math.max(0.1, (double) c / entries.size());
    }

    private void rebuildFrequencyGrid(int ny, int nx) {
        if (frequencyGrid == null || frequencyGrid.length != ny || frequencyGrid[0].length != nx) {
            frequencyGrid = new int[ny][nx];
        } else {
            for (int i = 0; i < ny; i++) {
                java.util.Arrays.fill(frequencyGrid[i], 0);
            }
        }
        for (Entry e : entries) {
            for (GridPoint g : e.path) {
                if (g.i >= 0 && g.i < ny && g.j >= 0 && g.j < nx) {
                    frequencyGrid[g.i][g.j]++;
                }
            }
        }
    }

    public double frequencyAt(int i, int j) {
        if (!active || frequencyGrid == null) return 0.0;
        if (i < 0 || i >= frequencyGrid.length || j < 0 || j >= frequencyGrid[0].length) return 0.0;
        if (entries.isEmpty()) return 0.0;
        return frequencyGrid[i][j] / (double) entries.size();
    }

    public Entry best() {
        return entries.isEmpty() ? null : entries.get(0);
    }

    /** 返回所有精英路径的列表（只读视图）。 */
    public List<List<GridPoint>> allPaths() {
        List<List<GridPoint>> paths = new ArrayList<>(entries.size());
        for (Entry e : entries) paths.add(e.path);
        return paths;
    }

    /** 返回精英路径和代价的独立快照，供分裂子群初始化与汇合使用。 */
    public synchronized List<Entry> snapshotEntries() {
        List<Entry> snapshot = new ArrayList<>(entries.size());
        for (Entry e : entries) {
            snapshot.add(new Entry(new ArrayList<>(e.path), e.cost));
        }
        return snapshot;
    }

    /** 将新候选并入当前精英集，再按容量重建频率缓存。 */
    public synchronized void offer(List<GridPoint> path, double cost, int ny, int nx) {
        if (path == null || path.size() < 2 || !Double.isFinite(cost)) return;
        List<List<GridPoint>> paths = new ArrayList<>(entries.size() + 1);
        List<Double> costs = new ArrayList<>(entries.size() + 1);
        for (Entry e : entries) {
            paths.add(e.path);
            costs.add(e.cost);
        }
        paths.add(path);
        costs.add(cost);
        rebuildFrom(paths, costs, ny, nx);
    }

    public static final class Entry {
        public final List<GridPoint> path;
        public final double cost;
        public Entry(List<GridPoint> path, double cost) {
            this.path = path;
            this.cost = cost;
        }
    }
}
