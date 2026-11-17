package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * 鱼个体：拥有自己的随机流、当前最佳路径与对应代价。
 */
public final class Fish {
    public final int id;
    public final Random rng;
    public List<GridPoint> personalBestPath;
    public double personalBestCost;

    public Fish(int id, long seed) {
        this.id = id;
        this.rng = new Random(seed);
        this.personalBestPath = new ArrayList<>();
        this.personalBestCost = Double.POSITIVE_INFINITY;
    }

    public void update(List<GridPoint> path, double cost) {
        if (!Double.isFinite(cost)) return;
        if (path == null || path.size() < 2) return;
        if (cost < personalBestCost) {
            personalBestCost = cost;
            personalBestPath = new ArrayList<>(path);
        }
    }
}
