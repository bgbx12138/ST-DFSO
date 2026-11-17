package Navigation.benchmark;

import Navigation.model.GridPoint;
import Navigation.Navigation3D.HybridFSA3D;
import Navigation.object.NavigationConfig;

import java.util.List;

/**
 * 所有 Dijkstra-seeded 群智能优化器的统一契约。
 * 每个算法只在 optimize() 内部有区别。
 */
public interface HybridSwarmOptimizer {

    /** 算法标识符 */
    String algorithmName();

    /**
     * @param seedPath  Dijkstra 最优路径
     * @param config    导航配置
     * @param mode      目标模式
     * @param weights   COMPOSITE 权重（非 COMPOSITE 时传 null）
     * @param runSeed   随机种子
     * @return 优化结果
     */
    SwarmOptResult optimize(
            List<GridPoint> seedPath,
            NavigationConfig config,
            HybridFSA3D.ObjectiveMode mode,
            Navigation.Navigation3D.Normalization.ObjectiveWeights weights,
            long runSeed
    );
}
