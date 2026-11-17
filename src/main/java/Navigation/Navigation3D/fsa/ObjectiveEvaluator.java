package Navigation.Navigation3D.fsa;

import Navigation.model.GridPoint;

import java.util.List;

/**
 * 目标评估接口。独立风险密度按整条路线评分，不把逐边比值相加。
 *
 * 语义：
 *   - edgeCost 映射单条已采样边的局部贡献；这不意味着每个目标都可加
 *   - heuristic 提供粗略的候选到终点吸引力
 *   - pathFinalCost 对完整路线打分，支持非加性目标
 */
public interface ObjectiveEvaluator {

    enum Mode { TIME, SAFETY_DENSITY, SAFETY_EXPOSURE, FUEL, COMPOSITE }

    Mode mode();

    /**
     * 评估一条边。若非法（进障碍/方向不可行）返回 null。
     */
    EdgeSampler.EdgeResult sampleEdge(int fromI, int fromJ, int candI, int candJ, double startElapsedHours);

    /**
     * 把一次 sampleEdge 结果转换为该目标下的局部边评分或可加边代价（越小越好）。
     * SAFETY_DENSITY 的正式目标只由 pathFinalCost 对整条路线的全局 R/L 给出。
     */
    double edgeCost(EdgeSampler.EdgeResult edge, int fromI, int fromJ, int candI, int candJ, int dayAtEdge);

    /**
     * 启发式：cand → goal 的剩余吸引力（越大越偏好）。
     * 实现应该是粗估，不涉及环境采样，避免拖慢概率选择。
     */
    double heuristic(int candI, int candJ, int goalI, int goalJ, int dayAtCand);

    /**
     * 整路径的最终代价（给精英排序/收敛曲线用）。
     * 对密度目标返回整条路线的 Σ(risk·ds)/Σds；不得由边比例求和构造。
     */
    double pathFinalCost(List<GridPoint> path);
}
