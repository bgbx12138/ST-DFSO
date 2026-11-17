package Navigation.object;

/**
 * 连续航迹采样点（仅用于可视化，不参与优化）。
 * 段内以 dt=segmentHours/N 步长做物理积分，沿途累积风/流影响，
 * 使 Easy 与 3D 即便 pathPoints 栅格序列相同，denseTrajectory 的
 * 几何形状也会因跨日/环境快照切换而出现轻微分叉。
 */
public class DenseTrajectoryPoint {
    public double lon;
    public double lat;
    public double elapsedHours;

    public DenseTrajectoryPoint() {
    }

    public DenseTrajectoryPoint(double lon, double lat, double elapsedHours) {
        this.lon = lon;
        this.lat = lat;
        this.elapsedHours = elapsedHours;
    }
}
