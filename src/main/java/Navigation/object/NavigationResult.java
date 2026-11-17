package Navigation.object;

import java.util.List;

public class NavigationResult {
    public String projectName;
    public String calculationType;
    public String navigationDate;
    public NavigationArea area;
    public NavigationGrid grid;
    public ShipParameters shipParameters;
    public double totalDistance;
    public double totalTime;
    public Double totalSafetyScore;
    public Double totalCumulativeRisk;
    public Double totalFuelEnergyJ;
    public Double totalCompositeCost;
    public Double safetyWeight;
    public Double timeWeight;
    public Double fuelWeight;
    public int pathPointCount;
    public String windFile;
    public String currentFile;
    public String waveFile;
    public String obstacleFile;
    public List<RoutePoint> pathPoints;
    public List<DaySwitchPoint> daySwitchPoints;
    // 段内 dt 物理积分得到的连续航迹（可视化用，不参与优化）。
    // 仅 Dijkstra 路径（NavigationTimeEasy / NavigationObjective3D）当前写入该字段；
    // FSA 等其他路径可能为 null，消费方需兼容缺失。
    public List<DenseTrajectoryPoint> denseTrajectory;

    public static class DaySwitchPoint {
        public String label;
        public Integer dayNumber;
        public double gridX;
        public double gridY;
        public double lon;
        public double lat;
        public Integer nearestPathIndex;

        public DaySwitchPoint() {
        }
    }

    public NavigationResult() {
    }
}