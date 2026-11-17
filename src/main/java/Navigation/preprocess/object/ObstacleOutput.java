package Navigation.preprocess.object;

import java.util.Objects;

public class ObstacleOutput extends BaseOutput {
    public boolean[][] obstacleMap;
    public int[][] obstacleCodeMap;
    public int obstacleCount;

    public ObstacleOutput() {
    }

    public ObstacleOutput(String projectName, Area area, Grid gridXY, boolean[][] obstacleMap, int[][] obstacleCodeMap, int obstacleCount) {
        super(projectName, area, gridXY);
        this.obstacleMap = Objects.requireNonNull(obstacleMap);
        this.obstacleCodeMap = Objects.requireNonNull(obstacleCodeMap);
        this.obstacleCount = obstacleCount;
    }
}