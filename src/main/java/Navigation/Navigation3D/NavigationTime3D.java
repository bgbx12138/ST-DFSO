package Navigation.Navigation3D;

import Navigation.model.GridPoint;
import Navigation.model.GridRouteGraph;
import Navigation.PathVisualizer;
import Navigation.object.NavigationConfig;
import Navigation.object.NavigationResult;
import Navigation.object.RoutePoint;
import Navigation.object.ShipParameters;
import Navigation.preprocess.object.CurrentOutput;
import Navigation.preprocess.object.ObstacleOutput;
import Navigation.preprocess.object.WaveOutput;
import Navigation.preprocess.object.WindOutput;
import Navigation.utils.ShipUtils;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.SerializationFeature;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.time.LocalDate;
import java.time.format.DateTimeFormatter;
import java.time.format.DateTimeParseException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.PriorityQueue;

/**
 * 三维分层图导航规划器
 *
 * 状态空间: (i, j, day)
 *   i, j — 网格行列索引
 *   day  — 第几天 (0-indexed)
 *
 * 状态含义:
 *   day = floor(totalSailingTime / 24)
 *   即第几个 24 小时时间层，由累计航行时间自然决定
 *
 * 状态转移:
 *   (i,j,day) → (ni,nj,nextDay)
 *   使用当前累计航行时间对应的环境场计算该段时间；
 *   当单段航行跨越 24h 边界时，直接切换到下一天环境继续积分，不引入等待时间。
 *
 * 目标: 最小化纯航行时间
 */
public class NavigationTime3D {

    private static final double DAY_HOURS = 24.0;
    private static final double EPSILON = 1e-9;
    private static final DateTimeFormatter DATE_FORMATTER = DateTimeFormatter.BASIC_ISO_DATE;

    private final NavigationConfig config;
    private final ObjectMapper objectMapper;

    private boolean[][] obstacleMap;
    private boolean[][] obstacleBufferMap;

    private final List<EnvironmentSnapshot> dailySnapshots = new ArrayList<>();
    private int numDays;

    private double dLatNm;
    private double dLonNm;
    private double diagDist;
    private double deltaLat;
    private double deltaLon;
    private double[] directionDistances;

    private LocalDate baseDate;
    private Path dataFolderPath;
    private double departureHour;
    private List<GridPoint> shortestTimePath = new ArrayList<>();
    private List<double[]> timedPathPoints = new ArrayList<>();
    private List<PathVisualizer.MarkerPoint> daySwitchPoints = new ArrayList<>();
    private int daysUsed;
    private double departureOffsetHours;

    private final int[][] directions = {
            {0, 1}, {0, -1}, {1, 0}, {-1, 0},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    private final double[] bearings = {90, 270, 0, 180, 45, 315, 135, 225};

    public NavigationTime3D(NavigationConfig config) {
        this.config = config;
        this.objectMapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
    }

    public NavigationResult execute() throws Exception {
        validateConfig();
        loadAllEnvironmentData();
        calculateGridResolution();
        initializeDirectionDistances();

        Dijkstra3DResult result3D = runTimeDijkstra3D();
        shortestTimePath = result3D.spatialPath;
        daysUsed = result3D.getDaysUsed();
        double totalDistance = calculatePathDistance(shortestTimePath);
    double totalTime = Math.max(0.0, result3D.totalTime - departureHour);
        timedPathPoints = buildTimedPathPoints(shortestTimePath);
        daySwitchPoints = calculateDaySwitchPoints(shortestTimePath);

        NavigationResult result = buildResult(totalDistance, totalTime);
        writeResult(result);

        if (config.showVisualization) {
            exportVisualization(totalDistance, totalTime, true);
        }

        return result;
    }

    public List<GridPoint> getShortestTimePath() {
        return new ArrayList<>(shortestTimePath);
    }

    public List<PathVisualizer.MarkerPoint> getDaySwitchPoints() {
        return new ArrayList<>(daySwitchPoints);
    }

    public int getDaysUsed() {
        return daysUsed;
    }

    public int getNumDays() {
        return numDays;
    }

    public List<double[]> getTimedPathPoints() {
        List<double[]> copy = new ArrayList<>(timedPathPoints.size());
        for (double[] point : timedPathPoints) {
            copy.add(Arrays.copyOf(point, point.length));
        }
        return copy;
    }

    // ==================== 校验 ====================

    private void validateConfig() throws IOException {
        if (config == null) {
            throw new IllegalArgumentException("导航配置不能为空");
        }
        if (config.area == null) {
            throw new IllegalArgumentException("导航区域参数不能为空");
        }
        if (config.grid == null) {
            throw new IllegalArgumentException("导航网格参数不能为空");
        }
        if (config.shipParameters == null) {
            throw new IllegalArgumentException("船舶参数不能为空");
        }
        if (config.navigationDate == null || config.navigationDate.isBlank()) {
            throw new IllegalArgumentException("导航日期不能为空");
        }
        if (config.dataFolder == null || config.dataFolder.isBlank()) {
            throw new IllegalArgumentException("预处理数据目录不能为空");
        }
        departureOffsetHours = config.resolveDepartureOffsetHours();

        try {
            baseDate = LocalDate.parse(config.navigationDate, DATE_FORMATTER);
        } catch (DateTimeParseException exception) {
            throw new IllegalArgumentException("导航日期格式非法，必须为 yyyyMMdd: " + config.navigationDate, exception);
        }

        departureHour = Double.isFinite(config.departureHour) ? config.departureHour : 0.0;
        if (departureHour < 0.0 || departureHour >= DAY_HOURS) {
            throw new IllegalArgumentException("出发时刻必须在 [0,24) 小时范围内，当前值=" + departureHour);
        }

        dataFolderPath = Path.of(config.dataFolder);
        if (!Files.isDirectory(dataFolderPath)) {
            throw new IOException("预处理数据目录不存在: " + dataFolderPath.toAbsolutePath());
        }

        validateGridPoint(config.grid.startX, config.grid.startY, "起点");
        validateGridPoint(config.grid.endX, config.grid.endY, "终点");
    }

    private void validateGridPoint(int x, int y, String label) {
        if (x < 1 || x > config.grid.nx || y < 1 || y > config.grid.ny) {
            throw new IllegalArgumentException(label + "超出网格范围: x=" + x + ", y=" + y);
        }
    }

    // ==================== 数据加载 ====================

    private void loadAllEnvironmentData() throws IOException {
        numDays = 0;
        for (int offset = 0; offset < 30; offset++) {
            LocalDate date = baseDate.plusDays(offset);
            String dateText = DATE_FORMATTER.format(date);
            if (Files.exists(dataFolderPath.resolve(dateText + "_wind.json"))
                    && Files.exists(dataFolderPath.resolve(dateText + "_cur.json"))
                    && Files.exists(dataFolderPath.resolve(dateText + "_wave.json"))) {
                numDays++;
            } else {
                break;
            }
        }

        if (numDays == 0) {
            throw new IOException("没有找到起始日期的环境数据: " + config.navigationDate);
        }

        System.out.println("三维分层导航: 检测到 " + numDays + " 天连续环境数据 ("
                + DATE_FORMATTER.format(baseDate) + " ~ "
                + DATE_FORMATTER.format(baseDate.plusDays(numDays - 1)) + ")");

        for (int offset = 0; offset < numDays; offset++) {
            dailySnapshots.add(loadSnapshot(baseDate.plusDays(offset)));
        }

        Path obstacleFile = dataFolderPath.resolve(
                config.obstacleFileName == null || config.obstacleFileName.isBlank()
                        ? "obstacle_map.json" : config.obstacleFileName);
        ObstacleOutput obstacleOutput = objectMapper.readValue(obstacleFile.toFile(), ObstacleOutput.class);
        validateGridConsistency(obstacleOutput.gridXY.nx, obstacleOutput.gridXY.ny, "障碍物");
        obstacleMap = obstacleOutput.obstacleMap;
        obstacleBufferMap = copyObstacleMap(obstacleMap);

        GridPoint start = buildStartPoint();
        GridPoint end = buildEndPoint();
        if (obstacleMap[start.i][start.j]) {
            throw new IllegalStateException("起点位于障碍物上: x=" + config.grid.startX + ", y=" + config.grid.startY);
        }
        if (obstacleMap[end.i][end.j]) {
            throw new IllegalStateException("终点位于障碍物上: x=" + config.grid.endX + ", y=" + config.grid.endY);
        }
    }

    private EnvironmentSnapshot loadSnapshot(LocalDate date) throws IOException {
        String dateText = DATE_FORMATTER.format(date);
        Path windFile = dataFolderPath.resolve(dateText + "_wind.json");
        Path currentFile = dataFolderPath.resolve(dateText + "_cur.json");
        Path waveFile = dataFolderPath.resolve(dateText + "_wave.json");

        WindOutput windOutput = objectMapper.readValue(windFile.toFile(), WindOutput.class);
        CurrentOutput currentOutput = objectMapper.readValue(currentFile.toFile(), CurrentOutput.class);
        WaveOutput waveOutput = objectMapper.readValue(waveFile.toFile(), WaveOutput.class);

        validateGridConsistency(windOutput.gridXY.nx, windOutput.gridXY.ny, "风场(" + dateText + ")");
        validateGridConsistency(currentOutput.gridXY.nx, currentOutput.gridXY.ny, "流场(" + dateText + ")");
        validateGridConsistency(waveOutput.gridXY.nx, waveOutput.gridXY.ny, "波浪(" + dateText + ")");

        return new EnvironmentSnapshot(
                to2DArray(windOutput.windDataU, config.grid.ny, config.grid.nx),
                to2DArray(windOutput.windDataV, config.grid.ny, config.grid.nx),
                to2DArray(currentOutput.currentDataU, config.grid.ny, config.grid.nx),
                to2DArray(currentOutput.currentDataV, config.grid.ny, config.grid.nx),
                to2DArray(waveOutput.waveDataD, config.grid.ny, config.grid.nx),
                to2DArray(waveOutput.waveDataP, config.grid.ny, config.grid.nx),
                to2DArray(waveOutput.waveDataH, config.grid.ny, config.grid.nx)
        );
    }

    private void validateGridConsistency(int nx, int ny, String label) {
        if (nx != config.grid.nx || ny != config.grid.ny) {
            throw new IllegalStateException(label + "网格尺寸与配置不一致: json=" + nx + "x" + ny
                    + ", config=" + config.grid.nx + "x" + config.grid.ny);
        }
    }

    private boolean[][] copyObstacleMap(boolean[][] source) {
        boolean[][] copy = new boolean[source.length][source[0].length];
        for (int i = 0; i < source.length; i++) {
            System.arraycopy(source[i], 0, copy[i], 0, source[i].length);
        }
        return copy;
    }

    private double[][] to2DArray(List<Double> data, int rows, int cols) {
        if (data == null || data.size() != rows * cols) {
            throw new IllegalStateException("列表长度与网格尺寸不匹配，期望=" + (rows * cols)
                    + "，实际=" + (data == null ? 0 : data.size()));
        }
        double[][] array = new double[rows][cols];
        int index = 0;
        for (int i = 0; i < rows; i++) {
            for (int j = 0; j < cols; j++) {
                Double value = data.get(index++);
                array[i][j] = value == null ? Double.NaN : value;
            }
        }
        return array;
    }

    // ==================== 网格 ====================

    private void calculateGridResolution() {
        deltaLat = (config.area.latMax - config.area.latMin) / Math.max(1, config.grid.ny - 1);
        deltaLon = (config.area.lonMax - config.area.lonMin) / Math.max(1, config.grid.nx - 1);

        double latCenter = (config.area.latMin + config.area.latMax) / 2.0;
        dLatNm = deltaLat * 60.0;
        dLonNm = deltaLon * 60.0 * Math.cos(Math.toRadians(latCenter));
        diagDist = Math.sqrt(dLatNm * dLatNm + dLonNm * dLonNm);
    }

    private void initializeDirectionDistances() {
        directionDistances = new double[directions.length];
        for (int d = 0; d < directions.length; d++) {
            if (Math.abs(directions[d][0]) == 1 && Math.abs(directions[d][1]) == 1) {
                directionDistances[d] = diagDist;
            } else if (directions[d][0] != 0) {
                directionDistances[d] = dLatNm;
            } else {
                directionDistances[d] = dLonNm;
            }
        }
    }

    // ==================== 三维 Dijkstra ====================

    private Dijkstra3DResult runTimeDijkstra3D() {
        int ny = config.grid.ny;
        int nx = config.grid.nx;
        GridPoint start = buildStartPoint();
        GridPoint end = buildEndPoint();

        double[][][] costArr = new double[numDays][ny][nx];
        int[][][] predI = new int[numDays][ny][nx];
        int[][][] predJ = new int[numDays][ny][nx];
        int[][][] predDay = new int[numDays][ny][nx];

        for (int d = 0; d < numDays; d++) {
            for (int i = 0; i < ny; i++) {
                Arrays.fill(costArr[d][i], Double.MAX_VALUE);
                Arrays.fill(predI[d][i], -1);
                Arrays.fill(predJ[d][i], -1);
                Arrays.fill(predDay[d][i], -1);
            }
        }

    costArr[0][start.i][start.j] = departureHour;

        PriorityQueue<State> pq = new PriorityQueue<>();
    pq.offer(new State(0, start.i, start.j, departureHour));

        State bestEnd = null;

        while (!pq.isEmpty()) {
            State cur = pq.poll();

            if (cur.totalCost > costArr[cur.day][cur.i][cur.j] + EPSILON) {
                continue;
            }

            if (cur.i == end.i && cur.j == end.j) {
                bestEnd = cur;
                break;
            }

            // --- 空间移动：累计航行时间跨 24h 时，直接切换到下一天环境继续航行 ---
            for (int d = 0; d < directions.length; d++) {
                int ni = cur.i + directions[d][0];
                int nj = cur.j + directions[d][1];
                if (ni < 0 || ni >= ny || nj < 0 || nj >= nx || obstacleBufferMap[ni][nj]) {
                    continue;
                }
                if (!GridRouteGraph.hasSupercoverClearance(cur.i, cur.j, ni, nj, ny, nx,
                        (row, col) -> obstacleBufferMap[row][col])) {
                    continue;
                }

                double segTime = calculateSegmentTime(cur.i, cur.j, d, cur.totalCost);
                if (!Double.isFinite(segTime) || segTime <= 0) {
                    continue;
                }

                double newCost = cur.totalCost + segTime;
                int nextDay = (int) Math.floor((departureOffsetHours + newCost) / DAY_HOURS);
                if (nextDay >= numDays) {
                    continue;
                }

                if (newCost + EPSILON < costArr[nextDay][ni][nj]) {
                    costArr[nextDay][ni][nj] = newCost;
                    predI[nextDay][ni][nj] = cur.i;
                    predJ[nextDay][ni][nj] = cur.j;
                    predDay[nextDay][ni][nj] = cur.day;
                    pq.offer(new State(nextDay, ni, nj, newCost));
                }
            }
        }

        if (bestEnd == null) {
            throw new IllegalStateException("三维分层导航计算失败，" + numDays + " 天内未找到可行路径");
        }

        // ---- 重建完整三维路径 ----
        List<int[]> fullPath3D = new ArrayList<>();
        int ci = bestEnd.i, cj = bestEnd.j, cd = bestEnd.day;
        while (ci != -1) {
            fullPath3D.add(0, new int[]{cd, ci, cj});
            int pi = predI[cd][ci][cj];
            int pj = predJ[cd][ci][cj];
            int pd = predDay[cd][ci][cj];
            ci = pi;
            cj = pj;
            cd = pd;
        }

        // 提取空间路径（同一空间点的跨天状态只保留一次）
        List<GridPoint> spatialPath = new ArrayList<>();
        for (int[] state : fullPath3D) {
            GridPoint pt = new GridPoint(state[1], state[2]);
            if (spatialPath.isEmpty() || !spatialPath.get(spatialPath.size() - 1).equals(pt)) {
                spatialPath.add(pt);
            }
        }

        return new Dijkstra3DResult(spatialPath, fullPath3D, bestEnd.totalCost);
    }

    // ==================== 段时间计算 ====================

    private double calculateSegmentTime(int i, int j, int directionIndex, double startElapsedHours) {
        double remainingDistance = directionDistances[directionIndex];
        double totalTime = 0.0;
        double currentElapsedHours = departureOffsetHours + startElapsedHours;

        for (int guard = 0; guard < 32 && remainingDistance > EPSILON; guard++) {
            int dayIndex = (int) Math.floor(Math.max(currentElapsedHours, 0.0) / DAY_HOURS);
            if (dayIndex >= numDays) {
                return Double.POSITIVE_INFINITY;
            }

            EnvironmentSnapshot snapshot = dailySnapshots.get(dayIndex);
            double fullSegmentTime = calculateSegmentTimeWithSnapshot(
                    snapshot, i, j, directionIndex, remainingDistance);
            if (!Double.isFinite(fullSegmentTime) || fullSegmentTime <= 0) {
                return Double.POSITIVE_INFINITY;
            }

            double nextBoundary = (Math.floor(currentElapsedHours / DAY_HOURS) + 1.0) * DAY_HOURS;
            double hoursUntilBoundary = nextBoundary - currentElapsedHours;
            if (hoursUntilBoundary <= EPSILON || fullSegmentTime <= hoursUntilBoundary + EPSILON) {
                totalTime += fullSegmentTime;
                return totalTime;
            }

            double speed = remainingDistance / fullSegmentTime;
            if (!Double.isFinite(speed) || speed <= 0) {
                return Double.POSITIVE_INFINITY;
            }

            double traversedDistance = Math.min(remainingDistance, speed * hoursUntilBoundary);
            if (traversedDistance <= EPSILON) {
                return Double.POSITIVE_INFINITY;
            }

            remainingDistance -= traversedDistance;
            totalTime += hoursUntilBoundary;
            currentElapsedHours += hoursUntilBoundary;
        }

        if (remainingDistance <= EPSILON) {
            return totalTime;
        }
        throw new IllegalStateException("单段航行跨越过多日期，无法完成时间积分");
    }

    private double calculateSegmentTimeWithSnapshot(EnvironmentSnapshot snapshot,
                                                    int i, int j,
                                                    int directionIndex,
                                                    double distance) {
        double bearing = bearings[directionIndex];

        double windU = safeValue(snapshot.windUData[i][j]);
        double windV = safeValue(snapshot.windVData[i][j]);
        double currentU = safeValue(snapshot.currentUData[i][j]);
        double currentV = safeValue(snapshot.currentVData[i][j]);

        double windSpeed = Math.sqrt(windU * windU + windV * windV);
        double currentSpeed = Math.sqrt(currentU * currentU + currentV * currentV);
        double windDir = calculateDirection(windU, windV);
        double currentDir = calculateDirection(currentU, currentV);

        ShipParameters ship = config.shipParameters;
        double time = ShipUtils.calculateTimeCost(
                distance, bearing,
                windSpeed, windDir,
                currentSpeed, currentDir,
            ship.shipSpeed,
                ship.shipLength, ship.shipWidth, ship.shipDraft, ship.shipDisplacement);

        if (Double.isFinite(time) && time > 0) {
            return time;
        }
        return distance / Math.max(ship.shipSpeed, 0.1);
    }

    // ==================== 工具方法 ====================

    private double safeValue(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private double calculateDirection(double u, double v) {
        if (Math.abs(u) < EPSILON && Math.abs(v) < EPSILON) {
            return 0.0;
        }
        double direction = Math.toDegrees(Math.atan2(v, u));
        return direction < 0 ? direction + 360.0 : direction;
    }

    private double calculatePathDistance(List<GridPoint> path) {
        if (path == null || path.size() < 2) {
            return 0.0;
        }
        double totalDistance = 0.0;
        for (int index = 0; index < path.size() - 1; index++) {
            GridPoint from = path.get(index);
            GridPoint to = path.get(index + 1);
            totalDistance += calculateSegmentDistance(from, to);
        }
        return totalDistance;
    }

    private double calculateSegmentDistance(GridPoint from, GridPoint to) {
        double latDistance = Math.abs(to.i - from.i) * dLatNm;
        double lonDistance = Math.abs(to.j - from.j) * dLonNm;
        return Math.sqrt(latDistance * latDistance + lonDistance * lonDistance);
    }

    private List<PathVisualizer.MarkerPoint> calculateDaySwitchPoints(List<GridPoint> path) {
        List<PathVisualizer.MarkerPoint> markers = new ArrayList<>();
    double elapsedHours = departureHour;
        for (int index = 0; index < path.size() - 1; index++) {
            GridPoint from = path.get(index);
            GridPoint to = path.get(index + 1);
            int directionIndex = findDirectionIndex(to.i - from.i, to.j - from.j);
            if (directionIndex < 0) {
                continue;
            }
            elapsedHours += appendDaySwitchPoints(markers, from, to, directionIndex, elapsedHours);
        }
        return markers;
    }

    private List<double[]> buildTimedPathPoints(List<GridPoint> path) {
        List<double[]> timedPoints = new ArrayList<>();
        if (path == null || path.isEmpty()) {
            return timedPoints;
        }

    double elapsedHours = departureHour;
        timedPoints.add(createTimedPathPoint(path.get(0), elapsedHours));

        for (int index = 0; index < path.size() - 1; index++) {
            GridPoint from = path.get(index);
            GridPoint to = path.get(index + 1);
            int directionIndex = findDirectionIndex(to.i - from.i, to.j - from.j);
            if (directionIndex < 0) {
                continue;
            }

            double segmentTime = calculateSegmentTime(from.i, from.j, directionIndex, elapsedHours);
            if (!Double.isFinite(segmentTime) || segmentTime <= 0) {
                throw new IllegalStateException("无法计算三维路径节点的累计时间");
            }

            elapsedHours += segmentTime;
            timedPoints.add(createTimedPathPoint(to, elapsedHours));
        }

        return timedPoints;
    }

    private double[] createTimedPathPoint(GridPoint point, double elapsedHours) {
        double absoluteElapsedHours = departureOffsetHours + elapsedHours;
        return new double[]{
                roundToSixDecimals(config.area.lonMin + point.j * deltaLon),
                roundToSixDecimals(config.area.latMin + point.i * deltaLat),
            roundToSixDecimals(absoluteElapsedHours / DAY_HOURS),
                roundToSixDecimals(elapsedHours),
                point.j,
                point.i
        };
    }

    private double appendDaySwitchPoints(List<PathVisualizer.MarkerPoint> markers,
                                         GridPoint from,
                                         GridPoint to,
                                         int directionIndex,
                                         double startElapsedHours) {
        double remainingDistance = directionDistances[directionIndex];
        double segmentDistance = remainingDistance;
        double traveledDistance = 0.0;
        double totalTime = 0.0;
        double currentElapsedHours = departureOffsetHours + startElapsedHours;

        for (int guard = 0; guard < 64 && remainingDistance > EPSILON; guard++) {
            int dayIndex = (int) Math.floor(Math.max(currentElapsedHours, 0.0) / DAY_HOURS);
            if (dayIndex >= numDays) {
                return totalTime;
            }

            EnvironmentSnapshot snapshot = dailySnapshots.get(dayIndex);
            double fullSegmentTime = calculateSegmentTimeWithSnapshot(snapshot, from.i, from.j, directionIndex, remainingDistance);
            if (!Double.isFinite(fullSegmentTime) || fullSegmentTime <= 0) {
                return totalTime;
            }

            double nextBoundary = (Math.floor(currentElapsedHours / DAY_HOURS) + 1.0) * DAY_HOURS;
            double hoursUntilBoundary = nextBoundary - currentElapsedHours;
            if (hoursUntilBoundary <= EPSILON || fullSegmentTime <= hoursUntilBoundary + EPSILON) {
                totalTime += fullSegmentTime;
                return totalTime;
            }

            double speed = remainingDistance / fullSegmentTime;
            if (!Double.isFinite(speed) || speed <= 0) {
                return totalTime;
            }

            double traversedDistance = Math.min(remainingDistance, speed * hoursUntilBoundary);
            traveledDistance += traversedDistance;
            double ratio = Math.max(0.0, Math.min(1.0, traveledDistance / segmentDistance));
            markers.add(createSwitchMarker(from, to, ratio, (int) Math.floor(nextBoundary / DAY_HOURS) + 1));

            remainingDistance -= traversedDistance;
            totalTime += hoursUntilBoundary;
            currentElapsedHours += hoursUntilBoundary;
        }
        return totalTime;
    }

    private PathVisualizer.MarkerPoint createSwitchMarker(GridPoint from, GridPoint to, double ratio, int dayNumber) {
        double gridX = from.j + (to.j - from.j) * ratio;
        double gridY = from.i + (to.i - from.i) * ratio;
        return new PathVisualizer.MarkerPoint(gridX, gridY, "D" + dayNumber);
    }

    private int findDirectionIndex(int di, int dj) {
        for (int index = 0; index < directions.length; index++) {
            if (directions[index][0] == di && directions[index][1] == dj) {
                return index;
            }
        }
        return -1;
    }

    // ==================== 结果输出 ====================

    private NavigationResult buildResult(double totalDistance, double totalTime) {
        NavigationResult result = new NavigationResult();
        result.projectName = "NavigationTime3D";
        result.calculationType = "shortTime3DLayered";
        result.navigationDate = config.navigationDate;
        result.area = config.area;
        result.grid = config.grid;
        result.shipParameters = config.shipParameters;
        result.totalDistance = roundToTwoDecimals(totalDistance);
        result.totalTime = roundToTwoDecimals(totalTime);
        result.pathPointCount = shortestTimePath.size();
        result.windFile = buildDataFileSummary("wind");
        result.currentFile = buildDataFileSummary("cur");
        result.waveFile = buildDataFileSummary("wave");
        result.obstacleFile = config.obstacleFileName == null || config.obstacleFileName.isBlank()
                ? "obstacle_map.json" : config.obstacleFileName;
        result.pathPoints = buildRoutePoints(shortestTimePath);
    result.daySwitchPoints = buildDaySwitchPoints(daySwitchPoints, result.pathPoints);

        PathEvaluation evaluation = evaluateFuelAndSafetyForPath(shortestTimePath);
        result.totalFuelEnergyJ = roundToSixDecimals(evaluation.totalFuelEnergyJ);
        result.totalSafetyScore = roundToSixDecimals(evaluation.avgSafetyScore);
        return result;
    }

    private PathEvaluation evaluateFuelAndSafetyForPath(List<GridPoint> path) {
        if (path == null || path.size() < 2) {
            return new PathEvaluation(0.0, 0.0);
        }

    double elapsedHours = departureHour;
        double totalFuelEnergyJ = 0.0;
        double weightedSafetySum = 0.0;
        double totalDistance = 0.0;

        Map<Long, Double> safetyIndexCache = new HashMap<>();

        for (int index = 0; index < path.size() - 1; index++) {
            GridPoint from = path.get(index);
            GridPoint to = path.get(index + 1);
            int directionIndex = findDirectionIndex(to.i - from.i, to.j - from.j);
            if (directionIndex < 0) {
                continue;
            }

            SegmentEvaluation segment = evaluateSegmentFuelSafety(from, to, directionIndex, elapsedHours, safetyIndexCache);
            elapsedHours += segment.totalHours;
            totalFuelEnergyJ += segment.fuelEnergyJ;
            weightedSafetySum += segment.weightedSafety;
            totalDistance += segment.distanceNm;
        }

        double avgSafety = totalDistance > EPSILON ? clamp(weightedSafetySum / totalDistance, 0.0, 1.0) : 0.0;
        return new PathEvaluation(totalFuelEnergyJ, avgSafety);
    }

    private SegmentEvaluation evaluateSegmentFuelSafety(GridPoint from,
                                                        GridPoint to,
                                                        int directionIndex,
                                                        double startElapsedHours,
                                                        Map<Long, Double> safetyIndexCache) {
        double remainingDistanceTime = directionDistances[directionIndex];
        double remainingDistancePhysical = calculateSegmentDistance(from, to);
        double totalHours = 0.0;
        double totalFuelEnergyJ = 0.0;
        double weightedSafety = 0.0;
        double currentElapsedHours = departureOffsetHours + startElapsedHours;
        double bearing = bearings[directionIndex];

        for (int guard = 0; guard < 32 && remainingDistanceTime > EPSILON; guard++) {
            int dayIndex = (int) Math.floor(Math.max(currentElapsedHours, 0.0) / DAY_HOURS);
            if (dayIndex >= numDays) {
                break;
            }

            EnvironmentSnapshot snapshot = dailySnapshots.get(dayIndex);
            double fullSegmentHours = calculateSegmentTimeWithSnapshot(snapshot, from.i, from.j, directionIndex, remainingDistanceTime);
            if (!Double.isFinite(fullSegmentHours) || fullSegmentHours <= 0) {
                break;
            }

            double nextBoundary = (Math.floor(currentElapsedHours / DAY_HOURS) + 1.0) * DAY_HOURS;
            double hoursUntilBoundary = nextBoundary - currentElapsedHours;
            boolean completeInCurrentDay = hoursUntilBoundary <= EPSILON || fullSegmentHours <= hoursUntilBoundary + EPSILON;

            double stepHours = completeInCurrentDay ? fullSegmentHours : hoursUntilBoundary;
            double fraction = clamp(stepHours / fullSegmentHours, 0.0, 1.0);
            double stepDistanceTime = remainingDistanceTime * fraction;
            double stepDistancePhysical = remainingDistancePhysical * fraction;
            if (stepHours <= EPSILON || stepDistanceTime <= EPSILON) {
                break;
            }

            double windU = safeValue(snapshot.windUData[from.i][from.j]);
            double windV = safeValue(snapshot.windVData[from.i][from.j]);
            double currentU = safeValue(snapshot.currentUData[from.i][from.j]);
            double currentV = safeValue(snapshot.currentVData[from.i][from.j]);
            double windSpeed = Math.sqrt(windU * windU + windV * windV);
            double currentSpeed = Math.sqrt(currentU * currentU + currentV * currentV);
            double windDir = calculateDirection(windU, windV);
            double currentDir = calculateDirection(currentU, currentV);

            double fuelEnergyJ = FuelCostCalculator.calculateFuelEnergyJ(
                    config.shipParameters,
                    windSpeed,
                    windDir,
                    currentSpeed,
                    currentDir,
                    currentU,
                    currentV,
                    bearing,
                    stepHours
            );
            if (Double.isFinite(fuelEnergyJ) && fuelEnergyJ >= 0.0) {
                totalFuelEnergyJ += fuelEnergyJ;
            }

            double safetyIndex = getOrComputeSafetyIndex(safetyIndexCache, dayIndex, from.i, from.j, directionIndex,
                    windU, windV, currentU, currentV,
                    Math.max(0.0, safeValue(snapshot.waveHeightData[from.i][from.j])),
                    safeValue(snapshot.waveDirData[from.i][from.j]),
                    Math.max(0.1, safeValue(snapshot.wavePeriodData[from.i][from.j])),
                    bearing);
            weightedSafety += safetyIndex * stepDistancePhysical;

            totalHours += stepHours;
            remainingDistanceTime -= stepDistanceTime;
            remainingDistancePhysical -= stepDistancePhysical;
            currentElapsedHours += stepHours;

            if (completeInCurrentDay) {
                break;
            }
        }

        double segmentDistance = calculateSegmentDistance(from, to);
        return new SegmentEvaluation(totalHours, totalFuelEnergyJ, weightedSafety, segmentDistance);
    }

    private double getOrComputeSafetyIndex(Map<Long, Double> cache,
                                           int dayIndex,
                                           int i,
                                           int j,
                                           int directionIndex,
                                           double windU,
                                           double windV,
                                           double currentU,
                                           double currentV,
                                           double waveHeight,
                                           double waveDir,
                                           double wavePeriod,
                                           double bearing) {
        long key = (((long) dayIndex) << 48) ^ (((long) i) << 32) ^ (((long) j) << 8) ^ (directionIndex & 0xFFL);
        Double cached = cache.get(key);
        if (cached != null) {
            return cached;
        }

        double safetyIndex = SafetyCostCalculator.calculateSafetyIndex(
                windU,
                windV,
                currentU,
                currentV,
                waveHeight,
                waveDir,
                bearing,
                wavePeriod,
                config.shipParameters
        );
        if (!Double.isFinite(safetyIndex)) {
            safetyIndex = 0.0;
        }
        safetyIndex = clamp(safetyIndex, 0.0, 1.0);
        cache.put(key, safetyIndex);
        return safetyIndex;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private static final class PathEvaluation {
        private final double totalFuelEnergyJ;
        private final double avgSafetyScore;

        private PathEvaluation(double totalFuelEnergyJ, double avgSafetyScore) {
            this.totalFuelEnergyJ = totalFuelEnergyJ;
            this.avgSafetyScore = avgSafetyScore;
        }
    }

    private static final class SegmentEvaluation {
        private final double totalHours;
        private final double fuelEnergyJ;
        private final double weightedSafety;
        private final double distanceNm;

        private SegmentEvaluation(double totalHours, double fuelEnergyJ, double weightedSafety, double distanceNm) {
            this.totalHours = totalHours;
            this.fuelEnergyJ = fuelEnergyJ;
            this.weightedSafety = weightedSafety;
            this.distanceNm = distanceNm;
        }
    }

    private String buildDataFileSummary(String suffix) {
        List<String> fileNames = new ArrayList<>();
        for (int offset = 0; offset < daysUsed; offset++) {
            fileNames.add(DATE_FORMATTER.format(baseDate.plusDays(offset)) + "_" + suffix + ".json");
        }
        return String.join(" -> ", fileNames);
    }

    private List<RoutePoint> buildRoutePoints(List<GridPoint> path) {
        List<RoutePoint> points = new ArrayList<>(path.size());
        for (int index = 0; index < path.size(); index++) {
            GridPoint point = path.get(index);
            RoutePoint routePoint = new RoutePoint();
            routePoint.index = index;
            routePoint.x = point.j + 1;
            routePoint.y = point.i + 1;
            routePoint.lon = roundToSixDecimals(config.area.lonMin + point.j * deltaLon);
            routePoint.lat = roundToSixDecimals(config.area.latMin + point.i * deltaLat);
            points.add(routePoint);
        }
        return points;
    }

    private List<NavigationResult.DaySwitchPoint> buildDaySwitchPoints(List<PathVisualizer.MarkerPoint> markers,
                                                                        List<RoutePoint> routePoints) {
        List<NavigationResult.DaySwitchPoint> points = new ArrayList<>();
        if (markers == null || markers.isEmpty()) {
            return points;
        }

        for (PathVisualizer.MarkerPoint marker : markers) {
            NavigationResult.DaySwitchPoint point = new NavigationResult.DaySwitchPoint();
            point.label = marker.label;
            point.dayNumber = parseDayNumber(marker.label);
            point.gridX = roundToSixDecimals(marker.gridX + 1.0);
            point.gridY = roundToSixDecimals(marker.gridY + 1.0);
            point.lon = roundToSixDecimals(config.area.lonMin + marker.gridX * deltaLon);
            point.lat = roundToSixDecimals(config.area.latMin + marker.gridY * deltaLat);
            point.nearestPathIndex = findNearestPathIndex(marker, routePoints);
            points.add(point);
        }
        return points;
    }

    private Integer findNearestPathIndex(PathVisualizer.MarkerPoint marker, List<RoutePoint> routePoints) {
        if (routePoints == null || routePoints.isEmpty()) {
            return null;
        }

        double targetX = marker.gridX + 1.0;
        double targetY = marker.gridY + 1.0;
        double bestDistance = Double.MAX_VALUE;
        Integer bestIndex = null;
        for (RoutePoint routePoint : routePoints) {
            double dx = routePoint.x - targetX;
            double dy = routePoint.y - targetY;
            double distance = dx * dx + dy * dy;
            if (distance < bestDistance) {
                bestDistance = distance;
                bestIndex = routePoint.index;
            }
        }
        return bestIndex;
    }

    private Integer parseDayNumber(String label) {
        if (label == null || label.isBlank() || !label.startsWith("D")) {
            return null;
        }
        try {
            return Integer.parseInt(label.substring(1));
        } catch (NumberFormatException exception) {
            return null;
        }
    }

    private void writeResult(NavigationResult result) throws IOException {
        if (config.outputFile == null || config.outputFile.isBlank()) {
            return;
        }
        Path outputPath = Path.of(config.outputFile);
        if (outputPath.getParent() != null) {
            Files.createDirectories(outputPath.getParent());
        }
        objectMapper.writeValue(outputPath.toFile(), result);
    }

    private void exportVisualization(double totalDistance, double totalTime, boolean openBrowser) {
        try {
            Path outputPath = resolveVisualizationOutputPath();
            NavigationTime3DHtmlVisualizer visualizer = new NavigationTime3DHtmlVisualizer(objectMapper);
            Path htmlPath = visualizer.writeVisualization(
                    outputPath,
                    config.navigationDate,
                    config.area,
                    obstacleMap,
                    obstacleBufferMap,
                    timedPathPoints,
                    daySwitchPoints,
                    deltaLat,
                    deltaLon,
                    daysUsed,
                    numDays,
                    totalDistance,
                    totalTime
            );
            System.out.println("三维可视化页面已生成: " + htmlPath.toAbsolutePath());
            if (openBrowser) {
                visualizer.openInBrowser(htmlPath);
            }
        } catch (IOException exception) {
            throw new IllegalStateException("生成三维可视化页面失败", exception);
        }
    }

    private Path resolveVisualizationOutputPath() {
        if (config.outputFile == null || config.outputFile.isBlank()) {
            return Path.of("NavigationTime3D_visualization.html");
        }

        Path outputPath = Path.of(config.outputFile);
        String fileName = outputPath.getFileName().toString();
        int extensionIndex = fileName.lastIndexOf('.');
        String visualizationFileName = extensionIndex >= 0
                ? fileName.substring(0, extensionIndex) + "_visualization.html"
                : fileName + "_visualization.html";
        return outputPath.resolveSibling(visualizationFileName);
    }

    // ==================== 坐标辅助 ====================

    private GridPoint buildStartPoint() {
        return new GridPoint(config.grid.startY - 1, config.grid.startX - 1);
    }

    private GridPoint buildEndPoint() {
        return new GridPoint(config.grid.endY - 1, config.grid.endX - 1);
    }

    private double roundToTwoDecimals(double value) {
        return Math.round(value * 100.0) / 100.0;
    }

    private double roundToSixDecimals(double value) {
        return Math.round(value * 1_000_000.0) / 1_000_000.0;
    }

    // ==================== 内部类 ====================

    private static class EnvironmentSnapshot {
        final double[][] windUData;
        final double[][] windVData;
        final double[][] currentUData;
        final double[][] currentVData;
        final double[][] waveDirData;
        final double[][] wavePeriodData;
        final double[][] waveHeightData;

        EnvironmentSnapshot(double[][] windUData, double[][] windVData,
                            double[][] currentUData, double[][] currentVData,
                            double[][] waveDirData, double[][] wavePeriodData,
                            double[][] waveHeightData) {
            this.windUData = windUData;
            this.windVData = windVData;
            this.currentUData = currentUData;
            this.currentVData = currentVData;
            this.waveDirData = waveDirData;
            this.wavePeriodData = wavePeriodData;
            this.waveHeightData = waveHeightData;
        }
    }

    private static class State implements Comparable<State> {
        final int day;
        final int i;
        final int j;
        final double totalCost;

        State(int day, int i, int j, double totalCost) {
            this.day = day;
            this.i = i;
            this.j = j;
            this.totalCost = totalCost;
        }

        @Override
        public int compareTo(State other) {
            return Double.compare(this.totalCost, other.totalCost);
        }
    }

    private static class Dijkstra3DResult {
        final List<GridPoint> spatialPath;
        final List<int[]> fullPath3D;
        final double totalTime;

        Dijkstra3DResult(List<GridPoint> spatialPath, List<int[]> fullPath3D, double totalTime) {
            this.spatialPath = spatialPath;
            this.fullPath3D = fullPath3D;
            this.totalTime = totalTime;
        }

        int getDaysUsed() {
            if (fullPath3D.isEmpty()) return 0;
            return fullPath3D.get(fullPath3D.size() - 1)[0] + 1;
        }
    }
}
