package Navigation.Navigation3D;

import Navigation.PathVisualizer;
import Navigation.model.GridPoint;
import Navigation.model.GridRouteGraph;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.Normalization.DirectionalFlowNormalizerSupport;
import Navigation.Navigation3D.Normalization.FuelNormalizer;
import Navigation.Navigation3D.Normalization.Normalizers;
import Navigation.Navigation3D.Normalization.TimeNormalizer;
import Navigation.object.NavigationConfig;
import Navigation.object.NavigationResult;
import Navigation.object.RoutePoint;
import Navigation.object.DenseTrajectoryPoint;
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
 * 三维分层多日导航（仅用于安全最优/油耗最优）。
 *
 * 与 NavigationTime3D 的区别：
 * 1) 不生成/展示 3D 页面。
 * 2) 目标函数可选：安全最优、油耗最优。
 */
public class NavigationObjective3D {

    public enum ObjectiveMode {
        SAFETY,
        FUEL,
        COMPOSITE
    }

    private static final double DAY_HOURS = 24.0;
    private static final double EPSILON = 1e-9;
    private static final DateTimeFormatter DATE_FORMATTER = DateTimeFormatter.BASIC_ISO_DATE;

    private final NavigationConfig config;
    private final ObjectiveMode objectiveMode;
    private final ObjectiveWeights objectiveWeights;
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
    private List<GridPoint> bestPath = new ArrayList<>();
    private List<PathVisualizer.MarkerPoint> daySwitchPoints = new ArrayList<>();
    private int daysUsed;
    private double departureOffsetHours;
    private double[][][][] safetyCache;
    private Map<Long, Double> safetyIndexCacheMap;
    private TimeNormalizer timeNormalizer;
    private FuelNormalizer fuelNormalizer;

    private final int[][] directions = {
            {0, 1}, {0, -1}, {1, 0}, {-1, 0},
            {1, 1}, {1, -1}, {-1, 1}, {-1, -1}
    };

    private final double[] bearings = {90, 270, 0, 180, 45, 315, 135, 225};

    public NavigationObjective3D(NavigationConfig config, ObjectiveMode objectiveMode) {
        this(config, objectiveMode, null);
    }

    public NavigationObjective3D(NavigationConfig config,
                                 ObjectiveMode objectiveMode,
                                 ObjectiveWeights objectiveWeights) {
        this.config = config;
        this.objectiveMode = objectiveMode;
        if (objectiveMode == ObjectiveMode.COMPOSITE) {
            if (objectiveWeights == null) {
                throw new IllegalArgumentException("综合目标模式需要提供安全/时间/油耗权重");
            }
            this.objectiveWeights = objectiveWeights.normalized();
        } else {
            this.objectiveWeights = objectiveWeights;
        }
        this.objectMapper = new ObjectMapper().enable(SerializationFeature.INDENT_OUTPUT);
    }

    public NavigationResult execute() throws Exception {
        validateConfig();
        loadAllEnvironmentData();
        calculateGridResolution();
        initializeDirectionDistances();
        initializeSafetyCacheIfNeeded();
        initializeNormalizerIfNeeded();

        // SAFETY 优化目标是累计风险暴露 E = Σ(risk·d)，保证边代价非负且可加，
        // 满足 Dijkstra 的标号设定。报告中的 S = 1 − Σ(risk·d)/Σd 是距离加权平均值；
        // 它是描述性指标，路线长度不同时未必与累计暴露给出相同排序。
        // 未采用 Dinkelbach 位移目标 Σ(risk·d) − λ·Σd：λ>0 时边代价可为负，
        // 时间分层状态 (day,i,j) 允许同日重复访问 ⇒ 存在负环，标号设定失效且可能产生绕圈路径。
        ObjectiveResult objectiveResult = runObjectiveDijkstra3D();
        bestPath = objectiveResult.path;
        daysUsed = objectiveResult.daysUsed;

        double totalDistance = calculatePathDistance(bestPath);
        double totalTime = objectiveResult.totalTimeHours;

        daySwitchPoints = calculateDaySwitchPoints(bestPath);

        NavigationResult result = buildResult(totalDistance, totalTime, objectiveResult.totalObjective);
        writeResult(result);
        return result;
    }

    public List<GridPoint> getBestPath() {
        return new ArrayList<>(bestPath);
    }

    public int getDaysUsed() {
        return daysUsed;
    }

    public int getNumDays() {
        return numDays;
    }

    public List<PathVisualizer.MarkerPoint> getDaySwitchPoints() {
        return new ArrayList<>(daySwitchPoints);
    }

    private List<PathVisualizer.MarkerPoint> calculateDaySwitchPoints(List<GridPoint> path) {
        List<PathVisualizer.MarkerPoint> markers = new ArrayList<>();
        if (path == null || path.size() < 2) {
            return markers;
        }

        double elapsedHours = 0.0;
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

    private void initializeSafetyCacheIfNeeded() {
        if (objectiveMode != ObjectiveMode.SAFETY && objectiveMode != ObjectiveMode.COMPOSITE) {
            return;
        }
        safetyCache = new double[numDays][config.grid.ny][config.grid.nx][directions.length];
        for (int day = 0; day < numDays; day++) {
            EnvironmentSnapshot snapshot = dailySnapshots.get(day);
            for (int i = 0; i < config.grid.ny; i++) {
                for (int j = 0; j < config.grid.nx; j++) {
                    if (obstacleMap != null && obstacleMap[i][j]) {
                        Arrays.fill(safetyCache[day][i][j], 0.0);
                        continue;
                    }
                    for (int directionIndex = 0; directionIndex < directions.length; directionIndex++) {
                        double safetyIndex = SafetyCostCalculator.calculateSafetyIndex(
                                safeValue(snapshot.windUData[i][j]),
                                safeValue(snapshot.windVData[i][j]),
                                safeValue(snapshot.currentUData[i][j]),
                                safeValue(snapshot.currentVData[i][j]),
                                Math.max(0.0, safeValue(snapshot.waveHeightData[i][j])),
                                safeValue(snapshot.waveDirData[i][j]),
                                bearings[directionIndex],
                                Math.max(0.1, safeValue(snapshot.wavePeriodData[i][j])),
                                config.shipParameters
                        );

                        if (!Double.isFinite(safetyIndex)) {
                            safetyIndex = 0.0;
                        }
                        safetyCache[day][i][j][directionIndex] = clamp(safetyIndex, 0.0, 1.0);
                    }
                }
            }
        }
    }

    private void initializeNormalizerIfNeeded() {
        if (objectiveMode != ObjectiveMode.COMPOSITE) {
            return;
        }

        if (!useCompositeTimeNormalization() && !useCompositeFuelNormalization()) {
            timeNormalizer = null;
            fuelNormalizer = null;
            return;
        }

        Normalizers.Bundle bundle = Normalizers.build(
                config,
                config.shipParameters,
                directions,
                bearings,
                new Normalizers.EnvironmentSampleProvider() {
                    @Override
                    public int numDays() { return numDays; }
                    @Override
                    public int ny() { return config.grid.ny; }
                    @Override
                    public int nx() { return config.grid.nx; }
                    @Override
                    public boolean isObstacle(int i, int j) {
                        return obstacleMap != null && obstacleMap[i][j];
                    }
                    @Override
                    public double windU(int day, int i, int j) { return safeValue(dailySnapshots.get(day).windUData[i][j]); }
                    @Override
                    public double windV(int day, int i, int j) { return safeValue(dailySnapshots.get(day).windVData[i][j]); }
                    @Override
                    public double currentU(int day, int i, int j) { return safeValue(dailySnapshots.get(day).currentUData[i][j]); }
                    @Override
                    public double currentV(int day, int i, int j) { return safeValue(dailySnapshots.get(day).currentVData[i][j]); }
                    @Override
                    public double waveHeight(int day, int i, int j) { return safeValue(dailySnapshots.get(day).waveHeightData[i][j]); }
                    @Override
                    public double wavePeriod(int day, int i, int j) { return safeValue(dailySnapshots.get(day).wavePeriodData[i][j]); }
                    @Override
                    public double waveDir(int day, int i, int j) { return safeValue(dailySnapshots.get(day).waveDirData[i][j]); }
                }
        );

        timeNormalizer = bundle.timeNormalizer();
        fuelNormalizer = bundle.fuelNormalizer();
    }

    private ObjectiveResult runObjectiveDijkstra3D() {
        int ny = config.grid.ny;
        int nx = config.grid.nx;
        GridPoint start = buildStartPoint();
        GridPoint end = buildEndPoint();

        double[][][] objectiveArr = new double[numDays][ny][nx];
        double[][][] elapsedHoursArr = new double[numDays][ny][nx];
        int[][][] predI = new int[numDays][ny][nx];
        int[][][] predJ = new int[numDays][ny][nx];
        int[][][] predDay = new int[numDays][ny][nx];

        for (int d = 0; d < numDays; d++) {
            for (int i = 0; i < ny; i++) {
                Arrays.fill(objectiveArr[d][i], Double.MAX_VALUE);
                Arrays.fill(elapsedHoursArr[d][i], Double.MAX_VALUE);
                Arrays.fill(predI[d][i], -1);
                Arrays.fill(predJ[d][i], -1);
                Arrays.fill(predDay[d][i], -1);
            }
        }

        objectiveArr[0][start.i][start.j] = 0.0;
        elapsedHoursArr[0][start.i][start.j] = 0.0;

        PriorityQueue<State> pq = new PriorityQueue<>();
        pq.offer(new State(0, start.i, start.j, 0.0, 0.0));

        State bestEnd = null;

        while (!pq.isEmpty()) {
            State cur = pq.poll();
            if (cur.totalObjective > objectiveArr[cur.day][cur.i][cur.j] + EPSILON) {
                continue;
            }

            if (cur.i == end.i && cur.j == end.j) {
                bestEnd = cur;
                break;
            }

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

                SegmentMetrics segment = calculateSegmentMetrics(cur.i, cur.j, d, cur.totalTimeHours);
                if (!Double.isFinite(segment.timeHours) || segment.timeHours <= 0
                        || !Double.isFinite(segment.objectiveCost) || segment.objectiveCost < 0) {
                    continue;
                }

                double newTime = cur.totalTimeHours + segment.timeHours;
                int nextDay = (int) Math.floor((departureOffsetHours + newTime) / DAY_HOURS);
                if (nextDay >= numDays) {
                    continue;
                }

                double newObjective = cur.totalObjective + segment.objectiveCost;
                if (newObjective + EPSILON < objectiveArr[nextDay][ni][nj]) {
                    objectiveArr[nextDay][ni][nj] = newObjective;
                    elapsedHoursArr[nextDay][ni][nj] = newTime;
                    predI[nextDay][ni][nj] = cur.i;
                    predJ[nextDay][ni][nj] = cur.j;
                    predDay[nextDay][ni][nj] = cur.day;
                    pq.offer(new State(nextDay, ni, nj, newObjective, newTime));
                }
            }
        }

        if (bestEnd == null) {
            throw new IllegalStateException("三维分层导航计算失败，" + numDays + " 天内未找到可行路径");
        }

        List<int[]> fullPath3D = new ArrayList<>();
        int ci = bestEnd.i;
        int cj = bestEnd.j;
        int cd = bestEnd.day;
        while (ci != -1) {
            fullPath3D.add(0, new int[]{cd, ci, cj});
            int pi = predI[cd][ci][cj];
            int pj = predJ[cd][ci][cj];
            int pd = predDay[cd][ci][cj];
            ci = pi;
            cj = pj;
            cd = pd;
        }

        List<GridPoint> spatialPath = new ArrayList<>();
        for (int[] state : fullPath3D) {
            GridPoint pt = new GridPoint(state[1], state[2]);
            if (spatialPath.isEmpty() || !spatialPath.get(spatialPath.size() - 1).equals(pt)) {
                spatialPath.add(pt);
            }
        }

        int finalDay = bestEnd.day;
        double finalHours = elapsedHoursArr[finalDay][bestEnd.i][bestEnd.j];
        int usedDays = Math.max(1, (int) Math.floor((departureOffsetHours + finalHours) / DAY_HOURS) + 1);
        return new ObjectiveResult(spatialPath, bestEnd.totalObjective, finalHours, usedDays);
    }

    private SegmentMetrics calculateSegmentMetrics(int i, int j, int directionIndex, double startElapsedHours) {
        double remainingDistanceNm = directionDistances[directionIndex];
        double totalHours = 0.0;
        double objectiveCost = 0.0;
        double safetyRiskDistance = 0.0;
        double segmentDistanceNm = 0.0;
        double currentElapsedHours = departureOffsetHours + startElapsedHours;
        double bearing = bearings[directionIndex];
        boolean useTimeNorm = useCompositeTimeNormalization();
        boolean useFuelNorm = useCompositeFuelNormalization();

        for (int guard = 0; guard < 32 && remainingDistanceNm > EPSILON; guard++) {
            int dayIndex = (int) Math.floor(Math.max(currentElapsedHours, 0.0) / DAY_HOURS);
            if (dayIndex >= numDays) {
                return SegmentMetrics.invalid();
            }

            EnvironmentSnapshot snapshot = dailySnapshots.get(dayIndex);
            double fullSegmentHours = calculateSegmentTimeWithSnapshot(snapshot, i, j, directionIndex, remainingDistanceNm);
            if (!Double.isFinite(fullSegmentHours) || fullSegmentHours <= 0) {
                return SegmentMetrics.invalid();
            }

            double nextBoundary = (Math.floor(currentElapsedHours / DAY_HOURS) + 1.0) * DAY_HOURS;
            double hoursUntilBoundary = nextBoundary - currentElapsedHours;
            boolean completeInCurrentDay = hoursUntilBoundary <= EPSILON || fullSegmentHours <= hoursUntilBoundary + EPSILON;

            double stepHours = completeInCurrentDay ? fullSegmentHours : hoursUntilBoundary;
            double stepDistanceNm = completeInCurrentDay
                    ? remainingDistanceNm
                    : remainingDistanceNm * (stepHours / fullSegmentHours);

            if (stepDistanceNm <= EPSILON || stepHours <= EPSILON) {
                return SegmentMetrics.invalid();
            }
            segmentDistanceNm += stepDistanceNm;

            if (objectiveMode == ObjectiveMode.SAFETY || objectiveMode == ObjectiveMode.COMPOSITE) {
                double safetyIndex = getOrComputeSafetyIndex(dayIndex, i, j, directionIndex);
                double risk = clamp(1.0 - safetyIndex, 0.0, 1.0);
                if (objectiveMode == ObjectiveMode.SAFETY) {
                    // SAFETY：按距离累加累计风险暴露 Σ(risk·d)，日边界处按步长距离拆分。
                    safetyRiskDistance += risk * stepDistanceNm;
                } else {
                    double fuelCost = calculateCompositeFuelCostWithSnapshot(snapshot, i, j, bearing, stepHours);
                    if (!Double.isFinite(fuelCost) || fuelCost < 0.0) {
                        fuelCost = stepDistanceNm * 1_000_000.0;
                    }

                    double safetyRisk = clamp(1.0 - safetyIndex, 0.0, 1.0);
                    // 归一化风险密度必须按距离积分；否则代价会随边/日切片数量变化。
                    double safetyTerm = safetyRisk * stepDistanceNm;
                    double timeTerm = (useTimeNorm && timeNormalizer != null)
                        ? timeNormalizer.normalize(stepHours, stepDistanceNm, directionIndex) * stepDistanceNm
                        : stepHours;
                    double fuelTerm = (useFuelNorm && fuelNormalizer != null)
                        ? fuelNormalizer.normalize(fuelCost, stepDistanceNm, directionIndex) * stepDistanceNm
                        : fuelCost;

                    objectiveCost += objectiveWeights.safety * safetyTerm
                        + objectiveWeights.time * timeTerm
                        + objectiveWeights.fuel * fuelTerm;
                }
            } else {
                double windU = safeValue(snapshot.windUData[i][j]);
                double windV = safeValue(snapshot.windVData[i][j]);
                double currentU = safeValue(snapshot.currentUData[i][j]);
                double currentV = safeValue(snapshot.currentVData[i][j]);
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
                if (Double.isFinite(fuelEnergyJ) && fuelEnergyJ >= 0) {
                    objectiveCost += fuelEnergyJ;
                } else {
                    objectiveCost += stepDistanceNm * 1_000_000.0;
                }
            }

            totalHours += stepHours;
            remainingDistanceNm -= stepDistanceNm;
            currentElapsedHours += stepHours;

            if (completeInCurrentDay) {
                break;
            }
        }

        if (remainingDistanceNm > EPSILON) {
            return SegmentMetrics.invalid();
        }

        if (objectiveMode == ObjectiveMode.SAFETY) {
            objectiveCost += safetyRiskDistance;
        }

        return new SegmentMetrics(totalHours, objectiveCost, segmentDistanceNm);
    }

    private double getOrComputeSafetyIndex(int dayIndex,
                                           int i,
                                           int j,
                                           int directionIndex) {
        if (dayIndex < 0 || dayIndex >= numDays) {
            return 0.0;
        }
        if (i < 0 || i >= config.grid.ny || j < 0 || j >= config.grid.nx) {
            return 0.0;
        }
        if (directionIndex < 0 || directionIndex >= directions.length) {
            return 0.0;
        }

        // SAFETY/COMPOSITE 模式可能会预计算整张图的 safetyCache；FUEL 模式为了节省内存通常不初始化。
        // 但为了统一输出“平均安全指数”，这里对未初始化的情况做懒计算 + 小缓存兜底。
        if (safetyCache != null) {
            return safetyCache[dayIndex][i][j][directionIndex];
        }

        if (obstacleMap != null && obstacleMap[i][j]) {
            return 0.0;
        }

        if (safetyIndexCacheMap == null) {
            safetyIndexCacheMap = new HashMap<>();
        }

        long key = (((long) dayIndex & 0xFFFFL) << 48)
            | (((long) i & 0xFFFFL) << 32)
            | (((long) j & 0xFFFFL) << 16)
            | ((long) directionIndex & 0xFFFFL);
        Double cached = safetyIndexCacheMap.get(key);
        if (cached != null) {
            return cached;
        }

        EnvironmentSnapshot snapshot = dailySnapshots.get(dayIndex);
        double safetyIndex = SafetyCostCalculator.calculateSafetyIndex(
            safeValue(snapshot.windUData[i][j]),
            safeValue(snapshot.windVData[i][j]),
            safeValue(snapshot.currentUData[i][j]),
            safeValue(snapshot.currentVData[i][j]),
            Math.max(0.0, safeValue(snapshot.waveHeightData[i][j])),
            safeValue(snapshot.waveDirData[i][j]),
            bearings[directionIndex],
            Math.max(0.1, safeValue(snapshot.wavePeriodData[i][j])),
            config.shipParameters
        );
        if (!Double.isFinite(safetyIndex)) {
            safetyIndex = 0.0;
        }
        safetyIndex = clamp(safetyIndex, 0.0, 1.0);
        safetyIndexCacheMap.put(key, safetyIndex);
        return safetyIndex;
    }

    private double calculateFuelEnergyWithSnapshot(EnvironmentSnapshot snapshot,
                                                   int i,
                                                   int j,
                                                   double bearing,
                                                   double segmentHours) {
        double windU = safeValue(snapshot.windUData[i][j]);
        double windV = safeValue(snapshot.windVData[i][j]);
        double currentU = safeValue(snapshot.currentUData[i][j]);
        double currentV = safeValue(snapshot.currentVData[i][j]);
        double windSpeed = Math.sqrt(windU * windU + windV * windV);
        double currentSpeed = Math.sqrt(currentU * currentU + currentV * currentV);
        double windDir = calculateDirection(windU, windV);
        double currentDir = calculateDirection(currentU, currentV);

        return FuelCostCalculator.calculateFuelEnergyJ(
                config.shipParameters,
                windSpeed,
                windDir,
                currentSpeed,
                currentDir,
                currentU,
                currentV,
                bearing,
            segmentHours,
            useCompositeShipClassCorrection()
        );
    }

    private double calculateCompositeFuelCostWithSnapshot(EnvironmentSnapshot snapshot,
                                                          int i,
                                                          int j,
                                                          double bearing,
                                                          double segmentHours) {
        return calculateFuelEnergyWithSnapshot(snapshot, i, j, bearing, segmentHours);
    }

    private double calculateSegmentTimeWithSnapshot(EnvironmentSnapshot snapshot,
                                                    int i,
                                                    int j,
                                                    int directionIndex,
                                                    double distanceNm) {
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
                distanceNm, bearing,
                windSpeed, windDir,
                currentSpeed, currentDir,
                ship.shipSpeed,
            ship.shipLength, ship.shipWidth, ship.shipDraft, ship.shipDisplacement,
            useCompositeShipClassCorrection());

        if (Double.isFinite(time) && time > 0) {
            return time;
        }

        double fallbackSpeedKnot = Math.max(ship.shipSpeed, 0.1);
        return distanceNm / fallbackSpeedKnot;
    }

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

    private int findDirectionIndex(int di, int dj) {
        for (int index = 0; index < directions.length; index++) {
            if (directions[index][0] == di && directions[index][1] == dj) {
                return index;
            }
        }
        return -1;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private boolean useCompositeTimeNormalization() {
        return objectiveMode == ObjectiveMode.COMPOSITE
                && config.enableCompositeNormalization
                && config.enableCompositeTimeNormalization;
    }

    private boolean useCompositeFuelNormalization() {
        return objectiveMode == ObjectiveMode.COMPOSITE
                && config.enableCompositeNormalization
                && config.enableCompositeFuelNormalization;
    }

    private boolean useCompositeSafetyNormalization() {
        return objectiveMode == ObjectiveMode.COMPOSITE
                && config.enableCompositeNormalization
                && config.enableCompositeSafetyNormalization;
    }

    private boolean useCompositeShipClassCorrection() {
        return objectiveMode != ObjectiveMode.COMPOSITE || config.enableCompositeShipClassCorrection;
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

    private NavigationResult buildResult(double totalDistance, double totalTime, double totalObjective) {
        NavigationResult result = new NavigationResult();
        if (objectiveMode == ObjectiveMode.SAFETY) {
            result.projectName = "NavigationSafety3D";
            result.calculationType = "safetyOptimal3DLayered";
        } else if (objectiveMode == ObjectiveMode.FUEL) {
            result.projectName = "NavigationFuel3D";
            result.calculationType = "fuelOptimal3DLayered";
        } else {
            result.projectName = "NavigationComposite3D";
            result.calculationType = "compositeOptimal3DLayered";
        }
        result.navigationDate = config.navigationDate;
        result.area = config.area;
        result.grid = config.grid;
        result.shipParameters = config.shipParameters;
        result.totalDistance = roundToTwoDecimals(totalDistance);
        result.totalTime = roundToTwoDecimals(totalTime);
        result.pathPointCount = bestPath.size();
        result.windFile = buildDataFileSummary("wind");
        result.currentFile = buildDataFileSummary("cur");
        result.waveFile = buildDataFileSummary("wave");
        result.obstacleFile = config.obstacleFileName == null || config.obstacleFileName.isBlank()
                ? "obstacle_map.json" : config.obstacleFileName;
        result.pathPoints = buildRoutePoints(bestPath);
        result.daySwitchPoints = buildDaySwitchPoints(daySwitchPoints, result.pathPoints);
        result.denseTrajectory = buildDenseTrajectory(bestPath);

        if (objectiveMode == ObjectiveMode.FUEL) {
            result.totalFuelEnergyJ = roundToSixDecimals(totalObjective);
        } else if (objectiveMode != ObjectiveMode.SAFETY) {
            result.totalCompositeCost = roundToSixDecimals(totalObjective);
            result.safetyWeight = roundToSixDecimals(objectiveWeights.safety);
            result.timeWeight = roundToSixDecimals(objectiveWeights.time);
            result.fuelWeight = roundToSixDecimals(objectiveWeights.fuel);
        }

        PathEvaluation evaluation = evaluateFuelAndSafetyForPath(bestPath);
        result.totalFuelEnergyJ = roundToSixDecimals(evaluation.totalFuelEnergyJ);
        result.totalSafetyScore = roundToSixDecimals(evaluation.avgSafetyScore);
        result.totalCumulativeRisk = roundToSixDecimals(evaluation.cumulativeRisk);

        return result;
    }

    private PathEvaluation evaluateFuelAndSafetyForPath(List<GridPoint> path) {
        if (path == null || path.size() < 2) {
            return new PathEvaluation(0.0, 0.0, 0.0);
        }

        double elapsedHours = 0.0;
        double totalFuelEnergyJ = 0.0;
        double safetySum = 0.0;
        double cumulativeRisk = 0.0;
        double totalDistanceNm = 0.0;

        for (int index = 0; index < path.size() - 1; index++) {
            GridPoint from = path.get(index);
            GridPoint to = path.get(index + 1);
            int directionIndex = findDirectionIndex(to.i - from.i, to.j - from.j);
            if (directionIndex < 0) {
                continue;
            }

            SegmentEvaluation segment = evaluateSegmentFuelSafety(from, to, directionIndex, elapsedHours);
            elapsedHours += segment.totalHours;
            totalFuelEnergyJ += segment.fuelEnergyJ;
            safetySum += segment.safetySum;
            cumulativeRisk += segment.cumulativeRisk;
            totalDistanceNm += segment.distanceNm;
        }

        // 平均安全性 = Σ(safetyIndex·d)/Σd（距离加权），与报告列 S 同口径。
        // 2026-09-23 修正：旧版为 Σ(safetyIndex·占比)/步数，分母含步数、结果与航段数相关。
        double avgSafety = totalDistanceNm > EPSILON
                ? clamp(safetySum / totalDistanceNm, 0.0, 1.0) : 0.0;
        return new PathEvaluation(totalFuelEnergyJ, avgSafety, cumulativeRisk);
    }

    private SegmentEvaluation evaluateSegmentFuelSafety(GridPoint from,
                                                        GridPoint to,
                                                        int directionIndex,
                                                        double startElapsedHours) {
        double remainingDistanceTime = directionDistances[directionIndex];
        double remainingDistancePhysical = calculateSegmentDistance(from, to);
        double totalHours = 0.0;
        double totalFuelEnergyJ = 0.0;
        double safetySum = 0.0;
        double cumulativeRisk = 0.0;
        double segmentDistanceNm = 0.0;
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

            double fuelEnergyJ = calculateFuelEnergyWithSnapshot(snapshot, from.i, from.j, bearing, stepHours);
            if (Double.isFinite(fuelEnergyJ) && fuelEnergyJ >= 0.0) {
                totalFuelEnergyJ += fuelEnergyJ;
            }

            double safetyIndex = clamp(getOrComputeSafetyIndex(dayIndex, from.i, from.j, directionIndex), 0.0, 1.0);
            // 距离加权报告指标：S = Σ(safetyIndex·d)/Σd；优化代价另累计 E = Σ(risk·d)。
            safetySum += safetyIndex * stepDistancePhysical;
            cumulativeRisk += (1.0 - safetyIndex) * stepDistancePhysical;
            segmentDistanceNm += stepDistancePhysical;

            totalHours += stepHours;
            remainingDistanceTime -= stepDistanceTime;
            remainingDistancePhysical -= stepDistancePhysical;
            currentElapsedHours += stepHours;

            if (completeInCurrentDay) {
                break;
            }
        }

        return new SegmentEvaluation(totalHours, totalFuelEnergyJ, safetySum, cumulativeRisk, segmentDistanceNm);
    }

    private static final class PathEvaluation {
        private final double totalFuelEnergyJ;
        private final double avgSafetyScore;
        private final double cumulativeRisk;

        private PathEvaluation(double totalFuelEnergyJ, double avgSafetyScore, double cumulativeRisk) {
            this.totalFuelEnergyJ = totalFuelEnergyJ;
            this.avgSafetyScore = avgSafetyScore;
            this.cumulativeRisk = cumulativeRisk;
        }
    }

    private static final class SegmentEvaluation {
        private final double totalHours;
        private final double fuelEnergyJ;
        private final double safetySum;
        private final double cumulativeRisk;
        private final double distanceNm;

        private SegmentEvaluation(double totalHours,
                                  double fuelEnergyJ,
                                  double safetySum,
                                  double cumulativeRisk,
                                  double distanceNm) {
            this.totalHours = totalHours;
            this.fuelEnergyJ = fuelEnergyJ;
            this.safetySum = safetySum;
            this.cumulativeRisk = cumulativeRisk;
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

    /**
     * 段内 dt 物理积分生成连续航迹（可视化用，不参与优化）。
     * 每步查询当前经纬度所在栅格在当日 snapshot 下的风/流，复用
     * ShipUtils 失速口径得到对地速度矢量，累积位置；段末用仿射校正
     * 将最后一点对齐至栅格 to 点。3D 侧跨日时自动切换 dailySnapshots，
     * 即便 pathPoints 栅格序列与 Easy 相同，连续轨迹因各段起始时刻
     * 不同会产生差异。
     */
    private List<DenseTrajectoryPoint> buildDenseTrajectory(List<GridPoint> path) {
        List<DenseTrajectoryPoint> dense = new ArrayList<>();
        if (path == null || path.size() < 2 || dailySnapshots.isEmpty()) {
            return dense;
        }
        final int N_PER_SEGMENT = 20;
        final double M_PER_DEG_LAT = 111320.0;
        final double KT_TO_MS = 0.5144;
        ShipParameters ship = config.shipParameters;
        if (ship == null) {
            return dense;
        }

        double elapsedHours = 0.0;
        double curLon = config.area.lonMin + path.get(0).j * deltaLon;
        double curLat = config.area.latMin + path.get(0).i * deltaLat;
        dense.add(new DenseTrajectoryPoint(
                roundToSixDecimals(curLon),
                roundToSixDecimals(curLat),
                roundToSixDecimals(elapsedHours)));

        for (int s = 0; s < path.size() - 1; s++) {
            GridPoint from = path.get(s);
            GridPoint to = path.get(s + 1);
            int directionIndex = findDirectionIndex(to.i - from.i, to.j - from.j);
            if (directionIndex < 0) {
                curLon = config.area.lonMin + to.j * deltaLon;
                curLat = config.area.latMin + to.i * deltaLat;
                dense.add(new DenseTrajectoryPoint(
                        roundToSixDecimals(curLon),
                        roundToSixDecimals(curLat),
                        roundToSixDecimals(elapsedHours)));
                continue;
            }
            double bearing = bearings[directionIndex];

            double clockHours0 = departureOffsetHours + elapsedHours;
            int dayIndex0 = (int) Math.max(0, Math.min(dailySnapshots.size() - 1,
                    (int) Math.floor(clockHours0 / DAY_HOURS)));
            EnvironmentSnapshot startSnap = dailySnapshots.get(dayIndex0);
            double segmentHours = calculateSegmentTimeWithSnapshot(
                    startSnap, from.i, from.j, directionIndex, directionDistances[directionIndex]);
            if (!Double.isFinite(segmentHours) || segmentHours <= 0) {
                curLon = config.area.lonMin + to.j * deltaLon;
                curLat = config.area.latMin + to.i * deltaLat;
                dense.add(new DenseTrajectoryPoint(
                        roundToSixDecimals(curLon),
                        roundToSixDecimals(curLat),
                        roundToSixDecimals(elapsedHours)));
                continue;
            }
            double dt = segmentHours / N_PER_SEGMENT;
            int startIdx = dense.size();

            for (int step = 1; step <= N_PER_SEGMENT; step++) {
                double clockHours = departureOffsetHours + elapsedHours;
                int dayIndex = (int) Math.max(0, Math.min(dailySnapshots.size() - 1,
                        (int) Math.floor(clockHours / DAY_HOURS)));
                EnvironmentSnapshot snap = dailySnapshots.get(dayIndex);
                int iCur = (int) Math.max(0, Math.min(config.grid.ny - 1,
                        Math.round((curLat - config.area.latMin) / deltaLat)));
                int jCur = (int) Math.max(0, Math.min(config.grid.nx - 1,
                        Math.round((curLon - config.area.lonMin) / deltaLon)));
                double windU = safeValue(snap.windUData[iCur][jCur]);
                double windV = safeValue(snap.windVData[iCur][jCur]);
                double currentU = safeValue(snap.currentUData[iCur][jCur]);
                double currentV = safeValue(snap.currentVData[iCur][jCur]);
                double windSpeed = Math.sqrt(windU * windU + windV * windV);
                double currentSpeed = Math.sqrt(currentU * currentU + currentV * currentV);
                double windDir = calculateDirection(windU, windV);
                double currentDir = calculateDirection(currentU, currentV);

                double thetaG = Math.toRadians(bearing);
                double vrX = ship.shipSpeed * KT_TO_MS * Math.cos(thetaG);
                double vrY = ship.shipSpeed * KT_TO_MS * Math.sin(thetaG);
                double thetaC = Math.toRadians(currentDir);
                double vcX = currentSpeed * Math.cos(thetaC);
                double vcY = currentSpeed * Math.sin(thetaC);
                double vgX = vrX + vcX;
                double vgY = vrY + vcY;
                double vgMag = Math.sqrt(vgX * vgX + vgY * vgY);
                double actualMs = ShipUtils.calculateActualSpeedMs(
                        bearing, windSpeed, windDir,
                        currentSpeed, currentDir,
                        ship.shipSpeed, ship.shipLength,
                        ship.shipWidth, ship.shipDraft,
                        ship.shipDisplacement,
                        useCompositeShipClassCorrection());
                double scale = vgMag > 1e-6 ? actualMs / vgMag : 0.0;
                double vX = vgX * scale;
                double vY = vgY * scale;
                double dtSec = dt * 3600.0;
                double dNorthM = vX * dtSec;
                double dEastM = vY * dtSec;
                curLat += dNorthM / M_PER_DEG_LAT;
                double cosLat = Math.max(0.1, Math.cos(Math.toRadians(curLat)));
                curLon += dEastM / (M_PER_DEG_LAT * cosLat);
                elapsedHours += dt;
                dense.add(new DenseTrajectoryPoint(curLon, curLat, elapsedHours));
            }

            double lonTo = config.area.lonMin + to.j * deltaLon;
            double latTo = config.area.latMin + to.i * deltaLat;
            double errLon = lonTo - curLon;
            double errLat = latTo - curLat;
            for (int step = 1; step <= N_PER_SEGMENT; step++) {
                double alpha = step / (double) N_PER_SEGMENT;
                DenseTrajectoryPoint pt = dense.get(startIdx + step - 1);
                pt.lon = roundToSixDecimals(pt.lon + alpha * errLon);
                pt.lat = roundToSixDecimals(pt.lat + alpha * errLat);
                pt.elapsedHours = roundToSixDecimals(pt.elapsedHours);
            }
            curLon = lonTo;
            curLat = latTo;
        }
        return dense;
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

    private static class SegmentMetrics {
        final double timeHours;
        final double objectiveCost;
        final double distanceNm;

        SegmentMetrics(double timeHours, double objectiveCost) {
            this(timeHours, objectiveCost, 0.0);
        }

        SegmentMetrics(double timeHours, double objectiveCost, double distanceNm) {
            this.timeHours = timeHours;
            this.objectiveCost = objectiveCost;
            this.distanceNm = distanceNm;
        }

        static SegmentMetrics invalid() {
            return new SegmentMetrics(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, 0.0);
        }
    }

    private static class State implements Comparable<State> {
        final int day;
        final int i;
        final int j;
        final double totalObjective;
        final double totalTimeHours;

        State(int day, int i, int j, double totalObjective, double totalTimeHours) {
            this.day = day;
            this.i = i;
            this.j = j;
            this.totalObjective = totalObjective;
            this.totalTimeHours = totalTimeHours;
        }

        @Override
        public int compareTo(State other) {
            return Double.compare(this.totalObjective, other.totalObjective);
        }
    }

    private static class ObjectiveResult {
        final List<GridPoint> path;
        final double totalObjective;
        final double totalTimeHours;
        final int daysUsed;

        ObjectiveResult(List<GridPoint> path, double totalObjective, double totalTimeHours, int daysUsed) {
            this.path = path;
            this.totalObjective = totalObjective;
            this.totalTimeHours = totalTimeHours;
            this.daysUsed = daysUsed;
        }
    }
}
