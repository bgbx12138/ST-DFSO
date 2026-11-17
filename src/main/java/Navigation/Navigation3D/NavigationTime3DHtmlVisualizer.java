package Navigation.Navigation3D;

import Navigation.PathVisualizer;
import Navigation.model.GridPoint;
import Navigation.object.NavigationArea;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.awt.Desktop;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
//对三维分层全局最优导航结果进行三维可视化
public class NavigationTime3DHtmlVisualizer {

    private final ObjectMapper objectMapper;

    public NavigationTime3DHtmlVisualizer(ObjectMapper objectMapper) {
        this.objectMapper = objectMapper;
    }

    public Path writeVisualization(Path outputPath,
                                   String navigationDate,
                                   NavigationArea area,
                                   boolean[][] obstacleMap,
                                   boolean[][] obstacleBufferMap,
                                   List<double[]> timedPathPoints,
                                   List<PathVisualizer.MarkerPoint> switchPoints,
                                   double deltaLat,
                                   double deltaLon,
                                   int daysUsed,
                                   int numDays,
                                   double totalDistance,
                                   double totalTime) throws IOException {
        return writeVisualization(outputPath, navigationDate, area, obstacleMap, obstacleBufferMap,
            timedPathPoints, switchPoints, deltaLat, deltaLon, daysUsed, numDays, totalDistance, totalTime,
            List.of());
    }

    public Path writeVisualization(Path outputPath,
                                   String navigationDate,
                                   NavigationArea area,
                                   boolean[][] obstacleMap,
                                   boolean[][] obstacleBufferMap,
                                   List<double[]> timedPathPoints,
                                   List<PathVisualizer.MarkerPoint> switchPoints,
                                   double deltaLat,
                                   double deltaLon,
                                   int daysUsed,
                                   int numDays,
                                   double totalDistance,
                                   double totalTime,
                                   List<RouteSeriesData> additionalRoutes) throws IOException {
        if (outputPath.getParent() != null) {
            Files.createDirectories(outputPath.getParent());
        }

        Map<String, Object> pageData = buildPageData(navigationDate, area, obstacleMap,
                timedPathPoints, switchPoints, deltaLat, deltaLon, daysUsed, numDays, totalDistance, totalTime,
                additionalRoutes);
        Files.writeString(outputPath, buildHtml(pageData), StandardCharsets.UTF_8);
        return outputPath;
    }

    public RouteSeriesData buildFlatRouteSeries(String name,
                                                String horizontalColor,
                                                String verticalColor,
                                                String startColor,
                                                String endColor,
                                                List<GridPoint> path) {
        List<Map<String, Object>> points = new ArrayList<>();
        if (path != null) {
            for (int index = 0; index < path.size(); index++) {
                GridPoint point = path.get(index);
                points.add(buildRoutePoint(point.j, point.i, 0.0, name + " 节点 " + index));
            }
        }
        return new RouteSeriesData(name, horizontalColor, verticalColor, startColor, endColor, points);
    }

    public RouteSeriesData buildSteppedRouteSeries(String name,
                                                   String horizontalColor,
                                                   String verticalColor,
                                                   String startColor,
                                                   String endColor,
                                                   List<GridPoint> path,
                                                   List<PathVisualizer.MarkerPoint> switchPoints) {
        List<Map<String, Object>> points = new ArrayList<>();
        if (path == null || path.isEmpty()) {
            return new RouteSeriesData(name, horizontalColor, verticalColor, startColor, endColor, points);
        }

        List<Integer> switchIndices = mapSwitchPointsToPathIndices(path, switchPoints);
        int switchCursor = 0;
        double currentLayer = 0.0;

        for (int index = 0; index < path.size(); index++) {
            GridPoint point = path.get(index);
            points.add(buildRoutePoint(point.j, point.i, currentLayer, name + " 节点 " + index));

            while (switchCursor < switchIndices.size() && switchIndices.get(switchCursor) == index) {
                PathVisualizer.MarkerPoint marker = switchPoints.get(switchCursor);
                double targetLayer = Math.max(currentLayer + 1.0, extractLayerFromLabel(marker.label));
                points.add(buildRoutePoint(point.j, point.i, targetLayer, name + " 切换 " + marker.label));
                currentLayer = targetLayer;
                switchCursor++;
            }
        }

        return new RouteSeriesData(name, horizontalColor, verticalColor, startColor, endColor, points);
    }

    public void openInBrowser(Path outputPath) {
        try {
            if (!Desktop.isDesktopSupported()) {
                return;
            }
            Desktop.getDesktop().browse(outputPath.toUri());
        } catch (Exception exception) {
            System.err.println("无法自动打开 3D 可视化页面: " + exception.getMessage());
        }
    }

    private Map<String, Object> buildPageData(String navigationDate,
                                              NavigationArea area,
                                              boolean[][] obstacleMap,
                                              List<double[]> timedPathPoints,
                                              List<PathVisualizer.MarkerPoint> switchPoints,
                                              double deltaLat,
                                              double deltaLon,
                                              int daysUsed,
                                              int numDays,
                                              double totalDistance,
                                              double totalTime,
                                              List<RouteSeriesData> additionalRoutes) {
        Map<String, Object> pageData = new LinkedHashMap<>();
        pageData.put("navigationDate", navigationDate);
        pageData.put("daysUsed", daysUsed);
        pageData.put("numDays", numDays);
        pageData.put("totalDistance", totalDistance);
        pageData.put("totalTime", totalTime);
        pageData.put("area", buildAreaData(area));
        pageData.put("grid", buildGridData(obstacleMap, deltaLat, deltaLon));
        pageData.put("path", buildPathData(timedPathPoints));
        pageData.put("switches", buildSwitchData(switchPoints));
    pageData.put("routes", buildRoutesData(timedPathPoints, switchPoints, additionalRoutes));
        pageData.put("obstacleRuns", buildObstacleRuns(obstacleMap));
        return pageData;
    }

    private List<Map<String, Object>> buildRoutesData(List<double[]> timedPathPoints,
                                                      List<PathVisualizer.MarkerPoint> switchPoints,
                                                      List<RouteSeriesData> additionalRoutes) {
        List<Map<String, Object>> routes = new ArrayList<>();

        List<Map<String, Object>> primaryPoints = buildSteppedPrimaryRoutePoints(timedPathPoints, switchPoints);
        routes.add(new RouteSeriesData("三维分层全局最优", "#d95c2b", "#8b5cf6", "#1d4ed8", "#16a34a", primaryPoints).toMap());

        if (additionalRoutes != null) {
            for (RouteSeriesData route : additionalRoutes) {
                routes.add(route.toMap());
            }
        }
        return routes;
    }

    private List<Map<String, Object>> buildSteppedPrimaryRoutePoints(List<double[]> timedPathPoints,
                                                                     List<PathVisualizer.MarkerPoint> switchPoints) {
        List<Map<String, Object>> points = new ArrayList<>();
        if (timedPathPoints == null || timedPathPoints.isEmpty()) {
            return points;
        }

        List<Integer> switchIndices = mapSwitchPointsToTimedPathIndices(timedPathPoints, switchPoints);
        int switchCursor = 0;
        double currentLayer = 0.0;

        for (int index = 0; index < timedPathPoints.size(); index++) {
            double[] point = timedPathPoints.get(index);
            double gridX = point.length > 4 ? point[4] : 0.0;
            double gridY = point.length > 5 ? point[5] : 0.0;
            points.add(buildRoutePoint(gridX, gridY, currentLayer, "三维分层全局最优 节点 " + index));

            while (switchCursor < switchIndices.size() && switchIndices.get(switchCursor) == index) {
                PathVisualizer.MarkerPoint marker = switchPoints.get(switchCursor);
                points.add(buildRoutePoint(marker.gridX, marker.gridY, currentLayer,
                    "三维分层全局最优 切换前 " + marker.label));

                double targetLayer = Math.max(currentLayer + 1.0, extractLayerFromLabel(marker.label));
                points.add(buildRoutePoint(marker.gridX, marker.gridY, targetLayer,
                    "三维分层全局最优 切换后 " + marker.label));

                currentLayer = targetLayer;
                switchCursor++;
            }
        }
        return points;
    }

    private Map<String, Object> buildRoutePoint(double gridX, double gridY, double layer, String label) {
        Map<String, Object> point = new LinkedHashMap<>();
        point.put("gridX", gridX);
        point.put("gridY", gridY);
        point.put("layer", layer);
        point.put("label", label);
        return point;
    }

    private List<Integer> mapSwitchPointsToPathIndices(List<GridPoint> path,
                                                       List<PathVisualizer.MarkerPoint> switchPoints) {
        List<Integer> indices = new ArrayList<>();
        if (switchPoints == null || switchPoints.isEmpty()) {
            return indices;
        }

        int searchStart = 0;
        for (PathVisualizer.MarkerPoint marker : switchPoints) {
            int nearest = -1;
            double bestDistance = Double.MAX_VALUE;
            for (int index = searchStart; index < path.size(); index++) {
                GridPoint point = path.get(index);
                double dx = point.j - marker.gridX;
                double dy = point.i - marker.gridY;
                double distance = dx * dx + dy * dy;
                if (distance < bestDistance) {
                    bestDistance = distance;
                    nearest = index;
                }
            }
            if (nearest >= 0) {
                indices.add(nearest);
                searchStart = Math.min(path.size(), nearest + 1);
            }
        }
        return indices;
    }

    private List<Integer> mapSwitchPointsToTimedPathIndices(List<double[]> timedPathPoints,
                                                            List<PathVisualizer.MarkerPoint> switchPoints) {
        List<Integer> indices = new ArrayList<>();
        if (switchPoints == null || switchPoints.isEmpty()) {
            return indices;
        }

        int searchStart = 0;
        for (PathVisualizer.MarkerPoint marker : switchPoints) {
            int nearest = -1;
            double bestDistance = Double.MAX_VALUE;
            for (int index = searchStart; index < timedPathPoints.size(); index++) {
                double[] point = timedPathPoints.get(index);
                double gridX = point.length > 4 ? point[4] : 0.0;
                double gridY = point.length > 5 ? point[5] : 0.0;
                double dx = gridX - marker.gridX;
                double dy = gridY - marker.gridY;
                double distance = dx * dx + dy * dy;
                if (distance < bestDistance) {
                    bestDistance = distance;
                    nearest = index;
                }
            }
            if (nearest >= 0) {
                indices.add(nearest);
                searchStart = Math.min(timedPathPoints.size(), nearest + 1);
            }
        }
        return indices;
    }

    public static class RouteSeriesData {
        public final String name;
        public final String horizontalColor;
        public final String verticalColor;
        public final String startColor;
        public final String endColor;
        public final List<Map<String, Object>> points;

        public RouteSeriesData(String name,
                               String horizontalColor,
                               String verticalColor,
                               String startColor,
                               String endColor,
                               List<Map<String, Object>> points) {
            this.name = name;
            this.horizontalColor = horizontalColor;
            this.verticalColor = verticalColor;
            this.startColor = startColor;
            this.endColor = endColor;
            this.points = points == null ? List.of() : points;
        }

        private Map<String, Object> toMap() {
            Map<String, Object> route = new LinkedHashMap<>();
            route.put("name", name);
            route.put("horizontalColor", horizontalColor);
            route.put("verticalColor", verticalColor);
            route.put("startColor", startColor);
            route.put("endColor", endColor);
            route.put("points", points);
            return route;
        }
    }

    private Map<String, Object> buildAreaData(NavigationArea area) {
        Map<String, Object> areaData = new LinkedHashMap<>();
        areaData.put("latMin", area.latMin);
        areaData.put("latMax", area.latMax);
        areaData.put("lonMin", area.lonMin);
        areaData.put("lonMax", area.lonMax);
        return areaData;
    }

    private Map<String, Object> buildGridData(boolean[][] obstacleMap, double deltaLat, double deltaLon) {
        Map<String, Object> gridData = new LinkedHashMap<>();
        gridData.put("nx", obstacleMap.length == 0 ? 0 : obstacleMap[0].length);
        gridData.put("ny", obstacleMap.length);
        gridData.put("deltaLat", deltaLat);
        gridData.put("deltaLon", deltaLon);
        return gridData;
    }

    private List<Map<String, Object>> buildPathData(List<double[]> timedPathPoints) {
        List<Map<String, Object>> pathData = new ArrayList<>(timedPathPoints.size());
        for (int index = 0; index < timedPathPoints.size(); index++) {
            double[] point = timedPathPoints.get(index);
            Map<String, Object> item = new LinkedHashMap<>();
            item.put("lon", point[0]);
            item.put("lat", point[1]);
            item.put("layer", point[2]);
            item.put("hours", point[3]);
            item.put("gridX", point.length > 4 ? point[4] : 0.0);
            item.put("gridY", point.length > 5 ? point[5] : 0.0);
            item.put("label", "节点 " + index + " | 累计时间 " + String.format("%.2f", point[3]) + " h");
            pathData.add(item);
        }
        return pathData;
    }

    private List<Map<String, Object>> buildSwitchData(List<PathVisualizer.MarkerPoint> switchPoints) {
        List<Map<String, Object>> switches = new ArrayList<>(switchPoints.size());
        for (PathVisualizer.MarkerPoint switchPoint : switchPoints) {
            Map<String, Object> item = new LinkedHashMap<>();
            item.put("gridX", switchPoint.gridX);
            item.put("gridY", switchPoint.gridY);
            item.put("layer", extractLayerFromLabel(switchPoint.label));
            item.put("label", switchPoint.label);
            switches.add(item);
        }
        return switches;
    }

    private List<Map<String, Object>> buildObstacleRuns(boolean[][] matrix) {
        List<Map<String, Object>> runs = new ArrayList<>();
        for (int row = 0; row < matrix.length; row++) {
            int col = 0;
            while (col < matrix[row].length) {
                if (!matrix[row][col]) {
                    col++;
                    continue;
                }
                int start = col;
                while (col + 1 < matrix[row].length && matrix[row][col + 1]) {
                    col++;
                }
                Map<String, Object> run = new LinkedHashMap<>();
                run.put("row", row);
                run.put("startX", start);
                run.put("endX", col);
                runs.add(run);
                col++;
            }
        }
        return runs;
    }

    private double extractLayerFromLabel(String label) {
        if (label == null || label.length() < 2) {
            return 0.0;
        }
        try {
            return Math.max(0, Integer.parseInt(label.substring(1)) - 1);
        } catch (NumberFormatException exception) {
            return 0.0;
        }
    }

    private String buildHtml(Map<String, Object> pageData) throws IOException {
        String pageDataJson = objectMapper.writeValueAsString(pageData);
        StringBuilder html = new StringBuilder();
        html.append("<!DOCTYPE html>\n");
        html.append("<html lang=\"zh-CN\">\n");
        html.append("<head>\n");
        html.append("  <meta charset=\"UTF-8\">\n");
        html.append("  <meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">\n");
        html.append("  <title>NavigationTime3D 立体可视化</title>\n");
        html.append("  <style>\n");
        html.append("    :root { --bg-a:#efe7d8; --bg-b:#dce8ee; --panel:rgba(255,255,255,.78); --text:#17222d; --muted:#5f6f7d; --line:rgba(23,34,45,.12); --shadow:0 24px 50px rgba(23,34,45,.12); }\n");
        html.append("    * { box-sizing:border-box; }\n");
        html.append("    body { margin:0; min-height:100vh; font-family:'Segoe UI','Microsoft YaHei',sans-serif; color:var(--text); background:radial-gradient(circle at top left, rgba(255,255,255,.72), transparent 32%), linear-gradient(135deg, var(--bg-a), var(--bg-b)); padding:18px; }\n");
        html.append("    .layout { display:grid; grid-template-columns:280px minmax(0,1fr); gap:18px; max-width:1800px; margin:0 auto; }\n");
        html.append("    .panel,.canvas-shell { background:var(--panel); border:1px solid rgba(255,255,255,.6); border-radius:26px; box-shadow:var(--shadow); backdrop-filter:blur(18px); }\n");
        html.append("    .panel { padding:22px 20px; }\n");
        html.append("    h1 { margin:0 0 18px; font-size:24px; }\n");
        html.append("    .legend { display:grid; gap:12px; }\n");
        html.append("    .legend-item { display:flex; align-items:center; gap:10px; color:var(--muted); font-size:14px; }\n");
        html.append("    .swatch { width:15px; height:15px; border-radius:4px; flex:0 0 auto; border:1px solid rgba(0,0,0,.08); }\n");
        html.append("    .tips { margin-top:18px; color:var(--muted); font-size:13px; line-height:1.7; }\n");
        html.append("    .canvas-shell { padding:10px; min-height:820px; }\n");
        html.append("    #scene { width:100%; height:800px; display:block; border-radius:20px; background:linear-gradient(180deg, rgba(252,252,251,.9), rgba(229,237,241,.96)); cursor:grab; }\n");
        html.append("    #scene.dragging { cursor:grabbing; }\n");
        html.append("    @media (max-width:1100px) { body { padding:12px; } .layout { grid-template-columns:1fr; } .canvas-shell { min-height:620px; } #scene { height:600px; } }\n");
        html.append("  </style>\n");
        html.append("</head>\n");
        html.append("<body>\n");
        html.append("  <div class=\"layout\">\n");
        html.append("    <aside class=\"panel\">\n");
        html.append("      <h1>图例</h1>\n");
        html.append("      <div class=\"legend\">\n");
        html.append("        <div class=\"legend-item\"><span class=\"swatch\" style=\"background:#121826;\"></span><span>每层障碍物矩阵</span></div>\n");
        html.append("        <div class=\"legend-item\"><span class=\"swatch\" style=\"background:rgba(18,24,38,.14);\"></span><span>时间层平面</span></div>\n");
    html.append("        <div id=\"routeLegend\"></div>\n");
        html.append("      </div>\n");
        html.append("      <div class=\"tips\">左键拖拽可 360 度旋转，滚轮缩放，右键拖拽平移。<br>航线同一天内始终贴在同一层上，只在跨天时垂直抬升。</div>\n");
        html.append("    </aside>\n");
        html.append("    <section class=\"canvas-shell\">\n");
        html.append("      <canvas id=\"scene\"></canvas>\n");
        html.append("    </section>\n");
        html.append("  </div>\n");
        html.append("  <script>\n");
        html.append("    const pageData = ").append(pageDataJson).append(";\n");
        html.append("    const canvas = document.getElementById('scene');\n");
        html.append("    const ctx = canvas.getContext('2d');\n");
        html.append("    const state = { yaw: ").append(0.3).append(", pitch: 2.2, zoom: 0.9, panX: 0, panY: 0, dragging: false, button: 0, lastX: 0, lastY: 0 };\n");
        html.append("    const grid = pageData.grid;\n");
        html.append("    const center = { x: (grid.nx - 1) / 2, y: (grid.ny - 1) / 2 };\n");
        html.append("    const layerGap = 7.5;\n");
        html.append("    const dpr = Math.max(1, window.devicePixelRatio || 1);\n");
        html.append("\n");
        html.append("    function baseScale() {\n");
        html.append("      const width = canvas.clientWidth || 1200;\n");
        html.append("      const height = canvas.clientHeight || 800;\n");
        html.append("      return Math.min(width / (grid.nx * 1.65), height / (grid.ny * 1.2 + pageData.numDays * layerGap * 1.6));\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    function resizeCanvas() {\n");
        html.append("      const rect = canvas.getBoundingClientRect();\n");
        html.append("      canvas.width = Math.round(rect.width * dpr);\n");
        html.append("      canvas.height = Math.round(rect.height * dpr);\n");
        html.append("      render();\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    function worldPoint(gridX, gridY, layer) {\n");
        html.append("      return { x: gridX - center.x, y: (grid.ny - 1 - gridY) - center.y, z: -layer * layerGap };\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    function rotatePoint(point) {\n");
        html.append("      const cosYaw = Math.cos(state.yaw);\n");
        html.append("      const sinYaw = Math.sin(state.yaw);\n");
        html.append("      const rx = point.x * cosYaw - point.y * sinYaw;\n");
        html.append("      const ry = point.x * sinYaw + point.y * cosYaw;\n");
        html.append("      const cosPitch = Math.cos(state.pitch);\n");
        html.append("      const sinPitch = Math.sin(state.pitch);\n");
        html.append("      const sy = ry * cosPitch - point.z * sinPitch;\n");
        html.append("      const sz = ry * sinPitch + point.z * cosPitch;\n");
        html.append("      return { x: rx, y: sy, depth: sz };\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    function project(point) {\n");
        html.append("      const rotated = rotatePoint(point);\n");
        html.append("      const scale = baseScale() * state.zoom;\n");
        html.append("      return { x: canvas.clientWidth / 2 + state.panX + rotated.x * scale, y: canvas.clientHeight / 2 + state.panY - rotated.y * scale, depth: rotated.depth };\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    function wrapAngle(angle) {\n");
        html.append("      const twoPi = Math.PI * 2;\n");
        html.append("      angle = angle % twoPi;\n");
        html.append("      return angle < 0 ? angle + twoPi : angle;\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    function layerPlane(layerIndex) {\n");
        html.append("      return [\n");
        html.append("        worldPoint(-0.5, -0.5, layerIndex),\n");
        html.append("        worldPoint(grid.nx - 0.5, -0.5, layerIndex),\n");
        html.append("        worldPoint(grid.nx - 0.5, grid.ny - 0.5, layerIndex),\n");
        html.append("        worldPoint(-0.5, grid.ny - 0.5, layerIndex)\n");
        html.append("      ];\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    function buildObstaclePolygons() {\n");
        html.append("      const polygons = [];\n");
        html.append("      for (let layer = 0; layer < pageData.numDays; layer += 1) {\n");
        html.append("        for (const run of pageData.obstacleRuns) {\n");
        html.append("          const corners = [\n");
        html.append("            worldPoint(run.startX - 0.5, run.row - 0.5, layer),\n");
        html.append("            worldPoint(run.endX + 0.5, run.row - 0.5, layer),\n");
        html.append("            worldPoint(run.endX + 0.5, run.row + 0.5, layer),\n");
        html.append("            worldPoint(run.startX - 0.5, run.row + 0.5, layer)\n");
        html.append("          ];\n");
        html.append("          polygons.push({ layer: layer, corners: corners });\n");
        html.append("        }\n");
        html.append("      }\n");
        html.append("      return polygons;\n");
        html.append("    }\n");
        html.append("\n");
    html.append("    function buildRouteSegments(route) {\n");
    html.append("      const segments = [];\n");
    html.append("      const points = route.points || [];\n");
    html.append("      for (let i = 0; i < points.length - 1; i += 1) {\n");
    html.append("        const from = points[i];\n");
    html.append("        const to = points[i + 1];\n");
    html.append("        const vertical = Math.abs((to.layer || 0) - (from.layer || 0)) > 1e-8;\n");
    html.append("        segments.push({\n");
    html.append("          type: vertical ? 'vertical' : 'horizontal',\n");
    html.append("          color: vertical ? (route.verticalColor || '#8b5cf6') : (route.horizontalColor || '#d95c2b'),\n");
    html.append("          from: worldPoint(from.gridX, from.gridY, from.layer || 0),\n");
    html.append("          to: worldPoint(to.gridX, to.gridY, to.layer || 0)\n");
    html.append("        });\n");
    html.append("      }\n");
    html.append("      return segments;\n");
    html.append("    }\n");
        html.append("\n");
    html.append("    function remapPrimaryRouteToStepped(route, switches) {\n");
    html.append("      if (!route || !route.points || route.points.length === 0) return route;\n");
    html.append("      const points = route.points;\n");
    html.append("      const switchList = (switches || []).slice();\n");
    html.append("      if (switchList.length === 0) {\n");
    html.append("        route.points = points.map((p, idx) => ({ ...p, layer: 0, label: p.label || ('节点 ' + idx) }));\n");
    html.append("        return route;\n");
    html.append("      }\n");
    html.append("\n");
    html.append("      const switchIndices = [];\n");
    html.append("      let searchStart = 0;\n");
    html.append("      for (const sw of switchList) {\n");
    html.append("        let nearest = -1;\n");
    html.append("        let best = Number.POSITIVE_INFINITY;\n");
    html.append("        for (let i = searchStart; i < points.length; i += 1) {\n");
    html.append("          const dx = (points[i].gridX || 0) - (sw.gridX || 0);\n");
    html.append("          const dy = (points[i].gridY || 0) - (sw.gridY || 0);\n");
    html.append("          const d2 = dx * dx + dy * dy;\n");
    html.append("          if (d2 < best) { best = d2; nearest = i; }\n");
    html.append("        }\n");
    html.append("        if (nearest >= 0) {\n");
    html.append("          switchIndices.push(nearest);\n");
    html.append("          searchStart = Math.min(points.length - 1, nearest + 1);\n");
    html.append("        }\n");
    html.append("      }\n");
    html.append("\n");
    html.append("      const remapped = [];\n");
    html.append("      let layer = 0;\n");
    html.append("      let cursor = 0;\n");
    html.append("      for (let i = 0; i < points.length; i += 1) {\n");
    html.append("        const p = points[i];\n");
    html.append("        remapped.push({ ...p, layer });\n");
    html.append("        while (cursor < switchIndices.length && switchIndices[cursor] === i) {\n");
    html.append("          const sw = switchList[cursor];\n");
    html.append("          remapped.push({ gridX: sw.gridX, gridY: sw.gridY, layer: layer, label: `三维分层全局最优 切换前 ${sw.label || ''}` });\n");
    html.append("          const raw = Number((sw.label || '').replace(/[^0-9]/g, ''));\n");
    html.append("          const target = Number.isFinite(raw) && raw > 0 ? (raw - 1) : (layer + 1);\n");
    html.append("          layer = Math.max(layer + 1, target);\n");
    html.append("          remapped.push({ gridX: sw.gridX, gridY: sw.gridY, layer: layer, label: `三维分层全局最优 切换后 ${sw.label || ''}` });\n");
    html.append("          cursor += 1;\n");
    html.append("        }\n");
    html.append("      }\n");
    html.append("      route.points = remapped;\n");
    html.append("      return route;\n");
    html.append("    }\n");
    html.append("\n");
    html.append("    if (pageData.routes && pageData.routes.length > 0) {\n");
    html.append("      pageData.routes[0] = remapPrimaryRouteToStepped(pageData.routes[0], pageData.switches || []);\n");
    html.append("    }\n");
    html.append("\n");
    html.append("    const obstaclePolygons = buildObstaclePolygons();\n");
    html.append("    const routeSegmentsByRoute = (pageData.routes || []).map(route => ({ route, segments: buildRouteSegments(route) }));\n");
    html.append("\n");
    html.append("    function renderRouteLegend() {\n");
    html.append("      const host = document.getElementById('routeLegend');\n");
    html.append("      if (!host) return;\n");
    html.append("      host.innerHTML = '';\n");
    html.append("      for (const route of (pageData.routes || [])) {\n");
    html.append("        const horizontal = document.createElement('div');\n");
    html.append("        horizontal.className = 'legend-item';\n");
    html.append("        horizontal.innerHTML = `<span class=\"swatch\" style=\"background:${route.horizontalColor || '#d95c2b'};\"></span><span>${route.name}（水平段）</span>`;\n");
    html.append("        host.appendChild(horizontal);\n");
    html.append("\n");
    html.append("        const vertical = document.createElement('div');\n");
    html.append("        vertical.className = 'legend-item';\n");
    html.append("        vertical.innerHTML = `<span class=\"swatch\" style=\"background:${route.verticalColor || '#8b5cf6'};\"></span><span>${route.name}（跨天抬升）</span>`;\n");
    html.append("        host.appendChild(vertical);\n");
    html.append("      }\n");
    html.append("      const startItem = document.createElement('div');\n");
    html.append("      startItem.className = 'legend-item';\n");
    html.append("      startItem.innerHTML = '<span class=\"swatch\" style=\"background:#1d4ed8;\"></span><span>起点</span>';\n");
    html.append("      host.appendChild(startItem);\n");
    html.append("      const endItem = document.createElement('div');\n");
    html.append("      endItem.className = 'legend-item';\n");
    html.append("      endItem.innerHTML = '<span class=\"swatch\" style=\"background:#16a34a;\"></span><span>终点</span>';\n");
    html.append("      host.appendChild(endItem);\n");
    html.append("    }\n");
        html.append("\n");
        html.append("    function drawPolygon(points, fillStyle, strokeStyle, lineWidth) {\n");
        html.append("      if (!points.length) return;\n");
        html.append("      ctx.beginPath();\n");
        html.append("      ctx.moveTo(points[0].x, points[0].y);\n");
        html.append("      for (let i = 1; i < points.length; i += 1) {\n");
        html.append("        ctx.lineTo(points[i].x, points[i].y);\n");
        html.append("      }\n");
        html.append("      ctx.closePath();\n");
        html.append("      if (fillStyle) { ctx.fillStyle = fillStyle; ctx.fill(); }\n");
        html.append("      if (strokeStyle) { ctx.strokeStyle = strokeStyle; ctx.lineWidth = lineWidth; ctx.stroke(); }\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    function drawLayerLabels() {\n");
        html.append("      ctx.save();\n");
        html.append("      ctx.font = '600 14px Segoe UI, Microsoft YaHei, sans-serif';\n");
        html.append("      ctx.textAlign = 'left';\n");
        html.append("      ctx.textBaseline = 'middle';\n");
        html.append("      for (let layer = 0; layer < pageData.numDays; layer += 1) {\n");
        html.append("        const anchor = project(worldPoint(grid.nx + 1.6, -1.8, layer));\n");
        html.append("        const label = 'D' + (layer + 1);\n");
        html.append("        const paddingX = 10;\n");
        html.append("        const width = Math.max(34, ctx.measureText(label).width + paddingX * 2);\n");
        html.append("        const height = 22;\n");
        html.append("        ctx.fillStyle = 'rgba(255,255,255,0.82)';\n");
        html.append("        ctx.strokeStyle = 'rgba(23,34,45,0.14)';\n");
        html.append("        ctx.lineWidth = 1;\n");
        html.append("        ctx.beginPath();\n");
        html.append("        ctx.roundRect(anchor.x, anchor.y - height / 2, width, height, 10);\n");
        html.append("        ctx.fill();\n");
        html.append("        ctx.stroke();\n");
        html.append("        ctx.fillStyle = '#17222d';\n");
        html.append("        ctx.fillText(label, anchor.x + paddingX, anchor.y);\n");
        html.append("      }\n");
        html.append("      ctx.restore();\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    function render() {\n");
        html.append("      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);\n");
        html.append("      ctx.clearRect(0, 0, canvas.clientWidth, canvas.clientHeight);\n");
        html.append("\n");
        html.append("      const planes = [];\n");
        html.append("      for (let layer = 0; layer < pageData.numDays; layer += 1) {\n");
        html.append("        const corners = layerPlane(layer);\n");
        html.append("        const depth = corners.map(rotatePoint).reduce((sum, item) => sum + item.depth, 0) / corners.length;\n");
        html.append("        planes.push({ layer: layer, depth: depth, points: corners.map(project) });\n");
        html.append("      }\n");
        html.append("\n");
        html.append("      const projectedObstacles = obstaclePolygons.map(shape => {\n");
        html.append("        const depth = shape.corners.map(rotatePoint).reduce((sum, item) => sum + item.depth, 0) / shape.corners.length;\n");
        html.append("        return { layer: shape.layer, depth: depth, points: shape.corners.map(project) };\n");
        html.append("      });\n");
        html.append("\n");
        html.append("      planes.sort((a, b) => a.depth - b.depth);\n");
        html.append("      projectedObstacles.sort((a, b) => a.depth - b.depth);\n");
        html.append("\n");
        html.append("      for (const plane of planes) {\n");
        html.append("        drawPolygon(plane.points, plane.layer === 0 ? 'rgba(18,24,38,0.05)' : 'rgba(18,24,38,0.035)', 'rgba(18,24,38,0.08)', 1);\n");
        html.append("      }\n");
        html.append("\n");
        html.append("      for (const obstacle of projectedObstacles) {\n");
        html.append("        drawPolygon(obstacle.points, obstacle.layer === 0 ? 'rgba(18,24,38,0.34)' : 'rgba(18,24,38,0.22)', 'rgba(18,24,38,0.08)', 0.6);\n");
        html.append("      }\n");
        html.append("\n");
        html.append("      drawLayerLabels();\n");
        html.append("\n");
    html.append("      for (const routeEntry of routeSegmentsByRoute) {\n");
    html.append("        for (const segment of routeEntry.segments) {\n");
    html.append("          const from = project(segment.from);\n");
    html.append("          const to = project(segment.to);\n");
    html.append("          ctx.beginPath();\n");
    html.append("          ctx.moveTo(from.x, from.y);\n");
    html.append("          ctx.lineTo(to.x, to.y);\n");
    html.append("          ctx.strokeStyle = segment.color;\n");
    html.append("          ctx.lineWidth = segment.type === 'vertical' ? 2.2 : 2.6;\n");
    html.append("          ctx.lineCap = 'round';\n");
    html.append("          ctx.stroke();\n");
    html.append("        }\n");
    html.append("      }\n");
        html.append("\n");
    html.append("      for (const route of (pageData.routes || [])) {\n");
    html.append("        const points = route.points || [];\n");
    html.append("        for (let index = 0; index < points.length; index += 1) {\n");
    html.append("          const point = points[index];\n");
    html.append("          const projected = project(worldPoint(point.gridX, point.gridY, point.layer || 0));\n");
    html.append("          const endpoint = index === 0 || index === points.length - 1;\n");
    html.append("          ctx.beginPath();\n");
    html.append("          ctx.arc(projected.x, projected.y, endpoint ? 4.8 : 1.9, 0, Math.PI * 2);\n");
    html.append("          ctx.fillStyle = index === 0 ? (route.startColor || '#1d4ed8') : (index === points.length - 1 ? (route.endColor || '#16a34a') : '#fff7ed');\n");
    html.append("          ctx.fill();\n");
    html.append("          ctx.lineWidth = endpoint ? 1.4 : 0.9;\n");
    html.append("          ctx.strokeStyle = index === 0 ? (route.startColor || '#1d4ed8') : (index === points.length - 1 ? (route.endColor || '#16a34a') : (route.horizontalColor || '#d95c2b'));\n");
    html.append("          ctx.stroke();\n");
    html.append("        }\n");
    html.append("      }\n");
        html.append("    }\n");
        html.append("\n");
        html.append("    canvas.addEventListener('mousedown', event => {\n");
        html.append("      state.dragging = true;\n");
        html.append("      state.button = event.button;\n");
        html.append("      state.lastX = event.clientX;\n");
        html.append("      state.lastY = event.clientY;\n");
        html.append("      canvas.classList.add('dragging');\n");
        html.append("    });\n");
        html.append("    window.addEventListener('mouseup', () => {\n");
        html.append("      state.dragging = false;\n");
        html.append("      canvas.classList.remove('dragging');\n");
        html.append("    });\n");
        html.append("    window.addEventListener('mousemove', event => {\n");
        html.append("      if (!state.dragging) return;\n");
        html.append("      const dx = event.clientX - state.lastX;\n");
        html.append("      const dy = event.clientY - state.lastY;\n");
        html.append("      state.lastX = event.clientX;\n");
        html.append("      state.lastY = event.clientY;\n");
        html.append("      if (state.button === 2) {\n");
        html.append("        state.panX += dx;\n");
        html.append("        state.panY += dy;\n");
        html.append("      } else {\n");
        html.append("        state.yaw -= dx * 0.008;\n");
        html.append("        state.pitch += dy * 0.006;\n");
        html.append("        state.yaw = wrapAngle(state.yaw);\n");
        html.append("      }\n");
        html.append("      render();\n");
        html.append("    });\n");
        html.append("    canvas.addEventListener('wheel', event => {\n");
        html.append("      event.preventDefault();\n");
        html.append("      const factor = event.deltaY < 0 ? 1.08 : 1 / 1.08;\n");
        html.append("      state.zoom = Math.max(0.45, Math.min(4, state.zoom * factor));\n");
        html.append("      render();\n");
        html.append("    }, { passive: false });\n");
        html.append("    canvas.addEventListener('contextmenu', event => event.preventDefault());\n");
        html.append("    window.addEventListener('resize', resizeCanvas);\n");
    html.append("    renderRouteLegend();\n");
    html.append("    resizeCanvas();\n");
        html.append("  </script>\n");
        html.append("</body>\n");
        html.append("</html>\n");
        return html.toString();
    }
}