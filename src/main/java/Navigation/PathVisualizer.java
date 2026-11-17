package Navigation;

import javax.swing.Icon;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingConstants;
import javax.swing.SwingUtilities;

import Navigation.model.GridPoint;

import java.awt.BasicStroke;
import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Component;
import java.awt.Cursor;
import java.awt.Dimension;
import java.awt.Font;
import java.awt.Graphics;
import java.awt.Graphics2D;
import java.awt.GridLayout;
import java.awt.RenderingHints;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.awt.event.MouseAdapter;
import java.awt.event.MouseEvent;
import java.awt.event.MouseMotionAdapter;
import java.awt.event.MouseWheelEvent;
import java.awt.geom.AffineTransform;
import java.util.ArrayList;
import java.util.List;

public class PathVisualizer extends JFrame {
    private static final int CELL_SIZE = 14;
    private static final int MAP_MARGIN = 24;
    private static final Color GRID_COLOR = new Color(220, 220, 220);
    private static final Color OBSTACLE_COLOR = new Color(70, 70, 70);
    private static final Color BUFFER_COLOR = new Color(255, 200, 200, 180);
    private static final Color PATH_COLOR = new Color(0, 102, 204);
    private static final Color START_COLOR = new Color(34, 139, 34);
    private static final Color END_COLOR = new Color(255, 140, 0);
    private static final Color SWITCH_COLOR = new Color(220, 20, 60);

    private final boolean[][] obstacleMap;
    private final boolean[][] obstacleBufferMap;
    private final List<PathLayer> pathLayers;
    private final int ny;
    private final int nx;

    public PathVisualizer(boolean[][] obstacleMap,
                          boolean[][] obstacleBufferMap,
                          List<GridPoint> path,
                          int ny,
                          int nx,
                          double totalDistance,
                          double totalTime) {
                this(obstacleMap, obstacleBufferMap,
                    List.of(new PathLayer("时间最短路径", PATH_COLOR, path, totalDistance, totalTime, null, null, List.of())),
                    ny, nx, "时间优先 Dijkstra 路径图");
    }

    public PathVisualizer(boolean[][] obstacleMap,
                          boolean[][] obstacleBufferMap,
                          List<GridPoint> path,
                          int ny,
                          int nx,
                          double totalDistance,
                          double totalTime,
                          String windowTitle,
                          String pathLabel) {
                this(obstacleMap, obstacleBufferMap,
                    List.of(new PathLayer(pathLabel, PATH_COLOR, path, totalDistance, totalTime, null, null, List.of())),
                    ny, nx, windowTitle);
                }

                public PathVisualizer(boolean[][] obstacleMap,
                          boolean[][] obstacleBufferMap,
                          List<PathLayer> pathLayers,
                          int ny,
                          int nx,
                          String windowTitle) {
        this.obstacleMap = obstacleMap;
        this.obstacleBufferMap = obstacleBufferMap;
                this.pathLayers = new ArrayList<>(pathLayers);
        this.ny = ny;
        this.nx = nx;

        setTitle(windowTitle);
        setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
        setLayout(new BorderLayout());

                int width = Math.min(nx * CELL_SIZE + 360, 1600);
        int height = Math.min(ny * CELL_SIZE + 160, 900);
        setSize(width, height);
        setLocationRelativeTo(null);

        add(new CanvasPanel(), BorderLayout.CENTER);
        add(createLegendPanel(), BorderLayout.EAST);
    }

    public void showVisualization() {
        setVisible(true);
    }

    private JPanel createLegendPanel() {
        JPanel panel = new JPanel(new GridLayout(0, 1, 0, 8));
        panel.setPreferredSize(new Dimension(320, 0));
        panel.setBackground(Color.WHITE);

        JLabel title = new JLabel("路径信息", SwingConstants.CENTER);
        title.setFont(new Font("微软雅黑", Font.BOLD, 16));
        panel.add(title);

        panel.add(createLegendItem("起点", START_COLOR));
        panel.add(createLegendItem("终点", END_COLOR));
        panel.add(createLegendItem("障碍物", OBSTACLE_COLOR));
        panel.add(createLegendItem("缓冲区", BUFFER_COLOR));
        panel.add(createLegendItem("时间切换点", SWITCH_COLOR));

        for (PathLayer layer : pathLayers) {
            panel.add(createLegendItem(layer.label, layer.color));
            panel.add(createTextLabel(String.format("  距离 %.2f 海里 | 时间 %.2f 小时 | 节点 %d",
                    layer.totalDistance,
                    layer.totalTime,
                    layer.path == null ? 0 : layer.path.size())));

            String safetyText = layer.averageSafetyScore == null
                ? "-"
                : String.format("%.6f", layer.averageSafetyScore);
            String fuelText = layer.totalFuelEnergyJ == null
                ? "-"
                : String.format("%.3e J", layer.totalFuelEnergyJ);
            panel.add(createTextLabel(String.format("  安全 %s / 1 | 油耗 %s", safetyText, fuelText)));
        }

        return panel;
    }

    private JLabel createLegendItem(String text, Color color) {
        JLabel label = new JLabel(text);
        label.setFont(new Font("微软雅黑", Font.PLAIN, 13));
        label.setIcon(new ColorIcon(color));
        return label;
    }

    private JLabel createTextLabel(String text) {
        JLabel label = new JLabel(text);
        label.setFont(new Font("微软雅黑", Font.PLAIN, 12));
        return label;
    }

    private class CanvasPanel extends JPanel {
        private double manualScale = 1.0;
        private double panX = 0.0;
        private double panY = 0.0;
        private int lastMouseX;
        private int lastMouseY;
        private boolean dragging;

        private CanvasPanel() {
            setBackground(Color.WHITE);
            setFocusable(true);
            installInteractions();
        }

        @Override
        protected void paintComponent(Graphics g) {
            super.paintComponent(g);
            Graphics2D g2d = (Graphics2D) g;
            g2d.setRenderingHint(RenderingHints.KEY_ANTIALIASING, RenderingHints.VALUE_ANTIALIAS_ON);

            double baseScale = calculateFitScale();
            double renderScale = baseScale * manualScale;
            double offsetX = MAP_MARGIN + panX;
            double offsetY = MAP_MARGIN + panY;

            AffineTransform oldTransform = g2d.getTransform();
            g2d.translate(offsetX, offsetY);
            g2d.scale(renderScale, renderScale);

            drawGrid(g2d);
            drawPaths(g2d);
            drawSwitchMarkers(g2d);
            drawEndpoints(g2d);

            g2d.setTransform(oldTransform);
            drawViewportHint(g2d, renderScale);
        }

        private void drawGrid(Graphics2D g2d) {
            for (int i = 0; i < ny; i++) {
                for (int j = 0; j < nx; j++) {
                    int x = j * CELL_SIZE;
                    int y = (ny - 1 - i) * CELL_SIZE;

                    if (obstacleBufferMap[i][j] && !obstacleMap[i][j]) {
                        g2d.setColor(BUFFER_COLOR);
                        g2d.fillRect(x, y, CELL_SIZE, CELL_SIZE);
                    }

                    if (obstacleMap[i][j]) {
                        g2d.setColor(OBSTACLE_COLOR);
                        g2d.fillRect(x, y, CELL_SIZE, CELL_SIZE);
                    }

                    g2d.setColor(GRID_COLOR);
                    g2d.drawRect(x, y, CELL_SIZE, CELL_SIZE);
                }
            }
        }

        private void drawPaths(Graphics2D g2d) {
            for (PathLayer layer : pathLayers) {
                if (layer.path == null || layer.path.size() < 2) {
                    continue;
                }

                g2d.setColor(layer.color);
                g2d.setStroke(new BasicStroke(2.5f));
                for (int index = 0; index < layer.path.size() - 1; index++) {
                    GridPoint from = layer.path.get(index);
                    GridPoint to = layer.path.get(index + 1);
                    int x1 = from.j * CELL_SIZE + CELL_SIZE / 2;
                    int y1 = (ny - 1 - from.i) * CELL_SIZE + CELL_SIZE / 2;
                    int x2 = to.j * CELL_SIZE + CELL_SIZE / 2;
                    int y2 = (ny - 1 - to.i) * CELL_SIZE + CELL_SIZE / 2;
                    g2d.drawLine(x1, y1, x2, y2);
                }
            }
        }

        private void drawSwitchMarkers(Graphics2D g2d) {
            g2d.setColor(SWITCH_COLOR);
            for (PathLayer layer : pathLayers) {
                for (MarkerPoint marker : layer.switchPoints) {
                    int x = (int) Math.round(marker.gridX * CELL_SIZE + CELL_SIZE / 2.0);
                    int y = (int) Math.round((ny - 1 - marker.gridY) * CELL_SIZE + CELL_SIZE / 2.0);
                    g2d.fillOval(x - 5, y - 5, 10, 10);
                    g2d.setColor(Color.BLACK);
                    g2d.drawOval(x - 5, y - 5, 10, 10);
                    if (marker.label != null && !marker.label.isBlank()
                        && (marker.label.startsWith("S") || marker.label.startsWith("补给"))) {
                        g2d.setFont(new Font("微软雅黑", Font.BOLD, 11));
                        g2d.drawString(marker.label, x + 8, y - 8);
                    }
                    g2d.setColor(SWITCH_COLOR);
                }
            }
        }

        private void drawEndpoints(Graphics2D g2d) {
            for (PathLayer layer : pathLayers) {
                if (layer.path == null || layer.path.isEmpty()) {
                    continue;
                }

                GridPoint start = layer.path.get(0);
                GridPoint end = layer.path.get(layer.path.size() - 1);
                drawPoint(g2d, start, START_COLOR, "起");
                drawPoint(g2d, end, END_COLOR, "终");
                return;
            }
        }

        private void drawPoint(Graphics2D g2d, GridPoint point, Color color, String label) {
            int x = point.j * CELL_SIZE + CELL_SIZE / 2;
            int y = (ny - 1 - point.i) * CELL_SIZE + CELL_SIZE / 2;
            g2d.setColor(color);
            g2d.fillOval(x - 7, y - 7, 14, 14);
            g2d.setColor(Color.BLACK);
            g2d.drawOval(x - 7, y - 7, 14, 14);
            g2d.setFont(new Font("微软雅黑", Font.BOLD, 12));
            g2d.drawString(label, x + 8, y - 8);
        }

        private double calculateFitScale() {
            double availableWidth = Math.max(120.0, getWidth() - MAP_MARGIN * 2.0);
            double availableHeight = Math.max(120.0, getHeight() - MAP_MARGIN * 2.0);
            double contentWidth = nx * CELL_SIZE;
            double contentHeight = ny * CELL_SIZE;
            return Math.min(availableWidth / contentWidth, availableHeight / contentHeight);
        }

        private void drawViewportHint(Graphics2D g2d, double renderScale) {
            g2d.setColor(new Color(0, 0, 0, 170));
            g2d.setFont(new Font("微软雅黑", Font.PLAIN, 12));
            String hint = String.format("滚轮缩放 | 左键拖拽移动 | 双击或 R 重置 | 当前比例 %.2fx",
                renderScale);
            g2d.drawString(hint, 12, Math.max(18, getHeight() - 12));
        }

        private void installInteractions() {
            addMouseWheelListener(this::handleMouseWheel);

            addMouseListener(new MouseAdapter() {
                @Override
                public void mousePressed(MouseEvent e) {
                    if (!SwingUtilities.isLeftMouseButton(e)) {
                        return;
                    }
                    requestFocusInWindow();
                    dragging = true;
                    lastMouseX = e.getX();
                    lastMouseY = e.getY();
                    setCursor(Cursor.getPredefinedCursor(Cursor.MOVE_CURSOR));
                }

                @Override
                public void mouseReleased(MouseEvent e) {
                    dragging = false;
                    setCursor(Cursor.getDefaultCursor());
                }

                @Override
                public void mouseClicked(MouseEvent e) {
                    if (e.getClickCount() == 2) {
                        resetView();
                    }
                }
            });

            addMouseMotionListener(new MouseMotionAdapter() {
                @Override
                public void mouseDragged(MouseEvent e) {
                    if (!dragging) {
                        return;
                    }
                    panX += e.getX() - lastMouseX;
                    panY += e.getY() - lastMouseY;
                    lastMouseX = e.getX();
                    lastMouseY = e.getY();
                    repaint();
                }
            });

            addKeyListener(new KeyAdapter() {
                @Override
                public void keyPressed(KeyEvent e) {
                    switch (e.getKeyCode()) {
                        case KeyEvent.VK_ADD:
                        case KeyEvent.VK_EQUALS:
                            zoomAtCenter(1.15);
                            break;
                        case KeyEvent.VK_SUBTRACT:
                        case KeyEvent.VK_MINUS:
                            zoomAtCenter(1.0 / 1.15);
                            break;
                        case KeyEvent.VK_R:
                        case KeyEvent.VK_0:
                            resetView();
                            break;
                        default:
                            break;
                    }
                }
            });
        }

        private void handleMouseWheel(MouseWheelEvent e) {
            requestFocusInWindow();
            double factor = e.getPreciseWheelRotation() < 0 ? 1.12 : 1.0 / 1.12;
            zoomAroundPoint(factor, e.getX(), e.getY());
        }

        private void zoomAtCenter(double factor) {
            zoomAroundPoint(factor, getWidth() / 2.0, getHeight() / 2.0);
        }

        private void zoomAroundPoint(double factor, double mouseX, double mouseY) {
            double newScale = clamp(manualScale * factor, 0.2, 8.0);
            factor = newScale / manualScale;
            manualScale = newScale;

            double anchorX = mouseX - MAP_MARGIN;
            double anchorY = mouseY - MAP_MARGIN;
            panX = anchorX - (anchorX - panX) * factor;
            panY = anchorY - (anchorY - panY) * factor;
            repaint();
        }

        private void resetView() {
            manualScale = 1.0;
            panX = 0.0;
            panY = 0.0;
            repaint();
        }

        private double clamp(double value, double min, double max) {
            return Math.max(min, Math.min(max, value));
        }
    }

    private static class ColorIcon implements Icon {
        private final Color color;

        private ColorIcon(Color color) {
            this.color = color;
        }

        @Override
        public void paintIcon(Component c, Graphics g, int x, int y) {
            g.setColor(color);
            g.fillRect(x, y, 14, 14);
            g.setColor(Color.BLACK);
            g.drawRect(x, y, 14, 14);
        }

        @Override
        public int getIconWidth() {
            return 14;
        }

        @Override
        public int getIconHeight() {
            return 14;
        }
    }

    public static class MarkerPoint {
        public final double gridX;
        public final double gridY;
        public final String label;

        public MarkerPoint(double gridX, double gridY, String label) {
            this.gridX = gridX;
            this.gridY = gridY;
            this.label = label;
        }
    }

    public static class PathLayer {
        public final String label;
        public final Color color;
        public final List<GridPoint> path;
        public final double totalDistance;
        public final double totalTime;
        public final Double averageSafetyScore;
        public final Double totalFuelEnergyJ;
        public final List<MarkerPoint> switchPoints;

        public PathLayer(String label,
                         Color color,
                         List<GridPoint> path,
                         double totalDistance,
                         double totalTime,
                         List<MarkerPoint> switchPoints) {
            this(label, color, path, totalDistance, totalTime, null, null, switchPoints);
        }

        public PathLayer(String label,
                         Color color,
                         List<GridPoint> path,
                         double totalDistance,
                         double totalTime,
                         Double averageSafetyScore,
                         Double totalFuelEnergyJ,
                         List<MarkerPoint> switchPoints) {
            this.label = label;
            this.color = color;
            this.path = path;
            this.totalDistance = totalDistance;
            this.totalTime = totalTime;
            this.averageSafetyScore = averageSafetyScore;
            this.totalFuelEnergyJ = totalFuelEnergyJ;
            this.switchPoints = switchPoints == null ? List.of() : switchPoints;
        }
    }
}