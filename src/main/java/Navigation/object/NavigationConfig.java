package Navigation.object;

public class NavigationConfig {
    public String dataFolder;
    public String outputFile;
    public String navigationDate;
    // 兼容旧代码：部分规划器直接读取 departureHour
    public double departureHour;
    public String departureTime;
    public Integer departureSecondsOfDay;
    public String obstacleFileName;
    public String btyFileName;
    public boolean showVisualization;
    public NavigationArea area;
    public NavigationGrid grid;
    public ShipParameters shipParameters;

    // 仅供实验4（综合目标消融）使用；默认值必须等价于基线行为
    public boolean enableCompositeNormalization = true;
    public boolean enableCompositeTimeNormalization = true;
    public boolean enableCompositeFuelNormalization = true;
    public boolean enableCompositeSafetyNormalization = true;
    public boolean enableCompositeShipClassCorrection = true;
    // 当后续日期缺少环境文件时，是否允许延用最后一天环境继续规划。
    // 默认关闭，保持原有行为；实验3会显式开启。
    public boolean enableEnvironmentCarryForwardBeyondAvailableDays = false;
    // 在开启延用环境时，规划天数上限（含起始日）。<=0 时按可用环境天数处理。
    public int environmentPlanningHorizonDays = 0;
    /** Optional hard limit on modeled travel time, measured from route departure. */
    public Double arrivalDeadlineHours;

    public NavigationConfig() {
    }

    public double resolveDepartureOffsetHours() {
        double resolvedHours;

        if (departureSecondsOfDay != null) {
            int seconds = departureSecondsOfDay;
            if (seconds < 0 || seconds >= 24 * 3600) {
                throw new IllegalArgumentException("departureSecondsOfDay 必须在 [0, 86399] 范围内");
            }
            resolvedHours = seconds / 3600.0;
        } else if (departureTime != null && !departureTime.isBlank()) {
            String text = departureTime.trim();
            String[] parts = text.split(":");
            if (parts.length != 3) {
                throw new IllegalArgumentException("departureTime 格式必须为 HH:mm:ss，例如 12:00:00");
            }

            int hour;
            int minute;
            int second;
            try {
                hour = Integer.parseInt(parts[0]);
                minute = Integer.parseInt(parts[1]);
                second = Integer.parseInt(parts[2]);
            } catch (NumberFormatException exception) {
                throw new IllegalArgumentException("departureTime 包含非法数字: " + departureTime, exception);
            }

            if (hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
                throw new IllegalArgumentException("departureTime 超出有效范围(00:00:00~23:59:59): " + departureTime);
            }
            resolvedHours = (hour * 3600 + minute * 60 + second) / 3600.0;
        } else {
            if (!Double.isFinite(departureHour)) {
                throw new IllegalArgumentException("departureHour 不是有效数字");
            }
            resolvedHours = departureHour;
        }

        if (resolvedHours < 0.0 || resolvedHours >= 24.0) {
            throw new IllegalArgumentException("出发时刻必须在 [0,24) 小时范围内，当前值=" + resolvedHours);
        }

        // 同步回旧字段，保证直接读取 departureHour 的旧代码行为一致。
        departureHour = resolvedHours;
        return resolvedHours;
    }

    public double resolveArrivalDeadlineHours() {
        if (arrivalDeadlineHours == null) return Double.POSITIVE_INFINITY;
        if (!Double.isFinite(arrivalDeadlineHours) || arrivalDeadlineHours < 0.0) {
            throw new IllegalArgumentException("arrivalDeadlineHours must be finite and non-negative");
        }
        return arrivalDeadlineHours;
    }
}
