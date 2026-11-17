package Navigation.utils;


import Navigation.model.GridPoint;
import Navigation.model.Point;

public class MathUtils {

    private static final double EARTH_RADIUS_KM = 6371.0;
    private static final double NM_TO_KM = 1.852;

    public static double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
        double lat1Rad = Math.toRadians(lat1);
        double lon1Rad = Math.toRadians(lon1);
        double lat2Rad = Math.toRadians(lat2);
        double lon2Rad = Math.toRadians(lon2);

        double dLat = lat2Rad - lat1Rad;
        double dLon = lon2Rad - lon1Rad;

        double a = Math.sin(dLat/2) * Math.sin(dLat/2) +
                Math.cos(lat1Rad) * Math.cos(lat2Rad) *
                        Math.sin(dLon/2) * Math.sin(dLon/2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1-a));

        return EARTH_RADIUS_KM * c / NM_TO_KM; // 转换为海里
    }

    public static double calculateBearing(double lat1, double lon1, double lat2, double lon2) {
        double lat1Rad = Math.toRadians(lat1);
        double lon1Rad = Math.toRadians(lon1);
        double lat2Rad = Math.toRadians(lat2);
        double lon2Rad = Math.toRadians(lon2);

        double y = Math.sin(lon2Rad - lon1Rad) * Math.cos(lat2Rad);
        double x = Math.cos(lat1Rad) * Math.sin(lat2Rad) -
                Math.sin(lat1Rad) * Math.cos(lat2Rad) * Math.cos(lon2Rad - lon1Rad);

        double bearingRad = Math.atan2(y, x);
        double bearing = Math.toDegrees(bearingRad);

        return (bearing + 360) % 360;
    }

    public static double[] calculateUnitVector(double dx, double dy) {
        double length = Math.sqrt(dx * dx + dy * dy);
        if (length < 1e-10) {
            return new double[]{1.0, 0.0};
        }
        return new double[]{dx / length, dy / length};
    }

    public static double[] calculateNormalVector(double[] direction) {
        return new double[]{-direction[1], direction[0]};
    }

    public static double deg2rad(double deg) {
        return deg * Math.PI / 180.0;
    }

    public static double rad2deg(double rad) {
        return rad * 180.0 / Math.PI;
    }

    public static double atan2d(double y, double x) {
        return rad2deg(Math.atan2(y, x));
    }

    // 新增：计算两点之间的网格距离
    public static double calculateGridDistance(int di, int dj, double dLatNm, double dLonNm, double diagDist) {
        if (Math.abs(di) == 1 && Math.abs(dj) == 1) {
            return diagDist;  // 对角线距离
        } else if (di != 0) {
            return dLatNm;   // 南北方向
        } else {
            return dLonNm;   // 东西方向
        }
    }

    // 新增：计算网格索引对应的经纬度
    public static Point gridIndexToLatLon(int i, int j, double latMin, double lonMin, double deltaLat, double deltaLon) {
        double lat = latMin + (i - 1) * deltaLat;
        double lon = lonMin + (j - 1) * deltaLon;
        return new Point(lon, lat);
    }

    // 新增：经纬度转换为网格索引
    public static GridPoint latLonToGridIndex(double lat, double lon, double latMin, double lonMin, double deltaLat, double deltaLon, int ny, int nx) {
        int iIndex = (int) Math.round((lat - latMin) / deltaLat) + 1;
        int jIndex = (int) Math.round((lon - lonMin) / deltaLon) + 1;

        // 边界检查
        iIndex = Math.max(1, Math.min(ny, iIndex));
        jIndex = Math.max(1, Math.min(nx, jIndex));

        return new GridPoint(iIndex, jIndex);
    }
}
