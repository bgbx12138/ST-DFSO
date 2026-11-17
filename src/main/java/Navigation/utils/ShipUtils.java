package Navigation.utils;

public class ShipUtils {

    public static int windSpeedToBeaufort(double windSpeed) {
        if (windSpeed < 0.3) return 0;
        else if (windSpeed < 1.6) return 1;
        else if (windSpeed < 3.4) return 2;
        else if (windSpeed < 5.5) return 3;
        else if (windSpeed < 8.0) return 4;
        else if (windSpeed < 10.8) return 5;
        else if (windSpeed < 13.9) return 6;
        else if (windSpeed < 17.2) return 7;
        else if (windSpeed < 20.8) return 8;
        else if (windSpeed < 24.5) return 9;
        else if (windSpeed < 28.5) return 10;
        else if (windSpeed < 32.7) return 11;
        else return 12;
    }

    public static double calculateTimeCost(double dist, double bearing,
                                           double windSpeed, double windDir,
                                           double currentSpeed, double currentDir,
                                           double shipSpeed, double shipLength,
                                           double shipWidth, double shipDraft,
                                           double shipDisplacement) {
        return calculateTimeCost(
            dist,
            bearing,
            windSpeed,
            windDir,
            currentSpeed,
            currentDir,
            shipSpeed,
            shipLength,
            shipWidth,
            shipDraft,
            shipDisplacement,
            true
        );
    }

    public static double calculateTimeCost(double dist, double bearing,
                                           double windSpeed, double windDir,
                                           double currentSpeed, double currentDir,
                                           double shipSpeed, double shipLength,
                                           double shipWidth, double shipDraft,
                                           double shipDisplacement,
                                           boolean enableShipClassCorrection) {

        double actualSpeedMs = calculateActualSpeedMs(
            bearing,
            windSpeed,
            windDir,
            currentSpeed,
            currentDir,
            shipSpeed,
            shipLength,
            shipWidth,
            shipDraft,
            shipDisplacement,
            enableShipClassCorrection
        );

        return dist / (actualSpeedMs / 0.5144);
        }

        public static double calculateActualSpeedMs(double bearing,
                            double windSpeed,
                            double windDir,
                            double currentSpeed,
                            double currentDir,
                            double shipSpeed,
                            double shipLength,
                            double shipWidth,
                            double shipDraft,
                            double shipDisplacement) {
        return calculateActualSpeedMs(
            bearing,
            windSpeed,
            windDir,
            currentSpeed,
            currentDir,
            shipSpeed,
            shipLength,
            shipWidth,
            shipDraft,
            shipDisplacement,
            true
        );
    }

        public static double calculateActualSpeedMs(double bearing,
                            double windSpeed,
                            double windDir,
                            double currentSpeed,
                            double currentDir,
                            double shipSpeed,
                            double shipLength,
                            double shipWidth,
                            double shipDraft,
                            double shipDisplacement,
                            boolean enableShipClassCorrection) {

        Navigation.object.ShipParameters ship=new Navigation.object.ShipParameters(shipDraft,shipSpeed,shipLength,shipWidth,shipDisplacement);
        double w=Math.toRadians(windDir),c=Math.toRadians(currentDir);
        Navigation.Navigation3D.VoyageSpeedModel.State state=Navigation.Navigation3D.VoyageSpeedModel.solve(ship,bearing,
            windSpeed*Math.cos(w),windSpeed*Math.sin(w),currentSpeed*Math.cos(c),currentSpeed*Math.sin(c));
        return state.valid()?state.groundSpeed:Double.NaN;
    }
    public static double calculateFuelCost(double dist, double actualSpeed,
                                           double fuelConsumptionRate, double ratedPower,
                                           double shipRatedSpeed) {
        // 公式: fuel = (fuel_consumption_rate * rated_power * dist * (actual_speed)^2) / (ship_rated_speed^3 * 1000)
        double fuelCost = (fuelConsumptionRate * ratedPower * dist * Math.pow(actualSpeed, 2))
                / (Math.pow(shipRatedSpeed, 3) * 1000.0);
        return fuelCost;
    }
    // 添加角度转换辅助方法
    public static double normalizeAngle(double angle) {
        while (angle < 0) angle += 360;
        while (angle >= 360) angle -= 360;
        return angle;
    }
}
