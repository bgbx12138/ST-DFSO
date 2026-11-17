package Navigation.Navigation3D.Normalization;

import Navigation.object.ShipParameters;
import Navigation.utils.ShipUtils;

public final class DirectionalFlowNormalizerSupport {

    private DirectionalFlowNormalizerSupport() {
    }

    public static double projectCurrentAlongDirection(double currentU, double currentV, int di, int dj) {
        double norm = Math.sqrt(di * di + dj * dj);
        if (norm < 1e-9) {
            return 0.0;
        }
        double eEast = dj / norm;
        double eNorth = di / norm;
        return currentU * eEast + currentV * eNorth;
    }

    public static double estimateWindLossMs(ShipParameters ship,
                                            double windSpeed,
                                            double windDir,
                                            double bearing) {
        double noCurrentSpeed = ShipUtils.calculateActualSpeedMs(
                bearing,
                windSpeed,
                windDir,
                0.0,
                0.0,
                ship.shipSpeed,
                ship.shipLength,
                ship.shipWidth,
                ship.shipDraft,
                ship.shipDisplacement
        );
        double setSpeedMs = Math.max(ship.shipSpeed, 0.1) * 0.5144;
        return Math.max(0.0, setSpeedMs - noCurrentSpeed);
    }

    public static double estimateWaveLossMs(double waveHeight, double wavePeriod, double waveDir, double bearing) {
        if (!Double.isFinite(waveHeight) || waveHeight <= 0.0) {
            return 0.0;
        }
        double period = Double.isFinite(wavePeriod) ? Math.max(0.1, wavePeriod) : 8.0;
        double diff = Math.toRadians(angleDiffDeg(waveDir, bearing));
        double encounterFactor = 0.5 + 0.5 * Math.abs(Math.cos(diff));

        // 经验映射：用于构造归一化边界中的 ΔV_wave^(0.99)
        double loss = 0.12 * waveHeight * encounterFactor * Math.sqrt(8.0 / period);
        return clamp(loss, 0.0, 2.0);
    }

    private static double angleDiffDeg(double a, double b) {
        double d = ((a - b) % 360.0 + 360.0) % 360.0;
        return d > 180.0 ? 360.0 - d : d;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
