package Navigation.Navigation3D.Normalization;

import Navigation.object.ShipParameters;

import java.util.EnumMap;
import java.util.Map;

public final class SafetyNormalizer {

    private static final double SEA_WATER_DENSITY = 1025.0;

    private static final double YAW_VMIN = 0.5;
    private static final double YAW_VMAX = 10.0;
    private static final double SWAY_VMIN = 0.2;
    private static final double SWAY_VMAX = 2.5;
    private static final double SURGE_VMIN = 0.5;
    private static final double SURGE_VMAX = 12.0;

    private static final Map<ShipType, RollThreshold> ROLL_THRESHOLDS = createRollThresholds();

    public double normalizeTap4dRisk(double rollAngleDeg,
                                     double yawRateDegPerSec,
                                     double swaySpeedMps,
                                     double surgeSpeedMps,
                                     ShipParameters shipParameters) {
        ShipType shipType = classifyShipType(shipParameters);
        RollThreshold rollThreshold = ROLL_THRESHOLDS.get(shipType);

        double sRoll = normalizeByMinMax(rollAngleDeg, rollThreshold.vMin, rollThreshold.vMax);
        double sYaw = normalizeByMinMax(yawRateDegPerSec, YAW_VMIN, YAW_VMAX);
        double sSway = normalizeByMinMax(swaySpeedMps, SWAY_VMIN, SWAY_VMAX);
        double sSurge = normalizeByMinMax(surgeSpeedMps, SURGE_VMIN, SURGE_VMAX);

        // TAP-4D 风险指数：越接近 1 越危险
        return clamp(0.45 * sRoll + 0.25 * sYaw + 0.20 * sSway + 0.10 * sSurge, 0.0, 1.0);
    }

    /**
     * Re-score the same four response channels with an explicit weight vector and
     * symmetrically expanded working bounds. This is intended for offline sensitivity
     * analysis; the canonical benchmark continues to use normalizeTap4dRisk().
     *
     * @param boundExpansion factor >= 1 applied to each range width around its midpoint;
     *                       lower bounds are clipped at zero because all inputs are magnitudes
     * @param weights        roll, yaw, sway, surge weights; values are normalized to sum to one
     */
    public double normalizeTap4dRiskWithSensitivity(double rollAngleDeg,
                                                    double yawRateDegPerSec,
                                                    double swaySpeedMps,
                                                    double surgeSpeedMps,
                                                    ShipParameters shipParameters,
                                                    double boundExpansion,
                                                    double[] weights) {
        if (!Double.isFinite(boundExpansion) || boundExpansion < 1.0) {
            throw new IllegalArgumentException("boundExpansion must be finite and >= 1");
        }
        if (weights == null || weights.length != 4) {
            throw new IllegalArgumentException("weights must contain roll, yaw, sway, and surge");
        }
        double totalWeight = 0.0;
        for (double weight : weights) {
            if (!Double.isFinite(weight) || weight < 0.0) {
                throw new IllegalArgumentException("weights must be finite and nonnegative");
            }
            totalWeight += weight;
        }
        if (totalWeight <= 0.0) {
            throw new IllegalArgumentException("at least one weight must be positive");
        }

        RollThreshold rollThreshold = ROLL_THRESHOLDS.get(classifyShipType(shipParameters));
        double sRoll = normalizeExpanded(rollAngleDeg, rollThreshold.vMin, rollThreshold.vMax, boundExpansion);
        double sYaw = normalizeExpanded(yawRateDegPerSec, YAW_VMIN, YAW_VMAX, boundExpansion);
        double sSway = normalizeExpanded(swaySpeedMps, SWAY_VMIN, SWAY_VMAX, boundExpansion);
        double sSurge = normalizeExpanded(surgeSpeedMps, SURGE_VMIN, SURGE_VMAX, boundExpansion);

        double risk = (weights[0] / totalWeight) * sRoll
                + (weights[1] / totalWeight) * sYaw
                + (weights[2] / totalWeight) * sSway
                + (weights[3] / totalWeight) * sSurge;
        return clamp(risk, 0.0, 1.0);
    }

    private double normalizeExpanded(double value, double vMin, double vMax, double expansion) {
        double extraEachSide = (expansion - 1.0) * (vMax - vMin) * 0.5;
        double expandedMin = Math.max(0.0, vMin - extraEachSide);
        double expandedMax = vMax + extraEachSide;
        return normalizeByMinMax(value, expandedMin, expandedMax);
    }

    public double toSafetyIndex0To10(double tap4dRisk) {
        return clamp((1.0 - tap4dRisk) * 10.0, 0.0, 10.0);
    }

    public double safetyIndexToRisk01(double safetyIndex0To10) {
        return clamp((10.0 - safetyIndex0To10) / 10.0, 0.0, 1.0);
    }

    public ShipType classifyShipType(ShipParameters shipParameters) {
        double l = Math.max(shipParameters.shipLength, 0.1);
        double b = Math.max(shipParameters.shipWidth, 0.1);
        double t = Math.max(shipParameters.shipDraft, 0.1);
        double displacementKg = Math.max(shipParameters.shipDisplacement, 1.0) * 1000.0;

        // 方形系数 CB = P / (rho_sw * L * B * T)
        double cb = displacementKg / (SEA_WATER_DENSITY * l * b * t);
        // 固有稳性比 SRPI = B / T
        double srpi = b / t;

        HullBand hullBand;
        if (cb < 0.70) {
            hullBand = HullBand.SLENDER;
        } else if (cb < 0.80) {
            hullBand = HullBand.MEDIUM;
        } else {
            hullBand = HullBand.FULL;
        }

        StiffnessBand stiffnessBand;
        if (srpi < 3.0) {
            stiffnessBand = StiffnessBand.TENDER;
        } else if (srpi < 4.5) {
            stiffnessBand = StiffnessBand.MODERATE;
        } else {
            stiffnessBand = StiffnessBand.STIFF;
        }

        return mapShipType(hullBand, stiffnessBand);
    }

    private double normalizeByMinMax(double value, double vMin, double vMax) {
        if (!Double.isFinite(value)) {
            return 1.0;
        }
        if (value <= vMin) {
            return 0.0;
        }
        if (value >= vMax) {
            return 1.0;
        }
        return (value - vMin) / (vMax - vMin);
    }

    private ShipType mapShipType(HullBand hullBand, StiffnessBand stiffnessBand) {
        if (hullBand == HullBand.SLENDER && stiffnessBand == StiffnessBand.TENDER) {
            return ShipType.A;
        }
        if (hullBand == HullBand.SLENDER && stiffnessBand == StiffnessBand.MODERATE) {
            return ShipType.B;
        }
        if (hullBand == HullBand.SLENDER) {
            return ShipType.C;
        }
        if (hullBand == HullBand.MEDIUM && stiffnessBand == StiffnessBand.TENDER) {
            return ShipType.D;
        }
        if (hullBand == HullBand.MEDIUM && stiffnessBand == StiffnessBand.MODERATE) {
            return ShipType.E;
        }
        if (hullBand == HullBand.MEDIUM) {
            return ShipType.F;
        }
        if (stiffnessBand == StiffnessBand.TENDER) {
            return ShipType.G;
        }
        if (stiffnessBand == StiffnessBand.MODERATE) {
            return ShipType.H;
        }
        return ShipType.I;
    }

    private static Map<ShipType, RollThreshold> createRollThresholds() {
        Map<ShipType, RollThreshold> map = new EnumMap<>(ShipType.class);
        map.put(ShipType.A, new RollThreshold(1.2, 20.0));
        map.put(ShipType.B, new RollThreshold(1.0, 18.0));
        map.put(ShipType.C, new RollThreshold(0.8, 15.0));
        map.put(ShipType.D, new RollThreshold(1.8, 24.0));
        map.put(ShipType.E, new RollThreshold(1.6, 22.0));
        map.put(ShipType.F, new RollThreshold(1.4, 20.0));
        map.put(ShipType.G, new RollThreshold(2.8, 28.0));
        map.put(ShipType.H, new RollThreshold(2.6, 26.0));
        map.put(ShipType.I, new RollThreshold(2.4, 25.0));
        return map;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public enum ShipType {
        A, B, C, D, E, F, G, H, I
    }

    private enum HullBand {
        SLENDER,
        MEDIUM,
        FULL
    }

    private enum StiffnessBand {
        TENDER,
        MODERATE,
        STIFF
    }

    private static final class RollThreshold {
        final double vMin;
        final double vMax;

        RollThreshold(double vMin, double vMax) {
            this.vMin = vMin;
            this.vMax = vMax;
        }
    }
}
