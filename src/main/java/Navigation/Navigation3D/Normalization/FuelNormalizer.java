package Navigation.Navigation3D.Normalization;

import Navigation.object.ShipParameters;

public final class FuelNormalizer {

    private static final double WATER_RHO = 1025.0;
    private static final double SEA_WATER_NU_15 = 1.19e-6;
    private static final double DEFAULT_WATER_DEPTH = 20.0;

    private final double[] minEnergyPerMeterByDirection;
    private final double[] maxEnergyPerMeterByDirection;

    private FuelNormalizer(double[] minEnergyPerMeterByDirection,
                           double[] maxEnergyPerMeterByDirection) {
        if (minEnergyPerMeterByDirection == null || maxEnergyPerMeterByDirection == null
                || minEnergyPerMeterByDirection.length != maxEnergyPerMeterByDirection.length) {
            throw new IllegalArgumentException("方向能耗边界长度不一致");
        }
        this.minEnergyPerMeterByDirection = minEnergyPerMeterByDirection;
        this.maxEnergyPerMeterByDirection = maxEnergyPerMeterByDirection;
    }

    public static FuelNormalizer fromTimeNormalizer(ShipParameters ship, TimeNormalizer timeNormalizer, int directionCount) {
        double[] minEnergyPerMeter = new double[directionCount];
        double[] maxEnergyPerMeter = new double[directionCount];

        for (int directionIndex = 0; directionIndex < directionCount; directionIndex++) {
            double vMinMs = Math.max(0.1, timeNormalizer.getMinSpeedMs(directionIndex));
            double vMaxMs = Math.max(0.1, timeNormalizer.getMaxSpeedMs(directionIndex));

            double eMin = calculateEnergyPerMeter(ship, vMinMs);
            double eMax = calculateEnergyPerMeter(ship, vMaxMs);
            if (eMin > eMax) {
                double tmp = eMin;
                eMin = eMax;
                eMax = tmp;
            }

            minEnergyPerMeter[directionIndex] = eMin;
            maxEnergyPerMeter[directionIndex] = eMax;
        }

        return new FuelNormalizer(minEnergyPerMeter, maxEnergyPerMeter);
    }

    public static FuelNormalizer fromEnergySamples(java.util.List<Double>[] samples) {
        double[] lo=new double[samples.length],hi=new double[samples.length];
        for(int i=0;i<samples.length;i++) {
            java.util.List<Double> s=new java.util.ArrayList<>(samples[i]);java.util.Collections.sort(s);
            if(s.isEmpty())throw new IllegalArgumentException("NO_VALID_ENERGY_NORMALISATION_SAMPLES");
            lo[i]=quantile(s,.01);hi[i]=quantile(s,.99);
        }
        return new FuelNormalizer(lo,hi);
    }
    private static double quantile(java.util.List<Double> s,double q) {
        double pos=q*(s.size()-1);int a=(int)Math.floor(pos),b=(int)Math.ceil(pos);
        return s.get(a)*(b-pos)+s.get(b)*(pos-a)+(a==b?s.get(a):0);
    }

    public double normalize(double fuelCostJ, double distanceNm, int directionIndex) {
        if (!Double.isFinite(fuelCostJ) || !Double.isFinite(distanceNm) || distanceNm <= 0.0) {
            return 1.0;
        }

        double distanceM = distanceNm * 1852.0;
        double min = minEnergyPerMeterByDirection[directionIndex] * distanceM;
        double max = maxEnergyPerMeterByDirection[directionIndex] * distanceM;
        if (max <= min) {
            return 0.0;
        }

        double value = (fuelCostJ - min) / (max - min);
        if (value < 0.0) {
            return 0.0;
        }
        if (value > 1.0) {
            return 1.0;
        }
        return value;
    }

    private static double calculateEnergyPerMeter(ShipParameters ship, double vrMs) {
        double lBp = Math.max(ship.shipLength, 1.0);
        double b = Math.max(ship.shipWidth, 0.1);
        double d = Math.max(ship.shipDraft, 0.1);
        double lWl = lBp / 0.97;

        double displacementKg = Math.max(ship.shipDisplacement, 1.0) * 1000.0;
        double cb = displacementKg / (WATER_RHO * Math.max(lBp * b * d, 1e-6));
        cb = clamp(cb, 0.2, 0.95);

        double wettedArea = 1.7 * lWl * d + cb * lWl * b;
        double reynolds = (vrMs * lWl) / SEA_WATER_NU_15;
        reynolds = Math.max(reynolds, 1e5);

        double cf = 0.075 / Math.pow(Math.log10(reynolds) - 2.0, 2.0) + 0.0004;
        double cp = (DEFAULT_WATER_DEPTH / d <= 1.5) ? 0.27 : 0.17;
        double cd = cf + cp * (b * d) / Math.max(wettedArea, 1e-6);

        // E = 0.5*rho*Cd*S*Vr^3*t, 且 t = distance/Vr -> E_per_meter = 0.5*rho*Cd*S*Vr^2
        return 0.5 * WATER_RHO * cd * wettedArea * vrMs * vrMs;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
