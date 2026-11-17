package Navigation.Navigation3D.Normalization;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public final class TimeNormalizer {

    private static final double KNOT_TO_MS = 0.5144;
    private static final double EPS_SPEED_KNOT = 0.1;

    private final double[] minSpeedKnotByDirection;
    private final double[] maxSpeedKnotByDirection;

    private TimeNormalizer(double[] minSpeedKnotByDirection, double[] maxSpeedKnotByDirection) {
        if (minSpeedKnotByDirection == null || maxSpeedKnotByDirection == null
                || minSpeedKnotByDirection.length != maxSpeedKnotByDirection.length) {
            throw new IllegalArgumentException("方向速度边界长度不一致");
        }
        this.minSpeedKnotByDirection = minSpeedKnotByDirection;
        this.maxSpeedKnotByDirection = maxSpeedKnotByDirection;
    }

    public static TimeNormalizer fromDirectionalSamples(double shipSpeedKnot,
                                                        List<Double>[] currentProjectionMsByDirection,
                                                        List<Double>[] windLossMsByDirection,
                                                        List<Double>[] waveLossMsByDirection) {
        int directionCount = currentProjectionMsByDirection.length;
        double[] minSpeedKnot = new double[directionCount];
        double[] maxSpeedKnot = new double[directionCount];

        for (int directionIndex = 0; directionIndex < directionCount; directionIndex++) {
            double q99ProjMs = quantile(currentProjectionMsByDirection[directionIndex], 0.99);
            double q01ProjMs = quantile(currentProjectionMsByDirection[directionIndex], 0.01);
            double q99WindLossMs = quantile(windLossMsByDirection[directionIndex], 0.99);
            double q99WaveLossMs = quantile(waveLossMsByDirection[directionIndex], 0.99);

            double vmax = shipSpeedKnot + q99ProjMs / KNOT_TO_MS;
            double vmin = shipSpeedKnot + q01ProjMs / KNOT_TO_MS
                    - q99WindLossMs / KNOT_TO_MS
                    - q99WaveLossMs / KNOT_TO_MS;

            vmax = Math.max(EPS_SPEED_KNOT, vmax);
            vmin = Math.max(EPS_SPEED_KNOT, vmin);
            if (vmin > vmax) {
                vmin = Math.max(EPS_SPEED_KNOT, 0.7 * vmax);
            }

            minSpeedKnot[directionIndex] = vmin;
            maxSpeedKnot[directionIndex] = vmax;
        }

        return new TimeNormalizer(minSpeedKnot, maxSpeedKnot);
    }

    public static TimeNormalizer fromGroundSpeedSamples(List<Double>[] samples) {
        double[] lo=new double[samples.length], hi=new double[samples.length];
        for(int i=0;i<samples.length;i++) {
            if(samples[i].isEmpty())throw new IllegalArgumentException("NO_VALID_SPEED_NORMALISATION_SAMPLES");
            lo[i]=quantile(samples[i],.01)/KNOT_TO_MS;hi[i]=quantile(samples[i],.99)/KNOT_TO_MS;
        }
        return new TimeNormalizer(lo,hi);
    }

    public double normalize(double timeHours, double distanceNm, int directionIndex) {
        if (!Double.isFinite(timeHours) || !Double.isFinite(distanceNm) || distanceNm <= 0.0) {
            return 1.0;
        }

        double vMin = minSpeedKnotByDirection[directionIndex];
        double vMax = maxSpeedKnotByDirection[directionIndex];
        if (vMax <= vMin) {
            return 0.0;
        }

        double tauMin = distanceNm / vMax;
        double tauMax = distanceNm / vMin;
        if (tauMax <= tauMin) {
            return 0.0;
        }

        double value = (timeHours - tauMin) / (tauMax - tauMin);
        if (value < 0.0) {
            return 0.0;
        }
        if (value > 1.0) {
            return 1.0;
        }
        return value;
    }

    public double getMinSpeedMs(int directionIndex) {
        return minSpeedKnotByDirection[directionIndex] * KNOT_TO_MS;
    }

    public double getMaxSpeedMs(int directionIndex) {
        return maxSpeedKnotByDirection[directionIndex] * KNOT_TO_MS;
    }

    private static double quantile(List<Double> samples, double q) {
        if (samples == null || samples.isEmpty()) {
            return 0.0;
        }
        List<Double> values = new ArrayList<>(samples.size());
        for (double sample : samples) {
            if (Double.isFinite(sample)) {
                values.add(sample);
            }
        }
        if (values.isEmpty()) {
            return 0.0;
        }

        Collections.sort(values);
        double pos = q * (values.size() - 1);
        int left = (int) Math.floor(pos);
        int right = (int) Math.ceil(pos);
        if (left == right) {
            return values.get(left);
        }

        double weight = pos - left;
        return values.get(left) * (1.0 - weight) + values.get(right) * weight;
    }
}
