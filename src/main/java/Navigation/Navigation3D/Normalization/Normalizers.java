package Navigation.Navigation3D.Normalization;

import Navigation.object.NavigationConfig;
import Navigation.Navigation3D.VoyageSpeedModel;
import Navigation.object.ShipParameters;

import java.util.ArrayList;
import java.util.List;

/** Common directional 1st/99th-percentile scales from the same speed and energy model
 * as voyage integration. No independent wind/wave speed-loss surrogate. */
public final class Normalizers {

    private Normalizers() {
    }

    /**
     * 归一化样本输入：由调用方按日循环提供每一层环境场（只关心 U/V + 波浪），
     * 工厂内部按 directions/bearings 投影并汇总分位样本。
     */
    public interface EnvironmentSampleProvider {
        int numDays();

        int ny();

        int nx();

        /** true 表示该格为障碍、跳过样本采集。 */
        boolean isObstacle(int i, int j);

        double windU(int day, int i, int j);

        double windV(int day, int i, int j);

        double currentU(int day, int i, int j);

        double currentV(int day, int i, int j);

        double waveHeight(int day, int i, int j);

        double wavePeriod(int day, int i, int j);

        double waveDir(int day, int i, int j);
    }

    public static final class Bundle {
        private final TimeNormalizer timeNormalizer;
        private final FuelNormalizer fuelNormalizer;

        public Bundle(TimeNormalizer timeNormalizer, FuelNormalizer fuelNormalizer) {
            this.timeNormalizer = timeNormalizer;
            this.fuelNormalizer = fuelNormalizer;
        }

        public TimeNormalizer timeNormalizer() {
            return timeNormalizer;
        }

        public FuelNormalizer fuelNormalizer() {
            return fuelNormalizer;
        }
    }

    /**
     * 构造 Time + Fuel 归一化器。
     *
     * @param cfg            配置（读取 enableCompositeNormalization / enableCompositeTimeNormalization / enableCompositeFuelNormalization）
     * @param ship           船舶参数（用于 TimeNormalizer 中的 shipSpeed 基线）
     * @param directions     8 邻域方向偏移（与 NavigationObjective3D.directions 一致）
     * @param bearings       8 邻域方位角（与 NavigationObjective3D.bearings 一致）
     * @param provider       环境场样本 provider
     * @return 构造好的 Bundle；若 Composite 未启用/子开关全关，返回 null 标识无需归一化
     */
    public static Bundle build(NavigationConfig cfg,
                               ShipParameters ship,
                               int[][] directions,
                               double[] bearings,
                               EnvironmentSampleProvider provider) {
        boolean useTimeNorm = cfg.enableCompositeNormalization && cfg.enableCompositeTimeNormalization;
        boolean useFuelNorm = cfg.enableCompositeNormalization && cfg.enableCompositeFuelNormalization;
        if (!useTimeNorm && !useFuelNorm) {
            return new Bundle(null, null);
        }

        int directionCount = directions.length;
        @SuppressWarnings("unchecked")
        List<Double>[] groundSpeeds = new List[directionCount], energyPerMetre = new List[directionCount];
        for(int d=0;d<directionCount;d++){groundSpeeds[d]=new ArrayList<>();energyPerMetre[d]=new ArrayList<>();}
        int numDays = provider.numDays();
        int ny = provider.ny();
        int nx = provider.nx();
        for (int day = 0; day < numDays; day++) {
            for (int i = 0; i < ny; i++) {
                for (int j = 0; j < nx; j++) {
                    if (provider.isObstacle(i, j)) {
                        continue;
                    }
                    double currentU = provider.currentU(day, i, j);
                    double currentV = provider.currentV(day, i, j);
                    double windU = provider.windU(day, i, j);
                    double windV = provider.windV(day, i, j);
                    double waveHeight = provider.waveHeight(day, i, j);
                    double wavePeriod = provider.wavePeriod(day, i, j);
                    double waveDir = provider.waveDir(day, i, j);
                    // A source-masked value is unsupported data, not calm weather.
                    // Derive shared normalizers only from complete samples; route
                    // scoring applies the matching fail-closed support rule.
                    if (!Double.isFinite(currentU) || !Double.isFinite(currentV)
                            || !Double.isFinite(windU) || !Double.isFinite(windV)
                            || !Double.isFinite(waveHeight) || !Double.isFinite(wavePeriod)
                            || !Double.isFinite(waveDir)) {
                        continue;
                    }
                    for (int d = 0; d < directionCount; d++) {
                        VoyageSpeedModel.State v=VoyageSpeedModel.solve(ship,bearings[d],windU,windV,currentU,currentV);
                        if(v.valid()) {
                            groundSpeeds[d].add(v.groundSpeed);
                            energyPerMetre[d].add(VoyageSpeedModel.waterResistance(ship,v.waterSpeed)*v.waterSpeed/v.groundSpeed);
                        }

                    }
                }
            }
        }

        TimeNormalizer timeNormalizer = TimeNormalizer.fromGroundSpeedSamples(groundSpeeds);
        FuelNormalizer fuelNormalizer = FuelNormalizer.fromEnergySamples(energyPerMetre);
        return new Bundle(timeNormalizer, fuelNormalizer);
    }

    private static double calculateDirection(double u, double v) {
        double eps = 1e-9;
        if (Math.abs(u) < eps && Math.abs(v) < eps) {
            return 0.0;
        }
        double direction = Math.toDegrees(Math.atan2(v, u));
        return direction < 0 ? direction + 360.0 : direction;
    }
}
