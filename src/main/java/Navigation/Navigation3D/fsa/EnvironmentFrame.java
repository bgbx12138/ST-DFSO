package Navigation.Navigation3D.fsa;

/**
 * 单日环境场（public 版本，与 NavigationObjective3D 内部的 EnvironmentSnapshot 解耦）。
 * 仅作为数据容器，不承担计算。
 */
public final class EnvironmentFrame {
    public final double[][] windUData;
    public final double[][] windVData;
    public final double[][] currentUData;
    public final double[][] currentVData;
    public final double[][] waveDirData;
    public final double[][] wavePeriodData;
    public final double[][] waveHeightData;

    public EnvironmentFrame(double[][] windUData, double[][] windVData,
                            double[][] currentUData, double[][] currentVData,
                            double[][] waveDirData, double[][] wavePeriodData,
                            double[][] waveHeightData) {
        this.windUData = windUData;
        this.windVData = windVData;
        this.currentUData = currentUData;
        this.currentVData = currentVData;
        this.waveDirData = waveDirData;
        this.wavePeriodData = wavePeriodData;
        this.waveHeightData = waveHeightData;
    }

    public static double safeValue(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
