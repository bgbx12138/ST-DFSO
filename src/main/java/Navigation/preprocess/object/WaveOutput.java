package Navigation.preprocess.object;

import java.util.List;

public class WaveOutput extends BaseOutput {
    public List<Double> waveDataD;
    public List<Double> waveDataP;
    public List<Double> waveDataH;

    public WaveOutput() {
    }

    public WaveOutput(String projectName, Area area, Grid gridXY, List<Double> waveDataD, List<Double> waveDataP, List<Double> waveDataH) {
        super(projectName, area, gridXY);
        this.waveDataD = waveDataD;
        this.waveDataP = waveDataP;
        this.waveDataH = waveDataH;
    }
}