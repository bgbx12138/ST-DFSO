package Navigation.preprocess.object;

import java.util.List;

public class CurrentOutput extends BaseOutput {
    public List<Double> currentDataU;
    public List<Double> currentDataV;

    public CurrentOutput() {
    }

    public CurrentOutput(String projectName, Area area, Grid gridXY, List<Double> currentDataU, List<Double> currentDataV) {
        super(projectName, area, gridXY);
        this.currentDataU = currentDataU;
        this.currentDataV = currentDataV;
    }
}