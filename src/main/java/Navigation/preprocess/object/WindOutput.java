package Navigation.preprocess.object;

import java.util.List;

public class WindOutput extends BaseOutput {
    public List<Double> windDataU;
    public List<Double> windDataV;

    public WindOutput() {
    }

    public WindOutput(String projectName, Area area, Grid gridXY, List<Double> windDataU, List<Double> windDataV) {
        super(projectName, area, gridXY);
        this.windDataU = windDataU;
        this.windDataV = windDataV;
    }
}