package Navigation.preprocess.object;

public class BaseOutput {
    public String projectName;
    public Area area;
    public Grid gridXY;

    public BaseOutput() {
    }

    public BaseOutput(String projectName, Area area, Grid gridXY) {
        this.projectName = projectName;
        this.area = area;
        this.gridXY = gridXY;
    }
}