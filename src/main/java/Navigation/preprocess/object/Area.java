package Navigation.preprocess.object;

public class Area {
    public double lonMin;
    public double lonMax;
    public double latMin;
    public double latMax;

    public Area() {
    }

    public Area(double lonMin, double lonMax, double latMin, double latMax) {
        this.lonMin = lonMin;
        this.lonMax = lonMax;
        this.latMin = latMin;
        this.latMax = latMax;
    }
}