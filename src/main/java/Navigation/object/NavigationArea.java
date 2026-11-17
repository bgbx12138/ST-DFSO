package Navigation.object;

public class NavigationArea {
    public double latMin;
    public double latMax;
    public double lonMin;
    public double lonMax;

    public NavigationArea() {
    }

    public NavigationArea(double latMin, double latMax, double lonMin, double lonMax) {
        this.latMin = latMin;
        this.latMax = latMax;
        this.lonMin = lonMin;
        this.lonMax = lonMax;
    }
}