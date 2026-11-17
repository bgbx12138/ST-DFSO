package Navigation.object;

public class NavigationGrid {
    public int nx;
    public int ny;
    public int startX;
    public int startY;
    public int endX;
    public int endY;

    public NavigationGrid() {
    }

    public NavigationGrid(int nx, int ny, int startX, int startY, int endX, int endY) {
        this.nx = nx;
        this.ny = ny;
        this.startX = startX;
        this.startY = startY;
        this.endX = endX;
        this.endY = endY;
    }
}