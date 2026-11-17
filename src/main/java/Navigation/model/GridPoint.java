package Navigation.model;

public class GridPoint {
    public int i;
    public int j;

    public GridPoint() {}

    public GridPoint(int i, int j) {
        this.i = i;
        this.j = j;
    }

    @Override
    public String toString() {
        return String.format("[%d, %d]", i, j);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        GridPoint that = (GridPoint) obj;
        return this.i == that.i && this.j == that.j;
    }

    @Override
    public int hashCode() {
        return i * 31 + j;
    }
}
