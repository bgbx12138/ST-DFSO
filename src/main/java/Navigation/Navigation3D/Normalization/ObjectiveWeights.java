package Navigation.Navigation3D.Normalization;

public final class ObjectiveWeights {

    public final double safety;
    public final double time;
    public final double fuel;

    public ObjectiveWeights(double safety, double time, double fuel) {
        this.safety = safety;
        this.time = time;
        this.fuel = fuel;
    }

    public double sum() {
        return safety + time + fuel;
    }

    public ObjectiveWeights normalized() {
        double total = sum();
        if (!Double.isFinite(total) || total <= 0.0) {
            throw new IllegalArgumentException("权重之和必须大于 0");
        }
        return new ObjectiveWeights(safety / total, time / total, fuel / total);
    }
}
