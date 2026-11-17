package Navigation.object;

public class ShipParameters {
    public double shipDraft;
    public double shipSpeed;
    public double shipLength;
    public double shipWidth;
    public double shipDisplacement;
    public double fuelConsumptionRate;
    public double ratedPower;
    public double ratedSpeed;

    public ShipParameters() {
    }

    public ShipParameters(double shipDraft, double shipSpeed, double shipLength, double shipWidth,
                          double shipDisplacement) {
        this.shipDraft = shipDraft;
        this.shipSpeed = shipSpeed;
        this.shipLength = shipLength;
        this.shipWidth = shipWidth;
        this.shipDisplacement = shipDisplacement;

    }
}