package Navigation.Navigation3D;

import Navigation.object.ShipParameters;
import Navigation.utils.ShipUtils;

public final class FuelCostCalculator {

	private static final double WATER_RHO = 1025.0;
	private static final double SEA_WATER_NU_15 = 1.19e-6;
	private static final double DEFAULT_WATER_DEPTH = 20.0;

	private FuelCostCalculator() {
	}

	public static double calculateFuelEnergyJ(ShipParameters ship,
											  double windSpeed,
											  double windDir,
											  double currentSpeed,
											  double currentDir,
											  double currentU,
											  double currentV,
											  double bearing,
											  double segmentHours) {
		return calculateFuelEnergyJ(
				ship,
				windSpeed,
				windDir,
				currentSpeed,
				currentDir,
				currentU,
				currentV,
				bearing,
				segmentHours,
				true
		);
	}

	public static double calculateFuelEnergyJ(ShipParameters ship,
										  double windSpeed,
										  double windDir,
										  double currentSpeed,
										  double currentDir,
										  double currentU,
										  double currentV,
										  double bearing,
										  double segmentHours,
										  boolean enableShipClassCorrection) {
        double w=Math.toRadians(windDir);
        VoyageSpeedModel.State state=VoyageSpeedModel.solve(ship,bearing,windSpeed*Math.cos(w),windSpeed*Math.sin(w),currentU,currentV);
        return state.valid()?VoyageSpeedModel.energyJ(ship,state.waterSpeed,segmentHours):Double.NaN;
    }
}
