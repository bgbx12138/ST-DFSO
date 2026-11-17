package Navigation.benchmark;

import Navigation.model.GridPoint;
import Navigation.object.NavigationResult;
import Navigation.Navigation3D.fsa.SearchWorkLedger;
import java.util.List;

/** 单次优化运行结果。所有算法返回同一结构。 */
public class SwarmOptResult {
    public final String algorithm, scenario, objective;
    public final int runId;
    public final long seed;
    public final double voyageTimeH, distanceNm, safetyScore, fuelEnergyJ, compositeCost;
    public final long runtimeMs;
    public final int populationSize, maxIterations;
    public final List<Double> deltaHistory;
    public final NavigationResult fullResult;   // FSA only (null for PSO/GWO/ACO)
    public final List<GridPoint> bestPath;       // PSO/GWO/ACO best path (null for FSA)
    public final boolean isBaseline;
    public final SearchWorkLedger.Snapshot searchWork;

    /** PSO/GWO/ACO (no full result) */
    public SwarmOptResult(String algorithm, String scenario, String objective,
            int runId, long seed, double voyageTimeH, double distanceNm,
            double safetyScore, double fuelEnergyJ, double compositeCost,
            long runtimeMs, int populationSize, int maxIterations,
            List<Double> deltaHistory) {
        this(algorithm, scenario, objective, runId, seed, voyageTimeH, distanceNm,
             safetyScore, fuelEnergyJ, compositeCost, runtimeMs,
             populationSize, maxIterations, deltaHistory, null, null, false);
    }

    /** FSA with full NavigationResult */
    public SwarmOptResult(String algorithm, String scenario, String objective,
            int runId, long seed, double voyageTimeH, double distanceNm,
            double safetyScore, double fuelEnergyJ, double compositeCost,
            long runtimeMs, int populationSize, int maxIterations,
            List<Double> deltaHistory, NavigationResult fullResult) {
        this(algorithm, scenario, objective, runId, seed, voyageTimeH, distanceNm,
             safetyScore, fuelEnergyJ, compositeCost, runtimeMs,
             populationSize, maxIterations, deltaHistory, fullResult, null, false);
    }

    /** Full constructor */
    public SwarmOptResult(String algorithm, String scenario, String objective,
            int runId, long seed, double voyageTimeH, double distanceNm,
            double safetyScore, double fuelEnergyJ, double compositeCost,
            long runtimeMs, int populationSize, int maxIterations,
            List<Double> deltaHistory, NavigationResult fullResult,
            List<GridPoint> bestPath, boolean isBaseline) {
        this(algorithm, scenario, objective, runId, seed, voyageTimeH, distanceNm,
                safetyScore, fuelEnergyJ, compositeCost, runtimeMs, populationSize,
                maxIterations, deltaHistory, fullResult, bestPath, isBaseline, null);
    }

    public SwarmOptResult(String algorithm, String scenario, String objective,
            int runId, long seed, double voyageTimeH, double distanceNm,
            double safetyScore, double fuelEnergyJ, double compositeCost,
            long runtimeMs, int populationSize, int maxIterations,
            List<Double> deltaHistory, NavigationResult fullResult,
            List<GridPoint> bestPath, boolean isBaseline,
            SearchWorkLedger.Snapshot searchWork) {
        this.algorithm = algorithm; this.scenario = scenario; this.objective = objective;
        this.runId = runId; this.seed = seed;
        this.voyageTimeH = voyageTimeH; this.distanceNm = distanceNm;
        this.safetyScore = safetyScore; this.fuelEnergyJ = fuelEnergyJ;
        this.compositeCost = compositeCost;
        this.runtimeMs = runtimeMs;
        this.populationSize = populationSize; this.maxIterations = maxIterations;
        this.deltaHistory = deltaHistory;
        this.fullResult = fullResult;
        this.bestPath = bestPath;
        this.isBaseline = isBaseline;
        this.searchWork = searchWork;
    }
}
