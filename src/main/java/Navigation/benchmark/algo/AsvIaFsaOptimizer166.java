package Navigation.benchmark.algo;

import Navigation.Navigation3D.HybridFSA3D;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.Navigation3D.fsa.SearchWorkLedger;
import Navigation.benchmark.HybridSwarmOptimizer;
import Navigation.benchmark.PathEvaluator;
import Navigation.benchmark.FairPathEvaluator166;
import Navigation.benchmark.ExternalRun166;
import Navigation.benchmark.SwarmOptResult;
import Navigation.model.GridPoint;
import Navigation.model.GridRouteGraph;
import Navigation.object.NavigationConfig;
import Navigation.object.NavigationResult;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;

/**
 * Graph-route adaptation of Zhao, Wang, and Bai's 2022 ASV IAFSA.
 *
 * The fish state here is a complete ordered route rather than the paper/code's
 * single raster-cell fish. Prey, follow, swarm, weighted random behavior, adaptive
 * visual radius, and waypoint-shortcut smoothing are reconstructed as route-level
 * operators on the shared GridRouteGraph and scored by the exact dynamic evaluator.
 * This is an explicit adaptation, not a code port or exact reproduction.
 */
public final class AsvIaFsaOptimizer166 implements HybridSwarmOptimizer {

    private static final double MIN_VISUAL_CELLS = 2.0;
    private static final double MAX_VISUAL_CELLS = GridRouteGraph.DEFAULT_MAX_EDGE_SPAN_CELLS;
    private static final double CROWDING_FACTOR = 0.618;

    private ExternalRun166 run166;
    public AsvIaFsaOptimizer166 withRun(ExternalRun166 run){this.run166=java.util.Objects.requireNonNull(run);return this;}
    private final int populationSize;
    private final int maxIterations;
    private final double detourKappa;
    private final long maxSubsegmentAttempts;

    private static final class Fish {
        List<GridPoint> route;
        double cost;
        List<GridPoint> bestRoute;
        double bestCost;
        int stagnation;

        Fish(List<GridPoint> route, double cost) {
            this.route = copyRoute(route);
            this.cost = cost;
            this.bestRoute = copyRoute(route);
            this.bestCost = cost;
        }

        Fish snapshot() {
            Fish copy = new Fish(route, cost);
            copy.bestRoute = copyRoute(bestRoute);
            copy.bestCost = bestCost;
            copy.stagnation = stagnation;
            return copy;
        }

        void accept(List<GridPoint> nextRoute, double nextCost, boolean allowWorsening) {
            if (!isValidCost(nextCost)) return;
            if (allowWorsening || nextCost < cost) {
                route = copyRoute(nextRoute);
                cost = nextCost;
            }
            if (nextCost < bestCost) {
                bestRoute = copyRoute(nextRoute);
                bestCost = nextCost;
                stagnation = 0;
            } else {
                stagnation++;
            }
        }
    }

    private static final class Candidate {
        final List<GridPoint> route;
        final double cost;

        Candidate(List<GridPoint> route, double cost) {
            this.route = route;
            this.cost = cost;
        }
    }

    public AsvIaFsaOptimizer166(int populationSize, int maxIterations) {
        this(populationSize, maxIterations, 0.0, 0L);
    }

    public AsvIaFsaOptimizer166(int populationSize, int maxIterations,
                             double detourKappa, long maxSubsegmentAttempts) {
        if (populationSize < 1) throw new IllegalArgumentException("population must be positive");
        if (maxIterations < 1) throw new IllegalArgumentException("iterations must be positive");
        if (detourKappa != 0.0 && (!Double.isFinite(detourKappa) || detourKappa < 1.0))
            throw new IllegalArgumentException("detour kappa must be zero (disabled) or at least 1.0");
        if (maxSubsegmentAttempts < 0L)
            throw new IllegalArgumentException("subsegment budget cannot be negative");
        this.populationSize = populationSize;
        this.maxIterations = maxIterations;
        this.detourKappa = detourKappa;
        this.maxSubsegmentAttempts = maxSubsegmentAttempts;
    }

    @Override
    public String algorithmName() {
        return "ASV-IAFSA-2022-adapted";
    }

    @Override
    public SwarmOptResult optimize(List<GridPoint> seedPath, NavigationConfig config,
                                   HybridFSA3D.ObjectiveMode mode,
                                   ObjectiveWeights weights, long runSeed) {
        if (seedPath == null || seedPath.size() < 3) {
            throw new IllegalArgumentException("IAFSA requires a feasible seed route with an interior waypoint");
        }

        SearchWorkLedger work = java.util.Objects.requireNonNull(run166,"explicit run166 required").ledger;
        try {
            try(FairPathEvaluator166 evaluator = new FairPathEvaluator166(config, work)) {
            GridRouteGraph graph = evaluator.getRouteGraph();
            if (detourKappa > 0.0) evaluator.configureDistanceBudgetFromSeed(seedPath, detourKappa);
            run166.start(maxSubsegmentAttempts);
            double seedCost = run166.checkedSeed(evaluator.evaluateStrict(seedPath, mode, weights));
            if (!isValidCost(seedCost)) {
                throw new IllegalArgumentException("common seed route is invalid under strict IAFSA scoring");
            }

            Random rng = new Random(runSeed);
            List<Fish> fish = initialize(seedPath, seedCost, evaluator, graph, mode, weights, rng, work);
            List<GridPoint> bestRoute = copyRoute(seedPath);
            double bestCost = seedCost;
            for (Fish member : fish) {
                if (member.bestCost < bestCost) {
                    bestCost = member.bestCost;
                    bestRoute = copyRoute(member.bestRoute);
                }
            }

            List<Double> history = new ArrayList<>(maxIterations + 1);
            history.add(bestCost);
            run166.checkpoint(bestCost,true,"initialization_boundary");
            long searchStart = System.currentTimeMillis();

            for (int iteration = 0; iteration < maxIterations && !work.shouldStopSearch(); iteration++) {
                double progress = maxIterations == 1 ? 1.0 : (double) iteration / (maxIterations - 1);
                // Adaptive visual radius contracts from the graph maximum to a local neighborhood.
                double visualCells = MAX_VISUAL_CELLS
                        - (MAX_VISUAL_CELLS - MIN_VISUAL_CELLS) * progress;
                work.recordOperatorEvent("iafsa_adaptive_visual_radius_updates");
                int preyAttempts = Math.max(2, (int) Math.round(10.0 - 7.0 * progress));
                List<Fish> snapshot = new ArrayList<>(fish.size());
                for (Fish member : fish) snapshot.add(member.snapshot());

                for (int i = 0; i < fish.size() && !work.shouldStopSearch(); i++) {
                    Fish current = fish.get(i);
                    Fish observed = snapshot.get(i);
                    if (observed.route.size() < 3) continue;
                    int pivot = 1 + rng.nextInt(observed.route.size() - 2);
                    double randomBehaviorProbability = Math.min(0.60,
                            0.12 + 0.035 * Math.min(14, observed.stagnation));
                    double draw = rng.nextDouble();
                    Candidate move;

                    if (draw < randomBehaviorProbability) {
                        work.recordOperatorEvent("iafsa_random_behavior_attempts");
                        move = randomMove(observed, pivot, visualCells, evaluator, graph,
                                mode, weights, rng);
                    } else {
                        double socialDraw = (draw - randomBehaviorProbability)
                                / Math.max(1e-12, 1.0 - randomBehaviorProbability);
                        List<Fish> neighbors = fishNeighbors(snapshot, i, pivot, visualCells);
                        if (socialDraw < 0.30 && hasBetterNeighbor(observed, neighbors)) {
                            work.recordOperatorEvent("iafsa_follow_attempts");
                            move = followMove(observed, neighbors, pivot, visualCells,
                                    evaluator, graph, mode, weights, rng);
                        } else if (socialDraw < 0.55 && neighbors.size() >= 2
                                && neighbors.size() <= Math.max(1, CROWDING_FACTOR * populationSize)) {
                            work.recordOperatorEvent("iafsa_swarm_attempts");
                            move = swarmMove(observed, neighbors, pivot, visualCells,
                                    evaluator, graph, mode, weights, rng);
                        } else {
                            work.recordOperatorEvent("iafsa_prey_attempts");
                            move = preyMove(observed, pivot, visualCells, preyAttempts,
                                    evaluator, graph, mode, weights, rng, progress, work);
                            if (move == null) {
                                work.recordOperatorEvent("iafsa_prey_random_fallbacks");
                                move = randomMove(observed, pivot, visualCells, evaluator,
                                        graph, mode, weights, rng);
                            }
                        }
                    }

                    if (move != null) {
                        current.accept(move.route, move.cost, draw < randomBehaviorProbability);
                    } else {
                        current.stagnation++;
                    }

                    // Route-level waypoint modification: attempt a graph-feasible shortcut and
                    // keep it only when the same dynamic objective improves on the exact route.
                    if (!work.shouldStopSearch() && current.route.size() > 2 && rng.nextDouble() < 0.18) {
                        work.recordOperatorEvent("iafsa_waypoint_smoothing_attempts");
                        Candidate smoothed = smoothOneWaypoint(current.route, current.cost,
                                evaluator, graph, mode, weights, rng);
                        if (smoothed != null && smoothed.cost < current.cost) {
                            current.accept(smoothed.route, smoothed.cost, false);
                            work.recordOperatorEvent("iafsa_waypoint_smoothing_accepts");
                        }
                    }

                    if (current.bestCost < bestCost) {
                        bestCost = current.bestCost;
                        bestRoute = copyRoute(current.bestRoute);
                    }
                }
                history.add(bestCost);
                run166.checkpoint(bestCost,false,"iteration_boundary");
            }

            long runtimeMs = System.currentTimeMillis() - searchStart;
            run166.finish(bestCost);
            // Unrequested response fields are deliberately absent from the public report.
            // No full-risk postreport is called in this TIME/FUEL-only adapter.
            double[] metrics = {Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN};
            NavigationResult fullResult = null;
            return new SwarmOptResult(algorithmName(), "", mode.name(), 0, runSeed,
                    metrics[0], metrics[2], metrics[4], metrics[1], bestCost,
                    runtimeMs, populationSize, maxIterations, history,
                    fullResult, copyRoute(bestRoute), false, work.snapshot());
            }
        } catch (Exception e) {
            throw new RuntimeException("ASV-IAFSA-2022-adapted optimize failed", e);
        }
    }

    private List<Fish> initialize(List<GridPoint> seedPath, double seedCost,
                                  PathEvaluator evaluator, GridRouteGraph graph,
                                  HybridFSA3D.ObjectiveMode mode, ObjectiveWeights weights,
                                  Random rng, SearchWorkLedger work) {
        List<Fish> population = new ArrayList<>(populationSize);
        population.add(new Fish(seedPath, seedCost));
        for (int i = 1; i < populationSize && !work.shouldStopSearch(); i++) {
            List<GridPoint> route = copyRoute(seedPath);
            double cost = seedCost;
            int attempts = 1 + rng.nextInt(3);
            for (int attempt = 0; attempt < attempts && !work.shouldStopSearch(); attempt++) {
                if (route.size() < 3) break;
                work.recordOperatorEvent("iafsa_initialization_mutation_attempts");
                int pivot = 1 + rng.nextInt(route.size() - 2);
                List<GridPoint> choices = compatibleNeighbors(route, pivot, 4.0, graph);
                if (choices.isEmpty()) continue;
                GridPoint replacement = choices.get(rng.nextInt(choices.size()));
                List<GridPoint> candidate = replace(route, pivot, replacement);
                double candidateCost = evaluator.evaluateStrict(candidate, mode, weights);
                if (isValidCost(candidateCost)) {
                    route = candidate;
                    cost = candidateCost;
                }
            }
            population.add(new Fish(route, cost));
        }
        return population;
    }

    private Candidate preyMove(Fish fish, int pivot, double visualCells, int tryCount,
                               PathEvaluator evaluator, GridRouteGraph graph,
                               HybridFSA3D.ObjectiveMode mode, ObjectiveWeights weights,
                               Random rng, double progress, SearchWorkLedger work) {
        List<GridPoint> all = compatibleNeighbors(fish.route, pivot, visualCells, graph);
        if (all.isEmpty()) return null;
        List<GridPoint> chosen = all;
        // Directional prey is a probabilistic bias toward the goal, not a hard route constraint.
        double directionalProbability = 0.70 - 0.20 * progress;
        if (rng.nextDouble() < directionalProbability) {
            List<GridPoint> towardGoal = new ArrayList<>();
            GridPoint goal = fish.route.get(fish.route.size() - 1);
            double oldDistance = cellDistance(fish.route.get(pivot), goal);
            for (GridPoint point : all) {
                if (cellDistance(point, goal) + 1e-9 < oldDistance) towardGoal.add(point);
            }
            if (!towardGoal.isEmpty()) {
                chosen = towardGoal;
                work.recordOperatorEvent("iafsa_directional_prey_uses");
            }
        }
        Collections.shuffle(chosen, rng);
        Candidate best = null;
        int limit = Math.min(tryCount, chosen.size());
        for (int k = 0; k < limit && !run166.ledger.shouldStopSearch(); k++) {
            work.recordOperatorEvent("iafsa_prey_candidate_trials");
            Candidate candidate = evaluateReplacement(fish.route, pivot, chosen.get(k),
                    evaluator, mode, weights);
            if (candidate != null && candidate.cost < fish.cost
                    && (best == null || candidate.cost < best.cost)) best = candidate;
        }
        return best;
    }

    private Candidate followMove(Fish fish, List<Fish> neighbors, int pivot, double visualCells,
                                 PathEvaluator evaluator, GridRouteGraph graph,
                                 HybridFSA3D.ObjectiveMode mode, ObjectiveWeights weights,
                                 Random rng) {
        Fish leader = null;
        double leaderCost = fish.cost;
        for (Fish neighbor : neighbors) {
            if (neighbor.cost < leaderCost) {
                leader = neighbor;
                leaderCost = neighbor.cost;
            }
        }
        if (leader == null) return null;
        double fraction = (double) pivot / Math.max(1, fish.route.size() - 1);
        GridPoint target = leader.route.get(indexAtFraction(leader.route, fraction));
        return bestMoveToward(fish, pivot, visualCells, target, evaluator, graph,
                mode, weights, rng, true);
    }

    private Candidate swarmMove(Fish fish, List<Fish> neighbors, int pivot, double visualCells,
                                PathEvaluator evaluator, GridRouteGraph graph,
                                HybridFSA3D.ObjectiveMode mode, ObjectiveWeights weights,
                                Random rng) {
        double fraction = (double) pivot / Math.max(1, fish.route.size() - 1);
        double row = 0.0, col = 0.0, meanCost = 0.0;
        for (Fish neighbor : neighbors) {
            GridPoint point = neighbor.route.get(indexAtFraction(neighbor.route, fraction));
            row += point.i;
            col += point.j;
            meanCost += neighbor.cost;
        }
        if (meanCost / neighbors.size() >= fish.cost) return null;
        GridPoint centroid = new GridPoint((int) Math.round(row / neighbors.size()),
                (int) Math.round(col / neighbors.size()));
        return bestMoveToward(fish, pivot, visualCells, centroid, evaluator, graph,
                mode, weights, rng, true);
    }

    private Candidate bestMoveToward(Fish fish, int pivot, double visualCells, GridPoint target,
                                    PathEvaluator evaluator, GridRouteGraph graph,
                                    HybridFSA3D.ObjectiveMode mode, ObjectiveWeights weights,
                                    Random rng, boolean requireImprovement) {
        List<GridPoint> choices = compatibleNeighbors(fish.route, pivot, visualCells, graph);
        choices.sort((a, b) -> Double.compare(cellDistance(a, target), cellDistance(b, target)));
        Candidate best = null;
        int probes = Math.min(3, choices.size());
        for (int k = 0; k < probes && !run166.ledger.shouldStopSearch(); k++) {
            Candidate candidate = evaluateReplacement(fish.route, pivot, choices.get(k),
                    evaluator, mode, weights);
            if (candidate == null) continue;
            if (requireImprovement && candidate.cost >= fish.cost) continue;
            if (best == null || candidate.cost < best.cost) best = candidate;
        }
        return best;
    }

    private Candidate randomMove(Fish fish, int pivot, double visualCells,
                                 PathEvaluator evaluator, GridRouteGraph graph,
                                 HybridFSA3D.ObjectiveMode mode, ObjectiveWeights weights,
                                 Random rng) {
        List<GridPoint> choices = compatibleNeighbors(fish.route, pivot, visualCells, graph);
        if (choices.isEmpty()) return null;
        GridPoint replacement = choices.get(rng.nextInt(choices.size()));
        return evaluateReplacement(fish.route, pivot, replacement, evaluator, mode, weights);
    }

    private Candidate smoothOneWaypoint(List<GridPoint> route, double currentCost,
                                        PathEvaluator evaluator,
                                        GridRouteGraph graph, HybridFSA3D.ObjectiveMode mode,
                                        ObjectiveWeights weights, Random rng) {
        int attempts = Math.min(3, route.size() - 2);
        Candidate best = null;
        for (int attempt = 0; attempt < attempts && !run166.ledger.shouldStopSearch(); attempt++) {
            int pivot = 1 + rng.nextInt(route.size() - 2);
            GridPoint before = route.get(pivot - 1), after = route.get(pivot + 1);
            if (!graph.isValidEdge(before, after)) continue;
            List<GridPoint> candidate = new ArrayList<>(route);
            candidate.remove(pivot);
            double cost = evaluator.evaluateStrict(candidate, mode, weights);
            if (isValidCost(cost) && cost < currentCost
                    && (best == null || cost < best.cost)) {
                best = new Candidate(candidate, cost);
            }
        }
        return best;
    }

    private Candidate evaluateReplacement(List<GridPoint> route, int pivot, GridPoint replacement,
                                         PathEvaluator evaluator, HybridFSA3D.ObjectiveMode mode,
                                         ObjectiveWeights weights) {
        List<GridPoint> candidate = replace(route, pivot, replacement);
        double cost = evaluator.evaluateStrict(candidate, mode, weights);
        return isValidCost(cost) ? new Candidate(candidate, cost) : null;
    }

    private List<GridPoint> compatibleNeighbors(List<GridPoint> route, int pivot,
                                                double visualCells, GridRouteGraph graph) {
        GridPoint current = route.get(pivot);
        GridPoint previous = route.get(pivot - 1), next = route.get(pivot + 1);
        List<GridPoint> choices = new ArrayList<>();
        for (GridPoint candidate : graph.neighbors(current)) {
            if (candidate.equals(previous) || candidate.equals(next)) continue;
            if (cellDistance(current, candidate) > visualCells + 1e-9) continue;
            if (!graph.isValidEdge(previous, candidate) || !graph.isValidEdge(candidate, next)) continue;
            choices.add(candidate);
        }
        return choices;
    }

    private List<Fish> fishNeighbors(List<Fish> population, int fishIndex, int pivot,
                                     double visualCells) {
        Fish focal = population.get(fishIndex);
        double fraction = (double) pivot / Math.max(1, focal.route.size() - 1);
        GridPoint point = focal.route.get(pivot);
        List<Fish> neighbors = new ArrayList<>();
        for (int i = 0; i < population.size(); i++) {
            if (i == fishIndex) continue;
            Fish candidate = population.get(i);
            GridPoint other = candidate.route.get(indexAtFraction(candidate.route, fraction));
            if (cellDistance(point, other) <= visualCells) neighbors.add(candidate);
        }
        return neighbors;
    }

    private boolean hasBetterNeighbor(Fish focal, List<Fish> neighbors) {
        for (Fish neighbor : neighbors) if (neighbor.cost < focal.cost) return true;
        return false;
    }

    private static int indexAtFraction(List<GridPoint> route, double fraction) {
        return Math.max(0, Math.min(route.size() - 1,
                (int) Math.round(fraction * Math.max(0, route.size() - 1))));
    }

    private static List<GridPoint> replace(List<GridPoint> route, int index, GridPoint point) {
        List<GridPoint> copy = copyRoute(route);
        copy.set(index, new GridPoint(point.i, point.j));
        return copy;
    }

    private static List<GridPoint> copyRoute(List<GridPoint> route) {
        List<GridPoint> copy = new ArrayList<>(route.size());
        for (GridPoint point : route) copy.add(new GridPoint(point.i, point.j));
        return copy;
    }

    private static double cellDistance(GridPoint a, GridPoint b) {
        return Math.hypot((double) a.i - b.i, (double) a.j - b.j);
    }

    private static boolean isValidCost(double cost) {
        return Double.isFinite(cost) && cost < 1e17;
    }
}
