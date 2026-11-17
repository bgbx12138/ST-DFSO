# Code guide

## Route planning

`Navigation/Navigation3D/HybridFSA3D.java` implements ST-DFSO. Classes under `Navigation/Navigation3D/fsa/` provide candidate construction, individual route memory, the shared archive, search mechanisms and evaluation support.

`Navigation/benchmark/NativeFinalRunner351.java` runs the configured algorithm. `FinalArm351.java` maps the recorded arm identifiers to the complete algorithm and mechanism-removal settings. The comparison launch path uses `AcoOptimizer166`, `AsvIaFsaOptimizer166` and `WindowIlsOptimizer169` in `Navigation/benchmark/algo/`. `BenchmarkLauncher` supplies the six scenario definitions and the shared vessel configuration.

`VoyageSpeedModel.java` solves the wind/propulsion balance from `config/voyage_speed.json`. `ResearchResponsePort` uses the resulting speed for time, energy, midnight integration and response requests; `Normalizers` uses the same model for its scales. `PathEvaluator` and the objective evaluators assemble voyage costs. `Navigation/Navigation3D/fsa/TableEntry211.java` starts the Python response interface used by the Java planner. `FrozenRollBridge.java` supplies the roll integration bridge.

## Ship-response component

The Python entry point is `navigation_safety.unified_response.SafetyFunction`; `navigation_safety/provider.py` exposes it to Java. The response model applies fixed equations and parameters under `vessel_response/`, while `navigation_safety/` supplies numerical solver integration and cache handling.

`navigation_safety/asset_loader.py` loads the hydrodynamic coefficients, geometry and calibrated parameters. `response_dynamics.py` calculates equivalent-linear roll and fixed linear horizontal responses; `wave_heading.py` calculates the wave-induced yaw component. `linear_response.py` applies the fixed controller. Their inputs and checksums are listed in `config/model_files.json`.

The component distinguishes numerical failure from a valid response. A single run defaults to a new cache initialised from the paper common response pool; a batch uses one shared cache for all algorithms and variants. A response from another physical revision is never reused as a current answer. The common pool contains only valid exact-input responses bound to the declared model.

## Configuration and data

`config/experiment.json` defines the 810 distinct experiment configurations. Its scene identifiers match manuscript S1–S6. Algorithm identifiers retain their implementation names; the launcher accepts ST-DFSO, ACO, IAFSA and ILS. See `QUICKSTART.md` and `FINAL_ROUTE_MEASURES.md`.

`data/results/accepted_runs.json` links configurations to stored measures and the coordinate arrays in `data/results/routes.json`. `analysis/result_data.py` reads and checks these arrays. `data/checksums.json` verifies the published result files and excludes the local detailed `runs/` directory. This check does not freeze README files, build configuration or general documentation.

## Maintenance checks

After a code or configuration change, compile the Java sources and run the checks in `RUNNING.md`. The model's own integrity checks bind fixed sources and assets to a runtime revision. Intentional model changes therefore require reviewing those dependencies and using a new response cache; do not merely replace checksums to reuse an old cache.

Generated files belong in `target/` or `outputs/`. The processed paper grids are included. Credentials, unrelated regional downloads, private work notes and editor settings are excluded by `.gitignore`.

`experiment_freeze.json` binds the tested sources, compiled classes, model assets, configuration and package versions. `tools/run_frozen_experiments.py` checks that identity before launching the complete batch and analysis.
