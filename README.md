# ST-DFSO

Research code and data for space–time discrete fish swarm optimisation (ST-DFSO) for multi-day ship routing. The repository contains the planner, a shared four-component ship-response model, ACO, IAFSA and window ILS comparisons, five mechanism-ablation variants, and all 810 recorded optimisation runs.

[中文说明](README_中文.md) · [Quick start](docs/QUICKSTART.md) · [Result data](docs/RESULTS.md) · [Model](docs/RESPONSE_MODEL.md)

## Repository contents

| Directory | Contents |
|---|---|
| `src/main/java/` | ST-DFSO, comparison algorithms, route construction and voyage-cost calculations |
| `src/main/python/` | Ship-response calculations, fixed model assets and the Java–Python interface |
| `config/` | Six scenarios, initial routes, ship parameters and the 810 experiment configurations |
| `data/environment/` | The processed environmental grids used in the paper; no external download required |
| `data/model/` | Model inputs and common initial-route responses |
| `data/results/` | All recorded outcomes, final-route coordinates and statistical tables |
| `analysis/` | Statistical reproduction and final-route exports |
| `tools/` | Build/input checks, measured experiment launchers and validation utilities |
| `docs/` | Usage, model sources, data definitions and licence information |

## Installation and checks

The measured experiments use Windows, Java 11, Maven and Python 3.9.9. The Python package versions are specified in `requirements.txt`. Run commands from the repository root:

```sh
python -m pip install -r requirements.txt
mvn package dependency:copy-dependencies -DskipTests
python tools/check_repository.py
python tools/check_model.py
python tools/check_environment.py --historical-hashes
```

The input check compares the included grids with their recorded checksums. The repository check verifies the 810 outcome identities, route coordinates, result files and model inputs. The model check initialises the response interface.

## Reproduce the paper statistics

```sh
python analysis/reproduce_results.py --output outputs/paper-statistics
```

This reads the recorded outcomes and reproduces the CPU-budget comparisons, final-route comparisons, cross-objective results and S6 decompositions. It does not run a new optimisation. The six CSV outputs under `outputs/paper-statistics/data/` correspond to the publication files in `data/results/tables/`. Use a new output directory for each invocation. [Data definitions and paper correspondence](docs/DATA_GUIDE.md).

## Run a route search

```sh
python tools/run_case.py --scenario S1 --objective TIME --algorithm ST-DFSO --output outputs/example --execute
```

For a short installation check, also add `--smoke`. Smoke runs use reduced limits and are not paper results. All six scenarios, five objectives and nine algorithm/ablation settings are available; see [the quick start](docs/QUICKSTART.md).

## Run all experiments

```sh
python tools/run_frozen_experiments.py --check-only
python tools/run_frozen_experiments.py --output outputs/paper-experiments
```

The launcher checks the declared source, input and dependency identity, then runs all 810 configurations and the analysis. There are 360 main-comparison runs and 540 ablation rows, with 90 complete-algorithm runs shared between the two analyses. Experiments run sequentially and use the same 441 exact initial-route responses. The common pool is prepared offline; new response calculations and final route checks are included in the measured per-run CPU. See [experiment settings and accounting](docs/EXPERIMENTS.md).

New runs record their own CPU times. Hardware and time-limited searches can affect their outcomes. The supplied recorded results allow exact reproduction of the paper's statistical summaries without rerunning the batch.

## Licence

Original project material is available under the [ST-DFSO Non-Commercial Academic Research Licence](LICENSE). Academic research, reproducibility and teaching are permitted; commercial use requires separate written permission. This is source-available research software, not an OSI-approved open-source licence. [Third-party materials](THIRD_PARTY_NOTICES.md) retain their own terms.
