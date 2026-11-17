# Quick start

Run commands from the repository root. Use Windows, Java 11, Maven and Python 3.9.9 for measured experiments.

```sh
python -m pip install -r requirements.txt
mvn package dependency:copy-dependencies -DskipTests
python tools/check_repository.py
python tools/check_environment.py --historical-hashes
python tools/check_model.py
```

## Read and reproduce recorded results

```sh
python analysis/reproduce_results.py --output outputs/paper-statistics
python analysis/export_route_metrics.py --output outputs/final_route_metrics.csv
```

These commands use the included results and do not require a new optimisation. They refuse existing output locations. See [the data guide](DATA_GUIDE.md).

## Select an experiment

```sh
python tools/run_case.py --scenario S1 --objective TIME --algorithm ST-DFSO --replicate 1 --output outputs/example --execute
```

Omit `--execute` to display settings. Add `--smoke` for a reduced installation check.

| Option | Values |
|---|---|
| `--scenario` | `S1` to `S6` |
| `--objective` | `TIME`, `ENERGY`, `MEAN_MOTION`, `EXPOSURE`, `COMPOSITE` |
| `--algorithm` | `ST-DFSO`, `ACO`, `IAFSA`, `ILS` |
| `--replicate` | `1`, `2`, `3` |
| `--variant` | ST-DFSO: `FULL`, `MINUS_LL`, `MINUS_AV`, `MINUS_FG`, `MINUS_ER`, `CORE` |

S1 is the Taiwan Strait, S2 the South China Sea, S3 the Miyako Strait, S4 the Bashi Channel, S5 waters near Penghu, and S6 the Qingdao–Busan voyage. S1 and S5 share environmental grids but have separate route settings. No provider account or download is required.

## Complete batch

```sh
python tools/run_frozen_experiments.py --check-only
python tools/run_frozen_experiments.py --output outputs/paper-experiments
```

To continue an interrupted batch, use the same command and directory with `--resume`. Keep its response cache and source/configuration identity unchanged. The complete batch includes all 810 configurations and statistical analysis. [Settings and interpretation](EXPERIMENTS.md).
