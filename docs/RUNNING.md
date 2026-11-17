# Running ST-DFSO

Follow [Quick start](QUICKSTART.md) for installation, single runs and the full experiment command. All commands run from the repository root. Processed inputs and model assets are included; external data credentials are unnecessary for the six paper scenarios.

Each run writes `outcome.json`, the configuration and input checksums in `launch.json`, final coordinates and reports under `case/`, and measured CPU in `cost.json` and `cpu_trace.jsonl`. These new outputs belong under `outputs/` and are excluded from Git. Do not write outputs into `src/`, `config/` or `data/`.

`--execute` starts a selected run; without it the command displays settings. `--smoke` reduces search limits. `--resume` on the batch launcher requires the same output location, response cache, selection and source identity.

Repository and model checks:

```sh
python tools/check_repository.py
python tools/check_environment.py --historical-hashes
python tools/check_model.py
python tools/run_frozen_experiments.py --check-only
```

For the recorded statistical calculations use `python analysis/reproduce_results.py --output outputs/statistics`. [Experimental settings](EXPERIMENTS.md) explain CPU accounting and the distinction between final routes and fixed-budget comparisons.

Additional checks of the current response model and all scenario/objective/algorithm combinations:

```sh
python tools/check_unified_runtime.py --output outputs/response-checks
python tools/run_validation.py --output outputs/response-validation
python tools/check_unified_route_coverage.py --output outputs/integration-checks
```

The last command uses reduced search limits for 270 combinations (six scenarios, five objectives and nine algorithm/ablation settings). Concurrent checks verify integration only; they do not replace the 810 sequential paper experiments. Each command needs a new output directory.
