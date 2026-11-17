# Experiment design

The 810 configurations in `config/experiment.json` cover six scenarios, five objectives, three matched random seeds and nine algorithm/variant settings: ST-DFSO, ACO, IAFSA, window ILS, and ST-DFSO without LL, AV, FG or ER, plus CORE without all four mechanisms. All use the same scenario inputs, common initial route, physical model and objective definitions.

ST-DFSO, ACO and IAFSA use 50 individuals; window ILS updates one route. Search stops at 500 iterations, 300,000 subsegment evaluation attempts or 120 s of actual search time, whichever is reached first. Final route checking follows search and is included in measured CPU. A calculation already in progress may finish after a search control point reaches its limit.

The primary comparison budget is 60 CPU s, supplemented by 10, 30 and 120 CPU s. Windows Job accounting includes Java and Python descendant processes, including exited descendants. At each budget, the analysis takes the latest complete-route cost whose recorded CPU upper endpoint is within the budget. It does not interpolate missing observations.

Runs execute sequentially in the configuration order. A common exact-input response cache starts with 441 precomputed responses from the six initial routes. Offline pool preparation is separate from per-run budgets. Later calculations are charged to the run that performs them; exact-input reuse incurs its actual runtime cost. New output directories receive their own cache.

```sh
python tools/run_frozen_experiments.py --check-only
python tools/run_frozen_experiments.py --output outputs/paper-experiments
```

`experiment_freeze.json` checks the source files, inputs, dependencies and declared Python environment; it also requires a complete Java build. For a new machine, install the specified Python/Java versions and rebuild before checking. Do not bypass a failed identity check. The code also records a source/environment signature on each new run and verifies it before continuing a batch.

Append `--resume` to continue the same interrupted batch without changing its code, configuration or cache. Reduced `--smoke` runs test integration; statistical analysis rejects them. Recorded results can be reproduced without rerunning the optimisation using the [statistics command](RESULTS.md).
